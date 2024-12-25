#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>

template <typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

inline double scale(double in, double min, double max, double into1, double into2) {
    if (min == max) {
        throw std::invalid_argument("scale function: min and max cannot be equal.");
    }
    return into1 + (in - min) * (into2 - into1) / (max - min);
}

class LowPassFilter {
public:
    LowPassFilter(double cutoff_frequency, double sampling_frequency)
        : alpha_(calculateAlpha(cutoff_frequency, sampling_frequency)), 
          prev_output_(0.0) {}

    double filter(double input) {
        prev_output_ = alpha_ * input + (1.0 - alpha_) * prev_output_;
        return prev_output_;
    }

private:
    double alpha_;
    double prev_output_;

    double calculateAlpha(double cutoff_freq, double sampling_freq) {
        double RC = 1.0 / (2.0 * M_PI * cutoff_freq);
        double dt = 1.0 / sampling_freq;
        return dt / (RC + dt);
    }
};

class PWMControllerNode : public rclcpp::Node {
public:
    PWMControllerNode() : Node("pwm_controller"),
          pw1(1000), pw2(1000), pw3(1000), pw4(1000), fly_mode_(0), vx(0), vy(0), x(0), y(0), range(0),
          ixx(0.0785), iyy(0.0785), izz(0.105), cphi_ctrl(3.25), ctheta_ctrl(3.25), cpsi_ctrl(1.75), 
          Kphi(0.75), Ktheta(0.75), Kpsi(0.25), Kx(3.5), Ky(3.5), Kz(1.25), Ka(10.75), eta(0.25), 
          phi_filter_(60, 65), theta_filter_(60, 65), psi_filter_(60, 65), Kdx(0.0000267), Kdy(0.0000267), Kdz(0.0000625),
          vx_filter_(75, 150), vy_filter_(75, 150), range_filter_(75, 150), cx(0.015), cy(0.015), cz(0.175), lam1(0.05), lam2(0.05),
          U1(0), U2(0), U3(0), U4(0), phi(0), theta(0), psi(0), prephi(0), pretheta(0), prepsi(0), prerange(0),
          last_time_(std::chrono::steady_clock::now()) {
        
        attitude_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/attitude", rclcpp::QoS(10), 
            std::bind(&PWMControllerNode::attitudeCallback, this, std::placeholders::_1));

        opticalflow_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/optical_flow", rclcpp::QoS(10), 
            std::bind(&PWMControllerNode::opticalflowCallback, this, std::placeholders::_1));

        rangefinder_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rangefinder_height", rclcpp::QoS(10), 
            std::bind(&PWMControllerNode::rangefinderCallback, this, std::placeholders::_1));

        rc_input_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rc_input", rclcpp::QoS(10), 
            std::bind(&PWMControllerNode::rcInputCallback, this, std::placeholders::_1));

        serial_fd_ = open("/dev/ttyACM1", O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
            configureSerialPort(serial_fd_);
        }
    }

    ~PWMControllerNode() {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr opticalflow_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rangefinder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rc_input_sub_;

    int serial_fd_;
    double pw1, pw2, pw3, pw4;
    double fly_mode_;
    double U1, U2, U3, U4;
    double phi, theta, psi;
    double prephi, pretheta, prepsi, prerange;
    double x, y, vx, vy, range;
    double rcroll, rcpitch, rcyaw, rcthrottle, rc5, rc9;
    double ixx, iyy, izz, cphi_ctrl, ctheta_ctrl, cpsi_ctrl, Kphi, Ktheta, Kpsi, Kx, Ky, Kz, Ka;
    double Kdx, Kdy, Kdz, cx, cy, cz, lam1, lam2, eta;
    LowPassFilter phi_filter_;
    LowPassFilter theta_filter_;
    LowPassFilter psi_filter_;
    LowPassFilter vx_filter_;
    LowPassFilter vy_filter_;
    LowPassFilter range_filter_;
    std::chrono::steady_clock::time_point last_time_;
    std::mutex data_mutex_;

    void attitudeCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        phi = phi_filter_.filter(msg->vector.x);
        theta = theta_filter_.filter(msg->vector.y);
        psi = psi_filter_.filter(msg->vector.z);
        sendPWMData();
    }

    void opticalflowCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        vx = vx_filter_.filter(msg->vector.x);
        vy = vy_filter_.filter(msg->vector.y);
    }

    void rangefinderCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!msg->data.empty()) {
            range = range_filter_.filter(msg->data[0]);
        }
    }

    void rcInputCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 6) {
            rcroll = msg->data[0];
            rcpitch = msg->data[1];
            rcyaw = msg->data[2];
            rcthrottle = msg->data[3];
            rc5 = msg->data[4];
            rc9 = msg->data[5];
        } else {
            RCLCPP_ERROR(this->get_logger(), "Expected 6 control values but got %zu", msg->data.size());
        }
    }

    void configureSerialPort(int fd) {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            return;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
        }
    }

    void sendPWMData() {
        fly_mode_ = (rc5 < 1200 && rc9 < 1200) ? 1 :
                (rc5 > 1200 && rc5 < 1650 && rc9 < 1200) ? 2 :
                (rc5 > 1650 && rc9 < 1200) ? 3 : 0;

        // Handle fly modes
        if (fly_mode_ == 0) {
            computeMotorEmergency();
        } else if (fly_mode_ == 1) {
            computeMotorManual();
        } else {
            autonomousControl();
        }
        int pwm_values[] = {static_cast<int>(pw1), static_cast<int>(pw2), static_cast<int>(pw3), static_cast<int>(pw4)};
        for (int pwm : pwm_values) {
            std::string pwm_value_str = std::to_string(pwm) + "\n";
            if (serial_fd_ >= 0) {
                write(serial_fd_, pwm_value_str.c_str(), pwm_value_str.size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Serial port is not open.");
            }
        }
    }

    void computeMotorEmergency() {
        pw1 = pw2 = pw3 = pw4 = 1000;
    }

    void computeMotorManual() {

        auto start_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(start_time - last_time_).count();
        last_time_ = start_time;

        U1 = scale(rcthrottle, 1037, 1917, 0, 25);

        double vxr = scale(rcroll, 935, 1801, -1.75, 1.75);
        double vyr = scale(rcpitch, 1065, 1928, -1.75, 1.75);

        double kvx, kvy;
        kvx = 0.15;
        kvy = 0.15;

        double phir, thetar;
        phir = kvx * (vxr - vx);
        thetar = kvy * (vyr - vy);

        phir = clamp(phir, -0.2, 0.2);
        thetar = clamp(thetar, -0.2, 0.2);

        double psir = scale(rcyaw, 1066, 1933, -1.5, 1.5);

        double phid, thetad, psid;

        double sampletime = 0.015;

        phid = (phi - prephi) / sampletime;
        thetad = (theta - pretheta) / sampletime;
        psid = (psi - prepsi) / sampletime;

        prephi = phi;
        pretheta = theta;
        prepsi = psi;

        double phird, thetard, psird;
        double gx, gy, gz;
        gx = 3.25;
        gy = 3.25;
        gz = 3.25;

        phird = gx * (phir - phi);
        thetard = gy * (thetar - theta);
        psird = gz * (psir - psi);

        double scphi = cphi_ctrl * (phi - phir) + (phid - phird);
        double sctheta = ctheta_ctrl * (theta - thetar) + (thetad - thetard);
        double scpsi = cpsi_ctrl * (psi - psir) + (psid - psird);
        double satphi = clamp(scphi, -0.1, 0.1);
        double sattheta = clamp(sctheta, -0.1, 0.1);
        double satpsi = clamp(scpsi, -0.1, 0.1);

        double U2s = ixx * (-Kx * cphi_ctrl * phi + Kx * cphi_ctrl * phir - (cphi_ctrl + Kx) * (phid - phird) - Kphi * satphi);
        double U3s = iyy * (-Ky * ctheta_ctrl * theta  + Ky * ctheta_ctrl * thetar - (ctheta_ctrl + Ky) * (thetad - thetard) - Ktheta * sattheta);
        double U4s = izz * (-Kz * cpsi_ctrl * psi + Kz * cpsi_ctrl * psir - (cpsi_ctrl + Kz) * (psid - psird) - Kpsi * satpsi);

        U2 = clamp(U2s, -0.75, 0.75);
        U3 = clamp(U3s, -0.75, 0.75);
        U4 = clamp(U4s, -1.25, 1.25);

        double kt = 0.05, kd = 0.015, l = 0.175;
        double pw1s, pw2s, pw3s, pw4s;
        // Compute the squared angular speeds for each motor
        pw1s = (U1*kd*l -  U2*kd + U3*kd + U4*kt*l)/(4*kd*kt*l);
        pw2s = (U1*kd*l +  U2*kd + U3*kd - U4*kt*l)/(4*kd*kt*l);
        pw3s = (U1*kd*l -  U2*kd - U3*kd - U4*kt*l)/(4*kd*kt*l);
        pw4s = (U1*kd*l +  U2*kd - U3*kd + U4*kt*l)/(4*kd*kt*l);

        double pw1c = clamp( pw1s, 0.0, 145.0);
        double pw2c = clamp( pw2s, 0.0, 145.0);
        double pw3c = clamp( pw3s, 0.0, 145.0);
        double pw4c = clamp( pw4s, 0.0, 145.0);

        pw1c = sqrt(pw1c);
        pw2c = sqrt(pw2c);
        pw3c = sqrt(pw3c);
        pw4c = sqrt(pw4c);

        pw1 = scale(pw1c, 0, 12, 1000, 2000);
        pw2 = scale(pw2c, 0, 12, 1000, 2000);
        pw3 = scale(pw3c, 0, 12, 1000, 2000);
        pw4 = scale(pw4c, 0, 12, 1000, 2000);
    }
    void autonomousControl() {

        auto start_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(start_time - last_time_).count();
        last_time_ = start_time;
        double sampletime = 0.015;

        double zr = scale(rcthrottle, 1037, 1917, 0, 2.5);
        double vxr = scale(rcroll, 935, 1801, -1.75, 1.75);
        double vyr = scale(rcpitch, 1065, 1928, -1.75, 1.75);
        double psir = scale(rcyaw, 1066, 1933, -1.5, 1.5);

        double cphi = cos(phi), sphi = sin(phi);
        double ctheta = cos(theta), stheta = sin(theta);
        double cpsi = cos(psi), spsi = sin(psi);

        double m = 1.5;
        double g = 9.81;

        double yd = -vx * sampletime;
        double xd = -vy * sampletime;
        double zd = (range - prerange) / sampletime;
        zd = clamp(zd, -0.025, 0.025);

        x = x + vx * dt;
        y = y + vy * dt;

        prerange = range;

        double yrd = 0;
        double xrd = 0;
        double zrd = 0;

        double fx = -Kdx * xd / m;
        double fy = -Kdy * yd / m;
        double fz = (-Kdz * zd - m * g) / m;
        double bx = 1 / m * (spsi * sphi + cpsi * stheta * cphi);
        double by = 1 / m * (spsi * stheta * cphi - cpsi * sphi);
        double bz = 1 / m * (ctheta * cphi);

        double sx = cx * vxr + (xrd - xd);
        double sy = cy * vyr + (yrd - yd);
        double sz = cz * (zr - range) - zd;

        double ueqx = (cx * (xrd - xd) - fx) / bx;
        double ueqy = (cy * (yrd - yd) - fy) / by;
        double ueqz = (cz * (zrd - zd) - fz) / bz;

        double s3 = lam2 * lam1 * sx + lam2 * sy + sz;
        double sats3 = clamp(s3, -0.25, 0.25);

        double usw = (lam2 * lam1 * bx * (ueqy + ueqz) + lam2 * by * (ueqx + ueqz) + bz * (ueqx + ueqy) + Ka * s3 + eta * sats3) / (lam2 * lam1 * bx + lam2 * by + bz);
        double Uz = ueqx + ueqy + ueqz + usw;

        double U1 = clamp(Uz, 12.5, 25.0);

        double kvx, kvy;
        kvx = 0.15;
        kvy = 0.15;

        double phir, thetar;
        phir = kvx * (vxr - vx);
        thetar = kvy * (vyr - vy);

        phir = clamp(phir, -0.2, 0.2);
        thetar = clamp(thetar, -0.2, 0.2);

        double phid, thetad, psid;

        phid = (phi - prephi) / sampletime;
        thetad = (theta - pretheta) / sampletime;
        psid = (psi - prepsi) / sampletime;

        prephi = phi;
        pretheta = theta;
        prepsi = psi;

        double phird, thetard, psird;
        double gx, gy, gz;
        gx = 3.25;
        gy = 3.25;
        gz = 3.25;

        phird = gx * (phir - phi);
        thetard = gy * (thetar - theta);
        psird = gz * (psir - psi);

        double scphi = cphi_ctrl * (phi - phir) + (phid - phird);
        double sctheta = ctheta_ctrl * (theta - thetar) + (thetad - thetard);
        double scpsi = cpsi_ctrl * (psi - psir) + (psid - psird);
        double satphi = clamp(scphi, -0.1, 0.1);
        double sattheta = clamp(sctheta, -0.1, 0.1);
        double satpsi = clamp(scpsi, -0.1, 0.1);

        double U2s = ixx * (-Kx * cphi_ctrl * phi + Kx * cphi_ctrl * phir - (cphi_ctrl + Kx) * (phid - phird) - Kphi * satphi);
        double U3s = iyy * (-Ky * ctheta_ctrl * theta  + Ky * ctheta_ctrl * thetar - (ctheta_ctrl + Ky) * (thetad - thetard) - Ktheta * sattheta);
        double U4s = izz * (-Kz * cpsi_ctrl * psi + Kz * cpsi_ctrl * psir - (cpsi_ctrl + Kz) * (psid - psird) - Kpsi * satpsi);

        U2 = clamp(U2s, -0.75, 0.75);
        U3 = clamp(U3s, -0.75, 0.75);
        U4 = clamp(U4s, -1.25, 1.25);

        double kt = 0.05, kd = 0.015, l = 0.175;
        double pw1s, pw2s, pw3s, pw4s;
        // Compute the squared angular speeds for each motor
        pw1s = (U1*kd*l -  U2*kd + U3*kd + U4*kt*l)/(4*kd*kt*l);
        pw2s = (U1*kd*l +  U2*kd + U3*kd - U4*kt*l)/(4*kd*kt*l);
        pw3s = (U1*kd*l -  U2*kd - U3*kd - U4*kt*l)/(4*kd*kt*l);
        pw4s = (U1*kd*l +  U2*kd - U3*kd + U4*kt*l)/(4*kd*kt*l);

        double pw1c = clamp( pw1s, 0.0, 145.0);
        double pw2c = clamp( pw2s, 0.0, 145.0);
        double pw3c = clamp( pw3s, 0.0, 145.0);
        double pw4c = clamp( pw4s, 0.0, 145.0);

        pw1c = sqrt(pw1c);
        pw2c = sqrt(pw2c);
        pw3c = sqrt(pw3c);
        pw4c = sqrt(pw4c);

        pw1 = scale(pw1c, 0, 12, 1000, 2000);
        pw2 = scale(pw2c, 0, 12, 1000, 2000);
        pw3 = scale(pw3c, 0, 12, 1000, 2000);
        pw4 = scale(pw4c, 0, 12, 1000, 2000);

        // use that for the data colection
        //auto node = rclcpp::Node::make_shared("time_logger");
        //rclcpp::Clock::SharedPtr clock = node->get_clock();

        //auto now = clock->now();
        //auto elapsed_time = now.nanoseconds(); // ROS time in nanoseconds

        //auto elapsed_time_ms = elapsed_time / 1'000'000; 

        //std::ofstream outfile1("datahsmc/datahsmcpositionxyz.txt", std::ios_base::app);
        //if (outfile1.is_open()) {
        //    outfile1 << "Time: " << elapsed_time_ms << ", x: " << x << ", y: " << y << ", z: " << range << ", vx: " << vx << ", vy: " << vy << ", vxr: " << vxr << ", vyr: " << vyr <<"\n";
        //    outfile1.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        //}

        //std::ofstream outfile2("datahsmc/datahsmcpositionrpy.txt", std::ios_base::app);
        //if (outfile2.is_open()) {
        //   outfile2 << "Time: " << elapsed_time_ms << ", Roll desi: " << phir << ", Pitch desi: " << thetar << ", Yaw desi: " << psir << ", Roll: " << phi << ", Pitch: " << theta << ", Yaw: " << psi << "\n";
        //    outfile2.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        //}

        //std::ofstream outfile3("datahsmc/datahsmcpositioncontrol.txt", std::ios_base::app);
        //if (outfile3.is_open()) {
        //    outfile3 << "Time: " << elapsed_time_ms << ", U1: " << U1 << ", U2: " << U2 << ", U3: " << U3 << ", U4: " << U4 << ", pw1: " << pw1 << ", pw2: " << pw2 << ", pw3: " << pw3 << ", pw4: " << pw4 << "\n";
        //    outfile3.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        //}
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PWMControllerNode>());
    rclcpp::shutdown();
    return 0;
}
