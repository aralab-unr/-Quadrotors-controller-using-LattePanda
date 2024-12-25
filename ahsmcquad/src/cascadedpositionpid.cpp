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

// Utility functions
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

// Low-pass filter implementation
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

// Main Node
class PWMControllerNode : public rclcpp::Node {
public:
    PWMControllerNode()
        : Node("pwm_controller"),
          pw1(1000), pw2(1000), pw3(1000), pw4(1000),
          fly_mode_(0), vx(0), vy(0), range(0),  
          U1(0), U2(0), U3(0), U4(0),
          phi(0), theta(0), psi(0), prephi(0), pretheta(0), prepsi(0), prez(0),
          phid(0), thetad(0), psid(0), zd(0), phii(0), thetai(0), psii(0), zi(0), x(0), y(0), 
          kdphi(0.5275), kpphi(1.05), kiphi(1.5), kdtheta(0.5275), kptheta(1.05), kitheta(1.5), 
          kdpsi(0.5), kppsi(3.25), kipsi(0.5), kdz(1.75), kpz(4.75), kiz(0.75),
          phi_filter_(60, 65), theta_filter_(60, 65), psi_filter_(60, 65),
          vx_filter_(75, 150), vy_filter_(75, 150), range_filter_(75, 150),
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
    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr opticalflow_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rangefinder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rc_input_sub_;

    int serial_fd_;
    double pw1, pw2, pw3, pw4;
    double fly_mode_;
    double vx, vy, range;
    double phi, theta, psi;
    double x, y;

    // Filters
    LowPassFilter phi_filter_;
    LowPassFilter theta_filter_;
    LowPassFilter psi_filter_;
    LowPassFilter vx_filter_;
    LowPassFilter vy_filter_;
    LowPassFilter range_filter_;

    double rcroll, rcpitch, rcyaw, rcthrottle, rc5, rc9;
    double U1, U2, U3, U4;

    double prephi, pretheta, prepsi, prez;
    double phii, thetai, psii, zi;
    double phid, thetad, psid, zd;
    double kpphi, kdphi, kiphi, kptheta, kdtheta, kitheta, kppsi, kdpsi, kipsi, kpz, kdz, kiz;
    std::chrono::steady_clock::time_point last_time_;
    std::mutex data_mutex_;
    rclcpp::TimerBase::SharedPtr pwm_timer_;  // Timer to handle PWM sending

    // Callbacks
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
        // Determine the fly mode based on rc5 and rc9 values
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

        // Prepare PWM values for sending
        int pwm_values[] = {static_cast<int>(pw1), static_cast<int>(pw2), static_cast<int>(pw3), static_cast<int>(pw4)};
        for (int pwm : pwm_values) {
            std::string pwm_value_str = std::to_string(pwm) + "\n";

            // Check if the serial port is open before writing
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
        x = x + vx * dt;
        y = y + vy * dt;

        phir = clamp(phir, -0.2, 0.2);
        thetar = clamp(thetar, -0.2, 0.2);

        double psir = scale(rcyaw, 1066, 1933, -1.5, 1.5);

        double errorphi = phir - phi;
        double errortheta = thetar - theta;
        double errorpsi = psir - psi;

        phii += errorphi*dt;
        thetai += errortheta*dt;
        psii += errorpsi*dt;

        phii = clamp(phii, -0.075, 0.075);
        thetai = clamp(thetai, -0.075, 0.075);
        psii = clamp(psii, -0.075, 0.075);

        double sampletime = 0.015;

        phid = (phi - prephi) / sampletime;
        thetad = (theta - pretheta) / sampletime;
        psid = (psi - prepsi);

        double phird, thetard, psird;
        double gx, gy, gz;
        gx = 3.25;
        gy = 3.25;
        gz = 3.25;

        phird = gx * (phir - phi);
        thetard = gy * (thetar - theta);
        psird = gz * (psir - psi);

        prephi = phi;
        pretheta = theta;
        prepsi = psi;

        U2 = kpphi * errorphi - kdphi * (phid - phird) + kiphi * phii;
        U3 = kptheta * errortheta - kdtheta * (thetad - thetard) + kitheta * thetai;
        U4 = kppsi * errorpsi - kdpsi * (psid - psird)  + kipsi * psii;

        U2 = clamp(U2, -0.75, 0.75);
        U3 = clamp(U3, -0.75, 0.75);
        U4 = clamp(U4, -0.75, 0.75);


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

        double zr;
        zr = scale(rcthrottle, 1037, 1917, 0, 2.5);
        double errorz = zr - range;

        double sampletime = 0.015;

        zi += errorz*dt;
        zi = clamp(zi, -0.125, 0.125);

        zd = (range - prez) / sampletime;

        prez = range;
        double m = 1.5;
        double g = 9.81;
        double pidz = kpz * errorz - kdz * zd + kiz * zi;

        U1 = m*g + kpz * errorz - kdz * zd + kiz * zi;
        U1 = clamp(U1, 12.5, 25.0);

        double vxr = scale(rcroll, 935, 1801, -1.75, 1.75);
        double vyr = scale(rcpitch, 1065, 1928, -1.75, 1.75);

        double kvx, kvy;
        kvx = 0.15;
        kvy = 0.15;

        double phir, thetar;
        phir = kvx * (vxr - vx);
        thetar = kvy * (vyr - vy);
        x = x + vx * dt;
        y = y + vy * dt;

        phir = clamp(phir, -0.2, 0.2);
        thetar = clamp(thetar, -0.2, 0.2);

        double psir = scale(rcyaw, 1066, 1933, -1.0, 1.0);

        double errorphi = phir - phi;
        double errortheta = thetar - theta;
        double errorpsi = psir - psi;

        phii += errorphi*dt;
        thetai += errortheta*dt;
        psii += errorpsi*dt;

        phii = clamp(phii, -0.075, 0.075);
        thetai = clamp(thetai, -0.075, 0.075);
        psii = clamp(psii, -0.075, 0.075);

        
        phid = (phi - prephi) / sampletime;
        thetad = (theta - pretheta) / sampletime;
        psid = (psi - prepsi);

        double phird, thetard, psird;
        double gx, gy, gz;
        gx = 3.25;
        gy = 3.25;
        gz = 3.25;

        phird = gx * (phir - phi);
        thetard = gy * (thetar - theta);
        psird = gz * (psir - psi);

        prephi = phi;
        pretheta = theta;
        prepsi = psi;

        U2 = kpphi * errorphi - kdphi * (phid - phird) + kiphi * phii;
        U3 = kptheta * errortheta - kdtheta * (thetad - thetard) + kitheta * thetai;
        U4 = kppsi * errorpsi - kdpsi * (psid - psird)  + kipsi * psii;

        U2 = clamp(U2, -0.75, 0.75);
        U3 = clamp(U3, -0.75, 0.75);
        U4 = clamp(U4, -1.25, 1.25);

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

        //std::ofstream outfile1("datapid/datapidxyz.txt", std::ios_base::app);
        //if (outfile1.is_open()) {
        //    outfile1 << "Time: " << elapsed_time_ms << ", x: " << x << ", y: " << y << ", z: " << range << ", vx: " << vx << ", vy: " << vy << ", vxr: " << vxr << ", vyr: " << vyr <<"\n";
        //    outfile1.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        // }

        //std::ofstream outfile2("datapid/datapidrpy.txt", std::ios_base::app);
        //if (outfile2.is_open()) {
        //    outfile2 << "Time: " << elapsed_time_ms << ", Roll desi: " << phir << ", Pitch desi: " << thetar << ", Yaw desi: " << psir << ", Roll: " << phi << ", Pitch: " << theta << ", Yaw: " << psi << "\n";
        //    outfile2.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        //}

        //std::ofstream outfile3("datapid/datapidcontrol.txt", std::ios_base::app);
        //if (outfile3.is_open()) {
        //   outfile3 << "Time: " << elapsed_time_ms << ", U1: " << U1 << ", U2: " << U2 << ", U3: " << U3 << ", U4: " << U4 << ", pw1: " << pw1 << ", pw2: " << pw2 << ", pw3: " << pw3 << ", pw4: " << pw4 << "\n";
        //    outfile3.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        //}
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PWMControllerNode>());
    rclcpp::shutdown();
    return 0;
}
