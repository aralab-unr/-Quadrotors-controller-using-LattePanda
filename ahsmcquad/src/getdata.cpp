#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <boost/asio.hpp>
#include <mavlink.h>

class MotorNode : public rclcpp::Node {
public:
    MotorNode()
        : Node("motor_node"), serial_port_(io_service_), stop_sending_(false) {
        
        // Serial connection to Pixhawk
        try {
            serial_port_.open("/dev/ttyACM0");
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(115200));
            RCLCPP_INFO(this->get_logger(), "Connected to Pixhawk via /dev/ttyACM0.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
        }

        // Publishers for optical flow (opt_m_x, opt_m_y), rangefinder height, attitude (roll, pitch, yaw), and RC input data
        optical_flow_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("optical_flow", 10);
        rangefinder_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("rangefinder_height", 10);
        attitude_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("attitude", 10);
        rc_input_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("rc_input", 10);
        rate_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("attitude_rates", 10);

        // Timer for connection monitoring
        last_message_time_ = this->get_clock()->now();
        connection_monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&MotorNode::checkConnection, this));

        // Start MAVLink message reading thread
        reading_thread_ = std::thread(&MotorNode::readMAVLinkMessages, this);
    }

    ~MotorNode() {
        stop_sending_ = true;
        if (reading_thread_.joinable()) reading_thread_.join();
    }

private:
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;
    std::atomic<bool> stop_sending_;
    std::thread reading_thread_;
    std::mutex data_mutex_;
    rclcpp::Time last_message_time_;

    // Publishers for optical flow, rangefinder height, attitude, and RC input data
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr optical_flow_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rangefinder_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rc_input_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rate_publisher_;
    rclcpp::TimerBase::SharedPtr connection_monitor_timer_;

    // Read MAVLink messages from Pixhawk
    void readMAVLinkMessages() {
        while (!stop_sending_) {
            uint8_t byte;
            mavlink_message_t message;
            mavlink_status_t status;

            if (serial_port_.is_open()) {
                boost::system::error_code ec;
                size_t bytes_read = serial_port_.read_some(boost::asio::buffer(&byte, 1), ec);

                if (ec) {
                    RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", ec.message().c_str());
                    continue;
                }

                if (bytes_read > 0 && mavlink_parse_char(MAVLINK_COMM_0, byte, &message, &status)) {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    last_message_time_ = this->get_clock()->now();
                    handleMAVLinkMessage(message);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open.");
                break;  // Exit loop if serial port is not open
            }
        }
    }

    void handleMAVLinkMessage(const mavlink_message_t &msg) {
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_OPTICAL_FLOW:
                publishOpticalFlowData(msg);
                break;
            case MAVLINK_MSG_ID_DISTANCE_SENSOR:
                publishRangefinderData(msg);
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                publishAttitudeData(msg);
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS:
                publishRCInputData(msg);
                break;
            case MAVLINK_MSG_ID_RAW_IMU:
                    // Handle the IMU message for gyro data (roll, pitch, yaw rates)
                    publishGyroscopeData(msg);
                    break;
            default:
                break;
        }
    }

    void publishOpticalFlowData(const mavlink_message_t &msg) {
        mavlink_optical_flow_t optical_flow;
        mavlink_msg_optical_flow_decode(&msg, &optical_flow);

        geometry_msgs::msg::Vector3Stamped optical_flow_msg;
        optical_flow_msg.header.stamp = this->get_clock()->now();
        optical_flow_msg.vector.x = optical_flow.flow_comp_m_x;  // opt_m_x
        optical_flow_msg.vector.y = optical_flow.flow_comp_m_y;  // opt_m_y

        optical_flow_publisher_->publish(optical_flow_msg);
    }

    void publishRangefinderData(const mavlink_message_t &msg) {
        mavlink_distance_sensor_t rangefinder;
        mavlink_msg_distance_sensor_decode(&msg, &rangefinder);

        std_msgs::msg::Float32MultiArray rangefinder_msg;
        rangefinder_msg.data = {static_cast<float>(rangefinder.current_distance / 100.0)};  // Convert cm to m

        rangefinder_publisher_->publish(rangefinder_msg);
    }

    void publishAttitudeData(const mavlink_message_t &msg) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);

        geometry_msgs::msg::Vector3Stamped attitude_msg;
        attitude_msg.header.stamp = this->get_clock()->now();
        attitude_msg.vector.x = attitude.roll;
        attitude_msg.vector.y = attitude.pitch;
        attitude_msg.vector.z = attitude.yaw;

        attitude_publisher_->publish(attitude_msg);
    }
    void publishGyroscopeData(const mavlink_message_t &msg) {
        mavlink_raw_imu_t imu_data;
        mavlink_msg_raw_imu_decode(&msg, &imu_data);

        // Gyroscope data (roll, pitch, yaw rates) are in the 'xgyro', 'ygyro', 'zgyro' fields of imu_data
        float roll_rate = imu_data.xgyro;  // Raw gyroscope x-axis rate (roll rate)
        float pitch_rate = imu_data.ygyro; // Raw gyroscope y-axis rate (pitch rate)
        float yaw_rate = imu_data.zgyro;   // Raw gyroscope z-axis rate (yaw rate)

        // Now, use this data to publish the rates as a message
        geometry_msgs::msg::Vector3Stamped rate_msg;
        rate_msg.header.stamp = this->get_clock()->now();
        rate_msg.vector.x = roll_rate;
        rate_msg.vector.y = pitch_rate;
        rate_msg.vector.z = yaw_rate;

        rate_publisher_->publish(rate_msg);  // Publish the rate message
    }


    void publishRCInputData(const mavlink_message_t &msg) {
        mavlink_rc_channels_t rc;
        mavlink_msg_rc_channels_decode(&msg, &rc);

        std_msgs::msg::Float32MultiArray rc_msg;
        rc_msg.data = {
            static_cast<float>(rc.chan1_raw),  // Roll
            static_cast<float>(rc.chan2_raw),  // Pitch
            static_cast<float>(rc.chan4_raw),  // Yaw
            static_cast<float>(rc.chan3_raw),  // Throttle
            static_cast<float>(rc.chan5_raw),  // Channel 5
            static_cast<float>(rc.chan9_raw)   // Channel 9
        };

        rc_input_publisher_->publish(rc_msg);
    }

    void checkConnection() {
        auto now = this->get_clock()->now();
        if ((now - last_message_time_).seconds() > 1.0) {
            RCLCPP_WARN(this->get_logger(), "No data received from Pixhawk. Resetting data to zero.");

            // Reset all published data to zero
            publishZeroData();
        }
    }

    void publishZeroData() {
        geometry_msgs::msg::Vector3Stamped zero_vector_msg;
        zero_vector_msg.header.stamp = this->get_clock()->now();
        zero_vector_msg.vector.x = zero_vector_msg.vector.y = zero_vector_msg.vector.z = 0.0;

        optical_flow_publisher_->publish(zero_vector_msg);
        attitude_publisher_->publish(zero_vector_msg);
        rate_publisher_->publish(zero_vector_msg);

        std_msgs::msg::Float32MultiArray zero_array_msg;
        zero_array_msg.data = {0.0};
        rangefinder_publisher_->publish(zero_array_msg);

        zero_array_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        rc_input_publisher_->publish(zero_array_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
