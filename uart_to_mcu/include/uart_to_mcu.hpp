#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace uart_to_mcu
{
// Packet format: header + id + data_length(u32) + payload + tail
inline constexpr uint16_t kFrameHeader = 0xAA55;
inline constexpr uint8_t kWheelPacketId = 0x01;
inline constexpr uint16_t kFrameTail = 0x55AA;
inline constexpr uint32_t kWheelPayloadLength = 8U;  // left(int32) + right(int32)
inline constexpr const char * kDefaultWheelSpeedsTopic = "/wheel_speeds";
inline constexpr const char * kDefaultSerialPort = "/dev/ttyUSB0";

class UartToMcuNode : public rclcpp::Node
{
public:
	UartToMcuNode();
	~UartToMcuNode() override;

private:
	bool open_serial();
	void close_serial();
	void wheel_speeds_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
	std::vector<uint8_t> build_packet(int32_t left_value, int32_t right_value) const;
	bool write_packet(const std::vector<uint8_t> & packet);

	static void append_u16_le(std::vector<uint8_t> & out, uint16_t value);
	static void append_u32_le(std::vector<uint8_t> & out, uint32_t value);
	static void append_i32_le(std::vector<uint8_t> & out, int32_t value);

	std::string wheel_speeds_topic_;
	std::string serial_port_;
	int serial_fd_;

	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_sub_;
};
}  // namespace uart_to_mcu
