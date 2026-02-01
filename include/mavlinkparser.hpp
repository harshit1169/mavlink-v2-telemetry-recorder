#pragma once

#include <cstdint>
#include <fstream>

/*
 * MAVLink headers are used only as a message dictionary.
 * Parsing logic is fully custom.
 */
#include "common/mavlink.h"

/*
 * MAVLink v2 constants
 */
constexpr uint8_t MAVLINK_V2_MAGIC = 0xFD;
constexpr uint8_t MAVLINK_V2_HEADER_LEN = 10;

/*
 * MAVLink Flight Data Recorder Parser
 */
class MavlinkParser
{
public:
	explicit MavlinkParser(std::ofstream &log);

	void processByte(uint8_t byte);

private:
	enum class State : uint8_t
	{
		HUNTING,
		HEADER,
		PAYLOAD,
		CRC
	};

	void decodeMessage();

	std::ofstream &log_file_;
	State state_;

	uint8_t buffer_[MAVLINK_MAX_PACKET_LEN];
	uint16_t index_;
	uint16_t payload_length_;
	uint32_t message_id_;
};
