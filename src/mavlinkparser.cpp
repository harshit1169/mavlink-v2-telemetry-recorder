#include "mavlinkparser.hpp"

#include <cstring>
#include <cmath>
#include "crc_x25.hpp"

/*
 * CRC extras for supported messages
 */
static uint8_t mavlinkCrcExtra(uint32_t msgid) {
	switch (msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			return 50U;
		case MAVLINK_MSG_ID_ATTITUDE:
			return 39U;
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			return 104U;
		default:
			return 0U;
	}
}

MavlinkParser::MavlinkParser(std::ofstream &log)
	: log_file_(log),
	  state_(State::HUNTING),
	  index_(0U),
	  payload_length_(0U),
	  message_id_(0U)
{
}

void MavlinkParser::processByte(uint8_t byte) {
	switch (state_) {

		case State::HUNTING:
			if (byte == MAVLINK_V2_MAGIC)
			{
				index_ = 0U;
				buffer_[index_++] = byte;
				state_ = State::HEADER;
			}
			break;

		case State::HEADER:
			buffer_[index_++] = byte;

			if (index_ == MAVLINK_V2_HEADER_LEN)
			{

				payload_length_ = static_cast<uint16_t>(buffer_[1]);

				message_id_ =
					static_cast<uint32_t>(buffer_[7]) |
					(static_cast<uint32_t>(buffer_[8]) << 8U) |
					(static_cast<uint32_t>(buffer_[9]) << 16U);

				if (payload_length_ > MAVLINK_MAX_PAYLOAD_LEN)
				{
					state_ = State::HUNTING; // invalid frame
				}
				else
				{
					state_ = State::PAYLOAD;
				}
			}
			break;

		case State::PAYLOAD:
			buffer_[index_++] = byte;

			if (index_ == MAVLINK_V2_HEADER_LEN + payload_length_)
			{
				state_ = State::CRC;
			}
			break;

		case State::CRC:
			buffer_[index_++] = byte;

			if (index_ == MAVLINK_V2_HEADER_LEN + payload_length_ + 2U)
			{

				uint16_t received_crc =
					static_cast<uint16_t>(buffer_[index_ - 2U]) |
					(static_cast<uint16_t>(buffer_[index_ - 1U]) << 8U);

				uint16_t computed_crc =
					crcX25(&buffer_[1], static_cast<uint16_t>(index_ - 3U));

				uint8_t extra = mavlinkCrcExtra(message_id_);
				computed_crc ^= crcX25(&extra, 1U);

				if (received_crc == computed_crc)
				{
					decodeMessage();
				}

				state_ = State::HUNTING;
			}
			break;
	}
}

void MavlinkParser::decodeMessage() {
	const uint8_t *payload = &buffer_[MAVLINK_V2_HEADER_LEN];

	switch (message_id_) {

		case MAVLINK_MSG_ID_HEARTBEAT: {
			mavlink_heartbeat_t hb{};
			std::memcpy(&hb, payload, sizeof(hb));

			log_file_
				<< "HEARTBEAT,"
				<< static_cast<int>(hb.type) << ","
				<< static_cast<int>(hb.base_mode) << ","
				<< static_cast<int>(hb.system_status)
				<< "\n";
			break;
		}

		case MAVLINK_MSG_ID_ATTITUDE: {
			mavlink_attitude_t att{};
			std::memcpy(&att, payload, sizeof(att));

			constexpr float RAD_TO_DEG = 57.2957795f;

			log_file_
				<< "ATTITUDE,"
				<< att.roll * RAD_TO_DEG << ","
				<< att.pitch * RAD_TO_DEG << ","
				<< att.yaw * RAD_TO_DEG
				<< "\n";
			break;
		}

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
			mavlink_global_position_int_t pos{};
			std::memcpy(&pos, payload, sizeof(pos));

			log_file_
				<< "GLOBAL_POSITION,"
				<< pos.lat / 1e7 << ","
				<< pos.lon / 1e7 << ","
				<< pos.alt / 1000.0
				<< "\n";
			break;
		}

		default:
			break;
	}
}
