#include "mavlinkparser.hpp"
#include "crc_x25.hpp"

#include <fstream>
#include <cstdint>
#include <cstring>

static uint8_t mavlinkCrcExtra(uint32_t msgid) {
	switch (msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT: return 50U;
		case MAVLINK_MSG_ID_ATTITUDE:  return 39U;
		default: return 0U;
	}
}

static void buildPacket(uint8_t *buffer,
						uint8_t &out_length,
						uint8_t seq,
						uint8_t sysid,
						uint8_t compid,
						uint32_t msgid,
						const uint8_t *payload,
						uint8_t payload_len) {

	buffer[0] = MAVLINK_V2_MAGIC;
	buffer[1] = payload_len;
	buffer[2] = 0U; // incompat flags
	buffer[3] = 0U; // compat flags
	buffer[4] = seq;
	buffer[5] = sysid;
	buffer[6] = compid;
	buffer[7] = msgid & 0xFFU;
	buffer[8] = (msgid >> 8U) & 0xFFU;
	buffer[9] = (msgid >> 16U) & 0xFFU;

	std::memcpy(buffer + 10U, payload, payload_len);

	// compute CRC over header+payload
	uint16_t crc = crcX25(buffer + 1U, 9U + payload_len);

	uint8_t extra = mavlinkCrcExtra(msgid);
	crc ^= crcX25(&extra, 1U);

	buffer[10U + payload_len] = crc & 0xFFU;
	buffer[11U + payload_len] = (crc >> 8U) & 0xFFU;

	out_length = 12U + payload_len;
}

int main() {
	std::ofstream log("flight_log.csv");
	log << "TYPE,A,B,C\n";

	MavlinkParser parser(log);

	mavlink_heartbeat_t hb{};
	hb.type = MAV_TYPE_QUADROTOR;
	hb.autopilot = MAV_AUTOPILOT_PX4;
	hb.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	hb.system_status = MAV_STATE_ACTIVE;
	hb.mavlink_version = 3U;

	mavlink_attitude_t att{};
	att.roll = 0.1f;
	att.pitch = 0.2f;
	att.yaw = 0.3f;

	uint8_t hb_pkt[64], att_pkt[64];
	uint8_t hb_len, att_len;

	buildPacket(hb_pkt, hb_len, 0U, 1U, 1U,
				MAVLINK_MSG_ID_HEARTBEAT,
				reinterpret_cast<uint8_t *>(&hb),
				sizeof(hb));

	buildPacket(att_pkt, att_len, 1U, 1U, 1U,
				MAVLINK_MSG_ID_ATTITUDE,
				reinterpret_cast<uint8_t *>(&att),
				sizeof(att));

	uint8_t stream[128];
	uint8_t idx = 0U;

	std::memcpy(stream + idx, hb_pkt, hb_len); idx += hb_len;
	std::memcpy(stream + idx, att_pkt, att_len); idx += att_len;

	for (uint8_t i = 0U; i < idx; ++i) {
		parser.processByte(stream[i]);
	}

	return 0;
}
