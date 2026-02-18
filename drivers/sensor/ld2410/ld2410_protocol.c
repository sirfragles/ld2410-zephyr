/*
 * Copyright (c) 2024 Matthew Macdonald-Wallace
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <ld2410/ld2410_protocol.h>

LOG_MODULE_DECLARE(ld2410, CONFIG_LD2410_LOG_LEVEL);

/* ---------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------------*/

static inline void put_le16(uint8_t *buf, uint16_t val)
{
	buf[0] = (uint8_t)(val & 0xFF);
	buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

static inline void put_le32(uint8_t *buf, uint32_t val)
{
	buf[0] = (uint8_t)(val & 0xFF);
	buf[1] = (uint8_t)((val >> 8) & 0xFF);
	buf[2] = (uint8_t)((val >> 16) & 0xFF);
	buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

static inline uint16_t get_le16(const uint8_t *buf)
{
	return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static inline uint32_t get_le32(const uint8_t *buf)
{
	return (uint32_t)buf[0] |
	       ((uint32_t)buf[1] << 8) |
	       ((uint32_t)buf[2] << 16) |
	       ((uint32_t)buf[3] << 24);
}

/* ---------------------------------------------------------------------------
 * Frame builder
 * ---------------------------------------------------------------------------*/

int ld2410_build_cmd_frame(uint8_t *buf, uint16_t cmd_word,
			   const uint8_t *data, size_t data_len)
{
	if (!buf) {
		return -EINVAL;
	}

	/* Intra-frame data length = 2 (command word) + data_len */
	uint16_t intra_len = 2 + data_len;

	if (4 + 2 + intra_len + 4 > LD2410_MAX_CMD_FRAME_SIZE) {
		return -ENOMEM;
	}

	size_t pos = 0;

	/* Frame header: FD FC FB FA */
	buf[pos++] = LD2410_CMD_FRAME_HEADER_0;
	buf[pos++] = LD2410_CMD_FRAME_HEADER_1;
	buf[pos++] = LD2410_CMD_FRAME_HEADER_2;
	buf[pos++] = LD2410_CMD_FRAME_HEADER_3;

	/* Intra-frame data length (2 bytes LE) */
	put_le16(&buf[pos], intra_len);
	pos += 2;

	/* Command word (2 bytes LE) */
	put_le16(&buf[pos], cmd_word);
	pos += 2;

	/* Command value */
	if (data && data_len > 0) {
		memcpy(&buf[pos], data, data_len);
		pos += data_len;
	}

	/* Frame end: 04 03 02 01 */
	buf[pos++] = LD2410_CMD_FRAME_END_0;
	buf[pos++] = LD2410_CMD_FRAME_END_1;
	buf[pos++] = LD2410_CMD_FRAME_END_2;
	buf[pos++] = LD2410_CMD_FRAME_END_3;

	return (int)pos;
}

/* ---------------------------------------------------------------------------
 * Parser — byte-by-byte state machine
 * ---------------------------------------------------------------------------*/

static const uint8_t cmd_header[] = {
	LD2410_CMD_FRAME_HEADER_0, LD2410_CMD_FRAME_HEADER_1,
	LD2410_CMD_FRAME_HEADER_2, LD2410_CMD_FRAME_HEADER_3
};
static const uint8_t cmd_footer[] = {
	LD2410_CMD_FRAME_END_0, LD2410_CMD_FRAME_END_1,
	LD2410_CMD_FRAME_END_2, LD2410_CMD_FRAME_END_3
};
static const uint8_t data_header[] = {
	LD2410_DATA_FRAME_HEADER_0, LD2410_DATA_FRAME_HEADER_1,
	LD2410_DATA_FRAME_HEADER_2, LD2410_DATA_FRAME_HEADER_3
};
static const uint8_t data_footer[] = {
	LD2410_DATA_FRAME_END_0, LD2410_DATA_FRAME_END_1,
	LD2410_DATA_FRAME_END_2, LD2410_DATA_FRAME_END_3
};

void ld2410_parser_init(struct ld2410_parser *parser)
{
	memset(parser, 0, sizeof(*parser));
	parser->state = LD2410_PARSE_HEADER;
}

bool ld2410_parser_feed(struct ld2410_parser *parser, uint8_t byte,
			struct ld2410_raw_frame *frame)
{
	switch (parser->state) {
	case LD2410_PARSE_HEADER:
		/* Try to match either command or data header */
		if (byte == cmd_header[parser->header_idx]) {
			parser->header_idx++;
			if (parser->header_idx == 4) {
				parser->frame_type = LD2410_FRAME_ACK;
				parser->state = LD2410_PARSE_LENGTH;
				parser->header_idx = 0;
				parser->payload_idx = 0;
			}
		} else if (byte == data_header[parser->header_idx]) {
			parser->header_idx++;
			if (parser->header_idx == 4) {
				parser->frame_type = LD2410_FRAME_DATA;
				parser->state = LD2410_PARSE_LENGTH;
				parser->header_idx = 0;
				parser->payload_idx = 0;
			}
		} else {
			/* Reset: check if this byte could start a new header */
			parser->header_idx = 0;
			if (byte == cmd_header[0] || byte == data_header[0]) {
				if (byte == cmd_header[0] && byte == data_header[0]) {
					/* Both start with same byte — advance */
					parser->header_idx = 1;
				} else if (byte == cmd_header[0]) {
					parser->header_idx = 1;
				} else if (byte == data_header[0]) {
					parser->header_idx = 1;
				}
			}
		}
		break;

	case LD2410_PARSE_LENGTH:
		if (parser->payload_idx == 0) {
			parser->payload_len = byte;
			parser->payload_idx = 1;
		} else {
			parser->payload_len |= ((uint16_t)byte << 8);
			parser->payload_idx = 0;

			if (parser->payload_len > LD2410_MAX_DATA_FRAME_SIZE) {
				LOG_WRN("Frame payload too large: %u", parser->payload_len);
				ld2410_parser_init(parser);
			} else if (parser->payload_len == 0) {
				parser->state = LD2410_PARSE_FOOTER;
				parser->footer_idx = 0;
			} else {
				parser->state = LD2410_PARSE_PAYLOAD;
			}
		}
		break;

	case LD2410_PARSE_PAYLOAD:
		parser->payload[parser->payload_idx++] = byte;
		if (parser->payload_idx >= parser->payload_len) {
			parser->state = LD2410_PARSE_FOOTER;
			parser->footer_idx = 0;
		}
		break;

	case LD2410_PARSE_FOOTER: {
		const uint8_t *expected_footer =
			(parser->frame_type == LD2410_FRAME_ACK) ?
			cmd_footer : data_footer;

		if (byte == expected_footer[parser->footer_idx]) {
			parser->footer_idx++;
			if (parser->footer_idx == 4) {
				/* Complete frame! */
				frame->type = parser->frame_type;
				frame->payload_len = parser->payload_len;
				memcpy(frame->payload, parser->payload,
				       parser->payload_len);

				ld2410_parser_init(parser);
				return true;
			}
		} else {
			LOG_WRN("Footer mismatch at idx %u", parser->footer_idx);
			ld2410_parser_init(parser);
		}
		break;
	}
	}

	return false;
}

/* ---------------------------------------------------------------------------
 * ACK parsers
 * ---------------------------------------------------------------------------*/

int ld2410_parse_ack_status(const struct ld2410_raw_frame *frame,
			    uint16_t expected_cmd)
{
	if (!frame || frame->type != LD2410_FRAME_ACK) {
		return -EINVAL;
	}

	if (frame->payload_len < 4) {
		return -ENODATA;
	}

	uint16_t ack_cmd = get_le16(&frame->payload[0]);
	uint16_t status  = get_le16(&frame->payload[2]);

	if (ack_cmd != LD2410_CMD_TO_ACK(expected_cmd)) {
		LOG_WRN("ACK cmd mismatch: expected 0x%04X, got 0x%04X",
			LD2410_CMD_TO_ACK(expected_cmd), ack_cmd);
		return -EINVAL;
	}

	if (status != 0) {
		LOG_WRN("ACK status failure: %u", status);
		return -EIO;
	}

	return 0;
}

int ld2410_parse_read_params(const struct ld2410_raw_frame *frame,
			     struct ld2410_config_params *params)
{
	int ret = ld2410_parse_ack_status(frame, LD2410_CMD_READ_PARAMS);

	if (ret) {
		return ret;
	}

	if (frame->payload_len < 4 + 1 + 1 + 1 + 1) {
		return -ENODATA;
	}

	const uint8_t *p = &frame->payload[4];

	if (p[0] != 0xAA) {
		LOG_WRN("Read params: missing 0xAA marker, got 0x%02X", p[0]);
		return -EPROTO;
	}

	uint8_t max_gate = p[1];

	if (max_gate >= LD2410_MAX_GATES) {
		max_gate = LD2410_MAX_GATES - 1;
	}

	params->max_moving_gate = p[2];
	params->max_static_gate = p[3];

	/* Minimum payload needed */
	size_t needed = 4 + 4 + (max_gate + 1) * 2 + 2;

	if (frame->payload_len < needed) {
		return -ENODATA;
	}

	const uint8_t *motion_sens = &p[4];
	const uint8_t *static_sens = &p[4 + max_gate + 1];

	memset(params->moving_sensitivity, 0, sizeof(params->moving_sensitivity));
	memset(params->static_sensitivity, 0, sizeof(params->static_sensitivity));

	for (uint8_t i = 0; i <= max_gate; i++) {
		params->moving_sensitivity[i] = motion_sens[i];
		params->static_sensitivity[i] = static_sens[i];
	}

	params->no_one_duration_s = get_le16(&p[4 + (max_gate + 1) * 2]);

	return 0;
}

int ld2410_parse_firmware_version(const struct ld2410_raw_frame *frame,
				  struct ld2410_firmware_version *ver)
{
	if (!frame || !ver || frame->type != LD2410_FRAME_ACK) {
		return -EINVAL;
	}

	if (frame->payload_len < 12) {
		return -ENODATA;
	}

	uint16_t status = get_le16(&frame->payload[2]);

	if (status != 0) {
		return -EIO;
	}

	ver->type  = get_le16(&frame->payload[4]);
	ver->major = get_le16(&frame->payload[6]);
	ver->minor = get_le32(&frame->payload[8]);

	return 0;
}

int ld2410_parse_mac_address(const struct ld2410_raw_frame *frame,
			     struct ld2410_mac_addr *mac)
{
	int ret = ld2410_parse_ack_status(frame, LD2410_CMD_GET_MAC);

	if (ret) {
		return ret;
	}

	if (frame->payload_len < 10) {
		return -ENODATA;
	}

	memcpy(mac->addr, &frame->payload[4], 6);

	return 0;
}

int ld2410_parse_distance_resolution(const struct ld2410_raw_frame *frame,
				     enum ld2410_distance_resolution *resolution)
{
	int ret = ld2410_parse_ack_status(frame, LD2410_CMD_GET_RESOLUTION);

	if (ret) {
		return ret;
	}

	if (frame->payload_len < 6) {
		return -ENODATA;
	}

	*resolution = (enum ld2410_distance_resolution)get_le16(&frame->payload[4]);

	return 0;
}

int ld2410_parse_aux_control(const struct ld2410_raw_frame *frame,
			     struct ld2410_aux_control *cfg)
{
	if (!frame || !cfg) {
		return -EINVAL;
	}

	int ret = ld2410_parse_ack_status(frame, LD2410_CMD_GET_AUX_CONTROL);
	if (ret) {
		return ret;
	}

	if (frame->payload_len < 8) {
		return -ENODATA;
	}

	cfg->light_mode    = (enum ld2410_aux_light_mode)frame->payload[4];
	cfg->light_threshold = frame->payload[5];
	cfg->out_polarity  = (enum ld2410_out_pin_polarity)frame->payload[6];

	return 0;
}

/* ---------------------------------------------------------------------------
 * Data frame parser
 * ---------------------------------------------------------------------------*/

int ld2410_parse_data_frame(const struct ld2410_raw_frame *raw,
			    struct ld2410_frame *frame)
{
	if (!raw || !frame || raw->type != LD2410_FRAME_DATA) {
		return -EINVAL;
	}

	const uint8_t *p = raw->payload;
	uint16_t len = raw->payload_len;

	if (len < 13) {
		return -ENODATA;
	}

	frame->type = (enum ld2410_data_type)p[0];

	if (p[1] != LD2410_DATA_HEAD) {
		LOG_WRN("Data frame missing 0xAA head marker");
		return -EPROTO;
	}

	/* Parse basic target info */
	frame->target.state              = (enum ld2410_target_state)p[2];
	frame->target.moving_distance_cm = get_le16(&p[3]);
	frame->target.moving_energy      = p[5];
	frame->target.static_distance_cm = get_le16(&p[6]);
	frame->target.static_energy      = p[8];
	frame->target.detection_distance_cm = get_le16(&p[9]);

	/* Parse engineering data if present */
	memset(&frame->engineering, 0, sizeof(frame->engineering));

	if (frame->type == LD2410_DATA_ENGINEERING) {
		if (len < 15) {
			return -ENODATA;
		}

		uint8_t max_m = p[11];
		uint8_t max_s = p[12];

		frame->engineering.max_moving_gate = max_m;
		frame->engineering.max_static_gate = max_s;

		uint8_t num_gates = (max_m > max_s) ? max_m : max_s;

		if (num_gates >= LD2410_MAX_GATES) {
			num_gates = LD2410_MAX_GATES - 1;
		}

		size_t offset = 13;
		size_t limit = (len >= 2) ? (size_t)(len - 2) : 0;

		for (uint8_t i = 0; i <= num_gates && offset < limit; i++) {
			frame->engineering.moving_energy[i] = p[offset++];
		}
		for (uint8_t i = 0; i <= num_gates && offset < limit; i++) {
			frame->engineering.static_energy[i] = p[offset++];
		}

		/* LD2410B/C append light-sense value + OUT pin state */
		if (offset < limit) {
			frame->engineering.light_value = p[offset++];
		}
		if (offset < limit) {
			frame->engineering.out_pin_state = p[offset++];
		}
	}

	return 0;
}
