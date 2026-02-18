/*
 * Copyright (c) 2024 Matthew Macdonald-Wallace
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file ld2410_protocol.h
 * @brief LD2410 protocol internals: frame construction and parsing.
 */

#ifndef LD2410_PROTOCOL_H_
#define LD2410_PROTOCOL_H_

#include <zephyr/kernel.h>
#include <ld2410/ld2410.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Protocol framing constants
 * ---------------------------------------------------------------------------*/

/* Command frame markers */
#define LD2410_CMD_FRAME_HEADER_0  0xFD
#define LD2410_CMD_FRAME_HEADER_1  0xFC
#define LD2410_CMD_FRAME_HEADER_2  0xFB
#define LD2410_CMD_FRAME_HEADER_3  0xFA

#define LD2410_CMD_FRAME_END_0     0x04
#define LD2410_CMD_FRAME_END_1     0x03
#define LD2410_CMD_FRAME_END_2     0x02
#define LD2410_CMD_FRAME_END_3     0x01

/* Data report frame markers */
#define LD2410_DATA_FRAME_HEADER_0 0xF4
#define LD2410_DATA_FRAME_HEADER_1 0xF3
#define LD2410_DATA_FRAME_HEADER_2 0xF2
#define LD2410_DATA_FRAME_HEADER_3 0xF1

#define LD2410_DATA_FRAME_END_0    0xF8
#define LD2410_DATA_FRAME_END_1    0xF7
#define LD2410_DATA_FRAME_END_2    0xF6
#define LD2410_DATA_FRAME_END_3    0xF5

/* Intra-frame markers */
#define LD2410_DATA_HEAD           0xAA
#define LD2410_DATA_TAIL           0x55
#define LD2410_DATA_CHECK          0x00

/* ---------------------------------------------------------------------------
 * Command words (Section 2.2)
 * ---------------------------------------------------------------------------*/

#define LD2410_CMD_ENABLE_CONFIG       0x00FF
#define LD2410_CMD_END_CONFIG          0x00FE
#define LD2410_CMD_SET_MAX_GATE        0x0060
#define LD2410_CMD_READ_PARAMS         0x0061
#define LD2410_CMD_ENABLE_ENGINEERING  0x0062
#define LD2410_CMD_DISABLE_ENGINEERING 0x0063
#define LD2410_CMD_SET_GATE_SENS       0x0064
#define LD2410_CMD_READ_FIRMWARE       0x00A0
#define LD2410_CMD_SET_BAUD_RATE       0x00A1
#define LD2410_CMD_FACTORY_RESET       0x00A2
#define LD2410_CMD_RESTART             0x00A3
#define LD2410_CMD_BLUETOOTH_SET       0x00A4
#define LD2410_CMD_GET_MAC             0x00A5
#define LD2410_CMD_BT_AUTHENTICATE     0x00A8
#define LD2410_CMD_BT_SET_PASSWORD     0x00A9
#define LD2410_CMD_SET_RESOLUTION      0x00AA
#define LD2410_CMD_GET_RESOLUTION      0x00AB
#define LD2410_CMD_SET_AUX_CONTROL     0x00AD  /* LD2410B/C only */
#define LD2410_CMD_GET_AUX_CONTROL     0x00AE  /* LD2410B/C only */

/* ACK command word = sent command word | 0x0100 */
#define LD2410_CMD_TO_ACK(cmd)  ((cmd) | 0x0100)

/* Parameter words for 0x0060 */
#define LD2410_PARAM_MAX_MOVING_GATE   0x0000
#define LD2410_PARAM_MAX_STATIC_GATE   0x0001
#define LD2410_PARAM_NO_ONE_DURATION   0x0002

/* Parameter words for 0x0064 */
#define LD2410_PARAM_GATE_INDEX        0x0000
#define LD2410_PARAM_MOVING_SENS       0x0001
#define LD2410_PARAM_STATIC_SENS       0x0002

/* ---------------------------------------------------------------------------
 * Maximum frame sizes
 * ---------------------------------------------------------------------------*/

#define LD2410_MAX_CMD_FRAME_SIZE      64
#define LD2410_MAX_ACK_FRAME_SIZE      64
#define LD2410_MAX_DATA_FRAME_SIZE     64

/* ---------------------------------------------------------------------------
 * Frame builder
 * ---------------------------------------------------------------------------*/

/**
 * @brief Build a command frame.
 *
 * @param buf       Output buffer (must be at least LD2410_MAX_CMD_FRAME_SIZE).
 * @param cmd_word  Command word (little-endian 16-bit).
 * @param data      Command value bytes (may be NULL if data_len == 0).
 * @param data_len  Length of command value bytes.
 * @return Total frame length in bytes, or negative errno.
 */
int ld2410_build_cmd_frame(uint8_t *buf, uint16_t cmd_word,
			   const uint8_t *data, size_t data_len);

/* ---------------------------------------------------------------------------
 * Frame parser — state machine
 * ---------------------------------------------------------------------------*/

/** Parser frame type */
enum ld2410_frame_type {
	LD2410_FRAME_NONE,
	LD2410_FRAME_ACK,
	LD2410_FRAME_DATA,
};

/** Parser states */
enum ld2410_parser_state {
	LD2410_PARSE_HEADER,
	LD2410_PARSE_LENGTH,
	LD2410_PARSE_PAYLOAD,
	LD2410_PARSE_FOOTER,
};

/** Parsed frame output */
struct ld2410_raw_frame {
	enum ld2410_frame_type type;
	uint16_t payload_len;
	uint8_t  payload[LD2410_MAX_DATA_FRAME_SIZE];
};

/** Parser context */
struct ld2410_parser {
	enum ld2410_parser_state state;
	enum ld2410_frame_type   frame_type;
	uint8_t  header_idx;
	uint16_t payload_len;
	uint16_t payload_idx;
	uint8_t  footer_idx;
	uint8_t  payload[LD2410_MAX_DATA_FRAME_SIZE];
};

/**
 * @brief Initialize the frame parser.
 */
void ld2410_parser_init(struct ld2410_parser *parser);

/**
 * @brief Feed one byte to the parser.
 *
 * @param parser  Parser context.
 * @param byte    Received byte.
 * @param frame   Output: filled when a complete frame is detected.
 * @return true if a complete frame is available in @p frame.
 */
bool ld2410_parser_feed(struct ld2410_parser *parser, uint8_t byte,
			struct ld2410_raw_frame *frame);

/* ---------------------------------------------------------------------------
 * ACK response parsers
 * ---------------------------------------------------------------------------*/

int ld2410_parse_ack_status(const struct ld2410_raw_frame *frame,
			    uint16_t expected_cmd);

int ld2410_parse_read_params(const struct ld2410_raw_frame *frame,
			     struct ld2410_config_params *params);

int ld2410_parse_firmware_version(const struct ld2410_raw_frame *frame,
				  struct ld2410_firmware_version *ver);

int ld2410_parse_mac_address(const struct ld2410_raw_frame *frame,
			     struct ld2410_mac_addr *mac);

int ld2410_parse_distance_resolution(const struct ld2410_raw_frame *frame,
				     enum ld2410_distance_resolution *resolution);

int ld2410_parse_aux_control(const struct ld2410_raw_frame *frame,
			     struct ld2410_aux_control *cfg);

/* ---------------------------------------------------------------------------
 * Data frame parser
 * ---------------------------------------------------------------------------*/

int ld2410_parse_data_frame(const struct ld2410_raw_frame *raw,
			    struct ld2410_frame *frame);

#ifdef __cplusplus
}
#endif

#endif /* LD2410_PROTOCOL_H_ */
