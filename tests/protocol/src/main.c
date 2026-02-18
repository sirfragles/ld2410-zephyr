/*
 * Copyright (c) 2024 Matthew Macdonald-Wallace
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ld2410/ld2410.h>
#include <ld2410/ld2410_protocol.h>
#include <zephyr/ztest.h>

/* ===========================================================================
 * Helper: feed an entire byte array through the parser, return result
 * ===========================================================================*/

static bool feed_bytes(struct ld2410_parser *parser,
		       const uint8_t *data, size_t len,
		       struct ld2410_raw_frame *frame)
{
	for (size_t i = 0; i < len; i++) {
		if (ld2410_parser_feed(parser, data[i], frame)) {
			return true;
		}
	}
	return false;
}

/* ===========================================================================
 * 1. Frame Builder Tests
 * ===========================================================================*/

/* Test: Enable configuration command (manual §2.2.1)
 * Expected: FD FC FB FA 04 00 FF 00 01 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_enable_config)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	uint8_t val[] = { 0x01, 0x00 };
	int len = ld2410_build_cmd_frame(buf, 0x00FF, val, sizeof(val));

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,  /* header */
		0x04, 0x00,              /* intra-frame len = 4 */
		0xFF, 0x00,              /* cmd word */
		0x01, 0x00,              /* cmd value */
		0x04, 0x03, 0x02, 0x01   /* footer */
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: End configuration command (manual §2.2.2)
 * Expected: FD FC FB FA 02 00 FE 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_end_config)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	int len = ld2410_build_cmd_frame(buf, 0x00FE, NULL, 0);

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x02, 0x00,
		0xFE, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Set max gate + duration (manual §2.2.3)
 * Expected: FD FC FB FA 14 00 60 00 00 00 08 00 00 00 01 00
 *          08 00 00 00 02 00 05 00 00 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_set_max_gate)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	uint8_t val[] = {
		0x00, 0x00, 0x08, 0x00, 0x00, 0x00,  /* max moving gate = 8 */
		0x01, 0x00, 0x08, 0x00, 0x00, 0x00,  /* max static gate = 8 */
		0x02, 0x00, 0x05, 0x00, 0x00, 0x00   /* no-one duration = 5 s */
	};
	int len = ld2410_build_cmd_frame(buf, 0x0060, val, sizeof(val));

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x14, 0x00,  /* intra-frame len = 20 */
		0x60, 0x00,  /* cmd word */
		0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x08, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x05, 0x00, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Read parameters command (manual §2.2.4)
 * Expected: FD FC FB FA 02 00 61 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_read_params)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	int len = ld2410_build_cmd_frame(buf, 0x0061, NULL, 0);

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x02, 0x00,
		0x61, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Enable engineering mode (manual §2.2.5)
 * Expected: FD FC FB FA 02 00 62 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_enable_engineering)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	int len = ld2410_build_cmd_frame(buf, 0x0062, NULL, 0);

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x02, 0x00,
		0x62, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Set gate 3 sensitivity 40/40 (manual §2.2.7)
 * Expected: FD FC FB FA 14 00 64 00 00 00 03 00 00 00 01 00
 *          28 00 00 00 02 00 28 00 00 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_set_gate_sensitivity)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	uint8_t val[] = {
		0x00, 0x00, 0x03, 0x00, 0x00, 0x00,  /* gate 3 */
		0x01, 0x00, 0x28, 0x00, 0x00, 0x00,  /* moving sens = 40 */
		0x02, 0x00, 0x28, 0x00, 0x00, 0x00   /* static sens = 40 */
	};
	int len = ld2410_build_cmd_frame(buf, 0x0064, val, sizeof(val));

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x14, 0x00,
		0x64, 0x00,
		0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x28, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x28, 0x00, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Set ALL gates sensitivity 40/40 (manual §2.2.7)
 * Expected: FD FC FB FA 14 00 64 00 00 00 FF FF 00 00 01 00
 *          28 00 00 00 02 00 28 00 00 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_set_all_gates_sensitivity)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	uint8_t val[] = {
		0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,  /* all gates */
		0x01, 0x00, 0x28, 0x00, 0x00, 0x00,  /* moving sens = 40 */
		0x02, 0x00, 0x28, 0x00, 0x00, 0x00   /* static sens = 40 */
	};
	int len = ld2410_build_cmd_frame(buf, 0x0064, val, sizeof(val));

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x14, 0x00,
		0x64, 0x00,
		0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
		0x01, 0x00, 0x28, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x28, 0x00, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Read firmware version (manual §2.2.8)
 * Expected: FD FC FB FA 02 00 A0 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_read_firmware)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	int len = ld2410_build_cmd_frame(buf, 0x00A0, NULL, 0);

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x02, 0x00,
		0xA0, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Set baud rate to 256000 (manual §2.2.9)
 * Expected: FD FC FB FA 04 00 A1 00 07 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_set_baud_rate)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	uint8_t val[] = { 0x07, 0x00 };
	int len = ld2410_build_cmd_frame(buf, 0x00A1, val, sizeof(val));

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0xA1, 0x00,
		0x07, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Restore factory settings (manual §2.2.10)
 * Expected: FD FC FB FA 02 00 A2 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_factory_reset)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	int len = ld2410_build_cmd_frame(buf, 0x00A2, NULL, 0);

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x02, 0x00,
		0xA2, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Restart module (manual §2.2.11) */
ZTEST(ld2410_protocol, test_build_restart)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	int len = ld2410_build_cmd_frame(buf, 0x00A3, NULL, 0);

	zassert_equal(len, 12);
	zassert_equal(buf[6], 0xA3);
	zassert_equal(buf[7], 0x00);
}

/* Test: Bluetooth on (manual §2.2.12)
 * Expected: FD FC FB FA 04 00 A4 00 01 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_bluetooth_on)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	uint8_t val[] = { 0x01, 0x00 };
	int len = ld2410_build_cmd_frame(buf, 0x00A4, val, sizeof(val));

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0xA4, 0x00,
		0x01, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Get MAC address (manual §2.2.13)
 * Expected: FD FC FB FA 04 00 A5 00 01 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_get_mac)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	uint8_t val[] = { 0x01, 0x00 };
	int len = ld2410_build_cmd_frame(buf, 0x00A5, val, sizeof(val));

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0xA5, 0x00,
		0x01, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Set distance resolution (manual §2.2.16)
 * Expected: FD FC FB FA 04 00 AA 00 01 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_set_resolution)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	uint8_t val[] = { 0x01, 0x00 };
	int len = ld2410_build_cmd_frame(buf, 0x00AA, val, sizeof(val));

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0xAA, 0x00,
		0x01, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: Query distance resolution (manual §2.2.17)
 * Expected: FD FC FB FA 02 00 AB 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_build_query_resolution)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	int len = ld2410_build_cmd_frame(buf, 0x00AB, NULL, 0);

	const uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x02, 0x00,
		0xAB, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	zassert_equal(len, (int)sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: NULL buffer returns -EINVAL */
ZTEST(ld2410_protocol, test_build_null_buf)
{
	int len = ld2410_build_cmd_frame(NULL, 0x00FF, NULL, 0);
	zassert_equal(len, -EINVAL);
}

/* Test: Oversized data returns -ENOMEM */
ZTEST(ld2410_protocol, test_build_oversized)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	uint8_t big[60]; /* 4+2+2+60+4 = 72 > 64 */
	memset(big, 0xAB, sizeof(big));
	int len = ld2410_build_cmd_frame(buf, 0x0060, big, sizeof(big));
	zassert_equal(len, -ENOMEM);
}

/* ===========================================================================
 * 2. Parser State Machine Tests
 * ===========================================================================*/

/* Test: Parse enable-config ACK from manual §2.2.1
 * FD FC FB FA 08 00 FF 01 00 00 01 00 40 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_parse_enable_config_ack)
{
	const uint8_t data[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x08, 0x00,
		0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));
	zassert_equal(frame.type, LD2410_FRAME_ACK);
	zassert_equal(frame.payload_len, 8);

	/* ACK word = 0x01FF, status = 0x0000, protocol = 0x0001, bufsize = 0x0040 */
	zassert_equal(frame.payload[0], 0xFF);
	zassert_equal(frame.payload[1], 0x01);
	zassert_equal(frame.payload[2], 0x00);
	zassert_equal(frame.payload[3], 0x00);

	/* Verify parse_ack_status */
	int ret = ld2410_parse_ack_status(&frame, LD2410_CMD_ENABLE_CONFIG);
	zassert_equal(ret, 0);
}

/* Test: Parse end-config ACK (manual §2.2.2)
 * FD FC FB FA 04 00 FE 01 00 00 04 03 02 01 */
ZTEST(ld2410_protocol, test_parse_end_config_ack)
{
	const uint8_t data[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0xFE, 0x01, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));
	zassert_equal(ld2410_parse_ack_status(&frame, LD2410_CMD_END_CONFIG), 0);
}

/* Test: Parse set-max-gate ACK (manual §2.2.3) */
ZTEST(ld2410_protocol, test_parse_set_max_gate_ack)
{
	const uint8_t data[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0x60, 0x01, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));
	zassert_equal(ld2410_parse_ack_status(&frame, LD2410_CMD_SET_MAX_GATE), 0);
}

/* Test: Parse read-parameters ACK (manual §2.2.4) -- full example from manual */
ZTEST(ld2410_protocol, test_parse_read_params_ack)
{
	/* From manual: max gate 8, moving gate 8, static gate 8,
	 * motion sensitivity 0-8 all = 0x14 (20), static sensitivity 0-8 all = 0x19 (25),
	 * no-one duration = 5 s */
	const uint8_t data[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x1C, 0x00,  /* payload len = 28 */
		/* ACK header */
		0x61, 0x01, 0x00, 0x00,
		/* Params: AA, max_gate=8, mov_gate=8, sta_gate=8 */
		0xAA, 0x08, 0x08, 0x08,
		/* Motion sensitivity gates 0-8 (all 0x14 = 20) */
		0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
		/* Static sensitivity gates 0-8 (all 0x19 = 25) */
		0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
		/* No-one duration = 5 (LE) */
		0x05, 0x00,
		/* Footer */
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));
	zassert_equal(frame.type, LD2410_FRAME_ACK);

	struct ld2410_config_params params;
	int ret = ld2410_parse_read_params(&frame, &params);
	zassert_equal(ret, 0);

	zassert_equal(params.max_moving_gate, 8);
	zassert_equal(params.max_static_gate, 8);
	zassert_equal(params.no_one_duration_s, 5);

	for (int i = 0; i < LD2410_MAX_GATES; i++) {
		zassert_equal(params.moving_sensitivity[i], 20);
		zassert_equal(params.static_sensitivity[i], 25);
	}
}

/* Test: Parse firmware version ACK (manual §2.2.8)
 * FD FC FB FA 0C 00 00 00 00 01 07 01 16 15 09 22 04 03 02 01
 * Version: V1.07.22091615 */
ZTEST(ld2410_protocol, test_parse_firmware_version_ack)
{
	const uint8_t data[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x0C, 0x00,  /* payload = 12 */
		/* Note: manual shows ack_cmd=0x0000 for this -- the firmware version
		 * ACK doesn't follow the |0x0100 pattern consistently in the manual.
		 * Our parser checks status only. */
		0xA0, 0x01, 0x00, 0x00,  /* ack_cmd, status=0 */
		0x01, 0x00,  /* fw type = 1 */
		0x07, 0x01,  /* major (stored LE, but manual shows 0x0107 -> V1.07) */
		0x16, 0x15, 0x09, 0x22,  /* minor = 0x22091516 */
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));

	struct ld2410_firmware_version ver;
	int ret = ld2410_parse_firmware_version(&frame, &ver);
	zassert_equal(ret, 0);
	zassert_equal(ver.type, 1);
	/* major = get_le16({0x07, 0x01}) = 0x0107 = 263 */
	zassert_equal(ver.major, 0x0107);
	/* minor = get_le32({0x16, 0x15, 0x09, 0x22}) = 0x22091516 */
	zassert_equal(ver.minor, 0x22091516);
}

/* Test: Parse MAC address ACK (manual §2.2.13)
 * FD FC FB FA 0A 00 A5 01 00 00 8F 27 2E B8 0F 65 04 03 02 01 */
ZTEST(ld2410_protocol, test_parse_mac_address_ack)
{
	const uint8_t data[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x0A, 0x00,
		0xA5, 0x01, 0x00, 0x00,
		0x8F, 0x27, 0x2E, 0xB8, 0x0F, 0x65,
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));

	struct ld2410_mac_addr mac;
	int ret = ld2410_parse_mac_address(&frame, &mac);
	zassert_equal(ret, 0);

	const uint8_t expected_mac[] = { 0x8F, 0x27, 0x2E, 0xB8, 0x0F, 0x65 };
	zassert_mem_equal(mac.addr, expected_mac, 6);
}

/* Test: Parse distance resolution ACK (manual §2.2.17)
 * FD FC FB FA 06 00 AB 01 00 00 01 00 04 03 02 01
 * Resolution = 0.2m (index 0x0001) */
ZTEST(ld2410_protocol, test_parse_distance_resolution_ack)
{
	const uint8_t data[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x06, 0x00,
		0xAB, 0x01, 0x00, 0x00,
		0x01, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));

	enum ld2410_distance_resolution res;
	int ret = ld2410_parse_distance_resolution(&frame, &res);
	zassert_equal(ret, 0);
	zassert_equal(res, LD2410_RESOLUTION_0_20M);
}

/* Test: ACK status failure (status != 0) */
ZTEST(ld2410_protocol, test_parse_ack_failure_status)
{
	const uint8_t data[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0xFE, 0x01, 0x01, 0x00,  /* status = 1 = failure */
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));
	zassert_equal(ld2410_parse_ack_status(&frame, LD2410_CMD_END_CONFIG), -EIO);
}

/* Test: ACK cmd mismatch returns -EINVAL */
ZTEST(ld2410_protocol, test_parse_ack_cmd_mismatch)
{
	const uint8_t data[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0xFE, 0x01, 0x00, 0x00,  /* ACK for END_CONFIG */
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));

	/* Expect ENABLE_CONFIG ACK but got END_CONFIG ACK */
	zassert_equal(ld2410_parse_ack_status(&frame, LD2410_CMD_ENABLE_CONFIG), -EINVAL);
}

/* Test: parse_ack_status with NULL frame */
ZTEST(ld2410_protocol, test_parse_ack_null)
{
	zassert_equal(ld2410_parse_ack_status(NULL, 0x00FF), -EINVAL);
}

/* Test: parse_ack_status with wrong frame type */
ZTEST(ld2410_protocol, test_parse_ack_wrong_type)
{
	struct ld2410_raw_frame frame = {
		.type = LD2410_FRAME_DATA,
		.payload_len = 4,
		.payload = { 0xFF, 0x01, 0x00, 0x00 }
	};

	zassert_equal(ld2410_parse_ack_status(&frame, LD2410_CMD_ENABLE_CONFIG), -EINVAL);
}

/* Test: parse_ack_status with short payload */
ZTEST(ld2410_protocol, test_parse_ack_short_payload)
{
	struct ld2410_raw_frame frame = {
		.type = LD2410_FRAME_ACK,
		.payload_len = 2,  /* need 4 */
		.payload = { 0xFF, 0x01 }
	};

	zassert_equal(ld2410_parse_ack_status(&frame, LD2410_CMD_ENABLE_CONFIG), -ENODATA);
}

/* ===========================================================================
 * 3. Data Frame Parser Tests
 * ===========================================================================*/

/* Test: Normal mode data frame (manual §2.3.2 example)
 * F4 F3 F2 F1 0D 00 02 AA 02 51 00 00 00 00 3B 00 00 55 00 F8 F7 F6 F5
 *
 * type=0x02 (basic), state=0x02 (stationary), mov_dist=0x0051=81cm, mov_energy=0,
 * sta_dist=0x0000=0cm, sta_energy=0x3B=59, det_dist=0x0000=0cm */
ZTEST(ld2410_protocol, test_parse_data_basic)
{
	const uint8_t data[] = {
		0xF4, 0xF3, 0xF2, 0xF1,
		0x0D, 0x00,
		0x02, 0xAA,
		0x02,              /* state: stationary */
		0x51, 0x00,        /* moving dist: 81 cm */
		0x00,              /* moving energy: 0 */
		0x00, 0x00,        /* static dist: 0 cm */
		0x3B,              /* static energy: 59 */
		0x00, 0x00,        /* detection dist: 0 cm */
		0x55, 0x00,        /* tail + check */
		0xF8, 0xF7, 0xF6, 0xF5
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame raw;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &raw));
	zassert_equal(raw.type, LD2410_FRAME_DATA);

	struct ld2410_frame frame;
	int ret = ld2410_parse_data_frame(&raw, &frame);
	zassert_equal(ret, 0);

	zassert_equal(frame.type, LD2410_DATA_BASIC);
	zassert_equal(frame.target.state, LD2410_TARGET_STATIONARY);
	zassert_equal(frame.target.moving_distance_cm, 81);
	zassert_equal(frame.target.moving_energy, 0);
	zassert_equal(frame.target.static_distance_cm, 0);
	zassert_equal(frame.target.static_energy, 59);
	zassert_equal(frame.target.detection_distance_cm, 0);
}

/* Test: Engineering mode data frame (manual §2.3.2 example) */
ZTEST(ld2410_protocol, test_parse_data_engineering)
{
	const uint8_t data[] = {
		0xF4, 0xF3, 0xF2, 0xF1,
		0x23, 0x00,  /* payload len = 35 */
		/* payload: */
		0x01, 0xAA,
		0x03,              /* state: both */
		0x1E, 0x00,        /* moving dist: 30 cm */
		0x3C,              /* moving energy: 60 */
		0x00, 0x00,        /* static dist: 0 cm */
		0x39,              /* static energy: 57 */
		0x00, 0x00,        /* detection dist: 0 cm */
		0x08, 0x08,        /* max_mov=8, max_sta=8 */
		/* moving gate 0-8 energy: */
		0x3C, 0x22, 0x05, 0x03, 0x03, 0x04, 0x03, 0x06, 0x05,
		/* static gate 0-8 energy: */
		0x00, 0x00, 0x39, 0x10, 0x13, 0x06, 0x06, 0x08, 0x04,
		/* trailing retained data + tail + check */
		0x03, 0x05, 0x55, 0x00,
		0xF8, 0xF7, 0xF6, 0xF5
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame raw;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &raw));
	zassert_equal(raw.type, LD2410_FRAME_DATA);

	struct ld2410_frame frame;
	int ret = ld2410_parse_data_frame(&raw, &frame);
	zassert_equal(ret, 0);

	zassert_equal(frame.type, LD2410_DATA_ENGINEERING);
	zassert_equal(frame.target.state, LD2410_TARGET_BOTH);
	zassert_equal(frame.target.moving_distance_cm, 30);
	zassert_equal(frame.target.moving_energy, 60);
	zassert_equal(frame.target.static_distance_cm, 0);
	zassert_equal(frame.target.static_energy, 57);
	zassert_equal(frame.target.detection_distance_cm, 0);

	zassert_equal(frame.engineering.max_moving_gate, 8);
	zassert_equal(frame.engineering.max_static_gate, 8);

	/* Moving gate energies */
	const uint8_t exp_mov[] = { 0x3C, 0x22, 0x05, 0x03, 0x03, 0x04, 0x03, 0x06, 0x05 };
	zassert_mem_equal(frame.engineering.moving_energy, exp_mov, 9);

	/* Static gate energies */
	const uint8_t exp_sta[] = { 0x00, 0x00, 0x39, 0x10, 0x13, 0x06, 0x06, 0x08, 0x04 };
	zassert_mem_equal(frame.engineering.static_energy, exp_sta, 9);
}

/* Test: Data frame with no target (state = 0x00) */
ZTEST(ld2410_protocol, test_parse_data_no_target)
{
	const uint8_t data[] = {
		0xF4, 0xF3, 0xF2, 0xF1,
		0x0D, 0x00,
		0x02, 0xAA,
		0x00,              /* state: none */
		0x00, 0x00,        /* moving dist: 0 */
		0x00,              /* moving energy: 0 */
		0x00, 0x00,        /* static dist: 0 */
		0x00,              /* static energy: 0 */
		0x00, 0x00,        /* detection dist: 0 */
		0x55, 0x00,
		0xF8, 0xF7, 0xF6, 0xF5
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame raw;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &raw));

	struct ld2410_frame frame;
	zassert_equal(ld2410_parse_data_frame(&raw, &frame), 0);
	zassert_equal(frame.target.state, LD2410_TARGET_NONE);
	zassert_equal(frame.target.moving_distance_cm, 0);
	zassert_equal(frame.target.static_distance_cm, 0);
	zassert_equal(frame.target.detection_distance_cm, 0);
}

/* Test: Moving target at max distance */
ZTEST(ld2410_protocol, test_parse_data_moving_far)
{
	/* 600 cm = 0x0258 LE */
	const uint8_t data[] = {
		0xF4, 0xF3, 0xF2, 0xF1,
		0x0D, 0x00,
		0x02, 0xAA,
		0x01,              /* state: moving */
		0x58, 0x02,        /* moving dist: 600 cm */
		0x50,              /* moving energy: 80 */
		0x00, 0x00,        /* static dist: 0 */
		0x00,              /* static energy: 0 */
		0x58, 0x02,        /* detection dist: 600 cm */
		0x55, 0x00,
		0xF8, 0xF7, 0xF6, 0xF5
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame raw;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &raw));

	struct ld2410_frame frame;
	zassert_equal(ld2410_parse_data_frame(&raw, &frame), 0);
	zassert_equal(frame.target.state, LD2410_TARGET_MOVING);
	zassert_equal(frame.target.moving_distance_cm, 600);
	zassert_equal(frame.target.moving_energy, 80);
	zassert_equal(frame.target.detection_distance_cm, 600);
}

/* Test: parse_data_frame with NULL inputs */
ZTEST(ld2410_protocol, test_parse_data_null)
{
	struct ld2410_frame frame;
	struct ld2410_raw_frame raw = { .type = LD2410_FRAME_DATA, .payload_len = 0 };

	zassert_equal(ld2410_parse_data_frame(NULL, &frame), -EINVAL);
	zassert_equal(ld2410_parse_data_frame(&raw, NULL), -EINVAL);

	/* Wrong type */
	raw.type = LD2410_FRAME_ACK;
	raw.payload_len = 13;
	zassert_equal(ld2410_parse_data_frame(&raw, &frame), -EINVAL);
}

/* Test: data frame too short returns -ENODATA */
ZTEST(ld2410_protocol, test_parse_data_too_short)
{
	struct ld2410_raw_frame raw = {
		.type = LD2410_FRAME_DATA,
		.payload_len = 5,
		.payload = { 0x02, 0xAA, 0x00, 0x00, 0x00 }
	};

	struct ld2410_frame frame;
	zassert_equal(ld2410_parse_data_frame(&raw, &frame), -ENODATA);
}

/* Test: data frame missing 0xAA marker returns -EPROTO */
ZTEST(ld2410_protocol, test_parse_data_bad_head)
{
	struct ld2410_raw_frame raw = {
		.type = LD2410_FRAME_DATA,
		.payload_len = 13,
		.payload = { 0x02, 0xBB /* not 0xAA */, 0x00, 0x00, 0x00,
			     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x00 }
	};

	struct ld2410_frame frame;
	zassert_equal(ld2410_parse_data_frame(&raw, &frame), -EPROTO);
}

/* ===========================================================================
 * 4. Parser Edge Cases
 * ===========================================================================*/

/* Test: Garbage bytes before a valid frame */
ZTEST(ld2410_protocol, test_parser_garbage_prefix)
{
	const uint8_t data[] = {
		0x00, 0xFF, 0xAB, 0x12, 0x34,  /* garbage */
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0xFE, 0x01, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));
	zassert_equal(frame.type, LD2410_FRAME_ACK);
	zassert_equal(ld2410_parse_ack_status(&frame, LD2410_CMD_END_CONFIG), 0);
}

/* Test: Two consecutive frames */
ZTEST(ld2410_protocol, test_parser_consecutive_frames)
{
	const uint8_t data[] = {
		/* Frame 1: ACK for END_CONFIG */
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0xFE, 0x01, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01,
		/* Frame 2: Data frame (basic, no target) */
		0xF4, 0xF3, 0xF2, 0xF1,
		0x0D, 0x00,
		0x02, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x00,
		0xF8, 0xF7, 0xF6, 0xF5
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);

	/* Feed all bytes -- should get frame 1 first */
	size_t i = 0;
	bool got_first = false;

	for (; i < sizeof(data); i++) {
		if (ld2410_parser_feed(&parser, data[i], &frame)) {
			got_first = true;
			i++;
			break;
		}
	}

	zassert_true(got_first);
	zassert_equal(frame.type, LD2410_FRAME_ACK);

	/* Continue feeding -- should get frame 2 */
	bool got_second = false;

	for (; i < sizeof(data); i++) {
		if (ld2410_parser_feed(&parser, data[i], &frame)) {
			got_second = true;
			break;
		}
	}

	zassert_true(got_second);
	zassert_equal(frame.type, LD2410_FRAME_DATA);
}

/* Test: Interleaved garbage between frames */
ZTEST(ld2410_protocol, test_parser_garbage_between_frames)
{
	const uint8_t data[] = {
		/* Frame 1 */
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00, 0x60, 0x01, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01,
		/* Garbage */
		0xDE, 0xAD, 0xBE, 0xEF,
		/* Frame 2 */
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00, 0xA2, 0x01, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01,
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);

	int frame_count = 0;

	for (size_t i = 0; i < sizeof(data); i++) {
		if (ld2410_parser_feed(&parser, data[i], &frame)) {
			frame_count++;
		}
	}

	zassert_equal(frame_count, 2);
}

/* Test: Partial header restart (FD FD FC FB FA ...) */
ZTEST(ld2410_protocol, test_parser_partial_header_restart)
{
	const uint8_t data[] = {
		0xFD, /* start of header -- but next byte doesn't match FC */
		0xFD, 0xFC, 0xFB, 0xFA,  /* real header */
		0x04, 0x00,
		0xFE, 0x01, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));
	zassert_equal(frame.type, LD2410_FRAME_ACK);
}

/* Test: Empty payload (payload_len == 0) -- footer immediately after length */
ZTEST(ld2410_protocol, test_parser_empty_payload)
{
	const uint8_t data[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x00, 0x00,  /* payload len = 0 */
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, data, sizeof(data), &frame));
	zassert_equal(frame.type, LD2410_FRAME_ACK);
	zassert_equal(frame.payload_len, 0);
}

/* Test: Bad footer causes reset, next frame still works */
ZTEST(ld2410_protocol, test_parser_bad_footer_recovery)
{
	const uint8_t data[] = {
		/* Frame with wrong footer */
		0xFD, 0xFC, 0xFB, 0xFA,
		0x02, 0x00,
		0xFE, 0x00,
		0xFF, 0xFF, 0xFF, 0xFF,  /* wrong footer */
		/* Real frame after */
		0xFD, 0xFC, 0xFB, 0xFA,
		0x04, 0x00,
		0x60, 0x01, 0x00, 0x00,
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);

	int count = 0;

	for (size_t i = 0; i < sizeof(data); i++) {
		if (ld2410_parser_feed(&parser, data[i], &frame)) {
			count++;
		}
	}

	zassert_equal(count, 1); /* only the second frame succeeds */
	zassert_equal(frame.type, LD2410_FRAME_ACK);
}

/* ===========================================================================
 * 5. Inline Helper Tests
 * ===========================================================================*/

ZTEST(ld2410_protocol, test_gate_to_cm_075)
{
	zassert_equal(ld2410_gate_to_cm(0, LD2410_RESOLUTION_0_75M), 0);
	zassert_equal(ld2410_gate_to_cm(1, LD2410_RESOLUTION_0_75M), 75);
	zassert_equal(ld2410_gate_to_cm(4, LD2410_RESOLUTION_0_75M), 300);
	zassert_equal(ld2410_gate_to_cm(8, LD2410_RESOLUTION_0_75M), 600);
}

ZTEST(ld2410_protocol, test_gate_to_cm_020)
{
	zassert_equal(ld2410_gate_to_cm(0, LD2410_RESOLUTION_0_20M), 0);
	zassert_equal(ld2410_gate_to_cm(1, LD2410_RESOLUTION_0_20M), 20);
	zassert_equal(ld2410_gate_to_cm(5, LD2410_RESOLUTION_0_20M), 100);
	zassert_equal(ld2410_gate_to_cm(8, LD2410_RESOLUTION_0_20M), 160);
}

ZTEST(ld2410_protocol, test_target_state_str)
{
	zassert_str_equal(ld2410_target_state_str(LD2410_TARGET_NONE), "none");
	zassert_str_equal(ld2410_target_state_str(LD2410_TARGET_MOVING), "moving");
	zassert_str_equal(ld2410_target_state_str(LD2410_TARGET_STATIONARY), "stationary");
	zassert_str_equal(ld2410_target_state_str(LD2410_TARGET_BOTH), "moving+stationary");
	zassert_str_equal(ld2410_target_state_str((enum ld2410_target_state)0xFF), "unknown");
}

/* ===========================================================================
 * 6. CMD_TO_ACK Macro Test
 * ===========================================================================*/

ZTEST(ld2410_protocol, test_cmd_to_ack_macro)
{
	zassert_equal(LD2410_CMD_TO_ACK(0x00FF), 0x01FF);
	zassert_equal(LD2410_CMD_TO_ACK(0x00FE), 0x01FE);
	zassert_equal(LD2410_CMD_TO_ACK(0x0060), 0x0160);
	zassert_equal(LD2410_CMD_TO_ACK(0x0061), 0x0161);
	zassert_equal(LD2410_CMD_TO_ACK(0x00A0), 0x01A0);
	zassert_equal(LD2410_CMD_TO_ACK(0x00A5), 0x01A5);
	zassert_equal(LD2410_CMD_TO_ACK(0x00AB), 0x01AB);
}

/* ===========================================================================
 * 7. Read params edge cases
 * ===========================================================================*/

/* Missing 0xAA marker */
ZTEST(ld2410_protocol, test_parse_read_params_no_marker)
{
	struct ld2410_raw_frame frame = {
		.type = LD2410_FRAME_ACK,
		.payload_len = 28,
	};

	/* ACK header */
	frame.payload[0] = 0x61; frame.payload[1] = 0x01;
	frame.payload[2] = 0x00; frame.payload[3] = 0x00;
	/* Missing 0xAA -- put 0xBB instead */
	frame.payload[4] = 0xBB;
	frame.payload[5] = 0x08;
	frame.payload[6] = 0x08;
	frame.payload[7] = 0x08;

	struct ld2410_config_params params;
	zassert_equal(ld2410_parse_read_params(&frame, &params), -EPROTO);
}

/* Short payload for read_params */
ZTEST(ld2410_protocol, test_parse_read_params_short)
{
	struct ld2410_raw_frame frame = {
		.type = LD2410_FRAME_ACK,
		.payload_len = 10,  /* too short for 9 gates x 2 + duration */
	};

	frame.payload[0] = 0x61; frame.payload[1] = 0x01;
	frame.payload[2] = 0x00; frame.payload[3] = 0x00;
	frame.payload[4] = 0xAA; frame.payload[5] = 0x08;
	frame.payload[6] = 0x08; frame.payload[7] = 0x08;

	struct ld2410_config_params params;
	zassert_equal(ld2410_parse_read_params(&frame, &params), -ENODATA);
}

/* ===========================================================================
 * 8. Firmware version edge cases
 * ===========================================================================*/

ZTEST(ld2410_protocol, test_parse_firmware_null)
{
	struct ld2410_firmware_version ver;
	zassert_equal(ld2410_parse_firmware_version(NULL, &ver), -EINVAL);

	struct ld2410_raw_frame frame = { .type = LD2410_FRAME_ACK, .payload_len = 12 };
	zassert_equal(ld2410_parse_firmware_version(&frame, NULL), -EINVAL);
}

ZTEST(ld2410_protocol, test_parse_firmware_short)
{
	struct ld2410_raw_frame frame = {
		.type = LD2410_FRAME_ACK,
		.payload_len = 8,  /* need 12 */
	};

	struct ld2410_firmware_version ver;
	zassert_equal(ld2410_parse_firmware_version(&frame, &ver), -ENODATA);
}

ZTEST(ld2410_protocol, test_parse_firmware_failure_status)
{
	struct ld2410_raw_frame frame = {
		.type = LD2410_FRAME_ACK,
		.payload_len = 12,
	};
	/* status = 1 at [2..3] */
	memset(frame.payload, 0, 12);
	frame.payload[2] = 0x01;

	struct ld2410_firmware_version ver;
	zassert_equal(ld2410_parse_firmware_version(&frame, &ver), -EIO);
}

/* ===========================================================================
 * 9. Roundtrip: build -> parse
 * ===========================================================================*/

ZTEST(ld2410_protocol, test_roundtrip_build_then_parse)
{
	uint8_t frame_buf[LD2410_MAX_CMD_FRAME_SIZE];
	uint8_t val[] = { 0x01, 0x00 };
	int len = ld2410_build_cmd_frame(frame_buf, 0x00FF, val, sizeof(val));

	zassert_true(len > 0);

	/* The command frame has the same header/footer as ACK frames,
	 * so feeding it through the parser should produce an ACK-type frame */
	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;

	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, frame_buf, len, &frame));
	zassert_equal(frame.type, LD2410_FRAME_ACK);  /* cmd shares ACK framing */
	zassert_equal(frame.payload_len, 4);  /* 2 cmd word + 2 value */
	zassert_equal(frame.payload[0], 0xFF);
	zassert_equal(frame.payload[1], 0x00);
	zassert_equal(frame.payload[2], 0x01);
	zassert_equal(frame.payload[3], 0x00);
}

/* ===========================================================================
 * Suite 8: LD2410B/C Auxiliary Control & Extended Engineering Data
 * ===========================================================================*/

/* Test: build set_aux_control command frame (§2.2.18) */
ZTEST(ld2410_protocol, test_build_set_aux_control)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	/* Command value: light_below(0x01), threshold 0x60, OUT low default(0x00), reserved 0x00 */
	uint8_t value[] = { 0x01, 0x60, 0x00, 0x00 };
	int len = ld2410_build_cmd_frame(buf, LD2410_CMD_SET_AUX_CONTROL, value, 4);
	zassert_true(len > 0);

	/* Verify from manual example: FD FC FB FA 06 00 AD 00 01 60 00 00 04 03 02 01 */
	uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x06, 0x00,  /* length = 6 (2 cmd + 4 value) */
		0xAD, 0x00,  /* cmd word */
		0x01, 0x60, 0x00, 0x00,  /* value */
		0x04, 0x03, 0x02, 0x01   /* footer */
	};
	zassert_equal((size_t)len, sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: build query_aux_control command frame (§2.2.19) */
ZTEST(ld2410_protocol, test_build_query_aux_control)
{
	uint8_t buf[LD2410_MAX_CMD_FRAME_SIZE];
	int len = ld2410_build_cmd_frame(buf, LD2410_CMD_GET_AUX_CONTROL, NULL, 0);
	zassert_true(len > 0);

	/* FD FC FB FA 02 00 AE 00 04 03 02 01 */
	uint8_t expected[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x02, 0x00,
		0xAE, 0x00,
		0x04, 0x03, 0x02, 0x01
	};
	zassert_equal((size_t)len, sizeof(expected));
	zassert_mem_equal(buf, expected, sizeof(expected));
}

/* Test: parse aux control ACK (§2.2.19 example) */
ZTEST(ld2410_protocol, test_parse_aux_control_ack)
{
	/* From manual: FD FC FB FA 08 00 AE 01 00 00 01 60 01 00 04 03 02 01 */
	uint8_t raw[] = {
		0xFD, 0xFC, 0xFB, 0xFA,
		0x08, 0x00,
		0xAE, 0x01,       /* ACK for 0x00AE */
		0x00, 0x00,        /* status: success */
		0x01,              /* light mode: below threshold */
		0x60,              /* threshold: 0x60 */
		0x01,              /* OUT default: high */
		0x00,              /* reserved */
		0x04, 0x03, 0x02, 0x01
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame frame;
	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, raw, sizeof(raw), &frame));
	zassert_equal(frame.type, LD2410_FRAME_ACK);

	/* Verify ACK status */
	int ret = ld2410_parse_ack_status(&frame, LD2410_CMD_GET_AUX_CONTROL);
	zassert_equal(ret, 0);

	/* Verify the 4-byte config value in payload after status */
	/* payload: AE 01 00 00 01 60 01 00 */
	/* bytes [4..7] = config value */
	zassert_equal(frame.payload[4], 0x01);  /* light mode: below */
	zassert_equal(frame.payload[5], 0x60);  /* threshold */
	zassert_equal(frame.payload[6], 0x01);  /* OUT default: high */
	zassert_equal(frame.payload[7], 0x00);  /* reserved */
}

/* Test: LD2410B engineering mode data frame with light + OUT pin bytes */
ZTEST(ld2410_protocol, test_parse_data_engineering_b)
{
	/* From LD2410B V1.06 manual §2.3.2 engineering example:
	 * F4 F3 F2 F1  23 00
	 * 01 AA 03 1E 00 3C 00 00 39 00 00  (type+head+basic: state=03, mov=30cm/60, stat=0cm/57, det=0cm)
	 * 08 08                              (max_moving=8, max_static=8)
	 * 3C 22 05 03 03 04 03 06 05         (moving energy gates 0-8)
	 * 00 00 39 10 13 06 06 08 04         (static energy gates 0-8)
	 * 60 01                              (light=0x60, OUT=0x01)
	 * 55 00                              (tail + check)
	 * F8 F7 F6 F5
	 */
	uint8_t raw[] = {
		0xF4, 0xF3, 0xF2, 0xF1,
		0x23, 0x00,  /* len=35 */
		0x01, 0xAA,  /* type=engineering, head=0xAA */
		0x03,        /* state: both */
		0x1E, 0x00,  /* moving dist: 30 cm */
		0x3C,        /* moving energy: 60 */
		0x00, 0x00,  /* static dist: 0 cm */
		0x39,        /* static energy: 57 */
		0x00, 0x00,  /* detection dist: 0 cm */
		0x08,        /* max moving gate: 8 */
		0x08,        /* max static gate: 8 */
		0x3C, 0x22, 0x05, 0x03, 0x03, 0x04, 0x03, 0x06, 0x05,  /* mov energy g0-8 */
		0x00, 0x00, 0x39, 0x10, 0x13, 0x06, 0x06, 0x08, 0x04,  /* stat energy g0-8 */
		0x60,        /* light value */
		0x01,        /* OUT pin state: someone */
		0x55, 0x00,  /* tail + check */
		0xF8, 0xF7, 0xF6, 0xF5
	};

	struct ld2410_parser parser;
	struct ld2410_raw_frame raw_frame;
	ld2410_parser_init(&parser);
	zassert_true(feed_bytes(&parser, raw, sizeof(raw), &raw_frame));
	zassert_equal(raw_frame.type, LD2410_FRAME_DATA);

	struct ld2410_frame frame;
	int ret = ld2410_parse_data_frame(&raw_frame, &frame);
	zassert_equal(ret, 0);
	zassert_equal(frame.type, LD2410_DATA_ENGINEERING);
	zassert_equal(frame.target.state, LD2410_TARGET_BOTH);
	zassert_equal(frame.target.moving_distance_cm, 30);
	zassert_equal(frame.target.moving_energy, 60);
	zassert_equal(frame.target.static_energy, 57);

	/* Engineering gate energies */
	zassert_equal(frame.engineering.max_moving_gate, 8);
	zassert_equal(frame.engineering.max_static_gate, 8);
	zassert_equal(frame.engineering.moving_energy[0], 0x3C);
	zassert_equal(frame.engineering.moving_energy[8], 0x05);
	zassert_equal(frame.engineering.static_energy[0], 0x00);
	zassert_equal(frame.engineering.static_energy[2], 0x39);

	/* LD2410B-specific: light + OUT pin */
	zassert_equal(frame.engineering.light_value, 0x60);
	zassert_equal(frame.engineering.out_pin_state, 0x01);
}

/* ===========================================================================
 * Test Suite Registration
 * ===========================================================================*/

ZTEST_SUITE(ld2410_protocol, NULL, NULL, NULL, NULL, NULL);
