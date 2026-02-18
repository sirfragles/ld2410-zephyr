/*
 * Copyright (c) 2024 Matthew Macdonald-Wallace
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <ld2410/ld2410.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ---------------------------------------------------------------------------
 * Data callback — invoked from the driver RX thread
 * ---------------------------------------------------------------------------*/

static void on_radar_data(const struct ld2410_frame *frame, void *user_data)
{
	ARG_UNUSED(user_data);

	const struct ld2410_target_data *t = &frame->target;

	LOG_INF("[%s] mov=%ucm(E%u) sta=%ucm(E%u) det=%ucm",
		ld2410_target_state_str(t->state),
		t->moving_distance_cm, t->moving_energy,
		t->static_distance_cm, t->static_energy,
		t->detection_distance_cm);

	if (frame->type == LD2410_DATA_ENGINEERING) {
		const struct ld2410_engineering_data *e = &frame->engineering;

		for (int i = 0; i < LD2410_MAX_GATES; i++) {
			LOG_INF("  Gate %d: mov=%3u sta=%3u",
				i, e->moving_energy[i], e->static_energy[i]);
		}

		LOG_INF("  Light: %u  OUT: %s", e->light_value,
			e->out_pin_state ? "presence" : "clear");
	}
}

/* ---------------------------------------------------------------------------
 * Configuration callback
 * ---------------------------------------------------------------------------*/

static int my_config(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	int ret;

	/* Read firmware version */
	struct ld2410_firmware_version ver;

	ret = ld2410_read_firmware_version(dev, &ver);
	if (ret == 0) {
		LOG_INF("Firmware: type=0x%04x V%u.%08X",
			ver.type, ver.major, ver.minor);
	}

	/* Set detection range: max gate 6 (~4.5m), timeout 10s */
	ret = ld2410_set_max_gate_and_duration(dev, 6, 6, 10);
	if (ret) {
		LOG_ERR("Set max gate failed: %d", ret);
		return ret;
	}

	/* Set all gates: moving=40, stationary=30 */
	ret = ld2410_set_gate_sensitivity(dev, LD2410_ALL_GATES, 40, 30);
	if (ret) {
		LOG_ERR("Set sensitivity failed: %d", ret);
		return ret;
	}

	/* Enable engineering mode for per-gate energy data */
	ret = ld2410_engineering_mode_enable(dev);
	if (ret) {
		LOG_ERR("Engineering mode failed: %d", ret);
		return ret;
	}

	LOG_INF("Configuration complete");
	return 0;
}

/* ---------------------------------------------------------------------------
 * Main
 * ---------------------------------------------------------------------------*/

int main(void)
{
	const struct device *sensor = DEVICE_DT_GET_ANY(hilink_ld2410);

	if (!sensor || !device_is_ready(sensor)) {
		LOG_ERR("LD2410 sensor not ready");
		return -ENODEV;
	}

	LOG_INF("LD2410 mmWave sensor ready");

	/* Register data callback */
	ld2410_register_callback(sensor, on_radar_data, NULL);

	/* Wait for sensor to stabilize after power-on */
	k_sleep(K_SECONDS(3));

	/* Run configuration sequence */
	int ret = ld2410_configure(sensor, my_config, NULL);

	if (ret) {
		LOG_ERR("Configuration failed: %d", ret);
		return ret;
	}

	LOG_INF("Sensor ready — receiving data via callback");

	/* Data arrives via callback — nothing else to do */
	while (true) {
		k_sleep(K_SECONDS(60));
	}

	return 0;
}
