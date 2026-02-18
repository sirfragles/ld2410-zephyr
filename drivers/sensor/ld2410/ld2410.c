/*
 * Copyright (c) 2024 Matthew Macdonald-Wallace
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hilink_ld2410

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include <ld2410/ld2410.h>
#include <ld2410/ld2410_protocol.h>

LOG_MODULE_REGISTER(ld2410, CONFIG_LD2410_LOG_LEVEL);

/* ---------------------------------------------------------------------------
 * Internal structures
 * ---------------------------------------------------------------------------*/

/** DT-derived configuration (const, per-instance) */
struct ld2410_config {
	const struct device *uart_dev;
	struct gpio_dt_spec out_gpio;
	bool has_out_gpio;
};

/** Runtime data (mutable, per-instance) */
struct ld2410_data {
	/* RX ring buffer fed by ISR */
	struct ring_buf rx_ring;
	uint8_t rx_ring_buf[CONFIG_LD2410_RX_BUF_SIZE];

	/* Parser */
	struct ld2410_parser parser;

	/* RX processing thread */
	struct k_thread rx_thread;
	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_LD2410_THREAD_STACK_SIZE);
	struct k_sem rx_sem;  /* posted by ISR when data arrives */

	/* Command/response synchronisation */
	struct k_sem cmd_sem; /* binary: protects command sequences */
	struct k_sem ack_sem; /* posted when ACK frame received */
	struct ld2410_raw_frame ack_frame;

	/* User data callback */
	ld2410_data_cb_t data_cb;
	void *data_cb_user;
};

/* ---------------------------------------------------------------------------
 * UART ISR callback
 * ---------------------------------------------------------------------------*/

static void ld2410_uart_isr(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;
	struct ld2410_data *data = dev->data;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {
		uint8_t buf[32];
		int len = uart_fifo_read(uart_dev, buf, sizeof(buf));

		if (len > 0) {
			ring_buf_put(&data->rx_ring, buf, len);
			k_sem_give(&data->rx_sem);
		}
	}
}

/* ---------------------------------------------------------------------------
 * RX processing thread
 * ---------------------------------------------------------------------------*/

static void ld2410_rx_thread_fn(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct ld2410_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&data->rx_sem, K_FOREVER);

		uint8_t byte;

		while (ring_buf_get(&data->rx_ring, &byte, 1) == 1) {
			struct ld2410_raw_frame frame;

			if (ld2410_parser_feed(&data->parser, byte, &frame)) {
				if (frame.type == LD2410_FRAME_ACK) {
					/* Store ACK and wake command waiter */
					memcpy(&data->ack_frame, &frame,
					       sizeof(frame));
					k_sem_give(&data->ack_sem);
				} else if (frame.type == LD2410_FRAME_DATA) {
					/* Parse and deliver to user */
					if (data->data_cb) {
						struct ld2410_frame parsed;

						if (ld2410_parse_data_frame(
							    &frame,
							    &parsed) == 0) {
							data->data_cb(
								&parsed,
								data->data_cb_user);
						}
					}
				}
			}
		}
	}
}

/* ---------------------------------------------------------------------------
 * TX helper
 * ---------------------------------------------------------------------------*/

static int ld2410_uart_send(const struct device *dev, const uint8_t *buf,
			    size_t len)
{
	const struct ld2410_config *config = dev->config;

	for (size_t i = 0; i < len; i++) {
		uart_poll_out(config->uart_dev, buf[i]);
	}

	return 0;
}

/* ---------------------------------------------------------------------------
 * Command helper — send command, wait for ACK
 * ---------------------------------------------------------------------------*/

static int ld2410_send_cmd(const struct device *dev, uint16_t cmd_word,
			   const uint8_t *cmd_data, size_t cmd_data_len,
			   struct ld2410_raw_frame *ack_out)
{
	struct ld2410_data *data = dev->data;
	uint8_t frame_buf[LD2410_MAX_CMD_FRAME_SIZE];

	int frame_len = ld2410_build_cmd_frame(frame_buf, cmd_word,
					       cmd_data, cmd_data_len);

	if (frame_len < 0) {
		return frame_len;
	}

	/* Reset ACK semaphore (drain any stale signals) */
	k_sem_reset(&data->ack_sem);

	/* Transmit */
	ld2410_uart_send(dev, frame_buf, frame_len);

	LOG_DBG("Sent cmd 0x%04X (%d bytes)", cmd_word, frame_len);

	/* Wait for ACK */
	int ret = k_sem_take(&data->ack_sem,
			     K_MSEC(CONFIG_LD2410_CMD_TIMEOUT_MS));

	if (ret) {
		LOG_ERR("Timeout waiting for ACK to cmd 0x%04X", cmd_word);
		return -ETIMEDOUT;
	}

	if (ack_out) {
		memcpy(ack_out, &data->ack_frame, sizeof(*ack_out));
	}

	return 0;
}

/**
 * Send a simple command (no value) and check ACK status.
 */
static int ld2410_simple_cmd(const struct device *dev, uint16_t cmd_word)
{
	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, cmd_word, NULL, 0, &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_ack_status(&ack, cmd_word);
}

/* ---------------------------------------------------------------------------
 * Device init
 * ---------------------------------------------------------------------------*/

static int ld2410_init_fn(const struct device *dev)
{
	const struct ld2410_config *config = dev->config;
	struct ld2410_data *data = dev->data;

	if (!device_is_ready(config->uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	/* Initialise ring buffer */
	ring_buf_init(&data->rx_ring, sizeof(data->rx_ring_buf),
		      data->rx_ring_buf);

	/* Initialise parser */
	ld2410_parser_init(&data->parser);

	/* Initialise semaphores */
	k_sem_init(&data->rx_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&data->cmd_sem, 1, 1);  /* binary mutex */
	k_sem_init(&data->ack_sem, 0, 1);

	/* Start RX thread */
	k_thread_create(&data->rx_thread, data->rx_stack,
			CONFIG_LD2410_THREAD_STACK_SIZE,
			ld2410_rx_thread_fn, (void *)dev, NULL, NULL,
			CONFIG_LD2410_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, dev->name);

	/* Configure UART ISR — pass the ld2410 device as user_data */
	uart_irq_callback_user_data_set(config->uart_dev, ld2410_uart_isr,
					(void *)dev);
	uart_irq_rx_enable(config->uart_dev);

	/* Auto-configure OUT GPIO if specified in devicetree */
	if (config->has_out_gpio) {
		if (!gpio_is_ready_dt(&config->out_gpio)) {
			LOG_ERR("OUT GPIO device not ready");
			return -ENODEV;
		}
		int ret = gpio_pin_configure_dt(&config->out_gpio, GPIO_INPUT);
		if (ret) {
			LOG_ERR("Failed to configure OUT GPIO: %d", ret);
			return ret;
		}
	}

	LOG_INF("LD2410 driver initialized");

	return 0;
}

/* ---------------------------------------------------------------------------
 * Public API — Callback
 * ---------------------------------------------------------------------------*/

int ld2410_register_callback(const struct device *dev, ld2410_data_cb_t cb,
			     void *user_data)
{
	struct ld2410_data *data = dev->data;

	data->data_cb = cb;
	data->data_cb_user = user_data;

	return 0;
}

/* ---------------------------------------------------------------------------
 * Public API — Configuration commands
 * ---------------------------------------------------------------------------*/

int ld2410_config_enable(const struct device *dev)
{
	struct ld2410_data *data = dev->data;

	k_sem_take(&data->cmd_sem, K_FOREVER);

	uint8_t val[2] = { 0x01, 0x00 }; /* 0x0001 LE */
	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_ENABLE_CONFIG, val, 2, &ack);

	if (ret) {
		k_sem_give(&data->cmd_sem);
		return ret;
	}

	ret = ld2410_parse_ack_status(&ack, LD2410_CMD_ENABLE_CONFIG);
	if (ret) {
		k_sem_give(&data->cmd_sem);
	}

	/* cmd_sem stays taken while in config mode */
	return ret;
}

int ld2410_config_end(const struct device *dev)
{
	struct ld2410_data *data = dev->data;
	int ret = ld2410_simple_cmd(dev, LD2410_CMD_END_CONFIG);

	k_sem_give(&data->cmd_sem);

	return ret;
}

int ld2410_set_max_gate_and_duration(const struct device *dev,
				     uint8_t max_moving_gate,
				     uint8_t max_static_gate,
				     uint16_t no_one_duration_s)
{
	uint8_t cmd_val[18];

	cmd_val[0] = 0x00; cmd_val[1] = 0x00; /* param word 0x0000 */
	cmd_val[2] = max_moving_gate;
	cmd_val[3] = 0x00; cmd_val[4] = 0x00; cmd_val[5] = 0x00;

	cmd_val[6] = 0x01; cmd_val[7] = 0x00; /* param word 0x0001 */
	cmd_val[8] = max_static_gate;
	cmd_val[9] = 0x00; cmd_val[10] = 0x00; cmd_val[11] = 0x00;

	cmd_val[12] = 0x02; cmd_val[13] = 0x00; /* param word 0x0002 */
	cmd_val[14] = (uint8_t)(no_one_duration_s & 0xFF);
	cmd_val[15] = (uint8_t)((no_one_duration_s >> 8) & 0xFF);
	cmd_val[16] = 0x00; cmd_val[17] = 0x00;

	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_SET_MAX_GATE,
				  cmd_val, sizeof(cmd_val), &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_ack_status(&ack, LD2410_CMD_SET_MAX_GATE);
}

int ld2410_read_config(const struct device *dev,
		       struct ld2410_config_params *params)
{
	if (!params) {
		return -EINVAL;
	}

	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_READ_PARAMS,
				  NULL, 0, &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_read_params(&ack, params);
}

int ld2410_engineering_mode_enable(const struct device *dev)
{
	return ld2410_simple_cmd(dev, LD2410_CMD_ENABLE_ENGINEERING);
}

int ld2410_engineering_mode_disable(const struct device *dev)
{
	return ld2410_simple_cmd(dev, LD2410_CMD_DISABLE_ENGINEERING);
}

int ld2410_set_gate_sensitivity(const struct device *dev, uint16_t gate,
				uint8_t moving_sensitivity,
				uint8_t static_sensitivity)
{
	uint8_t cmd_val[18];

	cmd_val[0] = 0x00; cmd_val[1] = 0x00; /* param word 0x0000 */
	cmd_val[2] = (uint8_t)(gate & 0xFF);
	cmd_val[3] = (uint8_t)((gate >> 8) & 0xFF);
	cmd_val[4] = 0x00; cmd_val[5] = 0x00;

	cmd_val[6] = 0x01; cmd_val[7] = 0x00; /* param word 0x0001 */
	cmd_val[8] = moving_sensitivity;
	cmd_val[9] = 0x00; cmd_val[10] = 0x00; cmd_val[11] = 0x00;

	cmd_val[12] = 0x02; cmd_val[13] = 0x00; /* param word 0x0002 */
	cmd_val[14] = static_sensitivity;
	cmd_val[15] = 0x00; cmd_val[16] = 0x00; cmd_val[17] = 0x00;

	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_SET_GATE_SENS,
				  cmd_val, sizeof(cmd_val), &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_ack_status(&ack, LD2410_CMD_SET_GATE_SENS);
}

int ld2410_read_firmware_version(const struct device *dev,
				 struct ld2410_firmware_version *ver)
{
	if (!ver) {
		return -EINVAL;
	}

	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_READ_FIRMWARE,
				  NULL, 0, &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_firmware_version(&ack, ver);
}

int ld2410_set_baud_rate(const struct device *dev, enum ld2410_baud_rate baud)
{
	uint8_t val[2] = { (uint8_t)(baud & 0xFF), (uint8_t)((baud >> 8) & 0xFF) };
	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_SET_BAUD_RATE,
				  val, 2, &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_ack_status(&ack, LD2410_CMD_SET_BAUD_RATE);
}

int ld2410_factory_reset(const struct device *dev)
{
	return ld2410_simple_cmd(dev, LD2410_CMD_FACTORY_RESET);
}

int ld2410_restart(const struct device *dev)
{
	return ld2410_simple_cmd(dev, LD2410_CMD_RESTART);
}

int ld2410_bluetooth_set(const struct device *dev, bool enable)
{
	uint8_t val[2] = { enable ? 0x01 : 0x00, 0x00 };
	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_BLUETOOTH_SET,
				  val, 2, &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_ack_status(&ack, LD2410_CMD_BLUETOOTH_SET);
}

int ld2410_get_mac_address(const struct device *dev,
			   struct ld2410_mac_addr *mac)
{
	if (!mac) {
		return -EINVAL;
	}

	uint8_t val[2] = { 0x01, 0x00 }; /* 0x0001 */
	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_GET_MAC, val, 2, &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_mac_address(&ack, mac);
}

int ld2410_bluetooth_authenticate(const struct device *dev,
				  const char *password)
{
	if (!password) {
		return -EINVAL;
	}

	size_t pw_len = strlen(password);

	if (pw_len != 6) {
		return -EINVAL;
	}

	uint8_t val[6];

	for (int i = 0; i < 6; i += 2) {
		val[i]     = password[i];
		val[i + 1] = password[i + 1];
	}

	struct ld2410_raw_frame ack;

	/* Note: ACK comes via Bluetooth, not serial — this may timeout */
	return ld2410_send_cmd(dev, LD2410_CMD_BT_AUTHENTICATE,
			       val, sizeof(val), &ack);
}

int ld2410_bluetooth_set_password(const struct device *dev,
				  const char *password)
{
	if (!password) {
		return -EINVAL;
	}

	size_t pw_len = strlen(password);

	if (pw_len != 6) {
		return -EINVAL;
	}

	uint8_t val[6];

	for (int i = 0; i < 6; i += 2) {
		val[i]     = password[i];
		val[i + 1] = password[i + 1];
	}

	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_BT_SET_PASSWORD,
				  val, sizeof(val), &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_ack_status(&ack, LD2410_CMD_BT_SET_PASSWORD);
}

int ld2410_set_distance_resolution(const struct device *dev,
				   enum ld2410_distance_resolution resolution)
{
	uint8_t val[2] = {
		(uint8_t)(resolution & 0xFF),
		(uint8_t)((resolution >> 8) & 0xFF)
	};
	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_SET_RESOLUTION,
				  val, 2, &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_ack_status(&ack, LD2410_CMD_SET_RESOLUTION);
}

int ld2410_get_distance_resolution(const struct device *dev,
				   enum ld2410_distance_resolution *resolution)
{
	if (!resolution) {
		return -EINVAL;
	}

	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_GET_RESOLUTION,
				  NULL, 0, &ack);

	if (ret) {
		return ret;
	}

	return ld2410_parse_distance_resolution(&ack, resolution);
}

/* ---------------------------------------------------------------------------
 * Public API — Auxiliary control (LD2410B/C)
 * ---------------------------------------------------------------------------*/

int ld2410_set_aux_control(const struct device *dev,
			   const struct ld2410_aux_control *cfg)
{
	if (!cfg) {
		return -EINVAL;
	}

	uint8_t val[4] = {
		(uint8_t)cfg->light_mode,
		cfg->light_threshold,
		(uint8_t)cfg->out_polarity,
		0x00  /* reserved */
	};

	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_SET_AUX_CONTROL,
				  val, sizeof(val), &ack);
	if (ret) {
		return ret;
	}

	return ld2410_parse_ack_status(&ack, LD2410_CMD_SET_AUX_CONTROL);
}

int ld2410_get_aux_control(const struct device *dev,
			   struct ld2410_aux_control *cfg)
{
	if (!cfg) {
		return -EINVAL;
	}

	struct ld2410_raw_frame ack;
	int ret = ld2410_send_cmd(dev, LD2410_CMD_GET_AUX_CONTROL,
				  NULL, 0, &ack);
	if (ret) {
		return ret;
	}

	return ld2410_parse_aux_control(&ack, cfg);
}

/* ---------------------------------------------------------------------------
 * Convenience
 * ---------------------------------------------------------------------------*/

int ld2410_configure(const struct device *dev,
		     int (*config_fn)(const struct device *dev, void *user_data),
		     void *user_data)
{
	int ret = ld2410_config_enable(dev);

	if (ret) {
		return ret;
	}

	ret = config_fn(dev, user_data);

	int end_ret = ld2410_config_end(dev);

	return ret ? ret : end_ret;
}

/* ---------------------------------------------------------------------------
 * Public API — OUT pin (hardware presence GPIO)
 * ---------------------------------------------------------------------------*/

int ld2410_out_pin_read(const struct device *dev)
{
	const struct ld2410_config *config = dev->config;

	if (!config->has_out_gpio) {
		return -ENOTSUP;
	}

	return gpio_pin_get_dt(&config->out_gpio);
}

int ld2410_out_pin_set_interrupt(const struct device *dev,
				 struct gpio_callback *cb,
				 gpio_callback_handler_t handler)
{
	const struct ld2410_config *config = dev->config;

	if (!cb || !handler) {
		return -EINVAL;
	}

	if (!config->has_out_gpio) {
		return -ENOTSUP;
	}

	int ret = gpio_pin_interrupt_configure_dt(&config->out_gpio,
						  GPIO_INT_EDGE_BOTH);
	if (ret) {
		LOG_ERR("Failed to configure OUT pin interrupt: %d", ret);
		return ret;
	}

	gpio_init_callback(cb, handler, BIT(config->out_gpio.pin));

	ret = gpio_add_callback(config->out_gpio.port, cb);
	if (ret) {
		LOG_ERR("Failed to add OUT pin callback: %d", ret);
		return ret;
	}

	LOG_INF("OUT pin interrupt enabled");

	return 0;
}

/* ---------------------------------------------------------------------------
 * Device instantiation
 * ---------------------------------------------------------------------------*/

#define LD2410_INIT(inst)                                                        \
	static const struct ld2410_config ld2410_config_##inst = {               \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                   \
		.out_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, out_gpios, {0}),     \
		.has_out_gpio = DT_INST_NODE_HAS_PROP(inst, out_gpios),         \
	};                                                                       \
                                                                                 \
	static struct ld2410_data ld2410_data_##inst;                            \
                                                                                 \
	DEVICE_DT_INST_DEFINE(inst, ld2410_init_fn, NULL,                        \
			      &ld2410_data_##inst, &ld2410_config_##inst,        \
			      POST_KERNEL, CONFIG_LD2410_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(LD2410_INIT)
