/*
 * Copyright (c) 2024 Matthew Macdonald-Wallace
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file ld2410.h
 * @brief LD2410 24GHz mmWave human presence sensor driver.
 *
 * Supports: LD2410, LD2410B, LD2410C
 *
 * Protocol references:
 *   - HLK-LD2410  Serial Communication Protocol V1.02
 *   - HLK-LD2410B Serial Communication Protocol V1.06
 *   - 24GHz mmWave Sensor for XIAO User Manual V1.00
 *
 * @defgroup ld2410 LD2410 mmWave Presence Sensor
 * @ingroup sensor_interface
 * @{
 */

#ifndef LD2410_LD2410_H_
#define LD2410_LD2410_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------------*/

/** Maximum number of distance gates supported by the sensor */
#define LD2410_MAX_GATES            9  /* gates 0..8 */

/** Distance per gate in default resolution (0.75 m) */
#define LD2410_GATE_DISTANCE_CM_75  75

/** Distance per gate in fine resolution (0.20 m) */
#define LD2410_GATE_DISTANCE_CM_20  20

/** Default Bluetooth password */
#define LD2410_BT_DEFAULT_PASSWORD  "HiLink"

/** Gate value used to configure ALL gates at once */
#define LD2410_ALL_GATES            0xFFFF

/* ---------------------------------------------------------------------------
 * Enumerations
 * ---------------------------------------------------------------------------*/

/** Target state reported by the sensor */
enum ld2410_target_state {
	LD2410_TARGET_NONE       = 0x00, /**< No target detected */
	LD2410_TARGET_MOVING     = 0x01, /**< Moving target detected */
	LD2410_TARGET_STATIONARY = 0x02, /**< Stationary target detected */
	LD2410_TARGET_BOTH       = 0x03, /**< Both moving & stationary targets */
};

/** Baud rate selection indices (matches protocol Table 6) */
enum ld2410_baud_rate {
	LD2410_BAUD_9600   = 0x0001,
	LD2410_BAUD_19200  = 0x0002,
	LD2410_BAUD_38400  = 0x0003,
	LD2410_BAUD_57600  = 0x0004,
	LD2410_BAUD_115200 = 0x0005,
	LD2410_BAUD_230400 = 0x0006,
	LD2410_BAUD_256000 = 0x0007, /**< Factory default */
	LD2410_BAUD_460800 = 0x0008,
};

/** Distance resolution selection (matches protocol Table 8) */
enum ld2410_distance_resolution {
	LD2410_RESOLUTION_0_75M = 0x0000, /**< 0.75 m per gate */
	LD2410_RESOLUTION_0_20M = 0x0001, /**< 0.20 m per gate */
};

/** Light-sense auxiliary control mode (LD2410B/C only) */
enum ld2410_aux_light_mode {
	LD2410_AUX_LIGHT_OFF        = 0x00, /**< Light-sense disabled */
	LD2410_AUX_LIGHT_BELOW      = 0x01, /**< Condition met when light < threshold */
	LD2410_AUX_LIGHT_ABOVE      = 0x02, /**< Condition met when light > threshold */
};

/** OUT pin default level / polarity (LD2410B/C only) */
enum ld2410_out_pin_polarity {
	LD2410_OUT_LOW_DEFAULT      = 0x00, /**< LOW=no target, HIGH=target (default) */
	LD2410_OUT_HIGH_DEFAULT     = 0x01, /**< HIGH=no target, LOW=target (inverted) */
};

/** Data frame type */
enum ld2410_data_type {
	LD2410_DATA_ENGINEERING = 0x01,
	LD2410_DATA_BASIC       = 0x02,
};

/* ---------------------------------------------------------------------------
 * Data structures
 * ---------------------------------------------------------------------------*/

/** Basic target information (always present in reports) */
struct ld2410_target_data {
	enum ld2410_target_state state;
	uint16_t moving_distance_cm;   /**< Distance to moving target (cm) */
	uint8_t  moving_energy;        /**< Moving target energy (0-100) */
	uint16_t static_distance_cm;   /**< Distance to stationary target (cm) */
	uint8_t  static_energy;        /**< Stationary target energy (0-100) */
	uint16_t detection_distance_cm;/**< Detection distance (cm) */
};

/** Engineering mode additional data (per-gate energy values) */
struct ld2410_engineering_data {
	uint8_t max_moving_gate;       /**< Configured max moving distance gate */
	uint8_t max_static_gate;       /**< Configured max static distance gate */
	uint8_t moving_energy[LD2410_MAX_GATES];   /**< Energy per gate (moving) */
	uint8_t static_energy[LD2410_MAX_GATES];   /**< Energy per gate (static) */
	uint8_t light_value;           /**< Photosensitive detection value (0-255, LD2410B/C) */
	uint8_t out_pin_state;         /**< OUT pin status (0=no one, 1=someone, LD2410B/C) */
};

/** Complete radar report frame */
struct ld2410_frame {
	enum ld2410_data_type type;
	struct ld2410_target_data target;
	struct ld2410_engineering_data engineering; /**< Valid only when type == ENGINEERING */
};

/** Configuration parameters read from the sensor */
struct ld2410_config_params {
	uint8_t max_moving_gate;       /**< Configured max moving distance gate */
	uint8_t max_static_gate;       /**< Configured max static distance gate */
	uint8_t moving_sensitivity[LD2410_MAX_GATES];
	uint8_t static_sensitivity[LD2410_MAX_GATES];
	uint16_t no_one_duration_s;    /**< Unoccupied duration (seconds) */
};

/** Firmware version info */
struct ld2410_firmware_version {
	uint16_t type;                 /**< Firmware type */
	uint16_t major;                /**< Major version */
	uint32_t minor;                /**< Minor version (encoded date) */
};

/** MAC address */
struct ld2410_mac_addr {
	uint8_t addr[6];
};

/** Auxiliary control function configuration (LD2410B/C) */
struct ld2410_aux_control {
	enum ld2410_aux_light_mode light_mode;  /**< Light-sense mode */
	uint8_t light_threshold;                /**< Light-sense threshold (0-255) */
	enum ld2410_out_pin_polarity out_polarity; /**< OUT pin default level */
};

/**
 * Callback invoked when a new data frame is received from the sensor.
 *
 * @param frame  Pointer to the parsed frame data (valid only during callback).
 * @param user_data  User-supplied context pointer.
 */
typedef void (*ld2410_data_cb_t)(const struct ld2410_frame *frame, void *user_data);

/* ---------------------------------------------------------------------------
 * Callback Registration
 * ---------------------------------------------------------------------------*/

/**
 * @brief Register a callback for incoming data frames.
 *
 * The callback is invoked from the driver's RX processing thread.
 *
 * @param dev        Pointer to the LD2410 device.
 * @param cb         Callback function.
 * @param user_data  Context passed to the callback.
 *
 * @retval 0       Success.
 * @retval -EINVAL Invalid arguments.
 */
int ld2410_register_callback(const struct device *dev, ld2410_data_cb_t cb,
			     void *user_data);

/* ---------------------------------------------------------------------------
 * Configuration Commands (Section 2.2)
 * ---------------------------------------------------------------------------*/

/**
 * @brief Enable configuration mode.
 *
 * Must be called before any other configuration command.
 *
 * @param dev  Pointer to the LD2410 device.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_config_enable(const struct device *dev);

/**
 * @brief End configuration mode and resume normal operation.
 *
 * @param dev  Pointer to the LD2410 device.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_config_end(const struct device *dev);

/**
 * @brief Set maximum distance gates and unoccupied duration.
 *
 * @param dev                Pointer to the LD2410 device.
 * @param max_moving_gate    Maximum moving detection gate (2-8).
 * @param max_static_gate    Maximum stationary detection gate (2-8).
 * @param no_one_duration_s  Unoccupied duration in seconds (0-65535).
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_set_max_gate_and_duration(const struct device *dev,
				     uint8_t max_moving_gate,
				     uint8_t max_static_gate,
				     uint16_t no_one_duration_s);

/**
 * @brief Read current configuration parameters from the sensor.
 *
 * @param dev     Pointer to the LD2410 device.
 * @param params  Output: populated with current configuration.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_read_config(const struct device *dev,
		       struct ld2410_config_params *params);

/**
 * @brief Enable engineering mode (adds per-gate energy to reports).
 *
 * This setting is lost on power cycle.
 *
 * @param dev  Pointer to the LD2410 device.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_engineering_mode_enable(const struct device *dev);

/**
 * @brief Disable engineering mode.
 *
 * @param dev  Pointer to the LD2410 device.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_engineering_mode_disable(const struct device *dev);

/**
 * @brief Configure sensitivity for a specific distance gate.
 *
 * @param dev               Pointer to the LD2410 device.
 * @param gate              Gate index (0-8) or LD2410_ALL_GATES for all.
 * @param moving_sensitivity  Moving target sensitivity (0-100).
 * @param static_sensitivity  Stationary target sensitivity (0-100).
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_set_gate_sensitivity(const struct device *dev, uint16_t gate,
				uint8_t moving_sensitivity,
				uint8_t static_sensitivity);

/**
 * @brief Read the sensor firmware version.
 *
 * @param dev   Pointer to the LD2410 device.
 * @param ver   Output: firmware version information.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_read_firmware_version(const struct device *dev,
				 struct ld2410_firmware_version *ver);

/**
 * @brief Set the UART baud rate of the sensor.
 *
 * Takes effect after module restart. The host UART must also be reconfigured.
 *
 * @param dev   Pointer to the LD2410 device.
 * @param baud  Baud rate selection.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_set_baud_rate(const struct device *dev, enum ld2410_baud_rate baud);

/**
 * @brief Restore all configuration to factory defaults.
 *
 * Takes effect after module restart.
 *
 * @param dev  Pointer to the LD2410 device.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_factory_reset(const struct device *dev);

/**
 * @brief Restart the sensor module.
 *
 * @param dev  Pointer to the LD2410 device.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_restart(const struct device *dev);

/**
 * @brief Enable or disable Bluetooth on the sensor.
 *
 * Takes effect after module restart.
 *
 * @param dev     Pointer to the LD2410 device.
 * @param enable  true to enable, false to disable.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_bluetooth_set(const struct device *dev, bool enable);

/**
 * @brief Get the sensor's Bluetooth MAC address.
 *
 * @param dev   Pointer to the LD2410 device.
 * @param mac   Output: MAC address.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_get_mac_address(const struct device *dev,
			   struct ld2410_mac_addr *mac);

/**
 * @brief Authenticate with the sensor over Bluetooth.
 *
 * @param dev       Pointer to the LD2410 device.
 * @param password  6-character password string (default: "HiLink").
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 *
 * @note ACK is returned via Bluetooth only, not serial.
 */
int ld2410_bluetooth_authenticate(const struct device *dev,
				  const char *password);

/**
 * @brief Set the Bluetooth control password.
 *
 * @param dev       Pointer to the LD2410 device.
 * @param password  6-character password string.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_bluetooth_set_password(const struct device *dev,
				  const char *password);

/**
 * @brief Set the distance resolution.
 *
 * Takes effect after module restart. Persists across power cycles.
 *
 * @param dev         Pointer to the LD2410 device.
 * @param resolution  Distance resolution selection.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_set_distance_resolution(const struct device *dev,
				   enum ld2410_distance_resolution resolution);

/**
 * @brief Query the current distance resolution setting.
 *
 * @param dev         Pointer to the LD2410 device.
 * @param resolution  Output: current distance resolution.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_get_distance_resolution(const struct device *dev,
				   enum ld2410_distance_resolution *resolution);

/**
 * @brief Set the auxiliary control function configuration.
 *
 * Controls OUT pin polarity and optional light-sense gating.
 * Requires configuration mode (config_enable / config_end).
 * Not supported on the original LD2410 (no photodiode).
 *
 * @param dev   Pointer to the LD2410 device.
 * @param cfg   Auxiliary control configuration.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_set_aux_control(const struct device *dev,
			   const struct ld2410_aux_control *cfg);

/**
 * @brief Query the current auxiliary control function configuration.
 *
 * @param dev   Pointer to the LD2410 device.
 * @param cfg   Output: current configuration.
 * @retval 0          Success.
 * @retval -ETIMEDOUT No ACK received.
 * @retval -EIO       Sensor returned failure status.
 */
int ld2410_get_aux_control(const struct device *dev,
			   struct ld2410_aux_control *cfg);

/* ---------------------------------------------------------------------------
 * Convenience helpers
 * ---------------------------------------------------------------------------*/

/**
 * @brief Execute a complete configuration sequence.
 *
 * Calls ld2410_config_enable(), the user callback, then ld2410_config_end().
 *
 * @param dev       Pointer to the LD2410 device.
 * @param config_fn  Function containing configuration commands.
 * @param user_data  Passed to config_fn.
 * @retval 0 Success, or first negative errno encountered.
 */
int ld2410_configure(const struct device *dev,
		     int (*config_fn)(const struct device *dev, void *user_data),
		     void *user_data);

/* ---------------------------------------------------------------------------
 * OUT Pin — Hardware presence GPIO
 * ---------------------------------------------------------------------------
 * Available when the devicetree node includes the "out-gpios" property.
 * The driver automatically configures the GPIO during initialization.
 * ---------------------------------------------------------------------------*/

/**
 * @brief Read the current state of the OUT pin.
 *
 * Requires "out-gpios" property in the devicetree node.
 *
 * @param dev  Pointer to the LD2410 device.
 * @retval 1        Presence detected (pin active).
 * @retval 0        No presence (pin inactive).
 * @retval -ENOTSUP OUT pin not configured in devicetree.
 */
int ld2410_out_pin_read(const struct device *dev);

/**
 * @brief Register a GPIO interrupt callback for the OUT pin.
 *
 * The callback fires on both rising (presence detected) and falling
 * (presence cleared) edges. Requires "out-gpios" property in devicetree.
 *
 * @param dev  Pointer to the LD2410 device.
 * @param cb   Zephyr GPIO callback struct (caller-allocated, persistent).
 * @param handler  Callback handler function.
 * @retval 0          Success.
 * @retval -ENOTSUP   OUT pin not configured in devicetree.
 */
int ld2410_out_pin_set_interrupt(const struct device *dev,
				 struct gpio_callback *cb,
				 gpio_callback_handler_t handler);

/**
 * @brief Convert a gate index and resolution to distance in centimeters.
 *
 * @param gate        Gate index (0-8).
 * @param resolution  Distance resolution.
 * @return Distance in centimeters.
 */
static inline uint16_t ld2410_gate_to_cm(uint8_t gate,
					 enum ld2410_distance_resolution resolution)
{
	return gate * ((resolution == LD2410_RESOLUTION_0_20M) ?
		       LD2410_GATE_DISTANCE_CM_20 : LD2410_GATE_DISTANCE_CM_75);
}

/**
 * @brief Get a human-readable string for a target state.
 */
static inline const char *ld2410_target_state_str(enum ld2410_target_state state)
{
	switch (state) {
	case LD2410_TARGET_NONE:       return "none";
	case LD2410_TARGET_MOVING:     return "moving";
	case LD2410_TARGET_STATIONARY: return "stationary";
	case LD2410_TARGET_BOTH:       return "moving+stationary";
	default:                       return "unknown";
	}
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* LD2410_LD2410_H_ */
