/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *
 * @brief Simple Zigbee light bulb implementation. 
 * Manju's Extension
 * 1. Added Additional GPIO PIN 0.02 to enabled an additional LED. - Done
 * 2. Move the primary bulb control exposed in Zibgee Cluster to GPIO (from Standard onboard LED) - Done
 *   TO DO
 * 3. Create a seperate Endpoint for the GPIO PIN which is exposed as a bulb. This would achive seperate endpoints for the primary (onboard LED) and additional (GPIO) bulb. 
 *    This would also require the change of the configuration at Zigbee2MQTT to accept Multi-Endpoint Zigbee End Device (Curently it is configured for the single / default endpoint)
 * 4. Build the configuration for the NRF52840 Dongle - PC100059
 * 5. Resolve the runtime error of 'I: Unimplemented signal (signal: 50, status: 0)'  Although this is not causing any observable issue in functionality
 * Manju - Adapted for Nordic Dongle
 */

#include <zephyr/types.h>
#include <zephyr.h>
#include <device.h>
#include <soc.h>
#include <drivers/pwm.h>
#include <logging/log.h>
#include <dk_buttons_and_leds.h>
#include <settings/settings.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_zcl_scenes.h>
#include <zb_nrf_platform.h>
// Manju- 
// #include "nrf_delay.h"
// #include "nrf_gpio.h"

/* Device endpoint, used to receive light controlling commands. */
#define HA_DIMMABLE_LIGHT_ENDPOINT      10

// Manju - Additional Endpoint for GPIO P0.02
#define HA_DIMMABLE_LIGHT_ENDPOINT_GPIO      11

/* Version of the application software (1 byte). */
#define BULB_INIT_BASIC_APP_VERSION     01

/* Version of the implementation of the Zigbee stack (1 byte). */
#define BULB_INIT_BASIC_STACK_VERSION   10

/* Version of the hardware of the device (1 byte). */
#define BULB_INIT_BASIC_HW_VERSION      11

/* Manufacturer name (32 bytes). */
#define BULB_INIT_BASIC_MANUF_NAME      "DongleNordic"

/* Model number assigned by manufacturer (32-bytes long string). */
#define BULB_INIT_BASIC_MODEL_ID        "DongleLight"

/* First 8 bytes specify the date of manufacturer of the device
 * in ISO 8601 format (YYYYMMDD). The rest (8 bytes) are manufacturer specific.
 */
#define BULB_INIT_BASIC_DATE_CODE       "202201207777"

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define BULB_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_BATTERY

/* Describes the physical location of the device (16 bytes).
 * May be modified during commisioning process.
 */
#define BULB_INIT_BASIC_LOCATION_DESC   "ManjuHome"

/* Describes the type of physical environment.
 * For possible values see section 3.2.2.2.10 of ZCL specification.
 */
#define BULB_INIT_BASIC_PH_ENV          ZB_ZCL_BASIC_ENV_UNSPECIFIED

#define TOGGLE_SWITCH					DK_BTN3_MSK

/* Use onboard led4 to act as a light bulb.
 * The app.overlay file has this at node label "pwm_led3" in /pwmleds.
 */
#define PWM_DK_LED4_NODE                DT_NODELABEL(pwm_led3)
// Manju - Define the GPIO P 0.02 - picked from the overlay file or the LED from Dongle which is configured in the Overlay file
#define PWM_GPIO_LED_P0_02_NODE         DT_NODELABEL(pwm_led4)

/* Nordic PWM nodes don't have flags cells in their specifiers, so
 * this is just future-proofing.
 */
#define FLAGS_OR_ZERO(node) \
	COND_CODE_1(DT_PHA_HAS_CELL(node, pwms, flags), \
		    (DT_PWMS_FLAGS(node)), (0))

#if DT_NODE_HAS_STATUS(PWM_DK_LED4_NODE, okay)
/* Get the defines from overlay file. */
#define PWM_DK_LED4_CTLR                DT_PWMS_CTLR(PWM_DK_LED4_NODE)
#define PWM_DK_LED4_CHANNEL             DT_PWMS_CHANNEL(PWM_DK_LED4_NODE)
#define PWM_DK_LED4_FLAGS               FLAGS_OR_ZERO(PWM_DK_LED4_NODE)
#else
#error "Choose supported PWM driver"
#endif

// Manju - define for the GPIO P 0.02
#if DT_NODE_HAS_STATUS(PWM_GPIO_LED_P0_02_NODE, okay)
/* Get the defines from overlay file. */
#define PWM_GPIO_LED_P0_02_CTLR                DT_PWMS_CTLR(PWM_GPIO_LED_P0_02_NODE)
#define PWM_GPIO_LED_P0_02_CHANNEL             DT_PWMS_CHANNEL(PWM_GPIO_LED_P0_02_NODE)
#define PWM_GPIO_LED_P0_02_FLAGS               FLAGS_OR_ZERO(PWM_GPIO_LED_P0_02_NODE)
#else
#error "Choose supported PWM driver for GPIO P 0.02"
#endif

/* Led PWM period, calculated for 50 Hz signal - in microseconds. */
#define LED_PWM_PERIOD_US               (USEC_PER_SEC / 50U)

#ifndef ZB_ED_ROLE
#error Define ZB_END_DEVICE_ROLE to compile enddevice source code.
#endif

// #ifndef ZB_ROUTER_ROLE
// #error Define ZB_ROUTER_ROLE to compile router source code.
// #endif

// Manju - Declare the On_off_set_value function
static void on_off_set_value(zb_bool_t on, zb_uint8_t endpoint_invoked);

LOG_MODULE_REGISTER(app);

/* Main application customizable context.
 * Stores all settings and static values.
 */
typedef struct {
	zb_zcl_basic_attrs_ext_t         basic_attr;
	zb_zcl_identify_attrs_t          identify_attr;
	zb_zcl_scenes_attrs_t            scenes_attr;
	zb_zcl_groups_attrs_t            groups_attr;
	zb_zcl_on_off_attrs_t            on_off_attr;
	zb_zcl_level_control_attrs_t     level_control_attr;
} bulb_device_ctx_t;

/////////// DECLARATION FOR FIRST ENDPOINT - START /////////////////////////////////////
/* Zigbee device application context storage. */
static bulb_device_ctx_t dev_ctx;

/* Pointer to PWM device controlling leds with pwm signal. */
static const struct device *led_pwm_dev;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(
	identify_attr_list,
	&dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(
	groups_attr_list,
	&dev_ctx.groups_attr.name_support);

ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(
	scenes_attr_list,
	&dev_ctx.scenes_attr.scene_count,
	&dev_ctx.scenes_attr.current_scene,
	&dev_ctx.scenes_attr.current_group,
	&dev_ctx.scenes_attr.scene_valid,
	&dev_ctx.scenes_attr.name_support);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(
	basic_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.app_version,
	&dev_ctx.basic_attr.stack_version,
	&dev_ctx.basic_attr.hw_version,
	dev_ctx.basic_attr.mf_name,
	dev_ctx.basic_attr.model_id,
	dev_ctx.basic_attr.date_code,
	&dev_ctx.basic_attr.power_source,
	dev_ctx.basic_attr.location_id,
	&dev_ctx.basic_attr.ph_env,
	dev_ctx.basic_attr.sw_ver);

/* On/Off cluster attributes additions data */
ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
	on_off_attr_list,
	&dev_ctx.on_off_attr.on_off);

ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST(
	level_control_attr_list,
	&dev_ctx.level_control_attr.current_level,
	&dev_ctx.level_control_attr.remaining_time);

ZB_HA_DECLARE_DIMMABLE_LIGHT_CLUSTER_LIST(
	dimmable_light_clusters,
	basic_attr_list,
	identify_attr_list,
	groups_attr_list,
	scenes_attr_list,
	on_off_attr_list,
	level_control_attr_list);

ZB_HA_DECLARE_DIMMABLE_LIGHT_EP(
	dimmable_light_ep,
	HA_DIMMABLE_LIGHT_ENDPOINT,
	dimmable_light_clusters);

/////////// DECLARATION FOR FIRST ENDPOINT - END /////////////////////////////////////

/////////// MANJU - DECLARATION FOR SECOND ENDPOINT - START /////////////////////////////////////
/* Zigbee device application context storage. */
static bulb_device_ctx_t dev_ctx_gpio;

/* Pointer to PWM device controlling leds with pwm signal. */
static const struct device *led_pwm_dev_gpio;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(
	identify_attr_list_gpio,
	&dev_ctx_gpio.identify_attr.identify_time);

ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(
	groups_attr_list_gpio,
	&dev_ctx_gpio.groups_attr.name_support);

ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(
	scenes_attr_list_gpio,
	&dev_ctx_gpio.scenes_attr.scene_count,
	&dev_ctx_gpio.scenes_attr.current_scene,
	&dev_ctx_gpio.scenes_attr.current_group,
	&dev_ctx_gpio.scenes_attr.scene_valid,
	&dev_ctx_gpio.scenes_attr.name_support);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(
	basic_attr_list_gpio,
	&dev_ctx_gpio.basic_attr.zcl_version,
	&dev_ctx_gpio.basic_attr.app_version,
	&dev_ctx_gpio.basic_attr.stack_version,
	&dev_ctx_gpio.basic_attr.hw_version,
	dev_ctx_gpio.basic_attr.mf_name,
	dev_ctx_gpio.basic_attr.model_id,
	dev_ctx_gpio.basic_attr.date_code,
	&dev_ctx_gpio.basic_attr.power_source,
	dev_ctx_gpio.basic_attr.location_id,
	&dev_ctx_gpio.basic_attr.ph_env,
	dev_ctx_gpio.basic_attr.sw_ver);

/* On/Off cluster attributes additions data */
ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
	on_off_attr_list_gpio,
	&dev_ctx_gpio.on_off_attr.on_off);

ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST(
	level_control_attr_list_gpio,
	&dev_ctx_gpio.level_control_attr.current_level,
	&dev_ctx_gpio.level_control_attr.remaining_time);

ZB_HA_DECLARE_DIMMABLE_LIGHT_CLUSTER_LIST(
	dimmable_light_clusters_gpio,
	basic_attr_list_gpio,
	identify_attr_list_gpio,
	groups_attr_list_gpio,
	scenes_attr_list_gpio,
	on_off_attr_list_gpio,
	level_control_attr_list_gpio);

/** 
 * This declaration is commented out since defining another endpoint through the macro gives error - ZB_HA_DECLARE_DIMMABLE_LIGHT_EP. 
 ** Hence, seperate typedef is expanded struct zb_af_simple_desc_6_0_struc 
 **/ 
// ZB_HA_DECLARE_DIMMABLE_LIGHT_EP(
// 	dimmable_light_ep_gpio,
// 	HA_DIMMABLE_LIGHT_ENDPOINT_GPIO,
// 	dimmable_light_clusters_gpio);

/** 
 *  Expansion of ZB_HA_DECLARE_DIMMABLE_LIGHT_EP - START
 *  As the declaration of macro ZB_HA_DECLARE_DIMMABLE_LIGHT_EP causes error due to duplication of zb_af_simple_desc_6_0.
 *  Hence, the typedefs contained in the macro are expanded and appended with _struc
 **/ 
typedef ZB_PACKED_PRE struct zb_af_simple_desc_6_0_struc 
{ 
	zb_uint8_t endpoint; 
	zb_uint16_t app_profile_id; 
	zb_uint16_t app_device_id; 
	zb_bitfield_t app_device_version:4; 
	zb_bitfield_t reserved:4; 
	zb_uint8_t app_input_cluster_count; 
	zb_uint8_t app_output_cluster_count; 
	zb_uint16_t app_cluster_list[(6) + (0)]; 
} ZB_PACKED_STRUCT zb_af_simple_desc_6_0_t_instance;

ZB_AF_SIMPLE_DESC_TYPE(6, 0) simple_desc_dimmable_light_ep_gpio = 
	{ 
	  11, ZB_AF_HA_PROFILE_ID, ZB_HA_DIMMABLE_LIGHT_DEVICE_ID, ZB_HA_DEVICE_VER_DIMMABLE_LIGHT, 0, 6, 0, 
	  { 
	    ZB_ZCL_CLUSTER_ID_BASIC, ZB_ZCL_CLUSTER_ID_IDENTIFY, ZB_ZCL_CLUSTER_ID_SCENES, ZB_ZCL_CLUSTER_ID_GROUPS, ZB_ZCL_CLUSTER_ID_ON_OFF, ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL, 
	  }
	};
ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_infodevice_ctx_name, ZB_HA_DIMMABLE_LIGHT_REPORT_ATTR_COUNT);
ZBOSS_DEVICE_DECLARE_LEVEL_CONTROL_CTX(cvc_alarm_infodevice_ctx_name, ZB_HA_DIMMABLE_LIGHT_CVC_ATTR_COUNT);
ZB_AF_DECLARE_ENDPOINT_DESC(dimmable_light_ep_gpio, HA_DIMMABLE_LIGHT_ENDPOINT_GPIO, ZB_AF_HA_PROFILE_ID, 0, NULL, ZB_ZCL_ARRAY_SIZE(dimmable_light_clusters_gpio, zb_zcl_cluster_desc_t), dimmable_light_clusters_gpio, (zb_af_simple_desc_1_1_t*)&simple_desc_dimmable_light_ep_gpio, ZB_HA_DIMMABLE_LIGHT_REPORT_ATTR_COUNT, reporting_infodevice_ctx_name, ZB_HA_DIMMABLE_LIGHT_CVC_ATTR_COUNT, cvc_alarm_infodevice_ctx_name);
/** 
 *  Expansion of ZB_HA_DECLARE_DIMMABLE_LIGHT_EP - END
 **/

/////////// MANJU - DECLARATION FOR SECOND ENDPOINT - END /////////////////////////////////////

/**
 * Combine the declaration of multiple Endpoints 
 **/ 
ZBOSS_DECLARE_DEVICE_CTX_2_EP(dimmable_light_ctx, dimmable_light_ep, dimmable_light_ep_gpio);

/**
 * Declares the context of single endpoint 
 * Commented the below as the multiple endpoints declaration above with method _CTX_2_EP takes care of the context of both the Endpoints
 **/
// ZB_HA_DECLARE_DIMMABLE_LIGHT_CTX(dimmable_light_ctx,	dimmable_light_ep);

/**@brief Callback for button events.
 *
 * @param[in]   button_state  Bitmask containing buttons state.
 * @param[in]   has_changed   Bitmask containing buttons
 *                            that have changed their state.
 */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	// zb_ret_t zb_err_code;

	/* Calculate bitmask of buttons that are pressed
	 * and have changed their state.
	 */
	uint32_t buttons = button_state & has_changed;

 	if (buttons & TOGGLE_SWITCH)  // Manju - Toggle the LED bulb by getting the context from the Zigbee object
	{
		LOG_INF("Current LED setting is %d", dev_ctx.on_off_attr.on_off);

		if(dev_ctx.on_off_attr.on_off)
		{
			// on_off_set_value(ZB_FALSE);
		}
		else
		{
			// on_off_set_value(ZB_TRUE);
		}

		LOG_INF("Toggling Completed");
	}
}

/**@brief Function for initializing additional PWM leds. */
static void pwm_led_init(void)
{
	led_pwm_dev = DEVICE_DT_GET(PWM_DK_LED4_CTLR);
	if (!device_is_ready(led_pwm_dev)) {
		LOG_ERR("Error: PWM device %s is not ready",
			led_pwm_dev->name);
	}

	// Manju - Control the GPIO 0.02 
	led_pwm_dev_gpio = DEVICE_DT_GET(PWM_GPIO_LED_P0_02_CTLR);
	if (!device_is_ready(led_pwm_dev_gpio)) 
	{
		LOG_ERR("Error: PWM device GPIO 0.02 %s is not ready", led_pwm_dev_gpio->name);
	}
}

/**@brief Function for initializing LEDs and Buttons. */
static void configure_gpio(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}

	pwm_led_init();
}

/**
 * Set the brightness for the respective LED based on the ENDPOINT INVOKED
 * @brief Sets brightness of bulb luminous executive element
 *
 * @param[in] brightness_level Brightness level, allowed values 0 ... 255,
 *                             0 - turn off, 255 - full brightness.
 */
static void light_bulb_set_brightness(zb_uint8_t brightness_level, zb_uint8_t ENDPOINT_INVOKED)
{
	uint32_t pulse = brightness_level * LED_PWM_PERIOD_US / 255U;

	if (ENDPOINT_INVOKED == HA_DIMMABLE_LIGHT_ENDPOINT) 
	{
		if (pwm_pin_set_usec(led_pwm_dev, PWM_DK_LED4_CHANNEL, LED_PWM_PERIOD_US, pulse, PWM_DK_LED4_FLAGS))
		{
			LOG_ERR("Pwm led 4 set fails:\n");
			return;
		}
	}
	else if (ENDPOINT_INVOKED == HA_DIMMABLE_LIGHT_ENDPOINT_GPIO) 
	{
		if(pwm_pin_set_usec(led_pwm_dev_gpio, PWM_GPIO_LED_P0_02_CHANNEL, LED_PWM_PERIOD_US, pulse, PWM_GPIO_LED_P0_02_FLAGS))
		{
			LOG_ERR("GPIO P 0.02 set fails:\n");
			return;
		}
	}
	else
	{
		LOG_INF("None of the endpoint was called");
	}
}

/**
 * Set the level of respective LED based on the Endpoint invoked
 * @brief Function for setting the light bulb brightness.
 *
 * @param[in] new_level   Light bulb brightness value.
 */
static void level_control_set_value(zb_uint16_t new_level, zb_uint8_t ENDPOINT_INVOKED)
{
	LOG_INF("Set level value: %i", new_level);

	ZB_ZCL_SET_ATTRIBUTE(
		ENDPOINT_INVOKED,
		ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
		(zb_uint8_t *)&new_level,
		ZB_FALSE);

	/* According to the table 7.3 of Home Automation Profile Specification
	 * v 1.2 rev 29, chapter 7.1.3.
	 */
	if (new_level == 0) {
		zb_uint8_t value = ZB_FALSE;

		ZB_ZCL_SET_ATTRIBUTE(
			ENDPOINT_INVOKED,
			ZB_ZCL_CLUSTER_ID_ON_OFF,
			ZB_ZCL_CLUSTER_SERVER_ROLE,
			ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			&value,
			ZB_FALSE);
	} else {
		zb_uint8_t value = ZB_TRUE;

		ZB_ZCL_SET_ATTRIBUTE(
			ENDPOINT_INVOKED,
			ZB_ZCL_CLUSTER_ID_ON_OFF,
			ZB_ZCL_CLUSTER_SERVER_ROLE,
			ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			&value,
			ZB_FALSE);
	}

	light_bulb_set_brightness(new_level, ENDPOINT_INVOKED);
}

/**
 * Sets the respective LEDs ON or OFF depending upon the ENDPOINT INVOKED 
 * @brief Function for turning ON/OFF the light bulb.
 *
 * @param[in]   on   Boolean light bulb state.
 */
static void on_off_set_value(zb_bool_t on, zb_uint8_t ENDPOINT_INVOKED)
{
	LOG_INF("Set ON/OFF value: %i", on);

	ZB_ZCL_SET_ATTRIBUTE(
		ENDPOINT_INVOKED,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&on,
		ZB_FALSE);

	if (on) 
	{
		level_control_set_value( dev_ctx.level_control_attr.current_level, ENDPOINT_INVOKED);
	} 
	else 
	{
		light_bulb_set_brightness(0U, ENDPOINT_INVOKED);
	}
}

/**@brief Function for initializing all clusters attributes.
 */
static void bulb_clusters_attr_init(void)
{
	/* Basic cluster attributes data */
	dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.app_version   = BULB_INIT_BASIC_APP_VERSION;
	dev_ctx.basic_attr.stack_version = BULB_INIT_BASIC_STACK_VERSION;
	dev_ctx.basic_attr.hw_version    = BULB_INIT_BASIC_HW_VERSION;

	/* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte
	 * should contain string length without trailing zero.
	 *
	 * For example "test" string wil be encoded as:
	 *   [(0x4), 't', 'e', 's', 't']
	 */
	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.mf_name,
		BULB_INIT_BASIC_MANUF_NAME,
		ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MANUF_NAME));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.model_id,
		BULB_INIT_BASIC_MODEL_ID,
		ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MODEL_ID));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.date_code,
		BULB_INIT_BASIC_DATE_CODE,
		ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_DATE_CODE));

	dev_ctx.basic_attr.power_source = BULB_INIT_BASIC_POWER_SOURCE;

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.location_id,
		BULB_INIT_BASIC_LOCATION_DESC,
		ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_LOCATION_DESC));

	dev_ctx.basic_attr.ph_env = BULB_INIT_BASIC_PH_ENV;

	/* Identify cluster attributes data. */
	dev_ctx.identify_attr.identify_time =
		ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

	/* On/Off cluster attributes data. */
	dev_ctx.on_off_attr.on_off = (zb_bool_t)ZB_ZCL_ON_OFF_IS_ON;  // Manju - Original code. Only useful for the initial setting to OFF for the LED
	// dev_ctx.on_off_attr.on_off = ZB_FALSE;  // Manju - Setting this option sets the intial value to ON - which may not be desirable, hence the above original which starts the LED in OFF position to start with

	dev_ctx.level_control_attr.current_level =
		ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE;
	dev_ctx.level_control_attr.remaining_time =
		ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;

	ZB_ZCL_SET_ATTRIBUTE(
		HA_DIMMABLE_LIGHT_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&dev_ctx.on_off_attr.on_off,
		ZB_FALSE);

	ZB_ZCL_SET_ATTRIBUTE(
		HA_DIMMABLE_LIGHT_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
		(zb_uint8_t *)&dev_ctx.level_control_attr.current_level,
		ZB_FALSE);
}

/**@brief Callback function for handling ZCL commands.
 *
 * @param[in]   bufid   Reference to Zigbee stack buffer
 *                      used to pass received data.
 */
static void zcl_device_cb(zb_bufid_t bufid)
{
	zb_uint8_t cluster_id;
	zb_uint8_t attr_id;
	zb_uint8_t endpoint_invoked;	
	zb_zcl_device_callback_param_t  *device_cb_param =
		ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);
	
	endpoint_invoked = device_cb_param->endpoint;

	LOG_INF("%s id %hd", __func__, device_cb_param->device_cb_id);

	LOG_INF("EndPoint Called is %d ", endpoint_invoked);

	/* Set default response value. */
	device_cb_param->status = RET_OK;
	LOG_INF("ZB Callback started %c", bufid);	

	switch (device_cb_param->device_cb_id) {
	case ZB_ZCL_LEVEL_CONTROL_SET_VALUE_CB_ID:
		// Set the level_control_set_value of the respective LED based on the endpoint invoked
		level_control_set_value(device_cb_param->cb_param.level_control_set_value_param.new_value, endpoint_invoked);
		break;

	case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
	    LOG_INF("Inside the ZB_ZCL_SET_ATTR_VALUE_CB_ID");
		cluster_id = device_cb_param->cb_param.set_attr_value_param.cluster_id;
		attr_id = device_cb_param->cb_param.set_attr_value_param.attr_id;

		if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF)
		{
			uint8_t value = device_cb_param->cb_param.set_attr_value_param.values.data8;

			LOG_INF("on/off attribute setting to %hd", value);
			if (attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) 
			{
				// on_off_set_value((zb_bool_t)value);
				// Manju - call the on_off_set_value with the endpoint invoked information
				on_off_set_value((zb_bool_t)value, endpoint_invoked);
			}
		} 
		else if (cluster_id == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) 
		{
			uint16_t value = device_cb_param->cb_param.set_attr_value_param.values.data16;

			LOG_INF("level control attribute setting to %hd",value);
			if (attr_id == ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID) 
			{
				// Call the  level_control_set_value for the respective LEDs and pass on the endpoint invoked
				level_control_set_value(value, endpoint_invoked);
			}
		} else 
		{
			/* Other clusters can be processed here */
			LOG_INF("Unhandled cluster attribute id: %d",
				cluster_id);
		}
		break;

	default:
		if (zcl_scenes_cb(bufid) == ZB_FALSE) 
		{
			device_cb_param->status = RET_ERROR;
		}
		break;
	}

	LOG_INF("%s status: %hd", __func__, device_cb_param->status);
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
	/* No application-specific behavior is required.
	 * Call default signal handler.
	 */
	ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

	/* All callbacks should either reuse or free passed buffers.
	 * If bufid == 0, the buffer is invalid (not passed).
	 */
	if (bufid) {
		zb_buf_free(bufid);
	}
}

void error(void)
{
	while (true) {
		/* Spin forever */
		k_sleep(K_MSEC(1000));
	}
}

void main(void)
{
	int err;

	LOG_INF("Starting Manju Light Bulb example");

	/* Initialize */
	configure_gpio();
	err = settings_subsys_init();
	if (err) {
		LOG_ERR("settings initialization failed");
	}

	LOG_INF("Registering Device");
	/* Register callback for handling ZCL commands. */
	ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);

	LOG_INF("Setting Light Context");
	/* Register dimmer switch device context (all the endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&dimmable_light_ctx);

	bulb_clusters_attr_init();
	// Initially set the level of the LED based on each of the ENDPOINT
	level_control_set_value(dev_ctx.level_control_attr.current_level, HA_DIMMABLE_LIGHT_ENDPOINT);
	level_control_set_value(dev_ctx.level_control_attr.current_level, HA_DIMMABLE_LIGHT_ENDPOINT_GPIO);
	

	LOG_INF("Bulb Cluster initiated and initial level set %d",dev_ctx.level_control_attr.current_level );

	/* Initialize ZCL scene table */
	zcl_scenes_init();

	LOG_INF("ZCL scenes initiated");

	/* Settings should be loaded after zcl_scenes_init */
	err = settings_load();
	if (err) {
		LOG_ERR("settings loading failed");
	}

	/* Start Zigbee default thread */
	zigbee_enable();

	LOG_INF("Manju Light Bulb example started");

	while (1) {
		// k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}