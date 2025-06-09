
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <zmk/behavior_queue.h>
#include <zmk/virtual_key_position.h>
#include <zmk/events/position_state_changed.h>

#include "behavior_sensor_rotate_common.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

int zmk_behavior_sensor_rotate_common_accept_data(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
    const struct zmk_sensor_config *sensor_config, size_t channel_data_size,
    const struct zmk_sensor_channel_data *channel_data) {

    LOG_DBG("Entering function: %s", __func__);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    LOG_DBG("Got device: %p for behavior_dev: %p", dev, binding->behavior_dev);

    struct behavior_sensor_rotate_data *data = dev->data;
    LOG_DBG("Loaded behavior sensor rotate data: %p", data);

    const struct sensor_value value = channel_data[0].value;
    LOG_DBG("Sensor value: val1=%d, val2=%d", value.val1, value.val2);

    int triggers;
    int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);
    LOG_DBG("Calculated sensor_index: %d from position: %d", sensor_index, event.position);

    if (value.val1 == 0) {
        // Old EC11 encoder behavior
        triggers = value.val2;
        LOG_DBG("Using legacy EC11 encoder behavior, triggers = val2 = %d", triggers);
    } else {
        struct sensor_value remainder = data->remainder[sensor_index][event.layer];
        LOG_DBG("Current remainder: val1=%d, val2=%d", remainder.val1, remainder.val2);

        remainder.val1 += value.val1;
        remainder.val2 += value.val2;
        LOG_DBG("Updated remainder (post-add): val1=%d, val2=%d", remainder.val1, remainder.val2);

        if (remainder.val2 >= 1000000 || remainder.val2 <= -1000000) {
            remainder.val1 += remainder.val2 / 1000000;
            remainder.val2 %= 1000000;
            LOG_DBG("Normalized remainder: val1=%d, val2=%d", remainder.val1, remainder.val2);
        }

        int trigger_degrees = 360 / sensor_config->triggers_per_rotation;
        LOG_DBG("Calculated trigger_degrees: %d from triggers_per_rotation: %d", trigger_degrees, sensor_config->triggers_per_rotation);

        triggers = remainder.val1 / trigger_degrees;
        remainder.val1 %= trigger_degrees;
        LOG_DBG("Calculated triggers: %d, new remainder.val1: %d", triggers, remainder.val1);

        data->remainder[sensor_index][event.layer] = remainder;
        LOG_DBG("Stored new remainder into data structure");
    }

    LOG_DBG("Final triggers: %d, inc keycode: 0x%02X, dec keycode: 0x%02X", triggers, binding->param1, binding->param2);

    data->triggers[sensor_index][event.layer] = triggers;
    LOG_DBG("Stored triggers[%d][%d] = %d", sensor_index, event.layer, triggers);

    LOG_DBG("Exiting function: %s", __func__);
    return 0;
}

int zmk_behavior_sensor_rotate_common_process(struct zmk_behavior_binding *binding,
                                              struct zmk_behavior_binding_event event,
                                              enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_rotate_config *cfg = dev->config;
    struct behavior_sensor_rotate_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->triggers[sensor_index][event.layer] = 0;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    int triggers = data->triggers[sensor_index][event.layer];

    struct zmk_behavior_binding triggered_binding;
    if (triggers > 0) {
        triggered_binding = cfg->cw_binding;
        if (cfg->override_params) {
            triggered_binding.param1 = binding->param1;
        }
    } else if (triggers < 0) {
        triggers = -triggers;
        triggered_binding = cfg->ccw_binding;
        if (cfg->override_params) {
            triggered_binding.param1 = binding->param2;
        }
    } else {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    LOG_DBG("Sensor binding: %s", binding->behavior_dev);

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    // set this value so that it always triggers on central, can be handled more properly later
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    for (int i = 0; i < triggers; i++) {
        zmk_behavior_queue_add(&event, triggered_binding, true, cfg->tap_ms);
        zmk_behavior_queue_add(&event, triggered_binding, false, 0);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}
