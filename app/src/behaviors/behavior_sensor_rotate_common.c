
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <zmk/behavior_queue.h>
#include <zmk/virtual_key_position.h>
#include <zmk/events/position_state_changed.h>

#include "behavior_sensor_rotate_common.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static struct k_mutex sensor_rotate_lock = Z_MUTEX_INITIALIZER(sensor_rotate_lock);

int zmk_behavior_sensor_rotate_common_accept_data(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
    const struct zmk_sensor_config *sensor_config, size_t channel_data_size,
    const struct zmk_sensor_channel_data *channel_data) {

    k_mutex_lock(&sensor_rotate_lock, K_FOREVER);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_sensor_rotate_data *data = dev->data;

    const struct sensor_value value = channel_data[0].value;
    int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    if (value.val1 == 0 && value.val2 == 0) {
        return 0;
    }
    
    struct sensor_value remainder = data->remainder[sensor_index][1];
    remainder.val1 += value.val1;
    remainder.val2 += value.val2;

    if (remainder.val2 >= 1000000 || remainder.val2 <= -1000000) {
        remainder.val1 += remainder.val2 / 1000000;
        remainder.val2 %= 1000000;
    }

    int trigger_degrees = 360 / sensor_config->triggers_per_rotation;
    int triggers = remainder.val1 / trigger_degrees;
    remainder.val1 %= trigger_degrees;

    if (triggers > 0) {
        remainder.val1 = 0;
        remainder.val2 = 0;
    }

    data->remainder[sensor_index][1] = remainder;
    data->triggers[sensor_index][1] = triggers;

    LOG_DBG("Device %p | Sensor[%d]: val1=%d val2=%d → triggers=%d | reminder.val1=%d reminder.val2=%d",
            dev, sensor_index, value.val1, value.val2, triggers, data->remainder[sensor_index][1].val1, data->remainder[sensor_index][1].val2);

    k_mutex_unlock(&sensor_rotate_lock);
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
        data->triggers[sensor_index][1] = 0;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    int triggers = data->triggers[sensor_index][1];

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
