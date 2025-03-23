import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, text_sensor, i2c, binary_sensor, button
from esphome.core import CORE, coroutine_with_priority
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    # CONF_MIN_TEMPERATURE, # Temperature sensors removed
    # CONF_MAX_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_OCCUPANCY,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    CONF_TIMEOUT,
    UNIT_METER, 
    ICON_ARROW_EXPAND_VERTICAL,
    ICON_RESTART,
    #CONF_PIXEL_DATA
)

CONF_I2C_ADDRESS = "address"
CONF_REFRESH_RATE = "refresh_rate"
CONF_SDA = "sda"
CONF_SCL = "scl"
CONF_FREQUENCY = "frequency"
# Temperature sensors removed
# CONF_MEAN_TEMPERATURE = "mean_temperature"
# CONF_MEDIAN_TEMPERATURE = "median_temperature"
CONF_FILTER_LEVEL = "filter_level"
CONF_ENABLE_MONITORING = "enable_monitoring"
CONF_LOOP_DURATION = "loop_duration"
# Debug sensors removed

# Occupancy detection configuration options
CONF_OCCUPANCY = "occupancy"
CONF_TEMP_DIFF_THRESHOLD = "temp_diff_threshold"
CONF_BLOB_SIZE_THRESHOLD = "blob_size_threshold"
CONF_HUMAN_TEMP_DIFF_MIN = "human_temp_diff_min"
CONF_HUMAN_TEMP_DIFF_MAX = "human_temp_diff_max"
CONF_EXIT_DEBOUNCE_FRAMES = "exit_debounce_frames"
CONF_BLOB_PROXIMITY_THRESHOLD = "blob_proximity_threshold"
CONF_CALIBRATE_ON_BOOT = "calibrate_on_boot"
CONF_CALIBRATION_FRAMES = "calibration_frames"
CONF_BLOB_SIZE = "blob_size"
CONF_TEMP_DIFF = "temp_diff"
CONF_BLOB_DETAILS = "blob_details"
CONF_NON_OCCUPIED_BLOB_DETAILS = "non_occupied_blob_details"
CONF_ALGORITHM_TYPE = "algorithm_type"
# CONF_CALIBRATION_BUTTON = "calibration" - Removed, now using standalone button


DEPENDENCIES = ['esp32']

mlx90640_ns = cg.esphome_ns.namespace("mlx90640_app")
MLX90640 = mlx90640_ns.class_("MLX90640", cg.PollingComponent)

CONFIG_SCHEMA = (
    cv.Schema({
      cv.GenerateID(): cv.declare_id(MLX90640),
      cv.Required(CONF_SCL): int,
      cv.Required(CONF_SDA): int,
      cv.Required(CONF_FREQUENCY): int,
      cv.Required(CONF_I2C_ADDRESS): int,
      cv.Optional(CONF_REFRESH_RATE): int,
      cv.Optional(CONF_FILTER_LEVEL): float,
      cv.Optional(CONF_ENABLE_MONITORING, default=True): cv.boolean,
      # Temperature sensors removed

      # Occupancy detection configuration
        cv.Optional(CONF_TEMP_DIFF_THRESHOLD, default=2.0): cv.float_,
        cv.Optional(CONF_BLOB_SIZE_THRESHOLD, default=50): cv.int_,
        cv.Optional(CONF_HUMAN_TEMP_DIFF_MIN, default=2.0): cv.float_,
        cv.Optional(CONF_HUMAN_TEMP_DIFF_MAX, default=5.0): cv.float_,
        cv.Optional(CONF_EXIT_DEBOUNCE_FRAMES, default=5): cv.int_,
        cv.Optional(CONF_BLOB_PROXIMITY_THRESHOLD, default=1): cv.int_,
        # Removed CONF_CALIBRATE_ON_BOOT as calibration starts automatically
        cv.Optional(CONF_CALIBRATION_FRAMES, default=10): cv.int_,
        cv.Optional(CONF_ALGORITHM_TYPE, default="baseline"): cv.one_of("baseline", "knn", lower=True),
        cv.Optional(CONF_OCCUPANCY): binary_sensor.binary_sensor_schema(
                    device_class=DEVICE_CLASS_OCCUPANCY,
                ),
        cv.Optional(CONF_BLOB_SIZE): sensor.sensor_schema(
                    unit_of_measurement="px",
                    accuracy_decimals=0,
                    state_class=STATE_CLASS_MEASUREMENT,
                ),
        cv.Optional(CONF_TEMP_DIFF): sensor.sensor_schema(
                    unit_of_measurement=UNIT_CELSIUS,
                    device_class=DEVICE_CLASS_TEMPERATURE,
                    accuracy_decimals=1,
                    state_class=STATE_CLASS_MEASUREMENT,
                ),
        cv.Optional(CONF_LOOP_DURATION): sensor.sensor_schema(
                    unit_of_measurement="ms",
                    icon="mdi:timer-outline",
                    accuracy_decimals=0,
                    state_class=STATE_CLASS_MEASUREMENT,
                ),
        cv.Optional(CONF_BLOB_DETAILS): text_sensor.text_sensor_schema(
                    icon="mdi:motion-sensor",
                ),
        cv.Optional(CONF_NON_OCCUPIED_BLOB_DETAILS): text_sensor.text_sensor_schema(
                    icon="mdi:motion-sensor-off",
                ),
        # Debug sensors removed
        # Removed calibration button - now implemented as standalone button
    }).extend(cv.polling_component_schema("60s"))
    #.extend(i2c.i2c_device_schema(CONF_I2C_ADDR))
)



@coroutine_with_priority(45.0)
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Temperature sensors removed
        
    if CONF_I2C_ADDRESS in config:
        addr = config[CONF_I2C_ADDRESS]
        cg.add(var.set_addr(addr))
    if CONF_SDA in config:
        sda = config[CONF_SDA]
        cg.add(var.set_sda(sda))
    if CONF_SCL in config:
        scl = config[CONF_SCL]
        cg.add(var.set_scl(scl))
    if CONF_FREQUENCY in config:
        freq = config[CONF_FREQUENCY]
        cg.add(var.set_frequency(freq))

    if CONF_REFRESH_RATE in config:
        refresh = config[CONF_REFRESH_RATE]
        cg.add(var.set_refresh_rate(refresh))

    if CONF_FILTER_LEVEL in config:
        level = config[CONF_FILTER_LEVEL]
        cg.add(var.set_filter_level(level))
    
    # Set monitoring flag
    enable_monitoring = config[CONF_ENABLE_MONITORING]
    cg.add(var.set_enable_monitoring(enable_monitoring))
    
    # Add loop duration sensor if configured
    if CONF_LOOP_DURATION in config:
        conf = config[CONF_LOOP_DURATION]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_loop_duration_sensor(sens))
        
    # Occupancy detection configuration
        cg.add(var.set_temp_diff_threshold(config[CONF_TEMP_DIFF_THRESHOLD]))
        cg.add(var.set_blob_size_threshold(config[CONF_BLOB_SIZE_THRESHOLD]))
        cg.add(var.set_human_temp_diff_min(config[CONF_HUMAN_TEMP_DIFF_MIN]))
        cg.add(var.set_human_temp_diff_max(config[CONF_HUMAN_TEMP_DIFF_MAX]))
        cg.add(var.set_exit_debounce_frames(config[CONF_EXIT_DEBOUNCE_FRAMES]))
        cg.add(var.set_blob_proximity_threshold(config[CONF_BLOB_PROXIMITY_THRESHOLD]))
        # Removed calibrate_on_boot as calibration starts automatically
        cg.add(var.set_calibration_frames(config[CONF_CALIBRATION_FRAMES]))
        cg.add(var.set_algorithm_type(config[CONF_ALGORITHM_TYPE]))
        
    if CONF_OCCUPANCY in config:
        conf = config[CONF_OCCUPANCY]
        sens = await binary_sensor.new_binary_sensor(conf)
        cg.add(var.set_occupancy_sensor(sens))
        
    if CONF_BLOB_SIZE in config:
        conf = config[CONF_BLOB_SIZE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_blob_size_sensor(sens))
        
    if CONF_TEMP_DIFF in config:
        conf = config[CONF_TEMP_DIFF]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_temp_diff_sensor(sens))
        
    if CONF_BLOB_DETAILS in config:
        conf = config[CONF_BLOB_DETAILS]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_blob_details_sensor(sens))
        
    if CONF_NON_OCCUPIED_BLOB_DETAILS in config:
        conf = config[CONF_NON_OCCUPIED_BLOB_DETAILS]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_non_occupied_blob_details_sensor(sens))
    
    # Debug sensors removed
            
    # Calibration button removed - now implemented as standalone button
    # The following code is left commented for reference
    # if CONF_CALIBRATION_BUTTON in config:
    #     conf = config[CONF_CALIBRATION_BUTTON]
    #     btn = await button.new_button(conf)
    #     cg.add(var.set_calibration_button(btn))