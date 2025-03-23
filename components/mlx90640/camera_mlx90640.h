#ifndef __MLX90640__
#define __MLX90640__
#include<esphome.h>
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "occupancy_detector_interface.h"
#include "occupancy_detector_factory.h"
#include <Wire.h>
#include <memory>


namespace esphome {
    namespace mlx90640_app{
         class MLX90640: public PollingComponent {
              private:
                TwoWire *wire ;
                uint8_t addr_ ;
                uint8_t sda_ ;
                uint8_t scl_ ;
                int frequency_ ;
                int refresh_rate_ = -1 ;
                float filter_level_= 10.0 ;
                // Temperature sensors removed
                binary_sensor::BinarySensor *occupancy_sensor_{nullptr};
                sensor::Sensor *blob_size_sensor_{nullptr};
                sensor::Sensor *temp_diff_sensor_{nullptr};
                sensor::Sensor *loop_duration_sensor_{nullptr};
                text_sensor::TextSensor *blob_details_sensor_{nullptr};
                text_sensor::TextSensor *non_occupied_blob_details_sensor_{nullptr};
                // Debug sensors removed

                // Monitoring parameters
                bool enable_monitoring_ = true;
                bool previous_occupancy_state_ = false;
                
                // Non-occupied blob tracking
                int largest_non_occupied_blob_size_ = 0;
                float largest_non_occupied_temp_diff_ = 0.0f;

                // Occupancy detection parameters
                float temp_diff_threshold_ = 2.0f;
                int blob_size_threshold_ = 50;
                float human_temp_diff_min_ = 2.0f;
                float human_temp_diff_max_ = 5.0f;
                int exit_debounce_frames_ = 5;
                int blob_proximity_threshold_ = 1;
                // Starts as false but will be set to true when first valid data is received
                bool is_calibrating_ = false;
                int calibration_frames_ = 10;
                int current_calibration_frame_ = 0;
                int iteration_ = 0;
                
                // Algorithm selection
                std::string algorithm_type_ = "baseline";
                
                // Occupancy detector interface
                std::unique_ptr<OccupancyDetectorInterface> occupancy_detector_ = nullptr;
                
              public:
                MLX90640();
               float get_setup_priority() const override { return setup_priority::LATE; }
               void setup() override ;
               void update() override ;
               void mlx_update() ;
               // Temperature sensor setters removed
               void set_occupancy_sensor(binary_sensor::BinarySensor *bs){this->occupancy_sensor_= bs;};
               void set_blob_size_sensor(sensor::Sensor *sensor){this->blob_size_sensor_= sensor;};
               void set_temp_diff_sensor(sensor::Sensor *sensor){this->temp_diff_sensor_= sensor;};
               void set_loop_duration_sensor(sensor::Sensor *sensor){this->loop_duration_sensor_= sensor;};
               void set_blob_details_sensor(text_sensor::TextSensor *sensor){this->blob_details_sensor_= sensor;};
               void set_non_occupied_blob_details_sensor(text_sensor::TextSensor *sensor){this->non_occupied_blob_details_sensor_= sensor;};
               // Debug sensor setters removed
               void set_enable_monitoring(bool enable){this->enable_monitoring_ = enable;}
          

               void set_addr(uint8_t addr){this->addr_ = addr;}
               void set_sda(uint8_t sda){this->sda_ = sda ;}
               void set_scl(uint8_t scl){this->scl_ = scl ;}
               void set_frequency(int freq){this->frequency_ = freq ;}
               void set_refresh_rate(int refresh){this->refresh_rate_ = refresh;}
              
               // Filtering function
               void set_filter_level(float level){this->filter_level_ = level ;}
               void filter_outlier_pixel(float *pixels , int size , float level);
               
               // Occupancy detection configurations
               void set_temp_diff_threshold(float threshold){this->temp_diff_threshold_ = threshold;}
               void set_blob_size_threshold(int threshold){this->blob_size_threshold_ = threshold;}
               void set_human_temp_diff_min(float min){this->human_temp_diff_min_ = min;}
               void set_human_temp_diff_max(float max){this->human_temp_diff_max_ = max;}
               void set_exit_debounce_frames(int frames){this->exit_debounce_frames_ = frames;}
               void set_blob_proximity_threshold(int threshold){this->blob_proximity_threshold_ = threshold;}
               // Calibration starts automatically when needed
               void set_calibration_frames(int frames){this->calibration_frames_ = frames;}
               // Algorithm selection
               void set_algorithm_type(const std::string &type){this->algorithm_type_ = type;}
               
               // Calibration control
               void start_calibration();
               bool is_calibrating() const { return is_calibrating_; }
        };
    }
}


#endif