#include "camera_mlx90640.h"


uint8_t MLX90640_address = 0x33;  // Default 7-bit unshifted address of the
                                     // MLX90640.  MLX90640的默认7位未移位地址
#define TA_SHIFT \
    8  // Default shift for MLX90640 in open air.  MLX90640在户外的默认移位

#define COLS   32
#define ROWS   24
#define COLS_2 (COLS * 2)
#define ROWS_2 (ROWS * 2)

float pixelsArraySize = COLS * ROWS;
float pixels[COLS * ROWS];
byte speed_setting = 2;  // High is 1 , Low is 2


static const char * TAG = "MLX90640" ;
paramsMLX90640 mlx90640;
bool dataValid = false ;

namespace esphome{
namespace mlx90640_app{

MLX90640::MLX90640() {

}

void MLX90640::start_calibration() {
    if (!dataValid) {
        ESP_LOGW(TAG, "Cannot start calibration - sensor data not valid");
        return;
    }
    
    ESP_LOGI(TAG, "Starting background model calibration...");
    is_calibrating_ = true;
    current_calibration_frame_ = 0;
}

void MLX90640::setup(){

    // Initialize the sensor data 
        ESP_LOGI(TAG, "SDA PIN %d ", this->sda_);
        ESP_LOGI(TAG, "SCL PIN %d ", this->scl_);
        ESP_LOGI(TAG, "I2C Frequency %d",  this->frequency_);
        ESP_LOGI(TAG, "Address %d ", this->addr_);
        MLX90640_address = this->addr_ ;
        Wire.begin((int)this->sda_, (int)this->scl_, (uint32_t)this->frequency_);
        Wire.setClock(this->frequency_);  // Increase I2C clock speed to 400kHz. 增加I2C时钟速度到400kHz
        MLX90640_I2CInit(&Wire);
        int status;
        uint16_t eeMLX90640[832];  // 32 * 24 = 768
        if(MLX90640_isConnected(MLX90640_address)){
        status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
        if (status != 0) 
        ESP_LOGE(TAG,"Failed to load system parameters");

        status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
        if (status != 0)  ESP_LOGE(TAG,"Parameter extraction failed");
        
        int SetRefreshRate;
        // Setting MLX90640 device at slave address 0x33 to work with 16Hz refresh
        // rate: 设置从地址0x33的MLX90640设备以16Hz刷新率工作:
        // 0x00 – 0.5Hz
        // 0x01 – 1Hz
        // 0x02 – 2Hz
        // 0x03 – 4Hz
        // 0x04 – 8Hz // OK
        // 0x05 – 16Hz // OK
        // 0x06 – 32Hz // Fail
        // 0x07 – 64Hz
        if(this->refresh_rate_){
            SetRefreshRate = MLX90640_SetRefreshRate(0x33, this->refresh_rate_);
            if(this->refresh_rate_==0x05){
                ESP_LOGI(TAG, "Refresh rate set to 16Hz ");

            }else if(this->refresh_rate_==0x04){
            ESP_LOGI(TAG, "Refresh rate set to 8Hz ");
            }else{
            ESP_LOGI(TAG, "Refresh rate Not Valid ");
            SetRefreshRate = MLX90640_SetRefreshRate(0x33, 0x05);
            }
            
        }else{
            SetRefreshRate = MLX90640_SetRefreshRate(0x33, 0x05);
            ESP_LOGI(TAG, "Refresh rate set to 16Hz ");
        }
        
        // Once params are extracted, we can release eeMLX90640 array.
        // 一旦提取了参数，我们就可以释放eeMLX90640数组
        }else{
            ESP_LOGE(TAG, "The sensor is not connected");
        }

        // Configure occupancy detector
        {
            ESP_LOGI(TAG, "Occupancy detection enabled");
            ESP_LOGI(TAG, "Temperature difference threshold: %.2f°C", temp_diff_threshold_);
            ESP_LOGI(TAG, "Blob size threshold: %d pixels", blob_size_threshold_);
            ESP_LOGI(TAG, "Human temperature difference range: %.2f-%.2f°C", 
                        human_temp_diff_min_, human_temp_diff_max_);
            
            // Create the appropriate detector based on the configured algorithm type
            OccupancyDetectorType detector_type = OccupancyDetectorType::BASELINE;
            
            if (algorithm_type_ == "knn") {
                detector_type = OccupancyDetectorType::KNN;
                ESP_LOGI(TAG, "Using KNN-based occupancy detection algorithm");
            } else {
                ESP_LOGI(TAG, "Using baseline occupancy detection algorithm");
            }
            
            // Create detector using factory
            occupancy_detector_ = OccupancyDetectorFactory::createDetector(detector_type);
            
            // Configure common parameters
            occupancy_detector_->setBlobSizeThreshold(blob_size_threshold_);
            occupancy_detector_->setHumanTempDiffRange(human_temp_diff_min_, human_temp_diff_max_);
            occupancy_detector_->setExitDebounceFrames(exit_debounce_frames_);
            
            // Configure algorithm-specific parameters if needed
            if (detector_type == OccupancyDetectorType::BASELINE) {
                BaselineOccupancyDetector* baseline_detector = static_cast<BaselineOccupancyDetector*>(occupancy_detector_.get());
                if (baseline_detector) {
                    baseline_detector->setTempDiffThreshold(temp_diff_threshold_);
                    baseline_detector->setBlobProximityThreshold(blob_proximity_threshold_);
                }
            } else if (detector_type == OccupancyDetectorType::KNN) {
                KnnOccupancyDetector* knn_detector = static_cast<KnnOccupancyDetector*>(occupancy_detector_.get());
                if (knn_detector) {
                    knn_detector->setDistThreshold(temp_diff_threshold_);
                    knn_detector->setBlobProximityThreshold(blob_proximity_threshold_);
                    knn_detector->setK(3);
                    knn_detector->setHistorySize(10);
                    knn_detector->setLearningRate(0.1f);
                }
            }
            
            ESP_LOGI(TAG, "Exit debounce frames: %d", exit_debounce_frames_);
            ESP_LOGI(TAG, "Blob proximity threshold: %d pixels", blob_proximity_threshold_);
            ESP_LOGI(TAG, "Calibration will start after first valid readings");
            // Calibration will start after the first valid readings
        }
        

}
void MLX90640::filter_outlier_pixel(float *pixels_ , int pixel_size , float level){
    for(int i=1 ; i<pixel_size -1 ; i++){
        if(abs(pixels_[i]-pixels_[i-1])>= level && abs((pixels_[i]-pixels_[i+1]))>= level ){
            pixels_[i] = (pixels_[i-1] + pixels_[i+1])/2.0 ;
        }
    }
    // Check the zero index pixel
    if(abs(pixels_[0]-pixels_[1])>=level && abs(pixels_[0]-pixels_[2])>=level){
        pixels_[0] = (pixels_[1] +pixels_[2])/2.0 ;
    }
    // Check the last index pixel
    if(abs(pixels_[pixel_size-1]-pixels_[pixel_size-2])>=level && abs(pixels_[pixel_size-1]-pixels_[pixel_size-3])>=level){
        pixels_[pixel_size-1] = (pixels_[pixel_size-2] +pixels_[pixel_size-3])/2.0 ;
    }
}

void MLX90640::update()
{
    // Temperature sensor publishing removed
    
    if(MLX90640_isConnected(MLX90640_address)){
            this->mlx_update();
    }else{
    ESP_LOGE(TAG, "The sensor is not connected");
    }

}


void MLX90640::mlx_update(){
    // Add rate limiting to prevent overwhelming the CPU
    static unsigned long lastUpdateTime = 0;
    unsigned long currentTime = millis();
    iteration_ += 1;
    
    // Ensure we don't call this function too often (at least 50ms between updates)
    if (currentTime - lastUpdateTime < 50) {
        return;
    }
    lastUpdateTime = currentTime;
    
    // Record start time for loop duration measurement
    unsigned long loopStartTime = millis();

    for (byte x = 0; x < speed_setting; x++)  // x < 2 Read both subpages
    {
        uint16_t mlx90640Frame[834];
        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
        if (status < 0) {
            ESP_LOGE(TAG,"GetFrame Error: %d",status);
            // If even one page is invalid, bail.
            return;
        } 

        float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
        float Ta  = MLX90640_GetTa(mlx90640Frame, &mlx90640);
        float tr = Ta - TA_SHIFT;  // Reflected temperature based on the sensor ambient
                            // temperature.  根据传感器环境温度反射温度
        float emissivity = 0.95;
        
        // Call CalculateTo with only fixed-point output
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, pixels);  
        
        int mode_ = MLX90640_GetCurMode(MLX90640_address);
        // amendment.  修正案
        MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, pixels, mode_, &mlx90640);
    }

    // Apply fixed-point filter directly to the fixed-point temperature array
    filter_outlier_pixel(pixels, COLS * ROWS, this->filter_level_);
    

    // Assume data is valid at this point.
    dataValid = true;

    // Debug sensors removed
    if (occupancy_sensor_ != nullptr && occupancy_detector_ != nullptr) {
        if (is_calibrating_) {
            // We're in calibration mode - keep collecting frames for the background model
            ESP_LOGD(TAG, "Calibration: collecting frame %d of %d", 
                    current_calibration_frame_ + 1, calibration_frames_);
            
            if (current_calibration_frame_ == 0) {
                // First frame, initialize the background model
                occupancy_detector_->calibrateBackground(pixels);
            }
            
            current_calibration_frame_++;
            
            if (current_calibration_frame_ >= calibration_frames_) {
                // Finished calibration
                ESP_LOGI(TAG, "Background calibration complete");
                is_calibrating_ = false;
                
                // Set initial state to not occupied
                occupancy_sensor_->publish_state(false);
            }
        } else if (occupancy_detector_->isBackgroundCalibrated()) {
            // Normal operation - detect occupancy
            OccupancyDetectionStatus status = occupancy_detector_->detectOccupancy(pixels);
            
            // Publish occupancy state
            occupancy_sensor_->publish_state(status.isOccupied);
            
            // Publish monitoring sensors if enabled and defined
            if (enable_monitoring_) {
                if (blob_size_sensor_ != nullptr) {
                    blob_size_sensor_->publish_state(status.blobSize);
                }
                if (temp_diff_sensor_ != nullptr) {
                    temp_diff_sensor_->publish_state(status.averageTemperatureDifference);
                }
                
                // When transitioning from not occupied to occupied, update blob details text sensor
                if (status.isOccupied && !previous_occupancy_state_ && blob_details_sensor_ != nullptr) {
                    char details[128];
                    snprintf(details, sizeof(details), "Blob Size: %d px, Temp Diff: %.2f°C", 
                           status.blobSize, status.averageTemperatureDifference);
                    blob_details_sensor_->publish_state(details);
                    ESP_LOGI(TAG, "Occupancy detected: %s", details);
                    
                    // Reset the non-occupied blob tracking when occupancy is detected
                    largest_non_occupied_blob_size_ = 0;
                    largest_non_occupied_temp_diff_ = 0.0f;
                    
                    // Publish empty state to indicate reset
                    if (non_occupied_blob_details_sensor_ != nullptr) {
                        non_occupied_blob_details_sensor_->publish_state("Reset");
                    }
                }
                
                // Track the largest non-occupied blob when no occupancy is detected
                if (!status.isOccupied && status.blobSize > 0) {
                    if (status.blobSize > largest_non_occupied_blob_size_ || 
                        (status.blobSize == largest_non_occupied_blob_size_ && 
                         status.averageTemperatureDifference > largest_non_occupied_temp_diff_)) {
                        
                        largest_non_occupied_blob_size_ = status.blobSize;
                        largest_non_occupied_temp_diff_ = status.averageTemperatureDifference;
                        
                        // Update the sensor with the new largest non-occupancy blob
                        if (non_occupied_blob_details_sensor_ != nullptr) {
                            char details[128];
                            snprintf(details, sizeof(details), "Largest Non-Occupied Blob: %d px, Temp Diff: %.2f°C", 
                                   largest_non_occupied_blob_size_, largest_non_occupied_temp_diff_);
                            non_occupied_blob_details_sensor_->publish_state(details);
                            ESP_LOGD(TAG, "Updated largest non-occupied blob: %s", details);
                        }
                    }
                }
                
                // Update previous occupancy state
                previous_occupancy_state_ = status.isOccupied;
            }
            
            ESP_LOGD(TAG, "Occupancy state: %s, Blob size: %d, Temp diff: %.2f", 
                     status.isOccupied ? "OCCUPIED" : "NOT OCCUPIED",
                     status.blobSize, status.averageTemperatureDifference);
        } else {
            // Start calibration on first valid reading
            ESP_LOGI(TAG, "Starting initial background calibration");
            start_calibration();
        }
        
        // Yield to other tasks to prevent watchdog timeout
        delay(1);
    }
    
    // Calculate and publish loop duration if monitoring is enabled
    if (enable_monitoring_ && loop_duration_sensor_ != nullptr) {
        unsigned long loopDuration = millis() - loopStartTime;
        loop_duration_sensor_->publish_state(loopDuration);
        ESP_LOGD(TAG, "Loop duration: %lu ms", loopDuration);
    }
}
        


}  // namespace
}  // namespace