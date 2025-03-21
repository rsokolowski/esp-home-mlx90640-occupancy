#ifndef __KNN_OCCUPANCY_DETECTOR_H__
#define __KNN_OCCUPANCY_DETECTOR_H__

#include "knn_background_subtraction.h"
#include "occupancy_detector_interface.h"
#include <cmath>
#include <queue>

namespace esphome {
namespace mlx90640_app {

class KnnOccupancyDetector : public OccupancyDetectorInterface {
private:
    // Constants for the algorithm
    static constexpr int COLS = 32;
    static constexpr int ROWS = 24;
    static constexpr int PIXEL_COUNT = COLS * ROWS;
    
    // Background subtraction model
    KnnBackgroundSubtraction background_model;
    
    // Detection parameters
    int blob_size_threshold = 50;           // Size threshold for a blob to be considered a person
    float human_temp_diff_min = 2.0f;       // Minimum temperature difference for a human
    float human_temp_diff_max = 5.0f;       // Maximum temperature difference for a human
    
    // Debouncing parameters
    int exit_debounce_frames = 5;
    int no_occupancy_frame_counter = 0;
    bool is_currently_occupied = false;
    
    // Blob proximity for background update
    int blob_proximity_threshold = 1;
    
    // Connected component analysis helpers
    void floodFill(const bool* binaryImage, bool* visited, 
                  int x, int y, int& blobSize, 
                  float& tempDiffSum, const float* tempDiffs, bool* blobMask);
    void expandBlobProximity(bool* blobMask, int proximityThreshold);
    std::pair<int, float> getLargestConnectedComponent(
        const bool* binaryImage, const float* tempDiffs, bool* blobMask);
    
public:
    KnnOccupancyDetector();
    virtual ~KnnOccupancyDetector() = default;
    
    // Interface methods
    void calibrateBackground(const float* thermalFrame) override;
    OccupancyDetectionStatus detectOccupancy(const float* thermalFrame) override;
    
    void setBlobSizeThreshold(int threshold) override { this->blob_size_threshold = threshold; }
    int getBlobSizeThreshold() const override { return this->blob_size_threshold; }
    
    void setHumanTempDiffRange(float min, float max) override { 
        this->human_temp_diff_min = min;
        this->human_temp_diff_max = max;
    }
    
    void setExitDebounceFrames(int frames) override { this->exit_debounce_frames = frames; }
    int getExitDebounceFrames() const override { return this->exit_debounce_frames; }
    
    bool isBackgroundCalibrated() const override { return background_model.isInitialized(); }
    
    // KNN specific parameter setters
    void setK(int value) { background_model.setK(value); }
    void setHistorySize(int value) { background_model.setHistorySize(value); }
    void setDistThreshold(float value) { background_model.setDistThreshold(value); }
    void setLearningRate(float value) { background_model.setLearningRate(value); }
    void setMinSamplesToDecide(unsigned long value) { background_model.setMinSamplesToDecide(value); }
    
    // Additional methods specific to KNN implementation
    void setBlobProximityThreshold(int threshold) { this->blob_proximity_threshold = threshold; }
    int getBlobProximityThreshold() const { return this->blob_proximity_threshold; }
};

} // namespace mlx90640_app
} // namespace esphome

#endif // __KNN_OCCUPANCY_DETECTOR_H__