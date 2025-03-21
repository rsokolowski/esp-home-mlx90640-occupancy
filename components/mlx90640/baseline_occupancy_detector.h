#ifndef __BASELINE_OCCUPANCY_DETECTOR_H__
#define __BASELINE_OCCUPANCY_DETECTOR_H__

#include "occupancy_detector_interface.h"
#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>

namespace esphome {
namespace mlx90640_app {

class BaselineOccupancyDetector : public OccupancyDetectorInterface {
private:
    // Constants for the algorithm
    static constexpr int COLS = 32;
    static constexpr int ROWS = 24;
    static constexpr int PIXEL_COUNT = COLS * ROWS;
    
    // Background model in fixed-point representation
    float backgroundModel[PIXEL_COUNT];
    bool isBackgroundInitialized = false;
    
    // Background update parameters
    unsigned long lastBackgroundUpdateTime = 0;
    static constexpr unsigned long BACKGROUND_UPDATE_INTERVAL = 5000; // 5 seconds in milliseconds
    
    // Occupancy detection parameters (fixed-point)
    float tempDiffThreshold = 0;           // Fixed-point representation of temperature difference threshold
    int blobSizeThreshold = 50;              // Size threshold for a blob to be considered a person
    float humanTempDiffMin = 0;            // Minimum temperature difference for a human
    float humanTempDiffMax = 0;            // Maximum temperature difference for a human
    
    // Debouncing parameters
    int exitDebounceFrames = 5;
    int noOccupancyFrameCounter = 0;
    bool isCurrentlyOccupied = false;
    
    // Blob proximity for background update
    int blobProximityThreshold = 1;
    
    // Connected component analysis helpers
    void floodFill(const bool* binaryImage, bool* visited, int x, int y, int& blobSize, float& tempDiffSum, const float* tempDiffs, bool* blobMask);
    void expandBlobProximity(bool* blobMask, int proximityThreshold);
    std::pair<int, float> getLargestConnectedComponent(const bool* binaryImage, const float* tempDiffs, bool* blobMask);
    void updateBackgroundSelective(const float* thermalFrame, const bool* excludeMask);
    
public:
    BaselineOccupancyDetector();
    
    // Initialize or reset the background model
    void calibrateBackground(const float* thermalFrame, int frameCount = 10);
    void updateBackground(const float* thermalFrame);
    void updateBackgroundWithMask(const float* thermalFrame, const bool* blobMask);
    
    // Interface methods
    void calibrateBackground(const float* thermalFrame) override;
    OccupancyDetectionStatus detectOccupancy(const float* thermalFrame) override;
    
    void setBlobSizeThreshold(int threshold) override { this->blobSizeThreshold = threshold; }
    int getBlobSizeThreshold() const override { return this->blobSizeThreshold; }
    
    void setHumanTempDiffRange(float min, float max) override { 
        this->humanTempDiffMin = min;
        this->humanTempDiffMax = max;
    }
    
    void setExitDebounceFrames(int frames) override { this->exitDebounceFrames = frames; }
    int getExitDebounceFrames() const override { return this->exitDebounceFrames; }
    
    bool isBackgroundCalibrated() const override { return this->isBackgroundInitialized; }
    
    // Additional methods specific to baseline implementation
    void setTempDiffThreshold(float threshold) { 
        this->tempDiffThreshold = threshold;
    }
    float getTempDiffThreshold() const { return this->tempDiffThreshold; }
    
    void setBlobProximityThreshold(int threshold) { this->blobProximityThreshold = threshold; }
    int getBlobProximityThreshold() const { return this->blobProximityThreshold; }
};

} // namespace mlx90640_app
} // namespace esphome

#endif // __BASELINE_OCCUPANCY_DETECTOR_H__