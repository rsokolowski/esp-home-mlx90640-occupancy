#ifndef __OCCUPANCY_DETECTOR_INTERFACE_H__
#define __OCCUPANCY_DETECTOR_INTERFACE_H__

namespace esphome {
namespace mlx90640_app {

struct OccupancyDetectionStatus {
    bool isOccupied = false;
    int blobSize = 0;
    float averageTemperatureDifference = 0.0f;
    bool blobMask[32*24] = {false}; // Stores which pixels are part of blobs
};

class OccupancyDetectorInterface {
public:
    // Virtual destructor for proper cleanup
    virtual ~OccupancyDetectorInterface() = default;
    
    // Initialize or reset the background model
    virtual void calibrateBackground(const float* thermalFrame) = 0;
    
    // Check for occupancy in the current frame
    virtual OccupancyDetectionStatus detectOccupancy(const float* thermalFrame) = 0;
    
    // Common parameters for all occupancy detectors
    virtual void setBlobSizeThreshold(int threshold) = 0;
    virtual int getBlobSizeThreshold() const = 0;
    
    virtual void setHumanTempDiffRange(float min, float max) = 0;
    
    virtual void setExitDebounceFrames(int frames) = 0;
    virtual int getExitDebounceFrames() const = 0;
    
    virtual bool isBackgroundCalibrated() const = 0;
};

} // namespace mlx90640_app
} // namespace esphome

#endif // __OCCUPANCY_DETECTOR_INTERFACE_H__