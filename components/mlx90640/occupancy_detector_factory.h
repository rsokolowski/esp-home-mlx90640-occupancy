#ifndef __OCCUPANCY_DETECTOR_FACTORY_H__
#define __OCCUPANCY_DETECTOR_FACTORY_H__

#include "occupancy_detector_interface.h"
#include "baseline_occupancy_detector.h"
#include "knn_occupancy_detector.h"
#include <memory>

namespace esphome {
namespace mlx90640_app {

enum class OccupancyDetectorType {
    BASELINE,  // Original implementation using simple background subtraction
    KNN        // K-Nearest Neighbor implementation
};

class OccupancyDetectorFactory {
public:
    // Create detector instance based on selected type
    static std::unique_ptr<OccupancyDetectorInterface> createDetector(OccupancyDetectorType type) {
        switch (type) {
            case OccupancyDetectorType::KNN:
                return std::unique_ptr<OccupancyDetectorInterface>(new KnnOccupancyDetector());
            case OccupancyDetectorType::BASELINE:
            default:
                return std::unique_ptr<OccupancyDetectorInterface>(new BaselineOccupancyDetector());
        }
    }
};

} // namespace mlx90640_app
} // namespace esphome

#endif // __OCCUPANCY_DETECTOR_FACTORY_H__