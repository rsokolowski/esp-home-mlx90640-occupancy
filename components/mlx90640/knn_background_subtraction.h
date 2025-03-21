#ifndef __KNN_BACKGROUND_SUBTRACTION_H__
#define __KNN_BACKGROUND_SUBTRACTION_H__

#include <cmath>
#include <algorithm>
#include <vector>
#include "esphome/core/log.h"

namespace esphome {
namespace mlx90640_app {

struct KnnSample {
    float value;
    unsigned long timestamp;
};

class KnnBackgroundSubtraction {
private:
    // Constants for MLX90640 sensor
    static constexpr int COLS = 32;
    static constexpr int ROWS = 24;
    static constexpr int PIXEL_COUNT = COLS * ROWS;
    
    // KNN algorithm parameters
    int k = 3;                           // Number of samples to maintain per pixel
    int history_size = 5;                // Number of history samples to keep per pixel
    float dist_threshold = 2.0f;         // Distance threshold for foreground
    float learning_rate = 0.01f;         // Rate at which new samples are learned (0-1)
    
    // Storage for historical samples (circular buffer per pixel)
    std::vector<KnnSample> samples[PIXEL_COUNT];
    int current_indices[PIXEL_COUNT];    // Current insertion points in circular buffers
    
    // Internal state
    bool is_initialized = false;
    unsigned long min_samples_to_decide = 2;  // Minimum samples needed before making decisions
    
    // Helper methods
    void updateSample(int pixel_idx, float value, unsigned long timestamp);
    float calculateDistance(float value1, float value2);
    int findKNearest(int pixel_idx, float value, float* distances);
    
public:
    KnnBackgroundSubtraction();
    
    // Initialize the model with the first frame
    void initialize(const float* frame);
    
    // Update the background model with a new frame
    void update(const float* frame, const bool* mask = nullptr);
    
    // Apply the model to detect foreground/background
    void apply(const float* frame, bool* foreground_mask, float* distances);
    
    // Parameter setters
    void setK(int value) { this->k = std::min(value, this->history_size); }
    void setHistorySize(int value);
    void setDistThreshold(float value) { this->dist_threshold = value; }
    void setLearningRate(float value) { this->learning_rate = std::max(0.0f, std::min(1.0f, value)); }
    void setMinSamplesToDecide(unsigned long value) { this->min_samples_to_decide = value; }
    
    // Parameter getters
    int getK() const { return this->k; }
    int getHistorySize() const { return this->history_size; }
    float getDistThreshold() const { return this->dist_threshold; }
    float getLearningRate() const { return this->learning_rate; }
    bool isInitialized() const { return this->is_initialized; }
    unsigned long getMinSamplesToDecide() const { return this->min_samples_to_decide; }
};

} // namespace mlx90640_app
} // namespace esphome

#endif // __KNN_BACKGROUND_SUBTRACTION_H__