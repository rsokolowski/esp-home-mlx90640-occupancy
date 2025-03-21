#include "knn_background_subtraction.h"
#include <Arduino.h>  // For millis() function

static const char* TAG = "KnnBackgroundSubtraction";

namespace esphome {
namespace mlx90640_app {

KnnBackgroundSubtraction::KnnBackgroundSubtraction() {
    // Reserve space for history samples for each pixel
    for (int i = 0; i < PIXEL_COUNT; i++) {
        samples[i].reserve(history_size);
        current_indices[i] = 0;
    }
    
    ESP_LOGI(TAG, "KNN Background Subtraction initialized with K=%d, history=%d", k, history_size);
}

void KnnBackgroundSubtraction::setHistorySize(int value) {
    if (value <= 0) {
        ESP_LOGE(TAG, "History size must be positive");
        return;
    }
    
    // Update history size and ensure k is not larger than history
    this->history_size = value;
    this->k = std::min(this->k, this->history_size);
    
    // Resize sample vectors
    for (int i = 0; i < PIXEL_COUNT; i++) {
        if (samples[i].size() > history_size) {
            samples[i].resize(history_size);
        }
        samples[i].reserve(history_size);
        if (current_indices[i] >= history_size) {
            current_indices[i] = 0;
        }
    }
    
    ESP_LOGI(TAG, "History size updated to %d, K=%d", history_size, k);
}

void KnnBackgroundSubtraction::initialize(const float* frame) {
    unsigned long current_time = millis();
    
    // Initialize with the first frame data
    for (int i = 0; i < PIXEL_COUNT; i++) {
        // Clear any existing samples
        samples[i].clear();
        current_indices[i] = 0;
        
        // Add the first sample
        updateSample(i, frame[i], current_time);
    }
    
    is_initialized = true;
    ESP_LOGI(TAG, "KNN background model initialized");
}

void KnnBackgroundSubtraction::updateSample(int pixel_idx, float value, unsigned long timestamp) {
    if (samples[pixel_idx].size() < history_size) {
        // Buffer not full yet, add new sample
        samples[pixel_idx].push_back({value, timestamp});
    } else {
        // Replace sample at current index in circular buffer
        samples[pixel_idx][current_indices[pixel_idx]] = {value, timestamp};
        // Move to next position in circular buffer
        current_indices[pixel_idx] = (current_indices[pixel_idx] + 1) % history_size;
    }
}

void KnnBackgroundSubtraction::update(const float* frame, const bool* mask) {
    if (!is_initialized) {
        ESP_LOGW(TAG, "Background model not initialized, initializing now");
        initialize(frame);
        return;
    }
    
    unsigned long current_time = millis();
    
    // Probabilistic update based on learning rate
    for (int i = 0; i < PIXEL_COUNT; i++) {
        // Skip masked pixels if mask is provided
        if (mask != nullptr && mask[i]) {
            continue;
        }
        
        // Update with probability proportional to learning rate
        if ((float)random(100) / 100.0f < learning_rate) {
            updateSample(i, frame[i], current_time);
        }
    }
    
    ESP_LOGD(TAG, "KNN background model updated");
}

// Calculate distance between two temperature values
float KnnBackgroundSubtraction::calculateDistance(float value1, float value2) {
    return std::abs(value1 - value2);
}

// Find k nearest neighbors and return their average distance
int KnnBackgroundSubtraction::findKNearest(int pixel_idx, float value, float* distances) {
    const auto& pixel_samples = samples[pixel_idx];
    int sample_count = pixel_samples.size();
    
    if (sample_count == 0) {
        return 0;
    }
    
    // Optimize for small k values using a more efficient approach
    // This avoids allocating a vector and sorting which can be expensive
    int k_to_use = std::min(k, sample_count);
    
    if (sample_count <= 5) {
        // For very small sample counts, just use direct calculation
        float all_distances[5]; // Fixed size array on stack
        
        // Calculate all distances
        for (int i = 0; i < sample_count; i++) {
            all_distances[i] = calculateDistance(value, pixel_samples[i].value);
        }
        
        // Simple insertion sort for small arrays
        for (int i = 1; i < sample_count; i++) {
            float key = all_distances[i];
            int j = i - 1;
            while (j >= 0 && all_distances[j] > key) {
                all_distances[j + 1] = all_distances[j];
                j--;
            }
            all_distances[j + 1] = key;
        }
        
        // Calculate average of k smallest distances
        float sum = 0;
        for (int i = 0; i < k_to_use; i++) {
            sum += all_distances[i];
        }
        *distances = sum / k_to_use;
        
    } else {
        // For larger sample counts, use a more efficient approach
        // Keep track of the k smallest values seen so far
        float smallest_distances[10]; // Fixed size array, assuming k <= 10
        for (int i = 0; i < k_to_use; i++) {
            smallest_distances[i] = INFINITY;
        }
        
        // Find k smallest distances
        for (int i = 0; i < sample_count; i++) {
            float curr_dist = calculateDistance(value, pixel_samples[i].value);
            
            // Check if this distance belongs in our k smallest
            if (curr_dist < smallest_distances[k_to_use-1]) {
                // Find insertion position (simple insertion sort)
                int j = k_to_use - 2;
                while (j >= 0 && smallest_distances[j] > curr_dist) {
                    smallest_distances[j + 1] = smallest_distances[j];
                    j--;
                }
                smallest_distances[j + 1] = curr_dist;
            }
        }
        
        // Calculate average of k smallest distances
        float sum = 0;
        for (int i = 0; i < k_to_use; i++) {
            sum += smallest_distances[i];
        }
        *distances = sum / k_to_use;
    }
    
    return k_to_use;
}

void KnnBackgroundSubtraction::apply(const float* frame, bool* foreground_mask, float* distances) {
    if (!is_initialized) {
        ESP_LOGW(TAG, "Background model not initialized, cannot detect foreground");
        // Set all to background
        for (int i = 0; i < PIXEL_COUNT; i++) {
            foreground_mask[i] = false;
            distances[i] = 0;
        }
        return;
    }
    
    // Apply the KNN algorithm to each pixel
    for (int i = 0; i < PIXEL_COUNT; i++) {
        float distance = 0;
        int samples_found = findKNearest(i, frame[i], &distance);
        
        // Store the distance for this pixel
        distances[i] = distance;
        
        // Apply thresholding to determine foreground/background
        // Only if we have enough samples to make a reliable decision
        if (samples_found >= min_samples_to_decide) {
            foreground_mask[i] = (distance > dist_threshold);
        } else {
            // Not enough samples, consider as background
            foreground_mask[i] = false;
        }
    }
    
    ESP_LOGD(TAG, "KNN foreground detection applied");
}

} // namespace mlx90640_app
} // namespace esphome