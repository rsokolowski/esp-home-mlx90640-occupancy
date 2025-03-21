#include "knn_occupancy_detector.h"
#include "esphome/core/log.h"
#include <Arduino.h>  // For millis() function

static const char* TAG = "KnnOccupancyDetector";

namespace esphome {
namespace mlx90640_app {

KnnOccupancyDetector::KnnOccupancyDetector() {
    // Set default parameters for the KNN model
    background_model.setK(3);
    background_model.setHistorySize(5);
    background_model.setDistThreshold(2.0f);
    background_model.setLearningRate(0.1f);
    background_model.setMinSamplesToDecide(2);
    
    // Set default detection parameters
    blob_size_threshold = 50;
    human_temp_diff_min = 2.0f;
    human_temp_diff_max = 5.0f;
    
    ESP_LOGI(TAG, "KNN Occupancy Detector initialized");
}

void KnnOccupancyDetector::calibrateBackground(const float* thermalFrame) {
    // Initialize the KNN background model
    background_model.initialize(thermalFrame);
    
    ESP_LOGI(TAG, "KNN Background model calibrated");
}

void KnnOccupancyDetector::floodFill(const bool* binaryImage, bool* visited, 
                                    int x, int y, int& blobSize, 
                                    float& tempDiffSum, const float* tempDiffs, bool* blobMask) {
    // Queue for flood fill
    std::queue<std::pair<int, int>> queue;
    queue.push({x, y});
    visited[y * COLS + x] = true;
    blobSize = 1;
    tempDiffSum = tempDiffs[y * COLS + x];
    
    // Mark this pixel as part of the blob in blobMask
    if (blobMask != nullptr) {
        blobMask[y * COLS + x] = true;
    }
    
    // 4-connected neighbors: up, right, down, left
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};
    
    while (!queue.empty()) {
        std::pair<int, int> curr = queue.front();
        int curr_x = curr.first;
        int curr_y = curr.second;
        queue.pop();
        
        // Check all 4 adjacent pixels
        for (int i = 0; i < 4; i++) {
            int nx = curr_x + dx[i];
            int ny = curr_y + dy[i];
            
            // Check boundaries
            if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS) {
                continue;
            }
            
            int idx = ny * COLS + nx;
            
            // If pixel is part of object and not visited
            if (binaryImage[idx] && !visited[idx]) {
                visited[idx] = true;
                queue.push({nx, ny});
                blobSize++;
                tempDiffSum += tempDiffs[idx];
                
                // Mark this pixel as part of the blob in blobMask
                if (blobMask != nullptr) {
                    blobMask[idx] = true;
                }
            }
        }
    }
}

// Expand the blob mask to include pixels within the proximity threshold
void KnnOccupancyDetector::expandBlobProximity(bool* blobMask, int proximityThreshold) {
    if (proximityThreshold <= 0 || blobMask == nullptr) {
        return; // No expansion needed
    }
    
    // Limit proximity threshold to prevent excessive processing
    proximityThreshold = std::min(proximityThreshold, 5);
    
    // Create a copy of the original mask
    bool originalMask[PIXEL_COUNT];
    bool tempMask[PIXEL_COUNT];
    memcpy(originalMask, blobMask, PIXEL_COUNT * sizeof(bool));
    memset(tempMask, 0, PIXEL_COUNT * sizeof(bool));
    
    // 8-connected neighbors (including diagonals)
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    
    // Use a more efficient approach with swapping buffers
    bool *current = originalMask;
    bool *next = tempMask;
    
    // Record start time to track processing duration
    unsigned long startTime = millis();
    const unsigned long MAX_PROCESSING_TIME = 500; // 500ms maximum processing time
    
    // Count active pixels in original mask for logging
    int originalActivePixels = 0;
    for (int i = 0; i < PIXEL_COUNT; i++) {
        if (blobMask[i]) {
            originalActivePixels++;
        }
    }
    
    for (int level = 0; level < proximityThreshold; level++) {
        // Check if we've exceeded processing time limit
        if (millis() - startTime > MAX_PROCESSING_TIME) {
            ESP_LOGW(TAG, "BlobExpansion: Time limit reached after %d of %d levels", 
                   level, proximityThreshold);
            break;
        }
        
        // Start with a clean destination buffer
        memset(next, 0, PIXEL_COUNT * sizeof(bool));
        
        // Copy current mask to next mask (preserving all currently masked pixels)
        memcpy(next, current, PIXEL_COUNT * sizeof(bool));
        
        // For each pixel
        for (int y = 0; y < ROWS; y++) {
            for (int x = 0; x < COLS; x++) {
                int idx = y * COLS + x;
                
                // If this pixel is already in the current mask, check its neighbors
                if (current[idx]) {
                    // Check all 8 neighbors
                    for (int i = 0; i < 8; i++) {
                        int nx = x + dx[i];
                        int ny = y + dy[i];
                        
                        // Check boundaries
                        if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS) {
                            continue;
                        }
                        
                        int nidx = ny * COLS + nx;
                        // Bounds check for safety
                        if (nidx < 0 || nidx >= PIXEL_COUNT) {
                            continue;
                        }
                        
                        // Add neighbor to next mask
                        next[nidx] = true;
                    }
                }
            }
            
            // Yield after processing each row to avoid watchdog timeouts
            if (y % 6 == 0) { // Yield every 6 rows
                yield();
            }
        }
        
        // Swap current and next for the next iteration
        bool *temp = current;
        current = next;
        next = temp;
    }
    
    // Copy final mask back to output
    if (current != blobMask) {
        memcpy(blobMask, current, PIXEL_COUNT * sizeof(bool));
    }
    
    // Only count expanded pixels for logging if processing wasn't aborted
    if (millis() - startTime <= MAX_PROCESSING_TIME) {
        int expandedPixels = 0;
        for (int i = 0; i < PIXEL_COUNT; i++) {
            if (blobMask[i]) {
                expandedPixels++;
            }
        }
        
        ESP_LOGD(TAG, "Blob mask expanded from %d to %d pixels (threshold %d)", 
               originalActivePixels, expandedPixels, proximityThreshold);
    }
}

std::pair<int, float> KnnOccupancyDetector::getLargestConnectedComponent(
    const bool* binaryImage, const float* tempDiffs, bool* blobMask) {
    
    bool visited[PIXEL_COUNT] = {false};
    int maxBlobSize = 0;
    float maxBlobTempDiffSum = 0;
    int maxBlobStartX = -1;
    int maxBlobStartY = -1;
    
    // First pass: find largest blob
    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLS; x++) {
            int idx = y * COLS + x;
            
            // If pixel is part of an object and not yet visited
            if (binaryImage[idx] && !visited[idx]) {
                int currentBlobSize = 0;
                float currentTempDiffSum = 0;
                
                // Perform flood fill from this pixel
                floodFill(binaryImage, visited, x, y, currentBlobSize, currentTempDiffSum, tempDiffs, nullptr);
                
                // Update maximum blob if this one is larger
                if (currentBlobSize > maxBlobSize) {
                    maxBlobSize = currentBlobSize;
                    maxBlobTempDiffSum = currentTempDiffSum;
                    maxBlobStartX = x;
                    maxBlobStartY = y;
                }
            }
        }
    }
    
    // If a blob mask is provided and we found a maximum blob, mark it in the mask
    if (blobMask != nullptr && maxBlobStartX >= 0) {
        // Reset visited array
        memset(visited, 0, PIXEL_COUNT * sizeof(bool));
        // Clear blob mask
        memset(blobMask, 0, PIXEL_COUNT * sizeof(bool));
        
        // Variables needed for the API, but we don't use the results
        int unused_size;
        float unused_sum;
        
        // Mark only the largest blob in the mask
        floodFill(binaryImage, visited, maxBlobStartX, maxBlobStartY, unused_size, unused_sum, tempDiffs, blobMask);
    }
    
    // Return the size of the largest blob and the average temperature difference
    float avgTempDiff = maxBlobSize > 0 ? maxBlobTempDiffSum / maxBlobSize : 0;
    return {maxBlobSize, avgTempDiff};
}

OccupancyDetectionStatus KnnOccupancyDetector::detectOccupancy(const float* thermalFrame) {
    OccupancyDetectionStatus status;
    unsigned long startTime = millis();
    
    // If background model is not initialized, we can't detect occupancy
    if (!background_model.isInitialized()) {
        ESP_LOGW(TAG, "KNN Background model not initialized, skipping occupancy detection");
        return status;
    }
    
    // Safety check for null input
    if (thermalFrame == nullptr) {
        ESP_LOGE(TAG, "Null thermal frame provided to KNN detector");
        return status;
    }
    
    // Set a watchdog timeout
    const unsigned long MAX_PROCESSING_TIME = 2000; // 2 seconds max
    
    // Apply KNN foreground detection
    bool foregroundMask[PIXEL_COUNT] = {false};
    float distances[PIXEL_COUNT] = {0};
    
    background_model.apply(thermalFrame, foregroundMask, distances);
    
    // Check if we're taking too long
    if (millis() - startTime > MAX_PROCESSING_TIME) {
        ESP_LOGE(TAG, "KNN detection timeout in foreground detection");
        return status;
    }
    
    // Calculate temperature differences for the foreground
    float tempDiffs[PIXEL_COUNT] = {0};
    
    for (int i = 0; i < PIXEL_COUNT; i++) {
        // KNN already gives us the distance, which is equivalent to temperature difference
        tempDiffs[i] = distances[i];
    }
    
    // Perform connected component analysis and get blob mask
    std::pair<int, float> blobResult = getLargestConnectedComponent(foregroundMask, tempDiffs, status.blobMask);
    int largestBlobSize = blobResult.first;
    float avgTempDiff = blobResult.second;
    
    // Check if we're taking too long
    if (millis() - startTime > MAX_PROCESSING_TIME) {
        ESP_LOGE(TAG, "KNN detection timeout in connected component analysis");
        return status;
    }
    
    // Save results in status
    status.blobSize = largestBlobSize;
    status.averageTemperatureDifference = avgTempDiff;
    
    // Determine if the largest blob indicates occupancy
    bool blobIndicatesOccupancy = false;
    
    if (largestBlobSize >= blob_size_threshold) {
        // For KNN, we might not need the temperature range check since KNN already
        // factors in typical variation, but keeping for consistency with original implementation
        if (avgTempDiff >= human_temp_diff_min && avgTempDiff <= human_temp_diff_max) {
            blobIndicatesOccupancy = true;
            ESP_LOGD(TAG, "KNN: Potential occupancy detected: blob size=%d, avg temp diff=%.2f°C", 
                   largestBlobSize, avgTempDiff);
        }
    }
    
    // Modified debouncing logic - immediate response for entry, debounced exit
    if (blobIndicatesOccupancy) {
        // Immediate transition to occupied state
        if (!is_currently_occupied) {
            ESP_LOGI(TAG, "KNN: Occupancy detected!");
        }
        is_currently_occupied = true;
        no_occupancy_frame_counter = 0;
    } else {
        // Debounced transition to unoccupied state
        no_occupancy_frame_counter++;
        
        if (no_occupancy_frame_counter >= exit_debounce_frames) {
            if (is_currently_occupied) {
                ESP_LOGI(TAG, "KNN: No occupancy detected after %d frames", exit_debounce_frames);
                is_currently_occupied = false;
            }
        }
    }
    
    status.isOccupied = is_currently_occupied;
    
    // Yield to other tasks to prevent watchdog timeouts
    yield();
    
    // Check if we're taking too long
    if (millis() - startTime > MAX_PROCESSING_TIME) {
        ESP_LOGE(TAG, "KNN detection timeout before mask expansion");
        return status;
    }
    
    // First expand the blob mask to include proximate pixels
    expandBlobProximity(status.blobMask, blob_proximity_threshold);
    
    // Always update background model selectively, regardless of occupancy state
    // Update background for pixels not in the expanded blob mask
    background_model.update(thermalFrame, status.blobMask);
    
    // Log processing time if it was excessive
    unsigned long processingTime = millis() - startTime;
    if (processingTime > 500) { // Log if it took more than 500ms
        ESP_LOGW(TAG, "KNN detection took %lums to process", processingTime);
    }
    
    return status;
}

} // namespace mlx90640_app
} // namespace esphome