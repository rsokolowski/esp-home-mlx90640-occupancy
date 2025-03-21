#include "baseline_occupancy_detector.h"
#include "esphome/core/log.h"
#include <Arduino.h>  // For millis() function

static const char* TAG = "BaselineOccupancyDetector";

namespace esphome {
namespace mlx90640_app {

BaselineOccupancyDetector::BaselineOccupancyDetector() {
    // Initialize background model with zeros
    for (int i = 0; i < PIXEL_COUNT; i++) {
        backgroundModel[i] = 0;
    }
    
    tempDiffThreshold = 2.0f;
    humanTempDiffMin = 2.0f;
    humanTempDiffMax = 5.0f;
}

void BaselineOccupancyDetector::calibrateBackground(const float* thermalFrame) {
    // Use default frameCount of 10
    calibrateBackground(thermalFrame, 10);
}

void BaselineOccupancyDetector::calibrateBackground(const float* thermalFrame, int frameCount) {
    // Reset the background model
    for (int i = 0; i < PIXEL_COUNT; i++) {
        backgroundModel[i] = thermalFrame[i];
    }
    
    // Mark as initialized after the first frame
    isBackgroundInitialized = true;
    lastBackgroundUpdateTime = millis();
    
    ESP_LOGI(TAG, "Background model initialized");
}

void BaselineOccupancyDetector::updateBackground(const float* thermalFrame) {
    // Gradually update the background model (exponential moving average)
    const float alpha = 0.1f; // Weight for new data
    
    for (int i = 0; i < PIXEL_COUNT; i++) {
        // Update fixed-point model
        backgroundModel[i] = (1.0f - alpha) * backgroundModel[i] + alpha * thermalFrame[i];
    }
    
    lastBackgroundUpdateTime = millis();
    ESP_LOGI(TAG, "Background model updated");
}

// Update background selectively, excluding pixels in the mask
void BaselineOccupancyDetector::updateBackgroundWithMask(const float* thermalFrame, const bool* excludeMask) {
    // Check if it's too soon to update the background model
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - lastBackgroundUpdateTime;
    
    // Only update every BACKGROUND_UPDATE_INTERVAL milliseconds to avoid too frequent updates
    if (elapsed < BACKGROUND_UPDATE_INTERVAL) {
        return;
    }
    
    // Gradually update the background model (exponential moving average) only for unmasked pixels
    const float alpha = 0.05f; // Reduced weight for new data (from 0.1 to 0.05)
    int updatedPixels = 0;
    
    // First check if we're updating too many pixels (which might indicate a detection issue)
    int excludedPixels = 0;
    for (int i = 0; i < PIXEL_COUNT; i++) {
        if (excludeMask[i]) {
            excludedPixels++;
        }
    }
    
    // If no pixels are excluded and we've previously detected occupancy,
    // be more conservative with background updates to avoid incorporating human presence
    const float effective_alpha = (excludedPixels == 0 && isCurrentlyOccupied) ? 0.01f : alpha;
    
    for (int i = 0; i < PIXEL_COUNT; i++) {
        // Only update pixels that are not in blobs or their proximity
        if (!excludeMask[i]) {
            // Update model using exponential moving average
            backgroundModel[i] = (1.0f - effective_alpha) * backgroundModel[i] + effective_alpha * thermalFrame[i];
            updatedPixels++;
        }
    }
    
    lastBackgroundUpdateTime = currentTime;
    // Log the message with the percentage.
    float updatedPercentage = (static_cast<float>(updatedPixels) / PIXEL_COUNT) * 100.0f;
    ESP_LOGD(TAG, "Background model updated selectively (%d pixels updated, %.2f%% of total, alpha=%.3f)", 
             updatedPixels, updatedPercentage, effective_alpha);
}

void BaselineOccupancyDetector::floodFill(const bool* binaryImage, bool* visited, 
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
void BaselineOccupancyDetector::expandBlobProximity(bool* blobMask, int proximityThreshold) {
    if (proximityThreshold <= 0) {
        return; // No expansion needed
    }
    
    // Create a copy of the original mask
    bool originalMask[PIXEL_COUNT];
    bool tempMask[PIXEL_COUNT];
    memcpy(originalMask, blobMask, PIXEL_COUNT * sizeof(bool));
    
    // 8-connected neighbors (including diagonals)
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    
    // Use a more efficient approach with swapping buffers
    bool *current = originalMask;
    bool *next = tempMask;
    
    for (int level = 0; level < proximityThreshold; level++) {
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
                        // Add neighbor to next mask
                        next[nidx] = true;
                    }
                }
            }
        }
        
        // Swap current and next for the next iteration
        bool *temp = current;
        current = next;
        next = temp;
        
        // Allow other tasks to run if this takes time
        if (level % 2 == 0) {
            yield();
        }
    }
    
    // If we ended with current pointing to tempMask, copy back to blobMask
    if (current != originalMask) {
        memcpy(blobMask, current, PIXEL_COUNT * sizeof(bool));
    }
    
    // Count expanded pixels for logging
    int expandedPixels = 0;
    for (int i = 0; i < PIXEL_COUNT; i++) {
        if (blobMask[i]) {
            expandedPixels++;
        }
    }
    
    ESP_LOGD(TAG, "Blob mask expanded to %d pixels with proximity threshold %d", 
           expandedPixels, proximityThreshold);
}

std::pair<int, float> BaselineOccupancyDetector::getLargestConnectedComponent(
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
    
    // If a blob mask is provided, mark it in the mask
    if (blobMask != nullptr) {
        // Reset visited array
        memset(visited, 0, PIXEL_COUNT * sizeof(bool));
        // Clear blob mask
        memset(blobMask, 0, PIXEL_COUNT * sizeof(bool));
        
        // If we found a maximum blob, mark it in the mask
        if (maxBlobStartX >= 0) {
            // Variables needed for the API, but we don't use the results
            int unused_size;
            float unused_sum;
            
            // Mark only the largest blob in the mask
            floodFill(binaryImage, visited, maxBlobStartX, maxBlobStartY, unused_size, unused_sum, tempDiffs, blobMask);
        }
    }
    
    // Return the size of the largest blob and the average temperature difference
    float avgTempDiff = maxBlobSize > 0 ? maxBlobTempDiffSum / maxBlobSize : 0;
    return {maxBlobSize, avgTempDiff};
}

OccupancyDetectionStatus BaselineOccupancyDetector::detectOccupancy(const float* thermalFrame) {
    OccupancyDetectionStatus status;
    
    // If background model is not initialized, we can't detect occupancy
    if (!isBackgroundInitialized) {
        ESP_LOGW(TAG, "Background model not initialized, skipping occupancy detection");
        return status;
    }
    
    // Create a binary image based on temperature differences
    bool binaryImage[PIXEL_COUNT] = {false};
    float tempDiffs[PIXEL_COUNT] = {0};
    
    for (int i = 0; i < PIXEL_COUNT; i++) {
        // Calculate temperature difference
        tempDiffs[i] = thermalFrame[i] - backgroundModel[i];
        
        // Apply threshold to create binary image (using abs)
        // Only mark pixels with positive difference (warmer than background) as potential human presence
        binaryImage[i] = (tempDiffs[i] >= tempDiffThreshold);
    }
    
    // Perform connected component analysis and get blob mask
    std::pair<int, float> blobResult = getLargestConnectedComponent(binaryImage, tempDiffs, status.blobMask);
    int largestBlobSize = blobResult.first;
    float avgTempDiff = blobResult.second;
    
    // Save results in status
    status.blobSize = largestBlobSize;
    status.averageTemperatureDifference = avgTempDiff;
    
    // Determine if the largest blob indicates occupancy
    bool blobIndicatesOccupancy = false;
    
    if (largestBlobSize >= blobSizeThreshold) {
        // Check if the average temperature difference is in the human range
        if (avgTempDiff >= humanTempDiffMin && avgTempDiff <= humanTempDiffMax) {
            blobIndicatesOccupancy = true;
            ESP_LOGD(TAG, "Potential occupancy detected: blob size=%d, avg temp diff=%.2f°C", 
                   largestBlobSize, avgTempDiff);
        }
    }
    
    // Modified debouncing logic - immediate response for entry, debounced exit
    if (blobIndicatesOccupancy) {
        // Immediate transition to occupied state
        if (!isCurrentlyOccupied) {
            ESP_LOGI(TAG, "Occupancy detected!");
        }
        isCurrentlyOccupied = true;
        noOccupancyFrameCounter = 0;
    } else {
        // Debounced transition to unoccupied state
        noOccupancyFrameCounter++;
        
        if (noOccupancyFrameCounter >= exitDebounceFrames) {
            if (isCurrentlyOccupied) {
                ESP_LOGI(TAG, "No occupancy detected after %d frames", exitDebounceFrames);
                isCurrentlyOccupied = false;
            }
        }
    }
    
    status.isOccupied = isCurrentlyOccupied;
    
    // Always update background model selectively, regardless of occupancy state
    
    // First expand the blob mask to include proximate pixels
    expandBlobProximity(status.blobMask, blobProximityThreshold);
    
    // Check if we have any pixels in the blob mask before updating
    int maskedPixels = 0;
    for (int i = 0; i < PIXEL_COUNT; i++) {
        if (status.blobMask[i]) {
            maskedPixels++;
        }
    }
    
    // Log the number of masked pixels
    ESP_LOGD(TAG, "Total masked pixels before background update: %d", maskedPixels);
    
    // Update background for pixels not in the expanded blob mask
    updateBackgroundWithMask(thermalFrame, status.blobMask);
    
    return status;
}

} // namespace mlx90640_app
} // namespace esphome