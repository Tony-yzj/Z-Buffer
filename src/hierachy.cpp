#include "hierachy.h"
#include "struct.h"

// Constructor: Initializes width and height, and builds the Z pyramid
HierarchicalZBuffer::HierarchicalZBuffer(int width, int height)
    : width(width), height(height) {
    Initialize();
}

// Initializes the Z pyramid and constructs Z buffers for each level
void HierarchicalZBuffer::Initialize() {
    int current_width = width, current_height = height;
    
    // Initialize the bottom level of the pyramid (highest resolution)
    z_pyramid.clear();
    z_pyramid.push_back(std::vector<float>(width * height, -FLT_MAX));  // Initialize with -∞, representing the farthest depth

    // Construct the pyramid's other levels
    while (current_width > 1 && current_height > 1) 
    {
        std::vector<float> next_level((current_width / 2) * (current_height / 2), -FLT_MAX);
        z_pyramid.push_back(next_level);
        current_width /= 2;
        current_height /= 2;
    }
}

// Updates the Z buffer in the pyramid for a specific pixel (x, y) with a new depth value
void HierarchicalZBuffer::Update(int x, int y, float new_z) 
{
    // Update the bottom level of the Z buffer
    z_pyramid[0][y * width + x] = new_z;

    int current_width = width, current_height = height;
    // Propagate the update upwards to higher levels
    for (int level = 1; level < z_pyramid.size(); ++level) 
    {
        x /= 2;
        y /= 2;
        int index = y * (current_width / 2) + x;

        // Get the minimum Z value in the current level from the corresponding area in the lower level
        float min_z = std::min({
            z_pyramid[level - 1][y * 2 * current_width + x * 2],
            z_pyramid[level - 1][y * 2 * current_width + x * 2 + 1],
            z_pyramid[level - 1][(y * 2 + 1) * current_width + x * 2],
            z_pyramid[level - 1][(y * 2 + 1) * current_width + x * 2 + 1]
        });

        // If the updated value is equal to the max value, stop propagating
        if (z_pyramid[level][index] == min_z) break;

        z_pyramid[level][index] = min_z;
        current_width /= 2;
        current_height /= 2;
    }
}

// Uses the Z pyramid to perform occlusion culling (rough-level check)
bool HierarchicalZBuffer::IsVisible(Polygon& polygon) 
{
    BoundingBox bbox = polygon.bbox;
    
    // Get the bounding box and depth range of the polygon
    int min_x = bbox.minX;
    int max_x = bbox.maxX;
    int min_y = bbox.minY;
    int max_y = bbox.maxY;

    float min_depth = bbox.maxZ;
    float max_depth = bbox.minZ;

    // Traverse the Z pyramid from coarse (higher levels) to fine (lower levels)
    for (int level = GetNumLevels() - 1; level >= 0; --level) {
        int pyramid_width = width >> level;
        int pyramid_height = height >> level;

        int start_x = min_x >> level;
        int start_y = min_y >> level;
        int end_x = max_x >> level;
        int end_y = max_y >> level;

        bool is_occluded = true;

        // Check the Z buffer at the current pyramid level
        for (int y = start_y; y <= end_y; ++y) {
            for (int x = start_x; x <= end_x; ++x) {
                // If any part of the polygon is not occluded, continue checking at finer levels
                // the polygon is ocluded if polygon's min depth < z_buffer's depth
                if (min_depth  > z_pyramid[level][y * pyramid_width + x]) {
                    is_occluded = false;
                    break;
                }
            }
            if (!is_occluded) break;
        }

        // If the polygon is visible at this level, stop and return true
        if (!is_occluded) {
            return true;
        }
    }
    return false; // If it's occluded
}

// Gets the depth value from the lowest level (highest resolution) of the Z pyramid
float HierarchicalZBuffer::GetDepth(int x, int y) 
{
    return z_pyramid[0][y * width + x];
}

// Returns the number of levels in the Z pyramid
int HierarchicalZBuffer::GetNumLevels() const 
{
    return z_pyramid.size();
}

float HierarchicalZBuffer::at(int x, int y, int level)
{   
    int current_width = width >> level;
    return z_pyramid[level][y * current_width + x];
}

void HierarchicalZBuffer::clear()
{
    z_pyramid.clear();
}
