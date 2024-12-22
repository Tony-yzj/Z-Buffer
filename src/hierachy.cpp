#include "hierachy.h"
#include "BVH.h"
#include "struct.h"

// Constructor: Initializes width and height, and builds the Z pyramid
HierarchicalZBuffer::HierarchicalZBuffer(int width, int height)
    : width(width), height(height) {
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

void HierarchicalZBuffer::Reset()
{
    for(int i = 0; i < z_pyramid.size(); i++)
    {
        fill(z_pyramid[i].begin(), z_pyramid[i].end(), -FLT_MAX);
    }
}

// Updates the Z buffer in the pyramid for a specific pixel (x, y) with a new depth value
void HierarchicalZBuffer::Update(int x, int y, float new_z) 
{
    // Update the bottom level of the Z buffer
    z_pyramid[0][y * width + x] = new_z;

    int current_width = width, current_height = height;
    // Propagate the update upwards to higher levels
    for (int level = 1; level < z_pyramid.size() - 2; ++level) 
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

        // if(min_z != -FLT_MAX)
        //     cout << "min_z: " << min_z << endl;

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

    // Traverse the Z pyramid from coarse (higher levels) to fine (lower levels)
    for (int level = GetNumLevels() - 3; level >= 1; --level) 
    {
        int pyramid_width = width >> level;

        int start_x = min_x >> level;
        int start_y = min_y >> level;
        int end_x = max_x >> level;
        int end_y = max_y >> level;

        // Check the Z buffer at the current pyramid level
        if(start_x == end_x && start_y == end_y && min_depth < z_pyramid[level][start_y * pyramid_width + start_x])
        {
            return false;
        }
        else if(start_x != end_x || start_y != end_y)
        {
            // stop when reaching the smallest level containing the polygon
            break;
        }
    }
    return true; // If it's not occluded
}

void HierarchicalZBuffer::IsVisible(BVHNode* node, vector<Polygon*>& polygons) 
{
    BoundingBox bbox = node->bbox;
    
    // Get the bounding box and depth range of the polygon
    int min_x = bbox.minX;
    int max_x = bbox.maxX;
    int min_y = bbox.minY;
    int max_y = bbox.maxY;

    float min_depth = bbox.maxZ;

    // Traverse the Z pyramid from coarse (higher levels) to fine (lower levels)
    for (int level = GetNumLevels() - 3; level >= 1; --level) 
    {
        int pyramid_width = width >> level;

        int start_x = min_x >> level;
        int start_y = min_y >> level;
        int end_x = max_x >> level;
        int end_y = max_y >> level;

        // Check the Z buffer at the current pyramid level
        if(start_x == end_x && start_y == end_y && min_depth < z_pyramid[level][start_y * pyramid_width + start_x])
        {
            return;
        }
        else if(start_x != end_x || start_y != end_y)
        {
            // stop when reaching the smallest level containing the polygon
            break;
        }
    }

    if(!node->isLeaf())
    {
        IsVisible(node->left, polygons);
        IsVisible(node->right, polygons);
    }
    else 
    {
        
    }
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

void HierarchicalZBuffer::Clear()
{
    z_pyramid.clear();
}
