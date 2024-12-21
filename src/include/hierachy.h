#pragma once
#ifndef HIERARCHICAL_ZBUFFER_H
#define HIERARCHICAL_ZBUFFER_H

#include "struct.h"

class HierarchicalZBuffer {
public:
    // Constructor: Initializes width and height, and builds the Z pyramid
    HierarchicalZBuffer(int width, int height);

    // Initializes the Z pyramid and constructs Z buffers for each level
    void Initialize();

    // Updates the Z buffer in the pyramid for a specific pixel (x, y) with a new depth value
    void Update(int x, int y, float new_z);

    // Uses the Z pyramid to perform occlusion culling (rough-level check)
    bool IsVisible(Polygon& polygon);

    // Gets the depth value from the lowest level (highest resolution) of the Z pyramid
    float GetDepth(int x, int y);

    // Returns the number of levels in the Z pyramid
    int GetNumLevels() const;

    float at(int x, int y, int level);

    void Clear();

    void Reset();

private:
    std::vector<std::vector<float>> z_pyramid;  // The Z pyramid, containing Z buffers for each level
    int width, height;  // The width and height of the image
};

#endif // HIERARCHICAL_ZBUFFER_H
