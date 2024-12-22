#include "BVH.h"
#include "struct.h"
#include <cstring>

BoundingBox combineBoundingBoxes(const BoundingBox &a, const BoundingBox &b) {
    BoundingBox combined;
    combined.minX = std::min(a.minX, b.minX);
    combined.minY = std::min(a.minY, b.minY);
    combined.minZ = std::min(a.minZ, b.minZ);
    combined.maxX = std::max(a.maxX, b.maxX);
    combined.maxY = std::max(a.maxY, b.maxY);
    combined.maxZ = std::max(a.maxZ, b.maxZ);
    return combined;
}
BVHNode* buildBVH(std::vector<Polygon*> &polygons, int start, int end, int capacity, int depth) {
    BVHNode* node = new BVHNode;

    // Compute the bounding box for the current set of polygons
    node->bbox = polygons[start]->bbox;
    for (int i = start + 1; i < end; ++i) {
        node->bbox = combineBoundingBoxes(node->bbox, polygons[i]->bbox);
    }

    // Base case: if only one polygon, create a leaf node
    int numPolygons = end - start;
    if (numPolygons <= capacity) {
        for(int i = start; i < end; i++)
            node->polygons.push_back(polygons[i]);
        node->left = node->right = nullptr;
        return node;
    }

    // Choose axis to split along (alternating between x, y, and z)
    int axis = depth % 3;

    // Sort polygons along the chosen axis
    auto comparator = [axis](const Polygon* a, const Polygon* b) {
        if (axis == 1) return a->bbox.minX < b->bbox.minX;
        if (axis == 2) return a->bbox.minY < b->bbox.minY;
        return a->bbox.maxZ > b->bbox.maxZ;
    };
    std::sort(polygons.begin() + start, polygons.begin() + end, comparator);

    // Split the polygons into two subsets
    int mid = start + numPolygons / 2;

    // Recursively build the left and right subtrees
    node->left = buildBVH(polygons, start, mid, depth + 1);
    node->right = buildBVH(polygons, mid, end, depth + 1);

    return node;
}

BVHNode* constructBVHTree(std::vector<Polygon*> &polygons) {
    return buildBVH(polygons, 0, polygons.size(), polygons.size()/500);
}

void freeBVHTree(BVHNode* node) {
    if (!node) return;
    freeBVHTree(node->left);
    freeBVHTree(node->right);
    delete node;
}