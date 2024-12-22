#pragma once
#ifndef BVH_H
#define BVH_H
#include "struct.h"

#include <vector>
#include <algorithm>
#include <memory>

struct BVHNode {
    BoundingBox bbox;
    BVHNode* left = nullptr;
    BVHNode* right = nullptr;
    std::vector<Polygon*> polygons; // Leaf nodes contain pointers to polygons

    bool isLeaf() const { return left == nullptr && right == nullptr; }
};

BoundingBox combineBoundingBoxes(const BoundingBox &a, const BoundingBox &b);
BVHNode* buildBVH(std::vector<Polygon*> &polygons, int start, int end, int depth = 0);
BVHNode* constructBVHTree(std::vector<Polygon*> &polygons);
void freeBVHTree(BVHNode* node);


#endif