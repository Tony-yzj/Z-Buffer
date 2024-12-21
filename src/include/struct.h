#pragma once
#ifndef STRUCT_H
#define STRUCT_H
#include "Features.h"

struct BoundingBox{
	float minX, minY, minZ;
	float maxX, maxY, maxZ;
};

// Define the polygon structure
struct Polygon
{
    // Polygon plane equation parameters
    float a, b, c, d;
    // Polygon identifier
    int id;
    // Polygon height difference
    float dy;

	// Bounding Box
	// #ifdef HIERACHYY
	BoundingBox bbox;
	// #endif

    // Polygon color
    cv::Vec3b color;
    // Polygon depth
    float depth;

    // Calculate the parameters of the plane equation
    void calculate_params(objl::Vector3 p1, objl::Vector3 p2, objl::Vector3 p3, objl::Vector3 &n)
    {
        // Calculate the normal vector of the plane through the cross product of three points, and normalize it
        // a = nx, b = ny, c = nz, d = -n x p1
        objl::Vector3 normal = (p2-p1).cross(p3-p1).normalize();
        // Ensure the normal vector direction is consistent
        if(n.dot(normal) < 0)
            normal = normal * (-1);
        n = normal;
        this->a = normal.X;
        this->b = normal.Y;
        this->c = normal.Z;
        this->d	= -1 * (normal.dot(p1));
    }
    // Calculate the parameters of the plane equation using a known normal vector
    void calculate_params(objl::Vector3 p, objl::Vector3 normal)
    {
        // a = nx, b = ny, c = nz, d = -n x p1
        this->a = normal.X;
        this->b = normal.Y;
        this->c = normal.Z;
        this->d	= -1 * (normal.dot(p));
    }
};

// Define the edge structure
struct Edge
{
    // Edge starting x-coordinate
    float x;
    // Edge x-increment per scan line
    float dx;
    // Edge height difference
    int dy;
    // Pointer to the polygon containing the edge
    Polygon* id;
};

// Define the active edge structure
struct ActiveEdge
{
    // Left edge starting x-coordinate
    float xl;
    // Left edge x-increment per scan line
    float dxl;
    // Left edge height difference
    float dyl;
    // Right edge starting x-coordinate
    float xr;
    // Right edge x-increment per scan line
    float dxr;
    // Right edge height difference
    float dyr;
    // Starting z-coordinate
    float zl;
    // Z-increment from left to right
    float dzx;
    // Z-increment per scan line
    float dzy;
    // Pointer to the polygon containing the active edge
    Polygon* id;
};

#endif