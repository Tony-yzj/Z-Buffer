#pragma once
#ifndef _LIST_H_
#define _LIST_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "readObj.h"

struct Polygon
{
	float a, b, c, d;
	int id;
	float dy;

	cv::Vec3b color;

	// calculate the parameters of the plane equation
	void calculate_params(objl::Vector3 p1, objl::Vector3 p2, objl::Vector3 p3, objl::Vector3 n)
	{
		// a = nx, b = ny, c = nz, d = -n x p1
		objl::Vector3 normal = (p2-p1).cross(p3-p1).normalize();
		if(n.dot(normal) < 0)
			normal = normal * (-1);
		this->a = normal.X;
        this->b = normal.Y;
        this->c = normal.Z;
        this->d	= -1 * (normal.dot(p1));
	}
	void calculate_params(objl::Vector3 p, objl::Vector3 normal)
	{
		// a = nx, b = ny, c = nz, d = -n x p1
		this->a = normal.X;
        this->b = normal.Y;
        this->c = normal.Z;
        this->d	= -1 * (normal.dot(p));
	}
};

struct Edge
{
	float x;
	float dx;
	int dy;
    Polygon* id;
};

struct ActiveEdge
{
	float xl, dxl, dyl;
	float xr, dxr, dyr;
	float zl, dzx, dzy;
	Polygon* id;
};

#endif