#pragma once
#include "readObj.h"
#ifndef _SCANHZB_H_
#define _SCANHZB_H_

#include "struct.h"
#include "model.h"
#include "hierachy.h"
#include "BVH.h"

// initial image
extern Mat image;
extern float min_z, max_z;

extern Vector3 cameraDirect;
extern Vector3 lightDirect;

void EdgeListContruct(Vector3 p1, Vector3 p2, int ymax, Polygon* id);

Vector3 RoundVertex(Vector3 v);

void ClearTable();
void ListContruct(vector<Triangle> * faces);

void addActiveEdge(ActiveEdge& AE , Edge edge1, Edge edge2, Polygon* polygon, int y);

void scanLine( HierarchicalZBuffer* hzb, vector<ActiveEdge>& AET);
void scanLine( HierarchicalZBuffer* hzb, BVHNode* bvh, vector<ActiveEdge>& AET);
#endif