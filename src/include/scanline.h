#pragma once
#include "readObj.h"
#ifndef _SCAN_H_
#define _SCAN_H_

#include "struct.h"
#include "model.h"
#include "hierachy.h"

// initial image
extern Mat image;
extern float min_z, max_z;

extern Vector3 cameraDirect;
extern Vector3 lightDirect;

void EdgeListContruct(Vector3 p1, Vector3 p2, int ymax, Polygon* id);

Vector3 RoundVertex(Vector3 v);

void ClearTable();
void ListContruct(vector<Triangle> * faces);
void ListContructHZB(vector<Triangle> * faces);

void addActiveEdge(vector<ActiveEdge>& AET, Edge edge1, Edge edge2, Polygon* polygon, int y);

void scanLine(float* z_buffer, vector<Polygon*>& APT, vector<ActiveEdge>& AET);
#endif