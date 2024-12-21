#pragma once
#include "readObj.h"
#ifndef _LIST_H_
#define _LIST_H_

#include "struct.h"
#include "model.h"
#include "hierachy.h"

extern vector<std::vector<Polygon*>> PT;
extern vector<std::vector<Edge>> ET;
// initial image
extern Mat image;
extern float min_z, max_z;

extern Vector3 cameraDirect;
extern Vector3 lightDirect;

void EdgeListContruct(Vector3 p1, Vector3 p2, int ymax, Polygon* id);

Vector3 RoundVertex(Vector3 v);

void ClearTable();
void ListContruct(vector<Triangle> * faces);
void ListContruct(Loader* loader, HierarchicalZBuffer* hzb);

void addActiveEdge(vector<ActiveEdge>& AET, Edge edge1, Edge edge2, Polygon* polygon, int y);

void scanLine(float* z_buffer, vector<Polygon*>& APT, vector<ActiveEdge>& AET);
void scanLine( HierarchicalZBuffer* hzb, vector<Polygon*>& APT, vector<ActiveEdge>& AET);
#endif