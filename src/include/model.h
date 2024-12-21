#pragma once
#include "readObj.h"
#ifndef MODEL_H
#define MODEL_H

#include "struct.h"

extern float min_z, max_z;

// Function to generate the combined rotation matrix
cv::Mat getRotationMatrix(float angleX, float angleY);

void scaleObj(Loader* loader);

void rotateObj(vector<Triangle> * faces, cv::Mat rot);

#endif