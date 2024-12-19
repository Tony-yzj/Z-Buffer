#pragma once
#ifndef MODEL_H
#define MODEL_H

#include "struct.h"

extern float min_z, max_z;

// Function to generate the combined rotation matrix
cv::Mat getRotationMatrix(float angleX, float angleY);

void scaleObj(Loader* loader);

void rotateObj(Loader* loader, cv::Mat rot);

#endif