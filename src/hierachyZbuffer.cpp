#include <algorithm>
#include <cassert>
#include <cfloat>
#include <ctime>
#include <iostream>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <time.h>

#include "Features.h"
#include "hierachy.h"
#include "readObj.h"
#include "scanlineHZB.h"
#define HIERACHY 1

using namespace std;
using namespace cv;
using namespace objl;

struct MouseParams {
    vector<Triangle> *faces;
    HierarchicalZBuffer* hvb;
    vector<ActiveEdge> * AET;
};

std::vector<Polygon*> PT;
std::vector<std::vector<Edge>> ET;
// initial image
Mat image(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, Vec3b(244, 234, 226));
float min_z = FLT_MAX, max_z = -FLT_MAX;
Point prevMousePos;     // Previous mouse position
bool isDragging = false;    // Flag to check if dragging

float rotationX = 0.0f;     // Rotation angle around X-axis
float rotationY = 0.0f;     // Rotation angle around Y-axis
Vector3 cameraDirect(0, 0, -1);
Vector3 lightDirect(-sqrt(2)/2, 0, -sqrt(2)/2);
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    // transfer userdata to mouseCallback Params
    MouseParams* params = static_cast<MouseParams*>(userdata);

    if (event == cv::EVENT_LBUTTONDOWN) 
    {
        // left button pressed, record initial position
        isDragging = true;
        prevMousePos = cv::Point(x, y);
        cout << "Mouse pressed at (" << x << ", " << y << ")" << endl;
    } 
    else if (event == cv::EVENT_MOUSEMOVE && isDragging) 
    {
        // Compute mouse movement differences and scale to degrees
        float dx = static_cast<float>(x - prevMousePos.x) * 0.5f; // Scale to degrees
        float dy = static_cast<float>(y - prevMousePos.y) * 0.5f;

        rotationY = dx;  // Horizontal movement -> Y-axis rotation
        rotationX = dy;  // Vertical movement -> X-axis rotation

        prevMousePos = cv::Point(x, y); // Update previous mouse position
        
        // Generate the rotation matrix
        float angleRadiansX = rotationX * CV_PI / 180.0f; // Convert degrees to radians
        float angleRadiansY = rotationY * CV_PI / 180.0f;

        params->hvb->Reset();

        cv::Mat R = getRotationMatrix(angleRadiansX, angleRadiansY);
        rotateObj(params->faces, R); // Rotate the object

        // Redo ScanLine algorithm
        clock_t start = clock();    
        ListContruct(params->faces);
        scanLine(params->hvb, *(params->AET));

        cout << "Mouse moved to (" << x << ", " << y << ")" << endl;
        clock_t end = clock();
        cout << "Time: " << (double)(end - start) / CLOCKS_PER_SEC << " seconds" << endl;
        // put text on image
        cv::putText(image, "Time: " + std::to_string((double)(end - start) / CLOCKS_PER_SEC) + " seconds", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

        cv::imshow("Zbuffer", image);
    } 
    else if (event == cv::EVENT_LBUTTONUP) 
    {
        isDragging = false;
        cout << "Event: Mouse button released" << endl;
    }
}

int main(int argc, char** argv)
{
    utils::logging::setLogLevel(utils::logging::LOG_LEVEL_SILENT);

    Loader* loader = new Loader();
    string path = "../models/";
    // load mesh
    if(argc == 1)
    {
        if(!loader->LoadFile("../models/bunny.obj"))
            cout << "Loaded File Fail." << endl;
    }
    else if(argc == 2)
    {
        path = path + argv[1];
        path = path + ".obj";
        if(!loader->LoadFile(path))
            cout << "Loaded File Fail." << endl;
    }

    cout << "Loaded File Success." << endl;
    cout << "Number of Polygons: " << loader->LoadedTriangles.size() << endl;
    // scale to fit in image
    scaleObj(loader);

    vector<Triangle> faces = loader->LoadedTriangles;

    // initialize z-buffer for one scan line
    HierarchicalZBuffer* hzb = new HierarchicalZBuffer(IMG_WIDTH, IMG_HEIGHT);

    // contruct polygon and edge list
    // PT.resize(image.rows);
    ET.resize(image.rows);

    // scan line z buffer algorithm
    vector<Polygon*> APT;
    vector<ActiveEdge> AET;

    MouseParams params;
    params.AET = &AET;
    params.faces = &faces;
    params.hvb = hzb;

    cv::namedWindow("Zbuffer");
    cv::setMouseCallback("Zbuffer", mouseCallback, &params);

    // time record
    clock_t start = clock();
    clock_t init_s = clock();
    hzb->Initialize();
    cout << "Initialize Time: " << (double)(clock() - init_s) / CLOCKS_PER_SEC << "s" << endl;
    clock_t construct_s = clock();
    ListContruct(&faces);
    cout << "Construct Time: " << (double)(clock() - construct_s) / CLOCKS_PER_SEC << "s" << endl;
    clock_t scan_s = clock();
    scanLine(hzb, AET);
    cout << "Scan Time: " << (double)(clock() - scan_s) / CLOCKS_PER_SEC << "s" << endl;

    
    clock_t end = clock();
    cout << "Time: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;
    // put text on image
    cv::putText(image, "Time: " + std::to_string((double)(end - start) / CLOCKS_PER_SEC) + " seconds", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

    imshow("Zbuffer", image);

    waitKey(0);

    delete hzb;
}

