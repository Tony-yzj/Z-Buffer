#include <algorithm>
#include <cassert>
#include <cfloat>
#include <iostream>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <time.h>

#include "readObj.h"
#include "List.h"
#include "Model.h"

using namespace std;
using namespace cv;
using namespace objl;

#define SCALE 0.5
#define IMG_WIDTH 800
#define IMG_HEIGHT 600
#define CULL_ENBALE

struct MouseParams {
    Loader* loader;
    float* zbuffer;
    vector<Polygon*> *APT;
    vector<ActiveEdge> * AET;
};

std::vector<std::vector<Polygon*>> PT;
std::vector<std::vector<Edge>> ET;
// initial image
Mat image(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, Vec3b(0, 0, 0));
float min_z = FLT_MAX, max_z = -FLT_MAX;
Point prevMousePos;     // Previous mouse position
bool isDragging = false;    // Flag to check if dragging

float rotationX = 0.0f;     // Rotation angle around X-axis
float rotationY = 0.0f;     // Rotation angle around Y-axis
Vector3 cameraDirect(0, 0, -1);

void EdgeListContruct(Vector3 p1, Vector3 p2, int ymax, Polygon* id)
{
    Edge edge;

    // let p1 always be the top point
    if(p1.Y <= p2.Y)
        swap(p1, p2);

    edge.dx = -(p1.X - p2.X)/(p1.Y - p2.Y);

    if(p2.Y > IMG_HEIGHT - 1)
        return;

    if(p1.Y > IMG_HEIGHT - 1)
    {
        p1.X += edge.dx * (p1.Y - IMG_HEIGHT + 1);
        p1.Y = IMG_HEIGHT - 1;
    }

    // neglect parallel-to-x-axis edges
    if(p1.Y == p2.Y)
        return;

    edge.x = p1.X;
    // assign max and min y
    int min_y = p2.Y;
    int max_y = p1.Y;

    edge.dy = max_y - min_y;
    edge.id = id;

    assert(edge.dy >= 1);

    // note: horizontal edges need to be considered separately
    // 
    // truncation edges
    if (p1.Y < ymax && ymax < IMG_HEIGHT)
    {
        edge.x = p1.X + edge.dx;
        edge.dy -= 1;
        max_y -= 1;
    }
    
    if(max_y > 0 && max_y < IMG_HEIGHT)
        ET[max_y].push_back(edge);
}

Vector3 RoundVertex(Vector3 v)
{
    return Vector3((int)round(v.X), (int)round(v.Y), v.Z);
}

void ClearTable()
{
    for(int i = 0; i < PT.size(); i++)
    {
        for(int j = 0; j < PT[i].size(); j++)
        {
            delete PT[i][j];
        }
    }
    PT.clear();
    ET.clear();
    PT.resize(IMG_HEIGHT);
    ET.resize(IMG_HEIGHT);  
}
void ListContruct(Loader* loader)
{
    // Clear PT & ET
    ClearTable();

    for (int i = 0; i < loader->LoadedTriangles.size(); i++)
    {
        Polygon* polygon = new Polygon();
        Triangle triangle = loader->LoadedTriangles[i];
        
        // calculate params
        Vector3 p = triangle.vertices[0].Position;
        Vector3 normal = triangle.vertices[0].Normal;

        polygon->calculate_params(RoundVertex(triangle.vertices[0].Position), RoundVertex(triangle.vertices[1].Position), RoundVertex(triangle.vertices[2].Position), normal);

        #ifdef CULL_ENBALE
        // Cull
        if(normal.dot(cameraDirect) < 0)
            continue;
        #endif
        
        polygon->id = i;

        // edge list construction
        int min_y = INT_MAX, max_y = INT_MIN;
        polygon->depth = 0;
        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            min_y = min(min_y, (int)round(triangle.vertices[j].Position.Y));
            max_y = max(max_y, (int)round(triangle.vertices[j].Position.Y));
            polygon->depth += triangle.vertices[j].Position.Z;
        }

        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            Vector3 p1 = RoundVertex(triangle.vertices[j].Position);
            Vector3 p2 = RoundVertex((j == (int)triangle.vertices.size() - 1) ? triangle.vertices[0].Position : triangle.vertices[j + 1].Position);

            EdgeListContruct(p1, p2, max_y, polygon);
        }

        polygon->dy = max_y - min_y;

        polygon->depth /= triangle.vertices.size();
        float scale = (polygon->depth - min_z)/(max_z - min_z);
        assert(scale <= 1);
        polygon->color = Vec3b(255 * scale, 255 * scale, 255 * scale);

        if(max_y >= IMG_HEIGHT)
            max_y = IMG_HEIGHT - 1;

        if(polygon->dy > 0 && max_y > 0 && min_y < IMG_HEIGHT - 1)
            PT[max_y].push_back(polygon);
        else
            delete polygon;
    }
}

void scaleObj(Loader* loader)
{
    float max_x = -FLT_MAX, max_y = -FLT_MAX;
    float min_x = FLT_MAX, min_y = FLT_MAX;
    // for (int i = 0; i < loader->LoadedTriangles.size(); i++)
    // {
    //     Triangle triangle = loader->LoadedTriangles[i];
    //     for (int j = 0; j < triangle.vertices.size(); j++)
    //     {
    //         loader->LoadedTriangles[i].vertices[j].Position.Y = -loader->LoadedTriangles[i].vertices[j].Position.Y;
    //     }
    // }
    for (int i = 0; i < loader->LoadedVertices.size(); i++)
    {
        max_x = max(max_x, loader->LoadedVertices[i].Position.X);
        max_y = max(max_y, loader->LoadedVertices[i].Position.Y);
        min_x = min(min_x, loader->LoadedVertices[i].Position.X);
        min_y = min(min_y, loader->LoadedVertices[i].Position.Y);
        min_z = min(min_z, loader->LoadedVertices[i].Position.Z);
        max_z = max(max_z, loader->LoadedVertices[i].Position.Z);
    }
    float scale = min(IMG_HEIGHT, IMG_WIDTH) / max(max_x - min_x, max_y - min_y);
    scale *= SCALE;
    float center_x = (max_x + min_x) / 2;
    float center_y = (max_y + min_y) / 2;
    float center_z = (max_z + min_z) / 2;

    min_z = FLT_MAX, max_z = -FLT_MAX;

    for (int i = 0; i < loader->LoadedTriangles.size(); i++)
    {
        Triangle triangle = loader->LoadedTriangles[i];
        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            Vertex &vertex = triangle.vertices[j];
            loader->LoadedTriangles[i].vertices[j].Position.X = (vertex.Position.X - center_x) * scale + IMG_WIDTH / 2;
            loader->LoadedTriangles[i].vertices[j].Position.Y = (vertex.Position.Y - center_y) * scale + IMG_HEIGHT / 2;
            loader->LoadedTriangles[i].vertices[j].Position.Z = (vertex.Position.Z - center_z) * scale;
            min_z = min(min_z, loader->LoadedTriangles[i].vertices[j].Position.Z);
            max_z = max(max_z, loader->LoadedTriangles[i].vertices[j].Position.Z);
        }
    }
}

void rotateObj(Loader* loader, cv::Mat rot)
{
    min_z = FLT_MAX;
    max_z = -FLT_MAX;

    for (int i = 0; i < loader->LoadedTriangles.size(); i++)
    {
        Triangle triangle = loader->LoadedTriangles[i];
        for (int j = 0; j < triangle.vertices.size(); j++)
        {

            Vertex &vertex = triangle.vertices[j];
            cv::Mat p = (cv::Mat_<float>(3, 1) << vertex.Position.X - IMG_WIDTH/2, vertex.Position.Y - IMG_HEIGHT/2, vertex.Position.Z);
            p =  rot * p;

            loader->LoadedTriangles[i].vertices[j].Position.X = p.at<float>(0) + IMG_WIDTH/2;
            loader->LoadedTriangles[i].vertices[j].Position.Y = p.at<float>(1) + IMG_HEIGHT/2;
            loader->LoadedTriangles[i].vertices[j].Position.Z = p.at<float>(2);
            
            #ifdef CULL_ENBALE
            cv::Mat n = (cv::Mat_<float>(3, 1) << vertex.Normal.X, vertex.Normal.Y, vertex.Normal.Z);
            n = rot * n;
            loader->LoadedTriangles[i].vertices[j].Normal.X = n.at<float>(0);
            loader->LoadedTriangles[i].vertices[j].Normal.Y = n.at<float>(1);
            loader->LoadedTriangles[i].vertices[j].Normal.Z = n.at<float>(2);
            #endif

            min_z = min(min_z, loader->LoadedTriangles[i].vertices[j].Position.Z);
            max_z = max(max_z, loader->LoadedTriangles[i].vertices[j].Position.Z);
        }
    }
}

void addActiveEdge(vector<ActiveEdge>& AET, Edge edge1, Edge edge2, Polygon* polygon, int y)
{
    ActiveEdge ae;
    assert(edge1.id == edge2.id);

    ae.id = edge1.id;

    // make sure edge1 is always the left edge
    if (edge1.x > edge2.x || (edge1.dx > edge2.dx && edge1.x == edge2.x))
        swap(edge1, edge2);

    ae.xl = edge1.x;
    ae.dxl = edge1.dx;
    ae.dyl = edge1.dy;
    ae.xr = edge2.x;
    ae.dxr = edge2.dx;
    ae.dyr = edge2.dy;

    ae.zl = polygon->c != 0 ?-(polygon->a * ae.xl + polygon->b * y + polygon->d)/polygon->c : polygon->depth;
    ae.dzx = polygon->c != 0 ?-polygon->a / polygon->c : 0;
    ae.dzy = polygon->c != 0 ? polygon->b / polygon->c : 0;

    AET.push_back(ae);
}

void scanLine(float* z_buffer, vector<Polygon*>& APT, vector<ActiveEdge>& AET)
{
    APT.clear();
    AET.clear();
    for (int y = IMG_HEIGHT - 1; y >= 0; y--)
    {
        // if new polygon is in the scan line, add it to active polygon list(APT)
        for (int i = 0; i < (int)PT[y].size(); i++)
        {
            assert(i >= 0);
            Polygon *polygon = PT[y][i];
            // add edges intersected with the scan line into active edge list(AET)
            ActiveEdge ae;
            Edge edge1, edge2;
            ae.id = PT[y][i];

            // find edge in current polygon with max y
            for (int j = 0; j < ET[y].size(); j++)
            {
                if (ET[y][j].id == PT[y][i])
                {
                    edge1 = ET[y][j];
                    edge2 = ET[y][j + 1];
                    break;
                }
            }

            addActiveEdge(AET, edge1, edge2, polygon, y);
            APT.push_back(polygon);
        }

        // initialize z buffer as min_z if there are active edges
        for (int j = 0; j < image.cols; j++)
        {
            z_buffer[j] = -FLT_MAX;
            image.at<Vec3b>(IMG_HEIGHT - 1 - y, j) = Vec3b(244, 234, 226);
        }
        

        // update z buffer and color of pixels in the scan line
        for (int i = 0; i < AET.size(); i++)
        {
            ActiveEdge& ae = AET[i];
            float zx = ae.zl;
            int left = (int)round(ae.xl), right = (int)round(ae.xr);

            if(left < 0)
            {
                left = 0;
                zx += ae.dzx * (0 - ae.xl);
            }    
            else if(left > IMG_WIDTH - 1)
                left = IMG_WIDTH - 1;

            if(right < 0)
                right = 0;
            else if(right > IMG_WIDTH - 1)
                right = IMG_WIDTH - 1;

            for(int j = left; j <= right; j++)
            {
                if (zx > z_buffer[j])
                {
                    image.at<Vec3b>((IMG_HEIGHT - 1 - y), j) = ae.id->color;
                    z_buffer[j] = zx;
                }
                zx += ae.dzx;
            }

            // update AET
            AET[i].xl += ae.dxl;
            AET[i].xr += ae.dxr;
            AET[i].zl += ae.dzx * ae.dxl + ae.dzy;
            AET[i].dyl -= 1;
            AET[i].dyr -= 1;

            if (AET[i].dyl < 0 && AET[i].dyr < 0)
            {
                if(y == 0)
                    break;
                
                Edge edge1, edge2;
                int j = 0;
                for(j = 0; j < ET[y-1].size(); j++)
                {
                    if(AET[i].id == ET[y-1][j].id)
                    {
                        edge1 = ET[y-1][j];
                        edge2 = ET[y-1][j+1];
                        break;
                    }
                }
                if(j == ET[y-1].size())
                {
                    AET.erase(AET.begin() + i);
                    i--;
                }
                else
                    addActiveEdge(AET, edge1, edge2, AET[i].id, y);
            }
            else if (AET[i].dyl < 0)
            {
                if (y == 0)
                    break;
                for (int j = 0; j < ET[y-1].size(); j++)
                {
                    if (AET[i].id == ET[y-1][j].id)
                    {
                        Edge edge = ET[y-1][j];
                        AET[i].xl = edge.x;
                        AET[i].dxl = edge.dx;
                        AET[i].dyl = edge.dy;
                        break;
                    }
                }
            }
            else if (AET[i].dyr < 0)
            {
                if (y == 0)
                    break;
                for (int j = 0; j < ET[y - 1].size(); j++)
                {
                    if (AET[i].id == ET[y - 1][j].id)
                    {
                        Edge edge = ET[y - 1][j];
                        AET[i].xr = edge.x;
                        AET[i].dxr = edge.dx;
                        AET[i].dyr = edge.dy;
                        break;
                    }
                }
            }

        }

        // update APT
        for (int i = 0; i < APT.size(); i++)
        {
            if(--APT[i]->dy < 0)
                APT.erase(APT.begin() + i);
        }
    }
}

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

        clock_t start = clock();    
        cv::Mat R = getRotationMatrix(angleRadiansX, angleRadiansY);
        rotateObj(params->loader, R); // Rotate the object

        // Redo ScanLine algorithm
        ListContruct(params->loader);
        scanLine(params->zbuffer, *(params->APT), *(params->AET));

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
        loader->LoadFile("../models/Crate/Crate1.obj");
    else if(argc == 2)
        loader->LoadFile(argv[1]);
    // scale to fit in image
    scaleObj(loader);
    rotateObj(loader, getRotationMatrix(36*CV_PI/180.0, 25 * CV_PI / 180.0));

    // initialize z-buffer for one scan line
    float* z_buffer = new float[image.cols];

    // contruct polygon and edge list
    PT.resize(image.rows);
    ET.resize(image.rows);

    // scan line z buffer algorithm
    vector<Polygon*> APT;
    vector<ActiveEdge> AET;

    MouseParams params;
    params.AET = &AET;
    params.APT = &APT;
    params.loader = loader;
    params.zbuffer = z_buffer;

    cv::namedWindow("Zbuffer");
    cv::setMouseCallback("Zbuffer", mouseCallback, &params);

    // time record
    clock_t start = clock();
    ListContruct(loader);
    scanLine(z_buffer, APT, AET);
    
    clock_t end = clock();
    cout << "Time: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;
    // put text on image
    cv::putText(image, "Time: " + std::to_string((double)(end - start) / CLOCKS_PER_SEC) + " seconds", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

    imshow("Zbuffer", image);

    waitKey(0);

    delete[] z_buffer;
}

