#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

#include "readObj.h"
#include "List.h"

using namespace std;
using namespace cv;
using namespace objl;

#define SCALE 0.75
#define IMG_WIDTH 800
#define IMG_HEIGHT 600

std::vector<std::vector<Polygon*>> PT;
std::vector<std::vector<Edge>> ET;
// initial image
Mat image(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, Vec3b(0, 0, 0));
float min_z = FLT_MAX;

void EdgeListContruct(Vector3 p1, Vector3 p2, float ymin, float ymax, Polygon* id)
{
    Edge edge;

    if(p1.Y <= p2.Y)
        swap(p1, p2);
    edge.x = p1.X;
    edge.dx = -(p1.X - p2.X)/(p1.Y - p2.Y);

    // find max and min y with rounding
    float min_y = min(p1.Y, p2.Y);
    float max_y = max(p1.Y, p2.Y);

    edge.dy = (int)floor(max_y - min_y);
    edge.id = id;

    // note: horizontal edges need to be considered separately
    // 
    // truncation edges
    if (p1.Y != ymax)
    {
        edge.x = p1.X + edge.dx;
        edge.dy -= 1;
        max_y -= 1;
    }

    ET[(int)floor(max_y)].push_back(edge);
}

void ListContruct(Loader &loader)
{
    srand(13);
    for (int i = 0; i < loader.LoadedTriangles.size(); i++)
    {
        Polygon* polygon = new Polygon();
        Triangle triangle = loader.LoadedTriangles[i];
        
        // calculate params
        Vector3 p = triangle.vertices[0].Position;
        Vector3 normal = triangle.vertices[0].Normal;

        polygon->calculate_params(p, normal);

        polygon->id = i;

        // edge list construction
        float min_y = FLT_MAX, max_y = -FLT_MAX;
        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            min_y = min(min_y, triangle.vertices[j].Position.Y);
            max_y = max(max_y, triangle.vertices[j].Position.Y);
        }

        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            Vertex p1 = triangle.vertices[j];
            Vertex p2 = (j == (int)triangle.vertices.size() - 1) ? triangle.vertices[0] : triangle.vertices[j + 1];

            EdgeListContruct(p1.Position, p2.Position, min_y, max_y, polygon);
        }

        polygon->dy = (int)floor(max_y - min_y);
        polygon->color = Vec3b(rand() % 256, rand() % 256, rand() % 256);

        PT[(int)floor(max_y)].push_back(polygon);
    }
}

void scaleObj(Loader& loader)
{
    float max_x = -FLT_MAX, max_y = -FLT_MAX;
    float min_x = FLT_MAX, min_y = FLT_MAX;
    for (int i = 0; i < loader.LoadedTriangles.size(); i++)
    {
        Triangle triangle = loader.LoadedTriangles[i];
        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            loader.LoadedTriangles[i].vertices[j].Position.Y = -loader.LoadedTriangles[i].vertices[j].Position.Y;
        }
    }
    for (int i = 0; i < loader.LoadedVertices.size(); i++)
    {
        max_x = max(max_x, loader.LoadedVertices[i].Position.X);
        max_y = max(max_y, -loader.LoadedVertices[i].Position.Y);
        min_x = min(min_x, loader.LoadedVertices[i].Position.X);
        min_y = min(min_y, -loader.LoadedVertices[i].Position.Y);
    }
    float scale = min(IMG_HEIGHT, IMG_WIDTH) / max(max_x - min_x, max_y - min_y);
    scale *= SCALE;
    float center_x = (max_x + min_x) / 2;
    float center_y = (max_y + min_y) / 2;

    for (int i = 0; i < loader.LoadedTriangles.size(); i++)
    {
        Triangle triangle = loader.LoadedTriangles[i];
        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            Vertex &vertex = triangle.vertices[j];
            loader.LoadedTriangles[i].vertices[j].Position.X = (vertex.Position.X - center_x) * scale + IMG_WIDTH / 2;
            loader.LoadedTriangles[i].vertices[j].Position.Y = (vertex.Position.Y - center_y) * scale + IMG_HEIGHT / 2;
            loader.LoadedTriangles[i].vertices[j].Position.Z *= scale;
            min_z = min(min_z, loader.LoadedTriangles[i].vertices[j].Position.Z);
        }
    }
}

void scanLine(float* z_buffer, vector<Polygon*>& APT, vector<ActiveEdge>& AET)
{
    for (int y = IMG_HEIGHT - 1; y >= 0; y--)
    {
        // if new polygon is in the scan line, add it to active polygon list(APT)
        for (int i = 0; i < PT[y].size(); i++)
        {
            Polygon *polygon = PT[y][i];
            // add edges intersected with the scan line into active edge list(AET)
            ActiveEdge ae;
            Edge edge1, edge2;
            ae.id = PT[y][i];

            for (int j = 0; j < ET[y].size(); j++)
            {
                if (ET[y][j].id == PT[y][i])
                {
                    edge1 = ET[y][j];
                    edge2 = ET[y][j + 1];
                    break;
                }
            }

            assert(edge1.id == edge2.id);

            ae.id = edge1.id;
            if (edge1.dx > edge2.dx)
                swap(edge1, edge2);

            ae.xl = edge1.x;
            ae.dxl = edge1.dx;
            ae.dyl = edge1.dy;
            ae.xr = edge2.x;
            ae.dxr = edge2.dx;
            ae.dyr = edge2.dy;

            ae.zl = -(polygon->a * ae.xl + polygon->b * y + polygon->d)/polygon->c;
            ae.dzx = polygon->c != 0 ?-polygon->a / polygon->c : 0;
            ae.dzy = polygon->c != 0 ? polygon->b / polygon->c : 0;

            AET.push_back(ae);
            APT.push_back(polygon);
        }

        // initialize z buffer as min_z
        for (int j = 0; j < image.cols; j++)
            z_buffer[j] = min_z;

        // update z buffer and color of pixels in the scan line
        for (int i = 0; i < AET.size(); i++)
        {
            ActiveEdge& ae = AET[i];
            float x = ae.xl, zx = ae.zl;
            for(int j = (int)round(x); j <= (int)round(ae.xr); j++)
            {
                if (zx > z_buffer[j])
                {
                    image.at<Vec3b>(y, j) = ae.id->color;
                    z_buffer[j] = zx;
                }
                zx += ae.dzx;
            }

            // update AET
            AET[i].xl += ae.dxl * (AET[i].dyl < 1 ? AET[i].dyl : 1);
            AET[i].xr += ae.dxr * (AET[i].dyr < 1 ? AET[i].dyr : 1);
            AET[i].zl += ae.dzx * ae.dxl + ae.dzy;
            AET[i].dyl -= 1;
            AET[i].dyr -= 1;

            if (AET[i].dyl < 0 && AET[i].dyr < 0)
            {
                AET.erase(AET.begin() + i);
                i--;
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

int main()
{
    utils::logging::setLogLevel(utils::logging::LOG_LEVEL_SILENT);

    Loader loader;
    // load mesh
    loader.LoadFile("/Users/jun/CS/2024 fall/CG/project1/models/bunny/bunny.obj");
    // scale to fit in image
    scaleObj(loader);

    // initialize z-buffer for one scan line
    float* z_buffer = new float[image.cols];

    // contruct polygon and edge list
    PT.resize(image.rows);
    ET.resize(image.rows);
    ListContruct(loader);

    // scan line z buffer algorithm
    vector<Polygon*> APT;
    vector<ActiveEdge> AET;

    scanLine(z_buffer, APT, AET);

    // show image
    imshow("image", image);
    waitKey(0);

    delete[] z_buffer;
}

