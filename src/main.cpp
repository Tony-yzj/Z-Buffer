#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <time.h>

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
float min_z = FLT_MAX, max_z = -FLT_MAX;

void EdgeListContruct(Vector3 p1, Vector3 p2, int ymax, Polygon* id)
{
    Edge edge;

    // let p1 always be the top point
    if(p1.Y <= p2.Y)
        swap(p1, p2);
    edge.x = p1.X;

    // neglect parallel-to-x-axis edges
    if(p1.Y == p2.Y)
        return;
    
    edge.dx = -(p1.X - p2.X)/(p1.Y - p2.Y);

    // assign max and min y
    int min_y = p2.Y;
    int max_y = p1.Y;

    edge.dy = max_y - min_y;
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

    ET[max_y].push_back(edge);
}

Vector3 RoundVertex(Vector3 v)
{
    return Vector3(round(v.X), round(v.Y), v.Z);
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

        if(normal.dot(Vector3(0, 0, -1)) < 0)
            continue;
        polygon->calculate_params(RoundVertex(triangle.vertices[0].Position), RoundVertex(triangle.vertices[1].Position), RoundVertex(triangle.vertices[2].Position), normal);

        // polygon->calculate_params(p, normal);
        polygon->id = i;

        // edge list construction
        int min_y = INT_MAX, max_y = INT_MIN;
        float color_z = 0;
        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            min_y = min(min_y, (int)round(triangle.vertices[j].Position.Y));
            max_y = max(max_y, (int)round(triangle.vertices[j].Position.Y));
            color_z += triangle.vertices[j].Position.Z;
        }

        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            Vector3 p1 = RoundVertex(triangle.vertices[j].Position);
            Vector3 p2 = RoundVertex((j == (int)triangle.vertices.size() - 1) ? triangle.vertices[0].Position : triangle.vertices[j + 1].Position);

            EdgeListContruct(p1, p2, max_y, polygon);
        }

        polygon->dy = max_y - min_y;

        color_z /= triangle.vertices.size();
        color_z = (color_z - min_z*2)/(max_z*2 - min_z*2);
        polygon->color = Vec3b(255 * color_z, 255 * color_z, 255 * color_z);

        if(polygon->dy > 0)
            PT[max_y].push_back(polygon);
        else
            delete polygon;
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
            max_z = max(max_z, loader.LoadedTriangles[i].vertices[j].Position.Z);
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

    ae.zl = -(polygon->a * ae.xl + polygon->b * y + polygon->d)/polygon->c;
    ae.dzx = polygon->c != 0 ?-polygon->a / polygon->c : 0;
    ae.dzy = polygon->c != 0 ? polygon->b / polygon->c : 0;

    AET.push_back(ae);
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
        if(AET.size() > 0)
        {
            for (int j = 0; j < image.cols; j++)
                z_buffer[j] = min_z;
        }
        

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

    // time record
    clock_t start = clock();
    ListContruct(loader);

    // scan line z buffer algorithm
    vector<Polygon*> APT;
    vector<ActiveEdge> AET;

    scanLine(z_buffer, APT, AET);
    clock_t end = clock();
    cout << "Time: " << (double)(end - start) / CLOCKS_PER_SEC << "s" << endl;

    // show image
    imshow("image", image);
    waitKey(0);

    delete[] z_buffer;
}

