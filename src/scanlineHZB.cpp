#include "scanlineHZB.h"
#include "BVH.h"
#include "Features.h"
#include "hierachy.h"
#include "readObj.h"
#include "struct.h"
#include <memory>
#include <opencv2/core/operations.hpp>

extern vector<Polygon*> PT;
extern vector<vector<Edge>> ET;

void EdgeListContruct(Vector3 p1, Vector3 p2, int ymax, Polygon* id)
{
    Edge edge;

    // let p1 always be the top point
    if(p1.Y <= p2.Y)
        swap(p1, p2);

    if(p2.Y > IMG_HEIGHT - 1 || p1.Y == p2.Y)
        return;
    
    // assign max and min y
    int min_y = p2.Y;
    int max_y = p1.Y;
    edge.dx = -(p1.X - p2.X)/(p1.Y - p2.Y); 

    if(max_y > IMG_HEIGHT - 1)
    {
        p1.X += edge.dx * (max_y - IMG_HEIGHT);
        max_y = IMG_HEIGHT - 1;
    }

    // neglect parallel-to-x-axis edges
    if(max_y == p2.Y)
        return;

    edge.x = p1.X;

    edge.dy = max_y - min_y;
    edge.id = id;

    assert(edge.dy >= 1);

    // note: horizontal edges need to be considered separately
    // 
    // truncation edges
    if (p1.Y < ymax && p1.Y < IMG_HEIGHT - 1)
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
    return Vector3((int)floor(v.X), (int)floor(v.Y), v.Z);
}

void ClearTable()
{
    for(int i = 0; i < PT.size(); i++)
    {
        delete PT[i];
    }
    PT.clear();
    for(int i = 0; i < ET.size(); i++)
    {
        ET[i].clear();
    }
}

void ListContruct(vector<Triangle> * faces)
{
    for (int i = 0; i < (*faces).size(); i++)
    {
        Polygon* polygon = new Polygon();
        Triangle triangle = (*faces)[i];
        
        // calculate params
        Vector3 p = triangle.vertices[0].Position;
        Vector3 normal = triangle.vertices[0].Normal;

        for(int j = 0; j < triangle.vertices.size(); j++)
        {
            triangle.vertices[j].Position = RoundVertex(triangle.vertices[j].Position);
        }

        polygon->calculate_params((triangle.vertices[0].Position), (triangle.vertices[1].Position), (triangle.vertices[2].Position), normal);

        #ifdef CULL_ENBALE
        // Cull
        if(normal.dot(cameraDirect) < 0)
            continue;
        #endif
        
        polygon->id = i;

        // edge list construction
        float min_y = INT_MAX, max_y = INT_MIN;
        float min_x = INT_MAX, max_x = INT_MIN;
        float pmin_z = INT_MAX, pmax_z = INT_MIN;
        polygon->depth = 0;
        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            min_y = min(min_y, floor(triangle.vertices[j].Position.Y));
            max_y = max(max_y, floor(triangle.vertices[j].Position.Y));

            // #ifdef HIERACHY
            min_x = min(min_x, floor(triangle.vertices[j].Position.X));
            max_x = max(max_x, floor(triangle.vertices[j].Position.X));
            pmin_z = min(pmin_z, floor(triangle.vertices[j].Position.Z));
            pmax_z = max(pmax_z, floor(triangle.vertices[j].Position.Z));
            // #endif

            polygon->depth += triangle.vertices[j].Position.Z;
        }

        // #ifdef HIERACHY
        polygon->bbox.minX = min_x;
        polygon->bbox.maxX = max_x;
        polygon->bbox.minY = min_y;
        polygon->bbox.maxY = max_y;
        polygon->bbox.minZ = pmin_z;
        polygon->bbox.maxZ = pmax_z;
        // #endif

        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            Vector3 p1 = (triangle.vertices[j].Position);
            Vector3 p2 = ((j == (int)triangle.vertices.size() - 1) ? triangle.vertices[0].Position : triangle.vertices[j + 1].Position);

            EdgeListContruct(p1, p2, max_y, polygon);
        }

        polygon->dy = max_y - min_y;

        polygon->depth /= triangle.vertices.size();
        float scale = (polygon->depth - min_z)/(max_z - min_z);
        assert(scale <= 1);
        polygon->color = Vec3b(255 * scale, 255 * scale, 255 * scale);

        if(max_y >= IMG_HEIGHT)
        {
            max_y = IMG_HEIGHT - 1;
            polygon->bbox.maxY = IMG_HEIGHT - 1;
        }

        if(polygon->dy > 0 && max_y > 0 && min_y < IMG_HEIGHT - 1)
            PT.push_back(polygon);
        else
            delete polygon;
    }
}
void addActiveEdge(ActiveEdge& ae, Edge edge1, Edge edge2, Polygon* polygon, int y)
{
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
}

void renderPolygon(Polygon* polygon, HierarchicalZBuffer* hzb)
{
    ActiveEdge AE;
    // add edges intersected with the scan line into active edge list(AET)
    int ynow = polygon->bbox.maxY;
    Edge edge1, edge2;

    // find edge in current polygon with max y
    for (int j = 0; j < ET[ynow].size(); j++)
    {
        if (ET[ynow][j].id == polygon)
        {
            edge1 = ET[ynow][j];
            edge2 = ET[ynow][j + 1];
            break;
        }
    }

    addActiveEdge(AE, edge1, edge2, polygon, ynow);

    while(polygon->dy >= 0 && ynow > 0)
    {   
        ActiveEdge& ae = AE;
        float zx = ae.zl;
        int left = (int)floor(ae.xl), right = (int)floor(ae.xr);

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
            if (zx > hzb->at(j, ynow, 0))
            {
                image.at<Vec3b>((IMG_HEIGHT - 1 - ynow), j) = ae.id->color;
                hzb->Update(j, ynow, zx);
            }
            zx += ae.dzx;
        }

        // update AET
        AE.xl += ae.dxl;
        AE.xr += ae.dxr;
        AE.zl += ae.dzx * ae.dxl + ae.dzy;
        AE.dyl -= 1;
        AE.dyr -= 1;

        if (AE.dyl < 0 && AE.dyr < 0)
        {
            if(ynow<=0)
            {
                break;
            }
            
            Edge edge1, edge2;
            int j = 0;
            for(j = 0; j < ET[ynow-1].size(); j++)
            {
                if(AE.id == ET[ynow-1][j].id)
                {
                    edge1 = ET[ynow-1][j];
                    edge2 = ET[ynow-1][j+1];
                    break;
                }
            }
            if(j < ET[ynow-1].size())
                addActiveEdge(AE, edge1, edge2, AE.id, ynow);
        }
        else if (AE.dyl < 0)
        {
            if(ynow<=0)
            {
                break;
            }
            for (int j = 0; j < ET[ynow-1].size(); j++)
            {
                if (AE.id == ET[ynow-1][j].id)
                {
                    Edge edge = ET[ynow-1][j];
                    AE.xl = edge.x;
                    AE.dxl = edge.dx;
                    AE.dyl = edge.dy;
                    break;
                }
            }
        }
        else if (AE.dyr < 0)
        {
            if(ynow<=0)
            {
                break;
            }
            for (int j = 0; j < ET[ynow-1].size(); j++)
            {
                if (AE.id == ET[ynow-1][j].id)
                {
                    Edge edge = ET[ynow-1][j];
                    AE.xr = edge.x;
                    AE.dxr = edge.dx;
                    AE.dyr = edge.dy;
                    break;
                }
            }
        }
        polygon->dy--;
        ynow--;
    }
}

int cull = 0;
void HierarchicalZBuffer::IsVisible(BVHNode* node) 
{
    BoundingBox bbox = node->bbox;
    
    // Get the bounding box and depth range of the polygon
    int min_x = bbox.minX;
    int max_x = bbox.maxX;
    int min_y = bbox.minY;
    int max_y = bbox.maxY;

    float min_depth = bbox.maxZ;

    // Traverse the Z pyramid from coarse (higher levels) to fine (lower levels)
    for (int level = GetNumLevels() - 3; level >= 1; --level) 
    {
        int pyramid_width = width >> level;

        int start_x = min_x >> level;
        int start_y = min_y >> level;
        int end_x = max_x >> level;
        int end_y = max_y >> level;

        // Check the Z buffer at the current pyramid level
        if(start_x == end_x && start_y == end_y && min_depth < z_pyramid[level][start_y * pyramid_width + start_x])
        {
            cull += node->polygons.size();
            return;
        }
        else if(start_x != end_x || start_y != end_y)
        {
            // stop when reaching the smallest level containing the polygon
            break;
        }
    }

    if(!node->isLeaf())
    {
        IsVisible(node->left);
        IsVisible(node->right);
    }
    else 
    {
        for(auto polygon : node->polygons)
        {
            renderPolygon(polygon, this);
        }
    }
}
void scanLine( HierarchicalZBuffer* hzb)
{
    int cull_cnt = 0;

    fill(image.begin<Vec3b>(), image.end<Vec3b>(), Vec3b(244, 234, 226));

    // if new polygon is in the scan line, add edge to AET
    for (int i = 0; i < (int)PT.size(); i++)
    {
        assert(i >= 0);
        Polygon *polygon = PT[i];

        if (!hzb->IsVisible(*polygon))
        {
            cull_cnt++;
            continue;
        }

        renderPolygon(PT[i], hzb);
    }

    cout << "cull_cnt: " << cull_cnt << endl;
    // Clear PT & ET
    ClearTable();
}
void scanLine( HierarchicalZBuffer* hzb, BVHNode* bvh)
{

    fill(image.begin<Vec3b>(), image.end<Vec3b>(), Vec3b(244, 234, 226));

    hzb->IsVisible(bvh);

    cout << "cull_cnt: " << cull << endl;
    // Clear PT & ET
    ClearTable();
    freeBVHTree(bvh);
}