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
void scanLine( HierarchicalZBuffer* hzb, vector<ActiveEdge>& AET)
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
void scanLine( HierarchicalZBuffer* hzb, BVHNode* bvh, vector<ActiveEdge>& AET)
{
    // AET.clear();

    // fill(image.begin<Vec3b>(), image.end<Vec3b>(), Vec3b(244, 234, 226));

    // bvh = constructBVHTree(PT);

    // vector<Polygon*> APT;
    // hzb->IsVisible(bvh, APT);

    // cout << "cull_cnt: " << PT.size() - APT.size() << endl;

    // // if new polygon is in the scan line, add edge to AET
    // for (int i = 0; i < (int)APT.size(); i++)
    // {
    //     assert(i >= 0);
    //     Polygon *polygon = APT[i];

    //     // add edges intersected with the scan line into active edge list(AET)
    //     int ynow = polygon->bbox.maxY;
    //     Edge edge1, edge2;

    //     // find edge in current polygon with max y
    //     for (int j = 0; j < ET[ynow].size(); j++)
    //     {
    //         if (ET[ynow][j].id == APT[i])
    //         {
    //             edge1 = ET[ynow][j];
    //             edge2 = ET[ynow][j + 1];
    //             break;
    //         }
    //     }

    //     addActiveEdge(AET, edge1, edge2, polygon, ynow);

    //     while(polygon->dy >= 0 && ynow > 0)
    //     {   
    //         int size = AET.size();
    //         for (int i = 0; i < size; i++)
    //         {
    //             ActiveEdge& ae = AET[i];
    //             float zx = ae.zl;
    //             int left = (int)floor(ae.xl), right = (int)floor(ae.xr);

    //             if(left < 0)
    //             {
    //                 left = 0;
    //                 zx += ae.dzx * (0 - ae.xl);
    //             }    
    //             else if(left > IMG_WIDTH - 1)
    //                 left = IMG_WIDTH - 1;

    //             if(right < 0)
    //                 right = 0;
    //             else if(right > IMG_WIDTH - 1)
    //                 right = IMG_WIDTH - 1;

    //             for(int j = left; j <= right; j++)
    //             {
    //                 if (zx > hzb->at(j, ynow, 0))
    //                 {
    //                     image.at<Vec3b>((IMG_HEIGHT - 1 - ynow), j) = ae.id->color;
    //                     hzb->Update(j, ynow, zx);
    //                 }
    //                 zx += ae.dzx;
    //             }

    //             // update AET
    //             AET[i].xl += ae.dxl;
    //             AET[i].xr += ae.dxr;
    //             AET[i].zl += ae.dzx * ae.dxl + ae.dzy;
    //             AET[i].dyl -= 1;
    //             AET[i].dyr -= 1;

    //             if (AET[i].dyl < 0 && AET[i].dyr < 0)
    //             {
    //                 if(ynow<=0)
    //                 {
    //                     break;
    //                 }
                    
    //                 Edge edge1, edge2;
    //                 int j = 0;
    //                 for(j = 0; j < ET[ynow-1].size(); j++)
    //                 {
    //                     if(AET[i].id == ET[ynow-1][j].id)
    //                     {
    //                         edge1 = ET[ynow-1][j];
    //                         edge2 = ET[ynow-1][j+1];
    //                         break;
    //                     }
    //                 }
    //                 if(j < ET[ynow-1].size())
    //                     addActiveEdge(AET, edge1, edge2, AET[i].id, ynow);
    //                 AET.erase(AET.begin() + i);
    //                 i--;
    //                 size--;
    //             }
    //             else if (AET[i].dyl < 0)
    //             {
    //                 if(ynow<=0)
    //                 {
    //                     break;
    //                 }
    //                 for (int j = 0; j < ET[ynow-1].size(); j++)
    //                 {
    //                     if (AET[i].id == ET[ynow-1][j].id)
    //                     {
    //                         Edge edge = ET[ynow-1][j];
    //                         AET[i].xl = edge.x;
    //                         AET[i].dxl = edge.dx;
    //                         AET[i].dyl = edge.dy;
    //                         break;
    //                     }
    //                 }
    //             }
    //             else if (AET[i].dyr < 0)
    //             {
    //                 if(ynow<=0)
    //                 {
    //                     break;
    //                 }
    //                 for (int j = 0; j < ET[ynow-1].size(); j++)
    //                 {
    //                     if (AET[i].id == ET[ynow-1][j].id)
    //                     {
    //                         Edge edge = ET[ynow-1][j];
    //                         AET[i].xr = edge.x;
    //                         AET[i].dxr = edge.dx;
    //                         AET[i].dyr = edge.dy;
    //                         break;
    //                     }
    //                 }
    //             }
    //         }
    //         polygon->dy--;
    //         ynow--;
    //     }
    //     AET.clear();
    // }
    // // Clear PT & ET
    // ClearTable();
    // freeBVHTree(bvh);
}