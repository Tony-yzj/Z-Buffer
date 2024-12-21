#include "model.h"
#include "readObj.h"

// Function to generate the combined rotation matrix
// Parameters: angleX - rotation angle around the X-axis
//             angleY - rotation angle around the Y-axis
// Return value: Combined rotation matrix
cv::Mat getRotationMatrix(float angleX, float angleY) 
{
    // Rotation matrix for X-axis
    cv::Mat Rx = (cv::Mat_<float>(3, 3) <<
                  1, 0, 0,
                  0, cos(angleX), -sin(angleX),
                  0, sin(angleX), cos(angleX));
    
    // Rotation matrix for Y-axis
    cv::Mat Ry = (cv::Mat_<float>(3, 3) <<
                  cos(angleY), 0, sin(angleY),
                  0, 1, 0,
                  -sin(angleY), 0, cos(angleY));
    
    // Combine rotations: R = Ry * Rx
    return Ry * Rx;
}

// Function to scale the object
// Parameters: loader - pointer to the Loader object, used to access model data
void scaleObj(Loader* loader)
{
    // Initialize max and min values for X and Y coordinates
    float max_x = -FLT_MAX, max_y = -FLT_MAX;
    float min_x = FLT_MAX, min_y = FLT_MAX;

    // Find the max and min values for X, Y, and Z coordinates
    for (int i = 0; i < loader->LoadedVertices.size(); i++)
    {
        max_x = max(max_x, loader->LoadedVertices[i].Position.X);
        max_y = max(max_y, loader->LoadedVertices[i].Position.Y);
        min_x = min(min_x, loader->LoadedVertices[i].Position.X);
        min_y = min(min_y, loader->LoadedVertices[i].Position.Y);
        min_z = min(min_z, loader->LoadedVertices[i].Position.Z);
        max_z = max(max_z, loader->LoadedVertices[i].Position.Z);
    }

    // Calculate scale factor based on the max differences in X and Y, and the image size
    float scale = min(IMG_HEIGHT, IMG_WIDTH) / max(max_x - min_x, max_y - min_y);
    scale *= SCALE;

    // Calculate the center point of the object
    float center_x = (max_x + min_x) / 2;
    float center_y = (max_y + min_y) / 2;
    float center_z = (max_z + min_z) / 2;

    // Reset max and min values for Z coordinate
    min_z = FLT_MAX, max_z = -FLT_MAX;

    // Scale each vertex of the triangles based on the calculated scale factor and center point
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

// Function to rotate the object
// Parameters: loader - pointer to the Loader object, used to access model data
//             rot - rotation matrix
void rotateObj(vector<Triangle>* faces, cv::Mat rot)
{
    // Reset max and min values for Z coordinate
    min_z = FLT_MAX;
    max_z = -FLT_MAX;

    // Rotate each vertex and normal vector of the triangles based on the given rotation matrix
    for (int i = 0; i < (*faces).size(); i++)
    {
        Triangle triangle = (*faces)[i];
        for (int j = 0; j < triangle.vertices.size(); j++)
        {
            Vertex &vertex = triangle.vertices[j];

            // Rotate the vertex position
            cv::Mat p = (cv::Mat_<float>(3, 1) << vertex.Position.X - IMG_WIDTH/2, vertex.Position.Y - IMG_HEIGHT/2, vertex.Position.Z);
            p =  rot * p;
            (*faces)[i].vertices[j].Position.X = p.at<float>(0) + IMG_WIDTH/2;
            (*faces)[i].vertices[j].Position.Y = p.at<float>(1) + IMG_HEIGHT/2;
            (*faces)[i].vertices[j].Position.Z = p.at<float>(2);

            // Rotate the vertex normal
            cv::Mat n = (cv::Mat_<float>(3, 1) << vertex.Normal.X, vertex.Normal.Y, vertex.Normal.Z);
            n = rot * n;
            (*faces)[i].vertices[j].Normal.X = n.at<float>(0);
            (*faces)[i].vertices[j].Normal.Y = n.at<float>(1);
            (*faces)[i].vertices[j].Normal.Z = n.at<float>(2);

            // Update max and min values for Z coordinate
            min_z = min(min_z, (*faces)[i].vertices[j].Position.Z);
            max_z = max(max_z, (*faces)[i].vertices[j].Position.Z);
        }
    }
}