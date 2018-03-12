
/*
3D Scanning & Motion Capture
Exercise I
Fatma Zehra Hayirci
Timea Gyarmathy
*/

#include <iostream>
#include <fstream>

#include "Eigen.h"

#include "VirtualSensor.h"

//Holds the index of the row on which the vertex is displayed on in the .off file
int indices[640 * 480];

struct Vertex
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // position stored as 4 floats (4th component is supposed to be 1.0)
        Vector4f position;
    // color stored as 4 unsigned char
    Vector4uc color;
};


/* Returns the length of the edge based on the two endpoints
 */
float calculateEdge(Vector4f position1, Vector4f position2) {
    float edge = 10.0f; // to assign a value big enough to compare
    float edgeA, edgeB, edgeC;

    edgeA = position1.x() - position2.x();
    edgeB = position1.y() - position2.y();
    edgeC = position1.z() - position2.z();
    edge = float(sqrt(pow(edgeA, 2.0) + pow(edgeB, 2.0) + pow(edgeC, 2.0)));

    return edge;


}

/* This function calculate the edges in a triangle and returns true if edges smaller than threshold
 */
bool calculateFace(float edgeThreshold, Vector4f position1, Vector4f position2, Vector4f position3) {
    float edgeA, edgeB, edgeC;

    bool isFace = false;
    if ((position1.x() != MINF) && (position2.x() != MINF) && (position3.x() != MINF))
    {
        edgeA = calculateEdge(position1, position2);
        edgeB = calculateEdge(position3, position2);
        edgeC = calculateEdge(position1, position3);

        if ((edgeA <= edgeThreshold) && (edgeB <= edgeThreshold) && (edgeC <= edgeThreshold))
        {
            isFace = true;
        }

    }

    return isFace;


}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
    float edgeThreshold = 0.01f; // 1cm

    // 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
    // - have a look at the "off_sample.off" file to see how to store the vertices and triangles
    // - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
    // - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
    // - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
    // - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
    // - only write triangles with valid vertices and an edge length smaller then edgeThreshold

    // Get number of vertices
    unsigned int nVertices = 0;
    unsigned int i, idx = 0;
    //We declare length for an easier looping
    unsigned int length = width * height;

    for (i = 0; i < length; i++) {
        if (vertices[i].position.x() != MINF) {
            nVertices++;
            //The row on which the vertex will be displayed on is idx, we store this value to be able to display correctly
            indices[i] = idx;
            idx++;
        }
        else indices[i] = -1;

    }
    //std::cout << "Vertices :" << nVertices;

    // Get number of faces
    unsigned nFaces = 0;
    std::vector<Vector3f> faceIndices;
    for (i = 0; i < length - width - 1; i++) { // Changed the upper limit for width-1 not to overflow
        if (i%width == 0)
            i += 1;
        if (vertices[i].position.x() != MINF) {
            //Trianglulation
            if (calculateFace(edgeThreshold, vertices[i].position, vertices[i + 1].position, vertices[i + width].position))
            {

                nFaces++;
                Vector3f newFace;
                newFace.x() = indices[i + width]; newFace.y() = indices[i + 1]; newFace.z() = indices[i];
                faceIndices.push_back(newFace);

            }
            //The below code for other side of the triangle Triangle  
            if (calculateFace(edgeThreshold, vertices[i + 1].position, vertices[i + width + 1].position, vertices[i + width].position))
            {
                nFaces++;
                Vector3f newFace;
                newFace.x() = indices[i + width]; newFace.y() = indices[i + width + 1]; newFace.z() = indices[i + 1];

                faceIndices.push_back(newFace);
            }
        }
    }




    // Write off file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    // write header
    outFile << "COFF" << std::endl;
    outFile << "# numVertices numFaces numEdges" << std::endl;
    outFile << nVertices << " " << nFaces << " 0" << std::endl;
    outFile << "# X Y Z R G B A" << std::endl;

    // save vertices
    for (i = 0; i < length; i++) {
        if (vertices[i].position.x() != MINF) {
            //X Y Z 
            outFile << vertices[i].position.x() << " " << vertices[i].position.y() << " " << vertices[i].position.z();
            //R G B A
            outFile << " " << static_cast<unsigned>(vertices[i].color.x()) << " " << static_cast<unsigned>(vertices[i].color.y()) << " " << static_cast<unsigned>(vertices[i].color.z()) << " " << static_cast<unsigned>(vertices[i].color.w()) << std::endl;
        }
    }

    // save faces
    outFile << "# list of faces" << std::endl;
    outFile << "# nVerticesPerFace idx0 idx1 idx2" << std::endl;
    for (i = 0; i < nFaces; i++) {
        //nVerticesPerFace is always 3 as we only have triangles
        outFile << 3 << " " << faceIndices[i].x() << " " << faceIndices[i].y() << " " << faceIndices[i].z() << std::endl;
    }

    // close file
    outFile.close();

    return true;
}

int main()
{
    std::string filenameIn = "./data/rgbd_dataset_freiburg1_xyz/";
    std::string filenameBaseOut = "mesh_";

    // load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.Init(filenameIn))
    {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }

    // convert video to meshes
    while (sensor.ProcessNextFrame())
    {
        // get ptr to the current depth frame
        // depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
        float* depthMap = sensor.GetDepth();
        // get ptr to the current color frame
        // color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
        BYTE* colorMap = sensor.GetColorRGBX();

        // get depth intrinsics
        Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
        float fovX = depthIntrinsics(0, 0);
        float fovY = depthIntrinsics(1, 1);
        float cX = depthIntrinsics(0, 2);
        float cY = depthIntrinsics(1, 2);

        // compute inverse depth extrinsics
        Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

        Matrix4f trajectory = sensor.GetTrajectory();
        Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

        // 1: back-projection
        // write result to the vertices array below, keep pixel ordering!
        // if the depth value at idx is invalid (MINF) write the following values to the vertices array
        // vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
        // vertices[idx].color = Vector4uc(0,0,0,0);
        // otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
        Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

        float X, Y, Z;
        unsigned int idx, u, v;
        for (v = 0; v < sensor.GetDepthImageHeight(); v++) {
            for (u = 0; u < sensor.GetDepthImageWidth(); u++) {
                idx = v * sensor.GetDepthImageWidth() + u;
                Z = depthMap[idx];
                if (Z != MINF) {
                    //Change from scene coordinates to camera coordinates
                    X = (u - cX) * Z / fovX;
                    Y = (v - cY) * Z / fovY;
                    //Changed to world coordinates and w
                    vertices[idx].position = trajectoryInv * Vector4f(X, Y, Z, 1.0);
                    vertices[idx].color = Vector4uc(colorMap[4 * idx], colorMap[4 * idx + 1], colorMap[4 * idx + 2], colorMap[4 * idx + 3]);
                }
                else {
                    vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
                    vertices[idx].color = Vector4uc(0, 0, 0, 0);
                }
            }
        }
        // write mesh file
        std::stringstream ss;
        ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
        if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
        {
            std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
            return -1;
        }

        // free mem
        delete[] vertices;
    }

    return 0;
}
