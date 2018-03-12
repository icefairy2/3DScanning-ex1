#include <iostream>
#include <fstream>

#include "Eigen.h"

#include "VirtualSensor.h"

int indices[640 * 480];

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// position stored as 4 floats (4th component is supposed to be 1.0)
		Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

float calculateEdge(Vector4f position1, Vector4f position2) {
	float edge = 10.0f; // to assign a value big enough to compare
	float edgeA, edgeB, edgeC;

	edgeA = position1.x() - position2.x();
	edgeB = position1.y() - position2.y();
	edgeC = position1.z() - position2.z();
	edge = sqrt(pow(edgeA, 2.0) + pow(edgeB, 2.0) + pow(edgeC, 2.0));

	return edge;


}


bool calculateFace(float edgeThreshold, Vector4f position1, Vector4f position2, Vector4f position3) {
	//float edgeA = 10.0f, edgeB = 10.0f, edgeC = 10.0f; //
	float edgeA, edgeB, edgeC ; //

	bool isFace = false;
	if( (position1.x() != MINF )&&( position2.x() != MINF )&&( position3.x() != MINF))
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

								 // TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
								 // - have a look at the "off_sample.off" file to see how to store the vertices and triangles
								 // - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
								 // - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
								 // - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
								 // - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
								 // - only write triangles with valid vertices and an edge length smaller then edgeThreshold

								 // TODO: Get number of vertices
	unsigned int nVertices = 0;
	int i, idx = 0;
	int length = width * height;

	for (i = 0; i < length; i++) {
		if (vertices[i].position.x() != MINF) {
			nVertices++;
			indices[i] = idx;
			idx++;
		}
		else indices[i] = -1;

	}
	std::cout << "Vertices :" << nVertices;

	// TODO: Get number of faces
	unsigned nFaces = 0;
	std::vector<Vector3f> faceIndices;
	for (i = 0; i < length - width - 1; i++) { // Changed the upper limit for width-1 not to overflow
		if (i%width == 0)
			i += 1;
		if (vertices[i].position.x() != MINF) {
			if (calculateFace(edgeThreshold, vertices[i].position, vertices[i + 1].position, vertices[i + width].position))
			{

				nFaces++;
				Vector3f newFace;
				//newFace.x() = indices[i]; newFace.y() = indices[i + 1]; newFace.z() = indices[i + width];
				newFace.x() = indices[i + width]; newFace.y() = indices[i + 1]; newFace.z() = indices[i ];
				faceIndices.push_back(newFace);

			}
			//The below code for other side of the triangle 
			if (calculateFace(edgeThreshold, vertices[i + 1].position, vertices[i + width + 1].position, vertices[i + width].position))
			{
				nFaces++;
				Vector3f newFace;
				//newFace.x() = indices[i + 1]; newFace.y() = indices[i + width + 1]; newFace.z() = indices[i + width];
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
	outFile << "# numVertices numFaces numEdges" << std::endl; // Do we have to write edges? 
	outFile << nVertices << " " << nFaces << " 0" << std::endl;
	outFile << "# X Y Z R G B A" << std::endl;
	// TODO: save vertices
	for (int i = 0; i < length; i++) {
		if (vertices[i].position.x() != MINF) {
			outFile << vertices[i].position.x() << " " << vertices[i].position.y() << " " << vertices[i].position.z();
			outFile << " " << static_cast<unsigned>(vertices[i].color.x()) << " " << static_cast<unsigned>(vertices[i].color.y()) << " " << static_cast<unsigned>(vertices[i].color.z()) << " " << static_cast<unsigned>(vertices[i].color.w()) << std::endl;
		}
	}




	// TODO: save faces
	outFile << "# list of faces" << std::endl;
	outFile << "# nVerticesPerFace idx0 idx1 idx2" << std::endl;
	outFile << "# nFaces : " << nFaces <<std::endl;
	outFile << "# faceIndices.size() : " << faceIndices.size() << std::endl;
	for (i = 0; i < nFaces; i++) { // nFaces must be equal to faceIndices.size() 



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

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		//int length = sensor.GetDepthImageHeight() * sensor.GetDepthImageWidth();
		double X, Y, Z;
		int idx;
		for (int v = 0; v < sensor.GetDepthImageHeight(); v++) {
			for (int u = 0; u < sensor.GetDepthImageWidth(); u++) {
				idx = v * sensor.GetDepthImageWidth() + u;
				Z = depthMap[idx];
				if (Z != MINF) {
					X = (u - cX) * Z / fovX;
					Y = (v - cY) * Z / fovY;
					vertices[idx].position = trajectoryInv * Vector4f(X, Y, Z, 1.0) ;
					vertices[idx].color = Vector4uc(colorMap[4 * idx], colorMap[4 * idx + 1], colorMap[4 * idx + 2], colorMap[4 * idx + 3]);
				}
				else {
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);
				}
				//std::cout << depthMap[idx] << std::endl;// << "  " << vertices[idx].position.y << "  " << vertices[idx].position.z << std::endl;
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
