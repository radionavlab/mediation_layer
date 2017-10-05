#include <readPLYfile.h>

Eigen::MatrixXd readPLYfile(std::string filename)
{
	int numFaces numVertices;
	

	//extract object size before making objects


	Eigen::MatrixXd vertices;
	Eigen::MatrixXd objectFaces[numFaces+1];  //last entry contains area of each object
	vertices.resize(3,numVertices);
	objectFaces[numFaces].resize(1,numFaces);


	for(int i=0;i<numVertices;i++)
	{

	}
	for(int i=0;i<numFaces;i++)
	{
		int numPtsThisLine;

		numPtsThisLine = 9001;
		objectFaces[i].resize(3,numPtsThisLine);
	}

	return objectFaces;

}


std::string readHeaderLine(std::string thisLine)
{

}


std::string readVertexLine(std::string thisLine)
{


}



std::string readFaceLine(std::string thisLine)
{


}