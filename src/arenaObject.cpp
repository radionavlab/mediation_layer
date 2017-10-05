#include <Eigen/Geometry>
#include <arenaObject.h>


arenaObject::arenaObject()
{
	numVertices=1;
	vertexList(1,3);
}

arenaObject::arenaObject(int i1)
{
	numVertices=i1;
	vertexList(il,3);
	vertexList.setZero();
}

void arenaObject::initializeSize(int i1)
{
	numVertices=i1;
	vertexList.resize(i1,3);
}

//note: This is horribly inefficient but Eigen has runtime issues with passing by value instead of reference
void arenaObject::setVertex(int index, double xloc, double yloc, double zloc)
{
	vertexList(index,0)=xloc;
	vertexList(index,1)=yloc;
	vertexList(index,2)=zloc;
}

int arenaObject::returnSize()
{
	return numVertices;
}

Eigen::MatrixXd arenaObject::returnPointMatrix()
{
	return vertexList;
}








