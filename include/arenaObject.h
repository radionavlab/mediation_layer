#include <Eigen/Geometry>

//Note: a single "object" represents a face of an object
class arenaObject{
     public:

        arenaObject(int i1);
        arenaObject();
        void setVertex(int index, double xloc, double yloc, double zloc);
        void initializeSize(int i1);
        int returnSize();
        Eigen::MatrixXd returnPointMatrix();

    private:
        int numVertices;
        Eigen::MatrixXd vertexList;
};
