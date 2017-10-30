#pragma once

//this #define may interrupt ROS compilation
//#define _GLIBCXX_USE_CXX11_ABI 0 //necessary because of std::__cxx11::string error

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Geometry>
#include <string>
#include <iostream>
#include <px4_control/PVA.h>
//#include <arenaObject.h>
#include <fstream>

class quad
{
    public:

        quad(){ pose.setZero(); vel.setZero();}

        //getters
        Eigen::Vector3d getPose() const{ return pose;}
        Eigen::Vector3d getVel() const{return vel;}
        const int getName() const { return quadnum; }

        //setters
        void setPose(const Eigen::Vector3d &state) {pose=state;}
        void setVel(const Eigen::Vector3d &state) {vel=state;}
        void setName(const int val) {quadnum=val;}

    private:
        int quadnum;
        Eigen::Vector3d pose, vel;
};

class mediationLayer
{
public:

    /**
    * Constructor.
    */
	mediationLayer(ros::NodeHandle &nh);

	void poseCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event);
	void pvaCallback(const ros::MessageEvent<px4_control::PVA const>& event);
    Eigen::Vector3d unitVector(const Eigen::Vector3d);
    void generateObjectMatFromFile(std::string filename);
    int indexOfMatchingString(const std::string (&stringmat)[10], const int listlen, const std::string &matchstring);
    Eigen::MatrixXd rotatePolygonTo2D(Eigen::MatrixXd &inmat);
    double polygonArea3d(Eigen::MatrixXd &inmat);
    bool readPLYfile(std::string filename);
    double area3D_Polygon( int n, Eigen::MatrixXd &pointmat,Eigen::Vector3d &vecNormal);
    double distSquaredToLine(Eigen::Vector3d &p,Eigen::Vector3d &v1, Eigen::Vector3d &v2);

private:

    /**
    * Reads ROS parameters from server into various class variables
    */
    void readROSParameters(); 

	int numQuads, numVertices, numFaces;
    double k_forcing, sizeThresh, minDistThresh;
    std::string quadPoseTopics[10], quadPVAListenTopics[10], quadPVAPublishTopics[10], inFile;
    ros::Subscriber pose_sub_[10], pva_sub_[10];
    ros::Publisher pva_pub_[10];
    quad quadArray[10];
    Eigen::Vector3d zeroCenter;
    Eigen::VectorXd faceAreas;
    Eigen::VectorXi indexToUseInCalculation;
    Eigen::MatrixXd objectFaces[100], faceCenter, vertexDist;
    Eigen::MatrixXd vertexMat; //vertex positions in 3D
    //NOTE: IF RESIZING arenaObjectFaces, change the FOR loop limit in readPLYfile
};

