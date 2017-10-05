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
#include <arenaObject.h>
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
    void readPLYfile(std::string filename);
    std::string readHeaderLine(std::string &thisLine);
    std::string readVertexLine(std::string &thisLine);
    std::string readFaceLine(std::string &thisLine);


private:

    /**
    * Reads ROS parameters from server into various class variables
    */
    void readROSParameters(); 

	int numQuads, numVertices, numFaces;
    double k_forcing;
    std::string quadPoseTopics[10], quadPVAListenTopics[10], quadPVAPublishTopics[10];
    ros::Subscriber  pose_sub_[10], pva_sub_[10];
    ros::Publisher pva_pub_[10];
    quad quadArray[10];
    Eigen::MatrixXd arenaObjectFaces[100], faceAreas, vertexMat;
    //NOTE: IF RESIZING arenaObjectFaces, change the FOR loop limit in readPLYfile
};

