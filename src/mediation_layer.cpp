#include "mediation_layer.hpp"

mediationLayer::mediationLayer(ros::NodeHandle &nh)
{

	//create all subscribers/publishers
	for(int i=0; i<numQuads; i++)
	{
		pose_sub_[i] = nh.subscribe(quadPoseTopics[i],10,&mediationLayer::poseCallback, this, ros::TransportHints().tcpNoDelay());
		pva_sub_[i] = nh.subscribe(quadPVAListenTopics[i],10,&mediationLayer::pvaCallback, this, ros::TransportHints().tcpNoDelay());
		pva_pub_[i] = nh.advertise<px4_control::PVA>(quadPVAPublishTopics[i], 10);
	}

//	//get initial pose
//	ROS_INFO("Waiting for first position measurement...");
//	initPose_ = ros::topic::waitForMessage<nav_msgs::Odometry>(quadPoseTopic);
//	ROS_INFO("Initial position: %f\t%f\t%f", initPose_->pose.pose.position.x, initPose_->pose.pose.position.y, initPose_->pose.pose.position.z);
//	next_wpt(0)=initPose_->pose.pose.position.x;
//	next_wpt(1)=initPose_->pose.pose.position.y;
//	next_wpt(2)=initPose_->pose.pose.position.z+1; //1m above initial pose
}

void mediationLayer::readROSParameters() 
{
    // Topic names
	ros::param::get("mediation_layer/numquads", numQuads);
	ros::param::get("mediation_layer/kforce_quad", k_forcing);
	ros::param::get("mediation_layer/kforce_object",k_forcing_object);
	ros::param::get("mediation_layer/sizeThresh",sizeThresh);
	for(int i=0; i<numQuads; i++)
	{
		ros::param::get("mediation_layer/quadPoseTopic_"+std::to_string(i+1),quadPoseTopics[i]);
		ros::param::get("mediation_layer/quadPVAListenTopic_"+std::to_string(i+1),quadPVAListenTopics[i]);
		ros::param::get("mediation_layer/quadPVAPublishTopic_"+std::to_string(i+1),quadPVAPublishTopics[i]);
		quadArray[i].setName(i);
	}

}

void mediationLayer::poseCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event)
{ 
	//extract message constents
	const nav_msgs::Odometry::ConstPtr& msg = event.getMessage();

	//get topic name in callback by using messageevent syntax
	std::string publisher_name = event.getPublisherName();

	//get quad's name
	int thisQuadNum = indexOfMatchingString(quadPoseTopics,numQuads,publisher_name);

	//update quad params
	Eigen::Vector3d tmp;
	tmp(0)=msg->pose.pose.position.x;
	tmp(1)=msg->pose.pose.position.y;
	tmp(2)=msg->pose.pose.position.z;
	quadArray[thisQuadNum].setPose(tmp);
	tmp(0)=msg->twist.twist.linear.x;
	tmp(1)=msg->twist.twist.linear.y;
	tmp(2)=msg->twist.twist.linear.z;
	quadArray[thisQuadNum].setVel(tmp);	

}

void mediationLayer::pvaCallback(const ros::MessageEvent<px4_control::PVA const>& event)
{ 
	//extract message contents
	const px4_control::PVA::ConstPtr& msg = event.getMessage();

	//get topic name in callback by using messageevent syntax
	std::string publisher_name = event.getPublisherName();

	//get quad's name
	int thisQuadNum = indexOfMatchingString(quadPoseTopics,numQuads,publisher_name);

	//update quad params
	Eigen::Vector3d thisQuadPose, quad2Pose;
	thisQuadPose=quadArray[thisQuadNum].getPose();

	//call ML
	Eigen::Vector3d netForcing;
	netForcing.setZero();
	for(int i=0; i<numQuads; i++)
	{
		if(i!=thisQuadNum)
		{
			//uses inverse square forcing law
			quad2Pose=quadArray[i].getPose();
			netForcing = netForcing + k_forcing*unitVector(thisQuadPose-quad2Pose) / ((thisQuadPose-quad2Pose).squaredNorm()+.01);
		}
	}

	//fill new message by passing on this message with changes
	px4_control::PVA PVA_Ref_msg;
	PVA_Ref_msg.yaw = msg->yaw;
	PVA_Ref_msg.Pos.x = msg->Pos.x;
	PVA_Ref_msg.Pos.y = msg->Pos.y;
	PVA_Ref_msg.Pos.z = msg->Pos.z;
	PVA_Ref_msg.Vel.x = msg->Vel.x;
	PVA_Ref_msg.Vel.y = msg->Vel.y;
	PVA_Ref_msg.Vel.z = msg->Vel.z;
	PVA_Ref_msg.Acc.x = msg->Acc.x + netForcing(0);
	PVA_Ref_msg.Acc.y = msg->Acc.y + netForcing(1);
	PVA_Ref_msg.Acc.z = msg->Acc.z + netForcing(2);

	pva_pub_[thisQuadNum].publish(PVA_Ref_msg);

}

int mediationLayer::indexOfMatchingString(const std::string (&stringmat)[10], const int listlen, const std::string &matchstring)
{
	//returns last index of matching string up to listlen.  Returns -1 if failed.
	int ij=-1;
	for(int i=0;i<listlen;i++)
	{
		if(strcmp( matchstring.c_str() , (stringmat[i]).c_str() )==0)
		{
			ij=i;
		}
	}
	return ij;
}


Eigen::Vector3d mediationLayer::unitVector(const Eigen::Vector3d v1)
{
	return v1/v1.norm();
}


/*This function reads the PLY file and generates objects.  The main generation
and handling is done in readPLYfile.  This function exists to handle errors and printing.*/
void mediationLayer::generateObjectMatFromFile(std::string filename)
{

	if(!filename.empty())
	{
		readPLYfile(filename);
	}

}



/*
//example usage of arenaObject class
arenaObject m1[2];
m1[1].initializeSize(2);
m1[1].setVertex(0,.1,.1,.1); //note: each vertex must be passed individually due to pass-by-reference problems in Eigen
m1[1].setVertex(1,.2,.2,.2);
Eigen::MatrixXd m2(2,3);
m2=m1[1].returnPointMatrix();
*/

//returns false on error
bool mediationLayer::readPLYfile(std::string filename)
{
	if(!filename.empty());
	{
		std::ifstream infile(filename);
		//Read header
		bool contvar=true;
		int whilecounter, posIndex, numPtsThisLine, vertexIndex;
		whilecounter=0; 
		std::string thisline, firstword, secondword, tmp;

		//read header
		while(contvar)
		{
			whilecounter++; //don't loop through the whole file
			std::getline(infile, thisline);

			firstword = thisline.substr(0,thisline.find(" "));
			if(firstword.compare("end_header") && headerflag==0) //header START
			{
				contvar=false;
			}else if(firstword.compare("element")) //pick out vertex length vs face length
			{
				secondword=thisline.substr(firstword.length(),thisline,find(" "));
				if(secondword.compare("vertex"))
				{
					tmp=thisline.substr(thisline.find("vertex")+7,thisline.size());
					numVertices=stoi(tmp);

				}else if(secondword.compare("face"))
				{
					tmp=thisline.substr(thisline.find("face")+5,thisline.size());
					numFaces=stoi(tmp);
				}

			}else if(firstword.compare("format")) //throw error if endian format
			{
				secondword=thisline.substr(firstword.length(),thisline,find(" "));
				if(!firstword.compare("ascii"))
				{
					return false;
				}
			}

			if(whilecounter>=100)
			{
				contvar=false;
			}
		}


		numFaces=10;
		int numVertices_thisface=5;
		arenaObjectFaces[1].resize(3,numVertices_thisface);

		//read in each vertex and ignore color information
		for(int i=0;i<numVertices;i++)
		{
			std::getline(infile, thisline);
			posIndex=thisline.find(" ");
			tmp = thisline.substr(0,posIndex);
			vertexMat(0,i)=stod(tmp);
			posIndex=thisline.find(" ",posIndex+1);
			tmp = thisline.substr(posIndex,thisline.find(" ",posIndex+1));
			vertexMat(1,i)=stod(tmp);
			posIndex=thisline.find(" ",posIndex+1);
			tmp = thisline.substr(posIndex,thisline.find(" ",posIndex+1));
			vertexMat(2,i)=stod(tmp);
		}

		//for each face, do whatever
		for(int i=0;(i<numFaces && i<100);i++)
		{
			std::getline(infile, thisline);
			posIndex = thisline.find(" ");
			tmp = thisline.substr(0,posIndex);
			numPtsThisLine = stoi(tmp); //first element in a row determines how many vertices appear on that row
			objectFaces[i].resize(3,numPtsThisLine);


			//iterate through object. Do not do last point in case find fails at end of line
			for(int j=0; j<numPtsThisLine-1; j++) 
			{
				posIndex = thisline.find(" ",posIndex+1);
				tmp = thisline.substr(posIndex,thisline.find(" ",posIndex+1));
				vertexIndex = stoi(tmp);
				objectFaces[i](0,j) = vertexMat(0,vertexIndex);
				objectFaces[i](1,j) = vertexMat(1,vertexIndex);
				objectFaces[i](2,j) = vertexMat(2,vertexIndex);
			}
			//handle last index
			posIndex = thisline.find(" ",posIndex+1);
			lastindex=thisline.find(" ",posIndex+1);
			if(lastindex=std::string::npos)
			{
				tmp = thisline.substr(posIndex,thisline.length());
			}else
			{
				tmp = thisline.substr(posIndex,lastindex);
			}
			vertexIndex = stoi(tmp);
			objectFaces[i](0,j) = vertexMat(0,vertexIndex);
			objectFaces[i](1,j) = vertexMat(1,vertexIndex);
			objectFaces[i](2,j) = vertexMat(2,vertexIndex);
		}

	}

	return true;

}
