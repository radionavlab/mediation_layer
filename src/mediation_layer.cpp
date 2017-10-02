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
	ros::param::get("mediation_layer/kforce", k_forcing);
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
			netForcing = netForcing + k_forcing*unitVector(thisQuadPose-quad2Pose) / ((thisQuadPose-quad2Pose).squaredNorm());
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



