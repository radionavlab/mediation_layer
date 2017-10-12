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
	ros::param::get("mediation_layer/kforce_point",k_forcing_point);
	ros::param::get("mediation_layer/sizeThresh",sizeThresh);
	ros::param::get("mediation_layer/distThresh",minDistThresh);
	ros::param::get("mediation_layer/zeroX",zeroCenter(0));
	ros::param::get("mediation_layer/zeroY",zeroCenter(1));
	ros::param::get("mediation_layer/zeroZ",zeroCenter(2));
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
	Eigen::MatrixXd thisMat, Am(3,3);

	//update quad params
	Eigen::Vector3d thisQuadPose, quad2Pose, netForcing, tempvec, nn, p0, abc, p0pvec;
	thisQuadPose=quadArray[thisQuadNum].getPose();
	Eigen::VectorXd distvec;
	Eigen::MatrixXd unitVecs;

	//call ML
	Eigen::Vector3d  v1, v2, v3;
	double tt, mindistSquared, productABC, thisDist;
	netForcing.setZero();
	//iterate through quads
	for(int i=0; i<numQuads; i++)
	{
		if(i!=thisQuadNum)
		{
			//uses inverse square forcing law
			quad2Pose=quadArray[i].getPose();
			netForcing = netForcing + k_forcing*unitVector(thisQuadPose-quad2Pose) / ((thisQuadPose-quad2Pose).squaredNorm()+.01);
		}
	}

	//get distance to each vertex in the list; precomputing for efficiency
	for(int i=0; i<numVertices; i++)
	{
		vertexDist(1,i) = pow(thisQuadPose(0)-vertexMat(0,i),2) + pow(thisQuadPose(1)-vertexMat(1,i),2) + pow(thisQuadPose(2)-vertexMat(2,i),2);
	}

	//iterate through objects
	for(int i=0; i<numFaces; i++)
	{
		if(faceAreas(i)<sizeThresh) //treat small objects as point masses
		{
			quad2Pose(0)=faceCenter(0,i); quad2Pose(1)=faceCenter(1,i); quad2Pose(2)=faceCenter(2,i);
			netForcing = netForcing + k_forcing_point*unitVector(thisQuadPose-quad2Pose) / ((thisQuadPose-quad2Pose).squaredNorm()+.01+sizeThresh);
		}else
		{
			distvec.resize(objectFaces[i].cols()-2);
			thisMat.setZero();
			thisMat.resize(3,objectFaces[i].cols());
			thisMat=objectFaces[i];
			//implement 2D method if too complicated
			v1(0)=thisMat(0,0); v1(1)=thisMat(1,0); v1(2)=thisMat(2,0);
			v2(0)=thisMat(0,1); v2(1)=thisMat(1,1); v2(2)=thisMat(2,1);

			for(int j=0; j<thisMat.cols()-2; j++)
			{
				v3(0)=thisMat(0,j+2); v3(1)=thisMat(1,j+2); v3(2)=thisMat(2,j+2);
				nn=unitVector((v2-v1).cross(v3-v1));
				tt=nn.dot(v1)-nn.dot(thisQuadPose);
				p0=thisQuadPose+tt*nn;
				if(tt>=0 && tt<=1)
				{
					distvec(j)=(thisQuadPose-p0).squaredNorm();
				}else
				{  //uses barycentric method for finding nearest point or line segment
					Am << v1,v2,v3;
					abc=Am.inverse()*thisQuadPose;
					productABC=abc(0)*abc(1)*abc(2);
					if(productABC>0)
					{
						if(abc(0)>0)
						{
							thisDist=(thisQuadPose-v1).squaredNorm();
						}else if(abc(1)>0)
						{
							thisDist=(thisQuadPose-v2).squaredNorm();
						}else
						{
							thisDist=(thisQuadPose-v3).squaredNorm();
						}
					}else
					{
						if(abc(0)<0)
						{
							thisDist=distSquaredToLine(thisQuadPose,v1,v2);
						}else if(abc(1)<0)
						{
							thisDist=distSquaredToLine(thisQuadPose,v1,v3);
						}else if(abc(2)<0)
						{
							thisDist=distSquaredToLine(thisQuadPose,v2,v3);
						}
					}
				}

				//completing 
				distvec(j)=thisDist;
				v2=v3;
			}
			mindistSquared=distvec.minCoeff();
			p0pvec = unitVector(thisQuadPose-p0); //can be calculated on last iteration since all points are coplanar
			netForcing = netForcing + k_forcing_object * p0pvec / mindistSquared;

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
		int whilecounter, posIndex, numPtsThisLine, vertexIndex, lastindex;
		double thisObjectDist;
		whilecounter=0; 
		std::string thisline, firstword, secondword, tmp;
		Eigen::MatrixXd tmpmat;
		Eigen::Vector3d unitnormal, v1, v2;

		//read header
		while(contvar)
		{
			whilecounter++; //don't loop through the whole file
			std::getline(infile, thisline);

			firstword = thisline.substr(0,thisline.find(" "));
			if(firstword.compare("end_header")) //header START
			{
				contvar=false;
			}else if(firstword.compare("element")) //pick out vertex length vs face length
			{
				secondword=thisline.substr(firstword.length(),thisline.find(" "));
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
				secondword=thisline.substr(firstword.length(),thisline.find(" "));
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
		faceCenter.resize(3,numFaces);
		vertexDist.resize(1,numVertices);

		/* //example usage DO NOT USE
		numFaces=10;
		int numVertices_thisface=5;
		objectFaces[1].resize(3,numVertices_thisface);
		*/

		//read in each vertex and ignore color information
		for(int i=0;i<numVertices;i++)
		{
			std::getline(infile, thisline);
			posIndex=thisline.find(" ");
			tmp = thisline.substr(0,posIndex);
			vertexMat(0,i)=stod(tmp)+zeroCenter(0);
			posIndex=thisline.find(" ",posIndex+1);
			tmp = thisline.substr(posIndex,thisline.find(" ",posIndex+1));
			vertexMat(1,i)=stod(tmp)+zeroCenter(1);
			
			//handle end of line
			posIndex = thisline.find(" ",posIndex+1);
			lastindex = thisline.find(" ",posIndex+1);
			if(lastindex=std::string::npos)
			{
				tmp = thisline.substr(posIndex,thisline.length());
			}else
			{
				tmp = thisline.substr(posIndex,lastindex);
			}
			vertexMat(2,i)=stod(tmp)+zeroCenter(2);
		}

		//for each face, do whatever
		for(int i=0;(i<numFaces && i<100);i++)
		{
			Eigen::VectorXd objectDistVector;
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
			lastindex = thisline.find(" ",posIndex+1);
			if(lastindex=std::string::npos)
			{
				tmp = thisline.substr(posIndex,thisline.length());
			}else
			{
				tmp = thisline.substr(posIndex,lastindex);
			}
			vertexIndex = stoi(tmp);
			objectFaces[i](0,numPtsThisLine-1) = vertexMat(0,vertexIndex);
			objectFaces[i](1,numPtsThisLine-1) = vertexMat(1,vertexIndex);
			objectFaces[i](2,numPtsThisLine-1) = vertexMat(2,vertexIndex);

			//find polygon areas and face vectors
			tmpmat.setZero();
			tmpmat.resize(3,numPtsThisLine+1);
			tmpmat.leftCols(numPtsThisLine)=objectFaces[i];
			tmpmat(0,numPtsThisLine)=objectFaces[i](0,0);
			tmpmat(1,numPtsThisLine)=objectFaces[i](1,0);
			tmpmat(2,numPtsThisLine)=objectFaces[i](2,0);
			v1(0)=tmpmat(0,1)-tmpmat(0,0); v1(1)=tmpmat(1,1)-tmpmat(1,0); v1(2)=tmpmat(2,1)-tmpmat(2,0);
			v2(0)=tmpmat(0,2)-tmpmat(0,0); v2(1)=tmpmat(1,2)-tmpmat(1,0); v2(2)=tmpmat(2,2)-tmpmat(2,0);
			unitnormal=unitVector(v1.cross(v2));
			faceAreas(i)=area3D_Polygon(numPtsThisLine,tmpmat,unitnormal);
			faceCenter(0,i)=(objectFaces[i].row(0)).mean();   //facevector denotes center of face on small objects
			faceCenter(1,i)=(objectFaces[i].row(1)).mean();
			faceCenter(2,i)=(objectFaces[i].row(2)).mean();

			/*
			//iterate through all objects currently in ML and add this to the index of indices to process
			if(i==0) //always add to next index
			{
				objectDistVector.resize(1,-1);
				indexToUseInCalculation(0)=i;
			}else if(indexToUseInCalculation.size()==2)  //it is initialized at size 2 for the compiler
			{
				indexToUseInCalculation(1)=i;
			}else
			{
				objectDistVector.resize(indexToUseInCalculation.cols(),-1);
				for(int j=0; j<indexToUseInCalculation.size(); j++)
				{
					//compare this object to all other objects in .ply
					objectDistVector(j)=(faceCenter.col(i)-faceCenter.col(j)).squaredNorm();
				}

				//use items from full table that are either (a) large or (b) at the center of a group
				if(faceAreas(i)>sizeThresh ||
						(sqrt(objectDistVector.minCoeff()) > minDistThresh && faceAreas(i)<sizeThresh) )
				{	
					indexToUseInCalculation.resize(indexToUseInCalculation.size()+1,i);
				}
			}
			*/
		}

	}

	return true;
}


//may encouter issues with referencing pointers, need to dereference this
Eigen::MatrixXd mediationLayer::rotatePolygonTo2D(Eigen::MatrixXd &inmat)
{
	int numcols;
	numcols=inmat.cols();

	//vector work to rotate to the origin
	Eigen::Vector3d origin, localx, localy, localz, unitx, unity, unitz, tmpvec;
	origin(0)=inmat(0,0); origin(1)=inmat(1,0); origin(2)=inmat(2,0);
	localx(0)=inmat(0,1)-inmat(0,0); localx(1)=inmat(1,1)-inmat(1,0); localx(2)=inmat(2,1)-inmat(2,0);
	unitx = unitVector(localx);
	tmpvec(0)=inmat(0,2)-inmat(0,0); tmpvec(1)=inmat(1,2)-inmat(1,0); tmpvec(2)=inmat(2,2)-inmat(2,0);
	localz = tmpvec.cross(localx);
	unitz = unitVector(localz);
	unity = unitz.cross(unitx);

	//create transformation matrix
	Eigen::MatrixXd Tmat(4,4);
	Tmat.setZero();
	Tmat(0,0)=unitx(0); Tmat(1,0)=unitx(1); Tmat(2,0)=unitx(2);
	Tmat(0,1)=unity(0); Tmat(1,1)=unity(1); Tmat(2,1)=unity(2);
	Tmat(0,2)=unitz(0); Tmat(1,2)=unitz(1); Tmat(2,2)=unitz(2);
	Tmat(0,3)=origin(0); Tmat(1,3)=origin(1); Tmat(2,3)=origin(2); 
	Tmat(3,3)=1;

	Eigen::MatrixXd Cmat(4,inmat.cols());
	Cmat.setOnes();
	Cmat.topRows(3)=inmat;
	Cmat=(Tmat.inverse()*Cmat.transpose()).transpose();
	
	return Cmat.topRows(3);
}


// http://geomalgorithms.com/a01-_area.html#2D%20Polygons
double mediationLayer::area3D_Polygon( int n, Eigen::MatrixXd &pointmat,Eigen::Vector3d &vecNormal)
{
	//vecNormal is a normal vector to the plane
	//pointmat is a 3 x n+1 matrix of points, with the initial point copied to the last point
    double area = 0;
    float an, ax, ay, az; // abs value of normal and its coords
    int  coord;           // coord to ignore: 1=x, 2=y, 3=z
    int  i, j, k;         // loop indices

    if (n < 3) return 0;  // a degenerate polygon

    // select largest abs coordinate to ignore for projection
    ax = (vecNormal(0)>0 ? vecNormal(0) : -vecNormal(0));    // abs x-coord
    ay = (vecNormal(1)>0 ? vecNormal(1) : -vecNormal(1));    // abs y-coord
    az = (vecNormal(2)>0 ? vecNormal(2) : -vecNormal(2));    // abs z-coord

    coord = 3;                    // ignore z-coord
    if (ax > ay) {
        if (ax > az) coord = 1;   // ignore x-coord
    }
    else if (ay > az) coord = 2;  // ignore y-coord

    // compute area of the 2D projection
    switch (coord) {
      case 1:
        for (i=1, j=2, k=0; i<n; i++, j++, k++)
            area += (pointmat(1,i) * (pointmat(2,j) - pointmat(2,k)));
        break;
      case 2:
        for (i=1, j=2, k=0; i<n; i++, j++, k++)
            area += (pointmat(2,i) * (pointmat(0,j) - pointmat(0,k)));
        break;
      case 3:
        for (i=1, j=2, k=0; i<n; i++, j++, k++)
            area += (pointmat(0,i) * (pointmat(1,j) - pointmat(1,k)));
        break;
    }
    switch (coord) {    // wrap-around term
      case 1:
        area += (pointmat(1,n) * (pointmat(2,1) - pointmat(2,n-1)));
        break;
      case 2:
        area += (pointmat(2,n) * (pointmat(0,1) - pointmat(0,n-1)));
        break;
      case 3:
        area += (pointmat(0,n) * (pointmat(1,1) - pointmat(1,n-1)));
        break;
    }

    // scale to get area before projection
    an = sqrt( ax*ax + ay*ay + az*az); // length of normal vector
    switch (coord) {
      case 1:
        area *= (an / (2 * (vecNormal(0))));
        break;
      case 2:
        area *= (an / (2 * (vecNormal(1))));
        break;
      case 3:
        area *= (an / (2 * (vecNormal(2))));
    }
    return area;
}

double mediationLayer::distSquaredToLine(Eigen::Vector3d &p,Eigen::Vector3d &v1, Eigen::Vector3d &v2)
{
	Eigen::Vector3d M;
	double t0;
	t0 = M.dot((p) - (v1))/M.dot(M);
	M=(v2)-(v1);
	if(t0<=0)
	{
		return ((p)-(v1)).squaredNorm();
	}else if(t0<1)
	{
		return ((p)-((v1)+t0*M)).squaredNorm();
	}else
	{
		return ((p)-((v1)+M)).squaredNorm();
	}
}


