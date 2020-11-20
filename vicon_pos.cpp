/*
 * vicon_pos.cpp
 *
 *  Created on: Jan 20, 2011
 *      Author: nlacock
 */

#include "vicon_pos.h"
#include <stdlib.h>

using namespace ViconDataStreamSDK::CPP;
using namespace std;

//TODO: Figure out how to make this work with multicast so that you can have multiple clients
vicon_pos::vicon_pos()
{
	c = new Client;
	std::cout << "Connecting to Vicon...\n";
	while(!c->IsConnected().Connected)
	{
		c->Connect(VICONIP);
	    usleep(VSLEEP);
	}
	c->EnableSegmentData();
	c->EnableMarkerData();
	c->EnableUnlabeledMarkerData();
	c->SetAxisMapping(ViconDataStreamSDK::CPP::Direction::Forward,
			ViconDataStreamSDK::CPP::Direction::Left,
			ViconDataStreamSDK::CPP::Direction::Up );
	c->SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
	c->GetFrame();

	num_subs = c->GetSubjectCount().SubjectCount;
	std::cout << "Found " << num_subs << " subjects.\n";
	subjects = new string[num_subs];
	positions = new float*[num_subs];//[3];
	rpy = new float *[num_subs];//[3];
	rotation_mat = new float *[num_subs];//[3];
	//quats = new float *[num_subs];//[4];

	std::string temp;
	for(i =0; i<num_subs; i++)
	{
		temp = c->GetSubjectName(i).SubjectName;
		std::cout << "Adding Subject: " << temp << std::endl;
		subjects[i] = temp;
		positions[i] = new float[3];
		rpy[i] = new float[3];
		rotation_mat[i] = new float [9];
		//quats[i] = new float[4];
	}
}

vicon_pos::~vicon_pos()
{
/*
	delete c;
	delete subjects;
	delete positions;
	delete grippers;
	delete rpy;
	delete segment;
	delete gripper;
	*/
}

int vicon_pos::find_sub(std::string subjectname)
{
	bool found = false;
	for(i=0; i<num_subs; i++)
	{
		if(subjects[i] == subjectname)
		{
			found = true;
			break;
		}
	}
	if(found)
	{
		return i;
	}
	else
	{
		cout << "Subject " << subjectname << " not found\n";
		exit(1);
		return -1;
	}
}

void vicon_pos::get_subject_rotmatrix(unsigned int index,float (&x)[9])
{
	for(int k=0;k<9;k++)x[k] = rotation_mat[index][k];
}
void vicon_pos::get_subject_rotmatrix(std::string subjectname,float(&x)[9])
{
	get_subject_rotmatrix(find_sub(subjectname), x);
}

void vicon_pos::get_subject_coord(unsigned int index,float& x,float& y,float& z)
{
	x = positions[index][0];
	y = positions[index][1];
	z = positions[index][2];
}
void vicon_pos::get_subject_coord(std::string subjectname,float& x ,float& y,float& z)
{
	get_subject_coord(find_sub(subjectname),x,y,z);
}

void vicon_pos::get_subject_euler_xyz(unsigned int index,float& x,float& y,float& z)
{
	x = rpy[index][0];
	y = rpy[index][1];
	z = rpy[index][2];
}
void vicon_pos::get_subject_euler_xyz(std::string subjectname,float& x,float& y,float& z)
{
	get_subject_euler_xyz(find_sub(subjectname),x,y,z);
}

void vicon_pos::get_coord_unlabeled(unsigned int index,float& x,float& y,float& z)
{
	unlabeled_marker = c->GetUnlabeledMarkerGlobalTranslation(index);
	x = unlabeled_marker.Translation[0];
	y = unlabeled_marker.Translation[1];
	z = unlabeled_marker.Translation[2];
}

void vicon_pos::get_subject_marker(unsigned int index,std::string markname,float& x,float& y,float& z)
{
	temp_marker = c->GetMarkerGlobalTranslation(subjects[index],markname);
	x = temp_marker.Translation[0];
	y = temp_marker.Translation[1];
	z = temp_marker.Translation[2];
}
void vicon_pos::get_subject_marker(std::string subname,std::string markname,float& x,float& y,float&z)
{
	temp_marker = c->GetMarkerGlobalTranslation(subname,markname);
	x = temp_marker.Translation[0];
	y = temp_marker.Translation[1];
	z = temp_marker.Translation[2];
}


void vicon_pos::update()
{
	while(c->GetFrame().Result != Result::Success)
	{
		usleep(VSLEEP);
	}

	for(i=0; i<num_subs; i++)
	{
		//std::cout << subjects[i] << endl;
	    temp_segment = c->GetSegmentGlobalTranslation(subjects[i],subjects[i]);
	    temp_rpy = c->GetSegmentGlobalRotationEulerXYZ(subjects[i],subjects[i]);
 	    temp_rotation_mat = c->GetSegmentGlobalRotationMatrix(subjects[i],subjects[i]);

	    	temp_marker = c->GetMarkerGlobalTranslation(subjects[i],subjects[i]);
	    	positions[i][0] = temp_marker.Translation[0];
	    	positions[i][1] = temp_marker.Translation[1];
	    	positions[i][2] = temp_marker.Translation[2];

	    //std::cout << grippers[i][0] << " " << grippers[i][1] << " " <<grippers[i][2] << std::endl;

	    rpy[i][0] = temp_rpy.Rotation[0];
	    rpy[i][1] = temp_rpy.Rotation[1];
	    rpy[i][2] = temp_rpy.Rotation[2];

	    for(int j=0;j<9;j++)rotation_mat[i][j] = temp_rotation_mat.Rotation[j];
	}
}
