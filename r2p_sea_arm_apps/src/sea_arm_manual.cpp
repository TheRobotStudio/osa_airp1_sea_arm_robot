/*
 * Copyright (c) 2016, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Oct 3, 2016
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
//ROS messages
#include <razor_imu_9dof/RazorImu.h>
#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Bool.h>
//ROS services
#include "bibot_apps/switchNode.h"
#include "bibot_apps/getSlaveCmdArray.h"
//Others
#include <boost/foreach.hpp>
#include <flann/flann.hpp> //used for the kdtree search
#include <stdio.h>
//ROS packages include 
#include "r2p_sea_arm_apps/robotDefines.h"

/*** Defines ***/
#define LOOP_RATE			HEART_BEAT
#define SPACE_DIM  			3  //roll pitch yaw
#define REF_DATA_DIM 			1  //only one reference point in kd space to find its nearest neighboor.
#define NUM_NN 				5  //number of nearest neighboors
#define DOF_DIM 			NUMBER_MOTORS_ARM
#define BICEPS_INC			32 //divide the kdtree

//MIN MAX
#define MIN_POS_BICEPS			6000
#define MIN_POS_SUPRA			0
#define MIN_POS_SUBSCAP			0
#define MIN_POS_INFRA			0
#define MIN_POS_TMIN			0
#define MIN_POS_LATDELT			0
#define MIN_POS_ANTDELT			0
#define MIN_POS_POSTDELT		0
#define MIN_POS_TRICEPS			37000
#define MIN_POS_BRACHI			18000
#define MIN_POS_HAND			10000
#define MIN_POS_ROTATOR			0 //-12000
#define MIN_POS_WRIST_IN		66000 //66000
#define MIN_POS_WRIST_UP		50000 //-146000
#define MIN_POS_WRIST_DOWN		40000 //-141000
#define MIN_POS_THUMB			30000 //70000

#define MAX_POS_BICEPS			78700
#define MAX_POS_SUPRA			0
#define MAX_POS_SUBSCAP			0
#define MAX_POS_INFRA			0
#define MAX_POS_TMIN			0
#define MAX_POS_LATDELT			0
#define MAX_POS_ANTDELT			0
#define MAX_POS_POSTDELT		0
#define MAX_POS_TRICEPS			73000
#define MAX_POS_BRACHI			40000
#define MAX_POS_HAND			60000
#define MAX_POS_ROTATOR			24000 //12000
#define MAX_POS_WRIST_IN		145000 //145000
#define MAX_POS_WRIST_UP		146000 //146000 //-50000
#define MAX_POS_WRIST_DOWN		190000 //141000 //-40000
#define MAX_POS_THUMB			125000 //180000

using namespace flann;

/*** Variables ***/
bool switch_node = false; //disable by default
osa_msgs::MotorCmdMultiArray motorCmd_ma;
//razor_imu_9dof::RazorImu razorImu;
//razor_imu_9dof::RazorImu playerImuRef;
sensor_msgs::Joy xboxJoy;

int selectedKdtree = 0;
int dataset_dim = 0; //this will depend on the size of the bag, number of lines of data

//right arm
//Matrix<float> dataset_R_angles; //matrix for the angles
Matrix<int> dataset_R_positions; //matrix for the positions
Matrix<float> query_R(new float[SPACE_DIM*REF_DATA_DIM], REF_DATA_DIM, SPACE_DIM); //matrix for the query, just one line
Matrix<int> indices_R(new int[query_R.rows*SPACE_DIM], query_R.rows, NUM_NN);
Matrix<float> dists_R(new float[query_R.rows*SPACE_DIM], query_R.rows, NUM_NN);

float handVal = 0;
float rotatorVal = 0;
float elbowVal = 0;

//bool imu_arrived = false;
bool joy_arrived = false;
bool velTest = false;

//bool enableImu = false;

//debug function to display a matrix
void displayMatrixFloat(const Matrix<float> matrix, char* name)
{
	std::cout << name << " matrix :" << std::endl;

	int count = 0;
	for(int i=0; i<matrix.rows; i++)
	{
		std::cout << count << " | ";

		for(int j=0; j<matrix.cols; j++)
		{
			std::cout << matrix.ptr()[matrix.cols*i+j] << "\t";
		}

		count += 1;
		std::cerr << std::endl;
	}
}

void displayMatrixInt(const Matrix<int> matrix, char* name)
{
	std::cout << name << " matrix :" << std::endl;

	int count = 0;
	for(int i=0; i<matrix.rows; i++)
	{
		std::cout << count << " | ";

		for(int j=0; j<matrix.cols; j++)
		{
			std::cout << matrix.ptr()[matrix.cols*i+j] << "\t";
		}

		count += 1;
		std::cerr << std::endl;
	}
}


int rotVal = 0;


/*** Callback functions ***/
/*
void imuRaw_cb(const razor_imu_9dof::RazorImuConstPtr& imu)
{
	razorImu = *imu;
	imu_arrived = true;
}
*/

void joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xboxJoy = *joy;
	joy_arrived = true;
}

/*** Services ***/
bool switchNode(bibot_apps::switchNode::Request  &req, bibot_apps::switchNode::Response &res)
{
	//ROS_INFO("switch node");
	switch_node = req.state;
	return true;
}

bool getMotorCmd_ma(bibot_apps::getSlaveCmdArray::Request  &req, bibot_apps::getSlaveCmdArray::Response &res)
{
	//ROS_INFO("cmd srv");

	//send the motorCmdSet set by the callback function motorDataSet_cb
	if(switch_node)
	{
		res.motorCmdMultiArray = motorCmd_ma;
		return true;
	}
	else
	{
		return false;
	}
}

/*** Main ***/
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "osa_armManual_server_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);

	//Subscribers
	//ros::Subscriber sub_imuRaw = nh.subscribe ("/imuRaw", 10, imuRaw_cb);
	ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joy_cb);

	//Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_right_arm_manual_srv", switchNode);
	ros::ServiceServer srv_getMotorCmd_ma = nh.advertiseService("get_right_arm_manual_cmd_srv", getMotorCmd_ma);

	//various kdtrees drives the shoulder and biceps
	//Initialization of the kd tree
	//ROS_INFO("Initialization of the kd-tree :");

	//read a bag to generate the data matrix
	rosbag::Bag bag(ros::package::getPath("bibot_apps") + "/bag/arm/imuRawToShoulder_1.bag"); //change this bag
	//rosbag::View view_joint(bag, rosbag::TopicQuery("/imuRaw")); //angle info
	rosbag::View view_posture(bag, rosbag::TopicQuery("/motor_data_array")); //motor data position info

	int line = 0;
	//create the dataset_angle matrix
	dataset_dim = view_posture.size(); //set the dataset dim equal to the number of lines in the bag file
/*	Matrix<float> tempR1(new float[SPACE_DIM*dataset_dim], dataset_dim, SPACE_DIM);
	dataset_R_angles = tempR1;

	//ROS_INFO("dataset_dim = %d", dataset_dim);
	ROS_INFO("Create the angles matrix");

	BOOST_FOREACH(rosbag::MessageInstance const m, view_joint)
	{
		razor_imu_9dof::RazorImu::Ptr i = m.instantiate<razor_imu_9dof::RazorImu>();

		if(i != NULL)
		{
			dataset_R_angles.ptr()[dataset_R_angles.cols*line+0] = i->roll;
			dataset_R_angles.ptr()[dataset_R_angles.cols*line+1] = i->pitch;
			dataset_R_angles.ptr()[dataset_R_angles.cols*line+2] = i->yaw;
		}
		else
			std::cout << "null" << std::endl;

	    	line++;
	}
*/
	ROS_INFO("Create posture matrix");
	//create the dataset_positions matrix
	int dataset_post_dim = view_posture.size(); //set the dataset dim equal to the number of lines in the bag file
	Matrix<int> tempR2(new int[DOF_DIM*dataset_post_dim], dataset_post_dim, DOF_DIM);
	dataset_R_positions = tempR2;

	line = 0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view_posture) //error compiles ok
	{
		osa_msgs::MotorDataMultiArray::Ptr i = m.instantiate<osa_msgs::MotorDataMultiArray>();

		if(i != NULL)
		{
			//Build the dataset_positions matrix
			for(int j=0; j<DOF_DIM; j++)
			{
				dataset_R_positions.ptr()[dataset_R_positions.cols*line+j] = i->motor_data[j].position;
			}
		}
		else
			std::cout << "null" << std::endl;

		line++;
	}

	bag.close();

	displayMatrixInt(dataset_R_positions, "postures");

	//create the commands multi array
	motorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motorCmd_ma.layout.dim[0].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motorCmd_ma.layout.dim[0].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motorCmd_ma.layout.dim[0].label = "motors";
	motorCmd_ma.layout.data_offset = 0;
	motorCmd_ma.motor_cmd.clear();
	motorCmd_ma.motor_cmd.resize(NUMBER_MAX_EPOS2_PER_SLAVE);
	
	for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		//motorCmd_ma.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
		motorCmd_ma.motor_cmd[i].node_id = i+1;
		motorCmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motorCmd_ma.motor_cmd[i].value = 0;
	}

	//speed of sequences
	int speedA = 1;
	int speedB = 1;
	int speedX = 2;
	int speedY = 1;

	float resetYAW = 0;

	//line to play in the database
	int data_R_idx = 42; //start somewhere in the database, line number 42

	bool crossTop = false;
	bool crossBottom = false;

	while(ros::ok())
	{	
		ros::spinOnce();

		if(switch_node)
		{

			if(joy_arrived)
			{
					
				//if((xboxJoy.buttons[4]==1) && (xboxJoy.buttons[5]==0)) //btn LB to enable manual control
				//{
/*
					if(!enableImu) //each time the player reactivate the manual control, it's YAW position is subsctracted
					{
						resetYAW = razorImu.yaw;
						ROS_INFO("resetYAW = %f", resetYAW);
					}

					enableImu = true; //then set to true so the previous if is no longer appied when the player keep pressing
*/
					//ROS_INFO("move shoulder");
					//move shoulder
/*
					if(query_R.cols == SPACE_DIM)
					{
						query_R.ptr()[0] = razorImu.roll;
						query_R.ptr()[1] = razorImu.pitch;
						query_R.ptr()[2] = razorImu.yaw-resetYAW;
					}

					//ROS_INFO("%f %f %f", razorImu.roll, razorImu.pitch, razorImu.yaw-resetYAW);

					//ROS_INFO("construct a randomized kd-tree index using 4 kd-trees");
					// construct a randomized kd-tree index using 4 kd-trees
					Index<L2<float> > data_R_index(dataset_R_angles, flann::KDTreeIndexParams(SPACE_DIM));
					data_R_index.buildIndex();

					//ROS_INFO("do a knn search, using 128 checks");
					// do a knn search, using 128 checks
					data_R_index.knnSearch(query_R, indices_R, dists_R, SPACE_DIM, flann::SearchParams(128));

					int data_R_idx = indices_R.ptr()[0];
*/
					//skip the IMU here and use the cross buttons on the xBox controller					
					
					if((!crossTop) && (xboxJoy.buttons[13]==1)) 
					{
						data_R_idx -= 10;
						crossTop = true;
					}

					if(crossTop && (xboxJoy.buttons[13]==0)) 
					{
						crossTop = false;
					}

					if((!crossBottom) && (xboxJoy.buttons[14]==1)) 
					{
						data_R_idx += 10;
						crossBottom = true;
					}

					if(crossBottom && (xboxJoy.buttons[14]==0)) 
					{
						crossBottom = false;
					}

					//if(crossTop) data_R_idx--;
					//if(crossBottom) data_R_idx++;
					
					//clip
					if(data_R_idx<0) data_R_idx=0;
					if(data_R_idx>=dataset_dim) data_R_idx=dataset_dim-1;

					//ROS_INFO("init of the packet");
					//init of the packet
					for(int i=1; i<DOF_DIM-2; i++) //no biceps, no brachi/triceps
					{
						//R
						//motorCmd_ma.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
						motorCmd_ma.motor_cmd[i].node_id = i+1;
						motorCmd_ma.motor_cmd[i].command = SET_TARGET_POSITION;
						motorCmd_ma.motor_cmd[i].value = dataset_R_positions.ptr()[dataset_R_positions.cols*data_R_idx+i];
					}

					//ROS_INFO("data_R_idx=%d", data_R_idx);

					//Move Hand
					float handVal_f = xboxJoy.axes[5];
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					handVal_f *= -(MAX_POS_HAND-MIN_POS_HAND)/2; // 2 = 1 - (-1)
					handVal_f += MAX_POS_HAND-(MAX_POS_HAND-MIN_POS_HAND)/2;
					int handVal_i = (int)handVal_f;

					//ROS_INFO("handVal_i = %d", handVal_i);

					//clip
					if(handVal_i>MAX_POS_HAND) handVal_i = MAX_POS_HAND;
					if(handVal_i<MIN_POS_HAND) handVal_i = MIN_POS_HAND;

					motorCmd_ma.motor_cmd[0+10].command = SET_TARGET_POSITION;
					motorCmd_ma.motor_cmd[0+10].value = handVal_i;

					//Move Thumb
					float thumbVal_f = xboxJoy.axes[2];
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					thumbVal_f *= -(MAX_POS_THUMB-MIN_POS_THUMB)/2; // 2 = 1 - (-1)
					thumbVal_f += MAX_POS_THUMB-(MAX_POS_THUMB-MIN_POS_THUMB)/2;
					int thumbVal_i = (int)thumbVal_f;

					//ROS_INFO("thumbVal_f = %d", thumbVal_i);

					//clip
					if(thumbVal_i>MAX_POS_THUMB) thumbVal_i = MAX_POS_THUMB;
					if(thumbVal_i<MIN_POS_THUMB) thumbVal_i = MIN_POS_THUMB;

					motorCmd_ma.motor_cmd[5+10].command = SET_TARGET_POSITION;
					motorCmd_ma.motor_cmd[5+10].value = thumbVal_i;

					//Move Rotator
					float rotatorVal_f = xboxJoy.axes[0];
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					rotatorVal_f *= (MAX_POS_ROTATOR-MIN_POS_ROTATOR)/2; // 2 = 1 - (-1)
					rotatorVal_f += MAX_POS_ROTATOR-(MAX_POS_ROTATOR-MIN_POS_ROTATOR)/2;
					int rotatorVal_i = (int)rotatorVal_f;

					//ROS_INFO("rotatorVal_i = %d", rotatorVal_i);

					//clip
					if(rotatorVal_i>MAX_POS_ROTATOR) rotatorVal_i = MAX_POS_ROTATOR;
					if(rotatorVal_i<MIN_POS_ROTATOR) rotatorVal_i = MIN_POS_ROTATOR;

					motorCmd_ma.motor_cmd[1+10].command = SET_TARGET_POSITION;
					motorCmd_ma.motor_cmd[1+10].value = rotatorVal_i;

					//Move Brachi
					float brachiVal_f = xboxJoy.axes[1];
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					brachiVal_f *= -(MAX_POS_BRACHI-MIN_POS_BRACHI)/2; // 2 = 1 - (-1)
					brachiVal_f += MAX_POS_BRACHI-(MAX_POS_BRACHI-MIN_POS_BRACHI)/2;
					int brachiVal_i = (int)brachiVal_f;

					//ROS_INFO("brachiVal_i = %d", brachiVal_i);

					//clip
					if(brachiVal_i>MAX_POS_BRACHI) brachiVal_i = MAX_POS_BRACHI;
					if(brachiVal_i<MIN_POS_BRACHI) brachiVal_i = MIN_POS_BRACHI;

					motorCmd_ma.motor_cmd[9].command = SET_TARGET_POSITION;
					motorCmd_ma.motor_cmd[9].value = brachiVal_i;

					//Move triceps
					float tricepsVal_f = xboxJoy.axes[1];
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					tricepsVal_f *= (MAX_POS_TRICEPS-MIN_POS_TRICEPS)/2; // 2 = 1 - (-1)
					tricepsVal_f += MAX_POS_TRICEPS-(MAX_POS_TRICEPS-MIN_POS_TRICEPS)/2;
					int tricepsVal_i = (int)tricepsVal_f;

					//ROS_INFO("tricepsVal_i = %d", tricepsVal_i);

					//clip
					if(tricepsVal_i>MAX_POS_TRICEPS) tricepsVal_i = MAX_POS_TRICEPS;
					if(tricepsVal_i<MIN_POS_TRICEPS) tricepsVal_i = MIN_POS_TRICEPS;

					motorCmd_ma.motor_cmd[8].command = SET_TARGET_POSITION;
					motorCmd_ma.motor_cmd[8].value = tricepsVal_i;

					//Move Wrist left right
					float wristLRVal_f = xboxJoy.axes[3]; //left right
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					wristLRVal_f *= -(MAX_POS_WRIST_UP-MIN_POS_WRIST_UP)/2; // 2 = 1 - (-1)
					wristLRVal_f += MAX_POS_WRIST_UP-(MAX_POS_WRIST_UP-MIN_POS_WRIST_UP)/2;
					int wristLRVal_i = (int)wristLRVal_f;

					//ROS_INFO("wristLRVal_i = %d", wristLRVal_i);

					//clip
					if(wristLRVal_i>MAX_POS_WRIST_UP) wristLRVal_i = MAX_POS_WRIST_UP;
					if(wristLRVal_i<MIN_POS_WRIST_UP) wristLRVal_i = MIN_POS_WRIST_UP;

					motorCmd_ma.motor_cmd[3+10].command = SET_TARGET_POSITION;
					motorCmd_ma.motor_cmd[3+10].value = wristLRVal_i;

					//Move Wrist up down
					float wristUDVal_f = xboxJoy.axes[4]; //up down
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					wristUDVal_f *= (MAX_POS_WRIST_IN-MIN_POS_WRIST_IN)/2; // 2 = 1 - (-1)
					wristUDVal_f += MAX_POS_WRIST_IN-(MAX_POS_WRIST_IN-MIN_POS_WRIST_IN)/2;
					int wristUDVal_i = (int)wristUDVal_f;

					//ROS_INFO("wristUDVal_i = %d", wristUDVal_i);

					//clip
					if(wristUDVal_i>MAX_POS_WRIST_IN) wristUDVal_i = MAX_POS_WRIST_IN;
					if(wristUDVal_i<MIN_POS_WRIST_IN) wristUDVal_i = MIN_POS_WRIST_IN;

					motorCmd_ma.motor_cmd[2+10].command = SET_TARGET_POSITION;
					motorCmd_ma.motor_cmd[2+10].value = wristUDVal_i;

					//Move Wrist left right
					float wristUDVal2_f = xboxJoy.axes[4]; //up down
					//remap value from [-1;1] to [MIN_POS;MAX_POS]
					wristUDVal2_f *= (MAX_POS_WRIST_DOWN-MIN_POS_WRIST_DOWN)/2; // 2 = 1 - (-1)
					wristUDVal2_f += MAX_POS_WRIST_DOWN-(MAX_POS_WRIST_DOWN-MIN_POS_WRIST_DOWN)/2;
					int wristUDVal2_i = (int)wristUDVal2_f;

					//ROS_INFO("wristUDVal_i = %d", wristUDVal_i);

					//clip
					if(wristUDVal2_i>MAX_POS_WRIST_DOWN) wristUDVal2_i = MAX_POS_WRIST_DOWN;
					if(wristUDVal2_i<MIN_POS_WRIST_DOWN) wristUDVal2_i = MIN_POS_WRIST_DOWN;

					motorCmd_ma.motor_cmd[4+10].command = SET_TARGET_POSITION;
					motorCmd_ma.motor_cmd[4+10].value = wristUDVal2_i;

					//by default apply current on biceps
					motorCmd_ma.motor_cmd[0].command = SET_CURRENT_MODE_SETTING_VALUE;
					motorCmd_ma.motor_cmd[0].value = 250;

/*
					if(xboxJoy.buttons[0]) //A
					{
						//ROS_INFO("butA_pressed");

						ros::Rate r1(LOOP_RATE);

						for(int i=0; i<btnA_positions.rows/speedA; i++)// /2
						{
							//ROS_INFO("for each rows");

							for(int j=0; j<NUMBER_MOTORS_ARM; j++)
							{
								rightArmCmd_ma.motor_cmd[j].node_id = j+1;
								rightArmCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightArmCmd_ma.motor_cmd[j].value = btnA_positions.ptr()[btnA_positions.cols*i*speedA+j]; //i*2

								//ROS_INFO("val = %d", rightArmCmd_ma.motor_cmd[j].value);
							}

							for(int j=0; j<NUMBER_MOTORS_HAND; j++)
							{
								rightHandCmd_ma.motor_cmd[j].node_id = j+1;
								rightHandCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightHandCmd_ma.motor_cmd[j].value = btnA_positions.ptr()[btnA_positions.cols*i*speedA+j+NUMBER_MOTORS_ARM]; // u*2
							}

							pub_motorHandCmdMultiArray.publish(rightHandCmd_ma);
							pub_motorArmCmdMultiArray.publish(rightArmCmd_ma);

							r1.sleep();
						}
					}

					if(xboxJoy.buttons[1]) //B
					{
						ros::Rate r1(LOOP_RATE);

						for(int i=0; i<btnB_positions.rows/speedB; i++)// /2
						{
							//ROS_INFO("for each rows");

							for(int j=0; j<NUMBER_MOTORS_ARM; j++)
							{
								rightArmCmd_ma.motor_cmd[j].node_id = j+1;
								rightArmCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightArmCmd_ma.motor_cmd[j].value = btnB_positions.ptr()[btnB_positions.cols*i*speedB+j]; //i*2

								//ROS_INFO("val = %d", rightArmCmd_ma.motor_cmd[j].value);
							}

							for(int j=0; j<NUMBER_MOTORS_HAND; j++)
							{
								rightHandCmd_ma.motor_cmd[j].node_id = j+1;
								rightHandCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightHandCmd_ma.motor_cmd[j].value = btnB_positions.ptr()[btnB_positions.cols*i*speedB+j+NUMBER_MOTORS_ARM]; // u*2
							}

							pub_motorHandCmdMultiArray.publish(rightHandCmd_ma);
							pub_motorArmCmdMultiArray.publish(rightArmCmd_ma);

							r1.sleep();
						}

						//butB_pressed = false;
					}

					if(xboxJoy.buttons[2]) //X
					{
						ros::Rate r1(LOOP_RATE);

						for(int i=0; i<btnX_positions.rows/speedX; i++)// /2
						{
							//ROS_INFO("for each rows");

							for(int j=0; j<NUMBER_MOTORS_ARM; j++)
							{
								rightArmCmd_ma.motor_cmd[j].node_id = j+1;
								rightArmCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightArmCmd_ma.motor_cmd[j].value = btnX_positions.ptr()[btnX_positions.cols*i*speedX+j]; //i*2

								//ROS_INFO("val = %d", rightArmCmd_ma.motor_cmd[j].value);
							}

							for(int j=0; j<NUMBER_MOTORS_HAND; j++)
							{
								rightHandCmd_ma.motor_cmd[j].node_id = j+1;
								rightHandCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightHandCmd_ma.motor_cmd[j].value = btnX_positions.ptr()[btnX_positions.cols*i*speedX+j+NUMBER_MOTORS_ARM]; // u*2
							}

							pub_motorHandCmdMultiArray.publish(rightHandCmd_ma);
							pub_motorArmCmdMultiArray.publish(rightArmCmd_ma);

							r1.sleep();
						}

						//butX_pressed = false;
					}

					if(xboxJoy.buttons[3]) //Y
					{
						ros::Rate r1(LOOP_RATE);

						for(int i=0; i<btnY_positions.rows/speedY; i++)// /2
						{
							//ROS_INFO("for each rows");

							for(int j=0; j<NUMBER_MOTORS_ARM; j++)
							{
								rightArmCmd_ma.motor_cmd[j].node_id = j+1;
								rightArmCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightArmCmd_ma.motor_cmd[j].value = btnY_positions.ptr()[btnY_positions.cols*i*speedY+j]; //i*2

								//ROS_INFO("val = %d", rightArmCmd_ma.motor_cmd[j].value);
							}

							for(int j=0; j<NUMBER_MOTORS_HAND; j++)
							{
								rightHandCmd_ma.motor_cmd[j].node_id = j+1;
								rightHandCmd_ma.motor_cmd[j].command = SET_TARGET_POSITION;
								rightHandCmd_ma.motor_cmd[j].value = btnY_positions.ptr()[btnY_positions.cols*i*speedY+j+NUMBER_MOTORS_ARM]; // u*2
							}

							pub_motorHandCmdMultiArray.publish(rightHandCmd_ma);
							pub_motorArmCmdMultiArray.publish(rightArmCmd_ma);

							r1.sleep();
						}

						//butY_pressed = false;
					}
*/

				//}
				//else
				//{
				//	enableImu = false;
				//}
/*
				if((xboxJoy.buttons[4]==0) && (xboxJoy.buttons[5]==1)) //btn RB to enable homing and current
				{
					if(xboxJoy.buttons[6]) //back button : zero homing mode
					{
						for(int i=0; i<NUMBER_MOTORS_ARM; i++)
						{
							rightArmCmd_ma.motor_cmd[i].node_id = i+1;
							rightArmCmd_ma.motor_cmd[i].command = SET_TARGET_POSITION;
							rightArmCmd_ma.motor_cmd[i].value = 0;
						}

						for(int i=0; i<NUMBER_MOTORS_HAND; i++)
						{
							rightHandCmd_ma.motor_cmd[i].node_id = i+1;
							rightHandCmd_ma.motor_cmd[i].command = SET_TARGET_POSITION;
							rightHandCmd_ma.motor_cmd[i].value = 0;
						}					
					}

					if(xboxJoy.buttons[7]) //start button : current mode
					{
						int curr = 120; //150;

						for(int i=0; i<NUMBER_MOTORS_ARM; i++)
						{
							rightArmCmd_ma.motor_cmd[i].node_id = i+1;
							rightArmCmd_ma.motor_cmd[i].command = SET_CURRENT_MODE_SETTING_VALUE;
							rightArmCmd_ma.motor_cmd[i].value = curr;
						}

						rightArmCmd_ma.motor_cmd[0].value = 180; // 250;
						rightArmCmd_ma.motor_cmd[1].value = 180; // 250;
						rightArmCmd_ma.motor_cmd[9].value = 180; // 250;

						for(int i=0; i<NUMBER_MOTORS_HAND; i++)
						{
							rightHandCmd_ma.motor_cmd[i].node_id = i+1;
							rightHandCmd_ma.motor_cmd[i].command = SET_CURRENT_MODE_SETTING_VALUE;
							rightHandCmd_ma.motor_cmd[i].value = curr;
						}

						//for the 2 inverted wrist motors
						//rightHandCmd_ma.motor_cmd[1].command = SET_TARGET_POSITION;
						rightHandCmd_ma.motor_cmd[1].value = 0;

						rightHandCmd_ma.motor_cmd[2].value = 60; //80;
						rightHandCmd_ma.motor_cmd[3].value = 60; //80;
						rightHandCmd_ma.motor_cmd[4].value = 60; //80;
						rightHandCmd_ma.motor_cmd[5].value = 50;
					}
				}*/	
			}
			else
			{
				//ROS_INFO("no joy");
			}

			//pub_motorHandCmdMultiArray.publish(rightHandCmd_ma);
			//pub_motorArmCmdMultiArray.publish(rightArmCmd_ma);		

			//joy_arrived = false;
		}//if(switch_node)

		//r.sleep();
	}

	//free matrix pointers from memory
	//delete[] dataset_R_angles.ptr();
	delete[] dataset_R_positions.ptr();
/*
	delete[] btnA_positions.ptr();
	delete[] btnB_positions.ptr();
	delete[] btnX_positions.ptr();
	delete[] btnY_positions.ptr();
*/
	delete[] query_R.ptr();
	delete[] indices_R.ptr();
	delete[] dists_R.ptr();

	return 0;
}
