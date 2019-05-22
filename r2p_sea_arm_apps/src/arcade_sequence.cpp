/*
 * Copyright (c) 2019, The Robot Studio
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
 *  Created on: May 6, 2019
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
//ROS
#include <ros/ros.h>
//#include <ros/package.h>
//ROS messages
#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/Bool.h>
//ROS services
#include "r2p_sea_arm_apps/switchNode.h"
#include "r2p_sea_arm_apps/getSlaveCmdArray.h"
//other
//#include <stdio.h>
//ROS packages include 
#include "r2p_sea_arm_apps/robotDefines.h"

/*** Defines ***/
#define LOOP_RATE	HEART_BEAT //50
#define NB_DOF 9

/*** Variables ***/
bool joy_arrived = false;
sensor_msgs::Joy xboxJoy;
osa_msgs::MotorCmdMultiArray armMotorCmd_ma;
//osa_msgs::MotorCmdMultiArray headMotorCmd_ma;
int targetPosition[NB_DOF] = {0};
//booleans
bool switch_node = false; //disable by default
bool armManual_enabled = false;

/*** Callback functions ***/
void joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xboxJoy = *joy;
	joy_arrived = true;
}

void pose_up() //up 
{ 
	targetPosition[5] = 300000;  
	targetPosition[6] = 136;  
	targetPosition[7] = -518;  
	targetPosition[8] = 365; 
} 

void pose_red() //red 
{ 
	targetPosition[5] = 40000;  
	targetPosition[6] = 0;  
	targetPosition[7] = -200;  
	targetPosition[8] = -250; 
} 

void pose_green() //green 
{ 
	targetPosition[5] = 30000;  
	targetPosition[6] = 136;  
	targetPosition[7] = -166;  
	targetPosition[8] = -200; 
} 

void pose_blue() //blue 
{ 
	targetPosition[5] = 23050;  
	targetPosition[6] = 141;  
	targetPosition[7] = 72;  
	targetPosition[8] = -200; 
} 

void pose_yellow() //yellow 
{ 
	targetPosition[5] = 30000; 
	targetPosition[6] = 350;  
	targetPosition[7] = 101;  
	targetPosition[8] = -350; 
} 

void pose_neutral() //neutral 
{ 
	targetPosition[5] = 0;  
	targetPosition[6] = 0;  
	targetPosition[7] = 0;  
	targetPosition[8] = 0; 
} 


/*** Main ***/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "osa_arm_sequencer_node");
	ros::NodeHandle nh;

	ros::Rate r(LOOP_RATE);

	//Publishers
	ros::Publisher pub_setRightArmCommand = nh.advertise<osa_msgs::MotorCmdMultiArray>("/sea_arm/motor_cmd_to_filter", 1); //100	

	//Subscribers
	//ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joy_cb);
	
	//create the commands multi array
	armMotorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	armMotorCmd_ma.layout.dim[0].size = NB_DOF;
	armMotorCmd_ma.layout.dim[0].stride = NB_DOF;
	armMotorCmd_ma.layout.dim[0].label = "motors";
	armMotorCmd_ma.layout.data_offset = 0;
	armMotorCmd_ma.motor_cmd.clear();
	armMotorCmd_ma.motor_cmd.resize(NB_DOF);

	for(int i=0; i<NB_DOF; i++)
	{
		armMotorCmd_ma.motor_cmd[i].node_id = i+1;
		armMotorCmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		armMotorCmd_ma.motor_cmd[i].value = 0;
	}

	while(ros::ok())
	{
		ros::spinOnce();
		//publish to the commandFilter node
		pub_setRightArmCommand.publish(armMotorCmd_ma);
	}//while ros ok

	return 0;
}
