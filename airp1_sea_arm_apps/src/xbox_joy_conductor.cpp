/*
 * Copyright (c) 2019, The Robot Studio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 *  Created on: Oct 2, 2016
 *      Author: Cyril Jourdan
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
#include "airp1_sea_arm_apps/switchNode.h"
#include "airp1_sea_arm_apps/getSlaveCmdArray.h"
//other
//#include <stdio.h>
//ROS packages include 
#include "airp1_sea_arm_apps/robotDefines.h"

/*** Defines ***/
#define LOOP_RATE	HEART_BEAT //50
#define NB_DOF 5

/*** Variables ***/
bool joy_arrived = false;
sensor_msgs::Joy xboxJoy;
osa_msgs::MotorCmdMultiArray armMotorCmd_ma;
//osa_msgs::MotorCmdMultiArray headMotorCmd_ma;

//booleans
bool switch_node = false; //disable by default
bool armManual_enabled = false;
bool headFaceTracking_enabled = true;

/*** Callback functions ***/
void joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xboxJoy = *joy;
	joy_arrived = true;
}

/*** Services ***/
bool switchNode(airp1_sea_arm_apps::switchNode::Request  &req, airp1_sea_arm_apps::switchNode::Response &res)
{
	switch_node = req.state;
	return true;
}

/*** Main ***/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "osa_arm_conductor_node");
	ros::NodeHandle nh;

	ros::Rate r(LOOP_RATE);

	//Publishers
	ros::Publisher pub_setRightArmCommand = nh.advertise<osa_msgs::MotorCmdMultiArray>("/sea_arm/motor_cmd_to_filter", 1); //100	
	//ros::Publisher pub_setHeadCommand = nh.advertise<osa_msgs::MotorCmdMultiArray>("/head/motor_cmd_to_filter", 1); //set_mobile_base_cmd

	//Subscribers
	//ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joy_cb);
	
	//Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_xbox_joy_conductor", switchNode);
	ros::ServiceClient srvClt_switchArmManual = nh.serviceClient<airp1_sea_arm_apps::switchNode>("switch_right_arm_manual_srv");
	ros::ServiceClient srvClt_getArmManualCmd = nh.serviceClient<airp1_sea_arm_apps::getSlaveCmdArray>("get_right_arm_manual_cmd_srv");	
	//ros::ServiceClient srvClt_switchHeadFaceTracking = nh.serviceClient<airp1_sea_arm_apps::switchNode>("switch_head_face_tracking_srv");
	//ros::ServiceClient srvClt_getHeadFaceTrackingCmd = nh.serviceClient<airp1_sea_arm_apps::getSlaveCmdArray>("get_head_face_tracking_cmd_srv");
	
	airp1_sea_arm_apps::getSlaveCmdArray srv_getSlaveCmdArrayArmManual;	
	//airp1_sea_arm_apps::getSlaveCmdArray srv_getSlaveCmdArrayHeadFaceTracking;

	airp1_sea_arm_apps::switchNode srv_switchNodeArmManual;
	//airp1_sea_arm_apps::switchNode srv_switchNodeHeadFaceTracking;

	srv_switchNodeArmManual.request.state = true;
	//srv_switchNodeHeadFaceTracking.request.state = true;

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
		//armMotorCmd_ma.motor_cmd[i].slaveBoardID = BIBOT_ARM_SLAVEBOARD_ID;
		armMotorCmd_ma.motor_cmd[i].node_id = i+1;
		armMotorCmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		armMotorCmd_ma.motor_cmd[i].value = 0;
	}
	
	//No conductor above, so switched on by default
	switch_node = true;

	while(ros::ok())
	{
		//Default arm value
		for(int i=0; i<NB_DOF; i++)
		{
			armMotorCmd_ma.motor_cmd[i].node_id = i+1;
			armMotorCmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
			armMotorCmd_ma.motor_cmd[i].value = 0;
		}
		
		//read the joystick inputs
		ros::spinOnce();

		if(switch_node)
		{									
			if(srvClt_getArmManualCmd.call(srv_getSlaveCmdArrayArmManual))
			{
				//publish to the commandBuilder node
				//ROS_INFO("Publishing motor cmds to be filtered");
				pub_setRightArmCommand.publish(srv_getSlaveCmdArrayArmManual.response.motorCmdMultiArray);
			}
			else
			{
				ROS_DEBUG("Failed to call service get_right_arm_manual_cmd");
			}		
		}
		else
		{
			//ROS_DEBUG("Conductor OFF");
		}
		//r.sleep();
	}//while ros ok

	return 0;
}
