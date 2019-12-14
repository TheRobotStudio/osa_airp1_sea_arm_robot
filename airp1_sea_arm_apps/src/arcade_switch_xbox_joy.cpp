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
 */

/*** Includes ***/
//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
//ROS messages
#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Bool.h>
//ROS services
#include "airp1_sea_arm_apps/switchNode.h"
#include "airp1_sea_arm_apps/getSlaveCmdArray.h"
//Others
#include <stdio.h>
//ROS packages include 
#include "airp1_sea_arm_apps/robotDefines.h"

/*** Defines ***/
#define LOOP_RATE			HEART_BEAT
#define NB_DOF 		9 

//MIN MAX
#define MIN_POS_SHOULDER_PITCH		0
#define MIN_POS_SHOULDER_YAW		0
#define MIN_POS_SHOULDER_HUMERUS	0
#define MIN_POS_ELBOW_ELBOW		0
#define MIN_POS_HEAD_PAN		0
#define MIN_POS_HEAD_TILT		0

#define MAX_POS_SHOULDER_PITCH		0
#define MAX_POS_SHOULDER_YAW		0
#define MAX_POS_SHOULDER_HUMERUS	0
#define MAX_POS_ELBOW_ELBOW		0
#define MAX_POS_HEAD_PAN		0
#define MAX_POS_HEAD_TILT		0

/*** Variables ***/
bool switch_node = true; //enable by default
osa_msgs::MotorCmdMultiArray motor_cmd_ma;
sensor_msgs::Joy xbox_joy;

//bool imu_arrived = false;
bool xbox_joy_arrived = false;

/*** Callback functions ***/
void xbox_joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xbox_joy = *joy;
	xbox_joy_arrived = true;
}

/*** Services ***/
bool switchNode(airp1_sea_arm_apps::switchNode::Request  &req, airp1_sea_arm_apps::switchNode::Response &res)
{
	//ROS_INFO("switch node");
	switch_node = req.state;
	return true;
}

bool getmotor_cmd_ma(airp1_sea_arm_apps::getSlaveCmdArray::Request  &req, airp1_sea_arm_apps::getSlaveCmdArray::Response &res)
{
	//ROS_INFO("cmd srv");

	//send the motorCmdSet set by the callback function motorDataSet_cb
	if(switch_node)
	{
		//ROS_INFO("cmd srv");
		res.motorCmdMultiArray = motor_cmd_ma;
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
	ros::init (argc, argv, "osa_sea_arm_arcade_switch_teleop_server_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);

	//Subscribers
	ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, xbox_joy_cb);

	//Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_right_arm_manual_srv", switchNode);
	ros::ServiceServer srv_getmotor_cmd_ma = nh.advertiseService("get_right_arm_manual_cmd_srv", getmotor_cmd_ma);

	//create the commands multi array
	motor_cmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_ma.layout.dim[0].size = NB_DOF;
	motor_cmd_ma.layout.dim[0].stride = NB_DOF;
	motor_cmd_ma.layout.dim[0].label = "motors";
	motor_cmd_ma.layout.data_offset = 0;
	motor_cmd_ma.motor_cmd.clear();
	motor_cmd_ma.motor_cmd.resize(NB_DOF);
	
	for(int i=0; i<NB_DOF; i++)
	{
		motor_cmd_ma.motor_cmd[i].node_id = i+1;
		motor_cmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_ma.motor_cmd[i].value = 0;
	}

	float targetPosition[NB_DOF] = {0};
	int elbow_max = 500;

	while(ros::ok())
	{	
		ros::spinOnce();

		if(switch_node)
		{
			if(xbox_joy_arrived)
			{
				if(xbox_joy.buttons[4] == 1) //left hand
                                {
					if(xbox_joy.buttons[5] == 1) //up
					{
						targetPosition[5] = 300000; 
						targetPosition[6] = 136; 
						targetPosition[7] = -518; 
						targetPosition[8] = 365;
					}

					if(xbox_joy.buttons[1] == 1) //red
					{
						targetPosition[5] = 40000; 
						targetPosition[6] = 0; 
						targetPosition[7] = -200; 
						targetPosition[8] = -250;
					}
					if(xbox_joy.buttons[0] == 1) //green
					{
						targetPosition[5] = 30000; 
						targetPosition[6] = 136; 
						targetPosition[7] = -166; 
						targetPosition[8] = -200;
					}
					if(xbox_joy.buttons[2] == 1) //blue
					{
						targetPosition[5] = 23050; 
						targetPosition[6] = 141; 
						targetPosition[7] = 72; 
						targetPosition[8] = -200;
					}
					if(xbox_joy.buttons[3] == 1) //yellow
					{
						targetPosition[5] = 30000;
						targetPosition[6] = 350; 
						targetPosition[7] = 101; 
						targetPosition[8] = -350;
					}
					if(xbox_joy.buttons[7] == 1) //neutral
					{
						targetPosition[5] = 0; 
						targetPosition[6] = 0; 
						targetPosition[7] = 0; 
						targetPosition[8] = 0;
					}

/*
					//clip	
					if(targetPosition[5] > 300000) targetPosition[5] = 300000; 
					if(targetPosition[5] < 0) targetPosition[5] = 0; 

					if(targetPosition[6] > 1200) targetPosition[6] = 1200; 
					if(targetPosition[6] < 0) targetPosition[6] = 0; 

					if(targetPosition[7] > 600) targetPosition[7] = 600; 
					if(targetPosition[7] < -600) targetPosition[7] = -600; 

					if(targetPosition[8] > elbow_max) targetPosition[8] = elbow_max; 
					if(targetPosition[8] < 0) targetPosition[8] = 0; 
*/
				}	

				//Send motor commands	
				for(int i=0; i<NB_DOF; i++) 
				{
					motor_cmd_ma.motor_cmd[i].node_id = i+1;
					motor_cmd_ma.motor_cmd[i].command = SET_TARGET_POSITION;
					motor_cmd_ma.motor_cmd[i].value = (int)targetPosition[i];
				}

				//ROS_INFO("motor_cmd_ma[][][][]");

				xbox_joy_arrived = false;	
			}
			else
			{
				//ROS_INFO("no joy");
			}
		}//if(switch_node)
		//r.sleep();
	}

	return 0;
}

