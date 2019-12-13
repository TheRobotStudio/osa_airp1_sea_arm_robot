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
#include <razor_imu_9dof/RazorImu.h>
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
#define NB_DOF 9 //4

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
	ros::init (argc, argv, "osa_sea_arm_teleop_server_node");
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
				if((xbox_joy.buttons[4] == 1) && (xbox_joy.buttons[5] == 0)) //left hand
                                {
					targetPosition[5] = ((xbox_joy.axes[4])*300000);
					targetPosition[6] = ((xbox_joy.axes[3])*1200);
					targetPosition[7] = ((xbox_joy.axes[0])*600);
					targetPosition[8] = ((xbox_joy.axes[1])*elbow_max);
					//targetPosition[9] = -((xbox_joy.axes[2])*22000);
	
					//clip	
					if(targetPosition[5] > 300000) targetPosition[5] = 300000; 
					if(targetPosition[5] < 0) targetPosition[5] = 0; 

					if(targetPosition[6] > 1200) targetPosition[6] = 1200; 
					if(targetPosition[6] < 0) targetPosition[6] = 0; 

					if(targetPosition[7] > 600) targetPosition[7] = 600; 
					if(targetPosition[7] < -600) targetPosition[7] = -600; 

					if(targetPosition[8] > elbow_max) targetPosition[8] = elbow_max; 
					if(targetPosition[8] < 0) targetPosition[8] = 0; 

					//if(targetPosition[9] > 22000) targetPosition[9] = 22000; 
					//if(targetPosition[9] < 0) targetPosition[9] = 0; 
				}	

				if((xbox_joy.buttons[4] == 0) && (xbox_joy.buttons[5] == 1)) //right hand
                                {
//					ROS_INFO("hello!");

					targetPosition[0] = ((xbox_joy.axes[4])*100000);
					targetPosition[1] = -((xbox_joy.axes[3])*1000);
					targetPosition[2] = -((xbox_joy.axes[0])*300);
					targetPosition[3] = ((xbox_joy.axes[1])*elbow_max);
					targetPosition[4] = -((xbox_joy.axes[5])*22000);

					//clip	
					if(targetPosition[0] > 100000) targetPosition[0] = 100000; 
					if(targetPosition[0] < 0) targetPosition[0] = 0; 
					
					if(targetPosition[1] > 800) targetPosition[1] = 800; 
					if(targetPosition[1] < 0) targetPosition[1] = 0; 

					if(targetPosition[2] > 600) targetPosition[2] = 600; 
					if(targetPosition[2] < -700) targetPosition[2] = -600; 

					if(targetPosition[3] > elbow_max) targetPosition[3] = elbow_max; 
					if(targetPosition[3] < 0) targetPosition[3] = 0; 

					if(targetPosition[4] > 22000) targetPosition[4] = 22000; 
					if(targetPosition[4] < 0) targetPosition[4] = 0; 


				}	 

				if((xbox_joy.buttons[4] == 0) && (xbox_joy.buttons[5] == 0) && (xbox_joy.buttons[2] == 1)) //both hands
                                {
					targetPosition[5] = ((xbox_joy.axes[4])*300000);
					targetPosition[6] = ((xbox_joy.axes[3])*1200);
					targetPosition[7] = ((xbox_joy.axes[0])*600);
					targetPosition[8] = ((xbox_joy.axes[1])*elbow_max);
					//targetPosition[9] = -((xbox_joy.axes[2])*22000);
	
					//clip	
					if(targetPosition[5] > 300000) targetPosition[5] = 300000; 
					if(targetPosition[5] < 0) targetPosition[5] = 0; 

					if(targetPosition[6] > 1200) targetPosition[6] = 1200; 
					if(targetPosition[6] < 0) targetPosition[6] = 0; 

					if(targetPosition[7] > 600) targetPosition[7] = 600; 
					if(targetPosition[7] < -600) targetPosition[7] = -600; 

					if(targetPosition[8] > elbow_max) targetPosition[8] = elbow_max; 
					if(targetPosition[8] < 0) targetPosition[8] = 0; 

					targetPosition[0] = ((xbox_joy.axes[4])*100000);
					targetPosition[1] = -((xbox_joy.axes[3])*1000);
					targetPosition[2] = -((xbox_joy.axes[0])*300);
					targetPosition[3] = ((xbox_joy.axes[1])*elbow_max);
					targetPosition[4] = -((xbox_joy.axes[5])*22000);

					//clip	
					if(targetPosition[0] > 100000) targetPosition[0] = 100000; 
					if(targetPosition[0] < 0) targetPosition[0] = 0; 
					
					if(targetPosition[1] > 800) targetPosition[1] = 800; 
					if(targetPosition[1] < 0) targetPosition[1] = 0; 

					if(targetPosition[2] > 600) targetPosition[2] = 600; 
					if(targetPosition[2] < -700) targetPosition[2] = -600; 

					if(targetPosition[3] > elbow_max) targetPosition[3] = elbow_max; 
					if(targetPosition[3] < 0) targetPosition[3] = 0; 

					if(targetPosition[4] > 22000) targetPosition[4] = 22000; 
					if(targetPosition[4] < 0) targetPosition[4] = 0; 
				}

				if((xbox_joy.buttons[4] == 0) && (xbox_joy.buttons[5] == 0) && (xbox_joy.buttons[3] == 1)) //both hands symetry
                                {
					targetPosition[5] = ((xbox_joy.axes[4])*100000);
					targetPosition[6] = -((xbox_joy.axes[3])*1000);
					targetPosition[7] = -((xbox_joy.axes[0])*300);
					targetPosition[8] = ((xbox_joy.axes[1])*elbow_max);
					//targetPosition[9] = -((xbox_joy.axes[2])*22000);
	
					//clip	
					if(targetPosition[5] > 100000) targetPosition[5] = 100000; 
					if(targetPosition[5] < 0) targetPosition[5] = 0; 

					if(targetPosition[6] > 800) targetPosition[6] = 800; 
					if(targetPosition[6] < 0) targetPosition[6] = 0; 

					if(targetPosition[7] > 600) targetPosition[7] = 600; 
					if(targetPosition[7] < -600) targetPosition[7] = -600; 

					if(targetPosition[8] > elbow_max) targetPosition[8] = elbow_max; 
					if(targetPosition[8] < 0) targetPosition[8] = 0; 

					targetPosition[0] = ((xbox_joy.axes[4])*100000);
					targetPosition[1] = -((xbox_joy.axes[3])*1000);
					targetPosition[2] = -((xbox_joy.axes[0])*300);
					targetPosition[3] = ((xbox_joy.axes[1])*elbow_max);
					targetPosition[4] = -((xbox_joy.axes[5])*22000);

					//clip	
					if(targetPosition[0] > 100000) targetPosition[0] = 100000; 
					if(targetPosition[0] < 0) targetPosition[0] = 0; 
					
					if(targetPosition[1] > 800) targetPosition[1] = 800; 
					if(targetPosition[1] < 0) targetPosition[1] = 0; 

					if(targetPosition[2] > 600) targetPosition[2] = 600; 
					if(targetPosition[2] < -600) targetPosition[2] = -600; 

					if(targetPosition[3] > elbow_max) targetPosition[3] = elbow_max; 
					if(targetPosition[3] < 0) targetPosition[3] = 0; 

					if(targetPosition[4] > 22000) targetPosition[4] = 22000; 
					if(targetPosition[4] < 0) targetPosition[4] = 0; 
				}
					// *** RIGHT ARM WALDO ***//
					// **********************************Arm position from Waldo readings*******************************************
					//An0: Shoulder Pitch 3200 - 12000
					//An1: Shoulder Yaw 12800 - 17000
					//An2: Humerus Rotation 8000 internal - 5000 external
					//An3: Elbow 6400 straight - 20700 fully bent
					//An4: Forearm Rotation
					//An5: Joy twist middle 16620 29540 cw 2056 ccw
					//An6: Joy pitch middle 14619 64 forwards 32375 backwards
					//An7: Joy Yaw middle 14000 to 14400 hard to use due to forearm rotation
					//Dig0-3: Thumb joy
					//Dig4: Main trigger 1
					//Dig5: Button 2
					//Dig6: Buttton 3
					//Dig7: Button 4
				
//					float targetPosition[NB_DOF] = {0};
/*
					targetPosition[0] = ((xbox_joy.axes[0] - 3200)*100000/(12000-3200));
					targetPosition[1] = ((xbox_joy.axes[1] - 12800)*1000/(17000-12800));
					targetPosition[2] = -((xbox_joy.axes[2])- 6000)*1000/(8000-5000);
					targetPosition[3] = ((xbox_joy.axes[3] - 6400)*2000/(20700-6400));
*/				

					// *** LEFT ARM WALDO ***//
					// **********************************Arm position from Waldo readings*******************************************     
					//An0: Shoulder Pitch  down 27000 up 16000 // 3200 - 12000
					//An1: Shoulder Yaw down 21000 up 17000 // 12800 - 17000
					//An2: Humerus Rotation in 25000 out 28000 // 8000 internal - 5000 external
					//An3: Elbow down 20000 up 8000 // 6400 straight - 20700 fully bent
					//An4: Forearm Rotation
					//An5: Joy twist middle 16620 29540 cw 2056 ccw
					//An6: Joy pitch middle 14619 64 forwards 32375 backwards
					//An7: Joy Yaw middle 14000 to 14400 hard to use due to forearm rotation
					//Dig0-3: Thumb joy
					//Dig4: Main trigger 1
					//Dig5: Button 2
					//Dig6: Buttton 3
					//Dig7: Button 4
				/*	
					float targetPosition[NB_DOF] = {0};
			
					targetPosition[0] = -((xbox_joy.axes[0] - 27000)*10000/(27000-16000));
					targetPosition[1] = -((xbox_joy.axes[1] - 21000)*800/(21000-17000));
					targetPosition[2] = ((xbox_joy.axes[2] -  25000)*1300/(28000-25000)) - 600;
					targetPosition[3] = -((xbox_joy.axes[3] - 20000)*1650/(20000-8000));
	*/

					//if(xbox_joy.buttons[5] == 0) targetPosition[4] = 0; //open hand
					//else targetPosition[4] = 17500; //close hand

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

