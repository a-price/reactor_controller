/**
 * \file dynamixel_monitor.cpp
 * \brief Handles Dynamixel motor feedback
 *
 * \author Andrew Price
 * \date April 6, 2014
 *
 * \copyright
 *
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/MotorStateList.h>


ros::Publisher jsPub;
ros::Subscriber dmxSub;

XmlRpc::XmlRpcValue motorXml;
XmlRpc::XmlRpcValue controllerXml;
std::map<int, std::string> motorIDtoName;


double ticksToRadians(int ticks, XmlRpc::XmlRpcValue& xml)
{
	return (double)(ticks - (int&)(xml["encoder_resolution"])/2) * (double&)(xml["radians_per_encoder_tick"]);
}

// Convert Dynamixel positions to URDF-compatible angles
void dmxCallback(const dynamixel_msgs::MotorStateListPtr& dmx)
{
	sensor_msgs::JointState js;

	for (dynamixel_msgs::MotorState motorState : dmx->motor_states)
	{
		std::map<int, std::string>::iterator entry = motorIDtoName.find(motorState.id);
		if (motorIDtoName.end() == entry) { continue; }

		std::string jointName = entry->second;

		double angle = ticksToRadians(motorState.position, motorXml[std::to_string(motorState.id)]);

		// Reverse any joints that are backwards
		if (controllerXml[jointName+"_controller"].hasMember("reverse"))
		{
			angle *= -1;
			if ((bool&)(controllerXml[jointName+"_controller"]["reverse"]))
			{
				//angle *= -1;
			}
		}

		// The listed offset is from URDF->DMX, and we want to go the other direction, hence the '-'
		angle -= (double&)(controllerXml[jointName+"_controller"]["offset"]);

		js.name.push_back(jointName);
		js.position.push_back(angle);
	}

	js.header.stamp = ros::Time::now();
	js.header.frame_id = "/base_link";

	jsPub.publish(js);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dynamixel_to_joint_state");
	ros::NodeHandle nh;

	ros::Rate r(0.2);
	r.sleep();
	
	// Grab and Parse URDF Model
	boost::shared_ptr<urdf::Model> robotModel(new urdf::Model);
	std::string robotDescription;
	if (!nh.getParam("robot_description", robotDescription))
	{
		ROS_FATAL("Parameter for robot description not provided");
	}
	
	if (!robotModel->initString(robotDescription))
	{
		ROS_FATAL("Unable to parse URDF model.");
	}

	if (!nh.getParam("dynamixel_controllers", controllerXml))
	{
		ROS_FATAL("Parameter for controller description not provided");
	}

	// Create map from ID to name
	for (XmlRpc::XmlRpcValue::iterator iter = controllerXml.begin();
		 iter != controllerXml.end(); ++iter)
	{
		XmlRpc::XmlRpcValue controller = iter->second;
		if (controller.hasMember("motor"))
		{
			motorIDtoName[(int&)(controller["motor"]["id"])] = (std::string&)(controller["joint_name"]);
		}
		else if (controller.hasMember("motor_master"))
		{
			motorIDtoName[(int&)(controller["motor_master"]["id"])] = (std::string&)(controller["joint_name"]);
		}
	}


	if (!nh.getParam("motor_descriptions", motorXml))
	{
		ROS_FATAL("Parameter for motor description not provided");
	}
	
	jsPub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	dmxSub = nh.subscribe("motor_states", 1, &dmxCallback);
	
	ros::spin();
	return 0;
}


