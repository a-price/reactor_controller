/**
 * \file twist_controller.cpp
 * \brief Provides workspace control for the reactor arm.
 *   Uses MoveIt to parse the URDF and create the Jacobian matrix.
 *   Designed for use with a SpaceNavigator joystick.
 *
 * \author Andrew Price
 * \date October 7, 2016
 *
 * \copyright
 * MIT License
 * 
 * Copyright (c) 2016 Andrew Price
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
**/

#include <ros/ros.h>
#include <urdf/model.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <Eigen/SVD>

const std::string ARM_GROUP_NAME = "arm";
const std::string GRIPPER_GROUP_NAME = "gripper";
const std::string INIT_STATE_NAME = "ready";

robot_model::RobotModelPtr pModel;
robot_model::JointModelGroup* pArmGroup;
robot_model::JointModelGroup* pGripperGroup; // NB: Memory managed by RobotModel

ros::Publisher jsPub, jtPub;

sensor_msgs::Joy joy;
robot_state::RobotStatePtr pSetpoint;


// Controller Settings
bool useDominantAxis = true;
bool useLocalCoordinates = true;
bool limitJointVelocities = true;

double deltaT = 0.1;
double linScale = 0.015;
double rotScale = 0.05;

double deadband = 0.0025;

namespace Eigen
{
/**
 * @brief Vector6d Defines fixed-size Eigen vector for e.g. twists
 */
typedef Matrix<double, 6, 1> Vector6d;

/**
 * @brief pseudoInverse Performs the Moore-Penrose pseudo-inverse
 * @param a The matrix to invert
 * @param result The inverted matrix
 * @param epsilon Threshold below which to zero-out singular values
 * @return
 */
template<typename _Matrix_Type_>
bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double
				   epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
{
	bool doTranspose = false;
	_Matrix_Type_ a_oriented;
	if(a.rows()<a.cols())
	{
		doTranspose = true;
		a_oriented = a.transpose();
	}
	else
	{
		a_oriented = a;
	}

	Eigen::JacobiSVD< _Matrix_Type_ > svd = a_oriented.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

	typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a_oriented.cols(),
																  a_oriented.rows()) * svd.singularValues().array().abs().maxCoeff();

	result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() >
											 tolerance).select(svd.singularValues().
															   array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();
	if (doTranspose)
	{
		result.transposeInPlace();
	}
	return true;
}
}

// Callback for Buttons Only
void gripper_cb(const sensor_msgs::JoyConstPtr pJoy)
{
	joy = *pJoy;
}

// Send state as command
void command_setpoint(const robot_state::RobotState& state)
{
	// Publication variables
	sensor_msgs::JointState js;
	trajectory_msgs::JointTrajectory jt;
	trajectory_msgs::JointTrajectoryPoint jtp;

	const std::vector<const robot_model::JointModel*>& mods = state.getRobotModel()->getSingleDOFJointModels();

	// Iterate through all joints and copy positions and velocities
	for (size_t i = 0; i < mods.size(); ++i)
	{
		if (mods[i]->getMimic()) { continue; }

		const std::string& name = mods[i]->getName();
		const size_t index = mods[i]->getFirstVariableIndex();
		const double pos = state.getVariablePosition(index);

		js.name.push_back(name);
		jt.joint_names.push_back(name);

		js.position.push_back(pos);
		jtp.positions.push_back(pos);

		if (state.hasVelocities())
		{
			const double vel = state.getVariableVelocity(index);
			js.velocity.push_back(vel);
			jtp.velocities.push_back(vel);
		}

		jtp.time_from_start = ros::Duration(deltaT);
	}

	// Check for inconsistent number of velocities
	if (js.velocity.size() != js.position.size())
	{
		js.velocity.clear();
		jtp.velocities.clear();
	}

	// Add trajectory point to trajectory
	jt.points.push_back(jtp);

	js.header.stamp = ros::Time::now();
	js.header.frame_id = state.getRobotModel()->getModelFrame();
	jsPub.publish(js);

	jt.header.stamp = ros::Time::now();
	jt.header.frame_id = state.getRobotModel()->getModelFrame();
	jtPub.publish(jt);
}

////////////////////////////////////////////////////////////////////
// Main Twist Callback
////////////////////////////////////////////////////////////////////
void twist_cb(const geometry_msgs::TwistConstPtr& twist)
{
	// Copy to Eigen
	Eigen::Vector6d xDot;
	xDot << twist->linear.x * linScale,
			twist->linear.y * linScale,
			twist->linear.z * linScale,
			twist->angular.x * rotScale,
			twist->angular.y * rotScale,
			twist->angular.z * rotScale;

	// Ignore tiny motions (deadband)
	double totalMotion = xDot.array().abs().sum();
	for (size_t i = 0; i < joy.buttons.size(); ++i) { totalMotion += (joy.buttons[i] == 0) ? 0 : deadband; }
	if (totalMotion < deadband)
	{
		command_setpoint(*pSetpoint);
		return;
	}

	// Limit movement to the primary commanded direction.
	// This helps limit drift in unintended directions.
	if (useDominantAxis)
	{
		size_t dominantAxis = 0;
		float max = 0;
		for (size_t i = 0; i < 6; i++)
		{
			if (fabs(xDot[i]) > max)
			{
				max = fabs(xDot[i]);
				dominantAxis = i;
			}
		}

		for (size_t i = 0; i < 6; i++)
		{
			if (i != dominantAxis)
			{
				xDot[i] = 0.0;
			}
		}
	}

	// Get the Jacobian point
	Eigen::Affine3d toolFrame = pSetpoint->getFrameTransform(pArmGroup->getLinkModelNames().back());

	// Control in local coordinates
	if (useLocalCoordinates)
	{
		Eigen::Matrix3d R = toolFrame.rotation();
		xDot.head<3>() = R * xDot.head<3>();
		xDot.tail<3>() = R * xDot.tail<3>();
	}

	// Invert the Jacobian
	Eigen::MatrixXd J = pSetpoint->getJacobian(pArmGroup, Eigen::Vector3d(0.088882,0,0));
	Eigen::MatrixXd Jinv;
	pseudoInverse(J, Jinv);

	// Compute the control signal
	Eigen::VectorXd thetaDot = Jinv * xDot;

	if (limitJointVelocities)
	{
		double largestRatio = 0;
		for (size_t i = 0; i < pArmGroup->getJointModels().size(); ++i)
		{
			const robot_model::JointModel* j = pArmGroup->getJointModels()[i];
			const robot_model::VariableBounds& vb = j->getVariableBounds().front();
			if (vb.velocity_bounded_)
			{
				double ratio = thetaDot[i] / vb.max_velocity_;
				largestRatio = std::max(ratio, largestRatio);
			}
		}

		if (largestRatio > 1)
		{
			thetaDot /= largestRatio;
		}
	}

	Eigen::VectorXd theta;
	pSetpoint->copyJointGroupPositions(pArmGroup, theta);
	theta += thetaDot*deltaT;

	// Add in last buttons for gripper velocity
	double g = 0;
	if (joy.buttons.size() >= 2)
	{
		float gripperVelocity = 0.0025;
		if (joy.buttons[0] != 0)
		{
			g = gripperVelocity;
		}
		else if (joy.buttons[1] != 0)
		{
			g = -gripperVelocity;
		}
		else
		{
			g = 0;
		}
	}
	else
	{
		g = 0;
	}

	Eigen::VectorXd gTheta;
	pSetpoint->copyJointGroupPositions(pGripperGroup, gTheta);
	gTheta[0] += g*deltaT;
	std::cerr << gTheta << std::endl;

	// Assign to target state
	pSetpoint->setJointGroupPositions(pArmGroup, theta);
	pSetpoint->setJointGroupPositions(pGripperGroup, gTheta);

	pSetpoint->enforceBounds();


	command_setpoint(*pSetpoint);
}

std::ostream& operator<<(std::ostream& os, const std::vector<std::string>& v)
{
	os << "{ ";
	for (const std::string s : v) { os << "'" << s << "' "; }
	os << "}";
	return os;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "twist_controller");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param("use_dominant_axis", useDominantAxis, useDominantAxis);
	pnh.param("use_local_coordinates", useLocalCoordinates, useLocalCoordinates);
	pnh.param("limit_joint_velocities", limitJointVelocities, limitJointVelocities);
	pnh.param("dt", deltaT, deltaT);
	pnh.param("linear_scale", linScale, linScale);
	pnh.param("angular_scale", rotScale, rotScale);
	pnh.param("deadband", deadband, deadband);

	std::unique_ptr<robot_model_loader::RobotModelLoader> pLoader(new robot_model_loader::RobotModelLoader());
	pModel = pLoader->getModel();

	const std::vector<std::string>& groupNames = pModel->getJointModelGroupNames();
	if (std::find(groupNames.begin(), groupNames.end(), ARM_GROUP_NAME) == groupNames.end())
	{
		ROS_FATAL_STREAM("Expected group name '"<<ARM_GROUP_NAME<<"' in SRDF. Found groups: " << groupNames);
		return -1;
	}
	pArmGroup = pModel->getJointModelGroup(ARM_GROUP_NAME);

	if (std::find(groupNames.begin(), groupNames.end(), GRIPPER_GROUP_NAME) == groupNames.end())
	{
		ROS_FATAL_STREAM("Expected group name '"<<GRIPPER_GROUP_NAME<<"' in SRDF. Found groups: " << groupNames);
		return -1;
	}
	pGripperGroup = pModel->getJointModelGroup(GRIPPER_GROUP_NAME);

	const std::vector<std::string>& stateNames = pArmGroup->getDefaultStateNames();
	if (std::find(stateNames.begin(), stateNames.end(), INIT_STATE_NAME) == stateNames.end())
	{
		ROS_FATAL_STREAM("Expected state name '"<<INIT_STATE_NAME<<"' in SRDF. Found states: " << stateNames);
		return -1;
	}

	pSetpoint.reset(new robot_state::RobotState(pModel));
	pSetpoint->setToDefaultValues();

	std::map<std::string, double> readyPose;
	pArmGroup->getVariableDefaultPositions(INIT_STATE_NAME, readyPose);
	for (const std::pair<std::string, double>& p : readyPose)
	{
		pSetpoint->setVariablePosition(p.first, p.second);
	}
	command_setpoint(*pSetpoint);

	jsPub = nh.advertise<sensor_msgs::JointState>("/joint_setpoints", 1, true);
	jtPub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1, true);

	ros::Subscriber twistSub = nh.subscribe("/spacenav/twist", 1, twist_cb);
	ros::Subscriber joySub = nh.subscribe("/spacenav/joy", 1, gripper_cb);

	ros::spin();

	return 0;
}
