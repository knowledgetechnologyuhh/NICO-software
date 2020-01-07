/**
This program is a server for kinematics requests for the NICO.
**/

// Ros
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <kinematics/IK_request.h>
#include <kinematics/FK_request.h>
#include <kinematics/collision_check.h>
#include <std_msgs/Float32.h>
#include <eigen_conversions/eigen_msg.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection/collision_tools.h>

// C++
#include <iostream>       // cout, endl
#include <fstream>        // read and write files
#include <vector>
#include <algorithm>      // copy
#include <iterator>       // ostream_operator
#include <string>         // stod
#include <ctime>          // clock
#include <Eigen/Geometry> // Eigen
#include <cmath>          // M_PI

#include <typeinfo>

#include <boost/regex.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost;

double radians (double d) {
  return d * M_PI / 180;
}
double degrees (double r) {
  return r * 180/ M_PI;
}

bool isValid(planning_scene::PlanningScene* scene,
	             bool ignore_collisions,
	             robot_state::RobotState* state,
	             const robot_model::JointModelGroup* jmg,
	             const double* joint_positions){

		if(ignore_collisions)
			return true;

		state->setJointGroupPositions(jmg, joint_positions);
		state->update();
		if( scene->isStateColliding(const_cast<const robot_state::RobotState&>(*state), jmg->getName()) ){
			return false;
		}
		return true;
}

namespace kinematics
{
  class KinematicsServer
  {

    private:

    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;

    public:

    KinematicsServer(void){
  	  // The URDF is available on the parameter server as "robot_description"
      robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
      // Build a kinematic model to the URDF
      kinematic_model = robot_model_loader.getModel();
      ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
      // Using the :moveit_core:RobotModel, we can construct a
      // :moveit_core:RobotState that maintains the configuration
      // of the robot. We will set all joints in the state to their
      // default values.
      kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
      kinematic_state->setToDefaultValues();
    }

    bool compute_ik(kinematics::IK_request::Request  &req,
             kinematics::IK_request::Response &res)
    {
      // Receive the current planning scene, which contains information about the environment such as collisions
      planning_scene::PlanningScene planning_scene(kinematic_model);

      // collisions should not be ignored for the inverse kinematic solutions
      bool ignore_collisions = req.ignore_collisions.data;

      const moveit::core::GroupStateValidityCallbackFn is_valid = std::bind(
		      &isValid,
		      &planning_scene,
		      ignore_collisions,
		      std::placeholders::_1,
		      std::placeholders::_2,
          std::placeholders::_3);

      // Get the desired joint group according to the planning group
      const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(req.planning_group.data);

      // joint_names is a vector with all joints names of the used group
      const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

      Eigen::Matrix4d targetMatrix;
      Eigen::Vector3d targetTranslation;
      Eigen::Matrix3d targetRotation;
      Eigen::Vector3d targetEulerAngles;
      Eigen::Isometry3d target;

      bool found_ik;

      // define the translation to use
      Eigen::Isometry3d translation(Eigen::Translation3d(Eigen::Vector3d(req.pose.position.x,req.pose.position.y,req.pose.position.z)));

      // define the rotation matrix
      Eigen::Matrix3d rot3 = Eigen::Quaterniond(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z).toRotationMatrix();
      Eigen::Matrix4d rot4 = Eigen::Matrix4d::Identity();
      rot4.block(0,0,3,3) = rot3;

      // build a affine transformation out of translation and rotation
      Eigen::Matrix4d mat = translation.matrix();
      mat *= rot4.matrix();
      target.matrix() = mat;


      // Print end-effector pose. Remember that this is in the model frame
      targetTranslation = target.translation();
      targetRotation = target.rotation();

      ROS_INFO_STREAM("Target Translation: " << targetTranslation.matrix());
      ROS_INFO_STREAM("Target Rotation: " << targetRotation.matrix());

      // Compute euler angles of target rotation,
      // this will be used to compute the distance to the inverse kinematics solution
      targetEulerAngles = targetRotation.eulerAngles(0, 1, 2);
      targetMatrix = target.matrix();

      // Set the initial joint values as requested
      std::vector<double> joint_values;
      for (unsigned i = 0; i < req.initial_joint_angles.data.size(); ++i){
        joint_values.push_back(req.initial_joint_angles.data[i].data);
      }

      kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
      kinematic_state->update();

      //  Compute the inverse kinematics and measure how long it takes
      clock_t begin = clock();
      found_ik = kinematic_state->setFromIK(joint_model_group,target,req.tip.data,3,0.25, is_valid);
        // * The number of attempts to be made at solving IK: 3
        // * The timeout for each attempt: 0.25 s
      clock_t end = clock();
      double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;

      if (found_ik){
        ROS_INFO_STREAM("Solution found in " << elapsedSecs << " seconds");
        // Copy the retrieved joint values into joint_values variable
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        ROS_INFO("The reached joint angles are");
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
          ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          std_msgs::Float32 joint_value;
          joint_value.data = joint_values[i];
          res.joint_angles.push_back(joint_value);
        }

        // Compute forward kinematics, save the result in reached
        const Eigen::Isometry3d &reached = kinematic_state->getGlobalLinkTransform(req.tip.data);

        Eigen::Vector3d reachedTranslation = reached.translation();
        Eigen::Matrix3d reachedRotation = reached.rotation();
        Eigen::Vector3d reachedEulerAngles = reachedRotation.eulerAngles(0, 1, 2);

        ROS_INFO_STREAM("Reached Translation: " << reachedTranslation);
        ROS_INFO_STREAM("Reached Rotation: " << reachedRotation);

        double distance = (reachedTranslation-targetTranslation).norm();
        double xDif = fabs(reachedEulerAngles[0]-targetEulerAngles[0]);
        double yDif = fabs(reachedEulerAngles[1]-targetEulerAngles[1]);
        double zDif = fabs(reachedEulerAngles[2]-targetEulerAngles[2]);

        ROS_INFO_STREAM("Translation distance: " << distance);
        ROS_INFO_STREAM("x-axis difference: " << xDif);
        ROS_INFO_STREAM("y-axis difference: " << yDif);
        ROS_INFO_STREAM("z-axis difference: " << zDif);


      } else{
        ROS_INFO_STREAM("Could not find solution");
      }

      res.found_solution.data = found_ik;

      return true;
    }

    bool compute_fk(kinematics::FK_request::Request  &req,
             kinematics::FK_request::Response &res)
    {
      // Get the desired joint group according to the planning group
      const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(req.planning_group.data);

      // joint_names is a vector with all joints names of the used group
      //const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

      // Set the joint values as requested
      std::vector<double> joint_values;
      for (unsigned i = 0; i < req.joint_angles.data.size(); ++i){
        joint_values.push_back(req.joint_angles.data[i].data);
      }

      kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
      kinematic_state->update();

      // Compute forward kinematics
      const Eigen::Isometry3d &fk_solution = kinematic_state->getGlobalLinkTransform(req.tip.data);

      // Convert solution into a ros message and put it into the response
      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(fk_solution, pose);
      res.pose = pose;

      return true;
    }

    bool collision_check(kinematics::collision_check::Request  &req,
             kinematics::collision_check::Response &res)
    {
      // Receive the current planning scene, which contains information about the environment such as collisions
      planning_scene::PlanningScene planning_scene(kinematic_model);

      // Get the desired joint group according to the planning group
      const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(req.planning_group.data);

      // Set the joint values as requested
      std::vector<double> joint_values;
      for (unsigned i = 0; i < req.joint_angles.data.size(); ++i){
        joint_values.push_back(req.joint_angles.data[i].data);
      }

      kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
      kinematic_state->update();

      res.collision_state.data = planning_scene.isStateColliding(*kinematic_state, req.planning_group.data);

      return true;
    }
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematics_server");
  ros::NodeHandle nh;

  kinematics::KinematicsServer server;

  ros::ServiceServer ik_service = nh.advertiseService("moveit/compute_ik", &kinematics::KinematicsServer::compute_ik, &server);
  ros::ServiceServer fk_service = nh.advertiseService("moveit/compute_fk", &kinematics::KinematicsServer::compute_fk, &server);
  ros::ServiceServer collision_check_service = nh.advertiseService("moveit/collision_check", &kinematics::KinematicsServer::collision_check, &server);

  ROS_INFO("Ready to compute kinematics for NICO");
  ros::spin();

  return 0;
}
