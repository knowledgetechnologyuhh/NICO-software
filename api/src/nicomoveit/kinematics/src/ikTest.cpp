/**
This program can be used to evaluate performance of an IK on a series of joint configurations.
**/

// Ros
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

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

int main(int argc, char **argv) {
  using namespace std;
  using namespace boost;

  std::string kinematics(ros::package::getPath("kinematics"));

  bool useGrid = false;
  bool useProvidedCompletePoses = true;

  string data(kinematics + "/testPoses/current"); // define the csv file with poses to test

  ifstream in(data.c_str());
  if (!in.is_open()) {
    ROS_WARN("File couldn't be opened");
    return 1;
  }
  typedef tokenizer< escaped_list_separator<char> > Tokenizer;

  vector< string > vec, line1, line2, line3, line4;
  string line;

  // Initialization of ros
  ros::init(argc, argv, "left_arm_kinematics");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // The URDF is available on the parameter server as "robot_description"
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  // Build a kinematic model to the URDF
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Receive the current planning scene, which contains information about the environment such as collisions
  planning_scene::PlanningScene planning_scene(kinematic_model);

  // collisions should not be ignored for the inverse kinematic solutions
  bool ignore_collisions = false;

  const moveit::core::GroupStateValidityCallbackFn is_valid = std::bind(
		  &isValid,
		  &planning_scene,
		  ignore_collisions,
		  std::placeholders::_1,
		  std::placeholders::_2,
      std::placeholders::_3);

  // Using the :moveit_core:RobotModel, we can construct a
  // :moveit_core:RobotState that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values.
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("leftArm");

  // joint_names is a vector with all joints names of the used group
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
  std::vector<double> joint_values;

  // Initialize some variables to track statistics
  int positives = 0, negatives = 0, nofRequests = 0;
  clock_t begin, end;
  double elapsedSecs = 0.0, timeForThis = 0.0, timeForPositives = 0.0, distanceToTarget = 0.0;
  double overallXdif = 0.0, overallYdif = 0.0, overallZdif = 0.0;

  Eigen::Matrix4d targetMatrix;
  Eigen::Vector3d targetTranslation;
  Eigen::Matrix3d targetRotation;
  Eigen::Vector3d targetEulerAngles;

  // the file 'grid' will contain all positions of a grid-target-file and an 'n' or 'y' depending on if a solution was found
  // the file 'targets' will contain all target poses used as targets in the current test
  // the file 'reachedTargets' will contain all targets poses for which a solution was found
  // the file 'reached' will contain the corresponding reached poses to the reached targets and some statistics in the very last line
  // the file 'jointValues' will contain the corresponding reached joint values
  int experiment_count_int;
  nh.param<int>("/experiment_count", experiment_count_int, -1);
  std::string experiment_count = std::to_string(experiment_count_int);

  ofstream grid, reachedTargetsFile, reachedFile, targetsFile, jointValuesFile;
  grid.open (kinematics+"/results/results_"+experiment_count+"/grid");
  targetsFile.open (kinematics+"/results/results_"+experiment_count+"/targets");
  reachedTargetsFile.open (kinematics+"/results/results_"+experiment_count+"/reachedTargets");
  reachedFile.open (kinematics+"/results/results_"+experiment_count+"/reached");
  jointValuesFile.open (kinematics+"/results/results_"+experiment_count+"/jointValues");

  int lineCounter = 0;

  // Iterate through the lines of the provided file
  while (getline(in,line)) {
    // remove white spaces at beginning and end of string
    trim(line);
    // replace multiple white spaces with a single one
    line = boost::regex_replace(line, boost::regex("[' ']{2,}"), " ");
    escaped_list_separator<char> sep('\\',' ', '\"');
    Tokenizer tok(line, sep);
    vec.assign(tok.begin(),tok.end());

    ++lineCounter;
    if (useProvidedCompletePoses and lineCounter < 5) {
      if (lineCounter == 1) {
        line1 = vec;
      } else if (lineCounter == 2) {
        line2 = vec;
      } else if (lineCounter == 3) {
        line3 = vec;
      } else if (lineCounter == 4) {
        line4 = vec;
      }
      continue;
    }
    lineCounter = 0;
    ++nofRequests;

    bool found_ik;

    if (useGrid or useProvidedCompletePoses) {
      Eigen::Isometry3d target;
      if (useGrid) {
        if (vec.size() != 3) {
          ROS_WARN("Wrong input format");
          continue;
        }

        // define the translation to use
        Eigen::Isometry3d translation(Eigen::Translation3d(Eigen::Vector3d(std::stod(vec[0]),std::stod(vec[1]),std::stod(vec[2]))));

        double o_x, o_y, o_z, o_w;
        // use a side grasp orientation
        //o_x = 0.699255043834;
        //o_y = -0.0896847733213;
        //o_z = 0.705730390728;
        //o_w = 0.0703110283715;

        // use a top grasp orientation
        o_x = -0.466315450108;
        o_y = -0.493845877759;
        o_z = 0.548284810402;
        o_w = 0.487903593646 ;

        // define the rotation matrix
        Eigen::Matrix3d rot3 = Eigen::Quaterniond(o_w, o_x, o_y, o_z).toRotationMatrix();
        Eigen::Matrix4d rot4 = Eigen::Matrix4d::Identity();
        rot4.block(0,0,3,3) = rot3;

        // build a affine transformation out of translation and rotation
        Eigen::Matrix4d mat = translation.matrix();
        mat *= rot4.matrix();
        target.matrix() = mat;
      } else {
        Eigen::Matrix4d mat;
        mat(0,0) = std::stod(line1[0]);
        mat(0,1) = std::stod(line1[1]);
        mat(0,2) = std::stod(line1[2]);
        mat(0,3) = std::stod(line1[3]);
        mat(1,0) = std::stod(line2[0]);
        mat(1,1) = std::stod(line2[1]);
        mat(1,2) = std::stod(line2[2]);
        mat(1,3) = std::stod(line2[3]);
        mat(2,0) = std::stod(line3[0]);
        mat(2,1) = std::stod(line3[1]);
        mat(2,2) = std::stod(line3[2]);
        mat(2,3) = std::stod(line3[3]);
        mat(3,0) = std::stod(line4[0]);
        mat(3,1) = std::stod(line4[1]);
        mat(3,2) = std::stod(line4[2]);
        mat(3,3) = std::stod(line4[3]);
        target.matrix() = mat;
      }

      // Print end-effector pose. Remember that this is in the model frame
      targetTranslation = target.translation();
      targetRotation = target.rotation();

      ROS_INFO_STREAM("Target Translation: " << targetTranslation.matrix());
      ROS_INFO_STREAM("Target Rotation: " << targetRotation.matrix());

      // Compute euler angles of target rotation,
      // this will be used to compute the distance to the inverse kinematics solution
      targetEulerAngles = targetRotation.eulerAngles(0, 1, 2);
      targetMatrix = target.matrix();

      // Set all joint angles to their default value
      // Without this, the perfect solution is always found! Why?
      kinematic_state->setToDefaultValues();

      //  Compute the inverse kinematics and measure how long it takes
      begin = clock();
      found_ik = kinematic_state->setFromIK(joint_model_group,target,"left_palm:11",3,0.25, is_valid);
        // * The number of attempts to be made at solving IK: 3
        // * The timeout for each attempt: 0.25 s
      end = clock();
      timeForThis = double(end - begin) / CLOCKS_PER_SEC;
      elapsedSecs += timeForThis;

    } else {

      if (vec.size() != 6) {
        ROS_WARN("Wrong input format");
        continue;
      }

      // Copy the current joint angles into joint_values
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

      // Assign new joint angle values
      joint_values[0] = radians(std::stod(vec[0]));
      joint_values[1] = radians(std::stod(vec[1]));
      joint_values[2] = radians(std::stod(vec[2]));
      joint_values[3] = radians(std::stod(vec[3]));
      joint_values[4] = radians(std::stod(vec[4]));
      joint_values[5] = radians(std::stod(vec[5]));

      kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
      kinematic_state->setToRandomPositions(joint_model_group); // this can be used if we want random joints values

      // Compute forward kinematics for the end of the arm, save the result in target
      const Eigen::Isometry3d &target = kinematic_state->getGlobalLinkTransform("left_palm:11");

      // Copy the target joint angles into joint_values and print them
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      ROS_INFO("The target joint angles");
      for (std::size_t i = 0; i < joint_names.size(); ++i) {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
      // Print end-effector pose. Remember that this is in the model frame
      targetTranslation = target.translation();
      targetRotation = target.rotation();

      ROS_INFO_STREAM("Target Translation: " << targetTranslation.matrix());
      ROS_INFO_STREAM("Target Rotation: " << targetRotation.matrix());

      // Compute euler angles of target rotation,
      // this will be used to compute the distance to the inverse kinematics solution
      targetEulerAngles = targetRotation.eulerAngles(0, 1, 2);
      targetMatrix = target.matrix();

      // Set all joint angles to their default value
      // Without this, the perfect solution is always found! Why?
      kinematic_state->setToDefaultValues();

      //  Compute the inverse kinematics and measure how long it takes
      begin = clock();
      found_ik = kinematic_state->setFromIK(joint_model_group,target,3,0.25,is_valid);
        // * The number of attempts to be made at solving IK: 3
        // * The timeout for each attempt: 0.25 s
      end = clock();
      timeForThis = double(end - begin) / CLOCKS_PER_SEC;
      elapsedSecs += timeForThis;
    }

    targetsFile << targetMatrix << "\n\n";

    // Now, we can use the IK solution (if found):
    if (found_ik) {
      if (useGrid) {
        grid << std::stod(vec[0]) << "," << std::stod(vec[1]) << "," << std::stod(vec[2]) << ",y\n";
      }

      // Copy the retrieved joint values into joint_values variable
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      ROS_INFO("The reached joint angles");
      for (std::size_t i = 0; i < joint_names.size(); ++i) {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }

      jointValuesFile << joint_values[0];
      for (unsigned i = 1; i < joint_values.size(); ++i){
        jointValuesFile << "," << joint_values[i];
      }
      jointValuesFile << "\n";

      // Compute forward kinematics for the end of the arm, save the result in reached
      const Eigen::Isometry3d &reached = kinematic_state->getGlobalLinkTransform("left_palm:11");

      Eigen::Vector3d reachedTranslation = reached.translation();
      Eigen::Matrix3d reachedRotation = reached.rotation();
      Eigen::Vector3d reachedEulerAngles = reachedRotation.eulerAngles(0, 1, 2);

      ROS_INFO_STREAM("Reached Translation: " << reachedTranslation);
      ROS_INFO_STREAM("Reached Rotation: " << reachedRotation);

      double distance = (reachedTranslation-targetTranslation).norm(); //.norm? test if it is sqrt(x^2)
      double xDif = fabs(reachedEulerAngles[0]-targetEulerAngles[0]);
      double yDif = fabs(reachedEulerAngles[1]-targetEulerAngles[1]);
      double zDif = fabs(reachedEulerAngles[2]-targetEulerAngles[2]);

      //ROS_INFO_STREAM("Translation distance: " << (reachedTranslation-targetTranslation).norm());
      //ROS_INFO_STREAM("x-axis difference: " << xDif);
      //ROS_INFO_STREAM("y-axis difference: " << yDif);
      //ROS_INFO_STREAM("z-axis difference: " << zDif);

      distanceToTarget += distance;
      overallXdif += xDif;
      overallYdif += yDif;
      overallZdif += zDif;

      timeForPositives += timeForThis;

      ++positives;

      // target and reached pose are saved into files, to compute error with python script
      reachedTargetsFile << targetMatrix << "\n\n";
      reachedFile << reached.matrix() << "\n\n";
    }
    else {
      if (useGrid) {
        grid << std::stod(vec[0]) << "," << std::stod(vec[1]) << "," << std::stod(vec[2]) << ",n\n";
      }
      ROS_WARN("Did not find IK solution");
      ++negatives;
    }
  }

  grid.close();
  targetsFile.close();
  reachedTargetsFile.close();
  reachedFile << "stats " << positives << " " << negatives << " " << elapsedSecs << " " << timeForPositives << "\n";
  reachedFile.close();



  cout << "Percentage of successful IK solutions: " << positives / (double) (negatives + positives) << " ("<< positives  << " positives and " << negatives << " negatives)" << endl;
  cout << "Overall processing time: " << elapsedSecs << endl;
  cout << "Average processing time per request: " << elapsedSecs / nofRequests  << endl;
  cout << "Average processing time per successful request: " << timeForPositives / positives  << endl;
  cout << "Average distance to target in successful attempts: " << distanceToTarget / positives  << endl;
  cout << "Average difference to x axis orientation: " << overallXdif / positives << endl;
  cout << "Average difference to y axis orientation: " << overallYdif / positives << endl;
  cout << "Average difference to z axis orientation: " << overallZdif / positives << endl;

  /*
  ofstream stats;
  stats.open (kinematics+"/statistics/experiment", std::ios_base::app);
  stats << positives << "," << negatives << "," << elapsedSecs / (positives + negatives) << "," << timeForPositives / positives << "," << distanceToTarget / positives << "," << overallXdif / positives << "," << overallYdif / positives << "," << overallZdif / positives << "\n";
  stats.close();
  */

  ros::ServiceClient restart_client = nh.serviceClient<std_srvs::Empty>("/restarter");
  std_srvs::Empty restart_srv;

  ROS_INFO("Requesting to restart the system for the next experiment");
  restart_client.call(restart_srv);


  ros::shutdown();
  return 0;
}
