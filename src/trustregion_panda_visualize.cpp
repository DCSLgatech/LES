#include <ros/ros.h>
#include <ros/package.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>
#include <mutex>
#include <fstream>
#include <sstream>

#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
// #include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include "rrtsharp/MRRTsharp.h"
#include "estrelrrtsharp/ESTRELRRTsharp.h"
#include "trustrrtsharp/TRUSTRRTsharp.h"
#include "drrt/DRRT.h"

// rosrun panda_moveit_config demo.launch

#define DIM_STATE 7

namespace ob = ompl::base;
namespace og = ompl::geometric;

ros::Publisher planning_scene_diff_publisher;
ros::Publisher joint_states;
planning_scene::PlanningScenePtr scene;
collision_detection::CollisionRequest collision_request;
collision_detection::CollisionResult collision_result;
const moveit::core::JointModelGroup* group;
robot_state::RobotState* current_state;
robot_model::RobotModelPtr kinematic_model;
moveit_msgs::PlanningScene planning_scene_msg;

Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
Eigen::MatrixXd jacobian;

ompl::base::StateSpacePtr space;

double vecNorm(std::vector<double> v)
{
	double sum=0;
	for (int i=0;i<v.size();i++)
	{
		sum=sum+v[i]*v[i];
	}
	return sqrt(sum);
}

bool checkCollision(robot_state::RobotState& st)
{
	// collision_result.clear();
	//scene->checkCollision(collision_request, collision_result,st);
	//return collision_result.collision;
	return !st.satisfiesBounds(group) || scene->isStateColliding(st);
	// return false;
}

void setState(robot_state::RobotState& st,std::vector<double> group_variable_values)
{
	st.setJointGroupPositions(group,group_variable_values);
}

std::mutex mut;

bool isObstacle(const ob::State* state)
{
  mut.lock();
	// ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space,state);
	std::vector<double> group_variable_values(DIM_STATE,0);
	for(int i=0;i<DIM_STATE;++i)
  {
			group_variable_values[i]=state->as<ob::RealVectorStateSpace::StateType>()->values[i];
	}
	setState(*current_state,group_variable_values);
	bool temp = checkCollision(*current_state);
  mut.unlock();
  return temp;
}

void addObstaclesToPlanningScene()
{
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "panda_link0";
	collision_object.id="box";

	int num_obs=6;
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);

	geometry_msgs::Pose box_pose;
	box_pose.orientation.x = 0.0;
	box_pose.orientation.y = 0.0;
	box_pose.orientation.z = 0.0;
	box_pose.orientation.w = 1.0;

	float posx=0.65;

	std::vector<std::vector<float>> pos
	{
		{posx, 0, 0.20},
		{posx, 0, 0.60},
		{posx, 0, 1.0},
		{posx, 0, 1.4},
		{posx, 0.55, .7},
		{posx, -0.55, .7}
	};
	std::vector<std::vector<float>> size
	{
		{0.60, 1.1, 0.02},
		{0.60, 1.1, 0.02},
		{0.60, 1.1, 0.02},
		{0.60, 1.1, 0.02},
		{0.60, 0.02, 1.4},
		{0.60, 0.02, 1.4}
	};

	for(int i=0;i<num_obs;i++)
	{
		box_pose.position.x = pos[i][0];
		box_pose.position.y = pos[i][1];
		box_pose.position.z = pos[i][2];
		primitive.dimensions[0]=size[i][0];
		primitive.dimensions[1]=size[i][1];
		primitive.dimensions[2]=size[i][2];
		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;
	}

	scene->getPlanningSceneMsg(planning_scene_msg);
	planning_scene_msg.world.collision_objects.push_back(collision_object);
	// scene->getCollisionWorld()->collision_objects.push_back(collision_object);
	planning_scene_msg.is_diff =true;
	scene->setPlanningSceneMsg(planning_scene_msg);
	planning_scene_diff_publisher.publish(planning_scene_msg);
}

void addObstaclesToPlanningScene2()
{
	moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene_msg.is_diff = true;
	std::string path = ros::package::getPath("trust_region");
	std::stringstream ss;
	ss << path << "/scenes/pillar.scene";
	std::ifstream file(ss.str());
	scene->loadGeometryFromStream(file);
	file.close();
	scene->getPlanningSceneMsg(planning_scene_msg);
	planning_scene_diff_publisher.publish(planning_scene_msg);
}

void publishState(robot_state::RobotState& st)
{
	sensor_msgs::JointState joint_msg;
	moveit::core::robotStateToJointStateMsg(st,joint_msg);
	joint_msg.header.stamp=ros::Time::now();
	joint_states.publish(joint_msg);
}

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}

    bool isValid(const ob::State* s) const
    {
        return !isObstacle(s);
    }
};

class ManipulabilityCostObjective : public ob::StateCostIntegralObjective
{
public:
    ManipulabilityCostObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
			std::cout << "ManipulabilityCostObjective" << '\n';
      setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
    }
    ob::Cost stateCost(const ob::State* state) const
    {
      double *coords=state->as<ob::RealVectorStateSpace::StateType>()->values;
			std::vector<double> group_variable_values(DIM_STATE,0);
			for(int i=0;i<DIM_STATE;++i)
		  {
					group_variable_values[i]=coords[i];
			}
			setState(*current_state,group_variable_values);
			current_state->getJacobian(group,current_state->getLinkModel(group->getLinkModelNames().back()),
																				reference_point_position, jacobian);

			double manipul = std::sqrt( (jacobian*jacobian.transpose()).determinant() );
			// std::cout << "manipul:" <<manipul<< '\n';
			double costval = 9*std::exp(-manipul );
      return ob::Cost(1.+costval);
    }
};

int main(int argc, char **argv)
{
	//Init all the variables
	ros::init (argc, argv, "panda_planner_ompl");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle;
	ros::Duration sleeper1(.01);
	ros::Duration sleeper01(.001);
	ros::Duration sleeper5(2);

	//publisher for world changes
	planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1);
	joint_states = node_handle.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states",1);

	//load robot model
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model_loader.getModel();
  scene=planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model));
	group=kinematic_model->getJointModelGroup("panda_arm");

	std::cout << "waiting for subscriber" << std::endl;
	while(planning_scene_diff_publisher.getNumSubscribers() < 1 && joint_states.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
  }
	sleeper5.sleep();
	sleeper5.sleep();
	std::cout << "done. modifying scene" << std::endl;

	addObstaclesToPlanningScene();
	collision_detection::CollisionRobotPtr colRob = scene->getCollisionRobotNonConst ();
	colRob->setPadding(0.004);
	scene	->propogateRobotPadding();
	sleeper5.sleep();

	//robot state, change robot state
	current_state = new robot_state::RobotState(kinematic_model);
	current_state = &(scene->getCurrentStateNonConst());
	// current_state->copyJointGroupPositions(group,group_variable_values);
	// std::vector<double> startvec = {2.05, -0.61, -2.28, -2.20, 2.13, 3.0, -1.85};
	// std::vector<double> startvec = {0.0, 0.0, 0.0, -1.57, 0.0, 1.571, 0.0};
	std::vector<double> startvec={1.9, -1.7, -2., -1.7, -0.8,3.0, 0.95};

  setState(*current_state,startvec);
  publishState(*current_state);

	// std::vector<double> goalvec= {1.11, .36, 0.2, -2.0, 2.0, 1.34, 0.15};
	// std::vector<float> goalvec={-2.29, -0.04, 2.17, -1.91, 1.47, 2.56, -2.77};
	std::vector<double> goalvec={1.83, 1.76, -1.33, -1.65, 1.67, 1.86, 0.93};
	// std::vector<float> goalvec={1.99, -0.6, .66, -1.42, .61, 1.31, 0};
	// std::vector<float> goalvec={-1.74, 0.48, .37, -1.11, 2.36, 2.17, .69};

	std::cout << "before any ompl call" << std::endl;
	moveit::core::JointBoundsVector bvec=group->getActiveJointModelsBounds();
	//Set Search Space Bounds
	ob::RealVectorBounds bound(DIM_STATE);
	for(int i=0;i<DIM_STATE;++i)
  {
		bound.setLow(i,(*(bvec[i]))[0].min_position_);
		bound.setHigh(i,(*(bvec[i]))[0].max_position_);
  }

  ROS_INFO("RRT planning");
	space.reset(new ob::RealVectorStateSpace(DIM_STATE));
  space->as<ob::RealVectorStateSpace>()->setBounds(bound);

  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
	si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
	si->setup();
	si->printSettings();

  // ob::ScopedState<> start(space);
	// ob::ScopedState<> goal(space);
	// for(int i=0;i<DIM_STATE;++i)
  // {
	// 	start->as<ob::RealVectorStateSpace::StateType>()->values[i]=startvec[i];
	// 	goal->as<ob::RealVectorStateSpace::StateType>()->values[i]=goalvec[i];
  // }

	ob::ValidStateSamplerPtr valid_sampler=si->allocValidStateSampler();
	ob::State *start=si->allocState();
	ob::State *goal=si->allocState();
	valid_sampler->sample(start);
	valid_sampler->sample(goal);

  std::cout << "valid start goal : " << !isObstacle(start) << " , " << !isObstacle(goal) << std::endl;
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
	pdef->setStartAndGoalStates(start, goal);
	ompl::base::OptimizationObjectivePtr opt = std::make_shared<ob::PathLengthOptimizationObjective>(si);
	// ompl::base::OptimizationObjectivePtr opt = std::make_shared<ManipulabilityCostObjective>(si);
	pdef->setOptimizationObjective(opt);

	// og::RRTstar *plan_pt=new og::RRTstar(si);
	// og::InformedRRTstar *plan_pt=new og::InformedRRTstar(si);
	// plan_pt->setTreePruning(true);
	og::MRRTsharp *plan_pt=new og::MRRTsharp(si);
	// og::TRUSTRRTsharp *plan_pt = new og::TRUSTRRTsharp(si);
  // og::DRRT *plan_pt = new og::DRRT(si);
	// og::ESTRELRRTsharp *plan_pt=new og::ESTRELRRTsharp(si);
	// plan_pt->setTrustRegionProbability(0.5);
	plan_pt->setGoalBias(0.1);
	plan_pt->setRange(1.2);
	plan_pt->setInformedSampling(true);
	plan_pt->setKNearest(false);

	std::cout << "create optimizingplanner" << std::endl;
	ob::PlannerPtr optimizingPlanner(plan_pt);
	// std::cout << "set problem defionition" << std::endl;
	optimizingPlanner->setProblemDefinition(pdef);
	// std::cout << "setup" << std::endl;
	optimizingPlanner->setup();

	ob::PlannerStatus solved;
	int it=0;
	std::cout << "run algo" << std::endl;
	while(solved!=ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION && it<1)
	{
		it++;
		solved = optimizingPlanner->solve(1.5);
	}

	std::vector<std::vector<double>> vec_path;
	std::vector<double> vec_state;
	if(solved==ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION || solved==ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
	{
		if(solved!=ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION)
		{
			std::cout << "approximate solution" << std::endl;
		}
		ob::PathPtr path =pdef->getSolutionPath();
		og::PathGeometric geo_path = static_cast<ompl::geometric::PathGeometric&>(*path);
		std::vector<ob::State *> sol1 = geo_path.getStates();
		// std::vector< ob::State * > sol1 = boost::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->getStates();
		std::cout << "Printing Path:" << '\n';
		for(int i=0;i<sol1.size();i++)
		{
			vec_state.clear();
			for(int j=0;j<DIM_STATE;j++)
			{
				std::cout <<sol1[i]->as<ob::RealVectorStateSpace::StateType>()->values[j]  << ",";
				vec_state.push_back(sol1[i]->as<ob::RealVectorStateSpace::StateType>()->values[j]);
			}
			vec_path.push_back(vec_state);
			std::cout << '\n';
		}
		ROS_INFO("Planning success");
	}

	// moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
	// moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// const moveit::core::JointModelGroup* joint_model_group =
  //     move_group.getCurrentState()->getJointModelGroup("panda_arm");
	// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	// // namespace rvt = rviz_visual_tools;
	// // moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
	// // visual_tools.deleteAllMarkers();
	// move_group.setJointValueTarget(goalvec);
	// move_group.setMaxVelocityScalingFactor(0.05);
  // move_group.setMaxAccelerationScalingFactor(0.05);
	// // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan (joint space goal) ");
	// while(ros::ok())
	// {
	// 	// setState(*current_state,startvec);
	// 	for(int i=0;i<vec_path.size()-1;i++)
	// 	{
	// 		setState(*current_state,vec_path[i]);
	// 		move_group.setJointValueTarget(vec_path[i+1]);
	// 		move_group.plan(my_plan);
	// 		sleeper1.sleep();
	// 	}
	// 	// sleeper5.sleep();
	// 	// sleeper5.sleep();
	// }

	std::vector<double> edir(DIM_STATE,0);
	double dist=0;
	double pas=0.001;
	for(int i=0;i<1;i++)
	{
		// std::vector<float> startvec = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
		for(int j=0;j<DIM_STATE;j++)
		{
			vec_state[j]=vec_path[0][j];
		}
		for(int j=1;j<vec_path.size();j++)
		{
			dist=0;
			for(int i=0;i<DIM_STATE;++i)
			{
				edir[i] = vec_path[j][i]-vec_state[i];
				dist=dist+edir[i]*edir[i];
			}
			dist=pow(dist,0.5);
			for(int i=0;i<DIM_STATE;i++)
			{
				edir[i]=edir[i]/dist;
			}
			std::vector<double> group_variable_values(DIM_STATE,0);
			for(float kmul=0.0;kmul<=dist;kmul=kmul+pas)
			{
				for(int i=0;i<group_variable_values.size();++i)
				{
					group_variable_values[i] = vec_state[i]+edir[i]*kmul;
				}
				setState(*current_state,group_variable_values);
				publishState(*current_state);
				sleeper01.sleep();
			}
			for(int i=0;i<DIM_STATE;i++)
			{
				vec_state[i]=vec_path[j][i];
			}
		}
		sleeper5.sleep();
		// break;
	}

	return 0;
}
