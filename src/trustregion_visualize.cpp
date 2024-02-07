#include <ros/ros.h>
#include <ros/package.h>
#include<iostream>
#include<vector>
#include <fstream>

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
#include "trustrrtsharp/TRUSTRRTsharp.h"
#include "drrt/DRRT.h"
#include "lesdrrt/LESDRRT.h"
#include "utilities/OMPLtools.h"

#define DIM_STATE 2
#define CASE 2

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

bool onObstacle(const ob::State *state)
{
  std::vector<float> xState;
  for(int i=0;i<DIM_STATE;i++)
  {
    xState.push_back(state->as<ob::RealVectorStateSpace::StateType>()->values[i]);
  }
	if(CASE==2)
	{
		if((xState[0]>=-8.5/4.) && (xState[0]<=8.5/4.) && (xState[1]>=2/2.-1/4.) && (xState[1]<=2/2.+1/4.))
		{
			return true;
		}

		if((xState[0]>=-4/2.-1/4.)&& (xState[0]<=-4/2.+1/4.) && (xState[1]>=8/2.-2.5/4.) && (xState[1]<=8/2.+2.5/4.))
		{
			return true;
		}

		if((xState[0]>=-4/2.-4/4.) && (xState[0]<=-4/2.+4/4.) && (xState[1]>=4.5/2.-2/4.) && (xState[1]<=4.5/2.+2/4.))
		{
			return true;
		}

		if((xState[0]>=7/2.-2.5/4.) && (xState[0]<=7/2.+2.5/4.) && (xState[1]>=3.5/2.-11/4.) && (xState[1]<=3.5/2.+11/4.))
		{
			return true;
		}

		if((xState[0]>=3/2.-2.5/4.) && (xState[0]<=3/2.+2.5/4.) && (xState[1]>=5/2.-2/4.) && (xState[1]<=5/2.+2/4.))
		{
			return true;
		}

		if((xState[0]>=-6/2.-3/4.) && (xState[0]<=-6/2.+3/4.) && (xState[1]>=-4/2.-5/4.) && (xState[1]<=-4/2.+5/4.))
		{
			return true;
		}

		if((xState[0]>=3/2.-2.5/4.) && (xState[0]<=3/2.+2.5/4.) && (xState[1]>=-5/2.-5/4.) && (xState[1]<=-5/2.+5/4.))
		{
			return true;
		}
	}
	if(CASE==1)
	{
		if((xState[0]>=-1) && (xState[0]<=1) && (xState[1]>=-1) && (xState[1]<=1))
		{
			return true;
		}
	}
	return(false);
}


class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}

    bool isValid(const ob::State* s) const
    {
        return !onObstacle(s);
    }
};

int main(int argc, char **argv)
{
	//Init all the variables
	ros::init (argc, argv, "ompl_visualize");
  ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle;

  ROS_INFO("RRT star planning");
  ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(DIM_STATE));

  ob::RealVectorBounds bounds(DIM_STATE);
  bounds.setLow(-5);
  bounds.setHigh(5);
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
  si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
  si->setup();

  ob::ScopedState<> start(space);
  start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0;
  start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;
  ob::ScopedState<> goal(space);
  goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 4;
  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
  pdef->setStartAndGoalStates(start, goal);

  ompl::base::OptimizationObjectivePtr opt = std::make_shared<ob::PathLengthOptimizationObjective>(si);
  pdef->setOptimizationObjective(opt);
  // og::InformedRRTstar *plan_pt=new og::InformedRRTstar(si);
  // og::TRUSTRRTsharp *plan_pt = new og::TRUSTRRTsharp(si);
  og::DRRT *plan_pt = new og::DRRT(si);
  // og::LESDRRT *plan_pt = new og::LESDRRT(si);
  // og::RRTsharp *plan_pt=new og::RRTsharp(si);
  plan_pt->setGoalBias(0.05);
  plan_pt->setRange(1.);
  plan_pt->setKNearest(true);
  plan_pt->setInformedSampling(true);
  // plan_pt->setRelevantEpsilon(1.0);
  // plan_pt->setTreePruning(true);
  // plan_pt->setTrustRegionProbability(0.5);
  // plan_pt->setMaxDepth(0);
  // plan_pt->setRandomMaxDepth(true);
  // plan_pt->setRandomGrad(true);
  // plan_pt->setDelayOptimizationUntilSolution(true);
  plan_pt->setDeformationFrequency(0.3);
  // plan_pt->setVariant(plan_pt->TREE);
  ob::PlannerPtr optimizingPlanner(plan_pt);
  optimizingPlanner->setProblemDefinition(pdef);
  optimizingPlanner->setup();
  ob::PlannerStatus solved;
  int it=0;
  solved = optimizingPlanner->solve(.1);

  if(solved==ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION)
  {
    // Write solution to file
    OMPLtools ompl_tools(si);
    std::string packagePath = ros::package::getPath("trust_region");
    string filePath=packagePath+"/src/Data/solution.txt";
    ompl_tools.WriteSolutionToFile(filePath,pdef);
    // Write graph to file
    ob::PlannerDataPtr planner_data(new ob::PlannerData(si));
    optimizingPlanner->getPlannerData(*planner_data);
    filePath=packagePath+"/src/Data/states.txt";
    ompl_tools.WriteGraphToFile(filePath,planner_data);
  }
  else
  {
    ROS_INFO("Planning failed");
    // planned=false;
  }

  return 0;
}
