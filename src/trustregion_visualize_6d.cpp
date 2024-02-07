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
#include "estrelrrtsharp/ESTRELRRTsharp.h"
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
    double sz=0.4;
    // std::vector<std::vector<double>> xc={{-3,-3},{-3,-1},{-3,1},{-3,3},
    //                                      {-2,-3},{-2,-1},{-2,1},{-2,3},
    //                                      {-1,-3},{-1,-1},{-1,1},{-1,3},
    //                                      {0,-3},{0,-1},{0,1},{0,3},
    //                                      {1,-3}, {1,-1},{1,1},{1,3},
    //                                      {2,-3},{2,-1},{2,1},{2,3},
    //                                      {3,-3},{3,-1},{3,1},{3,3}};
    std::vector<std::vector<double>> xc={{-3,-3},{-3,0},{-3,3},
                                         {-2,-3},{-2,0},{-2,3},
                                         {-1,-3},{-1,0},{-1,3},
                                         {0,-3},{0,0},{0,3},
                                         {1,-3}, {1,0},{1,3},
                                         {2,-3},{2,0},{2,3},
                                         {3,-3},{3,0},{3,3}};
    for(int i=0;i<xc.size();i++)
    {
      if((xState[0]>=xc[i][0]-sz) && (xState[0]<=xc[i][0]+sz) && (xState[1]>=xc[i][1]-sz) && (xState[1]<=xc[i][1]+sz))
  		{
  			return true;
  		}
    }
    return false;

	}
	if(CASE==1)
	{
		if((xState[0]>=-1.8) && (xState[0]<=3) && (xState[1]>=2) && (xState[1]<=3))
		{
			return true;
		}
    if((xState[0]>=-4) && (xState[0]<=-2) && (xState[1]>=2) && (xState[1]<=3))
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
  ob::ScopedState<> goal(space);
  for(int i=0;i<DIM_STATE;i++)
  {
    start->as<ob::RealVectorStateSpace::StateType>()->values[i] = .0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = 0;
  }
  start->as<ob::RealVectorStateSpace::StateType>()->values[1] = -4.5;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 4.5;
  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
  pdef->setStartAndGoalStates(start, goal);

  ompl::base::OptimizationObjectivePtr opt = std::make_shared<ob::PathLengthOptimizationObjective>(si);
  pdef->setOptimizationObjective(opt);
  // og::InformedRRTstar *plan_pt=new og::InformedRRTstar(si);
  og::TRUSTRRTsharp *plan_pt = new og::TRUSTRRTsharp(si);
  // og::ESTRELRRTsharp *plan_pt=new og::ESTRELRRTsharp(si);
  // og::DRRT *plan_pt = new og::DRRT(si);
  // og::LESDRRT *plan_pt = new og::LESDRRT(si);
  // og::RRTsharp *plan_pt=new og::RRTsharp(si);
  plan_pt->setGoalBias(0.1);
  plan_pt->setRange(.5);
  plan_pt->setKNearest(false);
  plan_pt->setInformedSampling(true);
  // plan_pt->setRelevantEpsilon(1.0);
  // plan_pt->setTreePruning(true);
  // plan_pt->setTrustRegionProbability(0.5);
  // plan_pt->setMaxDepth(0);
  // plan_pt->setRandomMaxDepth(true);
  // plan_pt->setRandomGrad(true);
  plan_pt->setUseDRRT(false);
  plan_pt->setDelayOptimizationUntilSolution(true);
  plan_pt->setDeformationFrequency(1.0);
  // plan_pt->setVariant(plan_pt->TREE);
  ob::PlannerPtr optimizingPlanner(plan_pt);
  optimizingPlanner->setProblemDefinition(pdef);
  optimizingPlanner->setup();
  ob::PlannerStatus solved;
  int it=0;
  solved = optimizingPlanner->solve(1);

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
