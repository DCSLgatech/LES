#include <ros/ros.h>
#include <ros/package.h>

#include <ompl/base/Cost.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include "rrtsharp/MRRTsharp.h"
#include "trustrrtsharp/TRUSTRRTsharp.h"
#include "drrt/DRRT.h"
#include "lesdrrt/LESDRRT.h"
#include "utilities/ppm.h"
#include "utilities/OMPLtools.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

#define DIM_STATE 2

class CostMapObjective : public ob::StateCostIntegralObjective
{
public:
  float maxBound;
  PPMImage *img;
  double *cost;
  std::string filePath;
    CostMapObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
      // default costToGo
      setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
      maxBound=10;

      filePath ="/home/sjoshi99/catkin_ws/src/panda_ompl/src/resources/grand_canyon.ppm";
      img = readPPM(filePath.c_str());
      cost=createCostMap(img);
    }
    ob::Cost stateCost(const ob::State* state) const
    {
      double *coords=state->as<ob::RealVectorStateSpace::StateType>()->values;
      unsigned int pixelX= coords[0]*(img->x/maxBound);
      pixelX=std::min(pixelX,img->x-1);
      unsigned int pixelY= coords[1]*(img->y/maxBound);
      pixelY=std::min(pixelY,img->y-1);
      int id=pixelY*img->x+pixelX;
      return ob::Cost(1.+1*cost[id]);
    }
};

class PotentialCostObjective : public ob::StateCostIntegralObjective
{
public:
  double cmax = 9;
  double gamma = 1;
  vector<vector<double>> p = {{2,3},
                             {2,7},
                             {5,3},
                             {5,7},
                             {8,3},
                             {8,7}};;
  PotentialCostObjective(const ob::SpaceInformationPtr& si) :
      ob::StateCostIntegralObjective(si, true)
  {
    // default costToGo
    setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
  }
  ob::Cost stateCost(const ob::State* state) const
  {
    double *coords=state->as<ob::RealVectorStateSpace::StateType>()->values;
    double costval = 0;
    for(int j=0;j<p.size();j++)
    {
      double normsq=0;
      for(int i=0;i<DIM_STATE;i++)
      {
        normsq=normsq+pow(coords[i]-p[j][i],2);
      }
      costval = costval + cmax*(exp(-normsq/gamma));
    }
    return ob::Cost(1.+costval);
  }
};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "ompl_costmap_visualize");
  ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle;
  // double maxStateCost=getMaxStateCost(cost,img->getSize());

  ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(DIM_STATE));
  ob::RealVectorBounds bounds(DIM_STATE);
  bounds.setLow(0);
  bounds.setHigh(10);
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
  // si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
  si->setup();

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);

  // start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 2;
  // start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 2;
  // goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 2;
  // goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 9;

  start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
  start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 9.;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 9.;

  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
  pdef->setStartAndGoalStates(start, goal);

  // ompl::base::OptimizationObjectivePtr opt = std::make_shared<CostMapObjective>(si);
  ompl::base::OptimizationObjectivePtr opt = std::make_shared<PotentialCostObjective>(si);
  pdef->setOptimizationObjective(opt);

  // og::MRRTsharp *plan_pt=new og::MRRTsharp(si);
  // og::ESTRELRRTsharp *plan_pt=new og::ESTRELRRTsharp(si);
  og::TRUSTRRTsharp *plan_pt = new og::TRUSTRRTsharp(si);
  // og::DRRT *plan_pt = new og::DRRT(si);
  // og::LESDRRT *plan_pt = new og::LESDRRT(si);
  // og::TRRTsharp *plan_pt=new og::TRRTsharp(si);
  plan_pt->setGoalBias(0.1);
  plan_pt->setRange(0.5);
  // plan_pt->setMaxStateCost(maxStateCost);
  plan_pt->setKNearest(false);
  plan_pt->setInformedSampling(true);
  // plan_pt->setTrustRegionProbability(0.5);
  // plan_pt->setMaxDepth(8);
  // plan_pt->setRandomGrad(true);
  // plan_pt->setTreePruning(true);
  // plan_pt->setDeformationFrequency(0.5);
  // plan_pt->setDelayOptimizationUntilSolution(true);
  ob::PlannerPtr optimizingPlanner(plan_pt);
  optimizingPlanner->setProblemDefinition(pdef);
  optimizingPlanner->setup();
  ob::PlannerStatus solved;
  int it=0;
  solved = optimizingPlanner->solve(2.0);
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
