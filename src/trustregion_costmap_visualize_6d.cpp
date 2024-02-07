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

#define DIM_STATE 6

class PotentialCostObjective : public ob::StateCostIntegralObjective
{
public:
  double cmax = 9.;
  double gamma = 3.;
  vector<vector<double>> p = {{5,5,5,5,5,5},
                              {5.20805,5.1556,4.79507,5.90786,7.88644,3.85745},
                            {3.83226,3.86456,3.86571,5.11152,2.73738,1.99655},
                          {7.90371,8.07641,6.3311,6.28637,7.59593,8.44751},
                        {8.83516,8.86107,8.5983,8.59174,8.78853,8.91659},
                      {2.8572,2.46773,7.12001,5.58831,6.0333,5.13955},
                    {8.45187,9.0659,10.1746,9.53281,9.0364,8.56388},
                  {4.71692,3.64549,2.48541,3.24694,0.618802,5.18102},
                {1.68765,1.86686,1.08252,1.31534,1.78389,1.33016},
              {5.32092,6.22737,7.47173,5.53284,3.24694,3.98185}};
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

  for(int i=0;i<DIM_STATE;i++)
  {
    start->as<ob::RealVectorStateSpace::StateType>()->values[i] = 1.0;
  }
  for(int i=0;i<DIM_STATE;i++)
  {
    goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = 9.0;
  }

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
  plan_pt->setRange(2.0);
  // plan_pt->setMaxStateCost(maxStateCost);
  plan_pt->setKNearest(false);
  plan_pt->setInformedSampling(true);
  // plan_pt->setTrustRegionProbability(0.5);
  // plan_pt->setMaxDepth(8);
  // plan_pt->setRandomGrad(true);
  // plan_pt->setTreePruning(true);
  plan_pt->setUseDRRT(true);
  plan_pt->setDeformationFrequency(1.3);
  plan_pt->setDelayOptimizationUntilSolution(true);
  ob::PlannerPtr optimizingPlanner(plan_pt);
  optimizingPlanner->setProblemDefinition(pdef);
  optimizingPlanner->setup();
  ob::PlannerStatus solved;
  int it=0;
  solved = optimizingPlanner->solve(6.0);
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
