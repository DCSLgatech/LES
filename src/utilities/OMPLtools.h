#ifndef OMPL_TOOLS_
#define OMPL_TOOLS_

#include<iostream>
#include<vector>
#include <fstream>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

// namespace ob = ompl::base;
// namespace og = ompl::geometric;
using namespace std;

class OMPLtools
{
public:
  unsigned int dim_state;

  OMPLtools(ompl::base::SpaceInformationPtr si);

  void WriteSolutionToFile(string filePath, ompl::base::ProblemDefinitionPtr pdef);

  void WriteGraphToFile(string filePath, ompl::base::PlannerDataPtr planner_data);

};
#endif
