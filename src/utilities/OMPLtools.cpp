#include "OMPLtools.h"

OMPLtools::OMPLtools(ompl::base::SpaceInformationPtr si)
{
  dim_state=si->getStateDimension();
}

void OMPLtools::WriteSolutionToFile(string filePath, ompl::base::ProblemDefinitionPtr pdef)
{
  ompl::base::PathPtr path =pdef->getSolutionPath();
  ompl::geometric::PathGeometric geo_path = static_cast<ompl::geometric::PathGeometric&>(*path);
  std::vector<ompl::base::State *> sol = geo_path.getStates();
  std::cout << "sol length:" <<sol.size()<< '\n';
  ofstream myfile;
  myfile.open (filePath);
  std::vector<float> vec_state(dim_state,0);
  for(int j=0;j<sol.size();j++)
  {
    for(int i=0;i<dim_state;i++)
    {
      vec_state[i]=sol[j]->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
      if(i==dim_state-1)
      {
        // cout<<vec_state[i];
        myfile<<vec_state[i];
      }
      else
      {
        // cout<<vec_state[i]<<",";
        myfile<<vec_state[i]<<",";
      }
    }
    myfile<<endl;
    // std::cout << '\n';
  }
  myfile.close();
}

void OMPLtools::WriteGraphToFile(string filePath, ompl::base::PlannerDataPtr planner_data)
{
  ofstream myfile;
  myfile.open (filePath);
  std::vector<float> vec_state(dim_state,0);
  for(int j=0;j<planner_data->numVertices();j++)
  {
     ompl::base::PlannerDataVertex parent_vertex = planner_data->getVertex(j);
     const ompl::base::State *parent_state=parent_vertex.getState();
     std::vector<unsigned int> edge_list;
     planner_data->getEdges(j,edge_list);
     for(int k=0;k<edge_list.size();k++)
     {
       ompl::base::PlannerDataVertex vertex = planner_data->getVertex(edge_list[k]);
       const ompl::base::State *state=vertex.getState();
       for(int i=0;i<dim_state;i++)
       {
         vec_state[i]=state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
         myfile<<vec_state[i]<<",";
       }
       for(int i=0;i<dim_state;i++)
       {
         vec_state[i]=parent_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
         if(i==dim_state-1)
         {
           myfile<<vec_state[i];
         }
         else
         {
           myfile<<vec_state[i]<<",";
         }
       }
       myfile<<endl;
     }
  }
  myfile.close();
}
