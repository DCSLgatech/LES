/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Georgia Institute of Technology
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Florian Hauer */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_ESTRELRRTXSTATIC_
#define OMPL_GEOMETRIC_PLANNERS_RRT_ESTRELRRTXSTATIC_

#include "BinaryHeap.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/goals/GoalState.h"

#include <deque>
#include <limits>
#include <list>
#include <queue>
#include <utility>
#include <vector>

namespace ompl
{
    namespace geometric
    {
        /**

        /** \brief Optimal Rapidly-exploring Random Trees Maintaining A Pseudo Optimal Tree*/
        class ESTRELRRTXstatic : public base::Planner
        {
        public:
            ESTRELRRTXstatic(const base::SpaceInformationPtr &si);

            virtual ~ESTRELRRTXstatic();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            virtual void setup();

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Use direct sampling of the heuristic for the generation of random samples (e.g., x_rand).
           If a direct sampling method is not defined for the objective, rejection sampling will be used by default. */
            void setInformedSampling(bool informedSampling);

            /** \brief Get the state direct heuristic sampling */
            bool getInformedSampling() const
            {
                return useInformedSampling_;
            }

            /** \brief Controls whether heuristic rejection is used on samples (e.g., x_rand) */
            void setSampleRejection(bool reject);

            /** \brief Get the state of the sample rejection option */
            bool getSampleRejection() const
            {
                return useRejectionSampling_;
            }

            /** \brief Set the number of attempts to make while performing rejection or informed sampling */
            void setNumSamplingAttempts(unsigned int numAttempts)
            {
                numSampleAttempts_ = numAttempts;
            }

            /** \brief Get the number of attempts to make while performing rejection or informed sampling */
            unsigned int getNumSamplingAttempts() const
            {
                return numSampleAttempts_;
            }

            /** \brief Set the threshold epsilon

                While propagating information, the propagation is done only if the cost enhancement is at least epsilon
               */
            virtual void setEpsilon(double epsilon)
            {
                epsilonCost_ = base::Cost(epsilon);
            }

            /** \brief Get the threshold epsilon the planner is using */
            double getEpsilon() const
            {
                return epsilonCost_.value();
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* (or k_rrg = s \times k_rrg*)
             */
            void setRewireFactor(double rewireFactor)
            {
                rewireFactor_ = rewireFactor;
                calculateRewiringLowerBounds();
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* > r_rrg* (or k_rrg = s \times
             * k_rrg* > k_rrg*) */
            double getRewireFactor() const
            {
                return rewireFactor_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            /** \brief Use a k-nearest search for rewiring instead of a r-disc search. */
            void setKNearest(bool useKNearest)
            {
                useKNearest_ = useKNearest;
            }

            /** \brief Get the state of using a k-nearest search for rewiring. */
            bool getKNearest() const
            {
                return useKNearest_;
            }

            /** \brief Set whether or not to always propagate cost updates to children */
            void setUpdateChildren(bool val)
            {
                updateChildren_ = val;
            }

            /** \brief True if the cost is always propagate to children */
            bool getUpdateChildren() const
            {
                return updateChildren_;
            }

            /** \brief Set variant used for rejection sampling */
            void setVariant(const int variant)
            {
                if (variant < 0 || variant > 3)
                    throw Exception("Variant must be 0 (original RRT#) or in [1, 3]");
                variant_ = variant;
            }

            /** \brief Get variant used for rejection sampling */
            int getVariant() const
            {
                return variant_;
            }

            /** \brief Set the value alpha used for rejection sampling */
            void setAlpha(const double a)
            {
                alpha_ = a;
            }

            /** \brief Get the value alpha used for rejection sampling */
            double getAlpha() const
            {
                return alpha_;
            }

            unsigned int numIterations() const
            {
                return iterations_;
            }

            ompl::base::Cost bestCost() const
            {
                return bestCost_;
            }

            void setMaxStateCost(double max)
            {
              maxStateCost=max;
            }
            double getMaxStateCost() const
            {
              return maxStateCost;
            }
            void setRelevantEpsilon(double relevant_epsilon)
            {
              relevantEpsilon_=relevant_epsilon;
            }
            double getRelevantEpsilon() const
            {
              return relevantEpsilon_;
            }

        protected:
            class Motion;

            /** \brief Defines the operator to compare motions */
            struct MotionCompare
            {
                /** \brief Constructor */
                MotionCompare(base::OptimizationObjectivePtr opt, base::ProblemDefinitionPtr pdef)
                  : opt_(std::move(opt)), pdef_(std::move(pdef))
                {
                }
                /** \brief Combines the current cost of a motion and the heuritic to the goal */
                inline base::Cost costPlusHeuristic(const Motion *m) const
                {
                    return opt_->combineCosts(m->cost, opt_->costToGo(m->state, pdef_->getGoal().get()));
                }
                /** \brief Combines the current cost of a motion, weighted by alpha, and the heuritic to the goal */
                inline base::Cost alphaCostPlusHeuristic(const Motion *m, double alpha) const
                {
                    return opt_->combineCosts(base::Cost(alpha * m->cost.value()),
                                              opt_->costToGo(m->state, pdef_->getGoal().get()));
                }
                /** \brief Ordering of motions */
                inline bool operator()(const Motion *m1, const Motion *m2) const
                {
                    // we use a max heap, to do a min heap so the operator < returns > in order to make it a min heap
                    return !opt_->isCostBetterThan(costPlusHeuristic(m1), costPlusHeuristic(m2));
                }
                /** \brief Pointer to the Optimization Objective */
                base::OptimizationObjectivePtr opt_;
                /** \brief Pointer to the Problem Definition */
                base::ProblemDefinitionPtr pdef_;
            };

            struct relMotionCompare
            {
              relMotionCompare()
              {
              }
              inline double relcost(const Motion *m) const
              {
                return (10*m->n_expanded + 5*m->nbh.size() ) + 100*(m->fcost.value())/m->bestSolCost.value();
                // return pow(m->n_expanded,1)*pow(m->nbh.size(),.1);
              }
              inline bool operator()(const Motion *m1, const Motion *m2) const
          		{
          			return relcost(m1)<relcost(m2);
          		}
            };

            /** \brief Representation of a motion (node of the tree) */
            class Motion
            {
            public:
                /** \brief Constructor that allocates memory for the state. This constructor automatically allocates
                 * memory for \e state, \e cost, and \e incCost */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(nullptr), handle(nullptr) , relhandle(nullptr)
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state;

                /** \brief The parent motion in the exploration tree */
                Motion *parent;

                /** \brief The cost up to this motion */
                base::Cost cost;

                /** \brief The estimate of solution cost through this motion*/
                base::Cost fcost;

                /** \brief store the value of current best solution cost */
                base::Cost bestSolCost;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion *> children;

                /** \brief The set of neighbors of this motion with a boolean indicating if the feasibility of edge as
                 * been tested */
                std::vector<std::pair<Motion *, bool>> nbh;

                /** \brief Handle to identify the motion in the queue */
                BinaryHeap<Motion *, MotionCompare>::Element *handle;

                /** \brief Handle to identify the motion in the relevant queue */
                BinaryHeap<Motion *, relMotionCompare>::Element *relhandle;
                /** \brief record no. of times motion is expanded */
                int n_expanded=0;
            };

            /** \brief Create the samplers */
            void allocSampler();

            /** \brief Generate a sample */
            bool sampleUniform(base::State *statePtr);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Update (or add) a motion in the queue */
            void updateQueue(Motion *x);

            /** \brief Removes the given motion from the parent's child list */
            void removeFromParent(Motion *m);

            /** \brief Gets the neighbours of a given motion, using either k-nearest of radius as appropriate. */
            void getNeighbors(Motion *motion) const;

            /** \brief Calculate the k_RRG* and r_RRG* terms */
            void calculateRewiringLowerBounds();

            /** \brief Calculate the rrg_r_ and rrg_k_ terms */
            void calculateRRG();

            /** \brief Test if the vertex should be included according to the variant in use */
            bool includeVertex(const Motion *x) const;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief An informed sampler */
            base::InformedSamplerPtr infSampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief Option to use k-nearest search for rewiring */
            bool useKNearest_{true};

            /** \brief The rewiring factor, s, so that r_rrg = s \times r_rrg* > r_rrg* (or k_rrg = s \times k_rrg* >
             * k_rrg*) */
            double rewireFactor_{1.1};

            /** \brief A constant for k-nearest rewiring calculations */
            double k_rrt_{0u};
            /** \brief A constant for r-disc rewiring calculations */
            double r_rrt_{0.};

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            /** \brief A list of states in the tree that satisfy the goal condition */
            std::vector<Motion *> goalMotions_;

            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief Number of iterations the algorithm performed */
            unsigned int iterations_{0u};

            /** \brief Comparator of motions, used to order the queue */
            MotionCompare mc_;
            relMotionCompare relmc_;
            /** \brief relevant sampling variables */
            double maxStateCost{1};
            double relevantEpsilon_{0.5};
            bool foundInitSol_{false};
            /** \brief Queue to order the nodes to update */
            BinaryHeap<Motion *, MotionCompare> q_;
            /** \brief Queue to order the relevant nodes for expansion */
            BinaryHeap<Motion *, relMotionCompare> relq_;
            /** \brief declare goal state */
            base::State *goalstate;
            /** \brief declare space bounds */

            /** \brief Threshold for the propagation of information */
            base::Cost epsilonCost_{0.};

            /** \brief Whether or not to propagate the cost to children if the update is less than epsilon */
            bool updateChildren_{true};

            /** \brief Current value of the radius used for the neighbors */
            double rrg_r_;

            /** \brief Current value of the number of neighbors used */
            unsigned int rrg_k_;

            /** \brief Variant used for rejection sampling */
            int variant_{0};

            /** \brief Alpha parameter, scaling the rejection sampling tests */
            double alpha_{1.};

            /** \brief Option to use informed sampling */
            bool useInformedSampling_{false};

            /** \brief The status of the sample rejection parameter. */
            bool useRejectionSampling_{false};

            /** \brief The number of attempts to make at informed sampling */
            unsigned int numSampleAttempts_{100u};

            /** \brief number of rewirings by algorithm */
            unsigned int rewireTest;

            ///////////////////////////////////////
            // Planner progress property functions
            std::string numIterationsProperty() const
            {
                return std::to_string(numIterations());
            }
            std::string bestCostProperty() const
            {
                return std::to_string(bestCost().value());
            }
            std::string rewireTestProperty() const
            {
                return std::to_string(rewireTest);
            }
            std::string numMotionsProperty() const
            {
                return std::to_string(nn_->size());
            }
        };
    }
}

#endif
