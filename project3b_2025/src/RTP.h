///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Dinuk & Zarek!!
//////////////////////////////////////

#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

#include <ompl/base/Planner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/RandomNumbers.h>
#include <vector>
#include <memory>

namespace ompl
{
    namespace geometric
    {
        class RTP : public base::Planner
        {
        public:
            RTP(const base::SpaceInformationPtr &si, double goal_bias = 0.05);
            virtual ~RTP();

            virtual void clear() override;
            virtual void setup() override;
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            virtual void getPlannerData(base::PlannerData &data) const override;

            // RTP specific parameters
            void setGoalBias(double goalBias) { goalBias_ = goalBias; }
            double getGoalBias() const { return goalBias_; }

            void setMaxDistance(double maxDistance) { maxDistance_ = maxDistance; }
            double getMaxDistance() const { return maxDistance_; }

        protected:
            // Node structure for the tree
            struct Node
            {
                base::State *state;
                Node *parent;
                std::vector<Node*> children;
                
                Node(base::State *s) : state(s), parent(nullptr) {}
                ~Node() = default; // Don't free state here - handled in freeTree()
            };

            // Helper functions
            Node* pickRandomNode() const;  // RTP: pick random existing node
            void addNode(Node *parent, Node *child);
            void freeTree(Node *node);
            base::PathPtr constructPath(Node *goalNode) const;

            // Tree root
            Node *root_;
            
            // Parameters
            double goalBias_;
            double maxDistance_;
            
            // Random number generator
            RNG rng_;
        };

    }  // namespace geometric
}  // namespace ompl

#endif
