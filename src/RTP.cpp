///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "RTP.h"
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <algorithm>
#include <limits>
#include <functional>
#include <random>

namespace ompl
{
    namespace geometric
    {
        RTP::RTP(const base::SpaceInformationPtr &si) 
            : base::Planner(si, "RTP"), root_(nullptr), goalBias_(0.05), maxDistance_(0.1)
        {
            specs_.approximateSolutions = true;
            specs_.directed = true;
        }

        RTP::~RTP()
        {
            clear();
        }

        void RTP::clear()
        {
            Planner::clear();
            freeTree(root_);
            root_ = nullptr;
        }

        void RTP::setup()
        {
            Planner::setup();
            if (!root_) {
                root_ = new Node(si_->allocState());
            }
        }

        base::PlannerStatus RTP::solve(const base::PlannerTerminationCondition &ptc)
        {
            checkValidity();
            
            // Get start and goal states
            base::State *startState = pis_.nextStart();
            if (!startState) {
                return base::PlannerStatus::INVALID_START;
            }
            
            base::Goal *goal = pdef_->getGoal().get();
            if (!goal) {
                return base::PlannerStatus::INVALID_GOAL;
            }

            // Initialize tree with start state
            si_->copyState(root_->state, startState);
            
            // Allocate state for planning
            base::State *randomState = si_->allocState();
            
            while (!ptc)
            {
                // RTP Step 1: Pick a random existing node from the tree
                Node *qa = pickRandomNode();
                
                // RTP Step 2: Sample a random configuration (with goal bias)
                auto *gsr = dynamic_cast<base::GoalSampleableRegion*>(goal);
                if (gsr && rng_.uniform01() < goalBias_) {
                    gsr->sampleGoal(randomState);
                } else {
                    si_->sampleUniform(randomState);
                }
                
                // RTP Step 3: Try straight-line edge qa -> qb
                base::State *qbTry = si_->allocState();
                si_->copyState(qbTry, randomState);
                
                // Optionally clamp by maxDistance_ if required
                if (maxDistance_ > 0.0) {
                    double d = si_->distance(qa->state, qbTry);
                    if (d > maxDistance_) {
                        si_->getStateSpace()->interpolate(qa->state, qbTry, maxDistance_/d, qbTry);
                    }
                }
                
                // Check if motion is valid
                if (si_->checkMotion(qa->state, qbTry)) {
                    // Create new node and add to tree
                    Node *newNode = new Node(qbTry);
                    addNode(qa, newNode);
                    
                    // Check if we reached the goal
                    if (goal->isSatisfied(newNode->state)) {
                        // Construct and return path
                        base::PathPtr path = constructPath(newNode);
                        pdef_->addSolutionPath(path, false, 0.0, getName());
                        
                        si_->freeState(randomState);
                        return base::PlannerStatus::EXACT_SOLUTION;
                    }
                } else {
                    si_->freeState(qbTry);
                }
            }
            
            si_->freeState(randomState);
            return base::PlannerStatus::TIMEOUT;
        }

        void RTP::getPlannerData(base::PlannerData &data) const
        {
            Planner::getPlannerData(data);
            
            std::vector<Node*> allNodes;
            std::function<void(Node*)> addNodes = [&](Node *node) {
                if (node) {
                    allNodes.push_back(node);
                    for (Node *child : node->children) {
                        addNodes(child);
                    }
                }
            };
            addNodes(root_);
            
            for (Node *node : allNodes) {
                data.addVertex(base::PlannerDataVertex(node->state));
            }
            
            for (Node *node : allNodes) {
                if (node->parent) {
                    data.addEdge(base::PlannerDataVertex(node->parent->state),
                                base::PlannerDataVertex(node->state));
                }
            }
        }

        RTP::Node* RTP::pickRandomNode() const
        {
            // Collect all nodes in the tree
            std::vector<Node*> nodes;
            std::function<void(Node*)> dfs = [&](Node *node) {
                if (node) {
                    nodes.push_back(node);
                    for (Node *child : node->children) {
                        dfs(child);
                    }
                }
            };
            dfs(root_);
            
            // Pick a random node uniformly
            if (nodes.empty()) {
                return root_;
            }
            
            std::uniform_int_distribution<size_t> uid(0, nodes.size() - 1);
            return nodes[uid(rng_)];
        }

        void RTP::addNode(Node *parent, Node *child)
        {
            child->parent = parent;
            parent->children.push_back(child);
        }

        void RTP::freeTree(Node *node)
        {
            if (node) {
                for (Node *child : node->children) {
                    freeTree(child);
                }
                if (node->state) {
                    si_->freeState(node->state);
                }
                delete node;
            }
        }

        base::PathPtr RTP::constructPath(Node *goalNode) const
        {
            auto path = std::make_shared<PathGeometric>(si_);
            
            std::vector<base::State*> states;
            Node *current = goalNode;
            
            while (current) {
                states.push_back(current->state);
                current = current->parent;
            }
            
            std::reverse(states.begin(), states.end());
            
            for (base::State *state : states) {
                path->append(state);
            }
            
            return path;
        }

    }  // namespace geometric
}  // namespace ompl
