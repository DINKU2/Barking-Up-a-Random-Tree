///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/PathGeometric.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

void planPoint(const std::vector<Rectangle> &obstacles)
{
    std::cout << "Planning for a point robot..." << std::endl;
    
    // Create the state space for a point robot (2D)
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    
    // Set bounds for the workspace (adjust as needed)
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10.0);  // Lower bound for x and y
    bounds.setHigh(10.0);  // Upper bound for x and y
    space->setBounds(bounds);
    
    // Create space information
    auto si = std::make_shared<ompl::base::SpaceInformation>(space);
    
    // Set state validity checker for point robot
    si->setStateValidityChecker([&obstacles](const ompl::base::State *state) {
        bool valid = isValidStatePoint(state, obstacles);
        const auto* r2State = state->as<ompl::base::RealVectorStateSpace::StateType>();
        if (!valid) {
            std::cout << "Invalid state: (" << r2State->values[0] << ", " << r2State->values[1] << ")" << std::endl;
        }
        return valid;
    });
    
    // Create problem definition
    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
    
    // Set start state (clear area, closer to goal)
    ompl::base::ScopedState<> start(space);
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = -5.0;  // x
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = -5.0;  // y
    pdef->addStartState(start);
    
    // Set goal state (clear area, closer to start)
    ompl::base::ScopedState<> goal(space);
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 5.0;   // x
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 5.0;   // y
    pdef->setGoalState(goal);
    
    
    // Create and configure RTP planner
    auto planner = std::make_shared<ompl::geometric::RTP>(si);
    planner->setGoalBias(0.3);      // 30% goal bias (higher for easier solution)
    planner->setMaxDistance(1.0);   // Larger step size
    planner->setProblemDefinition(pdef);
    planner->setup();
    
    // Attempt to solve the problem
    std::cout << "Attempting to find a path..." << std::endl;
    std::cout << "Start: (-5, -5), Goal: (5, 5)" << std::endl;
    ompl::base::PlannerStatus solved = planner->solve(ompl::base::timedPlannerTerminationCondition(5.0));  // 5 second timeout
    
    if (solved)
    {
        std::cout << "Found solution!" << std::endl;
        
        // Get the solution path
        auto path = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
        
        // Print path information
        std::cout << "Path length: " << path->length() << std::endl;
        std::cout << "Number of waypoints: " << path->getStateCount() << std::endl;
        
        // Print waypoints
        std::cout << "Path waypoints:" << std::endl;
        for (size_t i = 0; i < path->getStateCount(); ++i)
        {
            const auto* state = path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            std::cout << "  (" << state->values[0] << ", " << state->values[1] << ")" << std::endl;
        }
        
        // Save path to file for visualization
        std::ofstream pathFile("point_robot_path.txt");
        if (pathFile.is_open())
        {
            for (size_t i = 0; i < path->getStateCount(); ++i)
            {
                const auto* state = path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
                pathFile << state->values[0] << " " << state->values[1] << std::endl;
            }
            pathFile.close();
            std::cout << "Path saved to point_robot_path.txt" << std::endl;
        }
    }
    else
    {
        std::cout << "No solution found!" << std::endl;
    }
}

void planBox(const std::vector<Rectangle> &obstacles)
{
    // TODO: Use your implementation of RTP to plan for a rotating square robot.
}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    std::cout << "Creating Environment 1: Simple corridor with obstacles" << std::endl;
    
    // Clear any existing obstacles
    obstacles.clear();
    
    // Create a simple environment with a few obstacles
    // Central wall with gap
    obstacles.push_back({-2.0, -1.0, 1.0, 2.0});  // Left wall
    obstacles.push_back({1.0, -1.0, 1.0, 2.0});   // Right wall
    
    // Additional obstacles
    obstacles.push_back({-5.0, 2.0, 2.0, 1.0});   // Top obstacle
    obstacles.push_back({3.0, -5.0, 1.0, 2.0});   // Bottom obstacle
    
    std::cout << "Environment 1 created with " << obstacles.size() << " obstacles" << std::endl;
}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your second environment.
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;
        std::cout << "Enter choice (1 or 2): ";
        std::cout.flush();

        if (!(std::cin >> robot)) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            robot = 0;
        }
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) Simple corridor with obstacles" << std::endl;
        std::cout << " (2) TODO" << std::endl;
        std::cout << "Enter choice (1 or 2): ";
        std::cout.flush();

        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            choice = 0;
        }
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
