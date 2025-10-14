#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/Planner.h>
#include <vector>
#include <memory>
#include "PyBulletInterface.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * @brief Manipulator motion planner using OMPL
 * 
 * This class provides motion planning capabilities for robotic manipulators
 * using the Open Motion Planning Library (OMPL).
 */
class ManipulatorPlanner {
public:
    /**
     * @brief Constructor
     * @param simulator PyBullet interface for collision checking
     * @param start Start configuration
     * @param goals Goal configurations
     * @param planning_time Maximum planning time in seconds
     */
    ManipulatorPlanner(std::shared_ptr<PyBulletInterface> simulator,
                      const std::vector<double>& start,
                      const std::vector<std::vector<double>>& goals,
                      double planning_time = 10.0);
    
    /**
     * @brief Set the planner to use
     * @param planner OMPL planner instance
     */
    void setPlanner(ob::PlannerPtr planner);

    // STUDENT TODO START
    
    /**
     * @brief Set RTP planner parameters
     * @param goal_bias Goal bias probability (0.0 to 1.0)
     */
    void setRTPParameters(double goal_bias);

    // STUDENT TODO END
    
    /**
     * @brief Set RRT planner parameters
     * @param goal_bias Goal bias probability (0.0 to 1.0)
     * @param range Maximum extension range
     */
    void setRRTParameters(double goal_bias, double range);
    
    /**
     * @brief Set PRM planner parameters
     * @param max_neighbors Maximum number of neighbors to connect
     * @param connection_radius Connection radius for roadmap
     */
    void setPRMParameters(int max_neighbors);
    
    /**
     * @brief Solve the motion planning problem
     * @return Path as vector of configurations, empty if no solution found
     */
    std::vector<std::vector<double>> solve();
    
    /**
     * @brief Solve and process path (simplify, interpolate, validate)
     * @return Processed path as vector of configurations
     */
    std::vector<std::vector<double>> process();
    
    /**
     * @brief Get the space information object
     * @return Pointer to space information
     */
    ob::SpaceInformationPtr getSpaceInformation() const { return si_; }
    
    /**
     * @brief Get the simple setup object
     * @return Pointer to simple setup
     */
    og::SimpleSetupPtr getSimpleSetup() const { return ss_; }

private:
    /**
     * @brief State validity checker function
     * @param state State to check
     * @return true if state is valid, false otherwise
     */
    bool isStateValid(const ob::State* state);
    
    /**
     * @brief Convert OMPL state to configuration vector
     * @param state OMPL state
     * @return Configuration vector
     */
    std::vector<double> stateToConfig(const ob::State* state);
    
    /**
     * @brief Set OMPL state from configuration vector
     * @param state OMPL state to set
     * @param config Configuration vector
     */
    void configToState(ob::State* state, const std::vector<double>& config);

private:
    std::shared_ptr<PyBulletInterface> simulator_;
    double planning_time_;
    int n_dims_;
    
    ob::StateSpacePtr space_;
    ob::SpaceInformationPtr si_;
    og::SimpleSetupPtr ss_;
    ob::PlannerPtr current_planner_;
}; 