// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#ifndef EXPERIMENT_UTILITIES_H__
#define EXPERIMENT_UTILITIES_H__

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"

#include <collision_convex_model/collision_convex_model.h>
#include "kin_model/kin_model.h"
#include "planer_utils/marker_publisher.h"
#include "planer_utils/reachability_map.h"

class TestScenario {
public:

    void addNode(const KDL::Frame &T_W_G_dest, const Eigen::VectorXd &q_init, bool use_prev_q);

    void startTest();
    void nextNode();
    bool isFinished() const;

    int getNodeId() const;
    int getNodes() const;
    const KDL::Frame& getDestFrame() const;
    const Eigen::VectorXd& getInitQ() const;
    bool usePrevQ() const;

protected:

    class TestNode {
    public:
        KDL::Frame T_W_G_dest_;
        Eigen::VectorXd q_init_;
        bool use_prev_q_;
    };

    std::list<TestNode> nodes_list_;
    std::list<TestNode>::const_iterator nodes_it_;
    int nodeId_;
};

class TestResults {
public:
    void addResult(const std::string &planner_name, int nodeId, bool solutionFound, bool inCollision, double planningTime, double length);

    double getTotalMeanPathLength(const std::string &planner_name, int nodeId) const;
    double getTotalMeanPlanningTime(const std::string &planner_name, int nodeId) const;

    double getSuccessMeanPathLength(const std::string &planner_name, int nodeId) const;
    double getSuccessMeanPlanningTime(const std::string &planner_name, int nodeId) const;

    double getSuccessRate(const std::string &planner_name, int nodeId) const;

    void getTries(const std::string &planner_name, int nodeId, std::vector<bool> &solutionFoundVec, std::vector<bool> &inCollisionVec, std::vector<double> &planningTimeVec, std::vector<double> &lengthVec) const;

protected:

    class SingleResult {
    public:
        bool solutionFound_;
        bool inCollision_;
        double planningTime_;
        double length_;
        int nodeId_;
    };
    std::map<std::string, std::vector<SingleResult > > results_;
};

void showMetric(const KDL::Vector &lower_bound, const KDL::Vector &upper_bound,
                    const boost::shared_ptr<KinematicModel> &kin_model, const boost::shared_ptr<self_collision::CollisionModel> &col_model,
                    const boost::shared_ptr<ReachabilityMap > &r_map, MarkerPublisher &markers_pub);
void generateBox(std::vector<KDL::Vector > &vertices, std::vector<int> &polygons, double size_x, double size_y, double size_z);
void createEnvironment(self_collision::Link::VecPtrCollision &col_array, KDL::Frame &T_W_LOCK, KDL::Frame &T_W_BIN);

bool randomizedIkSolution(const boost::shared_ptr<KinematicModel> &kin_model, const KDL::Frame &T_W_G_dest, Eigen::VectorXd &q);

#endif  // EXPERIMENT_UTILITIES_H__

