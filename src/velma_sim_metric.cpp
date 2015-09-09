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

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"
#include "Eigen/LU"

#include "velma_dyn_model.h"
#include <collision_convex_model/collision_convex_model.h>
#include "kin_model/kin_model.h"
#include "planer_utils/marker_publisher.h"
#include "planer_utils/task_col.h"
#include "planer_utils/task_hand.h"
#include "planer_utils/task_jlc.h"
#include "planer_utils/task_wcc.h"
#include "planer_utils/random_uniform.h"
#include "planer_utils/utilities.h"
#include "planer_utils/simulator.h"
#include "planer_utils/reachability_map.h"

#include "experiments_utilities.h"

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    MarkerPublisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

    KDL::Frame int_marker_pose_;

public:
    TestDynamicModel() :
        nh_(),
        PI(3.141592653589793),
        markers_pub_(nh_)
    {
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    }

    ~TestDynamicModel() {
    }

    void generatePossiblePose(KDL::Frame &T_B_E, Eigen::VectorXd &q, int ndof, const std::string &effector_name, const boost::shared_ptr<self_collision::CollisionModel> &col_model, const boost::shared_ptr<KinematicModel> &kin_model) {
        while (true) {
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                q(q_idx) = randomUniform(kin_model->getLowerLimit(q_idx), kin_model->getUpperLimit(q_idx));
            }
            std::set<int> excluded_link_idx;
            std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }

            if (!self_collision::checkCollision(col_model, links_fk, excluded_link_idx)) {
                T_B_E = links_fk[col_model->getLinkIndex(effector_name)];
                break;
            }
        }
    }

    bool checkCollision(const KDL::Vector &x, const boost::shared_ptr<self_collision::CollisionModel> &col_model) {
        // create dummy object
        boost::shared_ptr< self_collision::Collision > pcol = self_collision::createCollisionSphere(0.07, KDL::Frame(x));
        KDL::Frame T_B_L1;
        KDL::Frame T_B_L2;
        return self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);
    }

    void printJointLimits(const Eigen::VectorXd &q, const boost::shared_ptr<KinematicModel> &kin_model, const std::vector<std::string> &joint_names) const {
                    int ndof = q.innerSize();
                    for (int q_idx = 0; q_idx < ndof; q_idx++) {
                        double lo = kin_model->getLowerLimit(q_idx), up = kin_model->getUpperLimit(q_idx);
                        double f = (q(q_idx) - lo) / (up - lo);
                        int steps = 50;
                        int step = static_cast<int >(f*steps);
                        if (step >= steps) {
                            step = steps-1;
                        }
                        for (int s = 0; s < steps; s++) {
                            std::cout << ((s == step)?"*":".");
                        }
                        std::cout << "  ";

                        steps = 3;
                        step = static_cast<int >(f*steps);
                        if (step >= steps) {
                            step = steps-1;
                        }
                        for (int s = 0; s < steps; s++) {
                            std::cout << ((s == step)?"*":".");
                        }

                        std::cout << "    " << joint_names[q_idx] << std::endl;
                    }
    }

    KDL::Twist distanceMetric(const KDL::Frame &F_a_b1, const KDL::Frame &F_a_b2, const boost::shared_ptr<ReachabilityMap > &r_map) const {
        KDL::Twist diff = KDL::diff(F_a_b1, F_a_b2, 1.0);
        if (diff.vel.Norm() < 0.05) {
            return diff;
        }

        double dist = (F_a_b1.p - F_a_b2.p).Norm();
        KDL::Vector gr;
        if (r_map->getGradient(F_a_b1.p, gr)) {
            diff.vel = gr * dist;
//            int m_id = 6000;
//            m_id = markers_pub_.addVectorMarker(m_id, F_a_b1.p, F_a_b1.p + gr*0.3, 0, 0, 1, 1, 0.005, "world");
//            markers_pub_.publish();
//            ros::spinOnce();
        }


        return diff;
    }

    void spin() {
        // initialize random seed
        srand(time(NULL));

        // dynamics model
        boost::shared_ptr<DynamicModel > dyn_model( new DynModelVelma() );

        std::string robot_description_str;
        std::string robot_semantic_description_str;
        nh_.getParam("/robot_description", robot_description_str);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str);

        //
        // collision model
        //
        boost::shared_ptr<self_collision::CollisionModel> col_model = self_collision::CollisionModel::parseURDF(robot_description_str);
	    col_model->parseSRDF(robot_semantic_description_str);
        col_model->generateCollisionPairs();

        // external collision objects - part of virtual link connected to the base link
        self_collision::Link::VecPtrCollision col_array;

        KDL::Frame T_W_LOCK, T_W_BIN;
        createEnvironment(col_array, T_W_LOCK, T_W_BIN);

        if (!col_model->addLink("env_link", "torso_base", col_array)) {
            ROS_ERROR("ERROR: could not add external collision objects to the collision model");
            return;
        }
        col_model->generateCollisionPairs();

        //
        // robot state
        //
        std::vector<std::string > joint_names;
        joint_names.push_back("torso_0_joint");
        joint_names.push_back("right_arm_0_joint");
        joint_names.push_back("right_arm_1_joint");
        joint_names.push_back("right_arm_2_joint");
        joint_names.push_back("right_arm_3_joint");
        joint_names.push_back("right_arm_4_joint");
        joint_names.push_back("right_arm_5_joint");
        joint_names.push_back("right_arm_6_joint");
        joint_names.push_back("left_arm_0_joint");
        joint_names.push_back("left_arm_1_joint");
        joint_names.push_back("left_arm_2_joint");
        joint_names.push_back("left_arm_3_joint");
        joint_names.push_back("left_arm_4_joint");
        joint_names.push_back("left_arm_5_joint");
        joint_names.push_back("left_arm_6_joint");

        int ndof = joint_names.size();

        Eigen::VectorXd q_eq(ndof);
        Eigen::VectorXd q(ndof), dq(ndof), ddq(ndof), torque(ndof);
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q[q_idx] = 0.5;
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
            torque[q_idx] = 0.0;
        }

        double init_q[15] = {
        0.0,                // torso_0_joint
        90.0/180.0*PI,      // right_arm_0_joint
        -90.0/180.0*PI,     // right_arm_1_joint
        110.0/180.0*PI,     // right_arm_2_joint
        120.0/180.0*PI,     // right_arm_3_joint
        0.0/180.0*PI,       // right_arm_4_joint
        -40.0/180.0*PI,     // right_arm_5_joint
        0.0/180.0*PI,       // right_arm_6_joint
        90.0/180.0*PI,      // left_arm_0_joint
        90.0/180.0*PI,      // left_arm_1_joint
        -90.0/180.0*PI,     // left_arm_2_joint
        -90.0/180.0*PI,     // left_arm_3_joint
        0.0/180.0*PI,       // left_arm_4_joint
        90.0/180.0*PI,      // left_arm_5_joint
        -90.0/180.0*PI,     // left_arm_6_joint
        };

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q_eq(q_idx) = init_q[q_idx];
            q(q_idx) = init_q[q_idx];
        }

        std::string effector_name = "right_HandGripLink";
        int effector_idx = col_model->getLinkIndex(effector_name);

        //
        // kinematic model
        //
        boost::shared_ptr<KinematicModel > kin_model( new KinematicModel(robot_description_str, joint_names) );
        kin_model->setIgnoredJointValue("torso_1_joint", -90.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerOneKnuckleTwoJoint", 120.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerTwoKnuckleTwoJoint", 120.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerThreeKnuckleTwoJoint", 120.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerOneKnuckleThreeJoint", 40.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerTwoKnuckleThreeJoint", 40.0/180.0*PI);
        kin_model->setIgnoredJointValue("right_HandFingerThreeKnuckleThreeJoint", 40.0/180.0*PI);
        Eigen::VectorXd ign_q;
        std::vector<std::string > ign_joint_names;
        kin_model->getIgnoredJoints(ign_q, ign_joint_names);

        kin_model->setLowerLimit(0, -110.0/180.0*PI);
        kin_model->setUpperLimit(0, 110.0/180.0*PI);
        kin_model->setUpperLimit(2, -5.0/180.0*PI);
        kin_model->setLowerLimit(4, 5.0/180.0*PI);
        kin_model->setUpperLimit(6, -5.0/180.0*PI);
        kin_model->setLowerLimit(9, 5.0/180.0*PI);
        kin_model->setUpperLimit(11, -5.0/180.0*PI);
        kin_model->setLowerLimit(13, 5.0/180.0*PI);

        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());

        Eigen::VectorXd max_q(ndof);
        max_q(0) = 10.0/180.0*PI;               // torso_0_joint
        max_q(1) = max_q(8) = 20.0/180.0*PI;    // arm_0_joint
        max_q(2) = max_q(9) = 20.0/180.0*PI;    // arm_1_joint
        max_q(3) = max_q(10) = 30.0/180.0*PI;   // arm_2_joint
        max_q(4) = max_q(11) = 40.0/180.0*PI;   // arm_3_joint
        max_q(5) = max_q(12) = 50.0/180.0*PI;   // arm_4_joint
        max_q(6) = max_q(13) = 50.0/180.0*PI;   // arm_5_joint
        max_q(7) = max_q(14) = 50.0/180.0*PI;   // arm_6_joint

        boost::shared_ptr<DynamicsSimulatorHandPose> sim(new DynamicsSimulatorHandPose(ndof, 6, effector_name, col_model, kin_model, dyn_model, joint_names, q_eq, 10000.0*max_q ) );

        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        int loop_counter = 50000;
        ros::Rate loop_rate(500);

//        boost::shared_ptr<ReachabilityMap > r_map(new ReachabilityMap(0.025, 3));
        boost::shared_ptr<ReachabilityMap > r_map(new ReachabilityMap(0.04, 3));

        KDL::Vector lower_bound(0.0, -0.9, 0.3);
        KDL::Vector upper_bound(1.5, 0.9, 2.2);
        if (!r_map->createDistanceMap(KDL::Vector(1.05, 0.0, 1.35), boost::bind(&TestDynamicModel::checkCollision, this, _1, col_model), lower_bound, upper_bound)) {
            std::cout << "could not create the distance map" << std::endl;
        }
        else {
            std::cout << "created distance map" << std::endl;
        }

        sim->updateMetric( boost::bind(&TestDynamicModel::distanceMetric, this, _1, _2, r_map) );

        // TEST: distance metric
        if (false) {
            showMetric(lower_bound, upper_bound, kin_model, col_model, r_map, markers_pub_);
            return;
        }

        int_marker_pose_ = r_HAND_target;

        ros::Duration(1.0).sleep();

        std::list<Eigen::VectorXd > q_history_latest;
        std::list<Eigen::VectorXd > q_history;
        bool collision_in_prev_step = false;
        bool stop = false;
        while (ros::ok() && !stop) {

            if (loop_counter > 1500*4) {
                    sim->setState(q, dq, ddq);
                    double rand01 = randomUniform(0, 1.4);

                    if (rand01 > 1.2) {
                        r_HAND_target = T_W_LOCK * KDL::Frame(KDL::Rotation::RotZ(90.0/180.0*PI) * KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.0, -0.11, 0.0));
                    }
                    else if (rand01 > 1.0) {
                        r_HAND_target = T_W_BIN * KDL::Frame(KDL::Rotation::RotY(180.0/180.0*PI), KDL::Vector(0.0, 0.0, 0.2));
                    }
                    else if (rand01 > 0.8) {
                        r_HAND_target = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.05, 0.0, 1.35+0.3));
                    }
                    else if (rand01 > 0.6){
                        r_HAND_target = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.05, 0.0, 1.35));
                    }
                    else if (rand01 > 0.4){
                        r_HAND_target = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.15, -0.5, 1.35));
                    }
                    else if (rand01 > 0.2){
                        r_HAND_target = KDL::Frame(KDL::Rotation::RotY(45.0/180.0*PI), KDL::Vector(1.05, 0, 2.0));
                    }
                    else {
                        r_HAND_target = KDL::Frame(KDL::Rotation::RotZ(45.0/180.0*PI) * KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.8, 0.6, 1.8));
                    }
                    sim->setTarget(r_HAND_target);

                    if (!r_map->createDistanceMap(r_HAND_target.p, boost::bind(&TestDynamicModel::checkCollision, this, _1, col_model), lower_bound, upper_bound)) {
                        std::cout << "could not create the distance map" << std::endl;
                    }
                    else {
                        std::cout << "created distance map" << std::endl;
                    }

                    loop_counter = 0;
            }
            loop_counter += 1;
            publishTransform(br, r_HAND_target, "effector_dest", "world");

            sim->oneStep();//&markers_pub_, 3000);
            if (sim->inCollision() && !collision_in_prev_step) {
                collision_in_prev_step = true;
                std::cout << "collision begin" << std::endl;
                printJointLimits(q, kin_model, joint_names);
//                stop = true;
            }
            else if (!sim->inCollision() && collision_in_prev_step) {
                collision_in_prev_step = false;
                std::cout << "collision end" << std::endl;
            }

            sim->getState(q, dq, ddq);
            KDL::Frame current_T_B_E;
            kin_model->calculateFk(current_T_B_E, effector_name, q);
            KDL::Twist diff = KDL::diff(r_HAND_target, current_T_B_E);
            if (diff.vel.Norm() < 0.015 && diff.rot.Norm() < 5.0/180.0*PI) {
                std::cout << "goal reached" << std::endl;
                loop_counter = 100000;
            }

            // publish markers and robot state with limited rate
            ros::Duration time_elapsed = ros::Time::now() - last_time;
            if (time_elapsed.toSec() > 0.05) {
                // calculate forward kinematics for all links
                for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                    kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
                }
                publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
                int m_id = 0;
                m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
                markers_pub_.publish();
//                markers_pub_.clear();
                last_time = ros::Time::now();
//                ros::spinOnce();
//                loop_rate.sleep();
            }
            else {
                markers_pub_.clear();
            }
            ros::spinOnce();
//            loop_rate.sleep();
//            ros::Duration(1.0).sleep();
//            return;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}


