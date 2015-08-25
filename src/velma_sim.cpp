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

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    MarkerPublisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

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
        col_array.push_back( self_collision::createCollisionCapsule(0.3, 1.3, KDL::Frame(KDL::Vector(1, 0, 1))) );
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

        Eigen::VectorXd q, dq, ddq, torque;
        q.resize( ndof );
        dq.resize( ndof );
        ddq.resize( ndof );
        torque.resize( ndof );
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q[q_idx] = 0.5;
            dq[q_idx] = -1.0;
            ddq[q_idx] = 0.0;
            torque[q_idx] = 0.0;
        }

        std::string effector_name = "right_HandPalmLink";
        int effector_idx = col_model->getLinkIndex(effector_name);

        //
        // kinematic model
        //
        boost::shared_ptr<KinematicModel > kin_model( new KinematicModel(robot_description_str, joint_names) );
        kin_model->setIgnoredJointValue("torso_1_joint", -90.0/180.0*PI);
        Eigen::VectorXd ign_q;
        std::vector<std::string > ign_joint_names;
        kin_model->getIgnoredJoints(ign_q, ign_joint_names);

        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());

        // joint limits
        Eigen::VectorXd lower_limit(ndof), upper_limit(ndof), limit_range(ndof), max_trq(ndof);
        int q_idx = 0;
        for (std::vector<std::string >::const_iterator name_it = joint_names.begin(); name_it != joint_names.end(); name_it++, q_idx++) {

            if (!col_model->getJointLimits( (*name_it), lower_limit[q_idx], upper_limit[q_idx] )) {
                ROS_ERROR("ERROR: could not find joint with name %s", name_it->c_str() );
                return;
            }
            limit_range[q_idx] = 15.0/180.0*PI;
            max_trq[q_idx] = 30.0;
        }

        KinematicModel::Jacobian J_r_HAND;
        J_r_HAND.resize(6, ndof);

        //
        // Tasks declaration
        //
        std::set<int > jlc_excluded_q_idx;
        jlc_excluded_q_idx.insert(6);
        jlc_excluded_q_idx.insert(7);
        jlc_excluded_q_idx.insert(13);
        jlc_excluded_q_idx.insert(14);
        Task_JLC task_JLC(lower_limit, upper_limit, limit_range, max_trq, jlc_excluded_q_idx);
        Task_WCC task_WCCr(ndof, 6, 7, false);
        Task_WCC task_WCCl(ndof, 13, 14, true);
        double activation_dist = 0.05;
        Task_COL task_COL(ndof, activation_dist, 10.0, kin_model, col_model);
        Task_HAND task_HAND(ndof, 6);

        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        int loop_counter = 10000;
        ros::Rate loop_rate(200);
/*
        while (ros::ok()) {
            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }

            publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
            int m_id = 0;
            m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
            markers_pub_.publish();
            ros::spinOnce();
            loop_rate.sleep();
        }

        return;
*/
        while (ros::ok()) {

            if (loop_counter > 1500) {
                r_HAND_target = KDL::Frame(KDL::Rotation::RotZ(randomUniform(-PI, PI)) * KDL::Rotation::RotY(randomUniform(-PI, PI)) * KDL::Rotation::RotX(randomUniform(-PI, PI)), KDL::Vector(randomUniform(-1,1), randomUniform(-1,1), randomUniform(1.3,2)));

                publishTransform(br, r_HAND_target, "effector_dest", "world");
                loop_counter = 0;
            }
            loop_counter += 1;

            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }
            dyn_model->computeM(q);

            //
            // joint limit avoidance
            //
            Eigen::VectorXd torque_JLC(ndof);
            Eigen::MatrixXd N_JLC(ndof, ndof);
            task_JLC.compute(q, dq, dyn_model->getInvM(), torque_JLC, N_JLC);

            //
            // wrist collision constraint
            //
            Eigen::VectorXd torque_WCCr(ndof);
            Eigen::MatrixXd N_WCCr(ndof, ndof);
            task_WCCr.compute(q, dq, dyn_model->getM(), dyn_model->getInvM(), torque_WCCr, N_WCCr, &markers_pub_);

            Eigen::VectorXd torque_WCCl(ndof);
            Eigen::MatrixXd N_WCCl(ndof, ndof);
            task_WCCl.compute(q, dq, dyn_model->getM(), dyn_model->getInvM(), torque_WCCl, N_WCCl, NULL);

//            std::cout << torque_WCC.transpose() << std::endl;

            //
            // collision constraints
            //
            std::vector<self_collision::CollisionInfo> link_collisions;
            self_collision::getCollisionPairs(col_model, links_fk, activation_dist, link_collisions);
//            std::cout << (long int)link_collisions.size() << std::endl;

            {
                    int m_id = 1000;
                    for (std::vector<self_collision::CollisionInfo>::const_iterator it = link_collisions.begin(); it != link_collisions.end(); it++) {
                        m_id = markers_pub_.addVectorMarker(m_id, it->p1_B, it->p2_B, 1, 1, 1, 1, 0.01, "world");
                        m_id = markers_pub_.addVectorMarker(m_id, it->p1_B, it->p1_B - 0.03 * it->n1_B, 1, 1, 1, 1, 0.01, "world");
                        m_id = markers_pub_.addVectorMarker(m_id, it->p2_B, it->p2_B - 0.03 * it->n2_B, 1, 1, 1, 1, 0.01, "world");
                    }
                    markers_pub_.addEraseMarkers(m_id, m_id+100);

            }

            Eigen::VectorXd torque_COL(ndof);
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                torque_COL[q_idx] = 0.0;
            }
            Eigen::MatrixXd N_COL(Eigen::MatrixXd::Identity(ndof, ndof));

            task_COL.compute(q, dq, dyn_model->getInvM(), links_fk, link_collisions, torque_COL, N_COL);

            //
            // effector task
            //
            Eigen::VectorXd torque_HAND(ndof);
            Eigen::MatrixXd N_HAND(ndof, ndof);

            KDL::Frame T_B_E = links_fk[effector_idx];
            KDL::Frame r_HAND_current = T_B_E;

            KDL::Twist diff = KDL::diff(r_HAND_current, r_HAND_target, 1.0);
            Eigen::VectorXd r_HAND_diff(6);
            for (int i=0; i<6; i++) {
                r_HAND_diff[i] = diff[i];
            }

            KDL::Twist diff_goal = KDL::diff(r_HAND_current, r_HAND_target, 1.0);

            if (diff_goal.vel.Norm() < 0.06 && diff.rot.Norm() < 5.0/180.0 * PI) {
            }

            Eigen::VectorXd Kc(6);
            for (int i=0; i<3; i++) {
                Kc[i] = 2.0;
                Kc[i+3] = 0.3;
            }
            Eigen::VectorXd Dxi(6);
            for (int i=0; i<6; i++) {
                Dxi[i] = 0.7;
            }

            kin_model->getJacobian(J_r_HAND, effector_name, q);

            task_HAND.compute(r_HAND_diff, Kc, Dxi, J_r_HAND, dq, dyn_model->getInvM(), torque_HAND, N_HAND);

//            torque = torque_JLC + N_JLC.transpose() * (torque_WCC + N_WCC.transpose() * (torque_COL + (N_COL.transpose() * (torque_HAND))));
            torque = (torque_JLC + torque_WCCr + torque_WCCl) + (N_JLC * N_WCCr * N_WCCl).transpose() * (torque_COL + (N_COL.transpose() * (torque_HAND)));

            // simulate one step
            Eigen::VectorXd prev_ddq(ddq), prev_dq(dq);
            dyn_model->accel(ddq, q, dq, torque);
            float time_d = 0.005;
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                dq[q_idx] += (prev_ddq[q_idx] + ddq[q_idx]) / 2.0 * time_d;
                q[q_idx] += (prev_dq[q_idx] + dq[q_idx]) / 2.0 * time_d;
            }

            // publish markers and robot state with limited rate
            ros::Duration time_elapsed = ros::Time::now() - last_time;
            if (time_elapsed.toSec() > 0.05) {
                publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
                int m_id = 0;
                m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
                markers_pub_.publish();
//                markers_pub_.clear();
                last_time = ros::Time::now();
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}


