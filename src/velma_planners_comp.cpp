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
#include <boost/bind.hpp>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"
#include "Eigen/LU"

#include "velma_dyn_model.h"
#include <collision_convex_model/collision_convex_model.h>
#include "kin_model/kin_model.h"
#include "planer_utils/marker_publisher.h"
#include "planer_utils/utilities.h"
#include "planer_utils/random_uniform.h"
#include "planer_utils/reachability_map.h"
#include "planer_utils/rrt_star.h"
#include "planer_utils/simulator.h"
#include "rrt.h"

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    MarkerPublisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

    std::list<Eigen::VectorXd > penalty_points_;

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

    double costLine(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const boost::shared_ptr<ReachabilityMap > &r_map) const {
        return (x1-x2).norm() * (2.0 - r_map->getValue(x1) - r_map->getValue(x2));
    }

    double costLine2(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const boost::shared_ptr<ReachabilityMap > &r_map) const {
        Eigen::VectorXd v = x2-x1;
        double norm = v.norm();
        double step_len = 0.05;

        if (r_map->getMaxValue() == 0.0) {
            return 1.0 * norm;
        }
        return ((r_map->getMaxValue() * r_map->getValue((x1+x2)/2.0)) + 1.0) * norm;
    }

    bool checkCollision(const Eigen::VectorXd &x, const boost::shared_ptr<self_collision::CollisionModel> &col_model) {
        // create dummy object
        boost::shared_ptr< self_collision::Collision > pcol = self_collision::createCollisionSphere(0.1, KDL::Frame(KDL::Vector(x[0], x[1], x[2])));
        KDL::Frame T_B_L1;
        KDL::Frame T_B_L2;
        self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("torso_base")), T_B_L2);
        self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);

        return self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("torso_base")), T_B_L2) || self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);
    }

    bool checkCollision2(const KDL::Frame &x, const boost::shared_ptr<self_collision::CollisionModel> &col_model) {
        // create dummy object
        boost::shared_ptr< self_collision::Collision > pcol = self_collision::createCollisionSphere(0.1, x);
        KDL::Frame T_B_L1;
        KDL::Frame T_B_L2;
        self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("torso_base")), T_B_L2);
        self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);

        return self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("torso_base")), T_B_L2) || self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);
    }

    void sampleSpace(KDL::Frame &x, const Eigen::VectorXd &lower_bound, const Eigen::VectorXd &upper_bound) {
        Eigen::VectorXd quat(4);
        randomUnitQuaternion(quat);
        x = KDL::Frame( KDL::Rotation::Quaternion(quat(0), quat(1), quat(2), quat(3)), KDL::Vector(randomUniform(lower_bound(0), upper_bound(0)), randomUniform(lower_bound(1), upper_bound(1)), randomUniform(lower_bound(2), upper_bound(2))) );
    }

    void generatePossiblePose(KDL::Frame &T_B_E, Eigen::VectorXd &q, int ndof, const std::string &effector_name, const boost::shared_ptr<self_collision::CollisionModel> &col_model, const boost::shared_ptr<KinematicModel> &kin_model) {
        while (true) {
//            Eigen::VectorXd q(ndof);
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

    void generateBox(std::vector<KDL::Vector > &vertices, std::vector<int> &polygons, double size_x, double size_y, double size_z) {
        vertices.clear();
        polygons.clear();
        const int poly[] = {
        4, 0, 1, 2, 3,
        4, 4, 7, 6, 5,
        4, 0, 4, 5, 1,
        4, 3, 2, 6, 7,
        4, 5, 6, 2, 1,
        4, 0, 3, 7, 4,
        };
        const double vert[] = {
        0.5, -0.5, 0.5,
        0.5, 0.5, 0.5,
        -0.5, 0.5, 0.5,
        -0.5, -0.5, 0.5,
        0.5, -0.5, -0.5,
        0.5, 0.5, -0.5,
        -0.5, 0.5, -0.5,
        -0.5, -0.5, -0.5,
        };
        for (int i=0; i<8; i++) {
            vertices.push_back(KDL::Vector(vert[i*3]*size_x, vert[i*3+1]*size_y, vert[i*3+2]*size_z));
        }
        for (int i=0; i<6*5; i++) {
            polygons.push_back(poly[i]);
        }
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

        // the walls
        std::vector<KDL::Vector > vertices;
        std::vector<int > polygons;
        generateBox(vertices, polygons, 0.2, 2.0, 2.0);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, KDL::Frame(KDL::Vector(-0.65, 0.0, 1.3))) );

        generateBox(vertices, polygons, 2.0, 0.2, 2.0);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, KDL::Frame(KDL::Vector(0.35, 1.0, 1.3))) );

        generateBox(vertices, polygons, 2.0, 2.0, 0.2);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, KDL::Frame(KDL::Vector(0.35, 0.0, 2.3))) );

        // the cabinet
        KDL::Frame T_W_C(KDL::Vector(1.2,0,1.5));
        generateBox(vertices, polygons, 0.4, 0.6, 0.02);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0,-0.3))) );
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0,0.0))) );
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0,0.3))) );

        generateBox(vertices, polygons, 0.02, 0.6, 0.6);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0.2,0,0))) );

        generateBox(vertices, polygons, 0.4, 0.02, 0.6);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,-0.3,0))) );
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0.3,0))) );

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

        Eigen::VectorXd saved_q(ndof), saved_dq(ndof), saved_ddq(ndof);
        saved_q.resize( ndof );
        saved_dq.resize( ndof );
        saved_ddq.resize( ndof );

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            saved_q[q_idx] = -0.5;
            saved_dq[q_idx] = 0.0;
            saved_ddq[q_idx] = 0.0;
        }

        saved_q[2] = 1.0;
//        saved_q[11] = -1.5;

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

//        kin_model->setUpperLimit(2, -10.0/180.0*PI);
//        kin_model->setLowerLimit(2, 10.0/180.0*PI);
//        kin_model->setLowerLimit(4, 10.0/180.0*PI);
//        kin_model->setUpperLimit(6, -10.0/180.0*PI);

        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());

        ros::Duration(1.0).sleep();

        Eigen::VectorXd lower_bound(3);
        Eigen::VectorXd upper_bound(3);
        lower_bound(0) = -0.2;
        upper_bound(0) = 1.3;
        lower_bound(1) = -1.0;
        upper_bound(1) = 1.0;
        lower_bound(2) = 1.0;
        upper_bound(2) = 2.0;

        Eigen::VectorXd max_q(ndof);
        max_q(0) = 10.0/180.0*PI;               // torso_0_joint
        max_q(1) = max_q(8) = 20.0/180.0*PI;    // arm_0_joint
        max_q(2) = max_q(9) = 20.0/180.0*PI;    // arm_1_joint
        max_q(3) = max_q(10) = 30.0/180.0*PI;   // arm_2_joint
        max_q(4) = max_q(11) = 40.0/180.0*PI;   // arm_3_joint
        max_q(5) = max_q(12) = 50.0/180.0*PI;   // arm_4_joint
        max_q(6) = max_q(13) = 50.0/180.0*PI;   // arm_5_joint
        max_q(7) = max_q(14) = 50.0/180.0*PI;   // arm_6_joint

        boost::shared_ptr<DynamicsSimulatorHandPose> sim( new DynamicsSimulatorHandPose(ndof, 6, effector_name, col_model, kin_model, dyn_model, joint_names, max_q) );
        sim->setState(saved_q, saved_dq, saved_ddq);

        RRT rrt(ndof, boost::bind(&TestDynamicModel::checkCollision2, this, _1, col_model),
//                boost::bind(&TestDynamicModel::generatePossiblePose, this, _1, ndof, effector_name, col_model, kin_model),
                boost::bind(&TestDynamicModel::sampleSpace, this, _1, lower_bound, upper_bound),
                0.05, 0.5, 0.8, kin_model, effector_name, sim);

        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        ros::Rate loop_rate(100);

/*
        while (ros::ok()) {
            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), saved_q);
            }
            r_HAND_target = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.85, 0.0, 1.35));
            publishTransform(br, r_HAND_target, "effector_dest", "world");

            publishJointState(joint_state_pub_, saved_q, joint_names, ign_q, ign_joint_names);
            int m_id = 0;
            m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
            markers_pub_.publish();
            ros::spinOnce();
            loop_rate.sleep();
        }

        return;
//*/

        while (ros::ok()) {

            // generate valid starting pose
            while (true) {
                generatePossiblePose(r_HAND_target, saved_q, ndof, effector_name, col_model, kin_model);
                saved_dq.setZero();
                saved_ddq.setZero();
                sim->setState(saved_q, saved_dq, saved_ddq);
                sim->setTarget(r_HAND_target);
                sim->oneStep();
                if (!sim->inCollision()) {
                    break;
                }
            }
            std::cout << saved_q.transpose() << " ";

            int successes = 0;
            // try n times to find the path to goal
            for (int try_idx=0; try_idx<5; try_idx++) {
                Eigen::VectorXd q_tmp(ndof);
                generatePossiblePose(r_HAND_target, q_tmp, ndof, effector_name, col_model, kin_model);
                r_HAND_target = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.00, 0.0, 1.34));
                publishTransform(br, r_HAND_target, "effector_dest", "world");

                // get the current pose
                sim->setState(saved_q, saved_dq, saved_ddq);

                // calculate forward kinematics for all links
                for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                    kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), saved_q);
                }
                publishJointState(joint_state_pub_, saved_q, joint_names, ign_q, ign_joint_names);
                int m_id = 1000;
                m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
                markers_pub_.publish();
                ros::spinOnce();

                std::list<KDL::Frame > path_x;
                std::list<Eigen::VectorXd > path_q;
                rrt.plan(saved_q, r_HAND_target, 0.05, &path_x, &path_q, markers_pub_);

                if (path_x.size() > 0) {
                    successes++;
                    double path_length = getPathLength(path_q);

//                    std::cout << "planning ended x: " << path_x.size() << "  q: " << path_q.size() << "  lq: " << path_length << std::endl;
/*
                    for (double d = 0.0; d < 1.0; d += 0.01/path_length) {
                        Eigen::VectorXd q(ndof);
                        getPointOnPath(path_q, d, q);
                        saved_q = q;
                        // calculate forward kinematics for all links
                        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                            kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
                        }
                        publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
                        int m_id = 1000;
                        m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
                        markers_pub_.publish();
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
*/
                }

    //                getchar();
                markers_pub_.addEraseMarkers( 0, 2000);
                markers_pub_.publish();
                ros::spinOnce();
            }
            std::cout << successes << std::endl;

        }

    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}

