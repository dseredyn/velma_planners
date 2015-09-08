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

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {
        int_marker_pose_ = KDL::Frame(KDL::Rotation::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w), KDL::Vector(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
//      ROS_INFO_STREAM( feedback->marker_name << " is now at "
//          << feedback->pose.position.x << ", " << feedback->pose.position.y
//          << ", " << feedback->pose.position.z );
    }

visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.scale * 0.05;
  marker.scale.y = msg.scale * 0.05;
  marker.scale.z = msg.scale * 0.05;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void make6DofMarker( interactive_markers::InteractiveMarkerServer &server, bool fixed, unsigned int interaction_mode, const KDL::Frame &T_W_M, bool show_6dof )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::Vector3 pos(T_W_M.p.x(), T_W_M.p.y(), T_W_M.p.z());
  double qx,qy,qz,qw;
  T_W_M.M.GetQuaternion(qx,qy,qz,qw);
  tf::Quaternion ori(qx,qy,qz,qw);
  tf::Transform pose(ori, pos);
  tf::poseTFToMsg(pose, int_marker.pose);
  int_marker.scale = 0.2;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server.insert(int_marker);
  server.setCallback(int_marker.name, boost::bind(&TestDynamicModel::processFeedback, this, _1) );
//  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
//    menu_handler.apply( server, int_marker.name );
}

    bool checkCollision(const KDL::Vector &x, const boost::shared_ptr<self_collision::CollisionModel> &col_model) {
        // create dummy object
        boost::shared_ptr< self_collision::Collision > pcol = self_collision::createCollisionSphere(0.1, KDL::Frame(x));
        KDL::Frame T_B_L1;
        KDL::Frame T_B_L2;
        return self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);
    }

    KDL::Twist distanceMetric(const KDL::Frame &F_a_b1, const KDL::Frame &F_a_b2, const boost::shared_ptr<ReachabilityMap > &r_map) {
        KDL::Twist diff = KDL::diff(F_a_b1, F_a_b2, 1.0);
        if (diff.vel.Norm() < 0.05) {
            return diff;
        }

        double dist = (F_a_b1.p - F_a_b2.p).Norm();
        KDL::Vector gr;
        if (r_map->getGradient(F_a_b1.p, gr)) {
            diff.vel = gr * dist;
            int m_id = 6000;
            m_id = markers_pub_.addVectorMarker(m_id, F_a_b1.p, F_a_b1.p + gr*0.3, 0, 0, 1, 1, 0.005, "world");
            markers_pub_.addEraseMarkers(m_id, m_id+300);
            markers_pub_.publish();
            ros::spinOnce();
        }


        return diff;

        KDL::Vector points_E[4] = {
        KDL::Vector(0, 0, 0),
        KDL::Vector(0, -0.13, 0.02),
        KDL::Vector(0.03, 0.13, 0.02),
        KDL::Vector(-0.03, 0.13, 0.02),
        };

        int m_id = 6000;

        std::list<std::vector<ReachabilityMap::GradientInfo > > gradients_list;
        for (int i=0; i<4; i++) {
            KDL::Vector pt_W = F_a_b1 * points_E[i];
            std::vector<ReachabilityMap::GradientInfo > gradients;
            gradients.resize(27);
            if (r_map->getAllGradients(pt_W, gradients)) {
                gradients_list.push_back(gradients);
            }
        }

        if (gradients_list.empty()) {
            return diff;
        }

        bool valid_directions[27];
        for (int i=0; i<27; i++) {
            valid_directions[i] = true;
        }

        // AND all gradient directions
        int max_restrictions = 0;
        std::list<std::vector<ReachabilityMap::GradientInfo > >::const_iterator max_restrictions_it = gradients_list.end(); 
        int invalid_dirs = 0;
        for (std::list<std::vector<ReachabilityMap::GradientInfo > >::const_iterator it = gradients_list.begin(); it != gradients_list.end(); it++) {
            int restrictions = 0;
            for (int i=0; i<27; i++) {
                if ( !(*it)[i].valid_ && valid_directions[i]) {
                    valid_directions[i] = false;
                    restrictions++;
                    invalid_dirs++;
                }
            }
            if (restrictions > max_restrictions) {
                max_restrictions = restrictions;
                max_restrictions_it = it;
            }
        }

        if (max_restrictions_it != gradients_list.end()) {
            double min_value = 10000.0;
            KDL::Vector min_direction;
            int min_direction_idx;
            for (int i=0; i<27; i++) {
                if ( valid_directions[i] ) {
                    if (min_value > (*max_restrictions_it)[i].value_) {
                        min_value = (*max_restrictions_it)[i].value_;
                        min_direction = (*max_restrictions_it)[i].direction_;
                        min_direction_idx = i;
                    }
                }
            }

            double dist = (F_a_b1.p - F_a_b2.p).Norm();
            m_id = markers_pub_.addVectorMarker(m_id, F_a_b1.p, F_a_b1.p + min_direction*0.2, 0, 0, 1, 1, 0.005, "world");
            diff.vel = min_direction * dist;
            return diff;
        }
        else {
            // no restrictions
            KDL::Vector mean_direction;
            int pt_i = 0;
            for (std::list<std::vector<ReachabilityMap::GradientInfo > >::const_iterator it = gradients_list.begin(); it != gradients_list.end(); it++) {
                double min_value = 10000.0;
                KDL::Vector min_direction;
                int min_direction_idx;
                for (int i=0; i<27; i++) {
                    if ( valid_directions[i] ) {
                        if (min_value > (*it)[i].value_) {
                            min_value = (*it)[i].value_;
                            min_direction = (*it)[i].direction_;
                            min_direction_idx = i;
                        }
                    }
                }

                KDL::Vector pt_W = F_a_b1 * points_E[pt_i];
                for (int i=0; i<27; i++) {
                    if ((*it)[i].valid_) {
                        if (min_direction_idx == i) {
                            m_id = markers_pub_.addVectorMarker(m_id, pt_W, pt_W + (*it)[i].direction_*0.05, 1, 1, 1, 1, 0.005, "world");
                        }
                        else {
                            m_id = markers_pub_.addVectorMarker(m_id, pt_W, pt_W + (*it)[i].direction_*0.05, 0, 0, 1, 1, 0.005, "world");
                        }
                    }
                }

                mean_direction += min_direction;
                pt_i++;
            }

            double dist = (F_a_b1.p - F_a_b2.p).Norm();
            m_id = markers_pub_.addVectorMarker(m_id, F_a_b1.p, F_a_b1.p + mean_direction*0.2, 0, 0, 1, 1, 0.005, "world");
            diff.vel = mean_direction * dist;
            return diff;
        }
/*
//        std::cout << invalid_dirs << std::endl;
        KDL::Vector mean_direction;

        double tmin_value = -10000.0;
        KDL::Vector tmin_direction;

        int i=0;
        for (std::list<std::vector<ReachabilityMap::GradientInfo > >::const_iterator it = gradients_list.begin(); it != gradients_list.end(); it++) {
            double min_value = 10000.0;
            KDL::Vector min_direction;
            int min_direction_idx;
            for (int i=0; i<27; i++) {
                if ( valid_directions[i] ) {
//                    if ((*it)[i].value_ < 0) {
//                        min_direction += (*it)[i].direction_;
//                    }
                    if (min_value > (*it)[i].value_) {
                        min_value = (*it)[i].value_;
                        min_direction = (*it)[i].direction_;
                        min_direction_idx = i;
                    }
                }
            }

            KDL::Vector pt_W = F_a_b1 * points_E[i];
            for (int i=0; i<27; i++) {
                if ((*it)[i].valid_) {
                    if (min_direction_idx == i) {
                        m_id = markers_pub_.addVectorMarker(m_id, pt_W, pt_W + (*it)[i].direction_*0.05, 1, 1, 1, 1, 0.005, "world");
                    }
                    else {
                        m_id = markers_pub_.addVectorMarker(m_id, pt_W, pt_W + (*it)[i].direction_*0.05, 0, 0, 1, 1, 0.005, "world");
                    }
                }
            }

            mean_direction += min_direction;
            if (tmin_value < min_value) {
                        tmin_value = min_value;
                        tmin_direction = min_direction;
            }
            i++;
        }

        if (!gradients_list.empty()) {
            mean_direction.Normalize();
            double dist = (F_a_b1.p - F_a_b2.p).Norm();

//            m_id = markers_pub_.addVectorMarker(m_id, F_a_b1.p, F_a_b1.p + mean_direction*0.2, 0, 0, 1, 1, 0.005, "world");
//            diff.vel = mean_direction * dist;
            m_id = markers_pub_.addVectorMarker(m_id, F_a_b1.p, F_a_b1.p + tmin_direction*0.2, 0, 0, 1, 1, 0.005, "world");
            diff.vel = tmin_direction * dist;
        }

        markers_pub_.addEraseMarkers(m_id, m_id+300);
        markers_pub_.publish();
        ros::spinOnce();

//        KDL::Vector gr;
//        if ((F_a_b1.p - F_a_b2.p).Norm() > 0.05 && r_map->getDistnace(F_a_b1.p, dist) && r_map->getGradient(F_a_b1.p, gr)) {
//        if (dist > 0.15 && r_map->getGradient(F_a_b1.p, gr)) {
//            diff.vel = gr * dist;
//        }

        return diff;*/
    }

    void spin() {
/*
        std::vector<std::list<Eigen::VectorXd > > q_start_vec(6);
        FILE *f = fopen("exp01.txt", "rt");
        while (true) {
            double x;
            int s;
            Eigen::VectorXd qq(15);
            for (int q_idx = 0; q_idx < 15; q_idx++) {
                fscanf(f, "%lf", &x);
                qq(q_idx) = x;
            }
            if (fscanf(f, "%d", &s) < 1){
                break;
            }
//            std::cout << qq.transpose() << " " << s << std::endl;
            q_start_vec[s].push_back(qq);
        }
        fclose(f);
//        return;
*/
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

        Eigen::VectorXd q, dq, ddq, torque;
        q.resize( ndof );
        dq.resize( ndof );
        ddq.resize( ndof );
        torque.resize( ndof );
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q[q_idx] = 0.5;
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
            torque[q_idx] = 0.0;
        }
//        q[0] = 1.2;
        q[2] = 1.0;

        double init_q[15] = {
        0.0,
        120.0/180.0*PI,
        -90.0/180.0*PI,
        110.0/180.0*PI,
        140.0/180.0*PI,
        0.0/180.0*PI,
        -40.0/180.0*PI,
        0.0/180.0*PI,
        90.0/180.0*PI,
        90.0/180.0*PI,
        -90.0/180.0*PI,
        -90.0/180.0*PI,
        0.0/180.0*PI,
        90.0/180.0*PI,
        -90.0/180.0*PI,
        };

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
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

//        kin_model->setUpperLimit(1, -5.0/180.0*PI);
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

        boost::shared_ptr<DynamicsSimulatorHandPose> sim(new DynamicsSimulatorHandPose(ndof, 6, effector_name, col_model, kin_model, dyn_model, joint_names, 100.0*max_q ) );

        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        int loop_counter = 50000;
        ros::Rate loop_rate(100);
/*
        while (true) {
            generatePossiblePose(r_HAND_target, q, ndof, effector_name, col_model, kin_model);
            sim->setState(q, dq, ddq);
            sim->setTarget(r_HAND_target);
            sim->oneStep();
            if (!sim->inCollision()) {
                break;
            }
        }
*/
        // TEST: distance map
        boost::shared_ptr<ReachabilityMap > r_map(new ReachabilityMap(0.025, 3));


        KDL::Vector lower_bound(0.5, -0.7, 1.0);
        KDL::Vector upper_bound(2.0, 0.7, 2.0);
        if (!r_map->createDistanceMap(KDL::Vector(1.05, 0.0, 1.35), boost::bind(&TestDynamicModel::checkCollision, this, _1, col_model), lower_bound, upper_bound)) {
            std::cout << "could not create the distance map" << std::endl;
        }
        else {
            std::cout << "created distance map" << std::endl;
        }

//        return;
        sim->updateMetric( boost::bind(&TestDynamicModel::distanceMetric, this, _1, _2, r_map) );


/*
        while (ros::ok()) {
            int m_id = 0;

            for (double x = lower_bound(0)-0.1; x < upper_bound(0)+0.1; x += 0.025) {
//                break;
                {
                    double y = 0.0;
//                for (double y = lower_bound(1)+0.2; y < upper_bound(1)-0.5; y += 0.025) {
                    for (double z = lower_bound(2)+0.1; z < upper_bound(2)-0.4; z += 0.025)
                    {
//                        double z = 1.4;
                        KDL::Vector pt_W(x,y,z);
                        double val = 0.0;
                        if (!r_map->getDistnace(pt_W, val)) {   
                            m_id = markers_pub_.addSinglePointMarker(m_id, pt_W, 0, 0, 0, 0.5, 0.01, "world");
                        }
                        else if (val == -1.0) {
                            m_id = markers_pub_.addSinglePointMarker(m_id, pt_W, 1, 1, 0, 0.5, 0.01, "world");
                        }
                        else if (val == -2.0) {
                            m_id = markers_pub_.addSinglePointMarker(m_id, pt_W, 0, 0, 1, 0.5, 0.01, "world");
                        }
                        else if (val >= 0.0) {
                            val /= 1.1;
                            KDL::Vector gr;
                            if (r_map->getGradient(pt_W, gr)) {
                                m_id = markers_pub_.addVectorMarker(m_id, pt_W, pt_W + gr*0.03, 0, 1, 0, 1, 0.01, "world");
                            }
                            else {
                                m_id = markers_pub_.addSinglePointMarker(m_id, pt_W, 0, 0, 1, 0.5, 0.02, "world");
                            }
                            m_id = markers_pub_.addSinglePointMarker(m_id, pt_W, 1, val, val, 0.5, 0.01, "world");
                        }
                    }
                }
            }
            KDL::Vector rand_pt(randomUniform(lower_bound(0)-1, upper_bound(0)+1), randomUniform(lower_bound(1), upper_bound(1)), randomUniform(lower_bound(2), upper_bound(2)));
            m_id = markers_pub_.addSinglePointMarker(m_id, rand_pt, 1, 0, 0, 0.5, 0.1, "world");
            int path_length = 0;
            while (true) {
                bool use_twist = true;
                if (use_twist) {
                    KDL::Twist twist = distanceMetric(KDL::Frame(rand_pt), KDL::Frame(KDL::Vector(1.05, 0.0, 1.35)), r_map);
                    KDL::Vector prev = rand_pt;
                    rand_pt = rand_pt + twist.vel * 0.02;
                    m_id = markers_pub_.addVectorMarker(m_id, prev, rand_pt, 0, 1, 0, 0.5, 0.01, "world");
                }
                else {
                    KDL::Vector gr;
                    double dist = 0.0;
                    if (!r_map->getDistnace(rand_pt, dist) || dist < 0.05) {
                        std::cout << "distance failed" << std::endl;
                        break;
                    }
                    std::vector<ReachabilityMap::GradientInfo > gradients;
                    gradients.resize(27);
                    r_map->getAllGradients(rand_pt, gradients);
                    for (int i=0; i<27; i++) {
                        if (gradients[i].valid_) {
                            m_id = markers_pub_.addVectorMarker(m_id, rand_pt, rand_pt + gradients[i].direction_*0.02, 0, 0, 1, 1, 0.005, "world");
                        }
                    }

                    if (r_map->getGradient(rand_pt, gr)) {
                        KDL::Vector prev = rand_pt;
                        rand_pt = rand_pt + gr * 0.02;
                        m_id = markers_pub_.addVectorMarker(m_id, prev, rand_pt, 0, 1, 0, 0.5, 0.01, "world");
                    }
                    else {
                        std::cout << "gradient failed" << std::endl;
                        break;
                    }
                }
                path_length++;
                if (path_length > 300) {
                    std::cout << "path length exceeded" << std::endl;
                    break;
                }
            }
            markers_pub_.addEraseMarkers(m_id, m_id+300);

            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }

            m_id = 5000;
            m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
            markers_pub_.publish();
            markers_pub_.publish();

            publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
            ros::spinOnce();
            loop_rate.sleep();
            ros::Duration(0.5).sleep();
            getchar();
        }

        return;
//*/

/*        // reachability map test
        boost::shared_ptr<ReachabilityMap > r_map(new ReachabilityMap(0.1, 3));
        r_map->generateForArm(kin_model, "torso_link2", "right_HandPalmLink");

        while (ros::ok()) {
            KDL::Frame T_W_T2, T_T2_W;
            kin_model->calculateFk(T_W_T2, "torso_link2", q);
            T_T2_W = T_W_T2.Inverse();
//            std::cout << r_map->getMaxValue() << std::endl;
            int m_id = 0;
            for (double x = -2.0; x < 2.0; x += 0.2) {
                for (double y = -2.0; y < 2.0; y += 0.2) {
                    for (double z = 0.5; z < 2.5; z += 0.2) {
                        KDL::Vector pt_W(x,y,z);
                        KDL::Vector pt_T2 = T_T2_W * pt_W;
                        Eigen::VectorXd p(3);
                        p(0) = pt_T2[0];
                        p(1) = pt_T2[1];
                        p(2) = pt_T2[2];
                        double val = r_map->getValue(p);
                        if (val > 0.001) {
                            m_id = markers_pub_.addSinglePointMarker(m_id, pt_W, 1, val, val, 1, val*0.2, "world");
                        }
                    }
                }
            }
            markers_pub_.publish();

            publishJointState(joint_state_pub_, q, joint_names, ign_q, ign_joint_names);
            ros::spinOnce();
            loop_rate.sleep();
            ros::Duration(0.5).sleep();
        }
        return;
*/

        int_marker_pose_ = r_HAND_target;
        // create an interactive marker server on the topic namespace simple_marker
        interactive_markers::InteractiveMarkerServer server("simple_marker");
        tf::Vector3 position;
        position = tf::Vector3( 0, 0, 0);
        make6DofMarker(server, false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, r_HAND_target, true);
        // 'commit' changes and send to all clients
        server.applyChanges();

        int s_idx = 5;

//        std::string mode = "random_dest";
//        std::string mode = "marker_dest";
        std::string mode = "random_start";

        ros::Duration(1.0).sleep();

/*
        // TEST
        int m_id = 5000;
        VelmaQ5Q6CollisionChecker cc(0, 1, 10.0/180.0*3.1415, false);
        for (double x = -3.0; x < 3.0; x+=0.1) {
            for (double y = -3.0; y < 3.0; y+=0.1) {
                Eigen::VectorXd q(2);
                q(0) = x;
                q(1) = y;
                if (cc.inCollision(q)) {
                    m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(x, y, 0), 1, 0, 0, 1, 0.03, "world");
                }
                else {
                    m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(x, y, 0), 0, 1, 0, 1, 0.03, "world");
                }
            }
        }
        {
        Eigen::VectorXd q(2);
        q(0) = -0.67725;
        q(1) = 1.64441;
                if (cc.inCollision(q)) {
                    m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(q(0), q(1), 0), 1, 0, 0, 1, 0.03, "world");
                }
                else {
                    m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(q(0), q(1), 0), 0, 1, 0, 1, 0.03, "world");
                }
        }
        ros::Duration(1.0).sleep();
        markers_pub_.publish();
        ros::spinOnce();

        return;
//*/

        std::list<Eigen::VectorXd > q_history_latest;
        std::list<Eigen::VectorXd > q_history;
        bool collision_in_prev_step = false;
        while (ros::ok()) {

            if (mode == "random_dest") {
                if (loop_counter > 1500*10) {
                    sim->getState(q, dq, ddq);
                    std::cout << "q: " << q.transpose() << std::endl;
                    std::cout << "dq: " << dq.transpose() << std::endl;

                    Eigen::VectorXd q_tmp(ndof);
                    generatePossiblePose(r_HAND_target, q_tmp, ndof, effector_name, col_model, kin_model);

                    sim->setTarget(r_HAND_target);

                    publishTransform(br, r_HAND_target, "effector_dest", "world");
                    loop_counter = 0;
                }
                loop_counter += 1;
            }
            else if (mode == "marker_dest") {
                r_HAND_target = int_marker_pose_;
                sim->setTarget(r_HAND_target);
            }
            else if (mode == "random_start") {
                if (loop_counter > 1500*10) {
/*                    while (true) {
                        generatePossiblePose(r_HAND_target, q, ndof, effector_name, col_model, kin_model);
                        sim->setState(q, dq, ddq);
                        sim->setTarget(r_HAND_target);
                        sim->oneStep();
                        if (!sim->inCollision()) {
                            break;
                        }
                    }*/
                    sim->setState(q, dq, ddq);
                    r_HAND_target = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(1.05, 0.0, 1.35));
                    sim->setTarget(r_HAND_target);
                    loop_counter = 0;
                }
                loop_counter += 1;
            }
            publishTransform(br, r_HAND_target, "effector_dest", "world");
/*
            const double q_tab[] = {0.932977, 2.23176, -0.444369, 1.51863, -0.149273, 2.02303, -0.0241965, 0.556891, 2.08885, -1.81702, 1.55818, 0.30735, 0.192247, 1.24464, -0.572893};
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                q(q_idx) = q_tab[q_idx];
            }
            sim->setState(q, dq, ddq);

            std::cout << "*" << std::endl;
            std::cout << "*" << std::endl;
            std::cout << "*" << std::endl;
*/
            sim->oneStep(&markers_pub_, 3000);
            if (sim->inCollision() && !collision_in_prev_step) {
                collision_in_prev_step = true;
                std::cout << "collision begin" << std::endl;
            }
            else if (!sim->inCollision() && collision_in_prev_step) {
                collision_in_prev_step = false;
                std::cout << "collision end" << std::endl;
            }

            if (q_history.empty()) {
                q_history.push_back(q);
            }
            else {
                if ((q_history.back() - q).norm() > 10.0/180.0*PI) {
                    q_history.push_back(q);
                }
            }

            sim->getState(q, dq, ddq);
            q_history_latest.push_back(q);
            if (q_history_latest.size() > 100) {
                q_history_latest.pop_front();
                Eigen::VectorXd q_mean(ndof);
                q_mean.setZero();
                for (std::list<Eigen::VectorXd >::const_iterator it = q_history_latest.begin(); it != q_history_latest.end(); it++) {
                    q_mean += (*it);
                }
                q_mean = q_mean / q_history_latest.size();
                double var = 0.0;
                for (std::list<Eigen::VectorXd >::const_iterator it = q_history_latest.begin(); it != q_history_latest.end(); it++) {
                    double norm = ((*it) - q_mean).norm();
                    var += norm * norm;
                }
//                std::cout << var << std::endl;
                if (var < 0.0002) {
                    KDL::Frame current_T_B_E;
                    kin_model->calculateFk(current_T_B_E, effector_name, q);
                    KDL::Twist diff = KDL::diff(r_HAND_target, current_T_B_E);
                    bool goal_reached = false;
                    if (diff.vel.Norm() < 0.005 && diff.rot.Norm() < 1.0/180.0*PI) {
                        goal_reached = true;
                    }

                    std::cout << "stopped" << std::endl;
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
                    if (goal_reached) {
                        std::cout << "goal reached" << std::endl;
                    } else {
                        std::cout << "could not reach goal" << std::endl;
                    }
                    getchar();
                    loop_counter = 100000;
                    q_history_latest.clear();
                }
            }

            
//            std::cout << "q: " << q.transpose() << std::endl;

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
            }
            else {
                markers_pub_.clear();
            }
            ros::spinOnce();
            loop_rate.sleep();
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


