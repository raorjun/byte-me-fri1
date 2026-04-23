#ifndef FOLLOWER_ROBOT_NODE_H
#define FOLLOWER_ROBOT_NODE_H

#include "follower_robot/MoveToTarget.h"

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

/*
    This class is a template for your solution to the homework.
    I have erased sections of the class, for you to implement.
    I have placed comments so you can follow along in the code.
*/
class FollowerRobotNode : public rclcpp::Node {
public:
    /*
        The constructor takes two arguments:
            follow_distance is how close to come to the tag while following.
            tag_motion_threshold is used to determine if the tag has moved.
                You should only follow the tag if it has moved.
            I provide the code to determine if the tag has moved.
    */
    FollowerRobotNode(
        double follow_distance = 0.25,
        double tag_motion_threshold = 0.1);
    ~FollowerRobotNode();

protected:
    //I provide this. It is called whenever the april tag is in view.
    void aprilTagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    //I also provide this. It determines whether the tag has moved.
    bool theTagMoved(
        geometry_msgs::msg::TransformStamped &map_to_base_link,
        geometry_msgs::msg::TransformStamped &base_link_to_tag1
        );

    //This has an outline, but parts of the implementation are part of the homework.
    void computeAndAct();

    //Implementing this is part of the homework.
    double computeDistanceBaseLinkTag1(geometry_msgs::msg::TransformStamped &base_link_to_tag1);

    
    //Implementing this is part of the homework.
    Eigen::MatrixXd computeGoToFrameFromBaseLink(
        geometry_msgs::msg::TransformStamped &base_link_to_tag1);

    /*
        These are all of the class attributes that I used for my solution.
        You may change them as necessary, but they're exactly what I used.
    */
    //How closely to follow the tag. How much the tag must move to have "moved."
    double follow_distance_, tag_motion_threshold_; 

    /*
        This class encapsulates the actual motion commands.
        It's in its own class to simplify the code.
    */
    MoveToTarget move_to_target_;   

    //TF2 ROS classes. They are correctly initialized in the constructor.
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    //Subscriber for the april tag detection topic.
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr april_tag_sub_;
    Eigen::MatrixXd m_map_to_tag_1_prev_, m_map_to_go_to_;
    rclcpp::Time previous_stamp_;
};

#endif
    