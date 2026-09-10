#include "show.h"

using namespace HybridAStar;

Path_plot::Path_plot() {

};

void Path_plot::publishPath(Node3D* pathnode) {
    if (pathnode == nullptr) {
        ROS_WARN("Path node is null, cannot publish path.");
        return;
    }

    ros::NodeHandle n;
    path_pub_ = n.advertise<nav_msgs::Path>("planned_path", 10);

    ros::Duration(0.5).sleep();

    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    std::vector<const Node3D*> path_nodes_reverse;

    const Node3D* current = pathnode;
    while (current != nullptr) {
        path_nodes_reverse.push_back(current);
        current = current->getPred();
    }

    std::reverse(path_nodes_reverse.begin(), path_nodes_reverse.end());

    for (const auto& node : path_nodes_reverse) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = path.header.stamp;

        pose_stamped.pose.position.x = node->getX();
        pose_stamped.pose.position.y = node->getY();
        pose_stamped.pose.position.z = 0;

        tf::Quaternion q = tf::createQuaternionFromYaw(node->getT());
        quaternionTFToMsg(q, pose_stamped.pose.orientation);

        path.poses.push_back(pose_stamped);
    }

    path_pub_.publish(path);
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    ROS_INFO("Published a path with %lu poses.", path.poses.size());
}
