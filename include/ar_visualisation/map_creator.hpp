#ifndef MESH_CREATOR_HPP
#define MESH_CREATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <unordered_map>
#include <string>

#include "ar_visualisation/mesh_wall.hpp"
#include "ar_visualisation/map_marker.hpp"
#include "situational_graphs_msgs/msg/planes_data.hpp"
#include "situational_graphs_msgs/msg/plane_data.hpp"
#include "ar_visualisation_msgs/msg/mesh_data.hpp"

namespace ar_visualisation 
{

class MapCreator: public rclcpp::Node
{

public:
    MapCreator();
    ~MapCreator();

private:
    void setupPublishers();
    void setupSubscriptions();
    void wallDataCallback(const situational_graphs_msgs::msg::PlanesData::SharedPtr msg);
    void markerDataCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    std::vector<geometry_msgs::msg::Point> update_position(const std::vector<geometry_msgs::msg::Point>& points, const std::string& source_frame);
    geometry_msgs::msg::Point update_position(const geometry_msgs::msg::Point& position, const std::string& source_frame);

    rclcpp::Subscription<situational_graphs_msgs::msg::PlanesData>::SharedPtr wall_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Publisher<ar_visualisation_msgs::msg::MeshData>::SharedPtr wall_pub_;
    rclcpp::Publisher<ar_visualisation_msgs::msg::MarkerData>::SharedPtr marker_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    std::string base_frame;

    std::unordered_map<int, MeshWall> walls;
    std::unordered_map<int, MapMarker> roomMarkers;
    std::unordered_map<int, MapMarker> corridorMarkers;
    std::unordered_map<int, MapMarker> floorMarkers;
};
}

#endif