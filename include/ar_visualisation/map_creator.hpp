#ifndef MESH_CREATOR_HPP
#define MESH_CREATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <unordered_map>
#include <string>

#include "ar_visualisation/mesh_wall.hpp"
#include "ar_visualisation/map_marker.hpp"
#include "situational_graphs_msgs/msg/planes_data.hpp"
#include "situational_graphs_msgs/msg/plane_data.hpp"
#include "ar_visualisation_msgs/msg/meshes_data.hpp"
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

    rclcpp::Subscription<situational_graphs_msgs::msg::PlanesData>::SharedPtr wall_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Publisher<ar_visualisation_msgs::msg::MeshData>::SharedPtr wall_pub_;
    rclcpp::Publisher<ar_visualisation_msgs::msg::MarkerData>::SharedPtr marker_pub_;

    std::unordered_map<int, MeshWall> walls;
    std::unordered_map<int, MapMarker> roomMarkers;
    std::unordered_map<int, MapMarker> corridorMarkers;
    std::unordered_map<int, MapMarker> floorMarkers;
};
}

#endif