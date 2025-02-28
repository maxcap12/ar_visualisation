#ifndef MESH_CREATOR_HPP
#define MESH_CREATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include <unordered_map>

#include "ar_visualisation/mesh_wall.hpp"
#include "situational_graphs_msgs/msg/planes_data.hpp"
#include "situational_graphs_msgs/msg/plane_data.hpp"
#include "situational_graphs_msgs/msg/meshes_data.hpp"
#include "situational_graphs_msgs/msg/mesh_data.hpp"

namespace ar_visualisation 
{

class MeshCreator: public rclcpp::Node
{

public:
    MeshCreator();
    ~MeshCreator();

private:
    void setupPublishers();
    void setupSubscriptions();
    void wallDataCallback(const situational_graphs_msgs::msg::PlanesData::SharedPtr msg);

    rclcpp::Subscription<situational_graphs_msgs::msg::PlanesData>::SharedPtr sub_;
    rclcpp::Publisher<situational_graphs_msgs::msg::MeshesData>::SharedPtr pub_;

    std::unordered_map<int, MeshWall> walls;
};
}

#endif