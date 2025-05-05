#include "ar_visualisation/mesh_creator.hpp"

namespace ar_visualisation
{

MeshCreator::MeshCreator()
 : Node("mesh_creator_node")
{
  setupPublishers();
  setupSubscriptions();
  RCLCPP_INFO(this->get_logger(), "mesh creator ready");
}

MeshCreator::~MeshCreator()
{

}

void MeshCreator::setupPublishers()
{

    pub_ = this->create_publisher<ar_visualisation_msgs::msg::MeshesData>(
        "/ar_visualisation/mesh_data", 10
    );
}

void MeshCreator::setupSubscriptions()
{
    sub_ = this->create_subscription<situational_graphs_msgs::msg::PlanesData>(
        "/s_graphs/all_map_planes", 10, 
        std::bind(&MeshCreator::wallDataCallback, this, std::placeholders::_1)
    );
}

void MeshCreator::wallDataCallback(const situational_graphs_msgs::msg::PlanesData::SharedPtr msg)
{
    std::vector<situational_graphs_msgs::msg::PlaneData> planes;
    planes.reserve(msg->x_planes.size() + msg->y_planes.size());
    planes.insert(planes.end(), msg->x_planes.begin(), msg->x_planes.end());
    planes.insert(planes.end(), msg->y_planes.begin(), msg->y_planes.end());

    auto data = ar_visualisation_msgs::msg::MeshesData();

    for (auto plane: planes)
    {
        auto id = plane.id;

        if (walls.find(id) != walls.end()) 
        {
            if (walls[id].update(plane.plane_points))
            {
                auto mesh_data = ar_visualisation_msgs::msg::MeshData();
                mesh_data.header.stamp = this->now();
                mesh_data.header.frame_id = "base_link";
                mesh_data.id = id;
                mesh_data.vertices = walls[id].getVertices();
                mesh_data.triangles = walls[id].getTriangles();

                data.meshes.emplace_back(mesh_data);
            }
        }
        else
        {
            walls[id] = MeshWall(plane.plane_points);

            auto mesh_data = ar_visualisation_msgs::msg::MeshData();
            mesh_data.header.stamp = this->now();
            mesh_data.header.frame_id = "base_link";
            mesh_data.id = id;
            mesh_data.vertices = walls[id].getVertices();
            mesh_data.triangles = walls[id].getTriangles();

            data.meshes.emplace_back(mesh_data);
        }
    }

    data.header.stamp = this->now();
    data.header.frame_id = "base_link";
    pub_->publish(data);
}

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ar_visualisation::MeshCreator>());
    rclcpp::shutdown();
    return 0;
}
