#include "ar_visualisation/map_creator.hpp"

namespace ar_visualisation
{

MapCreator::MapCreator()
 : Node("map_creator_node")
{
  setupPublishers();
  setupSubscriptions();
  RCLCPP_INFO(this->get_logger(), "map creator ready");
}

MapCreator::~MapCreator()
{

}

void MapCreator::setupPublishers()
{
    wall_pub_ = this->create_publisher<ar_visualisation_msgs::msg::MeshesData>(
        "/ar_visualisation/mesh_data", 10
    );

    marker_pub_ = this->create_publisher<ar_visualisation_msgs::msg::MarkerData>(
        "/ar_visualisation/marker_data", 10
    );
}

void MapCreator::setupSubscriptions()
{
    wall_sub_ = this->create_subscription<situational_graphs_msgs::msg::PlanesData>(
        "/s_graphs/all_map_planes", 10, 
        std::bind(&MapCreator::wallDataCallback, this, std::placeholders::_1)
    );

    marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/s_graphs/markers", 10,
        std::bind(&MapCreator::markerDataCallback, this, std::placeholders::_1)
    );
}

void MapCreator::wallDataCallback(const situational_graphs_msgs::msg::PlanesData::SharedPtr msg)
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
    wall_pub_->publish(data);
}

void MapCreator::markerDataCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    for (auto marker: msg->markers)
    {
        auto ns = marker.ns;
        auto id = marker.id;
        std::unordered_map<int, MapMarker> *map;
        int type;

        if (ns == "rooms" || ns == "rooms_line")
        {
            map = &roomMarkers;
            type = MapMarker::ROOM;
        }
        else if (ns == "floors" || ns == "floor_lines")
        {
            map = &floorMarkers;
            type = MapMarker::FLOOR;
        }
        else if (ns == "y_infinite_room" || ns == "infinite_room_y_lines")
        {
            map = &corridorMarkers;
            type = MapMarker::CORRIDOR;
        }
        else continue;

        if (map->find(id) != map->end())
        {
            switch (marker.action)
            {
                case 0:
                    if (ns.find("line") != std::string::npos)
                    {
                        (*map)[id].update(
                            marker.points,
                            marker.scale,
                            marker.color,
                            id,
                            type
                        );
                    }
                    else 
                    {
                        (*map)[id].update(
                            marker.pose.position,
                            marker.scale,
                            marker.color,
                            id,
                            type
                        );
                    }

                    marker_pub_->publish((*map)[id].getMessage());
                    break;

                case 2: {
                    auto msg = (*map)[id].getMessage();
                    msg.action = 1;
                    marker_pub_->publish(msg);
                    map->erase(map->find(id));
                    break;
                }
                default:
                    continue;
            }
        }
        else 
        {
            if (marker.action != 0) continue;
            
            if (ns.find("line") != std::string::npos)
            {
                (*map)[id] = MapMarker(
                    marker.points,
                    marker.scale,
                    marker.color,
                    id,
                    type
                );
            }
            else 
            {
                (*map)[id] = MapMarker(
                    marker.pose.position,
                    marker.scale,
                    marker.color,
                    id,
                    type
                );
            }
        }
    }
}

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ar_visualisation::MapCreator>());
    rclcpp::shutdown();
    return 0;
}
