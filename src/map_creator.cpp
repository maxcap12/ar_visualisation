#include "ar_visualisation/map_creator.hpp"

namespace ar_visualisation
{

MapCreator::MapCreator()
 :  Node("map_creator_node"),
    base_frame("floor_0_walls_layer")
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
    wall_pub_ = this->create_publisher<ar_visualisation_msgs::msg::MeshData>(
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

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void MapCreator::wallDataCallback(const situational_graphs_msgs::msg::PlanesData::SharedPtr msg)
{
    std::vector<situational_graphs_msgs::msg::PlaneData> planes;
    planes.reserve(msg->x_planes.size() + msg->y_planes.size());
    planes.insert(planes.end(), msg->x_planes.begin(), msg->x_planes.end());
    planes.insert(planes.end(), msg->y_planes.begin(), msg->y_planes.end());

    for (auto plane: planes)
    {
        auto id = plane.id;

        if (walls.find(id) != walls.end()) 
        {
            if (walls[id].update(plane.plane_points))
            {
                auto mesh_data = ar_visualisation_msgs::msg::MeshData();
                mesh_data.id = id;
                mesh_data.vertices = walls[id].getVertices();
                mesh_data.triangles = walls[id].getTriangles();

                wall_pub_->publish(mesh_data);
            }
        }
        else
        {
            walls[id] = MeshWall(plane.plane_points);

            auto mesh_data = ar_visualisation_msgs::msg::MeshData();
            mesh_data.id = id;
            mesh_data.vertices = walls[id].getVertices();
            mesh_data.triangles = walls[id].getTriangles();

            wall_pub_->publish(mesh_data);
        }
    }
}

void MapCreator::markerDataCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    return;
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
                            marker.colors,
                            id,
                            type
                        );
                    }
                    else 
                    {
                        (*map)[id].update(
                            update_position(
                                marker.pose.position,
                                marker.header.frame_id
                            ),
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
                    marker.colors,
                    id,
                    type
                );
            }
            else 
            {
                (*map)[id] = MapMarker(
                    update_position(
                        marker.pose.position,
                        marker.header.frame_id
                    ),
                    marker.scale,
                    marker.color,
                    id,
                    type
                );
            }
        }
    }
}

geometry_msgs::msg::Point MapCreator::update_position(const geometry_msgs::msg::Point& position, const std::string& source_frame)
{
    geometry_msgs::msg::Point transformed_position;
    
    try {
      geometry_msgs::msg::PointStamped stamped_point;
      stamped_point.header.frame_id = source_frame;
      stamped_point.header.stamp = this->now();
      stamped_point.point = position;
      
      geometry_msgs::msg::PointStamped transformed_point;
      
      geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform(base_frame, source_frame, tf2::TimePointZero);
      
      tf2::doTransform(stamped_point, transformed_point, transform_stamped);
      
      transformed_position = transformed_point.point;
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
      transformed_position = position;
    }
    
    return transformed_position;
}

std::vector<geometry_msgs::msg::Point> MapCreator::update_position(const std::vector<geometry_msgs::msg::Point>& points, const std::string& source_frame)
  {
    for (auto point: points)
    {
        RCLCPP_INFO(this->get_logger(), "old coordinates: x=%.2f, y=%.2f, z=%.2f",
                point.x, point.y, point.z);
    }
    
    std::vector<geometry_msgs::msg::Point> transformed_points;
    transformed_points.reserve(points.size());
    
    try {
      geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform(base_frame, source_frame, tf2::TimePointZero);
      
      // Transform each point in the vector
      for (const auto& point : points) {
        geometry_msgs::msg::PointStamped stamped_point;
        stamped_point.header.frame_id = source_frame;
        stamped_point.header.stamp = this->now();
        stamped_point.point = point;
        
        geometry_msgs::msg::PointStamped transformed_point;
        tf2::doTransform(stamped_point, transformed_point, transform_stamped);
        
        transformed_points.push_back(transformed_point.point);
      }
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform points: %s", ex.what());
      transformed_points = points;
    }

    for (auto point: transformed_points)
    {
        RCLCPP_INFO(this->get_logger(), "new coordinates: x=%.2f, y=%.2f, z=%.2f",
                point.x, point.y, point.z);
    }
    
    return transformed_points;
}

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ar_visualisation::MapCreator>());
    rclcpp::shutdown();
    return 0;
}
