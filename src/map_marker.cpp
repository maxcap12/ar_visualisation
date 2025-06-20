#include "ar_visualisation/map_marker.hpp"

namespace ar_visualisation
{

const int MapMarker::ROOM = 0;
const int MapMarker::CORRIDOR = 1;
const int MapMarker::FLOOR = 2;

MapMarker::MapMarker()
  : markerId(0),
    markerType(0),
    linePoints(),
    lineScale(),
    lineColors(),
    markerPoint(),
    markerScale(),
    markerColor()
{
}

// marker lines
MapMarker::MapMarker(
    const std::vector<geometry_msgs::msg::Point>& points,
    const geometry_msgs::msg::Vector3& scale,
    const std::vector<std_msgs::msg::ColorRGBA>& colors,
    int id,
    int type
)
  : markerId(id),
    markerType(type),
    hasLines(true),
    linePoints(),
    lineScale(scale),
    lineColors(),
    markerPoint(),
    markerScale(),
    markerColor()
{
    for (size_t i = 1; i < points.size(); i+=2)
        linePoints.push_back(ROS2Unity(points[i]));

    for (auto color: colors)
        lineColors.push_back(color);
}

// marker cube
MapMarker::MapMarker(
    const geometry_msgs::msg::Point& point,
    const geometry_msgs::msg::Vector3& scale,
    const std_msgs::msg::ColorRGBA& color,
    int id,
    int type
)
  : markerId(id),
    markerType(type),
    hasMarker(true),
    linePoints(),
    lineScale(),
    lineColors(),
    markerScale(scale),
    markerColor(color)
{
    markerPoint = ROS2Unity(point);
}

MapMarker::~MapMarker()
{
}

// marker lines
void MapMarker::update(
    const std::vector<geometry_msgs::msg::Point>& points,
    const geometry_msgs::msg::Vector3& scale,
    const std::vector<std_msgs::msg::ColorRGBA>& colors,
    int id,
    int type
) 
{
    if (id != markerId) throw std::runtime_error("Invalid marker ID");
    if (type != markerType) throw std::runtime_error("Invalid marker type");

    linePoints.clear();
    for (size_t i = 1; i < points.size(); i+=2)
        linePoints.push_back(ROS2Unity(points[i]));

    lineColors.clear();
    for (auto color: colors)
        lineColors.push_back(color);

    lineScale = scale;
    hasLines = true;
}

// marker cube
void MapMarker::update(
    const geometry_msgs::msg::Point& point,
    const geometry_msgs::msg::Vector3& scale,
    const std_msgs::msg::ColorRGBA& color,
    int id,
    int type
) 
{
    if (id != markerId) throw std::runtime_error("Invalid marker ID");
    if (type != markerType) throw std::runtime_error("Invalid marker type");

    markerPoint = ROS2Unity(point);
    markerScale = scale;
    markerColor = color;
    hasMarker = true;
}

ar_visualisation_msgs::msg::MarkerData MapMarker::getMessage()
{
    auto message = ar_visualisation_msgs::msg::MarkerData();
    message.action = 0;
    message.id = markerId;
    message.type = markerType;
    
    message.position = markerPoint;
    message.marker_scale = markerScale;
    message.marker_color = markerColor;

    message.lines = linePoints;
    message.lines_scale = lineScale;
    message.lines_colors = lineColors;
    
    return message;
}

geometry_msgs::msg::Point MapMarker::ROS2Unity(const geometry_msgs::msg::Point& point)
{
    auto result = point;
    result.x = -point.y;
    result.y = point.z;
    result.z = point.x;
    return result;
}

}