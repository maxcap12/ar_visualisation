#ifndef MAP_MARKER_HPP
#define MAP_MARKER_HPP

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <vector>

#include "ar_visualisation_msgs/msg/marker_data.hpp"

namespace ar_visualisation 
{

class MapMarker
{

public:
    MapMarker();

    MapMarker(
        const std::vector<geometry_msgs::msg::Point>& points,
        const geometry_msgs::msg::Vector3& scale,
        const std::vector<std_msgs::msg::ColorRGBA>& colors,
        int id,
        int type
    );
    
    MapMarker(
        const geometry_msgs::msg::Point& point,
        const geometry_msgs::msg::Vector3& scale,
        const std_msgs::msg::ColorRGBA& color,
        int id,
        int type
    );
    
    void update(
        const std::vector<geometry_msgs::msg::Point>& points,
        const geometry_msgs::msg::Vector3& scale,
        const std::vector<std_msgs::msg::ColorRGBA>& colors,
        int id,
        int type
    );

    void update(
        const geometry_msgs::msg::Point& point,
        const geometry_msgs::msg::Vector3& scale,
        const std_msgs::msg::ColorRGBA& color,
        int id,
        int type
    );

    ~MapMarker();
    ar_visualisation_msgs::msg::MarkerData getMessage();

    static const int ROOM;
    static const int CORRIDOR;
    static const int FLOOR;

private:
    void sendData();
    geometry_msgs::msg::Point ROS2Unity(const geometry_msgs::msg::Point& point);

    int markerId;
    int markerType;
    bool hasMarker = false;
    bool hasLines = false;

    std::vector<geometry_msgs::msg::Point> linePoints;
    geometry_msgs::msg::Vector3 lineScale;
    std::vector<std_msgs::msg::ColorRGBA> lineColors;

    geometry_msgs::msg::Point markerPoint;
    geometry_msgs::msg::Vector3 markerScale;
    std_msgs::msg::ColorRGBA markerColor;
};
}

#endif