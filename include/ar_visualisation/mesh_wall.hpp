#ifndef MESH_WALL_HPP
#define MESH_WALL_HPP

#include <vector>
#include <geometry_msgs/msg/vector3.hpp>

namespace ar_visualisation
{

class MeshWall
{

public:
    MeshWall() = default;
    MeshWall(std::vector<geometry_msgs::msg::Vector3> points);
    ~MeshWall();
    bool update(std::vector<geometry_msgs::msg::Vector3> points);
    std::vector<geometry_msgs::msg::Vector3> getVertices() { return vertices_; }
    std::vector<int> getTriangles() { return triangles_; }

private:
    void updateTriangles();
    std::vector<int> getNearestPointsIndex(int startIndex, float distanceThreshold, unsigned maxPoints);
    float sqDistance(geometry_msgs::msg::Vector3 p1, geometry_msgs::msg::Vector3 p2);
    geometry_msgs::msg::Vector3 ROS2Unity(geometry_msgs::msg::Vector3 point);

    std::vector<geometry_msgs::msg::Vector3> vertices_;
    std::vector<int> triangles_;
    int lastCount_;
    bool updating_;
};

}

#endif