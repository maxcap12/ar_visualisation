#include "ar_visualisation/mesh_wall.hpp"

namespace ar_visualisation
{

MeshWall::MeshWall(std::vector<geometry_msgs::msg::Vector3> points)
  : lastCount_(0),
    updating_(false)
{
    update(points);
}

MeshWall::~MeshWall()
{

}

bool MeshWall::update(std::vector<geometry_msgs::msg::Vector3> points)
{
    if (updating_) return false;
    if (points.size() - lastCount_ < 50) return false;

    updating_ = true;

    vertices_.clear();
    vertices_.reserve(points.size());

    for (auto point: points) vertices_.emplace_back(ROS2Unity(point));

    updateTriangles();

    lastCount_ = vertices_.size();
    updating_ = false;
    return true;
}

void MeshWall::updateTriangles()
{
    triangles_.clear();
    triangles_.reserve(vertices_.size() * 10 * 3); // estimation of the upperbound of triangles

    for (unsigned i = 0; i < vertices_.size() - 3; i++)
    {
        auto nearPoints = getNearestPointsIndex(i, 0.4f, 10);

        for (unsigned j = 0; j < nearPoints.size(); j++)
        {
            triangles_.push_back(i);
            triangles_.push_back(nearPoints[j]);
            triangles_.push_back(nearPoints[(j + 1) % nearPoints.size()]);
        }
    }
}

std::vector<int> MeshWall::getNearestPointsIndex(int index, float distanceThreshold, unsigned maxPoints)
{
    std::vector<int> nearPoints;
    nearPoints.reserve(maxPoints);

    std::vector<float> distances;
    distances.reserve(maxPoints);

    auto point = vertices_[index];

    distanceThreshold *= distanceThreshold;

    for (unsigned i = index + 1; i < vertices_.size(); i++)
    {
        float distance = sqDistance(point, vertices_[i]);

        if (distance < distanceThreshold)
        {
            if (nearPoints.size() == maxPoints)
            {
                int maxIndex = 0;
                float maxDist = distances[0];

                for (unsigned j = 1; j < maxPoints; j++)
                {
                    if (distances[j] < maxDist)
                    {
                        maxDist = distances[j];
                        maxIndex = j;
                    }
                }

                if (distance < maxDist)
                {
                    nearPoints[maxIndex] = i;
                    distances[maxIndex] = distance;
                }
            }
            else
            {
                nearPoints.push_back(i);
                distances.push_back(distance);
            }
        }
    }

    return nearPoints;
}

float MeshWall::sqDistance(geometry_msgs::msg::Vector3 p1, geometry_msgs::msg::Vector3 p2)
{
    float x = p1.x - p2.x;
    float y = p1.y - p2.y;
    float z = p1.z - p2.z;
    return x*x + y*y + z*z;
}

geometry_msgs::msg::Vector3 MeshWall::ROS2Unity(geometry_msgs::msg::Vector3 point)
{
    float x = -point.y;
    point.y = point.z;
    point.z = point.x;
    point.x = x;
    return point;
}

}