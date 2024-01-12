#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

struct Point {
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector3f color;
};

bool operator==(const Point& p1, const Point& p2) {
    return p1.position == p2.position && p1.normal == p2.normal && p1.color == p2.color;
}

// Function to compute distance from a point to a plane
float distanceToPlane(const Eigen::Vector3f& point, const Eigen::Vector3f& planePosition, const Eigen::Vector3f& planeNormal) {
    return std::abs((point - planePosition).dot(planeNormal)) / planeNormal.norm();
}

std::vector<Point> selectRandomPoints(const std::vector<Point>& points, int numPoints) {
    std::vector<Point> points_random;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, points.size() - 1);

    for (int i = 0; i < numPoints; ++i) {
        Point point = points[distrib(gen)];
        while (std::find(points_random.begin(), points_random.end(), point) != points_random.end()){
            point = points[distrib(gen)];
        }
        points_random.push_back(point);
    }

    return points_random;
}

// Helper function to compute the angle between two vectors in degrees
float angleBetween(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
    float dot = v1.dot(v2) / (v1.norm() * v2.norm());
    dot = std::max(-1.0f, std::min(1.0f, dot));
    return std::acos(dot) * (180.0f / M_PI); // Convert to degrees
}

float maxAngle(const std::vector<Point>& points) {
    float max_angle = 0.0f;
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            float angle = angleBetween(points[i].position, points[j].position);
            if (angle > max_angle) {
                max_angle = angle;
            }
        }
    }
    return max_angle;
}