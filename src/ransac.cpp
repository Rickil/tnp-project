#include <Eigen/Core>
#include <Eigen/Geometry>
#include <obj.h>
#include <iostream>
#include <random>
#include <algorithm>


// Function to compute distance from a point to a plane
float distanceToPlane(const Eigen::Vector3f& point, const Eigen::Vector3f& planePoint, const Eigen::Vector3f& planeNormal) {
    return std::abs((point - planePoint).dot(planeNormal)) / planeNormal.norm();
}

void removePlanePoints(std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& planePoint, const Eigen::Vector3f& planeNormal, float threshold) {
    // Remove points from 'points' vector if they are close to the plane
    points.erase(std::remove_if(points.begin(), points.end(), [&](const Eigen::Vector3f& p) {
        return distanceToPlane(p, planePoint, planeNormal) < threshold;
    }), points.end());
}

/*void colorPlanePointsRed(std::vector<Eigen::Vector3f>& colors, const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& planePoint, const Eigen::Vector3f& planeNormal, float threshold) {
    for (size_t i = 0; i < points.size(); ++i) {
        if (distanceToPlane(points[i], planePoint, planeNormal) < threshold) {
            // Set color to red (assuming RGB format)
            colors[i] = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        }
    }
}*/

void colorPlanePoints(std::vector<Eigen::Vector3f>& colors,
                      const std::vector<Eigen::Vector3f>& points,
                      const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& planes,
                      const std::vector<Eigen::Vector3f>& planeColors,
                      float threshold) {
    std::cout << colors.size() << std::endl;
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = 0; j < planes.size(); ++j) {
            if (distanceToPlane(points[i], planes[j].first, planes[j].second) < threshold) {
                colors[i] = planeColors[j];
                break; // Stop checking other planes if this point is already colored
            }
        }
    }
}


std::pair<Eigen::Vector3f, Eigen::Vector3f> ransac(const std::vector<Eigen::Vector3f>& points, int iterations, float threshold) {
    Eigen::Vector3f best_p;
    Eigen::Vector3f best_n;
    int best_count = 0;

    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < iterations; ++i) {
        // Randomly select 3 points by generatin 3 numbers between 0 and points.size()
        std::uniform_int_distribution<> distrib(0, points.size() - 1);
        std::vector<Eigen::Vector3f> points_random;
        for (int j = 0; j < 3; ++j) {
            points_random.push_back(points[distrib(gen)]);
        }

        Eigen::Vector3f p1 = points_random[0];
        Eigen::Vector3f p2 = points_random[1];
        Eigen::Vector3f p3 = points_random[2];

        // Compute plane defined by these 3 points
        Eigen::Vector3f normal = (p2 - p1).cross(p3 - p1).normalized();
        Eigen::Vector3f point = p1;

        int count = 0;

        // Count how many points lie close to the plane
        for (const auto& p : points) {
            if (distanceToPlane(p, point, normal) < threshold) {
                ++count;
            }
        }

        // Update best plane if current one is better
        if (count > best_count) {
            best_p = point;
            best_n = normal;
            best_count = count;
        }
    }

    return {best_p, best_n};
}

std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> detectMultiplePlanes(std::vector<Eigen::Vector3f>& points, int numPlanes, int iterations, float threshold) {
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> planes;

    for (int i = 0; i < numPlanes; ++i) {
        auto [best_p, best_n] = ransac(points, iterations, threshold);
        planes.push_back({best_p, best_n});

        // Remove points that are close to the detected plane
        removePlanePoints(points, best_p, best_n, threshold);

        if (points.empty()) {
            break; // No more points to process
        }
    }

    return planes;
}

//generate random color for each plane
std::vector<Eigen::Vector3f> generateRandomColors(int numPlanes) {
    std::vector<Eigen::Vector3f> colors;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distrib(0.0f, 1.0f);
    for (int i = 0; i < numPlanes; ++i) {
        colors.push_back(Eigen::Vector3f(distrib(gen), distrib(gen), distrib(gen)));
    }
    return colors;
}


int main(int argc, char const *argv[])
{
    // option -----------------------------------------------------------------
    if(argc <= 1) {
        std::cout << "Error: missing argument" << std::endl;
        std::cout << "Usage: ransac <filename>.obj" << std::endl;
        return 0;
    }
    const std::string filename = argv[1];

    // load -------------------------------------------------------------------
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector3f> colors;

    if(not tnp::load_obj(filename, points, normals, colors)) {
        std::cout << "Failed to open input file '" << filename << "'" << std::endl;
        return 1;
    }

    std::vector<Eigen::Vector3f> points_copy = points;
    if (colors.size() == 0){
        colors = std::vector<Eigen::Vector3f>(points.size(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    }

    // RANSAC parameters
    const int m = 100; // Number of iterations
    const float delta = 0.1f; // Threshold distance to consider a point as an inlier
    const int numPlanes = 3;

    std::vector<Eigen::Vector3f> planeColors = generateRandomColors(numPlanes);

    auto planes = detectMultiplePlanes(points_copy, numPlanes, m, delta);

    // Modify the points on the plane to be red
    colorPlanePoints(colors, points, planes, planeColors, delta);

    //save file
    if(not tnp::save_obj("ransac.obj", points, normals, colors)) {
        std::cout << "Failed to save output file 'ransac.obj'" << std::endl;
        return 1;
    }

    return 0;
}