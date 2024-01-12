#include <obj.h>
#include <iostream>
#include <algorithm>
#include <chrono>

#include "utils.h"

void removePlanePoints(std::vector<Point>& points, const Point& plane, float threshold) {
    // Remove points from 'points' vector if they are close to the plane
    points.erase(std::remove_if(points.begin(), points.end(), [&](const Point p) {
        return distanceToPlane(p.position, plane.position, plane.normal) < threshold;
    }), points.end());
}

void colorPlanePoints(const std::vector<Eigen::Vector3f>& positions,
                      std::vector<Eigen::Vector3f>& colors,
                      const std::vector<Point>& planes,
                      const std::vector<Eigen::Vector3f>& planeColors,
                      float threshold) {
    for (size_t i = 0; i < positions.size(); ++i) {
        for (size_t j = 0; j < planes.size(); ++j) {
            if (distanceToPlane(positions[i], planes[j].position, planes[j].normal) < threshold) {
                colors[i] = planeColors[j];
                break; // Stop checking other planes if this point is already colored
            }
        }
    }
}


Point ransac(const std::vector<Point>& points, int iterations, float threshold, float angleThreshold) {

    Point plane;
    int best_count = 0;

    #pragma omp parallel for shared(plane, best_count)
    for (int i = 0; i < iterations; ++i) {

        // Select 3 random points
        std::vector<Point> points_random = selectRandomPoints(points, 3);

        //filter normals
        while (maxAngle(points_random) > angleThreshold) {
            points_random = selectRandomPoints(points, 3);
        }

        Point p1 = points_random[0];
        Point p2 = points_random[1];
        Point p3 = points_random[2];

        // Compute plane defined by these 3 points
        Point ComputedPlane;
        ComputedPlane.normal = (p2.position - p1.position).cross(p3.position - p1.position).normalized();
        ComputedPlane.position = p1.position;

        int count = 0;

        // Count how many points lie close to the plane
        for (const auto& p : points) {
            if (distanceToPlane(p.position, ComputedPlane.position, ComputedPlane.normal) < threshold) {
                ++count;
            }
        }

        // Update best plane if current one is better
        if (count > best_count) {
            plane.position = ComputedPlane.position;
            plane.normal = ComputedPlane.normal;
            best_count = count;
        }
    }

    return plane;
}

std::vector<Point> detectMultiplePlanes(std::vector<Point>& points, int numPlanes, int iterations, float threshold, float angleThreshold){
    std::vector<Point> planes;

    for (int i = 0; i < numPlanes; ++i) {
        std::cout << "size of points: " << points.size() << std::endl;
        Point plane = ransac(points, iterations, threshold, angleThreshold);
        planes.push_back(plane);

        // Remove points that are close to the detected plane
        removePlanePoints(points, plane, threshold);

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
    std::vector<Eigen::Vector3f> positions;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector3f> colors;

    if(not tnp::load_obj(filename, positions, normals, colors)) {
        std::cout << "Failed to open input file '" << filename << "'" << std::endl;
        return 1;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    if (colors.empty()){
        colors = std::vector<Eigen::Vector3f>(positions.size(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    }

    std::vector<Point> points;
    for (size_t i = 0; i < positions.size(); ++i) {
        points.push_back({positions[i], normals[i], colors[i]});
    }

    // RANSAC parameters
    const int m = 200; // Number of iterations
    const float delta = 0.2f; // Threshold distance to consider a point as an inlier
    const float angleThreshold = 5.0f; // Threshold angle between normals of points to consider them as inliers
    const int numPlanes = 3;

    //generate random colors for the planes
    std::vector<Eigen::Vector3f> planeColors = generateRandomColors(numPlanes);

    //detect all the planes
    std::vector<Point> planes = detectMultiplePlanes(points, numPlanes, m, delta, angleThreshold);

    // Modify the color of the points on the plane to be red
    colorPlanePoints(positions, colors, planes, planeColors, delta);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "Time taken by RANSAC: " << duration.count() << " milliseconds" << std::endl;

    //save file
    if(not tnp::save_obj("ransac.obj", positions, normals, colors)) {
        std::cout << "Failed to save output file 'ransac.obj'" << std::endl;
        return 1;
    }

    return 0;
}