// Native synthetic tests for signed spherical two-view initialization.
#include "OmniRadtan.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>

using ORB_SLAM3::OmniRadtan;

namespace {

const std::vector<float> kParameters{
    1494.2722516f, 1495.4468159f, 972.4542798f, 789.0899264f, 1.91215097f,
    .9066884403f, -1.2465373777f, -3.36474096e-05f, .00073884086f
};

cv::KeyPoint key(OmniRadtan& camera, const Eigen::Vector3d& point) {
    const Eigen::Vector2d pixel = camera.project(point);
    assert(pixel.allFinite());
    return cv::KeyPoint(cv::Point2f(pixel.x(), pixel.y()), 1.0f);
}

void testChart(OmniRadtan& camera) {
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    assert(!camera.project(zero).allFinite());
    assert(!camera.project(Eigen::Vector3d(0, 0, -1)).allFinite());
    assert(!camera.projectJac(zero).allFinite());
    assert(!camera.unprojectEig(cv::Point2f(-9000, -9000)).allFinite());

    for (double angle : {0.0, 50.0, 89.9, 90.0, 98.3, 115.0}) {
        const double theta = angle * (std::acos(-1.0) / 180.0);
        for (double azimuth : {0.01, 0.37, 1.79, 3.1, 4.5, 5.45}) {
            const Eigen::Vector3d ray(std::sin(theta)*std::cos(azimuth),
                                      std::sin(theta)*std::sin(azimuth), std::cos(theta));
            const Eigen::Vector3d fromPixel = camera.unprojectEig(key(camera, ray).pt).cast<double>();
            assert((fromPixel - ray).norm() < 1e-5);
            if (angle > 90.0)
                assert(fromPixel.z() < 0.0);
        }
    }
    auto folded = kParameters;
    folded[6] = -5.0f;
    bool rejected = false;
    try {
        OmniRadtan invalid(folded);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}

void testMotion(OmniRadtan& camera) {
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.04, Eigen::Vector3d(.31, -.93, .28).normalized()).toRotationMatrix();
    const Eigen::Vector3d translation(.01, -.19, .014);
    std::vector<cv::KeyPoint> before, after, rotationOnly;
    std::vector<int> matches;
    std::mt19937 random(98);
    std::uniform_real_distribution<double> angle(-0.47, 0.47), range(0.8, 3.0),
                                           depression(0.02, 0.27);
    std::normal_distribution<float> noise(0.0f, 0.13f);
    const int trueMatches = 170;
    while (before.size() < trueMatches) {
        const double phi = angle(random), distance = range(random);
        // Alternate the outer left and right of the 1920x1536 image. Every
        // valid point is beyond the optical horizon in both views.
        const double side = (before.size() % 2) ? 1.0 : -1.0;
        const Eigen::Vector3d p1 = distance * Eigen::Vector3d(
            side * std::cos(phi), std::sin(phi), -depression(random)).normalized();
        const Eigen::Vector3d p2 = rotation * p1 + translation;
        if (p2.z() >= -0.01)
            continue;
        const cv::KeyPoint a = key(camera, p1), b = key(camera, p2);
        const cv::KeyPoint noTranslation = key(camera, rotation * p1);
        auto inImage = [](const cv::KeyPoint& k) {
            return 0.0f <= k.pt.x && k.pt.x < 1920.0f && 0.0f <= k.pt.y && k.pt.y < 1536.0f;
        };
        if (!inImage(a) || !inImage(b) || !inImage(noTranslation))
            continue;
        before.push_back(a);
        after.push_back(b);
        rotationOnly.push_back(noTranslation);
    }
    for (cv::KeyPoint& kp : after) {
        kp.pt.x += noise(random);
        kp.pt.y += noise(random);
    }
    matches.resize(before.size());
    std::iota(matches.begin(), matches.end(), 0);
    auto wrongMatches = matches;
    for (int i = 0; i < 40; ++i)
        wrongMatches[i] = (i + 49) % trueMatches;

    Sophus::SE3f pose;
    std::vector<cv::Point3f> triangulated;
    std::vector<bool> valid;
    assert(!camera.ReconstructWithTwoViews(before, rotationOnly, matches, pose, triangulated, valid));
    assert(camera.ReconstructWithTwoViews(before, after, wrongMatches, pose, triangulated, valid));
    const Eigen::Matrix3d estimated = pose.rotationMatrix().cast<double>();
    assert((estimated - rotation).norm() < 0.006);
    assert(pose.translation().cast<double>().normalized().dot(translation.normalized()) > 0.996);
    assert(valid.size() == trueMatches && triangulated.size() == trueMatches);
    std::size_t count = 0;
    for (std::size_t i = 0; i < valid.size(); ++i) {
        if (!valid[i])
            continue;
        assert(triangulated[i].z < 0.0f);
        ++count;
    }
    std::cout << "OMNI_SIGNED_INITIALIZATION_POINTS " << count << '\n';
    assert(count >= 110);
}

}  // namespace

int main() {
    OmniRadtan camera(kParameters);
    testChart(camera);
    testMotion(camera);
    std::cout << "OMNI_ALMOST_HORIZON_AND_NEGATIVE_Z_GEOMETRY_PASS\n";
}
