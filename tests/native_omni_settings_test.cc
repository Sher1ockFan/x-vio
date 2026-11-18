// Use deployed settings with the linked native camera and Atlas implementation.
#include "System.h"
#include "Settings.h"
#include "OmniRadtan.h"
#include "GeometricTools.h"
#include "G2oTypes.h"
#include "OptimizableTypes.h"
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <cassert>
#include <iostream>
#include <sstream>

using namespace ORB_SLAM3;

int main(int argc, char** argv) {
    assert(argc == 2);
    Settings settings(argv[1], System::IMU_MONOCULAR);
    assert(settings.cameraType() == Settings::Omni);
    assert(settings.newImSize() == cv::Size(1920, 1536));
    assert(!settings.needToResize() && !settings.needToUndistort() && settings.fps() == 60);
    auto* camera = settings.camera1();
    assert(camera->GetType() == GeometricCamera::CAM_OMNI);

    // Native Kalibr expression evaluated independently of the VIO camera code.
    const double fx = 1494.2722516274534, fy = 1495.4468158582101;
    const double cx = 972.4542797992651, cy = 789.0899264198501;
    const double xi = 1.9121509749083578, k1 = .9066884402718393, k2 = -1.2465373776904145;
    const double p1 = -3.364740959810424e-5, p2 = .0007388408566274154;
    const Eigen::Vector3d p(-.85, .53, -.09);
    const double scale = p.z() + xi * p.norm();
    const double x = p.x() / scale, y = p.y() / scale, r2 = x*x+y*y;
    const double radial = 1+k1*r2+k2*r2*r2;
    const Eigen::Vector2d expected(fx*(x*radial+2*p1*x*y+p2*(r2+2*x*x))+cx,
                                   fy*(y*radial+p1*(r2+2*y*y)+2*p2*x*y)+cy);
    assert(camera->isVisible(p) && !camera->isVisible(Eigen::Vector3d(0, 0, -1)));
    const Eigen::Vector2d pixel = camera->project(p);
    assert((pixel-expected).norm() < .001);
    const Eigen::Vector3f ray = camera->unprojectEig(cv::Point2f(pixel.x(), pixel.y()));
    assert(ray.z() < 0 && (ray.cast<double>() - p.normalized()).norm() < 1e-5);
    const Eigen::Matrix<double, 2, 3> jacobian = camera->projectJac(p);
    for(int j=0; j<3; ++j) {
        Eigen::Vector3d before = p, after = p;
        before[j] -= 1e-6;
        after[j] += 1e-6;
        assert(((camera->project(after)-camera->project(before))/(2e-6)-jacobian.col(j)).norm() < .01);
    }

    // The same point is past the optical horizon in view 1 and lies on
    // the horizon in view 2. A z-normalized triangulator cannot handle this.
    Eigen::Matrix<float,3,4> T1 = Eigen::Matrix<float,3,4>::Identity();
    Eigen::Matrix<float,3,4> T2 = T1;
    T2.col(3) = Eigen::Vector3f(-.28f, -.16f, .09f);
    const Eigen::Vector3d point2 = p + T2.col(3).cast<double>();
    assert(std::abs(point2.z()) < 1e-8 && camera->isVisible(point2));
    const Eigen::Vector2d pixel2 = camera->project(point2);
    Eigen::Vector3f ray1 = ray;
    Eigen::Vector3f ray2 = camera->unprojectEig(cv::Point2f(pixel2.x(), pixel2.y()));
    Eigen::Vector3f triangulated;
    assert(ray1.z() < 0 && std::abs(ray2.z()) < 1e-5f);
    assert(GeometricTools::Triangulate(ray1, ray2, T1, T2, triangulated));
    assert((triangulated.cast<double>() - p).norm() < 1e-4);
    assert(triangulated.dot(ray1) > 0 && (triangulated + T2.col(3)).dot(ray2) > 0);

    // Exercise both the ordinary camera projection edge and the inertial
    // pose's camera projection. The back of the chart is rejected with a
    // finite residual, while valid negative-depth observations are inliers.
    const Eigen::Vector3d invalid(0, 0, -1);
    g2o::VertexSE3Expmap pose;
    pose.setEstimate(g2o::SE3Quat());
    EdgeSE3ProjectXYZOnlyPose edge;
    edge.setVertex(0, &pose);
    edge.setMeasurement(pixel);
    edge.setInformation(Eigen::Matrix2d::Identity());
    edge.pCamera = camera;
    edge.Xw = p;
    edge.computeError();
    assert(edge.isDepthPositive() && edge.error().norm() < 1e-8);
    edge.Xw = invalid;
    edge.computeError();
    assert(!edge.isDepthPositive() && edge.error().allFinite() && edge.chi2() > 5.991);

    ImuCamPose inertialPose;
    inertialPose.pCamera = {camera};
    inertialPose.Rcw = {Eigen::Matrix3d::Identity()};
    inertialPose.tcw = {Eigen::Vector3d::Zero()};
    assert(inertialPose.isDepthPositive(p) && !inertialPose.isDepthPositive(invalid));
    assert(inertialPose.ReprojectionError(p, pixel).norm() < 1e-8);
    const Eigen::Vector2d invalidResidual = inertialPose.ReprojectionError(invalid, pixel);
    assert(invalidResidual.allFinite() && invalidResidual.squaredNorm() > 5.991);

    Atlas atlas(0);
    assert(atlas.AddCamera(camera) == camera);
    std::vector<float> intrinsics;
    for(size_t i=0; i<camera->size(); ++i) intrinsics.push_back(camera->getParameter(i));
    OmniRadtan same(intrinsics);
    assert(atlas.AddCamera(&same) == camera);
    std::stringstream storage(std::ios::in | std::ios::out | std::ios::binary);
    { boost::archive::binary_oarchive archive(storage); archive << atlas; }
    Atlas restored(0);
    { boost::archive::binary_iarchive archive(storage); archive >> restored; }
    const auto cameras = restored.GetAllCameras();
    assert(cameras.size() == 1 && cameras[0]->GetType() == GeometricCamera::CAM_OMNI);
    assert(cameras[0]->isVisible(p) && !cameras[0]->isVisible(Eigen::Vector3d(0, 0, -1)));
    assert((cameras[0]->project(p)-pixel).norm() < 1e-8);
    std::cout << "OMNI_SETTINGS_1920x1536_VISIBILITY_JACOBIAN_TRIANGULATION_OPTIMIZER_ATLAS_PASS\n";
}
