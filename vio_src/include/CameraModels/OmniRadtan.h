// Part of ORB-SLAM3; distributed under the terms of the GNU GPL, version 3 or later.
#ifndef CAMERAMODELS_OMNIRADTAN_H
#define CAMERAMODELS_OMNIRADTAN_H

#include "GeometricCamera.h"

namespace ORB_SLAM3 {

class OmniRadtan final : public GeometricCamera {
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int) {
        ar & boost::serialization::base_object<GeometricCamera>(*this);
        if (Archive::is_loading::value)
            validateParameters();
    }

public:
    // Internal order keeps the existing Settings resize indices stable:
    // fx, fy, cx, cy, xi, k1, k2, p1, p2 (Kalibr stores xi before fx).
    OmniRadtan();
    explicit OmniRadtan(const std::vector<float>& parameters);

    cv::Point2f project(const cv::Point3f& point) override;
    Eigen::Vector2d project(const Eigen::Vector3d& point) override;
    Eigen::Vector2f project(const Eigen::Vector3f& point) override;
    Eigen::Vector2f projectMat(const cv::Point3f& point) override;
    Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& point) override;

    // A successful inverse represents the oriented unit ray, including z < 0.
    // Invalid pixels/rays return NaN; callers must check before using them.
    Eigen::Vector3f unprojectEig(const cv::Point2f& pixel) override;
    cv::Point3f unproject(const cv::Point2f& pixel) override;
    bool isVisible(const Eigen::Vector3d& point) const override;
    float uncertainty2(const Eigen::Matrix<double,2,1>& pixel) override;

    cv::Mat toK() override;
    Eigen::Matrix3f toK_() override;
    bool IsEqual(GeometricCamera* other) const;

    bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& keys1,
                                 const std::vector<cv::KeyPoint>& keys2,
                                 const std::vector<int>& matches,
                                 Sophus::SE3f& T21, std::vector<cv::Point3f>& points,
                                 std::vector<bool>& triangulated) override;

    bool epipolarConstrain(GeometricCamera* other, const cv::KeyPoint& key1,
                          const cv::KeyPoint& key2, const Eigen::Matrix3f& R12,
                          const Eigen::Vector3f& t12, float variance1,
                          float variance2) override;
    bool matchAndtriangulate(const cv::KeyPoint& key1, const cv::KeyPoint& key2,
                            GeometricCamera* other, Sophus::SE3f& Tcw1,
                            Sophus::SE3f& Tcw2, float variance1, float variance2,
                            Eigen::Vector3f& worldPoint) override;

private:
    void validateParameters() const;
    bool chart(const Eigen::Vector3d& point, double& norm, double& denominator) const;
    Eigen::Vector2d distort(const Eigen::Vector2d& normalized) const;
    Eigen::Matrix2d distortionJac(const Eigen::Vector2d& normalized) const;
    Eigen::Vector3d inverse(const cv::Point2f& pixel) const;
};

}  // namespace ORB_SLAM3

#endif  // CAMERAMODELS_OMNIRADTAN_H
