#ifndef ORB_SLAM3_IMU_RECOVERY_H
#define ORB_SLAM3_IMU_RECOVERY_H
#include "ImuTypes.h"
#include <Eigen/Core>
#include <deque>
#include <string>
#include <vector>
#include <limits>

namespace ORB_SLAM3 {
struct RecoveryPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double t;
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d pwb;
};
using RecoveryPoses = std::vector<RecoveryPose, Eigen::aligned_allocator<RecoveryPose>>;
struct ImuRecoveryResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool valid = false;
    std::string reason;
    IMU::Bias bias;
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    Eigen::Matrix<double,15,15> information = Eigen::Matrix<double,15,15>::Zero();
    double rotation_rms = std::numeric_limits<double>::quiet_NaN();
    double position_rms = std::numeric_limits<double>::quiet_NaN();
    double condition_number = std::numeric_limits<double>::quiet_NaN();
};
// Fits only fresh-session dynamics against metric poses in a gravity-aligned,
// mature map. No old keyframe velocity, bias or integration is reused.
ImuRecoveryResult SolveImuRecovery(const RecoveryPoses& poses,
                                 const std::deque<IMU::Point>& samples,
                                 const IMU::Calib& calibration);
struct ImuRecoveryValidation {
    bool valid=false;
    std::string reason;
    double position_rms=std::numeric_limits<double>::quiet_NaN();
    double rotation_rms=std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3f velocity=Eigen::Vector3f::Zero();
};
// Independent future-data check. The fit is frozen: no holdout visual pose is
// used to refit velocity/bias. Predict from the last training pose through fresh
// raw IMU, then compare against geometrically verified future visual poses.
ImuRecoveryValidation ValidateImuRecovery(const RecoveryPose& fit_end,
    const ImuRecoveryResult& fit, const RecoveryPoses& holdout,
    const std::deque<IMU::Point>& samples, const IMU::Calib& calibration);
}
#endif
