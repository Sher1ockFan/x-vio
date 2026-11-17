#ifndef ORB_SLAM3_TRACKING_HEALTH_H
#define ORB_SLAM3_TRACKING_HEALTH_H
#include <array>
#include <limits>
#include <string>
namespace ORB_SLAM3 {
inline double HealthUnknown() { return std::numeric_limits<double>::quiet_NaN(); }
struct TrackingHealthDiagnostics {
    bool evaluated=false, accepted=false, stationary_candidate=false, dynamics_available=false, hard_fault=false, map_updated=false;
    bool stationary_evidence_held=false, output_pose_baseline=false;
    std::string reason;
    std::array<double,3> velocity_w{{HealthUnknown(),HealthUnknown(),HealthUnknown()}};
    std::array<double,3> gyro_bias{{HealthUnknown(),HealthUnknown(),HealthUnknown()}};
    std::array<double,3> accel_bias{{HealthUnknown(),HealthUnknown(),HealthUnknown()}};
    std::array<double,3> recovery_fit_velocity_w{{HealthUnknown(),HealthUnknown(),HealthUnknown()}};
    std::array<double,3> recovery_fit_gyro_bias{{HealthUnknown(),HealthUnknown(),HealthUnknown()}};
    std::array<double,3> recovery_fit_accel_bias{{HealthUnknown(),HealthUnknown(),HealthUnknown()}};
    int visual_inliers=0, visual_cells=0, flow_matches=0, imu_window_samples=0;
    double visual_rms_px=HealthUnknown(), fused_rms_px=HealthUnknown();
    double visual_translation_error_m=HealthUnknown(), visual_rotation_error_rad=HealthUnknown();
    double pose_step_m=HealthUnknown(), pose_step_dt_s=HealthUnknown();
    double output_pose_step_m=HealthUnknown(), output_pose_step_dt_s=HealthUnknown();
    double flow_median_px_s=HealthUnknown(), flow_p90_px_s=HealthUnknown();
    double imu_gyro_mean_norm=HealthUnknown(), imu_gyro_std=HealthUnknown();
    double imu_acc_mean_norm=HealthUnknown(), imu_acc_std=HealthUnknown();
    double stationary_duration_s=HealthUnknown(), stationary_displacement_m=HealthUnknown();
    double recovery_fit_position_rms_m=HealthUnknown(), recovery_fit_rotation_rms_rad=HealthUnknown();
    double recovery_holdout_position_rms_m=HealthUnknown(), recovery_holdout_rotation_rms_rad=HealthUnknown();
    double recovery_fit_span_s=HealthUnknown(), recovery_holdout_span_s=HealthUnknown();
    double recovery_fit_condition_number=HealthUnknown();
};
// Handheld gross-failure gates, not a precision/zero-motion certificate.
// Off mode does not call this gate. No pose is clamped or replaced by a zero pose.
struct TrackingHealthLimits {
    double max_speed_m_s=8.0, max_gyro_bias_rad_s=0.3, max_accel_bias_m_s2=2.0;
    double pose_step_margin_m=0.10, max_pose_speed_m_s=8.0;
    double max_visual_translation_error_m=0.12, max_visual_rotation_error_rad=0.12;
    int min_visual_inliers=20, min_visual_cells=3;
    double stationary_min_duration_s=0.7, stationary_max_speed_m_s=0.15;
    double stationary_max_displacement_m=0.05;
    double stationary_evidence_hold_s=0.5;
};
// The caller supplies an independent visual fit and a fused state from the same
// frame, before publishing or creating a keyframe. Missing required data fails.
bool EvaluateTrackingHealth(TrackingHealthDiagnostics& d, bool finite_pose,
                            const TrackingHealthLimits& limits=TrackingHealthLimits());
bool IsStationaryEvidence(const TrackingHealthDiagnostics& d);
// Immutable snapshot of a previously accepted published body position. It is
// owned by Tracking, never by a Frame or optimizable KeyFrame.
struct OutputPoseBaseline {
    bool available=false;
    long map_id=-1;
    unsigned long frame_id=0;
    double time=0;
    std::array<double,3> body_position{{0,0,0}};
};
void MeasurePublishedPoseStep(TrackingHealthDiagnostics& d,const OutputPoseBaseline& previous,
    long map_id,unsigned long frame_id,double time,const std::array<double,3>& body_position);

struct StationaryEvidenceState {
    double started_s=-1, last_full_s=-1, last_update_s=-1;
    bool confirmed=false;
};
// Preserve an already-confirmed visual/gyro-static interval briefly across an
// accelerometer transient. A real image/gyro change, missing coverage, invalid
// time or nonfinite statistic releases it immediately. This is not a ZUPT.
void UpdateStationaryEvidence(TrackingHealthDiagnostics& d,double time,
    StationaryEvidenceState& state,const TrackingHealthLimits& limits=TrackingHealthLimits());
}
#endif
