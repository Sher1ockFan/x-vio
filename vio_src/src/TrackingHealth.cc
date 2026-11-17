#include "TrackingHealth.h"
#include <cmath>
namespace ORB_SLAM3 {
namespace {
double Norm(const std::array<double,3>& a) {
    double s=0;for(double x:a){if(!std::isfinite(x))return HealthUnknown();s+=x*x;}return std::sqrt(s);
}
}
namespace {
bool IsQuietVisualGyroEvidence(const TrackingHealthDiagnostics& d) {
    return d.flow_matches>=40 && d.visual_cells>=4 && d.imu_window_samples>=100 &&
        std::isfinite(d.flow_median_px_s) && d.flow_median_px_s<3.0 &&
        std::isfinite(d.flow_p90_px_s) && d.flow_p90_px_s<12.0 &&
        std::isfinite(d.imu_gyro_mean_norm) && d.imu_gyro_mean_norm<0.03 &&
        std::isfinite(d.imu_gyro_std) && d.imu_gyro_std<0.02 &&
        std::isfinite(d.imu_acc_mean_norm) && std::isfinite(d.imu_acc_std);
}
}
bool IsStationaryEvidence(const TrackingHealthDiagnostics& d) {
    // Image evidence is essential: quiet IMU alone cannot distinguish standstill
    // from constant-velocity translation. This is only a consistency candidate.
    return IsQuietVisualGyroEvidence(d) && std::abs(d.imu_acc_mean_norm-9.81)<0.5 && d.imu_acc_std<0.15;
}
void UpdateStationaryEvidence(TrackingHealthDiagnostics& d,double time,
    StationaryEvidenceState& state,const TrackingHealthLimits& limits) {
    d.stationary_candidate=false;d.stationary_evidence_held=false;
    d.stationary_duration_s=HealthUnknown();
    if(!std::isfinite(time) || (state.last_update_s>=0 &&
       (time<=state.last_update_s || time-state.last_update_s>0.25))){
        state=StationaryEvidenceState();
        // Do not immediately reinterpret a gap/clock reversal as a continuation.
        if(std::isfinite(time))state.last_update_s=time;
        return;
    }
    if(IsStationaryEvidence(d)){
        if(state.started_s<0)state.started_s=time;
        state.last_full_s=time;
        if(time-state.started_s>=limits.stationary_min_duration_s)state.confirmed=true;
        d.stationary_candidate=true;
    }else if(state.confirmed && IsQuietVisualGyroEvidence(d) && state.last_full_s>=0 &&
             time-state.last_full_s<=limits.stationary_evidence_hold_s){
        // Previously confirmed image/gyro evidence remains. An accelerometer
        // step must not erase the old anchor and open another warm-up interval.
        d.stationary_candidate=true;d.stationary_evidence_held=true;
    }else state=StationaryEvidenceState();
    state.last_update_s=time;
    if(d.stationary_candidate)d.stationary_duration_s=time-state.started_s;
}
void MeasurePublishedPoseStep(TrackingHealthDiagnostics& d,const OutputPoseBaseline& previous,
    long map_id,unsigned long frame_id,double time,const std::array<double,3>& body_position) {
    d.output_pose_baseline=false;d.output_pose_step_m=HealthUnknown();d.output_pose_step_dt_s=HealthUnknown();
    const double dt=time-previous.time;
    if(!previous.available || previous.map_id!=map_id || frame_id!=previous.frame_id+1 ||
       !std::isfinite(dt) || dt<=0 || dt>0.25)return;
    d.output_pose_baseline=true;d.output_pose_step_dt_s=dt;
    std::array<double,3> delta;
    for(int k=0;k<3;++k)delta[k]=body_position[k]-previous.body_position[k];
    d.output_pose_step_m=Norm(delta);
}
bool EvaluateTrackingHealth(TrackingHealthDiagnostics& d,bool finite_pose,const TrackingHealthLimits& l) {
    d.evaluated=true;d.accepted=false;d.hard_fault=false;
    auto fail=[&](const char* reason,bool hard=false){d.reason=reason;d.hard_fault=hard;return false;};
    if(!finite_pose)return fail("health: nonfinite pose",true);
    const double speed=Norm(d.velocity_w),bg=Norm(d.gyro_bias),ba=Norm(d.accel_bias);
    if(!d.dynamics_available)return fail("health: velocity or bias unavailable");
    if(!std::isfinite(speed)||!std::isfinite(bg)||!std::isfinite(ba))
        return fail("health: nonfinite velocity or bias",true);
    if(speed>l.max_speed_m_s)return fail("health: implausible body speed",true);
    if(bg>l.max_gyro_bias_rad_s || ba>l.max_accel_bias_m_s2)
        return fail("health: implausible IMU bias",true);
    if(!std::isfinite(d.pose_step_dt_s)||d.pose_step_dt_s<=0||d.pose_step_dt_s>0.25)
        return fail("health: invalid pose interval");
    if(!std::isfinite(d.pose_step_m)||d.pose_step_m>l.pose_step_margin_m+l.max_pose_speed_m_s*d.pose_step_dt_s)
        return fail(d.map_updated?"health: pose discontinuity across backend map update":
                                 "health: implausible pose step",true);
    if(d.output_pose_baseline &&
       (!std::isfinite(d.output_pose_step_m)||!std::isfinite(d.output_pose_step_dt_s)||
        d.output_pose_step_dt_s<=0 || d.output_pose_step_dt_s>0.25 ||
        d.output_pose_step_m>l.pose_step_margin_m+l.max_pose_speed_m_s*d.output_pose_step_dt_s))
        return fail(d.map_updated?"health: published pose discontinuity across backend map update":
                                 "health: implausible published pose step",true);
    if(d.visual_inliers<l.min_visual_inliers||d.visual_cells<l.min_visual_cells)
        return fail("health: weak independent visual support");
    // Finite pixel RMS values remain diagnostic only. Do not interrupt tracking
    // solely because a visual/fused reprojection RMS exceeds a fixed pixel limit.
    if(!std::isfinite(d.visual_rms_px)||!std::isfinite(d.fused_rms_px))
        return fail("health: nonfinite reprojection residual");
    if(!std::isfinite(d.visual_translation_error_m)||!std::isfinite(d.visual_rotation_error_rad)||
       d.visual_translation_error_m>l.max_visual_translation_error_m ||
       d.visual_rotation_error_rad>l.max_visual_rotation_error_rad)
        return fail("health: visual/inertial pose disagreement",true);
    if(d.stationary_candidate && std::isfinite(d.stationary_duration_s) &&
       d.stationary_duration_s>=l.stationary_min_duration_s &&
       (speed>l.stationary_max_speed_m_s ||
        (std::isfinite(d.stationary_displacement_m)&&d.stationary_displacement_m>l.stationary_max_displacement_m)))
        return fail("health: static image/IMU evidence conflicts with estimated motion",true);
    d.accepted=true;d.reason="health checks passed";return true;
}
}
