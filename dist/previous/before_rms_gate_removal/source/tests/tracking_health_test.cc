// Gross-failure and static-consistency counterexamples, independent of a camera.
#include "TrackingHealth.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3;
void Require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
TrackingHealthDiagnostics Good(){
    TrackingHealthDiagnostics d;d.dynamics_available=true;
    d.velocity_w={{0.02,-0.01,0.01}};d.gyro_bias={{0.005,-0.004,0.003}};d.accel_bias={{0.05,-0.03,0.02}};
    d.visual_inliers=100;d.visual_cells=8;d.visual_rms_px=0.6;d.fused_rms_px=0.8;
    d.visual_translation_error_m=0.005;d.visual_rotation_error_rad=0.004;
    d.pose_step_dt_s=0.05;d.pose_step_m=0.005;
    return d;
}
int main(){try{
    auto d=Good();Require(EvaluateTrackingHealth(d,true),"valid noisy state rejected");
    d=Good();d.velocity_w={{2.0,-0.8,0.2}};d.pose_step_m=0.12;
    Require(EvaluateTrackingHealth(d,true),"plausible fast handheld motion rejected");
    d=Good();Require(!EvaluateTrackingHealth(d,false)&&d.hard_fault,"nonfinite pose accepted");
    d=Good();d.velocity_w[0]=HealthUnknown();
    Require(!EvaluateTrackingHealth(d,true)&&d.hard_fault,"nonfinite velocity accepted");
    d=Good();d.dynamics_available=false;
    Require(!EvaluateTrackingHealth(d,true)&&!d.hard_fault,"startup absence classified as corrupt map");
    d=Good();d.velocity_w[0]=12;
    Require(!EvaluateTrackingHealth(d,true)&&d.hard_fault,"excessive finite speed accepted");
    d=Good();d.accel_bias[1]=3;
    Require(!EvaluateTrackingHealth(d,true)&&d.hard_fault,"excessive bias accepted");
    // Exact finite jump observed in the user's static-tail trace, frames 3706/7.
    const double x= -10.0429697037+17.2596035004;
    const double y= -8.28586483002+2.72302889824;
    const double z= 2.11711788177-1.45205140114;
    d=Good();d.pose_step_dt_s=0.066663984;d.pose_step_m=std::sqrt(x*x+y*y+z*z);
    Require(d.pose_step_m>9 && !EvaluateTrackingHealth(d,true)&&d.hard_fault,"recorded 9.136m jump accepted");
    d=Good();d.visual_translation_error_m=0.3;
    Require(!EvaluateTrackingHealth(d,true)&&d.hard_fault,"visual/fused disagreement accepted");
    d=Good();d.visual_inliers=3;
    Require(!EvaluateTrackingHealth(d,true)&&!d.hard_fault,"occlusion classified as numerical corruption");
    d=Good();d.pose_step_dt_s=0.5;
    Require(!EvaluateTrackingHealth(d,true)&&!d.hard_fault,"sample gap accepted");
    d=Good();d.flow_matches=80;d.flow_median_px_s=0.2;d.flow_p90_px_s=1;
    d.imu_window_samples=150;d.imu_gyro_mean_norm=0.01;d.imu_gyro_std=0.003;
    d.imu_acc_mean_norm=9.82;d.imu_acc_std=0.03;
    Require(IsStationaryEvidence(d),"clean static image plus IMU not recognized");
    d.stationary_candidate=true;d.stationary_duration_s=1;d.stationary_displacement_m=0.01;
    Require(EvaluateTrackingHealth(d,true),"static stable state rejected");
    d.velocity_w={{0.25,0,0}};
    Require(!EvaluateTrackingHealth(d,true)&&d.hard_fault,"static motion conflict accepted");
    d.velocity_w={{0,0,0}};d.stationary_displacement_m=0.08;
    Require(!EvaluateTrackingHealth(d,true)&&d.hard_fault,"slow static displacement creep accepted");
    d.flow_median_px_s=50;d.flow_p90_px_s=90;
    Require(!IsStationaryEvidence(d),"quiet IMU with moving image falsely called stationary");
    d.stationary_candidate=false;d.velocity_w={{0.25,0,0}};
    Require(EvaluateTrackingHealth(d,true),"constant velocity wrongly forced to stationary");
    d.flow_median_px_s=HealthUnknown();d.flow_matches=0;
    Require(!IsStationaryEvidence(d),"IMU alone labeled stationary");
    // Temporal counterexample: a long-static camera sees no image/gyro change,
    // but an accelerometer step raises the 0.3s-window deviation. Preserve the
    // established anchor instead of starting another 0.7s grace period.
    auto quiet=Good();quiet.flow_matches=80;quiet.flow_median_px_s=0.2;quiet.flow_p90_px_s=1;
    quiet.imu_window_samples=150;quiet.imu_gyro_mean_norm=0.01;quiet.imu_gyro_std=0.003;
    quiet.imu_acc_mean_norm=9.82;quiet.imu_acc_std=0.03;
    StationaryEvidenceState state;
    for(int i=0;i<=20;++i){d=quiet;UpdateStationaryEvidence(d,i*0.05,state);}
    Require(state.confirmed && d.stationary_candidate && !d.stationary_evidence_held,"stable interval not confirmed");
    d=quiet;d.imu_acc_std=0.49;UpdateStationaryEvidence(d,1.05,state);
    Require(d.stationary_candidate && d.stationary_evidence_held && d.stationary_duration_s>=1,
            "accelerometer transient erased established visual/gyro-static interval");
    d.velocity_w={{0.25,0,0}};d.stationary_displacement_m=0.02;
    Require(!EvaluateTrackingHealth(d,true)&&d.hard_fault,"transient reopened grace period for false velocity");
    d=quiet;d.imu_acc_std=0.31;UpdateStationaryEvidence(d,1.1,state);
    Require(d.stationary_evidence_held,"second transient frame lost anchor");
    d=quiet;UpdateStationaryEvidence(d,1.2,state);
    Require(!d.stationary_evidence_held && d.stationary_duration_s>=1.2,"full evidence restart lost old anchor");
    d=quiet;d.imu_acc_std=0.4;d.flow_median_px_s=20;UpdateStationaryEvidence(d,1.25,state);
    Require(!d.stationary_candidate && !d.stationary_evidence_held,"real image motion failed to release anchor");
    d=quiet;UpdateStationaryEvidence(d,1.3,state);
    d=quiet;d.imu_acc_std=0.4;UpdateStationaryEvidence(d,1.35,state);
    Require(!d.stationary_candidate,"unconfirmed fresh interval incorrectly held");
    state=StationaryEvidenceState();
    for(int i=0;i<=20;++i){d=quiet;UpdateStationaryEvidence(d,2+i*0.05,state);}
    d=quiet;d.imu_acc_std=0.4;d.imu_gyro_mean_norm=0.2;UpdateStationaryEvidence(d,3.05,state);
    Require(!d.stationary_candidate,"real gyro motion failed to release anchor");
    state=StationaryEvidenceState();
    for(int i=0;i<=20;++i){d=quiet;UpdateStationaryEvidence(d,4+i*0.05,state);}
    for(int i=1;i<=11;++i){d=quiet;d.imu_acc_std=0.4;UpdateStationaryEvidence(d,5+i*0.05,state);}
    Require(!d.stationary_candidate,"accelerometer transient held indefinitely");
    state=StationaryEvidenceState();
    for(int i=0;i<=20;++i){d=quiet;UpdateStationaryEvidence(d,6+i*0.05,state);}
    d=quiet;d.imu_acc_std=HealthUnknown();UpdateStationaryEvidence(d,7.05,state);
    Require(!d.stationary_candidate,"missing/nonfinite IMU statistics were held");
    state=StationaryEvidenceState();
    for(int i=0;i<=20;++i){d=quiet;UpdateStationaryEvidence(d,8+i*0.05,state);}
    d=quiet;UpdateStationaryEvidence(d,9.4,state);
    Require(!d.stationary_candidate,"camera gap continued static interval");
    // A common backend coordinate jump can rewrite internal last-frame state;
    // it must not rewrite what the application actually received last frame.
    OutputPoseBaseline baseline;baseline.available=true;baseline.map_id=7;
    baseline.frame_id=3706;baseline.time=100;
    baseline.body_position={{-17.2596035004,-2.72302889824,1.45205140114}};
    d=Good();d.pose_step_m=0;d.map_updated=true;
    MeasurePublishedPoseStep(d,baseline,7,3707,100.066663984,{{-10.0429697037,-8.28586483002,2.11711788177}});
    Require(d.output_pose_baseline && d.output_pose_step_m>9,"published jump not measured");
    Require(!EvaluateTrackingHealth(d,true)&&d.hard_fault&&d.reason.find("backend map update")!=std::string::npos,
            "updated internal frame hid published coordinate discontinuity");
    d=Good();MeasurePublishedPoseStep(d,baseline,7,3707,100.05,{{-17.24,-2.72,1.45}});
    Require(d.output_pose_baseline && EvaluateTrackingHealth(d,true),"normal continuous output rejected");
    d=Good();MeasurePublishedPoseStep(d,baseline,8,3707,100.05,{{100,100,100}});
    Require(!d.output_pose_baseline&&EvaluateTrackingHealth(d,true),"new-map origins compared as physical motion");
    d=Good();MeasurePublishedPoseStep(d,OutputPoseBaseline(),7,3707,100.05,{{100,100,100}});
    Require(!d.output_pose_baseline&&EvaluateTrackingHealth(d,true),"recovery reset retained old output baseline");
    d=Good();MeasurePublishedPoseStep(d,baseline,7,3710,100.10,{{100,100,100}});
    Require(!d.output_pose_baseline,"nonconsecutive valid frames compared as continuous output");
    d=Good();MeasurePublishedPoseStep(d,baseline,7,3707,101.0,{{100,100,100}});
    Require(!d.output_pose_baseline,"long input gap compared as adjacent motion");
    std::cout<<"PUBLISHED_POSE_CONTINUITY_PASS\n";
    std::cout<<"TRACKING_HEALTH_COUNTEREXAMPLES_PASS\n";
    std::cout<<"STATIONARY_EVIDENCE_TRANSITION_PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
