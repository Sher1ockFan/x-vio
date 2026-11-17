// Analytic regression tests for fresh-session IMU recovery (no camera hardware).
#include "ImuRecovery.h"
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace ORB_SLAM3;
using V3=Eigen::Vector3d;
using M3=Eigen::Matrix3d;
static void Require(bool value,const std::string& why){
    if(!value)throw std::runtime_error(why);
}
static M3 Rotation(const V3& omega,double t){
    return omega.norm()>0?Eigen::AngleAxisd(omega.norm()*t,omega.normalized()).toRotationMatrix():M3::Identity();
}
struct Fixture {
    RecoveryPoses poses;
    std::deque<IMU::Point> imu;
    V3 finalVelocity,gyroBias,accelBias;
};
static Fixture MakeFixture(bool moving,double duration=1.5){
    Fixture f;
    const V3 velocity=moving?V3(0.35,-0.25,0.12):V3::Zero();
    const V3 acceleration=moving?V3(0.28,0.16,-0.09):V3::Zero();
    const V3 omega=moving?V3(0.32,-0.18,0.27):V3::Zero();
    f.gyroBias=V3(0.012,-0.008,0.015);f.accelBias=V3(0.08,-0.11,0.065);
    const V3 gravity(0,0,-IMU::GRAVITY_VALUE),origin(0.3,-0.1,0.7);
    for(int i=0;i<=int(duration/0.05+0.1);++i){
        double t=i*0.05;
        f.poses.push_back({10+t,Rotation(omega,t),origin+velocity*t+0.5*acceleration*t*t});
    }
    for(int i=-2;i<=int(duration/0.002+2.1);++i){
        double t=i*0.002;
        V3 specific=Rotation(omega,t).transpose()*(acceleration-gravity)+f.accelBias;
        V3 gyro=omega+f.gyroBias;
        f.imu.emplace_back(specific.x(),specific.y(),specific.z(),gyro.x(),gyro.y(),gyro.z(),10+t);
    }
    f.finalVelocity=velocity+acceleration*duration;
    return f;
}
int main(){
    try{
        const IMU::Calib calibration(Sophus::SE3f(),0.001,0.01,0.00001,0.0001);
        for(bool moving:{false,true}){
            auto f=MakeFixture(moving);
            auto start=std::chrono::steady_clock::now();
            auto r=SolveImuRecovery(f.poses,f.imu,calibration);
            Require(r.valid,"analytic valid case rejected: "+r.reason);
            V3 gyro(r.bias.bwx,r.bias.bwy,r.bias.bwz),accel(r.bias.bax,r.bias.bay,r.bias.baz);
            Require((gyro-f.gyroBias).norm()<0.002,"gyro bias estimate inaccurate");
            Require((accel-f.accelBias).norm()<0.015,"accelerometer bias estimate inaccurate");
            Require((r.velocity.cast<double>()-f.finalVelocity).norm()<0.01,"fresh velocity inaccurate");
            Require(r.information.allFinite() && r.information.diagonal().minCoeff()>0,"invalid prior");
            std::cout<<(moving?"MOVING":"STATIONARY")<<" PASS elapsed_ms="
                <<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<"\n";
        }
        auto base=MakeFixture(true);
        auto broken=base;
        broken.imu.erase(broken.imu.begin()+250,broken.imu.begin()+270);
        Require(!SolveImuRecovery(broken.poses,broken.imu,calibration).valid,"IMU gap accepted");
        broken=base;broken.poses[15].pwb.x()+=0.12;
        Require(!SolveImuRecovery(broken.poses,broken.imu,calibration).valid,"visual position outlier accepted");
        broken=base;broken.poses[12].Rwb=Rotation(V3(1,0,0),0.2)*broken.poses[12].Rwb;
        Require(!SolveImuRecovery(broken.poses,broken.imu,calibration).valid,"visual rotation outlier accepted");
        broken=base;broken.poses.resize(8);
        Require(!SolveImuRecovery(broken.poses,broken.imu,calibration).valid,"short window accepted");
        broken=base;broken.poses[4].t=broken.poses[3].t;
        Require(!SolveImuRecovery(broken.poses,broken.imu,calibration).valid,"duplicate pose time accepted");
        broken=base;broken.imu[4].a.x()=std::numeric_limits<float>::quiet_NaN();
        Require(!SolveImuRecovery(broken.poses,broken.imu,calibration).valid,"nonfinite IMU accepted");
        broken=base;broken.imu.erase(broken.imu.begin()+600,broken.imu.end());
        Require(!SolveImuRecovery(broken.poses,broken.imu,calibration).valid,"uncovered IMU interval accepted");
        broken=base;for(auto& sample:broken.imu)sample.w.x()+=0.6f;
        Require(!SolveImuRecovery(broken.poses,broken.imu,calibration).valid,"implausible gyro bias accepted");
        std::cout<<"NEGATIVE_CASES PASS\n";
        for(bool moving:{false,true}){
            auto extended=MakeFixture(moving,2.2);
            RecoveryPoses train(extended.poses.begin(),extended.poses.begin()+25);
            RecoveryPoses holdout(extended.poses.begin()+25,extended.poses.end());
            auto fit=SolveImuRecovery(train,extended.imu,calibration);
            Require(fit.valid,"training fit rejected");
            auto check=ValidateImuRecovery(train.back(),fit,holdout,extended.imu,calibration);
            Require(check.valid,"valid future data rejected: "+check.reason);
            Require((check.velocity.cast<double>()-extended.finalVelocity).norm()<0.02,"holdout propagated velocity inaccurate");
            auto badFit=fit;badFit.velocity.x()+=0.4;
            Require(!ValidateImuRecovery(train.back(),badFit,holdout,extended.imu,calibration).valid,
                    "future data failed to reject wrong frozen velocity");
            badFit=fit;badFit.bias.bax+=0.8;
            Require(!ValidateImuRecovery(train.back(),badFit,holdout,extended.imu,calibration).valid,
                    "future data failed to reject wrong frozen accel bias");
            auto badImu=extended.imu;
            for(auto& sample:badImu)if(sample.t>train.back().t)sample.a.x()+=1.0;
            Require(!ValidateImuRecovery(train.back(),fit,holdout,badImu,calibration).valid,
                    "post-fit IMU disturbance not rejected");
            auto badPoses=holdout;badPoses[5].pwb.y()+=0.1;
            Require(!ValidateImuRecovery(train.back(),fit,badPoses,extended.imu,calibration).valid,
                    "holdout visual outlier accepted");
            badImu=extended.imu;badImu.erase(badImu.begin()+700,badImu.begin()+720);
            Require(!ValidateImuRecovery(train.back(),fit,holdout,badImu,calibration).valid,
                    "holdout IMU gap accepted");
        }
        std::cout<<"FROZEN_STATE_FUTURE_DATA_VALIDATION_PASS\n";
        return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<"\n";return 1;}
}
