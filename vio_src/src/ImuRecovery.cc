// mono_imu_vio: fresh visual-inertial alignment after map relocalization.
#include "ImuRecovery.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <memory>

namespace ORB_SLAM3 {
namespace {
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using M6 = Eigen::Matrix<double,6,6>;
using V6 = Eigen::Matrix<double,6,1>;
V3 LogRotation(const M3& r) {
    const Eigen::AngleAxisd angle(r);
    return angle.axis() * angle.angle();
}
struct Segment { EIGEN_MAKE_ALIGNED_OPERATOR_NEW V3 a,w; double dt; };
using Segments = std::vector<Segment,Eigen::aligned_allocator<Segment>>;

bool MakeSegments(double begin, double end, const std::deque<IMU::Point>& s,
                  Segments& out) {
    if(s.size()<2 || !(end>begin) || begin<s.front().t-0.004 || end>s.back().t+0.004)
        return false;
    auto at = [&](double t, V3& a,V3& w) {
        auto right=std::lower_bound(s.begin(),s.end(),t,
            [](const IMU::Point& x,double y){return x.t<y;});
        if(right==s.begin()){a=right->a.cast<double>(); w=right->w.cast<double>(); return;}
        if(right==s.end()){a=s.back().a.cast<double>(); w=s.back().w.cast<double>(); return;}
        auto left=right-1;
        double u=(t-left->t)/(right->t-left->t);
        a=(1-u)*left->a.cast<double>()+u*right->a.cast<double>();
        w=(1-u)*left->w.cast<double>()+u*right->w.cast<double>();
    };
    double t=begin;
    auto it=std::upper_bound(s.begin(),s.end(),begin,
        [](double x,const IMU::Point& y){return x<y.t;});
    while(t<end-1e-9){
        double next=it==s.end()?end:std::min(end,it->t);
        if(next<=t){++it;continue;}
        if(next-t>0.010)return false;
        V3 a0,w0,a1,w1;at(t,a0,w0);at(next,a1,w1);
        out.push_back({(a0+a1)*0.5,(w0+w1)*0.5,next-t});
        t=next;if(it!=s.end() && it->t<=t+1e-9)++it;
    }
    return !out.empty();
}
std::unique_ptr<IMU::Preintegrated> Integrate(const Segments& s,const V3& bg,
                                           const V3& ba,const IMU::Calib& c) {
    IMU::Bias b(ba.x(),ba.y(),ba.z(),bg.x(),bg.y(),bg.z());
    std::unique_ptr<IMU::Preintegrated> p(new IMU::Preintegrated(b,c));
    for(const auto& x:s)p->IntegrateNewMeasurement(x.a.cast<float>(),x.w.cast<float>(),x.dt);
    return p;
}
M3 IntegrateRotation(const Segments& segments,const V3& bg) {
    M3 rotation=M3::Identity();
    // Bias fitting needs rotations only. Avoid repeatedly propagating a full
    // 15x15 covariance for the finite-difference evaluations on each frame.
    for(const auto& sample:segments){
        V3 angle=(sample.w-bg)*sample.dt;
        double norm=angle.norm();
        if(norm>1e-12)rotation=rotation*Eigen::AngleAxisd(norm,angle/norm).toRotationMatrix();
    }
    return rotation;
}
}

ImuRecoveryResult SolveImuRecovery(const RecoveryPoses& poses,
                                 const std::deque<IMU::Point>& samples,
                                 const IMU::Calib& calibration) {
    ImuRecoveryResult out;
    auto fail=[&](const std::string& reason){out.reason=reason;return out;};
    if(poses.size()<12 || poses.back().t-poses.front().t<1.0)
        return fail("need >=12 visual poses spanning >=1 second");
    if(poses.back().t-poses.front().t>4.0)return fail("alignment window too long");
    if(samples.size()<2)return fail("insufficient fresh IMU samples");
    for(size_t i=0;i<samples.size();++i){
        if(!std::isfinite(samples[i].t)||!samples[i].a.allFinite()||!samples[i].w.allFinite())
            return fail("nonfinite IMU sample");
        if(i && samples[i].t<=samples[i-1].t)return fail("nonmonotonic IMU samples");
        // Check gaps only inside the actual recovery interval.
        if(i && samples[i].t>=poses.front().t && samples[i-1].t<=poses.back().t &&
           samples[i].t-samples[i-1].t>0.010)return fail("IMU sample gap exceeds 10 ms");
    }
    std::vector<Segments> segments(poses.size()-1);
    for(size_t i=0;i<poses.size();++i){
        if(!std::isfinite(poses[i].t)||!poses[i].Rwb.allFinite()||!poses[i].pwb.allFinite())
            return fail("nonfinite visual pose");
        if((poses[i].Rwb.transpose()*poses[i].Rwb-M3::Identity()).norm()>1e-3 ||
           std::abs(poses[i].Rwb.determinant()-1)>1e-3)return fail("invalid visual rotation");
        if(i){
            double dt=poses[i].t-poses[i-1].t;
            if(dt<=0 || dt>0.25)return fail("visual pose gap exceeds 250 ms");
            if((poses[i].pwb-poses[i-1].pwb).norm()/dt>8 ||
               LogRotation(poses[i-1].Rwb.transpose()*poses[i].Rwb).norm()/dt>12)
                return fail("inconsistent visual pose jump");
            if(!MakeSegments(poses[i-1].t,poses[i].t,samples,segments[i-1]))
                return fail("IMU interval not covered");
        }
    }
    // Gyro bias is observed by metric-map visual rotations. Numerical Jacobians
    // use freshly integrated measurements, not stale KF preintegration objects.
    V3 bg=V3::Zero();M3 gyroH=M3::Zero();
    for(int iteration=0;iteration<7;++iteration){
        gyroH.setZero();V3 gradient=V3::Zero();
        for(size_t i=0;i<segments.size();++i){
            M3 visual=poses[i].Rwb.transpose()*poses[i+1].Rwb;
            V3 r=LogRotation(IntegrateRotation(segments[i],bg).transpose()*visual);
            M3 J;
            for(int k=0;k<3;++k){
                V3 b=bg;b[k]+=1e-4;
                J.col(k)=(LogRotation(IntegrateRotation(segments[i],b).transpose()*visual)-r)/1e-4;
            }
            gyroH.noalias()+=J.transpose()*J;gradient.noalias()+=J.transpose()*r;
        }
        Eigen::SelfAdjointEigenSolver<M3> eig(gyroH);
        if(eig.info()!=Eigen::Success || eig.eigenvalues().minCoeff()<1e-6)
            return fail("gyro bias fit ill conditioned: min="+std::to_string(eig.eigenvalues().minCoeff())+
                        " trace="+std::to_string(gyroH.trace())+" intervals="+std::to_string(segments.size()));
        V3 step=gyroH.ldlt().solve(-gradient);
        if(!step.allFinite())return fail("gyro bias fit failed");
        bg+=step;
        if(bg.norm()>0.3)return fail("estimated gyro bias exceeds 0.3 rad/s");
        if(step.norm()<1e-5)break;
    }
    std::vector<std::unique_ptr<IMU::Preintegrated>> integrations;
    double rotationSse=0;
    for(size_t i=0;i<segments.size();++i){
        auto p=Integrate(segments[i],bg,V3::Zero(),calibration);
        V3 r=LogRotation(p->dR.cast<double>().transpose()*poses[i].Rwb.transpose()*poses[i+1].Rwb);
        rotationSse+=r.squaredNorm();
        if(r.norm()>0.06)return fail("visual/gyro rotation residual exceeds 0.06 rad");
        integrations.push_back(std::move(p));
    }
    out.rotation_rms=std::sqrt(rotationSse/segments.size());
    if(out.rotation_rms>0.025)return fail("visual/gyro rotation RMS exceeds 0.025 rad");

    // Unknowns are first-window velocity and constant accelerometer bias.
    // Positions and rotations are fixed by independently verified mature-map
    // visual poses; gravity is the saved metric map's aligned negative z axis.
    const V3 gravity(0,0,-IMU::GRAVITY_VALUE);
    V3 dp=V3::Zero(),dv=V3::Zero();M3 Bp=M3::Zero(),Bv=M3::Zero();
    Eigen::MatrixXd A(3*(poses.size()-1),6);Eigen::VectorXd rhs(A.rows());
    for(size_t i=0;i<integrations.size();++i){
        const auto& p=*integrations[i];double dt=p.dT;
        const M3& R=poses[i].Rwb;
        dp+=dv*dt+0.5*gravity*dt*dt+R*p.dP.cast<double>();
        Bp+=Bv*dt+R*p.JPa.cast<double>();
        dv+=gravity*dt+R*p.dV.cast<double>();Bv+=R*p.JVa.cast<double>();
        double elapsed=poses[i+1].t-poses[0].t;
        A.block<3,3>(3*i,0)=elapsed*M3::Identity();A.block<3,3>(3*i,3)=Bp;
        rhs.segment<3>(3*i)=poses[i+1].pwb-poses[0].pwb-dp;
    }
    M6 H=A.transpose()*A;
    Eigen::SelfAdjointEigenSolver<M6> eig(H);
    if(eig.info()!=Eigen::Success || eig.eigenvalues().minCoeff()<1e-5 ||
       eig.eigenvalues().maxCoeff()/eig.eigenvalues().minCoeff()>1e6)
        return fail("velocity/accelerometer fit ill conditioned");
    out.condition_number=eig.eigenvalues().maxCoeff()/eig.eigenvalues().minCoeff();
    V6 x=H.ldlt().solve(A.transpose()*rhs);
    if(!x.allFinite())return fail("nonfinite velocity/bias fit");
    const V3 ba=x.tail<3>();
    if(ba.norm()>2.0)return fail("estimated accelerometer bias exceeds 2 m/s^2");
    V3 velocity=x.head<3>()+dv+Bv*ba;
    if(velocity.norm()>8)return fail("estimated speed exceeds 8 m/s");
    Eigen::VectorXd residual=A*x-rhs;
    out.position_rms=std::sqrt(residual.squaredNorm()/(poses.size()-1));
    for(int i=0;i<residual.rows();i+=3)
        if(residual.segment<3>(i).norm()>0.06)return fail("visual/IMU position residual exceeds 6 cm");
    if(out.position_rms>0.025)return fail("visual/IMU position RMS exceeds 2.5 cm");

    // A heuristic uncertainty-floored prior for the first frame-to-frame
    // inertial optimization. Correlations between separately fitted variables
    // are not available, so this is not a statistically conservative covariance.
    // Least-squares covariance is enlarged by floors to account for correlated
    // visual poses and model error; these values are not an accuracy certificate.
    M6 covariance=H.ldlt().solve(M6::Identity())*std::max(0.0001,out.position_rms*out.position_rms);
    Eigen::Matrix<double,3,6> Jv;Jv.leftCols<3>()=M3::Identity();Jv.rightCols<3>()=Bv;
    V3 varianceVelocity=(Jv*covariance*Jv.transpose()).diagonal();
    V3 varianceGyro=(gyroH.ldlt().solve(M3::Identity())*std::max(0.0001,out.rotation_rms*out.rotation_rms)).diagonal();
    for(int k=0;k<3;++k){
        out.information(k,k)=1.0/(0.03*0.03);
        out.information(k+3,k+3)=1.0/(0.03*0.03);
        out.information(k+6,k+6)=1.0/std::max(0.04,varianceVelocity[k]);
        out.information(k+9,k+9)=1.0/std::max(0.0004,varianceGyro[k]);
        out.information(k+12,k+12)=1.0/std::max(0.04,covariance(k+3,k+3));
    }
    out.bias=IMU::Bias(ba.x(),ba.y(),ba.z(),bg.x(),bg.y(),bg.z());
    out.velocity=velocity.cast<float>();out.valid=true;out.reason="fresh visual-inertial alignment accepted";
    return out;
}

ImuRecoveryValidation ValidateImuRecovery(const RecoveryPose& first,
    const ImuRecoveryResult& fit, const RecoveryPoses& holdout,
    const std::deque<IMU::Point>& samples, const IMU::Calib& calibration) {
    ImuRecoveryValidation out;
    auto fail=[&](const std::string& reason){out.reason=reason;return out;};
    if(!fit.valid || !fit.velocity.allFinite() || !first.Rwb.allFinite() || !first.pwb.allFinite())
        return fail("invalid frozen recovery state");
    if(holdout.size()<8 || holdout.back().t-first.t<0.6)
        return fail("need >=8 fresh holdout poses spanning >=0.6 second");
    if(holdout.back().t-first.t>2.0)return fail("recovery holdout interval too long");
    if(samples.size()<2)return fail("holdout IMU samples absent");
    for(size_t i=0;i<samples.size();++i){
        if(!std::isfinite(samples[i].t)||!samples[i].a.allFinite()||!samples[i].w.allFinite())
            return fail("nonfinite holdout IMU sample");
        if(i && samples[i].t<=samples[i-1].t)return fail("nonmonotonic holdout IMU sample");
        if(i && samples[i].t>=first.t && samples[i-1].t<=holdout.back().t &&
           samples[i].t-samples[i-1].t>0.010)return fail("holdout IMU gap exceeds 10 ms");
    }
    const V3 bg(fit.bias.bwx,fit.bias.bwy,fit.bias.bwz);
    const V3 ba(fit.bias.bax,fit.bias.bay,fit.bias.baz), gravity(0,0,-IMU::GRAVITY_VALUE);
    if(!bg.allFinite()||!ba.allFinite())return fail("nonfinite frozen recovery bias");
    double p2=0,r2=0,previous=first.t;size_t compared=0;
    for(const auto& pose:holdout){
        if(!std::isfinite(pose.t)||!pose.Rwb.allFinite()||!pose.pwb.allFinite()||
           pose.t<=previous||pose.t-previous>0.25)return fail("invalid holdout visual interval");
        Segments segments;
        if(!MakeSegments(first.t,pose.t,samples,segments))return fail("holdout IMU interval uncovered");
        auto pre=Integrate(segments,bg,ba,calibration);
        const double dt=pose.t-first.t;
        const V3 predicted=first.pwb+fit.velocity.cast<double>()*dt+0.5*gravity*dt*dt+
                           first.Rwb*pre->dP.cast<double>();
        const M3 rotation=first.Rwb*pre->dR.cast<double>();
        double pe=(predicted-pose.pwb).norm(),re=LogRotation(rotation.transpose()*pose.Rwb).norm();
        p2+=pe*pe;r2+=re*re;
        ++compared;out.position_rms=std::sqrt(p2/compared);
        out.rotation_rms=std::sqrt(r2/compared);
        if(pe>0.06||re>0.06)return fail("frozen recovery state failed future visual/IMU check");
        out.velocity=(fit.velocity.cast<double>()+gravity*dt+first.Rwb*pre->dV.cast<double>()).cast<float>();
        previous=pose.t;
    }
    if(out.position_rms>0.025||out.rotation_rms>0.025)
        return fail("recovery holdout RMS too large");
    if(!out.velocity.allFinite()||out.velocity.norm()>8)return fail("recovery holdout speed invalid");
    out.valid=true;out.reason="fresh visual-inertial alignment and future-data validation accepted";
    return out;
}

}
