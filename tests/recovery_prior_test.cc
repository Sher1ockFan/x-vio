// Analytic native regression for recovered-root soft dynamics. No sensor access.
#include "System.h"
#include "Optimizer.h"
#include "KannalaBrandt8.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <cassert>
#include <sstream>
#include <iostream>
using namespace ORB_SLAM3;

static Eigen::Vector3f BiasGyro(const IMU::Bias& b) {return {b.bwx,b.bwy,b.bwz};}
static Eigen::Vector3f BiasAcc(const IMU::Bias& b) {return {b.bax,b.bay,b.baz};}

struct Fixture {
    Map* map;
    KeyFrame* root;
    KeyFrame* latest;
    ORBVocabulary* vocabulary;
    Eigen::Vector3f initialVelocity;
    IMU::Bias initialBias;
};
static Fixture MakeFixture(ORBVocabulary* vocabulary,bool recovered) {
    auto map=new Map(0);
    map->SetInertialSensor();map->SetImuInitialized();map->SetIniertialBA1();map->SetIniertialBA2();
    auto database=new KeyFrameDatabase(*vocabulary);
    auto camera=new KannalaBrandt8({250.f,250.f,320.f,240.f,0.f,0.f,0.f,0.f});
    ORBextractor extractor(1500,1.2,8,20,7);
    cv::Mat image(480,640,CV_8U), distortion=cv::Mat::zeros(4,1,CV_32F);
    cv::RNG rng(19083);rng.fill(image,cv::RNG::UNIFORM,0,256);
    IMU::Calib calibration(Sophus::SE3f(),0.001,0.01,0.00001,0.0001);
    Frame frame(image,1.0,&extractor,vocabulary,camera,distortion,0,40,nullptr,calibration);
    frame.SetPose(Sophus::SE3f());frame.SetVelocity(Eigen::Vector3f::Zero());frame.mnDataset=0;
    frame.mpImuPreintegrated=nullptr;
    std::vector<KeyFrame*> fixed,chain;
    for(int i=0;i<12;++i) {
        frame.mTimeStamp=1.+i*0.25;
        auto kf=new KeyFrame(frame,map,database);map->AddKeyFrame(kf);fixed.push_back(kf);
    }
    const Eigen::Vector3f wrongVelocity(0.5f,-0.2f,0.1f);
    const IMU::Bias wrongBias(0.20f,-0.10f,0.05f,0.020f,-0.010f,0.005f);
    for(int i=0;i<7;++i) {
        frame.mTimeStamp=10.+i*0.25;
        frame.SetVelocity(wrongVelocity);frame.SetNewBias(wrongBias);
        frame.mpImuPreintegrated=i?new IMU::Preintegrated(wrongBias,calibration):nullptr;
        if(i) for(int j=0;j<125;++j)
            frame.mpImuPreintegrated->IntegrateNewMeasurement(Eigen::Vector3f(0,0,9.81f),Eigen::Vector3f::Zero(),0.002);
        auto kf=new KeyFrame(frame,map,database);map->AddKeyFrame(kf);
        if(i) {kf->mPrevKF=chain.back();chain.back()->mNextKF=kf;}
        chain.push_back(kf);
    }
    if(recovered) {
        Eigen::Matrix<double,9,9> info=Eigen::Matrix<double,9,9>::Identity()*25.;
        info.block<3,3>(3,3)=Eigen::Matrix3d::Identity()*2500.;
        chain.front()->SetRecoveryDynamicsPrior(wrongVelocity,wrongBias,info);
    }
    unsigned points=0;
    for(int i=0;i<frame.N;++i) {
        const auto uv=frame.mvKeysUn[i].pt;
        const auto ray=camera->unprojectEig(uv);
        if(ray.normalized().z()<0.3 || cv::norm(camera->project(cv::Point3f(ray.x(),ray.y(),ray.z()))-uv)>0.01)continue;
        auto point=new MapPoint(ray*(3.f+(i%9)*0.1f),fixed.front(),map);map->AddMapPoint(point);
        point->mTrackDepth=3.f;
        for(auto kf:fixed) {point->AddObservation(kf,i);kf->AddMapPoint(point,i);}
        for(auto kf:chain) {point->AddObservation(kf,i);kf->AddMapPoint(point,i);}
        point->ComputeDistinctiveDescriptors();point->UpdateNormalAndDepth();++points;
    }
    assert(points>300);
    map->mvpKeyFrameOrigins.push_back(fixed.front());
    for(auto kf:fixed) {kf->SetFirstConnection(false);if(kf!=fixed.front())kf->ChangeParent(fixed.front());kf->UpdateConnections();}
    for(auto kf:chain) {kf->SetFirstConnection(false);kf->ChangeParent(fixed.front());kf->UpdateConnections();}
    return {map,chain.front(),chain.back(),vocabulary,wrongVelocity,wrongBias};
}
static void RunBA(Fixture& f) {
    bool stop=false;int fixed=0,opt=0,points=0,edges=0;
    Optimizer::LocalInertialBA(f.latest,&stop,f.map,fixed,opt,points,edges,false,false);
}
static void PriorRoundTrip(Fixture& f) {
    Eigen::Vector3d v,bg,ba;Eigen::Matrix<double,9,9> info;
    assert(f.root->GetRecoveryDynamicsPrior(v,bg,ba,info));
    const auto originalV=v;const auto originalG=bg;const auto originalA=ba;const auto originalInfo=info;
    // Persistence prerequisites are established using the production PreSave.
    std::set<KeyFrame*> kfs;for(auto k:f.map->GetAllKeyFrames())kfs.insert(k);
    std::set<MapPoint*> mps;for(auto m:f.map->GetAllMapPoints())mps.insert(m);
    std::set<GeometricCamera*> cameras; cameras.insert(f.root->mpCamera);
    f.root->PreSave(kfs,mps,cameras);
    std::stringstream bytes;
    {boost::archive::binary_oarchive out(bytes);out<<f.root;}
    KeyFrame* restored=nullptr;
    {boost::archive::binary_iarchive in(bytes);in>>restored;}
    // Rebuild pose-derived members, as KeyFrame::PostLoad does.
    restored->SetPose(restored->GetPose());
    assert(restored->GetRecoveryDynamicsPrior(v,bg,ba,info));
    assert((v-originalV).norm()<1e-9 && (bg-originalG).norm()<1e-9 && (ba-originalA).norm()<1e-9);
    assert((info-originalInfo).norm()<1e-9);
    // A world rotation rotates only the world velocity part of the same prior.
    Eigen::Matrix3f R=Eigen::AngleAxisf(0.4f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    restored->SetPose(Sophus::SE3f(R.transpose(),Eigen::Vector3f::Zero()));
    assert(restored->GetRecoveryDynamicsPrior(v,bg,ba,info));
    assert((v-R.cast<double>()*originalV).norm()<1e-6 && (bg-originalG).norm()<1e-9);
    std::cout<<"RECOVERY_PRIOR_PERSISTENCE_AND_WORLD_ROTATION_PASS"<<std::endl;
}
int main(int argc,char** argv) {
    if(argc!=2)return 2;
    auto vocabulary=new ORBVocabulary();assert(vocabulary->loadFromTextFile(argv[1]));
    auto regular=MakeFixture(vocabulary,false);RunBA(regular);
    assert((regular.root->GetVelocity()-regular.initialVelocity).norm()==0);
    assert((BiasGyro(regular.root->GetImuBias())-BiasGyro(regular.initialBias)).norm()==0);
    assert((BiasAcc(regular.root->GetImuBias())-BiasAcc(regular.initialBias)).norm()==0);
    std::cout<<"ORDINARY_TEMPORAL_BOUNDARY_UNCHANGED_PASS"<<std::endl;
    auto recovered=MakeFixture(vocabulary,true);PriorRoundTrip(recovered);RunBA(recovered);
    const auto v=recovered.root->GetVelocity();const auto b=recovered.root->GetImuBias();
    std::cout<<"RECOVERY_ROOT_CORRECTION velocity="<<v.transpose()<<" gyro="<<BiasGyro(b).transpose()<<" acc="<<BiasAcc(b).transpose()<<std::endl;
    // Error is observable from identical visual poses and stationary exact IMU.
    // A hard-fixed old boundary cannot change at all; every dynamics group must
    // improve substantially under this same real LocalInertialBA implementation.
    assert(v.norm()<recovered.initialVelocity.norm()*0.5);
    assert(BiasGyro(b).norm()<BiasGyro(recovered.initialBias).norm()*0.5);
    assert(BiasAcc(b).norm()<BiasAcc(recovered.initialBias).norm()*0.5);
    // An invalid or nonpositive information matrix cannot enter serialization/BA.
    try {recovered.root->SetRecoveryDynamicsPrior(v,b,Eigen::Matrix<double,9,9>::Zero());assert(false);}
    catch(const std::runtime_error&) {}
    std::cout<<"RECOVERED_TEMPORAL_ROOT_SOFT_DYNAMICS_PASS"<<std::endl;
}
