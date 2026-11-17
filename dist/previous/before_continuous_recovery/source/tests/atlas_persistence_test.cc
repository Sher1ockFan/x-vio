// Native regression fixture for persistence/recovery, not a physical accuracy test.
// Builds real ORB Frame/KeyFrame/MapPoint objects with known metric geometry.
#include "System.h"
#include "AtlasPersistence.h"
#include "KannalaBrandt8.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <openssl/md5.h>
#include <cassert>
#include <chrono>
#include <thread>
#include <iostream>
#include <execinfo.h>
#include <signal.h>
using namespace ORB_SLAM3;

static IMU::Calib FixtureCalibration(const std::string& settings) {
    cv::FileStorage config(settings,cv::FileStorage::READ); cv::Mat matrix; config["IMU.T_b_c1"]>>matrix;
    Eigen::Matrix3f rotation; Eigen::Vector3f translation;
    for(int r=0;r<3;++r) { translation[r]=matrix.at<float>(r,3); for(int c=0;c<3;++c) rotation(r,c)=matrix.at<float>(r,c); }
    return IMU::Calib(Sophus::SE3f(rotation,translation),0.001,0.01,0.00001,0.0001);
}
static cv::Mat FixtureImage() {
    cv::Mat image(480,640,CV_8U);
    cv::RNG rng(452031);
    rng.fill(image,cv::RNG::UNIFORM,0,256);
    return image;
}
static std::string Md5(const std::string& path) {
    std::ifstream in(path,std::ios::binary);
    MD5_CTX state; MD5_Init(&state); char bytes[65536];
    while(in.read(bytes,sizeof(bytes)) || in.gcount()>0) MD5_Update(&state,bytes,in.gcount());
    unsigned char digest[16]; MD5_Final(digest,&state);
    std::ostringstream out; for(auto v:digest) out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(v);
    return out.str();
}
static void AssertAcyclic(Atlas* atlas) {
    for(auto map:atlas->GetAllMaps()) for(auto keyframe:map->GetAllKeyFrames()) {
        std::set<KeyFrame*> visited;
        for(KeyFrame* node=keyframe;node;node=node->GetParent()) {
            assert(visited.insert(node).second);
            assert(node->GetMap()==map);
        }
    }
}
static Atlas* Read(const std::string& path, ORBVocabulary* vocabulary) {
    std::ifstream in(path,std::ios::binary);
    char magic[17]; in.read(magic,sizeof(magic));
    assert(std::string(magic,sizeof(magic)) == "MONO_IMU_ATLAS_2\n");
    boost::archive::binary_iarchive archive(in);
    std::string name,hash,cal; Atlas* atlas=nullptr;
    archive>>name>>hash>>cal>>atlas;
    atlas->SetKeyFrameDababase(new KeyFrameDatabase(*vocabulary));
    atlas->SetORBVocabulary(vocabulary); atlas->PostLoad();
    AssertAcyclic(atlas);
    return atlas;
}
static void Fixture(const std::string& vocabularyPath,const std::string& settings,const std::string& path) {
    auto vocabulary=new ORBVocabulary(); assert(vocabulary->loadFromTextFile(vocabularyPath));
    auto database=new KeyFrameDatabase(*vocabulary);
    auto atlas=new Atlas(0); Map* map=atlas->GetCurrentMap();
    map->SetInertialSensor(); map->SetImuInitialized(); map->SetIniertialBA1(); map->SetIniertialBA2();
    cv::FileStorage fixtureSettings(settings,cv::FileStorage::READ);
    std::vector<float> params;
    for(const auto& key:{"Camera1.fx","Camera1.fy","Camera1.cx","Camera1.cy","Camera1.k1","Camera1.k2","Camera1.k3","Camera1.k4"})
        params.push_back(float(fixtureSettings[key]));
    auto camera=atlas->AddCamera(new KannalaBrandt8(params));
    ORBextractor extractor(1500,1.2,8,20,7);
    cv::Mat distortion=cv::Mat::zeros(4,1,CV_32F), image=FixtureImage();
    IMU::Calib calibration=FixtureCalibration(settings);
    const Eigen::Vector3f specificForce=calibration.mTbc.rotationMatrix()*Eigen::Vector3f(0,0,9.81);
    Frame frame(image,1.0,&extractor,vocabulary,camera,distortion,0,40,nullptr,calibration);
    frame.SetPose(Sophus::SE3f()); frame.SetVelocity(Eigen::Vector3f::Zero()); frame.mnDataset=0;
    assert(frame.N>500);
    std::vector<KeyFrame*> keyframes;
    for(int i=0;i<12;++i) {
        frame.mTimeStamp=1+i/30.;
        frame.mpImuPreintegrated=new IMU::Preintegrated(IMU::Bias(),calibration);
        frame.mpImuPreintegrated->IntegrateNewMeasurement(specificForce,Eigen::Vector3f::Zero(),1.f/30);
        auto kf=new KeyFrame(frame,map,database); kf->ComputeBoW(); kf->SetVelocity(Eigen::Vector3f::Zero());
        map->AddKeyFrame(kf); database->add(kf);
        if(!keyframes.empty()) { kf->mPrevKF=keyframes.back(); keyframes.back()->mNextKF=kf; }
        keyframes.push_back(kf);
    }
    map->mvpKeyFrameOrigins.push_back(keyframes.front());
    unsigned long pointCount=0;
    for(int i=0;i<frame.N;++i) {
        const cv::Point2f uv=frame.mvKeysUn[i].pt;
        const float depth=3.f+float(i%9)*0.1f;
        Eigen::Vector3f ray=camera->unprojectEig(uv);
        cv::Point2f projected=camera->project(cv::Point3f(ray.x(),ray.y(),ray.z()));
        // Upstream KB8 unprojection clamps extreme peripheral bearings; keep
        // physically consistent foreground rays for this exact-truth fixture.
        if(cv::norm(projected-uv)>0.01 || ray.normalized().z()<0.2f) continue;
        auto point=new MapPoint(ray*depth,keyframes.front(),map); ++pointCount;
        map->AddMapPoint(point);
        for(auto kf:keyframes) { point->AddObservation(kf,i); kf->AddMapPoint(point,i); }
        point->ComputeDistinctiveDescriptors(); point->UpdateNormalAndDepth();
    }
    // The fixture builds observations in a batch; unlike chronological SLAM,
    // selecting each KF's first parent now could choose a future KF and form
    // a cycle. Establish a valid rooted spanning tree explicitly first.
    for(auto kf:keyframes) {
        kf->SetFirstConnection(false);
        if(kf!=keyframes.front()) kf->ChangeParent(keyframes.front());
        kf->UpdateConnections();
    }
    AssertAcyclic(atlas);
    assert(Atlas::IsMatureMap(map));
    atlas->CreateNewMap(); // An empty active map must not enter the persisted view.
    atlas->PreSave(true);
    assert(atlas->GetAllMaps().size()==2); // Filter is non-destructive.
    const std::string checksum=Md5(vocabularyPath);
    cv::FileStorage config(settings,cv::FileStorage::READ);
    const std::string cal=AtlasPersistence::CalibrationFingerprint(config);
    AtlasPersistence::WriteAtomically(path,[&](std::ostream& out) {
        out.write("MONO_IMU_ATLAS_2\n",17);
        boost::archive::binary_oarchive archive(out);
        const std::string name="ORBvoc.txt";
        archive<<name<<checksum<<cal<<atlas;
    });
    Atlas* loaded=Read(path,vocabulary);
    assert(loaded->GetAllMaps().size()==1);
    Map* restored=loaded->GetAllMaps().front();
    assert(Atlas::IsMatureMap(restored)); assert(restored->KeyFramesInMap()==12);
    assert(restored->MapPointsInMap()==pointCount);
    auto restoredKF=restored->GetAllKeyFrames().front();
    assert(restoredKF->TrackedMapPoints(3)>=300);
    std::cout<<"ATLAS_FIXTURE_ROUNDTRIP_PASS points="<<pointCount<<std::endl;
}
static void CrashTrace(int signal) {
    const char message[]="NATIVE_ATLAS_TEST_SIGNAL_BACKTRACE\n";
    write(STDERR_FILENO,message,sizeof(message)-1);
    void* frames[80]; int size=backtrace(frames,80);
    backtrace_symbols_fd(frames,size,STDERR_FILENO);
    _exit(128+signal);
}
int main(int argc,char** argv) {
    signal(SIGSEGV,CrashTrace); signal(SIGABRT,CrashTrace);
    if(argc!=5) return 2;
    const std::string mode=argv[1],voc=argv[2],settings=argv[3],path=argv[4];
    try {
        if(mode=="fixture") { Fixture(voc,settings,path); return 0; }
        if(mode=="inspect") {
            auto vocabulary=new ORBVocabulary(); assert(vocabulary->loadFromTextFile(voc));
            Atlas* atlas=Read(path,vocabulary); int temporalRoots=0;
            for(auto map:atlas->GetAllMaps()) for(auto keyframe:map->GetAllKeyFrames()) {
                assert(keyframe->mpCamera != nullptr && keyframe->mpCamera2 == nullptr);
                if(!keyframe->mPrevKF) { ++temporalRoots; assert(keyframe->mpImuPreintegrated==nullptr); }
                else if(keyframe->mpImuPreintegrated) assert(keyframe->mpImuPreintegrated->dT>0);
            }
            assert(temporalRoots>=2);
            std::cout<<"FRESH_TEMPORAL_ROOTS_PRESERVED "<<temporalRoots<<std::endl;return 0;
        }
        if(mode=="atomic") {
            AtlasPersistence::WriteAtomically(path,[](std::ostream& out){out<<"known-good";});
            try { AtlasPersistence::WriteAtomically(path,[](std::ostream& out){out<<"partial";throw std::runtime_error("simulated interrupted serialization");}); }
            catch(const std::runtime_error&) {}
            std::ifstream in(path); std::string bytes; in>>bytes; assert(bytes=="known-good");
            std::cout<<"ATLAS_ATOMIC_FAILURE_PRESERVES_OLD_PASS"<<std::endl; return 0;
        }
        System system(voc,settings,System::IMU_MONOCULAR,false);
        if(mode=="expect_error" || mode=="expect_save_error") { system.Shutdown(); return 10; }
        if(mode=="requested") {
            system.RequestShutdown(); assert(system.isShutDown());
            system.TrackMonocular(FixtureImage(),10.,{});
            assert(system.GetTrackingState()!=2 && !system.GetMapReuseStatus().inertial_ready);
        }
        if(mode=="timeout") {
            bool searching=false,fallback=false,retry=false,restored=false;
            long fallbackMap=-1; unsigned long firstRecovery=0;
            double nextImuTime=9.90;
            const Eigen::Vector3f specificForce=FixtureCalibration(settings).mTbc.rotationMatrix()*Eigen::Vector3f(0,0,9.81);
            for(int frameIndex=0;frameIndex<500;++frameIndex) {
                const double time=10.+frameIndex/30.;
                const bool blank=frameIndex<140;
                std::vector<IMU::Point> imu;
                while(nextImuTime<=time+1e-9) {
                    imu.emplace_back(specificForce.x(),specificForce.y(),specificForce.z(),0,0,0,nextImuTime);
                    nextImuTime+=0.002;
                }
                system.TrackMonocular(blank ? cv::Mat::zeros(480,640,CV_8U) : FixtureImage(),time,imu);
                auto status=system.GetMapReuseStatus();
                if(blank) assert(!status.visual_ready && !status.inertial_ready);
                if(status.phase=="SEARCHING") {
                    searching=true;
                    if(!firstRecovery) firstRecovery=status.recovery_id;
                    if(fallback && status.recovery_id>firstRecovery) retry=true;
                }
                if(fallback && status.recovery_id>firstRecovery && status.phase=="ALIGNING_IMU") retry=true;
                if(status.phase=="MAPPING" && status.map_id>=0 && status.map_id!=0) {
                    fallback=true; fallbackMap=status.map_id;
                }
                if(status.inertial_ready) {
                    assert(!blank && fallback && retry && status.map_id==0);
                    restored=true;
                }
                if(frameIndex%30==0) std::cout<<"TIMEOUT_FRAME "<<frameIndex<<" "<<status.phase<<" map="<<status.map_id<<" "<<status.reason<<std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            system.Shutdown();
            std::cout<<"TIMEOUT_RETRY_RESULT search="<<searching<<" fallback="<<fallback<<" fallback_map="<<fallbackMap
                     <<" retry="<<retry<<" restored_old="<<restored<<std::endl;
            assert(searching && fallback && retry && restored);
        }
        else if(mode=="mapjump") {
            bool readyBefore=false,rejected=false,recovered=false;unsigned long oldRecovery=0;
            double nextImuTime=9.90,maxOutputStep=0;
            const Eigen::Vector3f specificForce=FixtureCalibration(settings).mTbc.rotationMatrix()*Eigen::Vector3f(0,0,9.81);
            for(int frameIndex=0;frameIndex<300;++frameIndex) {
                const double time=10.+frameIndex/30.;
                if(frameIndex==110) {
                    // Whole-map rigid correction keeps reprojections unchanged.
                    // Only comparison with immutable prior output sees the jump.
                    Map* map=nullptr;
                    for(auto point:system.GetTrackedMapPoints())if(point && !point->isBad()){map=point->GetMap();break;}
                    assert(readyBefore && map);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    std::unique_lock<std::mutex> lock(map->mMutexMapUpdate);
                    map->ApplyScaledRotation(Sophus::SE3f(Eigen::Matrix3f::Identity(),Eigen::Vector3f(9,0,0)),1.f,true);
                }
                std::vector<IMU::Point> imu;
                while(nextImuTime<=time+1e-9) {
                    imu.emplace_back(specificForce.x(),specificForce.y(),specificForce.z(),0,0,0,nextImuTime);
                    nextImuTime+=0.002;
                }
                system.TrackMonocular(FixtureImage(),time,imu);
                const auto status=system.GetMapReuseStatus();
                if(frameIndex<110 && status.inertial_ready){readyBefore=true;oldRecovery=status.recovery_id;}
                if(status.health.output_pose_baseline && std::isfinite(status.health.output_pose_step_m))
                    maxOutputStep=std::max(maxOutputStep,status.health.output_pose_step_m);
                if(frameIndex>=110 && status.health.hard_fault && status.health.output_pose_step_m>3.0) {
                    assert(!status.inertial_ready && !status.health.accepted);
                    rejected=true;
                    std::cout<<"OUTPUT_DISCONTINUITY_REJECTED internal="<<status.health.pose_step_m
                        <<" output="<<status.health.output_pose_step_m<<" reason="<<status.health.reason<<std::endl;
                }
                if(frameIndex>110 && status.inertial_ready && status.recovery_id>oldRecovery)recovered=true;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            system.Shutdown();
            assert(readyBefore && rejected && recovered && maxOutputStep>8.0);
            std::cout<<"WHOLE_MAP_OUTPUT_DISCONTINUITY_GUARD_PASS"<<std::endl;
        }
        else if(mode=="stationary" || mode=="biasstep") {
            // Exact static scene for longer than the user's failing 28 s tail.
            // Biasstep is a finite sensor inconsistency; valid poses must not
            // drift while the image supplies an independent fixed reference.
            bool sawReady=false, sawStationary=false, recoveredAfterStep=false;
            int validFrames=0, hardFaults=0; double maxTranslation=0;
            std::ofstream healthLog(path+".csv");
            healthLog << "frame,t,phase,valid,hard,reason,map_updated,x,y,z,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz,visual_rms,fused_rms,visual_delta,step,static,duration,static_displacement,flow,imu_std,inliers\n";
            healthLog << std::setprecision(16);
            double nextImuTime=9.90;
            const Eigen::Vector3f specificForce=FixtureCalibration(settings).mTbc.rotationMatrix()*Eigen::Vector3f(0,0,9.81);
            for(int frameIndex=0;frameIndex<1400;++frameIndex) {
                const double time=10.+frameIndex/30.;
                std::vector<IMU::Point> imu;
                while(nextImuTime<=time+1e-9) {
                    Eigen::Vector3f a=specificForce;
                    if(mode=="biasstep" && nextImuTime>=30. && nextImuTime<40.) a.x()+=1.0f;
                    imu.emplace_back(a.x(),a.y(),a.z(),0,0,0,nextImuTime);
                    nextImuTime+=0.002;
                }
                const auto pose=system.TrackMonocular(FixtureImage(),time,imu);
                const auto status=system.GetMapReuseStatus();
                const auto h=status.health;const auto center=pose.inverse().translation();
                healthLog<<frameIndex<<','<<time<<','<<status.phase<<','<<status.inertial_ready<<','<<h.hard_fault
                    <<','<<h.reason<<','<<h.map_updated<<','<<center.x()<<','<<center.y()<<','<<center.z();
                for(double value:h.velocity_w)healthLog<<','<<value;
                for(double value:h.accel_bias)healthLog<<','<<value;
                for(double value:h.gyro_bias)healthLog<<','<<value;
                healthLog<<','<<h.visual_rms_px<<','<<h.fused_rms_px<<','<<h.visual_translation_error_m
                    <<','<<h.pose_step_m<<','<<h.stationary_candidate<<','<<h.stationary_duration_s
                    <<','<<h.stationary_displacement_m<<','<<h.flow_median_px_s<<','<<h.imu_acc_std
                    <<','<<h.visual_inliers<<std::endl;
                sawStationary |= status.health.stationary_candidate;
                hardFaults += status.health.hard_fault;
                if(status.inertial_ready) {
                    sawReady=true; ++validFrames;
                    assert(status.visual_ready && status.health.evaluated && status.health.accepted);
                    assert(pose.matrix().allFinite());
                    maxTranslation=std::max(maxTranslation,double(pose.translation().norm()));
                    if(frameIndex>1200)recoveredAfterStep=true;
                }
                if(frameIndex%100==0) std::cout<<"HEALTH_FRAME "<<mode<<" "<<frameIndex<<" "<<status.phase
                    <<" accepted="<<status.health.accepted<<" hard_fault="<<status.health.hard_fault
                    <<" reason="<<status.health.reason<<std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            system.Shutdown();
            std::cout<<"LONG_STATIC_HEALTH_RESULT mode="<<mode<<" ready="<<sawReady<<" stationary="<<sawStationary
                <<" valid="<<validFrames<<" hard_faults="<<hardFaults<<" max_translation="<<maxTranslation
                <<" tail_ready="<<recoveredAfterStep<<std::endl;
            assert(sawReady && sawStationary && validFrames>500 && maxTranslation<0.08 && recoveredAfterStep);
            if(mode=="stationary")assert(hardFaults==0);
        }
        else if(mode=="recover") {
            bool sawSearch=false,sawAlign=false,sawReady=false,occludedNotReady=false,recoveredAgain=false;
            int readyFrames=0; double maxTranslation=0; long referenceMap=-1;
            unsigned long initialRecovery=0;
            double nextImuTime=9.90;
            const Eigen::Vector3f specificForce=FixtureCalibration(settings).mTbc.rotationMatrix()*Eigen::Vector3f(0,0,9.81);
            for(int frameIndex=0;frameIndex<250;++frameIndex) {
                double time=10.+frameIndex/30.;
                std::vector<IMU::Point> imu;
                while(nextImuTime<=time+1e-9) {
                    imu.emplace_back(specificForce.x(),specificForce.y(),specificForce.z(),0,0,0,nextImuTime);
                    nextImuTime+=0.002;
                }
                const bool occluded=frameIndex>=100 && frameIndex<125;
                Sophus::SE3f pose=system.TrackMonocular(occluded ? cv::Mat::zeros(480,640,CV_8U) : FixtureImage(),time,imu);
                auto status=system.GetMapReuseStatus();
                sawSearch|=status.phase=="SEARCHING";
                sawAlign|=status.phase=="ALIGNING_IMU";
                if(status.inertial_ready) {
                    sawReady=true; ++readyFrames;
                    maxTranslation=std::max(maxTranslation,double(pose.translation().norm()));
                    assert(status.visual_ready);
                    if(frameIndex<100) { referenceMap=status.map_id; initialRecovery=status.recovery_id; }
                    if(frameIndex>=125 && status.recovery_id>initialRecovery) {
                        assert(status.map_id==referenceMap); recoveredAgain=true;
                    }
                }
                if(occluded && !status.inertial_ready && status.phase=="SEARCHING") occludedNotReady=true;
                if(frameIndex%10==0) std::cout<<"RECOVERY_FRAME "<<frameIndex<<" "<<status.phase<<" "<<status.reason<<std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            std::cout<<"RECOVERY_RESULT search="<<sawSearch<<" align="<<sawAlign<<" ready="<<sawReady<<" frames="<<readyFrames<<" max_translation="<<maxTranslation
                     <<" occluded_not_ready="<<occludedNotReady<<" recovered_again="<<recoveredAgain<<std::endl;
            system.Shutdown();
            assert(sawSearch && sawAlign && sawReady && readyFrames>=10 && maxTranslation<0.03 && occludedNotReady && recoveredAgain);
        } else system.Shutdown();
        std::cout<<"SYSTEM_PERSISTENCE_PASS "<<mode<<std::endl; return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<std::endl;
        if(mode=="expect_save_error" && std::string(error.what()).find("MAP_E_SAVE")!=std::string::npos) {
            std::cout<<"ATLAS_SAVE_FAILURE_PROPAGATES_PASS"<<std::endl;return 0;
        }
        if(mode=="expect_error" && std::string(error.what()).find("MAP_E_LOAD")!=std::string::npos) {
            std::cout<<"ATLAS_LOAD_GUARD_PASS"<<std::endl;return 0;
        }
        return 1;
    }
}
