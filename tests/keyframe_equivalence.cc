// Regression harness with original and specialized ORB-SLAM3 NeedNewKeyFrame bodies.
// Original portions copyright ORB-SLAM3 authors; GPLv3, see ../ORB_SLAM3/LICENSE.
#include <vector>
#include <random>
#include <iostream>
#include <cassert>
struct System { enum { MONOCULAR,STEREO,RGBD,IMU_MONOCULAR,IMU_STEREO,IMU_RGBD }; };
struct Frame { double mTimeStamp; unsigned mnId; int Nleft=-1,N=0; std::vector<float> mvDepth; std::vector<int> mvpMapPoints,mvbOutlier; };
struct KeyFrame { double mTimeStamp; unsigned mnId; int matches; int TrackedMapPoints(int min) { return matches - (min == 3 ? 5 : 0); } };
struct Map { bool initialized; bool isImuInitialized() { return initialized; } };
struct Atlas { Map map; int count; Map* GetCurrentMap() { return &map; } int KeyFramesInMap() { return count; } };
struct Mapper { bool stopped,stop,accept,initializing; int interrupts=0; bool isStopped(){return stopped;} bool stopRequested(){return stop;} bool AcceptKeyFrames(){return accept;} bool IsInitializing(){return initializing;} void InterruptBA(){++interrupts;} int KeyframesInQueue(){return 0;} };
struct Tracking {
    int mSensor=System::IMU_MONOCULAR; bool mbOnlyTracking; int mState; enum { RECENTLY_LOST=3 };
    Frame mCurrentFrame; KeyFrame* mpLastKeyFrame; KeyFrame* mpReferenceKF; Atlas* mpAtlas; Mapper* mpLocalMapper; void* mpCamera2=nullptr;
    unsigned mnLastRelocFrameId,mnLastKeyFrameId; int mMaxFrames,mMinFrames,mnMatchesInliers; float mThDepth=0;
    bool oldPolicy() {
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }

    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
        /*if(mSensor == System::MONOCULAR)
        {
            std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
        }*/
        return false;
    }

    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;

    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;

            }
        }
        //Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);// Verbose::VERBOSITY_DEBUG);
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
    const int thStereoClosedPoints = 15;
    if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO || mSensor==System::IMU_STEREO))
    {
        //Pseudo-monocular, there are not enough close points to be confident about the stereo observations.
        thRefRatio = 0.9f;
    }*/

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2) thRefRatio = 0.75f;

    if(mSensor==System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle); //mpLocalMapper->KeyframesInQueue() < 2);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && mSensor!=System::IMU_RGBD && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) && mnMatchesInliers>15);

    //std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2 << std::endl;
    // Temporal condition for Inertial cases
    bool c3 = false;
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && (mSensor == System::IMU_MONOCULAR)) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4=true;
    else
        c4=false;

    if(((c1a||c1b||c1c) && c2)||c3 ||c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle || mpLocalMapper->IsInitializing())
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
            {
                //std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
                return false;
            }
        }
    }
    else
        return false;
}
    bool newPolicy() {
    if(!mpAtlas->GetCurrentMap()->isImuInitialized())
        return (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25;

    if(mbOnlyTracking)
        return false;

    // Do not insert keyframes while local mapping is stopped for loop closure.
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpAtlas->KeyFramesInMap();
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    const int nMinObs = nKFs<=2 ? 2 : 3;
    const int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);
    const bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();
    const float thRefRatio = mnMatchesInliers>350 ? 0.75f : 0.90f;

    // Original IMU-monocular insertion policy: frame interval, match support and elapsed time.
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    const bool c1b = mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle;
    const bool c2 = mnMatchesInliers<nRefMatches*thRefRatio && mnMatchesInliers>15;
    const bool c3 = mpLastKeyFrame && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5;
    const bool c4 = (mnMatchesInliers<75 && mnMatchesInliers>15) || mState==RECENTLY_LOST;

    if(((c1a||c1b) && c2)||c3||c4)
    {
        if(bLocalMappingIdle || mpLocalMapper->IsInitializing())
            return true;
        mpLocalMapper->InterruptBA();
    }
    return false;
}
};
int main(){
    std::mt19937 rng(326533); Atlas a; Mapper mapper; KeyFrame last,ref; Tracking t;
    t.mpAtlas=&a;t.mpLocalMapper=&mapper;t.mpReferenceKF=&ref;
    for(int i=0;i<100000;++i){
        a.map.initialized = rng()%2; a.count=rng()%100;
        t.mpLastKeyFrame = !a.map.initialized || rng()%3 ? &last : nullptr;
        mapper.stopped=rng()%7==0;mapper.stop=rng()%7==0;mapper.accept=rng()%2;mapper.initializing=rng()%7==0;
        t.mbOnlyTracking=rng()%7==0;t.mState=rng()%6;t.mCurrentFrame.mnId=rng()%5000;
        t.mnLastRelocFrameId=rng()%5000;t.mnLastKeyFrameId=rng()%5000;t.mMaxFrames=rng()%100;t.mMinFrames=rng()%10;
        t.mnMatchesInliers=rng()%600;ref.matches=5+rng()%600;
        last.mTimeStamp=1.0; t.mCurrentFrame.mTimeStamp=1.0+(rng()%1001)/1000.0;
        mapper.interrupts=0;bool old=t.oldPolicy();int calls=mapper.interrupts;
        mapper.interrupts=0;bool now=t.newPolicy();
        assert(old==now);assert(calls==mapper.interrupts);
    }
    std::cout<<"100000 IMU-monocular keyframe policy cases: identical decisions and BA-interrupt effects\n";
}
