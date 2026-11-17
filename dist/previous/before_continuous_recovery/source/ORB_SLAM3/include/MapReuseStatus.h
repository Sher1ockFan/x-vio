#ifndef ORB_SLAM3_MAP_REUSE_STATUS_H
#define ORB_SLAM3_MAP_REUSE_STATUS_H
#include <string>
#include "TrackingHealth.h"
namespace ORB_SLAM3 {
// Snapshot of current-frame validity, separate from a map's historical IMU flag.
struct MapReuseStatus {
    bool enabled = false;
    std::string phase = "MAPPING";
    long map_id = -1;
    long reference_keyframe_id = -1;
    int inliers = 0;
    unsigned long recovery_id = 0;
    bool visual_ready = false;
    bool inertial_ready = false;
    std::string reason;
    TrackingHealthDiagnostics health;
};
}
#endif
