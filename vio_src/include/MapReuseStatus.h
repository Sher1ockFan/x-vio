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
    // Per-frame recovery search telemetry; cumulative resets belong to recovery_id.
    std::string recovery_method = "NONE";
    double recovery_search_ms = 0;
    bool recovery_global_attempted = false;
    unsigned long recovery_window_resets = 0;
    std::string reason;
    TrackingHealthDiagnostics health;
};
}
#endif
