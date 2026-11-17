// External sensor-stream adapter for the mono_imu_vio single-camera inertial API.
#include <System.h>
#include <Map.h>
#include <MapPoint.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int image_width = 1920;
constexpr int image_height = 1440;
constexpr double ns_to_seconds = 1e-9;

std::int64_t host_raw_ns() {
    timespec t{};
    if (clock_gettime(CLOCK_MONOTONIC_RAW, &t))
        throw std::runtime_error("Cannot read host RAW clock for timing");
    return static_cast<std::int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

// Observe the map associated with this completed tracking frame via public APIs.
// Do not call GetTimeFromIMUInit(): in this pinned upstream version its local
// mapper fields may not yet be initialized before the first keyframe is handled.
bool initialized_tracked_map(ORB_SLAM3::System &slam, int state) {
    if (state != 2) return false;
    ORB_SLAM3::Map *tracked_map = nullptr;
    for (auto *point : slam.GetTrackedMapPoints()) {
        if (!point || point->isBad()) continue;
        auto *map = point->GetMap();
        if (!map) continue;
        if (tracked_map && map != tracked_map) return false;
        tracked_map = map;
    }
    return tracked_map && tracked_map->isImuInitialized();
}

std::string csv_string(const std::string &value) {
    std::string result = "\"";
    for (const char c : value) {
        if (c == '"') result += '"';
        result += (c == '\r' || c == '\n') ? ' ' : c;
    }
    return result + "\"";
}

int track(ORB_SLAM3::System &slam) {
    std::ofstream poses("poses.csv"), events("recovery_events.csv"), diagnostics("state_diagnostics.csv");
    if (!poses || !events || !diagnostics) throw std::runtime_error("Cannot open pose/recovery diagnostics");
    poses << "frame_idx,camera_ns,map_id,tracking_state,visual_ready,inertial_ready,valid,"
             "recovery_id,phase,tx,ty,tz,qx,qy,qz,qw" << std::endl;
    events << "frame_idx,camera_ns,recovery_id,phase,map_id,reference_keyframe_id,"
              "inliers,visual_ready,inertial_ready,reason" << std::endl;
    diagnostics << "frame_idx,camera_ns,map_id,recovery_id,phase,tracking_state,map_updated,valid,"
                   "health_evaluated,health_accepted,hard_fault,health_reason,dynamics_available,"
                   "velocity_w_x_mps,velocity_w_y_mps,velocity_w_z_mps,"
                   "gyro_bias_x_radps,gyro_bias_y_radps,gyro_bias_z_radps,"
                   "accel_bias_x_mps2,accel_bias_y_mps2,accel_bias_z_mps2,"
                   "tracking_inliers,visual_inliers,visual_cells,visual_rms_px,fused_rms_px,"
                   "visual_translation_error_m,visual_rotation_error_rad,pose_step_m,pose_step_dt_s,"
                   "output_pose_baseline,output_pose_step_m,output_pose_step_dt_s,"
                   "flow_matches,flow_median_px_s,flow_p90_px_s,imu_window_samples,"
                   "imu_gyro_mean_norm_radps,imu_gyro_std_radps,imu_acc_mean_norm_mps2,imu_acc_std_mps2,"
                   "stationary_candidate,stationary_evidence_held,stationary_duration_s,stationary_displacement_m,"
                   "recovery_fit_velocity_w_x_mps,recovery_fit_velocity_w_y_mps,recovery_fit_velocity_w_z_mps,"
                   "recovery_fit_gyro_bias_x_radps,recovery_fit_gyro_bias_y_radps,recovery_fit_gyro_bias_z_radps,"
                   "recovery_fit_accel_bias_x_mps2,recovery_fit_accel_bias_y_mps2,recovery_fit_accel_bias_z_mps2,"
                   "recovery_fit_position_rms_m,recovery_fit_rotation_rms_rad,"
                   "recovery_holdout_position_rms_m,recovery_holdout_rotation_rms_rad,"
                   "recovery_fit_span_s,recovery_holdout_span_s,recovery_fit_condition_number" << std::endl;
    poses << std::setprecision(12);
    diagnostics << std::setprecision(12);
    std::string previous_event;
    std::ofstream timing;
    const char *timing_env = std::getenv("VIO_TIMING");
    if (timing_env && std::string(timing_env) == "1") {
        timing.open("tracking_timing.csv");
        if (!timing) throw std::runtime_error("Cannot open tracking_timing.csv");
        timing << "frame_idx,camera_ns,imu_count,imu_first_ns,imu_last_ns,packet_read_end_raw_ns,"
                  "track_start_raw_ns,track_end_raw_ns,tracking_state,imu_initialized_proxy" << std::endl;
    }
    cv::Mat gray(image_height, image_width, CV_8UC1);
    std::vector<ORB_SLAM3::IMU::Point> batch;
    std::int64_t previous_camera_ns = 0;
    std::int64_t previous_imu_ns = 0;
    std::size_t frames = 0;
    std::string line;
    std::cout << "ORB_LIVE_READY" << std::endl;

    while (!slam.isShutDown() && std::getline(std::cin, line)) {
        std::int64_t camera_ns;
        unsigned imu_count;
        std::string extra;
        std::istringstream header(line);
        if (!(header >> camera_ns >> imu_count) || (header >> extra) ||
            camera_ns <= previous_camera_ns || imu_count > 1000 || (frames && imu_count == 0))
            throw std::runtime_error("Invalid camera timestamp or IMU count at frame " + std::to_string(frames));

        std::int64_t imu_first_ns = 0, imu_last_ns = 0;
        batch.clear();
        batch.reserve(imu_count);
        for (unsigned i = 0; i < imu_count; ++i) {
            if (!std::getline(std::cin, line))
                throw std::runtime_error("Truncated IMU batch at frame " + std::to_string(frames));
            std::int64_t ns;
            float ax, ay, az, gx, gy, gz;
            std::istringstream row(line);
            if (!(row >> ns >> ax >> ay >> az >> gx >> gy >> gz) || (row >> extra) ||
                ns <= previous_imu_ns || ns > camera_ns ||
                !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
                !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz))
                throw std::runtime_error("Invalid IMU timestamp or SI values at frame " + std::to_string(frames));
            if (!i) imu_first_ns = ns;
            imu_last_ns = ns;
            batch.emplace_back(ax, ay, az, gx, gy, gz, ns * ns_to_seconds);
            previous_imu_ns = ns;
        }
        if (!std::cin.read(reinterpret_cast<char *>(gray.data), image_width * image_height))
            throw std::runtime_error("Truncated gray image at frame " + std::to_string(frames));
        const auto packet_read_end = timing.is_open() ? host_raw_ns() : 0;
        if (slam.isShutDown())
            break;

        const auto track_start = timing.is_open() ? host_raw_ns() : 0;
        const Sophus::SE3f T_cw = slam.TrackMonocular(gray, camera_ns * ns_to_seconds, batch);
        const auto track_end = timing.is_open() ? host_raw_ns() : 0;
        const int state = slam.GetTrackingState();
        const auto reuse = slam.GetMapReuseStatus();
        // An old map's isImuInitialized flag cannot certify a new segment's
        // velocity/bias. Recovery remains invalid until the fresh IMU solve.
        const bool imu_ready = state == 2 && (reuse.enabled ? reuse.inertial_ready :
                                             initialized_tracked_map(slam, state));
        const bool visual_ready = reuse.enabled ? reuse.visual_ready : state == 2;
        const auto &health = reuse.health;
        const bool health_ok = !reuse.enabled || (health.evaluated && health.accepted && !health.hard_fault);
        const bool valid = imu_ready && health_ok && T_cw.matrix().allFinite();
        // Log every frame, including rejected candidates and periods when no
        // state exists. Unknown quantities remain NaN; they are not zero motion.
        diagnostics << frames << ',' << camera_ns << ',' << reuse.map_id << ',' << reuse.recovery_id << ','
                    << csv_string(reuse.phase) << ',' << state << ',' << health.map_updated << ',' << valid << ','
                    << health.evaluated << ',' << health.accepted << ',' << health.hard_fault << ','
                    << csv_string(health.reason) << ',' << health.dynamics_available;
        for (const auto value : health.velocity_w) diagnostics << ',' << value;
        for (const auto value : health.gyro_bias) diagnostics << ',' << value;
        for (const auto value : health.accel_bias) diagnostics << ',' << value;
        diagnostics << ',' << reuse.inliers << ',' << health.visual_inliers << ',' << health.visual_cells
                    << ',' << health.visual_rms_px << ',' << health.fused_rms_px
                    << ',' << health.visual_translation_error_m << ',' << health.visual_rotation_error_rad
                    << ',' << health.pose_step_m << ',' << health.pose_step_dt_s
                    << ',' << health.output_pose_baseline << ',' << health.output_pose_step_m
                    << ',' << health.output_pose_step_dt_s
                    << ',' << health.flow_matches << ',' << health.flow_median_px_s << ',' << health.flow_p90_px_s
                    << ',' << health.imu_window_samples << ',' << health.imu_gyro_mean_norm << ',' << health.imu_gyro_std
                    << ',' << health.imu_acc_mean_norm << ',' << health.imu_acc_std
                    << ',' << health.stationary_candidate << ',' << health.stationary_evidence_held
                    << ',' << health.stationary_duration_s
                    << ',' << health.stationary_displacement_m;
        for (const auto value : health.recovery_fit_velocity_w) diagnostics << ',' << value;
        for (const auto value : health.recovery_fit_gyro_bias) diagnostics << ',' << value;
        for (const auto value : health.recovery_fit_accel_bias) diagnostics << ',' << value;
        diagnostics << ',' << health.recovery_fit_position_rms_m << ',' << health.recovery_fit_rotation_rms_rad
                    << ',' << health.recovery_holdout_position_rms_m << ',' << health.recovery_holdout_rotation_rms_rad
                    << ',' << health.recovery_fit_span_s << ',' << health.recovery_holdout_span_s
                    << ',' << health.recovery_fit_condition_number << std::endl;
        poses << frames << ',' << camera_ns << ',' << reuse.map_id << ',' << state << ','
              << visual_ready << ',' << imu_ready << ',' << valid << ',' << reuse.recovery_id << ','
              << csv_string(reuse.phase) << ',';
        if (valid) {
            const Sophus::SE3f T_wc = T_cw.inverse();
            const auto p = T_wc.translation();
            const auto q = T_wc.unit_quaternion();
            poses << p.x() << ',' << p.y() << ',' << p.z() << ','
                  << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
        } else {
            poses << "nan,nan,nan,nan,nan,nan,nan";
        }
        poses << std::endl;
        std::ostringstream event_key;
        event_key << reuse.recovery_id << ',' << reuse.phase << ',' << reuse.map_id << ','
                  << visual_ready << ',' << imu_ready << ',' << reuse.reason;
        if (event_key.str() != previous_event) {
            events << frames << ',' << camera_ns << ',' << reuse.recovery_id << ','
                   << csv_string(reuse.phase) << ',' << reuse.map_id << ','
                   << reuse.reference_keyframe_id << ',' << reuse.inliers << ','
                   << visual_ready << ',' << imu_ready << ',' << csv_string(reuse.reason) << std::endl;
            previous_event = event_key.str();
        }
        if (!poses || !events || !diagnostics) throw std::runtime_error("Cannot write pose/recovery diagnostics");
        if (timing.is_open()) {
            timing << frames << ',' << camera_ns << ',' << imu_count << ','
                   << imu_first_ns << ',' << imu_last_ns << ',' << packet_read_end << ','
                   << track_start << ',' << track_end << ',' << state << ','
                   << (imu_ready ? 1 : 0) << std::endl;
            if (!timing) throw std::runtime_error("Cannot write tracking_timing.csv");
        }
        previous_camera_ns = camera_ns;
        ++frames;
        if (frames == 1 || frames % 30 == 0)
            std::cout << "STOCK frames=" << frames << " tracking_state=" << slam.GetTrackingState() << std::endl;
    }

    if (slam.isShutDown()) {
        std::cout << "Native Viewer stopped after " << frames << " frames." << std::endl;
        return 3;
    }
    if (std::cin.bad() || !std::cin.eof())
        throw std::runtime_error("Input stream read error");
    if (!frames)
        throw std::runtime_error("No complete sensor frames");
    std::cout << "STOCK_STREAM_COMPLETE frames=" << frames << std::endl;
    return 0;
}
} // namespace

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "Usage: stock_viewer VOCABULARY SETTINGS < sensor_stream\n";
        return 2;
    }
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    ORB_SLAM3::System *slam = nullptr;
    int result = 1;
    try {
        slam = new ORB_SLAM3::System(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
        result = track(*slam);
    } catch (const std::exception &error) {
        std::cerr << "STOCK_INPUT_ERROR " << error.what() << std::endl;
    }
    // The Viewer may have requested shutdown without performing serialization.
    // The caller thread always completes the worker joins and map save.
    if (slam) {
        try {
            slam->Shutdown();
        } catch (const std::exception &error) {
            std::cerr << "STOCK_SHUTDOWN_ERROR " << error.what() << std::endl;
            result = 1;
        }
    }

    // Shutdown has joined the native workers and finished map serialization.
    // Avoid invoking process-wide OpenGL static destructors after GUI teardown.
    std::cout.flush();
    std::cerr.flush();
    std::_Exit(result);
}
