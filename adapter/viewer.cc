// External sensor-stream adapter for the unmodified ORB-SLAM3 public API.
#include <System.h>
#include <Map.h>
#include <MapPoint.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <fstream>
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

int track(ORB_SLAM3::System &slam) {
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
        slam.TrackMonocular(gray, camera_ns * ns_to_seconds, batch);
        const auto track_end = timing.is_open() ? host_raw_ns() : 0;
        if (timing.is_open()) {
            const int state = slam.GetTrackingState();
            const bool imu_ready = initialized_tracked_map(slam, state);
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
    if (slam && !slam->isShutDown())
        slam->Shutdown();

    // Upstream Shutdown requests background workers to stop and leaves the
    // Viewer thread running. End this process after flushing our own output
    // without racing the native threads against OpenGL static destructors.
    std::cout.flush();
    std::cerr.flush();
    std::_Exit(result);
}
