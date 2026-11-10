// External sensor-stream adapter for the unmodified ORB-SLAM3 public API.
#include <System.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int image_width = 1920;
constexpr int image_height = 1440;
constexpr double ns_to_seconds = 1e-9;

int track(ORB_SLAM3::System &slam) {
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
            batch.emplace_back(ax, ay, az, gx, gy, gz, ns * ns_to_seconds);
            previous_imu_ns = ns;
        }
        if (!std::cin.read(reinterpret_cast<char *>(gray.data), image_width * image_height))
            throw std::runtime_error("Truncated gray image at frame " + std::to_string(frames));
        if (slam.isShutDown())
            break;

        slam.TrackMonocular(gray, camera_ns * ns_to_seconds, batch);
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
