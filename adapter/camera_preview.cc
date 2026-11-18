// Display the latest image published by the sensor recorder.
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <utility>

namespace {

constexpr char window_name[] = "Camera preview";

class RecordingFile {
public:
    explicit RecordingFile(std::filesystem::path path) : path_(std::move(path)) {}

    ~RecordingFile() {
        if (fd_ >= 0)
            close(fd_);
    }

    bool available() {
        if (fd_ >= 0)
            return true;

        fd_ = open(path_.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
        if (fd_ < 0 && errno != ENOENT)
            throw std::system_error(errno, std::generic_category(), "open " + path_.string());
        return fd_ >= 0;
    }

    off_t size() const {
        struct stat status {};
        if (fstat(fd_, &status) != 0)
            throw std::system_error(errno, std::generic_category(), "stat " + path_.string());
        return status.st_size;
    }

    std::size_t read(void* buffer, std::size_t bytes, off_t offset) const {
        ssize_t received;
        do {
            received = pread(fd_, buffer, bytes, offset);
        } while (received < 0 && errno == EINTR);
        if (received < 0)
            throw std::system_error(errno, std::generic_category(), "read " + path_.string());
        return static_cast<std::size_t>(received);
    }

    RecordingFile(const RecordingFile&) = delete;
    RecordingFile& operator=(const RecordingFile&) = delete;

private:
    std::filesystem::path path_;
    int fd_ = -1;
};

struct IndexedFrame {
    std::uint64_t index;
    off_t image_offset;
};

std::optional<IndexedFrame> latest_frame(const RecordingFile& index, std::size_t pixels) {
    // An index row is much shorter than this bounded tail even with 64-bit
    // offsets. Discard the unfinished suffix and any partial row at the start.
    std::array<char, 1024> tail;
    const off_t size = index.size();
    const off_t start = std::max<off_t>(0, size - static_cast<off_t>(tail.size()));
    const auto bytes = index.read(tail.data(), static_cast<std::size_t>(size - start), start);
    const std::string_view text(tail.data(), bytes);
    const auto end = text.rfind('\n');
    if (end == std::string_view::npos || end == 0)
        return std::nullopt;
    const auto preceding = text.rfind('\n', end - 1);
    if (preceding == std::string_view::npos && start != 0)
        return std::nullopt;
    const auto beginning = preceding == std::string_view::npos ? 0 : preceding + 1;
    auto row = text.substr(beginning, end - beginning);
    if (!row.empty() && row.back() == '\r')
        row.remove_suffix(1);
    if (row.substr(0, 10) == "frame_idx,")
        return std::nullopt;

    // frame_idx,sequence,camera_raw_ns,camera_ns,imu_count,offset,size
    std::array<std::uint64_t, 7> fields;
    for (std::size_t i = 0; i < fields.size(); ++i) {
        const auto comma = row.find(',');
        if ((i + 1 == fields.size()) != (comma == std::string_view::npos))
            throw std::runtime_error("Invalid camera recording index row.");
        const auto field = row.substr(0, comma);
        const char* last = field.data() + field.size();
        const auto result = std::from_chars(field.data(), last, fields[i]);
        if (result.ec != std::errc{} || result.ptr != last)
            throw std::runtime_error("Invalid number in camera recording index.");
        if (comma != std::string_view::npos)
            row.remove_prefix(comma + 1);
    }
    const auto imu_count = fields[4];
    const auto packet_offset = fields[5];
    const auto packet_bytes = fields[6];
    const auto max_offset = static_cast<std::uint64_t>(std::numeric_limits<off_t>::max());
    if (imu_count < 1 || imu_count > 1000 ||
        packet_bytes <= pixels || packet_bytes - pixels > 256 * (imu_count + 1) ||
        packet_bytes > max_offset || packet_offset > max_offset - packet_bytes)
        throw std::runtime_error("Invalid camera recording packet bounds.");
    return IndexedFrame{fields[0], static_cast<off_t>(packet_offset + packet_bytes - pixels)};
}

int dimension(std::string_view input) {
    int value = 0;
    const char* end = input.data() + input.size();
    const auto result = std::from_chars(input.data(), end, value);
    if (result.ec != std::errc{} || result.ptr != end || value < 1 || value > 8192)
        throw std::invalid_argument("Invalid camera preview dimensions.");
    return value;
}

bool window_open() {
    try {
        // OpenCV's packaged GTK backend has no WND_PROP_VISIBLE.
        const auto bounds = cv::getWindowImageRect(window_name);
        return bounds.width > 0 && bounds.height > 0;
    } catch (const cv::Exception&) {
        return false; // Some HighGUI backends report a destroyed window by throwing.
    }
}

void show(const std::filesystem::path& take, int width, int height) {
    RecordingFile index(take / "frames.csv");
    RecordingFile stream(take / "stream.bin");
    cv::Mat image(height, width, CV_8UC1, cv::Scalar(0));
    std::optional<std::uint64_t> displayed;

    cv::namedWindow(window_name, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::imshow(window_name, image);
    const double scale = std::min({1.0, 960.0 / width, 768.0 / height});
    const int window_width = std::max(1L, std::lround(width * scale));
    const int window_height = std::max(1L, std::lround(height * scale));
    cv::resizeWindow(window_name, window_width, window_height);

    for (;;) {
        const int key = cv::waitKey(8);
        if (key == 27 || key == 'q' || key == 'Q' || !window_open())
            break;
        if (!index.available() || !stream.available())
            continue;
        const auto frame = latest_frame(index, image.total());
        if (frame && (!displayed || frame->index > *displayed) &&
            stream.read(image.data, image.total(), frame->image_offset) == image.total()) {
            cv::imshow(window_name, image);
            displayed = frame->index;
        }
    }
    cv::destroyAllWindows();
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: camera_preview TAKE WIDTH HEIGHT\n";
        return EXIT_FAILURE;
    }
    try {
        show(argv[1], dimension(argv[2]), dimension(argv[3]));
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Camera preview: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
