// mono_imu_vio persistence helpers. GPLv3, same license as ORB-SLAM3.
#ifndef MONO_IMU_ATLAS_PERSISTENCE_H
#define MONO_IMU_ATLAS_PERSISTENCE_H

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <openssl/sha.h>

namespace ORB_SLAM3 { namespace AtlasPersistence {

inline std::string Parent(const std::string& path) {
    const std::size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? "." : (slash == 0 ? "/" : path.substr(0, slash));
}

inline void ValidateExactPath(const std::string& path) {
    if (path.empty() || path[0] != '/' || path.size() < 5 ||
        path.substr(path.size()-4) != ".osa" || path.find('\0') != std::string::npos)
        throw std::runtime_error("MAP_E_PATH: MapReuse paths must be absolute exact .osa filenames");
}

// The old file remains intact if encoding, flush, or fsync fails. The temporary
// file resides beside the destination, so rename is an atomic replacement.
inline void WriteAtomically(const std::string& path,
                            const std::function<void(std::ostream&)>& encode) {
    std::string pattern = path + ".tmp.XXXXXX";
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    int fd = mkstemp(name.data());
    if (fd < 0) throw std::runtime_error("MAP_E_SAVE: mkstemp: " + std::string(strerror(errno)));
    const std::string temp(name.data());
    try {
        std::ofstream out(temp, std::ios::binary | std::ios::trunc);
        out.exceptions(std::ios::failbit | std::ios::badbit);
        encode(out);
        out.flush();
        out.close();
        if (fsync(fd) != 0) throw std::runtime_error("MAP_E_SAVE: fsync failed");
        if (close(fd) != 0) { fd = -1; throw std::runtime_error("MAP_E_SAVE: close failed"); }
        fd = -1;
        if (rename(temp.c_str(), path.c_str()) != 0)
            throw std::runtime_error("MAP_E_SAVE: rename: " + std::string(strerror(errno)));
        int dirfd = open(Parent(path).c_str(), O_RDONLY | O_DIRECTORY);
        if (dirfd < 0) throw std::runtime_error("MAP_E_SAVE: directory open failed after rename");
        int synced = fsync(dirfd);
        close(dirfd);
        if (synced != 0) throw std::runtime_error("MAP_E_SAVE: directory fsync failed after rename");
    } catch (...) {
        if (fd >= 0) close(fd);
        unlink(temp.c_str());
        throw;
    }
}

inline std::string JsonString(const std::string& s) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') out << '\\' << char(c);
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
        else out << char(c);
    }
    out << '"';
    return out.str();
}

inline void CanonicalNode(std::ostream& out, const cv::FileNode& node) {
    if (node.isMap()) {
        std::vector<std::string> keys;
        for (auto it = node.begin(); it != node.end(); ++it) keys.push_back((*it).name());
        std::sort(keys.begin(), keys.end());
        out << '{';
        for (const auto& key : keys) { out << JsonString(key) << ':'; CanonicalNode(out, node[key]); out << ';'; }
        out << '}';
    } else if (node.isSeq()) {
        out << '[';
        for (auto it = node.begin(); it != node.end(); ++it) { CanonicalNode(out, *it); out << ';'; }
        out << ']';
    } else if (node.isString()) out << JsonString(std::string(node));
    else if (node.isInt() || node.isReal()) out << std::setprecision(17) << double(node);
    else out << "null";
}

inline std::string CalibrationFingerprint(const cv::FileStorage& settings) {
    std::vector<std::string> keys;
    for (auto it = settings.root().begin(); it != settings.root().end(); ++it) {
        std::string key = (*it).name();
        if (key.compare(0, 7, "Camera.") == 0 || key.compare(0, 8, "Camera1.") == 0 ||
            (key.compare(0, 4, "IMU.") == 0 && key != "IMU.InsertKFsWhenLost") ||
            key == "ORBextractor.scaleFactor" || key == "ORBextractor.nLevels" || key == "Tbc")
            keys.push_back(key);
    }
    std::sort(keys.begin(), keys.end());
    std::ostringstream canonical;
    canonical << "mono_imu_vio-calibration-v1;";
    for (const auto& key : keys) { canonical << JsonString(key) << ':'; CanonicalNode(canonical, settings[key]); canonical << ';'; }
    const std::string bytes = canonical.str();
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), digest);
    std::ostringstream hex;
    for (unsigned char c : digest) hex << std::hex << std::setw(2) << std::setfill('0') << unsigned(c);
    return hex.str();
}

}} // namespace ORB_SLAM3::AtlasPersistence
#endif
