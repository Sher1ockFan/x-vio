// Part of ORB-SLAM3; distributed under the terms of the GNU GPL, version 3 or later.
#include "OmniRadtan.h"

#include <opencv2/calib3d.hpp>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>

namespace ORB_SLAM3 {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int kMinTriangulated = 50;
constexpr double kCosOneDegree = 0.9998476951563913;

double minQuadratic(double a, double b, double last) {
    double result = std::min(1.0, 1.0 + last * (a + b * last));
    if (b > 0.0) {
        const double vertex = -a / (2.0 * b);
        if (vertex > 0.0 && vertex < last)
            result = std::min(result, 1.0 + vertex * (a + b * vertex));
    }
    return result;
}

// Sufficient lower bound on 1+a*r^2+b*r^4-c*r on r>=0. In the quartic case
// minimize the two halves separately to avoid an unconstrained root solver.
double unboundedRadialBound(double a, double b, double c) {
    if (b < 0.0 || (b == 0.0 && (a < 0.0 || (a == 0.0 && c > 0.0))))
        return -std::numeric_limits<double>::infinity();
    if (b == 0.0)
        return a > 0.0 ? 1.0 - c * c / (4.0 * a) : 1.0;
    const double quadraticLoss = a < 0.0 ? a * a / (2.0 * b) : 0.0;
    return 1.0 - quadraticLoss - 0.75 * c * std::cbrt(c / (2.0 * b));
}

Eigen::Vector2d pixelVector(const cv::Point2f& point) {
    return Eigen::Vector2d(point.x, point.y);
}

bool projectionMatches(GeometricCamera& camera, const Eigen::Vector3d& position,
                       const Eigen::Vector2d& pixel, double thresholdSquared) {
    if (!camera.isVisible(position))
        return false;
    const Eigen::Vector2d projected = camera.project(position);
    return projected.allFinite() && (projected - pixel).squaredNorm() <= thresholdSquared;
}

// The origin of camera two and its bearing are expressed in camera one.
// Positive distances along the two measured rays replace the pinhole z test.
bool triangulateRays(const Eigen::Vector3d& ray1, const Eigen::Vector3d& ray2,
                     const Eigen::Vector3d& center2, Eigen::Vector3d& point,
                     double& cosine) {
    cosine = ray1.dot(ray2);
    if (!std::isfinite(cosine) || std::abs(cosine) >= 0.999999)
        return false;
    const double denominator = 1.0 - cosine * cosine;
    const double projection1 = ray1.dot(center2);
    const double projection2 = ray2.dot(center2);
    const double range1 = (projection1 - cosine * projection2) / denominator;
    const double range2 = (cosine * projection1 - projection2) / denominator;
    if (!(range1 > 1e-8 && range2 > 1e-8))
        return false;
    point = 0.5 * (range1 * ray1 + center2 + range2 * ray2);
    return point.allFinite();
}

using BearingJac = Eigen::Matrix<double,3,2>;
struct BearingMatch {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::size_t index;
    Eigen::Vector2d pixel1, pixel2;
    Eigen::Vector3d ray1, ray2;
    BearingJac jac1, jac2;
    Eigen::Vector3d tangent1, tangent2;
};
using BearingMatches = std::vector<BearingMatch,Eigen::aligned_allocator<BearingMatch>>;

// A tangent-space pseudoinverse maps a pixel displacement to a change in the
// unit bearing. This gives the spherical epipolar score units of pixels.
bool bearingJacobian(OmniRadtan& camera, const Eigen::Vector3d& ray, BearingJac& result) {
    const Eigen::Matrix<double,2,3> projection = camera.projectJac(ray);
    const Eigen::Matrix2d gram = projection * projection.transpose();
    const double determinant = gram.determinant();
    if (!gram.allFinite() || determinant <= 1e-12)
        return false;
    result = projection.transpose() * gram.inverse();
    return result.allFinite();
}

enum class Geometry { Essential, Homography };

bool fitLinearModel(Geometry geometry, const BearingMatches& matches,
                    const std::vector<std::size_t>& indices, Eigen::Matrix3d& model) {
    const bool homography = geometry == Geometry::Homography;
    const int rowsPerMatch = homography ? 2 : 1;
    const int n = static_cast<int>(indices.size());
    if (n * rowsPerMatch < 8)
        return false;
    cv::Mat matrix(n * rowsPerMatch, 9, CV_64F);
    for (int i = 0; i < n; ++i) {
        const BearingMatch& match = matches[indices[i]];
        for (int row = 0; row < rowsPerMatch; ++row) {
            const Eigen::Vector3d& left =
                homography ? (row == 0 ? match.tangent1 : match.tangent2) : match.ray2;
            double* coefficients = matrix.ptr<double>(i * rowsPerMatch + row);
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    coefficients[3*j+k] = left[j] * match.ray1[k];
        }
    }
    cv::Mat values, u, vt;
    cv::SVD::compute(matrix, values, u, vt, matrix.rows < 9 ? cv::SVD::FULL_UV : 0);
    if (vt.rows != 9 || values.at<double>(7) <= 1e-8 * values.at<double>(0))
        return false;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            model(i,j) = vt.at<double>(8, 3*i+j);
    if (homography) {
        if (!model.allFinite() || std::abs(model.determinant()) < 1e-9)
            return false;
        double orientation = 0.0;
        for (std::size_t index : indices) {
            const BearingMatch& match = matches[index];
            orientation += match.ray2.dot(model * match.ray1);
        }
        if (orientation < 0.0)
            model = -model;
    } else {
        // Restrict the linear fit to the nearest normalized essential matrix.
        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(model, Eigen::ComputeFullU | Eigen::ComputeFullV);
        model = svd.matrixU() * Eigen::Vector3d(1.0, 1.0, 0.0).asDiagonal()
                * svd.matrixV().transpose();
    }
    return model.allFinite();
}

struct Model {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3d matrix;
    std::vector<std::size_t> inliers;
    double score = 0.0;
};

Model scoreModel(OmniRadtan& camera, Geometry geometry,
                 const BearingMatches& matches, const Eigen::Matrix3d& candidate) {
    Model result;
    result.matrix = candidate;
    const bool homography = geometry == Geometry::Homography;
    Eigen::Matrix3d inverse = Eigen::Matrix3d::Identity();
    if (homography)
        inverse = candidate.inverse();
    result.inliers.reserve(matches.size());
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const BearingMatch& match = matches[i];
        if (homography) {
            const Eigen::Vector3d predicted2 = candidate * match.ray1;
            const Eigen::Vector3d predicted1 = inverse * match.ray2;
            if (predicted1.dot(match.ray1) <= 0.0 || predicted2.dot(match.ray2) <= 0.0)
                continue;
            const Eigen::Vector2d image1 = camera.project(predicted1);
            const Eigen::Vector2d image2 = camera.project(predicted2);
            const double first = (image1 - match.pixel1).squaredNorm();
            const double second = (image2 - match.pixel2).squaredNorm();
            if (!(first < 5.991 && second < 5.991))
                continue;
            result.score += 1.0 - (first + second) / 11.982;
        } else {
            const Eigen::Vector3d plane2 = candidate * match.ray1;
            const Eigen::Vector3d plane1 = candidate.transpose() * match.ray2;
            const double derivativeSquared = (match.jac1.transpose() * plane1).squaredNorm() +
                                             (match.jac2.transpose() * plane2).squaredNorm();
            const double residual = match.ray2.dot(plane2);
            if (!(derivativeSquared > 1e-20))
                continue;
            const double error = residual * residual / derivativeSquared;
            if (!(error < 5.991))
                continue;
            result.score += 1.0 - error / 5.991;
        }
        result.inliers.push_back(i);
    }
    return result;
}

bool better(const Model& lhs, const Model& rhs) {
    return lhs.inliers.size() > rhs.inliers.size() ||
           (lhs.inliers.size() == rhs.inliers.size() && lhs.score > rhs.score);
}

Model estimate(OmniRadtan& camera, Geometry geometry, const BearingMatches& matches) {
    const int sampleSize = geometry == Geometry::Homography ? 4 : 8;
    int iterations = geometry == Geometry::Homography ? 240 : 512;
    std::mt19937 rng(0);  // Reproducible and local to this reconstruction.
    std::vector<std::size_t> pool(matches.size());
    std::iota(pool.begin(), pool.end(), std::size_t(0));
    std::vector<std::size_t> sample(sampleSize);
    Model best;
    for (int it = 0; it < iterations; ++it) {
        // Only the sampled prefix is shuffled; each iteration still draws a
        // uniform subset without replacement from the complete pool.
        for (int i = 0; i < sampleSize; ++i) {
            const std::size_t index = std::uniform_int_distribution<std::size_t>(
                static_cast<std::size_t>(i), pool.size() - 1)(rng);
            std::swap(pool[index], pool[i]);
            sample[i] = pool[i];
        }
        Eigen::Matrix3d matrix;
        if (!fitLinearModel(geometry, matches, sample, matrix))
            continue;
        Model current = scoreModel(camera, geometry, matches, matrix);
        if (!better(current, best))
            continue;
        best = std::move(current);
        const double probability = std::pow(
            static_cast<double>(best.inliers.size()) / matches.size(), sampleSize);
        if (probability >= 1.0) {
            iterations = std::min(iterations, 64);
        } else if (probability > 0.0) {
            const double bound = std::log(0.005) / std::log1p(-probability);
            if (bound < iterations)
                iterations = std::max(64, static_cast<int>(std::ceil(bound)));
        }
    }
    for (int refinement = 0; refinement < 2; ++refinement) {
        Eigen::Matrix3d matrix;
        if (!fitLinearModel(geometry, matches, best.inliers, matrix))
            break;
        Model current = scoreModel(camera, geometry, matches, matrix);
        if (!better(current, best))
            break;
        best = std::move(current);
    }
    return best;
}

cv::Mat cvMatrix(const Eigen::Matrix3d& matrix) {
    cv::Mat result(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            result.at<double>(i,j) = matrix(i,j);
    return result;
}

struct Motion {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    std::vector<cv::Point3f> points;
    std::vector<bool> valid;
    std::size_t count = 0;
    double error = 0.0;
};

using Motions = std::vector<Motion,Eigen::aligned_allocator<Motion>>;

void evaluateMotion(OmniRadtan& camera, const BearingMatches& matches, const Model& model,
                    const cv::Mat& rotation, const cv::Mat& translation,
                    std::size_t keyCount, Motions& motions) {
    Motion motion;
    for (int i = 0; i < 3; ++i) {
        motion.translation[i] = translation.at<double>(i);
        for (int j = 0; j < 3; ++j)
            motion.rotation(i,j) = rotation.at<double>(i,j);
    }
    if (!motion.rotation.allFinite() || !motion.translation.allFinite() ||
        motion.translation.norm() < 1e-7 || std::abs(motion.rotation.determinant() - 1.0) > 1e-5)
        return;
    motion.translation.normalize();  // Same arbitrary scale for all hypotheses.
    const Eigen::Vector3d center2 = -motion.rotation.transpose() * motion.translation;
    motion.points.resize(keyCount);
    motion.valid.assign(keyCount, false);
    std::size_t parallaxCount = 0;
    for (std::size_t index : model.inliers) {
        const BearingMatch& match = matches[index];
        const Eigen::Vector3d ray2 = motion.rotation.transpose() * match.ray2;
        Eigen::Vector3d point;
        double cosine;
        if (!triangulateRays(match.ray1, ray2, center2, point, cosine))
            continue;
        const Eigen::Vector3d point2 = motion.rotation * point + motion.translation;
        const double error1 = (camera.project(point) - match.pixel1).squaredNorm();
        const double error2 = (camera.project(point2) - match.pixel2).squaredNorm();
        if (!(error1 <= 5.991 && error2 <= 5.991))
            continue;
        motion.points[match.index] = cv::Point3f(point.x(), point.y(), point.z());
        motion.valid[match.index] = true;
        ++motion.count;
        if (cosine < kCosOneDegree)
            ++parallaxCount;
        motion.error += error1 + error2;
    }
    if (motion.count >= kMinTriangulated && motion.count >= 0.75 * model.inliers.size() &&
        parallaxCount >= std::min<std::size_t>(kMinTriangulated, motion.count))
        motions.push_back(std::move(motion));
}

bool equivalentMotion(const Motion& a, const Motion& b) {
    // The essential and homography solvers may produce the same hypothesis
    // with different noisy estimates. Treat nearly identical poses as one.
    const double relativeRotation = ((a.rotation.transpose() * b.rotation).trace() - 1.0) / 2.0;
    return relativeRotation > 0.999847695 && a.translation.dot(b.translation) > 0.996194698;
}

}  // namespace

OmniRadtan::OmniRadtan() : OmniRadtan({1.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}) {}

OmniRadtan::OmniRadtan(const std::vector<float>& parameters) : GeometricCamera(parameters) {
    mnType = CAM_OMNI;
    validateParameters();
    mnId = nNextId++;
}

void OmniRadtan::validateParameters() const {
    if (mnType != CAM_OMNI || mvParameters.size() != 9 ||
        !std::all_of(mvParameters.begin(), mvParameters.end(), [](float v) { return std::isfinite(v); }) ||
        mvParameters[0] <= 0.f || mvParameters[1] <= 0.f || mvParameters[4] < 0.f)
        throw std::invalid_argument("OmniRadtan requires [fx,fy,cx,cy,xi,k1,k2,p1,p2], with positive focal lengths and nonnegative xi.");
    const double xi = mvParameters[4];
    const double k1 = mvParameters[5], k2 = mvParameters[6];
    const double tangent = 6.0 * std::hypot(mvParameters[7], mvParameters[8]);
    double bound;
    if (xi > 1.0) {
        const double limitSquared = 1.0 / (xi * xi - 1.0);
        bound = std::min(minQuadratic(k1, k2, limitSquared),
                         minQuadratic(3.0*k1, 5.0*k2, limitSquared))
              - tangent * std::sqrt(limitSquared);
    } else {
        bound = std::min(unboundedRadialBound(k1, k2, tangent),
                         unboundedRadialBound(3.0*k1, 5.0*k2, tangent));
    }
    // The radial Jacobian is symmetric. The tangential perturbation's spectral
    // norm is <=6*hypot(p1,p2)*radius, so this bound certifies an injective map
    // over the convex Kalibr chart; an inverse root cannot silently change branch.
    if (!std::isfinite(bound) || bound <= 1e-9)
        throw std::invalid_argument("OmniRadtan distortion is not certified injective on the Kalibr chart.");
}

bool OmniRadtan::chart(const Eigen::Vector3d& point, double& norm, double& denominator) const {
    const double xi = mvParameters[4];
    if (!point.allFinite())
        return false;
    norm = point.norm();
    if (!std::isfinite(norm) || norm <= 0.0)
        return false;
    const double minimumZ = xi > 1.0 ? -1.0 / xi : -xi;
    denominator = point.z() + xi * norm;
    return point.z() > (minimumZ + 1e-12) * norm && denominator > 0.0 && std::isfinite(denominator);
}

bool OmniRadtan::isVisible(const Eigen::Vector3d& point) const {
    double norm, denominator;
    return chart(point, norm, denominator);
}

Eigen::Vector2d OmniRadtan::distort(const Eigen::Vector2d& q) const {
    const double x = q.x(), y = q.y(), r2 = q.squaredNorm();
    const double p1 = mvParameters[7], p2 = mvParameters[8];
    const double radial = 1.0 + r2 * (mvParameters[5] + mvParameters[6] * r2);
    return Eigen::Vector2d(x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0*x*x),
                           y * radial + p1 * (r2 + 2.0*y*y) + 2.0 * p2 * x * y);
}

Eigen::Matrix2d OmniRadtan::distortionJac(const Eigen::Vector2d& q) const {
    const double x = q.x(), y = q.y(), r2 = q.squaredNorm();
    const double k1 = mvParameters[5], k2 = mvParameters[6];
    const double p1 = mvParameters[7], p2 = mvParameters[8];
    const double radial = 1.0 + r2 * (k1 + k2 * r2);
    const double slope = 2.0 * k1 + 4.0 * k2 * r2;
    const double cross = slope * x * y + 2.0 * p1 * x + 2.0 * p2 * y;
    Eigen::Matrix2d jac;
    jac << radial + slope*x*x + 2.0*p1*y + 6.0*p2*x, cross,
           cross, radial + slope*y*y + 6.0*p1*y + 2.0*p2*x;
    return jac;
}

Eigen::Vector2d OmniRadtan::project(const Eigen::Vector3d& point) {
    double norm, denominator;
    if (!chart(point, norm, denominator))
        return Eigen::Vector2d::Constant(kNaN);
    const Eigen::Vector2d q = distort(point.head<2>() / denominator);
    return Eigen::Vector2d(mvParameters[0] * q.x() + mvParameters[2],
                           mvParameters[1] * q.y() + mvParameters[3]);
}

Eigen::Vector2f OmniRadtan::project(const Eigen::Vector3f& point) {
    return project(Eigen::Vector3d(point.cast<double>())).cast<float>();
}

Eigen::Vector2f OmniRadtan::projectMat(const cv::Point3f& point) {
    return project(Eigen::Vector3f(point.x, point.y, point.z));
}

cv::Point2f OmniRadtan::project(const cv::Point3f& point) {
    const Eigen::Vector2f uv = projectMat(point);
    return cv::Point2f(uv.x(), uv.y());
}

Eigen::Matrix<double,2,3> OmniRadtan::projectJac(const Eigen::Vector3d& point) {
    double norm, denominator;
    if (!chart(point, norm, denominator))
        return Eigen::Matrix<double,2,3>::Constant(kNaN);
    Eigen::Vector3d denominatorJac = (mvParameters[4] / norm) * point;
    denominatorJac.z() += 1.0;
    const Eigen::Vector2d q = point.head<2>() / denominator;
    Eigen::Matrix<double,2,3> sphereJac = Eigen::Matrix<double,2,3>::Identity();
    sphereJac -= q * denominatorJac.transpose();
    sphereJac /= denominator;
    return Eigen::Vector2d(mvParameters[0], mvParameters[1]).asDiagonal() * distortionJac(q) * sphereJac;
}

Eigen::Vector3d OmniRadtan::inverse(const cv::Point2f& pixel) const {
    const Eigen::Vector3d invalid = Eigen::Vector3d::Constant(kNaN);
    const Eigen::Vector2d target((static_cast<double>(pixel.x) - mvParameters[2]) / mvParameters[0],
                                 (static_cast<double>(pixel.y) - mvParameters[3]) / mvParameters[1]);
    if (!target.allFinite())
        return invalid;
    const double xi = mvParameters[4];
    const double radiusSquared = xi > 1.0 ? 1.0 / (xi*xi - 1.0)
                                         : std::numeric_limits<double>::infinity();
    Eigen::Vector2d q = target;
    if (q.squaredNorm() >= 0.64 * radiusSquared)
        q *= 0.8 * std::sqrt(radiusSquared) / q.norm();
    double error = (distort(q) - target).squaredNorm();
    for (int i = 0; i < 48 && error > 1e-24; ++i) {
        const Eigen::Matrix2d jac = distortionJac(q);
        if (!jac.allFinite() || jac.determinant() <= 0.0)
            return invalid;
        const Eigen::Vector2d delta = jac.inverse() * (distort(q) - target);
        double step = 1.0;
        bool improved = false;
        for (int j = 0; j < 32; ++j, step *= 0.5) {
            const Eigen::Vector2d trial = q - step * delta;
            if (trial.squaredNorm() >= radiusSquared)
                continue;
            const double nextError = (distort(trial) - target).squaredNorm();
            if (nextError < error) {
                improved = true;
                q = trial;
                error = nextError;
                break;
            }
        }
        if (!improved)
            break;
    }
    const double r2 = q.squaredNorm();
    const double discriminant = 1.0 + (1.0 - xi*xi) * r2;
    if (!(error <= 1e-22 && r2 < radiusSquared && discriminant > 0.0))
        return invalid;
    // Kalibr selects the positive root. The other root is outside the
    // one-to-one visibility chart when xi > 1.
    const double scale = (xi + std::sqrt(discriminant)) / (1.0 + r2);
    Eigen::Vector3d ray(scale * q.x(), scale * q.y(), scale - xi);
    if (!isVisible(ray))
        return invalid;
    return ray.normalized();
}

Eigen::Vector3f OmniRadtan::unprojectEig(const cv::Point2f& pixel) {
    return inverse(pixel).cast<float>();
}

cv::Point3f OmniRadtan::unproject(const cv::Point2f& pixel) {
    const Eigen::Vector3f ray = unprojectEig(pixel);
    return cv::Point3f(ray.x(), ray.y(), ray.z());
}

float OmniRadtan::uncertainty2(const Eigen::Matrix<double,2,1>&) {
    return 1.f;  // Same nominal pixel variance as the other mono camera models.
}

Eigen::Matrix3f OmniRadtan::toK_() {
    Eigen::Matrix3f intrinsic = Eigen::Matrix3f::Identity();
    intrinsic(0,0) = mvParameters[0];
    intrinsic(1,1) = mvParameters[1];
    intrinsic(0,2) = mvParameters[2];
    intrinsic(1,2) = mvParameters[3];
    return intrinsic;
}

cv::Mat OmniRadtan::toK() {
    const Eigen::Matrix3f matrix = toK_();
    cv::Mat result(3, 3, CV_32F);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            result.at<float>(i,j) = matrix(i,j);
    return result;
}

bool OmniRadtan::IsEqual(GeometricCamera* other) const {
    if (!other || other->GetType() != CAM_OMNI || other->size() != mvParameters.size())
        return false;
    for (std::size_t i = 0; i < mvParameters.size(); ++i)
        if (mvParameters[i] != other->getParameter(static_cast<int>(i)))
            return false;
    return true;
}

bool OmniRadtan::epipolarConstrain(GeometricCamera* other, const cv::KeyPoint& key1,
                                 const cv::KeyPoint& key2, const Eigen::Matrix3f& R12,
                                 const Eigen::Vector3f& t12, float variance1, float variance2) {
    // R12,t12 are camera 2 -> camera 1, as passed by ORBmatcher.
    const Eigen::Vector3d ray1 = inverse(key1.pt);
    const Eigen::Vector3d ray2 = other->unprojectEig(key2.pt).cast<double>().normalized();
    const Eigen::Matrix3d rotation = R12.cast<double>();
    const Eigen::Vector3d center = t12.cast<double>();
    Eigen::Vector3d point;
    double cosine;
    return triangulateRays(ray1, rotation * ray2, center, point, cosine)
        && std::abs(cosine) < 0.9998
        && projectionMatches(*this, point, pixelVector(key1.pt), 5.991 * variance1)
        && projectionMatches(*other, rotation.transpose() * (point - center),
                             pixelVector(key2.pt), 5.991 * variance2);
}

bool OmniRadtan::matchAndtriangulate(const cv::KeyPoint& key1, const cv::KeyPoint& key2,
                                   GeometricCamera* other, Sophus::SE3f& Tcw1,
                                   Sophus::SE3f& Tcw2, float variance1, float variance2,
                                   Eigen::Vector3f& worldPoint) {
    const Sophus::SE3f T12 = Tcw1 * Tcw2.inverse();
    const Eigen::Matrix3d rotation = T12.rotationMatrix().cast<double>();
    const Eigen::Vector3d center = T12.translation().cast<double>();
    const Eigen::Vector3d ray1 = inverse(key1.pt);
    const Eigen::Vector3d ray2 = other->unprojectEig(key2.pt).cast<double>().normalized();
    Eigen::Vector3d point;
    double cosine;
    if (!triangulateRays(ray1, rotation * ray2, center, point, cosine)
        || std::abs(cosine) >= 0.9998
        || !projectionMatches(*this, point, pixelVector(key1.pt), 5.991 * variance1)
        || !projectionMatches(*other, rotation.transpose() * (point - center),
                              pixelVector(key2.pt), 5.991 * variance2))
        return false;
    worldPoint = Tcw1.inverse() * Eigen::Vector3f(point.cast<float>());
    return worldPoint.allFinite();
}

bool OmniRadtan::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& keys1,
                                        const std::vector<cv::KeyPoint>& keys2,
                                        const std::vector<int>& indices,
                                        Sophus::SE3f& T21, std::vector<cv::Point3f>& points,
                                        std::vector<bool>& triangulated) {
    if (indices.size() != keys1.size())
        return false;
    BearingMatches matches;
    matches.reserve(keys1.size());
    for (std::size_t i = 0; i < indices.size(); ++i) {
        const int j = indices[i];
        if (j < 0 || static_cast<std::size_t>(j) >= keys2.size())
            continue;
        BearingMatch match;
        match.index = i;
        match.pixel1 = pixelVector(keys1[i].pt);
        match.pixel2 = pixelVector(keys2[j].pt);
        match.ray1 = inverse(keys1[i].pt);
        match.ray2 = inverse(keys2[j].pt);
        if (!match.ray1.allFinite() || !match.ray2.allFinite() ||
            !bearingJacobian(*this, match.ray1, match.jac1) ||
            !bearingJacobian(*this, match.ray2, match.jac2))
            continue;
        match.tangent1 = match.ray2.unitOrthogonal();
        match.tangent2 = match.ray2.cross(match.tangent1);
        matches.push_back(match);
    }
    if (matches.size() < kMinTriangulated)
        return false;

    // Estimate calibrated two-view models directly on oriented unit rays,
    // including the optical horizon and the part of the chart behind it.
    const Model essential = estimate(*this, Geometry::Essential, matches);
    const Model homography = estimate(*this, Geometry::Homography, matches);
    Motions motions;
    if (essential.inliers.size() >= kMinTriangulated) {
        cv::Mat R1, R2, t;
        cv::decomposeEssentialMat(cvMatrix(essential.matrix), R1, R2, t);
        evaluateMotion(*this, matches, essential, R1,  t, keys1.size(), motions);
        evaluateMotion(*this, matches, essential, R1, -t, keys1.size(), motions);
        evaluateMotion(*this, matches, essential, R2,  t, keys1.size(), motions);
        evaluateMotion(*this, matches, essential, R2, -t, keys1.size(), motions);
    }
    if (homography.inliers.size() >= kMinTriangulated) {
        std::vector<cv::Mat> rotations, translations, normals;
        cv::decomposeHomographyMat(cvMatrix(homography.matrix), cv::Mat::eye(3,3,CV_64F),
                                   rotations, translations, normals);
        for (std::size_t i = 0; i < rotations.size(); ++i)
            evaluateMotion(*this, matches, homography, rotations[i], translations[i], keys1.size(), motions);
    }
    if (motions.empty())
        return false;
    std::sort(motions.begin(), motions.end(), [](const Motion& a, const Motion& b) {
        return a.count != b.count ? a.count > b.count : a.error < b.error;
    });
    const Motion& best = motions.front();
    for (std::size_t i = 1; i < motions.size(); ++i)
        if (motions[i].count > 0.80 * best.count && !equivalentMotion(best, motions[i]))
            return false;  // Ambiguous depth/motion: wait for another frame.
    const Eigen::Matrix3f rotation = best.rotation.cast<float>();
    T21 = Sophus::SE3f(Eigen::Quaternionf(rotation).normalized(), best.translation.cast<float>());
    points = best.points;
    triangulated = best.valid;
    return true;
}

}  // namespace ORB_SLAM3
