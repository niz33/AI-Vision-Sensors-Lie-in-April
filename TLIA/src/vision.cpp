#include "tlia/vision.hpp"

#include <cmath>

namespace tlia {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;
constexpr double kDistanceScale = 166.37;

struct Detection {
    std::size_t sensorIndex = 0;
    Vec2 tagInRobot;
    Vec2 rayOrigin;
    Vec2 rayDirection;
};

double degToRad(double deg) {
    return deg * kPi / 180.0;
}

Vec2 operator+(const Vec2& lhs, const Vec2& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y};
}

Vec2 operator-(const Vec2& lhs, const Vec2& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y};
}

Vec2 operator*(const Vec2& value, double scalar) {
    return {value.x * scalar, value.y * scalar};
}

double cross(const Vec2& lhs, const Vec2& rhs) {
    return lhs.x * rhs.y - lhs.y * rhs.x;
}

Vec2 rotate(const Vec2& value, double deg) {
    const double angle = degToRad(deg);
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {
        value.x * c - value.y * s,
        value.x * s + value.y * c
    };
}

std::vector<Vec2> tagCorners(const pros::AIVision::Object& object) {
    const auto& tag = object.object.tag;
    return {
        {static_cast<double>(tag.x0), static_cast<double>(tag.y0)},
        {static_cast<double>(tag.x1), static_cast<double>(tag.y1)},
        {static_cast<double>(tag.x2), static_cast<double>(tag.y2)},
        {static_cast<double>(tag.x3), static_cast<double>(tag.y3)}
    };
}

// Finds a requested AprilTag in one sensor frame and converts it into a ray
// from the robot base frame. The ray is used for dual-sensor triangulation,
// and the estimated range is kept for single-sensor fallback.
bool readDetection(const SensorMount& mount, std::uint8_t tagId, std::size_t sensorIndex, Detection& detection) {
    if (mount.sensor == nullptr) return false;

    const auto objects = mount.sensor->get_all_objects();
    for (const auto& object : objects) {
        if (!pros::AIVision::is_type(object, pros::v5::AivisionDetectType::tag) || object.id != tagId) {
            continue;
        }

        const auto corners = tagCorners(object);
        const Vec2 center = findTagCenter(corners);
        const double height = findTagHeightAtCenter(corners, center);
        if (height < kEps) continue;

        const double bearing = std::atan2(center.x - mount.camera.cx, mount.camera.fx);
        const double distance = kDistanceScale / height;
        const Vec2 tagInCamera = {std::tan(bearing) * distance, distance};

        detection.sensorIndex = sensorIndex;
        detection.tagInRobot = mount.position + rotate(tagInCamera, mount.rotation);
        detection.rayOrigin = mount.position;
        detection.rayDirection = rotate({std::sin(bearing), std::cos(bearing)}, mount.rotation);
        return true;
    }

    return false;
}

// Intersects two camera rays in the robot frame. This generalizes the older
// fixed-baseline stereo math to arbitrary sensor positions and yaw rotations.
bool intersectRays(const Detection& a, const Detection& b, Vec2& tagInRobot) {
    const double denom = cross(a.rayDirection, b.rayDirection);
    if (std::abs(denom) < kEps) return false;

    const Vec2 delta = b.rayOrigin - a.rayOrigin;
    const double t = cross(delta, b.rayDirection) / denom;
    tagInRobot = a.rayOrigin + a.rayDirection * t;
    return true;
}

// Converts the tag position measured in robot coordinates into the robot base
// position in tag coordinates, using the caller-provided robot rotation.
Pose2D robotPoseFromTagVector(const Vec2& tagInRobot, double robotRotation) {
    const Vec2 robotInTag = rotate(tagInRobot * -1.0, robotRotation);
    return {robotInTag.x, robotInTag.y, robotRotation};
}

} // namespace

CameraSettings::CameraSettings() {
    updateIntrinsics();
}

CameraSettings::CameraSettings(int resX, int resY, double fov) : resX(resX), resY(resY), fov(fov) {
    updateIntrinsics();
}

// Updates the pinhole camera intrinsics from the configured resolution and
// horizontal field of view. The AI Vision image is treated as square-pixel.
void CameraSettings::updateIntrinsics() {
    fx = (resX / 2.0) / std::tan(fov / 2.0);
    fy = fx;
    cx = resX / 2.0;
    cy = resY / 2.0;
}

// Finds the tag center by intersecting the two projected diagonals. This is
// more stable under perspective skew than averaging side midpoints.
Vec2 findTagCenter(const std::vector<Vec2>& corners) {
    if (corners.size() < 4) return {};

    const Vec2 d02 = corners[2] - corners[0];
    const Vec2 d13 = corners[3] - corners[1];
    const Vec2 p1ToP0 = corners[1] - corners[0];

    const double denom = cross(d02, d13);
    if (std::abs(denom) < kEps) {
        return {
            (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0,
            (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0
        };
    }

    const double t = cross(p1ToP0, d13) / denom;
    return corners[0] + d02 * t;
}

// Measures the vertical image height of the quadrilateral at the center point.
// It intersects a vertical screen line with every edge, then uses the highest
// and lowest intersections as the local projected tag height.
double findTagHeightAtCenter(const std::vector<Vec2>& corners, const Vec2& center) {
    if (corners.size() < 4) return 0;

    double minY = 0;
    double maxY = 0;
    int hits = 0;

    for (std::size_t i = 0; i < 4; ++i) {
        const Vec2& a = corners[i];
        const Vec2& b = corners[(i + 1) % 4];
        const double minX = a.x < b.x ? a.x : b.x;
        const double maxX = a.x > b.x ? a.x : b.x;

        if (std::abs(b.x - a.x) < kEps) {
            if (std::abs(center.x - a.x) >= kEps) continue;
            const double y = (a.y + b.y) / 2.0;
            minY = hits == 0 || y < minY ? y : minY;
            maxY = hits == 0 || y > maxY ? y : maxY;
            ++hits;
            continue;
        }

        if (center.x < minX - kEps || center.x > maxX + kEps) continue;

        const double t = (center.x - a.x) / (b.x - a.x);
        const double y = a.y + t * (b.y - a.y);
        minY = hits == 0 || y < minY ? y : minY;
        maxY = hits == 0 || y > maxY ? y : maxY;
        ++hits;
    }

    return hits >= 2 ? std::abs(maxY - minY) : 0;
}

TagLocalizer::TagLocalizer(const std::vector<SensorMount>& sensors) : sensors(sensors) {}

// Places every configured AI Vision sensor into AprilTag mode. Call this once
// during robot initialization after the sensor objects have been constructed.
void TagLocalizer::initializeSensors(pros::v5::AivisionTagFamily family, bool resetSensors) {
    for (const auto& mount : sensors) {
        if (mount.sensor == nullptr) continue;
        if (resetSensors) mount.sensor->reset();
        mount.sensor->enable_detection_types(pros::v5::AivisionModeType::tags);
        mount.sensor->set_tag_family(family, true);
    }

    currentState = LocalizerState::ready;
}

// Calculates the robot base pose relative to one tag. Two detections use ray
// intersection; one detection uses apparent tag height for range; no detections
// return ResultType::noTag.
TagResult TagLocalizer::calculate(std::uint8_t tagId, double robotRotation) {
    std::vector<Detection> detections;
    detections.reserve(sensors.size());

    for (std::size_t i = 0; i < sensors.size(); ++i) {
        Detection detection;
        if (readDetection(sensors[i], tagId, i, detection)) {
            detections.push_back(detection);
        }
    }

    if (detections.size() >= 2) {
        Vec2 tagInRobot;
        if (intersectRays(detections[0], detections[1], tagInRobot)) {
            cachedResult = {ResultType::dualSensor, robotPoseFromTagVector(tagInRobot, robotRotation), 2};
            currentState = LocalizerState::dualSensor;
            return cachedResult;
        }
    }

    if (!detections.empty()) {
        cachedResult = {ResultType::singleSensor, robotPoseFromTagVector(detections[0].tagInRobot, robotRotation), 1};
        currentState = LocalizerState::singleSensor;
        return cachedResult;
    }

    cachedResult = {ResultType::noTag, {}, 0};
    currentState = LocalizerState::noTag;
    return cachedResult;
}

LocalizerState TagLocalizer::state() const {
    return currentState;
}

TagResult TagLocalizer::lastResult() const {
    return cachedResult;
}

} // namespace tlia
