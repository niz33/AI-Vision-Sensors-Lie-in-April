#pragma once

#include "pros/ai_vision.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace tlia {

struct Vec2 {
    double x = 0;
    double y = 0;
};

struct Pose2D {
    double x = 0;
    double y = 0;
    double theta = 0;
};

struct CameraSettings {
    int resX = 320;
    int resY = 240;
    double fov = 1.4;
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;

    CameraSettings();
    CameraSettings(int resX, int resY, double fov);

    void updateIntrinsics();
};

struct SensorMount {
    pros::AIVision* sensor = nullptr;
    Vec2 position;
    double rotation = 0;
    CameraSettings camera;
};

enum class ResultType {
    noTag,
    singleSensor,
    dualSensor
};

enum class LocalizerState {
    uninitialized,
    ready,
    noTag,
    singleSensor,
    dualSensor
};

struct TagResult {
    ResultType type = ResultType::noTag;
    Pose2D answer;
    std::uint8_t sensorsUsed = 0;
};

class TagLocalizer {
  public:
    explicit TagLocalizer(const std::vector<SensorMount>& sensors);

    void initializeSensors(
        pros::v5::AivisionTagFamily family = pros::v5::AivisionTagFamily::tag_21H7,
        bool resetSensors = true);

    TagResult calculate(std::uint8_t tagId, double robotRotation);

    LocalizerState state() const;
    TagResult lastResult() const;

  private:
    std::vector<SensorMount> sensors;
    LocalizerState currentState = LocalizerState::uninitialized;
    TagResult cachedResult;
};

Vec2 findTagCenter(const std::vector<Vec2>& corners);
double findTagHeightAtCenter(const std::vector<Vec2>& corners, const Vec2& center);

} // namespace tlia
