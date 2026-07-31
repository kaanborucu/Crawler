#include "policy_pipeline.h"

#include <cmath>
#include <limits>

#include "student_policy.h"

#if defined(ARDUINO)
#include <esp_timer.h>
#else
#include <chrono>
#endif

namespace {

uint64_t monotonicUs() {
#if defined(ARDUINO)
  return static_cast<uint64_t>(esp_timer_get_time());
#else
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                  now.time_since_epoch())
                                  .count());
#endif
}

float clampValue(float value, float minimum, float maximum) {
  if (!std::isfinite(value)) return 0.0f;
  return std::fmax(minimum, std::fmin(maximum, value));
}

bool finiteArray(const float* values, uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    if (!std::isfinite(values[i])) return false;
  }
  return true;
}

}  // namespace

PolicyPipeline::PolicyPipeline()
    : positionHistory_{},
      velocityHistory_{},
      actionHistory_{},
      commandHistory_{},
      previousFilteredTargetRad_{},
      observation_{},
      modelInput_{},
      modelOutput_{},
      loadAttempted_(false),
      loaded_(false),
      initialized_(false),
      failureCode_(crawler::FaultCode::None) {}

bool PolicyPipeline::begin() {
  if (!loadAttempted_) {
    loadAttempted_ = true;
    loaded_ = student_policy_load(nullptr);
    if (!loaded_) failureCode_ = crawler::FaultCode::PolicyLoadFailure;
  }
  return loaded_;
}

bool PolicyPipeline::preprocessJointState(const crawler::JointState& measured,
                                          float position[3], float velocity[3]) {
  if (!measured.valid || position == nullptr || velocity == nullptr) return false;
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    if (!std::isfinite(measured.positionRad[i]) ||
        !std::isfinite(measured.velocityRadPerSecond[i]) ||
        !std::isfinite(crawler::config::servo::calibrations[i].defaultPositionRad)) {
      return false;
    }
    position[i] = clampValue(
        measured.positionRad[i] -
            crawler::config::servo::calibrations[i].defaultPositionRad,
        -crawler::config::policy::positionClampRad,
        crawler::config::policy::positionClampRad);
    velocity[i] =
        clampValue(measured.velocityRadPerSecond[i],
                   -crawler::config::policy::velocityClampRadPerSecond,
                   crawler::config::policy::velocityClampRadPerSecond) *
        crawler::config::policy::velocityScale;
  }
  return true;
}

bool PolicyPipeline::preprocessCommand(const crawler::VelocityCommand& command,
                                       float& forward, float& lateral) {
  if (!command.valid || !std::isfinite(command.forwardMetersPerSecond) ||
      !std::isfinite(command.lateralMetersPerSecond)) {
    return false;
  }
  forward = clampValue(command.forwardMetersPerSecond,
                       -crawler::config::policy::commandClampMetersPerSecond,
                       crawler::config::policy::commandClampMetersPerSecond);
  lateral = clampValue(command.lateralMetersPerSecond,
                       -crawler::config::policy::commandClampMetersPerSecond,
                       crawler::config::policy::commandClampMetersPerSecond);
  return true;
}

void PolicyPipeline::clearHistories() {
  for (uint8_t frame = 0; frame < crawler::config::policy::historyFrames;
       ++frame) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      positionHistory_[frame][i] = 0.0f;
      velocityHistory_[frame][i] = 0.0f;
      actionHistory_[frame][i] = 0.0f;
    }
    commandHistory_[frame][0] = 0.0f;
    commandHistory_[frame][1] = 0.0f;
  }
}

void PolicyPipeline::reset() {
  clearHistories();
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    previousFilteredTargetRad_[i] = 0.0f;
  }
  for (uint8_t i = 0; i < crawler::config::policy::observationSize; ++i) {
    observation_[i] = 0.0f;
  }
  initialized_ = false;
  failureCode_ = crawler::FaultCode::None;
}

void PolicyPipeline::fillInitialHistory(const float position[3],
                                        const float velocity[3], float forward,
                                        float lateral) {
  clearHistories();
  for (uint8_t frame = 0; frame < crawler::config::policy::historyFrames;
       ++frame) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      positionHistory_[frame][i] = position[i];
      velocityHistory_[frame][i] = velocity[i];
    }
    commandHistory_[frame][0] = forward;
    commandHistory_[frame][1] = lateral;
  }
}

bool PolicyPipeline::initialize(
    const crawler::JointState& initialJointState,
    const crawler::VelocityCommand& initialCommand) {
  if (!begin()) return false;
  float position[3] = {};
  float velocity[3] = {};
  float forward = 0.0f;
  float lateral = 0.0f;
  if (!preprocessJointState(initialJointState, position, velocity) ||
      !preprocessCommand(initialCommand, forward, lateral)) {
    initialized_ = false;
    failureCode_ = crawler::FaultCode::NonFiniteObservation;
    return false;
  }
  fillInitialHistory(position, velocity, forward, lateral);
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    previousFilteredTargetRad_[i] = 0.0f;
  }
  initialized_ = true;
  return true;
}

void PolicyPipeline::pushPosition(const float position[3]) {
  for (uint8_t frame = 1; frame < crawler::config::policy::historyFrames;
       ++frame) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      positionHistory_[frame - 1][i] = positionHistory_[frame][i];
    }
  }
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    positionHistory_[crawler::config::policy::historyFrames - 1][i] = position[i];
  }
}

void PolicyPipeline::pushVelocity(const float velocity[3]) {
  for (uint8_t frame = 1; frame < crawler::config::policy::historyFrames;
       ++frame) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      velocityHistory_[frame - 1][i] = velocityHistory_[frame][i];
    }
  }
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    velocityHistory_[crawler::config::policy::historyFrames - 1][i] = velocity[i];
  }
}

void PolicyPipeline::pushCommand(float forward, float lateral) {
  for (uint8_t frame = 1; frame < crawler::config::policy::historyFrames;
       ++frame) {
    commandHistory_[frame - 1][0] = commandHistory_[frame][0];
    commandHistory_[frame - 1][1] = commandHistory_[frame][1];
  }
  commandHistory_[crawler::config::policy::historyFrames - 1][0] = forward;
  commandHistory_[crawler::config::policy::historyFrames - 1][1] = lateral;
}

void PolicyPipeline::pushAction(const float action[3]) {
  for (uint8_t frame = 1; frame < crawler::config::policy::historyFrames;
       ++frame) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      actionHistory_[frame - 1][i] = actionHistory_[frame][i];
    }
  }
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    actionHistory_[crawler::config::policy::historyFrames - 1][i] = action[i];
  }
}

void PolicyPipeline::flattenObservation() {
  uint8_t index = 0;
  for (uint8_t frame = 0; frame < crawler::config::policy::historyFrames;
       ++frame) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      observation_[index++] = positionHistory_[frame][i];
    }
  }
  for (uint8_t frame = 0; frame < crawler::config::policy::historyFrames;
       ++frame) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      observation_[index++] = velocityHistory_[frame][i];
    }
  }
  for (uint8_t frame = 0; frame < crawler::config::policy::historyFrames;
       ++frame) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      observation_[index++] = actionHistory_[frame][i];
    }
  }
  for (uint8_t frame = 0; frame < crawler::config::policy::historyFrames;
       ++frame) {
    observation_[index++] = commandHistory_[frame][0];
    observation_[index++] = commandHistory_[frame][1];
  }
}

bool PolicyPipeline::step(const crawler::JointState& jointState,
                          const crawler::VelocityCommand& command,
                          crawler::PolicyResult& result) {
  result = {};
  if (!initialized_ || !begin()) return false;

  float position[3] = {};
  float velocity[3] = {};
  float forward = 0.0f;
  float lateral = 0.0f;
  if (!preprocessJointState(jointState, position, velocity) ||
      !preprocessCommand(command, forward, lateral)) {
    failureCode_ = crawler::FaultCode::NonFiniteObservation;
    return false;
  }

  pushPosition(position);
  pushVelocity(velocity);
  pushCommand(forward, lateral);
  flattenObservation();
  if (!finiteArray(observation_, crawler::config::policy::observationSize)) {
    failureCode_ = crawler::FaultCode::NonFiniteObservation;
    return false;
  }
  for (uint8_t i = 0; i < crawler::config::policy::observationSize; ++i) {
    modelInput_[0][i] = observation_[i];
  }

  const uint64_t startUs = monotonicUs();
  student_policy(modelInput_, modelOutput_);
  const uint64_t elapsedUs = monotonicUs() - startUs;
  result.inferenceTimeUs = elapsedUs > std::numeric_limits<uint32_t>::max()
                               ? std::numeric_limits<uint32_t>::max()
                               : static_cast<uint32_t>(elapsedUs);

  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    if (!std::isfinite(modelOutput_[0][i])) {
      failureCode_ = crawler::FaultCode::NonFinitePolicyOutput;
      return false;
    }
    result.rawActions[i] = modelOutput_[0][i];
    result.clampedActions[i] =
        clampValue(result.rawActions[i], -1.0f, 1.0f);
  }
  // The current action is available to the next observation only. It must not
  // influence the observation that produced it.
  pushAction(result.clampedActions);
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    result.targetRad[i] = result.clampedActions[i] *
                          crawler::config::policy::actionScaleRad;
    result.filteredTargetRad[i] =
        crawler::config::policy::filterPreviousWeight *
            previousFilteredTargetRad_[i] +
        crawler::config::policy::filterNewWeight * result.targetRad[i];
    if (!std::isfinite(result.filteredTargetRad[i])) {
      failureCode_ = crawler::FaultCode::NonFinitePolicyOutput;
      return false;
    }
  }
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    previousFilteredTargetRad_[i] = result.filteredTargetRad[i];
  }
  result.valid = true;
  return true;
}

const float* PolicyPipeline::latestObservation() const { return observation_; }

crawler::FaultCode PolicyPipeline::failureCode() const { return failureCode_; }
