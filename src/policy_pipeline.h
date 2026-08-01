#pragma once

#include "crawler_config.h"
#include "crawler_types.h"

class PolicyPipeline {
 public:
  PolicyPipeline();

  bool begin();
  bool initialize(const crawler::JointState& initialJointState,
                  const crawler::ImuState& initialImuState,
                 const crawler::VelocityCommand& initialCommand);
  bool step(const crawler::JointState& jointState,
            const crawler::ImuState& imuState,
            const crawler::VelocityCommand& command,
            crawler::PolicyResult& result);
  void reset();

  const float* latestObservation() const;
  crawler::FaultCode failureCode() const;

  static bool preprocessJointState(const crawler::JointState& measured,
                                   float position[3], float velocity[3]);
  static bool preprocessImuState(const crawler::ImuState& measured,
                                 float linearAcceleration[3],
                                 float angularVelocity[3]);
  static bool preprocessCommand(const crawler::VelocityCommand& command,
                                float& forward, float& lateral);

 private:
  void clearHistories();
  void fillInitialHistory(const float position[3], const float velocity[3],
                          const float linearAcceleration[3],
                          const float angularVelocity[3], float forward,
                          float lateral);
  void pushPosition(const float position[3]);
  void pushVelocity(const float velocity[3]);
  void pushLinearAcceleration(const float linearAcceleration[3]);
  void pushAngularVelocity(const float angularVelocity[3]);
  void pushCommand(float forward, float lateral);
  void pushAction(const float action[3]);
  void flattenObservation();

  float positionHistory_[5][3];
  float velocityHistory_[5][3];
  float linearAccelerationHistory_[5][3];
  float angularVelocityHistory_[5][3];
  float actionHistory_[5][3];
  float commandHistory_[5][2];
  float previousFilteredTargetRad_[3];
  float observation_[85];
  alignas(16) float modelInput_[1][85];
  alignas(16) float modelOutput_[1][3];
  bool loadAttempted_;
  bool loaded_;
  bool initialized_;
  crawler::FaultCode failureCode_;
};
