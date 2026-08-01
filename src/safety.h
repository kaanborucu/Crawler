#pragma once

#include "crawler_config.h"
#include "crawler_types.h"

class Safety {
 public:
  Safety();

  void begin();
  crawler::SafetyDecision evaluate(const crawler::VelocityCommand& command,
                                   const crawler::JointState& joints,
                                   const crawler::ImuState& imu,
                                   bool bleConnected,
                                   bool calibrationValid);
  void raiseFault(crawler::FaultCode fault);

  crawler::RobotState state() const;
  crawler::FaultCode fault() const;
  bool faultActive() const;

 private:
  uint32_t nowMs() const;
  bool jointsNearDefault(const crawler::JointState& joints) const;
  void setFault(crawler::FaultCode fault);

  crawler::RobotState state_;
  crawler::FaultCode fault_;
  bool requireNextEnable_;
  uint16_t clearedFaultSequence_;
  bool requireNewSequence_;
};
