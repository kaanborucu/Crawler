#include <cmath>
#include <cstdio>

#include "policy_pipeline.h"
#include "student_policy.h"

namespace pipeline_tests {

void expect(bool condition, const char* message, int& failures) {
  if (!condition) {
    std::fprintf(stderr, "pipeline: %s\n", message);
    ++failures;
  }
}

bool near(float left, float right, float tolerance = 1.0e-5f) {
  return std::fabs(left - right) <= tolerance;
}

}  // namespace pipeline_tests

int runPipelineTests() {
  using namespace pipeline_tests;
  int failures = 0;

  crawler::JointState initial = {};
  initial.valid = true;
  initial.timestampMs = 100;
  initial.positionRad[0] = 0.1f;
  initial.positionRad[1] = -0.2f;
  initial.positionRad[2] = 0.3f;
  initial.velocityRadPerSecond[0] = 1.0f;
  initial.velocityRadPerSecond[1] = -2.0f;
  initial.velocityRadPerSecond[2] = 3.0f;

  crawler::VelocityCommand initialCommand = {};
  initialCommand.valid = true;
  initialCommand.forwardMetersPerSecond = 0.2f;
  initialCommand.lateralMetersPerSecond = -0.3f;
  initialCommand.sequence = 1;
  initialCommand.receivedAtMs = 100;

  PolicyPipeline pipeline;
  expect(pipeline.begin(), "generated policy did not load", failures);
  expect(pipeline.initialize(initial, initialCommand),
         "pipeline initialization failed", failures);

  float position[3] = {};
  float velocity[3] = {};
  expect(PolicyPipeline::preprocessJointState(initial, position, velocity),
         "joint preprocessing rejected valid input", failures);
  expect(near(position[0], 0.1f) && near(position[1], -0.2f) &&
             near(position[2], 0.3f),
         "joint position preprocessing changed the expected values", failures);
  expect(near(velocity[0], 0.1f) && near(velocity[1], -0.2f) &&
             near(velocity[2], 0.3f),
         "velocity scaling is incorrect", failures);

  crawler::JointState current = initial;
  current.timestampMs = 120;
  current.positionRad[0] = 0.4f;
  current.positionRad[1] = -0.5f;
  current.positionRad[2] = 0.6f;
  current.velocityRadPerSecond[0] = 4.0f;
  current.velocityRadPerSecond[1] = -5.0f;
  current.velocityRadPerSecond[2] = 6.0f;
  crawler::VelocityCommand command = initialCommand;
  command.forwardMetersPerSecond = 1.9f;
  command.lateralMetersPerSecond = -1.8f;
  command.sequence = 2;

  crawler::PolicyResult result = {};
  expect(pipeline.step(current, command, result), "pipeline step failed", failures);
  const float* observation = pipeline.latestObservation();
  expect(near(observation[0], 0.1f) && near(observation[12], 0.4f) &&
             near(observation[15], 0.1f) && near(observation[27], 0.4f),
         "position or velocity history order is incorrect", failures);
  expect(near(observation[30], 0.0f) && near(observation[44], 0.0f),
         "current action was inserted before inference", failures);
  expect(near(observation[45], 0.2f) && near(observation[46], -0.3f) &&
             near(observation[51], 0.2f) && near(observation[52], -0.3f) &&
             near(observation[53], 1.5f) && near(observation[54], -1.5f),
         "command history order or clamp is incorrect", failures);
  expect(near(observation[1], -0.2f) && near(observation[2], 0.3f) &&
             near(observation[13], -0.5f) && near(observation[14], 0.6f),
         "joint ordering is incorrect", failures);
  expect(result.valid && result.clampedActions[0] >= -1.0f &&
             result.clampedActions[0] <= 1.0f,
         "action clamp or result validity is incorrect", failures);

  crawler::JointState next = current;
  next.positionRad[0] = 0.7f;
  next.positionRad[1] = -0.8f;
  next.positionRad[2] = 0.9f;
  next.timestampMs = 140;
  expect(pipeline.step(next, command, result), "repeated pipeline step failed",
         failures);
  observation = pipeline.latestObservation();
  expect(near(observation[9], 0.4f) && near(observation[10], -0.5f) &&
             near(observation[11], 0.6f) && near(observation[12], 0.7f),
         "repeated history shift is incorrect", failures);

  crawler::JointState extreme = initial;
  extreme.positionRad[0] = 10.0f;
  extreme.velocityRadPerSecond[0] = 100.0f;
  expect(PolicyPipeline::preprocessJointState(extreme, position, velocity),
         "extreme finite joint input was rejected", failures);
  expect(near(position[0], crawler::config::policy::positionClampRad) &&
             near(velocity[0], 2.0f),
         "position or velocity clamp is incorrect", failures);
  float forward = 0.0f;
  float lateral = 0.0f;
  expect(PolicyPipeline::preprocessCommand(command, forward, lateral) &&
             near(forward, 1.5f) && near(lateral, -1.5f),
         "command preprocessing clamp is incorrect", failures);

  PolicyPipeline zeroPipeline;
  crawler::JointState zeroJoint = {};
  zeroJoint.valid = true;
  crawler::VelocityCommand zeroCommand = {};
  zeroCommand.valid = true;
  expect(zeroPipeline.initialize(zeroJoint, zeroCommand),
         "zero pipeline initialization failed", failures);
  crawler::PolicyResult zeroResult = {};
  expect(zeroPipeline.step(zeroJoint, zeroCommand, zeroResult),
         "zero policy step failed", failures);
  expect(near(zeroResult.rawActions[0], -68.1285858f, 1.0e-3f) &&
             near(zeroResult.rawActions[1], 0.7154212f, 1.0e-3f) &&
             near(zeroResult.rawActions[2], -45.0719681f, 1.0e-3f),
         "generated policy output no longer matches the zero reference",
         failures);
  expect(near(zeroResult.filteredTargetRad[0],
              -crawler::config::policy::filterNewWeight *
                  crawler::config::policy::actionScaleRad) &&
             near(zeroResult.filteredTargetRad[1],
                  crawler::config::policy::filterNewWeight * 0.7154212f *
                      crawler::config::policy::actionScaleRad),
         "action scaling or first-step filtering is incorrect", failures);

  zeroPipeline.reset();
  expect(near(zeroPipeline.latestObservation()[54], 0.0f),
         "pipeline reset did not clear the observation", failures);
  expect(!zeroPipeline.step(zeroJoint, zeroCommand, zeroResult),
         "pipeline ran after reset without reinitialization", failures);

  crawler::JointState invalidJoint = zeroJoint;
  invalidJoint.positionRad[1] = NAN;
  expect(!PolicyPipeline::preprocessJointState(invalidJoint, position, velocity),
         "non-finite sensor input was accepted", failures);
  return failures;
}
