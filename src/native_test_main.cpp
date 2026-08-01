#include <cstdio>

int runPipelineTests();
int runSafetyTests();
int runServoMappingTests();
int runWebSocketLifecycleTests();

#include "../test/test_pipeline.cpp"
#include "../test/test_safety.cpp"
#include "../test/test_servo_mapping.cpp"
#include "../test/test_websocket_lifecycle.cpp"

int main() {
  const int failures = runPipelineTests() + runSafetyTests() +
                       runServoMappingTests() + runWebSocketLifecycleTests();
  if (failures != 0) {
    std::fprintf(stderr, "%d native test failure(s)\n", failures);
    return 1;
  }
  std::printf("native tests passed: pipeline, safety, servo mapping, "
              "websocket lifecycle\n");
  return 0;
}
