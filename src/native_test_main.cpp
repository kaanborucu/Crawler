#include <cstdio>

int runPipelineTests();
int runSafetyTests();
int runServoMappingTests();

#include "../test/test_pipeline.cpp"
#include "../test/test_safety.cpp"
#include "../test/test_servo_mapping.cpp"

int main() {
  const int failures = runPipelineTests() + runSafetyTests() +
                       runServoMappingTests();
  if (failures != 0) {
    std::fprintf(stderr, "%d native test failure(s)\n", failures);
    return 1;
  }
  std::printf("native tests passed: pipeline, safety, servo mapping\n");
  return 0;
}
