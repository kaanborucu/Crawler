#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Public interface generated from student_policy.c.
 * The generated C declaration uses _Bool; C++ uses its equivalent bool.
 */
#ifdef __cplusplus
bool student_policy_load(const char *path);
void student_policy(const float obs[1][85], float actions[1][3]);
#else
#include <stdbool.h>
_Bool student_policy_load(const char *path);
void student_policy(const float obs[restrict 1][85], float actions[restrict 1][3]);
#endif

#ifdef __cplusplus
}
#endif
