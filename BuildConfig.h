#pragma once

// Build configuration flags (set by Makefile).
// DEBUG_BUILD or RELEASE_BUILD should be defined by the build system.

#ifndef DEBUG_BUILD
#define DEBUG_BUILD 0
#endif

#ifndef RELEASE_BUILD
#define RELEASE_BUILD 0
#endif

#ifndef PERF
#define PERF 0
#endif

#if DEBUG_BUILD
#define ENABLE_DEBUG_UI 1
#define ENABLE_SERIAL_LOG 1
#else
#define ENABLE_DEBUG_UI 0
#define ENABLE_SERIAL_LOG 0
#endif

#if PERF
#define ENABLE_PERF_COUNTERS 1
#else
#define ENABLE_PERF_COUNTERS 0
#endif

#if ENABLE_SERIAL_LOG
#include <cstdio>
#define LOGF(...) std::printf(__VA_ARGS__)
#define DBG_PRINTF(...) std::printf(__VA_ARGS__)
#else
#define LOGF(...) do {} while (0)
#define DBG_PRINTF(...) do {} while (0)
#endif
