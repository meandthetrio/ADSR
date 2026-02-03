#pragma once

#include <cstddef>
#include <cstdint>

enum class SampleIOPolicy
{
	PreloadToRam,
	StreamFromSd,
};

constexpr SampleIOPolicy kPolicy = SampleIOPolicy::PreloadToRam;

constexpr uint32_t kSampleRateHz = 48000;
constexpr uint32_t kMaxSampleSeconds = 5;
constexpr size_t kMaxSampleFrames =
	static_cast<size_t>(kSampleRateHz) * static_cast<size_t>(kMaxSampleSeconds);

constexpr int kMaxVoices = 5;
constexpr int kMaxLoadedPerformSamples = 1;

constexpr size_t kPerformSampleRamBudgetBytes = kMaxSampleFrames * sizeof(int16_t) * 2;

constexpr size_t kPreviewBufferFrames = 16384;
constexpr size_t kPreviewBufferBytes = kPreviewBufferFrames * sizeof(int16_t);
constexpr size_t kPreviewPreloadFrames = kMaxSampleFrames;
constexpr size_t kPreviewPreloadBytes = kPreviewPreloadFrames * sizeof(int16_t);

constexpr size_t kRecordBufferFrames = kMaxSampleFrames;
constexpr size_t kRecordBufferBytes = kRecordBufferFrames * sizeof(int16_t) * 2;

constexpr size_t kWaveformCacheBytes = 128 * sizeof(int16_t) * 2;

static_assert(kPreviewPreloadFrames <= kMaxSampleFrames, "Preview preload exceeds max sample frames");
static_assert(kRecordBufferFrames == kMaxSampleFrames, "Record buffer must match max sample frames");
