#pragma once

#include <cstddef>
#include <cstdint>
#include "daisy_pod.h"

struct AudioEngineStats
{
	uint32_t active_voices = 0;
	uint32_t voice_skips = 0;
	uint32_t voice_kills = 0;
	uint32_t preview_underruns = 0;
	float cpu_load_pct = 0.0f;
	float cpu_load_peak_pct = 0.0f;
	uint32_t callback_cycles_last = 0;
	uint32_t callback_cycles_max = 0;
	uint32_t callback_overruns = 0;
};

class AudioEngine
{
public:
	void Init(daisy::DaisyPod& hw);
	void Process(daisy::AudioHandle::InputBuffer in, daisy::AudioHandle::OutputBuffer out, size_t size);
	const AudioEngineStats& GetStats() const { return stats_; }

private:
	AudioEngineStats stats_;
};
