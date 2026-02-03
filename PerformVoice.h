#pragma once

#include <cstddef>
#include <cstdint>

struct PerformVoice
{
	bool active = false;
	bool releasing = false;
	bool sample_acquired = false;
	float phase = 0.0f;
	float rate = 1.0f;
	float amp = 1.0f;
	float env = 0.0f;
	float release_start = 0.0f;
	float release_pos = 0.0f;
	int32_t note = -1;
	int32_t track = -1;
	size_t offset = 0;
	size_t length = 0;
	uint32_t env_samples = 0;
	uint32_t start_tick = 0;
};
