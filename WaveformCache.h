#pragma once

#include <cstddef>
#include <cstdint>
#include "SamplerConfig.h"

class WaveformCache
{
public:
	WaveformCache();

	int16_t* Min() { return min_; }
	int16_t* Max() { return max_; }

	bool Ready() const { return ready_; }
	bool Dirty() const { return dirty_; }
	void SetReady(bool v) { ready_ = v; }
	void SetDirty(bool v) { dirty_ = v; }

	size_t BytesUsed() const { return kWaveformCacheBytes; }

private:
	int16_t min_[128] = {};
	int16_t max_[128] = {};
	bool ready_ = false;
	bool dirty_ = false;
};
