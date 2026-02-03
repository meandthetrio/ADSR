#pragma once

#include <cstddef>
#include <cstdint>

struct PerformVoice;

class VoiceManager
{
public:
	void Init(PerformVoice* voices, int count);

	int SelectVoiceIndex(int32_t note);
	void NoteOff(int32_t note);

	uint32_t StealCount() const { return steals_; }
	uint32_t ActiveCount() const { return active_; }

private:
	PerformVoice* voices_ = nullptr;
	int count_ = 0;
	uint32_t tick_ = 0;
	uint32_t steals_ = 0;
	uint32_t active_ = 0;
};
