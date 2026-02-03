#include "VoiceManager.h"
#include "PerformVoice.h"

void VoiceManager::Init(PerformVoice* voices, int count)
{
	voices_ = voices;
	count_ = count;
	tick_ = 0;
	steals_ = 0;
	active_ = 0;
}

int VoiceManager::SelectVoiceIndex(int32_t note)
{
	if (!voices_ || count_ <= 0)
	{
		return -1;
	}
	// Prefer same-note voice.
	for (int i = 0; i < count_; ++i)
	{
		if (voices_[i].active && voices_[i].note == note)
		{
			return i;
		}
	}
	// Prefer free voice.
	for (int i = 0; i < count_; ++i)
	{
		if (!voices_[i].active)
		{
			return i;
		}
	}
	// Steal oldest.
	int oldest = 0;
	uint32_t oldest_tick = voices_[0].start_tick;
	for (int i = 1; i < count_; ++i)
	{
		if (voices_[i].start_tick < oldest_tick)
		{
			oldest_tick = voices_[i].start_tick;
			oldest = i;
		}
	}
	++steals_;
	return oldest;
}

void VoiceManager::NoteOff(int32_t note)
{
	if (!voices_ || count_ <= 0)
	{
		return;
	}
	for (int i = 0; i < count_; ++i)
	{
		if (voices_[i].active && voices_[i].note == note)
		{
			voices_[i].releasing = true;
		}
	}
}
