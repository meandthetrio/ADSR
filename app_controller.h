#pragma once

#include <cstddef>
#include <cstdint>
#include "shared_messages.h"
#include "StorageService.h"

namespace daisy { class DaisyPod; }
class Ui;
class AudioEngine;
class StorageService;

struct AppContext
{
	daisy::DaisyPod* hw = nullptr;
	void* display = nullptr;
	Ui* ui = nullptr;
	AudioEngine* audio = nullptr;
	StorageService* storage = nullptr;

	// Control deltas/events

	// UI / app state
	volatile bool* perform_voices_active = nullptr;
	AudioUiState* audio_ui_state_buf = nullptr;
	volatile uint8_t* audio_ui_state_idx = nullptr;
	volatile RecordState* record_state = nullptr;
	volatile int32_t* record_source_index = nullptr;
	volatile int32_t* record_target_index = nullptr;
	volatile uint32_t* record_countdown_start_ms = nullptr;
	volatile size_t* record_pos = nullptr;
	volatile size_t* recorded_length_audio = nullptr;
	volatile uint32_t* record_start_ms = nullptr;
	volatile bool* record_waveform_pending = nullptr;
	volatile int32_t* encoder_r_accum = nullptr;
	volatile bool* encoder_r_button_press = nullptr;
	volatile bool* request_length_redraw = nullptr;
	volatile bool* request_playhead_redraw = nullptr;
	volatile bool* button1_press = nullptr;
	volatile bool* button2_press = nullptr;
	volatile bool* request_playback_stop_log = nullptr;
	double* record_anim_start_ms = nullptr;
	bool* request_delete_redraw = nullptr;
	volatile size_t* sample_mem_used_bytes = nullptr;
	volatile size_t* sample_mem_free_bytes = nullptr;

	// Trim helpers
};

class AppController
{
public:
	void Init(AppContext* ctx);
	void Tick(uint32_t now_ms);
};

const AudioUiState& GetAudioUiStateSnapshot(uint8_t& idx);
void StartPerformVoice(int32_t note);
void MountSd();
void ApplyLoadedSampleFade(size_t length, uint32_t rate);
double NowMs();
const char* SdFaultText(StorageService::SdErrorCode code);

bool AllocatePerformSample(size_t bytes, void** out_ptr);
void FreePerformSample();
