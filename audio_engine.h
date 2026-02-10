#pragma once

#include <cstddef>
#include <cstdint>
#include "daisy_pod.h"
#include "shared_messages.h"

struct AudioShared
{
	daisy::DaisyPod* hw = nullptr;
	volatile uint32_t* audio_cmd = nullptr;
	volatile uint32_t* audio_flags_bits = nullptr;
	volatile RecordInput* record_input = nullptr;
	volatile bool* playback_reverse_target = nullptr;
	volatile uint8_t* audio_params_pub_idx = nullptr;
	volatile uint8_t* rt_pub_idx = nullptr;
	volatile uint8_t* fx_chain_pub_idx = nullptr;
	volatile uint8_t* preview_pub_idx = nullptr;
	volatile float* phones_volume = nullptr;
	volatile bool* audio_recording_active = nullptr;
	volatile size_t* recorded_length_audio = nullptr;
	volatile size_t* record_pos = nullptr;
	bool* reset_voices_pending = nullptr;
	AudioParams* audio_params_buf = nullptr;
	volatile uint8_t* audio_params_active_idx = nullptr;
	AudioUiState* audio_ui_state_buf = nullptr;
	volatile uint8_t* audio_ui_state_idx = nullptr;
	volatile bool* playback_active = nullptr;
	volatile float* playback_phase = nullptr;
	volatile bool* perform_voices_active = nullptr;
	volatile bool* preview_active = nullptr;
	volatile size_t* preview_read_index = nullptr;
	volatile float* preview_read_frac = nullptr;
	volatile uint32_t* preview_fade_samples_left = nullptr;
	volatile uint32_t* preview_fade_samples_total = nullptr;
	PreviewControl* preview_ctl_buf = nullptr;
	volatile uint8_t* preview_active_idx = nullptr;
	volatile size_t* preview_write_index = nullptr;
	SampleRuntime* rt_buf = nullptr;
	volatile uint8_t* rt_active_idx = nullptr;
	FxChainRuntime* fx_chain_buf = nullptr;
	volatile uint8_t* fx_chain_active_idx = nullptr;
	FxChainRuntime* fx_chain_audio = nullptr;
	bool* fx_chain_audio_valid = nullptr;
	FxParamsAudio* fx_params_buf = nullptr;
	volatile uint8_t* fx_params_idx = nullptr;
	AudioParamsAudio* audio_params_audio_buf = nullptr;
	volatile uint8_t* audio_params_audio_idx = nullptr;
	volatile float* delay_time_alpha = nullptr;
	volatile float* delay_param_alpha = nullptr;
#if STORAGE_SERVICE_PREVIEW_STREAM
	volatile bool* preview_preload_active = nullptr;
	volatile size_t* preview_preload_frames = nullptr;
	volatile uint8_t* preview_pp_ready = nullptr;
	volatile uint8_t* preview_pp_active = nullptr;
	volatile uint32_t* preview_pp_pos = nullptr;
	volatile uint32_t* preview_underrun_count = nullptr;
	volatile uint32_t* preview_rb_min_level = nullptr;
#endif
	volatile float* playback_rate = nullptr;
	volatile float* playback_amp = nullptr;
	volatile size_t* playback_env_samples = nullptr;
	volatile bool* playback_release_active = nullptr;
	volatile float* playback_release_pos = nullptr;
	volatile float* playback_release_start = nullptr;
	volatile bool* playback_reverse_active = nullptr;
	volatile bool* preview_hold = nullptr;
	volatile int32_t* preview_index = nullptr;
	volatile uint32_t* preview_sample_rate = nullptr;
	volatile uint16_t* preview_channels = nullptr;
	volatile int32_t* active_voice_count = nullptr;
	uint32_t* voice_skip_count = nullptr;
	uint32_t* voice_kill_count = nullptr;
	volatile float* cpu_load_pct = nullptr;
	volatile float* cpu_load_peak_pct = nullptr;
	volatile uint32_t* callback_cycles_last = nullptr;
	volatile uint32_t* callback_cycles_max = nullptr;
	volatile uint32_t* callback_overruns = nullptr;
	float* cpu_load_ema = nullptr;
	volatile bool* request_playhead_redraw = nullptr;
	MidiCmd* midi_cmd_q = nullptr;
	volatile uint8_t* midi_cmd_wr = nullptr;
	volatile uint8_t* midi_cmd_rd = nullptr;
	PlaybackCmd* playback_cmd_q = nullptr;
	volatile uint8_t* playback_cmd_wr = nullptr;
	volatile uint8_t* playback_cmd_rd = nullptr;
};

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

struct PreviewBuffers
{
	int16_t* buffer = nullptr;
	size_t frames = 0;
	volatile size_t* write_index = nullptr;
	volatile size_t* read_index = nullptr;
	int16_t* preload_buf = nullptr;
	size_t preload_frames = 0;
	int16_t* pp_buf_a = nullptr;
	int16_t* pp_buf_b = nullptr;
	size_t pp_frames = 0;
	volatile uint8_t* pp_ready_a = nullptr;
	volatile uint8_t* pp_ready_b = nullptr;
	volatile uint8_t* pp_active = nullptr;
};

class AudioEngine
{
public:
	void Init(daisy::DaisyPod& hw);
	void BindShared(AudioShared* shared);
	void InitVoices();
	bool AllocatePerformSample(size_t bytes, void** out_ptr);
	void FreePerformSample();
	void GetSampleBuffers(int16_t*& l, int16_t*& r) const;
	void GetSampleMemUsage(size_t& used, size_t& free_bytes) const;
	void GetPreviewBuffers(PreviewBuffers& out) const;
	void Process(daisy::AudioHandle::InputBuffer in, daisy::AudioHandle::OutputBuffer out, size_t size);
	const AudioEngineStats& GetStats() const { return stats_; }

private:
	AudioEngineStats stats_;
};
