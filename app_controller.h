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
	uint32_t* last_draw_ms = nullptr;
	uint32_t* ui_tick_ms = nullptr;
	uint32_t* ui_tick_playback_ms = nullptr;

	// Control deltas/events
	volatile int32_t* enc_l_delta = nullptr;
	volatile int32_t* enc_r_delta = nullptr;
	volatile uint32_t* ctrl_events = nullptr;
	volatile bool* shift_held = nullptr;
	volatile bool* btn1_held = nullptr;
	bool* ui_button1_held = nullptr;

	// UI / app state
	volatile UiMode* ui_mode = nullptr;
	volatile int32_t* menu_index = nullptr;
	volatile int32_t* shift_menu_index = nullptr;
	volatile int32_t* perform_index = nullptr;
	volatile int32_t* amp_fader_index = nullptr;
	volatile int32_t* flt_fader_index = nullptr;
	volatile int32_t* fx_fader_index = nullptr;
	volatile int32_t* fx_chain_order = nullptr;
	volatile bool* fx_window_active = nullptr;
	volatile bool* amp_window_active = nullptr;
	volatile bool* flt_window_active = nullptr;
	volatile UiMode* shift_prev_mode = nullptr;
	volatile RecordInput* record_input = nullptr;
	volatile int32_t* load_selected = nullptr;
	volatile int32_t* load_scroll = nullptr;
	volatile bool* request_load_scan = nullptr;
	volatile bool* list_build_pending = nullptr;
	volatile bool* request_load_sample = nullptr;
	volatile int32_t* request_load_index = nullptr;
	bool* load_in_progress = nullptr;
	uint16_t* load_cookie_next = nullptr;
	uint16_t* load_cookie_active = nullptr;
	bool* load_target_is_edt = nullptr;
	LoadContext* load_context = nullptr;
	LoaderState* loader_state = nullptr;
	volatile uint32_t* load_success_count = nullptr;
	volatile uint32_t* load_fail_budget_count = nullptr;
	volatile uint32_t* load_fail_io_count = nullptr;
	volatile int32_t* wav_file_count = nullptr;
	bool* sd_mounted = nullptr;
	bool* sd_present = nullptr;
	bool* sd_fault = nullptr;
	const char** sd_fault_text = nullptr;
	uint32_t* sd_retries_remaining = nullptr;
	bool* sd_init_in_progress = nullptr;
	bool* sd_init_done = nullptr;
	bool* sd_init_success = nullptr;
	uint32_t* sd_init_start_ms = nullptr;
	uint32_t* sd_init_next_ms = nullptr;
	uint32_t* sd_init_result_until_ms = nullptr;
	uint32_t* sd_init_draw_next_ms = nullptr;
	int32_t* sd_init_attempts = nullptr;
	UiMode* sd_init_prev_mode = nullptr;
	bool* save_in_progress = nullptr;
	bool* save_done = nullptr;
	bool* save_success = nullptr;
	bool* save_started = nullptr;
	uint32_t* save_start_ms = nullptr;
	uint32_t* save_result_until_ms = nullptr;
	uint32_t* save_draw_next_ms = nullptr;
	UiMode* save_prev_mode = nullptr;
	char* save_filename = nullptr;
	size_t* save_frames_written = nullptr;
	volatile bool* delete_mode = nullptr;
	UiMode* delete_prev_mode = nullptr;
	UiMode* load_prev_mode = nullptr;
	UiMode* fx_detail_prev_mode = nullptr;
	UiMode* edt_prev_mode = nullptr;
	volatile bool* request_delete_scan = nullptr;
	volatile bool* request_delete_file = nullptr;
	volatile int32_t* request_delete_index = nullptr;
	volatile bool* delete_confirm = nullptr;
	char* delete_confirm_name = nullptr;
	volatile float* phones_volume = nullptr;
	bool* delete_in_progress = nullptr;
	uint16_t* delete_cookie_next = nullptr;
	uint16_t* delete_cookie_active = nullptr;
	char (*wav_files)[32] = nullptr;
	char* loaded_sample_name = nullptr;
	int32_t* load_lines = nullptr;
	int32_t* load_line_height = nullptr;
	int32_t* load_chars_per_line = nullptr;
	uint32_t* load_scan_start_ms = nullptr;
	SampleContext* current_sample_context = nullptr;
	SampleContext* edt_sample_context = nullptr;
	int32_t* load_mode_index = nullptr;
	LoadStubMode* load_stub_mode = nullptr;
	volatile size_t* sample_length = nullptr;
	volatile size_t* sample_play_start = nullptr;
	volatile size_t* sample_play_end = nullptr;
	volatile uint32_t* sample_rate = nullptr;
	volatile uint16_t* sample_channels = nullptr;
	volatile bool* sample_loaded = nullptr;
	volatile bool* playback_active = nullptr;
	volatile float* playback_rate = nullptr;
	volatile float* playback_phase = nullptr;
	volatile float* playback_reverse = nullptr;
	volatile bool* perform_voices_active = nullptr;
	volatile uint32_t* audio_cmd = nullptr;
	volatile uint32_t* audio_flags_bits = nullptr;
	AudioUiState* audio_ui_state_buf = nullptr;
	volatile uint8_t* audio_ui_state_idx = nullptr;
	volatile bool* audio_recording_active = nullptr;
	volatile int32_t* active_voice_count = nullptr;
	volatile float* delay_time_alpha = nullptr;
	volatile float* delay_param_alpha = nullptr;
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
	volatile float* reverb_wet = nullptr;
	volatile float* reverb_pre = nullptr;
	volatile float* reverb_damp = nullptr;
	volatile float* reverb_decay = nullptr;
	volatile float* delay_wet = nullptr;
	volatile float* delay_time = nullptr;
	volatile float* delay_feedback = nullptr;
	volatile float* delay_spread = nullptr;
	volatile float* delay_freeze = nullptr;
	volatile float* fx_s_wet = nullptr;
	volatile float* sat_drive = nullptr;
	volatile float* sat_tape_bump = nullptr;
	volatile float* sat_bit_reso = nullptr;
	volatile float* sat_bit_smpl = nullptr;
	volatile float* fx_c_wet = nullptr;
	volatile float* mod_depth = nullptr;
	volatile float* chorus_rate = nullptr;
	volatile int32_t* sat_mode = nullptr;
	volatile int32_t* chorus_mode = nullptr;
	volatile float* chorus_wow = nullptr;
	volatile float* tape_rate = nullptr;
	volatile bool* fx_params_dirty = nullptr;
	volatile bool* audio_params_dirty = nullptr;
	bool* sat_params_initialized = nullptr;
	bool* reverb_params_initialized = nullptr;
	bool* delay_params_initialized = nullptr;
	bool* mod_params_initialized = nullptr;
	volatile float* amp_attack = nullptr;
	volatile float* amp_decay = nullptr;
	volatile float* amp_sustain = nullptr;
	volatile float* amp_release = nullptr;
	volatile int32_t* fx_detail_index = nullptr;
	volatile int32_t* fx_detail_param_index = nullptr;
	volatile float* flt_cutoff = nullptr;
	volatile float* flt_res = nullptr;
	volatile bool* preview_hold = nullptr;
	float* fx_chain_fade_gain = nullptr;
	float* fx_chain_fade_target = nullptr;
	int32_t* fx_chain_fade_samples_left = nullptr;
	bool* fx_chain_pause_pending = nullptr;
	bool* fx_chain_paused = nullptr;
	uint32_t* fx_chain_last_move_ms = nullptr;
	volatile bool* preview_active = nullptr;
	volatile int32_t* preview_index = nullptr;
	volatile uint32_t* preview_sample_rate = nullptr;
	volatile uint16_t* preview_channels = nullptr;
	volatile float* preview_rate = nullptr;
	volatile float* preview_read_frac = nullptr;
	volatile size_t* preview_read_index = nullptr;
	volatile size_t* preview_write_index = nullptr;
	volatile uint32_t* preview_data_offset = nullptr;
	volatile uint32_t* preview_fade_samples_left = nullptr;
	volatile uint32_t* preview_fade_samples_total = nullptr;
	bool* preview_pending_start = nullptr;
	uint32_t* preview_pending_start_ms = nullptr;
	volatile uint32_t* preview_underrun_count = nullptr;
	volatile uint32_t* preview_rb_min_level = nullptr;
	uint16_t* preview_stream_cookie = nullptr;
	uint16_t* preview_stream_cookie_active = nullptr;
	volatile uint8_t* preview_pp_ready = nullptr;
	volatile uint8_t* preview_pp_active = nullptr;
	volatile uint32_t* preview_pp_pos = nullptr;
	volatile size_t* preview_preload_frames = nullptr;
	volatile bool* preview_preload_active = nullptr;
	float* led1_level = nullptr;
	float* led1_phase_ms = nullptr;
	double* record_anim_start_ms = nullptr;
	bool* request_shift_redraw = nullptr;
	bool* request_perform_redraw = nullptr;
	bool* request_fx_detail_redraw = nullptr;
	uint32_t* delay_snow_next_ms = nullptr;
	uint32_t* midi_ignore_until_ms = nullptr;
	Job* g_job = nullptr;
	WaveformJob* g_wf_job = nullptr;
	FileListJob* g_list_job = nullptr;
	bool* waveform_ready = nullptr;
	bool* waveform_dirty = nullptr;
	bool* waveform_from_recording = nullptr;
	const char** waveform_title = nullptr;
	bool* request_delete_redraw = nullptr;
	bool* display_update_pending = nullptr;
	volatile bool* waveform_compute_pending = nullptr;
	volatile SampleContext* waveform_compute_ctx = nullptr;
	volatile uint32_t* audio_event_bits = nullptr;
	volatile size_t* sample_mem_used_bytes = nullptr;
	volatile size_t* sample_mem_free_bytes = nullptr;
	volatile size_t* waveform_cache_bytes = nullptr;

	// Shared audio publish buffers
	AudioParams* audio_params_buf = nullptr;
	volatile uint8_t* audio_params_pub_idx = nullptr;
	volatile uint8_t* audio_params_active_idx = nullptr;
	SampleRuntime* rt_buf = nullptr;
	volatile uint8_t* rt_pub_idx = nullptr;
	volatile uint8_t* rt_active_idx = nullptr;
	FxChainRuntime* fx_chain_buf = nullptr;
	volatile uint8_t* fx_chain_pub_idx = nullptr;
	volatile uint8_t* fx_chain_active_idx = nullptr;
	PreviewControl* preview_ctl_buf = nullptr;
	volatile uint8_t* preview_pub_idx = nullptr;
	volatile uint8_t* preview_active_idx = nullptr;
	FxParamsAudio* fx_params_buf = nullptr;
	volatile uint8_t* fx_params_idx = nullptr;
	AudioParamsAudio* audio_params_audio_buf = nullptr;
	volatile uint8_t* audio_params_audio_idx = nullptr;

	// Recording UI masks
	uint8_t (*record_text_mask)[128] = nullptr;
	uint8_t (*record_invert_mask)[128] = nullptr;
	uint8_t (*record_fb_buf)[128] = nullptr;
	uint8_t (*record_bold_mask)[128] = nullptr;

	// Trim helpers
	float* trim_start = nullptr;
	float* trim_end = nullptr;
	uint32_t* snap_start_frame = nullptr;
	uint32_t* snap_end_frame = nullptr;

	// Perform envelope caches
	volatile float* perform_attack_norm = nullptr;
	volatile float* perform_release_norm = nullptr;
};

class AppController
{
public:
	void Init(AppContext* ctx);
	void Tick(uint32_t now_ms);
};

const AudioUiState& GetAudioUiStateSnapshot(uint8_t& idx);
void RequestAudioCmd(uint32_t bits);
void RequestPlaybackStart(uint8_t note, bool apply_pitch);
void RequestPlaybackStop(uint8_t note, bool apply_release);
void RequestPlaybackStopAll();
void StartPerformVoice(int32_t note);
void MountSd();
void ApplyLoadedSampleFade(size_t length, uint32_t rate);
double NowMs();
const char* SdFaultText(StorageService::SdErrorCode code);

bool AllocatePerformSample(size_t bytes, void** out_ptr);
void FreePerformSample();
