#include "ui.h"

#include "daisy_pod.h"
#include "dev/oled_ssd130x.h"
#include "shared_messages.h"
#include "SamplerConfig.h"
#include "StorageService.h"
#include "WaveformCache.h"
#include "app_controller.h"
#include "audio_dsp.h"
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <initializer_list>
#include <cstring>

using namespace daisy;
using PodDisplay = OledDisplay<SSD130xI2c128x64Driver>;

#ifndef STORAGE_SERVICE_PREVIEW_STREAM
#define STORAGE_SERVICE_PREVIEW_STREAM PREVIEW_STREAM_FROM_SD
#endif

enum CtrlEventBits : uint32_t {
	kEncLPress   = 1U << 0,
	kEncRPress   = 1U << 1,
	kBtn1Press   = 1U << 2,
	kBtn2Press   = 1U << 3,
	kShiftRise   = 1U << 4,
	kShiftFall   = 1U << 5,
};

constexpr int32_t kShiftMenuCount = 3;
constexpr int32_t kMenuCount = 3;
constexpr int32_t kPerformBoxCount = 4;
constexpr int32_t kRecordTargetCount = 2;
constexpr int32_t kRecordTargetSave = 0;
constexpr int32_t kPerformFaderCount = 4;
constexpr int32_t kPerformVoiceCount = kMaxVoices;
constexpr int32_t kPerformFltFaderCount = 2;
constexpr int32_t kPerformEdtIndex = 0;
constexpr int32_t kPerformAmpIndex = 1;
constexpr int32_t kPerformFltIndex = 2;
constexpr int32_t kPerformFxIndex = 3;
constexpr int32_t kFxSatIndex = 0;
constexpr int32_t kFxChorusIndex = 1;
constexpr int32_t kFxDelayIndex = 2;
constexpr int32_t kFxReverbIndex = 3;
constexpr int32_t kPerformEncoderScale = 4;
constexpr uint32_t kFxChainIdleMs = 300;
constexpr uint32_t kDrawIntervalMs = 33;
constexpr float kFxChainFadeMs = 20.0f;
constexpr float kFxParamStep = 0.02f;
constexpr float kAmpEnvStep = 0.02f;
constexpr float kFltParamStep = 0.02f;
constexpr float kPhonesVolumeStep = 0.01f;
constexpr uint32_t kRecordCountdownMs = 4000;
constexpr bool kLoadPresetsPlaceholder = true;
constexpr int kDisplayW = 128;
constexpr int kDisplayH = 64;
constexpr int32_t kLoadFontScale = 1;
constexpr int32_t kMaxWavNameLen = 32;
constexpr int32_t kMaxWavFiles = 32;
constexpr size_t kPreviewPpFrames = 2048;
constexpr float kSampleScale = 1.0f / 32768.0f;
constexpr size_t kSaveChunkFrames = 8192;
constexpr int32_t kDelayFaderCount = 5;
constexpr int32_t kReverbFaderCount = 5;
constexpr float kDelayParamStep = 0.02f;
constexpr float kDelayWetStep = 0.02f;
constexpr float kReverbParamStep = 0.02f;
constexpr float kReverbWetStep = 0.02f;
constexpr float kChorusRateStep = 0.02f;
constexpr int32_t kBitResoStepCount = 3;
constexpr const char* kBitResoLabels[kBitResoStepCount] = {"CRUSH", "STATIC", "HISS"};
constexpr int kBitResoSteps[kBitResoStepCount] = {2, 3, 4};
constexpr uint32_t kFxMapIntervalMs = 10;
constexpr uint32_t kLoadScanGraceMs = 300;
constexpr uint32_t kSdInitMinMs = 800;
constexpr uint32_t kSdInitRetryMs = 300;
constexpr uint32_t kSdInitResultMs = 1500;
constexpr int32_t kSdInitAttempts = 3;
constexpr float kDelayTimeMinMs = 50.0f;
constexpr float kDelayTimeMaxMs = 2000.0f;
constexpr float kDelayTimeSlewMs = 180.0f;
constexpr float kDelayParamSlewMs = 120.0f;
constexpr size_t kDelayMaxSamples = 96000;
constexpr float kDelayFeedbackMax = 0.98f;
constexpr float kChorusRateMinHz = 0.05f;
constexpr float kChorusRateMaxHz = 0.9f;
constexpr float kChorusMaxDepth = 3.0f;
constexpr float kReverbFeedback = 0.85f;
constexpr float kReverbDampMinHz = 800.0f;
constexpr float kReverbDampMaxHz = 20000.0f;
constexpr float kReverbPreDelayMaxMs = 1000.0f;
constexpr float kReverbDecayMinMs = 10.0f;
constexpr float kReverbDecayMaxMs = 4000.0f;
constexpr size_t kReverbPreDelayMaxSamples = 48000;
constexpr float kAmpEnvMinMs = 5.0f;
constexpr float kAmpEnvMaxMs = 5000.0f;
constexpr float kAmpEnvStepMs = 20.0f;
constexpr float kRecordWaveformScaleMinMic = 1.15f;
constexpr float kRecordWaveformScaleMaxMic = 1.75f;
constexpr float kRecordWaveformScaleMinLine = 0.7f;
constexpr float kRecordWaveformScaleMaxLine = 2.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr size_t kRecordMaxFrames = kMaxSampleFrames;
constexpr float kUiTickHz = 1000.0f;
constexpr uint32_t kPerformSampleId = 1;

struct SmoothParam
{
	float value = 0.0f;
	float target = 0.0f;
	float coeff = 1.0f;

	void Init(float initial, float time_ms, float tick_hz)
	{
		value = initial;
		target = initial;
		SetTime(time_ms, tick_hz);
	}

	void SetTime(float time_ms, float tick_hz)
	{
		if (time_ms <= 0.0f)
		{
			coeff = 1.0f;
			return;
		}
		const float dt = 1.0f / tick_hz;
		const float tau = time_ms / 1000.0f;
		coeff = 1.0f - expf(-dt / tau);
		if (coeff > 1.0f) coeff = 1.0f;
		if (coeff < 0.0f) coeff = 0.0f;
	}

	void SetTarget(float t) { target = t; }

	float Process()
	{
		value += coeff * (target - value);
		return value;
	}

	bool IsNearTarget(float eps = 1e-4f) const
	{
		return fabsf(target - value) <= eps;
	}
};


extern daisy::DaisyPod hw;
extern StorageService storage;

extern uint32_t kUiTickMs;
extern uint32_t kUiTickPlaybackMs;

extern volatile int32_t g_enc_l_delta;
extern volatile int32_t g_enc_r_delta;
extern volatile uint32_t g_ctrl_events;
extern volatile bool g_shift_held;
extern volatile bool g_btn1_held;
extern bool ui_button1_held;
extern volatile int32_t encoder_r_accum;
extern volatile bool encoder_r_button_press;
extern bool g_display_update_pending;
extern uint32_t g_last_draw_ms;
extern PodDisplay display;

extern void JobTick();

extern const AudioUiState& GetAudioUiStateSnapshot(uint8_t& idx);

extern float ClampF(float v, float min, float max);
extern int ClampI(int v, int min, int max);
extern float Clamp01(float v);

extern int32_t NextMenuIndex(int32_t current, int32_t delta);
extern int32_t NextPerformIndex(int32_t current, int32_t delta);
extern int32_t LoadVisibleLines();
extern void CopyString(char* dst, const char* src, size_t dst_len);
extern bool IsPerformUiMode(UiMode mode);
extern double NowMs();

extern void RequestPlaybackStopAll();
extern void RequestAudioCmd(uint32_t bits);
extern void PublishFxChainFromUi();
extern void PublishRuntimeFromUi();
extern void ResetSaveState();
extern void StartRecording();
void AdjustTrimNormalized(int32_t dl, int32_t dr, bool fine);
extern void ApplyPlaybackReverse(bool reverse);

extern volatile UiMode ui_mode;
extern volatile UiMode shift_prev_mode;
extern UiMode delete_prev_mode;
extern UiMode load_prev_mode;
extern UiMode fx_detail_prev_mode;
extern UiMode edt_prev_mode;
extern volatile int32_t menu_index;
extern volatile int32_t shift_menu_index;
extern volatile int32_t perform_index;
extern volatile int32_t amp_fader_index;
extern volatile int32_t flt_fader_index;
extern volatile int32_t fx_fader_index;
extern volatile int32_t fx_detail_index;
extern volatile int32_t fx_detail_param_index;
extern volatile int32_t load_selected;
extern volatile int32_t load_scroll;
extern volatile int32_t load_mode_index;
extern volatile int32_t wav_file_count;
extern volatile int32_t request_load_index;
extern volatile int32_t request_delete_index;
extern volatile int32_t record_source_index;
extern volatile int32_t record_target_index;
extern volatile int32_t sat_mode;
extern volatile int32_t chorus_mode;
extern volatile bool fx_window_active;
extern volatile bool amp_window_active;
extern volatile bool flt_window_active;
extern volatile bool preview_hold;
extern volatile bool fx_params_dirty;
extern volatile bool audio_params_dirty;
extern volatile bool request_load_sample;
extern volatile bool request_load_scan;
extern volatile bool request_delete_scan;
extern volatile bool request_delete_file;
extern bool request_delete_redraw;
extern bool request_shift_redraw;
extern bool request_perform_redraw;
extern bool request_fx_detail_redraw;
extern volatile bool request_length_redraw;
extern volatile bool button1_press;
extern volatile bool button2_press;
extern volatile bool sample_loaded;
extern volatile size_t sample_length;
extern volatile uint32_t record_countdown_start_ms;
extern volatile float phones_volume;
extern volatile float perform_attack_norm;
extern volatile float perform_release_norm;
extern volatile float sat_drive;
extern volatile float sat_tape_bump;
extern volatile float sat_bit_reso;
extern volatile float sat_bit_smpl;
extern volatile float fx_s_wet;
extern volatile float fx_c_wet;
extern volatile float mod_depth;
extern volatile float chorus_rate;
extern volatile float chorus_wow;
extern volatile float tape_rate;
extern volatile float delay_wet;
extern volatile float delay_time;
extern volatile float delay_feedback;
extern volatile float delay_spread;
extern volatile float delay_freeze;
extern volatile float reverb_wet;
extern volatile float reverb_pre;
extern volatile float reverb_damp;
extern volatile float reverb_decay;
extern volatile float playback_reverse;
extern volatile float amp_attack;
extern volatile float amp_decay;
extern volatile float amp_sustain;
extern volatile float amp_release;
extern volatile float flt_cutoff;
extern volatile float flt_res;

extern volatile bool sat_params_initialized;
extern volatile bool mod_params_initialized;
extern volatile bool delay_params_initialized;
extern volatile bool reverb_params_initialized;

extern bool save_in_progress;
extern bool save_done;
extern bool save_success;
extern bool save_started;
extern uint32_t save_start_ms;
extern uint32_t save_result_until_ms;
extern uint32_t save_draw_next_ms;
extern UiMode save_prev_mode;
extern char save_filename[];
extern bool sd_init_in_progress;
extern bool sd_mounted;
extern bool sd_fault;
extern const char* sd_fault_text;
extern bool sd_init_done;
extern bool sd_init_success;
extern int32_t sd_init_attempts;
extern volatile bool delete_mode;
extern volatile bool delete_confirm;
extern bool waveform_ready;
extern bool waveform_dirty;
extern bool waveform_from_recording;

extern LoadContext load_context;
extern LoadStubMode load_stub_mode;
extern volatile RecordState record_state;
extern volatile RecordInput record_input;
extern SampleContext current_sample_context;
extern SampleContext edt_sample_context;

extern uint32_t delay_snow_next_ms;
extern uint32_t midi_ignore_until_ms;

extern uint8_t record_text_mask[kDisplayH][kDisplayW];
extern uint8_t record_invert_mask[kDisplayH][kDisplayW];
extern uint8_t record_fb_buf[kDisplayH][kDisplayW];
extern uint8_t record_bold_mask[kDisplayH][kDisplayW];

extern int32_t load_lines;
extern int32_t load_line_height;
extern int32_t load_chars_per_line;

extern char wav_files[][kMaxWavNameLen];
extern char delete_confirm_name[];
extern char loaded_sample_name[];

extern volatile int32_t fx_chain_order[];
extern uint32_t fx_chain_last_move_ms;
extern float fx_chain_fade_target;
extern int32_t fx_chain_fade_samples_left;
extern float fx_chain_fade_gain;
extern bool fx_chain_paused;
extern bool fx_chain_pause_pending;
extern double record_anim_start_ms;

extern float FxWetStep(int32_t fx_index);
extern volatile float* FxWetTarget(int32_t fx_index);
extern int BitResoIndexFromValue(float value);
extern float BitResoValueFromIndex(int idx);
void PublishAudioParamsFromUi(const AudioParams& p);
float FxWetValue(int32_t fx_index);
static float AmpEnvMsFromFader(float value);
static float FltCutoffFromFader(float value, float sample_rate);
static float FltQFromFader(float value);
extern volatile uint32_t g_audio_cmd;
extern volatile uint8_t g_audio_params_pub_idx;
extern volatile uint8_t g_rt_pub_idx;
extern volatile uint8_t g_fx_chain_pub_idx;
extern volatile uint8_t g_preview_pub_idx;
extern AudioParams g_audio_params_buf[2];
extern SampleRuntime g_rt_buf[2];
extern FxChainRuntime g_fx_chain_buf[2];
extern PreviewControl g_preview_ctl_buf[2];
extern AudioUiState g_audio_ui_state_buf[2];
extern volatile uint8_t g_audio_ui_state_idx;
extern volatile uint8_t g_rt_active_idx;
extern FxParamsAudio g_fx_params_buf[2];
extern volatile uint8_t g_fx_params_idx;
extern AudioParamsAudio g_audio_params_audio_buf[2];
extern volatile uint8_t g_audio_params_audio_idx;
extern volatile float g_delay_time_alpha;
extern volatile float g_delay_param_alpha;
extern volatile bool g_audio_recording_active;
extern volatile size_t g_recorded_length_audio;
extern volatile size_t record_pos;
extern volatile uint32_t g_record_start_ms;
extern volatile int32_t g_active_voice_count;
extern int16_t* sample_buffer_l;
extern int16_t* sample_buffer_r;
extern volatile size_t sample_play_start;
extern volatile size_t sample_play_end;
extern volatile uint32_t sample_rate;
extern volatile uint16_t sample_channels;
extern float trim_start;
extern float trim_end;
extern uint32_t snap_start_frame;
extern uint32_t snap_end_frame;
extern volatile bool playback_active;
extern bool AllocatePerformSample(size_t bytes, void** out_ptr);
extern void FreePerformSample();
extern BiquadLp perform_lpf_l1[];
extern BiquadLp perform_lpf_l2[];
extern BiquadLp perform_lpf_r1[];
extern BiquadLp perform_lpf_r2[];
extern WaveformCache perform_waveform_cache;
extern bool g_reset_voices_pending;
extern WaveformJob g_wf_job;
extern FileListJob g_list_job;
extern Job g_job;
extern PerformState main_perform_state;
extern LoaderState loader_state;
extern bool load_in_progress;
extern bool load_target_is_edt;
extern uint16_t load_cookie_next;
extern uint16_t load_cookie_active;
extern uint32_t load_fail_budget_count;
extern uint32_t load_fail_io_count;
extern uint32_t load_success_count;
extern bool delete_in_progress;
extern uint16_t delete_cookie_next;
extern uint16_t delete_cookie_active;
extern uint32_t load_scan_start_ms;
extern bool list_build_pending;
extern size_t save_frames_written;
extern const char* waveform_title;
extern RecordInput waveform_record_input;
extern volatile int32_t preview_index;
extern volatile uint32_t preview_sample_rate;
extern volatile uint16_t preview_channels;
extern volatile float preview_rate;
extern volatile float preview_read_frac;
extern volatile size_t preview_read_index;
extern volatile size_t preview_write_index;
extern volatile uint32_t preview_data_offset;
extern volatile uint32_t preview_fade_samples_left;
extern volatile uint32_t preview_fade_samples_total;
extern int16_t preview_buffer[];
#if STORAGE_SERVICE_PREVIEW_STREAM
extern uint16_t preview_stream_cookie;
extern uint16_t preview_stream_cookie_active;
extern bool preview_pending_start;
extern uint32_t preview_pending_start_ms;
extern int16_t preview_pp_buf[2][kPreviewPpFrames];
extern volatile uint8_t preview_pp_ready[2];
extern volatile uint8_t preview_pp_active;
extern volatile uint32_t preview_pp_pos;
extern int16_t preview_preload_buf[];
extern volatile size_t preview_preload_frames;
extern volatile bool preview_preload_active;
#endif
extern volatile float cpu_load_pct;
extern volatile float cpu_load_peak_pct;
extern volatile uint32_t callback_cycles_last;
extern volatile uint32_t callback_cycles_max;
extern volatile uint32_t callback_overruns;
extern float cpu_load_ema;

// Build and publish g_fx_params_buf (heavy mapping; call at <=50-100Hz).
struct FxParamsUi
{
	float sat_drive;
	float sat_mix;
	float sat_bump;
	float bit_reso;
	float chorus_depth;
	float chorus_mix;
	float chorus_rate;
	float chorus_wow;
	float tape_rate;
	float delay_wet;
	float delay_time;
	float delay_feedback;
	float delay_spread;
	float delay_freeze;
	float reverb_wet;
	float reverb_pre;
	float reverb_damp;
	float reverb_decay;
};

static FxParamsUi BuildFxParamsUi(const AudioParams &p)
{
	FxParamsUi ui = {};
	ui.sat_drive = Clamp01(p.sat_drive);
	ui.sat_mix = Clamp01(p.fx_s_wet);
	ui.sat_bump = Clamp01(p.sat_tape_bump);
	ui.bit_reso = Clamp01(p.sat_bit_reso);
	ui.chorus_depth = Clamp01(p.mod_depth);
	ui.chorus_mix = Clamp01(p.fx_c_wet);
	ui.chorus_rate = Clamp01(p.chorus_rate);
	ui.chorus_wow = Clamp01(p.chorus_wow);
	ui.tape_rate = Clamp01(p.tape_rate);
	ui.delay_wet = Clamp01(p.delay_wet);
	ui.delay_time = Clamp01(p.delay_time);
	ui.delay_feedback = Clamp01(p.delay_feedback);
	ui.delay_spread = Clamp01(p.delay_spread);
	ui.delay_freeze = Clamp01(p.delay_freeze);
	ui.reverb_wet = Clamp01(p.reverb_wet);
	ui.reverb_pre = Clamp01(p.reverb_pre);
	ui.reverb_damp = Clamp01(p.reverb_damp);
	ui.reverb_decay = Clamp01(p.reverb_decay);
	return ui;
}

static FxParamsAudio MapFxParamsUiToAudio(const FxParamsUi &ui, float out_sr)
{
	FxParamsAudio fx = {};

	// Saturation / bitcrush
	fx.sat_mode = sat_mode;
	fx.sat_mix = ui.sat_mix;
	fx.sat_bump = ui.sat_bump;
	fx.sat_drive_amt = powf(ui.sat_drive, 0.7f);
	{
		const int bits_idx = BitResoIndexFromValue(ui.bit_reso);
		const int bits = kBitResoSteps[bits_idx];
		fx.bit_step = 1.0f / powf(2.0f, static_cast<float>(bits - 1));
	}

	// Chorus / tape
	fx.chorus_mode = chorus_mode;
	fx.chorus_mix = ui.chorus_mix;
	{
		const float depth_curve = ui.chorus_depth * ui.chorus_depth;
		const float depth_scale = kChorusMaxDepth * 1.2f;
		fx.chorus_depth_mapped = depth_curve * depth_scale;
	}
	{
		const float rate_curve = ui.chorus_rate * ui.chorus_rate;
		fx.chorus_rate_hz = kChorusRateMinHz + rate_curve * (kChorusRateMaxHz - kChorusRateMinHz);
	}
	fx.chorus_wow = ui.chorus_wow;
	fx.tape_rate = ui.tape_rate;
	fx.tape_drop_amt_mapped = (ui.chorus_wow > 0.0f) ? powf(ui.chorus_wow, 0.6f) : 0.0f;

	// Delay mapping
	fx.delay_wet = ui.delay_wet;
	{
		const float curve = ui.delay_time * ui.delay_time;
		float time_ms = kDelayTimeMinMs + curve * (kDelayTimeMaxMs - kDelayTimeMinMs);
		float samples = time_ms * 0.001f * out_sr;
		const float max_samp = static_cast<float>(kDelayMaxSamples - 1);
		if (samples > max_samp) samples = max_samp;
		if (samples < 1.0f) samples = 1.0f;
		fx.delay_time_samples = samples;
	}
	fx.delay_feedback = ui.delay_feedback;
	if (fx.delay_feedback > kDelayFeedbackMax)
	{
		fx.delay_feedback = kDelayFeedbackMax;
	}
	fx.delay_spread = ui.delay_spread;
	fx.delay_freeze = ui.delay_freeze;

	// Reverb mapping
	fx.reverb_wet = ui.reverb_wet;
	fx.reverb_gain = 1.0f;
	{
		const float decay_ms = kReverbDecayMinMs
			+ ui.reverb_decay * ui.reverb_decay * (kReverbDecayMaxMs - kReverbDecayMinMs);
		const float decay_samples = decay_ms * 0.001f * out_sr;
		if (ui.reverb_decay >= 0.999f) fx.reverb_release = 1.0f;
		else if (decay_samples > 1.0f) fx.reverb_release = expf(-1.0f / decay_samples);
		else fx.reverb_release = 0.0f;
	}
	fx.reverb_feedback = (ui.reverb_decay >= 0.999f) ? 0.99f : kReverbFeedback;
	{
		const float damp_curve = ui.reverb_damp * 1.6f;
		float rev_lp = kReverbDampMaxHz * powf(kReverbDampMinHz / kReverbDampMaxHz, damp_curve);
		const float rev_lp_max = out_sr * 0.49f;
		if (rev_lp > rev_lp_max) rev_lp = rev_lp_max;
		if (rev_lp < kReverbDampMinHz) rev_lp = kReverbDampMinHz;
		fx.reverb_lp_hz = rev_lp;
	}
	{
		const float pre_curve = powf(ui.reverb_pre, 3.0f);
		float samples = pre_curve * (kReverbPreDelayMaxMs * 0.001f * out_sr);
		const float max_samp = static_cast<float>(kReverbPreDelayMaxSamples - 1);
		if (samples > max_samp) samples = max_samp;
		if (samples < 0.0f) samples = 0.0f;
		fx.reverb_predelay_samples = samples;
	}
	return fx;
}

static void BuildAndPublishFxParamsAudio(const AudioParams &p, float out_sr)
{
	const FxParamsUi ui = BuildFxParamsUi(p);
	const FxParamsAudio fx = MapFxParamsUiToAudio(ui, out_sr);
	const uint8_t next = g_fx_params_idx ^ 1;
	g_fx_params_buf[next] = fx;
	{
		daisy::ScopedIrqBlocker irq;
		g_fx_params_idx = next;
	}
}

static void BuildAndPublishAudioParamsAudio(const AudioParams &p, float out_sr)
{
	AudioParamsAudio ap = {};
	const float amp_attack_ms = AmpEnvMsFromFader(p.amp_attack);
	const float amp_release_ms = AmpEnvMsFromFader(p.amp_release);
	ap.amp_attack_samples = amp_attack_ms * 0.001f * out_sr;
	ap.amp_release_samples = amp_release_ms * 0.001f * out_sr;
	ap.flt_cutoff_hz = FltCutoffFromFader(p.flt_cutoff, out_sr);
	ap.flt_q = FltQFromFader(p.flt_res);
	ap.flt_coeffs = ComputeBiquadLpCoeffs(out_sr, ap.flt_cutoff_hz, ap.flt_q);

	const uint8_t next = g_audio_params_audio_idx ^ 1;
	g_audio_params_audio_buf[next] = ap;
	{
		daisy::ScopedIrqBlocker irq;
		g_audio_params_audio_idx = next;
	}
}

static SmoothParam sm_amp_attack;
static SmoothParam sm_amp_decay;
static SmoothParam sm_amp_sustain;
static SmoothParam sm_amp_release;
static SmoothParam sm_flt_cutoff;
static SmoothParam sm_flt_res;
static SmoothParam sm_fx_s_wet;
static SmoothParam sm_sat_drive;
static SmoothParam sm_sat_tape_bump;
static SmoothParam sm_sat_bit_reso;
static SmoothParam sm_sat_bit_smpl;
static SmoothParam sm_fx_c_wet;
static SmoothParam sm_mod_depth;
static SmoothParam sm_chorus_rate;
static SmoothParam sm_chorus_wow;
static SmoothParam sm_tape_rate;
static SmoothParam sm_delay_wet;
static SmoothParam sm_delay_time;
static SmoothParam sm_delay_feedback;
static SmoothParam sm_delay_spread;
static SmoothParam sm_delay_freeze;
static SmoothParam sm_reverb_wet;
static SmoothParam sm_reverb_pre;
static SmoothParam sm_reverb_damp;
static SmoothParam sm_reverb_decay;

void RequestDisplayUpdate()
{
	g_display_update_pending = true;
}

void FlushDisplayIfDue(uint32_t now)
{
	if (!g_display_update_pending)
	{
		return;
	}
	if ((uint32_t)(now - g_last_draw_ms) < kDrawIntervalMs)
	{
		return;
	}
	g_last_draw_ms = now;
	display.Update();
	g_display_update_pending = false;
}

static uint32_t last_ui_ms = 0;
void UiTick(int32_t encoder_l_inc, int32_t encoder_r_inc, uint32_t ctrl_events, bool shift_held);

void Ui::Init()
{
	last_ui_ms = System::GetNow();
}

void Ui::Tick(uint32_t now_ms)
{
	uint8_t ui_idx = 0;
	const AudioUiState& uir = GetAudioUiStateSnapshot(ui_idx);
	const bool playback_busy = uir.playback_active || uir.perform_voices_active;
	const uint32_t ui_tick_ms = playback_busy ? kUiTickPlaybackMs : kUiTickMs;

	int32_t loops = 0;
	while ((int32_t)(now_ms - last_ui_ms) >= (int32_t)ui_tick_ms && loops < 4)
	{
		last_ui_ms += ui_tick_ms;
		int32_t enc_l_inc = 0;
		int32_t enc_r_inc = 0;
		uint32_t ev = 0;
		bool shift_held = false;
		bool btn1_held = false;
		{
			daisy::ScopedIrqBlocker irq;
			enc_l_inc = g_enc_l_delta;
			g_enc_l_delta = 0;
			enc_r_inc = g_enc_r_delta;
			g_enc_r_delta = 0;
			ev = g_ctrl_events;
			g_ctrl_events = 0;
			shift_held = g_shift_held;
			btn1_held = g_btn1_held;
		}

		ui_button1_held = btn1_held;
		UiTick(enc_l_inc, enc_r_inc, ev, shift_held);
		JobTick();
		UpdateSmoothedParamsPerTick();
		loops++;
	}
}
void UiTick(int32_t encoder_l_inc, int32_t encoder_r_inc, uint32_t ctrl_events, bool shift_held)
{
	const bool encoder_l_pressed = (ctrl_events & kEncLPress) != 0;
	const bool encoder_r_pressed = (ctrl_events & kEncRPress) != 0;
	const bool button1_press_event = (ctrl_events & kBtn1Press) != 0;
	const bool button2_press_event = (ctrl_events & kBtn2Press) != 0;

	if (encoder_r_inc != 0)
	{
		encoder_r_accum += encoder_r_inc;
	}
	bool encoder_r_consumed = false;
	if (encoder_r_pressed)
	{
		encoder_r_button_press = true;
	}

	const float out_sr = hw.AudioSampleRate();
	const uint32_t now_ms = System::GetNow();
	const bool ui_blocked = (sd_init_in_progress || save_in_progress);
	if (ui_mode == UiMode::FxDetail
		&& fx_detail_index == kFxDelayIndex
		&& delay_freeze >= 0.5f)
	{
		const uint32_t now = System::GetNow();
		if (now >= delay_snow_next_ms)
		{
			delay_snow_next_ms = now + 100;
			request_fx_detail_redraw = true;
		}
	}
	const bool shift_pressed = shift_held;
	const bool perform_ui = IsPerformUiMode(ui_mode);
	if (!perform_ui && ui_mode != UiMode::FxDetail)
	{
		fx_window_active = false;
	}
	if (!perform_ui)
	{
		amp_window_active = false;
		flt_window_active = false;
	}
	if (button1_press_event)
	{
		button1_press = true;
	}
	if (button2_press_event)
	{
		button2_press = true;
	}
	preview_hold = (ui_mode == UiMode::Load
		&& (load_context == LoadContext::Edt || delete_mode)
		&& !(kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode))
		? ui_button1_held
		: false;
	if (!sd_init_in_progress && ui_mode == UiMode::Shift)
	{
		if (encoder_l_inc != 0)
		{
			int32_t next = shift_menu_index + encoder_l_inc;
			while (next < 0)
			{
				next += kShiftMenuCount;
			}
			while (next >= kShiftMenuCount)
			{
				next -= kShiftMenuCount;
			}
			shift_menu_index = next;
			request_shift_redraw = true;
		}
		if (shift_menu_index == 2 && encoder_r_inc != 0)
		{
			const int inc = encoder_r_inc;
			const int abs_inc = (inc < 0) ? -inc : inc;
			int step_mult = ClampI(abs_inc * abs_inc, 1, 25);
			float next = phones_volume
				+ (static_cast<float>(inc) * kPhonesVolumeStep * static_cast<float>(step_mult));
			next = ClampF(next, 0.0f, 1.0f);
			{
				daisy::ScopedIrqBlocker irq;
				phones_volume = next;
			}
			request_shift_redraw = true;
		}
		if (encoder_r_pressed)
		{
			if (shift_menu_index == 0)
			{
				ui_mode = UiMode::PresetSaveStub;
			}
			else if (shift_menu_index == 1)
			{
				delete_mode = true;
				delete_confirm = false;
				delete_prev_mode = shift_prev_mode;
				load_context = LoadContext::Main;
				load_prev_mode = delete_prev_mode;
				ui_mode = UiMode::Load;
				load_selected = 0;
				load_scroll = 0;
				request_delete_scan = true;
				request_delete_redraw = true;
			}
		}
		if (encoder_l_pressed)
		{
			ui_mode = shift_prev_mode;
			request_shift_redraw = true;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::Main)
	{
		if (encoder_l_inc != 0)
		{
			menu_index = NextMenuIndex(menu_index, encoder_l_inc);
		}
		if (encoder_r_pressed && menu_index == 0)
		{
			delete_mode = false;
			delete_confirm = false;
			load_context = LoadContext::Main;
			load_prev_mode = UiMode::Main;
			load_mode_index = 0;
			ui_mode = UiMode::LoadModeSelect;
		}
		else if (encoder_r_pressed && menu_index == 1)
		{
			ui_mode = UiMode::Record;
			record_source_index = (record_input == RecordInput::Mic) ? 1 : 0;
			record_state = RecordState::SourceSelect;
			RequestPlaybackStopAll();
			record_anim_start_ms = NowMs();
		}
		else if (encoder_r_pressed && menu_index == 2)
		{
			ui_mode = UiMode::Perform;
			midi_ignore_until_ms = now_ms + 200;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::Load)
	{
		if (kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode)
		{
			if (encoder_l_pressed)
			{
				ui_mode = load_prev_mode;
				load_context = LoadContext::Main;
			}
		}
		else if (delete_mode && delete_confirm)
		{
			if (encoder_r_pressed)
			{
				request_delete_file = true;
				request_delete_index = load_selected;
				delete_confirm = false;
				request_delete_redraw = true;
			}
			if (encoder_l_pressed)
			{
				delete_confirm = false;
				request_delete_redraw = true;
			}
		}
		else
		{
			if (encoder_l_inc != 0 && wav_file_count > 0)
			{
				const int32_t count = wav_file_count;
				const int32_t visible_lines = LoadVisibleLines();
				int32_t next = load_selected + encoder_l_inc;
				while (next < 0)
				{
					next += count;
				}
				while (next >= count)
				{
					next -= count;
				}
				load_selected = next;
				int32_t max_top = count - visible_lines;
				if (max_top < 0)
				{
					max_top = 0;
				}
				if (load_selected < load_scroll)
				{
					load_scroll = load_selected;
				}
				else if (load_selected >= load_scroll + visible_lines)
				{
					load_scroll = load_selected - (visible_lines - 1);
				}
				if (load_scroll < 0)
				{
					load_scroll = 0;
				}
				if (load_scroll > max_top)
				{
					load_scroll = max_top;
				}
			}
			if (encoder_r_pressed && wav_file_count > 0)
			{
				if (delete_mode)
				{
					delete_confirm = true;
					CopyString(delete_confirm_name, wav_files[load_selected], kMaxWavNameLen);
					request_delete_redraw = true;
				}
				else if (load_context == LoadContext::Edt)
				{
					request_load_sample = true;
					request_load_index = load_selected;
				}
				else
				{
					request_load_sample = true;
					request_load_index = load_selected;
				}
			}
			if (encoder_l_pressed)
			{
				if (delete_mode)
				{
					delete_mode = false;
					delete_confirm = false;
					ui_mode = delete_prev_mode;
				}
				else
				{
					ui_mode = load_prev_mode;
					load_context = LoadContext::Main;
				}
			}
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::LoadModeSelect)
	{
		if (encoder_l_inc != 0)
		{
			int32_t next = load_mode_index + encoder_l_inc;
			while (next < 0)
			{
				next += 2;
			}
			while (next >= 2)
			{
				next -= 2;
			}
			load_mode_index = next;
		}
		if (encoder_r_pressed)
		{
			load_stub_mode = (load_mode_index == 0) ? LoadStubMode::Presets : LoadStubMode::Bake;
			ui_mode = UiMode::LoadStub;
		}
		if (encoder_l_pressed)
		{
			ui_mode = load_prev_mode;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::LoadStub)
	{
		if (encoder_l_pressed || encoder_r_pressed)
		{
			ui_mode = UiMode::LoadModeSelect;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::PresetSaveStub)
	{
		if (encoder_l_pressed || encoder_r_pressed)
		{
			ui_mode = UiMode::Shift;
			request_shift_redraw = true;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::Record)
	{
		if (record_state == RecordState::SourceSelect)
		{
			if (encoder_l_inc != 0)
			{
				int32_t next = record_source_index + encoder_l_inc;
				while (next < 0)
				{
					next += 2;
				}
				while (next >= 2)
				{
					next -= 2;
				}
				record_source_index = next;
			}
			if (encoder_r_pressed)
			{
				record_input = (record_source_index == 1) ? RecordInput::Mic : RecordInput::LineIn;
				record_state = RecordState::Armed;
				record_anim_start_ms = -1.0;
				encoder_r_consumed = true;
			}
		}
		else if (record_state == RecordState::TargetSelect)
		{
			if (encoder_l_inc != 0)
			{
				int32_t next = record_target_index + encoder_l_inc;
				while (next < 0)
				{
					next += kRecordTargetCount;
				}
				while (next >= kRecordTargetCount)
				{
					next -= kRecordTargetCount;
				}
				record_target_index = next;
			}
		}
		if (encoder_r_pressed && !encoder_r_consumed && record_state != RecordState::SourceSelect)
		{
			if (record_state == RecordState::Armed)
			{
				record_state = RecordState::Countdown;
				record_countdown_start_ms = System::GetNow();
				request_length_redraw = true;
			}
			else if (record_state == RecordState::Recording)
			{
				RequestAudioCmd(kCmdRecStop);
			}
			else if (record_state == RecordState::Review && sample_loaded)
			{
				record_target_index = kRecordTargetSave;
				record_state = RecordState::TargetSelect;
			}
			else if (record_state == RecordState::TargetSelect)
			{
				save_in_progress = true;
				save_done = false;
				save_success = false;
				save_started = false;
				save_prev_mode = UiMode::Main;
				save_start_ms = System::GetNow();
				save_result_until_ms = 0;
				save_draw_next_ms = 0;
				save_filename[0] = '\0';
				ResetSaveState();
				record_state = RecordState::Review;
			}
			else if (record_state == RecordState::BackConfirm)
			{
				sample_loaded = false;
				sample_length = 0;
				loaded_sample_name[0] = '\0';
				waveform_ready = false;
				waveform_dirty = true;
				waveform_from_recording = false;
				RequestPlaybackStopAll();
				PublishRuntimeFromUi();
				record_state = RecordState::SourceSelect;
				request_length_redraw = true;
			}
		}
		if (encoder_l_pressed)
		{
			if (record_state == RecordState::TargetSelect)
			{
				record_state = RecordState::Review;
				waveform_dirty = true;
				request_length_redraw = true;
			}
			else if (record_state == RecordState::SourceSelect)
			{
				ui_mode = UiMode::Main;
				RequestPlaybackStopAll();
				record_anim_start_ms = -1.0;
			}
			else if (record_state == RecordState::BackConfirm)
			{
				record_state = RecordState::Review;
				waveform_dirty = true;
				request_length_redraw = true;
			}
			else if (record_state == RecordState::Review && waveform_from_recording)
			{
				record_state = RecordState::BackConfirm;
				request_length_redraw = true;
			}
			else
			{
				record_state = RecordState::SourceSelect;
				RequestPlaybackStopAll();
				record_anim_start_ms = -1.0;
			}
		}
		if (record_state == RecordState::Review && sample_loaded)
		{
			if(encoder_l_inc != 0 || encoder_r_inc != 0)
			{
				int32_t dl = encoder_l_inc;
				int32_t dr = encoder_r_inc;
				if (shift_held)
				{
					if (dl > 0) dl = 1;
					else if (dl < 0) dl = -1;
					if (dr > 0) dr = 1;
					else if (dr < 0) dr = -1;
				}
				AdjustTrimNormalized(dl, dr, shift_held);
			}
		}
	}
	else if (!ui_blocked && IsPerformUiMode(ui_mode))
	{
		const bool fx_select_active = (perform_index == kPerformFxIndex && fx_window_active);
		const bool amp_select_active = (perform_index == kPerformAmpIndex && amp_window_active);
		const bool flt_select_active = (perform_index == kPerformFltIndex && flt_window_active);
		const bool fx_reorder_active = fx_select_active && shift_pressed;
		const bool has_sample = (sample_loaded && sample_length > 0);
		const int32_t perf_r_inc = encoder_r_inc * kPerformEncoderScale;
		if (encoder_l_inc != 0)
		{
			if (fx_select_active)
			{
				int32_t next = fx_fader_index + encoder_l_inc;
				while (next < 0)
				{
					next += kPerformFaderCount;
				}
				while (next >= kPerformFaderCount)
				{
					next -= kPerformFaderCount;
				}
				if (next != fx_fader_index)
				{
					fx_fader_index = next;
					request_perform_redraw = true;
				}
			}
			else if (amp_select_active)
			{
				int32_t next = amp_fader_index + encoder_l_inc;
				while (next < 0)
				{
					next += kPerformFaderCount;
				}
				while (next >= kPerformFaderCount)
				{
					next -= kPerformFaderCount;
				}
				if (next != amp_fader_index)
				{
					amp_fader_index = next;
					request_perform_redraw = true;
				}
			}
			else if (flt_select_active)
			{
				int32_t next = flt_fader_index + encoder_l_inc;
				while (next < 0)
				{
					next += kPerformFltFaderCount;
				}
				while (next >= kPerformFltFaderCount)
				{
					next -= kPerformFltFaderCount;
				}
				if (next != flt_fader_index)
				{
					flt_fader_index = next;
					request_perform_redraw = true;
				}
			}
			else
			{
				perform_index = NextPerformIndex(perform_index, encoder_l_inc);
			}
		}
		if (encoder_r_pressed)
		{
			if (perform_index == kPerformFxIndex)
			{
				if (!fx_window_active)
				{
					fx_window_active = true;
					amp_window_active = false;
					flt_window_active = false;
					request_perform_redraw = true;
				}
				else
				{
					fx_detail_index = fx_chain_order[fx_fader_index];
					fx_detail_param_index = 0;
					if (fx_detail_index == kFxSatIndex)
					{
						if (!sat_params_initialized)
						{
							sat_drive = 0.5f;
							sat_tape_bump = 0.5f;
							sat_bit_reso = 0.5f;
							sat_bit_smpl = 0.5f;
							sat_mode = 0;
							sat_params_initialized = true;
							fx_params_dirty = true;
						}
					}
					else if (fx_detail_index == kFxReverbIndex)
					{
						if (!reverb_params_initialized)
						{
							reverb_pre = 0.5f;
							reverb_damp = 0.5f;
							reverb_decay = 0.5f;
							reverb_params_initialized = true;
							fx_params_dirty = true;
						}
					}
					else if (fx_detail_index == kFxDelayIndex)
					{
						if (!delay_params_initialized)
						{
							delay_time = 0.5f;
							delay_feedback = 0.5f;
							delay_spread = 0.5f;
							delay_freeze = 0.0f;
							delay_params_initialized = true;
							fx_params_dirty = true;
						}
					}
					else if (fx_detail_index == kFxChorusIndex)
					{
						if (!mod_params_initialized)
						{
							mod_depth = 0.5f;
							chorus_rate = 0.5f;
							chorus_wow = 0.5f;
							tape_rate = 0.5f;
							chorus_mode = 0;
							mod_params_initialized = true;
							fx_params_dirty = true;
						}
					}
					fx_detail_prev_mode = ui_mode;
					ui_mode = UiMode::FxDetail;
					request_fx_detail_redraw = true;
				}
			}
			else if (perform_index == kPerformAmpIndex)
			{
				if (!amp_window_active)
				{
					amp_window_active = true;
					fx_window_active = false;
					flt_window_active = false;
					request_perform_redraw = true;
				}
			}
			else if (perform_index == kPerformFltIndex)
			{
				if (!flt_window_active)
				{
					flt_window_active = true;
					fx_window_active = false;
					amp_window_active = false;
					request_perform_redraw = true;
				}
			}
			else if (!fx_select_active && !amp_select_active && !flt_select_active
				&& perform_index == kPerformEdtIndex)
			{
				if (!has_sample)
				{
					delete_mode = false;
					delete_confirm = false;
					edt_sample_context = SampleContext::Perform;
					load_context = LoadContext::Edt;
					load_prev_mode = UiMode::Perform;
					ui_mode = UiMode::Load;
					load_selected = 0;
					load_scroll = 0;
					request_load_scan = true;
				}
				else
				{
					edt_prev_mode = ui_mode;
					edt_sample_context = SampleContext::Perform;
					ui_mode = UiMode::Edt;
					waveform_ready = true;
					waveform_dirty = true;
					request_length_redraw = true;
				}
			}
		}
		if (fx_reorder_active && encoder_r_inc != 0)
		{
			const int32_t dir = (encoder_r_inc > 0) ? 1 : -1;
			int32_t steps = (encoder_r_inc > 0) ? encoder_r_inc : -encoder_r_inc;
			for (int32_t s = 0; s < steps; ++s)
			{
				const int32_t from = fx_fader_index;
				int32_t to = from + dir;
				if (to < 0)
				{
					to = kPerformFaderCount - 1;
				}
				else if (to >= kPerformFaderCount)
				{
					to = 0;
				}
				const int32_t temp = fx_chain_order[from];
				fx_chain_order[from] = fx_chain_order[to];
				fx_chain_order[to] = temp;
				fx_fader_index = to;
			}
			request_perform_redraw = true;
			fx_chain_last_move_ms = now_ms;
			if (fx_chain_fade_target > 0.0f)
			{
				const int32_t fade_samples
					= static_cast<int32_t>((out_sr * (kFxChainFadeMs * 0.001f)) + 0.5f);
				fx_chain_fade_target = 0.0f;
				fx_chain_fade_samples_left = (fade_samples > 0) ? fade_samples : 1;
				fx_chain_pause_pending = true;
			}
			PublishFxChainFromUi();
		}
		else if (fx_select_active && perf_r_inc != 0)
		{
			const int32_t fx_id = fx_chain_order[fx_fader_index];
			const float step = FxWetStep(fx_id);
			volatile float* target = FxWetTarget(fx_id);
			const float current = *target;
			float next = current + (static_cast<float>(perf_r_inc) * step);
			if (next < 0.0f)
			{
				next = 0.0f;
			}
			if (next > 1.0f)
			{
				next = 1.0f;
			}
			if (next != current)
			{
				*target = next;
				request_perform_redraw = true;
				fx_params_dirty = true;
			}
		}
		else if (amp_select_active && perf_r_inc != 0)
		{
			const float step = kAmpEnvStep;
			volatile float* targets[kPerformFaderCount]
				= {&amp_attack, &amp_decay, &amp_sustain, &amp_release};
			const int idx = amp_fader_index;
			volatile float* target = targets[idx];
			const float current = *target;
			float next = current + (static_cast<float>(perf_r_inc) * step);
			if (next < 0.0f)
			{
				next = 0.0f;
			}
			if (next > 1.0f)
			{
				next = 1.0f;
			}
			if (next != current)
			{
				*target = next;
				request_perform_redraw = true;
			}
		}
		else if (flt_select_active && perf_r_inc != 0)
		{
			const float step = kFltParamStep;
			volatile float* targets[kPerformFltFaderCount] = {&flt_cutoff, &flt_res};
			const int idx = flt_fader_index;
			volatile float* target = targets[idx];
			const float current = *target;
			float next = current + (static_cast<float>(perf_r_inc) * step);
			if (next < 0.0f)
			{
				next = 0.0f;
			}
			if (next > 1.0f)
			{
				next = 1.0f;
			}
			if (next != current)
			{
				*target = next;
				request_perform_redraw = true;
			}
		}
		if (encoder_l_pressed)
		{
			if (fx_select_active || amp_select_active || flt_select_active)
			{
				fx_window_active = false;
				amp_window_active = false;
				flt_window_active = false;
				request_perform_redraw = true;
			}
			else
			{
				ui_mode = UiMode::Main;
			}
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::Edt)
	{
		if (encoder_l_inc != 0 || encoder_r_inc != 0)
		{
			int32_t dl = encoder_l_inc;
			int32_t dr = encoder_r_inc;
			if (shift_held)
			{
				if (dl > 0) dl = 1;
				else if (dl < 0) dl = -1;
				if (dr > 0) dr = 1;
				else if (dr < 0) dr = -1;
			}
			AdjustTrimNormalized(dl, dr, shift_held);
		}
		if (encoder_r_pressed)
		{
			delete_mode = false;
			delete_confirm = false;
			edt_sample_context = current_sample_context;
			load_context = LoadContext::Edt;
			load_prev_mode = edt_prev_mode;
			ui_mode = UiMode::Load;
			load_selected = 0;
			load_scroll = 0;
			request_load_scan = true;
		}
		if (encoder_l_pressed)
		{
			ui_mode = edt_prev_mode;
			if (ui_mode == UiMode::Perform)
			{
				midi_ignore_until_ms = now_ms + 200;
			}
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::FxDetail)
	{
		const int32_t fx_r_inc = encoder_r_inc * kPerformEncoderScale;
		const int32_t fx_r_dir = (encoder_r_inc > 0) ? 1 : (encoder_r_inc < 0 ? -1 : 0);
		if (fx_detail_index == kFxSatIndex)
		{
			if (encoder_l_inc != 0)
			{
				const int32_t param_count = 4;
				int32_t next = fx_detail_param_index + encoder_l_inc;
				while (next < 0)
				{
					next += param_count;
				}
				while (next >= param_count)
				{
					next -= param_count;
				}
				if (next != fx_detail_param_index)
				{
					fx_detail_param_index = next;
					request_fx_detail_redraw = true;
				}
			}
			if (encoder_r_inc != 0)
			{
				const float steps[3] = {kReverbWetStep, kReverbWetStep, kReverbWetStep};
				volatile float* targets[3]
					= {(sat_mode == 1) ? &sat_bit_reso : &sat_drive,
					   (sat_mode == 1) ? &sat_bit_smpl : &sat_tape_bump,
					   &fx_s_wet};
				const int idx = fx_detail_param_index;
				if (idx == 3)
				{
					if (fx_r_dir != 0)
					{
						sat_mode = (sat_mode == 0) ? 1 : 0;
					}
					request_fx_detail_redraw = true;
					fx_params_dirty = true;
				}
				if (idx >= 0 && idx < 3)
				{
					volatile float* target = targets[idx];
					const float current = *target;
					float next = current;
					if (sat_mode == 1 && idx == 0)
					{
						const int cur_idx = BitResoIndexFromValue(current);
						const int next_idx = ClampI(cur_idx + fx_r_dir, 0, kBitResoStepCount - 1);
						next = BitResoValueFromIndex(next_idx);
					}
					else
					{
						const float step = steps[idx];
						next = current + (static_cast<float>(fx_r_inc) * step);
						if (next < 0.0f)
						{
							next = 0.0f;
						}
						if (next > 1.0f)
						{
							next = 1.0f;
						}
					}
					if (next != current)
					{
						*target = next;
						request_fx_detail_redraw = true;
						fx_params_dirty = true;
					}
				}
			}
		}
		else if (fx_detail_index == kFxChorusIndex)
		{
			if (encoder_r_pressed)
			{
				chorus_mode = (chorus_mode == 0) ? 1 : 0;
				request_fx_detail_redraw = true;
				fx_params_dirty = true;
			}
			if (encoder_l_inc != 0)
			{
				const int32_t param_count = 4;
				int32_t next = fx_detail_param_index + encoder_l_inc;
				while (next < 0)
				{
					next += param_count;
				}
				while (next >= param_count)
				{
					next -= param_count;
				}
				if (next != fx_detail_param_index)
				{
					fx_detail_param_index = next;
					request_fx_detail_redraw = true;
				}
			}
			if (encoder_r_inc != 0)
			{
				if (fx_detail_param_index == 3)
				{
					int32_t next = chorus_mode + fx_r_dir;
					while (next < 0)
					{
						next += 2;
					}
					while (next >= 2)
					{
						next -= 2;
					}
					if (next != chorus_mode)
					{
						chorus_mode = next;
						request_fx_detail_redraw = true;
						fx_params_dirty = true;
					}
				}
				else
				{
					const float steps[3]
						= {kReverbWetStep, kChorusRateStep, kReverbWetStep};
					volatile float* targets[3]
						= {(chorus_mode == 1) ? &chorus_wow : &mod_depth,
						   (chorus_mode == 1) ? &tape_rate : &chorus_rate,
						   &fx_c_wet};
					const int idx = fx_detail_param_index;
					if (idx >= 0 && idx < 3)
					{
						const float step = steps[idx];
						volatile float* target = targets[idx];
						const float current = *target;
						float next = current + (static_cast<float>(fx_r_inc) * step);
						if (next < 0.0f)
						{
							next = 0.0f;
						}
						if (next > 1.0f)
						{
							next = 1.0f;
						}
						if (next != current)
						{
							*target = next;
							request_fx_detail_redraw = true;
							fx_params_dirty = true;
						}
					}
				}
			}
		}
		else if (fx_detail_index == kFxDelayIndex)
		{
			if (encoder_l_inc != 0)
			{
				const int32_t param_count = kDelayFaderCount;
				int32_t next = fx_detail_param_index + encoder_l_inc;
				while (next < 0)
				{
					next += param_count;
				}
				while (next >= param_count)
				{
					next -= param_count;
				}
				if (next != fx_detail_param_index)
				{
					fx_detail_param_index = next;
					request_fx_detail_redraw = true;
				}
			}
			if (encoder_r_inc != 0)
			{
				const float steps[kDelayFaderCount]
					= {kDelayParamStep,
					   kDelayParamStep,
					   kDelayParamStep,
					   kDelayParamStep,
					   kDelayWetStep};
				volatile float* targets[kDelayFaderCount]
					= {&delay_time, &delay_feedback, &delay_spread, &delay_freeze, &delay_wet};
				const int idx = fx_detail_param_index;
				if (idx >= 0 && idx < kDelayFaderCount)
				{
					volatile float* target = targets[idx];
					const float current = *target;
					float next = current;
					if (idx == 3)
					{
						const bool freeze_on = (current >= 0.5f);
						if (fx_r_dir != 0)
						{
							next = freeze_on ? 0.0f : 1.0f;
						}
					}
					else
					{
						const float step = steps[idx];
						next = current + (static_cast<float>(fx_r_inc) * step);
						if (next < 0.0f)
						{
							next = 0.0f;
						}
						if (next > 1.0f)
						{
							next = 1.0f;
						}
					}
					if (next != current)
					{
						*target = next;
						request_fx_detail_redraw = true;
						fx_params_dirty = true;
					}
				}
			}
		}
		else if (fx_detail_index == kFxReverbIndex)
		{
			if (encoder_l_inc != 0)
			{
				const int32_t param_count = kReverbFaderCount;
				int32_t next = fx_detail_param_index + encoder_l_inc;
				while (next < 0)
				{
					next += param_count;
				}
				while (next >= param_count)
				{
					next -= param_count;
				}
				if (next != fx_detail_param_index)
				{
					fx_detail_param_index = next;
					request_fx_detail_redraw = true;
				}
			}
			if (encoder_r_inc != 0)
			{
				const float steps[kReverbFaderCount]
					= {kReverbParamStep,
					   kReverbParamStep,
					   kReverbParamStep,
					   0.0f,
					   kReverbWetStep};
				volatile float* targets[kReverbFaderCount]
					= {&reverb_pre, &reverb_damp, &reverb_decay, &playback_reverse, &reverb_wet};
				const int idx = fx_detail_param_index;
				if (idx >= 0 && idx < kReverbFaderCount)
				{
					if (idx == 3)
					{
						const bool reverse_on = (playback_reverse >= 0.5f);
						if (fx_r_inc != 0)
						{
							ApplyPlaybackReverse(!reverse_on);
							request_fx_detail_redraw = true;
						}
					}
					else
					{
						const float step = steps[idx];
						volatile float* target = targets[idx];
						const float current = *target;
						float next = current + (static_cast<float>(fx_r_inc) * step);
						if (next < 0.0f)
						{
							next = 0.0f;
						}
						if (next > 1.0f)
						{
							next = 1.0f;
						}
						if (next != current)
						{
							*target = next;
							request_fx_detail_redraw = true;
							fx_params_dirty = true;
						}
					}
				}
			}
		}
		if (encoder_l_pressed)
		{
			ui_mode = fx_detail_prev_mode;
			if (ui_mode == UiMode::Perform)
			{
				midi_ignore_until_ms = now_ms + 200;
			}
		}
	}
	else
	{
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Main;
		}
	}

	if (fx_chain_last_move_ms != 0
		&& (now_ms - fx_chain_last_move_ms) >= kFxChainIdleMs
		&& fx_chain_fade_target < 1.0f)
	{
		const int32_t fade_samples
			= static_cast<int32_t>((out_sr * (kFxChainFadeMs * 0.001f)) + 0.5f);
		fx_chain_paused = false;
		fx_chain_pause_pending = false;
		fx_chain_fade_target = 1.0f;
		fx_chain_fade_samples_left = (fade_samples > 0) ? fade_samples : 1;
		fx_chain_fade_gain = 0.0f;
		PublishFxChainFromUi();
	}

	if (record_state == RecordState::Countdown)
	{
		if ((System::GetNow() - record_countdown_start_ms) >= kRecordCountdownMs)
		{
			StartRecording();
			RequestAudioCmd(kCmdRecStart);
		}
	}
}




struct Font5x7
{
	static constexpr int W = 5;
	static constexpr int H = 7;

	static void GetGlyphRows(char c, uint8_t out_rows[H])
	{
		if (c >= 'a' && c <= 'z')
		{
			c = static_cast<char>(c - 'a' + 'A');
		}

		auto set = [&](std::initializer_list<uint8_t> rows)
		{
			int i = 0;
			for (auto r : rows)
			{
				out_rows[i++] = r;
			}
		};

		if (c == ' ')
		{
			set({0, 0, 0, 0, 0, 0, 0});
			return;
		}

		switch (c)
		{
			case '0': set({0b01110,0b10001,0b10011,0b10101,0b11001,0b10001,0b01110}); return;
			case '1': set({0b00100,0b01100,0b00100,0b00100,0b00100,0b00100,0b01110}); return;
			case '2': set({0b01110,0b10001,0b00001,0b00010,0b00100,0b01000,0b11111}); return;
			case '3': set({0b11111,0b00010,0b00100,0b00010,0b00001,0b10001,0b01110}); return;
			case '4': set({0b00010,0b00110,0b01010,0b10010,0b11111,0b00010,0b00010}); return;
			case '5': set({0b11111,0b10000,0b11110,0b00001,0b00001,0b10001,0b01110}); return;
			case '6': set({0b00110,0b01000,0b10000,0b11110,0b10001,0b10001,0b01110}); return;
			case '7': set({0b11111,0b00001,0b00010,0b00100,0b01000,0b01000,0b01000}); return;
			case '8': set({0b01110,0b10001,0b10001,0b01110,0b10001,0b10001,0b01110}); return;
			case '9': set({0b01110,0b10001,0b10001,0b01111,0b00001,0b00010,0b01100}); return;
			default: break;
		}

		switch (c)
		{
			case 'A': set({0b01110,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001}); return;
			case 'B': set({0b11110,0b10001,0b10001,0b11110,0b10001,0b10001,0b11110}); return;
			case 'C': set({0b01110,0b10001,0b10000,0b10000,0b10000,0b10001,0b01110}); return;
			case 'D': set({0b11110,0b10001,0b10001,0b10001,0b10001,0b10001,0b11110}); return;
			case 'E': set({0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b11111}); return;
			case 'F': set({0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b10000}); return;
			case 'G': set({0b01110,0b10001,0b10000,0b10111,0b10001,0b10001,0b01110}); return;
			case 'H': set({0b10001,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001}); return;
			case 'I': set({0b01110,0b00100,0b00100,0b00100,0b00100,0b00100,0b01110}); return;
			case 'J': set({0b00111,0b00010,0b00010,0b00010,0b00010,0b10010,0b01100}); return;
			case 'K': set({0b10001,0b10010,0b10100,0b11000,0b10100,0b10010,0b10001}); return;
			case 'L': set({0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b11111}); return;
			case 'M': set({0b10001,0b11011,0b10101,0b10101,0b10001,0b10001,0b10001}); return;
			case 'N': set({0b10001,0b11001,0b10101,0b10011,0b10001,0b10001,0b10001}); return;
			case 'O': set({0b01110,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110}); return;
			case 'P': set({0b11110,0b10001,0b10001,0b11110,0b10000,0b10000,0b10000}); return;
			case 'Q': set({0b01110,0b10001,0b10001,0b10001,0b10101,0b10010,0b01101}); return;
			case 'R': set({0b11110,0b10001,0b10001,0b11110,0b10100,0b10010,0b10001}); return;
			case 'S': set({0b01111,0b10000,0b10000,0b01110,0b00001,0b00001,0b11110}); return;
			case 'T': set({0b11111,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100}); return;
			case 'U': set({0b10001,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110}); return;
			case 'V': set({0b10001,0b10001,0b10001,0b10001,0b10001,0b01010,0b00100}); return;
			case 'W': set({0b10001,0b10001,0b10001,0b10101,0b10101,0b11011,0b10001}); return;
			case 'X': set({0b10001,0b10001,0b01010,0b00100,0b01010,0b10001,0b10001}); return;
			case 'Y': set({0b10001,0b10001,0b01010,0b00100,0b00100,0b00100,0b00100}); return;
			case 'Z': set({0b11111,0b00001,0b00010,0b00100,0b01000,0b10000,0b11111}); return;
			default: break;
		}

		switch (c)
		{
			case '-': set({0,0,0,0b11111,0,0,0}); return;
			case '.': set({0,0,0,0,0,0b00100,0b00100}); return;
			case ':': set({0,0b00100,0b00100,0,0b00100,0b00100,0}); return;
			case '/': set({0b00001,0b00010,0b00100,0b01000,0b10000,0,0}); return;
			case '_': set({0,0,0,0,0,0,0b11111}); return;
			case '>': set({0b10000,0b01000,0b00100,0b00010,0b00100,0b01000,0b10000}); return;
			case '<': set({0b00001,0b00010,0b00100,0b01000,0b00100,0b00010,0b00001}); return;
			case '?': set({0b01110,0b10001,0b00010,0b00100,0b00100,0,0b00100}); return;
			default: break;
		}

		set({0b11111,0b10001,0b00010,0b00100,0b00100,0,0b00100});
	}
};

template <typename F>
static void ForCirclePixels(int cx, int cy, int r, F&& fn)
{
	if (r <= 0)
	{
		return;
	}
	int x = r;
	int y = 0;
	int err = 0;

	while (x >= y)
	{
		fn(cx + x, cy + y);
		fn(cx + y, cy + x);
		fn(cx - y, cy + x);
		fn(cx - x, cy + y);
		fn(cx - x, cy - y);
		fn(cx - y, cy - x);
		fn(cx + y, cy - x);
		fn(cx + x, cy - y);

		++y;
		if (err <= 0)
		{
			err += 2 * y + 1;
		}
		else
		{
			--x;
			err -= 2 * x + 1;
		}
	}
}

#if 0
#endif

const char* kMenuLabels[kMenuCount] = {"LOAD", "RECORD", "PERFORM"};

int32_t NextMenuIndex(int32_t current, int32_t delta)
{
	static const int32_t order[kMenuCount] = {0, 1, 2};
	int32_t pos = 0;
	for (int32_t i = 0; i < kMenuCount; ++i)
	{
		if (order[i] == current)
		{
			pos = i;
			break;
		}
	}
	pos += delta;
	while (pos < 0)
	{
		pos += kMenuCount;
	}
	while (pos >= kMenuCount)
	{
		pos -= kMenuCount;
	}
	return order[pos];
}
const char* kShiftMenuLabels[kShiftMenuCount] = {"SAVE PRESET", "DELETE", "VOLUME"};

int32_t NextPerformIndex(int32_t current, int32_t delta)
{
	static const int32_t order[kPerformBoxCount]
		= {kPerformEdtIndex, kPerformAmpIndex, kPerformFxIndex, kPerformFltIndex};
	int32_t pos = 0;
	for (int32_t i = 0; i < kPerformBoxCount; ++i)
	{
		if (order[i] == current)
		{
			pos = i;
			break;
		}
	}
	pos += delta;
	while (pos < 0)
	{
		pos += kPerformBoxCount;
	}
	while (pos >= kPerformBoxCount)
	{
		pos -= kPerformBoxCount;
	}
	return order[pos];
}

#if !STORAGE_SERVICE_SAVE
alignas(32) static int16_t wav_write[kSaveChunkFrames * 2];
#endif

static size_t StrLen(const char* str)
{
	size_t len = 0;
	while (str[len] != '\0')
	{
		++len;
	}
	return len;
}

void CopyString(char* dst, const char* src, size_t max_len)
{
	if (max_len == 0)
	{
		return;
	}
	size_t i = 0;
	for (; i + 1 < max_len && src[i] != '\0'; ++i)
	{
		dst[i] = src[i];
	}
	dst[i] = '\0';
}

static bool HasWavExtension(const char* name)
{
	const size_t len = StrLen(name);
	if (len < 4)
	{
		return false;
	}
	const char* ext = name + len - 4;
	return ext[0] == '.'
		&& (ext[1] == 'w' || ext[1] == 'W')
		&& (ext[2] == 'a' || ext[2] == 'A')
		&& (ext[3] == 'v' || ext[3] == 'V');
}

static void CopyNameSansWav(char* dst, const char* src, size_t max_len)
{
	if (max_len == 0)
	{
		return;
	}
	size_t len = StrLen(src);
	if (HasWavExtension(src) && len >= 4)
	{
		len -= 4;
	}
	if (len >= max_len)
	{
		len = max_len - 1;
	}
	for (size_t i = 0; i < len; ++i)
	{
		dst[i] = src[i];
	}
	dst[len] = '\0';
}

static void BuildFilePath(const char* name, char* out, size_t out_len)
{
	if (out_len == 0)
	{
		return;
	}
	CopyString(out, storage.GetSdPath(), out_len);
	size_t base_len = StrLen(out);
	size_t i = 0;
	while (name[i] != '\0' && base_len + 1 < out_len)
	{
		out[base_len] = name[i];
		++base_len;
		++i;
	}
	out[base_len] = '\0';
}

void InitLoadLayout()
{
	const FontDef font = Font_6x8;
	load_line_height = static_cast<int32_t>(font.FontHeight * kLoadFontScale);
	load_lines = static_cast<int32_t>(display.Height() / load_line_height);
	if (load_lines < 1)
	{
		load_lines = 1;
	}
	load_chars_per_line = static_cast<int32_t>(
		display.Width() / (font.FontWidth * kLoadFontScale));
	if (load_chars_per_line < 1)
	{
		load_chars_per_line = 1;
	}
}

int32_t LoadVisibleLines()
{
	return load_lines;
}

template <size_t N>
constexpr size_t ArraySize(const char* const (&)[N])
{
	return N;
}

void MountSd()
{
	if (sd_mounted)
	{
		return;
	}
	StorageService::Op op = {};
	op.kind = StorageService::OpKind::Mount;
	const StorageService::MountState mount_state = storage.GetMountState();
	if (mount_state == StorageService::MountState::Mounting
		|| mount_state == StorageService::MountState::Mounted)
	{
		return;
	}
	if (!storage.Enqueue(op))
	{
		return;
	}
}

const char* SdFaultText(StorageService::SdErrorCode code)
{
	switch (code)
	{
		case StorageService::SdErrorCode::NoCard: return "INSERT SD";
		case StorageService::SdErrorCode::MountFailed: return "MOUNT FAIL";
		case StorageService::SdErrorCode::FsCorrupt: return "FS CORRUPT";
		case StorageService::SdErrorCode::OpenFailed: return "OPEN ERROR";
		case StorageService::SdErrorCode::ReadFailed: return "READ ERROR";
		case StorageService::SdErrorCode::WriteFailed: return "WRITE ERROR";
		case StorageService::SdErrorCode::Timeout: return "SD TIMEOUT";
		default: return "SD ERROR";
	}
}

// Save names are generated inside StorageService.

void ResetSaveState()
{
	save_frames_written = 0;
}

// Legacy save path removed; StorageService handles save.

static void __attribute__((unused)) ComputeWaveform()
{
	const int32_t width = 128;
	if (!sample_loaded || sample_length == 0)
	{
		for (int32_t i = 0; i < width; ++i)
		{
			perform_waveform_cache.Min()[i] = 0;
			perform_waveform_cache.Max()[i] = 0;
		}
		return;
	}

	const size_t frames = sample_length;
	if (frames < 2)
	{
		return;
	}

	const size_t columns = 128;
	size_t step = frames / columns;
	if (step == 0)
	{
		step = 1;
	}

	const float scale = 28.0f;

	for (size_t col = 0; col < columns; ++col)
	{
		float minv = 1.0f;
		float maxv = -1.0f;

		const size_t start = col * step;
		const size_t end = (col == columns - 1) ? frames : (start + step);

		for (size_t i = start; i < end; ++i)
		{
			const float s = static_cast<float>(sample_buffer_l[i]) * kSampleScale;
			if (s < minv)
			{
				minv = s;
			}
			if (s > maxv)
			{
				maxv = s;
			}
		}

		perform_waveform_cache.Min()[col] = static_cast<int16_t>(minv * scale);
		perform_waveform_cache.Max()[col] = static_cast<int16_t>(maxv * scale);
	}
}


static inline bool JobsAllowedNow()
{
	if (record_state == RecordState::Recording)
	{
		return false;
	}
	{
		uint8_t ui_idx = 0;
		const AudioUiState& uir = GetAudioUiStateSnapshot(ui_idx);
		if (uir.playback_active || uir.perform_voices_active)
		{
			return false;
		}
	}
	return true;
}

static inline bool FileListAllowedNow()
{
	if (record_state == RecordState::Recording)
	{
		return false;
	}
	return true;
}

Job g_job = {};

static void WaveformJobCancel()
{
	g_wf_job.active = false;
}

static void ClearWaveformCache()
{
	for (int32_t i = 0; i < 128; ++i)
	{
		perform_waveform_cache.Min()[i] = 0;
		perform_waveform_cache.Max()[i] = 0;
	}
}

static void WaveformJobStart(SampleContext ctx)
{
	WaveformJobCancel();
	g_wf_job.ctx = ctx;
	if (!sample_loaded || sample_length == 0)
	{
		ClearWaveformCache();
		waveform_ready = true;
		waveform_dirty = true;
		return;
	}
	const size_t frames = sample_length;
	if (frames < 2)
	{
		waveform_ready = true;
		waveform_dirty = true;
		return;
	}
	g_wf_job.active = true;
	g_wf_job.frames = frames;
	g_wf_job.columns = 128;
	g_wf_job.step = frames / g_wf_job.columns;
	if (g_wf_job.step == 0)
	{
		g_wf_job.step = 1;
	}
	g_wf_job.col = 0;
	g_wf_job.idx = 0;
	g_wf_job.end = (g_wf_job.columns == 1) ? frames : g_wf_job.step;
	g_wf_job.minv = 1.0f;
	g_wf_job.maxv = -1.0f;
	g_wf_job.scale = 28.0f;
	waveform_ready = false;
}

static void WaveformJobTick(uint32_t budget_samples)
{
	if (!g_wf_job.active || budget_samples == 0)
	{
		return;
	}
	while (budget_samples > 0 && g_wf_job.active)
	{
		if (g_wf_job.idx >= g_wf_job.end)
		{
			perform_waveform_cache.Min()[g_wf_job.col]
				= static_cast<int16_t>(g_wf_job.minv * g_wf_job.scale);
			perform_waveform_cache.Max()[g_wf_job.col]
				= static_cast<int16_t>(g_wf_job.maxv * g_wf_job.scale);
			g_wf_job.col++;
			if (g_wf_job.col >= g_wf_job.columns)
			{
				g_wf_job.active = false;
				waveform_ready = true;
				waveform_dirty = true;
				return;
			}
			const size_t start = g_wf_job.col * g_wf_job.step;
			const size_t end = (g_wf_job.col == g_wf_job.columns - 1)
				? g_wf_job.frames
				: (start + g_wf_job.step);
			g_wf_job.idx = start;
			g_wf_job.end = end;
			g_wf_job.minv = 1.0f;
			g_wf_job.maxv = -1.0f;
			continue;
		}
		const float s = static_cast<float>(sample_buffer_l[g_wf_job.idx]) * kSampleScale;
		if (s < g_wf_job.minv)
		{
			g_wf_job.minv = s;
		}
		if (s > g_wf_job.maxv)
		{
			g_wf_job.maxv = s;
		}
		g_wf_job.idx++;
		budget_samples--;
	}
}

static void FileListJobCancel()
{
	g_list_job = {};
}

static uint16_t g_scan_cookie = 1;

static void FileListJobStart(bool wav_only)
{
	FileListJobCancel();
	g_list_job.active = true;
	g_list_job.done = false;
	g_list_job.wav_only = wav_only;
	g_list_job.count = 0;
	g_list_job.cookie = g_scan_cookie++;
	wav_file_count = 0;
	load_selected = 0;
	load_scroll = 0;

	StorageService::Op op = {};
	op.kind = StorageService::OpKind::ScanDir;
	CopyString(op.path, storage.GetSdPath(), sizeof(op.path));
	op.max_entries = static_cast<uint16_t>(kMaxWavFiles);
	op.cookie = g_list_job.cookie;
	op.wav_only = wav_only;
	if (!storage.Enqueue(op))
	{
		g_list_job.active = false;
		g_list_job.done = true;
	}
}

void FinalizeFileList()
{
	wav_file_count = g_list_job.count;
	if (wav_file_count <= 0)
	{
		load_selected = 0;
		load_scroll = 0;
		return;
	}
	if (load_selected >= wav_file_count)
	{
		load_selected = wav_file_count - 1;
	}
	if (load_selected < load_scroll)
	{
		load_scroll = load_selected;
	}
	else if (load_selected >= load_scroll + LoadVisibleLines())
	{
		load_scroll = load_selected - (LoadVisibleLines() - 1);
	}
	if (load_scroll < 0)
	{
		load_scroll = 0;
	}
	const int32_t max_top = wav_file_count - LoadVisibleLines();
	if (max_top < 0)
	{
		load_scroll = 0;
	}
	else if (load_scroll > max_top)
	{
		load_scroll = max_top;
	}
	request_delete_redraw = true;
	RequestDisplayUpdate();
}

static void FileListJobTick(uint32_t budget_entries)
{
	(void)budget_entries;
}

void InitSmoothers()
{
	sm_amp_attack.Init(amp_attack, 20.0f, kUiTickHz);
	sm_amp_decay.Init(amp_decay, 20.0f, kUiTickHz);
	sm_amp_sustain.Init(amp_sustain, 20.0f, kUiTickHz);
	sm_amp_release.Init(amp_release, 20.0f, kUiTickHz);
	sm_flt_cutoff.Init(flt_cutoff, 20.0f, kUiTickHz);
	sm_flt_res.Init(flt_res, 20.0f, kUiTickHz);
	sm_fx_s_wet.Init(fx_s_wet, 80.0f, kUiTickHz);
	sm_sat_drive.Init(sat_drive, 30.0f, kUiTickHz);
	sm_sat_tape_bump.Init(sat_tape_bump, 30.0f, kUiTickHz);
	sm_sat_bit_reso.Init(sat_bit_reso, 30.0f, kUiTickHz);
	sm_sat_bit_smpl.Init(sat_bit_smpl, 30.0f, kUiTickHz);
	sm_fx_c_wet.Init(fx_c_wet, 80.0f, kUiTickHz);
	sm_mod_depth.Init(mod_depth, 40.0f, kUiTickHz);
	sm_chorus_rate.Init(chorus_rate, 40.0f, kUiTickHz);
	sm_chorus_wow.Init(chorus_wow, 40.0f, kUiTickHz);
	sm_tape_rate.Init(tape_rate, 40.0f, kUiTickHz);
	sm_delay_wet.Init(delay_wet, 80.0f, kUiTickHz);
	sm_delay_time.Init(delay_time, 60.0f, kUiTickHz);
	sm_delay_feedback.Init(delay_feedback, 60.0f, kUiTickHz);
	sm_delay_spread.Init(delay_spread, 60.0f, kUiTickHz);
	sm_delay_freeze.Init(delay_freeze, 0.0f, kUiTickHz);
	sm_reverb_wet.Init(reverb_wet, 80.0f, kUiTickHz);
	sm_reverb_pre.Init(reverb_pre, 60.0f, kUiTickHz);
	sm_reverb_damp.Init(reverb_damp, 60.0f, kUiTickHz);
	sm_reverb_decay.Init(reverb_decay, 80.0f, kUiTickHz);
}

void UpdateSmoothedParamsPerTick()
{
	sm_amp_attack.SetTarget(amp_attack);
	sm_amp_decay.SetTarget(amp_decay);
	sm_amp_sustain.SetTarget(amp_sustain);
	sm_amp_release.SetTarget(amp_release);
	sm_flt_cutoff.SetTarget(flt_cutoff);
	sm_flt_res.SetTarget(flt_res);
	sm_fx_s_wet.SetTarget(fx_s_wet);
	sm_sat_drive.SetTarget(sat_drive);
	sm_sat_tape_bump.SetTarget(sat_tape_bump);
	sm_sat_bit_reso.SetTarget(sat_bit_reso);
	sm_sat_bit_smpl.SetTarget(sat_bit_smpl);
	sm_fx_c_wet.SetTarget(fx_c_wet);
	sm_mod_depth.SetTarget(mod_depth);
	sm_chorus_rate.SetTarget(chorus_rate);
	sm_chorus_wow.SetTarget(chorus_wow);
	sm_tape_rate.SetTarget(tape_rate);
	sm_delay_wet.SetTarget(delay_wet);
	sm_delay_time.SetTarget(delay_time);
	sm_delay_feedback.SetTarget(delay_feedback);
	sm_delay_spread.SetTarget(delay_spread);
	sm_delay_freeze.SetTarget(delay_freeze);
	sm_reverb_wet.SetTarget(reverb_wet);
	sm_reverb_pre.SetTarget(reverb_pre);
	sm_reverb_damp.SetTarget(reverb_damp);
	sm_reverb_decay.SetTarget(reverb_decay);

	AudioParams p = {};
	p.amp_attack = sm_amp_attack.Process();
	p.amp_decay = sm_amp_decay.Process();
	p.amp_sustain = sm_amp_sustain.Process();
	p.amp_release = sm_amp_release.Process();
	p.flt_cutoff = sm_flt_cutoff.Process();
	p.flt_res = sm_flt_res.Process();
	p.fx_s_wet = sm_fx_s_wet.Process();
	p.sat_drive = sm_sat_drive.Process();
	p.sat_tape_bump = sm_sat_tape_bump.Process();
	p.sat_bit_reso = sm_sat_bit_reso.Process();
	p.sat_bit_smpl = sm_sat_bit_smpl.Process();
	p.fx_c_wet = sm_fx_c_wet.Process();
	p.mod_depth = sm_mod_depth.Process();
	p.chorus_rate = sm_chorus_rate.Process();
	p.chorus_wow = sm_chorus_wow.Process();
	p.tape_rate = sm_tape_rate.Process();
	p.delay_wet = sm_delay_wet.Process();
	p.delay_time = sm_delay_time.Process();
	p.delay_feedback = sm_delay_feedback.Process();
	p.delay_spread = sm_delay_spread.Process();
	p.delay_freeze = sm_delay_freeze.Process();
	p.reverb_wet = sm_reverb_wet.Process();
	p.reverb_pre = sm_reverb_pre.Process();
	p.reverb_damp = sm_reverb_damp.Process();
	p.reverb_decay = sm_reverb_decay.Process();
	p.playback_reverse = playback_reverse;

	PublishAudioParamsFromUi(p);
	{
		static uint32_t next_fx_map_ms = 0;
		static uint32_t next_audio_map_ms = 0;
		const uint32_t now = System::GetNow();
		if (fx_params_dirty && (int32_t)(now - next_fx_map_ms) >= 0)
		{
			BuildAndPublishFxParamsAudio(p, hw.AudioSampleRate());
			fx_params_dirty = false;
			next_fx_map_ms = now + kFxMapIntervalMs;
		}
		if (audio_params_dirty && (int32_t)(now - next_audio_map_ms) >= 0)
		{
			BuildAndPublishAudioParamsAudio(p, hw.AudioSampleRate());
			audio_params_dirty = false;
			next_audio_map_ms = now + kFxMapIntervalMs;
		}
	}

	if (!sm_fx_s_wet.IsNearTarget()
		|| !sm_sat_drive.IsNearTarget()
		|| !sm_sat_tape_bump.IsNearTarget()
		|| !sm_sat_bit_reso.IsNearTarget()
		|| !sm_sat_bit_smpl.IsNearTarget()
		|| !sm_fx_c_wet.IsNearTarget()
		|| !sm_mod_depth.IsNearTarget()
		|| !sm_chorus_rate.IsNearTarget()
		|| !sm_chorus_wow.IsNearTarget()
		|| !sm_tape_rate.IsNearTarget()
		|| !sm_delay_wet.IsNearTarget()
		|| !sm_delay_time.IsNearTarget()
		|| !sm_delay_feedback.IsNearTarget()
		|| !sm_delay_spread.IsNearTarget()
		|| !sm_delay_freeze.IsNearTarget()
		|| !sm_reverb_wet.IsNearTarget()
		|| !sm_reverb_pre.IsNearTarget()
		|| !sm_reverb_damp.IsNearTarget()
		|| !sm_reverb_decay.IsNearTarget())
	{
		fx_params_dirty = true;
	}
	if (!sm_amp_attack.IsNearTarget()
		|| !sm_amp_release.IsNearTarget()
		|| !sm_flt_cutoff.IsNearTarget()
		|| !sm_flt_res.IsNearTarget())
	{
		audio_params_dirty = true;
	}
}

void UpdateDelaySlewCoeffs()
{
	const float out_sr = hw.AudioSampleRate();
	const float dt = static_cast<float>(kAudioBlockSize) / out_sr;
	const float time_tau = kDelayTimeSlewMs * 0.001f;
	const float param_tau = kDelayParamSlewMs * 0.001f;
	const float time_alpha = (time_tau > 0.0f)
		? (1.0f - expf(-dt / time_tau))
		: 1.0f;
	const float param_alpha = (param_tau > 0.0f)
		? (1.0f - expf(-dt / param_tau))
		: 1.0f;
	{
		daisy::ScopedIrqBlocker irq;
		g_delay_time_alpha = time_alpha;
		g_delay_param_alpha = param_alpha;
	}
}

void JobCancel()
{
	if (!g_job.active)
	{
		return;
	}
	if (g_job.type == JobType::WaveformBuild)
	{
		WaveformJobCancel();
	}
	else if (g_job.type == JobType::FileListScan)
	{
		FileListJobCancel();
	}
	g_job = {};
}

static bool JobCanPreempt(uint8_t priority)
{
	if (!g_job.active)
	{
		return true;
	}
	return priority >= g_job.priority;
}

void JobStartWaveform(SampleContext ctx, bool foreground)
{
	const uint8_t priority = foreground ? 2 : 1;
	if (!JobCanPreempt(priority))
	{
		return;
	}
	JobCancel();
	g_job.type = JobType::WaveformBuild;
	g_job.active = true;
	g_job.foreground = foreground;
	g_job.priority = priority;
	g_job.progress = 0;
	g_job.last_reported = 0;
	WaveformJobStart(ctx);
}

void JobStartFileList(const char* path, bool wav_only, bool foreground)
{
	(void)path;
	const uint8_t priority = foreground ? 2 : 1;
	if (!JobCanPreempt(priority))
	{
		return;
	}
	JobCancel();
	g_job.type = JobType::FileListScan;
	g_job.active = true;
	g_job.foreground = foreground;
	g_job.priority = priority;
	g_job.progress = 0;
	g_job.last_reported = 0;
	FileListJobStart(wav_only);
}

void JobTick()
{
	if (!g_job.active)
	{
		return;
	}
	const bool allow_now = (g_job.type == JobType::FileListScan)
		? FileListAllowedNow()
		: JobsAllowedNow();
	if (!allow_now)
	{
		if (!g_job.foreground)
		{
			JobCancel();
		}
		return;
	}
	constexpr uint32_t kWaveformBudget = 8192;
	constexpr uint32_t kListBudget = 6;
	if (g_job.type == JobType::WaveformBuild)
	{
		WaveformJobTick(kWaveformBudget);
		const uint32_t total = static_cast<uint32_t>(g_wf_job.frames);
		uint32_t done = 0;
		if (total > 0)
		{
			done = static_cast<uint32_t>(g_wf_job.col * g_wf_job.step + g_wf_job.idx);
			if (done > total) done = total;
			g_job.progress = (done * 1000U) / total;
		}
		if (!g_wf_job.active)
		{
			g_job = {};
		}
	}
	else if (g_job.type == JobType::FileListScan)
	{
		FileListJobTick(kListBudget);
		const uint32_t done = static_cast<uint32_t>(wav_file_count);
		const uint32_t total = kMaxWavFiles;
		g_job.progress = (done * 1000U) / total;
		if (!g_list_job.active)
		{
			g_job = {};
		}
	}
	if (g_job.foreground && g_job.progress >= g_job.last_reported + 10)
	{
		g_job.last_reported = g_job.progress;
		RequestDisplayUpdate();
	}
}

void PublishRuntimeFromUi();
void PublishFxChainFromUi();
#if STORAGE_SERVICE_PREVIEW_STREAM
void PublishPreviewControlFromUi();
#endif

void UpdateTrimFrames()
{
	if(sample_length < 2)
	{
		sample_play_start = 0;
		sample_play_end   = sample_length;
		PublishRuntimeFromUi();
		return;
	}

	if(trim_start < 0.0f) trim_start = 0.0f;
	if(trim_end   > 1.0f) trim_end   = 1.0f;

	const float min_norm = 2.0f / (float)sample_length;
	if(trim_end - trim_start < min_norm)
	{
		trim_end = trim_start + min_norm;
		if(trim_end > 1.0f)
		{
			trim_end = 1.0f;
			trim_start = trim_end - min_norm;
		}
	}

	snap_start_frame = (uint32_t)(trim_start * sample_length);
	snap_end_frame   = (uint32_t)(trim_end   * sample_length);

	if(snap_end_frame <= snap_start_frame)
		snap_end_frame = snap_start_frame + 2;

	sample_play_start = snap_start_frame;
	sample_play_end   = snap_end_frame;
	PublishRuntimeFromUi();
}

void AdjustTrimNormalized(int32_t start_delta, int32_t end_delta, bool fine)
{
	if(sample_length < 2)
		return;

	const float base_step = fine ? (1.0f / 64.0f) : (1.0f / 32.0f);

	auto step = [&](int d)
	{
		int mag = abs(d);
		if(mag < 1) mag = 1;
		int log2 = 0;
		while(mag > 1) { mag >>= 1; ++log2; }
		return base_step * (1 << log2);
	};

	if(start_delta != 0)
		trim_start += start_delta * step(start_delta);

	if(end_delta != 0)
		trim_end   += end_delta   * step(end_delta);

	UpdateTrimFrames();
	waveform_dirty = true;
	request_length_redraw = true;
}

bool IsPerformUiMode(UiMode mode)
{
	return (mode == UiMode::Perform);
}



void PublishRuntimeFromUi()
{
	const uint8_t next = static_cast<uint8_t>(g_rt_pub_idx ^ 1u);
	SampleRuntime rt;
	rt.l = sample_buffer_l;
	rt.r = sample_buffer_r;
	rt.length = sample_length;
	rt.play_start = sample_play_start;
	rt.play_end = sample_play_end;
	rt.rate = sample_rate;
	rt.channels = sample_channels;
	rt.loaded = sample_loaded;
	g_rt_buf[next] = rt;
	{
		daisy::ScopedIrqBlocker irq;
		g_rt_pub_idx = next;
		g_audio_cmd |= kCmdCommitRuntime;
	}
}

void PublishFxChainFromUi()
{
	const uint8_t next = static_cast<uint8_t>(g_fx_chain_pub_idx ^ 1u);
	FxChainRuntime rt = {};
	for (int i = 0; i < kPerformFaderCount; ++i)
	{
		rt.order[i] = static_cast<uint8_t>(fx_chain_order[i]);
	}
	rt.count = kPerformFaderCount;
	rt.paused = fx_chain_paused;
	rt.pause_pending = fx_chain_pause_pending;
	rt.fade_gain = fx_chain_fade_gain;
	rt.fade_target = fx_chain_fade_target;
	rt.fade_samples_left = fx_chain_fade_samples_left;
	g_fx_chain_buf[next] = rt;
	{
		daisy::ScopedIrqBlocker irq;
		g_fx_chain_pub_idx = next;
		g_audio_cmd |= kCmdCommitFxChain;
	}
}

#if STORAGE_SERVICE_PREVIEW_STREAM
void PublishPreviewControlFromUi()
{
	const uint8_t next = static_cast<uint8_t>(g_preview_pub_idx ^ 1u);
	PreviewControl ctl = {};
	ctl.l = preview_buffer;
	ctl.r = nullptr;
	ctl.length = kPreviewBufferFrames;
	ctl.rate = preview_rate;
	ctl.gain = 1.0f;
	ctl.mono = true;
	g_preview_ctl_buf[next] = ctl;
	{
		daisy::ScopedIrqBlocker irq;
		g_preview_pub_idx = next;
		g_audio_cmd |= kCmdCommitPreview;
	}
}
#endif

void PublishAudioParamsFromUi(const AudioParams& p)
{
	const uint8_t next = static_cast<uint8_t>(g_audio_params_pub_idx ^ 1u);
	g_audio_params_buf[next] = p;
	{
		daisy::ScopedIrqBlocker irq;
		g_audio_params_pub_idx = next;
		g_audio_cmd |= kCmdCommitAudioParams;
	}
}

void AudioUiResetLiveWaveform(AudioUiState& uiw)
{
	for (int i = 0; i < kWaveCols; ++i)
	{
		uiw.live_wave.minv[i] = 0;
		uiw.live_wave.maxv[i] = 0;
	}
	uiw.live_wave.last_col = -1;
	uiw.live_wave.peak = 1;
	uiw.live_wave.dirty = true;
}

static void CapturePerformState(PerformState& state)
{
	state.perform_index = perform_index;
	state.fx_fader_index = fx_fader_index;
	state.amp_fader_index = amp_fader_index;
	state.flt_fader_index = flt_fader_index;
	state.fx_detail_index = fx_detail_index;
	state.fx_detail_param_index = fx_detail_param_index;
	state.fx_window_active = fx_window_active;
	state.amp_window_active = amp_window_active;
	state.flt_window_active = flt_window_active;
	for (int i = 0; i < kPerformFaderCount; ++i)
	{
		state.fx_chain_order[i] = fx_chain_order[i];
	}
	state.amp_attack = amp_attack;
	state.amp_decay = amp_decay;
	state.amp_sustain = amp_sustain;
	state.amp_release = amp_release;
	state.flt_cutoff = flt_cutoff;
	state.flt_res = flt_res;
	state.fx_s_wet = fx_s_wet;
	state.sat_tape_bump = sat_tape_bump;
	state.sat_bit_reso = sat_bit_reso;
	state.sat_bit_smpl = sat_bit_smpl;
	state.sat_mode = sat_mode;
	state.fx_c_wet = fx_c_wet;
	state.chorus_rate = chorus_rate;
	state.chorus_wow = chorus_wow;
	state.tape_rate = tape_rate;
	state.chorus_mode = chorus_mode;
	state.delay_wet = delay_wet;
	state.delay_time = delay_time;
	state.delay_feedback = delay_feedback;
	state.delay_spread = delay_spread;
	state.delay_freeze = delay_freeze;
	state.reverb_wet = reverb_wet;
	state.reverb_pre = reverb_pre;
	state.reverb_damp = reverb_damp;
	state.reverb_decay = reverb_decay;
	state.sat_params_initialized = sat_params_initialized;
	state.reverb_params_initialized = reverb_params_initialized;
	state.delay_params_initialized = delay_params_initialized;
	state.mod_params_initialized = mod_params_initialized;
}

static void ApplyPerformState(const PerformState& state)
{
	perform_index = state.perform_index;
	fx_fader_index = state.fx_fader_index;
	amp_fader_index = state.amp_fader_index;
	flt_fader_index = state.flt_fader_index;
	fx_detail_index = state.fx_detail_index;
	fx_detail_param_index = state.fx_detail_param_index;
	fx_window_active = state.fx_window_active;
	amp_window_active = state.amp_window_active;
	flt_window_active = state.flt_window_active;
	for (int i = 0; i < kPerformFaderCount; ++i)
	{
		fx_chain_order[i] = state.fx_chain_order[i];
	}
	amp_attack = state.amp_attack;
	amp_decay = state.amp_decay;
	amp_sustain = state.amp_sustain;
	amp_release = state.amp_release;
	flt_cutoff = state.flt_cutoff;
	flt_res = state.flt_res;
	fx_s_wet = state.fx_s_wet;
	sat_tape_bump = state.sat_tape_bump;
	sat_bit_reso = state.sat_bit_reso;
	sat_bit_smpl = state.sat_bit_smpl;
	sat_mode = state.sat_mode;
	fx_c_wet = state.fx_c_wet;
	chorus_rate = state.chorus_rate;
	chorus_wow = state.chorus_wow;
	tape_rate = state.tape_rate;
	chorus_mode = state.chorus_mode;
	delay_wet = state.delay_wet;
	delay_time = state.delay_time;
	delay_feedback = state.delay_feedback;
	delay_spread = state.delay_spread;
	delay_freeze = state.delay_freeze;
	reverb_wet = state.reverb_wet;
	reverb_pre = state.reverb_pre;
	reverb_damp = state.reverb_damp;
	reverb_decay = state.reverb_decay;
	sat_params_initialized = state.sat_params_initialized;
	reverb_params_initialized = state.reverb_params_initialized;
	delay_params_initialized = state.delay_params_initialized;
	mod_params_initialized = state.mod_params_initialized;
	fx_params_dirty = true;
}

static FxContext fx_context = FxContext::Perform;

static void SaveFxContext()
{
	CapturePerformState(main_perform_state);
}

void SetFxContext(FxContext ctx, int32_t track)
{
	(void)track;
	if (fx_context == FxContext::Perform && ctx == FxContext::Perform)
	{
		return;
	}
	SaveFxContext();
	fx_context = FxContext::Perform;
	ApplyPerformState(main_perform_state);
}


void ApplyLoadedSampleFade(size_t length, uint32_t rate)
{
	if (length == 0 || rate == 0)
	{
		return;
	}
	size_t fade_len = static_cast<size_t>(static_cast<float>(rate) * 0.005f + 0.5f);
	if (fade_len > (length / 2))
	{
		fade_len = length / 2;
	}
	if (fade_len == 0)
	{
		return;
	}
	if (fade_len == 1)
	{
		sample_buffer_l[0] = 0;
		sample_buffer_r[0] = 0;
		sample_buffer_l[length - 1] = 0;
		sample_buffer_r[length - 1] = 0;
		return;
	}
	const float denom = static_cast<float>(fade_len - 1);
	for (size_t i = 0; i < fade_len; ++i)
	{
		const float fade_in = static_cast<float>(i) / denom;
		const float fade_out = static_cast<float>(fade_len - 1 - i) / denom;
		const size_t tail_idx = length - fade_len + i;
		sample_buffer_l[i] = static_cast<int16_t>(static_cast<float>(sample_buffer_l[i]) * fade_in);
		sample_buffer_r[i] = static_cast<int16_t>(static_cast<float>(sample_buffer_r[i]) * fade_in);
		sample_buffer_l[tail_idx] = static_cast<int16_t>(static_cast<float>(sample_buffer_l[tail_idx]) * fade_out);
		sample_buffer_r[tail_idx] = static_cast<int16_t>(static_cast<float>(sample_buffer_r[tail_idx]) * fade_out);
	}
}

static bool LoadSampleFromPath(const char* path)
{
	RequestPlaybackStopAll();
	RequestAudioCmd(kCmdAllNotesOff);
	sample_loaded = false;
	perform_attack_norm = 0.0f;
	perform_release_norm = 0.0f;
	sample_length = 0;
	sample_channels = 1;
	trim_start = 0.0f;
	trim_end = 1.0f;
	PublishRuntimeFromUi();

	if (load_in_progress)
	{
		return false;
	}
	void* sample_ptr = nullptr;
	if (!AllocatePerformSample(kPerformSampleRamBudgetBytes, &sample_ptr))
	{
		load_fail_budget_count++;
		return false;
	}
	StorageService::Op op = {};
	op.kind = StorageService::OpKind::LoadStart;
	CopyString(op.path, path, sizeof(op.path));
	op.dst_l = sample_buffer_l;
	op.dst_r = sample_buffer_r;
	op.max_frames = kMaxSampleFrames;
	op.cookie = load_cookie_next++;
	load_cookie_active = op.cookie;
	if (!storage.Enqueue(op))
	{
		loader_state = LoaderState::Failed;
		load_fail_io_count++;
		FreePerformSample();
		return false;
	}
	load_in_progress = true;
	loader_state = LoaderState::Requested;
	return true;
}

bool LoadSampleAtIndex(int32_t index)
{
	MountSd();
	if (!sd_mounted)
	{
		return false;
	}
	if (index < 0 || index >= wav_file_count)
	{
		return false;
	}
	char path[64];
	BuildFilePath(wav_files[index], path, sizeof(path));
	CopyString(loaded_sample_name, wav_files[index], kMaxWavNameLen);
	if (load_context == LoadContext::Edt)
	{
		ClearWaveformCache();
		waveform_ready = false;
		waveform_dirty = true;
	}
	return LoadSampleFromPath(path);
}

void StopPreview()
{
	preview_hold = false;
	preview_index = -1;
#if STORAGE_SERVICE_PREVIEW_STREAM
	preview_stream_cookie_active = 0;
	preview_pending_start = false;
	preview_pending_start_ms = 0;
	preview_pp_ready[0] = 0;
	preview_pp_ready[1] = 0;
	preview_pp_active = 0;
	preview_pp_pos = 0;
#endif
	{
		daisy::ScopedIrqBlocker irq;
		preview_write_index = 0;
	}
	RequestAudioCmd(kCmdPreviewStop);
#if STORAGE_SERVICE_PREVIEW_STREAM
	StorageService::Op op = {};
	op.kind = StorageService::OpKind::PreviewClose;
	storage.Enqueue(op);
#endif
}

bool BeginPreviewAtIndex(int32_t index)
{
	MountSd();
	if (!sd_mounted)
	{
		return false;
	}
	if (index < 0 || index >= wav_file_count)
	{
		return false;
	}
	char path[64];
	BuildFilePath(wav_files[index], path, sizeof(path));

#if STORAGE_SERVICE_PREVIEW_STREAM
	if (preview_index == index && preview_stream_cookie_active != 0)
	{
		return true;
	}
	StorageService::Op op = {};
	op.kind = StorageService::OpKind::PreviewOpen;
	CopyString(op.path, path, sizeof(op.path));
	op.cookie = preview_stream_cookie++;
	preview_stream_cookie_active = op.cookie;
	preview_index = index;
	preview_pending_start = false;
	preview_pending_start_ms = System::GetNow();
	if (!storage.Enqueue(op))
	{
		return false;
	}
	return true;
#else
	(void)path;
	return false;
#endif
}

#if !STORAGE_SERVICE_PREVIEW_STREAM
size_t PreviewAvailableFrames(size_t read_idx, size_t write_idx)
{
	if (write_idx >= read_idx)
	{
		return write_idx - read_idx;
	}
	return (kPreviewBufferFrames - read_idx) + write_idx;
}
#endif

#if !STORAGE_SERVICE_PREVIEW_STREAM
static size_t PreviewFreeFrames(size_t read_idx, size_t write_idx) __attribute__((unused));
static size_t PreviewFreeFrames(size_t read_idx, size_t write_idx)
{
	const size_t used = PreviewAvailableFrames(read_idx, write_idx);
	if (used >= kPreviewBufferFrames - 1)
	{
		return 0;
	}
	return (kPreviewBufferFrames - 1) - used;
}
#endif


const AudioUiState& GetAudioUiStateSnapshot(uint8_t& idx)
{
	{
		daisy::ScopedIrqBlocker irq;
		idx = g_audio_ui_state_idx;
	}
	return g_audio_ui_state_buf[idx];
}

void FillPreviewBuffer()
{
#if STORAGE_SERVICE_PREVIEW_STREAM
	return;
#endif
}

bool DeleteFileAtIndex(int32_t index)
{
	MountSd();
	if (!sd_mounted)
	{
		return false;
	}
	if (delete_in_progress)
	{
		return false;
	}
	if (index < 0 || index >= wav_file_count)
	{
		return false;
	}
	char path[64];
	BuildFilePath(wav_files[index], path, sizeof(path));
	StorageService::Op op = {};
	op.kind = StorageService::OpKind::DeleteFile;
	CopyString(op.path, path, sizeof(op.path));
	op.cookie = delete_cookie_next++;
	delete_cookie_active = op.cookie;
	if (!storage.Enqueue(op))
	{
		delete_cookie_active = 0;
		return false;
	}
	delete_in_progress = true;
	return true;
}

void DrawTinyString(const char* str, int x, int y, bool on);
static int TinyStringWidth(const char* str);

constexpr int kIconW = 61;
constexpr int kIconH = 29;
constexpr int kIconStride = 8;

static const uint8_t kIconLoadDisk61x29[kIconH * kIconStride] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0,
	0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
	0x60, 0x00, 0x3f, 0xff, 0xf8, 0x00, 0x00, 0x30,
	0x60, 0x00, 0x44, 0x01, 0x24, 0x00, 0x00, 0x30,
	0x60, 0x00, 0x84, 0x71, 0x24, 0x00, 0x00, 0x30,
	0x60, 0x21, 0x04, 0x89, 0x3c, 0x04, 0x00, 0x30,
	0x60, 0x61, 0x04, 0xa9, 0x04, 0x0c, 0x00, 0x30,
	0x60, 0xe1, 0x04, 0x89, 0x04, 0x1c, 0x00, 0x30,
	0x61, 0xe1, 0x04, 0x71, 0x04, 0x3c, 0x00, 0x30,
	0x63, 0xe1, 0x03, 0xfe, 0x04, 0x7c, 0x00, 0x30,
	0x67, 0xe1, 0x00, 0x00, 0x04, 0xfc, 0x00, 0x30,
	0x6f, 0xff, 0x3f, 0xff, 0xe5, 0xff, 0xff, 0xf0,
	0x7f, 0xff, 0x40, 0x00, 0x17, 0xff, 0xff, 0xf0,
	0x7f, 0xff, 0x4f, 0xff, 0x97, 0xff, 0xff, 0xf0,
	0x7f, 0xff, 0x40, 0x00, 0x17, 0xff, 0xff, 0xf0,
	0x7f, 0xff, 0x43, 0xfe, 0x17, 0xff, 0xff, 0xf0,
	0x6f, 0xff, 0x40, 0x00, 0x15, 0xff, 0xff, 0xf0,
	0x67, 0xe1, 0x40, 0x00, 0x14, 0xfc, 0x00, 0x30,
	0x63, 0xe1, 0x41, 0xfc, 0x14, 0x7c, 0x00, 0x30,
	0x61, 0xe1, 0x40, 0x00, 0x14, 0x3c, 0x00, 0x30,
	0x60, 0xe1, 0x40, 0x70, 0x14, 0x1c, 0x00, 0x30,
	0x60, 0x61, 0x40, 0x00, 0x14, 0x0c, 0x00, 0x30,
	0x60, 0x21, 0x40, 0x00, 0x14, 0x04, 0x00, 0x30,
	0x60, 0x00, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x30,
	0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
	0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
	0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t kIconRecordTape61x29[kIconH * kIconStride] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
	0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe8,
	0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28,
	0xa8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa8,
	0xa4, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xc1, 0x28,
	0xa2, 0x38, 0x00, 0x00, 0x00, 0x00, 0xe2, 0x28,
	0xa1, 0x37, 0xff, 0xff, 0xff, 0xff, 0x64, 0x28,
	0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x28,
	0xa0, 0x28, 0x7f, 0xff, 0xff, 0xf0, 0xa0, 0x28,
	0xa0, 0x28, 0xff, 0xff, 0xff, 0xf8, 0xa0, 0x28,
	0xa0, 0x29, 0xf8, 0x80, 0x08, 0xfc, 0xa0, 0x28,
	0xa0, 0x29, 0xf7, 0x87, 0xef, 0x7c, 0xa0, 0x28,
	0xa0, 0x29, 0xed, 0x83, 0xcd, 0xbc, 0xa0, 0x28,
	0xa0, 0x29, 0xe8, 0xb3, 0xa8, 0xbc, 0xa0, 0x28,
	0xa0, 0x29, 0xed, 0xb3, 0xcd, 0xbc, 0xa0, 0x28,
	0xa0, 0x29, 0xf7, 0x87, 0xef, 0x7c, 0xa0, 0x28,
	0xa0, 0x29, 0xf8, 0x80, 0x08, 0xfc, 0xa0, 0x28,
	0xa0, 0x29, 0xff, 0xff, 0xff, 0xfc, 0xa0, 0x28,
	0xa0, 0x28, 0xfc, 0x00, 0x01, 0xf8, 0xa0, 0x28,
	0xa0, 0x2c, 0x33, 0xff, 0xfe, 0x61, 0xa0, 0x28,
	0xa1, 0x37, 0x77, 0xc0, 0x1f, 0x77, 0x64, 0x28,
	0xa2, 0x38, 0x6f, 0xc0, 0x1f, 0xb0, 0xe2, 0x28,
	0xa4, 0x1f, 0xdf, 0xff, 0xff, 0xdf, 0xc1, 0x28,
	0xa8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa8,
	0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28,
	0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe8,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8,
};

static const uint8_t kIconPerformMpc61x29[kIconH * kIconStride] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
	0x87, 0xbd, 0xef, 0x7b, 0xcf, 0xff, 0xff, 0xc8,
	0x87, 0xbd, 0xef, 0x7b, 0xce, 0x0f, 0xff, 0xc8,
	0x87, 0xbd, 0xef, 0x7b, 0xce, 0xef, 0xf0, 0xc8,
	0x87, 0xbd, 0xef, 0x7b, 0xce, 0xef, 0xf6, 0xc8,
	0x80, 0x00, 0x00, 0x00, 0x0e, 0xef, 0xf6, 0xc8,
	0x87, 0xbd, 0xef, 0x7b, 0xce, 0xef, 0xf6, 0xc8,
	0x87, 0xbd, 0xef, 0x7b, 0xce, 0xee, 0x36, 0xc8,
	0x87, 0xbd, 0xef, 0x7b, 0xce, 0xee, 0xb6, 0xc8,
	0x87, 0xbd, 0xef, 0x7b, 0xce, 0xee, 0xb6, 0xc8,
	0x80, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x86, 0x08,
	0x80, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xc8,
	0x80, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xc8,
	0x89, 0x11, 0x11, 0x20, 0x00, 0x00, 0x00, 0x08,
	0x89, 0x11, 0x11, 0x20, 0x00, 0x00, 0x00, 0x08,
	0x89, 0x11, 0x11, 0x20, 0x00, 0x00, 0x00, 0x08,
	0x89, 0x11, 0x11, 0x20, 0xa0, 0x14, 0x02, 0x88,
	0x89, 0x11, 0x11, 0x21, 0xb0, 0x36, 0x06, 0xc8,
	0x8b, 0xbb, 0xbb, 0xa3, 0xb8, 0x77, 0x0e, 0xe8,
	0x8b, 0xbb, 0xbb, 0xa3, 0xb8, 0x77, 0x0e, 0xe8,
	0x8b, 0xbb, 0xbb, 0xa3, 0xf8, 0x7f, 0x0f, 0xe8,
	0x8b, 0xbb, 0xbb, 0xa1, 0xf0, 0x3e, 0x07, 0xc8,
	0x8f, 0xff, 0xff, 0xe0, 0xe0, 0x1c, 0x03, 0x88,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8,
};

void DrawBitmap1bpp(PodDisplay& disp,
						   int x,
						   int y,
						   int w,
						   int h,
						   int stride,
						   const uint8_t* data,
						   bool on)
{
	for (int row = 0; row < h; ++row)
	{
		const uint8_t* line = data + (row * stride);
		for (int col = 0; col < w; ++col)
		{
			const uint8_t byte = line[col >> 3];
			const int bit = 7 - (col & 7);
			if ((byte >> bit) & 0x1)
			{
				disp.DrawPixel(x + col, y + row, on);
			}
		}
	}
}

void DrawMenu(int32_t selected)
{
	auto clamp_i = [](int v, int lo, int hi) { return (v < lo) ? lo : (v > hi ? hi : v); };
	display.Fill(false);
	constexpr int kListLeftX = 2;
	constexpr int kListGapY = 6;
	const int text_h = Font5x7::H;
	int max_label_w = 0;
	for (int32_t i = 0; i < kMenuCount; ++i)
	{
		const int w = TinyStringWidth(kMenuLabels[i]);
		if (w > max_label_w)
		{
			max_label_w = w;
		}
	}
	const int total_h = (kMenuCount * text_h) + ((kMenuCount - 1) * kListGapY);
	const int start_y = (kDisplayH - total_h) / 2;
	const int list_w = max_label_w;
	const int icon_area_x = list_w + 4;
	const int icon_area_w = kDisplayW - icon_area_x;
	for (int32_t i = 0; i < kMenuCount; ++i)
	{
		const bool is_selected = (i == selected);
		const char* label = kMenuLabels[i];
		const int text_x = kListLeftX;
		const int text_y = start_y + i * (text_h + kListGapY);
		if (is_selected)
		{
			const int pad = 1;
			int rect_x0 = text_x - pad;
			int rect_y0 = text_y - pad;
			int rect_x1 = text_x + list_w + pad;
			int rect_y1 = text_y + text_h + pad;
			if (rect_x0 < 0) rect_x0 = 0;
			if (rect_y0 < 0) rect_y0 = 0;
			if (rect_x1 >= kDisplayW) rect_x1 = kDisplayW - 1;
			if (rect_y1 >= kDisplayH) rect_y1 = kDisplayH - 1;
			display.DrawRect(rect_x0,
							 rect_y0,
							 rect_x1,
							 rect_y1,
							 true,
							 true);
			DrawTinyString(label, text_x, text_y, false);
		}
		else
		{
			DrawTinyString(label, text_x, text_y, true);
		}

		if (is_selected)
		{
			const uint8_t* icon = nullptr;
			int icon_w = 0;
			int icon_h = 0;
			int icon_stride = 0;
			if (i == 0)
			{
				icon = kIconLoadDisk61x29;
				icon_w = kIconW;
				icon_h = kIconH;
				icon_stride = kIconStride;
			}
			else if (i == 1)
			{
				icon = kIconRecordTape61x29;
				icon_w = kIconW;
				icon_h = kIconH;
				icon_stride = kIconStride;
			}
			else if (i == 2)
			{
				icon = kIconPerformMpc61x29;
				icon_w = kIconW;
				icon_h = kIconH;
				icon_stride = kIconStride;
			}
			if (icon != nullptr && icon_area_w > icon_w)
			{
				const int icon_x = icon_area_x + (icon_area_w - icon_w) / 2;
				const int icon_y = (kDisplayH - icon_h) / 2;
				DrawBitmap1bpp(display,
							   icon_x,
							   icon_y,
							   icon_w,
							   icon_h,
							   icon_stride,
							   icon,
							   true);
			}
		}
	}
#if PERF_DIAGNOSTICS
	{
		const FontDef font = Font_6x8;
		char cpu_label[12];
		int cpu_pct = static_cast<int>(cpu_load_pct + 0.5f);
		cpu_pct = clamp_i(cpu_pct, 0, 100);
		snprintf(cpu_label, sizeof(cpu_label), "CPU %d%%", cpu_pct);
		const int text_w = static_cast<int>(StrLen(cpu_label)) * font.FontWidth;
		int x = kDisplayW - text_w - 1;
		if (x < 0)
		{
			x = 0;
		}
		const bool cpu_on = (selected != kPerformAmpIndex);
		display.SetCursor(x, 0);
		display.WriteString(cpu_label, font, cpu_on);
	}
#endif
	RequestDisplayUpdate();
}

void DrawScaledChar(char ch,
						   int x,
						   int y,
						   FontDef font,
						   int scale,
						   bool on)
{
	if (ch < 32 || ch > 126)
	{
		return;
	}
	const uint32_t base = static_cast<uint32_t>(ch - 32) * font.FontHeight;
	for (uint32_t row = 0; row < font.FontHeight; ++row)
	{
		const uint32_t bits = font.data[base + row];
		for (uint32_t col = 0; col < font.FontWidth; ++col)
		{
			const bool pixel_on = ((bits << col) & 0x8000) != 0;
			const bool draw_on = pixel_on ? on : !on;
			const int px = x + static_cast<int>(col * scale);
			const int py = y + static_cast<int>(row * scale);
			for (int dy = 0; dy < scale; ++dy)
			{
				for (int dx = 0; dx < scale; ++dx)
				{
					display.DrawPixel(px + dx, py + dy, draw_on);
				}
			}
		}
	}
}

void DrawScaledString(const char* str,
							 int x,
							 int y,
							 FontDef font,
							 int scale,
							 bool on,
							 int max_chars)
{
	const int char_width = font.FontWidth * scale;
	for (int i = 0; str[i] != '\0' && i < max_chars; ++i)
	{
		DrawScaledChar(str[i], x + i * char_width, y, font, scale, on);
	}
}

void DrawTinyString(const char* str, int x, int y, bool on)
{
	const int char_w = Font5x7::W + 1;
	for (int i = 0; str[i] != '\0'; ++i)
	{
		uint8_t rows[Font5x7::H] = {};
		Font5x7::GetGlyphRows(str[i], rows);
		for (int yy = 0; yy < Font5x7::H; ++yy)
		{
			const uint8_t row = rows[yy];
			for (int xx = 0; xx < Font5x7::W; ++xx)
			{
				if ((row >> (Font5x7::W - 1 - xx)) & 1)
				{
					const int px = x + i * char_w + xx;
					const int py = y + yy;
					if (px >= 0 && px < kDisplayW && py >= 0 && py < kDisplayH)
					{
						display.DrawPixel(px, py, on);
					}
				}
			}
		}
	}
}

void DrawTinyVerticalString(const char* str, int x, int y, int h, bool on)
{
	if (str == nullptr || str[0] == '\0')
	{
		return;
	}
	const int len = static_cast<int>(StrLen(str));
	constexpr int kLetterSpacing = 1;
	int glyph_h = Font5x7::H;
	int total_h = len * glyph_h + (len - 1) * kLetterSpacing;
	while (total_h > h && glyph_h > 1)
	{
		--glyph_h;
		total_h = len * glyph_h + (len - 1) * kLetterSpacing;
	}
	int start_y = y + (h - total_h) / 2;
	if (start_y < y)
	{
		start_y = y;
	}
	for (int i = 0; i < len; ++i)
	{
		uint8_t rows[Font5x7::H] = {};
		Font5x7::GetGlyphRows(str[i], rows);
		const int char_y = start_y + i * (glyph_h + kLetterSpacing);
		for (int yy = 0; yy < glyph_h; ++yy)
		{
			int src_row = 0;
			if (glyph_h == Font5x7::H)
			{
				src_row = yy;
			}
			else if (glyph_h == Font5x7::H - 1)
			{
				src_row = (yy < 2) ? yy : (yy + 1);
			}
			else
			{
				src_row = (yy * Font5x7::H) / glyph_h;
				if (src_row >= Font5x7::H)
				{
					src_row = Font5x7::H - 1;
				}
			}
			const uint8_t row = rows[src_row];
			for (int xx = 0; xx < Font5x7::W; ++xx)
			{
				if ((row >> (Font5x7::W - 1 - xx)) & 1)
				{
					const int px = x + xx;
					const int py = char_y + yy;
					if (px >= 0 && px < kDisplayW && py >= 0 && py < kDisplayH)
					{
						display.DrawPixel(px, py, on);
					}
				}
			}
		}
	}
}

void DrawTinyVerticalStringBold(const char* str, int x, int y, int h, bool on)
{
	DrawTinyVerticalString(str, x, y, h, on);
	DrawTinyVerticalString(str, x + 1, y, h, on);
}

static int TinyStringWidth(const char* str)
{
	if (str == nullptr || str[0] == '\0')
	{
		return 0;
	}
	const int char_w = Font5x7::W + 1;
	int count = 0;
	for (; str[count] != '\0'; ++count)
	{
	}
	return count * char_w - 1;
}

static const char* FxShortLabel(int32_t fx_index)
{
	switch (fx_index)
	{
		case kFxSatIndex: return "S";
		case kFxChorusIndex: return "M";
		case kFxDelayIndex: return "D";
		case kFxReverbIndex: return "R";
		default: return "?";
	}
}

float FxWetValue(int32_t fx_index)
{
	switch (fx_index)
	{
		case kFxSatIndex: return fx_s_wet;
		case kFxChorusIndex: return fx_c_wet;
		case kFxDelayIndex: return delay_wet;
		case kFxReverbIndex: return reverb_wet;
		default: return 0.0f;
	}
}

float FxWetStep(int32_t fx_index)
{
	switch (fx_index)
	{
		case kFxReverbIndex: return kReverbWetStep;
		case kFxSatIndex:
		case kFxChorusIndex:
		case kFxDelayIndex:
		default: return kDelayWetStep;
	}
}

volatile float* FxWetTarget(int32_t fx_index)
{
	switch (fx_index)
	{
		case kFxSatIndex: return &fx_s_wet;
		case kFxChorusIndex: return &fx_c_wet;
		case kFxDelayIndex: return &delay_wet;
		case kFxReverbIndex: return &reverb_wet;
		default: return &fx_s_wet;
	}
}

int ClampI(int v, int lo, int hi);

void DrawPerformScreen(int32_t selected,
							  bool fx_select_active,
							  int32_t fx_selected,
							  bool amp_select_active,
							  int32_t amp_selected,
							  bool flt_select_active,
							  int32_t flt_selected,
							  uint8_t redraw_mask)
{
	constexpr int kMarginX = 2;
	constexpr int kMarginY = 2;
	constexpr int kGapX = 2;
	constexpr int kGapY = 2;
	constexpr int kBoxW = (kDisplayW - (kMarginX * 2) - kGapX) / 2;
	constexpr int kBoxH = (kDisplayH - (kMarginY * 2) - kGapY) / 2;
	constexpr int kLabelPadX = 2;
	constexpr int kLabelPadY = 1;
	if (redraw_mask == 0x0F)
	{
		display.Fill(false);
	}

	struct Box
	{
		int x;
		int y;
		const char* label;
	};

	const Box boxes[] =
	{
		{ kMarginX, kMarginY, "EDT" },
		{ kMarginX + kBoxW + kGapX, kMarginY, "AMP" },
		{ kMarginX, kMarginY + kBoxH + kGapY, "FLT" },
		{ kMarginX + kBoxW + kGapX, kMarginY + kBoxH + kGapY, "FX" },
	};

	auto draw_faders = [&](const Box& box,
						   bool is_selected,
						   const char* const* labels,
						   const float* values,
						   int count,
						   bool select_active,
						   int32_t selected_index,
						   bool center_narrow)
	{
		const int label_y = box.y + kBoxH - kLabelPadY - Font5x7::H - 1;
		int line_top = box.y + kLabelPadY + 1;
		int line_bottom = label_y - 2;
		line_top += 1;
		line_bottom -= 1;
		if (line_bottom <= line_top)
		{
			return;
		}
		int fader_left = box.x + kLabelPadX + Font5x7::W + 8;
		int fader_right = box.x + kBoxW - 8;
		if (fader_right <= fader_left)
		{
			fader_left = box.x + kLabelPadX + Font5x7::W + 4;
			fader_right = box.x + kBoxW - 2;
		}
		if (center_narrow)
		{
			const int span = fader_right - fader_left;
			const int shrink = span / 4;
			if (span > 0 && (fader_right - shrink) > (fader_left + shrink))
			{
				fader_left += shrink;
				fader_right -= shrink;
			}
		}
		const int span_x = fader_right - fader_left;
		const int span_y = line_bottom - line_top;
		for (int f = 0; f < count; ++f)
		{
			int line_x = fader_left;
			if (count > 1 && span_x > 0)
			{
				line_x = fader_left + (span_x * f) / (count - 1);
			}
			const char* label = labels[f];
			const int label_w = TinyStringWidth(label);
			int label_x = line_x - (label_w / 2);
			if (label_x < box.x + 1)
			{
				label_x = box.x + 1;
			}
			if (label_x + label_w > box.x + kBoxW - 2)
			{
				label_x = box.x + kBoxW - 2 - label_w;
			}
			line_x = label_x + (label_w / 2);
			const bool fader_selected = select_active && (f == selected_index);
			const bool fill_on = !is_selected;
			bool line_on = !is_selected;
			if (fader_selected)
			{
				int rect_x0 = line_x - 4;
				int rect_x1 = line_x + 4;
				if (label_x - 1 < rect_x0)
				{
					rect_x0 = label_x - 1;
				}
				if (label_x + label_w + 1 > rect_x1)
				{
					rect_x1 = label_x + label_w + 1;
				}
				int rect_y0 = line_top - 1;
				int rect_y1 = label_y + Font5x7::H + 1;
				if (rect_x0 < box.x + 1)
				{
					rect_x0 = box.x + 1;
				}
				if (rect_x1 > box.x + kBoxW - 2)
				{
					rect_x1 = box.x + kBoxW - 2;
				}
				if (rect_y0 < box.y + 1)
				{
					rect_y0 = box.y + 1;
				}
				if (rect_y1 > box.y + kBoxH - 2)
				{
					rect_y1 = box.y + kBoxH - 2;
				}
				display.DrawRect(rect_x0, rect_y0, rect_x1, rect_y1, fill_on, true);
				line_on = !fill_on;
			}
			display.DrawLine(line_x, line_top, line_x, line_bottom, line_on);
			const float value = values[f];
			int tick_y = line_bottom - static_cast<int>(value * static_cast<float>(span_y) + 0.5f);
			const int tick_half = 3;
			int tick_x0 = line_x - tick_half;
			int tick_x1 = line_x + tick_half;
			if (tick_x0 < box.x + 1)
			{
				tick_x0 = box.x + 1;
			}
			if (tick_x1 > box.x + kBoxW - 2)
			{
				tick_x1 = box.x + kBoxW - 2;
			}
			display.DrawLine(tick_x0, tick_y, tick_x1, tick_y, line_on);
			if (label_x + label_w < box.x + kBoxW - 1)
			{
				DrawTinyString(label, label_x, label_y, line_on);
			}
		}
	};

	auto draw_edit_label = [&](const Box& box, bool is_selected)
	{
		const bool on = !is_selected;
		const char* line1 = "WAV";
		const char* line2 = "EDITOR";
		const int w1 = TinyStringWidth(line1);
		const int w2 = TinyStringWidth(line2);
		const int text_x1 = box.x + (kBoxW - w1) / 2;
		const int text_x2 = box.x + (kBoxW - w2) / 2;
		const int text_y = box.y + (kBoxH - (Font5x7::H * 2) - 2) / 2;
		DrawTinyString(line1, text_x1, text_y, on);
		DrawTinyString(line2, text_x2, text_y + Font5x7::H + 2, on);
	};

	for (int i = 0; i < kPerformBoxCount; ++i)
	{
		if ((redraw_mask & (1u << i)) == 0)
		{
			continue;
		}
		const auto& box = boxes[i];
		if (redraw_mask != 0x0F)
		{
			display.DrawRect(box.x,
							 box.y,
							 box.x + kBoxW - 1,
							 box.y + kBoxH - 1,
							 false,
							 true);
		}
		const bool is_selected = (i == selected);
		if (is_selected && kBoxW > 2 && kBoxH > 2)
		{
			display.DrawRect(box.x + 1,
							 box.y + 1,
							 box.x + kBoxW - 2,
							 box.y + kBoxH - 2,
							 true,
							 true);
		}
		if (i != kPerformEdtIndex)
		{
			DrawTinyVerticalStringBold(box.label,
									   box.x + kLabelPadX,
									   box.y + kLabelPadY,
									   kBoxH - (kLabelPadY * 2),
									   !is_selected);
		}
		if (i == kPerformEdtIndex)
		{
			draw_edit_label(box, is_selected);
		}
		if (i == kPerformAmpIndex)
		{
			const char* labels[kPerformFaderCount] = {"A", "D", "S", "R"};
			const float values[kPerformFaderCount] = {amp_attack, amp_decay, amp_sustain, amp_release};
			draw_faders(box,
						is_selected,
						labels,
						values,
						kPerformFaderCount,
						amp_select_active,
						amp_selected,
						false);
		}
		if (i == kPerformFltIndex)
		{
			const char* labels[kPerformFltFaderCount] = {"C", "R"};
			const float values[kPerformFltFaderCount] = {flt_cutoff, flt_res};
			draw_faders(box,
						is_selected,
						labels,
						values,
						kPerformFltFaderCount,
						flt_select_active,
						flt_selected,
						true);
		}
		if (i == kPerformFxIndex)
		{
			int32_t order[kPerformFaderCount];
			for (int f = 0; f < kPerformFaderCount; ++f)
			{
				order[f] = fx_chain_order[f];
			}
			const char* labels[kPerformFaderCount] =
				{FxShortLabel(order[0]),
				 FxShortLabel(order[1]),
				 FxShortLabel(order[2]),
				 FxShortLabel(order[3])};
			const float values[kPerformFaderCount] =
				{FxWetValue(order[0]),
				 FxWetValue(order[1]),
				 FxWetValue(order[2]),
				 FxWetValue(order[3])};
			draw_faders(box,
						is_selected,
						labels,
						values,
						kPerformFaderCount,
						fx_select_active,
						fx_selected,
						false);
		}
	}
#if PERF_DIAGNOSTICS
	{
		const FontDef font = Font_6x8;
		char cpu_label[12];
		int cpu_pct = static_cast<int>(cpu_load_pct + 0.5f);
		cpu_pct = ClampI(cpu_pct, 0, 100);
		snprintf(cpu_label, sizeof(cpu_label), "CPU %d%%", cpu_pct);
		const int text_w = static_cast<int>(StrLen(cpu_label)) * font.FontWidth;
		int x = kDisplayW - text_w - 1;
		if (x < 0)
		{
			x = 0;
		}
		const bool cpu_on = (selected != kPerformAmpIndex);
		display.SetCursor(x, 0);
		display.WriteString(cpu_label, font, cpu_on);
	}
#endif
	RequestDisplayUpdate();
}

void DrawProgressBar(int x, int y, int w, int h, int32_t percent)
{
	if (w <= 2 || h <= 2)
	{
		return;
	}
	if (percent < 0)
	{
		percent = 0;
	}
	if (percent > 100)
	{
		percent = 100;
	}
	display.DrawRect(x, y, x + w - 1, y + h - 1, true, false);
	const int inner_w = w - 2;
	const int fill_w = (inner_w * percent) / 100;
	if (fill_w > 0)
	{
		display.DrawRect(x + 1, y + 1, x + fill_w, y + h - 2, true, true);
	}
}

void DrawLoadMessage(const char* line1, const char* line2)
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	DrawScaledString(line1, 0, 0, font, kLoadFontScale, true, load_chars_per_line);
	if (line2 != nullptr)
	{
		DrawScaledString(line2,
						 0,
						 load_line_height,
						 font,
						 kLoadFontScale,
						 true,
						 load_chars_per_line);
	}
	RequestDisplayUpdate();
}

void DrawLoadMenu(int32_t top_index, int32_t selected)
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	if (kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode)
	{
		DrawLoadMessage("PRESETS", "COMING SOON");
		return;
	}

	if (sd_fault && sd_fault_text)
	{
		DrawLoadMessage(sd_fault_text, "FAILED TO LOAD SD");
		return;
	}
	if (!sd_mounted)
	{
		DrawLoadMessage("SD NOT", "MOUNTED");
		return;
	}
	if (wav_file_count == 0)
	{
		const uint32_t now = System::GetNow();
		const bool grace_active = (load_scan_start_ms != 0)
			&& (uint32_t)(now - load_scan_start_ms) < kLoadScanGraceMs;
		if (delete_mode)
		{
			DrawLoadMessage(grace_active ? "SCANNING" : "NO", "FILES");
		}
		else
		{
			DrawLoadMessage(grace_active ? "SCANNING" : "NO WAV", "FILES");
		}
		return;
	}

	if (top_index < 0)
	{
		top_index = 0;
	}
	int32_t max_top = wav_file_count - load_lines;
	if (max_top < 0)
	{
		max_top = 0;
	}
	if (top_index > max_top)
	{
		top_index = max_top;
	}

	const int32_t visible_lines = LoadVisibleLines();
	for (int32_t i = 0; i < visible_lines; ++i)
	{
		const int32_t idx = top_index + i;
		if (idx >= wav_file_count)
		{
			break;
		}
		const int y = i * load_line_height;
		const bool is_selected = (idx == selected);
		if (is_selected)
		{
			display.DrawRect(0,
							 y,
							 display.Width() - 1,
							 y + load_line_height - 1,
							 true,
							 true);
		}
		DrawScaledString(wav_files[idx],
						 0,
						 y,
						 font,
						 kLoadFontScale,
						 !is_selected,
						 load_chars_per_line);
	}
	RequestDisplayUpdate();
}

void DrawRecordReadyScreen()
{
	display.Fill(false);

	std::memset(record_text_mask, 0, sizeof(record_text_mask));
	std::memset(record_invert_mask, 0, sizeof(record_invert_mask));
	std::memset(record_fb_buf, 0, sizeof(record_fb_buf));
	std::memset(record_bold_mask, 0, sizeof(record_bold_mask));

	if (record_anim_start_ms < 0.0)
	{
		record_anim_start_ms = NowMs();
	}

	const double elapsed_s = (NowMs() - record_anim_start_ms) / 1000.0;
	const int cx = kDisplayW / 2;
	const int cy = kDisplayH / 2;

	// Prepare text mask using big font, but shrink if needed.
	const char* line1 = (record_input == RecordInput::Mic)
		? "RECORD MICROPHONE"
		: "RECORD LINE IN";
	const char* line2 = "READY";
	int scale = 2;
	int char_spacing = scale;
	int line_gap = scale * 2;
	int char_h = Font5x7::H * scale;
	const int lines = 2;
	int text_h = lines * char_h + (lines - 1) * line_gap;
	int y0 = (kDisplayH / 2) - (text_h / 2);

	auto mark_char = [&](int x, int y, char c)
	{
		uint8_t rows[Font5x7::H] = {};
		Font5x7::GetGlyphRows(c, rows);
		for (int yy = 0; yy < Font5x7::H; ++yy)
		{
			uint8_t row = rows[yy];
			for (int xx = 0; xx < Font5x7::W; ++xx)
			{
				if ((row >> (Font5x7::W - 1 - xx)) & 1)
				{
					for (int sy = 0; sy < scale; ++sy)
					{
						for (int sx = 0; sx < scale; ++sx)
						{
							const int px = x + xx * scale + sx;
							const int py = y + yy * scale + sy;
							if (px >= 0 && px < kDisplayW && py >= 0 && py < kDisplayH)
							{
								record_text_mask[py][px] = 1;
								record_fb_buf[py][px] = 1;
							}
						}
					}
				}
			}
		}
	};

	auto mark_line = [&](int x, int y, const char* text)
	{
		const int char_w = Font5x7::W * scale;
		int cx0 = x;
		for (const char* p = text; *p; ++p)
		{
			mark_char(cx0, y, *p);
			cx0 += char_w + char_spacing;
		}
	};

	auto width = [&](const char* t)
	{
		const int len = static_cast<int>(std::strlen(t));
		if (len <= 0)
		{
			return 0;
		}
		const int char_w = Font5x7::W * scale;
		return len * char_w + (len - 1) * char_spacing;
	};

	int max_w = width(line1);
	const int line2_w = width(line2);
	if (line2_w > max_w)
	{
		max_w = line2_w;
	}
	if (max_w > kDisplayW)
	{
		scale = 1;
		char_spacing = scale;
		line_gap = scale * 2;
		char_h = Font5x7::H * scale;
	}
	text_h = lines * char_h + (lines - 1) * line_gap;
	y0 = (kDisplayH / 2) - (text_h / 2);

	auto mark_centered = [&](const char* t1, const char* t2)
	{
		auto width = [&](const char* t)
		{
			const int len = static_cast<int>(std::strlen(t));
			if (len <= 0)
			{
				return 0;
			}
			const int char_w = Font5x7::W * scale;
			return len * char_w + (len - 1) * char_spacing;
		};

		const int line1_w = width(t1);
		const int line2_w = width(t2);
		const int x1 = (kDisplayW / 2) - (line1_w / 2);
		const int x2 = (kDisplayW / 2) - (line2_w / 2);

		mark_line(x1, y0, t1);
		mark_line(x2, y0 + char_h + line_gap, t2);
	};

	mark_centered(line1, line2);

	// Three circles shrinking into the center (staggered) + one growing out.
	const double max_visible_r = std::sqrt(std::pow(kDisplayW / 2.0, 2) + std::pow(kDisplayH / 2.0, 2));
	const double start_r = max_visible_r + 10.0;
	const double duration_s = 1.0;
	const double offset1_s = 0.2;
	const double offset2_s = offset1_s + 0.3;
	const double grow_duration_s = 0.5;
	const double grow_start_s = offset2_s + duration_s / 2.0;
	const double gap_s = 0.1;
	const double cycle_s = grow_start_s + grow_duration_s + gap_s;
	const double anim_t = std::fmod(elapsed_s, cycle_s);

	auto animate_circle = [&](double t_offset,
							  int thickness_px,
							  bool invert_text,
							  double speedup_after_abs = -1.0,
							  double speedup_factor = 1.0)
	{
		const double local_t = anim_t - t_offset;
		if (local_t < 0.0 || local_t > duration_s)
		{
			return;
		}
		double adj_local_t = local_t;
		if (speedup_after_abs >= 0.0 && speedup_factor != 1.0)
		{
			const double threshold_local = speedup_after_abs - t_offset;
			if (local_t > threshold_local)
			{
				const double extra = local_t - threshold_local;
				adj_local_t = threshold_local + extra * speedup_factor;
				if (adj_local_t > duration_s)
				{
					adj_local_t = duration_s;
				}
			}
		}

		const double f = 1.0 - (adj_local_t / duration_s);
		double r = start_r * f;
		if (r > max_visible_r)
		{
			return;
		}
		int ri = static_cast<int>(std::round(r));
		for (int t = 0; t < thickness_px; ++t)
		{
			const int rr = ri - t;
			if (rr <= 0)
			{
				continue;
			}
			ForCirclePixels(cx, cy, rr, [&](int px, int py)
			{
				if (px < 0 || px >= kDisplayW || py < 0 || py >= kDisplayH)
				{
					return;
				}
				if (record_text_mask[py][px] && invert_text)
				{
					record_invert_mask[py][px] = !record_invert_mask[py][px];
				}
				else if (!record_text_mask[py][px])
				{
					record_fb_buf[py][px] = 1;
				}
			});
		}
	};

	animate_circle(0.0, 2, false);
	animate_circle(offset1_s, 4, true);
	animate_circle(offset2_s, 2, false, offset1_s + duration_s, 2.0);

	auto animate_grow_circle = [&](double t_offset, int thickness_px)
	{
		const double local_t = anim_t - t_offset;
		if (local_t < 0.0 || local_t > grow_duration_s)
		{
			return;
		}
		const double f = local_t / grow_duration_s;
		const double base_r = thickness_px - 1;
		const double target_r = max_visible_r + thickness_px - 1;
		double r = base_r + f * (target_r - base_r);
		int ri = static_cast<int>(std::round(r));
		for (int t = 0; t < thickness_px; ++t)
		{
			const int rr = ri - t;
			if (rr <= 0)
			{
				continue;
			}
			ForCirclePixels(cx, cy, rr, [&](int px, int py)
			{
				if (px < 0 || px >= kDisplayW || py < 0 || py >= kDisplayH)
				{
					return;
				}
				if (record_text_mask[py][px])
				{
					record_invert_mask[py][px] = !record_invert_mask[py][px];
				}
				else
				{
					record_fb_buf[py][px] = !record_fb_buf[py][px];
				}
			});
		}
	};

	animate_grow_circle(grow_start_s, 16);

	const double flicker_on_s = 0.1;
	const double flicker_off_s = 0.1;
	const double flicker_period = flicker_on_s + flicker_off_s;

	auto apply_bold = [&]()
	{
		std::memset(record_bold_mask, 0, sizeof(record_bold_mask));
		for (int y = 0; y < kDisplayH; ++y)
		{
			for (int x = 0; x < kDisplayW; ++x)
			{
				if (!record_text_mask[y][x])
				{
					continue;
				}
				for (int dy = 0; dy <= 1; ++dy)
				{
					for (int dx = 0; dx <= 1; ++dx)
					{
						const int px = x + dx;
						const int py = y + dy;
						if (px >= 0 && px < kDisplayW && py >= 0 && py < kDisplayH)
						{
							record_bold_mask[py][px] = 1;
						}
					}
				}
			}
		}

		for (int y = 0; y < kDisplayH; ++y)
		{
			for (int x = 0; x < kDisplayW; ++x)
			{
				if (record_bold_mask[y][x])
				{
					record_text_mask[y][x] = 1;
					record_fb_buf[y][x] = 1;
				}
			}
		}
	};

	const double flicker_phase = std::fmod(anim_t, flicker_period);
	if (flicker_phase < flicker_on_s)
	{
		const bool enlarged = false;
		const bool extra_large = false;
		const bool scale_up = (static_cast<int>(std::floor(anim_t / flicker_period)) % 3) == 2;
		if (enlarged || extra_large || scale_up)
		{
			const double scale_factor = scale_up ? 1.3 : 1.0;
			const int max_d = extra_large ? 4 : (enlarged ? 2 : 0);
			std::memset(record_bold_mask, 0, sizeof(record_bold_mask));
			for (int y = 0; y < kDisplayH; ++y)
			{
				for (int x = 0; x < kDisplayW; ++x)
				{
					if (!record_text_mask[y][x])
					{
						continue;
					}
					const int pad = max_d;
					for (int dy = -pad; dy <= pad; ++dy)
					{
						for (int dx = -pad; dx <= pad; ++dx)
						{
							const int px = x + dx;
							const int py = y + dy;
							if (px >= 0 && px < kDisplayW && py >= 0 && py < kDisplayH)
							{
								record_bold_mask[py][px] = 1;
							}
						}
					}
					if (scale_up && pad == 0)
					{
						for (int dy = -1; dy <= 1; ++dy)
						{
							for (int dx = -1; dx <= 1; ++dx)
							{
								const int px = x + static_cast<int>(std::round(dx * scale_factor));
								const int py = y + static_cast<int>(std::round(dy * scale_factor));
								if (px >= 0 && px < kDisplayW && py >= 0 && py < kDisplayH)
								{
									record_bold_mask[py][px] = 1;
								}
							}
						}
					}
				}
			}

			for (int y = 0; y < kDisplayH; ++y)
			{
				for (int x = 0; x < kDisplayW; ++x)
				{
					if (record_bold_mask[y][x])
					{
						record_text_mask[y][x] = 1;
						record_fb_buf[y][x] = 1;
					}
				}
			}
		}
		else
		{
			apply_bold();
		}
	}

	for (int y = 0; y < kDisplayH; ++y)
	{
		for (int x = 0; x < kDisplayW; ++x)
		{
			if (record_text_mask[y][x] && record_invert_mask[y][x])
			{
				record_fb_buf[y][x] = !record_fb_buf[y][x];
			}
			display.DrawPixel(x, y, record_fb_buf[y][x]);
		}
	}

	RequestDisplayUpdate();
}

void DrawDeleteConfirm(const char* name)
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("DELETE?", font, true);
	if (name != nullptr && name[0] != '\0')
	{
		DrawScaledString(name, 0, font.FontHeight + 2, font, kLoadFontScale, true, load_chars_per_line);
	}
	display.SetCursor(0, (font.FontHeight + 2) * 3);
	display.WriteString("L=NO  R=YES", font, true);
	RequestDisplayUpdate();
}

void DrawRecordBackConfirm()
{
	const FontDef font = Font_6x8;

	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("ARE YOU SURE?", font, true);
	display.SetCursor(0, font.FontHeight + 2);
	display.WriteString("REC WILL", font, true);
	display.SetCursor(0, (font.FontHeight + 2) * 2);
	display.WriteString("BE LOST", font, true);
	display.SetCursor(0, (font.FontHeight + 2) * 4);
	display.WriteString("L=NO  R=YES", font, true);
	RequestDisplayUpdate();
}

void DrawRecordSourceScreen()
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("SOURCE:", font, true);

	const char* options[2] = {"LINE IN", "MICROPHONE"};
	const int line_h = font.FontHeight + 2;
	const int start_y = line_h + 2;
	for (int i = 0; i < 2; ++i)
	{
		const int y = start_y + i * line_h;
		const bool is_selected = (i == record_source_index);
		if (is_selected)
		{
			display.DrawRect(0,
							 y,
							 display.Width() - 1,
							 y + line_h - 1,
							 true,
							 true);
		}
		display.SetCursor(2, y + 1);
		display.WriteString(options[i], font, !is_selected);
	}
	RequestDisplayUpdate();
}

void DrawRecordArmed()
{
	DrawRecordReadyScreen();
}


int BitResoIndexFromValue(float value)
{
	if (value < 0.0f)
	{
		value = 0.0f;
	}
	else if (value > 1.0f)
	{
		value = 1.0f;
	}
	const int max_idx = kBitResoStepCount - 1;
	const int idx = static_cast<int>(value * static_cast<float>(max_idx) + 0.5f);
	return ClampI(idx, 0, max_idx);
}

float BitResoValueFromIndex(int idx)
{
	const int max_idx = kBitResoStepCount - 1;
	return static_cast<float>(ClampI(idx, 0, max_idx)) / static_cast<float>(max_idx);
}

// UI thread ONLY: bookkeeping + UI state + publish snapshot
static void PrepareRecordingUiState()
{
    // UI/bookkeeping (safe outside audio callback)
    sample_length = 0;
    sample_loaded = false;

    perform_attack_norm  = 0.0f;
    perform_release_norm = 0.0f;

    RequestAudioCmd(kCmdAllNotesOff);
    RequestPlaybackStopAll();

    sample_channels = 1;
    sample_rate     = 48000;

    trim_start = 0.0f;
    trim_end   = 1.0f;

    CopyString(loaded_sample_name, "UNSAVED AUDIO", kMaxWavNameLen);

    {
        daisy::ScopedIrqBlocker irq;
        AudioUiResetLiveWaveform(g_audio_ui_state_buf[0]);
        AudioUiResetLiveWaveform(g_audio_ui_state_buf[1]);
    }

    // Publish "empty" runtime so audio never sees half-cleared state
    PublishRuntimeFromUi();
}

// Audio thread ONLY: deterministic / bounded (called from AudioCallback)
void StartRecordingAudioRT()
{
    record_pos = 0;

    // Stop playback immediately & deterministically
    playback_active = false;

    // Begin recording
    g_record_start_ms = System::GetNow();
    g_audio_recording_active = true;
}


void StartRecording()
{
    waveform_record_input = record_input;

    // UI prep only
    PrepareRecordingUiState();

    // UI state machine
    record_state = RecordState::Recording;
}


void DrawRecordCountdown()
{
	const FontDef font = Font_6x8;
	const uint32_t now = System::GetNow();
	const uint32_t elapsed = now - record_countdown_start_ms;
	uint32_t remaining_ms = 0;
	if (elapsed < kRecordCountdownMs)
	{
		remaining_ms = kRecordCountdownMs - elapsed;
	}
	const uint32_t remaining_s = (remaining_ms + 999) / 1000;

	display.Fill(false);

	const int cx = kDisplayW / 2;
	const int cy = kDisplayH / 2;

	auto DrawPixelInv = [&](int x, int y, bool on)
	{
		if (x < 0 || x >= kDisplayW || y < 0 || y >= kDisplayH)
		{
			return;
		}
		display.DrawPixel(x, y, on);
	};

	auto DrawLineInv = [&](int x0, int y0, int x1, int y1, bool on)
	{
		int dx = abs(x1 - x0);
		int sx = x0 < x1 ? 1 : -1;
		int dy = -abs(y1 - y0);
		int sy = y0 < y1 ? 1 : -1;
		int err = dx + dy;
		while (true)
		{
			DrawPixelInv(x0, y0, on);
			if (x0 == x1 && y0 == y1)
				break;
			int e2 = 2 * err;
			if (e2 >= dy)
			{
				err += dy;
				x0 += sx;
			}
			if (e2 <= dx)
			{
				err += dx;
				y0 += sy;
			}
		}
	};

	auto DrawScaledCharInv = [&](char ch, int x, int y, int scale)
	{
		if (ch < 32 || ch > 126)
		{
			return;
		}
		const uint32_t base = static_cast<uint32_t>(ch - 32) * font.FontHeight;
		for (uint32_t row = 0; row < font.FontHeight; ++row)
		{
			const uint32_t bits = font.data[base + row];
			for (uint32_t col = 0; col < font.FontWidth; ++col)
			{
				const bool pixel_on = ((bits << col) & 0x8000) != 0;
				if (!pixel_on)
				{
					continue;
				}
				const int px = x + static_cast<int>(col * scale);
				const int py = y + static_cast<int>(row * scale);
				for (int dy = 0; dy < scale; ++dy)
				{
					for (int dx = 0; dx < scale; ++dx)
					{
						DrawPixelInv(px + dx, py + dy, true);
					}
				}
			}
		}
	};

	// Big countdown number centered.
	char buf[8];
	snprintf(buf, sizeof(buf), "%lu", static_cast<unsigned long>(remaining_s));
	const int scale = 4;
	const int text_w = static_cast<int>(std::strlen(buf)) * font.FontWidth * scale;
	const int text_h = font.FontHeight * scale;
	const int text_x = (kDisplayW - text_w) / 2;
	const int text_y = (kDisplayH - text_h) / 2;
	for (int i = 0; buf[i] != '\0'; ++i)
	{
		DrawScaledCharInv(buf[i], text_x + i * font.FontWidth * scale, text_y, scale);
	}

	// Old movie reel style: crosshair + double ring + sweeping hand.
	const int outer_r = 30;
	const int inner_r = 22;
	ForCirclePixels(cx, cy, outer_r, [&](int x, int y) {
		DrawPixelInv(x, y, true);
	});
	ForCirclePixels(cx, cy, inner_r, [&](int x, int y) {
		DrawPixelInv(x, y, true);
	});
	DrawLineInv(cx, 0, cx, kDisplayH - 1, true);
	DrawLineInv(0, cy, kDisplayW - 1, cy, true);

	const float phase = (elapsed % 1000) / 1000.0f;
	const float angle = phase * 2.0f * kPi;
	const int hand_r = outer_r - 2;
	const int hx = cx + static_cast<int>(cosf(angle) * hand_r);
	const int hy = cy + static_cast<int>(sinf(angle) * hand_r);
	DrawLineInv(cx, cy, hx, hy, true);

	RequestDisplayUpdate();
}

void DrawRecordRecording(const AudioUiState& ui_state)
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("RECORDING: 5 SEC MAX", font, true);

	uint32_t start_ms = 0;
	{
		daisy::ScopedIrqBlocker irq;
		start_ms = g_record_start_ms;
	}
	const uint32_t now = System::GetNow();
	const uint32_t elapsed_ms = (start_ms > 0 && now >= start_ms) ? (now - start_ms) : 0;
	const float record_ms = (static_cast<float>(kRecordMaxFrames) * 1000.0f)
		/ static_cast<float>(kSampleRateHz);
	const float progress = Clamp01(static_cast<float>(elapsed_ms) / record_ms);
	const int wave_top = font.FontHeight + 2;
	const int wave_bottom = kDisplayH - 1;
	const int wave_h = wave_bottom - wave_top;
	const float mic_boost = (waveform_record_input == RecordInput::Mic) ? 1.5f : 1.0f;
	static float mist_level[128] = {};
	static uint32_t mist_seed = 0xA5B35791u;
	for (int x = 0; x < 128; ++x)
	{
		int16_t minv = ui_state.live_wave.minv[x];
		int16_t maxv = ui_state.live_wave.maxv[x];
		int16_t abs_max = minv < 0 ? static_cast<int16_t>(-minv) : minv;
		const int16_t abs2 = maxv < 0 ? static_cast<int16_t>(-maxv) : maxv;
		if (abs2 > abs_max)
		{
			abs_max = abs2;
		}
		const float target = Clamp01((static_cast<float>(abs_max) / static_cast<float>(wave_h)) * mic_boost);
		float v = mist_level[x];
		const float rise = 0.35f;
		const float fall = 0.45f;
		if (target > v)
		{
			v += (target - v) * rise;
		}
		else
		{
			v += (target - v) * fall;
		}
		mist_level[x] = v;

		const float gain = 1.6f;
		const int h = ClampI(static_cast<int>(v * static_cast<float>(wave_h) * gain + 0.5f), 0, wave_h);
		const int start = wave_bottom - h;
		const int end = wave_bottom;
		for (int y = start; y <= end; ++y)
		{
			mist_seed = mist_seed * 1664525u + 1013904223u;
			const int frac = wave_bottom - y;
			const int dens = 1 + (frac / 3);
			if ((mist_seed % static_cast<uint32_t>(dens)) == 0u)
			{
				display.DrawPixel(x, y, true);
			}
		}
	}
	const int playhead_x = static_cast<int>(
		progress * static_cast<float>(kDisplayW - 1) + 0.5f);
	display.DrawLine(playhead_x, wave_top, playhead_x, kDisplayH - 1, true);
	RequestDisplayUpdate();
}

void DrawShiftMenu(int32_t selected)
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	const int line_h = font.FontHeight + 2;
	for (int32_t i = 0; i < kShiftMenuCount; ++i)
	{
		const int y = i * line_h;
		const bool is_selected = (i == selected);
		if (is_selected)
		{
			display.DrawRect(0,
							 y,
							 display.Width() - 1,
							 y + line_h - 1,
							 true,
							 true);
		}
		display.SetCursor(2, y + 1);
		display.WriteString(kShiftMenuLabels[i], font, !is_selected);
		if (i == 2)
		{
			float vol = 1.0f;
			{
				daisy::ScopedIrqBlocker irq;
				vol = phones_volume;
			}
			const int pct = ClampI(static_cast<int>(vol * 100.0f + 0.5f), 0, 100);
			char pct_buf[8];
			snprintf(pct_buf, sizeof(pct_buf), "%d", pct);
			const int text_w = static_cast<int>(StrLen(pct_buf)) * font.FontWidth;
			int x = kDisplayW - text_w - 2;
			if (x < 2)
			{
				x = 2;
			}
			display.SetCursor(x, y + 1);
			display.WriteString(pct_buf, font, !is_selected);
		}
	}
	RequestDisplayUpdate();
}

void DrawSdInitScreen()
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	if (sd_init_done)
	{
		display.SetCursor(0, 0);
		display.WriteString(sd_init_success ? "SD INIT OK" : "SD INIT FAILED", font, true);
	}
	else
	{
		display.SetCursor(0, 0);
		display.WriteString("INITIALIZING SD", font, true);

		const uint32_t now = System::GetNow();
		const uint32_t phase = (now / 200) % 4;
		char dots[5] = "....";
		for (uint32_t i = phase; i < 4; ++i)
		{
			dots[i] = ' ';
		}
		display.SetCursor(0, font.FontHeight + 2);
		display.WriteString(dots, font, true);
		display.SetCursor(0, (font.FontHeight + 2) * 2);
		char buf[24];
		snprintf(buf, sizeof(buf), "TRY %ld/%ld",
				 static_cast<long>(sd_init_attempts + 1),
				 static_cast<long>(kSdInitAttempts));
		display.WriteString(buf, font, true);
	}
	RequestDisplayUpdate();
}

void DrawSaveScreen()
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	if (save_done)
	{
		display.SetCursor(0, 0);
		display.WriteString(save_success ? "SAVE OK" : "SAVE FAILED", font, true);
		if (save_success)
		{
			display.SetCursor(0, font.FontHeight + 2);
			display.WriteString(save_filename, font, true);
		}
	}
	else
	{
		display.SetCursor(0, 0);
		display.WriteString("SAVING", font, true);
		const int bar_y = font.FontHeight + 16;
		const int bar_w = 96;
		const int bar_h = 6;
		const int bar_x = (kDisplayW - bar_w) / 2;
		int32_t percent = 0;
		if (save_started && sample_length > 0)
		{
			percent = static_cast<int32_t>(
				(save_frames_written * 100U) / sample_length);
		}
		DrawProgressBar(bar_x, bar_y, bar_w, bar_h, percent);
	}
	RequestDisplayUpdate();
}

int ClampI(int v, int lo, int hi)
{
	return (v < lo) ? lo : (v > hi ? hi : v);
}

float ClampF(float v, float lo, float hi)
{
	return (v < lo) ? lo : (v > hi ? hi : v);
}

void ValidateConfig()
{
#if PERF_DIAGNOSTICS
	if (kMaxSampleFrames != static_cast<size_t>(kSampleRateHz) * kMaxSampleSeconds)
	{
		while (1) {}
	}
	if (kPreviewPreloadFrames > kMaxSampleFrames)
	{
		while (1) {}
	}
	if (kPerformSampleRamBudgetBytes < kMaxSampleFrames * sizeof(int16_t) * 2)
	{
		while (1) {}
	}
	if (kWaveformCacheBytes < (128 * sizeof(int16_t) * 2))
	{
		while (1) {}
	}
#endif
}

static float AmpEnvMsFromFader(float value)
{
	if (value < 0.0f)
	{
		value = 0.0f;
	}
	else if (value > 1.0f)
	{
		value = 1.0f;
	}
	int steps = static_cast<int>(value / kAmpEnvStep + 0.5f);
	if (steps < 0)
	{
		steps = 0;
	}
	const int max_steps = static_cast<int>(
		((kAmpEnvMaxMs - kAmpEnvMinMs) / kAmpEnvStepMs) + 0.5f);
	if (steps > max_steps)
	{
		steps = max_steps;
	}
	float ms = kAmpEnvMinMs + (static_cast<float>(steps) * kAmpEnvStepMs);
	if (ms > kAmpEnvMaxMs)
	{
		ms = kAmpEnvMaxMs;
	}
	return ms;
}

static float FltCutoffFromFader(float value, float sample_rate)
{
	if (value < 0.0f)
	{
		value = 0.0f;
	}
	else if (value > 1.0f)
	{
		value = 1.0f;
	}
	const float min_hz = 20.0f;
	const float max_hz = 20000.0f;
	const float shaped = sqrtf(value);
	float hz = min_hz * powf(max_hz / min_hz, shaped);
	const float nyq = sample_rate * 0.49f;
	if (hz > nyq)
	{
		hz = nyq;
	}
	return hz;
}

static float FltQFromFader(float value)
{
	if (value < 0.0f)
	{
		value = 0.0f;
	}
	else if (value > 1.0f)
	{
		value = 1.0f;
	}
	float q = value * 5.0f;
	if (q < 0.001f)
	{
		q = 0.001f;
	}
	return q;
}

void DrawWaveform()
{
	if (!waveform_dirty)
	{
		return;
	}

	waveform_dirty = false;
	display.Fill(false);

	const int W = 128;
	const int H = 64;
	const int text_h = Font_6x8.FontHeight + 1;
	const int mid = text_h + (H - text_h) / 2;
	float record_scale = 1.0f;
	if (waveform_from_recording)
	{
		const bool from_mic = (waveform_record_input == RecordInput::Mic);
		const float min_scale = from_mic ? kRecordWaveformScaleMinMic : kRecordWaveformScaleMinLine;
		const float max_scale = from_mic ? kRecordWaveformScaleMaxMic : kRecordWaveformScaleMaxLine;
		int max_abs = 0;
		for (int i = 0; i < W; ++i)
		{
			const int a = std::abs(static_cast<int>(perform_waveform_cache.Min()[i]));
			const int b = std::abs(static_cast<int>(perform_waveform_cache.Max()[i]));
			if (a > max_abs) max_abs = a;
			if (b > max_abs) max_abs = b;
		}
		if (max_abs > 0)
		{
			const int target = (H - text_h - 2) / 2;
			record_scale = static_cast<float>(target) / static_cast<float>(max_abs);
			if (record_scale < min_scale) record_scale = min_scale;
			if (record_scale > max_scale) record_scale = max_scale;
		}
	}

	int start_x = (int)(trim_start * (W - 1));
	int end_x   = (int)(trim_end   * (W - 1));

	start_x = ClampI(start_x, 0, W - 1);
	end_x   = ClampI(end_x,   0, W - 1);
	if(end_x < start_x)
	{
		const int tmp = start_x;
		start_x = end_x;
		end_x = tmp;
	}

	for(int x = 0; x < W; x++)
	{
		int top    = mid + static_cast<int>(static_cast<float>(perform_waveform_cache.Min()[x]) * record_scale);
		int bottom = mid + static_cast<int>(static_cast<float>(perform_waveform_cache.Max()[x]) * record_scale);

		if(top > bottom)
		{
			const int tmp = top;
			top = bottom;
			bottom = tmp;
		}

		top    = ClampI(top,    text_h, H - 1);
		bottom = ClampI(bottom, text_h, H - 1);

		const bool inside = (x >= start_x && x <= end_x);

		if(inside)
		{
			for(int y = top; y <= bottom; y++)
				if((y & 1) == 0)
					display.DrawPixel(x, y, true);
		}
		else
		{
			display.DrawLine(x, top, x, bottom, true);
		}
	}

	const int cap = 5;
	auto DrawBracket = [&](int x, bool start)
	{
		for(int y = text_h; y < H; y++)
		{
			display.DrawPixel(x, y, true);
			if(x + 1 < W)
				display.DrawPixel(x + 1, y, true);
		}

		for(int dx = 0; dx < cap; dx++)
		{
			int px = start ? (x + dx) : (x - dx);
			if(px >= 0 && px < W)
			{
				display.DrawPixel(px, text_h, true);
				display.DrawPixel(px, H - 1, true);
			}
		}
	};

	DrawBracket(start_x, true);
	DrawBracket(end_x,   false);

	{
		uint8_t ui_idx = 0;
		const AudioUiState& uir = GetAudioUiStateSnapshot(ui_idx);
		if (ui_mode == UiMode::Edt && uir.playback_active)
		{
			const SampleRuntime rt = g_rt_buf[g_rt_active_idx];
			const size_t length = rt.length;
			if (length > 1)
			{
				const float denom = static_cast<float>(length - 1);
				float norm = uir.playback_phase / denom;
				if (norm < 0.0f)
				{
					norm = 0.0f;
				}
				else if (norm > 1.0f)
				{
					norm = 1.0f;
				}
				const int play_x = ClampI(static_cast<int>(norm * static_cast<float>(W - 1) + 0.5f), 0, W - 1);
				display.DrawLine(play_x, text_h, play_x, H - 1, true);
			}
		}
	}

	display.SetCursor(0, 0);
	display.WriteString(waveform_title ? waveform_title : loaded_sample_name, Font_6x8, true);

	RequestDisplayUpdate();
}

void DrawRecordReview()
{
	waveform_title = "RECORDED PLAYBACK";
	DrawWaveform();
	waveform_title = nullptr;
}

void DrawEdtScreen()
{
	char title[kMaxWavNameLen];
	if (sample_loaded && loaded_sample_name[0] != '\0')
	{
		CopyNameSansWav(title, loaded_sample_name, sizeof(title));
	}
	else
	{
		title[0] = '\0';
	}
	if (!waveform_ready)
	{
		const FontDef font = Font_6x8;
		display.Fill(false);
		display.SetCursor(0, 0);
		display.WriteString(title, font, true);
		RequestDisplayUpdate();
		return;
	}
	waveform_title = title;
	DrawWaveform();
	waveform_title = nullptr;
}

void DrawVerticalFadersInRect(int x,
									 int y,
									 int w,
									 int h,
									 const char* const* labels,
									 const float* values,
									 int count,
									 bool select_active,
									 int selected_index,
									 const int* x_offsets = nullptr,
									 const bool* circle_handles = nullptr,
									 const bool* hide_rails = nullptr,
									 const bool* hide_handles = nullptr)
{
	if (w <= 2 || h <= 2 || count <= 0)
	{
		return;
	}
	const int label_y = y + h - Font5x7::H - 1;
	int line_top = y + 2;
	int line_bottom = label_y - 2;
	if (line_bottom <= line_top)
	{
		return;
	}
	int fader_left = x + 2;
	int fader_right = x + w - 3;
	if (fader_right <= fader_left)
	{
		return;
	}
	const int span_x = fader_right - fader_left;
	const int span_y = line_bottom - line_top;
	for (int f = 0; f < count; ++f)
	{
		int line_x = fader_left;
		if (count > 1 && span_x > 0)
		{
			line_x = fader_left + (span_x * f) / (count - 1);
		}
		if (x_offsets != nullptr)
		{
			line_x += x_offsets[f];
		}
		const char* label = labels[f];
		const int label_w = TinyStringWidth(label);
		int label_x = line_x - (label_w / 2);
		if (label_x < x + 1)
		{
			label_x = x + 1;
		}
		if (label_x + label_w > x + w - 2)
		{
			label_x = x + w - 2 - label_w;
		}
		line_x = label_x + (label_w / 2);
		const bool selected = select_active && f == selected_index;
		const bool line_on = true;
		// Console-style rail: thin center line with small end caps.
		int rail_x = line_x;
		if (rail_x < x + 1)
		{
			rail_x = x + 1;
		}
		if (rail_x > x + w - 2)
		{
			rail_x = x + w - 2;
		}
		if (hide_rails == nullptr || !hide_rails[f])
		{
			display.DrawLine(rail_x, line_top, rail_x, line_bottom, line_on);
			display.DrawLine(rail_x - 1, line_top, rail_x + 1, line_top, line_on);
			display.DrawLine(rail_x - 1, line_bottom, rail_x + 1, line_bottom, line_on);
		}

		const float value = values[f];
		int tick_y = line_bottom - static_cast<int>(value * static_cast<float>(span_y) + 0.5f);
		const bool hide_handle = (hide_handles != nullptr && hide_handles[f]);
		const bool draw_circle = (circle_handles != nullptr && circle_handles[f]);
		if (hide_handle)
		{
			// No handle; used for stepped labels only.
		}
		else if (draw_circle)
		{
			const int r = 2;
			int cx = line_x;
			int cy = tick_y;
			if (cx - r < x + 1) cx = x + 1 + r;
			if (cx + r > x + w - 2) cx = x + w - 2 - r;
			if (cy - r < line_top) cy = line_top + r;
			if (cy + r > line_bottom) cy = line_bottom - r;
			display.DrawRect(cx - r, cy - r, cx + r, cy + r, true, false);
			display.DrawRect(cx - r + 1, cy - r + 1, cx + r - 1, cy + r - 1, false, true);
			display.DrawPixel(cx - r, cy - r, false);
			display.DrawPixel(cx + r, cy - r, false);
			display.DrawPixel(cx - r, cy + r, false);
			display.DrawPixel(cx + r, cy + r, false);
		}
		else
		{
			int handle_w = 7;
			if (handle_w > (x + w - 2) - (x + 1))
			{
				handle_w = (x + w - 2) - (x + 1);
			}
			int handle_x0 = line_x - handle_w / 2;
			int handle_x1 = handle_x0 + handle_w;
			if (handle_x0 < x + 1)
			{
				handle_x0 = x + 1;
				handle_x1 = handle_x0 + handle_w;
			}
			if (handle_x1 > x + w - 2)
			{
				handle_x1 = x + w - 2;
				handle_x0 = handle_x1 - handle_w;
			}
			int handle_y0 = tick_y - 5;
			int handle_y1 = tick_y + 5;
			if (handle_y0 < line_top)
			{
				handle_y0 = line_top;
			}
			if (handle_y1 > line_bottom)
			{
				handle_y1 = line_bottom;
			}
			// Inverted handle: white outline, black body, white stripes.
			display.DrawRect(handle_x0, handle_y0, handle_x1, handle_y1, true, false);
			if (handle_x1 - handle_x0 >= 2 && handle_y1 - handle_y0 >= 2)
			{
				display.DrawRect(handle_x0 + 1,
								 handle_y0 + 1,
								 handle_x1 - 1,
								 handle_y1 - 1,
								 false,
								 true);
			}
			// Slightly round the handle corners by knocking out corner pixels.
			if ((handle_x1 - handle_x0) >= 4 && (handle_y1 - handle_y0) >= 4)
			{
				display.DrawPixel(handle_x0, handle_y0, false);
				display.DrawPixel(handle_x1, handle_y0, false);
				display.DrawPixel(handle_x0, handle_y1, false);
				display.DrawPixel(handle_x1, handle_y1, false);
			}
			const int center_y = handle_y0 + ((handle_y1 - handle_y0) / 2);
			const int inner_x0 = handle_x0 + 1;
			const int inner_x1 = handle_x1 - 1;
			if (inner_x1 > inner_x0)
			{
				const int mid_pad = 1;
				display.DrawLine(inner_x0 + mid_pad, center_y, inner_x1 - mid_pad, center_y, true);
				if (center_y - 2 >= handle_y0 + 1)
				{
					display.DrawLine(inner_x0,
									 center_y - 2,
									 inner_x1,
									 center_y - 2,
									 true);
				}
				if (center_y + 2 <= handle_y1 - 1)
				{
					display.DrawLine(inner_x0,
									 center_y + 2,
									 inner_x1,
									 center_y + 2,
									 true);
				}
			}
		}
		if (label_x + label_w < x + w - 1)
		{
			if (selected)
			{
				int label_x0 = label_x - 1;
				int label_x1 = label_x + label_w;
				int label_y0 = label_y - 1;
				int label_y1 = label_y + Font5x7::H;
				if (label_x0 < x + 1)
				{
					label_x0 = x + 1;
				}
				if (label_x1 > x + w - 2)
				{
					label_x1 = x + w - 2;
				}
				if (label_y0 < y + 1)
				{
					label_y0 = y + 1;
				}
				if (label_y1 > y + h - 2)
				{
					label_y1 = y + h - 2;
				}
				display.DrawRect(label_x0, label_y0, label_x1, label_y1, true, true);
				DrawTinyString(label, label_x, label_y, false);
			}
			else
			{
				DrawTinyString(label, label_x, label_y, true);
			}
		}
	}
}

void DrawFxDetailScreen(int32_t index)
{
	const char* labels[kPerformFaderCount] = {"SATURATION", "MODULATION", "DELAY", "REVERB"};
	if (index < 0 || index >= kPerformFaderCount)
	{
		index = 0;
	}
	display.Fill(false);
	const char* label = labels[index];
	const int text_w = TinyStringWidth(label);
	int text_x = (kDisplayW - text_w) / 2;
	if (text_x < 0)
	{
		text_x = 0;
	}
	DrawTinyString(label, text_x, 1, true);
	if (index == kFxSatIndex)
	{
		constexpr int kMargin = 2;
		constexpr int kGap = 2;
		const int block_x = kMargin;
		const int block_w = kDisplayW / 4;
		const int block_y = Font5x7::H + 4;
		int block_h = kDisplayH - block_y - kMargin;
		if (block_h < 3)
		{
			block_h = 3;
		}
		const int box_h = (block_h - kGap) / 2;
		const bool tape_selected = (sat_mode == 0);
		const bool bit_selected = (sat_mode == 1);
		const bool mode_select_active = (fx_detail_param_index == 3);
		if (mode_select_active)
		{
			display.DrawRect(block_x - 1,
							 block_y - 1,
							 block_x + block_w,
							 block_y + block_h,
							 true,
							 false);
		}
		display.DrawRect(block_x,
						 block_y,
						 block_x + block_w - 1,
						 block_y + box_h - 1,
						 true,
						 tape_selected);
		display.DrawRect(block_x,
						 block_y + box_h + kGap,
						 block_x + block_w - 1,
						 block_y + (box_h * 2) + kGap - 1,
						 true,
						 bit_selected);
		const int label_w1 = TinyStringWidth("TAPE");
		const int label_w2 = TinyStringWidth("BIT");
		const int label_y1 = block_y + (box_h - Font5x7::H) / 2;
		const int label_y2 = block_y + box_h + kGap + (box_h - Font5x7::H) / 2;
		int label_x1 = block_x + (block_w - label_w1) / 2;
		int label_x2 = block_x + (block_w - label_w2) / 2;
		if (label_x1 < block_x + 1)
		{
			label_x1 = block_x + 1;
		}
		if (label_x2 < block_x + 1)
		{
			label_x2 = block_x + 1;
		}
		DrawTinyString("TAPE", label_x1, label_y1, !tape_selected);
		DrawTinyString("BIT", label_x2, label_y2, !bit_selected);

		const int fader_offset = 8;
		const int fader_x = block_x + block_w + kGap + fader_offset;
		const int fader_w = kDisplayW - fader_x - kMargin;
		if (fader_w > 4)
		{
			const char* fader_labels[3]
				= {(sat_mode == 1) ? "RESO" : "SAT",
				   (sat_mode == 1) ? "SMPL" : "BUMP",
				   "MIX"};
			const float fader_values[3]
				= {(sat_mode == 1) ? sat_bit_reso : sat_drive,
				   (sat_mode == 1) ? sat_bit_smpl : sat_tape_bump,
				   fx_s_wet};
			int param_index = fx_detail_param_index;
			const bool fader_select_active = (param_index >= 0 && param_index < 3);
			if (!fader_select_active && !mode_select_active)
			{
				param_index = 0;
			}
			const int fader_offsets[3] = {0, 0, 0};
			const bool circle_handles[3] = {false, false, false};
			const bool hide_rails[3] = {sat_mode == 1, false, false};
			const bool hide_handles[3] = {sat_mode == 1, false, false};
			DrawVerticalFadersInRect(fader_x,
									 block_y,
									 fader_w,
									 block_h,
									 fader_labels,
									 fader_values,
									 3,
									 fader_select_active,
									 param_index,
									 fader_offsets,
									 circle_handles,
									 hide_rails,
									 hide_handles);
			if (sat_mode == 1)
			{
				const int label_y = block_y + block_h - Font5x7::H - 1;
				int line_top = block_y + 2;
				int line_bottom = label_y - 2;
				if (line_bottom > line_top)
				{
					int fader_left = fader_x + 2;
					int fader_right = fader_x + fader_w - 3;
					const int span_x = fader_right - fader_left;
					int line_x = fader_left;
					if (span_x > 0)
					{
						line_x = fader_left;
					}
					const char* label = "RESO";
					const int label_w = TinyStringWidth(label);
					int label_x = line_x - (label_w / 2);
					if (label_x < fader_x + 1)
					{
						label_x = fader_x + 1;
					}
					if (label_x + label_w > fader_x + fader_w - 2)
					{
						label_x = fader_x + fader_w - 2 - label_w;
					}
					line_x = label_x + (label_w / 2);

					const int cur_idx = BitResoIndexFromValue(sat_bit_reso);
					const int label_top = line_top + 1;
					const int label_gap = 3;
					int label_y0 = label_top;
					for (int i = 0; i < kBitResoStepCount; ++i)
					{
						const char* bits_label = kBitResoLabels[i];
						const int text_w = TinyStringWidth(bits_label);
						const int text_x = line_x - (text_w / 2);
						const int text_y = label_y0 + (i * (Font5x7::H + label_gap));
						if (text_y >= line_top && text_y <= line_bottom - Font5x7::H)
						{
							const bool is_selected = (i == cur_idx);
							if (is_selected)
							{
								display.DrawRect(text_x - 1,
												 text_y - 1,
												 text_x + text_w,
												 text_y + Font5x7::H,
												 true,
												 true);
								DrawTinyString(bits_label, text_x, text_y, false);
							}
							else
							{
								DrawTinyString(bits_label, text_x, text_y, true);
							}
						}
					}
				}
			}
		}
	}
	else if (index == kFxChorusIndex)
	{
		constexpr int kMargin = 2;
		constexpr int kGap = 2;
		const int block_x = kMargin;
		const int block_w = kDisplayW / 4;
		const int block_y = Font5x7::H + 4;
		int block_h = kDisplayH - block_y - kMargin;
		if (block_h < 3)
		{
			block_h = 3;
		}
		const bool chorus_selected = (chorus_mode == 0);
		const bool tape_selected = (chorus_mode == 1);
		const bool algo_selected = (fx_detail_param_index == 3);
		const int algo_x0 = block_x;
		const int algo_y0 = block_y;
		const int algo_x1 = block_x + block_w - 1;
		const int algo_y1 = block_y + block_h - 1;
		const int algo_pad = 2;
		const int algo_gap = 2;
		const int inner_x0 = algo_x0 + algo_pad;
		const int inner_y0 = algo_y0 + algo_pad;
		const int inner_x1 = algo_x1 - algo_pad;
		const int inner_y1 = algo_y1 - algo_pad;
		const int inner_h = inner_y1 - inner_y0 + 1;
		const int box_h = (inner_h - algo_gap) / 2;
		const int box1_y0 = inner_y0;
		const int box1_y1 = box1_y0 + box_h - 1;
		const int box2_y0 = box1_y1 + algo_gap + 1;
		const int box2_y1 = box2_y0 + box_h - 1;
		if (algo_selected)
		{
			display.DrawRect(algo_x0, algo_y0, algo_x1, algo_y1, true, false);
			if (algo_x1 - algo_x0 > 2 && algo_y1 - algo_y0 > 2)
			{
				display.DrawRect(algo_x0 + 1, algo_y0 + 1, algo_x1 - 1, algo_y1 - 1, true, false);
			}
		}
		display.DrawRect(inner_x0, box1_y0, inner_x1, box1_y1, true, chorus_selected);
		display.DrawRect(inner_x0, box2_y0, inner_x1, box2_y1, true, tape_selected);
		const int label_w1 = TinyStringWidth("CHRS");
		const int label_w2 = TinyStringWidth("TAPE");
		const int label_y1 = box1_y0 + (box_h - Font5x7::H) / 2;
		const int label_y2 = box2_y0 + (box_h - Font5x7::H) / 2;
		const int inner_w = inner_x1 - inner_x0 + 1;
		int label_x1 = inner_x0 + (inner_w - label_w1) / 2;
		int label_x2 = inner_x0 + (inner_w - label_w2) / 2;
		if (label_x1 < inner_x0 + 1)
		{
			label_x1 = inner_x0 + 1;
		}
		if (label_x1 + label_w1 > inner_x1 - 1)
		{
			label_x1 = inner_x1 - 1 - label_w1;
		}
		if (label_x2 < inner_x0 + 1)
		{
			label_x2 = inner_x0 + 1;
		}
		if (label_x2 + label_w2 > inner_x1 - 1)
		{
			label_x2 = inner_x1 - 1 - label_w2;
		}
		DrawTinyString("CHRS", label_x1, label_y1, !chorus_selected);
		DrawTinyString("TAPE", label_x2, label_y2, !tape_selected);

		const int fader_offset = 8;
		const int fader_x = block_x + block_w + kGap + fader_offset;
		const int fader_w = kDisplayW - fader_x - kMargin;
		if (fader_w > 4)
		{
			const char* fader_labels[3]
				= {(chorus_mode == 1) ? "DROP" : "DPTH",
				   (chorus_mode == 1) ? "RATE" : "SPD",
				   "MIX"};
			const float fader_values[3]
				= {(chorus_mode == 1) ? chorus_wow : mod_depth,
				   (chorus_mode == 1) ? tape_rate : chorus_rate,
				   fx_c_wet};
			int param_index = fx_detail_param_index;
			const bool fader_select_active = (param_index >= 0 && param_index < 3);
			if (!fader_select_active && !algo_selected)
			{
				param_index = 0;
			}
			const int fader_offsets[3] = {0, 0, 0};
			DrawVerticalFadersInRect(fader_x,
									 block_y,
									 fader_w,
									 block_h,
									 fader_labels,
									 fader_values,
									 3,
									 fader_select_active,
									 param_index,
									 fader_offsets,
									 nullptr,
									 nullptr,
									 nullptr);
		}
	}
	else if (index == kFxDelayIndex)
	{
		constexpr int kMargin = 2;
		const int block_y = Font5x7::H + 4;
		int block_h = kDisplayH - block_y - kMargin;
		if (block_h < 3)
		{
			block_h = 3;
		}
		const int fader_x = kMargin;
		const int fader_w = kDisplayW - (kMargin * 2);
		if (fader_w > 4)
		{
			const char* fader_labels[kDelayFaderCount] = {"TIM", "FBK", "SPRD", "FRZ", "MIX"};
			const float fader_values[kDelayFaderCount]
				= {delay_time, delay_feedback, delay_spread, 0.0f, delay_wet};
			int param_index = fx_detail_param_index;
			const bool fader_select_active
				= (param_index >= 0 && param_index < kDelayFaderCount);
			if (!fader_select_active)
			{
				param_index = 0;
			}
			const bool hide_handles[kDelayFaderCount] = {false, false, false, true, false};
			const bool hide_rails[kDelayFaderCount] = {false, false, false, true, false};
			const int fader_offsets[kDelayFaderCount] = {0, 0, 0, 0, 0};
			DrawVerticalFadersInRect(fader_x,
									 block_y,
									 fader_w,
									 block_h,
									 fader_labels,
									 fader_values,
									 kDelayFaderCount,
									 fader_select_active,
									 param_index,
									 fader_offsets,
									 nullptr,
									 hide_rails,
									 hide_handles);
			// FRZ status box + snow animation.
			const int label_y = block_y + block_h - Font5x7::H - 1;
			const int line_top = block_y + 2;
			const int line_bottom = label_y - 2;
			const int fader_left = fader_x + 2;
			const int fader_right = fader_x + fader_w - 3;
			const int span_x = fader_right - fader_left;
			int line_x = fader_left;
			if (kDelayFaderCount > 1 && span_x > 0)
			{
				line_x = fader_left + (span_x * 3) / (kDelayFaderCount - 1);
			}
			const char* label = "FRZ";
			const int label_w = TinyStringWidth(label);
			int label_x = line_x - (label_w / 2);
			if (label_x < fader_x + 1)
			{
				label_x = fader_x + 1;
			}
			if (label_x + label_w > fader_x + fader_w - 2)
			{
				label_x = fader_x + fader_w - 2 - label_w;
			}
			line_x = label_x + (label_w / 2);
			const bool freeze_on = (delay_freeze >= 0.5f);
			const char* on_label = "ON";
			const char* off_label = "OFF";
			const int on_w = TinyStringWidth(on_label);
			const int off_w = TinyStringWidth(off_label);
			const int text_x_on = line_x - (on_w / 2);
			const int text_x_off = line_x - (off_w / 2);
			const int state_gap = 2;
			const int text_y_on = line_top + 1;
			const int text_y_off = text_y_on + Font5x7::H + state_gap;
			const int text_top = text_y_on - 1;
			const int text_bottom = text_y_off + Font5x7::H + 1;
			const bool highlight = (fader_select_active && param_index == 3);

			auto DrawSnowflake = [&](int x, int y, bool on)
			{
				display.DrawPixel(x, y, on);
				display.DrawPixel(x - 1, y, on);
				display.DrawPixel(x + 1, y, on);
				display.DrawPixel(x, y - 1, on);
				display.DrawPixel(x, y + 1, on);
			};

			const int area_left = line_x - 6;
			const int area_right = line_x + 6;
			const int area_top = line_top;
			const int area_bottom = line_bottom;
			const int area_w = area_right - area_left + 1;
			const int area_h = area_bottom - area_top + 1;
			if (area_w > 4 && area_h > 4 && freeze_on)
			{
				const uint32_t now = System::GetNow();
				for (int i = 0; i < 6; ++i)
				{
					const int sx = area_left + static_cast<int>((now / 120 + i * 7) % area_w);
					const int sy = area_top + static_cast<int>((now / 60 + i * 9) % area_h);
					if (sy < text_top || sy > text_bottom)
					{
						DrawSnowflake(sx, sy, true);
					}
				}
			}

			if (freeze_on)
			{
				if (highlight)
				{
					display.DrawRect(text_x_on - 1,
									 text_y_on - 1,
									 text_x_on + on_w,
									 text_y_on + Font5x7::H,
									 true,
									 true);
					DrawTinyString(on_label, text_x_on, text_y_on, false);
				}
				else
				{
					DrawTinyString(on_label, text_x_on, text_y_on, true);
				}
				DrawTinyString(off_label, text_x_off, text_y_off, true);
			}
			else
			{
				DrawTinyString(on_label, text_x_on, text_y_on, true);
				if (highlight)
				{
					display.DrawRect(text_x_off - 1,
									 text_y_off - 1,
									 text_x_off + off_w,
									 text_y_off + Font5x7::H,
									 true,
									 true);
					DrawTinyString(off_label, text_x_off, text_y_off, false);
				}
				else
				{
					DrawTinyString(off_label, text_x_off, text_y_off, true);
				}
			}
		}
	}
	else if (index == kFxReverbIndex)
	{
		constexpr int kMargin = 2;
		const int block_y = Font5x7::H + 4;
		int block_h = kDisplayH - block_y - kMargin;
		if (block_h < 3)
		{
			block_h = 3;
		}
		const int fader_x = kMargin;
		const int fader_w = kDisplayW - (kMargin * 2);
		if (fader_w > 4)
		{
			const char* fader_labels[kReverbFaderCount] = {"Pre", "Dmp", "Dcy", "DIR", "Wet"};
			const float fader_values[kReverbFaderCount]
				= {reverb_pre, reverb_damp, reverb_decay, playback_reverse, reverb_wet};
			int param_index = fx_detail_param_index;
			const bool fader_select_active
				= (param_index >= 0 && param_index < kReverbFaderCount);
			if (!fader_select_active)
			{
				param_index = 0;
			}
			const bool hide_handles[kReverbFaderCount] = {false, false, false, true, false};
			const bool hide_rails[kReverbFaderCount] = {false, false, false, true, false};
			const int fader_offsets[kReverbFaderCount] = {0, 1, -1, 0, 0};
			DrawVerticalFadersInRect(fader_x,
									 block_y,
									 fader_w,
									 block_h,
									 fader_labels,
									 fader_values,
									 kReverbFaderCount,
									 fader_select_active,
									 param_index,
									 fader_offsets,
									 nullptr,
									 hide_rails,
									 hide_handles);
			// REV status box.
			const int label_y = block_y + block_h - Font5x7::H - 1;
			const int line_top = block_y + 2;
			const int line_bottom = label_y - 2;
			const int fader_left = fader_x + 2;
			const int fader_right = fader_x + fader_w - 3;
			const int span_x = fader_right - fader_left;
			int line_x = fader_left;
			if (kReverbFaderCount > 1 && span_x > 0)
			{
				line_x = fader_left + (span_x * 3) / (kReverbFaderCount - 1);
			}
			const char* label = "DIR";
			const int label_w = TinyStringWidth(label);
			int label_x = line_x - (label_w / 2);
			if (label_x < fader_x + 1)
			{
				label_x = fader_x + 1;
			}
			if (label_x + label_w > fader_x + fader_w - 2)
			{
				label_x = fader_x + fader_w - 2 - label_w;
			}
			line_x = label_x + (label_w / 2);
			const bool reverse_on = (playback_reverse >= 0.5f);
			const char* on_label = "REV";
			const char* off_label = "FOR";
			const int on_w = TinyStringWidth(on_label);
			const int off_w = TinyStringWidth(off_label);
			const int text_y_on = line_top + 1;
			const int text_y_off = text_y_on + Font5x7::H + 2;
			const int text_bottom = text_y_off + Font5x7::H + 1;
			const int text_x_on = line_x - (on_w / 2);
			const int text_x_off = line_x - (off_w / 2);
			if (reverse_on)
			{
				display.DrawRect(text_x_on - 1,
								 text_y_on - 1,
								 text_x_on + on_w,
								 text_y_on + Font5x7::H,
								 true,
								 true);
				DrawTinyString(on_label, text_x_on, text_y_on, false);
				DrawTinyString(off_label, text_x_off, text_y_off, true);
			}
			else
			{
				DrawTinyString(on_label, text_x_on, text_y_on, true);
				display.DrawRect(text_x_off - 1,
								 text_y_off - 1,
								 text_x_off + off_w,
								 text_y_off + Font5x7::H,
								 true,
								 true);
				DrawTinyString(off_label, text_x_off, text_y_off, false);
			}
			if (reverse_on)
			{
				const int area_left = line_x - 6;
				const int area_right = line_x + 6;
				const int area_top = line_top;
				const int area_bottom = line_bottom;
				const int area_w = area_right - area_left + 1;
				const int area_h = area_bottom - area_top + 1;
				if (area_w > 4 && area_h > 4)
				{
					const uint32_t now = System::GetNow();
					const int icon_top = (text_bottom + 1 > area_top)
						? (text_bottom + 1)
						: area_top;
					const int icon_bottom = area_bottom;
					const int icon_h = icon_bottom - icon_top + 1;
					if (icon_h >= 7)
					{
						const int cx = line_x + 3;
						const int cy = icon_top + (icon_h / 2);
						const int travel = 6;
						int phase = static_cast<int>((now / 100) % (travel + 2));
						int shift = travel - phase;
						if (shift < 0)
						{
							shift = travel;
						}
						// Bar (left of triangles)
						display.DrawLine(cx - 8, cy - 3, cx - 8, cy + 3, true);
						const int tri_shift = shift;
						// First triangle
						display.DrawPixel(cx - 1 - tri_shift, cy, true);
						display.DrawPixel(cx - tri_shift, cy - 1, true);
						display.DrawPixel(cx - tri_shift, cy, true);
						display.DrawPixel(cx - tri_shift, cy + 1, true);
						display.DrawPixel(cx + 1 - tri_shift, cy - 2, true);
						display.DrawPixel(cx + 1 - tri_shift, cy - 1, true);
						display.DrawPixel(cx + 1 - tri_shift, cy, true);
						display.DrawPixel(cx + 1 - tri_shift, cy + 1, true);
						display.DrawPixel(cx + 1 - tri_shift, cy + 2, true);
						display.DrawPixel(cx + 2 - tri_shift, cy - 3, true);
						display.DrawPixel(cx + 2 - tri_shift, cy - 2, true);
						display.DrawPixel(cx + 2 - tri_shift, cy - 1, true);
						display.DrawPixel(cx + 2 - tri_shift, cy, true);
						display.DrawPixel(cx + 2 - tri_shift, cy + 1, true);
						display.DrawPixel(cx + 2 - tri_shift, cy + 2, true);
						display.DrawPixel(cx + 2 - tri_shift, cy + 3, true);
						// Second triangle (overlap)
						display.DrawPixel(cx + 3 - tri_shift, cy, true);
						display.DrawPixel(cx + 4 - tri_shift, cy - 1, true);
						display.DrawPixel(cx + 4 - tri_shift, cy, true);
						display.DrawPixel(cx + 4 - tri_shift, cy + 1, true);
						display.DrawPixel(cx + 5 - tri_shift, cy - 2, true);
						display.DrawPixel(cx + 5 - tri_shift, cy - 1, true);
						display.DrawPixel(cx + 5 - tri_shift, cy, true);
						display.DrawPixel(cx + 5 - tri_shift, cy + 1, true);
						display.DrawPixel(cx + 5 - tri_shift, cy + 2, true);
						display.DrawPixel(cx + 6 - tri_shift, cy - 3, true);
						display.DrawPixel(cx + 6 - tri_shift, cy - 2, true);
						display.DrawPixel(cx + 6 - tri_shift, cy - 1, true);
						display.DrawPixel(cx + 6 - tri_shift, cy, true);
						display.DrawPixel(cx + 6 - tri_shift, cy + 1, true);
						display.DrawPixel(cx + 6 - tri_shift, cy + 2, true);
						display.DrawPixel(cx + 6 - tri_shift, cy + 3, true);
					}
				}
			}
		}
	}
	RequestDisplayUpdate();
}

void DrawLoadModeSelect(int32_t selected)
{
	display.Fill(false);
	constexpr int kMargin = 2;
	constexpr int kGap = 4;
	const int box_h = (kDisplayH - (kMargin * 2) - kGap) / 2;
	const int box_w = kDisplayW - (kMargin * 2);
	const int top_y = kMargin;
	const int bottom_y = kMargin + box_h + kGap;
	const int x = kMargin;

	auto draw_box = [&](int y, const char* label, bool highlight)
	{
		display.DrawRect(x, y, x + box_w - 1, y + box_h - 1, true, highlight);
		const int text_w = static_cast<int>(StrLen(label)) * (Font5x7::W + 1);
		int text_x = x + (box_w - text_w) / 2;
		if (text_x < x + 1)
		{
			text_x = x + 1;
		}
		const int text_y = y + (box_h - Font5x7::H) / 2;
		DrawTinyString(label, text_x, text_y, !highlight);
	};

	draw_box(top_y, "PRESETS", selected == 0);
	draw_box(bottom_y, "BAKE", selected == 1);
	RequestDisplayUpdate();
}

void DrawLoadStubScreen(LoadStubMode mode)
{
	display.Fill(false);
	const char* line1 = (mode == LoadStubMode::Presets) ? "PRESETS" : "BAKE";
	const char* line2 = "COMING SOON";
	const int text_w1 = TinyStringWidth(line1);
	const int text_w2 = TinyStringWidth(line2);
	const int x1 = (kDisplayW - text_w1) / 2;
	const int x2 = (kDisplayW - text_w2) / 2;
	const int y1 = (kDisplayH / 2) - Font5x7::H - 2;
	const int y2 = y1 + Font5x7::H + 4;
	DrawTinyString(line1, x1, y1, true);
	DrawTinyString(line2, x2, y2, true);
	RequestDisplayUpdate();
}

void DrawPresetSaveStub()
{
	display.Fill(false);
	const char* line1 = "ABILITY TO";
	const char* line2 = "SAVE PRESETS";
	const char* line3 = "COMING SOON";
	const int w1 = TinyStringWidth(line1);
	const int w2 = TinyStringWidth(line2);
	const int w3 = TinyStringWidth(line3);
	const int x1 = (kDisplayW - w1) / 2;
	const int x2 = (kDisplayW - w2) / 2;
	const int x3 = (kDisplayW - w3) / 2;
	const int y1 = (kDisplayH / 2) - Font5x7::H - 6;
	const int y2 = y1 + Font5x7::H + 4;
	const int y3 = y2 + Font5x7::H + 4;
	DrawTinyString(line1, x1, y1, true);
	DrawTinyString(line2, x2, y2, true);
	DrawTinyString(line3, x3, y3, true);
	RequestDisplayUpdate();
}

void DrawRecordTargetScreen(int32_t selected)
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("SAVE SAMPLE?", font, true);
	display.SetCursor(0, (font.FontHeight + 2) * 2);
	display.WriteString("L=NO  R=YES", font, true);
	RequestDisplayUpdate();
}

