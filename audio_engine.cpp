#include "audio_engine.h"
#include "audio_dsp.h"
#include "SamplerConfig.h"
#include "shared_messages.h"
#include "PerformVoice.h"
#include "VoiceManager.h"
#include "SampleMemoryManager.h"
#include "util/scopedirqblocker.h"
#include "daisy_pod.h"
#include "daisysp.h"
#include <cmath>

#ifndef STORAGE_SERVICE_PREVIEW_STREAM
#define STORAGE_SERVICE_PREVIEW_STREAM PREVIEW_STREAM_FROM_SD
#endif

#ifndef PERF_DIAGNOSTICS
#define PERF_DIAGNOSTICS 1
#endif

#if PERF_DIAGNOSTICS
#define PERF_CYCLES_START(var) const uint32_t var = DWT->CYCCNT
#define PERF_CYCLES_END(var)   const uint32_t var = DWT->CYCCNT
#else
#define PERF_CYCLES_START(var) do {} while (0)
#define PERF_CYCLES_END(var)   do {} while (0)
#endif

using namespace daisy;
using namespace daisysp;

constexpr uint8_t kMidiCmdQSize = 16;
constexpr uint8_t kPlaybackCmdQSize = 8;

constexpr int32_t kPerformFaderCount = 4;
constexpr int32_t kFxSatIndex = 0;
constexpr int32_t kFxChorusIndex = 1;
constexpr int32_t kFxDelayIndex = 2;
constexpr int32_t kFxReverbIndex = 3;
constexpr int kPerformVoiceCount = kMaxVoices;
constexpr float kSampleScale = 1.0f / 32768.0f;
constexpr float kFxParamEpsilon = 1e-5f;
constexpr int kBitcrushMaxHold = 32;
constexpr float kChorusWidthMax = 2.2f;
constexpr size_t kDelayMaxSamples = 96000;
constexpr float kDelayFeedbackMax = 0.98f;
constexpr size_t kReverbPreDelayMaxSamples = 48000;
constexpr size_t kPreviewPpFrames = 2048;
constexpr uint32_t kPerformSampleId = 1;
constexpr float kSilentAmp = 1.0e-4f;
constexpr uint16_t kSilentSamplesToKill = 64;
constexpr size_t kRecordMaxFrames = kMaxSampleFrames;
constexpr float kRecordWaveformScaleMaxMic = 1.75f;
constexpr float kRecordWaveformScaleMaxLine = 2.0f;
constexpr size_t kLiveWaveWindowFrames = kSampleRateHz / 2;
constexpr size_t kLiveWaveStride = (kLiveWaveWindowFrames / kWaveCols) > 0
	? (kLiveWaveWindowFrames / kWaveCols)
	: 1;

// Extern shared state owned by WaveContV3.cpp (UI/control thread).
extern daisy::DaisyPod hw;
extern volatile uint32_t g_audio_cmd;
extern volatile uint32_t g_audio_flags_bits;
extern volatile RecordInput record_input;
extern volatile bool g_playback_reverse_target;
extern volatile uint8_t g_audio_params_pub_idx;
extern volatile uint8_t g_rt_pub_idx;
extern volatile uint8_t g_fx_chain_pub_idx;
extern volatile uint8_t g_preview_pub_idx;
extern volatile float phones_volume;
extern volatile bool g_audio_recording_active;
extern volatile size_t g_recorded_length_audio;
extern volatile size_t record_pos;
extern bool g_reset_voices_pending;
extern AudioParams g_audio_params_buf[2];
extern volatile uint8_t g_audio_params_active_idx;
extern AudioUiState g_audio_ui_state_buf[2];
extern volatile uint8_t g_audio_ui_state_idx;
extern volatile bool playback_active;
extern volatile float playback_phase;
extern volatile bool g_perform_voices_active;
extern volatile bool preview_active;
extern volatile size_t preview_read_index;
extern volatile float preview_read_frac;
extern volatile uint32_t preview_fade_samples_left;
extern volatile uint32_t preview_fade_samples_total;
extern PreviewControl g_preview_ctl_buf[2];
extern volatile uint8_t g_preview_active_idx;
extern volatile size_t preview_write_index;
extern SampleRuntime g_rt_buf[2];
extern volatile uint8_t g_rt_active_idx;
extern FxChainRuntime g_fx_chain_buf[2];
extern volatile uint8_t g_fx_chain_active_idx;
extern FxChainRuntime g_fx_chain_audio;
extern bool g_fx_chain_audio_valid;
extern FxParamsAudio g_fx_params_buf[2];
extern volatile uint8_t g_fx_params_idx;
extern AudioParamsAudio g_audio_params_audio_buf[2];
extern volatile uint8_t g_audio_params_audio_idx;
extern volatile float g_delay_time_alpha;
extern volatile float g_delay_param_alpha;
extern int16_t* sample_buffer_l;
extern int16_t* sample_buffer_r;
extern SampleMemoryManager sample_mem_mgr;
extern PerformVoice perform_voices[];
extern BiquadLp perform_lpf_l1[];
extern BiquadLp perform_lpf_l2[];
extern BiquadLp perform_lpf_r1[];
extern BiquadLp perform_lpf_r2[];
extern ReverbSc reverb;
extern DelayLine<float, kDelayMaxSamples> delay_line_l;
extern DelayLine<float, kDelayMaxSamples> delay_line_r;
extern DelayLine<float, kReverbPreDelayMaxSamples> reverb_predelay_l;
extern DelayLine<float, kReverbPreDelayMaxSamples> reverb_predelay_r;
extern ChorusEngine chorus_l;
extern ChorusEngine chorus_r;
extern TapeSaturator sat_l;
extern TapeSaturator sat_r;
extern BitCrushState g_sat_bit_state;
#if STORAGE_SERVICE_PREVIEW_STREAM
extern volatile bool preview_preload_active;
extern volatile size_t preview_preload_frames;
extern int16_t preview_preload_buf[];
extern int16_t preview_pp_buf[2][kPreviewPpFrames];
extern volatile uint8_t preview_pp_ready[2];
extern volatile uint8_t preview_pp_active;
extern volatile uint32_t preview_pp_pos;
extern volatile uint32_t preview_underrun_count;
extern volatile uint32_t preview_rb_min_level;
#endif
extern int16_t preview_buffer[];
extern void PushAudioEvent(uint32_t bits);
extern void ResetPerformVoices();
extern void StartRecordingAudioRT();
extern void ApplyPlaybackReverseAudio(bool reverse);
extern void StartPlaybackAudio(uint8_t note, bool apply_pitch, bool reverse_playback);
extern void StopPlaybackAudio(uint8_t note, bool apply_release);
extern void StopPlaybackAllAudio();
extern void StartPerformVoice(int32_t note);
extern void StopPerformVoice(int32_t note);
extern void DeactivateVoice(PerformVoice& voice);
extern void AudioUiResetLiveWaveform(AudioUiState& uiw);
extern size_t PreviewAvailableFrames(size_t read_idx, size_t write_idx);
extern volatile float playback_rate;
extern volatile float playback_amp;
extern volatile size_t playback_env_samples;
extern volatile bool playback_release_active;
extern volatile float playback_release_pos;
extern volatile float playback_release_start;
extern volatile bool playback_reverse_active;
extern volatile bool preview_hold;
extern volatile int32_t preview_index;
extern volatile uint32_t preview_sample_rate;
extern volatile uint16_t preview_channels;
extern volatile int32_t g_active_voice_count;
extern uint32_t g_voice_skip_count;
extern uint32_t g_voice_kill_count;
extern volatile float cpu_load_pct;
extern volatile float cpu_load_peak_pct;
extern volatile uint32_t callback_cycles_last;
extern volatile uint32_t callback_cycles_max;
extern volatile uint32_t callback_overruns;
extern float cpu_load_ema;
extern volatile bool request_playhead_redraw;
extern float SineTableLookup(float phase01);
extern MidiCmd g_midi_cmd_q[];
extern volatile uint8_t g_midi_cmd_wr;
extern volatile uint8_t g_midi_cmd_rd;
extern PlaybackCmd g_playback_cmd_q[];
extern volatile uint8_t g_playback_cmd_wr;
extern volatile uint8_t g_playback_cmd_rd;
static inline bool AnyPerformVoiceActive()
{
	return g_active_voice_count > 0;
}

static inline void ReleaseVoiceSample(PerformVoice& voice)
{
	if (voice.sample_acquired)
	{
		sample_mem_mgr.Release(kPerformSampleId);
		voice.sample_acquired = false;
	}
}

void DeactivateVoice(PerformVoice& voice)
{
	if (voice.active)
	{
		ReleaseVoiceSample(voice);
		voice.active = false;
		if (g_active_voice_count > 0)
		{
			--g_active_voice_count;
		}
	}
	voice.releasing = false;
	voice.sample_acquired = false;
	voice.phase = 0.0f;
	voice.rate = 1.0f;
	voice.amp = 1.0f;
	voice.env = 0.0f;
	voice.release_start = 0.0f;
	voice.release_pos = 0.0f;
	voice.note = -1;
	voice.offset = 0;
	voice.length = 0;
	voice.env_samples = 0;
	voice.silent_samples = 0;
}

void ResetPerformVoices()
{
	for (int i = 0; i < kPerformVoiceCount; ++i)
	{
		DeactivateVoice(perform_voices[i]);
	}
	g_active_voice_count = 0;
	for (int i = 0; i < kPerformVoiceCount; ++i)
	{
		perform_lpf_l1[i].Reset();
		perform_lpf_l2[i].Reset();
		perform_lpf_r1[i].Reset();
		perform_lpf_r2[i].Reset();
	}
}
static inline uint8_t MidiCmdPopBatchAudio(MidiCmd* out, uint8_t max_n)
{
	daisy::ScopedIrqBlocker irq;
	uint8_t rd = g_midi_cmd_rd;
	const uint8_t wr = g_midi_cmd_wr;
	uint8_t n = 0;
	while (rd != wr && n < max_n)
	{
		out[n++] = g_midi_cmd_q[rd];
		rd = (uint8_t)((rd + 1) & (kMidiCmdQSize - 1));
	}
	g_midi_cmd_rd = rd;
	return n;
}
static inline uint8_t PlaybackCmdPopBatchAudio(PlaybackCmd* out, uint8_t max_n)
{
	daisy::ScopedIrqBlocker irq;
	uint8_t rd = g_playback_cmd_rd;
	const uint8_t wr = g_playback_cmd_wr;
	uint8_t n = 0;
	while (rd != wr && n < max_n)
	{
		out[n++] = g_playback_cmd_q[rd];
		rd = (uint8_t)((rd + 1) & (kPlaybackCmdQSize - 1));
	}
	g_playback_cmd_rd = rd;
	return n;
}
#ifndef AUDIO_HOT_GUARD
#define AUDIO_HOT_GUARD 1
#endif
#if AUDIO_HOT_GUARD
#define powf  AUDIO_HOT_POWF
#define expf  AUDIO_HOT_EXPF
#define logf  AUDIO_HOT_LOGF
#define sinf  AUDIO_HOT_SINF
#define cosf  AUDIO_HOT_COSF
#define sqrtf AUDIO_HOT_SQRTF
#define tanf  AUDIO_HOT_TANF
#define atanf AUDIO_HOT_ATANF
#define asinf AUDIO_HOT_ASINF
#define acosf AUDIO_HOT_ACOSF
#endif
void AudioCallbackImpl(daisy::AudioHandle::InputBuffer in, daisy::AudioHandle::OutputBuffer out, size_t size)
{
	PERF_CYCLES_START(cyc_start);

	uint32_t cmd = 0;
	uint32_t flags_bits = 0;
	RecordInput record_input_audio = RecordInput::LineIn;
	bool reverse_target = false;
	uint8_t pub_params_idx = 0;
	uint8_t pub_rt_idx = 0;
	uint8_t pub_fx_idx = 0;
	uint8_t pub_preview_idx = 0;
	{
		daisy::ScopedIrqBlocker irq;
		cmd = g_audio_cmd;
		g_audio_cmd = 0;
		flags_bits = g_audio_flags_bits;
		record_input_audio = record_input;
		reverse_target = g_playback_reverse_target;
		pub_params_idx = g_audio_params_pub_idx;
		pub_rt_idx = g_rt_pub_idx;
		pub_fx_idx = g_fx_chain_pub_idx;
		pub_preview_idx = g_preview_pub_idx;
	}
	float phones_gain = 1.0f;
	{
		daisy::ScopedIrqBlocker irq;
		phones_gain = phones_volume;
	}
	const bool apply_reverse_cmd = (cmd & kCmdPlaybackReverse) != 0;

	if (cmd & kCmdCommitRuntime)
	{
		g_rt_active_idx = pub_rt_idx;
	}
	if (cmd & kCmdCommitFxChain)
	{
		g_fx_chain_active_idx = pub_fx_idx;
		g_fx_chain_audio = g_fx_chain_buf[g_fx_chain_active_idx];
		g_fx_chain_audio_valid = true;
	}
	if (cmd & kCmdCommitPreview)
	{
		g_preview_active_idx = pub_preview_idx;
	}
	if (cmd & kCmdCommitAudioParams)
	{
		g_audio_params_active_idx = pub_params_idx;
	}
	if (cmd & kCmdPreviewStart)
	{
		preview_active = true;
		preview_read_index = 0;
		preview_read_frac = 0.0f;
	}
	if (cmd & kCmdPreviewStop)
	{
		preview_active = false;
		preview_read_index = 0;
		preview_read_frac = 0.0f;
	}
	if (cmd & kCmdRecStart)
	{
		if (!g_audio_recording_active)
		{
			StartRecordingAudioRT();
		}
	}

	if (cmd & kCmdRecStop)
	{
		if (g_audio_recording_active)
		{
			g_recorded_length_audio = record_pos;
			g_audio_recording_active = false;
			PushAudioEvent(kAudioEventRecFinished);
		}
	}
	if (cmd & kCmdAllNotesOff)
	{
		g_reset_voices_pending = true;
	}
	if (g_reset_voices_pending)
	{
		g_reset_voices_pending = false;
		ResetPerformVoices();
	}

	const AudioParams& params = g_audio_params_buf[g_audio_params_active_idx];
	const bool reverse_playback = (params.playback_reverse >= 0.5f);
	const uint8_t ui_pub_idx = g_audio_ui_state_idx;
	const uint8_t ui_next_idx = ui_pub_idx ^ 1u;
	const AudioUiState& uiprev = g_audio_ui_state_buf[ui_pub_idx];
	AudioUiState& uiw = g_audio_ui_state_buf[ui_next_idx];
	uiw = uiprev;
	bool ui_wave_dirty = false;
	bool ui_state_changed = false;
	const bool preview_active_now = preview_active;
	const bool playback_active_now = playback_active;
	const bool perform_active_now = g_perform_voices_active;
	if (uiprev.preview_active != preview_active_now
		|| uiprev.playback_active != playback_active_now
		|| uiprev.perform_voices_active != perform_active_now)
	{
		ui_state_changed = true;
	}
	uiw.preview_active = preview_active_now;
	uiw.preview_read_index = static_cast<uint32_t>(preview_read_index);
	uiw.playback_active = playback_active_now;
	uiw.playback_phase = playback_phase;
	uiw.perform_voices_active = perform_active_now;
	#if PERF_DIAGNOSTICS
	uiw.cpu_load_pct = cpu_load_pct;
	uiw.cpu_load_peak_pct = cpu_load_peak_pct;
	uiw.callback_cycles_last = callback_cycles_last;
	uiw.callback_cycles_max = callback_cycles_max;
	uiw.callback_overruns = callback_overruns;
	#endif
	{
		static uint8_t ui_phase_tick = 0;
		const bool want_phase_update = playback_active_now || preview_active_now;
		if (want_phase_update)
		{
			++ui_phase_tick;
			if (ui_phase_tick >= 4)
			{
				ui_state_changed = true;
				ui_phase_tick = 0;
			}
		}
		else
		{
			ui_phase_tick = 0;
		}
	}
	if (cmd & kCmdRecStart)
	{
		if (!g_audio_recording_active)
		{
			AudioUiResetLiveWaveform(uiw);
			ui_wave_dirty = true;
		}
	}
	if (apply_reverse_cmd)
	{
		ApplyPlaybackReverseAudio(reverse_target);
	}
	// Apply queued MIDI note commands in audio thread (low latency).
	bool record_active = g_audio_recording_active;
	MidiCmd batch[4];
	const uint8_t n = MidiCmdPopBatchAudio(batch, 4);
	for (uint8_t i = 0; i < n; ++i)
	{
		const MidiCmd& c = batch[i];
		if (record_active)
		{
			continue;
		}
		const bool in_perform = (flags_bits & kFlagInPerformMode) != 0;
		if (in_perform)
		{
			if (c.kind == kMidiCmdNoteOn)
			{
				StartPerformVoice((int32_t)c.note);
			}
			else
			{
				StopPerformVoice((int32_t)c.note);
			}
		}
		else
		{
			if (c.kind == kMidiCmdNoteOn)
			{
				StartPlaybackAudio((uint8_t)c.note, true, reverse_playback);
			}
			else
			{
				StopPlaybackAudio((uint8_t)c.note, false);
			}
		}
	}
	PlaybackCmd pb_batch[4];
	const uint8_t pb_n = PlaybackCmdPopBatchAudio(pb_batch, 4);
	for (uint8_t i = 0; i < pb_n; ++i)
	{
		const PlaybackCmd& c = pb_batch[i];
		if (record_active)
		{
			continue;
		}
		if (c.kind == kPlaybackCmdStart)
		{
			const bool apply_pitch = (c.flags & kPlaybackCmdApplyPitch) != 0;
			StartPlaybackAudio(c.note, apply_pitch, reverse_playback);
		}
		else if (c.kind == kPlaybackCmdStop)
		{
			const bool apply_release = (c.flags & kPlaybackCmdApplyRelease) != 0;
			StopPlaybackAudio(c.note, apply_release);
		}
		else if (c.kind == kPlaybackCmdStopAll)
		{
			StopPlaybackAllAudio();
		}
	}
	const FxChainRuntime& fxrt = g_fx_chain_audio_valid
		? g_fx_chain_audio
		: g_fx_chain_buf[g_fx_chain_active_idx];
	const PreviewControl& pctl = g_preview_ctl_buf[g_preview_active_idx];
	const float out_sr = hw.AudioSampleRate();
	const SampleRuntime& rt = g_rt_buf[g_rt_active_idx];
	const int16_t* rt_l = rt.l;
	const int16_t* rt_r = rt.r ? rt.r : rt.l;
	size_t window_start = rt.play_start;
	size_t window_end = rt.play_end;
	const size_t length = rt.length;
	if (window_end > length || window_end == 0)
	{
		window_end = length;
	}
	if (window_end <= window_start)
	{
		window_start = 0;
		window_end = length;
	}
	const bool window_valid = (window_end > 0 && window_end > window_start);
	if (playback_active && (!rt.loaded || !window_valid || rt.l == nullptr))
	{
		playback_active = false;
		playback_env_samples = 0;
		playback_release_active = false;
		playback_release_pos = 0.0f;
		playback_release_start = 0.0f;
	}

static float cached_sat_mix = 0.0f;
static float cached_sat_bump = 0.0f;
static float cached_sat_smpl = 0.0f;
static int32_t cached_sat_mode = 0;
static float cached_chorus_depth = 0.0f;
static float cached_chorus_mix = 0.0f;
static int32_t cached_chorus_mode = 0;
static float cached_chorus_wow = 0.0f;
static float cached_tape_rate = 0.0f;
	static float cached_delay_wet = 0.0f;
	static float cached_delay_feedback = 0.0f;
	static float cached_delay_spread = 0.0f;
	static float cached_delay_freeze = 0.0f;
	static float delay_time_smoothed = -1.0f;
	static float delay_feedback_smoothed = -1.0f;
	static float delay_spread_smoothed = -1.0f;
	static float cached_reverb_wet = 0.0f;
	static float cached_reverb_gain = 1.0f;
	static float cached_reverb_release = 1.0f;
	static float cached_reverb_predelay_samples = 0.0f;
static float last_sat_drive = -1.0f;
static float last_sat_bump = -1.0f;
static int32_t last_sat_mode = -1;
static float last_chorus_depth = -1.0f;
static float last_chorus_rate = -1.0f;
static float last_chorus_wow = -1.0f;
	static float last_delay_wet = -1.0f;
	static float last_delay_time = -1.0f;
	static float last_delay_feedback = -1.0f;
	static float last_delay_spread = -1.0f;
	static float last_delay_freeze = -1.0f;
	static float last_rev_feedback = -1.0f;
	static float last_rev_lp = -1.0f;
	static float last_rev_predelay = -1.0f;

	FxParamsAudio fxp;
	{
		daisy::ScopedIrqBlocker irq;
		fxp = g_fx_params_buf[g_fx_params_idx];
	}
	cached_sat_mix = fxp.sat_mix;
	cached_sat_bump = fxp.sat_bump;
	cached_sat_smpl = params.sat_bit_smpl;
	cached_sat_mode = fxp.sat_mode;
	cached_chorus_depth = params.mod_depth;
	cached_chorus_mix = fxp.chorus_mix;
	cached_chorus_mode = fxp.chorus_mode;
	cached_chorus_wow = fxp.chorus_wow;
	cached_tape_rate = fxp.tape_rate;
	cached_delay_wet = fxp.delay_wet;
	cached_delay_feedback = fxp.delay_feedback;
	cached_delay_spread = fxp.delay_spread;
	cached_delay_freeze = fxp.delay_freeze;
	cached_reverb_wet = fxp.reverb_wet;
	cached_reverb_gain = fxp.reverb_gain;
	cached_reverb_release = fxp.reverb_release;
	cached_reverb_predelay_samples = fxp.reverb_predelay_samples;

	if (cached_sat_mode != last_sat_mode)
	{
		last_sat_mode = cached_sat_mode;
	}
	if (cached_sat_mode == 0)
	{
		const float sat_drive_amt = fxp.sat_drive_amt;
		if (fabsf(sat_drive_amt - last_sat_drive) > kFxParamEpsilon)
		{
			sat_l.SetDrive(sat_drive_amt);
			sat_r.SetDrive(sat_drive_amt);
			last_sat_drive = sat_drive_amt;
		}
		if (fabsf(cached_sat_bump - last_sat_bump) > kFxParamEpsilon)
		{
			sat_l.SetBump(cached_sat_bump);
			sat_r.SetBump(cached_sat_bump);
			last_sat_bump = cached_sat_bump;
		}
	}
	if (fabsf(fxp.chorus_depth_mapped - last_chorus_depth) > kFxParamEpsilon)
	{
		if (cached_chorus_mode == 0)
		{
			chorus_l.SetLfoDepth(fxp.chorus_depth_mapped);
			chorus_r.SetLfoDepth(fxp.chorus_depth_mapped);
		}
		last_chorus_depth = fxp.chorus_depth_mapped;
	}
	if (fabsf(fxp.chorus_rate_hz - last_chorus_rate) > kFxParamEpsilon)
	{
		chorus_l.SetLfoFreq(fxp.chorus_rate_hz);
		chorus_r.SetLfoFreq(-fxp.chorus_rate_hz);
		last_chorus_rate = fxp.chorus_rate_hz;
	}
	if (fabsf(cached_chorus_wow - last_chorus_wow) > kFxParamEpsilon)
	{
		last_chorus_wow = cached_chorus_wow;
	}
	if (fabsf(cached_delay_wet - last_delay_wet) > kFxParamEpsilon)
	{
		last_delay_wet = cached_delay_wet;
	}
	if (fabsf(cached_delay_feedback - last_delay_feedback) > kFxParamEpsilon)
	{
		last_delay_feedback = cached_delay_feedback;
	}
	if (fabsf(cached_delay_spread - last_delay_spread) > kFxParamEpsilon)
	{
		last_delay_spread = cached_delay_spread;
	}
	if (fabsf(cached_delay_freeze - last_delay_freeze) > kFxParamEpsilon)
	{
		last_delay_freeze = cached_delay_freeze;
	}
	if (fabsf(fxp.reverb_feedback - last_rev_feedback) > kFxParamEpsilon)
	{
		reverb.SetFeedback(fxp.reverb_feedback);
		last_rev_feedback = fxp.reverb_feedback;
	}
	if (fabsf(fxp.reverb_lp_hz - last_rev_lp) > kFxParamEpsilon)
	{
		reverb.SetLpFreq(fxp.reverb_lp_hz);
		last_rev_lp = fxp.reverb_lp_hz;
	}
	if (fabsf(fxp.reverb_predelay_samples - last_rev_predelay) >= 0.5f)
	{
		reverb_predelay_l.SetDelay(fxp.reverb_predelay_samples);
		reverb_predelay_r.SetDelay(fxp.reverb_predelay_samples);
		last_rev_predelay = fxp.reverb_predelay_samples;
	}

	static float drop_phase = 0.0f;
	static float trem_phase = 0.0f;
	static float drop_gain = 1.0f;
	static float drop_target = 1.0f;
	static int drop_hold = 0;
	static uint32_t drop_rng = 0x12345678;
	static int bit_hold = 0;
	static float bit_hold_l = 0.0f;
	static float bit_hold_r = 0.0f;
	static float reverb_tail_gain = 0.0f;

	const int32_t sat_mode_local = cached_sat_mode;
	const float sat_mix = cached_sat_mix;
	const float bit_smpl = cached_sat_smpl;
	const int32_t chorus_mode_local = cached_chorus_mode;
	const float chorus_mix = cached_chorus_mix;
	const float delay_mix = cached_delay_wet;
	const bool sat_active = (sat_mix > kFxParamEpsilon);
	const bool chorus_active = (chorus_mix > kFxParamEpsilon);
	const bool delay_active = (delay_mix > kFxParamEpsilon)
		|| (cached_delay_freeze >= 0.5f);
	const bool reverb_active = (cached_reverb_wet > kFxParamEpsilon);
	const float bit_step = fxp.bit_step;
	const float tape_drop_amt_mapped = fxp.tape_drop_amt_mapped;
	float time_alpha = 1.0f;
	float param_alpha = 1.0f;
	{
		daisy::ScopedIrqBlocker irq;
		time_alpha = g_delay_time_alpha;
		param_alpha = g_delay_param_alpha;
	}
	const float max_delay = static_cast<float>(kDelayMaxSamples - 1);
	float delay_target = fxp.delay_time_samples;
	if (delay_target > max_delay)
	{
		delay_target = max_delay;
	}
	if (delay_target < 1.0f)
	{
		delay_target = 1.0f;
	}
	if (delay_time_smoothed < 0.0f)
	{
		delay_time_smoothed = delay_target;
	}
	delay_time_smoothed += (delay_target - delay_time_smoothed) * time_alpha;
	if (delay_feedback_smoothed < 0.0f)
	{
		delay_feedback_smoothed = cached_delay_feedback;
	}
	delay_feedback_smoothed += (cached_delay_feedback - delay_feedback_smoothed) * param_alpha;
	if (delay_spread_smoothed < 0.0f)
	{
		delay_spread_smoothed = cached_delay_spread;
	}
	delay_spread_smoothed += (cached_delay_spread - delay_spread_smoothed) * param_alpha;
	if (fabsf(delay_time_smoothed - last_delay_time) > kFxParamEpsilon)
	{
		delay_line_l.SetDelay(delay_time_smoothed);
		delay_line_r.SetDelay(delay_time_smoothed);
		last_delay_time = delay_time_smoothed;
	}
	AudioParamsAudio ap;
	{
		daisy::ScopedIrqBlocker irq;
		ap = g_audio_params_audio_buf[g_audio_params_audio_idx];
	}
	const bool perform_mode = (flags_bits & kFlagInPerformMode) != 0;
	const bool main_mode = (flags_bits & kFlagInMainMode) != 0;
	const bool fx_allowed = (flags_bits & kFlagFxAllowed) != 0;
	const bool amp_env_active = perform_mode;
	const float amp_attack_samples = ap.amp_attack_samples;
	const float amp_release_samples = ap.amp_release_samples;
	const float inv_attack = (amp_attack_samples > 1.0f)
		? (1.0f / amp_attack_samples)
		: 0.0f;
	const float inv_release = (amp_release_samples > 1.0f)
		? (1.0f / amp_release_samples)
		: 0.0f;
	const bool use_poly = (!record_active) && (perform_mode && rt.loaded);
	const bool sample_stereo = (rt.channels == 2);
	static BiquadCoeffs last_flt_coeffs = {};
	static bool has_flt_coeffs = false;
	if (!has_flt_coeffs
		|| ap.flt_coeffs.a0 != last_flt_coeffs.a0
		|| ap.flt_coeffs.a1 != last_flt_coeffs.a1
		|| ap.flt_coeffs.a2 != last_flt_coeffs.a2
		|| ap.flt_coeffs.b1 != last_flt_coeffs.b1
		|| ap.flt_coeffs.b2 != last_flt_coeffs.b2)
	{
		for (int v = 0; v < kPerformVoiceCount; ++v)
		{
			perform_lpf_l1[v].SetCoeffs(ap.flt_coeffs);
			perform_lpf_l2[v].SetCoeffs(ap.flt_coeffs);
			perform_lpf_r1[v].SetCoeffs(ap.flt_coeffs);
			perform_lpf_r2[v].SetCoeffs(ap.flt_coeffs);
		}
		last_flt_coeffs = ap.flt_coeffs;
		has_flt_coeffs = true;
	}
	int32_t fx_order[kPerformFaderCount];
	for (int i = 0; i < kPerformFaderCount; ++i)
	{
		fx_order[i] = fxrt.order[i];
	}

	auto apply_saturation = [&](float &l, float &r)
	{
		if (sat_mix <= kFxParamEpsilon)
		{
			return;
		}
		const float dry_l = l;
		const float dry_r = r;
		float wet_l = l;
		float wet_r = r;
		if (sat_mode_local == 0)
		{
			wet_l = sat_l.Process(l);
			wet_r = sat_r.Process(r);
		}
		else
		{
			int hold_samples = 1 + static_cast<int>(bit_smpl * static_cast<float>(kBitcrushMaxHold - 1));
			if (hold_samples < 1)
			{
				hold_samples = 1;
			}
			if (bit_hold <= 0)
			{
				bit_hold = hold_samples;
				bit_hold_l = l;
				bit_hold_r = r;
			}
			else
			{
				--bit_hold;
			}
			const float step = bit_step;
			wet_l = roundf(bit_hold_l / step) * step;
			wet_r = roundf(bit_hold_r / step) * step;
			if (wet_l > 1.0f) wet_l = 1.0f;
			if (wet_l < -1.0f) wet_l = -1.0f;
			if (wet_r > 1.0f) wet_r = 1.0f;
			if (wet_r < -1.0f) wet_r = -1.0f;
		}
		l = (dry_l * (1.0f - sat_mix)) + (wet_l * sat_mix);
		r = (dry_r * (1.0f - sat_mix)) + (wet_r * sat_mix);
	};

	auto apply_chorus = [&](float &l, float &r)
	{
		const float dry_l = l;
		const float dry_r = r;
		float chorus_proc_l = chorus_l.Process(l);
		float chorus_proc_r = chorus_r.Process(r);
		float tape_drop = 1.0f;
		if (chorus_mode_local == 1)
		{
			const float drop_amt = cached_chorus_wow;
			if (drop_amt > 0.0f)
			{
				const float drop_amt_mapped = tape_drop_amt_mapped;
				const float drop_curve = drop_amt_mapped * drop_amt_mapped;
				const float rate_curve = cached_tape_rate * cached_tape_rate;
				const float rate_scale = 0.2f + (rate_curve * 6.0f);
				const float drop_rate = (0.2f + (drop_curve * 12.0f)) * rate_scale;
				const float drop_step = drop_rate / out_sr;
				drop_phase += drop_step;
				bool new_step = false;
				if (drop_phase >= 1.0f)
				{
					drop_phase -= 1.0f;
					new_step = true;
				}
				drop_rng = (drop_rng * 1664525u) + 1013904223u;
				const float r = static_cast<float>((drop_rng >> 8) & 0xFFFF) / 65535.0f;
				if (drop_hold > 0)
				{
					drop_hold--;
					drop_target = 0.0f;
				}
				else if (new_step)
				{
					const float drop_prob = 0.05f + (drop_curve * 0.9f);
					if (r < drop_prob)
					{
						const float hold_scale = 1.0f / (0.5f + rate_curve * 2.0f);
						drop_hold = 10 + static_cast<int>(r * 1200.0f * drop_curve * hold_scale);
						drop_target = 0.0f;
					}
					else
					{
						drop_target = 1.0f - (drop_curve * 0.9f) + (r * drop_curve * 0.9f);
					}
				}
				const float drop_slew = 0.08f + (drop_curve * 0.8f);
				drop_gain += (drop_target - drop_gain) * drop_slew;
				trem_phase += (1.0f + drop_curve * 20.0f) * rate_scale / out_sr;
				if (trem_phase >= 1.0f)
				{
					trem_phase -= 1.0f;
				}
				const float trem = 0.5f * (1.0f + SineTableLookup(trem_phase));
				const float trem_depth = drop_curve * 0.85f;
				tape_drop = drop_gain * (1.0f - trem_depth + (trem_depth * trem));
			}
		}
		float wet_l = chorus_proc_l * tape_drop;
		float wet_r = chorus_proc_r * tape_drop;
		if (chorus_mode_local == 0)
		{
			float width = 1.0f + (cached_chorus_depth * (kChorusWidthMax - 1.0f));
			if (width < 1.0f)
			{
				width = 1.0f;
			}
			if (width > kChorusWidthMax)
			{
				width = kChorusWidthMax;
			}
			const float mid = 0.5f * (wet_l + wet_r);
			const float side = 0.5f * (wet_l - wet_r);
			wet_l = mid + (side * width);
			wet_r = mid - (side * width);
		}
		l = (dry_l * (1.0f - chorus_mix)) + (wet_l * chorus_mix);
		r = (dry_r * (1.0f - chorus_mix)) + (wet_r * chorus_mix);
	};

	auto apply_delay = [&](float &l, float &r)
	{
		const float freeze = (cached_delay_freeze >= 0.5f) ? 1.0f : 0.0f;
		float feedback = delay_feedback_smoothed;
		if (feedback > kDelayFeedbackMax)
		{
			feedback = kDelayFeedbackMax;
		}
		if (feedback < 0.0f)
		{
			feedback = 0.0f;
		}
		const float freeze_mix = (freeze > 0.0f) ? freeze : 0.0f;
		const float feedback_mix = feedback + (freeze_mix * (1.0f - feedback));
		const float input_gain = 1.0f - freeze_mix;
		const float delay_in = 0.5f * (l + r);
		const float pingpong = feedback;
		const float input_l = delay_in * input_gain;
		const float input_r = delay_in * input_gain * (1.0f - pingpong);
		const float delay_out_l = delay_line_l.Read();
		const float delay_out_r = delay_line_r.Read();
		// Ping-pong delay with spread.
		float fb_l = delay_out_r * feedback_mix;
		float fb_r = delay_out_l * feedback_mix;
		delay_line_l.Write(input_l + fb_l);
		delay_line_r.Write(input_r + fb_r);
		const float spread = delay_spread_smoothed;
		float delay_l = delay_out_l;
		float delay_r = delay_out_r;
		if (spread > 0.0f)
		{
			const float width = 1.0f + (spread * spread * 2.5f);
			const float mid = 0.5f * (delay_l + delay_r);
			const float side = 0.5f * (delay_l - delay_r);
			delay_l = mid + (side * width);
			delay_r = mid - (side * width);
		}
		const float mix = delay_mix;
		const float delay_mix_l = (l * (1.0f - mix)) + (delay_l * mix);
		const float delay_mix_r = (r * (1.0f - mix)) + (delay_r * mix);
		l = delay_mix_l;
		r = delay_mix_r;
	};

	auto apply_reverb = [&](float &l, float &r)
	{
		float rev_in_l = 0.0f;
		float rev_in_r = 0.0f;
		const bool predelay_active = (cached_reverb_predelay_samples >= 1.0f);
		if (predelay_active)
		{
			rev_in_l = reverb_predelay_l.Read();
			rev_in_r = reverb_predelay_r.Read();
			reverb_predelay_l.Write(l);
			reverb_predelay_r.Write(r);
		}
		else
		{
			reverb_predelay_l.Write(l);
			reverb_predelay_r.Write(r);
			rev_in_l = l;
			rev_in_r = r;
		}
		const float rev_in_level = fabsf(l) + fabsf(r);
		if (rev_in_level > 1e-4f)
		{
			reverb_tail_gain = 1.0f;
		}
		else
		{
			reverb_tail_gain *= cached_reverb_release;
		}
		float rev_l = 0.0f;
		float rev_r = 0.0f;
		reverb.Process(rev_in_l, rev_in_r, &rev_l, &rev_r);
		rev_l *= cached_reverb_gain * reverb_tail_gain;
		rev_r *= cached_reverb_gain * reverb_tail_gain;
		const float wet = cached_reverb_wet;
		float wet_mix = wet;
		float dry_mix = 1.0f - wet;
		if (wet < 0.5f)
		{
			wet_mix = 2.0f * wet * wet;
			dry_mix = 1.0f - wet_mix;
		}
		else
		{
			dry_mix = 2.0f * (1.0f - wet) * (1.0f - wet);
			wet_mix = 1.0f - dry_mix;
		}
		wet_mix *= 1.12f;
		if (wet_mix > 1.0f)
		{
			wet_mix = 1.0f;
		}
		if (wet >= 0.999f)
		{
			wet_mix = 1.0f;
			dry_mix = 0.0f;
		}
		l = (l * dry_mix) + (rev_l * wet_mix);
		r = (r * dry_mix) + (rev_r * wet_mix);
	};

	float fx_gain = fxrt.fade_gain;
	int32_t fade_samples_left = fxrt.fade_samples_left;
	const float fade_step = (fade_samples_left > 0)
		? (fxrt.fade_target - fx_gain) / static_cast<float>(fade_samples_left)
		: 0.0f;

	const bool monitor_active = (flags_bits & kFlagMonitorEnabled) != 0;
	const bool any_perform_active = AnyPerformVoiceActive();
	g_perform_voices_active = any_perform_active;
	const bool idle_audio = !playback_active
		&& !preview_active
		&& !monitor_active
		&& !record_active
		&& !any_perform_active
		&& !sat_active
		&& !chorus_active
		&& (!delay_active || !rt.loaded)
		&& (!reverb_active || !rt.loaded)
		&& (fxrt.fade_samples_left == 0)
		&& !fxrt.pause_pending;
	if (idle_audio)
	{
		for (size_t i = 0; i < size; ++i)
		{
			out[0][i] = 0.0f;
			out[1][i] = 0.0f;
		}
		goto audio_done;
	}
	for (size_t i = 0; i < size; i++)
	{
		float sig_l = 0.0f;
		float sig_r = 0.0f;
		auto add_voice = [&](float v_l, float v_r)
		{
			sig_l += v_l;
			sig_r += v_r;
		};
		float monitor_l = 0.0f;
		float monitor_r = 0.0f;
		if (monitor_active)
		{
			const float monitor_sel = (record_input_audio == RecordInput::Mic)
				? in[1][i]
				: in[0][i];
			monitor_l = monitor_sel;
			monitor_r = monitor_sel;
		}
		if (record_active)
		{
			if (record_pos < kRecordMaxFrames)
			{
				const float in_sel = (record_input_audio == RecordInput::Mic)
					? in[1][i]
					: in[0][i];
				int32_t s = static_cast<int32_t>(in_sel * 32767.0f);
				if (s > 32767)
				{
					s = 32767;
				}
				else if (s < -32768)
				{
					s = -32768;
				}
				const int16_t samp = static_cast<int16_t>(s);
				sample_buffer_l[record_pos] = samp;
				sample_buffer_r[record_pos] = samp;
				int16_t abs_s = samp < 0 ? static_cast<int16_t>(-samp) : samp;
				if (abs_s > uiw.live_wave.peak)
				{
					uiw.live_wave.peak = abs_s;
					ui_wave_dirty = true;
				}
				static float live_scale_audio = 28.0f;
				static RecordInput live_scale_input = RecordInput::LineIn;
				if (record_pos == 0 || live_scale_input != record_input_audio)
				{
					const bool from_mic = (record_input_audio == RecordInput::Mic);
					const float max_scale = from_mic ? kRecordWaveformScaleMaxMic : kRecordWaveformScaleMaxLine;
					const float scale = max_scale;
					live_scale_audio = 28.0f * scale * 1.1f;
					live_scale_input = record_input_audio;
				}
				const float s_scaled = static_cast<float>(samp) * kSampleScale * live_scale_audio;
				int16_t s_pix = static_cast<int16_t>(s_scaled);
				const int32_t col = static_cast<int32_t>(
					(record_pos / kLiveWaveStride) % kWaveCols);
				if (col >= 0 && col < 128)
				{
					if (col != uiw.live_wave.last_col)
					{
						uiw.live_wave.minv[col] = s_pix;
						uiw.live_wave.maxv[col] = s_pix;
						uiw.live_wave.last_col = static_cast<int16_t>(col);
						uiw.live_wave.dirty = true;
						ui_wave_dirty = true;
					}
					else
					{
						if (s_pix < uiw.live_wave.minv[col]) uiw.live_wave.minv[col] = s_pix;
						if (s_pix > uiw.live_wave.maxv[col]) uiw.live_wave.maxv[col] = s_pix;
						uiw.live_wave.dirty = true;
						ui_wave_dirty = true;
					}
				}
				++record_pos;
			}
			if (record_pos >= kRecordMaxFrames)
			{
				g_recorded_length_audio = record_pos;
				g_audio_recording_active = false;
				record_active = false;
				PushAudioEvent(kAudioEventPlaybackStopped | kAudioEventRecFinished);
			}
		}
		if (rt.loaded && playback_active && !record_active && window_valid)
		{
			float amp_env = 1.0f;
			if (amp_env_active)
			{
				float attack_env = 1.0f;
				if (amp_attack_samples > 1.0f)
				{
					attack_env = static_cast<float>(playback_env_samples) * inv_attack;
					if (attack_env > 1.0f) attack_env = 1.0f;
				}
				float release_env = 1.0f;
				if (amp_release_samples > 1.0f && playback_rate > 0.0f)
				{
					float remaining = 0.0f;
					if (reverse_playback)
					{
						remaining = (playback_phase - static_cast<float>(window_start)) / playback_rate;
					}
					else
					{
						remaining = (static_cast<float>(window_end) - playback_phase) / playback_rate;
					}
					if (remaining < 0.0f) remaining = 0.0f;
					release_env = remaining * inv_release;
					if (release_env > 1.0f) release_env = 1.0f;
				}
				amp_env = (attack_env < release_env) ? attack_env : release_env;
				if (amp_env < 0.0f) amp_env = 0.0f;
			}
			if (playback_release_active)
			{
				if (playback_release_start < 0.0f)
				{
					playback_release_start = amp_env;
				}
				float noteoff_env = playback_release_start;
				if (amp_release_samples > 1.0f)
				{
					noteoff_env *= (1.0f - (playback_release_pos / amp_release_samples));
				}
				if (noteoff_env < 0.0f)
				{
					noteoff_env = 0.0f;
				}
				if (noteoff_env < amp_env)
				{
					amp_env = noteoff_env;
				}
			}
			if (length == 1)
			{
				sig_l = static_cast<float>(rt_l[0]) * kSampleScale * playback_amp;
				sig_r = static_cast<float>(rt_r[0]) * kSampleScale * playback_amp;
				if (amp_env_active)
				{
					sig_l *= amp_env;
					sig_r *= amp_env;
				}
				playback_active = false;
			}
			else
			{
				const size_t idx = static_cast<size_t>(playback_phase);
				if (!reverse_playback && idx + 1 < window_end)
				{
					const float frac = playback_phase - static_cast<float>(idx);
					const float l0 = static_cast<float>(rt_l[idx]);
					const float l1 = static_cast<float>(rt_l[idx + 1]);
					const float r0 = static_cast<float>(rt_r[idx]);
					const float r1 = static_cast<float>(rt_r[idx + 1]);
					sig_l = (l0 + (l1 - l0) * frac) * kSampleScale * playback_amp;
					sig_r = (r0 + (r1 - r0) * frac) * kSampleScale * playback_amp;
					if (amp_env_active)
					{
						sig_l *= amp_env;
						sig_r *= amp_env;
					}
					playback_phase += playback_rate;
					if (playback_phase >= static_cast<float>(window_end - 1))
					{
						playback_active = false;
						PushAudioEvent(kAudioEventPlaybackStopped);
					}
				}
				else if (reverse_playback && idx > window_start)
				{
					const float frac = playback_phase - static_cast<float>(idx);
					const float l0 = static_cast<float>(rt_l[idx]);
					const float l1 = static_cast<float>(rt_l[idx - 1]);
					const float r0 = static_cast<float>(rt_r[idx]);
					const float r1 = static_cast<float>(rt_r[idx - 1]);
					sig_l = (l0 + (l1 - l0) * frac) * kSampleScale * playback_amp;
					sig_r = (r0 + (r1 - r0) * frac) * kSampleScale * playback_amp;
					if (amp_env_active)
					{
						sig_l *= amp_env;
						sig_r *= amp_env;
					}
					playback_phase -= playback_rate;
					if (playback_phase <= static_cast<float>(window_start))
					{
						playback_active = false;
						PushAudioEvent(kAudioEventPlaybackStopped);
					}
				}
				else if (reverse_playback && idx == window_start)
				{
					const float l0 = static_cast<float>(rt_l[idx]);
					const float r0 = static_cast<float>(rt_r[idx]);
					sig_l = l0 * kSampleScale * playback_amp;
					sig_r = r0 * kSampleScale * playback_amp;
					if (amp_env_active)
					{
						sig_l *= amp_env;
						sig_r *= amp_env;
					}
					playback_active = false;
					PushAudioEvent(kAudioEventPlaybackStopped);
				}
				else
				{
					playback_active = false;
					PushAudioEvent(kAudioEventPlaybackStopped);
				}
			}
			if (playback_active)
			{
				++playback_env_samples;
				if (playback_release_active)
				{
					playback_release_pos += 1.0f;
					const float release_samples = (amp_release_samples > 1.0f)
						? amp_release_samples
						: 1.0f;
					if (playback_release_pos >= release_samples)
					{
						playback_active = false;
						playback_env_samples = 0;
						playback_release_active = false;
						playback_release_pos = 0.0f;
						playback_release_start = 0.0f;
						PushAudioEvent(kAudioEventPlaybackStopped);
					}
				}
			}
		}
		if (use_poly)
		{
			for (int v = 0; v < kPerformVoiceCount; ++v)
			{
				auto &voice = perform_voices[v];
				if (!voice.active || voice.length == 0)
				{
					++g_voice_skip_count;
					continue;
				}
				float env = 1.0f;
				if (amp_env_active)
				{
					if (amp_attack_samples > 1.0f)
					{
						env = static_cast<float>(voice.env_samples) * inv_attack;
					}
				}
				if (env > 1.0f)
				{
					env = 1.0f;
				}
				if (voice.releasing)
				{
					float noteoff_env = voice.release_start;
					if (amp_release_samples > 1.0f)
					{
						noteoff_env *= (1.0f - (voice.release_pos * inv_release));
					}
					if (noteoff_env < 0.0f)
					{
						noteoff_env = 0.0f;
					}
					if (noteoff_env < env)
					{
						env = noteoff_env;
					}
				}
				if (env < 0.0f)
				{
					env = 0.0f;
				}
				voice.env = env;
				if (env < kSilentAmp && voice.releasing)
				{
					voice.silent_samples++;
					if (voice.silent_samples >= kSilentSamplesToKill)
					{
						DeactivateVoice(voice);
						++g_voice_kill_count;
						continue;
					}
				}
				else
				{
					voice.silent_samples = 0;
				}
				if (voice.length == 1)
				{
					const size_t idx = voice.offset;
					const float amp = voice.amp * env;
					float samp_l = 0.0f;
					float samp_r = 0.0f;
					samp_l = static_cast<float>(rt_l[idx]) * kSampleScale * amp;
					const float r = sample_stereo
						? static_cast<float>(rt_r[idx])
						: static_cast<float>(rt_l[idx]);
					samp_r = r * kSampleScale * amp;
					if (perform_mode)
					{
						samp_l = perform_lpf_l2[v].Process(perform_lpf_l1[v].Process(samp_l));
						samp_r = perform_lpf_r2[v].Process(perform_lpf_r1[v].Process(samp_r));
					}
					add_voice(samp_l, samp_r);
					DeactivateVoice(voice);
					continue;
				}
				const size_t idx_rel = static_cast<size_t>(voice.phase);
				if (idx_rel + 1 >= voice.length)
				{
					DeactivateVoice(voice);
					continue;
				}
				const float frac = voice.phase - static_cast<float>(idx_rel);
				size_t idx = voice.offset + idx_rel;
				float l0 = 0.0f;
				float l1 = 0.0f;
				float r0 = 0.0f;
				float r1 = 0.0f;
				if (reverse_playback)
				{
					idx = voice.offset + (voice.length - 1 - idx_rel);
					l0 = static_cast<float>(rt_l[idx]);
					if (idx > voice.offset)
					{
						l1 = static_cast<float>(rt_l[idx - 1]);
					}
					else
					{
						l1 = l0;
					}
					if (sample_stereo)
					{
						r0 = static_cast<float>(rt_r[idx]);
						r1 = (idx > voice.offset)
							? static_cast<float>(rt_r[idx - 1])
							: r0;
					}
					else
					{
						r0 = l0;
						r1 = l1;
					}
				}
				else
				{
					l0 = static_cast<float>(rt_l[idx]);
					l1 = static_cast<float>(rt_l[idx + 1]);
					if (sample_stereo)
					{
						r0 = static_cast<float>(rt_r[idx]);
						r1 = static_cast<float>(rt_r[idx + 1]);
					}
					else
					{
						r0 = l0;
						r1 = l1;
					}
				}
				const float amp = voice.amp * env;
				float samp_l = (l0 + (l1 - l0) * frac) * kSampleScale * amp;
				float samp_r = (r0 + (r1 - r0) * frac) * kSampleScale * amp;
				if (perform_mode)
				{
					samp_l = perform_lpf_l2[v].Process(perform_lpf_l1[v].Process(samp_l));
					samp_r = perform_lpf_r2[v].Process(perform_lpf_r1[v].Process(samp_r));
				}
				add_voice(samp_l, samp_r);
				voice.phase += voice.rate;
				if (!voice.releasing)
				{
					++voice.env_samples;
				}
				else
				{
					voice.release_pos += 1.0f;
					if (amp_release_samples > 1.0f && voice.release_pos >= amp_release_samples)
					{
						DeactivateVoice(voice);
					}
				}
				if (voice.phase >= static_cast<float>(voice.length - 1))
				{
					DeactivateVoice(voice);
				}
			}
		}
		if (preview_active)
		{
#if STORAGE_SERVICE_PREVIEW_STREAM
			if (preview_preload_active)
			{
				const size_t frames = preview_preload_frames;
				if (frames < 2)
				{
					preview_underrun_count++;
				}
				else
				{
					const size_t idx0 = preview_read_index % frames;
					const size_t idx1 = (idx0 + 1) % frames;
					const float frac = preview_read_frac;
					const float s0 = static_cast<float>(preview_preload_buf[idx0]);
					const float s1 = static_cast<float>(preview_preload_buf[idx1]);
					float samp = (s0 + (s1 - s0) * frac) * kSampleScale * pctl.gain;
					if (preview_fade_samples_left > 0)
					{
						const float fade = 1.0f
							- (static_cast<float>(preview_fade_samples_left)
								/ static_cast<float>(preview_fade_samples_total));
						samp *= fade;
						preview_fade_samples_left--;
					}
					sig_l += samp;
					sig_r += samp;

					float next_frac = preview_read_frac + pctl.rate;
					uint32_t adv = (next_frac >= 1.0f) ? static_cast<uint32_t>(next_frac) : 0u;
					if (adv >= frames) adv %= frames;
					next_frac -= static_cast<float>(adv);
					if (next_frac >= 1.0f)
					{
						next_frac = 0.0f;
					}
					preview_read_index = (preview_read_index + adv) % frames;
					preview_read_frac = next_frac;
				}
			}
			else
			{
				const uint8_t active = preview_pp_active;
				__DMB();
				if (preview_pp_ready[active] == 0)
				{
					const uint8_t other = static_cast<uint8_t>(active ^ 1u);
					if (preview_pp_ready[other] != 0)
					{
						preview_pp_active = other;
					}
					else
					{
						preview_underrun_count++;
					}
				}
				else
				{
					const int16_t* buf = preview_pp_buf[active];
					const size_t pos = preview_pp_pos;
					float samp = static_cast<float>(buf[pos]) * kSampleScale * pctl.gain;
					if (preview_fade_samples_left > 0)
					{
						const float fade = 1.0f
							- (static_cast<float>(preview_fade_samples_left)
								/ static_cast<float>(preview_fade_samples_total));
						samp *= fade;
						preview_fade_samples_left--;
					}
					sig_l += samp;
					sig_r += samp;
					preview_pp_pos = static_cast<uint32_t>(pos + 1);
					if (preview_pp_pos >= kPreviewPpFrames)
					{
						preview_pp_pos = 0;
						preview_pp_ready[active] = 0;
						preview_pp_active = static_cast<uint8_t>(active ^ 1u);
					}
				}
			}
#else
			size_t read_idx = preview_read_index;
			size_t write_idx = 0;
			{
				daisy::ScopedIrqBlocker irq;
				write_idx = preview_write_index;
			}
			__DMB();
			size_t available = PreviewAvailableFrames(read_idx, write_idx);
			if (available >= 2)
			{
				const size_t idx0 = read_idx;
				const size_t idx1 = (idx0 + 1) % kPreviewBufferFrames;
				const float frac = preview_read_frac;
				const int16_t* preview_buf = pctl.l;
				const float s0 = static_cast<float>(preview_buf[idx0]);
				const float s1 = static_cast<float>(preview_buf[idx1]);
				float samp = (s0 + (s1 - s0) * frac) * kSampleScale * pctl.gain;
				if (preview_fade_samples_left > 0)
				{
					const float fade = 1.0f
						- (static_cast<float>(preview_fade_samples_left)
							/ static_cast<float>(preview_fade_samples_total));
					samp *= fade;
					preview_fade_samples_left--;
				}
				sig_l += samp;
				sig_r += samp;

				float next_frac = preview_read_frac + pctl.rate;
				uint32_t adv = (next_frac >= 1.0f) ? static_cast<uint32_t>(next_frac) : 0u;
				if (available > 1)
				{
					const uint32_t max_adv = static_cast<uint32_t>(available - 1);
					if (adv > max_adv) adv = max_adv;
				}
				else
				{
					adv = 0;
				}
				next_frac -= static_cast<float>(adv);
				if (next_frac >= 1.0f)
				{
					next_frac = 0.0f;
				}
				read_idx = (read_idx + adv) % kPreviewBufferFrames;
				preview_read_frac = next_frac;
				preview_read_index = read_idx;
			}
#endif
		}
		if (monitor_active)
		{
			sig_l += monitor_l;
			sig_r += monitor_r;
		}
		if (fade_samples_left > 0)
		{
			fx_gain += fade_step;
			--fade_samples_left;
			if (fade_samples_left == 0)
			{
				fx_gain = fxrt.fade_target;
			}
		}
		if (fxrt.paused)
		{
			out[0][i] = 0.0f;
			out[1][i] = 0.0f;
			continue;
		}
		if (main_mode)
		{
			out[0][i] = 0.0f;
			out[1][i] = 0.0f;
			continue;
		}
		if (!fx_allowed)
		{
			out[0][i] = sig_l * phones_gain;
			out[1][i] = sig_r * phones_gain;
			continue;
		}
		float fx_l = sig_l;
		float fx_r = sig_r;
		for (int stage = 0; stage < kPerformFaderCount; ++stage)
		{
			switch (fx_order[stage])
			{
				case kFxSatIndex:
					if (sat_active) apply_saturation(fx_l, fx_r);
					break;
				case kFxChorusIndex:
					if (chorus_active) apply_chorus(fx_l, fx_r);
					break;
				case kFxDelayIndex:
					if (delay_active) apply_delay(fx_l, fx_r);
					break;
				case kFxReverbIndex:
					if (reverb_active) apply_reverb(fx_l, fx_r);
					break;
				default: break;
			}
		}
		out[0][i] = fx_l * fx_gain * phones_gain;
		out[1][i] = fx_r * fx_gain * phones_gain;
	}
audio_done:
	if (ui_wave_dirty || ui_state_changed)
	{
		daisy::ScopedIrqBlocker irq;
		g_audio_ui_state_idx = ui_next_idx;
	}
	g_fx_chain_audio.fade_gain = fx_gain;
	g_fx_chain_audio.fade_samples_left = fade_samples_left;
	if (g_fx_chain_audio.pause_pending
		&& g_fx_chain_audio.fade_samples_left == 0
		&& g_fx_chain_audio.fade_target <= 0.0f)
	{
		g_fx_chain_audio.paused = true;
		g_fx_chain_audio.pause_pending = false;
		g_fx_chain_audio.fade_gain = 0.0f;
	}
	#if PERF_DIAGNOSTICS
	PERF_CYCLES_END(cyc_end);
	const uint32_t cyc_used = cyc_end - cyc_start;
	callback_cycles_last = cyc_used;
	if (cyc_used > callback_cycles_max)
	{
		callback_cycles_max = cyc_used;
	}
	static uint32_t sys_clk_hz = 0;
	if (sys_clk_hz == 0)
	{
		sys_clk_hz = System::GetSysClkFreq();
	}
	const float cycles_per_block = (static_cast<float>(sys_clk_hz) / out_sr)
		* static_cast<float>(size);
	float load_pct = 0.0f;
	if (cycles_per_block > 0.0f)
	{
		load_pct = 100.0f * (static_cast<float>(cyc_used) / cycles_per_block);
	}
	cpu_load_ema = (cpu_load_ema * 0.95f) + (load_pct * 0.05f);
	cpu_load_pct = cpu_load_ema;
	if (cyc_used > static_cast<uint32_t>(cycles_per_block))
	{
		callback_overruns++;
	}
	if (load_pct > cpu_load_peak_pct)
	{
		cpu_load_peak_pct = load_pct;
	}
	else
	{
		cpu_load_peak_pct *= 0.995f;
		if (cpu_load_peak_pct < 0.0f)
		{
			cpu_load_peak_pct = 0.0f;
		}
	}
	#endif
	request_playhead_redraw = true;
}

#if AUDIO_HOT_GUARD
#undef powf
#undef expf
#undef logf
#undef sinf
#undef cosf
#undef sqrtf
#undef tanf
#undef atanf
#undef asinf
#undef acosf
#undef AUDIO_HOT_GUARD
#endif
void AudioEngine::Init(daisy::DaisyPod& /*hw*/)
{
}
void AudioEngine::Process(daisy::AudioHandle::InputBuffer in, daisy::AudioHandle::OutputBuffer out, size_t size)
{
	AudioCallbackImpl(in, out, size);
}

