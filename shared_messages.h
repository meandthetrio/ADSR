#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "BuildConfig.h"

constexpr int kWaveCols = 128;

enum class RecordInput : int32_t
{
	LineIn,
	Mic,
};

enum class SampleContext : int32_t
{
	Perform,
};

enum class UiMode : int32_t
{
	Main,
	Load,
	LoadModeSelect,
	LoadStub,
	Perform,
	Edt,
	FxDetail,
	Record,
	PresetSaveStub,
	Shift,
};

enum class LoadStubMode : int32_t
{
	Presets = 0,
	Bake = 1,
};

enum class RecordState : int32_t
{
	SourceSelect,
	Armed,
	Countdown,
	Recording,
	Review,
	TargetSelect,
	BackConfirm,
};

enum class LoadContext : int32_t
{
	Main,
	Edt,
};

enum class LoaderState : uint8_t
{
	Idle,
	Requested,
	Loading,
	Ready,
	Failed,
};

enum class FxContext : int32_t
{
	Perform,
};

struct PerformState
{
	int32_t perform_index = 0;
	int32_t fx_fader_index = 0;
	int32_t amp_fader_index = 0;
	int32_t flt_fader_index = 0;
	int32_t fx_detail_index = 0;
	int32_t fx_detail_param_index = 0;
	bool fx_window_active = false;
	bool amp_window_active = false;
	bool flt_window_active = false;
	int32_t fx_chain_order[4] = {};
	float amp_attack = 0.0f;
	float amp_decay = 0.0f;
	float amp_sustain = 0.0f;
	float amp_release = 0.0f;
	float flt_cutoff = 1.0f;
	float flt_res = 0.02f;
	float fx_s_wet = 0.0f;
	float sat_tape_bump = 0.0f;
	float sat_bit_reso = 0.0f;
	float sat_bit_smpl = 0.0f;
	int32_t sat_mode = 0;
	float fx_c_wet = 0.0f;
	float chorus_rate = 0.0f;
	float chorus_wow = 0.0f;
	float tape_rate = 0.0f;
	int32_t chorus_mode = 0;
	float delay_wet = 0.0f;
	float delay_time = 0.0f;
	float delay_feedback = 0.0f;
	float delay_spread = 0.0f;
	float delay_freeze = 0.0f;
	float reverb_wet = 0.0f;
	float reverb_pre = 0.0f;
	float reverb_damp = 0.0f;
	float reverb_decay = 0.0f;
	bool sat_params_initialized = false;
	bool reverb_params_initialized = false;
	bool delay_params_initialized = false;
	bool mod_params_initialized = false;
};

struct WaveformJob
{
	bool active;
	size_t frames;
	size_t columns;
	size_t step;
	size_t col;
	size_t idx;
	size_t end;
	float minv;
	float maxv;
	float scale;
	SampleContext ctx;
};

struct FileListJob
{
	bool active;
	bool done;
	bool wav_only;
	int32_t count;
	uint16_t cookie;
};

enum class JobType : uint8_t
{
	None = 0,
	WaveformBuild,
	FileListScan,
};

struct Job
{
	JobType type;
	bool active;
	bool foreground;
	uint8_t priority;
	uint32_t progress;
	uint32_t last_reported;
};

enum MidiCmdKind : uint8_t { kMidiCmdNoteOn = 1, kMidiCmdNoteOff = 2 };

struct MidiCmd
{
	uint8_t kind = 0;
	uint8_t note = 0;
	uint8_t vel = 0;
	uint32_t t_ms = 0;
};

enum PlaybackCmdKind : uint8_t
{
	kPlaybackCmdStart   = 1,
	kPlaybackCmdStop    = 2,
	kPlaybackCmdStopAll = 3,
};

enum PlaybackCmdFlags : uint8_t
{
	kPlaybackCmdApplyPitch   = 1u << 0,
	kPlaybackCmdApplyRelease = 1u << 1,
};

struct PlaybackCmd
{
	uint8_t kind = 0;
	uint8_t note = 0;
	uint8_t flags = 0;
	uint8_t reserved = 0;
	uint32_t t_ms = 0;
};

enum AudioCmdBits : uint32_t
{
	kCmdNone            = 0,
	kCmdRecStart        = 1U << 0,
	kCmdRecStop         = 1U << 1,
	kCmdAllNotesOff     = 1U << 2,
	kCmdCommitRuntime   = 1U << 3,
	kCmdCommitFxChain   = 1U << 4,
	kCmdPreviewStart    = 1U << 5,
	kCmdPreviewStop     = 1U << 6,
	kCmdCommitPreview   = 1U << 7,
	kCmdPlaybackReverse = 1U << 8,
	kCmdCommitAudioParams = 1U << 9,
};

enum AudioFlagBits : uint32_t
{
	kFlagInPerformMode  = 1u << 0,
	kFlagMonitorEnabled = 1u << 1,
	kFlagInMainMode     = 1u << 2,
	kFlagFxAllowed      = 1u << 3,
};

enum AudioEventBits : uint32_t
{
	kAudioEventRecFinished     = 1u << 0,
	kAudioEventPlaybackStopped = 1u << 1,
};

struct AudioParams
{
	float amp_attack = 0.0f;
	float amp_decay = 0.0f;
	float amp_sustain = 0.0f;
	float amp_release = 0.0f;
	float flt_cutoff = 0.0f;
	float flt_res = 0.0f;
	float fx_s_wet = 0.0f;
	float sat_drive = 0.0f;
	float sat_tape_bump = 0.0f;
	float sat_bit_reso = 0.0f;
	float sat_bit_smpl = 0.0f;
	float fx_c_wet = 0.0f;
	float mod_depth = 0.0f;
	float chorus_rate = 0.0f;
	float chorus_wow = 0.0f;
	float tape_rate = 0.0f;
	float delay_wet = 0.0f;
	float delay_time = 0.0f;
	float delay_feedback = 0.0f;
	float delay_spread = 0.0f;
	float delay_freeze = 0.0f;
	float reverb_wet = 0.0f;
	float reverb_pre = 0.0f;
	float reverb_damp = 0.0f;
	float reverb_decay = 0.0f;
	float playback_reverse = 0.0f;
};

struct SampleRuntime
{
	const int16_t* l = nullptr;
	const int16_t* r = nullptr;
	size_t length = 0;
	size_t play_start = 0;
	size_t play_end = 0;
	uint32_t rate = 48000;
	uint16_t channels = 1;
	bool loaded = false;
};

struct FxChainRuntime
{
	uint8_t order[4] = {};
	uint8_t count = 4;
	bool paused = false;
	bool pause_pending = false;
	float fade_gain = 1.0f;
	float fade_target = 1.0f;
	int32_t fade_samples_left = 0;
};

struct PreviewControl
{
	const int16_t* l = nullptr;
	const int16_t* r = nullptr;
	size_t length = 0;
	float rate = 1.0f;
	float gain = 1.0f;
	bool mono = true;
};

struct FxParamsAudio
{
	int32_t sat_mode = 0;
	float   sat_mix = 0.0f;
	float   sat_drive_amt = 0.0f;
	float   sat_bump = 0.0f;
	float   bit_step = 1.0f;

	int32_t chorus_mode = 0;
	float   chorus_mix = 0.0f;
	float   chorus_depth_mapped = 0.0f;
	float   chorus_rate_hz = 0.0f;
	float   chorus_wow = 0.0f;
	float   tape_rate = 0.0f;
	float   tape_drop_amt_mapped = 0.0f;

	float delay_wet = 0.0f;
	float delay_time_samples = 0.0f;
	float delay_feedback = 0.0f;
	float delay_spread = 0.0f;
	float delay_freeze = 0.0f;

	float reverb_wet = 0.0f;
	float reverb_predelay_samples = 0.0f;
	float reverb_lp_hz = 0.0f;
	float reverb_feedback = 0.0f;
	float reverb_release = 1.0f;
	float reverb_gain = 1.0f;
};

struct BiquadCoeffs
{
	float a0 = 0.0f;
	float a1 = 0.0f;
	float a2 = 0.0f;
	float b1 = 0.0f;
	float b2 = 0.0f;
};

struct AudioParamsAudio
{
	float amp_attack_samples = 0.0f;
	float amp_release_samples = 0.0f;
	float flt_cutoff_hz = 0.0f;
	float flt_q = 0.0f;
	BiquadCoeffs flt_coeffs = {};
};

struct WaveformUi
{
	int16_t minv[kWaveCols] = {};
	int16_t maxv[kWaveCols] = {};
	int16_t peak = 1;
	int16_t last_col = -1;
	bool dirty = false;
};

struct AudioUiState
{
	WaveformUi live_wave;
	bool preview_active = false;
	uint32_t preview_read_index = 0;
	bool playback_active = false;
	float playback_phase = 0.0f;
	bool perform_voices_active = false;
#if ENABLE_PERF_COUNTERS
	float cpu_load_pct = 0.0f;
	float cpu_load_peak_pct = 0.0f;
	uint32_t callback_cycles_last = 0;
	uint32_t callback_cycles_max = 0;
	uint32_t callback_overruns = 0;
#endif
};
