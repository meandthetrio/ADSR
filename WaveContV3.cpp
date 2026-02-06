#include "daisy_pod.h"
#include "dev/oled_ssd130x.h"
#include "per/tim.h"
#include "util/scopedirqblocker.h"
#include "SamplerConfig.h"
#include "shared_messages.h"
#include "audio_engine.h"
#include "ui.h"
#include "app_controller.h"
#include <cmath>
#include <initializer_list>
//#include <math.h>
#include <cstring>
#include <cstdio>

using namespace daisy;

#ifndef STORAGE_SERVICE_PREVIEW_STREAM
#define STORAGE_SERVICE_PREVIEW_STREAM PREVIEW_STREAM_FROM_SD
#endif

#ifndef STORAGE_SERVICE_SAVE
#define STORAGE_SERVICE_SAVE 1
#endif

// Enforce IO policy: release builds preload into RAM.
static_assert(kPolicy == SampleIOPolicy::PreloadToRam,
	"StreamFromSd policy requires removing preload buffers; not supported in this build.");

using PodDisplay = OledDisplay<SSD130xI2c128x64Driver>;

#ifndef RAM_D3_MEM_SECTION
#define RAM_D3_MEM_SECTION __attribute__((section(".ramd3_bss")))
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

constexpr int32_t kMenuCount = 3;
constexpr int32_t kShiftMenuCount = 3;
constexpr int32_t kRecordTargetCount = 2;
constexpr int32_t kRecordTargetSave = 0;
/// constexpr int32_t kRecordTargetDiscard = 1;
constexpr int32_t kPerformBoxCount = 4;
constexpr int32_t kPerformEdtIndex = 0;
constexpr int32_t kPerformFaderCount = 4;
constexpr int32_t kFxSatIndex = 0;
constexpr int32_t kFxChorusIndex = 1;
constexpr int32_t kFxDelayIndex = 2;
constexpr int32_t kFxReverbIndex = 3;
constexpr int32_t kReverbFaderCount = 5;
constexpr int32_t kDelayFaderCount = 5;
constexpr int32_t kPerformFltFaderCount = 2;
constexpr int32_t kPerformAmpIndex = 1;
constexpr int32_t kPerformFltIndex = 2;
constexpr int32_t kPerformFxIndex = 3;
constexpr uint32_t kFxChainIdleMs = 300;
constexpr float kFxChainFadeMs = 20.0f;
constexpr uint32_t kFxMapIntervalMs = 10;
constexpr uint32_t kSdInitMinMs = 800;
constexpr uint32_t kSdInitRetryMs = 300;
constexpr uint32_t kSdInitResultMs = 1500;
constexpr int32_t kSdInitAttempts = 3;
constexpr uint32_t kSaveResultMs = 1500;
constexpr uint32_t kSaveStepBudgetMs = 20;
constexpr int32_t kMaxWavFiles = 32;
constexpr size_t kMaxWavNameLen = 32;
constexpr int32_t kLoadFontScale = 1;
constexpr size_t kSampleChunkFrames = 256;
constexpr size_t kSaveChunkFrames = 8192;
constexpr int32_t kBaseMidiNote = 60;
constexpr float kSampleScale = 1.0f / 32768.0f;
constexpr int32_t kLoadProgressStep = 5;
constexpr float kLedBlinkPeriodMs = 25.0f;
constexpr float kLedBlinkDuty = 0.5f;
constexpr uint32_t kPerformPlayheadIntervalMs = 33;
constexpr int32_t kPerformEncoderScale = 4;
constexpr float kPi = 3.14159265f;
constexpr float kTwoPi = 6.2831853f;
constexpr size_t kSineTableSize = 1024;
constexpr int kDisplayW = 128;
constexpr int kDisplayH = 64;
constexpr uint32_t kPreviewReadBudgetMs = 2;
// kPreviewBufferFrames defined in SamplerConfig.h
constexpr size_t kPreviewReadFrames = 256;
constexpr size_t kPreviewPpFrames = 2048;
// kPreviewPreloadFrames defined in SamplerConfig.h
constexpr uint32_t kStorageBudgetUs = 2000;
constexpr uint32_t kStoragePreviewBudgetUs = 12000;
constexpr uint32_t kPreviewFadeInMs = 10;
uint32_t kUiTickMs = 1;
uint32_t kUiTickPlaybackMs = 5;
constexpr uint32_t kLoadScanGraceMs = 300;
constexpr float kRecordWaveformScaleMinMic = 1.15f;
constexpr float kRecordWaveformScaleMaxMic = 1.75f;
constexpr float kRecordWaveformScaleMinLine = 0.7f;
constexpr float kRecordWaveformScaleMaxLine = 2.0f;

constexpr int kPerformVoiceCount = kMaxVoices;
constexpr float kReverbFeedback = 0.85f;
constexpr float kReverbLpFreq = 12000.0f;
constexpr float kReverbFeedbackMin = 0.2f;
constexpr float kReverbFeedbackMax = 0.98f;
constexpr float kReverbDampMinHz = 800.0f;
constexpr float kReverbDampMaxHz = 20000.0f;
constexpr float kReverbPreDelayMaxMs = 1000.0f;
constexpr float kReverbDecayMinMs = 10.0f;
constexpr float kReverbDecayMaxMs = 4000.0f;
constexpr float kReverbParamStep = 0.02f;
constexpr size_t kReverbPreDelayMaxSamples = 48000;
constexpr float kReverbDecayDefault =
	(kReverbFeedback - kReverbFeedbackMin) / (kReverbFeedbackMax - kReverbFeedbackMin);
constexpr float kReverbDampDefault =
	(kReverbDampMaxHz - kReverbLpFreq) / (kReverbDampMaxHz - kReverbDampMinHz);
constexpr float kReverbDefaultWet = 0.0f;
constexpr float kReverbWetStep = 0.02f;
constexpr float kChorusRateHz = 0.25f;
constexpr float kChorusRateMinHz = 0.05f;
constexpr float kChorusRateMaxHz = 0.9f;
constexpr float kChorusRateStep = 0.02f;
constexpr float kChorusDelayMs = 9.0f;
constexpr float kChorusFeedback = 0.18f;
constexpr float kChorusMaxDepth = 3.0f;
constexpr float kChorusWidthMax = 2.2f;
constexpr size_t kDelayMaxSamples = 96000;
constexpr float kDelayTimeMinMs = 50.0f;
constexpr float kDelayTimeMaxMs = 2000.0f;
constexpr float kDelayTimeSlewMs = 180.0f;
constexpr float kDelayParamSlewMs = 120.0f;
constexpr float kDelayFeedbackMax = 0.98f;
constexpr bool kLoadPresetsPlaceholder = true;
constexpr float kDelayDefaultWet = 0.0f;
constexpr float kDelayWetStep = 0.02f;
constexpr float kDelayParamStep = 0.02f;
constexpr int kBitResoStepCount = 3;
constexpr int kBitResoSteps[kBitResoStepCount] = {2, 3, 4};
constexpr const char* kBitResoLabels[kBitResoStepCount] = {"CRUSH", "STATIC", "HISS"};
constexpr int kBitcrushMaxHold = 32;
constexpr float kFxParamEpsilon = 1e-5f;
constexpr float kAmpEnvStep = 0.02f;
constexpr float kFltParamStep = 0.02f;
constexpr float kAmpEnvMinMs = 5.0f;
constexpr float kAmpEnvMaxMs = 5000.0f;
constexpr float kAmpEnvStepMs = 20.0f;
constexpr float kPhonesVolumeStep = 0.01f;


float g_sine_table[kSineTableSize + 1];
static float g_note_ratio[128];

float SineTableLookup(float phase01)
{
	const float idx = phase01 * static_cast<float>(kSineTableSize);
	const int i = static_cast<int>(idx);
	const float frac = idx - static_cast<float>(i);
	const float a = g_sine_table[i];
	const float b = g_sine_table[i + 1];
	return a + (b - a) * frac;
}

static void InitSineTable()
{
	for (size_t i = 0; i <= kSineTableSize; ++i)
	{
		const float phase = static_cast<float>(i) / static_cast<float>(kSineTableSize);
		g_sine_table[i] = sinf(kTwoPi * phase);
	}
}

static void InitNoteRatioTable()
{
	for (int i = 0; i < 128; ++i)
	{
		const float semis = static_cast<float>(i - kBaseMidiNote);
		g_note_ratio[i] = powf(2.0f, semis / 12.0f);
	}
}

static void EnableFtz()
{
#if defined(__FPU_PRESENT) && (__FPU_PRESENT == 1)
	uint32_t fpscr = __get_FPSCR();
	fpscr |= (1u << 24);
	__set_FPSCR(fpscr);
#endif
}

DaisyPod    hw;
PodDisplay  display;
bool g_display_update_pending = false;
uint32_t g_last_draw_ms = 0;
static AudioEngine g_audio_engine;
Ui g_ui;
static AppController g_app;
constexpr uint8_t kMidiCmdQSize = 16; // power of two
MidiCmd g_midi_cmd_q[kMidiCmdQSize];
volatile uint8_t g_midi_cmd_wr = 0;
volatile uint8_t g_midi_cmd_rd = 0;
static volatile bool g_midi_rx_started = false;

constexpr uint8_t kPlaybackCmdQSize = 8; // power of two

PlaybackCmd g_playback_cmd_q[kPlaybackCmdQSize];
volatile uint8_t g_playback_cmd_wr = 0;
volatile uint8_t g_playback_cmd_rd = 0;


static inline void MidiCmdPushIsr(uint8_t kind, uint8_t note, uint8_t vel)
{
	const uint8_t wr = g_midi_cmd_wr;
	const uint8_t next = (uint8_t)((wr + 1) & (kMidiCmdQSize - 1));
	if (next == g_midi_cmd_rd)
	{
		return;
	}

	g_midi_cmd_q[wr].kind = kind;
	g_midi_cmd_q[wr].note = note;
	g_midi_cmd_q[wr].vel = vel;
	g_midi_cmd_q[wr].t_ms = System::GetNow();
	g_midi_cmd_wr = next;
}

static inline bool MidiCmdPopAudio(MidiCmd &out)
{
	daisy::ScopedIrqBlocker irq;
	if (g_midi_cmd_rd == g_midi_cmd_wr)
	{
		return false;
	}
	out = g_midi_cmd_q[g_midi_cmd_rd];
	g_midi_cmd_rd = (uint8_t)((g_midi_cmd_rd + 1) & (kMidiCmdQSize - 1));
	return true;
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

static inline void PlaybackCmdPushUi(uint8_t kind, uint8_t note, uint8_t flags)
{
	daisy::ScopedIrqBlocker irq;
	const uint8_t wr = g_playback_cmd_wr;
	const uint8_t next = (uint8_t)((wr + 1) & (kPlaybackCmdQSize - 1));
	if (next == g_playback_cmd_rd)
	{
		return;
	}
	g_playback_cmd_q[wr].kind = kind;
	g_playback_cmd_q[wr].note = note;
	g_playback_cmd_q[wr].flags = flags;
	g_playback_cmd_wr = next;
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
static daisy::TimerHandle g_ctrl_timer;
static volatile bool g_ctrl_timer_running = false;


Encoder      encoder_r;
Switch       shift_button;

volatile UiMode ui_mode = UiMode::Main;
volatile int32_t menu_index = 0;
volatile int32_t shift_menu_index = 0;
volatile int32_t perform_index = 0;
volatile int32_t amp_fader_index = 0;
volatile int32_t flt_fader_index = 0;
volatile int32_t fx_fader_index = 0;
volatile int32_t fx_chain_order[kPerformFaderCount]
	= {kFxSatIndex, kFxChorusIndex, kFxDelayIndex, kFxReverbIndex};
volatile bool fx_window_active = false;
volatile bool amp_window_active = false;
volatile bool flt_window_active = false;
volatile UiMode shift_prev_mode = UiMode::Main;
volatile RecordInput record_input = RecordInput::LineIn;
volatile int32_t load_selected = 0;
volatile int32_t load_scroll = 0;
volatile bool request_load_scan = false;
volatile bool list_build_pending = false;
volatile bool request_load_sample = false;
volatile int32_t request_load_index = -1;
bool load_in_progress = false;
uint16_t load_cookie_next = 1;
uint16_t load_cookie_active = 0;
bool load_target_is_edt = false;
LoaderState loader_state = LoaderState::Idle;
volatile uint32_t load_success_count = 0;
volatile uint32_t load_fail_budget_count = 0;
volatile uint32_t load_fail_io_count = 0;
volatile size_t sample_mem_used_bytes = 0;
volatile size_t sample_mem_free_bytes = 0;
volatile size_t waveform_cache_bytes = 0;
volatile int32_t wav_file_count = 0;

bool sd_mounted = false;
bool sd_present = false;
bool sd_fault = false;
const char* sd_fault_text = nullptr;
uint32_t sd_retries_remaining = 0;
bool sd_init_in_progress = false;
bool sd_init_done = false;
bool sd_init_success = false;
uint32_t sd_init_start_ms = 0;
uint32_t sd_init_next_ms = 0;
uint32_t sd_init_result_until_ms = 0;
uint32_t sd_init_draw_next_ms = 0;
int32_t sd_init_attempts = 0;
UiMode sd_init_prev_mode = UiMode::Main;
bool save_in_progress = false;
bool save_done = false;
bool save_success = false;
bool save_started = false;
uint32_t save_start_ms = 0;
uint32_t save_result_until_ms = 0;
uint32_t save_draw_next_ms = 0;
UiMode save_prev_mode = UiMode::Main;
char save_filename[kMaxWavNameLen] = {0};
size_t save_frames_written = 0;
volatile bool delete_mode = false;
UiMode delete_prev_mode = UiMode::Main;
volatile bool request_delete_scan = false;
volatile bool request_delete_file = false;
volatile int32_t request_delete_index = -1;
volatile bool delete_confirm = false;
bool request_delete_redraw = false;
char delete_confirm_name[kMaxWavNameLen] = {0};
volatile float phones_volume = 1.0f;
bool delete_in_progress = false;
uint16_t delete_cookie_next = 1;
uint16_t delete_cookie_active = 0;
char wav_files[kMaxWavFiles][kMaxWavNameLen];
char loaded_sample_name[kMaxWavNameLen] = {0};
int32_t load_lines = 1;
int32_t load_line_height = 1;
int32_t load_chars_per_line = 1;
uint32_t load_scan_start_ms = 0;

struct SampleState
{
	char name[kMaxWavNameLen] = {};
	size_t length = 0;
	size_t play_start = 0;
	size_t play_end = 0;
	uint32_t rate = 48000;
	uint16_t channels = 1;
	bool loaded = false;
	float trim_start = 0.0f;
	float trim_end = 1.0f;
	bool from_recording = false;
};

static SampleState perform_sample_state;
SampleContext current_sample_context = SampleContext::Perform;

static constexpr uint32_t kPerformSampleId = 1;

// Memory Layout
// Perform:
// - perform_sample_buffer_l/r (SDRAM, kMaxSampleFrames frames each, 2 bytes/frame):
//   writer: StorageService load path and record path (UI/background),
//   reader: audio callback; valid when sample_loaded == true.
// Record:
// - recording writes into perform_sample_buffer_l/r (mono duplicated to L/R),
//   bounded by kRecordMaxFrames.
// Preview:
// - preview_buffer (SRAM, kPreviewBufferFrames frames, 2 bytes/frame):
//   writer: StorageService preview stream, reader: audio callback.
// - preview_pp_buf (SRAM, 2 x kPreviewPpFrames frames, 2 bytes/frame):
//   writer: StorageService preview stream, reader: audio callback.
// - preview_preload_buf (SDRAM, kPreviewPreloadFrames frames, 2 bytes/frame):
//   writer: StorageService preload, reader: audio callback.
// - preview_read_buf (SRAM, kPreviewReadFrames * 2 samples, 2 bytes/sample):
//   writer: StorageService, reader: StorageService only (scratch).
// Waveform Cache:
// - perform_waveform_cache (SRAM, 128 min + 128 max samples, 2 bytes/sample):
//   writer: waveform jobs (UI thread), reader: UI draw.
// Other:
// - record_*_mask buffers (RAM_D3, 128x64 bytes each) for record UI overlay.
// - perform_voices (DTCM) and filter state (DTCM) for audio processing.

volatile size_t sample_length = 0;
volatile size_t sample_play_start = 0;
volatile size_t sample_play_end = 0;
volatile uint32_t sample_rate = 48000;
volatile uint16_t sample_channels = 1;
volatile bool sample_loaded = false;
volatile bool playback_active = false;
volatile float playback_rate = 1.0f;
volatile float playback_phase = 0.0f;
volatile float playback_reverse = 0.0f;
static bool playback_reverse_audio = false;
volatile bool g_playback_reverse_target = false;
volatile bool g_perform_voices_active = false;
volatile float playback_amp = 0.0f;
volatile uint32_t playback_env_samples = 0;
volatile bool playback_release_active = false;
volatile float playback_release_pos = 0.0f;
volatile float playback_release_start = 0.0f;
volatile int32_t current_note = -1;

DTCM_MEM_SECTION SampleRuntime g_rt_buf[2];
volatile uint8_t g_rt_pub_idx = 0;
volatile uint8_t g_rt_active_idx = 0;
FxChainRuntime g_fx_chain_buf[2];
volatile uint8_t g_fx_chain_pub_idx = 0;
volatile uint8_t g_fx_chain_active_idx = 0;
PreviewControl g_preview_ctl_buf[2];
volatile uint8_t g_preview_pub_idx = 0;
volatile uint8_t g_preview_active_idx = 0;

extern int16_t* sample_buffer_l;
extern int16_t* sample_buffer_r;
extern int16_t preview_buffer[kPreviewBufferFrames];


PerformState main_perform_state;
UiMode edt_prev_mode = UiMode::Perform;
SampleContext edt_sample_context = SampleContext::Perform;
UiMode fx_detail_prev_mode = UiMode::Perform;
UiMode load_prev_mode = UiMode::Main;
LoadContext load_context = LoadContext::Main;
int32_t load_mode_index = 0;
LoadStubMode load_stub_mode = LoadStubMode::Presets;

// Normalized trim window (0..1 over entire sample)
float trim_start = 0.0f;
float trim_end = 1.0f;

// Derived frame window (engine space)
uint32_t snap_start_frame = 0;
uint32_t snap_end_frame = 0;

// Waveform preview buffers (128 columns)
bool waveform_ready = false;
bool waveform_dirty = false;

// Waveform computation request (from audio callback to main loop)
volatile bool waveform_compute_pending = false;
volatile SampleContext waveform_compute_ctx = SampleContext::Perform;

bool waveform_from_recording = false;
RecordInput waveform_record_input = RecordInput::LineIn;
volatile float perform_attack_norm = 0.0f;
volatile float perform_release_norm = 0.0f;
const char* waveform_title = nullptr;

constexpr size_t kRecordMaxFrames = kMaxSampleFrames;
constexpr uint32_t kRecordCountdownMs = 4000;

// Control event bits (for snapshot-based input passing from main loop to audio callback)
enum CtrlEventBits : uint32_t {
	kEncLPress   = 1U << 0,
	kEncRPress   = 1U << 1,
	kBtn1Press   = 1U << 2,
	kBtn2Press   = 1U << 3,
	kShiftRise   = 1U << 4,
	kShiftFall   = 1U << 5,
};

// Control snapshot (shared between main loop and audio callback)
volatile int32_t g_enc_l_delta = 0;
volatile int32_t g_enc_r_delta = 0;
volatile uint32_t g_ctrl_events = 0;
volatile bool g_shift_held = false;
volatile bool g_btn1_held = false;
bool ui_button1_held = false;

volatile uint32_t g_audio_cmd = 0;
volatile uint32_t g_audio_event_bits = 0;

void RequestAudioCmd(uint32_t bits)
{
	{
		daisy::ScopedIrqBlocker irq;
		g_audio_cmd |= bits;
	}
}

void RequestPlaybackStart(uint8_t note, bool apply_pitch)
{
	const uint8_t flags = apply_pitch ? kPlaybackCmdApplyPitch : 0;
	PlaybackCmdPushUi(kPlaybackCmdStart, note, flags);
}

void RequestPlaybackStop(uint8_t note, bool apply_release)
{
	const uint8_t flags = apply_release ? kPlaybackCmdApplyRelease : 0;
	PlaybackCmdPushUi(kPlaybackCmdStop, note, flags);
}

void RequestPlaybackStopAll()
{
	PlaybackCmdPushUi(kPlaybackCmdStopAll, 0, 0);
}

void PushAudioEvent(uint32_t bits)
{
	{
		daisy::ScopedIrqBlocker irq;
		g_audio_event_bits |= bits;
	}
}

WaveformJob g_wf_job = {};
FileListJob g_list_job = {};

FxParamsAudio g_fx_params_buf[2];
volatile uint8_t g_fx_params_idx = 0;
AudioParamsAudio g_audio_params_audio_buf[2];
volatile uint8_t g_audio_params_audio_idx = 0;

int BitResoIndexFromValue(float value);
extern volatile int32_t sat_mode;
extern volatile int32_t chorus_mode;

float Clamp01(float v)
{
	if (v < 0.0f) return 0.0f;
	if (v > 1.0f) return 1.0f;
	return v;
}

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

DTCM_MEM_SECTION AudioParams g_audio_params_buf[2] = {};
volatile uint8_t g_audio_params_pub_idx = 0;
volatile uint8_t g_audio_params_active_idx = 0;

constexpr float kSilentAmp = 1.0e-4f;
constexpr uint16_t kSilentSamplesToKill = 64;
constexpr size_t kLiveWaveWindowFrames = kSampleRateHz / 2;
constexpr size_t kLiveWaveStride = (kLiveWaveWindowFrames / kWaveCols) > 0
	? (kLiveWaveWindowFrames / kWaveCols)
	: 1;
const AudioUiState& GetAudioUiStateSnapshot(uint8_t& idx);

volatile uint32_t g_audio_flags_bits = 0;
AudioUiState g_audio_ui_state_buf[2];
volatile uint8_t g_audio_ui_state_idx = 0;
volatile bool g_audio_recording_active = false;
bool g_reset_voices_pending = false;
volatile int32_t g_active_voice_count = 0;
uint32_t g_voice_skip_count = 0;
uint32_t g_voice_kill_count = 0;
volatile float g_delay_time_alpha = 1.0f;
volatile float g_delay_param_alpha = 1.0f;
FxChainRuntime g_fx_chain_audio = {};
bool g_fx_chain_audio_valid = false;

volatile RecordState record_state = RecordState::Armed;
volatile int32_t record_source_index = 0;
volatile int32_t record_target_index = kRecordTargetSave;
volatile uint32_t record_countdown_start_ms = 0;
static uint32_t record_draw_next_ms = 0;
volatile size_t record_pos = 0;
volatile size_t g_recorded_length_audio = 0;
volatile uint32_t g_record_start_ms = 0;
volatile bool record_waveform_pending = false;
volatile int32_t encoder_r_accum = 0;
volatile bool encoder_r_button_press = false;
volatile bool request_length_redraw = false;
#if PERF_DIAGNOSTICS
float cpu_load_ema = 0.0f;
volatile float cpu_load_pct = 0.0f;
volatile float cpu_load_peak_pct = 0.0f;
volatile uint32_t callback_cycles_last = 0;
volatile uint32_t callback_cycles_max = 0;
volatile uint32_t callback_overruns = 0;
#endif
volatile bool request_playhead_redraw = false;
volatile bool button1_press = false;
volatile bool button2_press = false;
volatile bool request_playback_stop_log = false;
volatile float reverb_wet = kReverbDefaultWet;
volatile float reverb_pre = 0.5f;
volatile float reverb_damp = 0.5f;
volatile float reverb_decay = 0.5f;
volatile float delay_wet = kDelayDefaultWet;
volatile float delay_time = 0.5f;
volatile float delay_feedback = 0.5f;
volatile float delay_spread = 0.5f;
volatile float delay_freeze = 0.0f;
volatile float fx_s_wet = 0.0f;
volatile float sat_drive = 0.5f;
volatile float sat_tape_bump = 0.5f;
volatile float sat_bit_reso = 0.5f;
volatile float sat_bit_smpl = 0.5f;
volatile float fx_c_wet = 0.0f;
volatile float mod_depth = 0.5f;
volatile float chorus_rate = 0.5f;
volatile int32_t sat_mode = 0;
volatile int32_t chorus_mode = 0;
volatile float chorus_wow = 0.5f;
volatile float tape_rate = 0.5f;
volatile bool fx_params_dirty = true;
volatile bool audio_params_dirty = true;
bool sat_params_initialized = false;
bool reverb_params_initialized = false;
bool delay_params_initialized = false;
bool mod_params_initialized = false;
volatile float amp_attack = 0.0f;
volatile float amp_decay = 0.0f;
volatile float amp_sustain = 0.0f;
volatile float amp_release = 0.0f;
volatile int32_t fx_detail_index = 0;
volatile int32_t fx_detail_param_index = 0;
volatile float flt_cutoff = 1.0f;
volatile float flt_res = 0.02f;
volatile bool preview_hold = false;
float fx_chain_fade_gain = 1.0f;
float fx_chain_fade_target = 1.0f;
int32_t fx_chain_fade_samples_left = 0;
bool fx_chain_pause_pending = false;
bool fx_chain_paused = false;
uint32_t fx_chain_last_move_ms = 0;
volatile bool preview_active = false;
volatile int32_t preview_index = -1;
volatile uint32_t preview_sample_rate = 48000;
volatile uint16_t preview_channels = 1;
volatile float preview_rate = 1.0f;
volatile float preview_read_frac = 0.0f;
volatile size_t preview_read_index = 0;
volatile size_t preview_write_index = 0;
volatile uint32_t preview_data_offset = 0;
#if STORAGE_SERVICE_PREVIEW_STREAM
volatile uint32_t preview_underrun_count = 0;
volatile uint32_t preview_rb_min_level = 0xFFFFFFFFu;
#endif
volatile uint32_t preview_fade_samples_left = 0;
volatile uint32_t preview_fade_samples_total = 0;
#if STORAGE_SERVICE_PREVIEW_STREAM
uint16_t preview_stream_cookie = 1;
uint16_t preview_stream_cookie_active = 0;
#endif
bool preview_pending_start = false;
__attribute__((unused)) uint32_t preview_pending_start_ms = 0;
#if STORAGE_SERVICE_PREVIEW_STREAM
volatile uint8_t preview_pp_ready[2] = {0, 0};
volatile uint8_t preview_pp_active = 0;
volatile uint32_t preview_pp_pos = 0;
volatile size_t preview_preload_frames = 0;
volatile bool preview_preload_active = false;
#endif
float led1_level = 0.0f;
float led1_phase_ms = 0.0f;
double record_anim_start_ms = -1.0;
  RAM_D3_MEM_SECTION uint8_t record_text_mask[kDisplayH][kDisplayW];
  RAM_D3_MEM_SECTION uint8_t record_invert_mask[kDisplayH][kDisplayW];
  RAM_D3_MEM_SECTION uint8_t record_fb_buf[kDisplayH][kDisplayW];
  RAM_D3_MEM_SECTION uint8_t record_bold_mask[kDisplayH][kDisplayW];
bool request_shift_redraw = false;
bool request_perform_redraw = false;
bool request_fx_detail_redraw = false;
static uint32_t perform_redraw_next_ms = 0;
static float last_perform_amp_vals[kPerformFaderCount] = {-1.0f, -1.0f, -1.0f, -1.0f};
static float last_perform_flt_vals[kPerformFltFaderCount] = {-1.0f, -1.0f};
static float last_perform_fx_vals[kPerformFaderCount] = {-1.0f, -1.0f, -1.0f, -1.0f};
static int32_t last_perform_fx_order[kPerformFaderCount] = {-1, -1, -1, -1};
static bool last_perform_fx_select_active = false;
static bool last_perform_amp_select_active = false;
static bool last_perform_flt_select_active = false;
static int32_t last_perform_fx_selected = -1;
static int32_t last_perform_amp_selected = -1;
static int32_t last_perform_flt_selected = -1;
uint32_t delay_snow_next_ms = 0;
uint32_t midi_ignore_until_ms = 0;

extern Job g_job;

double NowMs()
{
	return static_cast<double>(System::GetNow());
}

void ApplyPlaybackReverse(bool reverse)
{
	const bool current = (playback_reverse >= 0.5f);
	if (reverse == current)
	{
		return;
	}
	playback_reverse = reverse ? 1.0f : 0.0f;
	{
		daisy::ScopedIrqBlocker irq;
		g_playback_reverse_target = reverse;
	}
	RequestAudioCmd(kCmdPlaybackReverse);
}


void StartPlaybackAudio(uint8_t note, bool apply_pitch, bool reverse_playback)
{
	const SampleRuntime rt = g_rt_buf[g_rt_active_idx];
	if (!rt.loaded || rt.length < 1 || rt.l == nullptr)
	{
		return;
	}
	size_t window_start = rt.play_start;
	size_t window_end = rt.play_end;
	if (window_end > rt.length || window_end == 0)
	{
		window_end = rt.length;
	}
	if (window_end <= window_start)
	{
		window_start = 0;
		window_end = rt.length;
	}
	if (window_end == 0)
	{
		return;
	}
	current_note = note;
	playback_amp = 1.0f;
	playback_env_samples = 0;
	playback_release_active = false;
	playback_release_pos = 0.0f;
	playback_release_start = 0.0f;
	const uint8_t idx = (note < 128) ? note : 127;
	const float pitch = apply_pitch ? g_note_ratio[idx] : 1.0f;
	const float sr = (rt.rate == 0) ? 48000.0f : static_cast<float>(rt.rate);
	playback_rate = pitch * (sr / hw.AudioSampleRate());
	playback_phase = static_cast<float>(reverse_playback ? (window_end - 1) : window_start);
	playback_active = true;
}

void StopPlaybackAudio(uint8_t note, bool apply_release)
{
	if (note == current_note)
	{
		if (apply_release && playback_active)
		{
			playback_release_active = true;
			playback_release_pos = 0.0f;
			playback_release_start = -1.0f;
		}
		else
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

void StopPlaybackAllAudio()
{
	if (playback_active)
	{
		playback_active = false;
		playback_env_samples = 0;
		playback_release_active = false;
		playback_release_pos = 0.0f;
		playback_release_start = 0.0f;
		PushAudioEvent(kAudioEventPlaybackStopped);
	}
}


static void __attribute__((unused)) HandleMidiMessage(MidiEvent msg)
{
	if (ui_mode == UiMode::Record && record_state == RecordState::Recording)
	{
		return;
	}
	if (IsPerformUiMode(ui_mode))
	{
		const bool ignore_note_on = (System::GetNow() < midi_ignore_until_ms);
		switch (msg.type)
		{
			case NoteOn:
			{
				const NoteOnEvent note = msg.AsNoteOn();
				if (ignore_note_on && note.velocity > 0)
				{
					return;
				}
				if (note.velocity == 0)
				{
					StopPerformVoice(note.note);
				}
				else
				{
					StartPerformVoice(note.note);
				}
			}
			break;
			case NoteOff:
			{
				const NoteOffEvent note = msg.AsNoteOff();
				StopPerformVoice(note.note);
			}
			break;
			default:
				break;
		}
		return;
	}
	switch (msg.type)
	{
		case NoteOn:
		{
			const NoteOnEvent note = msg.AsNoteOn();
			if (note.velocity == 0)
			{
				RequestPlaybackStop((uint8_t)note.note, false);
			}
			else
			{
				RequestPlaybackStart((uint8_t)note.note, true);
			}
		}
		break;
		case NoteOff:
	{
		const NoteOffEvent note = msg.AsNoteOff();
		RequestPlaybackStop((uint8_t)note.note, false);
	}
	break;
	default:
		break;
	}
}

static void CtrlTimerCb(void* /*data*/)
{
	if (!g_ctrl_timer_running)
	{
		return;
	}

	hw.ProcessAllControls();
	encoder_r.Debounce();
	shift_button.Debounce();

	const int32_t enc_l_inc = hw.encoder.Increment();
	const int32_t enc_r_inc = encoder_r.Increment();

	if (g_midi_rx_started)
	{
		// Listen() is only for overrun recovery; safe to call once RX is started.
		hw.midi.Listen();
		while (hw.midi.HasEvents())
		{
			MidiEvent msg = hw.midi.PopEvent();
			switch (msg.type)
			{
				case NoteOn:
				{
					const NoteOnEvent n = msg.AsNoteOn();
					if (n.velocity == 0)
					{
						MidiCmdPushIsr(kMidiCmdNoteOff, (uint8_t)n.note, 0);
					}
					else
					{
						if (IsPerformUiMode(ui_mode) && (System::GetNow() < midi_ignore_until_ms))
						{
							break;
						}
						MidiCmdPushIsr(kMidiCmdNoteOn, (uint8_t)n.note, (uint8_t)n.velocity);
					}
				}
				break;
				case NoteOff:
				{
					const NoteOffEvent n = msg.AsNoteOff();
					MidiCmdPushIsr(kMidiCmdNoteOff, (uint8_t)n.note, 0);
				}
				break;
				default:
					break;
			}
		}
	}

	uint32_t ev = 0;
	if (hw.encoder.RisingEdge())      ev |= kEncLPress;
	if (encoder_r.RisingEdge())       ev |= kEncRPress;
	if (hw.button1.RisingEdge())      ev |= kBtn1Press;
	if (hw.button2.RisingEdge())      ev |= kBtn2Press;
	if (shift_button.RisingEdge())    ev |= kShiftRise;
	if (shift_button.FallingEdge())   ev |= kShiftFall;

	const bool shift_held = shift_button.Pressed();
	const bool btn1_held = hw.button1.Pressed();

	{
		daisy::ScopedIrqBlocker irq;
		g_enc_l_delta += enc_l_inc;
		g_enc_r_delta += enc_r_inc;
		g_ctrl_events |= ev;
		g_shift_held = shift_held;
		g_btn1_held = btn1_held;
	}
}

static void InitControlTimer()
{
	daisy::TimerHandle::Config cfg;
	cfg.periph = daisy::TimerHandle::Config::Peripheral::TIM_3;
	cfg.dir = daisy::TimerHandle::Config::CounterDir::UP;
	cfg.enable_irq = true;

	g_ctrl_timer.Init(cfg);

	constexpr uint32_t kCtrlTimerHz = 10000;
	const uint32_t tim_clk = daisy::System::GetPClk1Freq() * 2U;
	uint32_t psc = tim_clk / kCtrlTimerHz;
	if (psc == 0)
	{
		psc = 1;
	}
	psc -= 1;
	if (psc > 0xFFFFU)
	{
		psc = 0xFFFFU;
	}
	g_ctrl_timer.SetPrescaler(psc);
	g_ctrl_timer.SetPeriod(kCtrlTimerHz / 1000);
	g_ctrl_timer.SetCallback(CtrlTimerCb, nullptr);

	g_ctrl_timer_running = true;
	g_ctrl_timer.Start();
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	g_audio_engine.Process(in, out, size);
}

int main(void)
{
	hw.Init();
	EnableFtz();
	InitSineTable();
	InitNoteRatioTable();
	#if PERF_DIAGNOSTICS
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	#endif
	hw.SetAudioBlockSize(kAudioBlockSize); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	UpdateDelaySlewCoeffs();
	reverb.Init(hw.AudioSampleRate());
	reverb.SetFeedback(kReverbFeedback);
	reverb.SetLpFreq(kReverbLpFreq);

	sat_l.Init(hw.AudioSampleRate());
	sat_r.Init(hw.AudioSampleRate());
	sat_l.SetTone(0.5f);
	sat_r.SetTone(0.5f);
	sat_l.SetBias(0.0f);
	sat_r.SetBias(0.0f);
	sat_l.SetOutput(0.666f);
	sat_r.SetOutput(0.666f);
	sat_l.SetDrive(0.0f);
	sat_r.SetDrive(0.0f);
	sat_l.SetMix(1.0f);
	sat_r.SetMix(1.0f);
	sat_l.SetBump(0.0f);
	sat_r.SetBump(0.0f);

	chorus_l.Init(hw.AudioSampleRate());
	chorus_r.Init(hw.AudioSampleRate());
	chorus_l.SetLfoFreq(kChorusRateHz);
	chorus_r.SetLfoFreq(-kChorusRateHz);
	chorus_l.SetDelayMs(kChorusDelayMs);
	chorus_r.SetDelayMs(kChorusDelayMs);
	chorus_l.SetFeedback(kChorusFeedback);
	chorus_r.SetFeedback(kChorusFeedback);
	chorus_l.SetLfoDepth(0.0f);
	chorus_r.SetLfoDepth(0.0f);
	chorus_rate = 0.5f;
	chorus_wow = 0.5f;
	tape_rate = 0.5f;

	delay_line_l.Init();
	delay_line_r.Init();
	const float delay_init_ms = kDelayTimeMinMs;
	float delay_samples = delay_init_ms * 0.001f * hw.AudioSampleRate();
	const float max_delay = static_cast<float>(kDelayMaxSamples - 1);
	if (delay_samples > max_delay)
	{
		delay_samples = max_delay;
	}
	if (delay_samples < 1.0f)
	{
		delay_samples = 1.0f;
	}
	delay_line_l.SetDelay(delay_samples);
	delay_line_r.SetDelay(delay_samples);

	reverb_predelay_l.Init();
	reverb_predelay_r.Init();
	reverb_predelay_l.SetDelay(0.0f);
	reverb_predelay_r.SetDelay(0.0f);

	InitSmoothers();
	UpdateSmoothedParamsPerTick();

	encoder_r.Init(seed::D7, seed::D8, seed::D22, hw.AudioSampleRate());
	shift_button.Init(seed::D9, 1000);
	InitControlTimer();

	PodDisplay::Config disp_cfg;
	disp_cfg.driver_config.transport_config.i2c_config.pin_config.scl = seed::D11;
	disp_cfg.driver_config.transport_config.i2c_config.pin_config.sda = seed::D12;
	disp_cfg.driver_config.transport_config.i2c_config.speed
		= I2CHandle::Config::Speed::I2C_400KHZ;
	disp_cfg.driver_config.transport_config.i2c_address = 0x3C;
	display.Init(disp_cfg);
	InitLoadLayout();

	g_rt_buf[0].l = sample_buffer_l;
	g_rt_buf[0].r = sample_buffer_r;
	g_rt_buf[0].length = 0;
	g_rt_buf[0].play_start = 0;
	g_rt_buf[0].play_end = 0;
	g_rt_buf[0].rate = sample_rate;
	g_rt_buf[0].channels = sample_channels;
	g_rt_buf[0].loaded = false;
	g_rt_buf[1] = g_rt_buf[0];
	g_rt_pub_idx = 0;
	g_rt_active_idx = 0;

	FxChainRuntime fx_init = {};
	for (int i = 0; i < kPerformFaderCount; ++i)
	{
		fx_init.order[i] = static_cast<uint8_t>(fx_chain_order[i]);
	}
	fx_init.count = kPerformFaderCount;
	fx_init.paused = fx_chain_paused;
	fx_init.pause_pending = fx_chain_pause_pending;
	fx_init.fade_gain = fx_chain_fade_gain;
	fx_init.fade_target = fx_chain_fade_target;
	fx_init.fade_samples_left = fx_chain_fade_samples_left;
	g_fx_chain_buf[0] = fx_init;
	g_fx_chain_buf[1] = fx_init;
	g_fx_chain_pub_idx = 0;
	g_fx_chain_active_idx = 0;
	g_fx_chain_audio = fx_init;
	g_fx_chain_audio_valid = true;

	PreviewControl preview_init = {};
	preview_init.l = preview_buffer;
	preview_init.r = nullptr;
	preview_init.length = kPreviewBufferFrames;
	preview_init.rate = 1.0f;
	preview_init.gain = 1.0f;
	preview_init.mono = true;
	g_preview_ctl_buf[0] = preview_init;
	g_preview_ctl_buf[1] = preview_init;
	g_preview_pub_idx = 0;
	g_preview_active_idx = 0;

	{
		uint32_t bits = 0;
		const bool in_perform_mode = IsPerformUiMode(ui_mode);
		if (in_perform_mode) bits |= kFlagInPerformMode;
		if (ui_mode == UiMode::Main) bits |= kFlagInMainMode;
		if (in_perform_mode || (ui_mode == UiMode::FxDetail))
		{
			bits |= kFlagFxAllowed;
		}
		if (ui_mode == UiMode::Record
			&& record_state != RecordState::Review
			&& record_state != RecordState::SourceSelect
			&& record_state != RecordState::BackConfirm
			&& record_state != RecordState::TargetSelect)
		{
			bits |= kFlagMonitorEnabled;
		}
		{
			daisy::ScopedIrqBlocker irq;
			g_audio_flags_bits = bits;
		}
	}

	hw.StartAdc();
	hw.StartAudio(AudioCallback);
	hw.midi.StartReceive();
	g_midi_rx_started = true;
	g_app.Init();
	while (1)
	{
		g_app.Tick(System::GetNow());
	}
}



 
