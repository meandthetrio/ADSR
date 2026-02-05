#include "daisy_pod.h"
#include "daisysp.h"
#include "dev/oled_ssd130x.h"
#include "per/tim.h"
#include "util/scopedirqblocker.h"
#include "SamplerConfig.h"
#include "shared_messages.h"
#include "audio_engine.h"
#include "ui.h"
#include "audio_dsp.h"
#include "PerformVoice.h"
#include "SampleMemoryManager.h"
#include "VoiceManager.h"
#include "WaveformCache.h"
#include "StorageService.h"
#include "ui.h"
#include <cmath>
#include <initializer_list>
//#include <math.h>
#include <cstring>
#include <cstdio>

using namespace daisy;
using namespace daisysp;

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
static Ui g_ui;
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


StorageService storage;
Encoder      encoder_r;
Switch       shift_button;
DSY_SDRAM_BSS ReverbSc reverb;
DelayLine<float, kDelayMaxSamples> DSY_SDRAM_BSS delay_line_l;
DelayLine<float, kDelayMaxSamples> DSY_SDRAM_BSS delay_line_r;
DelayLine<float, kReverbPreDelayMaxSamples> DSY_SDRAM_BSS reverb_predelay_l;
DelayLine<float, kReverbPreDelayMaxSamples> DSY_SDRAM_BSS reverb_predelay_r;
DTCM_MEM_SECTION ChorusEngine chorus_l;
DTCM_MEM_SECTION ChorusEngine chorus_r;
DTCM_MEM_SECTION TapeSaturator sat_l;
DTCM_MEM_SECTION TapeSaturator sat_r;
DTCM_MEM_SECTION BitCrushState g_sat_bit_state;
DTCM_MEM_SECTION BiquadLp perform_lpf_l1[kPerformVoiceCount];
DTCM_MEM_SECTION BiquadLp perform_lpf_l2[kPerformVoiceCount];
DTCM_MEM_SECTION BiquadLp perform_lpf_r1[kPerformVoiceCount];
DTCM_MEM_SECTION BiquadLp perform_lpf_r2[kPerformVoiceCount];

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
static volatile size_t sample_mem_used_bytes = 0;
static volatile size_t sample_mem_free_bytes = 0;
static volatile size_t waveform_cache_bytes = 0;
volatile int32_t wav_file_count = 0;

bool sd_mounted = false;
bool sd_present = false;
bool sd_fault = false;
StorageService::SdErrorCode sd_fault_code = StorageService::SdErrorCode::None;
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

DSY_SDRAM_BSS int16_t perform_sample_buffer_l[kMaxSampleFrames];
DSY_SDRAM_BSS int16_t perform_sample_buffer_r[kMaxSampleFrames];
int16_t* sample_buffer_l = perform_sample_buffer_l;
int16_t* sample_buffer_r = perform_sample_buffer_r;
SampleMemoryManager sample_mem_mgr(perform_sample_buffer_l, kPerformSampleRamBudgetBytes);
static VoiceManager voice_mgr;
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

DTCM_MEM_SECTION PerformVoice perform_voices[kPerformVoiceCount];

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
static volatile bool waveform_compute_pending = false;
static volatile SampleContext waveform_compute_ctx = SampleContext::Perform;

WaveformCache perform_waveform_cache;
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
static volatile uint32_t g_audio_event_bits = 0;

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
alignas(32) int16_t preview_buffer[kPreviewBufferFrames];
#if !STORAGE_SERVICE_PREVIEW_STREAM
alignas(32) __attribute__((unused)) static int16_t preview_read_buf[kPreviewReadFrames * 2];
#endif
#if STORAGE_SERVICE_PREVIEW_STREAM
alignas(32) int16_t preview_pp_buf[2][kPreviewPpFrames];
volatile uint8_t preview_pp_ready[2] = {0, 0};
volatile uint8_t preview_pp_active = 0;
volatile uint32_t preview_pp_pos = 0;
DSY_SDRAM_BSS int16_t preview_preload_buf[kPreviewPreloadFrames];
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

const char* SdFaultText(StorageService::SdErrorCode code);
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

void ApplyPlaybackReverseAudio(bool reverse)
{
	if (reverse == playback_reverse_audio)
	{
		return;
	}
	playback_reverse_audio = reverse;

	const SampleRuntime rt = g_rt_buf[g_rt_active_idx];
	if (!rt.loaded || rt.length == 0 || rt.l == nullptr)
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
	if (window_end <= window_start + 1)
	{
		return;
	}
	const float window_len = static_cast<float>(window_end - window_start - 1);
	const float rel = playback_phase - static_cast<float>(window_start);
	playback_phase = static_cast<float>(window_start) + (window_len - rel);
	if (playback_phase < static_cast<float>(window_start))
	{
		playback_phase = static_cast<float>(window_start);
	}
	if (playback_phase > static_cast<float>(window_end - 1))
	{
		playback_phase = static_cast<float>(window_end - 1);
	}
	for (auto &voice : perform_voices)
	{
		if (!voice.active || voice.length <= 1)
		{
			continue;
		}
		const float vlen = static_cast<float>(voice.length - 1);
		voice.phase = vlen - voice.phase;
		if (voice.phase < 0.0f)
		{
			voice.phase = 0.0f;
		}
		if (voice.phase > vlen)
		{
			voice.phase = vlen;
		}
	}
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

void StartPerformVoice(int32_t note)
{
	const SampleRuntime rt = g_rt_buf[g_rt_active_idx];
	size_t window_start = 0;
	size_t window_end = 0;
	if (!rt.loaded || rt.length == 0 || rt.l == nullptr)
	{
		return;
	}
	window_start = rt.play_start;
	window_end = rt.play_end;
	if (window_end > rt.length || window_end == 0)
	{
		window_end = rt.length;
	}
	if (window_end <= window_start)
	{
		window_start = 0;
		window_end = rt.length;
	}
	if (window_end <= window_start)
	{
		return;
	}

	const int voice_index = voice_mgr.SelectVoiceIndex(note);
	if (voice_index < 0)
	{
		return;
	}

	PerformVoice& voice = perform_voices[voice_index];
	DeactivateVoice(voice);
	voice.active = true;
	++g_active_voice_count;
	voice.releasing = false;
	if (sample_mem_mgr.Acquire(kPerformSampleId))
	{
		voice.sample_acquired = true;
	}
	else
	{
		voice.sample_acquired = false;
	}
	voice.note = note;
	voice.track = -1;
	voice.phase = 0.0f;
	voice.amp = 1.0f;
	voice.env = 0.0f;
	voice.release_start = 0.0f;
	voice.release_pos = 0.0f;
	voice.env_samples = 0;
	voice.silent_samples = 0;
	voice.start_tick = static_cast<uint32_t>(System::GetNow());
	perform_lpf_l1[voice_index].Reset();
	perform_lpf_l2[voice_index].Reset();
	perform_lpf_r1[voice_index].Reset();
	perform_lpf_r2[voice_index].Reset();
	const float sr = (rt.rate == 0) ? 48000.0f : static_cast<float>(rt.rate);
	const uint8_t idx = (note >= 0 && note < 128) ? static_cast<uint8_t>(note) : 127;
	const float pitch = g_note_ratio[idx];
	voice.rate = pitch * (sr / hw.AudioSampleRate());
	voice.offset = window_start;
	voice.length = window_end - window_start;
}

void StopPerformVoice(int32_t note)
{
	for (auto &voice : perform_voices)
	{
		if (voice.active && voice.note == note)
		{
			if (!voice.releasing)
			{
				voice.releasing = true;
				voice.release_pos = 0.0f;
				voice.release_start = voice.env;
				if (voice.release_start < 0.0f)
				{
					voice.release_start = 0.0f;
				}
				else if (voice.release_start > 1.0f)
				{
					voice.release_start = 1.0f;
				}
			}
		}
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

	storage.Init();
	voice_mgr.Init(perform_voices, kPerformVoiceCount);
	ValidateConfig();
	{
		StorageService::PreviewStreamConfig cfg = {};
		cfg.buffer = preview_buffer;
		cfg.frames = kPreviewBufferFrames;
		cfg.write_index = &preview_write_index;
		cfg.read_index = &preview_read_index;
#if STORAGE_SERVICE_PREVIEW_STREAM
		cfg.preload_buf = preview_preload_buf;
		cfg.preload_frames = kPreviewPreloadFrames;
		cfg.pp_buf_a = &preview_pp_buf[0][0];
		cfg.pp_buf_b = &preview_pp_buf[1][0];
		cfg.pp_frames = kPreviewPpFrames;
		cfg.pp_ready_a = &preview_pp_ready[0];
		cfg.pp_ready_b = &preview_pp_ready[1];
		cfg.pp_active = &preview_pp_active;
#endif
		storage.SetPreviewStreamConfig(cfg);
	}
	MountSd();

	DrawMenu(menu_index);
	g_last_draw_ms = System::GetNow();
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
	UiMode last_mode = UiMode::Main;
	int32_t last_menu = -1;
	int32_t last_shift_menu = -1;
	int32_t last_perform_index = -1;
	int32_t last_fx_detail_index = -1;
	int32_t last_fx_detail_param_index = -1;
	uint32_t rev_anim_next_ms = 0;
	int32_t last_scroll = -1;
	int32_t last_selected = -1;
	int32_t last_file_count = -1;
	bool last_sd_mounted = false;
	RecordState last_record_state = RecordState::Armed;
	bool last_playback_active = false;
	uint8_t last_audio_ui_idx = 0xFF;
	uint32_t last_edt_playhead_ms = 0;
	bool last_edt_playhead_active = false;
	g_ui.Init();
	while(1)
	{
		const uint32_t storage_budget = (preview_pending_start || preview_hold)
			? kStoragePreviewBudgetUs
			: kStorageBudgetUs;
		// Preview streaming advances only here (UI loop), never in audio.
		storage.RunSlice(storage_budget);
		{
			StorageService::Event ev = {};
			while (storage.DequeueEvent(ev))
			{
				if (ev.kind == StorageService::EventKind::MountOk)
				{
					sd_mounted = true;
				}
				else if (ev.kind == StorageService::EventKind::MountFail)
				{
					sd_mounted = false;
				}
				else if (ev.kind == StorageService::EventKind::DirEntry)
				{
					if (!g_list_job.active || ev.cookie != g_list_job.cookie)
					{
						continue;
					}
					if (g_list_job.count >= kMaxWavFiles)
					{
						continue;
					}
					CopyString(wav_files[g_list_job.count], ev.name, kMaxWavNameLen);
					g_list_job.count++;
					wav_file_count = g_list_job.count;
					request_delete_redraw = true;
					RequestDisplayUpdate();
				}
				else if (ev.kind == StorageService::EventKind::ScanDone)
				{
					if (!g_list_job.active || ev.cookie != g_list_job.cookie)
					{
						continue;
					}
					g_list_job.active = false;
					g_list_job.done = true;
					FinalizeFileList();
					if (g_job.type == JobType::FileListScan)
					{
						g_job = {};
					}
				}
#if STORAGE_SERVICE_PREVIEW_STREAM
				else if (ev.kind == StorageService::EventKind::PreviewOpenOk)
				{
					if (ev.cookie != preview_stream_cookie_active)
					{
						continue;
					}
					if (!preview_hold)
					{
						StorageService::Op op = {};
						op.kind = StorageService::OpKind::PreviewClose;
						storage.Enqueue(op);
						continue;
					}
					preview_sample_rate = ev.sample_rate;
					preview_channels = ev.channels;
					const uint32_t rate = (preview_sample_rate == 0) ? 48000 : preview_sample_rate;
					preview_rate = static_cast<float>(rate) / hw.AudioSampleRate();
					{
						daisy::ScopedIrqBlocker irq;
						preview_read_index = 0;
						preview_write_index = 0;
					}
					preview_underrun_count = 0;
					preview_rb_min_level = 0xFFFFFFFFu;
					preview_pp_ready[0] = 0;
					preview_pp_ready[1] = 0;
					preview_pp_active = 0;
					preview_pp_pos = 0;
					preview_preload_frames = static_cast<size_t>(ev.size);
					preview_preload_active = (preview_preload_frames > 0);
					const uint32_t fade_samples = static_cast<uint32_t>(
						(hw.AudioSampleRate() * static_cast<float>(kPreviewFadeInMs)) / 1000.0f + 0.5f);
					preview_fade_samples_total = (fade_samples > 0) ? fade_samples : 1;
					preview_fade_samples_left = preview_fade_samples_total;
					preview_pending_start = !preview_preload_active;
					preview_pending_start_ms = System::GetNow();
					if (preview_preload_active)
					{
						PublishPreviewControlFromUi();
						RequestAudioCmd(kCmdPreviewStart);
					}
				}
				else if (ev.kind == StorageService::EventKind::PreviewOpenFail)
				{
					if (ev.cookie != preview_stream_cookie_active)
					{
						continue;
					}
					StopPreview();
				}
				else if (ev.kind == StorageService::EventKind::PreviewReadError)
				{
					if (ev.cookie != preview_stream_cookie_active)
					{
						continue;
					}
					StopPreview();
				}
#endif
#if STORAGE_SERVICE_SAVE
				else if (ev.kind == StorageService::EventKind::SaveProgress)
				{
					save_frames_written = static_cast<size_t>(ev.value);
					if (ev.name[0] != '\0')
					{
						CopyString(save_filename, ev.name, kMaxWavNameLen);
					}
				}
				else if (ev.kind == StorageService::EventKind::SaveDone)
				{
					if (!save_done)
					{
						save_success = true;
						save_done = true;
						save_result_until_ms = System::GetNow() + kSaveResultMs;
						request_load_scan = true;
						CopyString(loaded_sample_name, save_filename, kMaxWavNameLen);
					}
				}
				else if (ev.kind == StorageService::EventKind::SaveError)
				{
					if (!save_done)
					{
						save_success = false;
						save_done = true;
						save_result_until_ms = System::GetNow() + kSaveResultMs;
					}
				}
#endif
#if 1
				else if (ev.kind == StorageService::EventKind::LoadProgress)
				{
					if (ev.cookie == load_cookie_active)
					{
						load_in_progress = true;
						loader_state = LoaderState::Loading;
					}
				}
				else if (ev.kind == StorageService::EventKind::LoadDone)
				{
					if (ev.cookie != load_cookie_active)
					{
						continue;
					}
					load_in_progress = false;
					load_success_count++;
					sample_length = static_cast<size_t>(ev.value);
					sample_channels = (ev.channels == 0) ? 1 : ev.channels;
					sample_rate = (ev.sample_rate == 0) ? 48000 : ev.sample_rate;
					sample_loaded = (sample_length > 0);
					if (!sample_loaded)
					{
						ui_mode = UiMode::Load;
						load_context = LoadContext::Main;
						continue;
					}
					ApplyLoadedSampleFade(sample_length, sample_rate);
					trim_start = 0.0f;
					trim_end = 1.0f;
					waveform_from_recording = false;
					JobStartWaveform(current_sample_context, true);
					UpdateTrimFrames();
					PublishRuntimeFromUi();
					if (load_target_is_edt)
					{
						ui_mode = UiMode::Edt;
						waveform_dirty = true;
						request_length_redraw = true;
					}
					else
					{
						ui_mode = UiMode::Perform;
						menu_index = 2;
					}
					load_context = LoadContext::Main;
					loader_state = LoaderState::Ready;
				}
				else if (ev.kind == StorageService::EventKind::LoadError)
				{
					if (ev.cookie != load_cookie_active)
					{
						continue;
					}
					load_in_progress = false;
					load_cookie_active = 0;
					load_fail_io_count++;
					sample_mem_mgr.Free(kPerformSampleId);
					ui_mode = UiMode::Load;
					load_context = LoadContext::Main;
					loader_state = LoaderState::Failed;
				}
				else if (ev.kind == StorageService::EventKind::DeleteOk)
				{
					if (ev.cookie != delete_cookie_active)
					{
						continue;
					}
					delete_in_progress = false;
					delete_cookie_active = 0;
					request_delete_scan = true;
					delete_confirm = false;
					request_delete_redraw = true;
				}
				else if (ev.kind == StorageService::EventKind::DeleteFail)
				{
					if (ev.cookie != delete_cookie_active)
					{
						continue;
					}
					delete_in_progress = false;
					delete_cookie_active = 0;
					delete_confirm = false;
					request_delete_redraw = true;
				}
#endif
			}
		}
		// Service pending waveform computation (from audio callback)
		if (waveform_compute_pending
			&& record_state != RecordState::Recording)
		{
			waveform_compute_pending = false;
			if (sample_loaded && !waveform_ready)
			{
				JobStartWaveform(waveform_compute_ctx, false);
			}
		}
		{
			uint32_t audio_events = 0;
			{
				daisy::ScopedIrqBlocker irq;
				audio_events = g_audio_event_bits;
				g_audio_event_bits = 0;
			}
			if (audio_events & kAudioEventRecFinished)
			{
				if (record_state == RecordState::Recording)
				{
					size_t recorded_length = 0;
					{
						daisy::ScopedIrqBlocker irq;
						recorded_length = g_recorded_length_audio;
					}
					sample_length = recorded_length;
					sample_channels = 1;
					sample_rate = 48000;
					sample_loaded = (sample_length > 0);
					RequestPlaybackStopAll();
					trim_start = 0.0f;
					trim_end = 1.0f;
					waveform_from_recording = true;
					UpdateTrimFrames();
					PublishRuntimeFromUi();
					record_state = RecordState::Review;
				}
				record_waveform_pending = true;
				if (sample_loaded)
				{
					request_length_redraw = true;
				}
			}
			if (audio_events & kAudioEventPlaybackStopped)
			{
				request_playback_stop_log = true;
			}
		}

		g_ui.Tick(System::GetNow());
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

		if (encoder_r_accum != 0)
		{
			encoder_r_accum = 0;
		}
		if (encoder_r_button_press)
		{
			encoder_r_button_press = false;
		}
		const bool ui_blocked = (sd_init_in_progress || save_in_progress);
		if (button2_press)
		{
			button2_press = false;
			if (!ui_blocked)
			{
				if (ui_mode != UiMode::Shift)
				{
					shift_prev_mode = ui_mode;
					ui_mode = UiMode::Shift;
					shift_menu_index = 0;
					request_shift_redraw = true;
				}
			}
		}
		{
			const StorageService::SdStatus& st = storage.GetSdStatus();
			sd_present = st.present;
			sd_mounted = st.mounted;
			sd_fault_code = st.last_error.code;
			sd_fault = (sd_fault_code != StorageService::SdErrorCode::None);
			sd_fault_text = sd_fault ? SdFaultText(sd_fault_code) : nullptr;
			const uint32_t max_retry = StorageService::kSdRetryMaxAttempts;
			sd_retries_remaining = (sd_fault && max_retry > st.last_error.retries)
				? (max_retry - st.last_error.retries)
				: 0;
		}
		if (button1_press)
		{
			button1_press = false;
			if (sd_fault && ui_mode == UiMode::Load)
			{
				storage.ClearSdError();
				MountSd();
				request_load_scan = true;
			}
			else
			{
			if (!ui_blocked
				&& sample_loaded
				&& ui_mode != UiMode::Load)
			{
				if (IsPerformUiMode(ui_mode))
				{
					StartPerformVoice(kBaseMidiNote);
				}
				else
				{
					RequestPlaybackStart(kBaseMidiNote, false);
				}
				request_playhead_redraw = true;
			}
			}
		}
		// Step advance is audio-clocked in AudioCallback.
		if (request_load_scan)
		{
			request_load_scan = false;
			load_scan_start_ms = System::GetNow();
			list_build_pending = true;
		}
		if (list_build_pending)
		{
			if (!ui_blocked)
			{
				if (kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode)
				{
					list_build_pending = false;
				}
				else
				{
					list_build_pending = false;
					JobStartFileList(storage.GetSdPath(), true, true);
				}
			}
		}
		if (request_delete_scan)
		{
			if (!ui_blocked)
			{
				request_delete_scan = false;
				JobStartFileList(storage.GetSdPath(), false, true);
			}
		}
		if (request_load_sample)
		{
			if (ui_blocked)
			{
				request_load_sample = false;
				request_load_index = -1;
			}
			else if (kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode)
			{
				request_load_sample = false;
				request_load_index = -1;
			}
			else
			{
				request_load_sample = false;
				const int32_t index = request_load_index;
				request_load_index = -1;
				if (load_context == LoadContext::Edt)
				{
					edt_sample_context = SampleContext::Perform;
				}
				load_target_is_edt = (load_context == LoadContext::Edt);
				if (!LoadSampleAtIndex(index))
				{
					ui_mode = UiMode::Load;
					load_context = LoadContext::Main;
					load_in_progress = false;
				}
			}
		}

		if (!ui_blocked)
		{
			uint8_t ui_idx = 0;
			const AudioUiState& uir = GetAudioUiStateSnapshot(ui_idx);
			const bool preview_allowed = (ui_mode == UiMode::Load
				&& (load_context == LoadContext::Edt || delete_mode)
				&& !(kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode)
				&& wav_file_count > 0);
			if (preview_hold && preview_allowed)
			{
				if (!uir.preview_active || preview_index != load_selected)
				{
					if (!BeginPreviewAtIndex(load_selected))
					{
						StopPreview();
					}
				}
			}
			else
			{
				if (uir.preview_active)
				{
					StopPreview();
				}
			}
		}
#if STORAGE_SERVICE_PREVIEW_STREAM
		if (preview_pending_start && !preview_preload_active)
		{
			const size_t ready_count
				= static_cast<size_t>(preview_pp_ready[0] != 0)
				+ static_cast<size_t>(preview_pp_ready[1] != 0);
			const uint32_t now = System::GetNow();
			const bool timeout = (now - preview_pending_start_ms) >= 150;
			if (ready_count >= 1 || timeout)
			{
				if (preview_pp_ready[0])
				{
					preview_pp_active = 0;
				}
				else if (preview_pp_ready[1])
				{
					preview_pp_active = 1;
				}
				PublishPreviewControlFromUi();
				RequestAudioCmd(kCmdPreviewStart);
				preview_pending_start = false;
			}
		}
#endif
		{
			uint8_t ui_idx = 0;
			const AudioUiState& uir = GetAudioUiStateSnapshot(ui_idx);
			if (uir.preview_active)
			{
				FillPreviewBuffer();
			}
		}
		if (request_delete_file)
		{
			if (ui_blocked)
			{
				request_delete_file = false;
				request_delete_index = -1;
			}
			else
			{
				request_delete_file = false;
				const int32_t index = request_delete_index;
				request_delete_index = -1;
				if (!DeleteFileAtIndex(index))
				{
					delete_confirm = false;
					request_delete_redraw = true;
				}
			}
		}

		if (sd_init_in_progress)
		{
			if (!sd_init_done)
			{
				const uint32_t now = System::GetNow();
				const StorageService::MountState ms = storage.GetMountState();
				if (ms == StorageService::MountState::Mounted)
				{
					sd_init_success = true;
					sd_init_done = true;
					sd_init_result_until_ms = now + kSdInitResultMs;
				}
				else if (sd_init_attempts < kSdInitAttempts
					&& now >= sd_init_next_ms
					&& ms != StorageService::MountState::Mounting)
				{
					MountSd();
					sd_init_attempts++;
					sd_init_next_ms = now + kSdInitRetryMs;
					if (sd_init_attempts >= kSdInitAttempts)
					{
						sd_init_success = false;
						sd_init_done = true;
						sd_init_result_until_ms = now + kSdInitResultMs;
					}
				}
			}
			if ((System::GetNow() - sd_init_start_ms) >= kSdInitMinMs && sd_init_done)
			{
				if (System::GetNow() >= sd_init_result_until_ms)
				{
					sd_init_in_progress = false;
					if (sd_init_success)
					{
						ui_mode = UiMode::Load;
						request_load_scan = true;
						last_mode = UiMode::Shift;
					}
					else
					{
						ui_mode = sd_init_prev_mode;
						last_mode = UiMode::Shift;
					}
					request_shift_redraw = true;
				}
			}
		}
		if (sd_init_in_progress)
		{
			const uint32_t now = System::GetNow();
			if (now >= sd_init_draw_next_ms)
			{
				DrawSdInitScreen();
				sd_init_draw_next_ms = now + 100;
			}
		}
		if (save_in_progress)
		{
			const uint32_t now = System::GetNow();
			if (!save_done)
			{
				if (!save_started)
				{
					DrawSaveScreen();
#if STORAGE_SERVICE_SAVE
					save_success = false;
					save_started = true;
					save_frames_written = 0;
					StorageService::Op op = {};
					op.kind = StorageService::OpKind::SaveStart;
					CopyString(op.path, storage.GetSdPath(), sizeof(op.path));
					op.src_l = sample_buffer_l;
					op.src_r = sample_buffer_r;
					op.frames = sample_length;
					op.channels = (sample_channels == 0) ? 1 : sample_channels;
					op.sample_rate = (sample_rate == 0) ? 48000 : sample_rate;
					if (storage.Enqueue(op))
					{
						save_success = true;
					}
					if (!save_success)
					{
						save_done = true;
						save_result_until_ms = now + kSaveResultMs;
					}
#else
					save_success = BeginSaveRecordedSample();
					save_started = true;
					if (!save_success)
					{
						save_done = true;
						save_result_until_ms = now + kSaveResultMs;
					}
#endif
				}
				else
				{
					bool step_done = false;
					DrawSaveScreen();
#if STORAGE_SERVICE_SAVE
					(void)step_done;
#else
					save_success = StepSaveRecordedSample(step_done);
					if (!save_success)
					{
						save_done = true;
						save_result_until_ms = now + kSaveResultMs;
					}
					else if (step_done)
					{
						save_done = true;
						save_result_until_ms = now + kSaveResultMs;
						request_load_scan = true;
					}
#endif
				}
			}
			if (now >= save_draw_next_ms)
			{
				DrawSaveScreen();
				save_draw_next_ms = now + 100;
			}
			if (save_done && now >= save_result_until_ms)
			{
				save_in_progress = false;
				ui_mode = save_prev_mode;
				last_mode = UiMode::Shift;
			}
		}

		if (record_waveform_pending
			&& record_state != RecordState::Recording)
		{
			record_waveform_pending = false;
			if (sample_loaded && sample_length > 0)
			{
				JobStartWaveform(current_sample_context, true);
				UpdateTrimFrames();
				if (ui_mode == UiMode::Record && record_state == RecordState::Review)
				{
					DrawRecordReview();
					last_record_state = record_state;
				}
			}
		}
		if (!ui_blocked && request_length_redraw)
		{
			request_length_redraw = false;
			if (ui_mode == UiMode::Record && record_state == RecordState::Review)
			{
				DrawRecordReview();
			}
			else if (ui_mode == UiMode::Edt)
			{
				DrawEdtScreen();
			}
		}
		if (!ui_blocked && request_shift_redraw && ui_mode == UiMode::Shift)
		{
			request_shift_redraw = false;
			DrawShiftMenu(shift_menu_index);
			last_shift_menu = shift_menu_index;
		}

		const UiMode mode = ui_mode;
		if (mode != last_mode)
		{
			if (last_mode == UiMode::Load && mode != UiMode::Load && g_job.type == JobType::FileListScan)
			{
				JobCancel();
			}
			if (mode == UiMode::FxDetail)
			{
				SetFxContext(FxContext::Perform);
			}
			else if (mode == UiMode::Edt)
			{
				SetFxContext(FxContext::Perform);
			}
			else if (mode == UiMode::Perform)
			{
				SetFxContext(FxContext::Perform);
			}
			if (IsPerformUiMode(last_mode) && !IsPerformUiMode(mode))
			{
				RequestAudioCmd(kCmdAllNotesOff);
			}
			if (mode == UiMode::Record)
			{
				record_anim_start_ms = NowMs();
			}
			else if (last_mode == UiMode::Record)
			{
				record_anim_start_ms = -1.0;
			}
			if (mode == UiMode::Perform)
			{
				midi_ignore_until_ms = System::GetNow() + 200;
			}
			if (sd_init_in_progress)
			{
				DrawSdInitScreen();
			}
			else if (save_in_progress)
			{
				DrawSaveScreen();
			}
			else if (mode == UiMode::Main)
			{
				DrawMenu(menu_index);
			}
			else if (mode == UiMode::Load)
			{
				if (delete_mode && delete_confirm)
				{
					DrawDeleteConfirm(delete_confirm_name);
				}
				else
				{
					DrawLoadMenu(load_scroll, load_selected);
				}
			}
			else if (mode == UiMode::LoadModeSelect)
			{
				DrawLoadModeSelect(load_mode_index);
			}
			else if (mode == UiMode::LoadStub)
			{
				DrawLoadStubScreen(load_stub_mode);
			}
			else if (mode == UiMode::Edt)
			{
				DrawEdtScreen();
			}
			else if (mode == UiMode::FxDetail)
			{
				DrawFxDetailScreen(fx_detail_index);
				last_fx_detail_index = fx_detail_index;
				last_fx_detail_param_index = fx_detail_param_index;
			}
			else if (mode == UiMode::Perform)
			{
				const bool fx_select_active = (perform_index == kPerformFxIndex)
					&& fx_window_active;
				const bool amp_select_active = (perform_index == kPerformAmpIndex)
					&& amp_window_active;
				const bool flt_select_active = (perform_index == kPerformFltIndex)
					&& flt_window_active;
				DrawPerformScreen(perform_index,
								  fx_select_active,
								  fx_fader_index,
								  amp_select_active,
								  amp_fader_index,
								  flt_select_active,
								  flt_fader_index);
				const float amp_vals[kPerformFaderCount]
					= {amp_attack, amp_decay, amp_sustain, amp_release};
				const float flt_vals[kPerformFltFaderCount] = {flt_cutoff, flt_res};
				for (int i = 0; i < kPerformFaderCount; ++i)
				{
					last_perform_amp_vals[i] = amp_vals[i];
					last_perform_fx_order[i] = fx_chain_order[i];
					last_perform_fx_vals[i] = FxWetValue(fx_chain_order[i]);
				}
				for (int i = 0; i < kPerformFltFaderCount; ++i)
				{
					last_perform_flt_vals[i] = flt_vals[i];
				}
				last_perform_fx_select_active = fx_select_active;
				last_perform_amp_select_active = amp_select_active;
				last_perform_flt_select_active = flt_select_active;
				last_perform_fx_selected = fx_fader_index;
				last_perform_amp_selected = amp_fader_index;
				last_perform_flt_selected = flt_fader_index;
				last_perform_index = perform_index;
				perform_redraw_next_ms = System::GetNow() + kPerformPlayheadIntervalMs;
			}
			else if (mode == UiMode::Shift)
			{
				DrawShiftMenu(shift_menu_index);
				last_shift_menu = shift_menu_index;
			}
			else if (mode == UiMode::PresetSaveStub)
			{
				DrawPresetSaveStub();
			}
			else
			{
				if (record_state == RecordState::BackConfirm)
				{
					DrawRecordBackConfirm();
				}
				else if (record_state == RecordState::SourceSelect)
				{
					DrawRecordSourceScreen();
				}
				else if (record_state == RecordState::Armed)
				{
					DrawRecordArmed();
				}
				else if (record_state == RecordState::Countdown)
				{
					DrawRecordCountdown();
				}
				else if (record_state == RecordState::Recording)
				{
					uint8_t ui_idx = 0;
					const AudioUiState& uir = GetAudioUiStateSnapshot(ui_idx);
					DrawRecordRecording(uir);
				}
				else
				{
					if (sample_loaded && sample_length > 0
						&& record_state != RecordState::Recording)
					{
						JobStartWaveform(current_sample_context, true);
						UpdateTrimFrames();
					}
					DrawRecordReview();
				}
			}
			last_mode = mode;
			last_menu = menu_index;
			last_scroll = load_scroll;
			last_selected = load_selected;
			last_file_count = wav_file_count;
			last_sd_mounted = sd_mounted;
			last_record_state = record_state;
			last_perform_index = perform_index;
		}
		else if (mode == UiMode::Main)
		{
			const int32_t current = menu_index;
			if (current != last_menu)
			{
				DrawMenu(current);
				last_menu = current;
			}
		}
		else if (mode == UiMode::Perform)
		{
			const int32_t current = perform_index;
			const bool fx_select_active = (current == kPerformFxIndex)
				&& fx_window_active;
			const bool amp_select_active = (current == kPerformAmpIndex)
				&& amp_window_active;
			const bool flt_select_active = (current == kPerformFltIndex)
				&& flt_window_active;

			bool amp_changed = false;
			const float amp_vals[kPerformFaderCount]
				= {amp_attack, amp_decay, amp_sustain, amp_release};
			for (int i = 0; i < kPerformFaderCount; ++i)
			{
				if (amp_vals[i] != last_perform_amp_vals[i])
				{
					amp_changed = true;
					break;
				}
			}
			bool flt_changed = false;
			const float flt_vals[kPerformFltFaderCount] = {flt_cutoff, flt_res};
			for (int i = 0; i < kPerformFltFaderCount; ++i)
			{
				if (flt_vals[i] != last_perform_flt_vals[i])
				{
					flt_changed = true;
					break;
				}
			}
			bool fx_changed = false;
			int32_t fx_order[kPerformFaderCount];
			float fx_vals[kPerformFaderCount];
			for (int i = 0; i < kPerformFaderCount; ++i)
			{
				fx_order[i] = fx_chain_order[i];
				fx_vals[i] = FxWetValue(fx_order[i]);
				if (fx_order[i] != last_perform_fx_order[i]
					|| fx_vals[i] != last_perform_fx_vals[i])
				{
					fx_changed = true;
				}
			}

			const bool selection_changed = (current != last_perform_index);
			const bool fx_select_changed
				= (fx_select_active != last_perform_fx_select_active)
				|| (fx_fader_index != last_perform_fx_selected);
			const bool amp_select_changed
				= (amp_select_active != last_perform_amp_select_active)
				|| (amp_fader_index != last_perform_amp_selected);
			const bool flt_select_changed
				= (flt_select_active != last_perform_flt_select_active)
				|| (flt_fader_index != last_perform_flt_selected);

			uint8_t redraw_mask = 0;
			if (selection_changed)
			{
				redraw_mask |= (1u << current);
				if (last_perform_index >= 0)
				{
					redraw_mask |= (1u << last_perform_index);
				}
			}
			if (amp_changed || amp_select_changed)
			{
				redraw_mask |= (1u << kPerformAmpIndex);
			}
			if (flt_changed || flt_select_changed)
			{
				redraw_mask |= (1u << kPerformFltIndex);
			}
			if (fx_changed || fx_select_changed)
			{
				redraw_mask |= (1u << kPerformFxIndex);
			}
			if (selection_changed && current == kPerformEdtIndex)
			{
				redraw_mask |= (1u << kPerformEdtIndex);
			}
			if (!selection_changed && current == kPerformEdtIndex
				&& last_perform_index == kPerformEdtIndex)
			{
				// Keep EDT highlight accurate if select state flips.
				redraw_mask |= (1u << kPerformEdtIndex);
			}

			if (request_perform_redraw && redraw_mask == 0)
			{
				redraw_mask = 0x0F;
			}

			if (redraw_mask != 0)
			{
				const uint32_t now = System::GetNow();
				if (now >= perform_redraw_next_ms)
				{
					DrawPerformScreen(current,
									  fx_select_active,
									  fx_fader_index,
									  amp_select_active,
									  amp_fader_index,
									  flt_select_active,
									  flt_fader_index,
									  redraw_mask);
					perform_redraw_next_ms = now + kPerformPlayheadIntervalMs;
					for (int i = 0; i < kPerformFaderCount; ++i)
					{
						last_perform_amp_vals[i] = amp_vals[i];
						last_perform_fx_order[i] = fx_order[i];
						last_perform_fx_vals[i] = fx_vals[i];
					}
					for (int i = 0; i < kPerformFltFaderCount; ++i)
					{
						last_perform_flt_vals[i] = flt_vals[i];
					}
					last_perform_fx_select_active = fx_select_active;
					last_perform_amp_select_active = amp_select_active;
					last_perform_flt_select_active = flt_select_active;
					last_perform_fx_selected = fx_fader_index;
					last_perform_amp_selected = amp_fader_index;
					last_perform_flt_selected = flt_fader_index;
					last_perform_index = current;
					request_perform_redraw = false;
				}
			}
			else
			{
				request_perform_redraw = false;
			}
		}
		else if (mode == UiMode::FxDetail)
		{
			if (fx_detail_index == kFxReverbIndex && playback_reverse >= 0.5f)
			{
				const uint32_t now = System::GetNow();
				if (now >= rev_anim_next_ms)
				{
					request_fx_detail_redraw = true;
					rev_anim_next_ms = now + 120;
				}
			}
			else
			{
				rev_anim_next_ms = 0;
			}
			if (request_fx_detail_redraw
				|| fx_detail_index != last_fx_detail_index
				|| fx_detail_param_index != last_fx_detail_param_index)
			{
				request_fx_detail_redraw = false;
				DrawFxDetailScreen(fx_detail_index);
				last_fx_detail_index = fx_detail_index;
				last_fx_detail_param_index = fx_detail_param_index;
			}
		}
		else if (mode == UiMode::Shift)
		{
			if (!ui_blocked)
			{
				const int32_t current = shift_menu_index;
				if (current != last_shift_menu)
				{
					DrawShiftMenu(current);
					last_shift_menu = current;
				}
			}
		}
		else if (mode == UiMode::PresetSaveStub)
		{
			DrawPresetSaveStub();
		}
		else if (mode == UiMode::Load)
		{
			if (delete_mode && delete_confirm)
			{
				if (request_delete_redraw)
				{
					request_delete_redraw = false;
					DrawDeleteConfirm(delete_confirm_name);
				}
			}
			else
			{
				const int32_t current_scroll = load_scroll;
				const int32_t current_count = wav_file_count;
				const int32_t current_selected = load_selected;
				if (request_delete_redraw
					|| current_scroll != last_scroll
					|| current_selected != last_selected
					|| current_count != last_file_count
					|| sd_mounted != last_sd_mounted)
				{
					request_delete_redraw = false;
					DrawLoadMenu(current_scroll, current_selected);
					if (current_selected != last_selected || current_count != last_file_count)
					{
					}
					last_scroll = current_scroll;
					last_selected = current_selected;
					last_file_count = current_count;
					last_sd_mounted = sd_mounted;
				}
			}
		}
		else if (mode == UiMode::LoadModeSelect)
		{
			DrawLoadModeSelect(load_mode_index);
		}
		else if (mode == UiMode::LoadStub)
		{
			DrawLoadStubScreen(load_stub_mode);
		}
		else if (mode == UiMode::Record)
		{
			const RecordState current_state = record_state;
			if (current_state != last_record_state)
			{
				if (current_state == RecordState::BackConfirm)
				{
					DrawRecordBackConfirm();
				}
				else if (current_state == RecordState::SourceSelect)
				{
					DrawRecordSourceScreen();
				}
				else if (current_state == RecordState::Armed)
				{
					record_anim_start_ms = NowMs();
					DrawRecordArmed();
				}
				else if (current_state == RecordState::Countdown)
				{
					DrawRecordCountdown();
				}
				else if (current_state == RecordState::Recording)
				{
					uint8_t ui_idx = 0;
					const AudioUiState& uir = GetAudioUiStateSnapshot(ui_idx);
					DrawRecordRecording(uir);
					record_draw_next_ms = 0;
				}
				else if (current_state == RecordState::TargetSelect)
				{
					DrawRecordTargetScreen(record_target_index);
				}
				else
				{
					DrawRecordReview();
				}
				last_record_state = current_state;
			}
		}
		if (!ui_blocked && mode == UiMode::Record && record_state == RecordState::Recording)
		{
			uint8_t ui_idx = 0;
			const AudioUiState& uir = GetAudioUiStateSnapshot(ui_idx);
			const uint32_t now = System::GetNow();
			if (record_draw_next_ms == 0 || now >= record_draw_next_ms)
			{
				DrawRecordRecording(uir);
				record_draw_next_ms = now + 33;
			}
		}
		else if (!ui_blocked && mode == UiMode::Record && record_state == RecordState::SourceSelect)
		{
			DrawRecordSourceScreen();
		}
		else if (!ui_blocked && mode == UiMode::Record && record_state == RecordState::Armed)
		{
			DrawRecordReadyScreen();
		}
		else if (!ui_blocked && mode == UiMode::Record && record_state == RecordState::Countdown)
		{
			DrawRecordCountdown();
		}
		else if (!ui_blocked && mode == UiMode::Record && record_state == RecordState::TargetSelect)
		{
			DrawRecordTargetScreen(record_target_index);
		}
		else if (!ui_blocked && mode == UiMode::Record && record_state == RecordState::BackConfirm)
		{
			DrawRecordBackConfirm();
		}
		uint8_t play_ui_idx = 0;
		const AudioUiState& play_uir = GetAudioUiStateSnapshot(play_ui_idx);
		const bool edt_playhead_active = (!ui_blocked
			&& mode == UiMode::Edt
			&& play_uir.playback_active);
		if (edt_playhead_active)
		{
			const uint32_t now = System::GetNow();
			if (!last_edt_playhead_active)
			{
				last_edt_playhead_ms = now;
				waveform_dirty = true;
				request_playhead_redraw = true;
			}
			else if ((now - last_edt_playhead_ms) >= kPerformPlayheadIntervalMs)
			{
				last_edt_playhead_ms = now;
				waveform_dirty = true;
				request_playhead_redraw = true;
			}
		}
		else
		{
			last_edt_playhead_ms = 0;
		}
		last_edt_playhead_active = edt_playhead_active;
		const bool playback_active_now = play_uir.playback_active;
		if (!ui_blocked && (request_playhead_redraw || (playback_active_now != last_playback_active)))
		{
			request_playhead_redraw = false;
			if (mode == UiMode::Edt
				|| (mode == UiMode::Record && record_state == RecordState::Review))
			{
				if (mode == UiMode::Edt)
				{
					DrawEdtScreen();
				}
				else
				{
					DrawRecordReview();
				}
			}
		}
		last_playback_active = playback_active_now;
		if (request_playback_stop_log)
		{
			request_playback_stop_log = false;
		}
		uint8_t led_ui_idx = 0;
		const AudioUiState& led_ui = GetAudioUiStateSnapshot(led_ui_idx);
		const bool audio_playing = (led_ui.preview_active
			|| led_ui.playback_active
			|| led_ui.perform_voices_active);
		const bool in_delete_menu = (ui_mode == UiMode::Load && delete_mode);
		const bool in_wav_editor_list = (ui_mode == UiMode::Load && load_context == LoadContext::Edt);
		const bool in_wav_editor_view = (ui_mode == UiMode::Edt);
		const bool previewable = (wav_file_count > 0);
		const bool solid_green = (in_delete_menu && previewable)
			|| (in_wav_editor_list && previewable)
			|| (in_wav_editor_view && sample_loaded)
			|| (IsPerformUiMode(ui_mode) && sample_loaded);
		if (audio_playing)
		{
			led1_phase_ms += 10.0f;
			if (led1_phase_ms >= kLedBlinkPeriodMs)
			{
				led1_phase_ms -= kLedBlinkPeriodMs;
			}
			const float on_time = kLedBlinkDuty * kLedBlinkPeriodMs;
			led1_level = (led1_phase_ms < on_time) ? 1.0f : 0.0f;
		}
		else
		{
			led1_phase_ms = 0.0f;
			led1_level = solid_green ? 1.0f : 0.0f;
		}
		hw.led1.Set(0.0f, led1_level, 0.0f);
		{
			uint8_t audio_ui_idx = 0;
			GetAudioUiStateSnapshot(audio_ui_idx);
			if (audio_ui_idx != last_audio_ui_idx)
			{
				last_audio_ui_idx = audio_ui_idx;
				RequestDisplayUpdate();
			}
		}
		const uint32_t draw_now = System::GetNow();
		const bool needs_draw = g_display_update_pending
			|| request_shift_redraw
			|| request_delete_redraw
			|| request_playhead_redraw
			|| waveform_dirty
  			|| request_perform_redraw
			|| request_fx_detail_redraw
			|| request_length_redraw;
		if (needs_draw)
		{
			FlushDisplayIfDue(draw_now);
		}
		sample_mem_used_bytes = sample_mem_mgr.BytesUsed();
		sample_mem_free_bytes = sample_mem_mgr.BytesFree();
		waveform_cache_bytes = perform_waveform_cache.BytesUsed();
 		hw.UpdateLeds();
		hw.DelayMs(1);
	}
}

 
 
