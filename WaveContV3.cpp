#include "daisy_pod.h"
#include "daisysp.h"
#include "dev/oled_ssd130x.h"
#include "fatfs.h"
#include "util/wav_format.h"
#include "util/bsp_sd_diskio.h"
#include <cmath>
#include <initializer_list>
#include <math.h>
#include <cstring>
#include <cstdio>

using namespace daisy;
using namespace daisysp;

using PodDisplay = OledDisplay<SSD130xI2c128x64Driver>;

constexpr bool kLogEnabled = true;
constexpr int32_t kMenuCount = 4;
constexpr int32_t kShiftMenuCount = 2;
constexpr int32_t kLoadTargetCount = 3;
constexpr int32_t kRecordTargetCount = 3;
constexpr int32_t kRecordTargetPerform = 0;
constexpr int32_t kRecordTargetPlay = 1;
constexpr int32_t kRecordTargetBake = 2;
constexpr int32_t kPerformBoxCount = 4;
constexpr int32_t kPerformEdtIndex = 0;
constexpr int32_t kPerformFaderCount = 4;
constexpr int32_t kFxReverbIndex = 3;
constexpr int32_t kFxChorusIndex = 1;
constexpr int32_t kReverbFaderCount = 5;
constexpr int32_t kPerformFltFaderCount = 2;
constexpr int32_t kPerformAmpIndex = 1;
constexpr int32_t kPerformFltIndex = 2;
constexpr int32_t kPerformFxIndex = 3;
constexpr uint32_t kSdInitMinMs = 800;
constexpr uint32_t kSdInitRetryMs = 300;
constexpr uint32_t kSdInitResultMs = 1500;
constexpr int32_t kSdInitAttempts = 3;
constexpr uint32_t kSaveResultMs = 1500;
constexpr uint32_t kSaveStepBudgetMs = 3;
constexpr int32_t kMaxWavFiles = 32;
constexpr size_t kMaxWavNameLen = 32;
constexpr int32_t kLoadFontScale = 1;
constexpr size_t kMaxSampleSamples = 2 * 1024 * 1024;
constexpr int32_t kRecordMaxSeconds = 5;
constexpr size_t kSampleChunkFrames = 256;
constexpr size_t kSaveChunkFrames = 2048;
constexpr int32_t kBaseMidiNote = 60;
constexpr float kSampleScale = 1.0f / 32768.0f;
constexpr int32_t kLoadProgressStep = 5;
constexpr float kLedBlinkPeriodMs = 25.0f;
constexpr float kLedBlinkDuty = 0.5f;
constexpr bool kPlaybackVerboseLog = false;
constexpr float kPi = 3.14159265f;
constexpr float kTwoPi = 6.2831853f;
constexpr int kDisplayW = 128;
constexpr int kDisplayH = 64;
constexpr int kPlayBpm = 120;
constexpr int kPlayStepCount = 16;
constexpr uint32_t kPlayStepMs = 60000U / (kPlayBpm * 4);
constexpr uint32_t kPreviewReadBudgetMs = 2;
constexpr size_t kPreviewBufferFrames = 4096;
constexpr size_t kPreviewReadFrames = 256;
constexpr size_t kPvLongSize = 1024;
constexpr size_t kPvShortSize = 256;
constexpr size_t kPvLongHop = kPvLongSize / 4;
constexpr size_t kPvShortHop = kPvShortSize / 4;
constexpr float kPvTransientThresh = 1.6f;
constexpr float kPvTransientFloor = 1e-4f;
constexpr float kPvTransientMix = 0.6f;
constexpr int kPvPhaseLockRadius = 8;
constexpr size_t kPvMaxFrames = (kMaxSampleSamples / kPvLongHop) + 4;
constexpr float kPvOutputGain = 0.9f;
constexpr size_t kPvStretchBufSize = kMaxSampleSamples * 2 + kPvLongSize;
constexpr int kBakeFirstMidi = kBaseMidiNote - 12;
constexpr int kBakeLastMidi = kBaseMidiNote + 12;
constexpr int kBakeNoteCount = kBakeLastMidi - kBakeFirstMidi + 1;
constexpr size_t kBakeBankFramesMax = kMaxSampleSamples * 4;
constexpr int kPerformVoiceCount = 5;
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
constexpr float kReverbShimmerStep = 0.02f;
constexpr float kShimmerHpHz = 900.0f;
constexpr size_t kShimmerBufferSize = 8192;
constexpr size_t kShimmerDelaySamples = 2048;
constexpr float kShimmerRateUp = 2.0f;
constexpr float kShimmerFeedback = 0.6f;
constexpr float kChorusRateHz = 0.25f;
constexpr float kChorusRateMinHz = 0.05f;
constexpr float kChorusRateMaxHz = 0.9f;
constexpr float kChorusRateStep = 0.02f;
constexpr float kChorusDelayMs = 9.0f;
constexpr float kChorusFeedback = 0.18f;
constexpr float kChorusMaxDepth = 3.0f;
constexpr float kChorusWidthMax = 2.2f;
constexpr size_t kDelayMaxSamples = 48000;
constexpr float kDelayTimeSec = 0.3f;
constexpr float kDelayFeedback = 0.55f;
constexpr float kDelayDefaultWet = 0.0f;
constexpr float kDelayWetStep = 0.02f;
constexpr float kFxParamEpsilon = 1e-5f;
constexpr float kAmpEnvStep = 0.02f;
constexpr float kFltParamStep = 0.02f;
constexpr float kAmpEnvMinMs = 5.0f;
constexpr float kAmpEnvMaxMs = 1000.0f;
constexpr float kAmpEnvStepMs = 20.0f;

enum class UiMode : int32_t
{
	Main,
	Load,
	LoadTarget,
	Perform,
	Edt,
	FxDetail,
	Play,
	Record,
	ConfirmBake,
	Shift,
};

enum class LoadDestination : int32_t
{
	Play = 0,
	Bake = 1,
	Perform = 2,
};

enum class RecordState : int32_t
{
	SourceSelect,
	Armed,
	Countdown,
	Recording,
	Review,
	TargetSelect,
};

enum class RecordInput : int32_t
{
	LineIn,
	Mic,
};

class TapeSaturator
{
public:
	void Init(float sample_rate)
	{
		sample_rate_ = sample_rate;
		dc_pre_.Init(sample_rate, 20.0f);
		dc_post_.Init(sample_rate, 20.0f);
		low_bump_lp_.Init(sample_rate, 90.0f);
		high_emph_lp_.Init(sample_rate, 2000.0f);
		post_lp_.Init(sample_rate, 16000.0f);
		release_coeff_ = expf(-1.0f / (0.08f * sample_rate));
		smooth_coeff_ = 1.0f - expf(-1.0f / (0.03f * sample_rate));
		drive_target_ = 0.0f;
		bias_target_ = 0.0f;
		tone_target_ = 0.5f;
		mix_target_ = 0.0f;
		output_gain_target_ = 1.0f;
		post_gain_target_ = 1.0f;
		drive_ = 0.0f;
		bias_ = 0.0f;
		tone_ = 0.5f;
		mix_ = 0.0f;
		output_gain_ = 1.0f;
		post_gain_ = 1.0f;
		env_ = 0.0f;
		prev_y_ = 0.0f;
		UpdatePostCutoff(drive_);
	}

	float Process(float x)
	{
		// Smooth parameters for real-time-safe changes.
		drive_ += smooth_coeff_ * (drive_target_ - drive_);
		bias_ += smooth_coeff_ * (bias_target_ - bias_);
		tone_ += smooth_coeff_ * (tone_target_ - tone_);
		mix_ += smooth_coeff_ * (mix_target_ - mix_);
		output_gain_ += smooth_coeff_ * (output_gain_target_ - output_gain_);
		post_gain_ += smooth_coeff_ * (post_gain_target_ - post_gain_);

		const float dry = x;

		// 1) DC blocker.
		x = dc_pre_.Process(x);

		// 2) Pre-emphasis: low bump + gentle high lift.
		const float pre_emph_boost = 1.0f + drive_;
		const float low_amt = (0.06f + 0.08f * (1.0f - tone_)) * pre_emph_boost;
		const float high_amt = (0.02f + 0.08f * tone_) * pre_emph_boost;
		const float low = low_bump_lp_.Process(x);
		const float high = x - high_emph_lp_.Process(x);
		x = x + (low * low_amt) + (high * high_amt);

		// 3) Envelope follower for mild tape compression.
		const float abs_x = fabsf(x);
		if (abs_x > env_)
		{
			env_ = abs_x;
		}
		else
		{
			env_ *= release_coeff_;
		}
		const float comp_amt = 0.35f * drive_;
		const float eff_drive = drive_ / (1.0f + comp_amt * env_);
		const float drive_boost = (1.0f + drive_) * (1.0f + drive_);

		// 4) Nonlinearity + bias + tape memory.
		const float bias_amt = bias_ * 0.12f;
		const float xb = x * (1.0f + eff_drive * 2.5f * drive_boost) + bias_amt;
		float y = FastTanh(xb);
		const float mem_amt = 0.05f + 0.1f * drive_;
		y += mem_amt * prev_y_;
		prev_y_ = y;
		y = dc_post_.Process(y);

		// 5) Post-shape HF loss.
		y = post_lp_.Process(y);
		y *= post_gain_;

		// 6) Wet/dry mix and 7) output trim.
		float out = (mix_ * y) + ((1.0f - mix_) * dry);
		out *= output_gain_;
		return out;
	}

	void SetDrive(float d01)
	{
		drive_target_ = Clamp(d01, 0.0f, 1.0f);
		UpdatePostCutoff(drive_target_);
		post_gain_target_ = DbToGain(-6.0f * drive_target_);
	}

	void SetBias(float b11)
	{
		bias_target_ = Clamp(b11, -1.0f, 1.0f);
	}

	void SetTone(float t01)
	{
		tone_target_ = Clamp(t01, 0.0f, 1.0f);
	}

	void SetMix(float m01)
	{
		mix_target_ = Clamp(m01, 0.0f, 1.0f);
	}

	void SetOutput(float o01)
	{
		const float out = Clamp(o01, 0.0f, 1.0f);
		const float gain_db = -12.0f + (18.0f * out);
		output_gain_target_ = powf(10.0f, gain_db / 20.0f);
	}

private:
	struct DcBlocker
	{
		float x1 = 0.0f;
		float y1 = 0.0f;
		float r = 0.0f;

		void Init(float sample_rate, float cutoff_hz)
		{
			r = expf(-2.0f * kPi * cutoff_hz / sample_rate);
			x1 = 0.0f;
			y1 = 0.0f;
		}

		float Process(float x)
		{
			const float y = x - x1 + (r * y1);
			x1 = x;
			y1 = y;
			return y;
		}
	};

	struct OnePoleLp
	{
		float a = 0.0f;
		float y = 0.0f;

		void Init(float sample_rate, float cutoff_hz)
		{
			SetFreq(sample_rate, cutoff_hz);
			y = 0.0f;
		}

		void SetFreq(float sample_rate, float cutoff_hz)
		{
			a = expf(-2.0f * kPi * cutoff_hz / sample_rate);
		}

		float Process(float x)
		{
			y = (1.0f - a) * x + (a * y);
			return y;
		}
	};

	static float Clamp(float v, float lo, float hi)
	{
		if (v < lo)
		{
			return lo;
		}
		if (v > hi)
		{
			return hi;
		}
		return v;
	}

	static float FastTanh(float x)
	{
		if (x > 3.0f)
		{
			x = 3.0f;
		}
		else if (x < -3.0f)
		{
			x = -3.0f;
		}
		const float x2 = x * x;
		return x * (27.0f + x2) / (27.0f + 9.0f * x2);
	}

	static float DbToGain(float db)
	{
		return powf(10.0f, db / 20.0f);
	}

	void UpdatePostCutoff(float drive)
	{
		float cutoff = 16000.0f - (drive * 8000.0f);
		if (cutoff < 8000.0f)
		{
			cutoff = 8000.0f;
		}
		post_lp_.SetFreq(sample_rate_, cutoff);
	}

	float sample_rate_ = 48000.0f;
	float drive_target_ = 0.0f;
	float bias_target_ = 0.0f;
	float tone_target_ = 0.5f;
	float mix_target_ = 0.0f;
	float output_gain_target_ = 1.0f;
	float post_gain_target_ = 1.0f;

	float drive_ = 0.0f;
	float bias_ = 0.0f;
	float tone_ = 0.5f;
	float mix_ = 0.0f;
	float output_gain_ = 1.0f;
	float post_gain_ = 1.0f;

	float env_ = 0.0f;
	float prev_y_ = 0.0f;
	float release_coeff_ = 0.0f;
	float smooth_coeff_ = 0.0f;

	DcBlocker dc_pre_;
	DcBlocker dc_post_;
	OnePoleLp low_bump_lp_;
	OnePoleLp high_emph_lp_;
	OnePoleLp post_lp_;
};

class BiquadLp
{
public:
	void Reset()
	{
		z1_ = 0.0f;
		z2_ = 0.0f;
	}

	void Set(float sample_rate, float freq, float q)
	{
		if (freq < 20.0f)
		{
			freq = 20.0f;
		}
		const float nyq = sample_rate * 0.49f;
		if (freq > nyq)
		{
			freq = nyq;
		}
		if (q < 0.001f)
		{
			q = 0.001f;
		}

		const float w0 = (2.0f * kPi * freq) / sample_rate;
		const float cos_w0 = cosf(w0);
		const float sin_w0 = sinf(w0);
		const float alpha = sin_w0 / (2.0f * q);

		const float b0 = (1.0f - cos_w0) * 0.5f;
		const float b1 = 1.0f - cos_w0;
		const float b2 = (1.0f - cos_w0) * 0.5f;
		const float a0 = 1.0f + alpha;
		const float a1 = -2.0f * cos_w0;
		const float a2 = 1.0f - alpha;

		a0_ = b0 / a0;
		a1_ = b1 / a0;
		a2_ = b2 / a0;
		b1_ = a1 / a0;
		b2_ = a2 / a0;
	}

	float Process(float x)
	{
		const float y = (a0_ * x) + z1_;
		z1_ = (a1_ * x) + z2_ - (b1_ * y);
		z2_ = (a2_ * x) - (b2_ * y);
		return y;
	}

private:
	float a0_ = 0.0f;
	float a1_ = 0.0f;
	float a2_ = 0.0f;
	float b1_ = 0.0f;
	float b2_ = 0.0f;
	float z1_ = 0.0f;
	float z2_ = 0.0f;
};

class OnePoleHp
{
public:
	void Init(float sample_rate, float cutoff_hz)
	{
		SetFreq(sample_rate, cutoff_hz);
		y_ = 0.0f;
		x1_ = 0.0f;
	}

	void SetFreq(float sample_rate, float cutoff_hz)
	{
		a_ = expf(-2.0f * kPi * cutoff_hz / sample_rate);
	}

	float Process(float x)
	{
		const float y = a_ * (y_ + x - x1_);
		x1_ = x;
		y_ = y;
		return y;
	}

	void Reset()
	{
		y_ = 0.0f;
		x1_ = 0.0f;
	}

private:
	float a_ = 0.0f;
	float y_ = 0.0f;
	float x1_ = 0.0f;
};

DaisyPod    hw;
PodDisplay  display;
SdmmcHandler   sdcard;
FatFSInterface fsi;
Encoder      encoder_r;
Switch       shift_button;
ReverbSc DSY_SDRAM_BSS reverb;
OnePoleHp shimmer_hp_l;
OnePoleHp shimmer_hp_r;
DSY_SDRAM_BSS float shimmer_buf_l[kShimmerBufferSize];
DSY_SDRAM_BSS float shimmer_buf_r[kShimmerBufferSize];
static size_t shimmer_write_idx = 0;
static float shimmer_read_idx = 0.0f;
static int shimmer_mode = 0;
DelayLine<float, kDelayMaxSamples> DSY_SDRAM_BSS delay_line_l;
DelayLine<float, kDelayMaxSamples> DSY_SDRAM_BSS delay_line_r;
DelayLine<float, kReverbPreDelayMaxSamples> DSY_SDRAM_BSS reverb_predelay_l;
DelayLine<float, kReverbPreDelayMaxSamples> DSY_SDRAM_BSS reverb_predelay_r;
ChorusEngine chorus_l;
ChorusEngine chorus_r;
TapeSaturator sat_l;
TapeSaturator sat_r;
BiquadLp perform_lpf_l1[kPerformVoiceCount];
BiquadLp perform_lpf_l2[kPerformVoiceCount];
BiquadLp perform_lpf_r1[kPerformVoiceCount];
BiquadLp perform_lpf_r2[kPerformVoiceCount];

volatile UiMode ui_mode = UiMode::Main;
volatile int32_t menu_index = 0;
volatile int32_t shift_menu_index = 0;
volatile int32_t perform_index = 0;
volatile int32_t amp_fader_index = 0;
volatile int32_t flt_fader_index = 0;
volatile int32_t fx_fader_index = 0;
volatile UiMode shift_prev_mode = UiMode::Main;
volatile RecordInput record_input = RecordInput::LineIn;
volatile int32_t load_selected = 0;
volatile int32_t load_scroll = 0;
volatile bool request_load_scan = false;
volatile bool request_load_sample = false;
volatile int32_t request_load_index = -1;
volatile LoadDestination request_load_destination = LoadDestination::Play;
volatile int32_t wav_file_count = 0;
volatile int32_t load_target_index = -1;
volatile LoadDestination load_target_selected = LoadDestination::Play;

bool sd_mounted = false;
static bool sd_detected_last = true;
static bool sd_need_reinit = false;
static bool sd_init_in_progress = false;
static bool sd_init_done = false;
static bool sd_init_success = false;
static uint32_t sd_init_start_ms = 0;
static uint32_t sd_init_next_ms = 0;
static uint32_t sd_init_result_until_ms = 0;
static uint32_t sd_init_draw_next_ms = 0;
static int32_t sd_init_attempts = 0;
static UiMode sd_init_prev_mode = UiMode::Main;
static bool save_in_progress = false;
static bool save_done = false;
static bool save_success = false;
static bool save_started = false;
static uint32_t save_start_ms = 0;
static uint32_t save_result_until_ms = 0;
static uint32_t save_draw_next_ms = 0;
static UiMode save_prev_mode = UiMode::Main;
static char save_filename[kMaxWavNameLen] = {0};
static FIL save_file;
static size_t save_frames_written = 0;
static bool save_file_open = false;
static bool save_header_written = false;
static uint16_t save_channels = 1;
static uint32_t save_sr = 48000;
static uint32_t save_data_bytes = 0;
static FRESULT save_last_error = FR_OK;
static volatile bool delete_mode = false;
static UiMode delete_prev_mode = UiMode::Main;
static volatile bool request_delete_scan = false;
static volatile bool request_delete_file = false;
static volatile int32_t request_delete_index = -1;
static volatile bool delete_confirm = false;
static bool request_delete_redraw = false;
static char delete_confirm_name[kMaxWavNameLen] = {0};
static UiMode confirm_bake_prev_mode = UiMode::Record;
static volatile int32_t confirm_bake_selected = 0;
static volatile bool request_confirm_bake_redraw = false;
static volatile bool request_bake_process = false;
static bool bake_in_progress = false;
char wav_files[kMaxWavFiles][kMaxWavNameLen];
char loaded_sample_name[kMaxWavNameLen] = {0};
int32_t load_lines = 1;
int32_t load_line_height = 1;
int32_t load_chars_per_line = 1;

DSY_SDRAM_BSS int16_t sample_buffer_l[kMaxSampleSamples];
DSY_SDRAM_BSS int16_t sample_buffer_r[kMaxSampleSamples];
DSY_SDRAM_BSS int16_t baked_buffer_l[kBakeBankFramesMax];
DSY_SDRAM_BSS int16_t baked_buffer_r[kBakeBankFramesMax];
DSY_SDRAM_BSS float baked_stretch_buf[kPvStretchBufSize];
volatile size_t sample_length = 0;
volatile size_t sample_play_start = 0;
volatile size_t sample_play_end = 0;
volatile uint32_t sample_rate = 48000;
volatile uint16_t sample_channels = 1;
volatile bool sample_loaded = false;
volatile bool playback_active = false;
volatile float playback_rate = 1.0f;
volatile float playback_phase = 0.0f;
volatile float playback_amp = 0.0f;
volatile uint32_t playback_env_samples = 0;
volatile bool playback_release_active = false;
volatile float playback_release_pos = 0.0f;
volatile float playback_release_start = 0.0f;
volatile int32_t current_note = -1;
volatile size_t baked_length = 0;
volatile bool baked_ready = false;
static size_t baked_note_offsets[kBakeNoteCount];
static bool baked_note_ready[kBakeNoteCount];

struct PerformVoice
{
	bool active = false;
	bool releasing = false;
	float phase = 0.0f;
	float rate = 1.0f;
	float amp = 1.0f;
	float env = 0.0f;
	float release_start = 0.0f;
	float release_pos = 0.0f;
	int32_t note = -1;
	size_t offset = 0;
	size_t length = 0;
	uint32_t env_samples = 0;
};

static PerformVoice perform_voices[kPerformVoiceCount];

// Normalized trim window (0..1 over entire sample)
float trim_start = 0.0f;
float trim_end = 1.0f;

// Derived frame window (engine space)
uint32_t snap_start_frame = 0;
uint32_t snap_end_frame = 0;

// Waveform preview buffers (128 columns)
static int16_t waveform_min[128];
static int16_t waveform_max[128];
static bool waveform_ready = false;
static bool waveform_dirty = false;
static bool waveform_from_recording = false;
static volatile bool perform_bake_active = false;
static size_t baked_play_start = 0;
static size_t baked_play_end = 0;
static bool baked_window_valid = false;
static volatile float perform_attack_norm = 0.0f;
static volatile float perform_release_norm = 0.0f;
static const char* waveform_title = nullptr;
static int16_t live_wave_min[128];
static int16_t live_wave_max[128];
static int16_t live_wave_peak = 1;
static bool live_wave_dirty = false;
static int32_t live_wave_last_col = -1;

constexpr size_t kRecordMaxFrames = static_cast<size_t>(kRecordMaxSeconds) * 48000U;
constexpr uint32_t kRecordCountdownMs = 4000;
volatile RecordState record_state = RecordState::Armed;
volatile int32_t record_source_index = 0;
volatile int32_t record_target_index = kRecordTargetPlay;
volatile uint32_t record_countdown_start_ms = 0;
volatile size_t record_pos = 0;
volatile bool record_waveform_pending = false;
volatile int32_t encoder_r_accum = 0;
volatile bool encoder_r_button_press = false;
volatile bool request_length_redraw = false;
static bool play_screen_dirty = true;
static bool playhead_running = false;
static int32_t playhead_step = 0;
static uint32_t playhead_last_step_ms = 0;
volatile bool request_playhead_redraw = false;
volatile bool button1_press = false;
volatile bool button2_press = false;
volatile bool request_playback_stop_log = false;
volatile float reverb_wet = kReverbDefaultWet;
volatile float reverb_pre = 0.0f;
volatile float reverb_damp = kReverbDampDefault;
volatile float reverb_decay = kReverbDecayDefault;
volatile float reverb_shimmer = 0.0f;
volatile float delay_wet = kDelayDefaultWet;
volatile float fx_s_wet = 0.0f;
volatile float fx_c_wet = 0.0f;
volatile float chorus_rate = 0.0f;
volatile int32_t chorus_mode = 0;
volatile float chorus_wow = 0.0f;
volatile float tape_rate = 0.0f;
volatile bool fx_params_dirty = true;
static bool reverb_params_initialized = false;
static bool mod_params_initialized = false;
volatile float amp_attack = 0.0f;
volatile float amp_decay = 0.0f;
volatile float amp_sustain = 0.0f;
volatile float amp_release = 0.0f;
volatile int32_t fx_detail_index = 0;
volatile int32_t fx_detail_param_index = 0;
volatile float flt_cutoff = 1.0f;
volatile float flt_res = 0.02f;
volatile bool preview_hold = false;
volatile bool preview_active = false;
volatile int32_t preview_index = -1;
volatile uint32_t preview_sample_rate = 48000;
volatile uint16_t preview_channels = 1;
volatile float preview_rate = 1.0f;
volatile float preview_read_frac = 0.0f;
volatile size_t preview_read_index = 0;
volatile size_t preview_write_index = 0;
volatile uint32_t preview_data_offset = 0;
static FIL preview_file;
static bool preview_file_open = false;
alignas(32) static int16_t preview_buffer[kPreviewBufferFrames];
alignas(32) static int16_t preview_read_buf[kPreviewReadFrames * 2];
float led1_level = 0.0f;
float led1_phase_ms = 0.0f;
constexpr bool kUiLogsEnabled = false;
static double record_anim_start_ms = -1.0;
static uint8_t record_text_mask[kDisplayH][kDisplayW];
static uint8_t record_invert_mask[kDisplayH][kDisplayW];
static uint8_t record_fb_buf[kDisplayH][kDisplayW];
static uint8_t record_bold_mask[kDisplayH][kDisplayW];
static bool request_shift_redraw = false;
static bool request_perform_redraw = false;
static bool request_fx_detail_redraw = false;
static bool fx_shift_active_prev = false;
static bool amp_shift_active_prev = false;
static bool flt_shift_active_prev = false;
static float pv_window_long[kPvLongSize];
static float pv_window_short[kPvShortSize];
static float pv_fft_re[kPvLongSize];
static float pv_fft_im[kPvLongSize];
static float pv_mag[kPvLongSize / 2 + 1];
static float pv_phase[kPvLongSize / 2 + 1];
static float pv_prev_phase[kPvLongSize / 2 + 1];
static float pv_sum_phase[kPvLongSize / 2 + 1];
static float pv_omega[kPvLongSize / 2 + 1];
static bool pv_window_ready = false;
static bool pv_transient_flags[kPvMaxFrames];

static inline bool UiLogEnabled()
{
	if (!kUiLogsEnabled)
	{
		return false;
	}
	if (ui_mode == UiMode::Record && record_state == RecordState::Review)
	{
		return false;
	}
	return true;
}

static double NowMs()
{
	return static_cast<double>(System::GetNow());
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

static FIL wav_file;
alignas(32) static int16_t wav_read[kSampleChunkFrames * 2];

const char* kMenuLabels[kMenuCount] = {"LOAD", "RECORD", "PERFORM", "PLAY"};

static int32_t NextMenuIndex(int32_t current, int32_t delta)
{
	static const int32_t order[kMenuCount] = {0, 1, 3, 2};
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
const char* kShiftMenuLabels[kShiftMenuCount] = {"SAVE", "DELETE"};

template <typename... Va>
static void LogLine(const char* format, Va... va)
{
	if (kLogEnabled)
	{
		DaisySeed::PrintLine(format, va...);
	}
	else
	{
		(void)format;
		(void)sizeof...(va);
	}
}

static const char* FresultName(FRESULT res)
{
	switch (res)
	{
		case FR_OK: return "FR_OK";
		case FR_DISK_ERR: return "FR_DISK_ERR";
		case FR_INT_ERR: return "FR_INT_ERR";
		case FR_NOT_READY: return "FR_NOT_READY";
		case FR_NO_FILE: return "FR_NO_FILE";
		case FR_NO_PATH: return "FR_NO_PATH";
		case FR_INVALID_NAME: return "FR_INVALID_NAME";
		case FR_DENIED: return "FR_DENIED";
		case FR_EXIST: return "FR_EXIST";
		case FR_INVALID_OBJECT: return "FR_INVALID_OBJECT";
		case FR_WRITE_PROTECTED: return "FR_WRITE_PROTECTED";
		case FR_INVALID_DRIVE: return "FR_INVALID_DRIVE";
		case FR_NOT_ENABLED: return "FR_NOT_ENABLED";
		case FR_NO_FILESYSTEM: return "FR_NO_FILESYSTEM";
		case FR_MKFS_ABORTED: return "FR_MKFS_ABORTED";
		case FR_TIMEOUT: return "FR_TIMEOUT";
		case FR_LOCKED: return "FR_LOCKED";
		case FR_NOT_ENOUGH_CORE: return "FR_NOT_ENOUGH_CORE";
		case FR_TOO_MANY_OPEN_FILES: return "FR_TOO_MANY_OPEN_FILES";
		case FR_INVALID_PARAMETER: return "FR_INVALID_PARAMETER";
		default: return "FR_UNKNOWN";
	}
}

static const char* UiModeName(UiMode mode)
{
	switch (mode)
	{
		case UiMode::Main: return "MAIN";
		case UiMode::Load: return "LOAD";
		case UiMode::LoadTarget: return "LOAD_TARGET";
		case UiMode::Perform: return "PERFORM";
		case UiMode::Edt: return "EDT";
		case UiMode::FxDetail: return "FX_DETAIL";
		case UiMode::Play: return "PLAY";
		case UiMode::Record: return "RECORD";
		case UiMode::ConfirmBake: return "CONFIRM_BAKE";
		case UiMode::Shift: return "SHIFT";
		default: return "UNKNOWN";
	}
}

static const char* LoadDestinationName(LoadDestination dest)
{
	switch (dest)
	{
		case LoadDestination::Play: return "PLAY";
		case LoadDestination::Bake: return "BAKE";
		case LoadDestination::Perform: return "PERFORM";
		default: return "UNKNOWN";
	}
}

static const char* MenuLabelForIndex(int32_t index)
{
	if (index < 0 || index >= kMenuCount)
	{
		return "UNKNOWN";
	}
	return kMenuLabels[index];
}

static const char* RegionForAddress(uintptr_t addr)
{
	if (addr >= 0x20000000 && addr < 0x20020000)
	{
		return "DTCM";
	}
	if (addr >= 0x24000000 && addr < 0x24080000)
	{
		return "SRAM";
	}
	if (addr >= 0x30000000 && addr < 0x30048000)
	{
		return "RAM_D2";
	}
	if (addr >= 0x38000000 && addr < 0x38010000)
	{
		return "RAM_D3";
	}
	if (addr >= 0xC0000000 && addr < 0xC4000000)
	{
		return "SDRAM";
	}
	return "OTHER";
}

static void LogSdCardStatus()
{
	const uint8_t detected = BSP_SD_IsDetected();
	const uint8_t state = BSP_SD_GetCardState();
	LogLine("SD status: detected=%u state=%u", static_cast<unsigned>(detected), static_cast<unsigned>(state));
	BSP_SD_CardInfo info;
	BSP_SD_GetCardInfo(&info);
	LogLine("SD info: block_size=%lu blocks=%lu speed=%lu",
			static_cast<unsigned long>(info.BlockSize),
			static_cast<unsigned long>(info.BlockNbr),
			static_cast<unsigned long>(info.CardSpeed));
}

struct WavInfo
{
	uint16_t num_channels = 0;
	uint32_t sample_rate = 0;
	uint16_t bits_per_sample = 0;
	uint32_t data_offset = 0;
	uint32_t data_size = 0;
};

alignas(32) static uint8_t wav_riff_hdr[12];
alignas(32) static uint8_t wav_chunk_hdr[8];
alignas(32) static uint8_t wav_fmt_buf[32];
alignas(32) static int16_t wav_write[kSaveChunkFrames * 2];

static bool ParseWavHeader(FIL* file, WavInfo& info)
{
	if (file == nullptr)
	{
		return false;
	}

	FRESULT fres;
	UINT bytes_read = 0;
	const uintptr_t riff_addr = reinterpret_cast<uintptr_t>(wav_riff_hdr);
	const uintptr_t chunk_addr = reinterpret_cast<uintptr_t>(wav_chunk_hdr);
	const uintptr_t fmt_addr = reinterpret_cast<uintptr_t>(wav_fmt_buf);
	const uintptr_t file_addr = reinterpret_cast<uintptr_t>(file);
	LogLine("WAV parse buffers: riff=0x%08lx (%s) chunk=0x%08lx (%s) fmt=0x%08lx (%s)",
			(unsigned long)riff_addr,
			RegionForAddress(riff_addr),
			(unsigned long)chunk_addr,
			RegionForAddress(chunk_addr),
			(unsigned long)fmt_addr,
			RegionForAddress(fmt_addr));
	LogLine("WAV parse file: 0x%08lx (%s)", (unsigned long)file_addr, RegionForAddress(file_addr));

	fres = f_lseek(file, 0);
	if (fres != FR_OK)
	{
		LogLine("Load failed: f_lseek error %s (%d)", FresultName(fres), (int)fres);
		return false;
	}

	fres = f_read(file, wav_riff_hdr, sizeof(wav_riff_hdr), &bytes_read);
	if (fres != FR_OK || bytes_read != sizeof(wav_riff_hdr))
	{
		LogLine("Load failed: RIFF header read %s (%d), bytes=%u",
				FresultName(fres),
				(int)fres,
				(unsigned)bytes_read);
		return false;
	}

	if (std::memcmp(wav_riff_hdr, "RIFF", 4) != 0
		|| std::memcmp(wav_riff_hdr + 8, "WAVE", 4) != 0)
	{
		LogLine("Load failed: not a RIFF/WAVE file");
		return false;
	}

	bool fmt_found = false;
	bool data_found = false;
	info = WavInfo();

	while (!data_found)
	{
		fres = f_read(file, wav_chunk_hdr, sizeof(wav_chunk_hdr), &bytes_read);
		if (fres != FR_OK || bytes_read != sizeof(wav_chunk_hdr))
		{
			LogLine("Load failed: chunk header read %s (%d), bytes=%u",
					FresultName(fres),
					(int)fres,
					(unsigned)bytes_read);
			return false;
		}

		const uint32_t chunk_size =
			(uint32_t)wav_chunk_hdr[4]
			| ((uint32_t)wav_chunk_hdr[5] << 8)
			| ((uint32_t)wav_chunk_hdr[6] << 16)
			| ((uint32_t)wav_chunk_hdr[7] << 24);

		if (std::memcmp(wav_chunk_hdr, "fmt ", 4) == 0)
		{
			const UINT to_read = (UINT)((chunk_size < sizeof(wav_fmt_buf)) ? chunk_size : sizeof(wav_fmt_buf));

			fres = f_read(file, wav_fmt_buf, to_read, &bytes_read);
			if (fres != FR_OK || bytes_read < 16)
			{
				LogLine("Load failed: fmt chunk read %s (%d), bytes=%u",
						FresultName(fres),
						(int)fres,
						(unsigned)bytes_read);
				return false;
			}

			const uint16_t audio_format =
				(uint16_t)(wav_fmt_buf[0] | (wav_fmt_buf[1] << 8));
			info.num_channels =
				(uint16_t)(wav_fmt_buf[2] | (wav_fmt_buf[3] << 8));
			info.sample_rate =
				(uint32_t)(wav_fmt_buf[4]
					| (wav_fmt_buf[5] << 8)
					| (wav_fmt_buf[6] << 16)
					| (wav_fmt_buf[7] << 24));
			info.bits_per_sample =
				(uint16_t)(wav_fmt_buf[14] | (wav_fmt_buf[15] << 8));

			if (audio_format != WAVE_FORMAT_PCM)
			{
				LogLine("Load failed: unsupported WAV format (fmt=%u)", (unsigned)audio_format);
				return false;
			}

			fmt_found = true;

			if (chunk_size > to_read)
			{
				const FSIZE_t skip = (FSIZE_t)(chunk_size - to_read);
				fres = f_lseek(file, f_tell(file) + skip);
				if (fres != FR_OK)
				{
					LogLine("Load failed: f_lseek skipping fmt %s (%d)",
							FresultName(fres),
							(int)fres);
					return false;
				}
			}
		}
		else if (std::memcmp(wav_chunk_hdr, "data", 4) == 0)
		{
			info.data_offset = (uint32_t)f_tell(file);
			info.data_size = chunk_size;
			data_found = true;
		}
		else
		{
			const FSIZE_t skip_to = f_tell(file) + (FSIZE_t)chunk_size;
			fres = f_lseek(file, skip_to);
			if (fres != FR_OK)
			{
				LogLine("Load failed: f_lseek skipping chunk %s (%d)",
						FresultName(fres),
						(int)fres);
				return false;
			}
		}
	}

	if (!fmt_found || !data_found)
	{
		LogLine("Load failed: missing fmt or data chunk (fmt=%d data=%d)",
				(int)fmt_found,
				(int)data_found);
		return false;
	}

	LogLine("WAV header OK: channels=%u sr=%lu bits=%u data_size=%lu data_offset=%lu",
			(unsigned)info.num_channels,
			(unsigned long)info.sample_rate,
			(unsigned)info.bits_per_sample,
			(unsigned long)info.data_size,
			(unsigned long)info.data_offset);

	return true;
}

static size_t StrLen(const char* str)
{
	size_t len = 0;
	while (str[len] != '\0')
	{
		++len;
	}
	return len;
}

static void CopyString(char* dst, const char* src, size_t max_len)
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
	CopyString(out, fsi.GetSDPath(), out_len);
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

static void InitLoadLayout()
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

static int32_t LoadVisibleLines()
{
	return load_lines;
}

static void MountSd()
{
	if (sd_mounted)
	{
		return;
	}
	if (!BSP_SD_IsDetected())
	{
		LogLine("SD mount skipped: no card detected");
		return;
	}
	sd_mounted = (f_mount(&fsi.GetSDFileSystem(), fsi.GetSDPath(), 1) == FR_OK);
	if (sd_mounted)
	{
		LogLine("SD mount OK at %s", fsi.GetSDPath());
	}
	else
	{
		LogLine("SD mount failed");
	}
}

static bool BuildNextSaveName(char* out_name, size_t out_len)
{
	if (out_len == 0)
	{
		return false;
	}
	for (int i = 1; i <= 9999; ++i)
	{
		char name[16];
		snprintf(name, sizeof(name), "Rec%04d.wav", i);
		char path[64];
		BuildFilePath(name, path, sizeof(path));
		FILINFO finfo;
		const FRESULT res = f_stat(path, &finfo);
		if (res != FR_OK)
		{
			CopyString(out_name, name, out_len);
			return true;
		}
	}
	return false;
}

static void ResetSaveState()
{
	save_frames_written = 0;
	save_file_open = false;
	save_header_written = false;
	save_channels = 1;
	save_sr = 48000;
	save_data_bytes = 0;
	save_last_error = FR_OK;
}

static bool BeginSaveRecordedSample()
{
	if (!sample_loaded || sample_length == 0 || !waveform_from_recording)
	{
		LogLine("Save skipped: no recorded sample");
		return false;
	}
	MountSd();
	if (!sd_mounted)
	{
		LogLine("Save failed: SD not mounted");
		return false;
	}
	if (!BuildNextSaveName(save_filename, sizeof(save_filename)))
	{
		LogLine("Save failed: no free filename");
		return false;
	}
	char path[64];
	BuildFilePath(save_filename, path, sizeof(path));

	const FRESULT open_res = f_open(&save_file, path, FA_WRITE | FA_CREATE_NEW);
	if (open_res != FR_OK)
	{
		LogLine("Save failed: f_open %s (%d)", FresultName(open_res), (int)open_res);
		return false;
	}
	save_file_open = true;

	save_channels = (sample_channels == 0) ? 1 : sample_channels;
	save_sr = (sample_rate == 0) ? 48000 : sample_rate;
	save_data_bytes = static_cast<uint32_t>(sample_length * save_channels * sizeof(int16_t));

	WAV_FormatTypeDef header = {};
	header.ChunkId = kWavFileChunkId;
	header.FileSize = 36 + save_data_bytes;
	header.FileFormat = kWavFileWaveId;
	header.SubChunk1ID = kWavFileSubChunk1Id;
	header.SubChunk1Size = 16;
	header.AudioFormat = WAVE_FORMAT_PCM;
	header.NbrChannels = save_channels;
	header.SampleRate = save_sr;
	header.BlockAlign = static_cast<uint16_t>(save_channels * sizeof(int16_t));
	header.ByteRate = save_sr * header.BlockAlign;
	header.BitPerSample = 16;
	header.SubChunk2ID = kWavFileSubChunk2Id;
	header.SubCHunk2Size = save_data_bytes;

	UINT written = 0;
	save_last_error = f_write(&save_file, &header, sizeof(header), &written);
	if (save_last_error != FR_OK || written != sizeof(header))
	{
		LogLine("Save failed: header write %s (%d)", FresultName(save_last_error), (int)save_last_error);
		f_close(&save_file);
		save_file_open = false;
		return false;
	}
	save_header_written = true;
	return true;
}

static bool StepSaveRecordedSample(bool& done)
{
	done = false;
	if (!save_file_open || !save_header_written)
	{
		return false;
	}
	const uint32_t start_ms = System::GetNow();
	while (save_frames_written < sample_length)
	{
		const size_t frames_left = sample_length - save_frames_written;
		const size_t frames_this = (frames_left > kSaveChunkFrames) ? kSaveChunkFrames : frames_left;
		UINT written = 0;
		if (save_channels == 1)
		{
			const int16_t* src = sample_buffer_l + save_frames_written;
			save_last_error = f_write(&save_file,
									  src,
									  static_cast<UINT>(frames_this * sizeof(int16_t)),
									  &written);
			if (save_last_error == FR_OK && written != (frames_this * sizeof(int16_t)))
			{
				save_last_error = FR_DISK_ERR;
			}
		}
		else
		{
			for (size_t i = 0; i < frames_this; ++i)
			{
				wav_write[i * 2] = sample_buffer_l[save_frames_written + i];
				wav_write[i * 2 + 1] = sample_buffer_r[save_frames_written + i];
			}
			save_last_error = f_write(&save_file,
									  wav_write,
									  static_cast<UINT>(frames_this * save_channels * sizeof(int16_t)),
									  &written);
			if (save_last_error == FR_OK && written != (frames_this * save_channels * sizeof(int16_t)))
			{
				save_last_error = FR_DISK_ERR;
			}
		}
		if (save_last_error != FR_OK)
		{
			LogLine("Save failed: data write %s (%d)", FresultName(save_last_error), (int)save_last_error);
			f_close(&save_file);
			save_file_open = false;
			return false;
		}
		save_frames_written += frames_this;
		if ((System::GetNow() - start_ms) >= kSaveStepBudgetMs)
		{
			break;
		}
	}

	if (save_frames_written >= sample_length)
	{
		save_last_error = f_sync(&save_file);
		if (save_last_error != FR_OK)
		{
			LogLine("Save failed: f_sync %s (%d)", FresultName(save_last_error), (int)save_last_error);
			f_close(&save_file);
			save_file_open = false;
			return false;
		}
		f_close(&save_file);
		save_file_open = false;
		done = true;
		LogLine("Save OK: %s (%lu frames)", save_filename, (unsigned long)sample_length);
		CopyString(loaded_sample_name, save_filename, kMaxWavNameLen);
		return true;
	}
	return true;
}

static bool ReinitSdNow()
{
	if (!BSP_SD_IsDetected())
	{
		LogLine("SD init (full): no card detected");
		return false;
	}
	f_mount(0, fsi.GetSDPath(), 0);
	sd_mounted = false;
	fsi.DeInit();
	SdmmcHandler::Config sd_cfg;
	sd_cfg.Defaults();
	sdcard.Init(sd_cfg);
	fsi.Init(FatFSInterface::Config::MEDIA_SD);
	const uint8_t init_res = BSP_SD_Init();
	LogLine("SD init (full): %u", static_cast<unsigned>(init_res));
	MountSd();
	return sd_mounted;
}

static void ScanSdFiles(bool wav_only)
{
	LogLine("Scanning WAV files...");
	bool detected = BSP_SD_IsDetected();
	if (!detected)
	{
		SdmmcHandler::Config sd_cfg;
		sd_cfg.Defaults();
		sdcard.Init(sd_cfg);
		fsi.Init(FatFSInterface::Config::MEDIA_SD);
		const uint8_t init_res = BSP_SD_Init();
		LogLine("SD init (detect): %u", static_cast<unsigned>(init_res));
		detected = BSP_SD_IsDetected();
		if (!detected)
		{
			sd_mounted = false;
			sd_need_reinit = true;
			sd_detected_last = false;
			wav_file_count = 0;
			load_selected = 0;
			load_scroll = 0;
			LogLine("Scan aborted: SD not detected");
			return;
		}
	}
	if (BSP_SD_GetCardState() != SD_TRANSFER_OK)
	{
		sd_mounted = false;
		sd_need_reinit = true;
		wav_file_count = 0;
		load_selected = 0;
		load_scroll = 0;
		LogLine("Scan aborted: SD not ready");
		return;
	}
	if (detected && !sd_detected_last)
	{
		sd_need_reinit = true;
	}
	sd_detected_last = true;

	if (sd_need_reinit)
	{
		f_mount(0, fsi.GetSDPath(), 0);
		sd_mounted = false;
		SdmmcHandler::Config sd_cfg;
		sd_cfg.Defaults();
		sdcard.Init(sd_cfg);
		fsi.Init(FatFSInterface::Config::MEDIA_SD);
		const uint8_t init_res = BSP_SD_Init();
		LogLine("SD init: %u", static_cast<unsigned>(init_res));
		MountSd();
		sd_need_reinit = !sd_mounted;
	}

	if (!sd_mounted)
	{
		MountSd();
	}
	if (!sd_mounted)
	{
		wav_file_count = 0;
		load_selected = 0;
		load_scroll = 0;
		LogLine("Scan aborted: SD not mounted");
		return;
	}

	DIR dir;
	FILINFO fno;
	FRESULT res = f_opendir(&dir, fsi.GetSDPath());
	if (res != FR_OK)
	{
		wav_file_count = 0;
		load_selected = 0;
		load_scroll = 0;
		LogLine("Scan failed: f_opendir error %d", static_cast<int>(res));
		return;
	}

	int32_t count = 0;
	for (;;)
	{
		res = f_readdir(&dir, &fno);
		if (res != FR_OK || fno.fname[0] == 0)
		{
			break;
		}
		if (fno.fattrib & (AM_DIR | AM_HID))
		{
			continue;
		}
		if (wav_only && !HasWavExtension(fno.fname))
		{
			continue;
		}
		if (count >= kMaxWavFiles)
		{
			break;
		}
		CopyString(wav_files[count], fno.fname, kMaxWavNameLen);
		++count;
	}
	f_closedir(&dir);
	wav_file_count = count;
	LogLine("Scan complete: %ld files", static_cast<long>(wav_file_count));
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
}

static void ComputeWaveform()
{
	const int32_t width = 128;
	if (!sample_loaded || sample_length == 0)
	{
		for (int32_t i = 0; i < width; ++i)
		{
			waveform_min[i] = 0;
			waveform_max[i] = 0;
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

		waveform_min[col] = static_cast<int16_t>(minv * scale);
		waveform_max[col] = static_cast<int16_t>(maxv * scale);
	}
}

static void UpdateTrimFrames()
{
	if(sample_length < 2)
	{
		sample_play_start = 0;
		sample_play_end   = sample_length;
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
}

static void AdjustTrimNormalized(int32_t start_delta, int32_t end_delta, bool fine = false)
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

static void AdjustPerformFade(volatile float& value, int32_t delta, bool fine = false)
{
	if (delta == 0)
	{
		return;
	}
	const float base_step = fine ? (1.0f / 128.0f) : (1.0f / 64.0f);
	int mag = abs(delta);
	if (mag < 1)
	{
		mag = 1;
	}
	int log2 = 0;
	while (mag > 1)
	{
		mag >>= 1;
		++log2;
	}
	const float step = base_step * static_cast<float>(1 << log2);
	value += static_cast<float>(delta) * step;
	if (value < 0.0f)
	{
		value = 0.0f;
	}
	else if (value > 1.0f)
	{
		value = 1.0f;
	}
}

static void ResetPerformVoices()
{
	for (auto &voice : perform_voices)
	{
		voice.active = false;
		voice.releasing = false;
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
	}
	for (int i = 0; i < kPerformVoiceCount; ++i)
	{
		perform_lpf_l1[i].Reset();
		perform_lpf_l2[i].Reset();
		perform_lpf_r1[i].Reset();
		perform_lpf_r2[i].Reset();
	}
}

static inline float WrapPhase(float phase)
{
	while (phase > kPi)
	{
		phase -= kTwoPi;
	}
	while (phase < -kPi)
	{
		phase += kTwoPi;
	}
	return phase;
}

static void InitPvTables()
{
	if (pv_window_ready)
	{
		return;
	}
	for (size_t i = 0; i < kPvLongSize; ++i)
	{
		pv_window_long[i] = 0.5f - 0.5f * cosf(kTwoPi * static_cast<float>(i)
			/ static_cast<float>(kPvLongSize - 1));
	}
	for (size_t i = 0; i < kPvShortSize; ++i)
	{
		pv_window_short[i] = 0.5f - 0.5f * cosf(kTwoPi * static_cast<float>(i)
			/ static_cast<float>(kPvShortSize - 1));
	}
	for (size_t k = 0; k <= kPvLongSize / 2; ++k)
	{
		pv_omega[k] = kTwoPi * static_cast<float>(k)
			/ static_cast<float>(kPvLongSize);
	}
	pv_window_ready = true;
}

static void Fft(float* re, float* im, size_t n, bool inverse)
{
	if (n < 2)
	{
		return;
	}
	for (size_t i = 1, j = 0; i < n; ++i)
	{
		size_t bit = n >> 1;
		for (; j & bit; bit >>= 1)
		{
			j ^= bit;
		}
		j ^= bit;
		if (i < j)
		{
			float tr = re[i];
			re[i] = re[j];
			re[j] = tr;
			float ti = im[i];
			im[i] = im[j];
			im[j] = ti;
		}
	}

	for (size_t len = 2; len <= n; len <<= 1)
	{
		const float ang = (inverse ? 1.0f : -1.0f) * kTwoPi / static_cast<float>(len);
		const float wlen_cos = cosf(ang);
		const float wlen_sin = sinf(ang);
		for (size_t i = 0; i < n; i += len)
		{
			float wcos = 1.0f;
			float wsin = 0.0f;
			const size_t half = len >> 1;
			for (size_t j = 0; j < half; ++j)
			{
				const size_t u = i + j;
				const size_t v = u + half;
				const float tre = re[v] * wcos - im[v] * wsin;
				const float tim = re[v] * wsin + im[v] * wcos;
				re[v] = re[u] - tre;
				im[v] = im[u] - tim;
				re[u] += tre;
				im[u] += tim;
				const float next_wcos = wcos * wlen_cos - wsin * wlen_sin;
				wsin = wcos * wlen_sin + wsin * wlen_cos;
				wcos = next_wcos;
			}
		}
	}

	if (inverse)
	{
		const float inv_n = 1.0f / static_cast<float>(n);
		for (size_t i = 0; i < n; ++i)
		{
			re[i] *= inv_n;
			im[i] *= inv_n;
		}
	}
}

static size_t DetectTransientFrames(const int16_t* in_l,
									const int16_t* in_r,
									bool stereo,
									size_t in_len,
									bool* out_flags,
									size_t max_frames)
{
	const size_t win = kPvLongSize;
	const size_t hop = kPvLongHop;
	if (in_len < win)
	{
		return 0;
	}
	size_t frames = 1 + (in_len - win) / hop;
	if (frames > max_frames)
	{
		frames = max_frames;
	}
	float prev_energy = 0.0f;
	for (size_t f = 0; f < frames; ++f)
	{
		const size_t start = f * hop;
		float energy = 0.0f;
		for (size_t i = 0; i < win; ++i)
		{
			const size_t idx = start + i;
			if (idx >= in_len)
			{
				break;
			}
			float s = static_cast<float>(in_l[idx]);
			if (stereo && in_r != nullptr)
			{
				s = 0.5f * (s + static_cast<float>(in_r[idx]));
			}
			const float v = s * kSampleScale;
			energy += v * v;
		}
		energy /= static_cast<float>(win);
		const bool transient = (prev_energy > 0.0f)
			&& (energy > prev_energy * kPvTransientThresh)
			&& (energy > kPvTransientFloor);
		out_flags[f] = transient;
		prev_energy = energy;
	}
	return frames;
}

static size_t PhaseVocoderStretchChannel(const int16_t* in,
										 size_t in_len,
										 float stretch,
										 const bool* transient_flags,
										 size_t frames,
										 float* out,
										 size_t out_cap)
{
	if (in_len == 0 || out_cap == 0)
	{
		return 0;
	}
	if (in_len < kPvLongSize || frames == 0)
	{
		const size_t copy_len = (in_len < out_cap) ? in_len : out_cap;
		for (size_t i = 0; i < copy_len; ++i)
		{
			out[i] = static_cast<float>(in[i]) * kSampleScale;
		}
		return copy_len;
	}

	std::memset(out, 0, sizeof(float) * out_cap);
	std::memset(pv_prev_phase, 0, sizeof(pv_prev_phase));
	std::memset(pv_sum_phase, 0, sizeof(pv_sum_phase));

	const size_t bins = kPvLongSize / 2;
	const float analysis_hop = static_cast<float>(kPvLongHop);
	const float synth_hop = analysis_hop * stretch;
	float out_pos = 0.0f;
	size_t out_len = 0;

	for (size_t f = 0; f < frames; ++f)
	{
		const size_t in_pos = f * kPvLongHop;
		for (size_t i = 0; i < kPvLongSize; ++i)
		{
			const size_t idx = in_pos + i;
			const float s = (idx < in_len)
				? static_cast<float>(in[idx]) * kSampleScale
				: 0.0f;
			pv_fft_re[i] = s * pv_window_long[i];
			pv_fft_im[i] = 0.0f;
		}

		Fft(pv_fft_re, pv_fft_im, kPvLongSize, false);

		for (size_t k = 0; k <= bins; ++k)
		{
			const float re = pv_fft_re[k];
			const float im = pv_fft_im[k];
			const float mag = sqrtf(re * re + im * im);
			const float phase = atan2f(im, re);
			pv_mag[k] = mag;
			pv_phase[k] = phase;

			const float delta = WrapPhase(phase - pv_prev_phase[k] - pv_omega[k] * analysis_hop);
			const float true_freq = pv_omega[k] + delta / analysis_hop;
			pv_sum_phase[k] += true_freq * synth_hop;
			pv_prev_phase[k] = phase;
		}

		const bool transient = transient_flags != nullptr && transient_flags[f];
		if (transient)
		{
			for (size_t k = 0; k <= bins; ++k)
			{
				pv_sum_phase[k] = pv_phase[k];
				pv_prev_phase[k] = pv_phase[k];
			}
		}

		for (size_t k = 0; k <= bins; ++k)
		{
			int peak = -1;
			float peak_mag = 0.0f;
			size_t start = (k > static_cast<size_t>(kPvPhaseLockRadius))
				? (k - static_cast<size_t>(kPvPhaseLockRadius))
				: 1;
			size_t end = k + static_cast<size_t>(kPvPhaseLockRadius);
			if (end >= bins)
			{
				end = bins - 1;
			}
			for (size_t p = start; p <= end; ++p)
			{
				if (p == 0 || p >= bins)
				{
					continue;
				}
				if (pv_mag[p] > pv_mag[p - 1] && pv_mag[p] >= pv_mag[p + 1])
				{
					if (pv_mag[p] > peak_mag)
					{
						peak_mag = pv_mag[p];
						peak = static_cast<int>(p);
					}
				}
			}
			float out_phase = pv_sum_phase[k];
			if (peak >= 0)
			{
				out_phase = pv_sum_phase[peak] + (pv_phase[k] - pv_phase[peak]);
			}
			pv_fft_re[k] = pv_mag[k] * cosf(out_phase);
			pv_fft_im[k] = pv_mag[k] * sinf(out_phase);
		}

		for (size_t k = 1; k < bins; ++k)
		{
			pv_fft_re[kPvLongSize - k] = pv_fft_re[k];
			pv_fft_im[kPvLongSize - k] = -pv_fft_im[k];
		}
		pv_fft_im[0] = 0.0f;
		pv_fft_im[bins] = 0.0f;

		Fft(pv_fft_re, pv_fft_im, kPvLongSize, true);

		const size_t out_index = static_cast<size_t>(out_pos + 0.5f);
		for (size_t i = 0; i < kPvLongSize; ++i)
		{
			const size_t oi = out_index + i;
			if (oi >= out_cap)
			{
				break;
			}
			out[oi] += pv_fft_re[i] * pv_window_long[i];
		}

		if (transient)
		{
			const size_t short_offset = (kPvLongSize - kPvShortSize) / 2;
			const size_t short_start = in_pos + short_offset;
			const size_t short_out_index = out_index + short_offset;
			for (size_t i = 0; i < kPvShortSize; ++i)
			{
				const size_t idx = short_start + i;
				const size_t oi = short_out_index + i;
				if (oi >= out_cap)
				{
					break;
				}
				const float s = (idx < in_len)
					? static_cast<float>(in[idx]) * kSampleScale
					: 0.0f;
				out[oi] += s * pv_window_short[i] * kPvTransientMix;
			}
		}

		out_pos += synth_hop;
		const size_t frame_end = out_index + kPvLongSize;
		if (frame_end > out_len)
		{
			out_len = frame_end;
		}
		if (out_len >= out_cap)
		{
			out_len = out_cap;
			break;
		}
	}
	return out_len;
}

static void ResampleToLength(const float* in, size_t in_len, int16_t* out, size_t out_len)
{
	if (out_len == 0)
	{
		return;
	}
	if (in_len == 0)
	{
		for (size_t i = 0; i < out_len; ++i)
		{
			out[i] = 0;
		}
		return;
	}
	if (in_len == 1)
	{
		const float val = in[0] * kPvOutputGain;
		int32_t s = static_cast<int32_t>(val * 32767.0f);
		if (s > 32767) s = 32767;
		if (s < -32768) s = -32768;
		const int16_t samp = static_cast<int16_t>(s);
		for (size_t i = 0; i < out_len; ++i)
		{
			out[i] = samp;
		}
		return;
	}
	const float scale = static_cast<float>(in_len - 1) / static_cast<float>(out_len - 1);
	for (size_t i = 0; i < out_len; ++i)
	{
		const float pos = static_cast<float>(i) * scale;
		const size_t idx = static_cast<size_t>(pos);
		const float frac = pos - static_cast<float>(idx);
		const float s0 = in[idx];
		const float s1 = in[(idx + 1 < in_len) ? (idx + 1) : idx];
		float val = (s0 + (s1 - s0) * frac) * kPvOutputGain;
		int32_t s = static_cast<int32_t>(val * 32767.0f);
		if (s > 32767) s = 32767;
		if (s < -32768) s = -32768;
		out[i] = static_cast<int16_t>(s);
	}
}

static bool ProcessBakedBank()
{
	if (!sample_loaded || !baked_window_valid)
	{
		return false;
	}
	const size_t window_start = baked_play_start;
	const size_t window_end = baked_play_end;
	if (window_end <= window_start)
	{
		return false;
	}
	const size_t window_len = window_end - window_start;
	if (window_len == 0 || window_len > kMaxSampleSamples)
	{
		return false;
	}
	const size_t needed_frames = window_len * static_cast<size_t>(kBakeNoteCount);
	if (needed_frames > kBakeBankFramesMax)
	{
		LogLine("Bake failed: window too long (%lu frames) for bank",
				static_cast<unsigned long>(window_len));
		return false;
	}

	InitPvTables();

	const bool stereo = (sample_channels == 2);
	const size_t frames = DetectTransientFrames(sample_buffer_l + window_start,
												stereo ? (sample_buffer_r + window_start) : nullptr,
												stereo,
												window_len,
												pv_transient_flags,
												kPvMaxFrames);
	const size_t out_cap = kPvStretchBufSize;

	for (int i = 0; i < kBakeNoteCount; ++i)
	{
		const int midi_note = kBakeFirstMidi + i;
		const int semis = midi_note - kBaseMidiNote;
		const size_t offset = static_cast<size_t>(i) * window_len;
		baked_note_offsets[i] = offset;
		baked_note_ready[i] = false;
		int16_t* out_l = baked_buffer_l + offset;
		int16_t* out_r = baked_buffer_r + offset;
		if (semis == 0)
		{
			for (size_t s = 0; s < window_len; ++s)
			{
				const size_t idx = window_start + s;
				out_l[s] = sample_buffer_l[idx];
				out_r[s] = (sample_channels == 2) ? sample_buffer_r[idx] : sample_buffer_l[idx];
			}
			baked_note_ready[i] = true;
			continue;
		}

		const float pitch_ratio = powf(2.0f, static_cast<float>(semis) / 12.0f);
		const size_t stretch_len_l = PhaseVocoderStretchChannel(sample_buffer_l + window_start,
																window_len,
																pitch_ratio,
																pv_transient_flags,
																frames,
																baked_stretch_buf,
																out_cap);
		ResampleToLength(baked_stretch_buf, stretch_len_l, out_l, window_len);

		if (stereo)
		{
			const size_t stretch_len_r = PhaseVocoderStretchChannel(sample_buffer_r + window_start,
																	window_len,
																	pitch_ratio,
																	pv_transient_flags,
																	frames,
																	baked_stretch_buf,
																	out_cap);
			ResampleToLength(baked_stretch_buf, stretch_len_r, out_r, window_len);
		}
		else
		{
			for (size_t s = 0; s < window_len; ++s)
			{
				out_r[s] = out_l[s];
			}
		}
		baked_note_ready[i] = true;
	}

	baked_length = window_len;
	baked_ready = true;
	return true;
}

static void ApplyLoadedSampleFade(size_t length, uint32_t rate)
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
	FIL* file = &wav_file;
	UINT bytes_read = 0;

	playback_active = false;
	playback_phase = 0.0f;
	sample_loaded = false;
	perform_bake_active = false;
	baked_window_valid = false;
	baked_ready = false;
	baked_length = 0;
	for (int i = 0; i < kBakeNoteCount; ++i)
	{
		baked_note_offsets[i] = 0;
		baked_note_ready[i] = false;
	}
	perform_attack_norm = 0.0f;
	perform_release_norm = 0.0f;
	ResetPerformVoices();
	sample_length = 0;
	sample_channels = 1;
	trim_start = 0.0f;
	trim_end = 1.0f;

	LogLine("Loading sample: %s", path);
	FILINFO finfo;
	FRESULT res = f_stat(path, &finfo);
	if (res == FR_OK)
	{
		LogLine("f_stat: size=%lu attrib=0x%02x", static_cast<unsigned long>(finfo.fsize), static_cast<unsigned>(finfo.fattrib));
	}
	else
	{
		LogLine("f_stat failed: %s (%d)", FresultName(res), static_cast<int>(res));
	}

	if (BSP_SD_GetCardState() != SD_TRANSFER_OK)
	{
		LogLine("Load failed: SD not ready");
		return false;
	}
	res = f_open(file, path, FA_READ);
	if (res != FR_OK)
	{
		LogLine("Load failed: f_open %s (%d)", FresultName(res), static_cast<int>(res));
		return false;
	}
	LogLine("f_open ok: size=%lu tell=%lu",
			static_cast<unsigned long>(f_size(file)),
			static_cast<unsigned long>(f_tell(file)));

	WavInfo wav;
	if (!ParseWavHeader(file, wav))
	{
		f_close(file);
		return false;
	}

	if (wav.bits_per_sample != 16)
	{
		LogLine("Load failed: unsupported bit depth %u", (unsigned)wav.bits_per_sample);
		f_close(file);
		return false;
	}

	if (wav.num_channels < 1 || wav.num_channels > 2)
	{
		LogLine("Load failed: unsupported channel count %u", (unsigned)wav.num_channels);
		f_close(file);
		return false;
	}
	sample_channels = wav.num_channels;

	const size_t bytes_per_sample = wav.bits_per_sample / 8;
	const size_t frame_bytes = bytes_per_sample * wav.num_channels;
	size_t total_frames = (frame_bytes == 0) ? 0 : (wav.data_size / frame_bytes);
	LogLine("WAV info: %lu frames @ %luHz, channels=%u",
			static_cast<unsigned long>(total_frames),
			static_cast<unsigned long>(wav.sample_rate),
			static_cast<unsigned>(wav.num_channels));
	if (total_frames == 0)
	{
		LogLine("Load failed: no data frames");
		f_close(file);
		return false;
	}
	if (total_frames > kMaxSampleSamples)
	{
		LogLine("Truncating to %lu frames", static_cast<unsigned long>(kMaxSampleSamples));
		total_frames = kMaxSampleSamples;
	}

	const uint32_t data_offset = wav.data_offset;
	res = f_lseek(file, data_offset);
	if (res != FR_OK)
	{
		LogLine("Load failed: f_lseek %s (%d)", FresultName(res), static_cast<int>(res));
		LogLine("f_error=%u f_eof=%u", static_cast<unsigned>(f_error(file)), static_cast<unsigned>(f_eof(file)));
		LogSdCardStatus();
		f_close(file);
		return false;
	}

	size_t dest_index = 0;
	int32_t last_percent = 0;
	LogLine("Load progress: 0%%");
	while (dest_index < total_frames)
	{
		size_t frames_to_read = total_frames - dest_index;
		if (frames_to_read > kSampleChunkFrames)
		{
			frames_to_read = kSampleChunkFrames;
		}
		const size_t bytes_to_read = frames_to_read * wav.num_channels * sizeof(int16_t);
		bytes_read = 0;
		res = f_read(file, wav_read, bytes_to_read, &bytes_read);
		if (res != FR_OK
			|| bytes_read == 0)
		{
			LogLine("Load read error %s (%d) at %lu frames",
					FresultName(res),
					static_cast<int>(res),
					static_cast<unsigned long>(dest_index));
			LogLine("f_error=%u f_eof=%u", static_cast<unsigned>(f_error(file)), static_cast<unsigned>(f_eof(file)));
			LogSdCardStatus();
			break;
		}
		const size_t frames_read = bytes_read / (wav.num_channels * sizeof(int16_t));
		for (size_t i = 0; i < frames_read; ++i)
		{
			if (wav.num_channels == 1)
			{
				const int16_t samp = wav_read[i];
				sample_buffer_l[dest_index] = samp;
				sample_buffer_r[dest_index] = samp;
				dest_index++;
			}
			else
			{
				sample_buffer_l[dest_index] = wav_read[i * 2];
				sample_buffer_r[dest_index] = wav_read[i * 2 + 1];
				dest_index++;
			}
			if (dest_index >= kMaxSampleSamples)
			{
				break;
			}
		}
		if (total_frames > 0)
		{
			const int32_t percent = static_cast<int32_t>(
				(dest_index * 100U) / total_frames);
			if (percent >= last_percent + kLoadProgressStep || percent == 100)
			{
				LogLine("Load progress: %ld%%", static_cast<long>(percent));
				last_percent = percent;
			}
		}
		if (frames_read < frames_to_read)
		{
			break;
		}
	}
	f_close(file);

	if (dest_index < total_frames)
	{
		LogLine("Load finished early: %lu/%lu frames",
				static_cast<unsigned long>(dest_index),
				static_cast<unsigned long>(total_frames));
	}
	sample_length = dest_index;
	sample_rate = wav.sample_rate;
	sample_loaded = (sample_length > 0);
	if (!sample_loaded)
	{
		LogLine("Load failed: no audio data read");
		return false;
	}
	ApplyLoadedSampleFade(sample_length, sample_rate);
	trim_start = 0.0f;
	trim_end = 1.0f;
	LogLine("Load complete: %lu frames", static_cast<unsigned long>(sample_length));
	waveform_from_recording = false;
	ComputeWaveform();
	waveform_ready = true;
	waveform_dirty = true;
	UpdateTrimFrames();
	return true;
}

static bool LoadSampleAtIndex(int32_t index)
{
	if (!BSP_SD_IsDetected())
	{
		LogLine("Load failed: SD not detected");
		sd_mounted = false;
		return false;
	}
	MountSd();
	if (!sd_mounted)
	{
		SdmmcHandler::Config sd_cfg;
		sd_cfg.Defaults();
		sdcard.Init(sd_cfg);
		fsi.Init(FatFSInterface::Config::MEDIA_SD);
		const uint8_t init_res = BSP_SD_Init();
		LogLine("SD init: %u", static_cast<unsigned>(init_res));
		MountSd();
	}
	if (!sd_mounted)
	{
		LogLine("Load failed: SD not mounted");
		return false;
	}
	if (BSP_SD_GetCardState() != SD_TRANSFER_OK)
	{
		LogLine("Load failed: SD not ready");
		return false;
	}
	if (index < 0 || index >= wav_file_count)
	{
		LogLine("Load failed: invalid index %ld", static_cast<long>(index));
		return false;
	}
	char path[64];
	BuildFilePath(wav_files[index], path, sizeof(path));
	CopyString(loaded_sample_name, wav_files[index], kMaxWavNameLen);
	LogLine("Load request: %s", loaded_sample_name);
	return LoadSampleFromPath(path);
}

static void StopPreview()
{
	preview_active = false;
	preview_hold = false;
	preview_index = -1;
	preview_read_frac = 0.0f;
	preview_read_index = 0;
	preview_write_index = 0;
	if (preview_file_open)
	{
		f_close(&preview_file);
		preview_file_open = false;
	}
}

static bool BeginPreviewAtIndex(int32_t index)
{
	if (!BSP_SD_IsDetected())
	{
		LogLine("Preview failed: SD not detected");
		sd_mounted = false;
		return false;
	}
	MountSd();
	if (!sd_mounted)
	{
		SdmmcHandler::Config sd_cfg;
		sd_cfg.Defaults();
		sdcard.Init(sd_cfg);
		fsi.Init(FatFSInterface::Config::MEDIA_SD);
		const uint8_t init_res = BSP_SD_Init();
		LogLine("SD init: %u", static_cast<unsigned>(init_res));
		MountSd();
	}
	if (!sd_mounted)
	{
		LogLine("Preview failed: SD not mounted");
		return false;
	}
	if (BSP_SD_GetCardState() != SD_TRANSFER_OK)
	{
		LogLine("Preview failed: SD not ready");
		return false;
	}
	if (index < 0 || index >= wav_file_count)
	{
		LogLine("Preview failed: invalid index %ld", static_cast<long>(index));
		return false;
	}
	char path[64];
	BuildFilePath(wav_files[index], path, sizeof(path));
	LogLine("Preview request: %s", wav_files[index]);

	if (preview_file_open)
	{
		f_close(&preview_file);
		preview_file_open = false;
	}
	const FRESULT open_res = f_open(&preview_file, path, FA_READ);
	if (open_res != FR_OK)
	{
		LogLine("Preview failed: f_open %s (%d)", FresultName(open_res), (int)open_res);
		return false;
	}
	preview_file_open = true;

	WavInfo wav;
	if (!ParseWavHeader(&preview_file, wav))
	{
		f_close(&preview_file);
		preview_file_open = false;
		return false;
	}
	if (wav.bits_per_sample != 16)
	{
		LogLine("Preview failed: unsupported bit depth %u", (unsigned)wav.bits_per_sample);
		f_close(&preview_file);
		preview_file_open = false;
		return false;
	}
	if (wav.num_channels < 1 || wav.num_channels > 2)
	{
		LogLine("Preview failed: unsupported channel count %u", (unsigned)wav.num_channels);
		f_close(&preview_file);
		preview_file_open = false;
		return false;
	}
	preview_sample_rate = wav.sample_rate;
	preview_channels = wav.num_channels;
	const uint32_t rate = (preview_sample_rate == 0) ? 48000 : preview_sample_rate;
	preview_rate = static_cast<float>(rate) / hw.AudioSampleRate();
	preview_data_offset = wav.data_offset;
	FRESULT seek_res = f_lseek(&preview_file, preview_data_offset);
	if (seek_res != FR_OK)
	{
		LogLine("Preview failed: f_lseek %s (%d)", FresultName(seek_res), (int)seek_res);
		f_close(&preview_file);
		preview_file_open = false;
		return false;
	}

	preview_read_frac = 0.0f;
	preview_read_index = 0;
	preview_write_index = 0;
	preview_index = index;
	preview_active = true;
	return true;
}

static size_t PreviewAvailableFrames(size_t read_idx, size_t write_idx)
{
	if (write_idx >= read_idx)
	{
		return write_idx - read_idx;
	}
	return (kPreviewBufferFrames - read_idx) + write_idx;
}

static size_t PreviewFreeFrames(size_t read_idx, size_t write_idx)
{
	const size_t used = PreviewAvailableFrames(read_idx, write_idx);
	if (used >= kPreviewBufferFrames - 1)
	{
		return 0;
	}
	return (kPreviewBufferFrames - 1) - used;
}

static void FillPreviewBuffer()
{
	if (!preview_active || !preview_file_open)
	{
		return;
	}
	const uint32_t start_ms = System::GetNow();
	while (true)
	{
		const size_t read_idx = preview_read_index;
		const size_t write_idx = preview_write_index;
		const size_t free_frames = PreviewFreeFrames(read_idx, write_idx);
		if (free_frames == 0)
		{
			break;
		}
		const size_t frames_to_read = (free_frames > kPreviewReadFrames) ? kPreviewReadFrames : free_frames;
		const size_t bytes_to_read = frames_to_read * preview_channels * sizeof(int16_t);
		UINT bytes_read = 0;
		FRESULT res = f_read(&preview_file, preview_read_buf, bytes_to_read, &bytes_read);
		if (res != FR_OK)
		{
			LogLine("Preview read error %s (%d)", FresultName(res), (int)res);
			StopPreview();
			return;
		}
		if (bytes_read == 0)
		{
			FRESULT seek_res = f_lseek(&preview_file, preview_data_offset);
			if (seek_res != FR_OK)
			{
				LogLine("Preview loop seek failed %s (%d)", FresultName(seek_res), (int)seek_res);
				StopPreview();
				return;
			}
			continue;
		}
		const size_t frames_read = bytes_read / (preview_channels * sizeof(int16_t));
		size_t w = write_idx;
		for (size_t i = 0; i < frames_read; ++i)
		{
			int32_t mono = 0;
			if (preview_channels == 1)
			{
				mono = preview_read_buf[i];
			}
			else
			{
				const int16_t l = preview_read_buf[i * 2];
				const int16_t r = preview_read_buf[i * 2 + 1];
				mono = (static_cast<int32_t>(l) + static_cast<int32_t>(r)) / 2;
			}
			preview_buffer[w] = static_cast<int16_t>(mono);
			w = (w + 1) % kPreviewBufferFrames;
		}
		preview_write_index = w;
		if ((System::GetNow() - start_ms) >= kPreviewReadBudgetMs)
		{
			break;
		}
	}
}

static bool DeleteFileAtIndex(int32_t index)
{
	if (!BSP_SD_IsDetected())
	{
		LogLine("Delete failed: SD not detected");
		sd_mounted = false;
		return false;
	}
	MountSd();
	if (!sd_mounted)
	{
		SdmmcHandler::Config sd_cfg;
		sd_cfg.Defaults();
		sdcard.Init(sd_cfg);
		fsi.Init(FatFSInterface::Config::MEDIA_SD);
		const uint8_t init_res = BSP_SD_Init();
		LogLine("SD init: %u", static_cast<unsigned>(init_res));
		MountSd();
	}
	if (!sd_mounted)
	{
		LogLine("Delete failed: SD not mounted");
		return false;
	}
	if (BSP_SD_GetCardState() != SD_TRANSFER_OK)
	{
		LogLine("Delete failed: SD not ready");
		return false;
	}
	if (index < 0 || index >= wav_file_count)
	{
		LogLine("Delete failed: invalid index %ld", static_cast<long>(index));
		return false;
	}
	char path[64];
	BuildFilePath(wav_files[index], path, sizeof(path));
	LogLine("Delete request: %s", wav_files[index]);
	const FRESULT res = f_unlink(path);
	if (res != FR_OK)
	{
		LogLine("Delete failed: f_unlink %s (%d)", FresultName(res), (int)res);
		return false;
	}
	LogLine("Delete OK: %s", wav_files[index]);
	return true;
}

static void DrawTinyString(const char* str, int x, int y, bool on);
static int TinyStringWidth(const char* str);

static void DrawMenu(int32_t selected)
{
	constexpr int kMarginX = 2;
	constexpr int kMarginY = 2;
	constexpr int kGapX = 2;
	constexpr int kGapY = 2;
	constexpr int kBoxW = (kDisplayW - (kMarginX * 2) - kGapX) / 2;
	constexpr int kBoxH = (kDisplayH - (kMarginY * 2) - kGapY) / 2;
	display.Fill(false);
	for (int32_t i = 0; i < kMenuCount; ++i)
	{
		const int row = static_cast<int>(i) / 2;
		const int col = static_cast<int>(i) % 2;
		const int x = kMarginX + col * (kBoxW + kGapX);
		const int y = kMarginY + row * (kBoxH + kGapY);
		const bool is_selected = (i == selected);
		display.DrawRect(x,
						 y,
						 x + kBoxW - 1,
						 y + kBoxH - 1,
						 true,
						 is_selected);
		const char* label = kMenuLabels[i];
		const int text_w = TinyStringWidth(label);
		const int text_h = Font5x7::H;
		const int text_x = x + (kBoxW - text_w) / 2;
		const int text_y = y + (kBoxH - text_h) / 2;
		DrawTinyString(label, text_x, text_y, !is_selected);
	}
	display.Update();
}

static void DrawScaledChar(char ch,
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

static void DrawScaledString(const char* str,
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

static void DrawTinyString(const char* str, int x, int y, bool on)
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

static void DrawTinyVerticalString(const char* str, int x, int y, int h, bool on)
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

static void DrawTinyVerticalStringBold(const char* str, int x, int y, int h, bool on)
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

static void DrawPerformScreen(int32_t selected,
							  bool fx_select_active,
							  int32_t fx_selected,
							  bool amp_select_active,
							  int32_t amp_selected,
							  bool flt_select_active,
							  int32_t flt_selected)
{
	constexpr int kMarginX = 2;
	constexpr int kMarginY = 2;
	constexpr int kGapX = 2;
	constexpr int kGapY = 2;
	constexpr int kBoxW = (kDisplayW - (kMarginX * 2) - kGapX) / 2;
	constexpr int kBoxH = (kDisplayH - (kMarginY * 2) - kGapY) / 2;
	constexpr int kLabelPadX = 2;
	constexpr int kLabelPadY = 1;
	display.Fill(false);

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

	auto draw_waveform_preview = [&](const Box& box, bool is_selected)
	{
		if (!sample_loaded || sample_length == 0)
		{
			return;
		}
		const int preview_x0 = box.x + kLabelPadX + Font5x7::W + 4;
		const int preview_x1 = box.x + kBoxW - 2;
		const int preview_y0 = box.y + kLabelPadY + 1;
		const int preview_y1 = box.y + kBoxH - kLabelPadY - 1;
		if (preview_x1 <= preview_x0 || preview_y1 <= preview_y0)
		{
			return;
		}
		const int preview_w = preview_x1 - preview_x0 + 1;
		const int preview_h = preview_y1 - preview_y0 + 1;
		if (preview_h < 3)
		{
			return;
		}
		const int mid = preview_y0 + (preview_h / 2);
		float scale = static_cast<float>((preview_h / 2) - 1) / 28.0f;
		if (scale <= 0.0f)
		{
			scale = 1.0f / 28.0f;
		}
		const bool on = !is_selected;
		for (int x = 0; x < preview_w; ++x)
		{
			const int wf_idx = (preview_w > 1) ? ((x * 127) / (preview_w - 1)) : 0;
			int top = mid + static_cast<int>(static_cast<float>(waveform_min[wf_idx]) * scale);
			int bottom = mid + static_cast<int>(static_cast<float>(waveform_max[wf_idx]) * scale);
			if (top > bottom)
			{
				const int tmp = top;
				top = bottom;
				bottom = tmp;
			}
			if (top < preview_y0)
			{
				top = preview_y0;
			}
			if (bottom > preview_y1)
			{
				bottom = preview_y1;
			}
			display.DrawLine(preview_x0 + x, top, preview_x0 + x, bottom, on);
		}
	};

	for (int i = 0; i < kPerformBoxCount; ++i)
	{
		const auto& box = boxes[i];
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
		DrawTinyVerticalStringBold(box.label,
								   box.x + kLabelPadX,
								   box.y + kLabelPadY,
								   kBoxH - (kLabelPadY * 2),
								   !is_selected);
		if (i == kPerformEdtIndex)
		{
			draw_waveform_preview(box, is_selected);
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
			const char* labels[kPerformFaderCount] = {"S", "M", "D", "R"};
			const float values[kPerformFaderCount] = {fx_s_wet, fx_c_wet, delay_wet, reverb_wet};
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
	display.Update();
}

static void DrawProgressBar(int x, int y, int w, int h, int32_t percent)
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

static void DrawLoadMessage(const char* line1, const char* line2)
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
	display.Update();
}

static void DrawLoadMenu(int32_t top_index, int32_t selected)
{
	const FontDef font = Font_6x8;
	display.Fill(false);

	if (!BSP_SD_IsDetected() || BSP_SD_GetCardState() != SD_TRANSFER_OK)
	{
		sd_mounted = false;
		DrawLoadMessage("SD NOT", "MOUNTED");
		return;
	}
	if (!sd_mounted)
	{
		DrawLoadMessage("SD NOT", "MOUNTED");
		return;
	 }
	if (wav_file_count == 0)
	{
		if (delete_mode)
		{
			DrawLoadMessage("NO", "FILES");
		}
		else
		{
			DrawLoadMessage("NO WAV", "FILES");
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
	display.Update();
}

static int LoadTargetDisplayIndex(LoadDestination selected)
{
	switch (selected)
	{
		case LoadDestination::Play: return 0;
		case LoadDestination::Bake: return 1;
		case LoadDestination::Perform: return 2;
		default: return 0;
	}
}

static LoadDestination LoadTargetFromDisplayIndex(int32_t index)
{
	switch (index)
	{
		case 0: return LoadDestination::Play;
		case 1: return LoadDestination::Bake;
		case 2: return LoadDestination::Perform;
		default: return LoadDestination::Play;
	}
}

static void DrawLoadTargetMenu(LoadDestination selected)
{
	const int selected_idx = LoadTargetDisplayIndex(selected);
	display.Fill(false);
	constexpr int kMargin = 2;
	constexpr int kGap = 2;
	const int top_h = (kDisplayH - (kMargin * 2) - kGap) / 2;
	const int bottom_h = kDisplayH - (kMargin * 2) - kGap - top_h;
	const int top_y = kMargin;
	const int bottom_y = kMargin + top_h + kGap;
	const int top_w = (kDisplayW - (kMargin * 2) - kGap) / 2;
	const int left_x = kMargin;
	const int right_x = kMargin + top_w + kGap;
	const int bottom_w = kDisplayW - (kMargin * 2);

	auto draw_box = [&](int x, int y, int w, int h, const char* label, bool highlight)
	{
		display.DrawRect(x, y, x + w - 1, y + h - 1, true, highlight);
		const int len = static_cast<int>(StrLen(label));
		if (len <= 0)
		{
			return;
		}
		const int char_w = Font5x7::W + 1;
		const int text_w = (len - 1) * char_w + Font5x7::W;
		const int text_h = Font5x7::H;
		int text_x = x + (w - text_w) / 2;
		int text_y = y + (h - text_h) / 2;
		if (text_x < x + 1)
		{
			text_x = x + 1;
		}
		if (text_y < y + 1)
		{
			text_y = y + 1;
		}
		DrawTinyString(label, text_x, text_y, !highlight);
	};

	draw_box(left_x, top_y, top_w, top_h, "PLAY", selected_idx == 0);
	draw_box(right_x, top_y, top_w, top_h, "BAKE", selected_idx == 1);
	draw_box(kMargin, bottom_y, bottom_w, bottom_h, "PERFORM", selected_idx == 2);
	display.Update();
}

static void DrawRecordReadyScreen()
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

	display.Update();
}

static void DrawDeleteConfirm(const char* name)
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
	display.WriteString("R=YES L=NO", font, true);
	display.Update();
}

static void DrawConfirmBakeScreen(int32_t selected)
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("Confirm Bake?", font, true);

	const int box_y = font.FontHeight + 6;
	const int box_h = 28;
	const int box_w = 50;
	const int gap = 8;
	const int start_x = (kDisplayW - (box_w * 2 + gap)) / 2;

	auto draw_box = [&](int x, const char* label, bool highlight)
	{
		display.DrawRect(x,
						 box_y,
						 x + box_w - 1,
						 box_y + box_h - 1,
						 true,
						 highlight);
		const int text_x = x + (box_w - static_cast<int>(StrLen(label)) * font.FontWidth) / 2;
		const int text_y = box_y + (box_h - font.FontHeight) / 2;
		display.SetCursor(text_x, text_y);
		display.WriteString(label, font, !highlight);
	};

	draw_box(start_x, "YES", selected == 0);
	draw_box(start_x + box_w + gap, "NO", selected == 1);
	display.Update();
}

static void DrawRecordSourceScreen()
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
	display.Update();
}

static void DrawRecordArmed()
{
	DrawRecordReadyScreen();
}

static inline int ClampI(int v, int lo, int hi);

static void StartRecording()
{
	record_pos = 0;
	sample_length = 0;
	sample_loaded = false;
	perform_bake_active = false;
	baked_window_valid = false;
	baked_ready = false;
	baked_length = 0;
	for (int i = 0; i < kBakeNoteCount; ++i)
	{
		baked_note_offsets[i] = 0;
		baked_note_ready[i] = false;
	}
	perform_attack_norm = 0.0f;
	perform_release_norm = 0.0f;
	ResetPerformVoices();
	playback_active = false;
	sample_channels = 1;
	sample_rate = 48000;
	trim_start = 0.0f;
	trim_end = 1.0f;
	CopyString(loaded_sample_name, "RECORD", kMaxWavNameLen);
	for (int i = 0; i < 128; ++i)
	{
		live_wave_min[i] = 0;
		live_wave_max[i] = 0;
	}
	live_wave_last_col = -1;
	live_wave_peak = 1;
	live_wave_dirty = true;
	record_state = RecordState::Recording;
	LogLine("Record: start (monitor ON)");
}

static void DrawRecordCountdown()
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

	display.Update();
}

static void DrawRecordRecording()
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("RECORDING: 5 SEC MAX", font, true);

	const int wave_top = font.FontHeight + 2;
	const int wave_bottom = kDisplayH - 1;
	const int mid = wave_top + (wave_bottom - wave_top) / 2;
	for (int x = 0; x < 128; ++x)
	{
		int top = mid + live_wave_min[x];
		int bottom = mid + live_wave_max[x];
		if (top > bottom)
		{
			const int tmp = top;
			top = bottom;
			bottom = tmp;
		}
		top = ClampI(top, wave_top, wave_bottom);
		bottom = ClampI(bottom, wave_top, wave_bottom);
		display.DrawLine(x, top, x, bottom, true);
	}
	display.Update();
}

static void DrawShiftMenu(int32_t selected)
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
	}
	display.Update();
}

static void DrawSdInitScreen()
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
	display.Update();
}

static void DrawSaveScreen()
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
	display.Update();
}

static inline int ClampI(int v, int lo, int hi)
{
	return (v < lo) ? lo : (v > hi ? hi : v);
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

static constexpr int kPlayTinyW = 3;
static constexpr int kPlayTinyH = 5;
static constexpr int kPlayTinySpacing = 1;

static void GetPlayTinyGlyph(char c, uint8_t out_rows[kPlayTinyH])
{
	for (int i = 0; i < kPlayTinyH; ++i)
	{
		out_rows[i] = 0;
	}

	auto set = [&](uint8_t r0, uint8_t r1, uint8_t r2, uint8_t r3, uint8_t r4)
	{
		out_rows[0] = r0;
		out_rows[1] = r1;
		out_rows[2] = r2;
		out_rows[3] = r3;
		out_rows[4] = r4;
	};

	switch (c)
	{
		case '0': set(0b111,0b101,0b101,0b101,0b111); return;
		case '1': set(0b010,0b110,0b010,0b010,0b111); return;
		case '2': set(0b111,0b001,0b111,0b100,0b111); return;
		case '3': set(0b111,0b001,0b111,0b001,0b111); return;
		case '4': set(0b101,0b101,0b111,0b001,0b001); return;
		case '5': set(0b111,0b100,0b111,0b001,0b111); return;
		case '6': set(0b111,0b100,0b111,0b101,0b111); return;
		case '7': set(0b111,0b001,0b001,0b001,0b001); return;
		case '8': set(0b111,0b101,0b111,0b101,0b111); return;
		case '9': set(0b111,0b101,0b111,0b001,0b111); return;
		case 'b': set(0b100,0b100,0b110,0b101,0b110); return;
		case 'p': set(0b110,0b101,0b110,0b100,0b100); return;
		case 'm': set(0b000,0b110,0b101,0b101,0b101); return;
		case 'B': set(0b110,0b101,0b110,0b101,0b110); return;
		case 'P': set(0b110,0b101,0b110,0b100,0b100); return;
		case 'M': set(0b101,0b111,0b111,0b101,0b101); return;
		case ' ': set(0b000,0b000,0b000,0b000,0b000); return;
		default: break;
	}
}

static void DrawPlayTinyChar(int x, int y, char c, bool on)
{
	uint8_t rows[kPlayTinyH] = {};
	GetPlayTinyGlyph(c, rows);
	for (int yy = 0; yy < kPlayTinyH; ++yy)
	{
		const uint8_t row = rows[yy];
		for (int xx = 0; xx < kPlayTinyW; ++xx)
		{
			if ((row >> (kPlayTinyW - 1 - xx)) & 1)
			{
				const int px = x + xx;
				const int py = y + yy;
				if (px >= 0 && px < kDisplayW && py >= 0 && py < kDisplayH)
				{
					display.DrawPixel(px, py, on);
				}
			}
		}
	}
}

static void DrawPlayTinyText(int x, int y, const char* s, bool on)
{
	int cx = x;
	for (const char* p = s; *p; ++p)
	{
		DrawPlayTinyChar(cx, y, *p, on);
		cx += kPlayTinyW + kPlayTinySpacing;
	}
}

static void DrawPlayScreen()
{
	if (!play_screen_dirty)
	{
		return;
	}
	play_screen_dirty = false;

	display.Fill(false);

	char label[12];
	snprintf(label, sizeof(label), "%d bpM", kPlayBpm);
	const int label_x = 1;
	const int label_y = 2;
	DrawPlayTinyText(label_x, label_y, label, true);

	const int line_y2 = label_y + kPlayTinyH + 2;
	if (line_y2 < kDisplayH)
	{
		display.DrawLine(0, line_y2, kDisplayW - 1, line_y2, true);
	}
	const int line_y3 = line_y2 - 1;
	if (line_y3 >= 0 && line_y3 < kDisplayH)
	{
		display.DrawLine(0, line_y3, kDisplayW - 1, line_y3, true);
	}

	const int label_box_w = kPlayTinyW + 3;
	const int divider_w = 2;
	const int sequencer_x = label_box_w + divider_w;
	const int sequencer_w = kDisplayW - sequencer_x;
	if (line_y3 + 1 < kDisplayH && label_box_w < kDisplayW)
	{
		int x0 = label_box_w;
		int x1 = label_box_w + divider_w - 1;
		if (x1 >= kDisplayW)
		{
			x1 = kDisplayW - 1;
		}
		if (x0 <= x1)
		{
			display.DrawRect(x0, line_y3 + 1, x1, kDisplayH - 1, true, true);
		}
	}

	const int sections_start_y = line_y2 + 1;
	if (sections_start_y < kDisplayH)
	{
		const int sections_h = kDisplayH - sections_start_y;
		const int section_h = sections_h / 4;
		if (section_h > 0)
		{
			int section_lines[3] = {-1, -1, -1};
			for (int i = 0; i < 4; ++i)
			{
				const int y = sections_start_y + i * section_h + (section_h - kPlayTinyH) / 2;
				if (y >= sections_start_y && (y + kPlayTinyH) <= kDisplayH)
				{
					char label_num[2] = {static_cast<char>('1' + i), '\0'};
					DrawPlayTinyText(2, y, label_num, true);
				}
			}
			for (int i = 1; i < 4; ++i)
			{
				const int y = sections_start_y + i * section_h - 1;
				section_lines[i - 1] = y;
				if (y >= sections_start_y && y < kDisplayH)
				{
					display.DrawLine(0, y, kDisplayW - 1, y, true);
				}
			}

			for (int i = 0; i < 4; ++i)
			{
				const int y = sections_start_y + i * section_h + (section_h / 2);
				if (y < sections_start_y || y >= kDisplayH)
				{
					continue;
				}
				for (int px = sequencer_x; px < kDisplayW; px += 2)
				{
					display.DrawPixel(px, y, true);
				}
			}

			if (sequencer_w > 0)
			{
				const int bottom_line_y = sections_start_y + 4 * section_h - 1;
				if (bottom_line_y >= sections_start_y && bottom_line_y < kDisplayH)
				{
					display.DrawLine(0, bottom_line_y, kDisplayW - 1, bottom_line_y, true);
				}
				const int bottom_line_y2 = bottom_line_y + 1;
				if (bottom_line_y2 >= sections_start_y && bottom_line_y2 < kDisplayH)
				{
					display.DrawLine(0, bottom_line_y2, kDisplayW - 1, bottom_line_y2, true);
				}
				display.DrawLine(0, kDisplayH - 1, kDisplayW - 1, kDisplayH - 1, true);

				for (int step = 1; step < kPlayStepCount; ++step)
				{
					const int x = sequencer_x + (step * sequencer_w) / kPlayStepCount;
					if (x <= sequencer_x || x >= kDisplayW)
					{
						continue;
					}
					int hash_lines[5] = {line_y2,
										 section_lines[0],
										 section_lines[1],
										 section_lines[2],
										 bottom_line_y};
					for (int i = 0; i < 5; ++i)
					{
						const int y = hash_lines[i];
						if (y < 0 || y >= kDisplayH)
						{
							continue;
						}
						int y0 = y - 1;
						int h = 3;
						if (y0 < 0)
						{
							h += y0;
							y0 = 0;
						}
						if (y0 + h > kDisplayH)
						{
							h = kDisplayH - y0;
						}
						if (h > 0)
						{
							display.DrawRect(x, y0, x, y0 + h - 1, true, true);
						}
					}
				}
				if (playhead_running)
				{
					int step = playhead_step;
					if (step < 0)
					{
						step = 0;
					}
					if (step >= kPlayStepCount)
					{
						step = kPlayStepCount - 1;
					}
					const int step_left = sequencer_x + (step * sequencer_w) / kPlayStepCount;
					const int step_right = sequencer_x + ((step + 1) * sequencer_w) / kPlayStepCount;
					int step_x = (step_left + step_right) / 2;
					if (step_x < sequencer_x)
					{
						step_x = sequencer_x;
					}
					if (step_x >= kDisplayW)
					{
						step_x = kDisplayW - 1;
					}

					auto draw_px = [&](int px, int py)
					{
						if (px >= 0 && px < kDisplayW && py >= 0 && py < kDisplayH)
						{
							display.DrawPixel(px, py, true);
						}
					};

					auto draw_line_triangle = [&](int cx, int line_y, int dir)
					{
						if (line_y < 0 || line_y >= kDisplayH)
						{
							return;
						}
						const int y1 = line_y + dir;
						const int y2 = line_y + (dir * 2);
						for (int dx = -2; dx <= 2; ++dx)
						{
							draw_px(cx + dx, line_y);
						}
						for (int dx = -1; dx <= 1; ++dx)
						{
							draw_px(cx + dx, y1);
						}
						draw_px(cx, y2);
					};

					for (int i = 0; i < 4; ++i)
					{
						const int top_line = (i == 0) ? line_y2 : section_lines[i - 1];
						const int bottom_line = (i == 3) ? bottom_line_y : section_lines[i];
						draw_line_triangle(step_x, top_line, 1);
						draw_line_triangle(step_x, bottom_line, -1);
					}
				}
			}
		}
	}

	display.Update();
}

static void DrawWaveform()
{
	if(!sample_loaded || sample_length == 0)
	{
		display.Fill(false);
		display.SetCursor(0, 0);
		display.WriteString("NO SAMPLE LOADED", Font_6x8, true);
		display.SetCursor(0, (Font_6x8.FontHeight + 1) * 2);
		display.WriteString("PLEASE LOAD A", Font_6x8, true);
		display.SetCursor(0, (Font_6x8.FontHeight + 1) * 3);
		display.WriteString("SAMPLE FROM", Font_6x8, true);
		display.SetCursor(0, (Font_6x8.FontHeight + 1) * 4);
		display.WriteString("THE MAIN MENU", Font_6x8, true);
		display.Update();
		return;
	}
	if(!waveform_ready || !waveform_dirty)
		return;

	waveform_dirty = false;
	display.Fill(false);

	const int W = 128;
	const int H = 64;
	const int text_h = Font_6x8.FontHeight + 1;
	const int mid = text_h + (H - text_h) / 2;
	const bool baked_view = perform_bake_active && ui_mode == UiMode::Play && baked_window_valid;

	if (baked_view)
	{
		size_t view_start = baked_play_start;
		size_t view_end = baked_play_end;
		if (view_end > sample_length || view_end == 0)
		{
			view_end = sample_length;
		}
		if (view_end <= view_start)
		{
			view_start = 0;
			view_end = sample_length;
		}
		const size_t view_len = (view_end > view_start) ? (view_end - view_start) : 0;
		const float scale = 28.0f;
		for (int x = 0; x < W; ++x)
		{
			const size_t start = view_start + (view_len * static_cast<size_t>(x)) / static_cast<size_t>(W);
			size_t end = view_start + (view_len * static_cast<size_t>(x + 1)) / static_cast<size_t>(W);
			if (end <= start)
			{
				end = start + 1;
			}
			if (end > view_end)
			{
				end = view_end;
			}

			float minv = 1.0f;
			float maxv = -1.0f;
			for (size_t i = start; i < end; ++i)
			{
				int32_t sample = sample_buffer_l[i];
				if (sample_channels == 2)
				{
					sample = (sample + sample_buffer_r[i]) / 2;
				}
				const float s = static_cast<float>(sample) * kSampleScale;
				if (s < minv)
				{
					minv = s;
				}
				if (s > maxv)
				{
					maxv = s;
				}
			}

			int top = mid + static_cast<int>(maxv * scale);
			int bottom = mid + static_cast<int>(minv * scale);
			if (top > bottom)
			{
				const int tmp = top;
				top = bottom;
				bottom = tmp;
			}
			top = ClampI(top, text_h, H - 1);
			bottom = ClampI(bottom, text_h, H - 1);
			display.DrawLine(x, top, x, bottom, true);
		}

		const int top_y = text_h;
		const int bottom_y = H - 1;
		const int attack_x = ClampI(
			static_cast<int>(perform_attack_norm * static_cast<float>(W - 1)),
			0,
			W - 1);
		const int release_x = ClampI(
			static_cast<int>((1.0f - perform_release_norm) * static_cast<float>(W - 1)),
			0,
			W - 1);
		display.DrawLine(0, bottom_y, attack_x, top_y, true);
		display.DrawLine(W - 1, bottom_y, release_x, top_y, true);

		display.SetCursor(0, 0);
		display.WriteString(waveform_title ? waveform_title : loaded_sample_name, Font_6x8, true);
		display.Update();
		return;
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
		int top    = mid + waveform_min[x];
		int bottom = mid + waveform_max[x];

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

	if (ui_mode == UiMode::Edt && playback_active && sample_length > 1)
	{
		const float denom = static_cast<float>(sample_length - 1);
		float norm = playback_phase / denom;
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

	display.SetCursor(0, 0);
	display.WriteString(waveform_title ? waveform_title : loaded_sample_name, Font_6x8, true);

	display.Update();
}

static void DrawRecordReview()
{
	waveform_title = "RECORDED PLAYBACK";
	DrawWaveform();
	waveform_title = nullptr;
}

static void DrawEdtScreen()
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
	waveform_title = title;
	DrawWaveform();
	waveform_title = nullptr;
}

static void DrawVerticalFadersInRect(int x,
									 int y,
									 int w,
									 int h,
									 const char* const* labels,
									 const float* values,
									 int count,
									 bool select_active,
									 int selected_index,
									 const int* x_offsets = nullptr)
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
		display.DrawLine(rail_x, line_top, rail_x, line_bottom, line_on);
		display.DrawLine(rail_x - 1, line_top, rail_x + 1, line_top, line_on);
		display.DrawLine(rail_x - 1, line_bottom, rail_x + 1, line_bottom, line_on);

		const float value = values[f];
		int tick_y = line_bottom - static_cast<int>(value * static_cast<float>(span_y) + 0.5f);
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

static void DrawFxDetailScreen(int32_t index)
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
	if (index == kFxChorusIndex)
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
		const bool chorus_selected = (chorus_mode == 0);
		const bool tape_selected = (chorus_mode == 1);
		display.DrawRect(block_x,
						 block_y,
						 block_x + block_w - 1,
						 block_y + box_h - 1,
						 true,
						 chorus_selected);
		display.DrawRect(block_x,
						 block_y + box_h + kGap,
						 block_x + block_w - 1,
						 block_y + (box_h * 2) + kGap - 1,
						 true,
						 tape_selected);
		const int label_w1 = TinyStringWidth("CHRS");
		const int label_w2 = TinyStringWidth("TAPE");
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
		DrawTinyString("CHRS", label_x1, label_y1, !chorus_selected);
		DrawTinyString("TAPE", label_x2, label_y2, !tape_selected);

		const int fader_offset = 8;
		const int fader_x = block_x + block_w + kGap + fader_offset;
		const int fader_w = kDisplayW - fader_x - kMargin;
		if (fader_w > 4)
		{
			const char* fader_labels[2]
				= {(chorus_mode == 1) ? "DROP" : "DEPTH",
				   (chorus_mode == 1) ? "RATE" : "SPEED"};
			const float fader_values[2]
				= {(chorus_mode == 1) ? chorus_wow : fx_c_wet,
				   (chorus_mode == 1) ? tape_rate : chorus_rate};
			int param_index = fx_detail_param_index;
			const bool fader_select_active = (param_index >= 0 && param_index < 2);
			if (!fader_select_active)
			{
				param_index = 0;
			}
			const int fader_offsets[2] = {0, 0};
			DrawVerticalFadersInRect(fader_x,
									 block_y,
									 fader_w,
									 block_h,
									 fader_labels,
									 fader_values,
									 2,
									 fader_select_active,
									 param_index,
									 fader_offsets);
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
			const char* fader_labels[kReverbFaderCount] = {"Pre", "Dmp", "Dcy", "Wet", "Shm"};
			const float fader_values[kReverbFaderCount]
				= {reverb_pre, reverb_damp, reverb_decay, reverb_wet, reverb_shimmer};
			int param_index = fx_detail_param_index;
			const bool fader_select_active
				= (param_index >= 0 && param_index < kReverbFaderCount);
			if (!fader_select_active)
			{
				param_index = 0;
			}
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
									 fader_offsets);
		}
	}
	display.Update();
}

static void DrawRecordTargetScreen(int32_t selected)
{
	display.Fill(false);
	constexpr int kMargin = 2;
	constexpr int kGap = 2;
	const int top_h = (kDisplayH - (kMargin * 2) - kGap) / 2;
	const int bottom_h = kDisplayH - (kMargin * 2) - kGap - top_h;
	const int top_y = kMargin;
	const int bottom_y = kMargin + top_h + kGap;
	const int top_w = (kDisplayW - (kMargin * 2) - kGap) / 2;
	const int left_x = kMargin;
	const int right_x = kMargin + top_w + kGap;
	const int bottom_w = kDisplayW - (kMargin * 2);

	auto draw_box = [&](int x, int y, int w, int h, const char* label, bool highlight)
	{
		display.DrawRect(x, y, x + w - 1, y + h - 1, true, highlight);
		const int len = static_cast<int>(StrLen(label));
		if (len <= 0)
		{
			return;
		}
		const int char_w = Font5x7::W + 1;
		const int text_w = (len - 1) * char_w + Font5x7::W;
		const int text_h = Font5x7::H;
		int text_x = x + (w - text_w) / 2;
		int text_y = y + (h - text_h) / 2;
		if (text_x < x + 1)
		{
			text_x = x + 1;
		}
		if (text_y < y + 1)
		{
			text_y = y + 1;
		}
		DrawTinyString(label, text_x, text_y, !highlight);
	};

	draw_box(left_x, top_y, top_w, top_h, "PLAY", selected == kRecordTargetPlay);
	draw_box(right_x, top_y, top_w, top_h, "BAKE", selected == kRecordTargetBake);
	draw_box(kMargin, bottom_y, bottom_w, bottom_h, "PERFORM", selected == kRecordTargetPerform);
	display.Update();
}

static void StartPlayback(uint8_t note)
{
	if (!sample_loaded || sample_length < 1)
	{
		if (kPlaybackVerboseLog && UiLogEnabled())
		{
			LogLine("Playback: start rejected (no sample)");
		}
		return;
	}
	size_t window_start = sample_play_start;
	size_t window_end = sample_play_end;
	if (perform_bake_active && ui_mode == UiMode::Play && baked_window_valid)
	{
		window_start = baked_play_start;
		window_end = baked_play_end;
	}
	if (window_end > sample_length || window_end == 0)
	{
		window_end = sample_length;
	}
	if (window_end <= window_start)
	{
		window_start = 0;
		window_end = sample_length;
	}
	if (window_end == 0)
	{
		if (kPlaybackVerboseLog && UiLogEnabled())
		{
			LogLine("Playback: start rejected (empty window)");
		}
		return;
	}
	current_note = note;
	playback_amp = 1.0f;
	playback_env_samples = 0;
	playback_release_active = false;
	playback_release_pos = 0.0f;
	playback_release_start = 0.0f;
	const float semis = static_cast<float>(note - kBaseMidiNote);
	const float pitch = powf(2.0f, semis / 12.0f);
	playback_rate = pitch * (static_cast<float>(sample_rate) / hw.AudioSampleRate());
	playback_phase = static_cast<float>(window_start);
	playback_active = true;
	if (kPlaybackVerboseLog && UiLogEnabled())
	{
		LogLine("Playback: start note=%u pitch=%.3f win=[%lu,%lu) rate=%.6f apply_pitch=1",
				static_cast<unsigned>(note),
				static_cast<double>(pitch),
				static_cast<unsigned long>(window_start),
				static_cast<unsigned long>(window_end),
				static_cast<double>(playback_rate));
	}
}

static void StartPlayback(uint8_t note, bool apply_pitch)
{
	if (!sample_loaded || sample_length < 1)
	{
		if (kPlaybackVerboseLog && UiLogEnabled())
		{
			LogLine("Playback: start rejected (no sample)");
		}
		return;
	}
	size_t window_start = sample_play_start;
	size_t window_end = sample_play_end;
	if (perform_bake_active && ui_mode == UiMode::Play && baked_window_valid)
	{
		window_start = baked_play_start;
		window_end = baked_play_end;
	}
	if (window_end > sample_length || window_end == 0)
	{
		window_end = sample_length;
	}
	if (window_end <= window_start)
	{
		window_start = 0;
		window_end = sample_length;
	}
	if (window_end == 0)
	{
		if (kPlaybackVerboseLog && UiLogEnabled())
		{
			LogLine("Playback: start rejected (empty window)");
		}
		return;
	}
	current_note = note;
	playback_amp = 1.0f;
	playback_env_samples = 0;
	playback_release_active = false;
	playback_release_pos = 0.0f;
	playback_release_start = 0.0f;
	const float semis = apply_pitch ? static_cast<float>(note - kBaseMidiNote) : 0.0f;
	const float pitch = powf(2.0f, semis / 12.0f);
	playback_rate = pitch * (static_cast<float>(sample_rate) / hw.AudioSampleRate());
	playback_phase = static_cast<float>(window_start);
	playback_active = true;
	if (kPlaybackVerboseLog && UiLogEnabled())
	{
		LogLine("Playback: start note=%u pitch=%.3f win=[%lu,%lu) rate=%.6f apply_pitch=%d",
				static_cast<unsigned>(note),
				static_cast<double>(pitch),
				static_cast<unsigned long>(window_start),
				static_cast<unsigned long>(window_end),
				static_cast<double>(playback_rate),
				static_cast<int>(apply_pitch));
	}
	request_playhead_redraw = true;
}

static void StopPlayback(uint8_t note)
{
	if (note == current_note)
	{
		if (ui_mode == UiMode::Perform && playback_active)
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
			request_playback_stop_log = true;
		}
	}
}

static int BakeNoteIndexForMidi(int32_t note)
{
	if (note < kBakeFirstMidi || note > kBakeLastMidi)
	{
		return -1;
	}
	return note - kBakeFirstMidi;
}

static void StartPerformVoice(int32_t note, bool use_baked)
{
	int index = -1;
	size_t window_start = 0;
	size_t window_end = 0;
	if (use_baked)
	{
		index = BakeNoteIndexForMidi(note);
		if (index < 0 || !baked_ready || !baked_note_ready[index] || baked_length == 0)
		{
			return;
		}
	}
	else
	{
		if (!sample_loaded || sample_length == 0)
		{
			return;
		}
		window_start = sample_play_start;
		window_end = sample_play_end;
		if (window_end > sample_length || window_end == 0)
		{
			window_end = sample_length;
		}
		if (window_end <= window_start)
		{
			window_start = 0;
			window_end = sample_length;
		}
		if (window_end <= window_start)
		{
			return;
		}
	}

	int voice_index = -1;
	for (int i = 0; i < kPerformVoiceCount; ++i)
	{
		if (perform_voices[i].active && perform_voices[i].note == note)
		{
			voice_index = i;
			break;
		}
	}
	if (voice_index < 0)
	{
		for (int i = 0; i < kPerformVoiceCount; ++i)
		{
			if (!perform_voices[i].active)
			{
				voice_index = i;
				break;
			}
		}
	}
	if (voice_index < 0)
	{
		voice_index = 0;
	}

	PerformVoice& voice = perform_voices[voice_index];
	voice.active = true;
	voice.releasing = false;
	voice.note = note;
	voice.phase = 0.0f;
	voice.amp = 1.0f;
	voice.env = 0.0f;
	voice.release_start = 0.0f;
	voice.release_pos = 0.0f;
	voice.env_samples = 0;
	perform_lpf_l1[voice_index].Reset();
	perform_lpf_l2[voice_index].Reset();
	perform_lpf_r1[voice_index].Reset();
	perform_lpf_r2[voice_index].Reset();
	const float sr = (sample_rate == 0) ? 48000.0f : static_cast<float>(sample_rate);
	if (use_baked)
	{
		voice.rate = sr / hw.AudioSampleRate();
		voice.offset = baked_note_offsets[index];
		voice.length = baked_length;
	}
	else
	{
		const float semis = static_cast<float>(note - kBaseMidiNote);
		const float pitch = powf(2.0f, semis / 12.0f);
		voice.rate = pitch * (sr / hw.AudioSampleRate());
		voice.offset = window_start;
		voice.length = window_end - window_start;
	}
}

static void StopPerformVoice(int32_t note)
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

static void HandleMidiMessage(MidiEvent msg)
{
	if (ui_mode == UiMode::Record && record_state == RecordState::Recording)
	{
		return;
	}
	if (ui_mode == UiMode::Perform)
	{
		const bool use_baked = perform_bake_active && baked_ready;
		switch (msg.type)
		{
			case NoteOn:
			{
				const NoteOnEvent note = msg.AsNoteOn();
				if (note.velocity == 0)
				{
					StopPerformVoice(note.note);
				}
				else
				{
					StartPerformVoice(note.note, use_baked);
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
	if (perform_bake_active && baked_ready && ui_mode == UiMode::Play)
	{
		switch (msg.type)
		{
			case NoteOn:
			{
				const NoteOnEvent note = msg.AsNoteOn();
				if (note.velocity == 0)
				{
					StopPerformVoice(note.note);
				}
				else
				{
					StartPerformVoice(note.note, true);
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
				StopPlayback(note.note);
			}
			else
			{
				StartPlayback(note.note);
			}
		}
		break;
		case NoteOff:
	{
		const NoteOffEvent note = msg.AsNoteOff();
		StopPlayback(note.note);
	}
	break;
	default:
		break;
	}
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	hw.ProcessAllControls();
	const int32_t encoder_l_inc = hw.encoder.Increment();
	const bool encoder_l_pressed = hw.encoder.RisingEdge();
	encoder_r.Debounce();
	const int32_t encoder_r_inc = encoder_r.Increment();
	if (encoder_r_inc != 0)
	{
		encoder_r_accum += encoder_r_inc;
	}
	const bool encoder_r_pressed = encoder_r.RisingEdge();
	bool encoder_r_consumed = false;
	if (encoder_r_pressed)
	{
		encoder_r_button_press = true;
	}
	const bool ui_blocked = (sd_init_in_progress || save_in_progress || bake_in_progress);
	shift_button.Debounce();
	const bool shift_pressed = shift_button.Pressed();
	const bool fx_shift_active = (ui_mode == UiMode::Perform
		&& perform_index == kPerformFxIndex
		&& shift_pressed);
	const bool amp_shift_active = (ui_mode == UiMode::Perform
		&& perform_index == kPerformAmpIndex
		&& shift_pressed);
	const bool flt_shift_active = (ui_mode == UiMode::Perform
		&& perform_index == kPerformFltIndex
		&& shift_pressed);
	if (fx_shift_active && !fx_shift_active_prev)
	{
		request_perform_redraw = true;
	}
	if (!fx_shift_active && fx_shift_active_prev)
	{
		request_perform_redraw = true;
	}
	fx_shift_active_prev = fx_shift_active;
	if (amp_shift_active && !amp_shift_active_prev)
	{
		request_perform_redraw = true;
	}
	if (!amp_shift_active && amp_shift_active_prev)
	{
		request_perform_redraw = true;
	}
	amp_shift_active_prev = amp_shift_active;
	if (flt_shift_active && !flt_shift_active_prev)
	{
		request_perform_redraw = true;
	}
	if (!flt_shift_active && flt_shift_active_prev)
	{
		request_perform_redraw = true;
	}
	flt_shift_active_prev = flt_shift_active;
	if (hw.button1.RisingEdge())
	{
		button1_press = true;
	}
	if (hw.button2.RisingEdge())
	{
		button2_press = true;
	}
	preview_hold = (ui_mode == UiMode::Load && !delete_mode) ? hw.button1.Pressed() : false;
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
		if (encoder_r_pressed)
		{
			LogLine("Shift menu select: %s", kShiftMenuLabels[shift_menu_index]);
			if (shift_menu_index == 0)
			{
				save_in_progress = true;
				save_done = false;
				save_success = false;
				save_started = false;
				save_prev_mode = shift_prev_mode;
				save_start_ms = System::GetNow();
				save_result_until_ms = 0;
				save_draw_next_ms = 0;
				save_filename[0] = '\0';
				ResetSaveState();
			}
			else if (shift_menu_index == 1)
			{
				delete_mode = true;
				delete_confirm = false;
				delete_prev_mode = shift_prev_mode;
				ui_mode = UiMode::Load;
				load_selected = 0;
				load_scroll = 0;
				request_delete_scan = true;
				request_delete_redraw = true;
			}
			if (!sd_init_in_progress && shift_menu_index == 0)
			{
				ui_mode = shift_prev_mode;
				request_shift_redraw = true;
			}
		}
		if (encoder_l_pressed)
		{
			if (!sd_init_in_progress)
			{
				ui_mode = shift_prev_mode;
				request_shift_redraw = true;
			}
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
			ui_mode = UiMode::Load;
			load_selected = 0;
			load_scroll = 0;
			request_load_scan = true;
		}
		else if (encoder_r_pressed && menu_index == 1)
		{
			ui_mode = UiMode::Record;
			record_source_index = (record_input == RecordInput::Mic) ? 1 : 0;
			record_state = RecordState::SourceSelect;
			record_pos = 0;
			playback_active = false;
			record_anim_start_ms = NowMs();
			LogLine("Record: entered (monitor ON), max %ld s @48k mono",
					static_cast<long>(kRecordMaxSeconds));
		}
		else if (encoder_r_pressed && menu_index == 2)
		{
			ui_mode = UiMode::Perform;
		}
		else if (encoder_r_pressed && menu_index == 3)
		{
			ui_mode = UiMode::Play;
			if(sample_loaded && sample_length > 0)
			{
				waveform_ready = true;
			}
			waveform_dirty = true;
			request_length_redraw = true;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::Load)
	{
		if (delete_mode && delete_confirm)
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
				else
				{
					load_target_selected = LoadDestination::Play;
					load_target_index = load_selected;
					ui_mode = UiMode::LoadTarget;
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
					ui_mode = UiMode::Main;
				}
			}
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::LoadTarget)
	{
		if (encoder_l_inc != 0)
		{
			int32_t next = LoadTargetDisplayIndex(load_target_selected) + encoder_l_inc;
			while (next < 0)
			{
				next += kLoadTargetCount;
			}
			while (next >= kLoadTargetCount)
			{
				next -= kLoadTargetCount;
			}
			load_target_selected = LoadTargetFromDisplayIndex(next);
		}
		if (encoder_r_pressed && load_target_index >= 0 && load_target_index < wav_file_count)
		{
			request_load_destination = load_target_selected;
			request_load_sample = true;
			request_load_index = load_target_index;
		}
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Load;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::ConfirmBake)
	{
		if (encoder_l_inc != 0)
		{
			int32_t next = confirm_bake_selected + encoder_l_inc;
			while (next < 0)
			{
				next += 2;
			}
			while (next >= 2)
			{
				next -= 2;
			}
			confirm_bake_selected = next;
			request_confirm_bake_redraw = true;
		}
		if (encoder_r_pressed)
		{
			if (confirm_bake_selected == 0 && sample_loaded)
			{
				size_t window_start = sample_play_start;
				size_t window_end = sample_play_end;
				if (window_end > sample_length || window_end == 0)
				{
					window_end = sample_length;
				}
				if (window_end <= window_start)
				{
					window_start = 0;
					window_end = sample_length;
				}
				if (window_end > window_start)
				{
					baked_play_start = window_start;
					baked_play_end = window_end;
					baked_window_valid = true;
					baked_ready = false;
					baked_length = 0;
					perform_bake_active = false;
					request_bake_process = true;
					bake_in_progress = true;
				}
				else
				{
					baked_window_valid = false;
					perform_bake_active = false;
					ui_mode = confirm_bake_prev_mode;
				}
				waveform_dirty = true;
			}
			else
			{
				ui_mode = confirm_bake_prev_mode;
				waveform_dirty = true;
			}
		}
		if (encoder_l_pressed)
		{
			ui_mode = confirm_bake_prev_mode;
			waveform_dirty = true;
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
				LogLine("Record input set: %s",
						(record_input == RecordInput::Mic) ? "MIC (R)" : "LINE IN (L)");
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
				sample_length = record_pos;
				sample_channels = 1;
				sample_rate = 48000;
				sample_loaded = (sample_length > 0);
				playback_active = false;
				trim_start = 0.0f;
				trim_end = 1.0f;
				waveform_from_recording = true;
				record_state = RecordState::Review;
				record_waveform_pending = true;
				if (sample_loaded)
				{
					ComputeWaveform();
					waveform_ready = true;
					waveform_dirty = true;
					UpdateTrimFrames();
					request_length_redraw = true;
				}
				LogLine("Record: stop, frames=%lu", static_cast<unsigned long>(sample_length));
			}
			else if (record_state == RecordState::Review && sample_loaded)
			{
				if (waveform_from_recording)
				{
					record_state = RecordState::TargetSelect;
				}
				else
				{
					confirm_bake_prev_mode = ui_mode;
					confirm_bake_selected = 0;
					ui_mode = UiMode::ConfirmBake;
					request_confirm_bake_redraw = true;
				}
			}
			else if (record_state == RecordState::TargetSelect)
			{
				if (record_target_index == kRecordTargetPerform)
				{
					ui_mode = UiMode::Perform;
					request_perform_redraw = true;
				}
				else if (record_target_index == kRecordTargetPlay)
				{
					ui_mode = UiMode::Play;
					if (sample_loaded && sample_length > 0)
					{
						waveform_ready = true;
					}
					waveform_dirty = true;
					request_length_redraw = true;
				}
				else
				{
					confirm_bake_prev_mode = UiMode::Record;
					confirm_bake_selected = 0;
					ui_mode = UiMode::ConfirmBake;
					request_confirm_bake_redraw = true;
				}
			}
		}
		if (encoder_l_pressed)
		{
			if (record_state == RecordState::TargetSelect)
			{
				record_state = RecordState::Review;
				waveform_dirty = true;
			}
			else
			{
				ui_mode = UiMode::Main;
				record_state = RecordState::SourceSelect;
				playback_active = false;
				record_anim_start_ms = -1.0;
			}
		}
		if (record_state == RecordState::Review && sample_loaded)
		{
			if(encoder_l_inc != 0 || encoder_r_inc != 0)
			{
				int32_t dl = encoder_l_inc;
				int32_t dr = encoder_r_inc;
				if (shift_button.Pressed())
				{
					if (dl > 0) dl = 1;
					else if (dl < 0) dl = -1;
					if (dr > 0) dr = 1;
					else if (dr < 0) dr = -1;
				}
				AdjustTrimNormalized(dl, dr, shift_button.Pressed());
			}
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::Perform)
	{
		if (encoder_l_inc != 0)
		{
			if (fx_shift_active)
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
			else if (amp_shift_active)
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
			else if (flt_shift_active)
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
				int32_t next = perform_index + encoder_l_inc;
				while (next < 0)
				{
					next += kPerformBoxCount;
				}
				while (next >= kPerformBoxCount)
				{
					next -= kPerformBoxCount;
				}
				perform_index = next;
			}
		}
		if (!fx_shift_active && !amp_shift_active && !flt_shift_active
			&& encoder_r_pressed && perform_index == kPerformEdtIndex)
		{
			ui_mode = UiMode::Edt;
			if (sample_loaded && sample_length > 0)
			{
				waveform_ready = true;
			}
			waveform_dirty = true;
			request_length_redraw = true;
		}
		if (fx_shift_active && encoder_r_pressed)
		{
			fx_detail_index = fx_fader_index;
			fx_detail_param_index = 0;
			if (fx_detail_index == kFxReverbIndex)
			{
				if (!reverb_params_initialized)
				{
					reverb_pre = 0.0f;
					reverb_damp = 0.0f;
					reverb_decay = 0.0f;
					reverb_wet = 0.0f;
					reverb_shimmer = 0.0f;
					reverb_params_initialized = true;
					fx_params_dirty = true;
				}
			}
			else if (fx_detail_index == kFxChorusIndex)
			{
				if (!mod_params_initialized)
				{
					fx_c_wet = 0.0f;
					chorus_rate = 0.0f;
					chorus_wow = 0.0f;
					tape_rate = 0.0f;
					chorus_mode = 0;
					mod_params_initialized = true;
					fx_params_dirty = true;
				}
			}
			ui_mode = UiMode::FxDetail;
			request_fx_detail_redraw = true;
		}
		if (fx_shift_active && encoder_r_inc != 0)
		{
			const float* steps[kPerformFaderCount]
				= {&kDelayWetStep, &kDelayWetStep, &kDelayWetStep, &kReverbWetStep};
			volatile float* targets[kPerformFaderCount]
				= {&fx_s_wet, &fx_c_wet, &delay_wet, &reverb_wet};
			const int idx = fx_fader_index;
			const float step = *steps[idx];
			volatile float* target = targets[idx];
			const float current = *target;
			float next = current + (static_cast<float>(encoder_r_inc) * step);
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
		else if (amp_shift_active && encoder_r_inc != 0)
		{
			const float step = kAmpEnvStep;
			volatile float* targets[kPerformFaderCount]
				= {&amp_attack, &amp_decay, &amp_sustain, &amp_release};
			const int idx = amp_fader_index;
			volatile float* target = targets[idx];
			const float current = *target;
			float next = current + (static_cast<float>(encoder_r_inc) * step);
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
		else if (flt_shift_active && encoder_r_inc != 0)
		{
			const float step = kFltParamStep;
			volatile float* targets[kPerformFltFaderCount] = {&flt_cutoff, &flt_res};
			const int idx = flt_fader_index;
			volatile float* target = targets[idx];
			const float current = *target;
			float next = current + (static_cast<float>(encoder_r_inc) * step);
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
			ui_mode = UiMode::Main;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::Edt)
	{
		if (encoder_l_inc != 0 || encoder_r_inc != 0)
		{
			int32_t dl = encoder_l_inc;
			int32_t dr = encoder_r_inc;
			if (shift_button.Pressed())
			{
				if (dl > 0) dl = 1;
				else if (dl < 0) dl = -1;
				if (dr > 0) dr = 1;
				else if (dr < 0) dr = -1;
			}
			AdjustTrimNormalized(dl, dr, shift_button.Pressed());
		}
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Perform;
			request_perform_redraw = true;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::FxDetail)
	{
		if (fx_detail_index == kFxChorusIndex)
		{
			if (encoder_r_pressed)
			{
				chorus_mode = (chorus_mode == 0) ? 1 : 0;
				request_fx_detail_redraw = true;
				fx_params_dirty = true;
			}
			if (encoder_l_inc != 0)
			{
				const int32_t param_count = 2;
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
				const float steps[2]
					= {(chorus_mode == 1) ? kChorusRateStep : kReverbWetStep,
					   kChorusRateStep};
				volatile float* targets[2]
					= {(chorus_mode == 1) ? &chorus_wow : &fx_c_wet,
					   (chorus_mode == 1) ? &tape_rate : &chorus_rate};
				const int idx = fx_detail_param_index;
				if (idx >= 0 && idx < 2)
				{
					const float step = steps[idx];
					volatile float* target = targets[idx];
					const float current = *target;
					float next = current + (static_cast<float>(encoder_r_inc) * step);
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
					   kReverbWetStep,
					   kReverbShimmerStep};
				volatile float* targets[kReverbFaderCount]
					= {&reverb_pre, &reverb_damp, &reverb_decay, &reverb_wet, &reverb_shimmer};
				const int idx = fx_detail_param_index;
				if (idx >= 0 && idx < kReverbFaderCount)
				{
					const float step = steps[idx];
					volatile float* target = targets[idx];
					const float current = *target;
					float next = current + (static_cast<float>(encoder_r_inc) * step);
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
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Perform;
			request_perform_redraw = true;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::Play)
	{
		if (perform_bake_active)
		{
			const bool fine = shift_button.Pressed();
			if (encoder_l_inc != 0)
			{
				AdjustPerformFade(perform_attack_norm, encoder_l_inc, fine);
				waveform_dirty = true;
			}
			if (encoder_r_inc != 0)
			{
				AdjustPerformFade(perform_release_norm, encoder_r_inc, fine);
				waveform_dirty = true;
			}
		}
		else if (encoder_l_inc != 0 || encoder_r_inc != 0)
		{
			int32_t dl = encoder_l_inc;
			int32_t dr = encoder_r_inc;
			if (shift_button.Pressed())
			{
				if (dl > 0) dl = 1;
				else if (dl < 0) dl = -1;
				if (dr > 0) dr = 1;
				else if (dr < 0) dr = -1;
			}
			AdjustTrimNormalized(dl, dr, shift_button.Pressed());
		}
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Main;
		}
	}
	else
	{
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Main;
		}
	}

	if (record_state == RecordState::Countdown)
	{
		if ((System::GetNow() - record_countdown_start_ms) >= kRecordCountdownMs)
		{
			StartRecording();
		}
	}

	size_t window_start = sample_play_start;
	size_t window_end = sample_play_end;
	if (perform_bake_active && ui_mode == UiMode::Play && baked_window_valid)
	{
		window_start = baked_play_start;
		window_end = baked_play_end;
	}
	if (window_end > sample_length || window_end == 0)
	{
		window_end = sample_length;
	}
	if (window_end <= window_start)
	{
		window_start = 0;
		window_end = sample_length;
	}
	const bool window_valid = (window_end > 0 && window_end > window_start);
	if (playback_active && !window_valid)
	{
		playback_active = false;
		playback_env_samples = 0;
		playback_release_active = false;
		playback_release_pos = 0.0f;
		playback_release_start = 0.0f;
	}
	const bool baked_valid = baked_ready && baked_length > 0;
	if (perform_bake_active && !baked_valid)
	{
		for (auto &voice : perform_voices)
		{
			voice.active = false;
		}
	}
	static float cached_sat_depth = 0.0f;
	static float cached_chorus_depth = 0.0f;
	static float cached_chorus_rate = 0.0f;
	static int32_t cached_chorus_mode = 0;
	static float cached_chorus_wow = 0.0f;
	static float cached_tape_rate = 0.0f;
	static float cached_delay_wet = 0.0f;
	static float cached_reverb_wet = 0.0f;
	static float cached_reverb_pre = 0.0f;
	static float cached_reverb_damp = 0.0f;
	static float cached_reverb_decay = 0.0f;
	static float cached_reverb_shimmer = 0.0f;
	static float cached_reverb_gain = 1.0f;
	static float cached_reverb_release = 1.0f;
	static float cached_reverb_predelay_samples = 0.0f;
	static float last_sat_depth = -1.0f;
	static float last_chorus_depth = -1.0f;
	static float last_chorus_rate = -1.0f;
	static float last_chorus_wow = -1.0f;
	static float last_delay_wet = -1.0f;
	static float last_rev_feedback = -1.0f;
	static float last_rev_lp = -1.0f;
	static float last_rev_predelay = -1.0f;
	const float out_sr = hw.AudioSampleRate();

	if (fx_params_dirty)
	{
		fx_params_dirty = false;

		cached_sat_depth = fx_s_wet;
		cached_chorus_depth = fx_c_wet;
		cached_chorus_rate = chorus_rate;
		cached_chorus_mode = chorus_mode;
		cached_chorus_wow = chorus_wow;
		cached_tape_rate = tape_rate;
		cached_delay_wet = delay_wet;
		cached_reverb_wet = reverb_wet;
		cached_reverb_pre = reverb_pre;
		cached_reverb_damp = reverb_damp;
		cached_reverb_decay = reverb_decay;
		cached_reverb_shimmer = reverb_shimmer;

		if (cached_sat_depth < 0.0f) cached_sat_depth = 0.0f;
		if (cached_sat_depth > 1.0f) cached_sat_depth = 1.0f;
		if (cached_chorus_depth < 0.0f) cached_chorus_depth = 0.0f;
		if (cached_chorus_depth > 1.0f) cached_chorus_depth = 1.0f;
		if (cached_chorus_rate < 0.0f) cached_chorus_rate = 0.0f;
		if (cached_chorus_rate > 1.0f) cached_chorus_rate = 1.0f;
		if (cached_chorus_wow < 0.0f) cached_chorus_wow = 0.0f;
		if (cached_chorus_wow > 1.0f) cached_chorus_wow = 1.0f;
		if (cached_tape_rate < 0.0f) cached_tape_rate = 0.0f;
		if (cached_tape_rate > 1.0f) cached_tape_rate = 1.0f;
		if (cached_delay_wet < 0.0f) cached_delay_wet = 0.0f;
		if (cached_delay_wet > 1.0f) cached_delay_wet = 1.0f;
		if (cached_reverb_wet < 0.0f) cached_reverb_wet = 0.0f;
		if (cached_reverb_wet > 1.0f) cached_reverb_wet = 1.0f;
		if (cached_reverb_pre < 0.0f) cached_reverb_pre = 0.0f;
		if (cached_reverb_pre > 1.0f) cached_reverb_pre = 1.0f;
		if (cached_reverb_damp < 0.0f) cached_reverb_damp = 0.0f;
		if (cached_reverb_damp > 1.0f) cached_reverb_damp = 1.0f;
		if (cached_reverb_decay < 0.0f) cached_reverb_decay = 0.0f;
		if (cached_reverb_decay > 1.0f) cached_reverb_decay = 1.0f;
		if (cached_reverb_shimmer < 0.0f) cached_reverb_shimmer = 0.0f;
		if (cached_reverb_shimmer > 1.0f) cached_reverb_shimmer = 1.0f;

		cached_reverb_gain = 1.0f;
		const float decay_ms = kReverbDecayMinMs
			+ cached_reverb_decay * cached_reverb_decay * (kReverbDecayMaxMs - kReverbDecayMinMs);
		const float decay_samples = decay_ms * 0.001f * out_sr;
		if (cached_reverb_decay >= 0.999f)
		{
			cached_reverb_release = 1.0f;
		}
		else if (decay_samples > 1.0f)
		{
			cached_reverb_release = expf(-1.0f / decay_samples);
		}
		else
		{
			cached_reverb_release = 0.0f;
		}

		if (fabsf(cached_sat_depth - last_sat_depth) > kFxParamEpsilon)
		{
			sat_l.SetDrive(cached_sat_depth);
			sat_r.SetDrive(cached_sat_depth);
			sat_l.SetMix(cached_sat_depth);
			sat_r.SetMix(cached_sat_depth);
			last_sat_depth = cached_sat_depth;
		}
		if (fabsf(cached_chorus_depth - last_chorus_depth) > kFxParamEpsilon)
		{
			if (cached_chorus_mode == 0)
			{
				const float depth_curve = cached_chorus_depth * cached_chorus_depth;
				const float depth_scale = kChorusMaxDepth * 1.2f;
				const float depth = depth_curve * depth_scale;
				chorus_l.SetLfoDepth(depth);
				chorus_r.SetLfoDepth(depth);
			}
			last_chorus_depth = cached_chorus_depth;
		}
		if (fabsf(cached_chorus_rate - last_chorus_rate) > kFxParamEpsilon)
		{
			const float rate_curve = cached_chorus_rate * cached_chorus_rate;
			const float rate_hz = kChorusRateMinHz
				+ rate_curve * (kChorusRateMaxHz - kChorusRateMinHz);
			chorus_l.SetLfoFreq(rate_hz);
			chorus_r.SetLfoFreq(-rate_hz);
			last_chorus_rate = cached_chorus_rate;
		}
		if (fabsf(cached_chorus_wow - last_chorus_wow) > kFxParamEpsilon)
		{
			last_chorus_wow = cached_chorus_wow;
		}
		if (fabsf(cached_delay_wet - last_delay_wet) > kFxParamEpsilon)
		{
			last_delay_wet = cached_delay_wet;
		}

		float rev_feedback = kReverbFeedback;
		if (cached_reverb_decay >= 0.999f)
		{
			rev_feedback = 0.99f;
		}
		const float damp_curve = cached_reverb_damp * 1.6f;
		float rev_lp = kReverbDampMaxHz
			* powf(kReverbDampMinHz / kReverbDampMaxHz, damp_curve);
		const float rev_lp_max = out_sr * 0.49f;
		if (rev_lp > rev_lp_max)
		{
			rev_lp = rev_lp_max;
		}
		if (rev_lp < kReverbDampMinHz)
		{
			rev_lp = kReverbDampMinHz;
		}
		const float pre_curve = powf(cached_reverb_pre, 3.0f);
		float rev_predelay_samples
			= pre_curve * (kReverbPreDelayMaxMs * 0.001f * out_sr);
		const float rev_predelay_max = static_cast<float>(kReverbPreDelayMaxSamples - 1);
		if (rev_predelay_samples > rev_predelay_max)
		{
			rev_predelay_samples = rev_predelay_max;
		}
		cached_reverb_predelay_samples = rev_predelay_samples;
		if (fabsf(rev_feedback - last_rev_feedback) > kFxParamEpsilon)
		{
			reverb.SetFeedback(rev_feedback);
			last_rev_feedback = rev_feedback;
		}
		if (fabsf(rev_lp - last_rev_lp) > kFxParamEpsilon)
		{
			reverb.SetLpFreq(rev_lp);
			last_rev_lp = rev_lp;
		}
		if (fabsf(rev_predelay_samples - last_rev_predelay) >= 0.5f)
		{
			reverb_predelay_l.SetDelay(rev_predelay_samples);
			reverb_predelay_r.SetDelay(rev_predelay_samples);
			last_rev_predelay = rev_predelay_samples;
		}
	}

	static float drop_phase = 0.0f;
	static float trem_phase = 0.0f;
	static float drop_gain = 1.0f;
	static float drop_target = 1.0f;
	static int drop_hold = 0;
	static uint32_t drop_rng = 0x12345678;

	const float chorus_depth = cached_chorus_depth;
	const float delay_mix = cached_delay_wet;
	const float rev_shimmer = cached_reverb_shimmer;
	const bool perform_mode = (ui_mode == UiMode::Perform);
	const bool amp_env_active = perform_mode;
	const float amp_attack_ms = AmpEnvMsFromFader(amp_attack);
	const float amp_release_ms = AmpEnvMsFromFader(amp_release);
	const float amp_attack_samples = amp_attack_ms * 0.001f * out_sr;
	const float amp_release_samples = amp_release_ms * 0.001f * out_sr;
	const bool play_baked_mode = perform_bake_active && ui_mode == UiMode::Play && baked_ready;
	const bool use_baked = perform_bake_active && baked_ready && (perform_mode || play_baked_mode);
	const bool use_poly = (record_state != RecordState::Recording)
		&& ((perform_mode && sample_loaded) || play_baked_mode);
	const bool sample_stereo = (sample_channels == 2);
	const float flt_cutoff_hz = FltCutoffFromFader(flt_cutoff, out_sr);
	const float flt_q = FltQFromFader(flt_res);
	static float last_flt_cutoff = -1.0f;
	static float last_flt_q = -1.0f;
	if (flt_cutoff_hz != last_flt_cutoff || flt_q != last_flt_q)
	{
		for (int v = 0; v < kPerformVoiceCount; ++v)
		{
			perform_lpf_l1[v].Set(out_sr, flt_cutoff_hz, flt_q);
			perform_lpf_l2[v].Set(out_sr, flt_cutoff_hz, flt_q);
			perform_lpf_r1[v].Set(out_sr, flt_cutoff_hz, flt_q);
			perform_lpf_r2[v].Set(out_sr, flt_cutoff_hz, flt_q);
		}
		last_flt_cutoff = flt_cutoff_hz;
		last_flt_q = flt_q;
	}
	const float perform_attack = perform_attack_norm;
	const float perform_release = perform_release_norm;
	const float baked_attack_frames = (baked_valid && baked_length > 1)
		? perform_attack * static_cast<float>(baked_length - 1)
		: 0.0f;
	const float baked_release_frames = (baked_valid && baked_length > 1)
		? perform_release * static_cast<float>(baked_length - 1)
		: 0.0f;

	for (size_t i = 0; i < size; i++)
	{
		float sig_l = 0.0f;
		float sig_r = 0.0f;
		static float reverb_tail_gain = 0.0f;
		const bool monitor_active =
			(ui_mode == UiMode::Record
				&& record_state != RecordState::Review
				&& record_state != RecordState::TargetSelect);
		float monitor_l = 0.0f;
		float monitor_r = 0.0f;
		if (monitor_active)
		{
			const float monitor_sel = (record_input == RecordInput::Mic) ? in[1][i] : in[0][i];
			monitor_l = monitor_sel;
			monitor_r = monitor_sel;
		}
		if (record_state == RecordState::Recording)
	{
		if (record_pos < kRecordMaxFrames)
		{
			const float in_sel = (record_input == RecordInput::Mic) ? in[1][i] : in[0][i];
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
				if (abs_s > live_wave_peak)
				{
					live_wave_peak = abs_s;
				}
				const float norm = (live_wave_peak > 0)
					? (28.0f / (static_cast<float>(live_wave_peak) * kSampleScale))
					: 1.0f;
				const float s_scaled = static_cast<float>(samp) * kSampleScale * norm;
				int16_t s_pix = static_cast<int16_t>(s_scaled);
				const int32_t col = static_cast<int32_t>(
					(static_cast<uint64_t>(record_pos) * 128U) / kRecordMaxFrames);
				if (col >= 0 && col < 128)
				{
					if (col != live_wave_last_col)
					{
						live_wave_min[col] = s_pix;
						live_wave_max[col] = s_pix;
						live_wave_last_col = col;
						live_wave_dirty = true;
					}
					else
					{
						if (s_pix < live_wave_min[col]) live_wave_min[col] = s_pix;
						if (s_pix > live_wave_max[col]) live_wave_max[col] = s_pix;
					}
				}
				++record_pos;
			}
			if (record_pos >= kRecordMaxFrames)
			{
				sample_length = record_pos;
				sample_channels = 1;
				sample_rate = 48000;
				sample_loaded = (sample_length > 0);
				playback_active = false;
				request_playback_stop_log = true;
				trim_start = 0.0f;
				trim_end = 1.0f;
				waveform_from_recording = true;
				record_state = RecordState::Review;
				record_waveform_pending = true;
				if (sample_loaded)
				{
					ComputeWaveform();
					waveform_ready = true;
					waveform_dirty = true;
					UpdateTrimFrames();
					request_length_redraw = true;
				}
				LogLine("Record: auto-stop at max frames=%lu",
						static_cast<unsigned long>(sample_length));
			}
		}
		if (sample_loaded && playback_active && record_state != RecordState::Recording && window_valid)
		{
			float amp_env = 1.0f;
			if (amp_env_active)
			{
				float attack_env = 1.0f;
				if (amp_attack_samples > 1.0f)
				{
					attack_env = static_cast<float>(playback_env_samples) / amp_attack_samples;
					if (attack_env > 1.0f)
					{
						attack_env = 1.0f;
					}
				}
				float release_env = 1.0f;
				if (amp_release_samples > 1.0f && playback_rate > 0.0f)
				{
					float remaining = (static_cast<float>(window_end) - playback_phase) / playback_rate;
					if (remaining < 0.0f)
					{
						remaining = 0.0f;
					}
					release_env = remaining / amp_release_samples;
					if (release_env > 1.0f)
					{
						release_env = 1.0f;
					}
				}
				amp_env = (attack_env < release_env) ? attack_env : release_env;
				if (amp_env < 0.0f)
				{
					amp_env = 0.0f;
				}
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
			if (sample_length == 1)
			{
				sig_l = static_cast<float>(sample_buffer_l[0]) * kSampleScale * playback_amp;
				sig_r = static_cast<float>(sample_buffer_r[0]) * kSampleScale * playback_amp;
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
				if (idx + 1 < window_end)
				{
					const float frac = playback_phase - static_cast<float>(idx);
					const float l0 = static_cast<float>(sample_buffer_l[idx]);
					const float l1 = static_cast<float>(sample_buffer_l[idx + 1]);
					const float r0 = static_cast<float>(sample_buffer_r[idx]);
					const float r1 = static_cast<float>(sample_buffer_r[idx + 1]);
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
						request_playback_stop_log = true;
					}
				}
				else
				{
					playback_active = false;
					request_playback_stop_log = true;
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
						request_playback_stop_log = true;
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
					continue;
				}
				float env = 1.0f;
				if (amp_env_active)
				{
					if (amp_attack_samples > 1.0f)
					{
						env = static_cast<float>(voice.env_samples) / amp_attack_samples;
					}
				}
				else
				{
					if (baked_attack_frames > 0.0f && voice.phase < baked_attack_frames)
					{
						env = voice.phase / baked_attack_frames;
					}
					if (baked_release_frames > 0.0f)
					{
						const float rel_pos = static_cast<float>(voice.length - 1) - voice.phase;
						const float rel_env = rel_pos / baked_release_frames;
						if (rel_env < env)
						{
							env = rel_env;
						}
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
						noteoff_env *= (1.0f - (voice.release_pos / amp_release_samples));
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
				if (voice.length == 1)
				{
					const size_t idx = voice.offset;
					const float amp = voice.amp * env;
					float samp_l = 0.0f;
					float samp_r = 0.0f;
					if (use_baked)
					{
						samp_l = static_cast<float>(baked_buffer_l[idx]) * kSampleScale * amp;
						samp_r = static_cast<float>(baked_buffer_r[idx]) * kSampleScale * amp;
					}
					else
					{
						samp_l = static_cast<float>(sample_buffer_l[idx]) * kSampleScale * amp;
						const float r = sample_stereo
							? static_cast<float>(sample_buffer_r[idx])
							: static_cast<float>(sample_buffer_l[idx]);
						samp_r = r * kSampleScale * amp;
					}
					if (perform_mode)
					{
						samp_l = perform_lpf_l2[v].Process(perform_lpf_l1[v].Process(samp_l));
						samp_r = perform_lpf_r2[v].Process(perform_lpf_r1[v].Process(samp_r));
					}
					sig_l += samp_l;
					sig_r += samp_r;
					voice.active = false;
					voice.releasing = false;
					voice.release_pos = 0.0f;
					voice.env_samples = 0;
					continue;
				}
				const size_t idx_rel = static_cast<size_t>(voice.phase);
				if (idx_rel + 1 >= voice.length)
				{
					voice.active = false;
					voice.releasing = false;
					voice.release_pos = 0.0f;
					voice.env_samples = 0;
					continue;
				}
				const float frac = voice.phase - static_cast<float>(idx_rel);
				const size_t idx = voice.offset + idx_rel;
				float l0 = 0.0f;
				float l1 = 0.0f;
				float r0 = 0.0f;
				float r1 = 0.0f;
				if (use_baked)
				{
					l0 = static_cast<float>(baked_buffer_l[idx]);
					l1 = static_cast<float>(baked_buffer_l[idx + 1]);
					r0 = static_cast<float>(baked_buffer_r[idx]);
					r1 = static_cast<float>(baked_buffer_r[idx + 1]);
				}
				else
				{
					l0 = static_cast<float>(sample_buffer_l[idx]);
					l1 = static_cast<float>(sample_buffer_l[idx + 1]);
					if (sample_stereo)
					{
						r0 = static_cast<float>(sample_buffer_r[idx]);
						r1 = static_cast<float>(sample_buffer_r[idx + 1]);
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
				sig_l += samp_l;
				sig_r += samp_r;
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
						voice.active = false;
						voice.releasing = false;
						voice.release_pos = 0.0f;
						voice.env_samples = 0;
					}
				}
				if (voice.phase >= static_cast<float>(voice.length - 1))
				{
					voice.active = false;
					voice.releasing = false;
					voice.release_pos = 0.0f;
					voice.env_samples = 0;
				}
			}
		}
		if (preview_active)
		{
			size_t read_idx = preview_read_index;
			const size_t write_idx = preview_write_index;
			size_t available = PreviewAvailableFrames(read_idx, write_idx);
			if (available >= 2)
			{
				const size_t idx0 = read_idx;
				const size_t idx1 = (idx0 + 1) % kPreviewBufferFrames;
				const float frac = preview_read_frac;
				const float s0 = static_cast<float>(preview_buffer[idx0]);
				const float s1 = static_cast<float>(preview_buffer[idx1]);
				const float samp = (s0 + (s1 - s0) * frac) * kSampleScale;
				sig_l += samp;
				sig_r += samp;

				float next_frac = preview_read_frac + preview_rate;
				while (next_frac >= 1.0f && available > 0)
				{
					next_frac -= 1.0f;
					read_idx = (read_idx + 1) % kPreviewBufferFrames;
					available = PreviewAvailableFrames(read_idx, write_idx);
				}
				preview_read_frac = next_frac;
				preview_read_index = read_idx;
			}
		}
		if (monitor_active)
		{
			sig_l += monitor_l;
			sig_r += monitor_r;
		}
		sig_l = sat_l.Process(sig_l);
		sig_r = sat_r.Process(sig_r);
		const int32_t chorus_mode_local = cached_chorus_mode;
		float chorus_proc_l = chorus_l.Process(sig_l);
		float chorus_proc_r = chorus_r.Process(sig_r);
		float chorus_mix = (chorus_mode_local == 1) ? 1.0f : chorus_depth;
		float tape_drop = 1.0f;
		if (chorus_mode_local == 1)
		{
			const float drop_amt = cached_chorus_wow;
			if (drop_amt > 0.0f)
			{
				const float drop_amt_mapped = powf(drop_amt, 0.6f);
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
				const float trem = 0.5f * (1.0f + sinf(trem_phase * kTwoPi));
				const float trem_depth = drop_curve * 0.85f;
				tape_drop = drop_gain * (1.0f - trem_depth + (trem_depth * trem));
			}
		}
		float chorus_out_l = ((sig_l * (1.0f - chorus_mix)) + (chorus_proc_l * chorus_mix))
			* tape_drop;
		float chorus_out_r = ((sig_r * (1.0f - chorus_mix)) + (chorus_proc_r * chorus_mix))
			* tape_drop;
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
			const float mid = 0.5f * (chorus_out_l + chorus_out_r);
			const float side = 0.5f * (chorus_out_l - chorus_out_r);
			chorus_out_l = mid + (side * width);
			chorus_out_r = mid - (side * width);
		}

		const float delay_in = 0.5f * (chorus_out_l + chorus_out_r);
		const float delay_out_l = delay_line_l.Read();
		const float delay_out_r = delay_line_r.Read();
		// Ping-pong delay: mono input with cross-feedback for left/right bounce.
		delay_line_l.Write(delay_in + (delay_out_r * kDelayFeedback));
		delay_line_r.Write(delay_out_l * kDelayFeedback);
		const float delay_mix_l = (chorus_out_l * (1.0f - delay_mix)) + (delay_out_l * delay_mix);
		const float delay_mix_r = (chorus_out_r * (1.0f - delay_mix)) + (delay_out_r * delay_mix);

		float rev_in_l = 0.0f;
		float rev_in_r = 0.0f;
		const bool predelay_active = (cached_reverb_predelay_samples >= 1.0f);
		if (predelay_active)
		{
			rev_in_l = reverb_predelay_l.Read();
			rev_in_r = reverb_predelay_r.Read();
			reverb_predelay_l.Write(delay_mix_l);
			reverb_predelay_r.Write(delay_mix_r);
		}
		else
		{
			reverb_predelay_l.Write(delay_mix_l);
			reverb_predelay_r.Write(delay_mix_r);
			rev_in_l = delay_mix_l;
			rev_in_r = delay_mix_r;
		}
		const float rev_in_level = fabsf(delay_mix_l) + fabsf(delay_mix_r);
		if (rev_in_level > 1e-4f)
		{
			reverb_tail_gain = 1.0f;
		}
		else
		{
			reverb_tail_gain *= cached_reverb_release;
		}
		// Shimmer: pitch-shifted octave layer around midpoint (0.5 = no shimmer).
		float shimmer_amount = 0.0f;
		float shimmer_rate = 1.0f;
		int next_mode = 0;
		if (rev_shimmer > 0.0f)
		{
			shimmer_amount = rev_shimmer;
			if (shimmer_amount > 1.0f)
			{
				shimmer_amount = 1.0f;
			}
			if (shimmer_amount < 0.25f)
			{
				shimmer_amount *= 2.0f;
			}
			else
			{
				shimmer_amount = 0.5f + (shimmer_amount - 0.25f) * (0.5f / 0.75f);
			}
			if (shimmer_amount > 1.0f)
			{
				shimmer_amount = 1.0f;
			}
			shimmer_rate = kShimmerRateUp;
			next_mode = 1;
		}
		else
		{
			shimmer_amount = 0.0f;
			shimmer_rate = kShimmerRateUp;
			next_mode = 0;
		}

		if (next_mode != shimmer_mode)
		{
			shimmer_mode = next_mode;
			shimmer_read_idx = static_cast<float>(
				(shimmer_write_idx + kShimmerBufferSize - kShimmerDelaySamples)
				% kShimmerBufferSize);
		}

		float shimmer_fb_l = 0.0f;
		float shimmer_fb_r = 0.0f;
		if (shimmer_amount > 0.0f)
		{
			const size_t read_i0 = static_cast<size_t>(shimmer_read_idx) % kShimmerBufferSize;
			const size_t read_i1 = (read_i0 + 1) % kShimmerBufferSize;
			const float frac = shimmer_read_idx - static_cast<float>(read_i0);
			const float sh_l = shimmer_buf_l[read_i0]
				+ (shimmer_buf_l[read_i1] - shimmer_buf_l[read_i0]) * frac;
			const float sh_r = shimmer_buf_r[read_i0]
				+ (shimmer_buf_r[read_i1] - shimmer_buf_r[read_i0]) * frac;
			shimmer_fb_l = shimmer_amount * kShimmerFeedback * shimmer_hp_l.Process(sh_l);
			shimmer_fb_r = shimmer_amount * kShimmerFeedback * shimmer_hp_r.Process(sh_r);

			shimmer_read_idx += shimmer_rate;
			while (shimmer_read_idx >= static_cast<float>(kShimmerBufferSize))
			{
				shimmer_read_idx -= static_cast<float>(kShimmerBufferSize);
			}
			const size_t read_int = static_cast<size_t>(shimmer_read_idx);
			size_t dist = (shimmer_write_idx >= read_int)
				? (shimmer_write_idx - read_int)
				: (shimmer_write_idx + kShimmerBufferSize - read_int);
			if (dist < 4)
			{
				shimmer_read_idx = static_cast<float>(
					(shimmer_write_idx + kShimmerBufferSize - kShimmerDelaySamples)
					% kShimmerBufferSize);
			}
		}

		float rev_l = 0.0f;
		float rev_r = 0.0f;
		reverb.Process(rev_in_l + shimmer_fb_l, rev_in_r + shimmer_fb_r, &rev_l, &rev_r);
		rev_l *= cached_reverb_gain * reverb_tail_gain;
		rev_r *= cached_reverb_gain * reverb_tail_gain;

		shimmer_buf_l[shimmer_write_idx] = rev_l;
		shimmer_buf_r[shimmer_write_idx] = rev_r;
		shimmer_write_idx = (shimmer_write_idx + 1) % kShimmerBufferSize;
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
		out[0][i] = (delay_mix_l * dry_mix) + (rev_l * wet_mix);
		out[1][i] = (delay_mix_r * dry_mix) + (rev_r * wet_mix);
	}
	request_playhead_redraw = true;
}

int main(void)
{
	hw.Init();
	hw.SetAudioBlockSize(16); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.seed.StartLog(false);
	LogLine("Logger started");

	reverb.Init(hw.AudioSampleRate());
	reverb.SetFeedback(kReverbFeedback);
	reverb.SetLpFreq(kReverbLpFreq);
	shimmer_hp_l.Init(hw.AudioSampleRate(), kShimmerHpHz);
	shimmer_hp_r.Init(hw.AudioSampleRate(), kShimmerHpHz);
	for (size_t i = 0; i < kShimmerBufferSize; ++i)
	{
		shimmer_buf_l[i] = 0.0f;
		shimmer_buf_r[i] = 0.0f;
	}
	shimmer_write_idx = 0;
	shimmer_read_idx = static_cast<float>(kShimmerBufferSize - kShimmerDelaySamples);
	shimmer_mode = 0;

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
	sat_l.SetMix(0.0f);
	sat_r.SetMix(0.0f);

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
	chorus_rate = (kChorusRateHz - kChorusRateMinHz)
		/ (kChorusRateMaxHz - kChorusRateMinHz);
	chorus_wow = 0.0f;
	tape_rate = 0.0f;

	delay_line_l.Init();
	delay_line_r.Init();
	float delay_samples = kDelayTimeSec * hw.AudioSampleRate();
	const float max_delay = static_cast<float>(kDelayMaxSamples - 1);
	if (delay_samples > max_delay)
	{
		delay_samples = max_delay;
	}
	delay_line_l.SetDelay(delay_samples);
	delay_line_r.SetDelay(delay_samples);

	reverb_predelay_l.Init();
	reverb_predelay_r.Init();
	reverb_predelay_l.SetDelay(0.0f);
	reverb_predelay_r.SetDelay(0.0f);

	encoder_r.Init(seed::D7, seed::D8, seed::D22, hw.AudioSampleRate());
	shift_button.Init(seed::D9, 1000);

	PodDisplay::Config disp_cfg;
	disp_cfg.driver_config.transport_config.i2c_config.pin_config.scl = seed::D11;
	disp_cfg.driver_config.transport_config.i2c_config.pin_config.sda = seed::D12;
	disp_cfg.driver_config.transport_config.i2c_config.speed
		= I2CHandle::Config::Speed::I2C_400KHZ;
	disp_cfg.driver_config.transport_config.i2c_address = 0x3C;
	display.Init(disp_cfg);
	InitLoadLayout();

	SdmmcHandler::Config sd_cfg;
	sd_cfg.Defaults();
	sdcard.Init(sd_cfg);
	fsi.Init(FatFSInterface::Config::MEDIA_SD);
	MountSd();

	DrawMenu(menu_index);

	hw.StartAdc();
	hw.StartAudio(AudioCallback);
	hw.midi.StartReceive();
	UiMode last_mode = UiMode::Main;
	int32_t last_menu = -1;
	int32_t last_shift_menu = -1;
	int32_t last_perform_index = -1;
	int32_t last_fx_detail_index = -1;
	int32_t last_fx_detail_param_index = -1;
	int32_t last_scroll = -1;
	int32_t last_selected = -1;
	int32_t last_file_count = -1;
	bool last_sd_mounted = false;
	RecordState last_record_state = RecordState::Armed;
	bool last_playback_active = false;
	LoadDestination last_load_target = LoadDestination::Play;
	while(1)
	{
		hw.midi.Listen();
		while(hw.midi.HasEvents())
		{
			HandleMidiMessage(hw.midi.PopEvent());
		}
		int32_t aux_delta = 0;
		if (encoder_r_accum != 0)
		{
			aux_delta = encoder_r_accum;
			encoder_r_accum = 0;
		}
		if (aux_delta != 0 && UiLogEnabled())
		{
			LogLine("Encoder R delta=%ld", static_cast<long>(aux_delta));
		}
		if (encoder_r_button_press)
		{
			encoder_r_button_press = false;
			if (UiLogEnabled())
		{
			LogLine("Encoder R button pressed");
		}
	}
	const bool ui_blocked = (sd_init_in_progress || save_in_progress || bake_in_progress);
	if (button2_press)
	{
		button2_press = false;
		if (!ui_blocked)
		{
			shift_prev_mode = ui_mode;
			ui_mode = UiMode::Shift;
			shift_menu_index = 0;
			request_shift_redraw = true;
		}
	}
	if (button1_press)
	{
		button1_press = false;
		if (!ui_blocked && ui_mode == UiMode::Play)
		{
			if (!playhead_running)
			{
				playhead_running = true;
				playhead_step = 0;
				playhead_last_step_ms = System::GetNow();
				play_screen_dirty = true;
				request_playhead_redraw = true;
			}
			else
			{
				playhead_running = false;
				playhead_last_step_ms = 0;
				play_screen_dirty = true;
				request_playhead_redraw = true;
			}
		}
	else if (ui_mode != UiMode::Play
				 && sample_loaded
				 && ui_mode != UiMode::Load
				 && ui_mode != UiMode::LoadTarget)
		{
			if (UiLogEnabled())
			{
				LogLine("Button1: playback request (unpitched)");
			}
			StartPlayback(kBaseMidiNote, false);
			request_playhead_redraw = true;
		}
	}
	if (!ui_blocked && ui_mode == UiMode::Play && playhead_running)
	{
		const uint32_t now = System::GetNow();
		if (playhead_last_step_ms == 0)
		{
			playhead_last_step_ms = now;
		}
		if (kPlayStepMs > 0)
		{
			const uint32_t elapsed = now - playhead_last_step_ms;
			if (elapsed >= kPlayStepMs)
			{
				const uint32_t steps = elapsed / kPlayStepMs;
				playhead_last_step_ms += steps * kPlayStepMs;
				playhead_step = (playhead_step + static_cast<int32_t>(steps)) % kPlayStepCount;
				play_screen_dirty = true;
				request_playhead_redraw = true;
			}
		}
	}
		if (request_load_scan)
		{
			if (!ui_blocked)
			{
				request_load_scan = false;
				LogLine("Load menu: scan requested");
				ScanSdFiles(true);
			}
		}
		if (request_delete_scan)
		{
			if (!ui_blocked)
			{
				request_delete_scan = false;
				LogLine("Delete menu: scan requested");
				ScanSdFiles(false);
			}
		}
		if (request_load_sample)
		{
			if (ui_blocked)
			{
				request_load_sample = false;
				request_load_index = -1;
				LogLine("Load menu: sample request ignored during UI block");
			}
			else
			{
				request_load_sample = false;
				const int32_t index = request_load_index;
				request_load_index = -1;
				const LoadDestination dest = request_load_destination;
					LogLine("Load menu: sample request index=%ld target=%s",
							static_cast<long>(index),
							LoadDestinationName(dest));
					if (index >= 0 && index < wav_file_count)
					{
						LogLine("Load menu: sample name=%s", wav_files[index]);
				}
				else
				{
					LogLine("Load menu: sample name unavailable (count=%ld)", static_cast<long>(wav_file_count));
				}
				if (LoadSampleAtIndex(index))
				{
					if (dest == LoadDestination::Bake)
					{
						LogLine("Load success, entering BAKE (trim review)");
						ui_mode = UiMode::Record;
						record_state = RecordState::Review;
						request_length_redraw = true;
						}
						else
						{
							if (dest == LoadDestination::Perform)
							{
								LogLine("Load success, entering PERFORM menu");
								ui_mode = UiMode::Perform;
								menu_index = 2;
							}
							else
							{
								LogLine("Load success, entering PLAY menu");
								ui_mode = UiMode::Play;
							}
						}
					}
					else
					{
					LogLine("Load failed");
					ui_mode = UiMode::Load;
				}
			}
		}

		if (!ui_blocked)
		{
			const bool preview_allowed = (ui_mode == UiMode::Load && !delete_mode && wav_file_count > 0);
			if (preview_hold && preview_allowed)
			{
				if (!preview_active || preview_index != load_selected)
				{
					if (!BeginPreviewAtIndex(load_selected))
					{
						StopPreview();
					}
				}
			}
			else
			{
				if (preview_active)
				{
					StopPreview();
				}
			}
		}
		if (preview_active)
		{
			FillPreviewBuffer();
		}
		if (request_delete_file)
		{
			if (ui_blocked)
			{
				request_delete_file = false;
				request_delete_index = -1;
				LogLine("Delete menu: request ignored during UI block");
			}
			else
			{
				request_delete_file = false;
				const int32_t index = request_delete_index;
				request_delete_index = -1;
				LogLine("Delete menu: request index=%ld", static_cast<long>(index));
				if (index >= 0 && index < wav_file_count)
				{
					LogLine("Delete menu: file name=%s", wav_files[index]);
				}
				else
				{
					LogLine("Delete menu: file name unavailable (count=%ld)", static_cast<long>(wav_file_count));
				}
				if (DeleteFileAtIndex(index))
				{
					LogLine("Delete success");
					request_delete_scan = true;
					delete_confirm = false;
					request_delete_redraw = true;
				}
				else
				{
					LogLine("Delete failed");
					delete_confirm = false;
					request_delete_redraw = true;
				}
			}
		}
		if (request_bake_process)
		{
			request_bake_process = false;
			const bool ok = ProcessBakedBank();
			bake_in_progress = false;
				if (ok)
				{
					perform_bake_active = true;
					perform_attack_norm = 0.0f;
					perform_release_norm = 0.0f;
					menu_index = 3;
					ui_mode = UiMode::Play;
					waveform_dirty = true;
				}
			else
			{
				baked_ready = false;
				baked_length = 0;
				baked_window_valid = false;
				perform_bake_active = false;
				ui_mode = confirm_bake_prev_mode;
			}
		}

		if (sd_init_in_progress)
		{
			if (!sd_init_done)
			{
				const uint32_t now = System::GetNow();
				if (sd_init_attempts < kSdInitAttempts && now >= sd_init_next_ms)
				{
					sd_init_success = ReinitSdNow();
					sd_init_attempts++;
					if (sd_init_success || sd_init_attempts >= kSdInitAttempts)
					{
						sd_init_done = true;
						sd_init_result_until_ms = now + kSdInitResultMs;
					}
					else
					{
						sd_init_next_ms = now + kSdInitRetryMs;
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
					save_success = BeginSaveRecordedSample();
					save_started = true;
					if (!save_success)
					{
						save_done = true;
						save_result_until_ms = now + kSaveResultMs;
					}
				}
				else
				{
					bool step_done = false;
					DrawSaveScreen();
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

		if (record_waveform_pending)
		{
			record_waveform_pending = false;
			if (sample_loaded && sample_length > 0)
			{
				ComputeWaveform();
				waveform_ready = true;
				waveform_dirty = true;
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
			else if (ui_mode == UiMode::Play)
			{
				play_screen_dirty = true;
				DrawPlayScreen();
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
			if (last_mode == UiMode::Play && mode != UiMode::Play)
			{
				playhead_running = false;
				playhead_step = 0;
				playhead_last_step_ms = 0;
				play_screen_dirty = true;
			}
			if (mode == UiMode::Record)
			{
				record_anim_start_ms = NowMs();
			}
			else if (last_mode == UiMode::Record)
			{
				record_anim_start_ms = -1.0;
			}
			LogLine("UI mode: %s", UiModeName(mode));
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
					LogLine("Load menu: selected=%ld name=%s",
							static_cast<long>(load_selected),
							(load_selected >= 0 && load_selected < wav_file_count)
								? wav_files[load_selected]
								: "UNKNOWN");
				}
			}
			else if (mode == UiMode::Play)
			{
				play_screen_dirty = true;
				DrawPlayScreen();
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
					&& shift_button.Pressed();
					const bool amp_select_active = (perform_index == kPerformAmpIndex)
						&& shift_button.Pressed();
					const bool flt_select_active = (perform_index == kPerformFltIndex)
						&& shift_button.Pressed();
					DrawPerformScreen(perform_index,
									  fx_select_active,
									  fx_fader_index,
									  amp_select_active,
									  amp_fader_index,
									  flt_select_active,
									  flt_fader_index);
				}
				else if (mode == UiMode::LoadTarget)
				{
					DrawLoadTargetMenu(load_target_selected);
					LogLine("Load target: %s",
							LoadDestinationName(load_target_selected));
				}
			else if (mode == UiMode::ConfirmBake)
			{
				DrawConfirmBakeScreen(confirm_bake_selected);
			}
			else if (mode == UiMode::Shift)
			{
				DrawShiftMenu(shift_menu_index);
				last_shift_menu = shift_menu_index;
			}
			else
			{
				if (record_state == RecordState::SourceSelect)
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
					DrawRecordRecording();
				}
				else
				{
					if (sample_loaded && sample_length > 0)
					{
						ComputeWaveform();
						waveform_ready = true;
						waveform_dirty = true;
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
				last_load_target = load_target_selected;
				last_perform_index = perform_index;
			}
			else if (mode == UiMode::Main)
			{
				const int32_t current = menu_index;
			if (current != last_menu)
			{
				LogLine("Menu highlight: %s (%ld)",
						MenuLabelForIndex(current),
						static_cast<long>(current));
				DrawMenu(current);
					last_menu = current;
				}
			}
			else if (mode == UiMode::Perform)
			{
				const int32_t current = perform_index;
				if (request_perform_redraw || current != last_perform_index)
				{
					request_perform_redraw = false;
					const bool fx_select_active = (current == kPerformFxIndex)
						&& shift_button.Pressed();
					const bool amp_select_active = (current == kPerformAmpIndex)
						&& shift_button.Pressed();
					const bool flt_select_active = (current == kPerformFltIndex)
						&& shift_button.Pressed();
					DrawPerformScreen(current,
									  fx_select_active,
									  fx_fader_index,
									  amp_select_active,
									  amp_fader_index,
									  flt_select_active,
									  flt_fader_index);
					last_perform_index = current;
				}
			}
			else if (mode == UiMode::FxDetail)
			{
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
						LogLine("Load menu: selected=%ld name=%s",
								static_cast<long>(current_selected),
								(current_selected >= 0 && current_selected < current_count)
									? wav_files[current_selected]
									: "UNKNOWN");
					}
					last_scroll = current_scroll;
					last_selected = current_selected;
					last_file_count = current_count;
					last_sd_mounted = sd_mounted;
				}
			}
		}
		else if (mode == UiMode::LoadTarget)
		{
			const LoadDestination current_target = load_target_selected;
			if (current_target != last_load_target)
			{
				DrawLoadTargetMenu(current_target);
				last_load_target = current_target;
			}
		}
		else if (mode == UiMode::ConfirmBake)
		{
			if (request_confirm_bake_redraw)
			{
				request_confirm_bake_redraw = false;
				DrawConfirmBakeScreen(confirm_bake_selected);
			}
		}
		else if (mode == UiMode::Record)
		{
			const RecordState current_state = record_state;
			if (current_state != last_record_state)
			{
				if (current_state == RecordState::SourceSelect)
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
					DrawRecordRecording();
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
			if (live_wave_dirty)
			{
				live_wave_dirty = false;
				DrawRecordRecording();
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
		if (!ui_blocked && (request_playhead_redraw || (playback_active != last_playback_active)))
		{
			request_playhead_redraw = false;
			if (mode == UiMode::Play
				|| (mode == UiMode::Edt)
				|| (mode == UiMode::Record && record_state == RecordState::Review))
			{
				if (mode == UiMode::Play)
				{
					DrawPlayScreen();
				}
				else if (mode == UiMode::Edt)
				{
					DrawEdtScreen();
				}
				else
				{
					DrawRecordReview();
				}
			}
		}
		last_playback_active = playback_active;
		if (request_playback_stop_log)
		{
			request_playback_stop_log = false;
			if (UiLogEnabled())
			{
				LogLine("Playback: stopped (win=[%lu,%lu) phase=%.3f)",
						static_cast<unsigned long>(sample_play_start),
						static_cast<unsigned long>(sample_play_end),
						static_cast<double>(playback_phase));
			}
		}
		if (ui_mode == UiMode::Play)
		{
			led1_phase_ms = 0.0f;
			led1_level = 0.0f;
			if (playhead_running)
			{
				hw.led1.Set(1.0f, 0.0f, 0.0f);
			}
			else
			{
				hw.led1.Set(0.0f, 1.0f, 0.0f);
			}
		}
		else
		{
			if ((ui_mode == UiMode::Record && record_state == RecordState::Review)
				|| (ui_mode == UiMode::Load && !delete_mode)
				|| (ui_mode == UiMode::Perform && sample_loaded)
				|| (ui_mode == UiMode::FxDetail && sample_loaded))
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
				led1_level = 0.0f;
			}
			hw.led1.Set(0.0f, led1_level, 0.0f);
		}
		hw.UpdateLeds();
		hw.DelayMs(10);
	}
}
