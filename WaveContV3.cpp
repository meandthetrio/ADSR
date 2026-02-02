#include "daisy_pod.h"
#include "daisysp.h"
#include "dev/oled_ssd130x.h"
#include "fatfs.h"
#include "per/tim.h"
#include "util/wav_format.h"
#include "util/bsp_sd_diskio.h"
#include "util/scopedirqblocker.h"
#include "StorageService.h"
#include <cmath>
#include <initializer_list>
//#include <math.h>
#include <cstring>
#include <cstdio>

using namespace daisy;
using namespace daisysp;

#ifndef STORAGE_SERVICE_PREVIEW_STREAM
#define STORAGE_SERVICE_PREVIEW_STREAM 1
#endif

#ifndef STORAGE_SERVICE_SAVE
#define STORAGE_SERVICE_SAVE 1
#endif

using PodDisplay = OledDisplay<SSD130xI2c128x64Driver>;

#ifndef RAM_D3_MEM_SECTION
#define RAM_D3_MEM_SECTION __attribute__((section(".ramd3_bss")))
#endif

#ifndef PERF_DIAGNOSTICS
#define PERF_DIAGNOSTICS 0
#endif

#if PERF_DIAGNOSTICS
#define PERF_CYCLES_START(var) const uint32_t var = DWT->CYCCNT
#define PERF_CYCLES_END(var)   const uint32_t var = DWT->CYCCNT
#else
#define PERF_CYCLES_START(var) do {} while (0)
#define PERF_CYCLES_END(var)   do {} while (0)
#endif

constexpr int32_t kMenuCount = 3;
constexpr int32_t kShiftMenuCount = 2;
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
constexpr size_t kAudioBlockSize = 48;
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
constexpr size_t kMaxSampleSamples = 240000;
constexpr int32_t kRecordMaxSeconds = 5;
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
constexpr size_t kPreviewBufferFrames = 16384;
constexpr size_t kPreviewReadFrames = 256;
constexpr size_t kPreviewPpFrames = 2048;
constexpr size_t kPreviewPreloadFrames = 240000;
constexpr uint32_t kStorageBudgetUs = 2000;
constexpr uint32_t kStoragePreviewBudgetUs = 12000;
constexpr uint32_t kPreviewFadeInMs = 10;
constexpr uint32_t kUiTickMs = 1;
constexpr uint32_t kUiTickPlaybackMs = 5;
constexpr uint32_t kLoadScanGraceMs = 300;
constexpr float kRecordWaveformScaleMinMic = 1.15f;
constexpr float kRecordWaveformScaleMaxMic = 1.75f;
constexpr float kRecordWaveformScaleMinLine = 0.7f;
constexpr float kRecordWaveformScaleMaxLine = 2.0f;

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
		bump_target_ = 0.0f;
		mix_target_ = 0.0f;
		output_gain_target_ = 1.0f;
		post_gain_target_ = 1.0f;
		drive_ = 0.0f;
		bias_ = 0.0f;
		tone_ = 0.5f;
		bump_ = 0.0f;
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
		bump_ += smooth_coeff_ * (bump_target_ - bump_);
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
		const float bump_curve = bump_ * bump_;
		const float mid = x - low - high;
		const float bump_low = bump_curve * 0.25f;
		const float bump_mid = bump_curve * 1.6f;
		const float mid_res = bump_curve * 1.8f;
		const float mid_res_sample = (mid + (prev_y_ * 0.35f)) * mid_res;
		x = x + (low * (low_amt + bump_low))
			+ (mid * bump_mid)
			+ (mid_res_sample)
			+ (high * high_amt);

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

	void SetBump(float b01)
	{
		bump_target_ = Clamp(b01, 0.0f, 1.0f);
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
	float bump_target_ = 0.0f;
	float mix_target_ = 0.0f;
	float output_gain_target_ = 1.0f;
	float post_gain_target_ = 1.0f;

	float drive_ = 0.0f;
	float bias_ = 0.0f;
	float tone_ = 0.5f;
	float bump_ = 0.0f;
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

struct BitCrushState
{
	int hold = 0;
	float hold_l = 0.0f;
	float hold_r = 0.0f;
};

static float g_sine_table[kSineTableSize + 1];

static inline float SineTableLookup(float phase01)
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

DaisyPod    hw;
PodDisplay  display;
static bool g_display_update_pending = false;
static uint32_t g_last_draw_ms = 0;
constexpr uint32_t kDrawIntervalMs = 33;
constexpr uint8_t kMidiCmdQSize = 16; // power of two
enum MidiCmdKind : uint8_t { kMidiCmdNoteOn = 1, kMidiCmdNoteOff = 2 };

struct MidiCmd
{
	uint8_t kind;
	uint8_t note;
	uint8_t vel;
	uint32_t t_ms;
};

static MidiCmd g_midi_cmd_q[kMidiCmdQSize];
static volatile uint8_t g_midi_cmd_wr = 0;
static volatile uint8_t g_midi_cmd_rd = 0;
static volatile bool g_midi_rx_started = false;

constexpr uint8_t kPlaybackCmdQSize = 8; // power of two
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
	uint8_t kind;
	uint8_t note;
	uint8_t flags;
	uint8_t pad;
};

static PlaybackCmd g_playback_cmd_q[kPlaybackCmdQSize];
static volatile uint8_t g_playback_cmd_wr = 0;
static volatile uint8_t g_playback_cmd_rd = 0;


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
	g_playback_cmd_q[wr].pad = 0;
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

static inline void RequestDisplayUpdate()
{
	g_display_update_pending = true;
}

static inline void FlushDisplayIfDue(uint32_t now)
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
SdmmcHandler   sdcard;
FatFSInterface fsi;
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
DTCM_MEM_SECTION static BitCrushState g_sat_bit_state;
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
static volatile bool list_build_pending = false;
volatile bool request_load_sample = false;
volatile int32_t request_load_index = -1;
volatile int32_t wav_file_count = 0;

bool sd_mounted = false;
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
#if !STORAGE_SERVICE_SAVE
static FIL save_file;
#endif
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
char wav_files[kMaxWavFiles][kMaxWavNameLen];
char loaded_sample_name[kMaxWavNameLen] = {0};
int32_t load_lines = 1;
int32_t load_line_height = 1;
int32_t load_chars_per_line = 1;
static uint32_t load_scan_start_ms = 0;

enum class SampleContext : int32_t
{
	Perform,
};

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
static SampleContext current_sample_context = SampleContext::Perform;

DSY_SDRAM_BSS int16_t perform_sample_buffer_l[kMaxSampleSamples];
DSY_SDRAM_BSS int16_t perform_sample_buffer_r[kMaxSampleSamples];
static int16_t* sample_buffer_l = perform_sample_buffer_l;
static int16_t* sample_buffer_r = perform_sample_buffer_r;
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
static volatile bool g_playback_reverse_target = false;
static volatile bool g_perform_voices_active = false;
volatile float playback_amp = 0.0f;
volatile uint32_t playback_env_samples = 0;
volatile bool playback_release_active = false;
volatile float playback_release_pos = 0.0f;
volatile float playback_release_start = 0.0f;
volatile int32_t current_note = -1;

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
	uint8_t order[kPerformFaderCount] = {};
	uint8_t count = kPerformFaderCount;
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

static SampleRuntime g_rt_buf[2];
static volatile uint8_t g_rt_pub_idx = 0;
static volatile uint8_t g_rt_active_idx = 0;
static FxChainRuntime g_fx_chain_buf[2];
static volatile uint8_t g_fx_chain_pub_idx = 0;
static volatile uint8_t g_fx_chain_active_idx = 0;
static PreviewControl g_preview_ctl_buf[2];
static volatile uint8_t g_preview_pub_idx = 0;
static volatile uint8_t g_preview_active_idx = 0;

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
	int32_t track = -1;
	size_t offset = 0;
	size_t length = 0;
	uint32_t env_samples = 0;
};

DTCM_MEM_SECTION static PerformVoice perform_voices[kPerformVoiceCount];

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
	int32_t fx_chain_order[kPerformFaderCount] = {};
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

enum class LoadContext : int32_t
{
	Main,
	Edt,
};

static PerformState main_perform_state;
static UiMode edt_prev_mode = UiMode::Perform;
static SampleContext edt_sample_context = SampleContext::Perform;
static UiMode fx_detail_prev_mode = UiMode::Perform;
static UiMode load_prev_mode = UiMode::Main;
static LoadContext load_context = LoadContext::Main;
static int32_t load_mode_index = 0;
static LoadStubMode load_stub_mode = LoadStubMode::Presets;

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

// Waveform computation request (from audio callback to main loop)
static volatile bool waveform_compute_pending = false;
static volatile SampleContext waveform_compute_ctx = SampleContext::Perform;

struct WaveformCache
{
	int16_t min[128] = {};
	int16_t max[128] = {};
	bool ready = false;
	bool dirty = false;
};

static WaveformCache perform_waveform_cache;
static bool waveform_from_recording = false;
static RecordInput waveform_record_input = RecordInput::LineIn;
static volatile float perform_attack_norm = 0.0f;
static volatile float perform_release_norm = 0.0f;
static const char* waveform_title = nullptr;

constexpr size_t kRecordMaxFrames = static_cast<size_t>(kRecordMaxSeconds) * 48000U;
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
static bool ui_button1_held = false;

enum AudioCmdBits : uint32_t
{
	kCmdNone         = 0,
	kCmdRecStart     = 1U << 0,
	kCmdRecStop      = 1U << 1,
	kCmdAllNotesOff  = 1U << 2,
	kCmdCommitRuntime = 1U << 3,
	kCmdCommitFxChain = 1U << 4,
	kCmdPreviewStart = 1U << 5,
	kCmdPreviewStop  = 1U << 6,
	kCmdCommitPreview = 1U << 7,
	kCmdPlaybackReverse = 1U << 8,
	kCmdCommitAudioParams = 1U << 9,
};

static volatile uint32_t g_audio_cmd = 0;
static volatile uint32_t g_audio_event_bits = 0;

static void RequestAudioCmd(uint32_t bits)
{
	{
		daisy::ScopedIrqBlocker irq;
		g_audio_cmd |= bits;
	}
}

static inline void RequestPlaybackStart(uint8_t note, bool apply_pitch)
{
	const uint8_t flags = apply_pitch ? kPlaybackCmdApplyPitch : 0;
	PlaybackCmdPushUi(kPlaybackCmdStart, note, flags);
}

static inline void RequestPlaybackStop(uint8_t note, bool apply_release)
{
	const uint8_t flags = apply_release ? kPlaybackCmdApplyRelease : 0;
	PlaybackCmdPushUi(kPlaybackCmdStop, note, flags);
}

static inline void RequestPlaybackStopAll()
{
	PlaybackCmdPushUi(kPlaybackCmdStopAll, 0, 0);
}

static void PushAudioEvent(uint32_t bits)
{
	{
		daisy::ScopedIrqBlocker irq;
		g_audio_event_bits |= bits;
	}
}

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

static WaveformJob g_wf_job = {};

struct FileListJob
{
	bool active;
	bool done;
	bool wav_only;
	int32_t count;
	uint16_t cookie;
};

static FileListJob g_list_job = {};

constexpr float kUiTickHz = 1000.0f;

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

struct AudioParams
{
	float amp_attack;
	float amp_decay;
	float amp_sustain;
	float amp_release;
	float flt_cutoff;
	float flt_res;
	float fx_s_wet;
	float sat_drive;
	float sat_tape_bump;
	float sat_bit_reso;
	float sat_bit_smpl;
	float fx_c_wet;
	float mod_depth;
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
	float playback_reverse;
};

// Audio-ready FX parameters (pre-mapped; no powf/expf needed in AudioCallback).
struct FxParamsAudio
{
	// Saturation / bitcrush
	int32_t sat_mode = 0;
	float   sat_mix = 0.0f;
	float   sat_drive_amt = 0.0f;
	float   sat_bump = 0.0f;
	float   bit_step = 1.0f;

	// Chorus / tape
	int32_t chorus_mode = 0;
	float   chorus_mix = 0.0f;
	float   chorus_depth_mapped = 0.0f;
	float   chorus_rate_hz = 0.0f;
	float   chorus_wow = 0.0f;
	float   tape_rate = 0.0f;
	float   tape_drop_amt_mapped = 0.0f;

	// Delay
	float delay_wet = 0.0f;
	float delay_time_samples = 0.0f;
	float delay_feedback = 0.0f;
	float delay_spread = 0.0f;
	float delay_freeze = 0.0f;

	// Reverb
	float reverb_wet = 0.0f;
	float reverb_predelay_samples = 0.0f;
	float reverb_lp_hz = 0.0f;
	float reverb_feedback = 0.0f;
	float reverb_release = 1.0f;
	float reverb_gain = 1.0f;
};

static FxParamsAudio g_fx_params_buf[2];
static volatile uint8_t g_fx_params_idx = 0;

static int BitResoIndexFromValue(float value);
extern volatile int32_t sat_mode;
extern volatile int32_t chorus_mode;

static inline float Clamp01(float v)
{
	if (v < 0.0f) return 0.0f;
	if (v > 1.0f) return 1.0f;
	return v;
}

// Build and publish g_fx_params_buf (heavy mapping; call at <=50-100Hz).
static void BuildAndPublishFxParamsAudio(const AudioParams &p, float out_sr)
{
	FxParamsAudio fx = {};
	const float sat_drive = Clamp01(p.sat_drive);
	const float sat_mix = Clamp01(p.fx_s_wet);
	const float sat_bump = Clamp01(p.sat_tape_bump);
	const float bit_reso = Clamp01(p.sat_bit_reso);
	const float chorus_depth = Clamp01(p.mod_depth);
	const float chorus_mix = Clamp01(p.fx_c_wet);
	const float chorus_rate = Clamp01(p.chorus_rate);
	const float chorus_wow = Clamp01(p.chorus_wow);
	const float tape_rate = Clamp01(p.tape_rate);
	const float delay_wet = Clamp01(p.delay_wet);
	const float delay_time = Clamp01(p.delay_time);
	const float delay_feedback = Clamp01(p.delay_feedback);
	const float delay_spread = Clamp01(p.delay_spread);
	const float delay_freeze = Clamp01(p.delay_freeze);
	const float reverb_wet = Clamp01(p.reverb_wet);
	const float reverb_pre = Clamp01(p.reverb_pre);
	const float reverb_damp = Clamp01(p.reverb_damp);
	const float reverb_decay = Clamp01(p.reverb_decay);

	// Saturation / bitcrush
	fx.sat_mode = sat_mode;
	fx.sat_mix = sat_mix;
	fx.sat_bump = sat_bump;
	fx.sat_drive_amt = powf(sat_drive, 0.7f);
	{
		const int bits_idx = BitResoIndexFromValue(bit_reso);
		const int bits = kBitResoSteps[bits_idx];
		fx.bit_step = 1.0f / powf(2.0f, static_cast<float>(bits - 1));
	}

	// Chorus / tape
	fx.chorus_mode = chorus_mode;
	fx.chorus_mix = chorus_mix;
	{
		const float depth_curve = chorus_depth * chorus_depth;
		const float depth_scale = kChorusMaxDepth * 1.2f;
		fx.chorus_depth_mapped = depth_curve * depth_scale;
	}
	{
		const float rate_curve = chorus_rate * chorus_rate;
		fx.chorus_rate_hz = kChorusRateMinHz + rate_curve * (kChorusRateMaxHz - kChorusRateMinHz);
	}
	fx.chorus_wow = chorus_wow;
	fx.tape_rate = tape_rate;
	fx.tape_drop_amt_mapped = (chorus_wow > 0.0f) ? powf(chorus_wow, 0.6f) : 0.0f;

	// Delay mapping
	fx.delay_wet = delay_wet;
	{
		const float curve = delay_time * delay_time;
		float time_ms = kDelayTimeMinMs + curve * (kDelayTimeMaxMs - kDelayTimeMinMs);
		float samples = time_ms * 0.001f * out_sr;
		const float max_samp = static_cast<float>(kDelayMaxSamples - 1);
		if (samples > max_samp) samples = max_samp;
		if (samples < 1.0f) samples = 1.0f;
		fx.delay_time_samples = samples;
	}
	fx.delay_feedback = delay_feedback;
	if (fx.delay_feedback > kDelayFeedbackMax)
	{
		fx.delay_feedback = kDelayFeedbackMax;
	}
	fx.delay_spread = delay_spread;
	fx.delay_freeze = delay_freeze;

	// Reverb mapping
	fx.reverb_wet = reverb_wet;
	fx.reverb_gain = 1.0f;
	{
		const float decay_ms = kReverbDecayMinMs
			+ reverb_decay * reverb_decay * (kReverbDecayMaxMs - kReverbDecayMinMs);
		const float decay_samples = decay_ms * 0.001f * out_sr;
		if (reverb_decay >= 0.999f) fx.reverb_release = 1.0f;
		else if (decay_samples > 1.0f) fx.reverb_release = expf(-1.0f / decay_samples);
		else fx.reverb_release = 0.0f;
	}
	fx.reverb_feedback = (reverb_decay >= 0.999f) ? 0.99f : kReverbFeedback;
	{
		const float damp_curve = reverb_damp * 1.6f;
		float rev_lp = kReverbDampMaxHz * powf(kReverbDampMinHz / kReverbDampMaxHz, damp_curve);
		const float rev_lp_max = out_sr * 0.49f;
		if (rev_lp > rev_lp_max) rev_lp = rev_lp_max;
		if (rev_lp < kReverbDampMinHz) rev_lp = kReverbDampMinHz;
		fx.reverb_lp_hz = rev_lp;
	}
	{
		const float pre_curve = powf(reverb_pre, 3.0f);
		float samples = pre_curve * (kReverbPreDelayMaxMs * 0.001f * out_sr);
		const float max_samp = static_cast<float>(kReverbPreDelayMaxSamples - 1);
		if (samples > max_samp) samples = max_samp;
		if (samples < 0.0f) samples = 0.0f;
		fx.reverb_predelay_samples = samples;
	}

	const uint8_t next = g_fx_params_idx ^ 1;
	g_fx_params_buf[next] = fx;
	{
		daisy::ScopedIrqBlocker irq;
		g_fx_params_idx = next;
	}
}

DTCM_MEM_SECTION static AudioParams g_audio_params_buf[2] = {};
static volatile uint8_t g_audio_params_pub_idx = 0;
static uint8_t g_audio_params_active_idx = 0;

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

constexpr int kWaveCols = 128;

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
#if PERF_DIAGNOSTICS
	float cpu_load_pct = 0.0f;
	float cpu_load_peak_pct = 0.0f;
#endif
};

static const AudioUiState& GetAudioUiStateSnapshot(uint8_t& idx);
static void PublishAudioParamsFromUi(const AudioParams& p);

static volatile uint32_t g_audio_flags_bits = 0;
static AudioUiState g_audio_ui_state_buf[2];
static volatile uint8_t g_audio_ui_state_idx = 0;
static volatile bool g_audio_recording_active = false;
static bool g_reset_voices_pending = false;
static volatile float g_delay_time_alpha = 1.0f;
static volatile float g_delay_param_alpha = 1.0f;
static FxChainRuntime g_fx_chain_audio = {};
static bool g_fx_chain_audio_valid = false;

volatile RecordState record_state = RecordState::Armed;
volatile int32_t record_source_index = 0;
volatile int32_t record_target_index = kRecordTargetSave;
volatile uint32_t record_countdown_start_ms = 0;
volatile size_t record_pos = 0;
static volatile size_t g_recorded_length_audio = 0;
volatile bool record_waveform_pending = false;
volatile int32_t encoder_r_accum = 0;
volatile bool encoder_r_button_press = false;
volatile bool request_length_redraw = false;
#if PERF_DIAGNOSTICS
static float cpu_load_ema = 0.0f;
static volatile float cpu_load_pct = 0.0f;
static volatile float cpu_load_peak_pct = 0.0f;
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
static bool sat_params_initialized = false;
static bool reverb_params_initialized = false;
static bool delay_params_initialized = false;
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
static float fx_chain_fade_gain = 1.0f;
static float fx_chain_fade_target = 1.0f;
static int32_t fx_chain_fade_samples_left = 0;
static bool fx_chain_pause_pending = false;
static bool fx_chain_paused = false;
static uint32_t fx_chain_last_move_ms = 0;
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
static volatile uint32_t preview_underrun_count = 0;
static volatile uint32_t preview_rb_min_level = 0xFFFFFFFFu;
#endif
static volatile uint32_t preview_fade_samples_left = 0;
static volatile uint32_t preview_fade_samples_total = 0;
#if STORAGE_SERVICE_PREVIEW_STREAM
static uint16_t preview_stream_cookie = 1;
static uint16_t preview_stream_cookie_active = 0;
#endif
static bool preview_pending_start = false;
static uint32_t preview_pending_start_ms = 0;
alignas(32) static int16_t preview_buffer[kPreviewBufferFrames];
#if !STORAGE_SERVICE_PREVIEW_STREAM
alignas(32) static int16_t preview_read_buf[kPreviewReadFrames * 2];
#endif
#if STORAGE_SERVICE_PREVIEW_STREAM
alignas(32) static int16_t preview_pp_buf[2][kPreviewPpFrames];
static volatile uint8_t preview_pp_ready[2] = {0, 0};
static volatile uint8_t preview_pp_active = 0;
static volatile uint32_t preview_pp_pos = 0;
DSY_SDRAM_BSS static int16_t preview_preload_buf[kPreviewPreloadFrames];
static volatile size_t preview_preload_frames = 0;
static volatile bool preview_preload_active = false;
#endif
float led1_level = 0.0f;
float led1_phase_ms = 0.0f;
static double record_anim_start_ms = -1.0;
  RAM_D3_MEM_SECTION static uint8_t record_text_mask[kDisplayH][kDisplayW];
  RAM_D3_MEM_SECTION static uint8_t record_invert_mask[kDisplayH][kDisplayW];
  RAM_D3_MEM_SECTION static uint8_t record_fb_buf[kDisplayH][kDisplayW];
  RAM_D3_MEM_SECTION static uint8_t record_bold_mask[kDisplayH][kDisplayW];
static bool request_shift_redraw = false;
static bool request_perform_redraw = false;
static bool request_fx_detail_redraw = false;
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
static uint32_t delay_snow_next_ms = 0;
static uint32_t midi_ignore_until_ms = 0;

static void __attribute__((unused)) ComputeWaveform();
static bool AnyPerformVoiceActive();

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

#if 0
static FIL wav_file;
#endif

const char* kMenuLabels[kMenuCount] = {"LOAD", "RECORD", "PERFORM"};

static int32_t NextMenuIndex(int32_t current, int32_t delta)
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
const char* kShiftMenuLabels[kShiftMenuCount] = {"SAVE PRESET", "DELETE"};

static int32_t NextPerformIndex(int32_t current, int32_t delta)
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

template <size_t N>
constexpr size_t ArraySize(const char* const (&)[N])
{
	return N;
}

static void MountSd()
{
	if (sd_mounted)
	{
		return;
	}
	if (!BSP_SD_IsDetected())
	{
		return;
	}
	StorageService::Op op = {};
	op.kind = StorageService::OpKind::Mount;
	if (!storage.Enqueue(op))
	{
		return;
	}
	storage.RunSlice(0);
	StorageService::Event ev = {};
	while (storage.DequeueEvent(ev))
	{
		if (ev.kind == StorageService::EventKind::MountOk)
		{
			sd_mounted = true;
			return;
		}
		if (ev.kind == StorageService::EventKind::MountFail)
		{
			sd_mounted = false;
			return;
		}
	}
	sd_mounted = (storage.GetMountState() == StorageService::MountState::Mounted);
}

// Save names are generated inside StorageService.

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

// Legacy save path removed; StorageService handles save.

static bool ReinitSdNow()
{
	if (!BSP_SD_IsDetected())
	{
		return false;
	}
	storage.UnmountSd();
	sd_mounted = false;
	fsi.DeInit();
	SdmmcHandler::Config sd_cfg;
	sd_cfg.Defaults();
	sdcard.Init(sd_cfg);
	fsi.Init(FatFSInterface::Config::MEDIA_SD);
	(void)BSP_SD_Init();
	MountSd();
	return sd_mounted;
}

static void __attribute__((unused)) ComputeWaveform()
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

static Job g_job = {};

static void WaveformJobCancel()
{
	g_wf_job.active = false;
}

static void WaveformJobStart(SampleContext ctx)
{
	WaveformJobCancel();
	g_wf_job.ctx = ctx;
	if (!sample_loaded || sample_length == 0)
	{
		for (int32_t i = 0; i < 128; ++i)
		{
			waveform_min[i] = 0;
			waveform_max[i] = 0;
		}
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
			waveform_min[g_wf_job.col]
				= static_cast<int16_t>(g_wf_job.minv * g_wf_job.scale);
			waveform_max[g_wf_job.col]
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
	CopyString(op.path, fsi.GetSDPath(), sizeof(op.path));
	op.max_entries = static_cast<uint16_t>(kMaxWavFiles);
	op.cookie = g_list_job.cookie;
	op.wav_only = wav_only;
	if (!storage.Enqueue(op))
	{
		g_list_job.active = false;
		g_list_job.done = true;
	}
}

static void FinalizeFileList()
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

static void InitSmoothers()
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

static void UpdateSmoothedParamsPerTick()
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
		const uint32_t now = System::GetNow();
		if (fx_params_dirty && (int32_t)(now - next_fx_map_ms) >= 0)
		{
			BuildAndPublishFxParamsAudio(p, hw.AudioSampleRate());
			fx_params_dirty = false;
			next_fx_map_ms = now + kFxMapIntervalMs;
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
}

static void UpdateDelaySlewCoeffs()
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

static void JobCancel()
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

static void JobStartWaveform(SampleContext ctx, bool foreground = true)
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

static void JobStartFileList(const char* path, bool wav_only, bool foreground = true)
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

static void JobTick()
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
	constexpr uint32_t kWaveformBudget = 2048;
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

static void PublishRuntimeFromUi();
static void PublishFxChainFromUi();
static void PublishPreviewControlFromUi();

static void UpdateTrimFrames()
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

static bool IsPerformUiMode(UiMode mode)
{
	return (mode == UiMode::Perform);
}

static bool AnyPerformVoiceActive()
{
	for (int v = 0; v < kPerformVoiceCount; ++v)
	{
		if (perform_voices[v].active)
		{
			return true;
		}
	}
	return false;
}

static void PublishRuntimeFromUi()
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

static void PublishFxChainFromUi()
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

static void PublishPreviewControlFromUi()
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

static void PublishAudioParamsFromUi(const AudioParams& p)
{
	const uint8_t next = static_cast<uint8_t>(g_audio_params_pub_idx ^ 1u);
	g_audio_params_buf[next] = p;
	{
		daisy::ScopedIrqBlocker irq;
		g_audio_params_pub_idx = next;
		g_audio_cmd |= kCmdCommitAudioParams;
	}
}

static void AudioUiResetLiveWaveform(AudioUiState& uiw)
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

enum class FxContext : int32_t
{
	Perform,
};

static FxContext fx_context = FxContext::Perform;

static void SaveFxContext()
{
	CapturePerformState(main_perform_state);
}

static void SetFxContext(FxContext ctx, int32_t track = 0)
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

	size_t frames = 0;
	uint16_t channels = 1;
	uint32_t rate = 0;
	(void)storage.LoadSampleBlocking(path,
											   sample_buffer_l,
											   sample_buffer_r,
											   kMaxSampleSamples,
											   frames,
											   channels,
											   rate);
	sample_length = frames;
	sample_rate = rate;
	sample_channels = channels;
	sample_loaded = (sample_length > 0);
	if (!sample_loaded)
	{
		return false;
	}
	ApplyLoadedSampleFade(sample_length, sample_rate);
	trim_start = 0.0f;
	trim_end = 1.0f;
	waveform_from_recording = false;
	JobStartWaveform(current_sample_context, true);
	UpdateTrimFrames();
	PublishRuntimeFromUi();
	return true;
}

static bool LoadSampleAtIndex(int32_t index)
{
	if (!BSP_SD_IsDetected())
	{
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
		(void)BSP_SD_Init();
		MountSd();
	}
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
	return LoadSampleFromPath(path);
}

static void StopPreview()
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

static bool BeginPreviewAtIndex(int32_t index)
{
	if (!BSP_SD_IsDetected())
	{
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
		(void)BSP_SD_Init();
		MountSd();
	}
	if (!sd_mounted)
	{
		return false;
	}
	if (BSP_SD_GetCardState() != SD_TRANSFER_OK)
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
#endif
}

#if !STORAGE_SERVICE_PREVIEW_STREAM
static size_t PreviewAvailableFrames(size_t read_idx, size_t write_idx)
{
	if (write_idx >= read_idx)
	{
		return write_idx - read_idx;
	}
	return (kPreviewBufferFrames - read_idx) + write_idx;
}
#endif

#if !STORAGE_SERVICE_PREVIEW_STREAM
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


static const AudioUiState& GetAudioUiStateSnapshot(uint8_t& idx)
{
	{
		daisy::ScopedIrqBlocker irq;
		idx = g_audio_ui_state_idx;
	}
	return g_audio_ui_state_buf[idx];
}

static void FillPreviewBuffer()
{
#if STORAGE_SERVICE_PREVIEW_STREAM
	return;
#endif
}

static bool DeleteFileAtIndex(int32_t index)
{
	if (!BSP_SD_IsDetected())
	{
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
		(void)BSP_SD_Init();
		MountSd();
	}
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
	return storage.DeleteFileBlocking(path);
}

static void DrawTinyString(const char* str, int x, int y, bool on);
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

static void DrawBitmap1bpp(PodDisplay& disp,
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

static void DrawMenu(int32_t selected)
{
	constexpr int kMarginX = 2;
	constexpr int kMarginY = 2;
	constexpr int kGapX = 2;
	constexpr int kGapY = 2;
	constexpr int kTopBoxW = (kDisplayW - (kMarginX * 2) - kGapX) / 2;
	constexpr int kBoxH = (kDisplayH - (kMarginY * 2) - kGapY) / 2;
	display.Fill(false);
	for (int32_t i = 0; i < kMenuCount; ++i)
	{
		const bool is_bottom = (i == 2);
		const int row = is_bottom ? 1 : 0;
		const int col = is_bottom ? 0 : static_cast<int>(i);
		const int box_w = is_bottom ? (kDisplayW - (kMarginX * 2)) : kTopBoxW;
		const int box_h = kBoxH;
		const int x = is_bottom ? kMarginX : (kMarginX + col * (kTopBoxW + kGapX));
		const int y = kMarginY + row * (kBoxH + kGapY);
		const bool is_selected = (i == selected);
		display.DrawRect(x,
						 y,
						 x + box_w - 1,
						 y + box_h - 1,
						 false,
						 true);
		display.DrawRect(x,
						 y,
						 x + box_w - 1,
						 y + box_h - 1,
						 true,
						 false);
		display.DrawRect(x,
						 y,
						 x + box_w - 1,
						 y + box_h - 1,
						 false,
						 true);

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

		if (icon != nullptr && is_selected)
		{
			const int icon_x = x + (box_w - icon_w) / 2;
			const int icon_y = y + (box_h - icon_h) / 2;
			DrawBitmap1bpp(display,
						   icon_x,
						   icon_y,
						   icon_w,
						   icon_h,
						   icon_stride,
						   icon,
						   true);
		}

		if (!is_selected)
		{
			const char* label = kMenuLabels[i];
			const int text_w = TinyStringWidth(label);
			const int text_h = Font5x7::H;
			const int text_x = x + (box_w - text_w) / 2;
			const int text_y = y + (box_h - text_h) / 2;
			DrawTinyString(label, text_x, text_y, true);
		}
	}
	RequestDisplayUpdate();
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

static float FxWetValue(int32_t fx_index)
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

static float FxWetStep(int32_t fx_index)
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

static volatile float* FxWetTarget(int32_t fx_index)
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

static inline int ClampI(int v, int lo, int hi);

static void DrawPerformScreen(int32_t selected,
							  bool fx_select_active,
							  int32_t fx_selected,
							  bool amp_select_active,
							  int32_t amp_selected,
							  bool flt_select_active,
							  int32_t flt_selected,
							  uint8_t redraw_mask = 0x0F)
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
		display.SetCursor(x, 0);
		display.WriteString(cpu_label, font, true);
	}
	#endif
	RequestDisplayUpdate();
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
	RequestDisplayUpdate();
}

static void DrawLoadMenu(int32_t top_index, int32_t selected)
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	if (kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode)
	{
		DrawLoadMessage("PRESETS", "COMING SOON");
		return;
	}

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

	RequestDisplayUpdate();
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
	display.WriteString("L=NO  R=YES", font, true);
	RequestDisplayUpdate();
}

static void DrawRecordBackConfirm()
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
	RequestDisplayUpdate();
}

static void DrawRecordArmed()
{
	DrawRecordReadyScreen();
}

static inline int ClampI(int v, int lo, int hi);

static int BitResoIndexFromValue(float value)
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

static float BitResoValueFromIndex(int idx)
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

    // Publish "empty" runtime so audio never sees half-cleared state
    PublishRuntimeFromUi();
}

// Audio thread ONLY: deterministic / bounded (called from AudioCallback)
static void StartRecordingAudioRT()
{
    record_pos = 0;

    // Stop playback immediately & deterministically
    playback_active = false;

    // Begin recording
    g_audio_recording_active = true;
}


static void StartRecording()
{
    waveform_record_input = record_input;

    // UI prep only
    PrepareRecordingUiState();

    // UI state machine
    record_state = RecordState::Recording;
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

	RequestDisplayUpdate();
}

static void DrawRecordRecording(const AudioUiState& ui_state)
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
		int top = mid + ui_state.live_wave.minv[x];
		int bottom = mid + ui_state.live_wave.maxv[x];
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
	RequestDisplayUpdate();
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
	RequestDisplayUpdate();
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
	RequestDisplayUpdate();
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
	RequestDisplayUpdate();
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

static void DrawWaveform()
{
	if(!waveform_ready || !waveform_dirty)
		return;

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
			const int a = std::abs(static_cast<int>(waveform_min[i]));
			const int b = std::abs(static_cast<int>(waveform_max[i]));
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
		int top    = mid + static_cast<int>(static_cast<float>(waveform_min[x]) * record_scale);
		int bottom = mid + static_cast<int>(static_cast<float>(waveform_max[x]) * record_scale);

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

static void DrawLoadModeSelect(int32_t selected)
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

static void DrawLoadStubScreen(LoadStubMode mode)
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

static void DrawPresetSaveStub()
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

static void DrawRecordTargetScreen(int32_t selected)
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("SAVE SAMPLE?", font, true);
	display.SetCursor(0, (font.FontHeight + 2) * 2);
	display.WriteString("L=NO  R=YES", font, true);
	RequestDisplayUpdate();
}

static void ApplyPlaybackReverse(bool reverse)
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

static void ApplyPlaybackReverseAudio(bool reverse)
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

static void StartPlaybackAudio(uint8_t note, bool apply_pitch, bool reverse_playback)
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
	const float semis = apply_pitch ? static_cast<float>(note - kBaseMidiNote) : 0.0f;
	const float pitch = powf(2.0f, semis / 12.0f);
	const float sr = (rt.rate == 0) ? 48000.0f : static_cast<float>(rt.rate);
	playback_rate = pitch * (sr / hw.AudioSampleRate());
	playback_phase = static_cast<float>(reverse_playback ? (window_end - 1) : window_start);
	playback_active = true;
}

static void StopPlaybackAudio(uint8_t note, bool apply_release)
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

static void StopPlaybackAllAudio()
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

static void StartPerformVoice(int32_t note)
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
	voice.track = -1;
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
	const float sr = (rt.rate == 0) ? 48000.0f : static_cast<float>(rt.rate);
	const float semis = static_cast<float>(note - kBaseMidiNote);
	const float pitch = powf(2.0f, semis / 12.0f);
	voice.rate = pitch * (sr / hw.AudioSampleRate());
	voice.offset = window_start;
	voice.length = window_end - window_start;
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

static void UiTick(int32_t encoder_l_inc, int32_t encoder_r_inc, uint32_t ctrl_events, bool shift_held)
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

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
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
	const bool perform_mode = (flags_bits & kFlagInPerformMode) != 0;
	const bool main_mode = (flags_bits & kFlagInMainMode) != 0;
	const bool fx_allowed = (flags_bits & kFlagFxAllowed) != 0;
	const bool amp_env_active = perform_mode;
	const float amp_attack_ms = AmpEnvMsFromFader(params.amp_attack);
	const float amp_release_ms = AmpEnvMsFromFader(params.amp_release);
	const float amp_attack_samples = amp_attack_ms * 0.001f * out_sr;
	const float amp_release_samples = amp_release_ms * 0.001f * out_sr;
	const bool use_poly = (!record_active) && (perform_mode && rt.loaded);
	const bool sample_stereo = (rt.channels == 2);
	const float flt_cutoff_hz = FltCutoffFromFader(params.flt_cutoff, out_sr);
	const float flt_q = FltQFromFader(params.flt_res);
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
				const float norm = (uiw.live_wave.peak > 0)
					? (28.0f / (static_cast<float>(uiw.live_wave.peak) * kSampleScale))
					: 1.0f;
				const float s_scaled = static_cast<float>(samp) * kSampleScale * norm;
				int16_t s_pix = static_cast<int16_t>(s_scaled);
				const int32_t col = static_cast<int32_t>(
					(static_cast<uint64_t>(record_pos) * 128U) / kRecordMaxFrames);
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
					attack_env = static_cast<float>(playback_env_samples) / amp_attack_samples;
					if (attack_env > 1.0f)
					{
						attack_env = 1.0f;
					}
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
			out[0][i] = sig_l;
			out[1][i] = sig_r;
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
		out[0][i] = fx_l * fx_gain;
		out[1][i] = fx_r * fx_gain;
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

int main(void)
{
	hw.Init();
	InitSineTable();
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
	{
		StorageService::PreviewStreamConfig cfg = {};
		cfg.buffer = preview_buffer;
		cfg.frames = kPreviewBufferFrames;
		cfg.write_index = &preview_write_index;
		cfg.read_index = &preview_read_index;
		cfg.preload_buf = preview_preload_buf;
		cfg.preload_frames = kPreviewPreloadFrames;
		cfg.pp_buf_a = &preview_pp_buf[0][0];
		cfg.pp_buf_b = &preview_pp_buf[1][0];
		cfg.pp_frames = kPreviewPpFrames;
		cfg.pp_ready_a = &preview_pp_ready[0];
		cfg.pp_ready_b = &preview_pp_ready[1];
		cfg.pp_active = &preview_pp_active;
		storage.SetPreviewStreamConfig(cfg);
	}
	SdmmcHandler::Config sd_cfg;
	sd_cfg.Defaults();
	sdcard.Init(sd_cfg);
	fsi.Init(FatFSInterface::Config::MEDIA_SD);
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
	uint8_t last_wave_ui_idx = 0;
	uint8_t last_audio_ui_idx = 0xFF;
	uint32_t last_edt_playhead_ms = 0;
	bool last_edt_playhead_active = false;
	uint32_t last_ui_ms = System::GetNow();
	while(1)
	{
		const uint32_t storage_budget = (preview_pending_start || preview_hold)
			? kStoragePreviewBudgetUs
			: kStorageBudgetUs;
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

		{
			const uint32_t now = System::GetNow();
			uint8_t ui_idx = 0;
			const AudioUiState& uir = GetAudioUiStateSnapshot(ui_idx);
			const bool playback_busy = uir.playback_active || uir.perform_voices_active;
			const uint32_t ui_tick_ms = playback_busy ? kUiTickPlaybackMs : kUiTickMs;
			int32_t loops = 0;
			while ((int32_t)(now - last_ui_ms) >= (int32_t)ui_tick_ms && loops < 4)
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
				shift_prev_mode = ui_mode;
				ui_mode = UiMode::Shift;
				shift_menu_index = 0;
				request_shift_redraw = true;
			}
		}
		if (button1_press)
		{
			button1_press = false;
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
					JobStartFileList(fsi.GetSDPath(), true, true);
				}
			}
		}
		if (request_delete_scan)
		{
			if (!ui_blocked)
			{
				request_delete_scan = false;
				JobStartFileList(fsi.GetSDPath(), false, true);
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
				if (index >= 0 && index < wav_file_count)
				{
				}
				else
				{
				}
				if (LoadSampleAtIndex(index))
				{
					if (load_context == LoadContext::Edt)
					{
						ui_mode = UiMode::Edt;
						if (sample_loaded && sample_length > 0)
						{
							waveform_ready = true;
						}
						waveform_dirty = true;
						request_length_redraw = true;
					}
					else
					{
						ui_mode = UiMode::Perform;
						menu_index = 2;
					}
					load_context = LoadContext::Main;
				}
				else
				{
					ui_mode = UiMode::Load;
					load_context = LoadContext::Main;
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
				if (index >= 0 && index < wav_file_count)
				{
				}
				else
				{
				}
				if (DeleteFileAtIndex(index))
				{
					request_delete_scan = true;
					delete_confirm = false;
					request_delete_redraw = true;
				}
				else
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
#if STORAGE_SERVICE_SAVE
					save_success = false;
					save_started = true;
					save_frames_written = 0;
					StorageService::Op op = {};
					op.kind = StorageService::OpKind::SaveStart;
					CopyString(op.path, fsi.GetSDPath(), sizeof(op.path));
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
					last_wave_ui_idx = ui_idx;
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
					last_wave_ui_idx = ui_idx;
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
			if (ui_idx != last_wave_ui_idx)
			{
				DrawRecordRecording(uir);
				last_wave_ui_idx = ui_idx;
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
		if ((ui_mode == UiMode::Record && record_state == RecordState::Review)
			|| (IsPerformUiMode(ui_mode) && sample_loaded)
			|| (ui_mode == UiMode::FxDetail && sample_loaded)
			|| (ui_mode == UiMode::Edt && sample_loaded)
			|| (ui_mode == UiMode::Load && load_context == LoadContext::Edt)
			|| (ui_mode == UiMode::Load && delete_mode))
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
		hw.UpdateLeds();
		hw.DelayMs(1);
	}
}
