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
constexpr int32_t kLoadTargetCount = 2;
constexpr int32_t kRecordTargetCount = 2;
constexpr int32_t kRecordTargetSave = 0;
constexpr int32_t kRecordTargetDiscard = 1;
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
constexpr bool kPlaybackVerboseLog = false;
constexpr float kPi = 3.14159265f;
constexpr float kTwoPi = 6.2831853f;
constexpr int kDisplayW = 128;
constexpr int kDisplayH = 64;
constexpr int kPlayBpm = 120;
constexpr int32_t kPlayBpmMin = 40;
constexpr int32_t kPlayBpmMax = 240;
constexpr int32_t kPlayTrackCount = 4;
constexpr int kPlayStepCount = 16;
constexpr uint32_t kPreviewReadBudgetMs = 2;
constexpr size_t kPreviewBufferFrames = 4096;
constexpr size_t kPreviewReadFrames = 256;

static const char* kSaveColors[] =
{
	"Red",
	"White",
	"Orange",
	"Amber",
	"Gold",
	"Yellow",
	"Lime",
	"Green",
	"Emerald",
	"Teal",
	"Cyan",
	"Azure",
	"Blue",
	"Indigo",
	"Violet",
	"Magenta",
	"Pink",
	"Rose",
	"Crimson",
	"Scarlet",
	"Maroon",
	"Purple",
	"Plum",
	"Lavender",
	"Lemon",
	"Sage",
	"Olive",
	"Brown",
	"Umber",
	"Slate",
	"Black",
};

static const char* kSaveAdjectives[] =
{
	"Calm",
	"Quiet",
	"Soft",
	"Still",
	"Dream",
	"Wish",
	"Sigh",
	"Hush",
	"Ashen",
	"Chill",
	"Lunar",
	"Solar",
	"Aura",
	"Chord",
	"Tempo",
	"Pulse",
	"Reflect",
	"Vibe",
	"Tone",
	"Hum",
	"Fern",
	"Moss",
	"Briar",
	"Grove",
	"Shade",
	"River",
	"Raw",
	"Shore",
	"Tidal",
	"Wave",
	"Spray",
	"Rain",
	"Cloud",
	"Spark",
	"Smoke",
	"Snow",
	"Ice",
	"Orbit",
	"Nova",
	"Comet",
	"Star",
	"Sky",
	"Void",
	"Ether",
	"Zen",
	"Thin",
	"Cliff",
	"Ridge",
	"Peak",
	"Vale",
	"Field",
	"Path",
	"Trail",
	"Omni",
	"Sad",
	"Harp",
	"Viola",
	"Bass",
	"Organ",
	"Flute",
	"Drone",
	"Scale",
	"Note",
	"Desert",
	"Heath",
	"Cove",
	"Delta",
	"Bland",
	"Reef",
	"Loam",
	"Glint",
	"Fable",
	"Verse",
	"Rhyme",
	"Psalm",
	"Chant",
	"Veil",
	"Glow",
	"Curve",
	"Fragile",
	"Cursed",
	"Magic",
	"Holy",
	"Fever",
};

static const char* kSaveNouns[] =
{
	"Mist",
	"Horizon",
	"Ember",
	"Drift",
	"Canopy",
	"Echo",
	"Mound",
	"Stillness",
	"Moonlight",
	"Ash",
	"Breeze",
	"Meadow",
	"Shadow",
	"Tides",
	"Sorrow",
	"Thicket",
	"Dew",
	"Lumins",
	"Dusk",
	"Frost",
	"Hollow",
	"Murmur",
	"Storm",
	"Feather",
	"Veil",
	"Ripple",
	"Retro",
	"Bloom",
	"Stone",
	"Whisper",
	"Dawn",
	"Drip",
	"Current",
	"Glimmer",
	"Hammer",
	"Soil",
	"Flame",
	"Canter",
	"Fogbank",
	"Reflection",
	"Raindrop",
	"Skyward",
	"Tempest",
	"Arrow",
	"Leaf",
	"Glow",
	"Thunderhead",
	"Pathway",
	"Snowdrift",
	"Again",
	"Breath",
	"Overcast",
	"Pool",
	"Petal",
	"Lilt",
	"Club",
	"Cradle",
	"Moonrise",
	"Erosion",
	"Sunshower",
	"Branch",
	"Gales",
	"Pool",
	"Shimmer",
	"Silence",
	"Lineman",
	"Brother",
	"Solstice",
	"Driftwood",
	"Clearing",
	"Glowfly",
	"Buffalo",
	"Carnage",
	"Basin",
	"Frostline",
	"Skyscape",
	"Undertone",
	"Wreckage",
	"Lichen",
	"Daybreak",
	"Haze",
	"Bramble",
	"Rainlight",
	"Snowmelt",
	"Stones",
	"Backwind",
	"Twilight",
	"Stream",
	"Overhang",
	"Quiver",
	"Shadowland",
	"Fieldnote",
	"Softrain",
	"String",
	"Sundown",
	"Nightmoves",
	"Tiger",
	"Mother",
	"Lowtide",
	"Earthsong",
};
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
constexpr float kAmpEnvMaxMs = 1000.0f;
constexpr float kAmpEnvStepMs = 20.0f;

enum class UiMode : int32_t
{
	Main,
	Load,
	LoadModeSelect,
	LoadStub,
	LoadTarget,
	Perform,
	Edt,
	FxDetail,
	Play,
	PlayTrack,
	Record,
	PresetSaveStub,
	Shift,
};

enum class LoadDestination : int32_t
{
	Play = 0,
	Perform = 1,
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

enum class PlaySelectMode : int32_t
{
	Bpm,
	TrackLabel,
	GridCell,
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

DaisyPod    hw;
PodDisplay  display;
SdmmcHandler   sdcard;
FatFSInterface fsi;
Encoder      encoder_r;
Switch       shift_button;
DSY_SDRAM_BSS ReverbSc reverb;
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
char wav_files[kMaxWavFiles][kMaxWavNameLen];
char loaded_sample_name[kMaxWavNameLen] = {0};
int32_t load_lines = 1;
int32_t load_line_height = 1;
int32_t load_chars_per_line = 1;

enum class SampleContext : int32_t
{
	Perform,
	Play,
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
static SampleState play_sample_state;
static SampleContext current_sample_context = SampleContext::Perform;

DSY_SDRAM_BSS int16_t perform_sample_buffer_l[kMaxSampleSamples];
DSY_SDRAM_BSS int16_t perform_sample_buffer_r[kMaxSampleSamples];
DSY_SDRAM_BSS int16_t play_sample_buffer_l[kMaxSampleSamples];
DSY_SDRAM_BSS int16_t play_sample_buffer_r[kMaxSampleSamples];
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
volatile float playback_amp = 0.0f;
volatile uint32_t playback_env_samples = 0;
volatile bool playback_release_active = false;
volatile float playback_release_pos = 0.0f;
volatile float playback_release_start = 0.0f;
volatile int32_t current_note = -1;

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
	float reverb_shimmer = 0.0f;
	bool sat_params_initialized = false;
	bool reverb_params_initialized = false;
	bool delay_params_initialized = false;
	bool mod_params_initialized = false;
};

struct TrackSampleState
{
	bool loaded = false;
	char name[kMaxWavNameLen] = {};
	float trim_start = 0.0f;
	float trim_end = 1.0f;
};

enum class PerformContext : int32_t
{
	Main,
	Track,
};

enum class LoadContext : int32_t
{
	Main,
	Track,
	Edt,
};

static PerformState main_perform_state;
static PerformState play_perform_state;
static PerformState track_perform_state[kPlayTrackCount];
static PerformContext perform_context = PerformContext::Main;
static int32_t perform_context_track = 0;
static TrackSampleState track_samples[kPlayTrackCount];
static UiMode edt_prev_mode = UiMode::Perform;
static SampleContext edt_sample_context = SampleContext::Perform;
static UiMode fx_detail_prev_mode = UiMode::Perform;
static UiMode load_prev_mode = UiMode::Main;
static LoadContext load_context = LoadContext::Main;
static int32_t load_context_track = 0;
static int32_t load_mode_index = 0;
static LoadStubMode load_stub_mode = LoadStubMode::Presets;
static volatile bool request_track_sample_load = false;
static volatile int32_t request_track_sample_index = -1;

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

struct WaveformCache
{
	int16_t min[128] = {};
	int16_t max[128] = {};
	bool ready = false;
	bool dirty = false;
};

static WaveformCache perform_waveform_cache;
static WaveformCache play_waveform_cache;
static bool waveform_from_recording = false;
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
volatile int32_t record_target_index = kRecordTargetSave;
volatile uint32_t record_countdown_start_ms = 0;
volatile size_t record_pos = 0;
volatile bool record_waveform_pending = false;
volatile int32_t encoder_r_accum = 0;
volatile bool encoder_r_button_press = false;
volatile bool request_length_redraw = false;
static int32_t play_bpm = kPlayBpm;
static uint32_t play_step_ms = 0;
static PlaySelectMode play_select_mode = PlaySelectMode::Bpm;
static int32_t play_select_row = 0;
static int32_t play_select_col = 0;
static bool play_screen_dirty = true;
static bool playhead_running = false;
static int32_t playhead_step = 0;
static uint32_t playhead_last_step_ms = 0;
static bool play_steps[kPlayTrackCount][kPlayStepCount] = {};
volatile bool request_playhead_redraw = false;
volatile bool button1_press = false;
volatile bool button2_press = false;
volatile bool request_playback_stop_log = false;
volatile float reverb_wet = kReverbDefaultWet;
volatile float reverb_pre = 0.5f;
volatile float reverb_damp = 0.5f;
volatile float reverb_decay = 0.5f;
volatile float reverb_shimmer = 0.5f;
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
static uint32_t delay_snow_next_ms = 0;
static uint32_t midi_ignore_until_ms = 0;

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

static void ComputeWaveform();

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
const char* kShiftMenuLabels[kShiftMenuCount] = {"SAVE PRESET", "DELETE"};

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
		case UiMode::LoadModeSelect: return "LOAD_MODE";
		case UiMode::LoadStub: return "LOAD_STUB";
		case UiMode::LoadTarget: return "LOAD_TARGET";
		case UiMode::Perform: return "PERFORM";
		case UiMode::Edt: return "EDT";
		case UiMode::FxDetail: return "FX_DETAIL";
		case UiMode::Play: return "PLAY";
		case UiMode::PlayTrack: return "PLAY_TRACK";
		case UiMode::Record: return "RECORD";
		case UiMode::PresetSaveStub: return "PRESET_SAVE_STUB";
		case UiMode::Shift: return "SHIFT";
		default: return "UNKNOWN";
	}
}

static const char* LoadDestinationName(LoadDestination dest)
{
	switch (dest)
	{
		case LoadDestination::Play: return "PLAY";
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
	static uint32_t save_name_seed = 0;
	if (save_name_seed == 0)
	{
		save_name_seed = static_cast<uint32_t>(System::GetNow()) ^ 0xA5A5A5A5u;
	}
	auto next_rand = [&]()
	{
		save_name_seed = (save_name_seed * 1664525u) + 1013904223u;
		return save_name_seed;
	};

	const size_t color_count = ArraySize(kSaveColors);
	const size_t adj_count = ArraySize(kSaveAdjectives);
	const size_t noun_count = ArraySize(kSaveNouns);

	for (int attempt = 0; attempt < 5000; ++attempt)
	{
		const char* adj = kSaveAdjectives[next_rand() % adj_count];
		const char* color = kSaveColors[next_rand() % color_count];
		const char* noun = kSaveNouns[next_rand() % noun_count];

		char base[64];
		const int base_len = snprintf(base, sizeof(base), "%s%s%s", adj, color, noun);
		if (base_len <= 0)
		{
			continue;
		}
		if (static_cast<size_t>(base_len) + 4 >= out_len)
		{
			continue;
		}
		if (base_len >= static_cast<int>(sizeof(base) - 1))
		{
			continue;
		}

		for (int suffix = 0; suffix < 100; ++suffix)
		{
			char name[64];
			if (suffix == 0)
			{
				if (static_cast<size_t>(base_len) + 4 >= sizeof(name))
				{
					continue;
				}
				snprintf(name, sizeof(name), "%s.wav", base);
			}
			else
			{
				if (static_cast<size_t>(base_len) + 6 >= sizeof(name))
				{
					continue;
				}
				snprintf(name, sizeof(name), "%s%02d.wav", base, suffix);
			}
			if (StrLen(name) >= out_len)
			{
				continue;
			}
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

static bool IsPlayUiMode(UiMode mode)
{
	return (mode == UiMode::Play || mode == UiMode::PlayTrack);
}

static bool IsPerformUiMode(UiMode mode)
{
	return (mode == UiMode::Perform || mode == UiMode::PlayTrack);
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

static SampleState& SampleStateForContext(SampleContext ctx)
{
	return (ctx == SampleContext::Perform) ? perform_sample_state : play_sample_state;
}

static WaveformCache& WaveformCacheForContext(SampleContext ctx)
{
	return (ctx == SampleContext::Perform) ? perform_waveform_cache : play_waveform_cache;
}

static void SaveWaveformCache(SampleContext ctx)
{
	WaveformCache& cache = WaveformCacheForContext(ctx);
	std::memcpy(cache.min, waveform_min, sizeof(waveform_min));
	std::memcpy(cache.max, waveform_max, sizeof(waveform_max));
	cache.ready = waveform_ready;
	cache.dirty = waveform_dirty;
}

static void LoadWaveformCache(SampleContext ctx)
{
	WaveformCache& cache = WaveformCacheForContext(ctx);
	std::memcpy(waveform_min, cache.min, sizeof(waveform_min));
	std::memcpy(waveform_max, cache.max, sizeof(waveform_max));
	waveform_ready = cache.ready;
	waveform_dirty = cache.dirty;
}

static void SaveSampleState(SampleState& state)
{
	CopyString(state.name, loaded_sample_name, kMaxWavNameLen);
	state.length = sample_length;
	state.play_start = sample_play_start;
	state.play_end = sample_play_end;
	state.rate = sample_rate;
	state.channels = sample_channels;
	state.loaded = sample_loaded;
	state.trim_start = trim_start;
	state.trim_end = trim_end;
	state.from_recording = waveform_from_recording;
}

static void LoadSampleState(const SampleState& state)
{
	CopyString(loaded_sample_name, state.name, kMaxWavNameLen);
	sample_length = state.length;
	sample_play_start = state.play_start;
	sample_play_end = state.play_end;
	sample_rate = state.rate;
	sample_channels = state.channels;
	sample_loaded = state.loaded;
	trim_start = state.trim_start;
	trim_end = state.trim_end;
	waveform_from_recording = state.from_recording;
}

static void SetSampleContext(SampleContext ctx)
{
	if (current_sample_context == ctx)
	{
		return;
	}
	SaveWaveformCache(current_sample_context);
	SaveSampleState(SampleStateForContext(current_sample_context));
	current_sample_context = ctx;
	if (ctx == SampleContext::Perform)
	{
		sample_buffer_l = perform_sample_buffer_l;
		sample_buffer_r = perform_sample_buffer_r;
	}
	else
	{
		sample_buffer_l = play_sample_buffer_l;
		sample_buffer_r = play_sample_buffer_r;
	}
	LoadSampleState(SampleStateForContext(ctx));
	LoadWaveformCache(ctx);
	if (!waveform_ready && sample_loaded)
	{
		ComputeWaveform();
		waveform_ready = true;
		waveform_dirty = true;
	}
}

static void UpdatePlayStepMs()
{
	if (play_bpm < kPlayBpmMin)
	{
		play_bpm = kPlayBpmMin;
	}
	if (play_bpm > kPlayBpmMax)
	{
		play_bpm = kPlayBpmMax;
	}
	const uint32_t bpm = static_cast<uint32_t>(play_bpm);
	play_step_ms = (bpm > 0) ? (60000U / (bpm * 4U)) : 0;
	if (play_step_ms == 0)
	{
		play_step_ms = 1;
	}
}

static bool TrackHasSampleState(int32_t track)
{
	if (track < 0 || track >= kPlayTrackCount)
	{
		return false;
	}
	return track_samples[track].loaded && track_samples[track].name[0] != '\0';
}

static bool TrackSampleMatchesLoaded(int32_t track)
{
	if (!TrackHasSampleState(track))
	{
		return false;
	}
	if (!sample_loaded || sample_length < 1)
	{
		return false;
	}
	return strcmp(loaded_sample_name, track_samples[track].name) == 0;
}

static bool ComputeTrimWindowFrames(float trim_start_in,
									float trim_end_in,
									size_t length,
									size_t& out_start,
									size_t& out_end)
{
	if (length < 1)
	{
		out_start = 0;
		out_end = length;
		return false;
	}
	if (length < 2)
	{
		out_start = 0;
		out_end = length;
		return (out_end > out_start);
	}

	float start = trim_start_in;
	float end = trim_end_in;
	if (start < 0.0f) start = 0.0f;
	if (end > 1.0f) end = 1.0f;
	if (end < 0.0f) end = 0.0f;
	if (start > 1.0f) start = 1.0f;

	const float min_norm = 2.0f / static_cast<float>(length);
	if (end - start < min_norm)
	{
		end = start + min_norm;
		if (end > 1.0f)
		{
			end = 1.0f;
			start = end - min_norm;
		}
	}

	uint32_t snap_start = static_cast<uint32_t>(start * static_cast<float>(length));
	uint32_t snap_end = static_cast<uint32_t>(end * static_cast<float>(length));
	if (snap_end <= snap_start)
	{
		snap_end = snap_start + 2;
	}
	if (snap_end > length)
	{
		snap_end = static_cast<uint32_t>(length);
	}
	out_start = snap_start;
	out_end = snap_end;
	return (out_end > out_start);
}

static void StartSequencerVoiceWindow(size_t window_start, size_t window_end)
{
	if (!sample_loaded || sample_length < 1)
	{
		return;
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
	if (window_end <= window_start)
	{
		return;
	}

	int voice_index = -1;
	for (int i = 0; i < kPerformVoiceCount; ++i)
	{
		if (!perform_voices[i].active)
		{
			voice_index = i;
			break;
		}
	}
	if (voice_index < 0)
	{
		voice_index = 0;
	}

	PerformVoice& voice = perform_voices[voice_index];
	voice.active = true;
	voice.releasing = false;
	voice.note = -1;
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
	voice.rate = sr / hw.AudioSampleRate();
	voice.offset = window_start;
	voice.length = window_end - window_start;
}

static void TriggerSequencerStep(int32_t step)
{
	if (step < 0 || step >= kPlayStepCount)
	{
		return;
	}
	for (int track = 0; track < kPlayTrackCount; ++track)
	{
		if (!play_steps[track][step])
		{
			continue;
		}
		if (!TrackSampleMatchesLoaded(track))
		{
			continue;
		}
		size_t window_start = 0;
		size_t window_end = 0;
		if (!ComputeTrimWindowFrames(track_samples[track].trim_start,
									 track_samples[track].trim_end,
									 sample_length,
									 window_start,
									 window_end))
		{
			continue;
		}
		StartSequencerVoiceWindow(window_start, window_end);
	}
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
	state.reverb_shimmer = reverb_shimmer;
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
	reverb_shimmer = state.reverb_shimmer;
	sat_params_initialized = state.sat_params_initialized;
	reverb_params_initialized = state.reverb_params_initialized;
	delay_params_initialized = state.delay_params_initialized;
	mod_params_initialized = state.mod_params_initialized;
	fx_params_dirty = true;
}

enum class FxContext : int32_t
{
	Perform,
	Play,
	Track,
};

static FxContext fx_context = FxContext::Perform;

static void SaveFxContext()
{
	switch (fx_context)
	{
		case FxContext::Perform:
			CapturePerformState(main_perform_state);
			break;
		case FxContext::Play:
			CapturePerformState(play_perform_state);
			break;
		case FxContext::Track:
			if (perform_context_track >= 0 && perform_context_track < kPlayTrackCount)
			{
				CapturePerformState(track_perform_state[perform_context_track]);
			}
			break;
		default:
			break;
	}
}

static void SetFxContext(FxContext ctx, int32_t track = 0)
{
	if (fx_context == ctx)
	{
		if (ctx != FxContext::Track || track == perform_context_track)
		{
			return;
		}
	}
	SaveFxContext();
	fx_context = ctx;
	if (ctx == FxContext::Perform)
	{
		perform_context = PerformContext::Main;
		ApplyPerformState(main_perform_state);
	}
	else if (ctx == FxContext::Play)
	{
		perform_context = PerformContext::Main;
		ApplyPerformState(play_perform_state);
	}
	else
	{
		if (track < 0 || track >= kPlayTrackCount)
		{
			return;
		}
		perform_context = PerformContext::Track;
		perform_context_track = track;
		ApplyPerformState(track_perform_state[track]);
	}
}

static void StoreTrackSampleState(int32_t track)
{
	if (track < 0 || track >= kPlayTrackCount)
	{
		return;
	}
	track_samples[track].loaded = sample_loaded;
	CopyString(track_samples[track].name, loaded_sample_name, kMaxWavNameLen);
	track_samples[track].trim_start = trim_start;
	track_samples[track].trim_end = trim_end;
}

static void InitTrackStates()
{
	CapturePerformState(main_perform_state);
	play_perform_state = main_perform_state;
	for (int t = 0; t < kPlayTrackCount; ++t)
	{
		track_perform_state[t] = main_perform_state;
		track_samples[t].loaded = false;
		track_samples[t].name[0] = '\0';
		track_samples[t].trim_start = 0.0f;
		track_samples[t].trim_end = 1.0f;
	}
}

static void EnterPlayTrack(int32_t track)
{
	if (track < 0 || track >= kPlayTrackCount)
	{
		return;
	}
	SetSampleContext(SampleContext::Play);
	SetFxContext(FxContext::Track, track);
	request_track_sample_load = true;
	request_track_sample_index = track;
	ui_mode = UiMode::PlayTrack;
	request_perform_redraw = true;
}

static void ExitPlayTrack()
{
	if (perform_context == PerformContext::Track)
	{
		CapturePerformState(track_perform_state[perform_context_track]);
		StoreTrackSampleState(perform_context_track);
	}
	SetFxContext(FxContext::Play);
	ui_mode = UiMode::Play;
	play_screen_dirty = true;
	request_playhead_redraw = true;
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
	FIL* file = &wav_file;
	UINT bytes_read = 0;

	playback_active = false;
	playback_phase = 0.0f;
	sample_loaded = false;
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

static bool ApplyTrackSampleState(int32_t track)
{
	if (track < 0 || track >= kPlayTrackCount)
	{
		return false;
	}
	const TrackSampleState& state = track_samples[track];
	if (!state.loaded || state.name[0] == '\0')
	{
		sample_loaded = false;
		sample_length = 0;
		loaded_sample_name[0] = '\0';
		waveform_ready = false;
		waveform_dirty = true;
		request_length_redraw = true;
		return false;
	}
	char path[64];
	BuildFilePath(state.name, path, sizeof(path));
	CopyString(loaded_sample_name, state.name, kMaxWavNameLen);
	if (!LoadSampleFromPath(path))
	{
		return false;
	}
	trim_start = state.trim_start;
	trim_end = state.trim_end;
	UpdateTrimFrames();
	waveform_dirty = true;
	request_length_redraw = true;
	return true;
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
			const char* line1 = "SELECT";
			const char* line2 = "WAV";
			const int text_w1 = TinyStringWidth(line1);
			const int text_w2 = TinyStringWidth(line2);
			const int text_x1 = box.x + (kBoxW - text_w1) / 2;
			const int text_x2 = box.x + (kBoxW - text_w2) / 2;
			const int text_y = box.y + (kBoxH - (Font5x7::H * 2) - 2) / 2;
			DrawTinyString(line1, text_x1, text_y, !is_selected);
			DrawTinyString(line2, text_x2, text_y + Font5x7::H + 2, !is_selected);
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

		if (sample_length > 1)
		{
			bool has_playhead = false;
			float norm = 0.0f;
			if (playback_active)
			{
				norm = playback_phase / static_cast<float>(sample_length - 1);
				has_playhead = true;
			}
			else
			{
				for (int v = 0; v < kPerformVoiceCount; ++v)
				{
					const auto& voice = perform_voices[v];
					if (voice.active && voice.length > 0)
					{
						const float pos = static_cast<float>(voice.offset) + voice.phase;
						norm = pos / static_cast<float>(sample_length - 1);
						has_playhead = true;
						break;
					}
				}
			}
			if (has_playhead)
			{
				if (norm < 0.0f)
				{
					norm = 0.0f;
				}
				else if (norm > 1.0f)
				{
					norm = 1.0f;
				}
				int trim_x0 = preview_x0
					+ static_cast<int>(trim_start * static_cast<float>(preview_w - 1) + 0.5f);
				int trim_x1 = preview_x0
					+ static_cast<int>(trim_end * static_cast<float>(preview_w - 1) + 0.5f);
				if (trim_x0 > trim_x1)
				{
					const int tmp = trim_x0;
					trim_x0 = trim_x1;
					trim_x1 = tmp;
				}
				int play_x = preview_x0
					+ static_cast<int>(norm * static_cast<float>(preview_w - 1) + 0.5f);
				if (play_x < trim_x0)
				{
					play_x = trim_x0;
				}
				else if (play_x > trim_x1)
				{
					play_x = trim_x1;
				}
				display.DrawLine(play_x, preview_y0, play_x, preview_y1, on);
			}
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
		case LoadDestination::Perform: return 1;
		default: return 0;
	}
}

static LoadDestination LoadTargetFromDisplayIndex(int32_t index)
{
	switch (index)
	{
		case 0: return LoadDestination::Play;
		case 1: return LoadDestination::Perform;
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
	const int box_w = kDisplayW - (kMargin * 2);

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

	draw_box(kMargin, top_y, box_w, top_h, "PLAY", selected_idx == 0);
	draw_box(kMargin, bottom_y, box_w, bottom_h, "PERFORM", selected_idx == 1);
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
	display.WriteString("L=NO  R=YES", font, true);
	display.Update();
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

static void StartRecording()
{
	record_pos = 0;
	sample_length = 0;
	sample_loaded = false;
	perform_attack_norm = 0.0f;
	perform_release_norm = 0.0f;
	ResetPerformVoices();
	playback_active = false;
	sample_channels = 1;
	sample_rate = 48000;
	trim_start = 0.0f;
	trim_end = 1.0f;
	CopyString(loaded_sample_name, "UNSAVED AUDIO", kMaxWavNameLen);
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
	snprintf(label, sizeof(label), "%ld bpM", static_cast<long>(play_bpm));
	const int label_x = 1;
	const int label_y = 2;
	const int label_len = static_cast<int>(StrLen(label));
	const int label_w = (label_len > 0)
		? (label_len * kPlayTinyW + (label_len - 1) * kPlayTinySpacing)
		: 0;
	const int label_h = kPlayTinyH;
	const bool bpm_selected = (play_select_mode == PlaySelectMode::Bpm);
	if (bpm_selected && label_w > 0)
	{
		int x0 = label_x - 1;
		int y0 = label_y - 1;
		int x1 = label_x + label_w;
		int y1 = label_y + label_h;
		if (x0 < 0) x0 = 0;
		if (y0 < 0) y0 = 0;
		if (x1 >= kDisplayW) x1 = kDisplayW - 1;
		if (y1 >= kDisplayH) y1 = kDisplayH - 1;
		if (x0 <= x1 && y0 <= y1)
		{
			display.DrawRect(x0, y0, x1, y1, true, true);
		}
		DrawPlayTinyText(label_x, label_y, label, false);
	}
	else
	{
		DrawPlayTinyText(label_x, label_y, label, true);
	}

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
					const bool track_selected = (play_select_mode == PlaySelectMode::TrackLabel
						&& play_select_row == i);
					if (track_selected)
					{
						const int row_top = sections_start_y + i * section_h;
						const int row_bottom = row_top + section_h - 1;
						int x0 = 0;
						int y0 = row_top;
						int x1 = label_box_w - 1;
						int y1 = row_bottom;
						if (x0 < 0) x0 = 0;
						if (y0 < 0) y0 = 0;
						if (x1 >= kDisplayW) x1 = kDisplayW - 1;
						if (y1 >= kDisplayH) y1 = kDisplayH - 1;
						if (x0 <= x1 && y0 <= y1)
						{
							display.DrawRect(x0, y0, x1, y1, true, true);
						}
						DrawPlayTinyText(2, y, label_num, false);
					}
					else
					{
						DrawPlayTinyText(2, y, label_num, true);
					}
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
				if (play_select_mode == PlaySelectMode::GridCell
					&& play_select_row >= 0 && play_select_row < kPlayTrackCount
					&& play_select_col >= 0 && play_select_col < kPlayStepCount)
				{
					const int row_top = sections_start_y + play_select_row * section_h;
					const int row_bottom = row_top + section_h - 1;
					const int step_left = sequencer_x
						+ (play_select_col * sequencer_w) / kPlayStepCount;
					const int step_right = sequencer_x
						+ ((play_select_col + 1) * sequencer_w) / kPlayStepCount;
					int x0 = step_left + 1;
					int x1 = step_right - 1;
					int y0 = row_top + 1;
					int y1 = row_bottom - 1;
					if (x0 < sequencer_x) x0 = sequencer_x;
					if (x1 >= kDisplayW) x1 = kDisplayW - 1;
					if (y0 < sections_start_y) y0 = sections_start_y;
					if (y1 >= kDisplayH) y1 = kDisplayH - 1;
					if (x0 <= x1 && y0 <= y1)
					{
						display.DrawRect(x0, y0, x1, y1, true, true);
					}
				}

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

				auto draw_grid_px = [&](int px, int py, bool on)
				{
					if (px >= 0 && px < kDisplayW && py >= 0 && py < kDisplayH)
					{
						display.DrawPixel(px, py, on);
					}
				};

				auto draw_diamond = [&](int cx, int cy, bool on)
				{
					for (int dy = -2; dy <= 2; ++dy)
					{
						const int span = 2 - ((dy < 0) ? -dy : dy);
						for (int dx = -span; dx <= span; ++dx)
						{
							draw_grid_px(cx + dx, cy + dy, on);
						}
					}
				};

				for (int track = 0; track < kPlayTrackCount; ++track)
				{
					const int row_top = sections_start_y + track * section_h;
					const int row_bottom = row_top + section_h - 1;
					const int cy = row_top + (section_h / 2);
					if (row_top < sections_start_y || row_bottom >= kDisplayH)
					{
						continue;
					}
					for (int step = 0; step < kPlayStepCount; ++step)
					{
						if (!play_steps[track][step])
						{
							continue;
						}
						const int step_left = sequencer_x + (step * sequencer_w) / kPlayStepCount;
						const int step_right = sequencer_x + ((step + 1) * sequencer_w) / kPlayStepCount;
						const int cx = (step_left + step_right) / 2;
						const bool cell_selected = (play_select_mode == PlaySelectMode::GridCell
							&& play_select_row == track
							&& play_select_col == step);
						draw_diamond(cx, cy, !cell_selected);
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
	if(!waveform_ready || !waveform_dirty)
		return;

	waveform_dirty = false;
	display.Fill(false);

	const int W = 128;
	const int H = 64;
	const int text_h = Font_6x8.FontHeight + 1;
	const int mid = text_h + (H - text_h) / 2;

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
									 fader_offsets,
									 nullptr,
									 nullptr,
									 nullptr);
		}
	}
	display.Update();
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
	display.Update();
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
	display.Update();
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
	display.Update();
}

static void DrawRecordTargetScreen(int32_t selected)
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("SAVE SAMPLE?", font, true);
	display.SetCursor(0, (font.FontHeight + 2) * 2);
	display.WriteString("L=NO  R=YES", font, true);
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
}

static void StopPlayback(uint8_t note)
{
	if (note == current_note)
	{
		if (IsPerformUiMode(ui_mode) && playback_active)
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

static void StartPerformVoice(int32_t note)
{
	size_t window_start = 0;
	size_t window_end = 0;
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

static void HandleMidiMessage(MidiEvent msg)
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
	static float fx_chain_fade_gain = 1.0f;
	static float fx_chain_fade_target = 1.0f;
	static int32_t fx_chain_fade_samples_left = 0;
	static bool fx_chain_pause_pending = false;
	static bool fx_chain_paused = false;
	static uint32_t fx_chain_last_move_ms = 0;
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
	shift_button.Debounce();
	const bool shift_pressed = shift_button.Pressed();
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
	if (hw.button1.RisingEdge())
	{
		button1_press = true;
	}
	if (hw.button2.RisingEdge())
	{
		button2_press = true;
	}
	preview_hold = (ui_mode == UiMode::Load
		&& !(kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode))
		? hw.button1.Pressed()
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
			LogLine("Shift menu select: %s", kShiftMenuLabels[shift_menu_index]);
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
			SetSampleContext(SampleContext::Play);
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
			midi_ignore_until_ms = now_ms + 200;
		}
		else if (encoder_r_pressed && menu_index == 3)
		{
			SetSampleContext(SampleContext::Play);
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
						request_load_destination = LoadDestination::Perform;
						request_load_sample = true;
						request_load_index = load_selected;
					}
					else if (load_context == LoadContext::Track)
					{
						request_load_destination = LoadDestination::Play;
						request_load_sample = true;
						request_load_index = load_selected;
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
				playback_active = false;
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
				playback_active = false;
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
	else if (!ui_blocked && IsPerformUiMode(ui_mode))
	{
		const bool track_mode = (ui_mode == UiMode::PlayTrack);
		const bool fx_select_active = (perform_index == kPerformFxIndex && fx_window_active);
		const bool amp_select_active = (perform_index == kPerformAmpIndex && amp_window_active);
		const bool flt_select_active = (perform_index == kPerformFltIndex && flt_window_active);
		const bool fx_reorder_active = fx_select_active && shift_pressed;
		const bool has_sample = (sample_loaded && sample_length > 0);
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
							reverb_shimmer = 0.5f;
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
				if (!has_sample && track_mode)
				{
					load_context = LoadContext::Track;
					load_context_track = perform_context_track;
					load_prev_mode = UiMode::PlayTrack;
					ui_mode = UiMode::Load;
					load_selected = 0;
					load_scroll = 0;
					request_load_scan = true;
				}
				else if (!has_sample)
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
					edt_sample_context = IsPlayUiMode(ui_mode) ? SampleContext::Play : SampleContext::Perform;
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
			if (!fx_chain_paused)
			{
				const int32_t fade_samples
					= static_cast<int32_t>((out_sr * (kFxChainFadeMs * 0.001f)) + 0.5f);
				fx_chain_fade_target = 0.0f;
				fx_chain_fade_samples_left = (fade_samples > 0) ? fade_samples : 1;
				fx_chain_pause_pending = true;
			}
		}
		else if (fx_select_active && encoder_r_inc != 0)
		{
			const int32_t fx_id = fx_chain_order[fx_fader_index];
			const float step = FxWetStep(fx_id);
			volatile float* target = FxWetTarget(fx_id);
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
		else if (amp_select_active && encoder_r_inc != 0)
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
		else if (flt_select_active && encoder_r_inc != 0)
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
				if (fx_select_active || amp_select_active || flt_select_active)
				{
					fx_window_active = false;
					amp_window_active = false;
					flt_window_active = false;
					request_perform_redraw = true;
				}
				else if (track_mode)
				{
					ExitPlayTrack();
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
			if (shift_button.Pressed())
			{
				if (dl > 0) dl = 1;
				else if (dl < 0) dl = -1;
				if (dr > 0) dr = 1;
				else if (dr < 0) dr = -1;
			}
			AdjustTrimNormalized(dl, dr, shift_button.Pressed());
			if (perform_context == PerformContext::Track)
			{
				StoreTrackSampleState(perform_context_track);
			}
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
			request_perform_redraw = true;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::FxDetail)
	{
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
					sat_mode = (sat_mode == 0) ? 1 : 0;
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
						const int next_idx = ClampI(cur_idx + encoder_r_inc, 0, kBitResoStepCount - 1);
						next = BitResoValueFromIndex(next_idx);
					}
					else
					{
						const float step = steps[idx];
						next = current + (static_cast<float>(encoder_r_inc) * step);
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
					int32_t next = chorus_mode + encoder_r_inc;
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
						next = freeze_on ? 0.0f : 1.0f;
					}
					else
					{
						const float step = steps[idx];
						next = current + (static_cast<float>(encoder_r_inc) * step);
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
			ui_mode = fx_detail_prev_mode;
			if (ui_mode == UiMode::Perform)
			{
				midi_ignore_until_ms = now_ms + 200;
			}
			request_perform_redraw = true;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::Play)
	{
		bool selection_changed = false;
		if (encoder_l_inc != 0)
		{
			const int32_t dir = (encoder_l_inc > 0) ? 1 : -1;
			int32_t steps = (encoder_l_inc > 0) ? encoder_l_inc : -encoder_l_inc;
			for (int32_t s = 0; s < steps; ++s)
			{
				if (play_select_mode == PlaySelectMode::Bpm)
				{
					if (dir > 0)
					{
						play_select_mode = PlaySelectMode::TrackLabel;
						play_select_row = 0;
						selection_changed = true;
					}
				}
				else if (play_select_mode == PlaySelectMode::TrackLabel)
				{
					if (dir > 0)
					{
						play_select_mode = PlaySelectMode::GridCell;
						play_select_col = 0;
						selection_changed = true;
					}
				}
				else
				{
					if (dir > 0)
					{
						if (play_select_col < kPlayStepCount - 1)
						{
							play_select_col += 1;
							selection_changed = true;
						}
					}
					else
					{
						if (play_select_col > 0)
						{
							play_select_col -= 1;
							selection_changed = true;
						}
						else
						{
							play_select_mode = PlaySelectMode::TrackLabel;
							selection_changed = true;
						}
					}
				}
			}
		}
		if (play_select_mode == PlaySelectMode::Bpm && encoder_r_inc != 0)
		{
			int32_t next_bpm = play_bpm + encoder_r_inc;
			if (next_bpm < kPlayBpmMin)
			{
				next_bpm = kPlayBpmMin;
			}
			if (next_bpm > kPlayBpmMax)
			{
				next_bpm = kPlayBpmMax;
			}
			if (next_bpm != play_bpm)
			{
				play_bpm = next_bpm;
				UpdatePlayStepMs();
				play_screen_dirty = true;
			}
		}
		else if (encoder_r_inc != 0)
		{
			const int32_t dir = (encoder_r_inc > 0) ? 1 : -1;
			int32_t steps = (encoder_r_inc > 0) ? encoder_r_inc : -encoder_r_inc;
			for (int32_t s = 0; s < steps; ++s)
			{
				if (dir > 0)
				{
					if (play_select_row < (kPlayTrackCount - 1))
					{
						play_select_row += 1;
						selection_changed = true;
					}
				}
				else
				{
					if (play_select_row > 0)
					{
						play_select_row -= 1;
						selection_changed = true;
					}
					else
					{
						play_select_mode = PlaySelectMode::Bpm;
						selection_changed = true;
					}
				}
			}
		}
		if (selection_changed)
		{
			play_screen_dirty = true;
		}
		if (encoder_r_pressed)
		{
			if (play_select_mode == PlaySelectMode::TrackLabel)
			{
				EnterPlayTrack(play_select_row);
			}
			else if (play_select_mode == PlaySelectMode::GridCell)
			{
				if (TrackHasSampleState(play_select_row)
					&& play_select_col >= 0
					&& play_select_col < kPlayStepCount)
				{
					play_steps[play_select_row][play_select_col]
						= !play_steps[play_select_row][play_select_col];
					play_screen_dirty = true;
				}
			}
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

	if (fx_chain_paused)
	{
		if (fx_chain_last_move_ms != 0
			&& (now_ms - fx_chain_last_move_ms) >= kFxChainIdleMs)
		{
			const int32_t fade_samples
				= static_cast<int32_t>((out_sr * (kFxChainFadeMs * 0.001f)) + 0.5f);
			fx_chain_paused = false;
			fx_chain_fade_target = 1.0f;
			fx_chain_fade_samples_left = (fade_samples > 0) ? fade_samples : 1;
			fx_chain_fade_gain = 0.0f;
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
static float cached_sat_drive = 0.0f;
static float cached_sat_mix = 0.0f;
static float cached_sat_bump = 0.0f;
static float cached_sat_smpl = 0.0f;
static float cached_sat_reso = 0.0f;
static int32_t cached_sat_mode = 0;
static float cached_chorus_depth = 0.0f;
static float cached_chorus_mix = 0.0f;
static float cached_chorus_rate = 0.0f;
static int32_t cached_chorus_mode = 0;
static float cached_chorus_wow = 0.0f;
static float cached_tape_rate = 0.0f;
	static float cached_delay_wet = 0.0f;
	static float cached_delay_time = 0.0f;
	static float cached_delay_feedback = 0.0f;
	static float cached_delay_spread = 0.0f;
	static float cached_delay_freeze = 0.0f;
	static float delay_time_smoothed = -1.0f;
	static float delay_feedback_smoothed = -1.0f;
	static float delay_spread_smoothed = -1.0f;
	static float cached_reverb_wet = 0.0f;
	static float cached_reverb_pre = 0.0f;
	static float cached_reverb_damp = 0.0f;
	static float cached_reverb_decay = 0.0f;
	static float cached_reverb_shimmer = 0.0f;
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

	if (fx_params_dirty)
	{
		fx_params_dirty = false;

		cached_sat_drive = sat_drive;
		cached_sat_mix = fx_s_wet;
		cached_sat_bump = sat_tape_bump;
		cached_sat_smpl = sat_bit_smpl;
		cached_sat_reso = sat_bit_reso;
		cached_sat_mode = sat_mode;
		cached_chorus_depth = mod_depth;
		cached_chorus_mix = fx_c_wet;
		cached_chorus_rate = chorus_rate;
		cached_chorus_mode = chorus_mode;
		cached_chorus_wow = chorus_wow;
		cached_tape_rate = tape_rate;
		cached_delay_wet = delay_wet;
		cached_delay_time = delay_time;
		cached_delay_feedback = delay_feedback;
		cached_delay_spread = delay_spread;
		cached_delay_freeze = delay_freeze;
		cached_reverb_wet = reverb_wet;
		cached_reverb_pre = reverb_pre;
		cached_reverb_damp = reverb_damp;
		cached_reverb_decay = reverb_decay;
		cached_reverb_shimmer = reverb_shimmer;

		if (cached_sat_drive < 0.0f) cached_sat_drive = 0.0f;
		if (cached_sat_drive > 1.0f) cached_sat_drive = 1.0f;
		if (cached_sat_mix < 0.0f) cached_sat_mix = 0.0f;
		if (cached_sat_mix > 1.0f) cached_sat_mix = 1.0f;
		if (cached_sat_bump < 0.0f) cached_sat_bump = 0.0f;
		if (cached_sat_bump > 1.0f) cached_sat_bump = 1.0f;
		if (cached_sat_smpl < 0.0f) cached_sat_smpl = 0.0f;
		if (cached_sat_smpl > 1.0f) cached_sat_smpl = 1.0f;
		if (cached_sat_reso < 0.0f) cached_sat_reso = 0.0f;
		if (cached_sat_reso > 1.0f) cached_sat_reso = 1.0f;
		if (cached_chorus_depth < 0.0f) cached_chorus_depth = 0.0f;
		if (cached_chorus_depth > 1.0f) cached_chorus_depth = 1.0f;
		if (cached_chorus_mix < 0.0f) cached_chorus_mix = 0.0f;
		if (cached_chorus_mix > 1.0f) cached_chorus_mix = 1.0f;
		if (cached_chorus_rate < 0.0f) cached_chorus_rate = 0.0f;
		if (cached_chorus_rate > 1.0f) cached_chorus_rate = 1.0f;
		if (cached_chorus_wow < 0.0f) cached_chorus_wow = 0.0f;
		if (cached_chorus_wow > 1.0f) cached_chorus_wow = 1.0f;
		if (cached_tape_rate < 0.0f) cached_tape_rate = 0.0f;
		if (cached_tape_rate > 1.0f) cached_tape_rate = 1.0f;
		if (cached_delay_wet < 0.0f) cached_delay_wet = 0.0f;
		if (cached_delay_wet > 1.0f) cached_delay_wet = 1.0f;
		if (cached_delay_time < 0.0f) cached_delay_time = 0.0f;
		if (cached_delay_time > 1.0f) cached_delay_time = 1.0f;
		if (cached_delay_feedback < 0.0f) cached_delay_feedback = 0.0f;
		if (cached_delay_feedback > 1.0f) cached_delay_feedback = 1.0f;
		if (cached_delay_spread < 0.0f) cached_delay_spread = 0.0f;
		if (cached_delay_spread > 1.0f) cached_delay_spread = 1.0f;
		if (cached_delay_freeze < 0.0f) cached_delay_freeze = 0.0f;
		if (cached_delay_freeze > 1.0f) cached_delay_freeze = 1.0f;
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

		if (cached_sat_mode != last_sat_mode)
		{
			last_sat_mode = cached_sat_mode;
		}
		if (cached_sat_mode == 0)
		{
			const float sat_drive_amt = powf(cached_sat_drive, 0.7f);
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
	static int bit_hold = 0;
	static float bit_hold_l = 0.0f;
	static float bit_hold_r = 0.0f;
	static float reverb_tail_gain = 0.0f;

	const int32_t sat_mode_local = cached_sat_mode;
	const float sat_mix = cached_sat_mix;
	const float bit_reso = cached_sat_reso;
	const float bit_smpl = cached_sat_smpl;
	const int32_t chorus_mode_local = cached_chorus_mode;
	const float chorus_mix = cached_chorus_mix;
	const float delay_mix = cached_delay_wet;
	const float rev_shimmer = cached_reverb_shimmer;
	const float dt = static_cast<float>(size) / out_sr;
	const float time_tau = kDelayTimeSlewMs * 0.001f;
	const float time_alpha = (time_tau > 0.0f) ? (1.0f - expf(-dt / time_tau)) : 1.0f;
	const float param_tau = kDelayParamSlewMs * 0.001f;
	const float param_alpha = (param_tau > 0.0f) ? (1.0f - expf(-dt / param_tau)) : 1.0f;
	const float time_curve = cached_delay_time * cached_delay_time;
	float delay_ms = kDelayTimeMinMs
		+ (time_curve * (kDelayTimeMaxMs - kDelayTimeMinMs));
	const float delay_samples = delay_ms * 0.001f * out_sr;
	const float max_delay = static_cast<float>(kDelayMaxSamples - 1);
	float delay_target = delay_samples;
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
	const bool perform_mode = IsPerformUiMode(ui_mode);
	const bool main_mode = (ui_mode == UiMode::Main);
	const bool fx_allowed = perform_mode || IsPlayUiMode(ui_mode) || ui_mode == UiMode::FxDetail;
	const bool amp_env_active = perform_mode;
	const float amp_attack_ms = AmpEnvMsFromFader(amp_attack);
	const float amp_release_ms = AmpEnvMsFromFader(amp_release);
	const float amp_attack_samples = amp_attack_ms * 0.001f * out_sr;
	const float amp_release_samples = amp_release_ms * 0.001f * out_sr;
	const bool play_seq_mode = IsPlayUiMode(ui_mode) && sample_loaded;
	const bool use_poly = (record_state != RecordState::Recording)
		&& ((perform_mode && sample_loaded) || play_seq_mode);
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
	int32_t fx_order[kPerformFaderCount];
	for (int i = 0; i < kPerformFaderCount; ++i)
	{
		fx_order[i] = fx_chain_order[i];
	}

	auto apply_saturation = [&](float &l, float &r)
	{
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
			const int bits_idx = BitResoIndexFromValue(bit_reso);
			const int bits = kBitResoSteps[bits_idx];
			const float step = 1.0f / powf(2.0f, static_cast<float>(bits - 1));
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
		l = (l * dry_mix) + (rev_l * wet_mix);
		r = (r * dry_mix) + (rev_r * wet_mix);
	};

	float fx_gain = fx_chain_fade_gain;
	int32_t fade_samples_left = fx_chain_fade_samples_left;
	const float fade_step = (fade_samples_left > 0)
		? (fx_chain_fade_target - fx_gain) / static_cast<float>(fade_samples_left)
		: 0.0f;

	for (size_t i = 0; i < size; i++)
	{
		float sig_l = 0.0f;
		float sig_r = 0.0f;
		const bool monitor_active =
			(ui_mode == UiMode::Record
				&& record_state != RecordState::Review
				&& record_state != RecordState::SourceSelect
				&& record_state != RecordState::BackConfirm
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
					samp_l = static_cast<float>(sample_buffer_l[idx]) * kSampleScale * amp;
					const float r = sample_stereo
						? static_cast<float>(sample_buffer_r[idx])
						: static_cast<float>(sample_buffer_l[idx]);
					samp_r = r * kSampleScale * amp;
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
		if (fade_samples_left > 0)
		{
			fx_gain += fade_step;
			--fade_samples_left;
			if (fade_samples_left == 0)
			{
				fx_gain = fx_chain_fade_target;
			}
		}
		if (fx_chain_paused)
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
				case kFxSatIndex: apply_saturation(fx_l, fx_r); break;
				case kFxChorusIndex: apply_chorus(fx_l, fx_r); break;
				case kFxDelayIndex: apply_delay(fx_l, fx_r); break;
				case kFxReverbIndex: apply_reverb(fx_l, fx_r); break;
				default: break;
			}
		}
		out[0][i] = fx_l * fx_gain;
		out[1][i] = fx_r * fx_gain;
	}
	fx_chain_fade_gain = fx_gain;
	fx_chain_fade_samples_left = fade_samples_left;
	if (fx_chain_pause_pending
		&& fx_chain_fade_samples_left == 0
		&& fx_chain_fade_target <= 0.0f)
	{
		fx_chain_paused = true;
		fx_chain_pause_pending = false;
		fx_chain_fade_gain = 0.0f;
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

	UpdatePlayStepMs();
	InitTrackStates();

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
	uint32_t last_perform_playhead_ms = 0;
	bool last_perform_playhead_active = false;
	uint32_t last_edt_playhead_ms = 0;
	bool last_edt_playhead_active = false;
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
				if (!ui_blocked && IsPlayUiMode(ui_mode))
				{
					if (!playhead_running)
					{
						playhead_running = true;
						playhead_step = 0;
						playhead_last_step_ms = System::GetNow();
						play_screen_dirty = true;
						request_playhead_redraw = true;
						TriggerSequencerStep(playhead_step);
					}
					else
					{
						playhead_running = false;
						playhead_last_step_ms = 0;
					play_screen_dirty = true;
					request_playhead_redraw = true;
				}
			}
			else if (!IsPlayUiMode(ui_mode)
					 && sample_loaded
					 && ui_mode != UiMode::Load
					 && ui_mode != UiMode::LoadTarget)
			{
				if (UiLogEnabled())
				{
					LogLine("Button1: playback request (unpitched)");
				}
				if (IsPerformUiMode(ui_mode))
				{
					StartPerformVoice(kBaseMidiNote);
				}
				else
				{
					StartPlayback(kBaseMidiNote, false);
				}
				request_playhead_redraw = true;
			}
		}
			if (!ui_blocked && IsPlayUiMode(ui_mode) && playhead_running)
		{
		const uint32_t now = System::GetNow();
			if (playhead_last_step_ms == 0)
			{
				playhead_last_step_ms = now;
			}
			if (play_step_ms > 0)
			{
				const uint32_t elapsed = now - playhead_last_step_ms;
				if (elapsed >= play_step_ms)
				{
					const uint32_t steps = elapsed / play_step_ms;
					playhead_last_step_ms += steps * play_step_ms;
					const int32_t start_step = playhead_step;
					for (uint32_t s = 0; s < steps; ++s)
					{
						const int32_t step = (start_step + static_cast<int32_t>(s) + 1) % kPlayStepCount;
						TriggerSequencerStep(step);
					}
					playhead_step = (start_step + static_cast<int32_t>(steps)) % kPlayStepCount;
					play_screen_dirty = true;
					request_playhead_redraw = true;
				}
			}
		}
		if (request_load_scan)
		{
			if (!ui_blocked)
			{
				if (kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode)
				{
					request_load_scan = false;
				}
				else
				{
					request_load_scan = false;
					LogLine("Load menu: scan requested");
					ScanSdFiles(true);
				}
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
			else if (kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode)
			{
				request_load_sample = false;
				request_load_index = -1;
				LogLine("Load menu: sample request ignored (presets placeholder)");
			}
			else
			{
				request_load_sample = false;
				const int32_t index = request_load_index;
				request_load_index = -1;
				const LoadDestination dest = request_load_destination;
				SampleContext target_ctx = SampleContext::Play;
				if (load_context == LoadContext::Edt)
				{
					target_ctx = edt_sample_context;
				}
				else if (load_context == LoadContext::Track)
				{
					target_ctx = SampleContext::Play;
				}
				else if (dest == LoadDestination::Perform)
				{
					target_ctx = SampleContext::Perform;
				}
				SetSampleContext(target_ctx);
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
					if (load_context == LoadContext::Track)
					{
						const int32_t track = load_context_track;
						if (track >= 0 && track < kPlayTrackCount)
						{
							track_samples[track].loaded = sample_loaded;
							CopyString(track_samples[track].name, loaded_sample_name, kMaxWavNameLen);
							track_samples[track].trim_start = trim_start;
							track_samples[track].trim_end = trim_end;
						}
						LogLine("Load success, entering TRACK menu");
						ui_mode = UiMode::PlayTrack;
						request_perform_redraw = true;
					}
					else if (load_context == LoadContext::Edt)
					{
						LogLine("Load success, returning to EDT");
						ui_mode = UiMode::Edt;
						if (sample_loaded && sample_length > 0)
						{
							waveform_ready = true;
						}
						waveform_dirty = true;
						request_length_redraw = true;
					}
					else if (dest == LoadDestination::Perform)
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
					load_context = LoadContext::Main;
				}
				else
				{
					LogLine("Load failed");
					ui_mode = UiMode::Load;
					if (load_context != LoadContext::Track)
					{
						load_context = LoadContext::Main;
					}
				}
			}
		}
		if (request_track_sample_load)
		{
			if (ui_blocked)
			{
				request_track_sample_load = false;
				request_track_sample_index = -1;
				LogLine("Track sample load ignored during UI block");
			}
			else
			{
				request_track_sample_load = false;
				const int32_t track = request_track_sample_index;
				request_track_sample_index = -1;
				if (perform_context == PerformContext::Track && track == perform_context_track)
				{
					ApplyTrackSampleState(track);
					request_perform_redraw = true;
				}
			}
		}

		if (!ui_blocked)
		{
			const bool preview_allowed = (ui_mode == UiMode::Load
				&& !(kLoadPresetsPlaceholder && load_context == LoadContext::Main && !delete_mode)
				&& wav_file_count > 0);
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
			const bool was_play = IsPlayUiMode(last_mode);
			const bool is_play = IsPlayUiMode(mode);
			if (was_play && !is_play)
			{
				playhead_running = false;
				playhead_step = 0;
				playhead_last_step_ms = 0;
				play_screen_dirty = true;
			}
			if (mode == UiMode::FxDetail)
			{
				if (IsPlayUiMode(fx_detail_prev_mode))
				{
					SetSampleContext(SampleContext::Play);
					if (fx_detail_prev_mode == UiMode::PlayTrack)
					{
						SetFxContext(FxContext::Track, perform_context_track);
					}
					else
					{
						SetFxContext(FxContext::Play);
					}
				}
				else
				{
					SetSampleContext(SampleContext::Perform);
					SetFxContext(FxContext::Perform);
				}
			}
			else if (mode == UiMode::Edt)
			{
				SetSampleContext(edt_sample_context);
				SetFxContext((edt_sample_context == SampleContext::Play)
					? FxContext::Play
					: FxContext::Perform);
			}
			else if (mode == UiMode::Perform)
			{
				SetSampleContext(SampleContext::Perform);
				SetFxContext(FxContext::Perform);
			}
			else if (mode == UiMode::Play)
			{
				SetSampleContext(SampleContext::Play);
				SetFxContext(FxContext::Play);
			}
			else if (mode == UiMode::PlayTrack)
			{
				SetSampleContext(SampleContext::Play);
				SetFxContext(FxContext::Track, perform_context_track);
			}
			if (IsPerformUiMode(last_mode) && !IsPerformUiMode(mode))
			{
				ResetPerformVoices();
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
					LogLine("Load menu: selected=%ld name=%s",
							static_cast<long>(load_selected),
							(load_selected >= 0 && load_selected < wav_file_count)
								? wav_files[load_selected]
								: "UNKNOWN");
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
			else if (mode == UiMode::Play)
			{
				play_screen_dirty = true;
				DrawPlayScreen();
			}
				else if (mode == UiMode::PlayTrack)
				{
					request_perform_redraw = true;
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
			else if (mode == UiMode::Perform || mode == UiMode::PlayTrack)
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
			}
				else if (mode == UiMode::LoadTarget)
				{
					DrawLoadTargetMenu(load_target_selected);
					LogLine("Load target: %s",
							LoadDestinationName(load_target_selected));
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
			else if (mode == UiMode::Perform || mode == UiMode::PlayTrack)
			{
				const int32_t current = perform_index;
				if (request_perform_redraw || current != last_perform_index)
				{
					request_perform_redraw = false;
					const bool fx_select_active = (current == kPerformFxIndex)
						&& fx_window_active;
					const bool amp_select_active = (current == kPerformAmpIndex)
						&& amp_window_active;
					const bool flt_select_active = (current == kPerformFltIndex)
						&& flt_window_active;
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
		else if (!ui_blocked && mode == UiMode::Record && record_state == RecordState::BackConfirm)
		{
			DrawRecordBackConfirm();
		}
		const bool perform_playhead_active = (!ui_blocked
			&& (mode == UiMode::Perform || mode == UiMode::PlayTrack)
			&& perform_index == kPerformEdtIndex
			&& (playback_active || AnyPerformVoiceActive()));
		if (perform_playhead_active)
		{
			const uint32_t now = System::GetNow();
			if (!last_perform_playhead_active)
			{
				last_perform_playhead_ms = now;
				request_perform_redraw = true;
			}
			else if ((now - last_perform_playhead_ms) >= kPerformPlayheadIntervalMs)
			{
				last_perform_playhead_ms = now;
				request_perform_redraw = true;
			}
		}
		else
		{
			last_perform_playhead_ms = 0;
		}
		last_perform_playhead_active = perform_playhead_active;
		const bool edt_playhead_active = (!ui_blocked
			&& mode == UiMode::Edt
			&& playback_active);
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
		if (IsPlayUiMode(ui_mode))
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
		}
		hw.UpdateLeds();
		hw.DelayMs(10);
	}
}
