#include "daisy_pod.h"
#include "dev/oled_ssd130x.h"
#include "per/tim.h"
#include "util/scopedirqblocker.h"
#include "SamplerConfig.h"
#include "BuildConfig.h"
#include "shared_messages.h"
#include "audio_engine.h"
#include "ui.h"
#include "app_controller.h"
#include "StorageService.h"
#include <cmath>
#include <initializer_list>
//#include <math.h>
#include <cstring>

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

#if ENABLE_PERF_COUNTERS
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
static AudioEngine g_audio_engine;
static StorageService g_storage;
Ui ui;
static AppController app;
static AppContext g_app_ctx;
static AudioShared g_audio_shared;
static daisy::TimerHandle g_ctrl_timer;


Encoder      encoder_r;
Switch       shift_button;


// (moved to UiState)

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

 

PerformState main_perform_state;

// Normalized trim window (0..1 over entire sample)

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


int BitResoIndexFromValue(float value);

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

constexpr float kSilentAmp = 1.0e-4f;
constexpr uint16_t kSilentSamplesToKill = 64;
constexpr size_t kLiveWaveWindowFrames = kSampleRateHz / 2;
constexpr size_t kLiveWaveStride = (kLiveWaveWindowFrames / kWaveCols) > 0
	? (kLiveWaveWindowFrames / kWaveCols)
	: 1;
const AudioUiState& GetAudioUiStateSnapshot(uint8_t& idx);



double NowMs()
{
	return static_cast<double>(System::GetNow());
}

void ApplyPlaybackReverse(bool reverse)
{
	float& playback_reverse = ui.PlaybackReverseRef();
	const bool current = (playback_reverse >= 0.5f);
	if (reverse == current)
	{
		return;
	}
	playback_reverse = reverse ? 1.0f : 0.0f;
	{
		daisy::ScopedIrqBlocker irq;
		ui.PlaybackReverseTargetRef() = reverse;
	}
	g_audio_engine.RequestAudioCmd(kCmdPlaybackReverse);
}


void StartPlaybackAudio(uint8_t note, bool apply_pitch, bool reverse_playback)
{
	const SampleRuntime rt = ui.RtBuf()[ui.RtActiveIdxRef()];
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
	ui.CurrentNoteRef() = note;
	ui.PlaybackAmpRef() = 1.0f;
	ui.PlaybackEnvSamplesRef() = 0;
	ui.PlaybackReleaseActiveRef() = false;
	ui.PlaybackReleasePosRef() = 0.0f;
	ui.PlaybackReleaseStartRef() = 0.0f;
	const uint8_t idx = (note < 128) ? note : 127;
	const float pitch = apply_pitch ? g_audio_engine.NoteRatio(idx) : 1.0f;
	const float sr = (rt.rate == 0) ? 48000.0f : static_cast<float>(rt.rate);
	ui.PlaybackRateRef() = pitch * (sr / hw.AudioSampleRate());
	ui.PlaybackPhaseRef() = static_cast<float>(reverse_playback ? (window_end - 1) : window_start);
	ui.PlaybackActiveRef() = true;
}

void StopPlaybackAudio(uint8_t note, bool apply_release)
{
	if (note == ui.CurrentNoteRef())
	{
		if (apply_release && ui.PlaybackActiveRef())
		{
			ui.PlaybackReleaseActiveRef() = true;
			ui.PlaybackReleasePosRef() = 0.0f;
			ui.PlaybackReleaseStartRef() = -1.0f;
		}
		else
		{
			ui.PlaybackActiveRef() = false;
			ui.PlaybackEnvSamplesRef() = 0;
			ui.PlaybackReleaseActiveRef() = false;
			ui.PlaybackReleasePosRef() = 0.0f;
			ui.PlaybackReleaseStartRef() = 0.0f;
			g_audio_engine.PushAudioEvent(kAudioEventPlaybackStopped);
		}
	}
}

void StopPlaybackAllAudio()
{
	if (ui.PlaybackActiveRef())
	{
		ui.PlaybackActiveRef() = false;
		ui.PlaybackEnvSamplesRef() = 0;
		ui.PlaybackReleaseActiveRef() = false;
		ui.PlaybackReleasePosRef() = 0.0f;
		ui.PlaybackReleaseStartRef() = 0.0f;
		g_audio_engine.PushAudioEvent(kAudioEventPlaybackStopped);
	}
}


static void __attribute__((unused)) HandleMidiMessage(MidiEvent msg)
{
	if (ui.GetMode() == UiMode::Record && ui.RecordStateRef() == RecordState::Recording)
	{
		return;
	}
	if (IsPerformUiMode(ui.GetMode()))
	{
		const bool ignore_note_on = (System::GetNow() < ui.MidiIgnoreUntilMsRef());
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
				g_audio_engine.RequestPlaybackStop((uint8_t)note.note, false);
			}
			else
			{
				g_audio_engine.RequestPlaybackStart((uint8_t)note.note, true);
			}
		}
		break;
		case NoteOff:
	{
		const NoteOffEvent note = msg.AsNoteOff();
		g_audio_engine.RequestPlaybackStop((uint8_t)note.note, false);
	}
	break;
	default:
		break;
	}
}

static void CtrlTimerCb(void* /*data*/)
{
	if (!ui.CtrlTimerRunningRef())
	{
		return;
	}

	hw.ProcessAllControls();
	encoder_r.Debounce();
	shift_button.Debounce();

	const int32_t enc_l_inc = hw.encoder.Increment();
	const int32_t enc_r_inc = encoder_r.Increment();

	if (ui.MidiRxStartedRef())
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
						g_audio_engine.MidiCmdPushIsr(kMidiCmdNoteOff, (uint8_t)n.note, 0);
					}
					else
					{
						if (IsPerformUiMode(ui.GetMode()) && (System::GetNow() < ui.MidiIgnoreUntilMsRef()))
						{
							break;
						}
						g_audio_engine.MidiCmdPushIsr(kMidiCmdNoteOn, (uint8_t)n.note, (uint8_t)n.velocity);
					}
				}
				break;
				case NoteOff:
				{
					const NoteOffEvent n = msg.AsNoteOff();
					g_audio_engine.MidiCmdPushIsr(kMidiCmdNoteOff, (uint8_t)n.note, 0);
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

	ui.AccumulateControls(enc_l_inc, enc_r_inc, ev, shift_held, btn1_held);
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

	ui.CtrlTimerRunningRef() = true;
	g_ctrl_timer.Start();
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	g_audio_engine.Process(in, out, size);
}

static void InitAppContext(AppContext& ctx)
{
	ctx.hw = &hw;
	ctx.display = &display;
	ctx.ui = &ui;
	ctx.audio = &g_audio_engine;
	ctx.storage = &g_storage;

	ctx.perform_voices_active = &ui.PerformVoicesActiveRef();
	ctx.audio_ui_state_buf = ui.AudioUiStateBuf();
	ctx.audio_ui_state_idx = &ui.AudioUiStateIdxRef();
	ctx.record_state = &ui.RecordStateRef();
	ctx.record_source_index = &ui.RecordSourceIndexRef();
	ctx.record_target_index = &ui.RecordTargetIndexRef();
	ctx.record_countdown_start_ms = &ui.RecordCountdownStartMsRef();
	ctx.record_pos = &ui.RecordPosRef();
	ctx.recorded_length_audio = &ui.RecordedLengthAudioRef();
	ctx.record_start_ms = &ui.RecordStartMsRef();
	ctx.record_waveform_pending = &ui.RecordWaveformPendingRef();
	ctx.encoder_r_accum = &ui.EncoderRAccumRef();
	ctx.encoder_r_button_press = &ui.EncoderRButtonPressRef();
	ctx.request_length_redraw = &ui.RequestLengthRedrawRef();
	ctx.request_playhead_redraw = &ui.RequestPlayheadRedrawRef();
	ctx.button1_press = &ui.Button1PressRef();
	ctx.button2_press = &ui.Button2PressRef();
	ctx.request_playback_stop_log = &ui.RequestPlaybackStopLogRef();
	ctx.record_anim_start_ms = &ui.RecordAnimStartMsRef();
	ctx.request_delete_redraw = &ui.RequestDeleteRedrawRef();
	ctx.sample_mem_used_bytes = &ui.SampleMemUsedBytesRef();
	ctx.sample_mem_free_bytes = &ui.SampleMemFreeBytesRef();
	ctx.waveform_cache_bytes = &ui.WaveformCacheBytesRef();

}

static void InitAudioShared(AudioShared& sh)
{
	sh.hw = &hw;
	sh.record_input = ui.GetRecordInputPtr();
	sh.playback_reverse_target = &ui.PlaybackReverseTargetRef();
	sh.audio_params_pub_idx = &ui.AudioParamsPubIdxRef();
	sh.rt_pub_idx = &ui.RtPubIdxRef();
	sh.fx_chain_pub_idx = &ui.FxChainPubIdxRef();
	sh.preview_pub_idx = &ui.PreviewPubIdxRef();
	sh.phones_volume = &ui.PhonesVolumeRef();
	sh.audio_recording_active = &ui.AudioRecordingActiveRef();
	sh.recorded_length_audio = &ui.RecordedLengthAudioRef();
	sh.record_pos = &ui.RecordPosRef();
	sh.reset_voices_pending = &ui.ResetVoicesPendingRef();
	sh.audio_params_buf = ui.AudioParamsBuf();
	sh.audio_params_active_idx = &ui.AudioParamsActiveIdxRef();
	sh.audio_ui_state_buf = ui.AudioUiStateBuf();
	sh.audio_ui_state_idx = &ui.AudioUiStateIdxRef();
	sh.playback_active = &ui.PlaybackActiveRef();
	sh.playback_phase = &ui.PlaybackPhaseRef();
	sh.perform_voices_active = &ui.PerformVoicesActiveRef();
	sh.preview_active = &ui.PreviewActiveRef();
	sh.preview_read_index = &ui.PreviewReadIndexRef();
	sh.preview_read_frac = &ui.PreviewReadFracRef();
	sh.preview_fade_samples_left = &ui.PreviewFadeSamplesLeftRef();
	sh.preview_fade_samples_total = &ui.PreviewFadeSamplesTotalRef();
	sh.preview_ctl_buf = ui.PreviewCtlBuf();
	sh.preview_active_idx = &ui.PreviewActiveIdxRef();
	sh.preview_write_index = &ui.PreviewWriteIndexRef();
	sh.rt_buf = ui.RtBuf();
	sh.rt_active_idx = &ui.RtActiveIdxRef();
	sh.fx_chain_buf = ui.FxChainBuf();
	sh.fx_chain_active_idx = &ui.FxChainActiveIdxRef();
	sh.fx_chain_audio = &ui.FxChainAudioRef();
	sh.fx_chain_audio_valid = &ui.FxChainAudioValidRef();
	sh.fx_params_buf = ui.FxParamsBuf();
	sh.fx_params_idx = &ui.FxParamsIdxRef();
	sh.audio_params_audio_buf = ui.AudioParamsAudioBuf();
	sh.audio_params_audio_idx = &ui.AudioParamsAudioIdxRef();
	sh.delay_time_alpha = &ui.DelayTimeAlphaRef();
	sh.delay_param_alpha = &ui.DelayParamAlphaRef();
#if STORAGE_SERVICE_PREVIEW_STREAM
	sh.preview_preload_active = &ui.PreviewPreloadActiveRef();
	sh.preview_preload_frames = &ui.PreviewPreloadFramesRef();
	sh.preview_pp_ready = ui.PreviewPpReadyPtr();
	sh.preview_pp_active = &ui.PreviewPpActiveRef();
	sh.preview_pp_pos = &ui.PreviewPpPosRef();
	sh.preview_underrun_count = &ui.PreviewUnderrunCountRef();
	sh.preview_rb_min_level = &ui.PreviewRbMinLevelRef();
#endif
	sh.playback_rate = &ui.PlaybackRateRef();
	sh.playback_amp = &ui.PlaybackAmpRef();
	sh.playback_env_samples = &ui.PlaybackEnvSamplesRef();
	sh.playback_release_active = &ui.PlaybackReleaseActiveRef();
	sh.playback_release_pos = &ui.PlaybackReleasePosRef();
	sh.playback_release_start = &ui.PlaybackReleaseStartRef();
	sh.playback_reverse_active = &ui.PlaybackReverseActiveRef();
	sh.preview_hold = &ui.PreviewHoldRef();
	sh.preview_index = &ui.PreviewIndexRef();
	sh.preview_sample_rate = &ui.PreviewSampleRateRef();
	sh.preview_channels = &ui.PreviewChannelsRef();
	sh.active_voice_count = &ui.ActiveVoiceCountRef();
	sh.voice_skip_count = &ui.VoiceSkipCountRef();
	sh.voice_kill_count = &ui.VoiceKillCountRef();
	sh.cpu_load_pct = &ui.CpuLoadPctRef();
	sh.cpu_load_peak_pct = &ui.CpuLoadPeakPctRef();
	sh.callback_cycles_last = &ui.CallbackCyclesLastRef();
	sh.callback_cycles_max = &ui.CallbackCyclesMaxRef();
	sh.callback_overruns = &ui.CallbackOverrunsRef();
	sh.cpu_load_ema = &ui.CpuLoadEmaRef();
	sh.request_playhead_redraw = &ui.RequestPlayheadRedrawRef();
}

int main(void)
{
	hw.Init();
	EnableFtz();
	g_audio_engine.Init(hw);
	#if ENABLE_PERF_COUNTERS
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
	ui.ChorusRateRef() = 0.5f;
	ui.ChorusWowRef() = 0.5f;
	ui.TapeRateRef() = 0.5f;

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

	{
		int16_t* sb_l = nullptr;
		int16_t* sb_r = nullptr;
		g_audio_engine.GetSampleBuffers(sb_l, sb_r);
		ui.RtBuf()[0].l = sb_l;
		ui.RtBuf()[0].r = sb_r;
	}
	ui.RtBuf()[0].length = 0;
	ui.RtBuf()[0].play_start = 0;
	ui.RtBuf()[0].play_end = 0;
	ui.RtBuf()[0].rate = ui.SampleRateRef();
	ui.RtBuf()[0].channels = ui.SampleChannelsRef();
	ui.RtBuf()[0].loaded = false;
	ui.RtBuf()[1] = ui.RtBuf()[0];
	ui.RtPubIdxRef() = 0;
	ui.RtActiveIdxRef() = 0;

	FxChainRuntime fx_init = {};
	const int32_t* fx_order = ui.FxChainOrder();
	for (int i = 0; i < kPerformFaderCount; ++i)
	{
		fx_init.order[i] = static_cast<uint8_t>(fx_order[i]);
	}
	fx_init.count = kPerformFaderCount;
	fx_init.paused = ui.FxChainPausedRef();
	fx_init.pause_pending = ui.FxChainPausePendingRef();
	fx_init.fade_gain = ui.FxChainFadeGainRef();
	fx_init.fade_target = ui.FxChainFadeTargetRef();
	fx_init.fade_samples_left = ui.FxChainFadeSamplesLeftRef();
	ui.FxChainBuf()[0] = fx_init;
	ui.FxChainBuf()[1] = fx_init;
	ui.FxChainPubIdxRef() = 0;
	ui.FxChainActiveIdxRef() = 0;
	ui.FxChainAudioRef() = fx_init;
	ui.FxChainAudioValidRef() = true;

	PreviewControl preview_init = {};
	{
		PreviewBuffers pb = {};
		g_audio_engine.GetPreviewBuffers(pb);
		preview_init.l = pb.buffer;
		preview_init.length = pb.frames;
	}
	preview_init.r = nullptr;
	preview_init.rate = 1.0f;
	preview_init.gain = 1.0f;
	preview_init.mono = true;
	ui.PreviewCtlBuf()[0] = preview_init;
	ui.PreviewCtlBuf()[1] = preview_init;
	ui.PreviewPubIdxRef() = 0;
	ui.PreviewActiveIdxRef() = 0;

	{
		uint32_t bits = 0;
		const UiMode mode = ui.GetMode();
		const bool in_perform_mode = IsPerformUiMode(mode);
		if (in_perform_mode) bits |= kFlagInPerformMode;
		if (mode == UiMode::Main) bits |= kFlagInMainMode;
		if (in_perform_mode || (mode == UiMode::FxDetail))
		{
			bits |= kFlagFxAllowed;
		}
		if (mode == UiMode::Record
			&& ui.RecordStateRef() != RecordState::Review
			&& ui.RecordStateRef() != RecordState::SourceSelect
			&& ui.RecordStateRef() != RecordState::BackConfirm
			&& ui.RecordStateRef() != RecordState::TargetSelect)
		{
			bits |= kFlagMonitorEnabled;
		}
		g_audio_engine.SetAudioFlagsBits(bits);
	}

	InitAppContext(g_app_ctx);
	InitAudioShared(g_audio_shared);
	g_audio_engine.BindShared(&g_audio_shared);
	hw.StartAdc();
	hw.StartAudio(AudioCallback);
	hw.midi.StartReceive();
	ui.MidiRxStartedRef() = true;
	app.Init(&g_app_ctx);
	while (1)
	{
		app.Tick(System::GetNow());
	}
}



 
