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
constexpr float kKnobArc = 4.1887902f; // 240 degrees total sweep (-120 to +120)
constexpr bool kPlaybackVerboseLog = false;
constexpr float kPi = 3.14159265f;
constexpr int kDisplayW = 128;
constexpr int kDisplayH = 64;
constexpr uint32_t kPreviewReadBudgetMs = 2;
constexpr size_t kPreviewBufferFrames = 4096;
constexpr size_t kPreviewReadFrames = 256;

enum class UiMode : int32_t
{
	Main,
	Load,
	LoadTarget,
	Play,
	Record,
	Tune,
	AdsrSelect,
	Shift,
};

enum class LoadDestination : int32_t
{
	Play = 0,
	Bake = 1,
};

enum class RecordState : int32_t
{
	SourceSelect,
	Armed,
	Countdown,
	Recording,
	Review,
};

enum class RecordInput : int32_t
{
	LineIn,
	Mic,
};

DaisyPod    hw;
PodDisplay  display;
SdmmcHandler   sdcard;
FatFSInterface fsi;
Encoder      encoder_r;
Switch       shift_button;

volatile UiMode ui_mode = UiMode::Main;
volatile int32_t menu_index = 0;
volatile int32_t shift_menu_index = 0;
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
volatile bool request_sample_pitch_estimate = false;

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

DSY_SDRAM_BSS int16_t sample_buffer_l[kMaxSampleSamples];
DSY_SDRAM_BSS int16_t sample_buffer_r[kMaxSampleSamples];
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
volatile int32_t current_note = -1;
volatile float last_tune_freq_hz = 440.0f;
float sample_pitch_hz = 0.0f;
int32_t adsr_selected = 0;
size_t adsr_a_end = 0;
size_t adsr_d_end = 0;
size_t adsr_s_end = 0;
size_t adsr_r_end = 0;
size_t adsr_last_init_start = 0;
size_t adsr_last_init_end = 0;
bool adsr_initialized = false;
bool request_adsr_redraw = false;

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
static const char* waveform_title = nullptr;
static int16_t live_wave_min[128];
static int16_t live_wave_max[128];
static int16_t live_wave_peak = 1;
static bool live_wave_dirty = false;
static int32_t live_wave_last_col = -1;
static float pitch_buf[4096];
static float pitch_scores[4096];

constexpr size_t kRecordMaxFrames = static_cast<size_t>(kRecordMaxSeconds) * 48000U;
constexpr uint32_t kRecordCountdownMs = 4000;
volatile RecordState record_state = RecordState::Armed;
volatile int32_t record_source_index = 0;
volatile uint32_t record_countdown_start_ms = 0;
volatile size_t record_pos = 0;
volatile bool record_waveform_pending = false;
volatile int32_t encoder_r_accum = 0;
volatile bool encoder_r_button_press = false;
volatile bool request_length_redraw = false;
volatile bool request_playhead_redraw = false;
volatile bool button1_press = false;
volatile bool button2_press = false;
volatile bool request_playback_stop_log = false;
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
int32_t tune_selected = 0;
int32_t tune_coarse = 0;
int32_t tune_fine = 0;
int32_t tune_level_pos = -120;
volatile bool request_tune_redraw = false;
float tune_phase = 0.0f;
constexpr bool kUiLogsEnabled = false;
static double record_anim_start_ms = -1.0;
static uint8_t record_text_mask[kDisplayH][kDisplayW];
static uint8_t record_invert_mask[kDisplayH][kDisplayW];
static uint8_t record_fb_buf[kDisplayH][kDisplayW];
static uint8_t record_bold_mask[kDisplayH][kDisplayW];
static bool request_shift_redraw = false;

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
	if (ui_mode == UiMode::Tune)
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
		case UiMode::Play: return "PLAY";
		case UiMode::Record: return "RECORD";
		case UiMode::Tune: return "TUNE";
		case UiMode::AdsrSelect: return "ADSR_SELECT";
		case UiMode::Shift: return "SHIFT";
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

static float FreqToMidi(float freq_hz)
{
	if (freq_hz <= 0.0f)
	{
		return 24.0f;
	}
	const float midi = 69.0f + 12.0f * log2f(freq_hz / 440.0f);
	if (midi < 24.0f)
	{
		return 24.0f;
	}
	if (midi > 108.0f)
	{
		return 108.0f;
	}
	return midi;
}

static int FreqToStripX(float freq_hz, int strip_x, int strip_w)
{
	if (strip_w <= 1)
	{
		return strip_x;
	}
	const float midi = FreqToMidi(freq_hz);
	const float norm = (midi - 24.0f) / (108.0f - 24.0f);
	const float clamped = (norm < 0.0f) ? 0.0f : (norm > 1.0f ? 1.0f : norm);
	const float x = static_cast<float>(strip_x) + clamped * static_cast<float>(strip_w - 1);
	return strip_x + static_cast<int>(x - static_cast<float>(strip_x) + 0.5f);
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
		adsr_initialized = false;
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
	adsr_initialized = false;
	if (ui_mode == UiMode::AdsrSelect)
	{
		request_adsr_redraw = true;
	}
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
	if (ui_mode == UiMode::AdsrSelect)
	{
		request_adsr_redraw = true;
	}
}

static void EnsureAdsrInit()
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
	const size_t window_len = (window_end > window_start) ? (window_end - window_start) : 0;
	if (!adsr_initialized || window_start != adsr_last_init_start || window_end != adsr_last_init_end)
	{
		adsr_initialized = true;
		adsr_last_init_start = window_start;
		adsr_last_init_end = window_end;
		adsr_a_end = window_start;
		adsr_d_end = window_start + (window_len * 2) / 4;
		adsr_s_end = window_start + (window_len * 3) / 4;
		adsr_r_end = window_end;
		if (adsr_d_end < adsr_a_end) adsr_d_end = adsr_a_end;
		if (adsr_s_end < adsr_d_end) adsr_s_end = adsr_d_end;
		if (adsr_r_end < adsr_s_end) adsr_r_end = adsr_s_end;
	}
}

static void MoveAdsrMarker(int index, int32_t delta)
{
	if (delta == 0 || !sample_loaded)
	{
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
	const size_t window_len = (window_end > window_start) ? (window_end - window_start) : 0;
	if (window_len == 0)
	{
		return;
	}

	size_t* markers[4] = {&adsr_a_end, &adsr_d_end, &adsr_s_end, &adsr_r_end};
	const size_t base_step = (window_len >= 256) ? (window_len / 256) : 1;
	auto scaled_step = [base_step](int32_t d) -> int64_t
	{
		int32_t mag = (d < 0) ? -d : d;
		if (mag < 1) mag = 1;
		int32_t log2_mag = 0;
		int32_t tmp = mag;
		while (tmp > 1) { tmp >>= 1; ++log2_mag; }
		const int64_t scale = static_cast<int64_t>(2) * (1 + static_cast<int64_t>(log2_mag));
		return static_cast<int64_t>(base_step) * scale;
	};

	const int64_t step = scaled_step(delta);
	const int64_t move = static_cast<int64_t>(delta) * step;

	size_t current = *markers[index];
	int64_t next = static_cast<int64_t>(current) + move;

	size_t min_bound = (index == 0) ? window_start : (*markers[index - 1] + 1);
	size_t max_bound = (index == 3) ? window_end : (*markers[index + 1] - 1);
	if (max_bound < min_bound)
	{
		max_bound = min_bound;
	}

	if (next < static_cast<int64_t>(min_bound))
	{
		next = static_cast<int64_t>(min_bound);
	}
	if (next > static_cast<int64_t>(max_bound))
	{
		next = static_cast<int64_t>(max_bound);
	}

	*markers[index] = static_cast<size_t>(next);
}

static bool EstimateSamplePitchHzWindow(float& out_hz)
{
	if (!sample_loaded || sample_length < 64 || sample_rate == 0)
	{
		return false;
	}

	const float kMinFreq = 32.703f;   // C1
	const float kMaxFreq = 4186.01f;  // C8
	const int kDownsample = 4;
	const float kConfidenceThresh = 0.4f;

	size_t region_start = sample_play_start;
	size_t region_end = sample_play_end;
	if (region_end > sample_length || region_end == 0)
	{
		region_end = sample_length;
	}
	if (region_end <= region_start)
	{
		return false;
	}
	const size_t region_len = region_end - region_start;

	auto analyze = [&](size_t target_n, float& best_hz) -> bool
	{
		if (target_n == 0 || target_n > sizeof(pitch_buf) / sizeof(pitch_buf[0]))
		{
			return false;
		}

		size_t needed_frames = target_n * static_cast<size_t>(kDownsample);
		if (needed_frames > region_len)
		{
			const size_t possible_n = region_len / static_cast<size_t>(kDownsample);
			if (possible_n < 128)
			{
				return false;
			}
			target_n = possible_n;
			needed_frames = target_n * static_cast<size_t>(kDownsample);
		}

		size_t start = region_start;
		if (needed_frames < region_len)
		{
			const size_t slack = region_len - needed_frames;
			start = region_start + slack / 2;
		}
		if (start + needed_frames > sample_length)
		{
			start = (sample_length > needed_frames) ? (sample_length - needed_frames) : 0;
		}

		const size_t actual_n = target_n;
		float mean = 0.0f;
		for (size_t i = 0; i < actual_n; ++i)
		{
			const size_t idx = start + i * static_cast<size_t>(kDownsample);
			float s = static_cast<float>(sample_buffer_l[idx]);
			if (sample_channels == 2)
			{
				s = 0.5f * (s + static_cast<float>(sample_buffer_r[idx]));
			}
			mean += s;
		}
		mean /= static_cast<float>(actual_n);

		float energy0 = 0.0f;
		for (size_t i = 0; i < actual_n; ++i)
		{
			const size_t idx = start + i * static_cast<size_t>(kDownsample);
			float s = static_cast<float>(sample_buffer_l[idx]);
			if (sample_channels == 2)
			{
				s = 0.5f * (s + static_cast<float>(sample_buffer_r[idx]));
			}
			const float v = s * kSampleScale - mean * kSampleScale;
			pitch_buf[i] = v;
			energy0 += v * v;
		}

		if (energy0 <= 1e-9f)
		{
			return false;
		}

		const float ds_rate = static_cast<float>(sample_rate) / static_cast<float>(kDownsample);
		size_t lag_min = static_cast<size_t>(ceilf(ds_rate / kMaxFreq));
		size_t lag_max = static_cast<size_t>(floorf(ds_rate / kMinFreq));
		if (lag_min < 2)
		{
			lag_min = 2;
		}
		if (lag_max >= actual_n - 1)
		{
			lag_max = (actual_n > 1) ? (actual_n - 2) : 1;
		}
		if (lag_max <= lag_min)
		{
			return false;
		}

		const size_t scores_limit = (lag_max + 2 < (sizeof(pitch_scores) / sizeof(pitch_scores[0])))
			? (lag_max + 2)
			: (sizeof(pitch_scores) / sizeof(pitch_scores[0]));
		for (size_t i = 0; i < scores_limit; ++i)
		{
			pitch_scores[i] = 0.0f;
		}

		for (size_t lag = lag_min; lag <= lag_max; ++lag)
		{
			const size_t limit = actual_n - lag;
			float corr = 0.0f;
			for (size_t i = 0; i < limit; ++i)
			{
				corr += pitch_buf[i] * pitch_buf[i + lag];
			}
			const float score = corr / energy0;
			if (lag < (sizeof(pitch_scores) / sizeof(pitch_scores[0])))
			{
				pitch_scores[lag] = score;
			}
		}

		float best_peak_score = -1.0f;
		size_t best_peak_lag = lag_min;
		for (size_t lag = lag_min + 1; lag + 1 <= lag_max; ++lag)
		{
			const float s = pitch_scores[lag];
			if (s > pitch_scores[lag - 1] && s >= pitch_scores[lag + 1])
			{
				if (s > best_peak_score)
				{
					best_peak_score = s;
					best_peak_lag = lag;
				}
			}
		}

		if (best_peak_score <= 0.0f)
		{
			return false;
		}

		if (best_peak_score < kConfidenceThresh)
		{
			return false;
		}

		const float near_thresh = best_peak_score * 0.9f;
		size_t chosen_lag = best_peak_lag;
		for (size_t lag = lag_min + 1; lag + 1 <= lag_max; ++lag)
		{
			const float s = pitch_scores[lag];
			if (s > pitch_scores[lag - 1] && s >= pitch_scores[lag + 1] && s >= near_thresh)
			{
				if (lag > chosen_lag)
				{
					chosen_lag = lag;
				}
			}
		}

		best_hz = ds_rate / static_cast<float>(chosen_lag);
		return true;
	};

	float hz = 0.0f;
	if (analyze(2048, hz))
	{
		out_hz = hz;
		return true;
	}
	if (analyze(4096, hz))
	{
		out_hz = hz;
		return true;
	}
	return false;
}

static bool LoadSampleFromPath(const char* path)
{
	FIL* file = &wav_file;
	UINT bytes_read = 0;

	playback_active = false;
	playback_phase = 0.0f;
	sample_loaded = false;
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
	sample_pitch_hz = 0.0f;
	if (!sample_loaded)
	{
		LogLine("Load failed: no audio data read");
		return false;
	}
	trim_start = 0.0f;
	trim_end = 1.0f;
	LogLine("Load complete: %lu frames", static_cast<unsigned long>(sample_length));
	waveform_from_recording = false;
	ComputeWaveform();
	waveform_ready = true;
	waveform_dirty = true;
	UpdateTrimFrames();
	adsr_initialized = false;
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

static void DrawLoadTargetMenu(LoadDestination selected)
{
	const FontDef font = Font_6x8;
	display.Fill(false);

	display.SetCursor(0, 0);
	display.WriteString("LOAD TO?", font, true);

	const int box_y = font.FontHeight + 4;
	const int box_h = 40;
	const int box_w = 56;
	const int gap = 8;
	const int start_x = (static_cast<int>(display.Width()) - (box_w * 2 + gap)) / 2;

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

	draw_box(start_x, "PLAY", selected == LoadDestination::Play);
	draw_box(start_x + box_w + gap, "BAKE", selected == LoadDestination::Bake);
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
	sample_pitch_hz = 0.0f;
	playback_active = false;
	sample_channels = 1;
	sample_rate = 48000;
	trim_start = 0.0f;
	trim_end = 1.0f;
	CopyString(loaded_sample_name, "RECORD", kMaxWavNameLen);
	adsr_initialized = false;
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

static float DbToLin(float db)
{
	return powf(10.0f, db / 20.0f);
}

static void DrawKnob(int cx,
					 int cy,
					 int radius,
					 const char* label,
					 int32_t value,
					 int32_t min_val,
					 int32_t max_val,
					 bool selected)
{
	const FontDef font = Font_6x8;
	const int box_w = 30;
	const int box_h = font.FontHeight + 4;
	const int box_x = cx - box_w / 2;
	const int box_y = 0;
	display.DrawRect(box_x,
					 box_y,
					 box_x + box_w - 1,
					 box_y + box_h - 1,
					 true,
					 selected);
	const int label_x = box_x + (box_w - font.FontWidth * static_cast<int>(StrLen(label))) / 2;
	const int label_y = box_y + 2;
	display.SetCursor(label_x, label_y);
	display.WriteString(label, font, !selected);

	display.DrawCircle(cx, cy, radius, true);
	display.DrawCircle(cx, cy, radius - 1, true);

	const float norm = (max_val == min_val)
		? 0.0f
		: (static_cast<float>(value - min_val)
			/ static_cast<float>(max_val - min_val));
	const float angle = (-kPi * 0.5f) + (norm - 0.5f) * kKnobArc;
	const float dx = cosf(angle);
	const float dy = sinf(angle);
	const int line_len = radius - 2;
	const int x2 = cx + static_cast<int>(dx * line_len);
	const int y2 = cy + static_cast<int>(dy * line_len);
	display.DrawLine(cx, cy, x2, y2, true);
}

static void DrawTuneScreen()
{
	display.Fill(false);
	const int radius = 12;
	const int y = 32;
	DrawKnob(22, y, radius, "LVL", tune_level_pos, -120, 120, tune_selected == 2);
	DrawKnob(64, y, radius, "CRSE", tune_coarse, -48, 48, tune_selected == 0);
	DrawKnob(106, y, radius, "FINE", tune_fine, -100, 100, tune_selected == 1);

	const int strip_margin_x = 4;
	const int strip_gap_y = 4;
	const int strip_height = 10;
	int strip_x = strip_margin_x;
	int strip_w = static_cast<int>(display.Width()) - strip_margin_x * 2;
	if (strip_w < 2)
	{
		strip_w = 2;
	}
	int strip_y = y + radius + strip_gap_y;
	if (strip_y + strip_height > static_cast<int>(display.Height()))
	{
		strip_y = static_cast<int>(display.Height()) - strip_height;
		if (strip_y < 0)
		{
			strip_y = 0;
		}
	}
	const int strip_bottom = strip_y + strip_height - 1;
	display.DrawRect(strip_x,
					 strip_y,
					 strip_x + strip_w - 1,
					 strip_bottom,
					 true,
					 false);

	const int triangle_x = FreqToStripX(last_tune_freq_hz, strip_x, strip_w);
	const int tri_base_y = strip_bottom - 1;
	const int tri_height = 4;
	for (int dy = 0; dy < tri_height; ++dy)
	{
		const int row_y = tri_base_y - dy;
		if (row_y < strip_y)
		{
			break;
		}
		int x0 = triangle_x - dy;
		int x1 = triangle_x + dy;
		if (x0 < strip_x)
		{
			x0 = strip_x;
		}
		if (x1 > strip_x + strip_w - 1)
		{
			x1 = strip_x + strip_w - 1;
		}
		display.DrawLine(x0, row_y, x1, row_y, true);
	}

	if (sample_pitch_hz > 0.0f)
	{
		const int circle_x = FreqToStripX(sample_pitch_hz, strip_x, strip_w);
		const int r = 2;
		int circle_y = strip_y + strip_height / 2;
		if (circle_y - r < strip_y)
		{
			circle_y = strip_y + r;
		}
		if (circle_y + r > strip_bottom)
		{
			circle_y = strip_bottom - r;
		}
		for (int dy = -r; dy <= r; ++dy)
		{
			const int yy = circle_y + dy;
			const int span = static_cast<int>(sqrtf(static_cast<float>(r * r - dy * dy)));
			int x0 = circle_x - span;
			int x1 = circle_x + span;
			if (x0 < strip_x)
			{
				x0 = strip_x;
			}
			if (x1 > strip_x + strip_w - 1)
			{
				x1 = strip_x + strip_w - 1;
			}
			display.DrawLine(x0, yy, x1, yy, true);
		}
	}
	display.Update();
}

static void DrawAdsrSelectScreen()
{
	EnsureAdsrInit();
	display.Fill(false);

	const FontDef font = Font_6x8;
	const char* labels[4] = {"A", "D", "S", "R"};
	const int boxes = 4;
	const int box_w = static_cast<int>(display.Width()) / boxes;
	const int box_h = font.FontHeight + 6;

	for (int i = 0; i < boxes; ++i)
	{
		const int x0 = i * box_w;
		const int x1 = (i == boxes - 1) ? static_cast<int>(display.Width()) - 1 : (x0 + box_w - 1);
		const bool sel = (i == adsr_selected);
		display.DrawRect(x0, 0, x1, box_h - 1, true, sel);
		const int label_w = static_cast<int>(StrLen(labels[i])) * font.FontWidth;
		const int lx = x0 + (box_w - label_w) / 2;
		const int ly = (box_h - font.FontHeight) / 2;
		display.SetCursor(lx, ly);
		display.WriteString(labels[i], font, !sel);
	}

	const int wave_top = box_h + 4;
	int wave_h = static_cast<int>(display.Height()) - wave_top;
	if (wave_h < 2)
	{
		display.Update();
		return;
	}
	const int wave_center = wave_top + wave_h / 2;

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
	const size_t window_len = (window_end > window_start) ? (window_end - window_start) : 0;

	if (!sample_loaded || window_len == 0)
	{
		display.SetCursor(0, wave_top);
		display.WriteString("NO WINDOW", font, true);
		display.Update();
		return;
	}

	const int width = static_cast<int>(display.Width());
	const float scale = static_cast<float>(wave_h / 2 - 1) * kSampleScale;
	for (int x = 0; x < width; ++x)
	{
		const size_t start = window_start + (window_len * static_cast<size_t>(x)) / static_cast<size_t>(width);
		size_t end = window_start + (window_len * static_cast<size_t>(x + 1)) / static_cast<size_t>(width);
		if (end <= start)
		{
			end = start + 1;
		}
		if (end > window_end)
		{
			end = window_end;
		}

		int16_t min_val = 32767;
		int16_t max_val = -32768;
		for (size_t i = start; i < end; ++i)
		{
			int32_t sample = sample_buffer_l[i];
			if (sample_channels == 2)
			{
				sample = (sample + sample_buffer_r[i]) / 2;
			}
			const int16_t samp = static_cast<int16_t>(sample);
			if (samp < min_val) min_val = samp;
			if (samp > max_val) max_val = samp;
		}

		int y1 = wave_center - static_cast<int>(max_val * scale);
		int y2 = wave_center - static_cast<int>(min_val * scale);
		if (y1 < wave_top) y1 = wave_top;
		if (y2 > wave_top + wave_h - 1) y2 = wave_top + wave_h - 1;
		display.DrawLine(x, y1, x, y2, true);
	}

	auto MarkerX = [&](size_t pos) -> int
	{
		if (window_len == 0) return 0;
		const uint64_t num = (pos - window_start) * static_cast<uint64_t>(width);
		int x = static_cast<int>(num / window_len);
		if (x < 0) x = 0;
		if (x >= width) x = width - 1;
		return x;
	};

	const int markers[4] = {
		MarkerX(adsr_a_end),
		MarkerX(adsr_d_end),
		MarkerX(adsr_s_end),
		MarkerX(adsr_r_end),
	};
	for (int i = 0; i < 4; ++i)
	{
		const int x = markers[i];
		display.DrawLine(x, wave_top, x, wave_top + wave_h - 1, true);
	}

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
		playback_active = false;
		request_playback_stop_log = true;
	}
}

static void HandleMidiMessage(MidiEvent msg)
{
	if (ui_mode == UiMode::Record && record_state == RecordState::Recording)
	{
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
	const bool ui_blocked = (sd_init_in_progress || save_in_progress);
	shift_button.Debounce();
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
			ui_mode = UiMode::Play;
			if(sample_loaded && sample_length > 0)
			{
				waveform_ready = true;
			}
			waveform_dirty = true;
			request_length_redraw = true;
		}
		else if (encoder_r_pressed && menu_index == 3)
		{
			LogLine("Play menu selected (not implemented)");
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
			int32_t next = static_cast<int32_t>(load_target_selected) + encoder_l_inc;
			while (next < 0)
			{
				next += 2;
			}
			while (next >= 2)
			{
				next -= 2;
			}
			load_target_selected = static_cast<LoadDestination>(next);
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
				LogLine("Record review: enter TUNE");
				ui_mode = UiMode::Tune;
				tune_selected = 0;
				tune_coarse = 0;
				tune_fine = 0;
				tune_level_pos = -120;
				request_tune_redraw = true;
				tune_phase = 0.0f;
			}
		}
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Main;
			record_state = RecordState::SourceSelect;
			playback_active = false;
			record_anim_start_ms = -1.0;
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
	else if (!ui_blocked && ui_mode == UiMode::Tune)
	{
		if (encoder_l_inc != 0)
		{
			tune_selected += encoder_l_inc;
			while (tune_selected < 0)
			{
				tune_selected += 3;
			}
			while (tune_selected >= 3)
			{
				tune_selected -= 3;
			}
			request_tune_redraw = true;
		}
		const int32_t tune_step_coarse = encoder_r_inc * 4;
		const int32_t tune_step_fast = encoder_r_inc * 8;
		if (encoder_r_inc != 0)
		{
			if (tune_selected == 0)
			{
				tune_coarse += tune_step_coarse;
				if (tune_coarse < -48)
				{
					tune_coarse = -48;
				}
				if (tune_coarse > 48)
				{
					tune_coarse = 48;
				}
			}
			else if (tune_selected == 1)
			{
				tune_fine += tune_step_fast;
				if (tune_fine < -100)
				{
					tune_fine = -100;
				}
				if (tune_fine > 100)
				{
					tune_fine = 100;
				}
			}
			else
			{
				tune_level_pos += tune_step_fast;
				if (tune_level_pos < -120)
				{
					tune_level_pos = -120;
				}
				if (tune_level_pos > 120)
				{
					tune_level_pos = 120;
				}
			}
			request_tune_redraw = true;
		}
		if (encoder_r_pressed)
		{
			ui_mode = UiMode::AdsrSelect;
			adsr_selected = 0;
			adsr_initialized = false;
			request_adsr_redraw = true;
		}
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::AdsrSelect;
			adsr_selected = 0;
			adsr_initialized = false;
			request_adsr_redraw = true;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::Play)
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
			ui_mode = UiMode::Main;
		}
	}
	else if (!ui_blocked && ui_mode == UiMode::AdsrSelect)
	{
		EnsureAdsrInit();
		if (encoder_l_inc != 0)
		{
			adsr_selected += encoder_l_inc;
			while (adsr_selected < 0) adsr_selected += 4;
			while (adsr_selected >= 4) adsr_selected -= 4;
			request_adsr_redraw = true;
		}
		if (encoder_r_inc != 0)
		{
			MoveAdsrMarker(adsr_selected, encoder_r_inc);
			request_adsr_redraw = true;
		}
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Tune;
			request_tune_redraw = true;
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
	}

	for (size_t i = 0; i < size; i++)
	{
		float sig_l = 0.0f;
		float sig_r = 0.0f;
	const bool monitor_active =
			(ui_mode == UiMode::Record && record_state != RecordState::Review);
	const bool tune_active = (ui_mode == UiMode::Tune);
	const float tune_sr = static_cast<float>(hw.AudioSampleRate());
	const float tune_freq = 440.0f
		* powf(2.0f, (static_cast<float>(tune_coarse)
			+ static_cast<float>(tune_fine) / 100.0f) / 12.0f);
	last_tune_freq_hz = tune_freq;
	const float lvl_norm_raw = (static_cast<float>(tune_level_pos) + 120.0f) / 240.0f;
	const float lvl_norm = (lvl_norm_raw < 0.0f) ? 0.0f : (lvl_norm_raw > 1.0f ? 1.0f : lvl_norm_raw);
	const float lvl_shape = powf(lvl_norm, 0.193f); // more logarithmic
	float tune_level_db = -50.0f + 40.0f * lvl_shape; // -50dB @ -120deg, ~-15dB @ 0deg, -10dB @ +120deg
	float tune_amp = DbToLin(tune_level_db);
	if (tune_amp > 2.0f)
	{
		tune_amp = 2.0f;
	}
		float monitor_l = 0.0f;
		float monitor_r = 0.0f;
		if (monitor_active)
		{
			const float monitor_sel = (record_input == RecordInput::Mic) ? in[1][i] : in[0][i];
			monitor_l = monitor_sel;
			monitor_r = monitor_sel;
		}
		float tune_sig = 0.0f;
		if (tune_active)
		{
			tune_phase += tune_freq / tune_sr;
			if (tune_phase >= 1.0f)
			{
				tune_phase -= 1.0f;
			}
			tune_sig = sinf(2.0f * kPi * tune_phase) * tune_amp;
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
			if (sample_length == 1)
			{
				sig_l = static_cast<float>(sample_buffer_l[0]) * kSampleScale * playback_amp;
				sig_r = static_cast<float>(sample_buffer_r[0]) * kSampleScale * playback_amp;
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
		if (tune_active)
		{
			sig_l += tune_sig;
			sig_r += tune_sig;
		}
		out[0][i] = sig_l;
		out[1][i] = sig_r;
	}
	request_playhead_redraw = true;
}

int main(void)
{
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.seed.StartLog(false);
	LogLine("Logger started");

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
	if (button1_press && sample_loaded && ui_mode != UiMode::Load && ui_mode != UiMode::LoadTarget)
	{
		button1_press = false;
		if (UiLogEnabled())
		{
				LogLine("Button1: playback request (unpitched)");
			}
			StartPlayback(kBaseMidiNote, false);
			request_sample_pitch_estimate = true;
			request_playhead_redraw = true;
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
						(dest == LoadDestination::Bake) ? "BAKE" : "PLAY");
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
						LogLine("Load success, entering PLAY menu");
						ui_mode = UiMode::Play;
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
			else if (ui_mode == UiMode::Play)
			{
				DrawWaveform();
			}
		}
		if (!ui_blocked && request_tune_redraw && ui_mode == UiMode::Tune)
		{
			request_tune_redraw = false;
			DrawTuneScreen();
		}
		if (!ui_blocked && request_sample_pitch_estimate)
		{
			request_sample_pitch_estimate = false;
			if (sample_loaded)
			{
				float hz = 0.0f;
				if (EstimateSamplePitchHzWindow(hz))
				{
					sample_pitch_hz = hz;
					request_tune_redraw = true;
					if (UiLogEnabled())
					{
						LogLine("Pitch estimate: %.2f Hz", static_cast<double>(sample_pitch_hz));
					}
				}
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
				DrawWaveform();
			}
			else if (mode == UiMode::LoadTarget)
			{
				DrawLoadTargetMenu(load_target_selected);
				LogLine("Load target: %s",
						(load_target_selected == LoadDestination::Bake) ? "BAKE" : "PLAY");
			}
			else if (mode == UiMode::Tune)
			{
				DrawTuneScreen();
			}
			else if (mode == UiMode::AdsrSelect)
			{
				DrawAdsrSelectScreen();
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
		else if (mode == UiMode::AdsrSelect)
		{
			if (request_adsr_redraw)
			{
				request_adsr_redraw = false;
				DrawAdsrSelectScreen();
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
		if (!ui_blocked && (request_playhead_redraw || (playback_active != last_playback_active)))
		{
			request_playhead_redraw = false;
			if (mode == UiMode::Play
				|| (mode == UiMode::Record && record_state == RecordState::Review))
			{
				if (mode == UiMode::Play)
				{
					DrawWaveform();
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
		if ((ui_mode == UiMode::Record && record_state == RecordState::Review)
			|| ui_mode == UiMode::Tune
			|| (ui_mode == UiMode::Load && !delete_mode))
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
		hw.UpdateLeds();
		hw.DelayMs(10);
	}
}
