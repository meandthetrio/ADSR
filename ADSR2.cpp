#include "daisy_pod.h"
#include "daisysp.h"
#include "dev/oled_ssd130x.h"
#include "fatfs.h"
#include "util/wav_format.h"
#include "util/bsp_sd_diskio.h"
#include <math.h>
#include <cstring>
#include <cstdio>

using namespace daisy;
using namespace daisysp;

using PodDisplay = OledDisplay<SSD130xI2c128x64Driver>;

constexpr bool kLogEnabled = true;
constexpr int32_t kMenuCount = 3;
constexpr int32_t kMaxWavFiles = 32;
constexpr size_t kMaxWavNameLen = 32;
constexpr int32_t kLoadFontScale = 2;
constexpr size_t kMaxSampleSamples = 2 * 1024 * 1024;
constexpr int32_t kRecordMaxSeconds = 5;
constexpr size_t kSampleChunkFrames = 256;
constexpr int32_t kBaseMidiNote = 60;
constexpr float kSampleScale = 1.0f / 32768.0f;
constexpr int32_t kLoadProgressStep = 5;
constexpr float kLedBlinkPeriodMs = 25.0f;
constexpr float kLedBlinkDuty = 0.5f;
constexpr float kKnobArc = 4.1887902f; // 240 degrees total sweep (-120 to +120)
constexpr bool kPlaybackVerboseLog = false;
constexpr float kPi = 3.14159265f;

enum class UiMode : int32_t
{
	Main,
	Load,
	Play,
	Record,
	Tune,
};

enum class RecordState : int32_t
{
	Armed,
	Recording,
	Review,
};

DaisyPod    hw;
PodDisplay  display;
SdmmcHandler   sdcard;
FatFSInterface fsi;
Encoder      encoder_r;

volatile UiMode ui_mode = UiMode::Main;
volatile int32_t menu_index = 0;
volatile int32_t load_selected = 0;
volatile int32_t load_scroll = 0;
volatile bool request_load_scan = false;
volatile bool request_load_sample = false;
volatile int32_t request_load_index = -1;
volatile int32_t wav_file_count = 0;

bool sd_mounted = false;
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

constexpr size_t kRecordMaxFrames = static_cast<size_t>(kRecordMaxSeconds) * 48000U;
volatile RecordState record_state = RecordState::Armed;
volatile size_t record_pos = 0;
volatile bool record_waveform_pending = false;
volatile int32_t encoder_r_accum = 0;
volatile bool encoder_r_button_press = false;
volatile bool request_length_redraw = false;
volatile bool request_playhead_redraw = false;
volatile bool button1_press = false;
volatile bool request_playback_stop_log = false;
float led1_level = 0.0f;
float led1_phase_ms = 0.0f;
int32_t tune_selected = 0;
int32_t tune_coarse = 0;
int32_t tune_fine = 0;
int32_t tune_level_pos = -120;
volatile bool request_tune_redraw = false;
float tune_phase = 0.0f;
constexpr bool kUiLogsEnabled = false;

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

int16_t waveform_min[128];
int16_t waveform_max[128];

static FIL wav_file;
alignas(32) static int16_t wav_read[kSampleChunkFrames * 2];

const char* kMenuLabels[kMenuCount] = {"LOAD", "RECORD", "PLAY"};

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
		case UiMode::Play: return "PLAY";
		case UiMode::Record: return "RECORD";
		case UiMode::Tune: return "TUNE";
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

static void MountSd()
{
	if (sd_mounted)
	{
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

static void ScanWavFiles()
{
	LogLine("Scanning WAV files...");
	MountSd();
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
		if (!HasWavExtension(fno.fname))
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
	LogLine("Scan complete: %ld wav files", static_cast<long>(wav_file_count));
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
	else if (load_selected >= load_scroll + load_lines)
	{
		load_scroll = load_selected - (load_lines - 1);
	}
	if (load_scroll < 0)
	{
		load_scroll = 0;
	}
	const int32_t max_top = wav_file_count - load_lines;
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
	int32_t width = static_cast<int32_t>(display.Width());
	if (width > 128)
	{
		width = 128;
	}
	if (!sample_loaded || sample_length == 0 || width <= 0)
	{
		for (int32_t i = 0; i < width; ++i)
		{
			waveform_min[i] = 0;
			waveform_max[i] = 0;
		}
		return;
	}

	for (int32_t x = 0; x < width; ++x)
	{
		const size_t start = (sample_length * static_cast<size_t>(x)) / width;
		size_t end = (sample_length * static_cast<size_t>(x + 1)) / width;
		if (end <= start)
		{
			end = start + 1;
		}
		if (end > sample_length)
		{
			end = sample_length;
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
			if (samp < min_val)
			{
				min_val = samp;
			}
			if (samp > max_val)
			{
				max_val = samp;
			}
		}
		waveform_min[x] = min_val;
		waveform_max[x] = max_val;
	}
}

static void AdjustPlayWindow(int32_t delta_start, int32_t delta_end)
{
	if (!sample_loaded || sample_length == 0)
	{
		return;
	}
	size_t start = sample_play_start;
	size_t end = sample_play_end;
	if (end > sample_length || end == 0)
	{
		end = sample_length;
	}
	if (end <= start)
	{
		start = 0;
		end = sample_length;
	}
	const size_t min_window = (sample_length > 1) ? 2 : 1;
	const size_t base_step = (sample_length >= 512) ? (sample_length / 256) : 1;

	const auto scaled_step = [base_step](int32_t delta) -> int64_t {
		int32_t mag = (delta < 0) ? -delta : delta;
		if (mag < 1)
		{
			mag = 1;
		}
		int32_t log2_mag = 0;
		int32_t tmp = mag;
		while (tmp > 1)
		{
			tmp >>= 1;
			++log2_mag;
		}
		const int64_t scale = static_cast<int64_t>(2) * (1 + static_cast<int64_t>(log2_mag));
		return static_cast<int64_t>(base_step) * scale;
	};

	if (delta_start != 0)
	{
		const int64_t step = scaled_step(delta_start);
		int64_t next = static_cast<int64_t>(start) + static_cast<int64_t>(delta_start) * step;
		if (next < 0)
		{
			next = 0;
		}
		if (static_cast<size_t>(next) + min_window > end && end > min_window)
		{
			next = static_cast<int64_t>(end - min_window);
		}
		start = static_cast<size_t>(next);
	}

	if (delta_end != 0)
	{
		const int64_t step = scaled_step(delta_end);
		int64_t next = static_cast<int64_t>(end) + static_cast<int64_t>(delta_end) * step;
		if (next < static_cast<int64_t>(start + min_window))
		{
			next = static_cast<int64_t>(start + min_window);
		}
		if (next > static_cast<int64_t>(sample_length))
		{
			next = static_cast<int64_t>(sample_length);
		}
		end = static_cast<size_t>(next);
	}

	if (end > sample_length)
	{
		end = sample_length;
	}
	if (end <= start)
	{
		end = (sample_length > 0) ? sample_length : 0;
		start = 0;
	}
	sample_play_start = start;
	sample_play_end = end;
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
	sample_play_start = 0;
	sample_play_end = 0;

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
	sample_play_start = 0;
	sample_play_end = sample_length;
	LogLine("Load complete: %lu frames", static_cast<unsigned long>(sample_length));
	ComputeWaveform();
	return true;
}

static bool LoadSampleAtIndex(int32_t index)
{
	MountSd();
	if (!sd_mounted)
	{
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

static void DrawVerticalLabel(const char* label,
							  int x,
							  int y,
							  FontDef font,
							  bool on)
{
	for (size_t i = 0; label[i] != '\0'; ++i)
	{
		display.SetCursor(x, y + static_cast<int>(i) * font.FontHeight);
		display.WriteChar(label[i], font, on);
	}
}

static void DrawMenu(int32_t selected)
{
	constexpr int kMarginX = 2;
	constexpr int kGapX = 2;
	constexpr int kBoxW = 40;
	constexpr int kBoxY = 2;
	constexpr int kBoxH = 60;
	const FontDef font = Font_6x8;

	display.Fill(false);
	for (int32_t i = 0; i < kMenuCount; ++i)
	{
		const int x = kMarginX + static_cast<int>(i) * (kBoxW + kGapX);
		const bool is_selected = (i == selected);
		display.DrawRect(x,
						 kBoxY,
						 x + kBoxW - 1,
						 kBoxY + kBoxH - 1,
						 true,
						 is_selected);
		const size_t label_len = StrLen(kMenuLabels[i]);
		const int label_height = static_cast<int>(label_len) * font.FontHeight;
		const int text_x = x + (kBoxW - font.FontWidth) / 2;
		const int text_y = kBoxY + (kBoxH - label_height) / 2;
		DrawVerticalLabel(kMenuLabels[i], text_x, text_y, font, !is_selected);
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

	if (!sd_mounted)
	{
		DrawLoadMessage("SD NOT", "MOUNTED");
		return;
	}
	if (wav_file_count == 0)
	{
		DrawLoadMessage("NO WAV", "FILES");
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

	for (int32_t i = 0; i < load_lines; ++i)
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

static void DrawRecordArmed()
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("RECORD ENABLE", font, true);
	display.SetCursor(0, font.FontHeight + 2);
	display.WriteString("PRESS ENC TO REC", font, true);
	display.Update();
}

static void DrawRecordRecording()
{
	const FontDef font = Font_6x8;
	display.Fill(false);
	display.SetCursor(0, 0);
	display.WriteString("RECORDING", font, true);
	const int32_t percent =
		(record_pos >= kRecordMaxFrames)
			? 100
			: static_cast<int32_t>((record_pos * 100U) / kRecordMaxFrames);
	char buf[24];
	snprintf(buf, sizeof(buf), "PROGRESS %ld%%", static_cast<long>(percent));
	display.SetCursor(0, font.FontHeight + 2);
	display.WriteString(buf, font, true);
	display.SetCursor(0, font.FontHeight * 2 + 4);
	display.WriteString("MAX 5S MONO", font, true);
	display.Update();
}

static void DrawPlayMenu()
{
	const FontDef font = Font_6x8;
	display.Fill(false);

	if (!sample_loaded || sample_length == 0)
	{
		display.SetCursor(0, 0);
		display.WriteString("NO SAMPLE", font, true);
		display.Update();
		return;
	}

	display.SetCursor(0, 0);
	display.WriteString(loaded_sample_name, font, true);

	int32_t width = static_cast<int32_t>(display.Width());
	if (width > 128)
	{
		width = 128;
	}
	const int32_t wave_top = font.FontHeight + 1;
	const int32_t wave_height = static_cast<int32_t>(display.Height()) - wave_top;
	if (wave_height <= 1)
	{
		display.Update();
		return;
	}
	const int32_t center = wave_top + wave_height / 2;
	const float scale = static_cast<float>(wave_height / 2 - 1) * kSampleScale;
	int start_x = 0;
	int end_x = width - 1;
	if (sample_length > 0)
	{
		size_t window_start = sample_play_start;
		size_t window_end = sample_play_end;
		if (window_end > sample_length)
		{
			window_end = sample_length;
		}
		if (window_end <= window_start)
		{
			window_start = 0;
			window_end = sample_length;
		}
		start_x = static_cast<int>((static_cast<uint64_t>(window_start) * static_cast<uint64_t>(width)) / sample_length);
		end_x = static_cast<int>((static_cast<uint64_t>(window_end) * static_cast<uint64_t>(width)) / sample_length);
		if (start_x < 0)
		{
			start_x = 0;
		}
		if (start_x >= width)
		{
			start_x = width - 1;
		}
		if (end_x < 0)
		{
			end_x = 0;
		}
		if (end_x >= width)
		{
			end_x = width - 1;
		}
	}
	for (int32_t x = 0; x < width; ++x)
	{
		const int16_t min_val = waveform_min[x];
		const int16_t max_val = waveform_max[x];
		const bool in_window = (x >= start_x && x <= end_x);
		if (!in_window && (x & 1))
		{
			continue; // shade: skip every other column outside window
		}
		int32_t y1 = center - static_cast<int32_t>(max_val * scale);
		int32_t y2 = center - static_cast<int32_t>(min_val * scale);
		if (y1 < wave_top)
		{
			y1 = wave_top;
		}
		if (y2 > wave_top + wave_height - 1)
		{
			y2 = wave_top + wave_height - 1;
		}
		display.DrawLine(x, y1, x, y2, true);
	}
	if (sample_length > 0)
	{
		display.DrawLine(start_x, wave_top, start_x, wave_top + wave_height - 1, true);
		display.DrawLine(end_x, wave_top, end_x, wave_top + wave_height - 1, true);
	}
	if (sample_length > 0 && playback_active)
	{
		int play_x = static_cast<int>((static_cast<uint64_t>(playback_phase) * static_cast<uint64_t>(width)) / sample_length);
		if (play_x < 0)
		{
			play_x = 0;
		}
		if (play_x >= width)
		{
			play_x = width - 1;
		}
		display.DrawLine(play_x, wave_top, play_x, wave_top + wave_height - 1, true);
	}
	display.Update();
}

static void DrawRecordReview()
{
	DrawPlayMenu();
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
	const int y = 40;
	DrawKnob(22, y, radius, "LVL", tune_level_pos, -120, 120, tune_selected == 2);
	DrawKnob(64, y, radius, "CRSE", tune_coarse, -24, 24, tune_selected == 0);
	DrawKnob(106, y, radius, "FINE", tune_fine, -100, 100, tune_selected == 1);
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
	if (encoder_r_pressed)
	{
		encoder_r_button_press = true;
	}
	if (hw.button1.RisingEdge())
	{
		button1_press = true;
	}
	if (ui_mode == UiMode::Main)
	{
		if (encoder_l_inc != 0)
		{
			int32_t next = menu_index + encoder_l_inc;
			while (next < 0)
			{
				next += kMenuCount;
			}
			while (next >= kMenuCount)
			{
				next -= kMenuCount;
			}
			menu_index = next;
		}
		if (encoder_r_pressed && menu_index == 0)
		{
			ui_mode = UiMode::Load;
			load_selected = 0;
			load_scroll = 0;
			request_load_scan = true;
		}
		else if (encoder_r_pressed && menu_index == 1)
		{
			ui_mode = UiMode::Record;
			record_state = RecordState::Armed;
			record_pos = 0;
			playback_active = false;
			LogLine("Record: entered (monitor ON), max %ld s @48k mono",
					static_cast<long>(kRecordMaxSeconds));
		}
		else if (encoder_r_pressed && menu_index == 2 && sample_loaded)
		{
			ui_mode = UiMode::Play;
		}
	}
	else if (ui_mode == UiMode::Load)
	{
		if (encoder_l_inc != 0 && wav_file_count > 0)
		{
			const int32_t count = wav_file_count;
			int32_t next = load_selected + encoder_l_inc;
			if (next < 0)
			{
				next = 0;
			}
			if (next >= count)
			{
				next = count - 1;
			}
			load_selected = next;
			int32_t max_top = count - load_lines;
			if (max_top < 0)
			{
				max_top = 0;
			}
			if (load_selected < load_scroll)
			{
				load_scroll = load_selected;
			}
			else if (load_selected >= load_scroll + load_lines)
			{
				load_scroll = load_selected - (load_lines - 1);
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
			request_load_sample = true;
			request_load_index = load_selected;
		}
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Main;
		}
	}
	else if (ui_mode == UiMode::Record)
	{
		if (encoder_r_pressed)
		{
			if (record_state == RecordState::Armed)
			{
				record_pos = 0;
				sample_length = 0;
				sample_loaded = false;
				playback_active = false;
				sample_channels = 1;
				sample_rate = 48000;
				sample_play_start = 0;
				sample_play_end = 0;
				CopyString(loaded_sample_name, "RECORD", kMaxWavNameLen);
				record_state = RecordState::Recording;
				LogLine("Record: start (monitor ON)");
			}
			else if (record_state == RecordState::Recording)
			{
				sample_length = record_pos;
				sample_channels = 1;
				sample_rate = 48000;
				sample_loaded = (sample_length > 0);
				playback_active = false;
				sample_play_start = 0;
				sample_play_end = sample_length;
				record_state = RecordState::Review;
				record_waveform_pending = true;
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
			record_state = RecordState::Armed;
			playback_active = false;
		}
		if (record_state == RecordState::Review && sample_loaded)
		{
			if (encoder_l_inc != 0 || encoder_r_inc != 0)
			{
				AdjustPlayWindow(encoder_l_inc, encoder_r_inc);
				request_length_redraw = true;
			}
		}
	}
	else if (ui_mode == UiMode::Tune)
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
		if (encoder_r_inc != 0)
		{
			if (tune_selected == 0)
			{
				tune_coarse += encoder_r_inc;
				if (tune_coarse < -24)
				{
					tune_coarse = -24;
				}
				if (tune_coarse > 24)
				{
					tune_coarse = 24;
				}
			}
			else if (tune_selected == 1)
			{
				tune_fine += encoder_r_inc;
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
				tune_level_pos += encoder_r_inc;
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
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Record;
			record_state = RecordState::Review;
			request_length_redraw = true;
		}
	}
	else
	{
		if (encoder_l_pressed)
		{
			ui_mode = UiMode::Main;
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
			monitor_l = in[0][i];
			monitor_r = in[1][i];
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
			float in_l = in[0][i];
				int32_t s = static_cast<int32_t>(in_l * 32767.0f);
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
				sample_play_start = 0;
				sample_play_end = sample_length;
				record_state = RecordState::Review;
				record_waveform_pending = true;
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
	int32_t last_scroll = -1;
	int32_t last_selected = -1;
	int32_t last_file_count = -1;
	bool last_sd_mounted = false;
	RecordState last_record_state = RecordState::Armed;
	bool last_playback_active = false;
	while(1)
	{
		hw.midi.Listen();
		while(hw.midi.HasEvents())
		{
			HandleMidiMessage(hw.midi.PopEvent());
		}
	if (button1_press && sample_loaded)
	{
		button1_press = false;
		if (UiLogEnabled())
		{
			LogLine("Button1: playback request (unpitched)");
		}
		StartPlayback(kBaseMidiNote, false);
		request_playhead_redraw = true;
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

		if (request_load_scan)
		{
			request_load_scan = false;
			LogLine("Load menu: scan requested");
			ScanWavFiles();
		}
	if (request_load_sample)
	{
		request_load_sample = false;
		const int32_t index = request_load_index;
		request_load_index = -1;
		LogLine("Load menu: sample request index=%ld", static_cast<long>(index));
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
				LogLine("Load success, entering PLAY menu");
				ui_mode = UiMode::Play;
			}
			else
			{
				LogLine("Load failed");
			}
		}

		if (record_waveform_pending)
		{
			record_waveform_pending = false;
			if (sample_loaded && sample_length > 0)
			{
				ComputeWaveform();
				if (ui_mode == UiMode::Record && record_state == RecordState::Review)
				{
					DrawRecordReview();
					last_record_state = record_state;
				}
			}
		}
		if (request_length_redraw)
		{
			request_length_redraw = false;
			if (ui_mode == UiMode::Record && record_state == RecordState::Review)
			{
				DrawRecordReview();
			}
			else if (ui_mode == UiMode::Play)
			{
				DrawPlayMenu();
			}
		}
		if (request_tune_redraw && ui_mode == UiMode::Tune)
		{
			request_tune_redraw = false;
			DrawTuneScreen();
		}

		const UiMode mode = ui_mode;
		if (mode != last_mode)
		{
			LogLine("UI mode: %s", UiModeName(mode));
			if (mode == UiMode::Main)
			{
				DrawMenu(menu_index);
			}
			else if (mode == UiMode::Load)
			{
				DrawLoadMenu(load_scroll, load_selected);
				LogLine("Load menu: selected=%ld name=%s",
						static_cast<long>(load_selected),
						(load_selected >= 0 && load_selected < wav_file_count)
							? wav_files[load_selected]
							: "UNKNOWN");
			}
			else if (mode == UiMode::Play)
			{
				DrawPlayMenu();
			}
			else if (mode == UiMode::Tune)
			{
				DrawTuneScreen();
			}
			else
			{
				if (record_state == RecordState::Armed)
				{
					DrawRecordArmed();
				}
				else if (record_state == RecordState::Recording)
				{
					DrawRecordRecording();
				}
				else
				{
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
		else if (mode == UiMode::Load)
		{
			const int32_t current_scroll = load_scroll;
			const int32_t current_count = wav_file_count;
			const int32_t current_selected = load_selected;
			if (current_scroll != last_scroll
				|| current_selected != last_selected
				|| current_count != last_file_count
				|| sd_mounted != last_sd_mounted)
			{
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
		else if (mode == UiMode::Record)
		{
			const RecordState current_state = record_state;
			if (current_state != last_record_state)
			{
				if (current_state == RecordState::Armed)
				{
					DrawRecordArmed();
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
		if (request_playhead_redraw || (playback_active != last_playback_active))
		{
			request_playhead_redraw = false;
			if (mode == UiMode::Play
				|| (mode == UiMode::Record && record_state == RecordState::Review))
			{
				if (mode == UiMode::Play)
				{
					DrawPlayMenu();
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
			|| ui_mode == UiMode::Tune)
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
