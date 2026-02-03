#include "StorageService.h"

#include "daisy_pod.h"
#include "sys/fatfs.h"
#include "sys/system.h"
#include "util/bsp_sd_diskio.h"
#include "util/scopedirqblocker.h"
#include "util/wav_format.h"
#include <cstring>
#include <cstdio>

daisy::SdmmcHandler sdcard;
daisy::FatFSInterface fsi;

static DIR g_scan_dir;
static FILINFO g_scan_fno;
static bool g_scan_dir_open = false;

static FIL g_preview_file;
static bool g_preview_open = false;
static uint32_t g_preview_data_offset = 0;
static uint32_t g_preview_sample_rate = 0;
static uint16_t g_preview_channels = 0;
static uint16_t g_preview_cookie = 0;
static uint32_t g_preview_deadline_ms = 0;

static volatile uint32_t g_storage_slice_max_us = 0;
static volatile uint32_t g_storage_readchunk_max_us = 0;
static volatile uint32_t g_storage_readchunk_bytes = 0;
static volatile uint32_t g_storage_readchunk_calls = 0;

static constexpr uint32_t kSdRetryBaseBackoffMs = 200;
static constexpr uint32_t kSdRetryMaxBackoffMs = 2000;
static constexpr uint32_t kSdOpTimeoutMountMs = 1500;
static constexpr uint32_t kSdOpTimeoutLoadMs = 8000;
static constexpr uint32_t kSdOpTimeoutSaveMs = 8000;
static constexpr uint32_t kSdOpTimeoutPreviewMs = 1500;
static constexpr uint32_t kSdOpTimeoutScanMs = 3000;
static constexpr uint32_t kSdOpTimeoutDeleteMs = 2000;

alignas(32) static uint8_t wav_riff_hdr[12];
alignas(32) static uint8_t wav_chunk_hdr[8];
alignas(32) static uint8_t wav_fmt_buf[32];
alignas(32) static int16_t g_preview_read_buf[2048 * 2];
alignas(32) static int16_t g_save_write_buf[8192 * 2];
alignas(32) static int16_t g_load_read_buf[256 * 2];

struct WavInfo
{
	uint16_t num_channels = 0;
	uint32_t sample_rate = 0;
	uint16_t bits_per_sample = 0;
	uint32_t data_offset = 0;
	uint32_t data_size = 0;
};

static bool ParseWavHeader(FIL* file, WavInfo& info)
{
	if (file == nullptr)
	{
		return false;
	}

	FRESULT fres;
	UINT bytes_read = 0;
	fres = f_lseek(file, 0);
	if (fres != FR_OK)
	{
		return false;
	}

	fres = f_read(file, wav_riff_hdr, sizeof(wav_riff_hdr), &bytes_read);
	if (fres != FR_OK || bytes_read != sizeof(wav_riff_hdr))
	{
		return false;
	}

	if (std::memcmp(wav_riff_hdr, "RIFF", 4) != 0
		|| std::memcmp(wav_riff_hdr + 8, "WAVE", 4) != 0)
	{
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

			if (audio_format != daisy::WAVE_FORMAT_PCM)
			{
				return false;
			}

			fmt_found = true;

			if (chunk_size > to_read)
			{
				const FSIZE_t skip = (FSIZE_t)(chunk_size - to_read);
				fres = f_lseek(file, f_tell(file) + skip);
				if (fres != FR_OK)
				{
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
				return false;
			}
		}
	}

	if (!fmt_found || !data_found)
	{
		return false;
	}

	return true;
}

static bool HasWavExtension(const char* name)
{
	if (!name)
	{
		return false;
	}
	size_t len = 0;
	while (name[len] != '\0')
	{
		++len;
	}
	if (len < 4)
	{
		return false;
	}
	const char* ext = name + (len - 4);
	return (ext[0] == '.')
		&& (ext[1] == 'w' || ext[1] == 'W')
		&& (ext[2] == 'a' || ext[2] == 'A')
		&& (ext[3] == 'v' || ext[3] == 'V');
}

static void CopyString(char* dst, size_t dst_len, const char* src)
{
	if (dst_len == 0)
	{
		return;
	}
	size_t i = 0;
	for (; i + 1 < dst_len && src && src[i] != '\0'; ++i)
	{
		dst[i] = src[i];
	}
	dst[i] = '\0';
}

static bool BuildNextSavePath(char* out_name,
							  size_t out_name_len,
							  char* out_path,
							  size_t out_path_len,
							  const char* base_dir)
{
	if (!out_name || !out_path || !base_dir || out_name_len == 0 || out_path_len == 0)
	{
		return false;
	}
	static uint32_t save_counter = 0;
	if (save_counter == 0)
	{
		save_counter = static_cast<uint32_t>(daisy::System::GetNow());
	}
	const uint32_t now = static_cast<uint32_t>(daisy::System::GetNow());
	const uint32_t counter = save_counter++;
	const int name_len = snprintf(out_name,
								  out_name_len,
								  "Rec_%08lX_%04lX.wav",
								  static_cast<unsigned long>(now),
								  static_cast<unsigned long>(counter & 0xFFFFu));
	if (name_len <= 0 || name_len >= static_cast<int>(out_name_len))
	{
		return false;
	}
	const int path_len = snprintf(out_path, out_path_len, "%s%s",
								  base_dir, out_name);
	if (path_len <= 0 || path_len >= static_cast<int>(out_path_len))
	{
		return false;
	}
	return true;
}

static bool EventQueueHasRoom(uint8_t wr, uint8_t rd, size_t size)
{
	const uint8_t next = static_cast<uint8_t>((wr + 1) % size);
	return next != rd;
}

static bool PushEvent(StorageService::Event* queue,
					  uint8_t& wr,
					  uint8_t rd,
					  size_t size,
					  const StorageService::Event& ev)
{
	const uint8_t next = static_cast<uint8_t>((wr + 1) % size);
	if (next == rd)
	{
		return false;
	}
	queue[wr] = ev;
	wr = next;
	return true;
}

static bool IsRetryable(StorageService::SdErrorCode code)
{
	switch (code)
	{
		case StorageService::SdErrorCode::MountFailed:
		case StorageService::SdErrorCode::ReadFailed:
		case StorageService::SdErrorCode::WriteFailed:
		case StorageService::SdErrorCode::Timeout:
			return true;
		default:
			return false;
	}
}

static uint32_t BackoffMs(uint8_t attempt)
{
	uint32_t backoff = kSdRetryBaseBackoffMs << attempt;
	if (backoff > kSdRetryMaxBackoffMs)
	{
		backoff = kSdRetryMaxBackoffMs;
	}
	return backoff;
}

static uint32_t DefaultDeadlineMs(StorageService::OpKind kind)
{
	switch (kind)
	{
		case StorageService::OpKind::Mount: return kSdOpTimeoutMountMs;
		case StorageService::OpKind::ScanDir: return kSdOpTimeoutScanMs;
		case StorageService::OpKind::PreviewOpen: return kSdOpTimeoutPreviewMs;
		case StorageService::OpKind::SaveStart: return kSdOpTimeoutSaveMs;
		case StorageService::OpKind::LoadStart: return kSdOpTimeoutLoadMs;
		case StorageService::OpKind::DeleteFile: return kSdOpTimeoutDeleteMs;
		default: return 1000;
	}
}

void StorageService::Init()
{
	op_wr_ = 0;
	op_rd_ = 0;
	ev_wr_ = 0;
	ev_rd_ = 0;
	mount_state_ = MountState::Uninit;
	sd_hw_inited_ = false;
	scan_active_ = false;
	scan_wav_only_ = false;
	scan_done_pending_ = false;
	scan_cookie_ = 0;
	scan_max_entries_ = 0;
	scan_count_ = 0;
	scan_path_[0] = '\0';
	g_scan_dir_open = false;
	preview_buffer_ = nullptr;
	preview_frames_ = 0;
	preview_write_index_ = nullptr;
	preview_read_index_ = nullptr;
	preview_preload_buf_ = nullptr;
	preview_preload_frames_ = 0;
	preview_preload_filled_ = 0;
	preview_preload_target_frames_ = 0;
	preview_preload_active_ = false;
	preview_pp_a_ = nullptr;
	preview_pp_b_ = nullptr;
	preview_pp_frames_ = 0;
	preview_pp_ready_a_ = nullptr;
	preview_pp_ready_b_ = nullptr;
	preview_pp_active_ = nullptr;
	save_ = {};
	load_ = {};
	g_preview_open = false;
	g_preview_data_offset = 0;
	g_preview_sample_rate = 0;
	g_preview_channels = 0;
	g_preview_cookie = 0;
	fsi.Init(daisy::FatFSInterface::Config::MEDIA_SD);
	sd_status_ = {};
	next_op_id_ = 1;
}

void StorageService::SetPreviewStreamConfig(const PreviewStreamConfig& cfg)
{
	preview_buffer_ = cfg.buffer;
	preview_frames_ = cfg.frames;
	preview_write_index_ = cfg.write_index;
	preview_read_index_ = cfg.read_index;
	preview_preload_buf_ = cfg.preload_buf;
	preview_preload_frames_ = cfg.preload_frames;
	preview_pp_a_ = cfg.pp_buf_a;
	preview_pp_b_ = cfg.pp_buf_b;
	preview_pp_frames_ = cfg.pp_frames;
	preview_pp_ready_a_ = cfg.pp_ready_a;
	preview_pp_ready_b_ = cfg.pp_ready_b;
	preview_pp_active_ = cfg.pp_active;
}

void StorageService::UnmountSd()
{
	(void)f_mount(0, fsi.GetSDPath(), 0);
}

void StorageService::ClearSdError()
{
	sd_status_.last_error = {};
	sd_status_.last_error_time_ms = daisy::System::GetNow();
}

void StorageService::CancelAllOps(SdErrorCode reason)
{
	op_rd_ = op_wr_;
	scan_active_ = false;
	scan_done_pending_ = false;
	if (g_scan_dir_open)
	{
		f_closedir(&g_scan_dir);
		g_scan_dir_open = false;
	}
	if (g_preview_open)
	{
		f_close(&g_preview_file);
		g_preview_open = false;
	}
	save_.active = false;
	load_.active = false;
	preview_preload_active_ = false;
	preview_preload_filled_ = 0;
	preview_preload_target_frames_ = 0;
	sd_status_.last_error.code = reason;
	sd_status_.last_error_time_ms = daisy::System::GetNow();
	++sd_counters_.cancels;
}

const char* StorageService::GetSdPath() const
{
	return fsi.GetSDPath();
}

bool StorageService::Enqueue(const Op& op)
{
	const uint8_t next = static_cast<uint8_t>((op_wr_ + 1) % kOpQueueSize);
	if (next == op_rd_)
	{
		return false;
	}
	Op queued = op;
	if (queued.op_id == 0)
	{
		queued.op_id = next_op_id_++;
	}
	if (queued.start_ms == 0)
	{
		queued.start_ms = daisy::System::GetNow();
	}
	if (queued.deadline_ms == 0)
	{
		queued.deadline_ms = queued.start_ms + DefaultDeadlineMs(queued.kind);
	}
	op_queue_[op_wr_] = queued;
	op_wr_ = next;
	return true;
}

bool StorageService::DequeueEvent(Event& out_event)
{
	if (ev_rd_ == ev_wr_)
	{
		return false;
	}
	out_event = event_queue_[ev_rd_];
	ev_rd_ = static_cast<uint8_t>((ev_rd_ + 1) % kEventQueueSize);
	return true;
}

void StorageService::RunSlice(uint32_t budget_us)
{
	if (budget_us == 0)
	{
		budget_us = kMinStorageBudgetUs;
	}
	const uint32_t slice_start_ms = daisy::System::GetNow();
	static constexpr uint32_t kMaxStepsPerSlice = 8;
	uint32_t steps = 0;
	auto step_guard = [&]()
	{
		if (++steps > kMaxStepsPerSlice)
		{
			++sd_counters_.op_timeouts;
		}
	};
	const bool present = BSP_SD_IsDetected();
	sd_status_.present = present;
	if (!present)
	{
		mount_state_ = MountState::NoCard;
		sd_status_.mounted = false;
		CancelAllOps(SdErrorCode::NoCard);
		return;
	}
	if (op_rd_ != op_wr_)
	{
		step_guard();
		const Op op = op_queue_[op_rd_];
		if (op.next_attempt_ms != 0 && slice_start_ms < op.next_attempt_ms)
		{
			return;
		}
		if (op.deadline_ms != 0 && slice_start_ms > op.deadline_ms)
		{
			sd_status_.last_error = {SdErrorCode::Timeout, -1, op.op_id, op.attempt};
			sd_status_.last_error_time_ms = daisy::System::GetNow();
			++sd_counters_.op_timeouts;
			Event ev = {};
			switch (op.kind)
			{
				case OpKind::Mount: ev.kind = EventKind::MountFail; break;
				case OpKind::ScanDir: ev.kind = EventKind::ScanDone; ev.cookie = op.cookie; break;
				case OpKind::PreviewOpen: ev.kind = EventKind::PreviewOpenFail; ev.cookie = op.cookie; break;
				case OpKind::SaveStart: ev.kind = EventKind::SaveError; break;
				case OpKind::LoadStart: ev.kind = EventKind::LoadError; ev.cookie = op.cookie; break;
				case OpKind::DeleteFile: ev.kind = EventKind::DeleteFail; ev.cookie = op.cookie; break;
				default: break;
			}
			if (ev.kind != EventKind::None)
			{
				PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
			}
			op_rd_ = static_cast<uint8_t>((op_rd_ + 1) % kOpQueueSize);
			return;
		}
		op_rd_ = static_cast<uint8_t>((op_rd_ + 1) % kOpQueueSize);
		auto retry_op = [&](SdErrorCode code, int32_t fs_result) -> bool
		{
			if (!IsRetryable(code) || op.attempt >= kSdRetryMaxAttempts)
			{
				sd_status_.last_error = {code, fs_result, op.op_id, op.attempt};
				sd_status_.last_error_time_ms = daisy::System::GetNow();
				return false;
			}
			Op retry = op;
			retry.attempt = static_cast<uint8_t>(op.attempt + 1);
			retry.next_attempt_ms = slice_start_ms + BackoffMs(retry.attempt - 1);
			if (!Enqueue(retry))
			{
				sd_status_.last_error = {code, fs_result, op.op_id, op.attempt};
				sd_status_.last_error_time_ms = daisy::System::GetNow();
				return false;
			}
			sd_status_.last_error = {code, fs_result, op.op_id, retry.attempt};
			sd_status_.last_error_time_ms = daisy::System::GetNow();
			return true;
		};

		switch (op.kind)
		{
			case OpKind::Mount:
			{
				++sd_counters_.mount_attempts;
				mount_state_ = MountState::Mounting;
				if (!sd_hw_inited_)
				{
					daisy::SdmmcHandler::Config sd_cfg;
					sd_cfg.Defaults();
					sdcard.Init(sd_cfg);
					(void)BSP_SD_Init();
					sd_hw_inited_ = true;
				}
				const FRESULT res = f_mount(&fsi.GetSDFileSystem(), fsi.GetSDPath(), 1);
				if (res == FR_OK)
				{
					mount_state_ = MountState::Mounted;
					sd_status_.mounted = true;
					Event ev = {};
					ev.kind = EventKind::MountOk;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				}
				else
				{
					mount_state_ = MountState::Error;
					sd_status_.mounted = false;
					sd_status_.last_error = {SdErrorCode::MountFailed, res, op.op_id, op.attempt};
					sd_status_.last_error_time_ms = daisy::System::GetNow();
					++sd_counters_.mount_failures;
					CancelAllOps(SdErrorCode::MountFailed);
					Event ev = {};
					ev.kind = EventKind::MountFail;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				}
				break;
			}
			case OpKind::DeleteFile:
			{
				Event ev = {};
				ev.cookie = op.cookie;
				if (op.path[0] == '\0')
				{
					ev.kind = EventKind::DeleteFail;
				}
				else
				{
					const FRESULT res = f_unlink(op.path);
					ev.kind = (res == FR_OK) ? EventKind::DeleteOk : EventKind::DeleteFail;
					if (res != FR_OK)
					{
						sd_status_.last_error = {SdErrorCode::WriteFailed, res, op.op_id, op.attempt};
						sd_status_.last_error_time_ms = daisy::System::GetNow();
						++sd_counters_.write_failures;
					}
				}
				PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				break;
			}
			case OpKind::ScanDir:
			{
				if (g_scan_dir_open)
				{
					f_closedir(&g_scan_dir);
					g_scan_dir_open = false;
				}
				scan_active_ = false;
				scan_done_pending_ = false;
				scan_cookie_ = op.cookie;
				scan_max_entries_ = op.max_entries;
				scan_wav_only_ = op.wav_only;
				scan_count_ = 0;
				CopyString(scan_path_, kPathMaxLen, op.path);

				if (scan_path_[0] == '\0')
				{
					Event ev = {};
					ev.kind = EventKind::ScanDone;
					ev.cookie = scan_cookie_;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				const FRESULT res = f_opendir(&g_scan_dir, scan_path_);
				if (res != FR_OK)
				{
					if (retry_op(SdErrorCode::OpenFailed, res))
					{
						break;
					}
					Event ev = {};
					ev.kind = EventKind::ScanDone;
					ev.cookie = scan_cookie_;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				g_scan_dir_open = true;
				scan_active_ = true;
				break;
			}
			case OpKind::PreviewOpen:
			{
				if (g_preview_open)
				{
					f_close(&g_preview_file);
					g_preview_open = false;
				}
				g_preview_cookie = op.cookie;
				g_preview_deadline_ms = op.deadline_ms;
				const FRESULT open_res = f_open(&g_preview_file, op.path, FA_READ);
				if (open_res != FR_OK)
				{
					if (retry_op(SdErrorCode::OpenFailed, open_res))
					{
						break;
					}
					Event ev = {};
					ev.kind = EventKind::PreviewOpenFail;
					ev.cookie = g_preview_cookie;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				WavInfo wav;
				if (!ParseWavHeader(&g_preview_file, wav)
					|| wav.bits_per_sample != 16
					|| wav.num_channels < 1
					|| wav.num_channels > 2)
				{
					f_close(&g_preview_file);
					g_preview_open = false;
					if (retry_op(SdErrorCode::ReadFailed, -1))
					{
						break;
					}
					++sd_counters_.read_failures;
					Event ev = {};
					ev.kind = EventKind::PreviewOpenFail;
					ev.cookie = g_preview_cookie;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}

				g_preview_data_offset = wav.data_offset;
				g_preview_sample_rate = wav.sample_rate;
				g_preview_channels = wav.num_channels;
				if (f_lseek(&g_preview_file, g_preview_data_offset) != FR_OK)
				{
					f_close(&g_preview_file);
					g_preview_open = false;
					Event ev = {};
					ev.kind = EventKind::PreviewOpenFail;
					ev.cookie = g_preview_cookie;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				g_preview_open = true;
				if (preview_preload_buf_ != nullptr && preview_preload_frames_ > 0)
				{
					preview_preload_active_ = true;
					preview_preload_filled_ = 0;
					if (g_preview_sample_rate > 0)
					{
						const uint64_t scaled = (uint64_t)preview_preload_frames_
							* (uint64_t)g_preview_sample_rate;
						const size_t target = (size_t)(scaled / 48000u);
						preview_preload_target_frames_ = (target < preview_preload_frames_)
							? target
							: preview_preload_frames_;
					}
					else
					{
						preview_preload_target_frames_ = preview_preload_frames_;
					}
				}
				else
				{
					Event ev = {};
					ev.kind = EventKind::PreviewOpenOk;
					ev.cookie = g_preview_cookie;
					ev.sample_rate = g_preview_sample_rate;
					ev.channels = g_preview_channels;
					ev.bits_per_sample = 16;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				}
				break;
			}
			case OpKind::PreviewClose:
			{
				if (g_preview_open)
				{
					f_close(&g_preview_file);
				}
				g_preview_open = false;
				preview_preload_active_ = false;
				preview_preload_filled_ = 0;
				preview_preload_target_frames_ = 0;
				break;
			}
			case OpKind::SaveStart:
			{
				if (save_.active)
				{
					break;
				}
				save_ = {};
				save_.src_l = op.src_l;
				save_.src_r = op.src_r;
				save_.frames_total = op.frames;
				save_.channels = (op.channels == 0) ? 1 : op.channels;
				save_.sample_rate = (op.sample_rate == 0) ? 48000 : op.sample_rate;
				save_.data_bytes = static_cast<uint32_t>(
					save_.frames_total * save_.channels * sizeof(int16_t));
				save_.start_ms = daisy::System::GetNow();
				save_.deadline_ms = op.deadline_ms;
				char save_name[32] = {};
				if (!BuildNextSavePath(save_name,
									   sizeof(save_name),
									   save_.path,
									   sizeof(save_.path),
									   fsi.GetSDPath()))
				{
					sd_status_.last_error = {SdErrorCode::OpenFailed, -1, op.op_id, op.attempt};
					sd_status_.last_error_time_ms = daisy::System::GetNow();
					Event ev = {};
					ev.kind = EventKind::SaveError;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}

				const FRESULT open_res = f_open(&save_.file, save_.path, FA_WRITE | FA_CREATE_NEW);
				if (open_res != FR_OK)
				{
					if (retry_op(SdErrorCode::OpenFailed, open_res))
					{
						break;
					}
					sd_status_.last_error = {SdErrorCode::OpenFailed, open_res, op.op_id, op.attempt};
					sd_status_.last_error_time_ms = daisy::System::GetNow();
					Event ev = {};
					ev.kind = EventKind::SaveError;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}

				daisy::WAV_FormatTypeDef header = {};
				header.ChunkId = daisy::kWavFileChunkId;
				header.FileSize = 36;
				header.FileFormat = daisy::kWavFileWaveId;
				header.SubChunk1ID = daisy::kWavFileSubChunk1Id;
				header.SubChunk1Size = 16;
				header.AudioFormat = daisy::WAVE_FORMAT_PCM;
				header.NbrChannels = save_.channels;
				header.SampleRate = save_.sample_rate;
				header.BlockAlign = static_cast<uint16_t>(save_.channels * sizeof(int16_t));
				header.ByteRate = save_.sample_rate * header.BlockAlign;
				header.BitPerSample = 16;
				header.SubChunk2ID = daisy::kWavFileSubChunk2Id;
				header.SubCHunk2Size = 0;

				UINT written = 0;
				const FRESULT wr = f_write(&save_.file, &header, sizeof(header), &written);
				if (wr != FR_OK || written != sizeof(header))
				{
					f_close(&save_.file);
					if (retry_op(SdErrorCode::WriteFailed, wr))
					{
						break;
					}
					sd_status_.last_error = {SdErrorCode::WriteFailed, wr, op.op_id, op.attempt};
					sd_status_.last_error_time_ms = daisy::System::GetNow();
					++sd_counters_.write_failures;
					Event ev = {};
					ev.kind = EventKind::SaveError;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}

				save_.header_written = true;
				save_.active = true;
				{
					Event ev = {};
					ev.kind = EventKind::SaveProgress;
					CopyString(ev.name, kNameMaxLen, save_name);
					ev.value = 0;
					ev.value2 = static_cast<uint32_t>(save_.frames_total);
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				}
				break;
			}
			case OpKind::LoadStart:
			{
				if (load_.active)
				{
					break;
				}
				load_ = {};
				load_.dst_l = op.dst_l;
				load_.dst_r = op.dst_r;
				load_.max_frames = op.max_frames;
				load_.cookie = op.cookie;
				load_.start_ms = daisy::System::GetNow();
				load_.deadline_ms = op.deadline_ms;
				if (!op.path[0] || load_.dst_l == nullptr || load_.dst_r == nullptr)
				{
					Event ev = {};
					ev.kind = EventKind::LoadError;
					ev.cookie = load_.cookie;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				FRESULT res = f_open(&load_.file, op.path, FA_READ);
				if (res != FR_OK)
				{
					if (retry_op(SdErrorCode::OpenFailed, res))
					{
						break;
					}
					sd_status_.last_error = {SdErrorCode::OpenFailed, res, op.op_id, op.attempt};
					sd_status_.last_error_time_ms = daisy::System::GetNow();
					Event ev = {};
					ev.kind = EventKind::LoadError;
					ev.cookie = load_.cookie;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				WavInfo wav;
				if (!ParseWavHeader(&load_.file, wav)
					|| wav.bits_per_sample != 16
					|| wav.num_channels < 1
					|| wav.num_channels > 2)
				{
					f_close(&load_.file);
					if (retry_op(SdErrorCode::ReadFailed, -1))
					{
						break;
					}
					sd_status_.last_error = {SdErrorCode::ReadFailed, -1, op.op_id, op.attempt};
					sd_status_.last_error_time_ms = daisy::System::GetNow();
					++sd_counters_.read_failures;
					Event ev = {};
					ev.kind = EventKind::LoadError;
					ev.cookie = load_.cookie;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				load_.channels = wav.num_channels;
				load_.sample_rate = wav.sample_rate;
				const size_t bytes_per_sample = wav.bits_per_sample / 8;
				const size_t frame_bytes = bytes_per_sample * wav.num_channels;
				size_t total_frames = (frame_bytes == 0) ? 0 : (wav.data_size / frame_bytes);
				if (total_frames == 0)
				{
					f_close(&load_.file);
					Event ev = {};
					ev.kind = EventKind::LoadError;
					ev.cookie = load_.cookie;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				if (total_frames > load_.max_frames)
				{
					total_frames = load_.max_frames;
				}
				load_.frames_total = total_frames;
				load_.data_offset = wav.data_offset;
				if (f_lseek(&load_.file, load_.data_offset) != FR_OK)
				{
					f_close(&load_.file);
					Event ev = {};
					ev.kind = EventKind::LoadError;
					ev.cookie = load_.cookie;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				load_.active = true;
				{
					Event ev = {};
					ev.kind = EventKind::LoadProgress;
					ev.cookie = load_.cookie;
					ev.value = 0;
					ev.value2 = static_cast<uint32_t>(load_.frames_total);
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				}
				break;
			}
			default:
				break;
		}
	}

	const bool have_pp = (preview_pp_a_ != nullptr && preview_pp_b_ != nullptr
		&& preview_pp_frames_ > 0
		&& preview_pp_ready_a_ != nullptr
		&& preview_pp_ready_b_ != nullptr
		&& preview_pp_active_ != nullptr);
	const bool have_rb = (preview_buffer_ != nullptr && preview_frames_ > 0
		&& preview_write_index_ != nullptr && preview_read_index_ != nullptr);

	if (!g_preview_open || (!have_pp && !have_rb))
	{
		// Skip preview fill.
	}
	else
	{
		step_guard();
		const uint32_t start_ms = daisy::System::GetNow();
		if (g_preview_deadline_ms != 0 && start_ms > g_preview_deadline_ms)
		{
			f_close(&g_preview_file);
			g_preview_open = false;
			preview_preload_active_ = false;
			sd_status_.last_error = {SdErrorCode::Timeout, -1, 0, 0};
			sd_status_.last_error_time_ms = daisy::System::GetNow();
			++sd_counters_.op_timeouts;
			Event ev = {};
			ev.kind = EventKind::PreviewReadError;
			ev.cookie = g_preview_cookie;
			PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
			return;
		}
		const uint32_t budget_ms = (budget_us + 999) / 1000;
		const size_t max_frames_per_slice = 2048;

		while (g_preview_open)
		{
			if (preview_preload_active_)
			{
				const size_t target_frames = (preview_preload_target_frames_ > 0)
					? preview_preload_target_frames_
					: preview_preload_frames_;
				if (preview_preload_filled_ >= target_frames)
				{
					f_close(&g_preview_file);
					g_preview_open = false;
					preview_preload_active_ = false;
					Event ev = {};
					ev.kind = EventKind::PreviewOpenOk;
					ev.cookie = g_preview_cookie;
					ev.sample_rate = g_preview_sample_rate;
					ev.channels = g_preview_channels;
					ev.bits_per_sample = 16;
					ev.size = static_cast<uint32_t>(preview_preload_filled_);
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}

				size_t frames_to_read = target_frames - preview_preload_filled_;
				if (frames_to_read > max_frames_per_slice)
				{
					frames_to_read = max_frames_per_slice;
				}
				const size_t bytes_to_read = frames_to_read * g_preview_channels * sizeof(int16_t);
				const uint32_t read_start_ms = daisy::System::GetNow();
				UINT bytes_read = 0;
				FRESULT res = f_read(&g_preview_file, g_preview_read_buf, bytes_to_read, &bytes_read);
				const uint32_t read_elapsed_us = (daisy::System::GetNow() - read_start_ms) * 1000u;
				if (read_elapsed_us > g_storage_readchunk_max_us)
				{
					g_storage_readchunk_max_us = read_elapsed_us;
				}
				g_storage_readchunk_bytes += bytes_read;
				g_storage_readchunk_calls += 1;
				if (res != FR_OK)
				{
					f_close(&g_preview_file);
					g_preview_open = false;
					preview_preload_active_ = false;
					sd_status_.last_error = {SdErrorCode::ReadFailed, res, 0, 0};
					sd_status_.last_error_time_ms = daisy::System::GetNow();
					++sd_counters_.read_failures;
					Event ev = {};
					ev.kind = EventKind::PreviewReadError;
					ev.cookie = g_preview_cookie;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				if (bytes_read == 0)
				{
					if (f_lseek(&g_preview_file, g_preview_data_offset) != FR_OK)
					{
						f_close(&g_preview_file);
						g_preview_open = false;
						preview_preload_active_ = false;
						sd_status_.last_error = {SdErrorCode::ReadFailed, -1, 0, 0};
						sd_status_.last_error_time_ms = daisy::System::GetNow();
						++sd_counters_.read_failures;
						Event ev = {};
						ev.kind = EventKind::PreviewReadError;
						ev.cookie = g_preview_cookie;
						PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
						break;
					}
					continue;
				}
				const size_t frames_read = bytes_read / (g_preview_channels * sizeof(int16_t));
				for (size_t i = 0; i < frames_read; ++i)
				{
					int32_t mono = 0;
					if (g_preview_channels == 1)
					{
						mono = g_preview_read_buf[i];
					}
					else
					{
						const int16_t l = g_preview_read_buf[i * 2];
						const int16_t r = g_preview_read_buf[i * 2 + 1];
						mono = (static_cast<int32_t>(l) + static_cast<int32_t>(r)) / 2;
					}
					preview_preload_buf_[preview_preload_filled_ + i] = static_cast<int16_t>(mono);
				}
				preview_preload_filled_ += frames_read;
				if (budget_ms > 0 && (daisy::System::GetNow() - start_ms) >= budget_ms)
				{
					break;
				}
				continue;
			}

			size_t frames_to_read = 0;
			int16_t* dst_buf = nullptr;
			size_t dst_write_idx = 0;

			uint8_t pp_fill = 0;
			if (have_pp)
			{
				uint8_t ready_a = 0;
				uint8_t ready_b = 0;
				{
					daisy::ScopedIrqBlocker irq;
					ready_a = *preview_pp_ready_a_;
					ready_b = *preview_pp_ready_b_;
				}
				if (ready_a == 0)
				{
					pp_fill = 0;
					dst_buf = preview_pp_a_;
				}
				else if (ready_b == 0)
				{
					pp_fill = 1;
					dst_buf = preview_pp_b_;
				}
				else
				{
					break;
				}
				frames_to_read = preview_pp_frames_;
			}
			else
			{
				size_t read_idx = 0;
				size_t write_idx = 0;
				{
					daisy::ScopedIrqBlocker irq;
					read_idx = *preview_read_index_;
					write_idx = *preview_write_index_;
				}
				size_t used = (write_idx >= read_idx)
					? (write_idx - read_idx)
					: (preview_frames_ - read_idx + write_idx);
				if (used >= preview_frames_ - 1)
				{
					break;
				}
				size_t free_frames = (preview_frames_ - 1) - used;
				frames_to_read = free_frames;
				if (frames_to_read > max_frames_per_slice)
				{
					frames_to_read = max_frames_per_slice;
				}
				dst_buf = preview_buffer_;
				dst_write_idx = write_idx;
			}

			const size_t bytes_to_read = frames_to_read * g_preview_channels * sizeof(int16_t);
			const uint32_t read_start_ms = daisy::System::GetNow();
			UINT bytes_read = 0;
			FRESULT res = f_read(&g_preview_file, g_preview_read_buf, bytes_to_read, &bytes_read);
			const uint32_t read_elapsed_us = (daisy::System::GetNow() - read_start_ms) * 1000u;
			if (read_elapsed_us > g_storage_readchunk_max_us)
			{
				g_storage_readchunk_max_us = read_elapsed_us;
			}
			g_storage_readchunk_bytes += bytes_read;
			g_storage_readchunk_calls += 1;
			if (res != FR_OK)
			{
				f_close(&g_preview_file);
				g_preview_open = false;
				Event ev = {};
				ev.kind = EventKind::PreviewReadError;
				ev.cookie = g_preview_cookie;
				PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				break;
			}
			if (bytes_read == 0)
			{
				if (f_lseek(&g_preview_file, g_preview_data_offset) != FR_OK)
				{
					f_close(&g_preview_file);
					g_preview_open = false;
					Event ev = {};
					ev.kind = EventKind::PreviewReadError;
					ev.cookie = g_preview_cookie;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
					break;
				}
				continue;
			}
			const size_t frames_read = bytes_read / (g_preview_channels * sizeof(int16_t));
			if (have_pp)
			{
				for (size_t i = 0; i < frames_read; ++i)
				{
					int32_t mono = 0;
					if (g_preview_channels == 1)
					{
						mono = g_preview_read_buf[i];
					}
					else
					{
						const int16_t l = g_preview_read_buf[i * 2];
						const int16_t r = g_preview_read_buf[i * 2 + 1];
						mono = (static_cast<int32_t>(l) + static_cast<int32_t>(r)) / 2;
					}
					dst_buf[i] = static_cast<int16_t>(mono);
				}
				{
					daisy::ScopedIrqBlocker irq;
					if (pp_fill == 0)
					{
						StoragePublishBarrier();
						*preview_pp_ready_a_ = 1;
					}
					else
					{
						StoragePublishBarrier();
						*preview_pp_ready_b_ = 1;
					}
				}
			}
			else
			{
				size_t w = dst_write_idx;
				for (size_t i = 0; i < frames_read; ++i)
				{
					int32_t mono = 0;
					if (g_preview_channels == 1)
					{
						mono = g_preview_read_buf[i];
					}
					else
					{
						const int16_t l = g_preview_read_buf[i * 2];
						const int16_t r = g_preview_read_buf[i * 2 + 1];
						mono = (static_cast<int32_t>(l) + static_cast<int32_t>(r)) / 2;
					}
					dst_buf[w] = static_cast<int16_t>(mono);
					w = (w + 1) % preview_frames_;
				}
				{
					daisy::ScopedIrqBlocker irq;
					StoragePublishBarrier();
					*preview_write_index_ = w;
				}
			}
			if (budget_ms > 0 && (daisy::System::GetNow() - start_ms) >= budget_ms)
			{
				break;
			}
		}
	}

	// Defer scans while preview streaming to prioritize audio.
	if (g_preview_open)
	{
		return;
	}

	// Save write slice.
	if (save_.active && save_.header_written)
	{
		step_guard();
		const uint32_t start_ms = daisy::System::GetNow();
		if (save_.deadline_ms != 0 && start_ms > save_.deadline_ms)
		{
			f_close(&save_.file);
			save_.active = false;
			sd_status_.last_error = {SdErrorCode::Timeout, -1, 0, 0};
			sd_status_.last_error_time_ms = daisy::System::GetNow();
			++sd_counters_.op_timeouts;
			Event ev = {};
			ev.kind = EventKind::SaveError;
			PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
			return;
		}
		const uint32_t budget_ms = (budget_us + 999) / 1000;
		const size_t max_frames_per_slice = 8192;

		while (save_.frames_written < save_.frames_total)
		{
			const size_t frames_left = save_.frames_total - save_.frames_written;
			size_t frames_this = (frames_left > max_frames_per_slice) ? max_frames_per_slice : frames_left;
			UINT written = 0;
			FRESULT res = FR_OK;

			if (save_.channels == 1)
			{
				const int16_t* src = save_.src_l + save_.frames_written;
				res = f_write(&save_.file,
							  src,
							  static_cast<UINT>(frames_this * sizeof(int16_t)),
							  &written);
				if (res == FR_OK && written != (frames_this * sizeof(int16_t)))
				{
					res = FR_DISK_ERR;
				}
			}
			else
			{
				for (size_t i = 0; i < frames_this; ++i)
				{
					g_save_write_buf[i * 2] = save_.src_l[save_.frames_written + i];
					g_save_write_buf[i * 2 + 1] = save_.src_r[save_.frames_written + i];
				}
				res = f_write(&save_.file,
							  g_save_write_buf,
							  static_cast<UINT>(frames_this * save_.channels * sizeof(int16_t)),
							  &written);
				if (res == FR_OK
					&& written != (frames_this * save_.channels * sizeof(int16_t)))
				{
					res = FR_DISK_ERR;
				}
			}

			if (res != FR_OK)
			{
				f_close(&save_.file);
				save_.active = false;
				Event ev = {};
				ev.kind = EventKind::SaveError;
				PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				break;
			}

			save_.frames_written += frames_this;
			Event pev = {};
			pev.kind = EventKind::SaveProgress;
			pev.value = static_cast<uint32_t>(save_.frames_written);
			pev.value2 = static_cast<uint32_t>(save_.frames_total);
			PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, pev);

			if (budget_ms > 0 && (daisy::System::GetNow() - start_ms) >= budget_ms)
			{
				break;
			}
		}

		if (save_.active && save_.frames_written >= save_.frames_total)
		{
			daisy::WAV_FormatTypeDef header = {};
			header.ChunkId = daisy::kWavFileChunkId;
			header.FileSize = 36 + save_.data_bytes;
			header.FileFormat = daisy::kWavFileWaveId;
			header.SubChunk1ID = daisy::kWavFileSubChunk1Id;
			header.SubChunk1Size = 16;
			header.AudioFormat = daisy::WAVE_FORMAT_PCM;
			header.NbrChannels = save_.channels;
			header.SampleRate = save_.sample_rate;
			header.BlockAlign = static_cast<uint16_t>(save_.channels * sizeof(int16_t));
			header.ByteRate = save_.sample_rate * header.BlockAlign;
			header.BitPerSample = 16;
			header.SubChunk2ID = daisy::kWavFileSubChunk2Id;
			header.SubCHunk2Size = save_.data_bytes;

			if (f_lseek(&save_.file, 0) != FR_OK)
			{
				f_close(&save_.file);
				save_.active = false;
				Event ev = {};
				ev.kind = EventKind::SaveError;
				PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
			}
			else
			{
				UINT written = 0;
				FRESULT wr = f_write(&save_.file, &header, sizeof(header), &written);
				if (wr != FR_OK || written != sizeof(header))
				{
					f_close(&save_.file);
					save_.active = false;
					Event ev = {};
					ev.kind = EventKind::SaveError;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				}
				else
				{
					FRESULT sync_res = f_sync(&save_.file);
					f_close(&save_.file);
					save_.active = false;
					Event ev = {};
					ev.kind = (sync_res == FR_OK) ? EventKind::SaveDone : EventKind::SaveError;
					PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				}
			}
		}
	}

	// Load read slice.
	if (load_.active)
	{
		step_guard();
		const uint32_t start_ms = daisy::System::GetNow();
		if (load_.deadline_ms != 0 && start_ms > load_.deadline_ms)
		{
			f_close(&load_.file);
			load_.active = false;
			sd_status_.last_error = {SdErrorCode::Timeout, -1, 0, 0};
			sd_status_.last_error_time_ms = daisy::System::GetNow();
			++sd_counters_.op_timeouts;
			Event ev = {};
			ev.kind = EventKind::LoadError;
			ev.cookie = load_.cookie;
			PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
			return;
		}
		const uint32_t budget_ms = (budget_us + 999) / 1000;
		const size_t max_frames_per_slice = 1024;

		while (load_.frames_loaded < load_.frames_total)
		{
			size_t frames_left = load_.frames_total - load_.frames_loaded;
			size_t frames_this = (frames_left > max_frames_per_slice) ? max_frames_per_slice : frames_left;
			const size_t bytes_to_read = frames_this * load_.channels * sizeof(int16_t);
			UINT bytes_read = 0;
			FRESULT res = f_read(&load_.file, g_load_read_buf, bytes_to_read, &bytes_read);
			if (res != FR_OK || bytes_read == 0)
			{
				f_close(&load_.file);
				load_.active = false;
				Event ev = {};
				ev.kind = EventKind::LoadError;
				ev.cookie = load_.cookie;
				PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				break;
			}
			const size_t frames_read = bytes_read / (load_.channels * sizeof(int16_t));
			for (size_t i = 0; i < frames_read; ++i)
			{
				if (load_.channels == 1)
				{
					const int16_t samp = g_load_read_buf[i];
					load_.dst_l[load_.frames_loaded] = samp;
					load_.dst_r[load_.frames_loaded] = samp;
					load_.frames_loaded++;
				}
				else
				{
					load_.dst_l[load_.frames_loaded] = g_load_read_buf[i * 2];
					load_.dst_r[load_.frames_loaded] = g_load_read_buf[i * 2 + 1];
					load_.frames_loaded++;
				}
				if (load_.frames_loaded >= load_.frames_total)
				{
					break;
				}
			}
			{
				Event ev = {};
				ev.kind = EventKind::LoadProgress;
				ev.cookie = load_.cookie;
				ev.value = static_cast<uint32_t>(load_.frames_loaded);
				ev.value2 = static_cast<uint32_t>(load_.frames_total);
				PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
			}
			if (budget_ms > 0 && (daisy::System::GetNow() - start_ms) >= budget_ms)
			{
				break;
			}
		}

		if (load_.active && load_.frames_loaded >= load_.frames_total)
		{
			f_close(&load_.file);
			Event ev = {};
			ev.kind = EventKind::LoadDone;
			ev.cookie = load_.cookie;
			ev.value = static_cast<uint32_t>(load_.frames_loaded);
			ev.value2 = static_cast<uint32_t>(load_.frames_total);
			ev.sample_rate = load_.sample_rate;
			ev.channels = load_.channels;
			PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
			load_.active = false;
		}
	}

	if (scan_active_ || scan_done_pending_)
	{
		if (scan_done_pending_)
		{
			if (EventQueueHasRoom(ev_wr_, ev_rd_, kEventQueueSize))
			{
				Event ev = {};
				ev.kind = EventKind::ScanDone;
				ev.cookie = scan_cookie_;
				PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				scan_done_pending_ = false;
			}
		}
		else
		{
			const uint32_t start_ms = daisy::System::GetNow();
			const uint32_t budget_ms = (budget_us + 999) / 1000;
			const uint32_t max_entries_per_slice = 8;
			uint32_t entries = 0;

			while (scan_active_)
			{
				if (!EventQueueHasRoom(ev_wr_, ev_rd_, kEventQueueSize))
				{
					break;
				}
				if (scan_max_entries_ > 0 && scan_count_ >= scan_max_entries_)
				{
					f_closedir(&g_scan_dir);
					g_scan_dir_open = false;
					scan_active_ = false;
					scan_done_pending_ = true;
					break;
				}
				const FRESULT res = f_readdir(&g_scan_dir, &g_scan_fno);
				if (res != FR_OK || g_scan_fno.fname[0] == 0)
				{
					f_closedir(&g_scan_dir);
					g_scan_dir_open = false;
					scan_active_ = false;
					scan_done_pending_ = true;
					break;
				}
				if (g_scan_fno.fattrib & (AM_DIR | AM_HID))
				{
					continue;
				}
				if (scan_wav_only_ && !HasWavExtension(g_scan_fno.fname))
				{
					continue;
				}

				Event ev = {};
				ev.kind = EventKind::DirEntry;
				ev.cookie = scan_cookie_;
				ev.is_dir = false;
				ev.size = g_scan_fno.fsize;
				CopyString(ev.name, kNameMaxLen, g_scan_fno.fname);
				PushEvent(event_queue_, ev_wr_, ev_rd_, kEventQueueSize, ev);
				scan_count_++;
				entries++;

				if (entries >= max_entries_per_slice)
				{
					break;
				}
				if (budget_ms > 0 && (daisy::System::GetNow() - start_ms) >= budget_ms)
				{
					break;
				}
			}
		}
	}

	const uint32_t slice_elapsed_us = (daisy::System::GetNow() - slice_start_ms) * 1000u;
	if (slice_elapsed_us > g_storage_slice_max_us)
	{
		g_storage_slice_max_us = slice_elapsed_us;
	}
}
