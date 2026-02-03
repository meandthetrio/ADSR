#pragma once

#include <cstddef>
#include <cstdint>
#include "ff.h"

class StorageService
{
public:
	enum class SdErrorCode : uint8_t
	{
		None = 0,
		NoCard,
		MountFailed,
		FsCorrupt,
		OpenFailed,
		ReadFailed,
		WriteFailed,
		SeekFailed,
		Timeout,
	};

	struct SdError
	{
		SdErrorCode code = SdErrorCode::None;
		int32_t fs_result = 0;
		uint32_t op_id = 0;
		uint32_t retries = 0;
	};

	struct SdStatus
	{
		bool mounted = false;
		bool present = false;
		SdError last_error = {};
		uint32_t last_error_time_ms = 0;
	};

	struct SdCounters
	{
		uint32_t mount_attempts = 0;
		uint32_t mount_failures = 0;
		uint32_t read_failures = 0;
		uint32_t write_failures = 0;
		uint32_t op_timeouts = 0;
		uint32_t cancels = 0;
	};

	enum class MountState : uint8_t
	{
		Uninit = 0,
		NoCard,
		Mounting,
		Mounted,
		Error,
	};

	enum class OpKind : uint8_t
	{
		None = 0,
		Mount,
		ScanDir,
		PreviewOpen,
		PreviewClose,
		DeleteFile,
		SaveStart,
		LoadStart,
	};

	enum class EventKind : uint8_t
	{
		None = 0,
		MountOk,
		MountFail,
		DirEntry,
		ScanDone,
		PreviewOpenOk,
		PreviewOpenFail,
		PreviewReadError,
		DeleteOk,
		DeleteFail,
		SaveProgress,
		SaveDone,
		SaveError,
		LoadProgress,
		LoadDone,
		LoadError,
	};

	struct Op
	{
		OpKind kind = OpKind::None;
		char path[64] = {};
		uint16_t max_entries = 0;
		uint16_t cookie = 0;
		uint32_t op_id = 0;
		uint8_t attempt = 0;
		uint32_t next_attempt_ms = 0;
		uint32_t start_ms = 0;
		uint32_t deadline_ms = 0;
		bool wav_only = false;
		const int16_t* src_l = nullptr;
		const int16_t* src_r = nullptr;
		size_t frames = 0;
		uint16_t channels = 0;
		uint32_t sample_rate = 0;
		int16_t* dst_l = nullptr;
		int16_t* dst_r = nullptr;
		size_t max_frames = 0;
	};

	struct Event
	{
		EventKind kind = EventKind::None;
		char name[32] = {};
		uint32_t size = 0;
		uint16_t cookie = 0;
		bool is_dir = false;
		uint32_t sample_rate = 0;
		uint16_t channels = 0;
		uint16_t bits_per_sample = 0;
		uint32_t value = 0;
		uint32_t value2 = 0;
	};

	struct PreviewStreamConfig
	{
		int16_t* buffer = nullptr;
		size_t frames = 0;
		volatile size_t* write_index = nullptr;
		volatile size_t* read_index = nullptr;
		int16_t* preload_buf = nullptr;
		size_t preload_frames = 0;
		int16_t* pp_buf_a = nullptr;
		int16_t* pp_buf_b = nullptr;
		size_t pp_frames = 0;
		volatile uint8_t* pp_ready_a = nullptr;
		volatile uint8_t* pp_ready_b = nullptr;
		volatile uint8_t* pp_active = nullptr;
	};

	static constexpr size_t kOpQueueSize = 16;
	static constexpr size_t kEventQueueSize = 16;
	static constexpr uint32_t kMinStorageBudgetUs = 1000;
	static constexpr uint32_t kSdRetryMaxAttempts = 3;

	static inline void StoragePublishBarrier() { __DMB(); }

	void Init();
	void SetPreviewStreamConfig(const PreviewStreamConfig& cfg);
	bool Enqueue(const Op& op);
	bool DequeueEvent(Event& out_event);
	// budget_us == 0 is clamped to kMinStorageBudgetUs.
	void RunSlice(uint32_t budget_us);
	void UnmountSd();
	const char* GetSdPath() const;
	const SdStatus& GetSdStatus() const { return sd_status_; }
	const SdCounters& GetSdCounters() const { return sd_counters_; }
	void ClearSdError();
	void CancelAllOps(SdErrorCode reason);

	MountState GetMountState() const { return mount_state_; }

private:
	static constexpr size_t kNameMaxLen = 32;
	static constexpr size_t kPathMaxLen = 64;

	Op op_queue_[kOpQueueSize];
	Event event_queue_[kEventQueueSize];
	uint8_t op_wr_ = 0;
	uint8_t op_rd_ = 0;
	uint8_t ev_wr_ = 0;
	uint8_t ev_rd_ = 0;
	MountState mount_state_ = MountState::Uninit;
	bool sd_hw_inited_ = false;
	SdStatus sd_status_ = {};
	SdCounters sd_counters_ = {};
	uint32_t next_op_id_ = 1;

	bool scan_active_ = false;
	bool scan_wav_only_ = false;
	bool scan_done_pending_ = false;
	uint16_t scan_cookie_ = 0;
	uint16_t scan_max_entries_ = 0;
	uint16_t scan_count_ = 0;
	char scan_path_[kPathMaxLen] = {};

	int16_t* preview_buffer_ = nullptr;
	size_t preview_frames_ = 0;
	volatile size_t* preview_write_index_ = nullptr;
	volatile size_t* preview_read_index_ = nullptr;
	int16_t* preview_preload_buf_ = nullptr;
	size_t preview_preload_frames_ = 0;
	size_t preview_preload_filled_ = 0;
	size_t preview_preload_target_frames_ = 0;
	bool preview_preload_active_ = false;

	struct SaveState
	{
		bool active = false;
		bool header_written = false;
		bool done = false;
		FIL file = {};
		const int16_t* src_l = nullptr;
		const int16_t* src_r = nullptr;
		size_t frames_total = 0;
		size_t frames_written = 0;
		uint16_t channels = 0;
		uint32_t sample_rate = 0;
		uint32_t data_bytes = 0;
		uint32_t start_ms = 0;
		uint32_t deadline_ms = 0;
		char path[kPathMaxLen] = {};
	};

	SaveState save_;

	struct LoadState
	{
		bool active = false;
		FIL file = {};
		int16_t* dst_l = nullptr;
		int16_t* dst_r = nullptr;
		size_t max_frames = 0;
		size_t frames_total = 0;
		size_t frames_loaded = 0;
		uint16_t channels = 0;
		uint32_t sample_rate = 0;
		uint32_t data_offset = 0;
		uint16_t cookie = 0;
		uint32_t start_ms = 0;
		uint32_t deadline_ms = 0;
	};

	LoadState load_;

	int16_t* preview_pp_a_ = nullptr;
	int16_t* preview_pp_b_ = nullptr;
	size_t preview_pp_frames_ = 0;
	volatile uint8_t* preview_pp_ready_a_ = nullptr;
	volatile uint8_t* preview_pp_ready_b_ = nullptr;
	volatile uint8_t* preview_pp_active_ = nullptr;
};
