#include "app_controller.h"

#include "daisy_pod.h"
#include "SamplerConfig.h"
#include "StorageService.h"
#include "audio_engine.h"
#include "ui.h"
#include "shared_messages.h"

using namespace daisy;

#ifndef STORAGE_SERVICE_PREVIEW_STREAM
#define STORAGE_SERVICE_PREVIEW_STREAM PREVIEW_STREAM_FROM_SD
#endif

#ifndef STORAGE_SERVICE_SAVE
#define STORAGE_SERVICE_SAVE 1
#endif

constexpr uint32_t kStorageBudgetUs = 2000;
constexpr uint32_t kStoragePreviewBudgetUs = 12000;
constexpr uint32_t kPreviewFadeInMs = 10;
constexpr uint32_t kSaveResultMs = 1500;
constexpr uint32_t kSdInitMinMs = 800;
constexpr uint32_t kSdInitRetryMs = 300;
constexpr uint32_t kSdInitResultMs = 1500;
constexpr int32_t kSdInitAttempts = 3;
constexpr uint32_t kLoadScanGraceMs = 300;
constexpr uint32_t kPerformPlayheadIntervalMs = 33;
constexpr float kLedBlinkPeriodMs = 25.0f;
constexpr float kLedBlinkDuty = 0.5f;
constexpr int32_t kPerformBoxCount = 4;
constexpr int32_t kPerformEdtIndex = 0;
constexpr int32_t kPerformFaderCount = 4;
constexpr int32_t kPerformAmpIndex = 1;
constexpr int32_t kPerformFltIndex = 2;
constexpr int32_t kPerformFxIndex = 3;
constexpr int32_t kPerformFltFaderCount = 2;
constexpr int32_t kFxReverbIndex = 3;
constexpr int32_t kMaxWavFiles = 32;
constexpr size_t kMaxWavNameLen = 32;
constexpr size_t kPreviewPpFrames = 2048;
constexpr bool kLoadPresetsPlaceholder = true;
constexpr int32_t kBaseMidiNote = 60;


static AppContext* g_ctx = nullptr;

#define hw (*g_ctx->hw)
#define storage (*g_ctx->storage)
#define g_ui (*g_ctx->ui)
#define g_last_draw_ms (*g_ctx->last_draw_ms)

#define ui_mode (*g_ctx->ui_mode)
#define menu_index (*g_ctx->menu_index)
#define shift_menu_index (*g_ctx->shift_menu_index)
#define perform_index (*g_ctx->perform_index)
#define amp_fader_index (*g_ctx->amp_fader_index)
#define flt_fader_index (*g_ctx->flt_fader_index)
#define fx_fader_index (*g_ctx->fx_fader_index)
#define fx_chain_order (g_ctx->fx_chain_order)
#define fx_window_active (*g_ctx->fx_window_active)
#define amp_window_active (*g_ctx->amp_window_active)
#define flt_window_active (*g_ctx->flt_window_active)
#define shift_prev_mode (*g_ctx->shift_prev_mode)
#define record_input (*g_ctx->record_input)
#define load_selected (*g_ctx->load_selected)
#define load_scroll (*g_ctx->load_scroll)
#define request_load_scan (*g_ctx->request_load_scan)
#define list_build_pending (*g_ctx->list_build_pending)
#define request_load_sample (*g_ctx->request_load_sample)
#define request_load_index (*g_ctx->request_load_index)
#define load_in_progress (*g_ctx->load_in_progress)
#define load_cookie_next (*g_ctx->load_cookie_next)
#define load_cookie_active (*g_ctx->load_cookie_active)
#define load_target_is_edt (*g_ctx->load_target_is_edt)
#define load_context (*g_ctx->load_context)
#define loader_state (*g_ctx->loader_state)
#define load_success_count (*g_ctx->load_success_count)
#define load_fail_budget_count (*g_ctx->load_fail_budget_count)
#define load_fail_io_count (*g_ctx->load_fail_io_count)
#define wav_file_count (*g_ctx->wav_file_count)
#define sd_mounted (*g_ctx->sd_mounted)
#define sd_present (*g_ctx->sd_present)
#define sd_fault (*g_ctx->sd_fault)
#define sd_fault_text (*g_ctx->sd_fault_text)
#define sd_retries_remaining (*g_ctx->sd_retries_remaining)
#define sd_init_in_progress (*g_ctx->sd_init_in_progress)
#define sd_init_done (*g_ctx->sd_init_done)
#define sd_init_success (*g_ctx->sd_init_success)
#define sd_init_start_ms (*g_ctx->sd_init_start_ms)
#define sd_init_next_ms (*g_ctx->sd_init_next_ms)
#define sd_init_result_until_ms (*g_ctx->sd_init_result_until_ms)
#define sd_init_draw_next_ms (*g_ctx->sd_init_draw_next_ms)
#define sd_init_attempts (*g_ctx->sd_init_attempts)
#define sd_init_prev_mode (*g_ctx->sd_init_prev_mode)
#define save_in_progress (*g_ctx->save_in_progress)
#define save_done (*g_ctx->save_done)
#define save_success (*g_ctx->save_success)
#define save_started (*g_ctx->save_started)
#define save_start_ms (*g_ctx->save_start_ms)
#define save_result_until_ms (*g_ctx->save_result_until_ms)
#define save_draw_next_ms (*g_ctx->save_draw_next_ms)
#define save_prev_mode (*g_ctx->save_prev_mode)
#define save_filename (g_ctx->save_filename)
#define save_frames_written (*g_ctx->save_frames_written)
#define delete_mode (*g_ctx->delete_mode)
#define delete_prev_mode (*g_ctx->delete_prev_mode)
#define request_delete_scan (*g_ctx->request_delete_scan)
#define request_delete_file (*g_ctx->request_delete_file)
#define request_delete_index (*g_ctx->request_delete_index)
#define delete_confirm (*g_ctx->delete_confirm)
#define delete_confirm_name (g_ctx->delete_confirm_name)
#define phones_volume (*g_ctx->phones_volume)
#define delete_in_progress (*g_ctx->delete_in_progress)
#define delete_cookie_next (*g_ctx->delete_cookie_next)
#define delete_cookie_active (*g_ctx->delete_cookie_active)
#define wav_files (g_ctx->wav_files)
#define loaded_sample_name (g_ctx->loaded_sample_name)
#define load_lines (*g_ctx->load_lines)
#define load_line_height (*g_ctx->load_line_height)
#define load_chars_per_line (*g_ctx->load_chars_per_line)
#define load_scan_start_ms (*g_ctx->load_scan_start_ms)
#define current_sample_context (*g_ctx->current_sample_context)
#define edt_sample_context (*g_ctx->edt_sample_context)
#define load_mode_index (*g_ctx->load_mode_index)
#define load_stub_mode (*g_ctx->load_stub_mode)
#define sample_length (*g_ctx->sample_length)
#define sample_play_start (*g_ctx->sample_play_start)
#define sample_play_end (*g_ctx->sample_play_end)
#define sample_rate (*g_ctx->sample_rate)
#define sample_channels (*g_ctx->sample_channels)
#define sample_loaded (*g_ctx->sample_loaded)
#define playback_active (*g_ctx->playback_active)
#define playback_rate (*g_ctx->playback_rate)
#define playback_phase (*g_ctx->playback_phase)
#define playback_reverse (*g_ctx->playback_reverse)
#define g_perform_voices_active (*g_ctx->perform_voices_active)
#define g_audio_cmd (*g_ctx->audio_cmd)
#define g_audio_flags_bits (*g_ctx->audio_flags_bits)
#define g_audio_ui_state_buf (g_ctx->audio_ui_state_buf)
#define g_audio_ui_state_idx (*g_ctx->audio_ui_state_idx)
#define g_audio_recording_active (*g_ctx->audio_recording_active)
#define g_active_voice_count (*g_ctx->active_voice_count)
#define g_delay_time_alpha (*g_ctx->delay_time_alpha)
#define g_delay_param_alpha (*g_ctx->delay_param_alpha)
#define record_state (*g_ctx->record_state)
#define record_source_index (*g_ctx->record_source_index)
#define record_target_index (*g_ctx->record_target_index)
#define record_countdown_start_ms (*g_ctx->record_countdown_start_ms)
#define record_pos (*g_ctx->record_pos)
#define g_recorded_length_audio (*g_ctx->recorded_length_audio)
#define g_record_start_ms (*g_ctx->record_start_ms)
#define record_waveform_pending (*g_ctx->record_waveform_pending)
#define encoder_r_accum (*g_ctx->encoder_r_accum)
#define encoder_r_button_press (*g_ctx->encoder_r_button_press)
#define request_length_redraw (*g_ctx->request_length_redraw)
#define request_playhead_redraw (*g_ctx->request_playhead_redraw)
#define button1_press (*g_ctx->button1_press)
#define button2_press (*g_ctx->button2_press)
#define request_playback_stop_log (*g_ctx->request_playback_stop_log)
#define reverb_wet (*g_ctx->reverb_wet)
#define reverb_pre (*g_ctx->reverb_pre)
#define reverb_damp (*g_ctx->reverb_damp)
#define reverb_decay (*g_ctx->reverb_decay)
#define delay_wet (*g_ctx->delay_wet)
#define delay_time (*g_ctx->delay_time)
#define delay_feedback (*g_ctx->delay_feedback)
#define delay_spread (*g_ctx->delay_spread)
#define delay_freeze (*g_ctx->delay_freeze)
#define fx_s_wet (*g_ctx->fx_s_wet)
#define sat_drive (*g_ctx->sat_drive)
#define sat_tape_bump (*g_ctx->sat_tape_bump)
#define sat_bit_reso (*g_ctx->sat_bit_reso)
#define sat_bit_smpl (*g_ctx->sat_bit_smpl)
#define fx_c_wet (*g_ctx->fx_c_wet)
#define mod_depth (*g_ctx->mod_depth)
#define chorus_rate (*g_ctx->chorus_rate)
#define sat_mode (*g_ctx->sat_mode)
#define chorus_mode (*g_ctx->chorus_mode)
#define chorus_wow (*g_ctx->chorus_wow)
#define tape_rate (*g_ctx->tape_rate)
#define fx_params_dirty (*g_ctx->fx_params_dirty)
#define audio_params_dirty (*g_ctx->audio_params_dirty)
#define sat_params_initialized (*g_ctx->sat_params_initialized)
#define reverb_params_initialized (*g_ctx->reverb_params_initialized)
#define delay_params_initialized (*g_ctx->delay_params_initialized)
#define mod_params_initialized (*g_ctx->mod_params_initialized)
#define amp_attack (*g_ctx->amp_attack)
#define amp_decay (*g_ctx->amp_decay)
#define amp_sustain (*g_ctx->amp_sustain)
#define amp_release (*g_ctx->amp_release)
#define fx_detail_index (*g_ctx->fx_detail_index)
#define fx_detail_param_index (*g_ctx->fx_detail_param_index)
#define flt_cutoff (*g_ctx->flt_cutoff)
#define flt_res (*g_ctx->flt_res)
#define preview_hold (*g_ctx->preview_hold)
#define fx_chain_fade_gain (*g_ctx->fx_chain_fade_gain)
#define fx_chain_fade_target (*g_ctx->fx_chain_fade_target)
#define fx_chain_fade_samples_left (*g_ctx->fx_chain_fade_samples_left)
#define fx_chain_pause_pending (*g_ctx->fx_chain_pause_pending)
#define fx_chain_paused (*g_ctx->fx_chain_paused)
#define fx_chain_last_move_ms (*g_ctx->fx_chain_last_move_ms)
#define preview_active (*g_ctx->preview_active)
#define preview_index (*g_ctx->preview_index)
#define preview_sample_rate (*g_ctx->preview_sample_rate)
#define preview_channels (*g_ctx->preview_channels)
#define preview_rate (*g_ctx->preview_rate)
#define preview_read_frac (*g_ctx->preview_read_frac)
#define preview_read_index (*g_ctx->preview_read_index)
#define preview_write_index (*g_ctx->preview_write_index)
#define preview_data_offset (*g_ctx->preview_data_offset)
#define preview_fade_samples_left (*g_ctx->preview_fade_samples_left)
#define preview_fade_samples_total (*g_ctx->preview_fade_samples_total)
#define preview_pending_start (*g_ctx->preview_pending_start)
#define preview_pending_start_ms (*g_ctx->preview_pending_start_ms)
#define preview_underrun_count (*g_ctx->preview_underrun_count)
#define preview_rb_min_level (*g_ctx->preview_rb_min_level)
#define preview_stream_cookie (*g_ctx->preview_stream_cookie)
#define preview_stream_cookie_active (*g_ctx->preview_stream_cookie_active)
#define preview_pp_ready (g_ctx->preview_pp_ready)
#define preview_pp_active (*g_ctx->preview_pp_active)
#define preview_pp_pos (*g_ctx->preview_pp_pos)
#define preview_preload_frames (*g_ctx->preview_preload_frames)
#define preview_preload_active (*g_ctx->preview_preload_active)
#define led1_level (*g_ctx->led1_level)
#define led1_phase_ms (*g_ctx->led1_phase_ms)
#define record_anim_start_ms (*g_ctx->record_anim_start_ms)
#define request_shift_redraw (*g_ctx->request_shift_redraw)
#define request_perform_redraw (*g_ctx->request_perform_redraw)
#define request_fx_detail_redraw (*g_ctx->request_fx_detail_redraw)
#define delay_snow_next_ms (*g_ctx->delay_snow_next_ms)
#define midi_ignore_until_ms (*g_ctx->midi_ignore_until_ms)
#define g_job (*g_ctx->g_job)
#define g_wf_job (*g_ctx->g_wf_job)
#define g_list_job (*g_ctx->g_list_job)
#define waveform_ready (*g_ctx->waveform_ready)
#define waveform_dirty (*g_ctx->waveform_dirty)
#define waveform_from_recording (*g_ctx->waveform_from_recording)
#define waveform_title (*g_ctx->waveform_title)
#define request_delete_redraw (*g_ctx->request_delete_redraw)
#define g_display_update_pending (*g_ctx->display_update_pending)
#define waveform_compute_pending (*g_ctx->waveform_compute_pending)
#define waveform_compute_ctx (*g_ctx->waveform_compute_ctx)
#define g_audio_event_bits (*g_ctx->audio_event_bits)
#define sample_mem_used_bytes (*g_ctx->sample_mem_used_bytes)
#define sample_mem_free_bytes (*g_ctx->sample_mem_free_bytes)
#define waveform_cache_bytes (*g_ctx->waveform_cache_bytes)


// OWNER: UI/main loop.
// WRITES: AppController only.
// READS: AppController only.
struct AppState
{
	StorageService::SdErrorCode sd_fault_code = StorageService::SdErrorCode::None;
	UiMode last_mode = UiMode::Main;
	int32_t last_menu = 0;
	int32_t last_scroll = 0;
	int32_t last_selected = 0;
	int32_t last_file_count = 0;
	bool last_sd_mounted = false;
	int32_t last_perform_index = -1;
	int32_t last_shift_menu = 0;
	int32_t last_fx_detail_index = -1;
	int32_t last_fx_detail_param_index = -1;
	uint32_t perform_redraw_next_ms = 0;
	float last_perform_amp_vals[kPerformFaderCount] = {-1.0f, -1.0f, -1.0f, -1.0f};
	float last_perform_flt_vals[kPerformFltFaderCount] = {-1.0f, -1.0f};
	float last_perform_fx_vals[kPerformFaderCount] = {-1.0f, -1.0f, -1.0f, -1.0f};
	int32_t last_perform_fx_order[kPerformFaderCount] = {-1, -1, -1, -1};
	bool last_perform_fx_select_active = false;
	bool last_perform_amp_select_active = false;
	bool last_perform_flt_select_active = false;
	int32_t last_perform_fx_selected = -1;
	int32_t last_perform_amp_selected = -1;
	int32_t last_perform_flt_selected = -1;
	uint32_t rev_anim_next_ms = 0;
	uint32_t record_draw_next_ms = 0;
	uint32_t last_edt_playhead_ms = 0;
	bool last_edt_playhead_active = false;
	bool last_playback_active = false;
	RecordState last_record_state = RecordState::Armed;
	uint8_t last_audio_ui_idx = 0;
};
static AppState g_app;

bool AllocatePerformSample(size_t bytes, void** out_ptr)
{
	return g_ctx->audio->AllocatePerformSample(bytes, out_ptr);
}

void FreePerformSample()
{
	g_ctx->audio->FreePerformSample();
}

void AppController::Init(AppContext* ctx)
{
	g_ctx = ctx;
	storage.Init();
	g_ctx->audio->InitVoices();
	ValidateConfig();
	{
		StorageService::PreviewStreamConfig cfg = {};
		const PreviewBuffers pb = g_ctx->audio->GetPreviewBuffers();
		cfg.buffer = pb.buffer;
		cfg.frames = pb.frames;
		cfg.write_index = pb.write_index;
		cfg.read_index = pb.read_index;
#if STORAGE_SERVICE_PREVIEW_STREAM
		cfg.preload_buf = pb.preload_buf;
		cfg.preload_frames = pb.preload_frames;
		cfg.pp_buf_a = pb.pp_buf_a;
		cfg.pp_buf_b = pb.pp_buf_b;
		cfg.pp_frames = pb.pp_frames;
		cfg.pp_ready_a = pb.pp_ready_a;
		cfg.pp_ready_b = pb.pp_ready_b;
		cfg.pp_active = pb.pp_active;
#endif
		storage.SetPreviewStreamConfig(cfg);
	}
	g_ui.Init(g_ctx);
	MountSd();
	DrawMenu(menu_index);
	g_last_draw_ms = System::GetNow();
}

void AppController::Tick(uint32_t /*now_ms*/)
{
	// remaining state lives in g_app

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
					g_ctx->audio->FreePerformSample();
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
			g_app.sd_fault_code = st.last_error.code;
			sd_fault = (g_app.sd_fault_code != StorageService::SdErrorCode::None);
			sd_fault_text = sd_fault ? SdFaultText(g_app.sd_fault_code) : nullptr;
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
						g_app.last_mode = UiMode::Shift;
					}
					else
					{
						ui_mode = sd_init_prev_mode;
						g_app.last_mode = UiMode::Shift;
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
					const SampleBuffers bufs = g_ctx->audio->GetSampleBuffers();
					op.src_l = bufs.l;
					op.src_r = bufs.r;
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
				g_app.last_mode = UiMode::Shift;
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
					g_app.last_record_state = record_state;
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
			g_app.last_shift_menu = shift_menu_index;
		}

		const UiMode mode = ui_mode;
		if (mode != g_app.last_mode)
		{
			if (g_app.last_mode == UiMode::Load && mode != UiMode::Load && g_job.type == JobType::FileListScan)
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
			if (IsPerformUiMode(g_app.last_mode) && !IsPerformUiMode(mode))
			{
				RequestAudioCmd(kCmdAllNotesOff);
			}
			if (mode == UiMode::Record)
			{
				record_anim_start_ms = NowMs();
			}
			else if (g_app.last_mode == UiMode::Record)
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
				g_app.last_fx_detail_index = fx_detail_index;
				g_app.last_fx_detail_param_index = fx_detail_param_index;
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
					g_app.last_perform_amp_vals[i] = amp_vals[i];
					g_app.last_perform_fx_order[i] = fx_chain_order[i];
					g_app.last_perform_fx_vals[i] = FxWetValue(fx_chain_order[i]);
				}
				for (int i = 0; i < kPerformFltFaderCount; ++i)
				{
					g_app.last_perform_flt_vals[i] = flt_vals[i];
				}
				g_app.last_perform_fx_select_active = fx_select_active;
				g_app.last_perform_amp_select_active = amp_select_active;
				g_app.last_perform_flt_select_active = flt_select_active;
				g_app.last_perform_fx_selected = fx_fader_index;
				g_app.last_perform_amp_selected = amp_fader_index;
				g_app.last_perform_flt_selected = flt_fader_index;
				g_app.last_perform_index = perform_index;
				g_app.perform_redraw_next_ms = System::GetNow() + kPerformPlayheadIntervalMs;
			}
			else if (mode == UiMode::Shift)
			{
				DrawShiftMenu(shift_menu_index);
				g_app.last_shift_menu = shift_menu_index;
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
			g_app.last_mode = mode;
			g_app.last_menu = menu_index;
			g_app.last_scroll = load_scroll;
			g_app.last_selected = load_selected;
			g_app.last_file_count = wav_file_count;
			g_app.last_sd_mounted = sd_mounted;
			g_app.last_record_state = record_state;
			g_app.last_perform_index = perform_index;
		}
		else if (mode == UiMode::Main)
		{
			const int32_t current = menu_index;
			if (current != g_app.last_menu)
			{
				DrawMenu(current);
				g_app.last_menu = current;
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
				if (amp_vals[i] != g_app.last_perform_amp_vals[i])
				{
					amp_changed = true;
					break;
				}
			}
			bool flt_changed = false;
			const float flt_vals[kPerformFltFaderCount] = {flt_cutoff, flt_res};
			for (int i = 0; i < kPerformFltFaderCount; ++i)
			{
				if (flt_vals[i] != g_app.last_perform_flt_vals[i])
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
				if (fx_order[i] != g_app.last_perform_fx_order[i]
					|| fx_vals[i] != g_app.last_perform_fx_vals[i])
				{
					fx_changed = true;
				}
			}

			const bool selection_changed = (current != g_app.last_perform_index);
			const bool fx_select_changed
				= (fx_select_active != g_app.last_perform_fx_select_active)
				|| (fx_fader_index != g_app.last_perform_fx_selected);
			const bool amp_select_changed
				= (amp_select_active != g_app.last_perform_amp_select_active)
				|| (amp_fader_index != g_app.last_perform_amp_selected);
			const bool flt_select_changed
				= (flt_select_active != g_app.last_perform_flt_select_active)
				|| (flt_fader_index != g_app.last_perform_flt_selected);

			uint8_t redraw_mask = 0;
			if (selection_changed)
			{
				redraw_mask |= (1u << current);
				if (g_app.last_perform_index >= 0)
				{
					redraw_mask |= (1u << g_app.last_perform_index);
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
				&& g_app.last_perform_index == kPerformEdtIndex)
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
				if (now >= g_app.perform_redraw_next_ms)
				{
					DrawPerformScreen(current,
									  fx_select_active,
									  fx_fader_index,
									  amp_select_active,
									  amp_fader_index,
									  flt_select_active,
									  flt_fader_index,
									  redraw_mask);
					g_app.perform_redraw_next_ms = now + kPerformPlayheadIntervalMs;
					for (int i = 0; i < kPerformFaderCount; ++i)
					{
						g_app.last_perform_amp_vals[i] = amp_vals[i];
						g_app.last_perform_fx_order[i] = fx_order[i];
						g_app.last_perform_fx_vals[i] = fx_vals[i];
					}
					for (int i = 0; i < kPerformFltFaderCount; ++i)
					{
						g_app.last_perform_flt_vals[i] = flt_vals[i];
					}
					g_app.last_perform_fx_select_active = fx_select_active;
					g_app.last_perform_amp_select_active = amp_select_active;
					g_app.last_perform_flt_select_active = flt_select_active;
					g_app.last_perform_fx_selected = fx_fader_index;
					g_app.last_perform_amp_selected = amp_fader_index;
					g_app.last_perform_flt_selected = flt_fader_index;
					g_app.last_perform_index = current;
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
				if (now >= g_app.rev_anim_next_ms)
				{
					request_fx_detail_redraw = true;
					g_app.rev_anim_next_ms = now + 120;
				}
			}
			else
			{
				g_app.rev_anim_next_ms = 0;
			}
			if (request_fx_detail_redraw
				|| fx_detail_index != g_app.last_fx_detail_index
				|| fx_detail_param_index != g_app.last_fx_detail_param_index)
			{
				request_fx_detail_redraw = false;
				DrawFxDetailScreen(fx_detail_index);
				g_app.last_fx_detail_index = fx_detail_index;
				g_app.last_fx_detail_param_index = fx_detail_param_index;
			}
		}
		else if (mode == UiMode::Shift)
		{
			if (!ui_blocked)
			{
				const int32_t current = shift_menu_index;
				if (current != g_app.last_shift_menu)
				{
					DrawShiftMenu(current);
					g_app.last_shift_menu = current;
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
					|| current_scroll != g_app.last_scroll
					|| current_selected != g_app.last_selected
					|| current_count != g_app.last_file_count
					|| sd_mounted != g_app.last_sd_mounted)
				{
					request_delete_redraw = false;
					DrawLoadMenu(current_scroll, current_selected);
					if (current_selected != g_app.last_selected || current_count != g_app.last_file_count)
					{
					}
					g_app.last_scroll = current_scroll;
					g_app.last_selected = current_selected;
					g_app.last_file_count = current_count;
					g_app.last_sd_mounted = sd_mounted;
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
			if (current_state != g_app.last_record_state)
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
					g_app.record_draw_next_ms = 0;
				}
				else if (current_state == RecordState::TargetSelect)
				{
					DrawRecordTargetScreen(record_target_index);
				}
				else
				{
					DrawRecordReview();
				}
				g_app.last_record_state = current_state;
			}
		}
		if (!ui_blocked && mode == UiMode::Record && record_state == RecordState::Recording)
		{
			uint8_t ui_idx = 0;
			const AudioUiState& uir = GetAudioUiStateSnapshot(ui_idx);
			const uint32_t now = System::GetNow();
			if (g_app.record_draw_next_ms == 0 || now >= g_app.record_draw_next_ms)
			{
				DrawRecordRecording(uir);
				g_app.record_draw_next_ms = now + 33;
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
			if (!g_app.last_edt_playhead_active)
			{
				g_app.last_edt_playhead_ms = now;
				waveform_dirty = true;
				request_playhead_redraw = true;
			}
			else if ((now - g_app.last_edt_playhead_ms) >= kPerformPlayheadIntervalMs)
			{
				g_app.last_edt_playhead_ms = now;
				waveform_dirty = true;
				request_playhead_redraw = true;
			}
		}
		else
		{
			g_app.last_edt_playhead_ms = 0;
		}
		g_app.last_edt_playhead_active = edt_playhead_active;
		const bool playback_active_now = play_uir.playback_active;
		if (!ui_blocked && (request_playhead_redraw || (playback_active_now != g_app.last_playback_active)))
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
		g_app.last_playback_active = playback_active_now;
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
			if (audio_ui_idx != g_app.last_audio_ui_idx)
			{
				g_app.last_audio_ui_idx = audio_ui_idx;
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
		g_ctx->audio->GetSampleMemUsage(sample_mem_used_bytes, sample_mem_free_bytes);
		waveform_cache_bytes = g_ctx->ui->GetWaveformCacheBytes();
 		hw.UpdateLeds();
		hw.DelayMs(1);
}


