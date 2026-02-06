#pragma once

#include <cstdint>
#include <cstddef>
#include "shared_messages.h"

struct AudioUiState;

class Ui
{
public:
	void Init();
	void Tick(uint32_t now_ms);
};

void RequestDisplayUpdate();
void FlushDisplayIfDue(uint32_t now);

enum class LoadStubMode : int32_t;

void DrawMenu(int32_t selected);
void DrawPerformScreen(int32_t selected,
					   bool fx_select_active,
					   int32_t fx_selected,
					   bool amp_select_active,
					   int32_t amp_selected,
					   bool flt_select_active,
					   int32_t flt_selected,
					   uint8_t redraw_mask = 0x0F);
void DrawLoadMenu(int32_t top_index, int32_t selected);
void DrawLoadModeSelect(int32_t selected);
void DrawLoadStubScreen(LoadStubMode mode);
void DrawPresetSaveStub();
void DrawRecordReadyScreen();
void DrawRecordBackConfirm();
void DrawRecordSourceScreen();
void DrawRecordArmed();
void DrawRecordCountdown();
void DrawRecordRecording(const AudioUiState& ui_state);
void DrawRecordTargetScreen(int32_t selected);
void DrawRecordReview();
void DrawEdtScreen();
void DrawShiftMenu(int32_t selected);
void DrawSdInitScreen();
void DrawSaveScreen();
void DrawDeleteConfirm(const char* name);
void DrawFxDetailScreen(int32_t index);

void CopyString(char* dst, const char* src, size_t dst_len);

void UpdateDelaySlewCoeffs();
void InitSmoothers();
void UpdateSmoothedParamsPerTick();
void InitLoadLayout();
void ValidateConfig();
void MountSd();
bool IsPerformUiMode(UiMode mode);
void FinalizeFileList();
void PublishPreviewControlFromUi();
void StopPreview();
void JobStartWaveform(SampleContext ctx, bool foreground = true);
void UpdateTrimFrames();
void PublishRuntimeFromUi();
void PublishAudioParamsFromUi(const AudioParams& p);
float FxWetValue(int32_t fx_index);
void JobCancel();
void SetFxContext(FxContext ctx, int32_t track = 0);
void JobStartFileList(const char* path, bool wav_only, bool foreground = true);
bool LoadSampleAtIndex(int32_t index);
bool BeginPreviewAtIndex(int32_t index);
void FillPreviewBuffer();
bool DeleteFileAtIndex(int32_t index);
void ApplyLoadedSampleFade(size_t length, uint32_t rate);
