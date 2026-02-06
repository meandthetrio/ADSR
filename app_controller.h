#pragma once

#include <cstddef>
#include <cstdint>

class AppController
{
public:
	void Init();
	void Tick(uint32_t now_ms);
};

bool AllocatePerformSample(size_t bytes, void** out_ptr);
void FreePerformSample();
