#include "SampleMemoryManager.h"

SampleMemoryManager::SampleMemoryManager(void* pool, size_t bytes)
	: pool_(pool)
	, pool_bytes_(bytes)
{
	entry_.ptr = pool_;
}

bool SampleMemoryManager::CanFit(size_t bytes) const
{
	return bytes <= pool_bytes_;
}

bool SampleMemoryManager::Allocate(uint32_t sample_id, size_t bytes, void** out_ptr)
{
	if (!out_ptr || !pool_ || bytes == 0 || bytes > pool_bytes_)
	{
		return false;
	}
	if (entry_.allocated && entry_.sample_id != sample_id)
	{
		return false;
	}
	entry_.allocated = true;
	entry_.sample_id = sample_id;
	entry_.bytes = bytes;
	*out_ptr = pool_;
	return true;
}

void SampleMemoryManager::Free(uint32_t sample_id)
{
	if (entry_.allocated && entry_.sample_id == sample_id)
	{
		entry_.allocated = false;
		entry_.sample_id = 0;
		entry_.bytes = 0;
		entry_.refcount = 0;
	}
}

bool SampleMemoryManager::Acquire(uint32_t sample_id)
{
	if (!entry_.allocated || entry_.sample_id != sample_id)
	{
		return false;
	}
	++entry_.refcount;
	return true;
}

void SampleMemoryManager::Release(uint32_t sample_id)
{
	if (entry_.allocated && entry_.sample_id == sample_id && entry_.refcount > 0)
	{
		--entry_.refcount;
	}
}

size_t SampleMemoryManager::BytesUsed() const
{
	return entry_.allocated ? entry_.bytes : 0;
}

size_t SampleMemoryManager::BytesFree() const
{
	return pool_bytes_ - BytesUsed();
}
