#pragma once

#include <cstddef>
#include <cstdint>

class SampleMemoryManager
{
public:
	SampleMemoryManager(void* pool, size_t bytes);

	bool CanFit(size_t bytes) const;
	bool Allocate(uint32_t sample_id, size_t bytes, void** out_ptr);
	void Free(uint32_t sample_id);
	bool Acquire(uint32_t sample_id);
	void Release(uint32_t sample_id);

	size_t BytesUsed() const;
	size_t BytesFree() const;

private:
	struct Entry
	{
		uint32_t sample_id = 0;
		void* ptr = nullptr;
		size_t bytes = 0;
		uint32_t refcount = 0;
		bool allocated = false;
	};

	void* pool_ = nullptr;
	size_t pool_bytes_ = 0;
	Entry entry_;
};
