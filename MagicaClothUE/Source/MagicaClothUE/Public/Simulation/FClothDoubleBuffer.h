#pragma once

#include "CoreMinimal.h"
#include "Containers/Array.h"
#include "Math/Transform.h"
#include <atomic>

/**
 * Thread-safe double buffer for passing simulation results to the game thread.
 * The physics worker thread writes to one buffer while the game thread reads from the other.
 * Uses atomic index swap — no locks required. Safe for single-producer/single-consumer.
 */
struct FClothDoubleBuffer
{
	TArray<FTransform> Buffer[2];
	std::atomic<int32> WriteIndex{0};

	void Write(const TArray<FTransform>& Data)
	{
		const int32 Next = 1 - WriteIndex.load(std::memory_order_acquire);
		Buffer[Next] = Data;
		WriteIndex.store(Next, std::memory_order_release);
	}

	const TArray<FTransform>& Read() const
	{
		return Buffer[WriteIndex.load(std::memory_order_acquire)];
	}

	void Reset()
	{
		Buffer[0].Empty();
		Buffer[1].Empty();
		WriteIndex.store(0, std::memory_order_relaxed);
	}

	int32 Num() const
	{
		return Buffer[WriteIndex.load(std::memory_order_acquire)].Num();
	}
};
