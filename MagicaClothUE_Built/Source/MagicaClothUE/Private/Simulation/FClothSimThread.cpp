#include "Simulation/FClothSimThread.h"
#include "Colliders/MagicaColliderShapes.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Math/UnrealMathUtility.h"

FClothSimThread::FClothSimThread() = default;

FClothSimThread::~FClothSimThread()
{
	// Do NOT call Stop() here — it would cause infinite recursion:
	// ~FClothSimThread -> Stop() -> delete Thread -> ~FRunnableThreadWin -> delete Runnable(this) -> ~FClothSimThread -> ...
	// Instead, just signal the thread to stop. The thread must be stopped explicitly via Stop() before destruction.
	bRunning.store(false, std::memory_order_release);
}

void FClothSimThread::Start()
{
	if (bRunning.load(std::memory_order_acquire))
	{
		return;
	}

	bRunning.store(true, std::memory_order_release);
	LastSimTime = FPlatformTime::Seconds();
	Thread = FRunnableThread::Create(this, TEXT("MagicaClothSimThread"), 0, TPri_AboveNormal);
}

void FClothSimThread::Stop()
{
	bRunning.store(false, std::memory_order_release);

	if (Thread)
	{
		// Null out FIRST to prevent re-entry from the destructor chain:
		// delete FRunnableThreadWin -> delete FRunnable(this) -> ~FClothSimThread -> Stop()
		FRunnableThread* ThreadToClean = Thread;
		Thread = nullptr;

		ThreadToClean->WaitForCompletion();
		delete ThreadToClean;
	}
}

bool FClothSimThread::Init()
{
	return true;
}

uint32 FClothSimThread::Run()
{
	while (bRunning.load(std::memory_order_acquire))
	{
		const float SimDt = 1.f / FMath::Max(SimulationHz, MinSimulationHz);
		const double FrameStart = FPlatformTime::Seconds();

		// Handle reset request
		if (bResetRequested.load(std::memory_order_acquire))
		{
			Solver.ResetToRestPose(PendingResetPose);
			bResetRequested.store(false, std::memory_order_release);
		}

		// Feed animation transforms to fixed particles
		{
			FScopeLock Lock(&AnimTransformCS);
			if (PendingAnimTransforms.Num() > 0)
			{
				Solver.UpdateFixedParticles(PendingAnimTransforms);
			}
		}

		// Feed collider world transforms
		{
			FScopeLock Lock(&ColliderTransformCS);
			if (PendingColliderTransforms.Num() > 0)
			{
				for (int32 i = 0; i < Solver.Colliders.Num() && i < PendingColliderTransforms.Num(); ++i)
				{
					if (Solver.Colliders[i].IsValid())
					{
						Solver.Colliders[i]->WorldTransform = PendingColliderTransforms[i];
					}
				}
			}
		}

		// Run PBD step
		Solver.Step(SimDt);

		// Write results to double buffer
		TArray<FTransform> Results;
		Solver.GetResultTransforms(Results);
		DoubleBuffer.Write(MoveTemp(Results));

		// Calculate alpha for interpolation
		const double Now = FPlatformTime::Seconds();
		const float Elapsed = static_cast<float>(Now - LastSimTime);
		CurrentAlpha = FMath::Clamp(Elapsed / SimDt, 0.f, 1.f);
		LastSimTime = Now;

		// Sleep for remaining time to maintain target Hz
		const double SimElapsed = FPlatformTime::Seconds() - FrameStart;
		const float SleepTime = SimDt - static_cast<float>(SimElapsed);

		if (SleepTime > 0.f)
		{
			FPlatformProcess::Sleep(SleepTime);
		}
	}

	return 0;
}

void FClothSimThread::Exit()
{
	bRunning.store(false, std::memory_order_release);
}

void FClothSimThread::UpdateAnimTransforms(const TArray<FTransform>& Transforms)
{
	FScopeLock Lock(&AnimTransformCS);
	PendingAnimTransforms = Transforms;
}

void FClothSimThread::UpdateColliderTransforms(const TArray<FTransform>& Transforms)
{
	FScopeLock Lock(&ColliderTransformCS);
	PendingColliderTransforms = Transforms;
}

float FClothSimThread::GetInterpolationAlpha() const
{
	return CurrentAlpha;
}

void FClothSimThread::SetTargetHz(float Hz)
{
	SimulationHz = FMath::Clamp(Hz, MinSimulationHz, 120.f);
}

void FClothSimThread::RequestReset(const TArray<FTransform>& RestPose)
{
	PendingResetPose = RestPose;
	bResetRequested.store(true, std::memory_order_release);
}
