#pragma once

#include "CoreMinimal.h"
#include "Containers/Array.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/CriticalSection.h"
#include "Math/Transform.h"
#include "Simulation/FPBDSolver.h"
#include "Simulation/FClothDoubleBuffer.h"

/**
 * Dedicated worker thread for cloth physics simulation.
 * Runs PBD solver at a fixed frequency (default 90Hz), fully decoupled from the game thread.
 * Results are passed via a lock-free double buffer — no mutex contention on read.
 *
 * UE 5.6: Uses FRunnable (still the canonical way for long-lived worker threads).
 */
class MAGICACLOTHUE_API FClothSimThread : public FRunnable
{
public:
	FClothSimThread();
	virtual ~FClothSimThread() override;

	void Start();
	void Stop();

	bool IsRunning() const { return bRunning.load(std::memory_order_acquire); }

	// --- FRunnable interface ---
	virtual bool Init() override;
	virtual uint32 Run() override;
	virtual void Exit() override;

	// --- Public State ---
	FPBDSolver Solver;
	FClothDoubleBuffer DoubleBuffer;

	/** Feed new animation transforms to the solver (called from game thread). */
	void UpdateAnimTransforms(const TArray<FTransform>& Transforms);

	/** Feed new collider world transforms (called from game thread). */
	void UpdateColliderTransforms(const TArray<FTransform>& Transforms);

	/** Get interpolation alpha for smooth blending between sim frames. */
	float GetInterpolationAlpha() const;

	// --- Configuration ---
	float SimulationHz     = 90.f;
	float MinSimulationHz  = 30.f;
	int32 MaxStepsPerFrame = 3;

	void SetTargetHz(float Hz);
	void RequestReset(const TArray<FTransform>& RestPose);

private:
	FRunnableThread* Thread = nullptr;
	std::atomic<bool> bRunning{false};

	TArray<FTransform> PendingAnimTransforms;
	FCriticalSection AnimTransformCS;

	TArray<FTransform> PendingColliderTransforms;
	FCriticalSection ColliderTransformCS;

	TArray<FTransform> PendingResetPose;
	std::atomic<bool> bResetRequested{false};

	double LastSimTime  = 0.0;
	float CurrentAlpha  = 0.f;
};
