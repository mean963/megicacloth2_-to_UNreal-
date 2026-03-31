// MagicaClothUE/Public/Simulation/SimulationManager.h
#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/CriticalSection.h"
#include "Core/ClothTypes.h"
#include "Simulation/FPBDSolver.h"
#include "Simulation/FClothDoubleBuffer.h"

/**
 * Central simulation manager: single worker thread for ALL cloth Teams.
 * Replaces per-instance FClothSimThread.
 */
class MAGICACLOTHUE_API FMagicaSimulationManager : public FRunnable
{
public:
	FMagicaSimulationManager();
	virtual ~FMagicaSimulationManager() override;

	void Start();
	void Stop();
	bool IsRunning() const { return bRunning.load(std::memory_order_acquire); }

	// --- FRunnable ---
	virtual bool Init() override;
	virtual uint32 Run() override;
	virtual void Exit() override;

	// --- Team Management ---
	struct FTeamData
	{
		FMagicaTeamId TeamId = MAGICA_INVALID_TEAM_ID;
		FMagicaPBDSolver Solver;
		FClothDoubleBuffer DoubleBuffer;

		// Pending input (written by game thread, read by sim thread)
		TArray<FVector> PendingAnimPositions;
		TArray<FTransform> PendingColliderTransforms;
		bool bResetRequested = false;
		TArray<FVector> PendingResetPositions;
	};

	/** Register a new Team. Returns TeamId. Caller must configure Solver before calling Start(). */
	FMagicaTeamId RegisterTeam();

	/** Unregister a Team. */
	void UnregisterTeam(FMagicaTeamId TeamId);

	/** Get Team data (for configuration). NOT thread-safe — call before Start() or use locks. */
	FTeamData* GetTeam(FMagicaTeamId TeamId);

	/** Feed animation positions to a Team (game thread). */
	void UpdateAnimPositions(FMagicaTeamId TeamId, const TArray<FVector>& Positions);

	/** Feed collider transforms to a Team (game thread). */
	void UpdateColliderTransforms(FMagicaTeamId TeamId, const TArray<FTransform>& Transforms);

	/** Request simulation reset for a Team (game thread). */
	void RequestReset(FMagicaTeamId TeamId, const TArray<FVector>& RestPositions);

	/** Read results from a Team's double buffer (game thread). */
	const TArray<FTransform>& ReadResults(FMagicaTeamId TeamId) const;

	/** Get interpolation alpha for smooth blending. */
	float GetInterpolationAlpha() const;

	// --- Configuration ---
	float TargetHz = 90.0f;
	float MinHz = 30.0f;
	int32 MaxStepsPerFrame = 3;

	void SetTargetHz(float Hz);

private:
	FRunnableThread* Thread = nullptr;
	std::atomic<bool> bRunning{false};

	TMap<FMagicaTeamId, TUniquePtr<FTeamData>> Teams;
	FCriticalSection TeamsCS;

	int32 NextTeamId = 0;
	double LastSimTime = 0.0;
	float CurrentAlpha = 0.0f;

	// Empty buffer for invalid reads
	static const TArray<FTransform> EmptyTransforms;
};
