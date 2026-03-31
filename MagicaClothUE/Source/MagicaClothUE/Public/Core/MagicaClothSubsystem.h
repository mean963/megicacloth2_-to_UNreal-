// MagicaClothUE/Public/Core/MagicaClothSubsystem.h
#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Core/ClothTypes.h"
#include "MagicaClothSubsystem.generated.h"

class FMagicaSimulationManager;
class UMagicaTeamManager;

/**
 * World Subsystem: one per world, coordinates all cloth simulation.
 * Owns the SimulationManager (worker thread) and TeamManager.
 */
UCLASS()
class MAGICACLOTHUE_API UMagicaClothSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	FMagicaSimulationManager* GetSimulationManager() const { return SimulationManager.Get(); }
	UMagicaTeamManager* GetTeamManager() const { return TeamManager; }

	/** Start the simulation thread (called when first Team is registered). */
	void EnsureSimulationRunning();

private:
	TUniquePtr<FMagicaSimulationManager> SimulationManager;

	UPROPERTY()
	TObjectPtr<UMagicaTeamManager> TeamManager;

	bool bSimulationStarted = false;
};
