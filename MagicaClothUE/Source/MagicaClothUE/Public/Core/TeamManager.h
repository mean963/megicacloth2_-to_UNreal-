// MagicaClothUE/Public/Core/TeamManager.h
#pragma once

#include "CoreMinimal.h"
#include "Core/ClothTypes.h"
#include "TeamManager.generated.h"

class FMagicaSimulationManager;

/**
 * Manages cloth Team registration and lifecycle.
 * Thin wrapper around SimulationManager for team bookkeeping.
 */
UCLASS()
class MAGICACLOTHUE_API UMagicaTeamManager : public UObject
{
	GENERATED_BODY()

public:
	void Initialize(FMagicaSimulationManager* InSimManager);

	FMagicaTeamId CreateTeam();
	void DestroyTeam(FMagicaTeamId TeamId);
	int32 GetActiveTeamCount() const { return ActiveTeamIds.Num(); }

private:
	FMagicaSimulationManager* SimManager = nullptr;
	TSet<FMagicaTeamId> ActiveTeamIds;
};
