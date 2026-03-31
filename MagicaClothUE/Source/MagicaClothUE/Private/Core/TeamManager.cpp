// MagicaClothUE/Private/Core/TeamManager.cpp
#include "Core/TeamManager.h"
#include "Simulation/SimulationManager.h"

void UMagicaTeamManager::Initialize(FMagicaSimulationManager* InSimManager)
{
	SimManager = InSimManager;
}

FMagicaTeamId UMagicaTeamManager::CreateTeam()
{
	if (!SimManager) return MAGICA_INVALID_TEAM_ID;

	const FMagicaTeamId Id = SimManager->RegisterTeam();
	ActiveTeamIds.Add(Id);
	return Id;
}

void UMagicaTeamManager::DestroyTeam(FMagicaTeamId TeamId)
{
	if (!SimManager) return;

	SimManager->UnregisterTeam(TeamId);
	ActiveTeamIds.Remove(TeamId);
}
