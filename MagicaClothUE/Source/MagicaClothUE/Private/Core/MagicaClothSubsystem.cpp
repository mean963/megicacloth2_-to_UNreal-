// MagicaClothUE/Private/Core/MagicaClothSubsystem.cpp
#include "Core/MagicaClothSubsystem.h"
#include "Core/TeamManager.h"
#include "Simulation/SimulationManager.h"

void UMagicaClothSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	SimulationManager = MakeUnique<FMagicaSimulationManager>();

	TeamManager = NewObject<UMagicaTeamManager>(this);
	TeamManager->Initialize(SimulationManager.Get());
}

void UMagicaClothSubsystem::Deinitialize()
{
	if (SimulationManager)
	{
		SimulationManager->Stop();
	}

	Super::Deinitialize();
}

void UMagicaClothSubsystem::EnsureSimulationRunning()
{
	if (!bSimulationStarted && SimulationManager)
	{
		SimulationManager->Start();
		bSimulationStarted = true;
	}
}
