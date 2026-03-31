// MagicaClothUE/Private/Simulation/SimulationManager.cpp
#include "Simulation/SimulationManager.h"
#include "HAL/PlatformProcess.h"
#include "Colliders/MagicaColliderShapes.h"

const TArray<FTransform> FMagicaSimulationManager::EmptyTransforms;

FMagicaSimulationManager::FMagicaSimulationManager() = default;

FMagicaSimulationManager::~FMagicaSimulationManager()
{
	Stop();
}

void FMagicaSimulationManager::Start()
{
	if (bRunning.load()) return;
	bRunning.store(true, std::memory_order_release);
	LastSimTime = FPlatformTime::Seconds();
	Thread = FRunnableThread::Create(this, TEXT("MagicaClothSimulation"), 0, TPri_Normal);
}

void FMagicaSimulationManager::Stop()
{
	if (!Thread) return;
	bRunning.store(false, std::memory_order_release);
	FRunnableThread* ThreadToDelete = Thread;
	Thread = nullptr;  // Prevent re-entrant call from FRunnableThread destructor
	ThreadToDelete->WaitForCompletion();
	delete ThreadToDelete;
}

bool FMagicaSimulationManager::Init()
{
	return true;
}

uint32 FMagicaSimulationManager::Run()
{
	const float SimDt = 1.0f / TargetHz;

	while (bRunning.load(std::memory_order_acquire))
	{
		const double FrameStart = FPlatformTime::Seconds();

		{
			FScopeLock Lock(&TeamsCS);

			for (auto& Pair : Teams)
			{
				FTeamData& Team = *Pair.Value;

				// Handle reset
				if (Team.bResetRequested)
				{
					Team.Solver.Reset(Team.PendingResetPositions);
					Team.bResetRequested = false;
					continue;
				}

				// Feed animation positions
				if (Team.PendingAnimPositions.Num() > 0)
				{
					Team.Solver.UpdateFixedParticles(Team.PendingAnimPositions);
				}

				// Update collider transforms
				const int32 ColliderCount = FMath::Min(
					Team.PendingColliderTransforms.Num(),
					Team.Solver.Colliders.Num());
				for (int32 c = 0; c < ColliderCount; ++c)
				{
					Team.Solver.Colliders[c]->WorldTransform = Team.PendingColliderTransforms[c];
				}

				// Step simulation
				Team.Solver.Step(SimDt);

				// Write results to double buffer
				// Convert positions to transforms for bone override
				TArray<FTransform> Results;
				const auto& Positions = Team.Solver.GetPositions();
				Results.SetNum(Positions.Num());
				for (int32 i = 0; i < Positions.Num(); ++i)
				{
					Results[i].SetTranslation(Positions[i]);
					Results[i].SetRotation(FQuat::Identity);
					Results[i].SetScale3D(FVector::OneVector);
				}
				Team.DoubleBuffer.Write(Results);
			}
		}

		// Timing
		const double FrameEnd = FPlatformTime::Seconds();
		const double Elapsed = FrameEnd - FrameStart;
		const double SleepTime = SimDt - Elapsed;
		CurrentAlpha = static_cast<float>(Elapsed / SimDt);

		if (SleepTime > 0.0)
		{
			FPlatformProcess::Sleep(static_cast<float>(SleepTime));
		}
	}

	return 0;
}

void FMagicaSimulationManager::Exit()
{
}

FMagicaTeamId FMagicaSimulationManager::RegisterTeam()
{
	FScopeLock Lock(&TeamsCS);
	const FMagicaTeamId Id = NextTeamId++;
	auto TeamData = MakeUnique<FTeamData>();
	TeamData->TeamId = Id;
	Teams.Add(Id, MoveTemp(TeamData));
	return Id;
}

void FMagicaSimulationManager::UnregisterTeam(FMagicaTeamId TeamId)
{
	FScopeLock Lock(&TeamsCS);
	Teams.Remove(TeamId);
}

FMagicaSimulationManager::FTeamData* FMagicaSimulationManager::GetTeam(FMagicaTeamId TeamId)
{
	FScopeLock Lock(&TeamsCS);
	auto* Found = Teams.Find(TeamId);
	return Found ? Found->Get() : nullptr;
}

void FMagicaSimulationManager::UpdateAnimPositions(FMagicaTeamId TeamId, const TArray<FVector>& Positions)
{
	FScopeLock Lock(&TeamsCS);
	if (auto* Found = Teams.Find(TeamId))
	{
		(*Found)->PendingAnimPositions = Positions;
	}
}

void FMagicaSimulationManager::UpdateColliderTransforms(FMagicaTeamId TeamId, const TArray<FTransform>& Transforms)
{
	FScopeLock Lock(&TeamsCS);
	if (auto* Found = Teams.Find(TeamId))
	{
		(*Found)->PendingColliderTransforms = Transforms;
	}
}

void FMagicaSimulationManager::RequestReset(FMagicaTeamId TeamId, const TArray<FVector>& RestPositions)
{
	FScopeLock Lock(&TeamsCS);
	if (auto* Found = Teams.Find(TeamId))
	{
		(*Found)->bResetRequested = true;
		(*Found)->PendingResetPositions = RestPositions;
	}
}

const TArray<FTransform>& FMagicaSimulationManager::ReadResults(FMagicaTeamId TeamId) const
{
	// No lock needed — DoubleBuffer.Read() is lock-free
	const auto* Found = Teams.Find(TeamId);
	if (Found)
	{
		return (*Found)->DoubleBuffer.Read();
	}
	return EmptyTransforms;
}

float FMagicaSimulationManager::GetInterpolationAlpha() const
{
	return FMath::Clamp(CurrentAlpha, 0.0f, 1.0f);
}

void FMagicaSimulationManager::SetTargetHz(float Hz)
{
	TargetHz = FMath::Clamp(Hz, MinHz, 120.0f);
}
