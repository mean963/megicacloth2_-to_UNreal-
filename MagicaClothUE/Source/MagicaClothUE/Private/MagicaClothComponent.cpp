#include "MagicaClothComponent.h"
#include "Engine/World.h"
#include "Colliders/MagicaColliderComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Core/MagicaClothSubsystem.h"
#include "Core/TeamManager.h"
#include "Simulation/SimulationManager.h"

UMagicaClothComponent::UMagicaClothComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UMagicaClothComponent::BeginPlay()
{
	Super::BeginPlay();
	if (UWorld* World = GetWorld())
	{
		if (UMagicaClothSubsystem* Subsystem = World->GetSubsystem<UMagicaClothSubsystem>())
		{
			TeamId = Subsystem->GetTeamManager()->CreateTeam();
			Subsystem->EnsureSimulationRunning();
		}
	}
}

void UMagicaClothComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (TeamId != MAGICA_INVALID_TEAM_ID)
	{
		if (UWorld* World = GetWorld())
		{
			if (UMagicaClothSubsystem* Subsystem = World->GetSubsystem<UMagicaClothSubsystem>())
			{
				Subsystem->GetTeamManager()->DestroyTeam(TeamId);
			}
		}
		TeamId = MAGICA_INVALID_TEAM_ID;
	}

	Super::EndPlay(EndPlayReason);
}

void UMagicaClothComponent::TickComponent(
	float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (TeamId == MAGICA_INVALID_TEAM_ID)
	{
		return;
	}

	UpdateColliderTransforms();
}

void UMagicaClothComponent::ResetSimulation()
{
	if (TeamId != MAGICA_INVALID_TEAM_ID)
	{
		if (UWorld* World = GetWorld())
		{
			if (UMagicaClothSubsystem* Subsystem = World->GetSubsystem<UMagicaClothSubsystem>())
			{
				TArray<FVector> EmptyPose;
				Subsystem->GetSimulationManager()->RequestReset(TeamId, EmptyPose);
			}
		}
	}
}

void UMagicaClothComponent::SetSimulationHz(float NewHz)
{
	SimulationHz = FMath::Clamp(NewHz, MinSimulationHz, 120.f);
	if (UWorld* World = GetWorld())
	{
		if (UMagicaClothSubsystem* Subsystem = World->GetSubsystem<UMagicaClothSubsystem>())
		{
			Subsystem->GetSimulationManager()->SetTargetHz(SimulationHz);
		}
	}
}

bool UMagicaClothComponent::IsSimulationRunning() const
{
	return TeamId != MAGICA_INVALID_TEAM_ID;
}

void UMagicaClothComponent::UpdateColliderTransforms()
{
	if (TeamId == MAGICA_INVALID_TEAM_ID)
	{
		return;
	}

	TArray<FTransform> Transforms;
	Transforms.Reserve(Colliders.Num());
	for (const auto& Collider : Colliders)
	{
		if (Collider)
		{
			Transforms.Add(Collider->GetComponentTransform());
		}
	}

	if (Transforms.Num() > 0)
	{
		if (UWorld* World = GetWorld())
		{
			if (UMagicaClothSubsystem* Subsystem = World->GetSubsystem<UMagicaClothSubsystem>())
			{
				Subsystem->GetSimulationManager()->UpdateColliderTransforms(TeamId, Transforms);
			}
		}
	}
}
