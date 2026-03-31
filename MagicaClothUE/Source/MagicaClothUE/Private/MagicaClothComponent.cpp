#include "MagicaClothComponent.h"
#include "Simulation/FClothSimThread.h"
#include "Colliders/MagicaColliderComponent.h"
#include "Math/UnrealMathUtility.h"

UMagicaClothComponent::UMagicaClothComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UMagicaClothComponent::BeginPlay()
{
	Super::BeginPlay();
}

void UMagicaClothComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (SimThread.IsValid())
	{
		SimThread->Stop();
		SimThread.Reset();
	}

	Super::EndPlay(EndPlayReason);
}

void UMagicaClothComponent::TickComponent(
	float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!SimThread.IsValid())
	{
		return;
	}

	UpdateColliderTransforms();

	// Adaptive Hz: simple CPU-load heuristic
	if (bAdaptiveHz)
	{
		const float CurrentHz = SimThread->SimulationHz;
		const float FrameBudget = 1.f / CurrentHz;

		if (DeltaTime > FrameBudget * 1.5f && CurrentHz > MinSimulationHz)
		{
			SimThread->SetTargetHz(FMath::Max(CurrentHz - 10.f, MinSimulationHz));
		}
		else if (DeltaTime < FrameBudget * 0.8f && CurrentHz < SimulationHz)
		{
			SimThread->SetTargetHz(FMath::Min(CurrentHz + 5.f, SimulationHz));
		}
	}
}

void UMagicaClothComponent::ResetSimulation()
{
	if (SimThread.IsValid())
	{
		TArray<FTransform> EmptyPose;
		SimThread->RequestReset(EmptyPose);
	}
}

void UMagicaClothComponent::SetSimulationHz(float NewHz)
{
	SimulationHz = FMath::Clamp(NewHz, MinSimulationHz, 120.f);
	if (SimThread.IsValid())
	{
		SimThread->SetTargetHz(SimulationHz);
	}
}

bool UMagicaClothComponent::IsSimulationRunning() const
{
	return SimThread.IsValid() && SimThread->IsRunning();
}

void UMagicaClothComponent::UpdateColliderTransforms()
{
	if (!SimThread.IsValid())
	{
		return;
	}

	for (int32 i = 0; i < Colliders.Num(); ++i)
	{
		if (Colliders[i] && SimThread->Solver.Colliders.IsValidIndex(i))
		{
			Colliders[i]->UpdateShapeTransform(SimThread->Solver.Colliders[i]);
		}
	}
}
