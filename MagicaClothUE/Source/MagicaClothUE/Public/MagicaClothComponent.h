#pragma once

#include "CoreMinimal.h"
#include "Containers/Array.h"
#include "Math/Vector.h"
#include "Templates/SharedPointer.h"
#include "Components/ActorComponent.h"
#include "Core/ClothTypes.h"
#include "MagicaClothComponent.generated.h"

class FClothSimThread;
class UMagicaColliderComponent;

/** Simulation update mode — matches Magica Cloth 2's UpdateMode concept. */
UENUM(BlueprintType)
enum class EMagicaUpdateMode : uint8
{
	/** Normal: simulation tied to game time scale. */
	Normal,
	/** Unscaled: ignores time dilation. */
	UnscaledTime,
	/** Fixed Hz: always runs at target Hz regardless of framerate. */
	FixedHz
};

/**
 * High-level component that manages cloth simulation for a SkeletalMesh.
 * Provides a Blueprint-friendly interface for configuring simulation parameters,
 * attaching colliders, and controlling simulation state.
 *
 * For AnimBP integration, use FAnimNode_MagicaCloth instead.
 * This component is for standalone/programmatic usage.
 */
UCLASS(ClassGroup = "MagicaCloth", meta = (BlueprintSpawnableComponent))
class MAGICACLOTHUE_API UMagicaClothComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UMagicaClothComponent();

	// ── Simulation Parameters ───────────────────────────

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Physics",
		meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Stiffness = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Physics",
		meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BendStiffness = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Physics",
		meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Damping = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Physics")
	FVector Gravity = FVector(0.0, 0.0, -980.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Physics",
		meta = (ClampMin = "0.0"))
	float MaxVelocity = 5000.f;

	// ── Performance ─────────────────────────────────────

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Performance")
	EMagicaUpdateMode UpdateMode = EMagicaUpdateMode::FixedHz;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Performance",
		meta = (ClampMin = "30.0", ClampMax = "120.0"))
	float SimulationHz = 90.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Performance",
		meta = (ClampMin = "1", ClampMax = "8"))
	int32 SolverIterations = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Performance",
		meta = (ClampMin = "1", ClampMax = "5"))
	int32 MaxStepsPerFrame = 3;

	// ── Colliders ───────────────────────────────────────

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Collision")
	TArray<TObjectPtr<UMagicaColliderComponent>> Colliders;

	// ── Broadcast Safety ────────────────────────────────

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Broadcast Safety")
	bool bAdaptiveHz = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Cloth|Broadcast Safety",
		meta = (ClampMin = "15.0", EditCondition = "bAdaptiveHz"))
	float MinSimulationHz = 30.f;

	// ── Blueprint Functions ─────────────────────────────

	UFUNCTION(BlueprintCallable, Category = "Magica Cloth")
	void ResetSimulation();

	UFUNCTION(BlueprintCallable, Category = "Magica Cloth")
	void SetSimulationHz(float NewHz);

	UFUNCTION(BlueprintCallable, Category = "Magica Cloth")
	bool IsSimulationRunning() const;

	// ── UActorComponent ─────────────────────────────────

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

private:
	TSharedPtr<FClothSimThread> SimThread;

	FMagicaTeamId TeamId = MAGICA_INVALID_TEAM_ID;

	void UpdateColliderTransforms();
};
