#pragma once

#include "CoreMinimal.h"
#include "Math/Vector.h"
#include "Templates/SharedPointer.h"
#include "Components/SceneComponent.h"
#include "Colliders/MagicaColliderShapes.h"
#include "MagicaColliderComponent.generated.h"

UENUM(BlueprintType)
enum class EMagicaColliderType : uint8
{
	Sphere,
	Capsule,
	Box
};

/**
 * Component that defines a collision shape for Magica Cloth simulation.
 * Attach to any bone or actor to create collision boundaries.
 */
UCLASS(ClassGroup = "MagicaCloth", meta = (BlueprintSpawnableComponent))
class MAGICACLOTHUE_API UMagicaColliderComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	UMagicaColliderComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Collider")
	EMagicaColliderType ColliderType = EMagicaColliderType::Sphere;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Collider", meta = (ClampMin = "0.1"))
	float Radius = 5.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Collider",
		meta = (EditCondition = "ColliderType == EMagicaColliderType::Capsule", ClampMin = "0.1"))
	float HalfHeight = 10.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Collider",
		meta = (EditCondition = "ColliderType == EMagicaColliderType::Box"))
	FVector BoxHalfExtent = FVector(5.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Magica Collider",
		meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Friction = 0.1f;

	/** Create the runtime collider shape from current settings. */
	TSharedPtr<FMagicaColliderShape> CreateColliderShape() const;

	/** Update the world transform on an existing shape. */
	void UpdateShapeTransform(TSharedPtr<FMagicaColliderShape>& Shape) const;
};
