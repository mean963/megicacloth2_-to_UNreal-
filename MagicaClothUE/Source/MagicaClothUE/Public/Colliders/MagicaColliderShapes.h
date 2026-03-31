#pragma once

#include "CoreMinimal.h"
#include "Math/Vector.h"
#include "Math/Transform.h"
#include "Math/UnrealMathUtility.h"

/** Collider shape type identifier (for debug drawing and type-safe casting). */
enum class EMagicaColliderShapeType : uint8
{
	Sphere,
	Capsule,
	Box
};

/**
 * Base class for all Magica collider shapes (pure data, no UObject).
 */
struct MAGICACLOTHUE_API FMagicaColliderShape
{
	FTransform WorldTransform;
	float Friction = 0.1f;
	EMagicaColliderShapeType ShapeType = EMagicaColliderShapeType::Sphere;

	virtual ~FMagicaColliderShape() = default;
	virtual void ResolveCollision(FVector& InOutPosition) const = 0;
};

/** Sphere collider — pushes particles outside the sphere volume. */
struct MAGICACLOTHUE_API FMagicaSphereCollider : public FMagicaColliderShape
{
	float Radius = 5.f;

	FMagicaSphereCollider()
	{
		ShapeType = EMagicaColliderShapeType::Sphere;
	}

	explicit FMagicaSphereCollider(float InRadius) : Radius(InRadius)
	{
		ShapeType = EMagicaColliderShapeType::Sphere;
	}

	virtual void ResolveCollision(FVector& InOutPosition) const override
	{
		const FVector Center = WorldTransform.GetTranslation();
		const FVector Delta  = InOutPosition - Center;
		const float DistSq   = Delta.SizeSquared();
		const float R         = Radius * WorldTransform.GetScale3D().GetMax();

		if (DistSq < R * R && DistSq > UE_SMALL_NUMBER)
		{
			const float Dist = FMath::Sqrt(DistSq);
			const FVector Dir = Delta / Dist;
			const FVector ResolvedPos = Center + Dir * R;
			InOutPosition = FMath::Lerp(ResolvedPos, InOutPosition, Friction);
		}
	}
};

/** Capsule collider — line segment + radius, commonly used for limbs. */
struct MAGICACLOTHUE_API FMagicaCapsuleCollider : public FMagicaColliderShape
{
	float Radius     = 5.f;
	float HalfHeight = 10.f;

	FMagicaCapsuleCollider()
	{
		ShapeType = EMagicaColliderShapeType::Capsule;
	}

	FMagicaCapsuleCollider(float InRadius, float InHalfHeight)
		: Radius(InRadius), HalfHeight(InHalfHeight)
	{
		ShapeType = EMagicaColliderShapeType::Capsule;
	}

	virtual void ResolveCollision(FVector& InOutPosition) const override
	{
		const float Scale            = WorldTransform.GetScale3D().GetMax();
		const float ScaledRadius     = Radius * Scale;
		const float ScaledHalfHeight = HalfHeight * Scale;

		const FVector Center = WorldTransform.GetTranslation();
		const FVector UpAxis = WorldTransform.GetRotation().GetUpVector();

		const FVector PointA = Center + UpAxis * ScaledHalfHeight;
		const FVector PointB = Center - UpAxis * ScaledHalfHeight;

		const FVector AB     = PointB - PointA;
		const float ABLenSq  = AB.SizeSquared();
		float T = 0.f;

		if (ABLenSq > UE_SMALL_NUMBER)
		{
			T = FMath::Clamp(FVector::DotProduct(InOutPosition - PointA, AB) / ABLenSq, 0.f, 1.f);
		}

		const FVector ClosestOnLine = PointA + AB * T;
		const FVector Delta  = InOutPosition - ClosestOnLine;
		const float DistSq   = Delta.SizeSquared();

		if (DistSq < ScaledRadius * ScaledRadius && DistSq > UE_SMALL_NUMBER)
		{
			const float Dist = FMath::Sqrt(DistSq);
			const FVector Dir = Delta / Dist;
			InOutPosition = ClosestOnLine + Dir * ScaledRadius;
		}
	}
};

/** Box collider — AABB in local space, pushes out on closest face. */
struct MAGICACLOTHUE_API FMagicaBoxCollider : public FMagicaColliderShape
{
	FVector HalfExtent = FVector(5.0);

	FMagicaBoxCollider()
	{
		ShapeType = EMagicaColliderShapeType::Box;
	}

	explicit FMagicaBoxCollider(const FVector& InHalfExtent) : HalfExtent(InHalfExtent)
	{
		ShapeType = EMagicaColliderShapeType::Box;
	}

	virtual void ResolveCollision(FVector& InOutPosition) const override
	{
		FVector LocalPos = WorldTransform.InverseTransformPosition(InOutPosition);
		const FVector ScaledExtent = HalfExtent * WorldTransform.GetScale3D();

		if (FMath::Abs(LocalPos.X) < ScaledExtent.X &&
			FMath::Abs(LocalPos.Y) < ScaledExtent.Y &&
			FMath::Abs(LocalPos.Z) < ScaledExtent.Z)
		{
			const FVector Distances(
				ScaledExtent.X - FMath::Abs(LocalPos.X),
				ScaledExtent.Y - FMath::Abs(LocalPos.Y),
				ScaledExtent.Z - FMath::Abs(LocalPos.Z)
			);

			if (Distances.X <= Distances.Y && Distances.X <= Distances.Z)
			{
				LocalPos.X = FMath::Sign(LocalPos.X) * ScaledExtent.X;
			}
			else if (Distances.Y <= Distances.Z)
			{
				LocalPos.Y = FMath::Sign(LocalPos.Y) * ScaledExtent.Y;
			}
			else
			{
				LocalPos.Z = FMath::Sign(LocalPos.Z) * ScaledExtent.Z;
			}

			InOutPosition = WorldTransform.TransformPosition(LocalPos);
		}
	}
};
