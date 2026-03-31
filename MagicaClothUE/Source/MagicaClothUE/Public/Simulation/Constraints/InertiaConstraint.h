// MagicaClothUE/Public/Simulation/Constraints/InertiaConstraint.h
#pragma once

#include "Simulation/Constraints/ConstraintBase.h"
#include "Core/ClothTypes.h"

/**
 * Inertia Constraint: stabilizes cloth when character moves/rotates.
 * Handles teleport detection and world/local inertia damping.
 */
struct MAGICACLOTHUE_API FMagicaInertiaConstraint : public FMagicaConstraintBase
{
	FMagicaInertiaParams Params;

	// Updated each frame by the AnimNode
	FVector PrevOrigin = FVector::ZeroVector;
	FQuat PrevRotation = FQuat::Identity;
	FVector CurrentOrigin = FVector::ZeroVector;
	FQuat CurrentRotation = FQuat::Identity;
	bool bFirstFrame = true;
	bool bTeleported = false;

	/** Call each frame before solving to update origin tracking. */
	void UpdateOrigin(const FVector& NewOrigin, const FQuat& NewRotation);

	/** Check if teleport occurred. If so, returns true (caller should reset sim). */
	bool CheckTeleport() const { return bTeleported; }

	virtual void Solve(FMagicaParticleArrays& Particles) override;
};
