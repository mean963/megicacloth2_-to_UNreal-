// MagicaClothUE/Private/Simulation/Constraints/InertiaConstraint.cpp
#include "Simulation/Constraints/InertiaConstraint.h"

void FMagicaInertiaConstraint::UpdateOrigin(const FVector& NewOrigin, const FQuat& NewRotation)
{
	PrevOrigin = CurrentOrigin;
	PrevRotation = CurrentRotation;
	CurrentOrigin = NewOrigin;
	CurrentRotation = NewRotation;

	if (bFirstFrame)
	{
		PrevOrigin = NewOrigin;
		PrevRotation = NewRotation;
		bFirstFrame = false;
		bTeleported = false;
		return;
	}

	// Teleport detection
	const float MoveDist = FVector::Dist(CurrentOrigin, PrevOrigin);
	const float RotAngle = FMath::RadiansToDegrees(CurrentRotation.AngularDistance(PrevRotation));
	bTeleported = (MoveDist > Params.TeleportDistanceThreshold) ||
	              (RotAngle > Params.TeleportRotationThreshold);
}

void FMagicaInertiaConstraint::Solve(FMagicaParticleArrays& Particles)
{
	if (bFirstFrame || bTeleported) return;

	// World Inertia: apply character movement shift to all particles
	const FVector WorldShift = CurrentOrigin - PrevOrigin;
	if (!WorldShift.IsNearlyZero() && Params.WorldInertia > 0.0f)
	{
		const FVector InertiaShift = WorldShift * Params.WorldInertia;
		for (int32 i = 0; i < Particles.PredictedPositions.Num(); ++i)
		{
			if (Particles.InvMasses[i] > 0.0f)
			{
				// Reduce shift based on depth (deeper = less inertia compensation)
				const float DepthFactor = 1.0f - Particles.Depths[i] * 0.5f;
				Particles.PredictedPositions[i] += InertiaShift * DepthFactor;
			}
		}
	}

	// Local Inertia: apply rotation damping
	const FQuat DeltaRot = CurrentRotation * PrevRotation.Inverse();
	if (!DeltaRot.IsIdentity() && Params.LocalInertia > 0.0f)
	{
		for (int32 i = 0; i < Particles.PredictedPositions.Num(); ++i)
		{
			if (Particles.InvMasses[i] > 0.0f)
			{
				const FVector RelPos = Particles.PredictedPositions[i] - CurrentOrigin;
				const FVector RotatedPos = DeltaRot.RotateVector(RelPos);
				const FVector RotShift = (RotatedPos - RelPos) * Params.LocalInertia;
				const float DepthFactor = 1.0f - Particles.Depths[i] * 0.5f;
				Particles.PredictedPositions[i] += RotShift * DepthFactor;
			}
		}
	}
}
