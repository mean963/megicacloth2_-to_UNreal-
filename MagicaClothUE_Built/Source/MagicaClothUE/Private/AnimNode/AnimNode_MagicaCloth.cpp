#include "AnimNode/AnimNode_MagicaCloth.h"
#include "Simulation/FClothSimThread.h"
#include "Simulation/FClothConstraints.h"
#include "Simulation/FPBDSolver.h"
#include "Colliders/MagicaColliderShapes.h"
#include "Animation/AnimInstanceProxy.h"
#include "Animation/AnimTypes.h"
#include "ReferenceSkeleton.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "DrawDebugHelpers.h"

// ── FAnimNode_SkeletalControlBase interface ─────────────

void FAnimNode_MagicaCloth::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	RootBone.Initialize(RequiredBones);
	EndBone.Initialize(RequiredBones);
	for (FBoneReference& BoneRef : ColliderBoneFilter)
	{
		BoneRef.Initialize(RequiredBones);
	}
	for (FBoneReference& BoneRef : ExcludeChainBones)
	{
		BoneRef.Initialize(RequiredBones);
	}
	bNeedsRebuild = true;
}

void FAnimNode_MagicaCloth::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);
	ShutdownSimulation();
	bInitialized = false;
	bNeedsRebuild = true;
}

void FAnimNode_MagicaCloth::UpdateInternal(const FAnimationUpdateContext& Context)
{
	FAnimNode_SkeletalControlBase::UpdateInternal(Context);
}

bool FAnimNode_MagicaCloth::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return RootBone.IsValidToEvaluate(RequiredBones);
}

// ── Single-Chain Bone Discovery ─────────────────────────

void FAnimNode_MagicaCloth::BuildBoneChain(const FBoneContainer& RequiredBones)
{
	BoneChainIndices.Empty();
	BoneChainSkeletonIndices.Empty();

	if (!RootBone.IsValidToEvaluate(RequiredBones))
	{
		return;
	}

	const FReferenceSkeleton& RefSkeleton = RequiredBones.GetReferenceSkeleton();
	const int32 RootIndex = RootBone.BoneIndex;
	int32 CurrentIndex = RootIndex;
	const int32 EndIndex = EndBone.IsValidToEvaluate(RequiredBones) ? EndBone.BoneIndex : INDEX_NONE;

	TArray<int32> Chain;
	Chain.Add(CurrentIndex);

	constexpr int32 MaxChainLength = 64;

	if (EndIndex != INDEX_NONE)
	{
		TArray<int32> PathToRoot;
		int32 Idx = EndIndex;
		while (Idx != INDEX_NONE && Idx != RootIndex)
		{
			PathToRoot.Add(Idx);
			Idx = RefSkeleton.GetParentIndex(Idx);
		}
		if (Idx == RootIndex)
		{
			for (int32 i = PathToRoot.Num() - 1; i >= 0; --i)
			{
				Chain.Add(PathToRoot[i]);
			}
		}
	}
	else
	{
		for (int32 Step = 0; Step < MaxChainLength; ++Step)
		{
			int32 ChildIndex = INDEX_NONE;
			const int32 BoneCount = RefSkeleton.GetNum();
			for (int32 i = 0; i < BoneCount; ++i)
			{
				if (RefSkeleton.GetParentIndex(i) == CurrentIndex)
				{
					ChildIndex = i;
					break;
				}
			}
			if (ChildIndex == INDEX_NONE)
			{
				break;
			}
			Chain.Add(ChildIndex);
			CurrentIndex = ChildIndex;
		}
	}

	for (const int32 SkeletonIdx : Chain)
	{
		const FCompactPoseBoneIndex CompactIdx = RequiredBones.MakeCompactPoseIndex(
			FMeshPoseBoneIndex(SkeletonIdx));
		if (CompactIdx != INDEX_NONE)
		{
			BoneChainIndices.Add(CompactIdx);
			BoneChainSkeletonIndices.Add(SkeletonIdx);
		}
	}

	bNeedsRebuild = false;
}

// ── Multi-Chain (Skirt) Bone Discovery ──────────────────

void FAnimNode_MagicaCloth::BuildMultiChain(const FBoneContainer& RequiredBones)
{
	AllBoneCompactIndices.Empty();
	AllBoneSkeletonIndices.Empty();
	TotalParticleCount = 0;

	if (!RootBone.IsValidToEvaluate(RequiredBones))
	{
		return;
	}

	const FReferenceSkeleton& RefSkeleton = RequiredBones.GetReferenceSkeleton();
	const int32 RootIndex = RootBone.BoneIndex;
	const int32 BoneCount = RefSkeleton.GetNum();

	// Particle 0 = shared root
	const FCompactPoseBoneIndex RootCompact = RequiredBones.MakeCompactPoseIndex(
		FMeshPoseBoneIndex(RootIndex));
	if (RootCompact == INDEX_NONE)
	{
		return;
	}

	AllBoneCompactIndices.Add(RootCompact);
	AllBoneSkeletonIndices.Add(RootIndex);

	// Build exclude set
	TSet<int32> ExcludedBoneIndices;
	for (const FBoneReference& BoneRef : ExcludeChainBones)
	{
		if (BoneRef.BoneIndex != INDEX_NONE)
		{
			ExcludedBoneIndices.Add(BoneRef.BoneIndex);
		}
	}

	// Find all direct children of root (excluding filtered bones)
	TArray<int32> ChildRoots;
	for (int32 i = 0; i < BoneCount; ++i)
	{
		if (RefSkeleton.GetParentIndex(i) == RootIndex && !ExcludedBoneIndices.Contains(i))
		{
			ChildRoots.Add(i);
		}
	}

	// Note: Chain spatial sorting is done in BuildCrossChainMeshConstraints using actual
	// component-space particle positions (not ref pose local coords which don't work).

	// For each child, walk to leaf → one chain
	constexpr int32 MaxChainLength = 64;

	for (const int32 ChildRoot : ChildRoots)
	{
		int32 Current = ChildRoot;
		TArray<int32> ChainBones;
		ChainBones.Add(Current);

		for (int32 Step = 0; Step < MaxChainLength; ++Step)
		{
			int32 NextChild = INDEX_NONE;
			for (int32 i = 0; i < BoneCount; ++i)
			{
				if (RefSkeleton.GetParentIndex(i) == Current)
				{
					NextChild = i;
					break;
				}
			}
			if (NextChild == INDEX_NONE)
			{
				break;
			}
			ChainBones.Add(NextChild);
			Current = NextChild;
		}

		// Append chain bones to flat array
		for (const int32 BoneIdx : ChainBones)
		{
			const FCompactPoseBoneIndex Compact = RequiredBones.MakeCompactPoseIndex(
				FMeshPoseBoneIndex(BoneIdx));
			if (Compact != INDEX_NONE)
			{
				AllBoneCompactIndices.Add(Compact);
				AllBoneSkeletonIndices.Add(BoneIdx);
			}
		}
	}

	TotalParticleCount = AllBoneCompactIndices.Num();
	bNeedsRebuild = false;
}

// ── Physics Asset Collider Extraction ───────────────────

void FAnimNode_MagicaCloth::BuildPhysicsAssetColliders(FComponentSpacePoseContext& Context)
{
	ColliderBoneMappings.Empty();

	if (!bUsePhysicsAssetColliders || !SimThread.IsValid())
	{
		return;
	}

	USkeletalMeshComponent* SkelComp = Context.AnimInstanceProxy->GetSkelMeshComponent();
	if (!SkelComp)
	{
		return;
	}

	UPhysicsAsset* PA = PhysicsAssetOverride ? PhysicsAssetOverride.Get() : SkelComp->GetPhysicsAsset();
	if (!PA)
	{
		return;
	}

	const FBoneContainer& RequiredBones = Context.Pose.GetPose().GetBoneContainer();
	const FReferenceSkeleton& RefSkeleton = RequiredBones.GetReferenceSkeleton();

	// Build allowed bone set from filter
	TSet<FName> AllowedBones;
	const bool bHasFilter = ColliderBoneFilter.Num() > 0;
	if (bHasFilter)
	{
		for (const FBoneReference& BoneRef : ColliderBoneFilter)
		{
			if (BoneRef.BoneIndex != INDEX_NONE)
			{
				AllowedBones.Add(RefSkeleton.GetBoneName(BoneRef.BoneIndex));
			}
		}
	}

	for (USkeletalBodySetup* BodySetup : PA->SkeletalBodySetups)
	{
		if (!BodySetup)
		{
			continue;
		}

		// Filter: if ColliderBoneFilter is set, only include matching bones
		if (bHasFilter && !AllowedBones.Contains(BodySetup->BoneName))
		{
			continue;
		}

		const int32 BoneIdx = RefSkeleton.FindBoneIndex(BodySetup->BoneName);
		if (BoneIdx == INDEX_NONE)
		{
			continue;
		}

		const FCompactPoseBoneIndex CompactIdx = RequiredBones.MakeCompactPoseIndex(
			FMeshPoseBoneIndex(BoneIdx));
		if (CompactIdx == INDEX_NONE)
		{
			continue;
		}

		const FTransform BoneCS = Context.Pose.GetComponentSpaceTransform(CompactIdx);
		const FKAggregateGeom& AggGeom = BodySetup->AggGeom;

		// Sphere bodies
		for (const FKSphereElem& Sphere : AggGeom.SphereElems)
		{
			auto SphereCollider = MakeShared<FMagicaSphereCollider>(Sphere.Radius);
			SphereCollider->Friction = ColliderFriction;

			const FTransform LocalOffset(FQuat::Identity, Sphere.Center, FVector::OneVector);
			SphereCollider->WorldTransform = LocalOffset * BoneCS;

			const int32 ColliderIdx = SimThread->Solver.Colliders.Num();
			SimThread->Solver.Colliders.Add(SphereCollider);

			FColliderBoneMapping Mapping;
			Mapping.CompactBoneIndex = CompactIdx;
			Mapping.ColliderIndex = ColliderIdx;
			Mapping.LocalOffset = LocalOffset;
			ColliderBoneMappings.Add(Mapping);
		}

		// Capsule (Sphyl) bodies
		for (const FKSphylElem& Sphyl : AggGeom.SphylElems)
		{
			auto CapsuleCollider = MakeShared<FMagicaCapsuleCollider>(Sphyl.Radius, Sphyl.Length * 0.5f);
			CapsuleCollider->Friction = ColliderFriction;

			const FTransform LocalOffset(Sphyl.Rotation.Quaternion(), Sphyl.Center, FVector::OneVector);
			CapsuleCollider->WorldTransform = LocalOffset * BoneCS;

			const int32 ColliderIdx = SimThread->Solver.Colliders.Num();
			SimThread->Solver.Colliders.Add(CapsuleCollider);

			FColliderBoneMapping Mapping;
			Mapping.CompactBoneIndex = CompactIdx;
			Mapping.ColliderIndex = ColliderIdx;
			Mapping.LocalOffset = LocalOffset;
			ColliderBoneMappings.Add(Mapping);
		}

		// Tapered capsule bodies (treat as capsule with average radius)
		for (const FKTaperedCapsuleElem& TapCap : AggGeom.TaperedCapsuleElems)
		{
			const float AvgRadius = (TapCap.Radius0 + TapCap.Radius1) * 0.5f;
			auto CapsuleCollider = MakeShared<FMagicaCapsuleCollider>(AvgRadius, TapCap.Length * 0.5f);
			CapsuleCollider->Friction = ColliderFriction;

			const FTransform LocalOffset(TapCap.Rotation.Quaternion(), TapCap.Center, FVector::OneVector);
			CapsuleCollider->WorldTransform = LocalOffset * BoneCS;

			const int32 ColliderIdx = SimThread->Solver.Colliders.Num();
			SimThread->Solver.Colliders.Add(CapsuleCollider);

			FColliderBoneMapping Mapping;
			Mapping.CompactBoneIndex = CompactIdx;
			Mapping.ColliderIndex = ColliderIdx;
			Mapping.LocalOffset = LocalOffset;
			ColliderBoneMappings.Add(Mapping);
		}
	}
}

void FAnimNode_MagicaCloth::UpdateColliderTransformsFromPose(FComponentSpacePoseContext& Output)
{
	if (ColliderBoneMappings.Num() == 0 || !SimThread.IsValid())
	{
		return;
	}

	const int32 NumColliders = SimThread->Solver.Colliders.Num();
	if (NumColliders == 0)
	{
		return;
	}

	// Initialize with current collider transforms (so unmapped colliders keep their position)
	TArray<FTransform> ColliderTransforms;
	ColliderTransforms.SetNum(NumColliders);
	for (int32 i = 0; i < NumColliders; ++i)
	{
		if (SimThread->Solver.Colliders[i].IsValid())
		{
			ColliderTransforms[i] = SimThread->Solver.Colliders[i]->WorldTransform;
		}
	}

	// Update mapped colliders from bone pose
	for (const FColliderBoneMapping& Mapping : ColliderBoneMappings)
	{
		if (Mapping.ColliderIndex >= 0 && Mapping.ColliderIndex < NumColliders)
		{
			const FTransform BoneCS = Output.Pose.GetComponentSpaceTransform(Mapping.CompactBoneIndex);
			ColliderTransforms[Mapping.ColliderIndex] = Mapping.LocalOffset * BoneCS;
		}
	}

	SimThread->UpdateColliderTransforms(MoveTemp(ColliderTransforms));
}

// ── Simulation Lifecycle ────────────────────────────────

void FAnimNode_MagicaCloth::InitializeSimulation(FComponentSpacePoseContext& Context)
{
	if (bMultiChainMode)
	{
		// ── Multi-chain mode ────────────────────────────
		if (TotalParticleCount < 2)
		{
			return;
		}

		TArray<FTransform> AllTransforms;
		AllTransforms.SetNum(TotalParticleCount);
		for (int32 i = 0; i < TotalParticleCount; ++i)
		{
			AllTransforms[i] = Context.Pose.GetComponentSpaceTransform(AllBoneCompactIndices[i]);
		}

		SimThread = MakeShared<FClothSimThread>();
		SimThread->SimulationHz = SimulationHz;
		SimThread->MaxStepsPerFrame = 3;
		SimThread->Solver.Gravity = Gravity;
		SimThread->Solver.Damping = Damping;
		SimThread->Solver.MaxVelocity = MaxVelocity;
		SimThread->Solver.SolverIterations = SolverIterations;

		// Build chain ranges: particle 0 = root, then chains
		TArray<FChainRange> Ranges;
		int32 Offset = 1; // skip root
		const FReferenceSkeleton& RefSkeleton = Context.Pose.GetPose().GetBoneContainer().GetReferenceSkeleton();
		const int32 RootIndex = RootBone.BoneIndex;
		const int32 BoneCount = RefSkeleton.GetNum();

		// Find direct children of root to determine chain boundaries
		TArray<int32> ChildRoots;
		for (int32 i = 0; i < BoneCount; ++i)
		{
			if (RefSkeleton.GetParentIndex(i) == RootIndex)
			{
				ChildRoots.Add(i);
			}
		}

		for (const int32 ChildRoot : ChildRoots)
		{
			// Count how many bones in this chain (walk to leaf)
			int32 ChainLen = 0;
			int32 Current = ChildRoot;
			constexpr int32 MaxLen = 64;
			while (Current != INDEX_NONE && ChainLen < MaxLen)
			{
				ChainLen++;
				int32 Next = INDEX_NONE;
				for (int32 i = 0; i < BoneCount; ++i)
				{
					if (RefSkeleton.GetParentIndex(i) == Current)
					{
						Next = i;
						break;
					}
				}
				Current = Next;
			}

			FChainRange Range;
			Range.StartIndex = Offset;
			Range.Count = ChainLen;
			Ranges.Add(Range);
			Offset += ChainLen;
		}

		SimThread->Solver.InitializeMultiChain(AllTransforms, AllBoneSkeletonIndices, 1, Ranges);

		FClothConstraintFactory::BuildMultiChainConstraints(
			SimThread->Solver.Particles,
			1, Ranges,
			Stiffness, BendStiffness,
			SimThread->Solver.Constraints);

		// Build cross-chain mesh net constraints (horizontal ring + diagonal shear)
		if (bEnableMeshNet && Ranges.Num() >= 2)
		{
			FClothConstraintFactory::BuildCrossChainMeshConstraints(
				SimThread->Solver.Particles,
				Ranges,
				HorizontalStiffness,
				ShearStiffness,
				bClosedLoop,
				SimThread->Solver.Constraints);
		}

		if (bEnableSelfCollision)
		{
			SimThread->Solver.Constraints.Add(MakeShared<FSelfCollisionConstraint>(SelfCollisionRadius));
		}

		// Build PA colliders
		BuildPhysicsAssetColliders(Context);

		PrevResults = AllTransforms;
		SimThread->Start();
		bInitialized = true;
	}
	else
	{
		// ── Single-chain mode ───────────────────────────
		if (BoneChainIndices.Num() < 2)
		{
			return;
		}

		TArray<FTransform> InitBoneTransforms;
		InitBoneTransforms.SetNum(BoneChainIndices.Num());
		for (int32 i = 0; i < BoneChainIndices.Num(); ++i)
		{
			InitBoneTransforms[i] = Context.Pose.GetComponentSpaceTransform(BoneChainIndices[i]);
		}

		SimThread = MakeShared<FClothSimThread>();
		SimThread->SimulationHz = SimulationHz;
		SimThread->MaxStepsPerFrame = 3;
		SimThread->Solver.Gravity = Gravity;
		SimThread->Solver.Damping = Damping;
		SimThread->Solver.MaxVelocity = MaxVelocity;
		SimThread->Solver.SolverIterations = SolverIterations;
		SimThread->Solver.Initialize(InitBoneTransforms, BoneChainSkeletonIndices, FixedBoneCount);

		FClothConstraintFactory::BuildChainConstraints(
			SimThread->Solver.Particles,
			Stiffness, BendStiffness,
			SimThread->Solver.Constraints);

		if (bEnableSelfCollision)
		{
			SimThread->Solver.Constraints.Add(MakeShared<FSelfCollisionConstraint>(SelfCollisionRadius));
		}

		// Build PA colliders
		BuildPhysicsAssetColliders(Context);

		PrevResults = InitBoneTransforms;
		SimThread->Start();
		bInitialized = true;
	}
}

void FAnimNode_MagicaCloth::ShutdownSimulation()
{
	if (SimThread.IsValid())
	{
		SimThread->Stop();
		SimThread.Reset();
	}
}

// ── Evaluate: read from double buffer, write to bones ───

void FAnimNode_MagicaCloth::EvaluateSkeletalControl_AnyThread(
	FComponentSpacePoseContext& Output,
	TArray<FBoneTransform>& OutBoneTransforms)
{
	if (bNeedsRebuild)
	{
		if (bMultiChainMode)
		{
			BuildMultiChain(Output.Pose.GetPose().GetBoneContainer());
		}
		else
		{
			BuildBoneChain(Output.Pose.GetPose().GetBoneContainer());
		}
	}

	// Determine which bone arrays to use
	const TArray<FCompactPoseBoneIndex>& BoneIndices = bMultiChainMode ? AllBoneCompactIndices : BoneChainIndices;
	const int32 NumBones = BoneIndices.Num();

	if (NumBones < 2)
	{
		return;
	}

	if (!bInitialized)
	{
		InitializeSimulation(Output);
		if (!bInitialized)
		{
			return;
		}
	}

	// Feed current animation transforms to the sim thread
	TArray<FTransform> CurrentAnimTransforms;
	CurrentAnimTransforms.SetNum(NumBones);
	for (int32 i = 0; i < NumBones; ++i)
	{
		CurrentAnimTransforms[i] = Output.Pose.GetComponentSpaceTransform(BoneIndices[i]);
	}
	SimThread->UpdateAnimTransforms(MoveTemp(CurrentAnimTransforms));

	// Update collider transforms from current pose
	UpdateColliderTransformsFromPose(Output);

	// Live-update solver parameters
	SimThread->Solver.Gravity = Gravity;
	SimThread->Solver.Damping = Damping;
	SimThread->Solver.MaxVelocity = MaxVelocity;
	SimThread->SetTargetHz(SimulationHz);

	// Read results from double buffer
	const TArray<FTransform>& SimResult = SimThread->DoubleBuffer.Read();

	if (SimResult.Num() != NumBones)
	{
		return;
	}

	const float InterpAlpha = SimThread->GetInterpolationAlpha();
	const float BlendAlpha = FAnimWeight::IsRelevant(ActualAlpha) ? ActualAlpha : 1.f;

	// Determine how many leading bones to SKIP (pinned roots — anchor only, don't override animation).
	// Multi-chain: SharedRootCount bones are pinned anchors.
	// Single-chain: FixedBoneCount bones are pinned.
	const int32 SkipCount = bMultiChainMode
		? SimThread->Solver.SharedRootCount
		: FixedBoneCount;

	for (int32 i = 0; i < NumBones; ++i)
	{
		// Skip pinned bones — they stay at their animation pose (don't write sim results)
		if (i < SkipCount)
		{
			continue;
		}

		FTransform Interped;

		if (PrevResults.IsValidIndex(i))
		{
			Interped.Blend(PrevResults[i], SimResult[i], InterpAlpha);
		}
		else
		{
			Interped = SimResult[i];
		}

		// Blend with original animation
		if (BlendAlpha < 1.f - UE_SMALL_NUMBER)
		{
			const FTransform OriginalTransform = Output.Pose.GetComponentSpaceTransform(BoneIndices[i]);
			Interped.Blend(OriginalTransform, Interped, BlendAlpha);
		}

		OutBoneTransforms.Add(FBoneTransform(BoneIndices[i], Interped));
	}

	// UE requires OutBoneTransforms sorted by BoneIndex (ascending).
	// Multi-chain mode produces bones from multiple chains in interleaved order.
	OutBoneTransforms.Sort(FCompareBoneTransformIndex());

	PrevResults = SimResult;

#if ENABLE_DRAW_DEBUG
	if (bShowDebug)
	{
		DrawDebug(Output);
	}
#endif
}

// ── Debug Drawing ───────────────────────────────────────

// Uses FAnimInstanceProxy's thread-safe drawing API (safe to call from worker threads)
void FAnimNode_MagicaCloth::DrawDebug(FComponentSpacePoseContext& Output) const
{
	if (!SimThread.IsValid())
	{
		return;
	}

	FAnimInstanceProxy* Proxy = Output.AnimInstanceProxy;
	if (!Proxy)
	{
		return;
	}

	USkeletalMeshComponent* SkelComp = Proxy->GetSkelMeshComponent();
	if (!SkelComp)
	{
		return;
	}

	const FTransform CompToWorld = SkelComp->GetComponentTransform();
	const TArray<FPBDParticle>& Particles = SimThread->Solver.Particles;
	const TArray<TSharedPtr<FClothConstraintBase>>& Constraints = SimThread->Solver.Constraints;

	// ── Draw Particles ──────────────────────────────────
	for (int32 i = 0; i < Particles.Num(); ++i)
	{
		const FVector WorldPos = CompToWorld.TransformPosition(Particles[i].Position);
		const FColor Color = (Particles[i].InvMass <= 0.f) ? FColor::Red : FColor::Green;
		const float Size = (Particles[i].InvMass <= 0.f) ? DebugParticleSize * 2.f : DebugParticleSize;

		// Draw as small cross (thread-safe via Proxy)
		Proxy->AnimDrawDebugPoint(WorldPos, Size * 3.f, Color, false, -1.f, SDPG_Foreground);
	}

	// ── Draw Constraints (lines) ────────────────────────
	for (const TSharedPtr<FClothConstraintBase>& Constraint : Constraints)
	{
		if (!Constraint.IsValid())
		{
			continue;
		}

		// Only draw structural constraints (they have ParticleA/B)
		const FStructuralConstraint* Structural = static_cast<const FStructuralConstraint*>(Constraint.Get());
		const int32 A = Structural->ParticleA;
		const int32 B = Structural->ParticleB;

		if (!Particles.IsValidIndex(A) || !Particles.IsValidIndex(B))
		{
			continue;
		}

		const FVector PosA = CompToWorld.TransformPosition(Particles[A].Position);
		const FVector PosB = CompToWorld.TransformPosition(Particles[B].Position);

		// Color: White = same chain (vertical), Cyan = cross-chain (mesh net)
		FColor LineColor;
		const bool bCrossChain = (Particles[A].ChainId != Particles[B].ChainId) ||
			(Particles[A].ChainId == INDEX_NONE) || (Particles[B].ChainId == INDEX_NONE);

		if (bCrossChain && Particles[A].ChainId != INDEX_NONE && Particles[B].ChainId != INDEX_NONE)
		{
			LineColor = FColor::Cyan; // mesh net constraint
		}
		else
		{
			LineColor = FColor::White; // vertical / root-to-chain
		}

		Proxy->AnimDrawDebugLine(PosA, PosB, LineColor, false, -1.f, 0.5f, SDPG_Foreground);
	}

	// ── Draw Colliders ──────────────────────────────────
	for (const TSharedPtr<FMagicaColliderShape>& Collider : SimThread->Solver.Colliders)
	{
		if (!Collider.IsValid())
		{
			continue;
		}

		const FTransform ColliderWorld = Collider->WorldTransform * CompToWorld;
		const FVector Center = ColliderWorld.GetTranslation();

		switch (Collider->ShapeType)
		{
		case EMagicaColliderShapeType::Sphere:
		{
			const auto* Sphere = static_cast<const FMagicaSphereCollider*>(Collider.Get());
			const float R = Sphere->Radius * ColliderWorld.GetScale3D().GetMax();
			// Draw sphere as diamond cross
			Proxy->AnimDrawDebugLine(Center + FVector(R,0,0), Center - FVector(R,0,0), FColor::Orange, false, -1.f, 0.4f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Center + FVector(0,R,0), Center - FVector(0,R,0), FColor::Orange, false, -1.f, 0.4f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Center + FVector(0,0,R), Center - FVector(0,0,R), FColor::Orange, false, -1.f, 0.4f, SDPG_Foreground);
			break;
		}
		case EMagicaColliderShapeType::Capsule:
		{
			const auto* Capsule = static_cast<const FMagicaCapsuleCollider*>(Collider.Get());
			const float Scale = ColliderWorld.GetScale3D().GetMax();
			const float R = Capsule->Radius * Scale;
			const float HH = Capsule->HalfHeight * Scale;
			const FVector Up = ColliderWorld.GetRotation().GetUpVector();
			const FVector Top = Center + Up * HH;
			const FVector Bot = Center - Up * HH;
			const FVector Right = ColliderWorld.GetRotation().GetRightVector() * R;
			const FVector Fwd = ColliderWorld.GetRotation().GetForwardVector() * R;

			// Capsule: 4 side lines + top/bottom crosses
			Proxy->AnimDrawDebugLine(Top + Right, Bot + Right, FColor::Magenta, false, -1.f, 0.5f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Top - Right, Bot - Right, FColor::Magenta, false, -1.f, 0.5f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Top + Fwd, Bot + Fwd, FColor::Magenta, false, -1.f, 0.5f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Top - Fwd, Bot - Fwd, FColor::Magenta, false, -1.f, 0.5f, SDPG_Foreground);
			// Top cross
			Proxy->AnimDrawDebugLine(Top + Right, Top - Right, FColor::Magenta, false, -1.f, 0.3f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Top + Fwd, Top - Fwd, FColor::Magenta, false, -1.f, 0.3f, SDPG_Foreground);
			// Bottom cross
			Proxy->AnimDrawDebugLine(Bot + Right, Bot - Right, FColor::Magenta, false, -1.f, 0.3f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Bot + Fwd, Bot - Fwd, FColor::Magenta, false, -1.f, 0.3f, SDPG_Foreground);
			// Center axis
			Proxy->AnimDrawDebugLine(Top, Bot, FColor::Magenta, false, -1.f, 0.3f, SDPG_Foreground);
			break;
		}
		case EMagicaColliderShapeType::Box:
		{
			// Draw 4 vertical lines for box outline
			const auto* Box = static_cast<const FMagicaBoxCollider*>(Collider.Get());
			const FVector Ext = Box->HalfExtent * ColliderWorld.GetScale3D();
			const FQuat Rot = ColliderWorld.GetRotation();
			const FVector X = Rot.GetAxisX() * Ext.X;
			const FVector Y = Rot.GetAxisY() * Ext.Y;
			const FVector Z = Rot.GetAxisZ() * Ext.Z;
			// 4 top corners to 4 bottom corners
			Proxy->AnimDrawDebugLine(Center+X+Y+Z, Center+X+Y-Z, FColor::Yellow, false, -1.f, 0.4f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Center-X+Y+Z, Center-X+Y-Z, FColor::Yellow, false, -1.f, 0.4f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Center+X-Y+Z, Center+X-Y-Z, FColor::Yellow, false, -1.f, 0.4f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Center-X-Y+Z, Center-X-Y-Z, FColor::Yellow, false, -1.f, 0.4f, SDPG_Foreground);
			break;
		}
		}
	}
}
