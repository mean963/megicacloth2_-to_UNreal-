#include "AnimNode/AnimNode_MagicaCloth.h"
#include "Engine/World.h"
#include "Core/MagicaClothSubsystem.h"
#include "Simulation/SimulationManager.h"
#include "Simulation/FPBDSolver.h"
#include "Simulation/Constraints/DistanceConstraint.h"
#include "Simulation/Constraints/BendingConstraint.h"
#include "Simulation/Constraints/TetherConstraint.h"
#include "Simulation/Constraints/InertiaConstraint.h"
#include "Simulation/Constraints/AngleConstraint.h"
#include "VirtualMesh/VirtualMesh.h"
#include "Colliders/MagicaColliderShapes.h"
#include "Colliders/MagicaLimitsDataAsset.h"
#include "Animation/AnimInstanceProxy.h"
#include "Animation/AnimTypes.h"
#include "ReferenceSkeleton.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "DrawDebugHelpers.h"

// ═════════════════════════════════════════════════════════════
// FAnimNode_SkeletalControlBase interface
// ═════════════════════════════════════════════════════════════

void FAnimNode_MagicaCloth::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	RootBone.Initialize(RequiredBones);
	EndBone.Initialize(RequiredBones);

	for (FBoneReference& BoneRef : ExcludeBones)
	{
		BoneRef.Initialize(RequiredBones);
	}
	for (FMagicaRootBoneSetting& Setting : AdditionalRootBones)
	{
		Setting.RootBone.Initialize(RequiredBones);
		for (FBoneReference& BoneRef : Setting.OverrideExcludeBones)
		{
			BoneRef.Initialize(RequiredBones);
		}
	}
	for (FBoneReference& BoneRef : PhysicsAssetBoneFilter)
	{
		BoneRef.Initialize(RequiredBones);
	}

	// Initialize driving bones in inline limits
	for (FMagicaSphericalLimit& Limit : SphericalLimits)
	{
		Limit.DrivingBone.Initialize(RequiredBones);
	}
	for (FMagicaCapsuleLimit& Limit : CapsuleLimits)
	{
		Limit.DrivingBone.Initialize(RequiredBones);
	}
	for (FMagicaBoxLimit& Limit : BoxLimits)
	{
		Limit.DrivingBone.Initialize(RequiredBones);
	}
	for (FMagicaPlanarLimit& Limit : PlanarLimits)
	{
		Limit.DrivingBone.Initialize(RequiredBones);
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

// ═════════════════════════════════════════════════════════════
// Single-Chain Bone Discovery (reused from existing code)
// ═════════════════════════════════════════════════════════════

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
		// Walk from EndBone up to RootBone, then reverse
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
		// Auto-walk: follow first child to leaf
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

// ═════════════════════════════════════════════════════════════
// Multi-Chain (Skirt) Bone Discovery (reused from existing code)
// ═════════════════════════════════════════════════════════════

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
	for (const FBoneReference& BoneRef : ExcludeBones)
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

	// For each child, walk to leaf -> one chain
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

// ═════════════════════════════════════════════════════════════
// Subsystem Helper
// ═════════════════════════════════════════════════════════════

UMagicaClothSubsystem* FAnimNode_MagicaCloth::GetSubsystem(FComponentSpacePoseContext& Context) const
{
	if (CachedSubsystem.IsValid())
	{
		return CachedSubsystem.Get();
	}

	USkeletalMeshComponent* SkelComp = Context.AnimInstanceProxy->GetSkelMeshComponent();
	if (!SkelComp)
	{
		return nullptr;
	}

	UWorld* World = SkelComp->GetWorld();
	if (!World)
	{
		return nullptr;
	}

	return World->GetSubsystem<UMagicaClothSubsystem>();
}

// ═════════════════════════════════════════════════════════════
// Collider Building: Inline Limits
// ═════════════════════════════════════════════════════════════

void FAnimNode_MagicaCloth::BuildInlineLimitColliders(
	FComponentSpacePoseContext& Context,
	FMagicaSimulationManager::FTeamData& Team)
{
	const FBoneContainer& RequiredBones = Context.Pose.GetPose().GetBoneContainer();

	// Spherical limits
	for (const FMagicaSphericalLimit& Limit : SphericalLimits)
	{
		if (!Limit.bEnable)
		{
			continue;
		}

		auto Collider = MakeShared<FMagicaSphereCollider>(Limit.Radius);
		Collider->Friction = ColliderFriction;
		Collider->LimitType = Limit.LimitType;

		FTransform ColliderTransform = FTransform(Limit.OffsetRotation.Quaternion(), Limit.OffsetLocation);
		if (Limit.DrivingBone.IsValidToEvaluate(RequiredBones))
		{
			const FCompactPoseBoneIndex CompactIdx = RequiredBones.MakeCompactPoseIndex(
				FMeshPoseBoneIndex(Limit.DrivingBone.BoneIndex));
			if (CompactIdx != INDEX_NONE)
			{
				const FTransform BoneCS = Context.Pose.GetComponentSpaceTransform(CompactIdx);
				ColliderTransform = ColliderTransform * BoneCS;

				FColliderBoneMapping Mapping;
				Mapping.CompactBoneIndex = CompactIdx;
				Mapping.ColliderIndex = Team.Solver.Colliders.Num();
				Mapping.LocalOffset = FTransform(Limit.OffsetRotation.Quaternion(), Limit.OffsetLocation);
				ColliderBoneMappings.Add(Mapping);
			}
		}

		Collider->WorldTransform = ColliderTransform;
		Team.Solver.Colliders.Add(Collider);
	}

	// Capsule limits
	for (const FMagicaCapsuleLimit& Limit : CapsuleLimits)
	{
		if (!Limit.bEnable)
		{
			continue;
		}

		auto Collider = MakeShared<FMagicaCapsuleCollider>(Limit.Radius, Limit.Length * 0.5f);
		Collider->Friction = ColliderFriction;
		Collider->LimitType = Limit.LimitType;

		FTransform ColliderTransform = FTransform(Limit.OffsetRotation.Quaternion(), Limit.OffsetLocation);
		if (Limit.DrivingBone.IsValidToEvaluate(RequiredBones))
		{
			const FCompactPoseBoneIndex CompactIdx = RequiredBones.MakeCompactPoseIndex(
				FMeshPoseBoneIndex(Limit.DrivingBone.BoneIndex));
			if (CompactIdx != INDEX_NONE)
			{
				const FTransform BoneCS = Context.Pose.GetComponentSpaceTransform(CompactIdx);
				ColliderTransform = ColliderTransform * BoneCS;

				FColliderBoneMapping Mapping;
				Mapping.CompactBoneIndex = CompactIdx;
				Mapping.ColliderIndex = Team.Solver.Colliders.Num();
				Mapping.LocalOffset = FTransform(Limit.OffsetRotation.Quaternion(), Limit.OffsetLocation);
				ColliderBoneMappings.Add(Mapping);
			}
		}

		Collider->WorldTransform = ColliderTransform;
		Team.Solver.Colliders.Add(Collider);
	}

	// Box limits
	for (const FMagicaBoxLimit& Limit : BoxLimits)
	{
		if (!Limit.bEnable)
		{
			continue;
		}

		auto Collider = MakeShared<FMagicaBoxCollider>(Limit.Extent);
		Collider->Friction = ColliderFriction;
		Collider->LimitType = Limit.LimitType;

		FTransform ColliderTransform = FTransform(Limit.OffsetRotation.Quaternion(), Limit.OffsetLocation);
		if (Limit.DrivingBone.IsValidToEvaluate(RequiredBones))
		{
			const FCompactPoseBoneIndex CompactIdx = RequiredBones.MakeCompactPoseIndex(
				FMeshPoseBoneIndex(Limit.DrivingBone.BoneIndex));
			if (CompactIdx != INDEX_NONE)
			{
				const FTransform BoneCS = Context.Pose.GetComponentSpaceTransform(CompactIdx);
				ColliderTransform = ColliderTransform * BoneCS;

				FColliderBoneMapping Mapping;
				Mapping.CompactBoneIndex = CompactIdx;
				Mapping.ColliderIndex = Team.Solver.Colliders.Num();
				Mapping.LocalOffset = FTransform(Limit.OffsetRotation.Quaternion(), Limit.OffsetLocation);
				ColliderBoneMappings.Add(Mapping);
			}
		}

		Collider->WorldTransform = ColliderTransform;
		Team.Solver.Colliders.Add(Collider);
	}

	// Planar limits
	for (const FMagicaPlanarLimit& Limit : PlanarLimits)
	{
		if (!Limit.bEnable)
		{
			continue;
		}

		auto Collider = MakeShared<FMagicaPlaneCollider>();
		Collider->Plane = Limit.Plane;
		Collider->Friction = ColliderFriction;
		Collider->LimitType = Limit.LimitType;

		FTransform ColliderTransform = FTransform(Limit.OffsetRotation.Quaternion(), Limit.OffsetLocation);
		if (Limit.DrivingBone.IsValidToEvaluate(RequiredBones))
		{
			const FCompactPoseBoneIndex CompactIdx = RequiredBones.MakeCompactPoseIndex(
				FMeshPoseBoneIndex(Limit.DrivingBone.BoneIndex));
			if (CompactIdx != INDEX_NONE)
			{
				const FTransform BoneCS = Context.Pose.GetComponentSpaceTransform(CompactIdx);
				ColliderTransform = ColliderTransform * BoneCS;

				FColliderBoneMapping Mapping;
				Mapping.CompactBoneIndex = CompactIdx;
				Mapping.ColliderIndex = Team.Solver.Colliders.Num();
				Mapping.LocalOffset = FTransform(Limit.OffsetRotation.Quaternion(), Limit.OffsetLocation);
				ColliderBoneMappings.Add(Mapping);
			}
		}

		Collider->WorldTransform = ColliderTransform;
		Team.Solver.Colliders.Add(Collider);
	}
}

// ═════════════════════════════════════════════════════════════
// Collider Building: DataAsset
// ═════════════════════════════════════════════════════════════

void FAnimNode_MagicaCloth::BuildDataAssetColliders(
	FComponentSpacePoseContext& Context,
	FMagicaSimulationManager::FTeamData& Team)
{
	if (!LimitsDataAsset)
	{
		return;
	}

	const FBoneContainer& RequiredBones = Context.Pose.GetPose().GetBoneContainer();

	// Helper lambda to add a collider with bone mapping
	auto AddLimitCollider = [&](const FMagicaCollisionLimitBase& Limit, TSharedPtr<FMagicaColliderShape> Collider)
	{
		FTransform ColliderTransform = FTransform(Limit.OffsetRotation.Quaternion(), Limit.OffsetLocation);
		if (Limit.DrivingBone.BoneIndex != INDEX_NONE)
		{
			const FCompactPoseBoneIndex CompactIdx = RequiredBones.MakeCompactPoseIndex(
				FMeshPoseBoneIndex(Limit.DrivingBone.BoneIndex));
			if (CompactIdx != INDEX_NONE)
			{
				const FTransform BoneCS = Context.Pose.GetComponentSpaceTransform(CompactIdx);
				ColliderTransform = ColliderTransform * BoneCS;

				FColliderBoneMapping Mapping;
				Mapping.CompactBoneIndex = CompactIdx;
				Mapping.ColliderIndex = Team.Solver.Colliders.Num();
				Mapping.LocalOffset = FTransform(Limit.OffsetRotation.Quaternion(), Limit.OffsetLocation);
				ColliderBoneMappings.Add(Mapping);
			}
		}

		Collider->WorldTransform = ColliderTransform;
		Collider->Friction = ColliderFriction;
		Collider->LimitType = Limit.LimitType;
		Team.Solver.Colliders.Add(Collider);
	};

	for (const FMagicaSphericalLimit& Limit : LimitsDataAsset->SphericalLimits)
	{
		if (!Limit.bEnable) continue;
		AddLimitCollider(Limit, MakeShared<FMagicaSphereCollider>(Limit.Radius));
	}

	for (const FMagicaCapsuleLimit& Limit : LimitsDataAsset->CapsuleLimits)
	{
		if (!Limit.bEnable) continue;
		AddLimitCollider(Limit, MakeShared<FMagicaCapsuleCollider>(Limit.Radius, Limit.Length * 0.5f));
	}

	for (const FMagicaBoxLimit& Limit : LimitsDataAsset->BoxLimits)
	{
		if (!Limit.bEnable) continue;
		AddLimitCollider(Limit, MakeShared<FMagicaBoxCollider>(Limit.Extent));
	}

	for (const FMagicaPlanarLimit& Limit : LimitsDataAsset->PlanarLimits)
	{
		if (!Limit.bEnable) continue;
		auto PlaneCollider = MakeShared<FMagicaPlaneCollider>();
		PlaneCollider->Plane = Limit.Plane;
		AddLimitCollider(Limit, PlaneCollider);
	}
}

// ═════════════════════════════════════════════════════════════
// Collider Building: PhysicsAsset (reused logic from existing code)
// ═════════════════════════════════════════════════════════════

void FAnimNode_MagicaCloth::BuildPhysicsAssetColliders(
	FComponentSpacePoseContext& Context,
	FMagicaSimulationManager::FTeamData& Team)
{
	if (!PhysicsAssetForLimits)
	{
		return;
	}

	const float Friction = ColliderFriction;

	USkeletalMeshComponent* SkelComp = Context.AnimInstanceProxy->GetSkelMeshComponent();
	if (!SkelComp)
	{
		return;
	}

	UPhysicsAsset* PA = PhysicsAssetForLimits.Get();
	if (!PA)
	{
		return;
	}

	const FBoneContainer& RequiredBones = Context.Pose.GetPose().GetBoneContainer();
	const FReferenceSkeleton& RefSkeleton = RequiredBones.GetReferenceSkeleton();

	// Build allowed bone set from filter
	TSet<FName> AllowedBones;
	const bool bHasFilter = PhysicsAssetBoneFilter.Num() > 0;
	if (bHasFilter)
	{
		for (const FBoneReference& BoneRef : PhysicsAssetBoneFilter)
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

			const int32 ColliderIdx = Team.Solver.Colliders.Num();
			Team.Solver.Colliders.Add(SphereCollider);

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

			const int32 ColliderIdx = Team.Solver.Colliders.Num();
			Team.Solver.Colliders.Add(CapsuleCollider);

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

			const int32 ColliderIdx = Team.Solver.Colliders.Num();
			Team.Solver.Colliders.Add(CapsuleCollider);

			FColliderBoneMapping Mapping;
			Mapping.CompactBoneIndex = CompactIdx;
			Mapping.ColliderIndex = ColliderIdx;
			Mapping.LocalOffset = LocalOffset;
			ColliderBoneMappings.Add(Mapping);
		}
	}
}

// ═════════════════════════════════════════════════════════════
// Collider Transform Update (per frame)
// ═════════════════════════════════════════════════════════════

void FAnimNode_MagicaCloth::UpdateColliderTransformsFromPose(FComponentSpacePoseContext& Output)
{
	if (ColliderBoneMappings.Num() == 0 || TeamId == MAGICA_INVALID_TEAM_ID)
	{
		return;
	}

	UMagicaClothSubsystem* Subsystem = CachedSubsystem.Get();
	if (!Subsystem)
	{
		return;
	}

	FMagicaSimulationManager* SimManager = Subsystem->GetSimulationManager();
	if (!SimManager)
	{
		return;
	}

	FMagicaSimulationManager::FTeamData* Team = SimManager->GetTeam(TeamId);
	if (!Team)
	{
		return;
	}

	const int32 NumColliders = Team->Solver.Colliders.Num();
	if (NumColliders == 0)
	{
		return;
	}

	TArray<FTransform> ColliderTransforms;
	ColliderTransforms.SetNum(NumColliders);
	for (int32 i = 0; i < NumColliders; ++i)
	{
		if (Team->Solver.Colliders[i].IsValid())
		{
			ColliderTransforms[i] = Team->Solver.Colliders[i]->WorldTransform;
		}
	}

	for (const FColliderBoneMapping& Mapping : ColliderBoneMappings)
	{
		if (Mapping.ColliderIndex >= 0 && Mapping.ColliderIndex < NumColliders)
		{
			const FTransform BoneCS = Output.Pose.GetComponentSpaceTransform(Mapping.CompactBoneIndex);
			ColliderTransforms[Mapping.ColliderIndex] = Mapping.LocalOffset * BoneCS;
		}
	}

	SimManager->UpdateColliderTransforms(TeamId, ColliderTransforms);
}

// ═════════════════════════════════════════════════════════════
// Constraint Building
// ═════════════════════════════════════════════════════════════

void FAnimNode_MagicaCloth::SetupConstraints(FMagicaSimulationManager::FTeamData& Team)
{
	if (!VirtualMeshPtr)
	{
		return;
	}

	const FMagicaVirtualMesh& VM = *VirtualMeshPtr;
	const TArray<FVector>& Positions = VM.Positions;
	const TArray<int32>& ParentIndices = VM.ParentIndices;

	// Cache UPROPERTY values into locals to avoid UHT-generated accessor conflicts in -Rocket builds
	const float LocalStiffness = PhysicsSettings.Stiffness;
	const float LocalBendStiffness = PhysicsSettings.BendStiffness;
	const bool bLocalMultiChainMode = bMultiChainMode;
	const bool bLocalEnableMeshNet = bEnableMeshNet;
	const float LocalHorizontalStiffness = HorizontalStiffness;
	const float LocalShearStiffness = ShearStiffness;
	const bool bLocalClosedLoop = bClosedLoop;
	const bool bLocalEnableSelfCollision = bEnableSelfCollision;
	const float LocalSelfCollisionRadius = SelfCollisionRadius;
	const FMagicaInertiaParams LocalInertiaParams = InertiaParams;

	// --- Distance Constraint (Vertical: parent-child) ---
	{
		auto Constraint = MakeShared<FMagicaDistanceConstraint>();
		Constraint->BuildVertical(Positions, ParentIndices, LocalStiffness);
		Team.Solver.PreCollisionConstraints.Add(Constraint);
	}

	// --- Bending Constraint ---
	if (LocalBendStiffness > UE_SMALL_NUMBER)
	{
		auto Constraint = MakeShared<FMagicaBendingConstraint>();
		if (bLocalMultiChainMode && VM.ChainRanges.Num() > 0)
		{
			Constraint->BuildFromMultiChain(Positions, VM.ChainRanges, VM.SharedRootCount, LocalBendStiffness);
		}
		else
		{
			Constraint->BuildFromChain(Positions, ParentIndices, LocalBendStiffness);
		}
		Team.Solver.PreCollisionConstraints.Add(Constraint);
	}

	// --- Tether Constraint (generous max distance) ---
	{
		auto Constraint = MakeShared<FMagicaTetherConstraint>();
		Constraint->Build(Positions, ParentIndices, VM.Attributes, 2.0f); // Allow 2x rest distance
		Team.Solver.PreCollisionConstraints.Add(Constraint);
	}

	// --- Angle Constraint (BoneCloth only, very soft) ---
	if (VM.RestLocalRotations.Num() > 0 && VM.RestLocalOffsets.Num() > 0)
	{
		auto Constraint = MakeShared<FMagicaAngleConstraint>();
		Constraint->Build(Positions, ParentIndices, VM.RestLocalRotations, VM.RestLocalOffsets,
			LocalStiffness * 0.05f); // Very soft — just a hint, not a hard constraint
		Team.Solver.PostCollisionConstraints.Add(Constraint); // Post-collision so it doesn't fight distance
	}

	// --- Inertia Constraint ---
	{
		auto Constraint = MakeShared<FMagicaInertiaConstraint>();
		Constraint->Params = LocalInertiaParams;
		Team.Solver.InertiaConstraint = Constraint;
	}

	// --- Mesh Net Constraints (multi-chain only) ---
	if (bLocalMultiChainMode && bLocalEnableMeshNet && VM.ChainRanges.Num() >= 2)
	{
		// Horizontal distance constraints
		{
			auto Constraint = MakeShared<FMagicaDistanceConstraint>();
			Constraint->BuildHorizontal(Positions, VM.ChainRanges, VM.SharedRootCount,
				LocalHorizontalStiffness, bLocalClosedLoop);
			Team.Solver.PreCollisionConstraints.Add(Constraint);
		}

		// Diagonal (shear) distance constraints
		if (LocalShearStiffness > UE_SMALL_NUMBER)
		{
			auto Constraint = MakeShared<FMagicaDistanceConstraint>();
			Constraint->BuildDiagonal(Positions, VM.ChainRanges, VM.SharedRootCount,
				LocalShearStiffness, bLocalClosedLoop);
			Team.Solver.PreCollisionConstraints.Add(Constraint);
		}
	}
}

// ═════════════════════════════════════════════════════════════
// Simulation Lifecycle
// ═════════════════════════════════════════════════════════════

void FAnimNode_MagicaCloth::InitializeSimulation(FComponentSpacePoseContext& Context)
{
	// Determine which bone arrays to use
	const TArray<FCompactPoseBoneIndex>& BoneIndices = bMultiChainMode ? AllBoneCompactIndices : BoneChainIndices;
	const int32 NumBones = BoneIndices.Num();

	if (NumBones < 2)
	{
		return;
	}

	// Get subsystem
	UMagicaClothSubsystem* Subsystem = GetSubsystem(Context);
	if (!Subsystem)
	{
		return;
	}
	CachedSubsystem = Subsystem;

	FMagicaSimulationManager* SimManager = Subsystem->GetSimulationManager();
	if (!SimManager)
	{
		return;
	}

	// Extract initial bone transforms
	TArray<FTransform> InitTransforms;
	InitTransforms.SetNum(NumBones);
	for (int32 i = 0; i < NumBones; ++i)
	{
		InitTransforms[i] = Context.Pose.GetComponentSpaceTransform(BoneIndices[i]);
	}

	// Build VirtualMesh
	VirtualMeshOwned = MakeShared<FMagicaVirtualMesh>();
	VirtualMeshPtr = VirtualMeshOwned.Get();

	if (bMultiChainMode)
	{
		// Build chain ranges
		TArray<FMagicaChainRange> ChainRanges;
		int32 Offset = 1; // skip root
		const FReferenceSkeleton& RefSkeleton = Context.Pose.GetPose().GetBoneContainer().GetReferenceSkeleton();
		const int32 RootIndex = RootBone.BoneIndex;
		const int32 BoneCount = RefSkeleton.GetNum();

		// Build exclude set
		TSet<int32> ExcludedBoneIndices;
		for (const FBoneReference& BoneRef : ExcludeBones)
		{
			if (BoneRef.BoneIndex != INDEX_NONE)
			{
				ExcludedBoneIndices.Add(BoneRef.BoneIndex);
			}
		}

		TArray<int32> ChildRoots;
		for (int32 i = 0; i < BoneCount; ++i)
		{
			if (RefSkeleton.GetParentIndex(i) == RootIndex && !ExcludedBoneIndices.Contains(i))
			{
				ChildRoots.Add(i);
			}
		}

		for (const int32 ChildRoot : ChildRoots)
		{
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

			FMagicaChainRange Range;
			Range.StartIndex = Offset;
			Range.Count = ChainLen;
			ChainRanges.Add(Range);
			Offset += ChainLen;
		}

		VirtualMeshPtr->BuildFromMultiChain(InitTransforms, 1, ChainRanges);
	}
	else
	{
		VirtualMeshPtr->BuildFromBoneChain(InitTransforms, 1);
	}

	VirtualMeshPtr->CalculateDepth();

	// Register Team with SimulationManager
	TeamId = SimManager->RegisterTeam();
	FMagicaSimulationManager::FTeamData* Team = SimManager->GetTeam(TeamId);
	if (!Team)
	{
		return;
	}

	// Initialize solver from VirtualMesh
	Team->Solver.Gravity = Gravity;
	Team->Solver.Damping = PhysicsSettings.Damping;
	Team->Solver.MaxVelocity = PhysicsSettings.MaxVelocity;
	Team->Solver.SolverIterations = SolverIterations;
	Team->Solver.Initialize(*VirtualMeshPtr);

	// Build all constraints
	SetupConstraints(*Team);

	// Build colliders from all 3 sources
	ColliderBoneMappings.Empty();
	BuildInlineLimitColliders(Context, *Team);
	BuildDataAssetColliders(Context, *Team);
	BuildPhysicsAssetColliders(Context, *Team);

	// Feed initial positions so sim starts from the right place
	TArray<FVector> InitPositions;
	InitPositions.SetNum(NumBones);
	for (int32 i = 0; i < NumBones; ++i)
	{
		InitPositions[i] = InitTransforms[i].GetTranslation();
	}
	SimManager->UpdateAnimPositions(TeamId, InitPositions);

	// Note: simulation runs synchronously in EvaluateSkeletalControl, no worker thread needed

	PrevResults = InitTransforms;
	bInitialized = true;
}

void FAnimNode_MagicaCloth::ShutdownSimulation()
{
	if (TeamId != MAGICA_INVALID_TEAM_ID)
	{
		if (CachedSubsystem.IsValid())
		{
			FMagicaSimulationManager* SimManager = CachedSubsystem->GetSimulationManager();
			if (SimManager)
			{
				SimManager->UnregisterTeam(TeamId);
			}
		}
		TeamId = MAGICA_INVALID_TEAM_ID;
	}

	VirtualMeshOwned.Reset();
	VirtualMeshPtr = nullptr;
	ColliderBoneMappings.Empty();
	CachedSubsystem.Reset();
}

// ═════════════════════════════════════════════════════════════
// Evaluate: read from double buffer, write to bones
// ═════════════════════════════════════════════════════════════

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

	// Ensure subsystem is still valid
	UMagicaClothSubsystem* Subsystem = CachedSubsystem.Get();
	if (!Subsystem)
	{
		Subsystem = GetSubsystem(Output);
		if (!Subsystem)
		{
			return;
		}
		CachedSubsystem = Subsystem;
	}

	FMagicaSimulationManager* SimManager = Subsystem->GetSimulationManager();
	if (!SimManager)
	{
		return;
	}

	FMagicaSimulationManager::FTeamData* Team = SimManager->GetTeam(TeamId);
	if (!Team)
	{
		return;
	}

	// === Synchronous simulation (run PBD directly, no worker thread) ===

	// 1. Update fixed particles from current animation pose
	TArray<FVector> CurrentAnimPositions;
	CurrentAnimPositions.SetNum(NumBones);
	for (int32 i = 0; i < NumBones; ++i)
	{
		CurrentAnimPositions[i] = Output.Pose.GetComponentSpaceTransform(BoneIndices[i]).GetTranslation();
	}
	Team->Solver.UpdateFixedParticles(CurrentAnimPositions);

	// 2. Update collider transforms
	for (const FColliderBoneMapping& Mapping : ColliderBoneMappings)
	{
		if (Mapping.ColliderIndex >= 0 && Mapping.ColliderIndex < Team->Solver.Colliders.Num()
			&& Mapping.CompactBoneIndex != FCompactPoseBoneIndex(INDEX_NONE))
		{
			const FTransform BoneCS = Output.Pose.GetComponentSpaceTransform(Mapping.CompactBoneIndex);
			Team->Solver.Colliders[Mapping.ColliderIndex]->WorldTransform = Mapping.LocalOffset * BoneCS;
		}
	}

	// 3. Update solver parameters
	// Convert world-space gravity to component space
	USkeletalMeshComponent* SkelComp = Output.AnimInstanceProxy->GetSkelMeshComponent();
	if (SkelComp)
	{
		const FTransform CompToWorld = SkelComp->GetComponentTransform();
		Team->Solver.Gravity = CompToWorld.InverseTransformVectorNoScale(Gravity);
	}
	else
	{
		Team->Solver.Gravity = Gravity;
	}
	Team->Solver.Damping = PhysicsSettings.Damping;
	Team->Solver.MaxVelocity = PhysicsSettings.MaxVelocity;
	Team->Solver.SolverIterations = SolverIterations;

	// 4. Step simulation synchronously
	const float DeltaTime = Output.AnimInstanceProxy->GetDeltaSeconds();
	const float ClampedDt = FMath::Clamp(DeltaTime, 0.0001f, 0.033f); // Clamp to avoid instability
	Team->Solver.Step(ClampedDt);

	// 5. Read results directly from solver
	const TArray<FVector>& SimPositions = Team->Solver.GetPositions();

	// Debug logging removed for release
	if (SimPositions.Num() != NumBones)
	{
		return;
	}

	const float BlendAlpha = FAnimWeight::IsRelevant(ActualAlpha) ? ActualAlpha : 1.f;

	// Skip pinned root bones
	const int32 SkipCount = bMultiChainMode
		? 1  // shared root
		: 1; // first bone is fixed

	for (int32 i = 0; i < NumBones; ++i)
	{
		if (i < SkipCount)
		{
			continue;
		}

		// Get simulation position directly from solver
		const FVector SimPos = SimPositions[i];

		// Reconstruct bone rotation from parent-child direction
		const FTransform OriginalTransform = Output.Pose.GetComponentSpaceTransform(BoneIndices[i]);
		FQuat SimRotation = OriginalTransform.GetRotation();

		// Find parent index in the bone chain
		int32 ParentInChain = INDEX_NONE;
		if (VirtualMeshPtr && i < VirtualMeshPtr->ParentIndices.Num())
		{
			ParentInChain = VirtualMeshPtr->ParentIndices[i];
		}

		if (ParentInChain != INDEX_NONE && ParentInChain < NumBones)
		{
			const FVector ParentSimPos = SimPositions[ParentInChain];
			const FVector Dir = SimPos - ParentSimPos;
			const float DirLen = Dir.Size();

			if (DirLen > UE_SMALL_NUMBER)
			{
				const FVector OrigParentPos = Output.Pose.GetComponentSpaceTransform(BoneIndices[ParentInChain]).GetTranslation();
				const FVector OrigDir = OriginalTransform.GetTranslation() - OrigParentPos;
				const float OrigDirLen = OrigDir.Size();

				if (OrigDirLen > UE_SMALL_NUMBER)
				{
					const FQuat DeltaRot = FQuat::FindBetweenNormals(
						OrigDir / OrigDirLen, Dir / DirLen);
					SimRotation = DeltaRot * OriginalTransform.GetRotation();
				}
			}
		}

		FTransform FinalTransform(SimRotation, SimPos, OriginalTransform.GetScale3D());

		// Blend with original animation
		if (BlendAlpha < 1.f - UE_SMALL_NUMBER)
		{
			FinalTransform.Blend(OriginalTransform, FinalTransform, BlendAlpha);
		}

		OutBoneTransforms.Add(FBoneTransform(BoneIndices[i], FinalTransform));
	}

	// UE requires OutBoneTransforms sorted by BoneIndex (ascending)
	OutBoneTransforms.Sort(FCompareBoneTransformIndex());

	// Store current sim positions as PrevResults for next frame
	PrevResults.SetNum(NumBones);
	for (int32 i = 0; i < NumBones; ++i)
	{
		PrevResults[i].SetTranslation(SimPositions[i]);
	}

#if ENABLE_DRAW_DEBUG
	if (bShowDebug)
	{
		DrawDebug(Output);
	}
#endif
}

// ═════════════════════════════════════════════════════════════
// Debug Drawing (reused from existing code, adapted for new solver)
// ═════════════════════════════════════════════════════════════

void FAnimNode_MagicaCloth::DrawDebug(FComponentSpacePoseContext& Output) const
{
	if (TeamId == MAGICA_INVALID_TEAM_ID || !CachedSubsystem.IsValid())
	{
		return;
	}

	FMagicaSimulationManager* SimManager = CachedSubsystem->GetSimulationManager();
	if (!SimManager)
	{
		return;
	}

	FMagicaSimulationManager::FTeamData* Team = SimManager->GetTeam(TeamId);
	if (!Team)
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
	const FMagicaPBDSolver& Solver = Team->Solver;
	const TArray<FVector>& Positions = Solver.Positions;
	const TArray<float>& InvMasses = Solver.InvMasses;

	// Draw particles
	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		const FVector WorldPos = CompToWorld.TransformPosition(Positions[i]);
		const FColor Color = (InvMasses.IsValidIndex(i) && InvMasses[i] <= 0.f) ? FColor::Red : FColor::Green;
		const float Size = (InvMasses.IsValidIndex(i) && InvMasses[i] <= 0.f) ? DebugParticleSize * 2.f : DebugParticleSize;

		Proxy->AnimDrawDebugPoint(WorldPos, Size * 3.f, Color, false, -1.f, SDPG_Foreground);
	}

	// Draw parent-child connections
	if (VirtualMeshPtr)
	{
		for (int32 i = 0; i < VirtualMeshPtr->ParentIndices.Num(); ++i)
		{
			const int32 ParentIdx = VirtualMeshPtr->ParentIndices[i];
			if (ParentIdx >= 0 && ParentIdx < Positions.Num() && i < Positions.Num())
			{
				const FVector PosA = CompToWorld.TransformPosition(Positions[ParentIdx]);
				const FVector PosB = CompToWorld.TransformPosition(Positions[i]);
				Proxy->AnimDrawDebugLine(PosA, PosB, FColor::White, false, -1.f, 0.5f, SDPG_Foreground);
			}
		}
	}

	// Draw colliders
	for (const TSharedPtr<FMagicaColliderShape>& Collider : Solver.Colliders)
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

			Proxy->AnimDrawDebugLine(Top + Right, Bot + Right, FColor::Magenta, false, -1.f, 0.5f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Top - Right, Bot - Right, FColor::Magenta, false, -1.f, 0.5f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Top + Fwd, Bot + Fwd, FColor::Magenta, false, -1.f, 0.5f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Top - Fwd, Bot - Fwd, FColor::Magenta, false, -1.f, 0.5f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Top + Right, Top - Right, FColor::Magenta, false, -1.f, 0.3f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Top + Fwd, Top - Fwd, FColor::Magenta, false, -1.f, 0.3f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Bot + Right, Bot - Right, FColor::Magenta, false, -1.f, 0.3f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Bot + Fwd, Bot - Fwd, FColor::Magenta, false, -1.f, 0.3f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Top, Bot, FColor::Magenta, false, -1.f, 0.3f, SDPG_Foreground);
			break;
		}
		case EMagicaColliderShapeType::Box:
		{
			const auto* Box = static_cast<const FMagicaBoxCollider*>(Collider.Get());
			const FVector Ext = Box->HalfExtent * ColliderWorld.GetScale3D();
			const FQuat Rot = ColliderWorld.GetRotation();
			const FVector X = Rot.GetAxisX() * Ext.X;
			const FVector Y = Rot.GetAxisY() * Ext.Y;
			const FVector Z = Rot.GetAxisZ() * Ext.Z;
			Proxy->AnimDrawDebugLine(Center+X+Y+Z, Center+X+Y-Z, FColor::Yellow, false, -1.f, 0.4f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Center-X+Y+Z, Center-X+Y-Z, FColor::Yellow, false, -1.f, 0.4f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Center+X-Y+Z, Center+X-Y-Z, FColor::Yellow, false, -1.f, 0.4f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Center-X-Y+Z, Center-X-Y-Z, FColor::Yellow, false, -1.f, 0.4f, SDPG_Foreground);
			break;
		}
		case EMagicaColliderShapeType::Plane:
		{
			// Draw plane as a cross at center with normal indicator
			const FVector Normal = ColliderWorld.GetRotation().GetUpVector();
			const FVector Right = ColliderWorld.GetRotation().GetRightVector() * 20.f;
			const FVector Fwd = ColliderWorld.GetRotation().GetForwardVector() * 20.f;
			Proxy->AnimDrawDebugLine(Center + Right, Center - Right, FColor::Turquoise, false, -1.f, 0.4f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Center + Fwd, Center - Fwd, FColor::Turquoise, false, -1.f, 0.4f, SDPG_Foreground);
			Proxy->AnimDrawDebugLine(Center, Center + Normal * 15.f, FColor::Turquoise, false, -1.f, 0.4f, SDPG_Foreground);
			break;
		}
		}
	}
}
