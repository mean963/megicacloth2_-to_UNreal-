// MagicaClothUE/Private/VirtualMesh/VirtualMesh.cpp
#include "VirtualMesh/VirtualMesh.h"

void FMagicaVirtualMesh::BuildFromBoneChain(const TArray<FTransform>& BoneTransforms, int32 FixedCount)
{
	const int32 Count = BoneTransforms.Num();
	if (Count == 0) return;

	Positions.SetNum(Count);
	Attributes.SetNum(Count);
	ParentIndices.SetNum(Count);
	ChildIndices.SetNum(Count);
	RestLocalOffsets.SetNum(Count);
	RestLocalRotations.SetNum(Count);

	SharedRootCount = FixedCount;
	ChainRanges.Empty();

	for (int32 i = 0; i < Count; ++i)
	{
		Positions[i] = BoneTransforms[i].GetTranslation();

		Attributes[i].Flags = (i < FixedCount)
			? EMagicaVertexFlag::Fixed
			: EMagicaVertexFlag::Move;

		ParentIndices[i] = (i > 0) ? (i - 1) : INDEX_NONE;
		ChildIndices[i].Reset();

		if (i > 0)
		{
			ChildIndices[i - 1].Add(i);
			const FVector ParentPos = BoneTransforms[i - 1].GetTranslation();
			RestLocalOffsets[i] = BoneTransforms[i - 1].InverseTransformPosition(Positions[i]);
			RestLocalRotations[i] = BoneTransforms[i - 1].GetRotation().Inverse() * BoneTransforms[i].GetRotation();
		}
		else
		{
			RestLocalOffsets[i] = FVector::ZeroVector;
			RestLocalRotations[i] = FQuat::Identity;
		}
	}

	CalculateDepth();
}

void FMagicaVirtualMesh::BuildFromMultiChain(
	const TArray<FTransform>& AllTransforms,
	int32 InSharedRootCount,
	const TArray<FMagicaChainRange>& InChainRanges)
{
	const int32 Count = AllTransforms.Num();
	if (Count == 0) return;

	SharedRootCount = InSharedRootCount;
	ChainRanges = InChainRanges;

	Positions.SetNum(Count);
	Attributes.SetNum(Count);
	ParentIndices.SetNum(Count);
	ChildIndices.SetNum(Count);
	RestLocalOffsets.SetNum(Count);
	RestLocalRotations.SetNum(Count);

	// Shared roots are Fixed
	for (int32 i = 0; i < SharedRootCount; ++i)
	{
		Positions[i] = AllTransforms[i].GetTranslation();
		Attributes[i].Flags = EMagicaVertexFlag::Fixed;
		ParentIndices[i] = (i > 0) ? (i - 1) : INDEX_NONE;
		ChildIndices[i].Reset();
		RestLocalOffsets[i] = (i > 0)
			? AllTransforms[i - 1].InverseTransformPosition(Positions[i])
			: FVector::ZeroVector;
		RestLocalRotations[i] = (i > 0)
			? (AllTransforms[i - 1].GetRotation().Inverse() * AllTransforms[i].GetRotation())
			: FQuat::Identity;
	}

	// Per-chain particles
	for (const FMagicaChainRange& Range : ChainRanges)
	{
		for (int32 j = 0; j < Range.Count; ++j)
		{
			const int32 Idx = Range.StartIndex + j;
			Positions[Idx] = AllTransforms[Idx].GetTranslation();

			// First bone of each chain connects to last shared root
			const int32 Parent = (j == 0)
				? (SharedRootCount - 1)
				: (Idx - 1);

			ParentIndices[Idx] = Parent;
			ChildIndices[Idx].Reset();
			ChildIndices[Parent].Add(Idx);

			// All chain bones are Move (only shared roots are Fixed)
			Attributes[Idx].Flags = EMagicaVertexFlag::Move;

			RestLocalOffsets[Idx] = AllTransforms[Parent].InverseTransformPosition(Positions[Idx]);
			RestLocalRotations[Idx] = AllTransforms[Parent].GetRotation().Inverse() * AllTransforms[Idx].GetRotation();
		}
	}

	CalculateDepth();
}

void FMagicaVirtualMesh::CalculateDepth()
{
	const int32 Count = Positions.Num();

	// Calculate total chain length for normalization
	TotalChainLength = 0.0f;
	TArray<float> DistFromRoot;
	DistFromRoot.SetNumZeroed(Count);

	for (int32 i = 0; i < Count; ++i)
	{
		if (ParentIndices[i] != INDEX_NONE)
		{
			const float EdgeLen = FVector::Dist(Positions[i], Positions[ParentIndices[i]]);
			DistFromRoot[i] = DistFromRoot[ParentIndices[i]] + EdgeLen;
			TotalChainLength = FMath::Max(TotalChainLength, DistFromRoot[i]);
		}
	}

	// Normalize to 0..1
	for (int32 i = 0; i < Count; ++i)
	{
		Attributes[i].Depth = (TotalChainLength > UE_SMALL_NUMBER)
			? (DistFromRoot[i] / TotalChainLength)
			: 0.0f;
	}
}

float FMagicaVirtualMesh::SampleDepthCurve(const FRuntimeFloatCurve& Curve, float Depth)
{
	if (const FRichCurve* RichCurve = Curve.GetRichCurveConst())
	{
		if (RichCurve->GetNumKeys() > 0)
		{
			return RichCurve->Eval(Depth);
		}
	}
	return 1.0f;
}
