#include "Colliders/MagicaColliderComponent.h"

UMagicaColliderComponent::UMagicaColliderComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

TSharedPtr<FMagicaColliderShape> UMagicaColliderComponent::CreateColliderShape() const
{
	TSharedPtr<FMagicaColliderShape> Shape;

	switch (ColliderType)
	{
	case EMagicaColliderType::Sphere:
	{
		auto Sphere = MakeShared<FMagicaSphereCollider>(Radius);
		Sphere->Friction = Friction;
		Sphere->WorldTransform = GetComponentTransform();
		Shape = Sphere;
		break;
	}
	case EMagicaColliderType::Capsule:
	{
		auto Capsule = MakeShared<FMagicaCapsuleCollider>(Radius, HalfHeight);
		Capsule->Friction = Friction;
		Capsule->WorldTransform = GetComponentTransform();
		Shape = Capsule;
		break;
	}
	case EMagicaColliderType::Box:
	{
		auto Box = MakeShared<FMagicaBoxCollider>(BoxHalfExtent);
		Box->Friction = Friction;
		Box->WorldTransform = GetComponentTransform();
		Shape = Box;
		break;
	}
	}

	return Shape;
}

void UMagicaColliderComponent::UpdateShapeTransform(TSharedPtr<FMagicaColliderShape>& Shape) const
{
	if (Shape.IsValid())
	{
		Shape->WorldTransform = GetComponentTransform();
	}
}
