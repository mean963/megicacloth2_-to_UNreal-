#include "MagicaClothUEModule.h"
#include "Modules/ModuleManager.h"

#define LOCTEXT_NAMESPACE "FMagicaClothUEModule"

void FMagicaClothUEModule::StartupModule()
{
	UE_LOG(LogTemp, Log, TEXT("MagicaClothUE: Module loaded (UE 5.6)"));
}

void FMagicaClothUEModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FMagicaClothUEModule, MagicaClothUE)
