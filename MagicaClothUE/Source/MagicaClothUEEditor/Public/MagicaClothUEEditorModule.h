#pragma once

#include "Modules/ModuleManager.h"

class FMagicaClothUEEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
