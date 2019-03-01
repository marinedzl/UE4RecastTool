// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FToolBarBuilder;
class FMenuBuilder;

class FRecastToolModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	/** ExportNavMesh Button */
public:
	void ExportNavMeshButtonClicked();
	void ExportTileCacheButtonClicked();


	/** NavDataAutoUpdate ToggleButton */
public:
	const FUIAction NavDataAutoUpdate_Custom(const FGuid SessionInstanceID) const;
protected:
	void NavDataAutoUpdate_Execute(const FGuid SessionInstanceID);
	bool NavDataAutoUpdate_CanExecute(const FGuid SessionInstanceID) const;
	ECheckBoxState NavDataAutoUpdate_GetCheckState(const FGuid SessionInstanceID) const;
	
private:
	void AddToolbarExtension(FToolBarBuilder& Builder);
	void AddMenuExtension(FMenuBuilder& Builder);

private:
	TSharedPtr<class FUICommandList> PluginCommands;
};
