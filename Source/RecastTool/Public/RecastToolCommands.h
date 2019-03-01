// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Framework/Commands/Commands.h"
#include "RecastToolStyle.h"

class FRecastToolCommands : public TCommands<FRecastToolCommands>
{
public:

	FRecastToolCommands()
		: TCommands<FRecastToolCommands>(TEXT("RecastTool"), NSLOCTEXT("Contexts", "RecastTool", "RecastTool Plugin"), NAME_None, FRecastToolStyle::GetStyleSetName())
	{
	}

	// TCommands<> interface
	virtual void RegisterCommands() override;

public:
	TSharedPtr< FUICommandInfo > NavDataAutoUpdate;
	TSharedPtr< FUICommandInfo > ExportNavMeshObj;
	TSharedPtr< FUICommandInfo > ExportTileCache;
};
