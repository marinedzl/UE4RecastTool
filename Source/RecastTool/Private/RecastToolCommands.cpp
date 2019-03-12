// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

#include "RecastToolCommands.h"

#define LOCTEXT_NAMESPACE "FRecastToolModule"

void FRecastToolCommands::RegisterCommands()
{
	UI_COMMAND(NavDataEnableDrawing, "NavDataEnableDrawing", "NavDataEnableDrawing", EUserInterfaceActionType::ToggleButton, FInputGesture());
	UI_COMMAND(NavDataAutoUpdate, "NavDataAutoUpdate", "NavDataAutoUpdate", EUserInterfaceActionType::ToggleButton, FInputGesture());
	UI_COMMAND(ExportNavGeom, "ExportNavGeom", "ExportNavGeom", EUserInterfaceActionType::Button, FInputGesture());
	UI_COMMAND(ExportTileCache, "ExportTileCache", "ExportTileCache", EUserInterfaceActionType::Button, FInputGesture());
	UI_COMMAND(ExportGeomSet, "ExportGeomSet", "ExportGeomSet", EUserInterfaceActionType::Button, FInputGesture());
}

#undef LOCTEXT_NAMESPACE
