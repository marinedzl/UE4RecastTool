// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

#include "RecastTool.h"
#include "RecastToolStyle.h"
#include "RecastToolCommands.h"
#include "Misc/MessageDialog.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "LevelEditor.h"
#include "NavigationSystem.h"

#include "Editor.h"
#include "NavMeshExporter.h"

static const FName RecastToolTabName("RecastTool");

#define LOCTEXT_NAMESPACE "FRecastToolModule"

void FRecastToolModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
	
	FRecastToolStyle::Initialize();
	FRecastToolStyle::ReloadTextures();

	FRecastToolCommands::Register();
	
	PluginCommands = MakeShareable(new FUICommandList);

	PluginCommands->MapAction(FRecastToolCommands::Get().NavDataEnableDrawing, NavDataEnableDrawing_Custom(FGuid()));
	PluginCommands->MapAction(FRecastToolCommands::Get().NavDataAutoUpdate, NavDataAutoUpdate_Custom(FGuid()));

	PluginCommands->MapAction(
		FRecastToolCommands::Get().ExportNavGeom,
		FExecuteAction::CreateRaw(this, &FRecastToolModule::ExportNavGeomButtonClicked),
		FCanExecuteAction());

	PluginCommands->MapAction(
		FRecastToolCommands::Get().ExportTileCache,
		FExecuteAction::CreateRaw(this, &FRecastToolModule::ExportTileCacheButtonClicked),
		FCanExecuteAction());

	PluginCommands->MapAction(
		FRecastToolCommands::Get().ExportGeomSet,
		FExecuteAction::CreateRaw(this, &FRecastToolModule::ExportGeomSetButtonClicked),
		FCanExecuteAction());
		
	FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	
	{
		TSharedPtr<FExtender> MenuExtender = MakeShareable(new FExtender());
		MenuExtender->AddMenuExtension("WindowLayout", EExtensionHook::After, PluginCommands, FMenuExtensionDelegate::CreateRaw(this, &FRecastToolModule::AddMenuExtension));

		LevelEditorModule.GetMenuExtensibilityManager()->AddExtender(MenuExtender);
	}
	
	{
		TSharedPtr<FExtender> ToolbarExtender = MakeShareable(new FExtender);
		ToolbarExtender->AddToolBarExtension("Settings", EExtensionHook::After, PluginCommands, FToolBarExtensionDelegate::CreateRaw(this, &FRecastToolModule::AddToolbarExtension));
		
		LevelEditorModule.GetToolBarExtensibilityManager()->AddExtender(ToolbarExtender);
	}
}

void FRecastToolModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
	FRecastToolStyle::Shutdown();

	FRecastToolCommands::Unregister();
}

void FRecastToolModule::ExportNavGeomButtonClicked()
{
	FText msg = FText::FromString(TEXT("ExportNavGeom Successful!"));
	NavMeshExporter::ExportNavGeom(msg);
	FMessageDialog::Open(EAppMsgType::Ok, msg);
}

void FRecastToolModule::ExportTileCacheButtonClicked()
{
	FText msg = FText::FromString(TEXT("ExportTileCache Successful!"));
	NavMeshExporter::ExportTileCache(msg);
	FMessageDialog::Open(EAppMsgType::Ok, msg);
}

void FRecastToolModule::ExportGeomSetButtonClicked()
{
	FText msg = FText::FromString(TEXT("ExportGeomSet Successful!"));
	NavMeshExporter::ExportGeomSet(msg);
	FMessageDialog::Open(EAppMsgType::Ok, msg);
}

// NavDataAutoUpdate Begin
const FUIAction FRecastToolModule::NavDataAutoUpdate_Custom(const FGuid SessionInstanceID) const
{
	FUIAction UIAction;
	UIAction.ExecuteAction = FExecuteAction::CreateRaw(this, &FRecastToolModule::NavDataAutoUpdate_Execute, SessionInstanceID);
	UIAction.CanExecuteAction = FCanExecuteAction::CreateRaw(this, &FRecastToolModule::NavDataAutoUpdate_CanExecute, SessionInstanceID);
	UIAction.GetActionCheckState = FGetActionCheckState::CreateRaw(this, &FRecastToolModule::NavDataAutoUpdate_GetCheckState, SessionInstanceID);
	return UIAction;
}

void FRecastToolModule::NavDataAutoUpdate_Execute(const FGuid SessionInstanceID)
{
	UWorld* World = GEditor->GetEditorWorldContext().World();
	UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	//UNavigationSystemV1::SetNavigationAutoUpdateEnabled(!NavSystem->GetIsNavigationAutoUpdateEnabled(), NavSystem);
}

bool FRecastToolModule::NavDataAutoUpdate_CanExecute(const FGuid SessionInstanceID) const
{
	return true;
}

ECheckBoxState FRecastToolModule::NavDataAutoUpdate_GetCheckState(const FGuid SessionInstanceID) const
{
	UWorld* World = GEditor->GetEditorWorldContext().World();
	UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);

	//return NavSystem->GetIsNavigationAutoUpdateEnabled() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
	return ECheckBoxState::Unchecked;
}
// NavDataAutoUpdate End

// NavDataEnableDrawing Begin
const FUIAction FRecastToolModule::NavDataEnableDrawing_Custom(const FGuid SessionInstanceID) const
{
	FUIAction UIAction;
	UIAction.ExecuteAction = FExecuteAction::CreateRaw(this, &FRecastToolModule::NavDataEnableDrawing_Execute, SessionInstanceID);
	UIAction.CanExecuteAction = FCanExecuteAction::CreateRaw(this, &FRecastToolModule::NavDataEnableDrawing_CanExecute, SessionInstanceID);
	UIAction.GetActionCheckState = FGetActionCheckState::CreateRaw(this, &FRecastToolModule::NavDataEnableDrawing_GetCheckState, SessionInstanceID);
	return UIAction;
}

void FRecastToolModule::NavDataEnableDrawing_Execute(const FGuid SessionInstanceID)
{
	UWorld* World = GEditor->GetEditorWorldContext().World();
	UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	ANavigationData* NavData = NavSystem->GetDefaultNavDataInstance();
	if (!NavData)
		return;
	NavData->SetNavRenderingEnabled(!NavData->IsDrawingEnabled());
}

bool FRecastToolModule::NavDataEnableDrawing_CanExecute(const FGuid SessionInstanceID) const
{
	UWorld* World = GEditor->GetEditorWorldContext().World();
	UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	ANavigationData* NavData = NavSystem->GetDefaultNavDataInstance();
	return NavData;
}

ECheckBoxState FRecastToolModule::NavDataEnableDrawing_GetCheckState(const FGuid SessionInstanceID) const
{
	UWorld* World = GEditor->GetEditorWorldContext().World();
	UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	ANavigationData* NavData = NavSystem->GetDefaultNavDataInstance();
	if (!NavData)
		return ECheckBoxState::Unchecked;
	return NavData->IsDrawingEnabled() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
}
// NavDataEnableDrawing End

void FRecastToolModule::AddMenuExtension(FMenuBuilder& Builder)
{
	Builder.BeginSection(TEXT("RecastTool"));
	Builder.AddSubMenu(FText::FromString("RecastTool"), FText::FromString("RecastTool"), FNewMenuDelegate::CreateLambda([](FMenuBuilder& Builder)
	{
		Builder.AddMenuEntry(FRecastToolCommands::Get().NavDataAutoUpdate);
		Builder.AddMenuEntry(FRecastToolCommands::Get().NavDataEnableDrawing);
		Builder.AddMenuEntry(FRecastToolCommands::Get().ExportNavGeom);
		Builder.AddMenuEntry(FRecastToolCommands::Get().ExportTileCache);
		Builder.AddMenuEntry(FRecastToolCommands::Get().ExportGeomSet);
	}));
	Builder.EndSection();
}

void FRecastToolModule::AddToolbarExtension(FToolBarBuilder& Builder)
{
	Builder.BeginSection("RecastTool");

	Builder.AddComboButton(FUIAction(), FOnGetContent::CreateLambda([this]() -> TSharedRef<SWidget>
	{
		FMenuBuilder MenuBuilder(true, this->PluginCommands);

		MenuBuilder.BeginSection("RecastTool", LOCTEXT("RecastTool", "RecastTool"));

		MenuBuilder.AddMenuEntry(FRecastToolCommands::Get().NavDataAutoUpdate);
		MenuBuilder.AddMenuEntry(FRecastToolCommands::Get().NavDataEnableDrawing);
		MenuBuilder.AddMenuEntry(FRecastToolCommands::Get().ExportNavGeom);
		MenuBuilder.AddMenuEntry(FRecastToolCommands::Get().ExportTileCache);
		MenuBuilder.AddMenuEntry(FRecastToolCommands::Get().ExportGeomSet);

		MenuBuilder.EndSection();

		return MenuBuilder.MakeWidget();

	}), LOCTEXT("RecastTool", "RecastTool"),
		LOCTEXT("RecastTool", "RecastTool"),
		FSlateIcon(FRecastToolStyle::GetStyleSetName(), "RecastTool.PluginAction"));

	Builder.EndSection();
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FRecastToolModule, RecastTool)