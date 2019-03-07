#include "NavMeshExporter.h"

#include "Editor.h"
#include "DesktopPlatform/Public/DesktopPlatformModule.h"
#include "NavigationSystem.h"
#include "RecastNavMesh.h"
#include "NavDataGenerator.h"
#include "HAL/FileManager.h"
#include "Detour/DetourNavMesh.h"
#include "DetourTileCache/DetourTileCache.h"
#include "RecastNavMeshGenerator.h"
#include "RecastHelpers.h"
#include "Recast/Recast.h"
#include "PlatformFilemanager.h"
#include "GenericPlatformFile.h"
#include "Detour/DetourCommon.h"
#include "DetourTileCache/DetourTileCacheBuilder.h"
#include "fastlz.h"

namespace
{
	const float UnitScaling = 0.01f;
}

namespace Recast2
{
	struct dtTileCacheLayerHeader
	{
		int magic;								///< Data magic
		int version;							///< Data version
		int tx, ty, tlayer;
		float bmin[3], bmax[3];
		unsigned short hmin, hmax;				///< Height min/max range
		unsigned char width, height;			///< Dimension of the layer.
		unsigned char minx, maxx, miny, maxy;	///< Usable sub-region.
	};

	struct dtTileCacheLayer
	{
		dtTileCacheLayerHeader* header;
		unsigned char regCount;					///< Region count.
		unsigned char* heights;
		unsigned char* areas;
		unsigned char* cons;
		unsigned char* regs;
	};
}

namespace Recast
{
	static const int TILECACHESET_MAGIC = 'T' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'TSET';
	static const int TILECACHESET_VERSION = 1;

	struct dtTileCacheParams
	{
		float orig[3];
		float cs, ch;
		int width, height;
		float walkableHeight;
		float walkableRadius;
		float walkableClimb;
		float maxSimplificationError;
		int maxTiles;
		int maxObstacles;
	};

	struct TileCacheSetHeader
	{
		int magic;
		int version;
		int numTiles;
		dtNavMeshParams meshParams;
		dtTileCacheParams cacheParams;
	};

	struct TileCacheTileHeader
	{
		dtCompressedTileRef tileRef;
		int dataSize;
	};

	const int DT_TILECACHE_MAGIC = 'D' << 24 | 'T' << 16 | 'L' << 8 | 'R'; ///< 'DTLR';
	const int DT_TILECACHE_VERSION = 1;
	const int DT_MAX_LAYER_PER_TILE = 8;
	const int DT_MAX_DYNAMIC_OBSTACLES = 64;

	struct FastLZCompressor : public dtTileCacheCompressor
	{
		virtual int maxCompressedSize(const int bufferSize)
		{
			return (int)(bufferSize* 1.05f);
		}

		virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
			unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
		{
			*compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
			return DT_SUCCESS;
		}

		virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
			unsigned char* buffer, const int maxBufferSize, int* bufferSize)
		{
			*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
			return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
		}
	};
}

using namespace Recast;

bool NavMeshExporter::ExportNavGeom(FText& err)
{
	UWorld* World = GEditor->GetEditorWorldContext().World();
	UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	ANavigationData* NavData = NavSystem->GetDefaultNavDataInstance();
	if (NavData == NULL)
	{
		err = FText::FromString(TEXT("ExportNavGeom Failed ,None NavData Found ,Build Path Data First"));
		return false;
	}

	ARecastNavMesh* RecastNavmesh = StaticCast<ARecastNavMesh*>(NavData);
	if (RecastNavmesh == NULL)
	{
		err = FText::FromString(TEXT("ExportNavGeom Failed ,The NavData Is Not a ARecastNavMesh"));
		return false;
	}

	TArray<FString> SaveFilePath;
	FDesktopPlatformModule::Get()->SaveFileDialog(NULL,
		TEXT("Choose A Path to Save The Navmesh"),
		TEXT(""),
		GEditor->GetEditorWorldContext().World()->GetMapName(),
		TEXT("Navmesh File|*.obj"),
		0,
		SaveFilePath
	);

	if (SaveFilePath.Num() != 1)
	{
		err = FText::FromString(TEXT("ExportNavGeom Failed ,Please Choose A Path to Save The Navmesh"));
		return false;
	}

	// 记得修改RecastNavMeshGenerator.cpp里的ExportGeomToOBJFile，单位要缩小100才能被RecastDemo使用
	FNavDataGenerator* generator = RecastNavmesh->GetGenerator();
	generator->ExportNavigationData(*(SaveFilePath[0]));

	return true;
}

bool NavMeshExporter::ExportTileCache(FText& err)
{
	bool ret = false;

	TArray<FString> SaveFilePath;
	FDesktopPlatformModule::Get()->SaveFileDialog(NULL,
		TEXT("Choose A Path to Save The Navmesh"),
		TEXT(""),
		GEditor->GetEditorWorldContext().World()->GetMapName(),
		TEXT("Tilecache File|*.cache"),
		0,
		SaveFilePath
	);

	if (SaveFilePath.Num() != 1)
	{
		err = FText::FromString(TEXT("ExportNavGeom Failed ,Please Choose A Path to Save The Navmesh"));
		return false;
	}

	UWorld* World = GEditor->GetEditorWorldContext().World();
	UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	ANavigationData* NavData = NavSystem->GetDefaultNavDataInstance();
	if (NavData == NULL)
	{
		err = FText::FromString(TEXT("ExportNavGeom Failed ,None NavData Found ,Build Path Data First"));
		return false;
	}

	ARecastNavMesh* RecastNavmesh = StaticCast<ARecastNavMesh*>(NavData);
	if (RecastNavmesh == NULL)
	{
		err = FText::FromString(TEXT("ExportNavGeom Failed ,The NavData Is Not a ARecastNavMesh"));
		return false;
	}

	const dtNavMesh* dtMesh = RecastNavmesh->GetRecastMesh();
	const auto tilecacheMap = RecastNavmesh->GetAllTileCaches();
	if (tilecacheMap == nullptr || !tilecacheMap->Num())
	{
		err = FText::FromString(TEXT("ExportNavGeom Failed ,TileLayer cache is not exist ,please build path manually"));
		return false;
	}

	// Store header.
	TileCacheSetHeader fileHeader;
	fileHeader.magic = TILECACHESET_MAGIC;
	fileHeader.version = TILECACHESET_VERSION;

	FRecastNavMeshGenerator* NavDataGenerator = (FRecastNavMeshGenerator*)(NavData->GetGenerator());
	const FRecastBuildConfig& Config = NavDataGenerator->GetConfig();
	FBox Bound = Unreal2RecastBox(NavDataGenerator->GetTotalBounds());

	int gw = 0, gh = 0;
	rcCalcGridSize(&Bound.Min.X, &Bound.Max.X, Config.cs, &gw, &gh);
	const int tw = (gw + Config.cs - 1) / Config.cs;
	const int th = (gh + Config.cs - 1) / Config.cs;

	// dtNavMeshParams
	memcpy(fileHeader.cacheParams.orig, dtMesh->getParams()->orig, sizeof(fileHeader.cacheParams.orig));
	for (size_t i = 0; i < 3; i++)
	{
		fileHeader.cacheParams.orig[i] *= UnitScaling;
	}
	fileHeader.cacheParams.cs = Config.cs * UnitScaling;
	fileHeader.cacheParams.ch = Config.ch * UnitScaling;
	fileHeader.cacheParams.width = Config.tileSize;
	fileHeader.cacheParams.height = Config.tileSize;
	fileHeader.cacheParams.walkableHeight = Config.walkableHeight;
	fileHeader.cacheParams.walkableClimb = Config.walkableClimb;
	fileHeader.cacheParams.walkableRadius = Config.walkableRadius;
	fileHeader.cacheParams.maxSimplificationError = Config.maxSimplificationError;
	fileHeader.cacheParams.maxTiles = tw * th * DT_MAX_LAYER_PER_TILE;
	fileHeader.cacheParams.maxObstacles = DT_MAX_DYNAMIC_OBSTACLES;

	// dtNavMeshParams
	memcpy(&fileHeader.meshParams, dtMesh->getParams(), sizeof(dtNavMeshParams));
	for (size_t i = 0; i < 3; i++)
	{
		fileHeader.meshParams.orig[i] *= UnitScaling;
	}
	fileHeader.meshParams.tileWidth *= UnitScaling;
	fileHeader.meshParams.tileHeight *= UnitScaling;

	fileHeader.numTiles = 0;
	for (const auto& tilecacheArray : *tilecacheMap)
	{
		for (const FNavMeshTileData& tile : tilecacheArray.Value)
		{
			++fileHeader.numTiles;
		}
	}

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	IFileHandle* FileHandle = PlatformFile.OpenWrite(*(SaveFilePath[0]));
	if (!FileHandle)
	{
		err = FText::FromString(TEXT("ExportNavGeom Failed , Create file faied"));
		return false;
	}

	FileHandle->Write((const uint8*)&fileHeader, sizeof(fileHeader));

	// Store tiles.
	FastLZCompressor compressor;
	int index = 0;
	for (const auto& tilecacheArray : *tilecacheMap)
	{
		for (const FNavMeshTileData& tilecache : tilecacheArray.Value)
		{
			dtTileCacheLayer *ueLayer = nullptr;
			dtTileCacheLayerHeader *ueLayerHeader = nullptr;
			if (!RecastNavmesh->DecompressTileCacheLayer(tilecache, &ueLayerHeader, &ueLayer))
			{
				err = FText::FromString(TEXT("ExportNavGeom Failed ,DecompressTileCacheLayer Failed"));
				goto Exit0;
			}

			size_t headerSize = dtAlign4(sizeof(Recast2::dtTileCacheLayerHeader));
			uint32_t gridSize = ueLayerHeader->width * ueLayerHeader->height;
			const int maxDataSize = headerSize + compressor.maxCompressedSize(gridSize * 3);
			unsigned char* data = (unsigned char*)dtAlloc(maxDataSize, DT_ALLOC_PERM);

			// head
			Recast2::dtTileCacheLayerHeader* header = (Recast2::dtTileCacheLayerHeader*)data;
			header->magic = Recast::DT_TILECACHE_MAGIC;
			header->version = Recast::DT_TILECACHE_VERSION;
			header->tx = ueLayer->header->tx;
			header->ty = ueLayer->header->ty;
			header->tlayer = ueLayer->header->tlayer;
			memcpy(header->bmin, ueLayer->header->bmin, sizeof(float) * 3);
			memcpy(header->bmax, ueLayer->header->bmax, sizeof(float) * 3);
			for (size_t i = 0; i < 3; i++)
			{
				header->bmin[i] *= UnitScaling;
				header->bmax[i] *= UnitScaling;
			}
			header->hmin = ueLayer->header->hmin * UnitScaling;
			header->hmax = ueLayer->header->hmax * UnitScaling;
			header->height = ueLayer->header->height;
			header->width = ueLayer->header->width;
			header->minx = ueLayer->header->minx;
			header->maxx = ueLayer->header->maxx;
			header->miny = ueLayer->header->miny;
			header->maxy = ueLayer->header->maxy;
			if (header->tx == -1 && header->ty == -3)
				header->tx = header->tx + 1 - 1;

			// data
			size_t temp_buffer_size = gridSize * 3;
			unsigned char* temp_buffer = (unsigned char*)dtAlloc(temp_buffer_size, DT_ALLOC_TEMP);
			uint8_t* Heights = temp_buffer;
			uint8_t* Areas = Heights + gridSize;
			uint8_t* Cons = Areas + gridSize;
			for (uint32_t i = 0; i < gridSize; i++)
				Heights[i] = (uint8_t)ueLayer->heights[i];
			memcpy(Areas, ueLayer->areas, gridSize);
			memcpy(Cons, ueLayer->cons, gridSize);

			// Compress
			unsigned char* compressed = data + headerSize;
			const int maxCompressedSize = maxDataSize - headerSize;
			int compressedSize = 0;
			compressor.compress(temp_buffer, temp_buffer_size, compressed, maxCompressedSize, &compressedSize);

			// write
			TileCacheTileHeader tileHeader;
			tileHeader.tileRef = 1; // no use
			tileHeader.dataSize = headerSize + compressedSize;
			FileHandle->Write((const uint8*)&tileHeader, sizeof(tileHeader));
			FileHandle->Write(data, tileHeader.dataSize);
			index++;
		}
	}

	FileHandle->Flush();
	ret = true;

Exit0:
	if (FileHandle)
		delete FileHandle;
	return ret;
}

FORCEINLINE void GrowConvexHull(const float ExpandBy, const TArray<FVector>& Verts, TArray<FVector>& OutResult)
{
	if (Verts.Num() < 3)
	{
		return;
	}

	struct FSimpleLine
	{
		FVector P1, P2;

		FSimpleLine() {}

		FSimpleLine(FVector Point1, FVector Point2)
			: P1(Point1), P2(Point2)
		{

		}
		static FVector Intersection(const FSimpleLine& Line1, const FSimpleLine& Line2)
		{
			const float A1 = Line1.P2.X - Line1.P1.X;
			const float B1 = Line2.P1.X - Line2.P2.X;
			const float C1 = Line2.P1.X - Line1.P1.X;

			const float A2 = Line1.P2.Y - Line1.P1.Y;
			const float B2 = Line2.P1.Y - Line2.P2.Y;
			const float C2 = Line2.P1.Y - Line1.P1.Y;

			const float Denominator = A2 * B1 - A1 * B2;
			if (Denominator != 0)
			{
				const float t = (B1*C2 - B2 * C1) / Denominator;
				return Line1.P1 + t * (Line1.P2 - Line1.P1);
			}

			return FVector::ZeroVector;
		}
	};

	TArray<FVector> AllVerts(Verts);
	AllVerts.Add(Verts[0]);
	AllVerts.Add(Verts[1]);

	const int32 VertsCount = AllVerts.Num();
	const FQuat Rotation90(FVector(0, 0, 1), FMath::DegreesToRadians(90));

	float RotationAngle = MAX_FLT;
	for (int32 Index = 0; Index < VertsCount - 2; ++Index)
	{
		const FVector& V1 = AllVerts[Index + 0];
		const FVector& V2 = AllVerts[Index + 1];
		const FVector& V3 = AllVerts[Index + 2];

		const FVector V01 = (V1 - V2).GetSafeNormal();
		const FVector V12 = (V2 - V3).GetSafeNormal();
		const FVector NV1 = Rotation90.RotateVector(V01);
		const float d = FVector::DotProduct(NV1, V12);

		if (d < 0)
		{
			// CW
			RotationAngle = -90;
			break;
		}
		else if (d > 0)
		{
			//CCW
			RotationAngle = 90;
			break;
		}
	}

	// check if we detected CW or CCW direction
	if (RotationAngle >= BIG_NUMBER)
	{
		return;
	}

	const float ExpansionThreshold = 2 * ExpandBy;
	const float ExpansionThresholdSQ = ExpansionThreshold * ExpansionThreshold;
	const FQuat Rotation(FVector(0, 0, 1), FMath::DegreesToRadians(RotationAngle));
	FSimpleLine PreviousLine;
	OutResult.Reserve(Verts.Num());
	for (int32 Index = 0; Index < VertsCount - 2; ++Index)
	{
		const FVector& V1 = AllVerts[Index + 0];
		const FVector& V2 = AllVerts[Index + 1];
		const FVector& V3 = AllVerts[Index + 2];

		FSimpleLine Line1;
		if (Index > 0)
		{
			Line1 = PreviousLine;
		}
		else
		{
			const FVector V01 = (V1 - V2).GetSafeNormal();
			const FVector N1 = Rotation.RotateVector(V01).GetSafeNormal();
			const FVector MoveDir1 = N1 * ExpandBy;
			Line1 = FSimpleLine(V1 + MoveDir1, V2 + MoveDir1);
		}

		const FVector V12 = (V2 - V3).GetSafeNormal();
		const FVector N2 = Rotation.RotateVector(V12).GetSafeNormal();
		const FVector MoveDir2 = N2 * ExpandBy;
		const FSimpleLine Line2(V2 + MoveDir2, V3 + MoveDir2);

		const FVector NewPoint = FSimpleLine::Intersection(Line1, Line2);
		if (NewPoint == FVector::ZeroVector)
		{
			// both lines are parallel so just move our point by expansion distance
			OutResult.Add(V2 + MoveDir2);
		}
		else
		{
			const FVector VectorToNewPoint = NewPoint - V2;
			const float DistToNewVector = VectorToNewPoint.SizeSquared2D();
			if (DistToNewVector > ExpansionThresholdSQ)
			{
				//clamp our point to not move to far from original location
				const FVector HelpPos = V2 + VectorToNewPoint.GetSafeNormal2D() * ExpandBy * 1.4142;
				OutResult.Add(HelpPos);
			}
			else
			{
				OutResult.Add(NewPoint);
			}
		}

		PreviousLine = Line2;
	}
}

bool NavMeshExporter::ExportNavArea(FText& err)
{
	bool ret = false;

	TArray<FString> SaveFilePath;
	FDesktopPlatformModule::Get()->SaveFileDialog(NULL,
		TEXT("Choose A Path to Save The Navmesh"),
		TEXT(""),
		GEditor->GetEditorWorldContext().World()->GetMapName(),
		TEXT("Tilecache File|*.obst"),
		0,
		SaveFilePath
	);

	if (SaveFilePath.Num() != 1)
	{
		err = FText::FromString(TEXT("ExportNavArea Failed ,Please Choose A Path to Save The Navmesh"));
		return false;
	}

	UWorld* World = GEditor->GetEditorWorldContext().World();
	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	ANavigationData* _NavData = NavSys->GetDefaultNavDataInstance();
	if (_NavData == NULL)
	{
		err = FText::FromString(TEXT("ExportNavMesh Failed ,None NavData Found ,Build Path Data First"));
		return false;
	}

	ARecastNavMesh* NavData = StaticCast<ARecastNavMesh*>(_NavData);
	if (NavData == NULL)
	{
		err = FText::FromString(TEXT("ExportNavArea Failed ,The NavData Is Not a ARecastNavMesh"));
		return false;
	}

	FRecastNavMeshGenerator* NavDataGenerator = (FRecastNavMeshGenerator*)(NavData->GetGenerator());
	const FRecastBuildConfig& Config = NavDataGenerator->GetConfig();
	FBox TotalBounds = NavDataGenerator->GetTotalBounds();

	struct FAreaExportData
	{
		FString Name;
		FConvexNavAreaData Convex;
		uint8 AreaId;
	};
	TArray<FAreaExportData> AreaExport;

	for (FNavigationOctree::TConstElementBoxIterator<FNavigationOctree::DefaultStackAllocator> It(*NavSys->GetNavOctree(), TotalBounds);
		It.HasPendingElements();
		It.Advance())
	{
		const FNavigationOctreeElement& Element = It.GetCurrentElement();
		const TArray<FAreaNavModifier>& AreaMods = Element.Data->Modifiers.GetAreas();
		for (int32 i = 0; i < AreaMods.Num(); i++)
		{
			FAreaExportData ExportInfo;
			ExportInfo.AreaId = NavData->GetAreaID(AreaMods[i].GetAreaClass());

			if (AreaMods[i].GetShapeType() == ENavigationShapeType::Convex)
			{
				AreaMods[i].GetConvex(ExportInfo.Convex);

				TArray<FVector> ConvexVerts;
				GrowConvexHull(Config.AgentRadius, ExportInfo.Convex.Points, ConvexVerts);
				ExportInfo.Convex.MinZ -= Config.ch;
				ExportInfo.Convex.MaxZ += Config.ch;
				ExportInfo.Convex.Points = ConvexVerts;
				ExportInfo.Name = Element.GetOwner()->GetName();

				AreaExport.Add(ExportInfo);
			}
		}
	}

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	IFileHandle* FileHandle = PlatformFile.OpenWrite(*(SaveFilePath[0]));
	if (!FileHandle)
	{
		err = FText::FromString(TEXT("ExportNavArea Failed , Create file faied"));
		return false;
	}

	FString AreaExportStr = TEXT("\n# Area export\n");

	for (int32 i = 0; i < AreaExport.Num(); i++)
	{
		const FAreaExportData& ExportInfo = AreaExport[i];
		AreaExportStr += FString::Printf(TEXT("\nae %s %d %d %f %f\n"), *ExportInfo.Name,
			ExportInfo.AreaId, ExportInfo.Convex.Points.Num(), ExportInfo.Convex.MinZ * UnitScaling, ExportInfo.Convex.MaxZ * UnitScaling);

		for (int32 iv = 0; iv < ExportInfo.Convex.Points.Num(); iv++)
		{
			FVector Pt = Unreal2RecastPoint(ExportInfo.Convex.Points[iv]);
			AreaExportStr += FString::Printf(TEXT("%f %f %f\n"), Pt.X * UnitScaling, Pt.Y * UnitScaling, Pt.Z * UnitScaling);
		}
	}

	FString AdditionalData;

	AdditionalData += FString::Printf(TEXT("# AgentHeight\n"));
	AdditionalData += FString::Printf(TEXT("rd_agh %5.5f\n"), Config.AgentHeight * UnitScaling);
	AdditionalData += FString::Printf(TEXT("# AgentRadius\n"));
	AdditionalData += FString::Printf(TEXT("rd_agr %5.5f\n"), Config.AgentRadius * UnitScaling);

	AdditionalData += AreaExportStr;

	auto AnsiAdditionalData = StringCast<ANSICHAR>(*AdditionalData);
	FileHandle->Write((const uint8*)AnsiAdditionalData.Get(), AnsiAdditionalData.Length());

	FileHandle->Flush();
	ret = true;

	if (FileHandle)
		delete FileHandle;
	return ret;
}
