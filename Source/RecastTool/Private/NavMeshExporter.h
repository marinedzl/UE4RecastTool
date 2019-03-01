#pragma once

class NavMeshExporter
{
public:
	static bool ExportTileCacheData(FText& err);
	static bool ExportNavMeshObj(FText& err);
};
