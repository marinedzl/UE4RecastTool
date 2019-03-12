#pragma once

class NavMeshExporter
{
public:
	static bool ExportTileCache(FText& err);
	static bool ExportNavGeom(FText& err);
	static bool ExportGeomSet(FText& err);
};
