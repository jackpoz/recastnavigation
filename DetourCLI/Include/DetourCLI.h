// DetourCLI.h

#pragma once

using namespace System;
using namespace System::IO;
using namespace System::Collections::Generic;
using namespace System::Runtime::InteropServices;

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourNavMeshBuilder.h"
#include "DetourCommon.h"

namespace DetourCLI
{
    public ref struct Point
    {
    public:
        Point(float x, float y, float z) : X(x), Y(y), Z(z)
        {}

        float X, Y, Z;
    };

	public ref class Detour
	{
    public:
        static const int MAX_PATH_LENGTH = 74;
        static const int MAX_POINT_PATH_LENGTH = 74;
        static const int VERTEX_SIZE = 3;
        static const int MAX_MAP_ID = 724 + 1;
        static const float SIZE_OF_GRIDS = 533.3333f;

        bool FindPath(float startX, float startY, float startZ, float endX, float endY, float endZ, int mapID, [Out] List<Point^>^% path);

        Detour();
        ~Detour();

        static void Initialize(String^ mmapsPath);

    private:
        bool LoadTiles(float position[3], int mapID, dtNavMesh* navMesh);
        dtNavMeshQuery* navQuery;
        dtQueryFilter* filter;

        static String^ mmapsFolderPath;
        static array<dtNavMesh*>^ navMeshes;
        static Detour()
        {
            navMeshes = gcnew array<dtNavMesh*>(MAX_MAP_ID);
        };
	};
}

typedef unsigned int uint32;
const uint32 MMAP_MAGIC = 0x4d4d4150; // 'MMAP'

struct MmapTileHeader
{
    uint32 mmapMagic;
    uint32 dtVersion;
    uint32 mmapVersion;
    uint32 size;
    bool usesLiquids : 1;

    MmapTileHeader() : mmapMagic(MMAP_MAGIC), dtVersion(DT_NAVMESH_VERSION),
        mmapVersion(0), size(0), usesLiquids(true) { }
};