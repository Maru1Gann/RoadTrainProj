
#pragma once

#include "CoreMinimal.h"

struct FPathNode;
class ARuntimeTerrain;

class FPathFinder
{
    friend class ARuntimeTerrain; // debugging

public:
    FPathFinder( ARuntimeTerrain& RTref );
    
    void GetPath( const FPathNode& Start, const FPathNode& End, TArray<FIntPoint>& OutPath );
    void FindPathGates( const FPathNode& Start, const FPathNode& End, TArray<FPathNode>& OutGates );

private:

    ARuntimeTerrain& RTref; // don't change member values!!

    float GetBestGate( const FIntPoint& Chunk, const FIntPoint& NextChunk, const FIntPoint& Pos, FPathNode& OutNode );

    // -----------------tools-----------------
    // FVector2D NodeToVector( const FPathNode& Node );
    bool IsPosInChunk( const FIntPoint& Pos, const int32& VertexCount );
    bool IsPosInChunk( const FIntPoint& Pos );
    void GetNeighbors( const FIntPoint& Pos, TArray<FIntPoint>& OutNeighbors );
    void GetGoalSet( const FIntPoint& Chunk, const FIntPoint& NextChunk, const int32& VertexCount, TSet<FIntPoint>& OutGoalSet );
    void GetGoalSet( const FIntPoint& Chunk, const FIntPoint& NextChunk, TSet<FIntPoint>& OutGoalSet );
    int32 GetUnitDistSquared( const FIntPoint& PosA, const FIntPoint PosB );
    float GetSlopeSquared( const FIntPoint& Chunk, const FIntPoint& PosA, const FIntPoint& PosB );
    FVector2D ConvertToVector2D( const FIntPoint& Chunk, const FIntPoint& Pos );
    int32 Heuristic( const FIntPoint& PosA, const FIntPoint& PosB );
    FIntPoint ConvertToGlobal( const FIntPoint& Chunk, const FIntPoint& Pos );
    FIntPoint GlobalToLocal( const FIntPoint& Chunk, const FIntPoint& GlobalPos );
    
};
