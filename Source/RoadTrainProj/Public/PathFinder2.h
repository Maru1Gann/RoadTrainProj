
#pragma once

#include "CoreMinimal.h"

struct FPathNode;
class ARuntimeTerrain;

class FPathFinder
{
public:
    FPathFinder( const ARuntimeTerrain& RTref ): RTref(RTref) {}
    
    void GetPath( const FPathNode& Start, const FPathNode& End, TArray<FPathNode>& OutPath );
    void FindPath( const FPathNode& Start, const FPathNode& End, TArray<FPathNode>& OutGates );

private:

    const ARuntimeTerrain& RTref;

    float GetBestGate( const FIntPoint& Chunk, const FIntPoint& NextChunk, const FIntPoint& Pos, FPathNode& OutNode );

    // -----------------tools-----------------
    // FVector2D NodeToVector( const FPathNode& Node );
    bool IsPosInChunk( const FIntPoint& Pos, const int32& VertexCount );
    bool IsPosInChunk( const FIntPoint& Pos );
    void GetNeighbors( const FIntPoint& Pos, TArray<FIntPoint>& OutNeighbors );
    void GetGoalSet( const FIntPoint& Chunk, const FIntPoint& NextChunk, const int32& VertexCount, TSet<FIntPoint>& OutGoalSet );
    void GetGoalSet( const FIntPoint& Chunk, const FIntPoint& NextChunk, TSet<FIntPoint>& OutGoalSet );
    int32 GetUnitDistSquared( const FIntPoint& PosA, const FIntPoint PosB );

};
