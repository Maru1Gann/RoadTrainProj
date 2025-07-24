
#pragma once

#include "CoreMinimal.h"

struct FPathNode;

class FPathFinder
{

public:
    FPathFinder();

    
    void GetBestGate( const FIntPoint& Chunk, const FPathnode& Start, const TSet<FPathNode>& ChunkSide );
    void GetPath( const FPathNode& Start, const FPathNode& End, TArray<FPathNode>& OutPath );


private:


    // -----------------tools-----------------
    // FVector2D NodeToVector( const FPathNode& Node );

};