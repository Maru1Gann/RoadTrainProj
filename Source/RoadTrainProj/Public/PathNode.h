
#pragma once

#include "CoreMinimal.h"


// this Node is a position of a Vertex in worldspace.
struct FPathNode
{
    FPathNode(){}
    FPathNode( const FIntPoint& Chunk, const FIntPoint& Pos ) : Chunk(Chunk), Pos(Pos) {};

    FIntPoint Chunk;
    FIntPoint Pos;

    bool operator==( const FPathNode& Other ) const
    {
        return Chunk == Other.Chunk && Pos == Other.Pos;
    }
};

FORCEINLINE uint32 GetTypeHash( const FPathNode& Key )
{
    return HashCombineFast( GetTypeHash( Key.Chunk ), GetTypeHash( Key.Pos ) );
}