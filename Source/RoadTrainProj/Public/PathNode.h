
#pragma once

#include "CoreMinimal.h"


// this Node is a position of a Vertex in worldspace.
struct FPathNode
{
    
    FPathNode(){}
    FPathNode( const FIntPoint& Belong, const FIntPoint& Next, const FIntPoint& Pos ) : Belong(Belong), Next(Next), Pos(Pos) {};

    FIntPoint Belong;   // chunk it belongs
    FIntPoint Next;     // chunk it's headed ( if it is gate )
    FIntPoint Pos;      // Coordinate inside the chunk. ( * VertexSpacing )

    bool operator==( const FPathNode& Other ) const
    {
        return Belong == Other.Belong && Next == Other.Next && Pos == Other.Pos;
    }
};

FORCEINLINE uint32 GetTypeHash( const FPathNode& Key )
{
    uint32 Hash = HashCombine( GetTypeHash( Key.Belong ), GetTypeHash( Key.Next ) );
    Hash = HashCombine( Hash, GetTypeHash( Key.Pos ) );
    return Hash;
}