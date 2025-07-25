
#include "PathFinder2.h"
#include "PathNode.h"
#include "RuntimeTerrain.h"

#include <limits>
const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles

// this returns Path from Start to End
void FPathFinder::GetPath( const FPathNode& Start, const FPathNode& End, TArray<FPathNode>& OutPath )
{
    
}

void FPathFinder::FindPath( const FPathNode& Start, const FPathNode& End, TArray<FPathNode>& OutGates )
{

}

// returns Cost, OutPos is the gate we found. returns INFLOAT if no path.
float FPathFinder::GetBestGate( const FIntPoint& Chunk, const FIntPoint& NextChunk, const FIntPoint& StartPos, FPathNode& OutNode )
{
    // we do this only inside the specific chunk.
    // we just need FIntPoint for coordinate. (we know the chunk)

    // we'll make this min heap by float(priority).
    TArray< TPair<float, FIntPoint> > Frontier; 
    // min heap predicate
    auto Predicate = []( const TPair<float, FIntPoint>& A, const TPair<float, FIntPoint>& B ){ return A.Key < B.Key; };
    Frontier.Heapify( Predicate );
    Frontier.HeapPush( TPair<float, FIntPoint>( 0.f, StartPos ), Predicate );

    // Map for Cost
    TMap< FIntPoint, float > CostMap;
    CostMap.Emplace( StartPos , 0.f );

    // Goal set.
    TSet< FIntPoint > GoalSet;
    GetGoalSet(Chunk, NextChunk, GoalSet); // out param.

    while( !Frontier.IsEmpty() )
    {
        TPair<float, FIntPoint> Current;
        Frontier.HeapPop( Current, Predicate );
        
        FIntPoint PosNow = Current.Value;
        float CostNow = CostMap[PosNow]; // cost always in the map. add cost before pushing to the heap.

        // if we met goal, return.   ||  if we see INF on top of min heap, no path. return.
        if( GoalSet.Contains(PosNow) || CostNow >= INFLOAT )
        {
            OutNode = FPathNode(Chunk, NextChunk, PosNow);
            return CostNow;
        }

        
        TArray<FIntPoint> Neighbors;
        GetNeighbors(PosNow, Neighbors);
        for( auto& PosNext : Neighbors )
        {
            float NewCost = CostNow + GetUnitDistSquared( PosNow, PosNext );

            // TODO : Check SLOPES!!!

            float* OldCost = CostMap.Find( PosNext );
            if( !OldCost || *OldCost > NewCost )
            {
                CostMap.Add( PosNext, NewCost );
                float Priority = NewCost;           // TODO : + Heuristic!!!

                Frontier.HeapPush(TPair<float, FIntPoint>( Priority, PosNext ), Predicate);
            }
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("GetBestGate Failed. Path Not Found"));
    return INFLOAT;
}

// returns whether Pos is in chunk range. including boundaries.
bool FPathFinder::IsPosInChunk( const FIntPoint& Pos, const int32& VertexCount )
{
    return Pos.X < VertexCount && Pos.Y < VertexCount && Pos.X >= 0 && Pos.Y >= 0;
}

// simpler overload
bool FPathFinder::IsPosInChunk( const FIntPoint& Pos )
{
    return IsPosInChunk( Pos, this->RTref.VerticesPerChunk );
}

// returns neighbors of a Pos. 3x3 grid.
void FPathFinder::GetNeighbors( const FIntPoint& Pos, TArray<FIntPoint>& OutNeighbors )
{

    OutNeighbors.Empty();

    for( int32 j = Pos.Y -1; j <= Pos.Y + 1; j++ )
    {
        for( int32 i = Pos.X - 1; i <= Pos.X + 1; i++ )
        {
            if( FIntPoint( i,j ) == Pos || !IsPosInChunk( FIntPoint(i,j) ) ) 
            { continue; }; // ignore self and out of boundaries.
            OutNeighbors.Add( FIntPoint( i,j ) );
        }
    }

    return;
}

// returns GoalSet of Pos(gate to Next Chunk). VertexCount * VertexCount grid.
void FPathFinder::GetGoalSet( const FIntPoint& Chunk, const FIntPoint& NextChunk, const int32& VertexCount, TSet<FIntPoint>& OutGoalSet )
{
    OutGoalSet.Empty();
    FIntPoint Case = NextChunk - Chunk;
    // Case value ( depends on NextChunk Location )
	//(-1,1)	(0,1)	(1,1)
	//(-1,0)	S		(1,0)
	//(-1,-1)	(0,-1)	(1,-1)

    int32 LastIndex = VertexCount - 1;

    // diagonals first.
    if(         Case == FIntPoint(1, 1) )    {      OutGoalSet.Add( FIntPoint(LastIndex, LastIndex) );   }
    else if(    Case == FIntPoint(1, -1) )   {     OutGoalSet.Add( FIntPoint(LastIndex, 0) );            }
    else if(    Case == FIntPoint(-1, -1) )  {    OutGoalSet.Add( FIntPoint(0, 0) );                     }
    else if(    Case == FIntPoint(-1, 1) )   {     OutGoalSet.Add( FIntPoint(0, LastIndex) );            }
    // now sides.
    else if(    Case == FIntPoint(1, 0) )
    {
        for( int32 i = 0; i < VertexCount; i++ )
        {
            OutGoalSet.Add( FIntPoint(LastIndex, i) );
        }
    }
    else if(    Case == FIntPoint(-1, 0) )
    {
        for( int32 i = 0; i < VertexCount; i++ )
        {
            OutGoalSet.Add( FIntPoint(0, i) );
        }
    }
    else if(    Case == FIntPoint(0, 1) )
    {
        for( int32 i = 0; i < VertexCount; i++ )
        {
            OutGoalSet.Add( FIntPoint(i, LastIndex) );
        }
    }
    else if(    Case == FIntPoint(0, -1) )
    {
        for( int32 i = 0; i < VertexCount; i++ )
        {
            OutGoalSet.Add( FIntPoint(i, 0) );
        }
    }
    
    return;
}

// simpler overload
void FPathFinder::GetGoalSet( const FIntPoint& Chunk, const FIntPoint& NextChunk, TSet<FIntPoint>& OutGoalSet )
{
    GetGoalSet(Chunk, NextChunk, this->RTref.VerticesPerChunk, OutGoalSet );
    return;
}

// returns Unit Distance Squared
int32 FPathFinder::GetUnitDistSquared( const FIntPoint& PosA, const FIntPoint PosB )
{
    int32 DeltaX = PosB.X - PosA.X;
    int32 DeltaY = PosB.Y - PosA.Y;

    return DeltaX * DeltaX + DeltaY * DeltaY;
}