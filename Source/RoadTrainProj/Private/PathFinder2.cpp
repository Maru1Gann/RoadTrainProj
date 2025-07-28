
#include "PathFinder2.h"
#include "PathNode.h"
#include "RuntimeTerrain.h"

#include <limits>
const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles

FPathFinder::FPathFinder( ARuntimeTerrain& RTref ) : RTref(RTref)
{

}

// this returns Path from Start to End ( should be the exact next gate )
void FPathFinder::GetPath( const FPathNode& Start, const FPathNode& End, TArray<FIntPoint>& OutPath )
{
    if( Start.Next != End.Belong )
    {
        UE_LOG(LogTemp, Warning, TEXT("GetPath Input wrong nodes"));
        return;
    }

    OutPath.Empty();

    // change the Pos to the chunk it needs to be.
    FIntPoint StartPos = ConvertToGlobal( Start.Belong, Start.Pos );
    StartPos = GlobalToLocal( End.Belong, StartPos );

    // this works almost same as GetBestGate, A*. check it out for detailed comments
    TArray< TPair< float, FIntPoint> > Frontier;
    auto Predicate = [](const TPair<float, FIntPoint>& A, const TPair<float, FIntPoint>& B){ return A.Key < B.Key; }; // min heap
    Frontier.Heapify( Predicate );
    Frontier.HeapPush( TPair<float, FIntPoint>( 0.f, StartPos ), Predicate );

    TMap< FIntPoint, float > CostMap;
    CostMap.Emplace( StartPos, 0.f );

    TMap< FIntPoint, FIntPoint > Came_From;
    TArray<FIntPoint> ReversePath;

    while( !Frontier.IsEmpty() )
    {
        TPair<float, FIntPoint> Current;
        Frontier.HeapPop( Current, Predicate );

        FIntPoint PosNow = Current.Value;
        float CostNow = CostMap[PosNow];
        if( CostNow >= INFLOAT )
        {
            UE_LOG(LogTemp, Warning, TEXT("Path Rebuilding Failed... How??"));
            return;
        }
        if( PosNow == End.Pos )
        {
            //.. do data passing
            FIntPoint PathPos = End.Pos;
            ReversePath.Emplace(PathPos);

            while( PathPos != StartPos )
            {
                FIntPoint* PathFrom = Came_From.Find( PathPos );
                if( !PathFrom ) 
                { UE_LOG(LogTemp, Warning, TEXT("Came_From nullptr error")); return; }
                PathPos = *PathFrom;
                ReversePath.Emplace( PathPos );
            }
            // reverse the ReversePath
            OutPath.SetNum( ReversePath.Num() );
            for( int32 i = 0; i < ReversePath.Num(); i++ )
            {
                OutPath[i] = ReversePath[ ReversePath.Num() - 1 - i];
            }

            return;
        }

        TArray<FIntPoint> Neighbors;
        GetNeighbors( PosNow, Neighbors );
        for( auto& PosNext : Neighbors )
        {
            float NewCost = CostNow + GetUnitDistSquared( PosNow, PosNext );

            // check slopes
            if( GetSlopeSquared( End.Belong, PosNow, PosNext ) > RTref.Slope * RTref.Slope )
            {
                NewCost = INFLOAT;
            }

            float* OldCost = CostMap.Find( PosNext );
            if( !OldCost || *OldCost > NewCost )
            {
                CostMap.Add( PosNext, NewCost );
                Came_From.Add( PosNext, PosNow );
                
                float Priority = NewCost + GetUnitDistSquared( PosNow, PosNext ); // use instead of heuristic()
                Frontier.HeapPush( TPair<float, FIntPoint>( Priority, PosNext ), Predicate );
            }
        }
    }

}

// this returns all gates on the path. Chunk level PathFinding.
void FPathFinder::FindPathGates( const FPathNode& Start, const FPathNode& End, TArray<FPathNode>& OutGates )
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
    auto Predicate = []( const TPair<float, FIntPoint>& A, const TPair<float, FIntPoint>& B ){ return A.Key < B.Key; }; // function ptr
    Frontier.Heapify( Predicate );
    Frontier.HeapPush( TPair<float, FIntPoint>( 0.f, StartPos ), Predicate );

    // Map for Cost
    TMap< FIntPoint, float > CostMap;
    CostMap.Emplace( StartPos , 0.f );

    // Goal set.
    TSet< FIntPoint > GoalSet;
    GetGoalSet(Chunk, NextChunk, GoalSet); // out param.
    if( GoalSet.IsEmpty() )
    {
        UE_LOG(LogTemp, Warning, TEXT("GoalSetEmpty"));
        return INFLOAT;
    }

    while( !Frontier.IsEmpty() )
    {
        TPair<float, FIntPoint> Current;
        Frontier.HeapPop( Current, Predicate );
        
        FIntPoint PosNow = Current.Value;
        float CostNow = CostMap[PosNow]; // cost always in the map. add cost before pushing to the heap.

        // if we met goal, return.   ||  if we see INF on top of min heap, no path. return.
        if( GoalSet.Contains( PosNow ) || CostNow >= INFLOAT )
        {
            OutNode = FPathNode(Chunk, NextChunk, PosNow);
            return CostNow;
        }
        
        TArray<FIntPoint> Neighbors;
        GetNeighbors( PosNow, Neighbors );
        for( auto& PosNext : Neighbors )
        {
            float NewCost = CostNow + GetUnitDistSquared( PosNow, PosNext );

            // check slopes
            if( GetSlopeSquared( Chunk, PosNow, PosNext ) > RTref.Slope * RTref.Slope )
            {
                NewCost = INFLOAT; // too steep
            }

            float* OldCost = CostMap.Find( PosNext );
            if( !OldCost || *OldCost > NewCost )
            {
                CostMap.Add( PosNext, NewCost );
                float Priority = NewCost + Heuristic( ConvertToGlobal( Chunk, PosNext ), RTref.End ); // CHECK : direction not accounted!!

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
    UE_LOG(LogTemp, Display, TEXT("Case : %s "), *Case.ToString());
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

// returns slope %, squared
float FPathFinder::GetSlopeSquared( const FIntPoint& Chunk, const FIntPoint& PosA, const FIntPoint& PosB )
{
    // get tangent and multiply 100.

    float UnitBase = GetUnitDistSquared( PosA, PosB );
    float UnitHeight = RTref.GetHeight( ConvertToVector2D( Chunk, PosA ) ) - RTref.GetHeight( ConvertToVector2D( Chunk, PosB ) );
    UnitHeight /= RTref.VertexSpacing;// Convert to UnitHeight
    UnitHeight *= UnitHeight; // square
    
    return ( UnitHeight / UnitBase ) * 100.f * 100.f ;
}

FVector2D FPathFinder::ConvertToVector2D( const FIntPoint& Chunk, const FIntPoint& Pos )
{
    float ChunkLength = RTref.VertexSpacing * ( RTref.VerticesPerChunk-1 );
    FVector2D Offset = FVector2D( Chunk.X, Chunk.Y ) * ChunkLength;

    return FVector2D( Pos.X, Pos.Y ) * RTref.VertexSpacing + Offset;
}

int32 FPathFinder::Heuristic( const FIntPoint& PosA, const FIntPoint& PosB )
{
    return GetUnitDistSquared( PosA, PosB );
}

FIntPoint FPathFinder::ConvertToGlobal( const FIntPoint& Chunk, const FIntPoint& Pos )
{
    FIntPoint Offset = Chunk * RTref.VerticesPerChunk;

    return Offset + Pos;
}

FIntPoint FPathFinder::GlobalToLocal( const FIntPoint& Chunk, const FIntPoint& GlobalPos )
{
    FIntPoint ChunkGlobalPos = Chunk * RTref.VerticesPerChunk;
    return GlobalPos - ChunkGlobalPos;
}