
#include "PathFinder2.h"
#include "PathNode.h"
#include "RuntimeTerrain.h"

#include <limits>
const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles

FPathFinder::FPathFinder( ARuntimeTerrain& RTref ) : RTref(RTref)
{

}

// this returns Path from Start to End ( should be the exact next gate )
float FPathFinder::GetPath( const FPathNode& Start, const FPathNode& End, TArray<FIntPoint>& OutPath )
{
    if( Start.Next != End.Belong )
    {
        UE_LOG(LogTemp, Warning, TEXT("GetPath Input wrong nodes"));
        return INFLOAT;
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
            return CostNow;
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
                { UE_LOG(LogTemp, Warning, TEXT("Came_From nullptr error")); return INFLOAT; }
                PathPos = *PathFrom;
                ReversePath.Emplace( PathPos );
            }
            // reverse the ReversePath
            OutPath.SetNum( ReversePath.Num() );
            for( int32 i = 0; i < ReversePath.Num(); i++ )
            {
                OutPath[i] = ReversePath[ ReversePath.Num() - 1 - i];
            }

            return CostNow;
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
    } // end of while

    return INFLOAT;
}

// this returns all gates on the path. Chunk level PathFinding. Start, End are Global grid FIntPoint
void FPathFinder::FindPathGates( const FIntPoint& Start, const FIntPoint& End, TArray<FPathNode>& OutGates )
{
    OutGates.Empty();

    FIntPoint StartChunk = GetChunk(Start);
    FIntPoint EndChunk = GetChunk(End);

    FIntPoint StartPos = GlobalToLocal( StartChunk, Start );
    FIntPoint EndPos = GlobalToLocal( EndChunk, End );

    FPathNode StartNode( StartChunk, StartChunk, StartPos );
    FPathNode EndNode( EndChunk, EndChunk, EndPos );

    // similar A*. check GetBestGate for detailed comments
    TArray< TPair<float, FPathNode> > Frontier;
    auto Predicate = []( const TPair<float, FPathNode>& A, const TPair<float, FPathNode>& B ){ return A.Key < B.Key; };
    Frontier.Heapify( Predicate ); // don't mean anything if it's empty. context purpose.
    Frontier.HeapPush(TPair<float, FPathNode>(0.f, StartNode), Predicate);

    TMap<FIntPoint, float> CostMap; // CostMap for Chunk.
    CostMap.Add( StartChunk, 0.f );

    TMap<FPathNode, FPathNode> Came_From; // for gate graph. !! Same Position Node aren't same if headed chunk is different !!
    
    while( !Frontier.IsEmpty() )
    {
        UE_LOG(LogTemp, Warning, TEXT("ChunkLevel While"));

        TPair< float, FPathNode > Current;
        Frontier.HeapPop( Current, Predicate );
        FPathNode CurrentNode = Current.Value;

        float CostNow = CostMap[ CurrentNode.Next ];

        if( CostNow >= INFLOAT )    // No Path Found ( INF is lowest cost )
        { UE_LOG(LogTemp, Warning, TEXT("No ChunkLevel Path Found.")); return; } 

        if( CurrentNode.Next == EndNode.Belong )  // if we met goal    
        {
            // TODO: check if goal is reachable!! 
            
            // .. do data passing.
            TArray<FPathNode> ReversePath;
            ReversePath.Add( EndNode );

            FPathNode NodeNow = CurrentNode;
            ReversePath.Add( NodeNow );

            while( NodeNow != StartNode )
            {
                FPathNode* NodeFrom = Came_From.Find( NodeNow );
                if( !NodeFrom )
                { UE_LOG(LogTemp, Warning, TEXT("HighLevelPath Error")); return; }
                NodeNow = *NodeFrom;
                ReversePath.Add( NodeNow );
            }

            OutGates.SetNum( ReversePath.Num() );
            for( int32 i = 0; i < ReversePath.Num(); i++ )
            {
                OutGates[i] = ReversePath[ ReversePath.Num() - 1 - i ];
            }

            return;
        }

        // traversing
        TArray<FIntPoint> Neighbors;
        GetNeighbors( CurrentNode.Next, Neighbors, true );
        for( auto& Neighbor : Neighbors )   // for each neighbor
        {
            FPathNode Gate;
            // we have to change pos to next chunk relative
            FIntPoint NewPos = GlobalToLocal( CurrentNode.Next, ConvertToGlobal(CurrentNode.Belong, CurrentNode.Pos) );
            float CostToGate = GetBestGate( CurrentNode.Next, Neighbor, NewPos, Gate ); 
            if( CostToGate >= INFLOAT ) { continue; }   // No gate available
            
            float NewCost = CostNow + CostToGate;
            float* OldCost = CostMap.Find( Neighbor );
            if( !OldCost || *OldCost > NewCost )
            {
                CostMap.Add( Neighbor, NewCost );
                Came_From.Add( Gate, CurrentNode );

                float Priority = NewCost + Heuristic( ConvertToGlobal( Gate.Belong, Gate.Pos ), End );

                UE_LOG(LogTemp, Display, TEXT("Priority: %f"), Priority);

                // Solve same priority conflict (on diagonal gates) with direction priority.
                Priority -= GetDirectionBias( Gate, End ) * 10;

                //UE_LOG(LogTemp, Display, TEXT("Priority + bias: %f"), Priority);

                Frontier.HeapPush( TPair<float, FPathNode>( Priority, Gate ), Predicate );
            }
        }

    }   // end of while

    UE_LOG(LogTemp, Warning, TEXT("ChunkLevel PathFinding Error"));
    return;

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
void FPathFinder::GetNeighbors( const FIntPoint& Pos, TArray<FIntPoint>& OutNeighbors, bool IsGlobal )
{

    OutNeighbors.Empty();

    for( int32 j = Pos.Y -1; j <= Pos.Y + 1; j++ )
    {
        for( int32 i = Pos.X - 1; i <= Pos.X + 1; i++ )
        {
            if( FIntPoint( i,j ) == Pos ) 
            { continue; }; // ignore self and out of boundaries.
            if( !IsGlobal && !IsPosInChunk( FIntPoint(i,j) ) )
            { continue; }; // if not global, check if Pos is in Chunk
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
    // UE_LOG(LogTemp, Display, TEXT("Case : %s "), *Case.ToString());
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

// returns global FIntPoint grid Position
FIntPoint FPathFinder::ConvertToGlobal( const FIntPoint& Chunk, const FIntPoint& Pos )
{
    FIntPoint Offset = Chunk * RTref.VerticesPerChunk;

    return Offset + Pos;
}

// returns Pos relative to Chunk
FIntPoint FPathFinder::GlobalToLocal( const FIntPoint& Chunk, const FIntPoint& GlobalPos )
{
    FIntPoint ChunkGlobalPos = Chunk * RTref.VerticesPerChunk;
    return GlobalPos - ChunkGlobalPos;
}

float FPathFinder::GetDirectionBias( const FPathNode& Gate, const FIntPoint& GlobalEnd )
{
    FIntPoint GlobalPos = ConvertToGlobal( Gate.Belong, Gate.Pos );
    //TODO: slight vector angle priority for diagonal situations. (same point, same priority problem)
    FVector2D ToGoal = FVector2D( GlobalEnd.X, GlobalEnd.Y ) - FVector2D( GlobalPos.X, GlobalPos.Y );
    FVector2D ToNext = FVector2D( Gate.Next.X, Gate.Next.Y ) - FVector2D( Gate.Belong.X, Gate.Belong.Y );
    if( ToGoal.IsNearlyZero() || ToNext.IsNearlyZero() ) { return 0.f; }    // error check.
    ToGoal.Normalize();
    ToNext.Normalize();

    float DotResult = FVector2D::DotProduct(ToGoal, ToNext);
    // Result is close to 1 if angle is small. -1 ~ 0 ~ 1
    // 같은 방향 -> 1 에 가까움. 반대 방향 -> -1. 수직 방향 -> 0

    return DotResult;
}

FIntPoint FPathFinder::GetChunk( const FIntPoint& GlobalPos )
{
    FIntPoint Chunk;
    Chunk.X = FMath::FloorToInt32( float(GlobalPos.X) / float(RTref.VerticesPerChunk) );
    Chunk.Y = FMath::FloorToInt32( float(GlobalPos.Y) / float(RTref.VerticesPerChunk) );

    return Chunk;
}