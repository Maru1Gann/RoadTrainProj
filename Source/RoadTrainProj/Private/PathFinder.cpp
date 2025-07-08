// Fill out your copyright notice in the Description page of Project Settings.


#include "PathFinder.h"
#include <limits>

const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles

bool FPathFinder::Init()
{
    // Should the thread start?
    return true;
}

uint32 FPathFinder::Run()
{
    // work on dedicated thread


    return 0;
}

void FPathFinder::Exit()
{
    // Post-Run code, threaded
}

void FPathFinder::Stop()
{
    // when Thread->Kill() called, Stop() goes off.
}



// declaration of struct for pathfinding
struct Node
{
	// 암시적 형변환 금지. (explicit)
	explicit Node();
	explicit Node(FVector2D Loc, float Priority) : Loc(Loc), Priority(Priority) {}

	FVector2D Loc;
	float Priority;

};

// declaration of operator for Heap
struct NodePredicate
{
	bool operator()(const Node& A, const Node& B) const
	{
		// true means B is prioritized.
		// hence this makes mean heap.
		return A.Priority > B.Priority;
	}
};


// Final return needed is TArray of FVector2D -> makes a path.
void FPathFinder::FindPath()
{

}


// Let's find path to one side
// and then return the first met gate (this will be our 'gate')
// Chunk == current chunk we traverse.
// will return Start if cannot traverse.
FVector2D FPathFinder::GetBestGate(const FIntPoint& Chunk, const FVector2D& Start, const TSet<FVector2D>& ChunkSide)
{
	// we do A*
	TArray<Node> Frontier; // this is Heap
	Frontier.Heapify( NodePredicate() );
	Frontier.HeapPush( Node( Start, 0.f ), NodePredicate() );
	
	TMap<FVector2D, float> CostMap;
	CostMap.Add( Start, 0.f );

	// we don't actually need the path.
	// we just need to figure out best gate.

	while( !Frontier.IsEmpty() )
	{
		Node Current;
		Frontier.HeapPop( Current, false ); // out param
		FVector CurrentLoc3D = ConvertTo3D(Current.Loc);

		// check if we've already been here.
		// if so, we do the process only if it's less costly.
		float* OldCost = CostMap.Find( Current.Loc );
		if( OldCost != nullptr && *OldCost + Heuristic(CurrentLoc3D) > Current.Priority )
		{
			continue;
		}

		// if frontier met the side of the chunk ( if met destination )
		if( ChunkSide.Contains( Current.Loc ) ) 
		{
			float* FinalCost = CostMap.Find( Current.Loc );
			if( FMath::IsFinite(*FinalCost) )
			{
				return Current.Loc;
			}
			else
			{
				// return Start since Cost == INF
				return Start;
			}
		}

		

		// for all neighbors (total 8, ignore '5'(current) )
		//	7	8	9
		//	4	cur	6
		//	1	2	3
		FVector2D Loc = Current.Loc;
		Loc.X -= VertexSpacing;
		Loc.Y -= VertexSpacing;
		
		for( int32 i = 0; i<3; i++ )
		{
			float StepY = i*VertexSpacing;

			for( int32 j = 0; j<3; j++ )
			{
				if( j == 1 && i == 1) // ignore current('5')
				{
					continue;
				}
				
				float StepX = j * VertexSpacing;

				Node Neighbor;
				Neighbor.Loc = FVector2D( Loc.X + StepX, Loc.Y + StepY );

				// check if out of Chunk Boundary
				if( !IsInBoundary( Chunk, Neighbor.Loc ) )
				{
					continue;
				}

				FVector NeighborLoc3D = ConvertTo3D(Neighbor.Loc);

				// check slope
				float NewCost = INFLOAT;
				if( GetSlopeSquared( CurrentLoc3D, NeighborLoc3D ) <= SlopeSquared )
				{
					NewCost = GetDistSquared( CurrentLoc3D, NeighborLoc3D );
				}
				

				float* OldCost = CostMap.Find( Neighbor.Loc );
				if( OldCost == nullptr || *OldCost > NewCost )
				{
					// TMap Add() replaces if overlaps
					CostMap.Add( Neighbor.Loc, NewCost );

					Neighbor.Priority = NewCost + Heuristic( NeighborLoc3D );
					Frontier.HeapPush( Neighbor, NodePredicate() );
				}

			}

		}

	} // end of while

	return Start;
}


// tools

float FPathFinder::GetHeight(const FVector2D& Location)
{
    	float height = 0.0f;

	if(NoiseLayers.Num() <= 0)
	{
		return 0.0f;
	}

	for ( int32 i = 0; i < NoiseLayers.Num(); i++)
	{
		float Frequency = NoiseLayers[i].Frequency;
		if(Frequency == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("Perlin Noise frequency can't be 0"));
			Frequency = 0.001;
		}

		float NoiseScale = 1.0f / Frequency;
		float Amplitude = NoiseLayers[i].Amplitude;
		float Offset = NoiseLayers[i].Offset;

		height += FMath::PerlinNoise2D(Location * NoiseScale + Offset) * Amplitude;
	}

	return height;
}
float FPathFinder::GetHeight(const FVector& Location)
{
	return GetHeight(FVector2D(Location.X, Location.Y));
}

float FPathFinder::GetDistSquared(const FVector& Start, const FVector& Dest)
{
	return FVector::DistSquared( Start , Dest );
}
float FPathFinder::GetDistSquared(const FVector2D& Start, const FVector2D& Dest)
{
	return FVector::DistSquared( ConvertTo3D(Start) , ConvertTo3D(Dest) );
}

FVector FPathFinder::ConvertTo3D(const FVector2D& Vector2D)
{
	return FVector( Vector2D.X, Vector2D.Y, GetHeight(Vector2D) );
}

// when Location is on the border exactly, returns previous chunk number.
// 나눠서 나오는 몫이 ChunkNumber 가 됌. 나머지는 무시 (floor). -> 0 과 1번 청크의 정확히 경계션에 있는 경우 0 반환.
FIntPoint FPathFinder::GetChunk(const FVector2D& Location)
{
	return FIntPoint( FMath::FloorToInt32(Location.X / ChunkLength), FMath::FloorToInt32(Location.Y / ChunkLength) );
}

float FPathFinder::Heuristic(const FVector2D& Current)
{
	return GetDistSquared(Current, this->End);
}
float FPathFinder::Heuristic(const FVector& Current)
{
	return GetDistSquared(Current, this->End3D);
}


// returns slope %, squared
float FPathFinder::GetSlopeSquared(const FVector& Current, const FVector& Next)
{
	// get tangent and multiply 100.

	
	float Base = FVector2D::DistSquared( FVector2D(Current.X, Current.Y), FVector2D(Next.X, Next.Y) ); // 밑변. base
	float Height = Current.Z - Next.Z;
	Height *= Height; // square

	return ( Height / Base ) * 100.f * 100.f;

}

bool FPathFinder::IsInBoundary(const FIntPoint &Chunk, const FVector2D &Pos)
{
	FVector2D Offset = FVector2D(Chunk.X, Chunk.Y) * ChunkLength;

	float SmallValue = 0.001f;
	bool bInX = (Pos.X >= Offset.X - SmallValue ) && ( Pos.X <=  Offset.X + ChunkLength + SmallValue );
	bool bInY = (Pos.Y >= Offset.Y - SmallValue ) && ( Pos.Y <=  Offset.Y + ChunkLength + SmallValue );

	return bInX && bInY;
}
