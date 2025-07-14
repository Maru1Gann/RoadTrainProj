// Fill out your copyright notice in the Description page of Project Settings.


#include "PathFinder.h"
#include "RMCLandscape.h"
#include "PerlinNoiseVariables.h"
#include <limits>

#include "Async/Async.h" // for Exit(). check the func.

const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles

// struct for high level (Chunk level) pathfinding
struct ChunkNode
{
	explicit ChunkNode() {}
	explicit ChunkNode(FIntPoint Chunk, FVector2D Loc, float Priority) 
	: Chunk(Chunk), Loc(Loc), Priority(Priority) {} 

	FIntPoint Chunk;
	FVector2D Loc;
	float Priority;
};
struct ChunkNodePredicate
{
	bool operator() (const ChunkNode& A, const ChunkNode& B) const
	{
		return A.Priority < B.Priority;
	}
};
// declaration of struct for gate pathfinding
struct Node
{
	// 암시적 형변환 금지. (explicit)
	explicit Node() {}
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
		return A.Priority < B.Priority;
	}
};

// -------------------temporary struct declaration-------------------

bool FPathFinder::Init()
{
    // Should the thread start?
    return true;
}

uint32 FPathFinder::Run()
{
    // work on dedicated thread
	// FindPath();
	// RMCLandscape->SetPath(this->Path);

	// TSet<FVector2D> Test = GetDestSide( GetChunk(Begin), GetChunk(End) );
	// for (auto& Elem : Test)
	// {
	// 	UE_LOG(LogTemp, Display, TEXT("Dest : %s"), *Elem.ToString() );
	// }
	
	Node Temp = GetBestGate( GetChunk(Begin), this->Begin, GetDestSide( GetChunk(Begin), GetChunk(End) ) );
	UE_LOG(LogTemp, Display, TEXT("Node pos %s"), *Temp.Loc.ToString());

	RMCLandscape->SetPath(Path);

	UE_LOG(LogTemp, Display, TEXT("PathFinding Done!!!"));

    return 0;
}

void FPathFinder::Exit()
{
    // Post-Run code, threaded

	AsyncTask(ENamedThreads::GameThread, 
		[this]()
		{
			this->RMCLandscape->DrawPathDebug();
		}
	);

}

void FPathFinder::Stop()
{
    // when Thread->Kill() called, Stop() goes off.
}

FPathFinder::FPathFinder( ARMCLandscape* RMCLandscape, const FVector2D& Begin, const FVector2D& End, const float& Slope )
{
	this->RMCLandscape = RMCLandscape;
	this->Begin = Begin;
	this->End = End;
	this->SlopeSquared = Slope*Slope;
	this->VertexSpacing = RMCLandscape->VertexSpacing;
	this->VerticesPerChunk = RMCLandscape->VerticesPerChunk;
	this->NoiseLayers = RMCLandscape->PerlinNoiseLayers;

	End3D = ConvertTo3D(End);
	ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;
	Thread = FRunnableThread::Create(this, TEXT("PathFinder"));
}

// Final return needed is TArray of FVector2D -> makes a path.
void FPathFinder::FindPath()
{
	// from this->Begin to this->End
	// Begin - Gates - ... - Gates - End
	// we'll store this in TArray<FVector2D> Path (member var)
	this->Path.Empty();

	FIntPoint BeginChunk = GetChunk(this->Begin);
	FIntPoint EndChunk = GetChunk(this->End);
	UE_LOG(LogTemp, Display, TEXT("BeginChunk : %d, %d"), BeginChunk.X, BeginChunk.Y);
	UE_LOG(LogTemp, Display, TEXT("EndChunk : %d, %d"), EndChunk.X, EndChunk.Y);

	TArray<ChunkNode> Frontier; // this is mean Heap.
	Frontier.Heapify( ChunkNodePredicate() );
	Frontier.HeapPush( ChunkNode( BeginChunk, Begin, 0.f), ChunkNodePredicate() );

	// this is our map for cost.
	TMap<FIntPoint, float> CostMap;
	CostMap.Add( BeginChunk, 0.f );

	// this is our map for path
	TMap<FVector2D, FVector2D> Came_from;

	while( !Frontier.IsEmpty() )
	{
		UE_LOG(LogTemp, Display, TEXT("HighLevel while"));

		ChunkNode Current;
		Frontier.HeapPop( Current, ChunkNodePredicate() );

		// check if we've already been here.
		// if so, we do the iteration only if it's less costly.
		float* OldCost = CostMap.Find( Current.Chunk );
		if( OldCost != nullptr && *OldCost + Heuristic( ConvertTo3D(Current.Loc) ) < Current.Priority )
		{
			UE_LOG(LogTemp, Display, TEXT("H continue"));

			continue;
		}

		// we met goal chunk
		if( Current.Chunk == EndChunk )
		{
			UE_LOG(LogTemp, Display, TEXT("H met goal chunk"));

			// do the value passing
			FVector2D Gate = Current.Loc;
			this->Path.Add( Gate );
			
			while( Gate != this->Begin )
			{
				FVector2D* GateFrom = Came_from.Find(Gate);
				
				if( GateFrom == nullptr ) // error check
				{
					UE_LOG(LogTemp, Warning, TEXT(" Came_from nullptr, path error "));
				}

				this->Path.Add( *GateFrom );
			}

			this->Path.Add( this->Begin ); // path is made backwards.
			return;
		}

		// we didn't meet goal chunk.
		// 3x3 plane, widen frontier
		for( int32 i = 0; i<3; i++ )
		{
			for (int32 j = 0; j<3; j++ )
			{
				if( i == 1 && j == 1) // when hit self.
				{
					continue;
				}

				FIntPoint NextChunk = Current.Chunk - FIntPoint(1,1) + FIntPoint( i, j );
				UE_LOG(LogTemp, Display, TEXT(" NextChunk %s"), *NextChunk.ToString());

				// Tmp.priority == 'cost' ( for return of GetBestGate )
				Node Tmp = GetBestGate( Current.Chunk, Current.Loc, GetDestSide( Current.Chunk, NextChunk) );
				ChunkNode Next = ChunkNode( NextChunk, Tmp.Loc, INFLOAT );

				// add only if we haven't been to NextChunk, or it's less costly.
				OldCost = CostMap.Find( Next.Chunk );
				if( OldCost == nullptr || *OldCost > Tmp.Priority )
				{
					CostMap.Add( Next.Chunk, Tmp.Priority );
					Next.Priority = Tmp.Priority + Heuristic(Next.Loc);

					Came_from.Add( Next.Loc, Current.Loc );
					Frontier.HeapPush( Next, ChunkNodePredicate() );
				}

			}
		}

	} // end of while

	UE_LOG(LogTemp, Warning, TEXT(" H level while ended "));
	return;
}

// Let's find path to one side
// and then return the first met gate (this will be our 'gate')
// Chunk == current chunk we traverse.
// will return Start if cannot traverse.
Node FPathFinder::GetBestGate(const FIntPoint& Chunk, const FVector2D& Start, const TSet<FVector2D>& ChunkSide)
{
	if( ChunkSide.IsEmpty() )
	{
		UE_LOG(LogTemp, Warning, TEXT(" ChunkSide Empty "));
		return Node();
	}
	// we do A*
	TArray<Node> Frontier; // this is mean Heap
	Frontier.Heapify( NodePredicate() );
	Frontier.HeapPush( Node( Start, 0.f ), NodePredicate() );
	
	TMap<FVector2D, float> CostMap;
	CostMap.Add( Start, 0.f );

	// we don't actually need the path.
	// we just need to figure out best gate.

	while( !Frontier.IsEmpty() )
	{

		Node Current;
		Frontier.HeapPop( Current, NodePredicate() ); // out param
		FVector CurrentLoc3D = ConvertTo3D(Current.Loc);

		// check if we've already been here.
		// if so, we do the iteration only if it's less costly.
		float* OldCost = CostMap.Find( Current.Loc );
		if( OldCost != nullptr && *OldCost + Heuristic(CurrentLoc3D) < Current.Priority )
		{
			continue;
		}


		// DEBUGGING@@@@@!!!!!!!
		UE_LOG(LogTemp, Display, TEXT("Priority : %f "), Current.Priority);
		Path.Add(Current.Loc);
		// DEBUGGING@@@@@!!!!!!!

		// if frontier met the side of the chunk ( if met destination )
		if( ChunkSide.Contains( Current.Loc ) ) 
		{
			float* FinalCost = CostMap.Find( Current.Loc );
			// if cost != INFLOAT, return found gate.
			if( FMath::IsFinite(*FinalCost) )
			{
				// put 'cost' in 'priority' for return
				return Node(Current.Loc, *FinalCost);
			}
			else // return Start since Cost == INF ( no route )
			{
				return Node(Start, INFLOAT);
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
				// ignore cur('5')
				if( j == 1 && i == 1) 
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

				float NewCost = INFLOAT;
				FVector NeighborLoc3D = ConvertTo3D(Neighbor.Loc);

				// check slope
				if( GetSlopeSquared( CurrentLoc3D, NeighborLoc3D ) <= SlopeSquared )
				{
					NewCost = GetDistSquared( CurrentLoc3D, NeighborLoc3D );
				}
				
				// check if it's new or less costly. (replaceable)
				OldCost = CostMap.Find( Neighbor.Loc );
				if( OldCost == nullptr || *OldCost > NewCost )
				{
					// TMap Add() replaces if overlaps
					CostMap.Add( Neighbor.Loc, NewCost );

					Neighbor.Priority = NewCost + Heuristic( NeighborLoc3D );
					Frontier.HeapPush( Neighbor, NodePredicate() );
				}

			}
		} // end of for loop

	} // end of while

	return Node(Start, INFLOAT);
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

// returns set of destination chunk's side vertices.
// 8 cases total
TSet<FVector2D> FPathFinder::GetDestSide(const FIntPoint& StartChunk, const FIntPoint& DestChunk)
{
	TSet<FVector2D> ChunkSide;
	FVector2D Offset = FVector2D( StartChunk.X , StartChunk.Y ) * ChunkLength;

	FIntPoint Case = DestChunk - StartChunk;
	// Case value ( depends on DestChunk Location )
	//(-1,1)	(0,1)	(1,1)
	//(-1,0)	S		(1,0)
	//(-1,-1)	(0,-1)	(1,-1)

	if( Case.X > 1 || Case.X <-1 || 
		Case.Y > 1 || Case.Y < -1 || StartChunk == DestChunk )
	{
		ChunkSide.Empty(); // this is error.
		return ChunkSide;
	}
	else if( Case == FIntPoint(1,1) ) // Diagonal
	{
		ChunkSide.Add( Offset + FVector2D( ChunkLength, ChunkLength ) );
	}
	else if( Case == FIntPoint(1,0) )
	{
		for(int32 i = 0 ; i < VerticesPerChunk; i++)
		{
			FVector2D Side = FVector2D( ChunkLength, VertexSpacing * i );
			ChunkSide.Add( Offset + Side );
		}
	}
	else if( Case == FIntPoint(1,-1) ) // Diagonal
	{
		ChunkSide.Add( Offset + FVector2D( ChunkLength, 0.f ) );
	}
	else if( Case == FIntPoint(0,1) )
	{
		for(int32 i = 0 ; i < VerticesPerChunk; i++)
		{
			FVector2D Side = FVector2D( VertexSpacing * i, ChunkLength );
			ChunkSide.Add( Offset + Side );
		}
	}
	else if( Case == FIntPoint(0,-1) )
	{
		for(int32 i = 0 ; i < VerticesPerChunk; i++)
		{
			FVector2D Side = FVector2D( VertexSpacing * i, 0.f );
			ChunkSide.Add( Offset + Side );
		}
	}
	else if( Case == FIntPoint(-1,1) ) // Diagonal
	{
		ChunkSide.Add( Offset + FVector2D( -ChunkLength, ChunkLength ) );
	}
	else if( Case == FIntPoint(-1,0) )
	{
		for(int32 i = 0 ; i < VerticesPerChunk; i++)
		{
			FVector2D Side = FVector2D( 0.f , VertexSpacing * i );
			ChunkSide.Add( Offset + Side );
		}
	}
	else if( Case == FIntPoint(-1,-1) ) // Diagonal
	{
		ChunkSide.Add( Offset + FVector2D( -ChunkLength, -ChunkLength ) );
	}


	return ChunkSide;
}