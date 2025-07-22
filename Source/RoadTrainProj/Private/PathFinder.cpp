// Fill out your copyright notice in the Description page of Project Settings.


#include "PathFinder.h"
#include "RMCLandscape.h"
#include "PerlinNoiseVariables.h"
#include <limits>

#include "Async/Async.h" // for Exit(). check the func.

const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles

// struct for high level (Chunk level) pathfinding
struct FChunkNode
{
	explicit FChunkNode() {}
	explicit FChunkNode(FIntPoint Chunk, FVector2D Loc, float Priority) 
	: Chunk(Chunk), Loc(Loc), Priority(Priority) {} 

	FIntPoint Chunk;
	FVector2D Loc;
	float Priority;

	bool operator==(const FChunkNode& Other) const
	{
		return Chunk == Other.Chunk && Loc == Other.Loc;
	}
	bool operator!=(const FChunkNode& Other) const
	{
		return !( *this == Other );
	}
};
FORCEINLINE uint32 GetTypeHash(const FChunkNode& Key)
{
	// convert to int. To avoid floating point errors making hash difference
	FIntPoint LocInt = FIntPoint(FMath::RoundToInt32(Key.Loc.X), FMath::RoundToInt32(Key.Loc.Y));
	return HashCombineFast( GetTypeHash(Key.Chunk), GetTypeHash(LocInt) );
}
struct FChunkNodePredicate
{
	bool operator() (const FChunkNode& A, const FChunkNode& B) const
	{
		return A.Priority < B.Priority;
	}
};
// declaration of struct for gate pathfinding
struct FNode
{
	// 암시적 형변환 금지. (explicit)
	explicit FNode() {}
	explicit FNode(FVector2D Loc, float Priority) : Loc(Loc), Priority(Priority) {}

	FVector2D Loc;
	float Priority;

};
// declaration of operator for Heap
struct FNodePredicate
{
	bool operator()(const FNode& A, const FNode& B) const
	{
		// this makes min heap.
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

	FindPath();
	//GetBestGate( GetChunk(Begin), this->Begin, GetDestSide( GetChunk(Begin), GetChunk(End) ) );

	RMCLandscape->SetPath(this->Path);
	UE_LOG(LogTemp, Display, TEXT("PathFinding Done!!!"));


    return 0;
}

void FPathFinder::Exit()
{
    // Post-Run code, threaded

	AsyncTask(ENamedThreads::GameThread, 
		[this]()
		{
			this->RMCLandscape->RunAfterPathFinding();
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
	this->ShouldGenerateHeight = RMCLandscape->ShouldGenerateHeight;

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
	//this->Path.Empty();

	FIntPoint BeginChunk = GetChunk(this->Begin);
	FIntPoint EndChunk = GetChunk(this->End);

	UE_LOG(LogTemp, Display, TEXT("BeginChunk : %d, %d"), BeginChunk.X, BeginChunk.Y);
	UE_LOG(LogTemp, Display, TEXT("EndChunk : %d, %d"), EndChunk.X, EndChunk.Y);

	TArray<FChunkNode> Frontier; // this is mean Heap.
	Frontier.Heapify( FChunkNodePredicate() );
	Frontier.HeapPush( FChunkNode( BeginChunk, this->Begin, 0.f), FChunkNodePredicate() );

	// this is our map for cost.
	TMap<FIntPoint, float> CostMap;
	CostMap.Add( BeginChunk, 0.f );

	// this is our map for path
	TMap<FChunkNode, FChunkNode> Came_from;
	FChunkNode BeginNode = FChunkNode(BeginChunk, this->Begin, 0.f);
	Came_from.Add(BeginNode, BeginNode);

	while( !Frontier.IsEmpty() )
	{
		UE_LOG(LogTemp, Display, TEXT("HighLevel while"));

		FChunkNode Current;
		Frontier.HeapPop( Current, FChunkNodePredicate() );

		if( Current.Priority >= INFLOAT ) // this means no path. (lowest cost == INF)
		{
			UE_LOG(LogTemp, Warning, TEXT("High level hit INF, no path"));
			return;
		}

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
			
			// we need to check if we have path to 'End' from 'Current.Loc'
			FNode Temp = GetBestGate(Current.Chunk, Current.Loc, TSet<FVector2D>{this->End} );
			
			if( Temp.Priority < INFLOAT )
			{
				// do the value passing
				this->Path.Empty();

				this->Path.Emplace( EndChunk, End );
				FChunkNode Gate = Current;
				this->Path.Emplace( Gate.Chunk, Gate.Loc );

				// UE_LOG(LogTemp, Display, TEXT("Gate. Chunk:%s Loc:%s "), *Gate.Chunk.ToString(), *Gate.Loc.ToString());

				while( Gate != BeginNode )
				{
					
					FChunkNode* GateFrom = Came_from.Find( Gate );
					if( GateFrom == nullptr ) // error check
					{
						UE_LOG(LogTemp, Warning, TEXT(" Came_from nullptr, path error "));
						break;
					}

					this->Path.Emplace( GateFrom->Chunk , GateFrom->Loc );

					Gate = *GateFrom;
					//UE_LOG(LogTemp, Display, TEXT("Gate. Chunk:%s Loc:%s "), *Gate.Chunk.ToString(), *Gate.Loc.ToString());
				}

				UE_LOG(LogTemp, Display, TEXT("Data Pass End, PathNum %d"), this->Path.Num());

				return;
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("Met goal Chunk, but no path. continuing"));
			}
		} // end of 'if we met goal chunk'


		// we didn't meet goal chunk.
		// 3x3 plane, widen frontier
		for( int32 i = 0; i<3; i++ )
		{
			for (int32 j = 0; j<3; j++ )
			{
				if( i == 1 && j == 1 ) // when hit self.
				{
					//UE_LOG(LogTemp, Display, TEXT("CurrentChunk %s"), *Current.Chunk.ToString());
					continue;
				}

				FIntPoint NextChunk = Current.Chunk - FIntPoint(1,1) + FIntPoint( j, i );
				//UE_LOG(LogTemp, Display, TEXT("NextChunk %s"), *NextChunk.ToString() );

				// Tmp.priority == 'cost of Loc to Side' ( return of GetBestGate == cost )
				FNode Tmp = GetBestGate( Current.Chunk, Current.Loc, GetDestSide( Current.Chunk, NextChunk) );
				FChunkNode Next = FChunkNode( NextChunk, Tmp.Loc, INFLOAT );

				float* CostNow = CostMap.Find(Current.Chunk);
				float NewCost = INFLOAT;
				if( CostNow == nullptr )
				{
					UE_LOG(LogTemp, Warning, TEXT("HOWTHEFUCK"));
				}
				else
				{
					NewCost = Tmp.Priority + *CostNow;
				}

				// add only if we haven't been to NextChunk, or it's less costly.
				OldCost = CostMap.Find( Next.Chunk );
				if( OldCost == nullptr || *OldCost > NewCost )
				{
					if(Next == Current)
					{
						UE_LOG(LogTemp, Warning, TEXT("Same FChunkNode, map error"));
					}
					CostMap.Add( Next.Chunk, NewCost );

					//TODO: add slight vector angle priority for diagonal situations. (same point, same priority problem)
					FVector2D ToGoal = FVector2D( End.X, End.Y ) - FVector2D( Next.Loc.X, Next.Loc.Y );
					FVector2D ToNext = FVector2D( NextChunk.X, NextChunk.Y ) - FVector2D( Current.Chunk.X, Current.Chunk.Y );
					ToGoal.Normalize();
					ToNext.Normalize();

					float DotResult = FVector2D::DotProduct(ToGoal, ToNext);
					// Result is close to 1 if angle is small. -1 ~ 0 ~ 1
					// 같은 방향 -> 1 에 가까움. 반대 방향 -> -1. 수직 방향 -> 0

					Next.Priority = NewCost + Heuristic(Next.Loc) - DotResult;

					// UE_LOG(LogTemp, Display, TEXT("NewCost %f"), NewCost);
					// UE_LOG(LogTemp, Display, TEXT("Heuristic %f"), Heuristic(Next.Loc) );
					// UE_LOG(LogTemp, Display, TEXT("DotResult %f"), DotResult);
					// UE_LOG(LogTemp, Display, TEXT("NewCost + Heuristic %f"), NewCost+Heuristic(Next.Loc));
					// UE_LOG(LogTemp, Display, TEXT("NewCost + Heuristic(Next.Loc) - DotResult:%f"), Next.Priority );

					Came_from.Add( Next, Current ); // Next 'came from' Current
					Frontier.HeapPush( Next, FChunkNodePredicate() );
					
					// UE_LOG(LogTemp, Display, TEXT("Pushing next FChunkNode : %s, %s, %f"), *Next.Chunk.ToString(), *Next.Loc.ToString(), Next.Priority);
				}

			}
		}

	} // end of while

	UE_LOG(LogTemp, Warning, TEXT(" H level while ended (Not intended) "));
	return;
}

// Let's find path to one side
// and then return the first met gate (this will be our 'gate')
// Chunk == current chunk we traverse.
// will return INFLOAT cost if cannot traverse.
// this function returns 'cost' in FNode::Priority
FNode FPathFinder::GetBestGate(const FIntPoint& Chunk, const FVector2D& Start, const TSet<FVector2D>& ChunkSide)
{
	if( ChunkSide.IsEmpty() )
	{
		UE_LOG(LogTemp, Warning, TEXT(" ChunkSide Empty "));
		return FNode();
	}
	// we do A*
	TArray<FNode> Frontier; // this is mean Heap
	Frontier.Heapify( FNodePredicate() );
	Frontier.HeapPush( FNode( Start, 0.f ), FNodePredicate() );
	
	TMap<FVector2D, float> CostMap;
	CostMap.Add( Start, 0.f );

	// we don't actually need the path.
	// we just need to figure out best gate.

	FNode Current;

	while( !Frontier.IsEmpty() )
	{

		//FNode Current;
		Frontier.HeapPop( Current, FNodePredicate() ); // out param
		FVector CurrentLoc3D = ConvertTo3D(Current.Loc);

		// debugging------------------------------------------------------------
		//Path.Add(Current.Loc);
		// debugging------------------------------------------------------------
		
		if( Current.Priority >= INFLOAT ) // if popped node == INFLOAT, it means everything else are INFLOAT
		{
			UE_LOG(LogTemp, Display, TEXT("hit INFLOAT, no path found.") );
			return Current;
		}

		// check if we've already been here.
		// if so, we do the iteration only if it's less costly.
		float* OldCost = CostMap.Find( Current.Loc );
		if( OldCost != nullptr && *OldCost + Heuristic(CurrentLoc3D) < Current.Priority )
		{
			continue;
		}

		// if frontier met the side of the chunk ( if met destination )
		if( ChunkSide.Contains( Current.Loc ) ) 
		{
			float* FinalCost = CostMap.Find( Current.Loc );
			// if cost != INFLOAT, return found gate.
			if( FMath::IsFinite(*FinalCost) )
			{
				// put 'cost' in 'priority' for return
				return FNode(Current.Loc, *FinalCost);
			}
			else // return Cost == INF ( no route )
			{
				UE_LOG(LogTemp, Warning, TEXT("LowLevel A* Failed"));
				return FNode(Current.Loc, INFLOAT);
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
				FNode Neighbor;
				Neighbor.Loc = FVector2D( Loc.X + StepX, Loc.Y + StepY );

				// check if out of Chunk Boundary
				if( !IsInBoundary( Chunk, Neighbor.Loc ) )
				{
					continue;
				}

				float NewCost = INFLOAT;
				FVector NeighborLoc3D = ConvertTo3D(Neighbor.Loc);

				// check slope
				float SlopeCheck = GetSlopeSquared( CurrentLoc3D, NeighborLoc3D );
				if( SlopeCheck <= SlopeSquared )
				{
					NewCost = GetDistSquared( CurrentLoc3D, NeighborLoc3D );
				}
				float CostNow = Current.Priority - Heuristic( CurrentLoc3D );
				NewCost += CostNow;

				// check if it's new or less costly. (replaceable)
				OldCost = CostMap.Find( Neighbor.Loc );
				if( OldCost == nullptr || *OldCost > NewCost )
				{
					// TMap Add() replaces if overlaps
					CostMap.Add( Neighbor.Loc, NewCost );

					Neighbor.Priority = NewCost + Heuristic( NeighborLoc3D );
					Frontier.HeapPush( Neighbor, FNodePredicate() );
				}

			}
		} // end of for loop

	} // end of while

	UE_LOG(LogTemp, Warning, TEXT("LowLevel while done (Not intended)"));
	return FNode(Current.Loc, INFLOAT);
}

// tools

float FPathFinder::GetHeight(const FVector2D& Location)
{
	if( ShouldGenerateHeight == false )
	{
		return 0.0f;
	}
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

// returns unit distance value.
float FPathFinder::GetDistSquared(const FVector& Start, const FVector& Dest)
{
	return FVector::DistSquared( Start , Dest ) / ( VertexSpacing * VertexSpacing );
	// divide with vertexSpacing^2 since value too big.
}
float FPathFinder::GetDistSquared(const FVector2D& Start, const FVector2D& Dest)
{
	return GetDistSquared(ConvertTo3D(Start), ConvertTo3D(Dest));
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
	return Heuristic( ConvertTo3D(Current) ) ;
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
		ChunkSide.Add( Offset + FVector2D( 0.f, ChunkLength ) );
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
		ChunkSide.Add( Offset + FVector2D( 0.f, 0.f ) );
	}


	return ChunkSide;
}