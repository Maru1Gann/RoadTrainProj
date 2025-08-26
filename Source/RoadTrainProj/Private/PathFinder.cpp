
#include "PathFinder.h"
#include "LandscapeManager.h"
#include "DrawDebugHelpers.h"

#include <limits>
const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles
const float SQRT2 = FMath::Sqrt(2.0f);

FPathFinder::FPathFinder( ALandscapeManager* pLM ) : pLM( pLM )
{
}

// find path, but do it on (0,0) local chunk.
bool FPathFinder::GetPath(const FGate& StartGate, const FGate& EndGate, TArray<FIntPoint>& OutPath, bool DrawDebug)
{
	FIntPoint Chunk = GetChunk(StartGate.B);
	if (Chunk != GetChunk(EndGate.A))
	{
		UE_LOG(LogTemp, Warning, TEXT("GetPath input gates Chunk error, %s %s"), *Chunk.ToString(), *GetChunk(EndGate.A).ToString());
		return false;
	}

	static const FIntPoint NoConnection(-100, -100);

	// temporary struct
	struct FNode
	{
		FNode() { 
			GCost = INFLOAT; 
			FCost = INFLOAT; 
			CameFrom = NoConnection;  
			IsOpen = false; 
			Prev = NoConnection; 
			Next = NoConnection; };

		FNode(
			const float& GCost,
			const float& FCost,
			const FIntPoint& CameFrom,
			const bool& IsOpen,
			const FIntPoint& Prev,
			const FIntPoint& Next
		) : GCost(GCost), FCost(FCost), CameFrom(CameFrom), IsOpen(IsOpen), Prev(Prev), Next(Next) {};

		// f(n) = g(n) + h(n) -> final cost = cost + heuristic
		float GCost; // g(n). cost total to this point.
		float FCost; // f(n). g(n) + h(n).
		FIntPoint CameFrom;
		bool IsOpen;
		FIntPoint Prev; // open list doubly linked
		FIntPoint Next;
	};


	// FIntPoint Chunk = GetChunk(StartGate.EndGate);
	FIntPoint Start = GlobalToLocal(Chunk, StartGate.B);
	FIntPoint End = GlobalToLocal(Chunk, EndGate.A);

	TArray<FNode> Frontier;
	int32 FrontierNum = FMath::Square((pLM->VerticesPerChunk - 1));
	Frontier.SetNum(FrontierNum);

	TArray<bool> Visited;
	Visited.SetNum(FrontierNum);
	for (auto& Elem : Visited)
	{
		Elem = false;
	}
	Visited[GetFlatIndex(Start)] = true;
	Frontier[GetFlatIndex(Start)] = FNode(0, GetMoveCost(Start,End), NoConnection, true, NoConnection, NoConnection);

	FIntPoint OpenListStart = Start;
	FIntPoint OpenListEnd = Start;

	int32 Counter = 0;

	// start A*
	while (OpenListStart != NoConnection && OpenListEnd != NoConnection)
	{
		if (Counter >= pLM->CounterHardLock)
		{
			UE_LOG(LogTemp, Warning, TEXT("CounterLockHit, %d"), Counter);
			break;
		}
		Counter++;

		// find lowest GCost node. (highest priority node)
		float LowestCost = INFLOAT;
		FIntPoint Current = NoConnection;

		for (FIntPoint Finder = OpenListStart; Finder != NoConnection; Finder = Frontier[GetFlatIndex(Finder)].Next)
		{
			FNode& FinderNode = Frontier[GetFlatIndex(Finder)];
			if (LowestCost > FinderNode.FCost)
			{
				Current = Finder;
				LowestCost = FinderNode.FCost;
			}
		}

		if (Current == NoConnection) // something went off.
		{
			return false;
		}

		if (DrawDebug) // false by defualt
		{
			DrawDebugPoint(
				pLM->GetWorld(),
				pLM->GridToVector(LocalToGlobal(Chunk, Current)),
				3.0f,
				FColor::Red,
				true
			);
		}


		// ----------if met goal---------
		if (Current == End)
		{
			// do data passing
			TArray<FIntPoint> ReversePath;
			ReversePath.Empty();
			while (Current != NoConnection)
			{
				ReversePath.Add(Current);
				Current = Frontier[GetFlatIndex(Current)].CameFrom;
			}
			OutPath.Empty();
			OutPath.SetNum(ReversePath.Num());
			for (int32 i = 0; i < ReversePath.Num(); i++)
			{
				OutPath[i] = LocalToGlobal( Chunk, ReversePath[ReversePath.Num() - 1 - i] );
			}
			UE_LOG(LogTemp, Warning, TEXT("WhileCount, %d"), Counter);
			return true;
		}
		
		// ---------if didn't meet goal---------
		TArray<FIntPoint> Neighbors;
		FNode& NodeNow = Frontier[GetFlatIndex(Current)];
		GetNeighbors(Current, Neighbors);
		for (auto& Neighbor : Neighbors)
		{
			// continue if blocked (Check slope)
			if ( !IsInBoundary(Neighbor) )
			{
				continue;
			}

			// continue if 'visited && lower cost'
			float NewCost = NodeNow.GCost + GetMoveCost(Current, Neighbor);
			if (Visited[GetFlatIndex(Neighbor)] == true
				&&
				Frontier[GetFlatIndex(Neighbor)].GCost <= NewCost + 0.01 ) // add to ignore irrelavent difference
			{
				continue;
			}
			
			float Heuristic = GetMoveCost(Neighbor, End);
			FNode& NodeNext = Frontier[GetFlatIndex(Neighbor)];
			NodeNext.GCost = NewCost;
			NodeNext.FCost = NewCost + Heuristic;
			NodeNext.CameFrom = Current;
			Visited[GetFlatIndex(Neighbor)] = true; // visited.

			// add this to open list
			if (!NodeNext.IsOpen)
			{
				Frontier[GetFlatIndex(OpenListEnd)].Next = Neighbor;
				NodeNext.Prev = OpenListEnd;
				NodeNext.Next = NoConnection;
				NodeNext.IsOpen = true;
				OpenListEnd = Neighbor;
			}
		}

		// update list start and end
		if (OpenListStart == Current)
		{
			OpenListStart = NodeNow.Next;
		}
		if (OpenListEnd == Current)
		{
			OpenListEnd = NodeNow.Prev;
		}
		// remove current from open list.
		NodeNow.IsOpen = false;
		if (NodeNow.Prev != NoConnection)
		{
			FNode& PrevNode = Frontier[GetFlatIndex(NodeNow.Prev)];
			PrevNode.Next = NodeNow.Next;
		}
		if (NodeNow.Next != NoConnection)
		{
			FNode& NextNode = Frontier[GetFlatIndex(NodeNow.Next)];
			NextNode.Prev = NodeNow.Prev;
		}


	}

	
	return false; // no path found
}


float FPathFinder::GetHeight(const FIntPoint& GlobalGrid)
{
	FVector2D ActualPos = FVector2D(GlobalGrid.X, GlobalGrid.Y) * pLM->VertexSpacing;
	return pLM->GetHeight(ActualPos);
}

float FPathFinder::GetCellHeight(const FIntPoint& GlobalGrid)
{
	FVector2D CellCenter = FVector2D(GlobalGrid.X + 0.5f, GlobalGrid.Y + 0.5f) * pLM->VertexSpacing;
	return pLM->GetHeight(CellCenter);
}

float FPathFinder::GetCellHeight(const FIntPoint& Chunk, const FIntPoint& LocalGrid)
{
	return GetCellHeight(LocalToGlobal(Chunk, LocalGrid));
}

FIntPoint FPathFinder::LocalToGlobal(const FIntPoint& Chunk, const FIntPoint& LocalGrid)
{
	return Chunk * (pLM->VerticesPerChunk - 1) + LocalGrid;
}

FIntPoint FPathFinder::GlobalToLocal(const FIntPoint& Chunk, const FIntPoint& GlobalGrid)
{
	return GlobalGrid - Chunk * (pLM->VerticesPerChunk - 1);
}

FIntPoint FPathFinder::GetChunk(const FIntPoint& GlobalGrid)
{
	FIntPoint Chunk;
	Chunk.X = FMath::FloorToInt32(float(GlobalGrid.X) / float(pLM->VerticesPerChunk-1));
	Chunk.Y = FMath::FloorToInt32(float(GlobalGrid.Y) / float(pLM->VerticesPerChunk-1));
	return Chunk;
}

float FPathFinder::GetMoveCost(const FIntPoint& A, const FIntPoint& B)
{
	int dx = FMath::Abs(A.X - B.X);
	int dy = FMath::Abs(A.Y - B.Y);
	return (dx + dy) + (SQRT2 - 2) * FMath::Min(dx, dy); // manhattan distance - diagonal movement
}

int32 FPathFinder::GetFlatIndex(const FIntPoint& Index2D)
{
	int32 FlatIndex = Index2D.Y * (pLM->VerticesPerChunk - 1) + Index2D.X;
	return FlatIndex;
}

FIntPoint FPathFinder::GetIndex2D(const int32& FlatIndex)
{
	FIntPoint Index2D;
	Index2D.Y = FlatIndex / (pLM->VerticesPerChunk - 1);
	Index2D.X = FlatIndex % (pLM->VerticesPerChunk - 1);
	return Index2D;
}

int32 FPathFinder::GetUnitDistSqr(const FIntPoint& A, const FIntPoint& B)
{
	return FMath::Square(B.X - A.X) + FMath::Square(B.Y - A.Y);
}

void FPathFinder::GetNeighbors(const FIntPoint& A, TArray<FIntPoint>& OutNeighbors)
{
	OutNeighbors.Empty();
	OutNeighbors.Reserve(8);
	for (int32 iY = -1; iY <= 1; iY++)
	{
		for (int32 iX = -1; iX <= 1; iX++)
		{
			if(iX == 0 && iY == 0)
			{ continue; }
			else
			{ OutNeighbors.Add(A + FIntPoint(iX, iY)); }
		}
	}

}

bool FPathFinder::IsInBoundary(const FIntPoint& A)
{
	int32 Boundary = pLM->VerticesPerChunk - 1;
	if (A.X >= 0 && A.X < Boundary 
		&& 
		A.Y >= 0 && A.Y < Boundary)
	{ return true; }
	else
	{ return false; }

}

