
#include "PathFinder.h"
#include "LandscapeManager.h"
#include "DrawDebugHelpers.h"

#include <limits>
const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles
const float SQRT2 = FMath::Sqrt(2.0f);

FPathFinder::FPathFinder( ALandscapeManager* pLM ) : pLM( pLM )
{
	MaxSlopeTanSqr = FMath::Square(pLM->MaxSlope / 100.f);
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
	Frontier[GetFlatIndex(Start)] = FNode(0, GetMoveCost(Chunk, Start, End), NoConnection, true, NoConnection, NoConnection);

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
		// now Current == LowestCost LocalGrid in open list

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
			if ( !IsInBoundary(Neighbor) || GetTanSqr(Chunk, Current, Neighbor) > MaxSlopeTanSqr )
			{
				continue;
			}

			// continue if 'visited && lower cost'
			float NewCost = NodeNow.GCost + GetMoveCost(Chunk, Current, Neighbor);
			if (Visited[GetFlatIndex(Neighbor)] == true
				&&
				Frontier[GetFlatIndex(Neighbor)].GCost <= NewCost + 0.01 ) // add to ignore irrelavent difference
			{
				continue;
			}
			
			float Heuristic = GetMoveCost(Chunk, Neighbor, End);
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

void FPathFinder::SmoothPath(const FIntPoint& Chunk, TArray<FIntPoint>& Path)
{
	if (Path.IsEmpty() || Path.Num() < 3)
	{
		return;
	}

	TArray<FIntPoint> SmoothPath;
	int32 CheckPoint = 0;
	int32 CurrentPoint = 1;
	SmoothPath.Add(Path[CheckPoint]);

	while (CurrentPoint < Path.Num())
	{
		if (IsWalkable(Chunk, Path[CheckPoint], Path[CurrentPoint]))
		{
			CurrentPoint++;
		}
		else // last point was walkable.
		{
			CheckPoint = CurrentPoint - 1;
			SmoothPath.Add(Path[CheckPoint]);
			CurrentPoint++;
		}
	}
	SmoothPath.Add(Path.Last());
	
	Path.Empty();

	// dogshits
	TSet<FIntPoint> TwoBlock;
	for (int32 i = -2; i <= 2; i++)
	{
		for (int32 j = -2; j <= 2; j++)
		{
			TwoBlock.Add(FIntPoint(j, i));
		}
	}

	for (int32 i = 0; i < SmoothPath.Num() - 1; i++)
	{
		Path.Add(SmoothPath[i]);
		int32 temp = i + 1;
		while ( TwoBlock.Contains(SmoothPath[temp] - SmoothPath[i]) ) // not in 5x5 neighbors
		{
			temp++;
		}
		i = temp;
	}
	Path.Add(SmoothPath.Last());
}

bool FPathFinder::IsWalkable(const FIntPoint& Chunk, const FIntPoint& A, const FIntPoint& B)
{
	if ( !IsInBoundary(A) || !IsInBoundary(B) )
	{
		return false;
	}
	if (IsNeighbor(A, B))
	{
		return true;
	}

	FVector2D LocalA = FVector2D(A.X + 0.5f, A.Y + 0.5f) * pLM->VertexSpacing;
	FVector2D LocalB = FVector2D(B.X + 0.5f, B.Y + 0.5f) * pLM->VertexSpacing;
	FVector2D Direction = LocalB - LocalA;
	Direction.Normalize();
	Direction *= pLM->VertexSpacing;
	float DistSqr = GetDistSqr(LocalA, LocalB);

	FVector2D Now = LocalA;
	FVector2D Next = LocalA + Direction;
	while ( GetDistSqr(LocalA, Next) <= DistSqr )
	{
		if ( GetTanSqr(Chunk, Now, Next) > MaxSlopeTanSqr )
		{
			return false;
		}
		Now = Next;
		Next += Direction;
	}

	return true;
}

float FPathFinder::GetHeight(const FIntPoint& GlobalGrid)
{
	FVector2D ActualPos = FVector2D(GlobalGrid.X, GlobalGrid.Y) * pLM->VertexSpacing;
	return pLM->GetHeight(ActualPos);
}

float FPathFinder::GetHeight(const FIntPoint& Chunk, const FVector2D& Local)
{
	FVector2D ActualPos = FVector2D(Chunk.X, Chunk.Y) * (pLM->VerticesPerChunk - 1) * pLM->VertexSpacing + Local;
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

// diagonal approximation.
float FPathFinder::GetMoveCost(const FIntPoint& A, const FIntPoint& B)
{
	int dx = FMath::Abs(A.X - B.X);
	int dy = FMath::Abs(A.Y - B.Y);
	return (dx + dy) + (SQRT2 - 2) * FMath::Min(dx, dy); // manhattan distance - diagonal movement
}

// use manhattan dist on height
float FPathFinder::GetMoveCost(const FIntPoint& Chunk, const FIntPoint& A, const FIntPoint& B)
{
	float UnitHeight = ( FMath::Abs(GetHeight(Chunk, A) - GetHeight(Chunk, B)) ) / pLM->VertexSpacing;
	return GetMoveCost(A, B) + UnitHeight;
}

float FPathFinder::GetTanSqr(const FIntPoint& Chunk, const FIntPoint& A, const FIntPoint& B)
{
	float UnitDistSqr = GetUnitDistSqr(A, B);
	float UnitHeightSqr = ( GetCellHeight(Chunk, A) - GetCellHeight(Chunk, B) ) / pLM->VertexSpacing;
	UnitHeightSqr = FMath::Square(UnitHeightSqr);

	float TanSqr = UnitHeightSqr / UnitDistSqr;
	return TanSqr;
}

float FPathFinder::GetTanSqr(const FIntPoint& Chunk, const FVector2D& LocalA, const FVector2D& LocalB)
{
	float DistSqr = GetDistSqr(LocalA, LocalB);
	float HeightSqr = GetHeight(Chunk, LocalA) - GetHeight(Chunk, LocalB);
	HeightSqr = FMath::Square(HeightSqr);

	float TanSqr = HeightSqr / DistSqr;
	return TanSqr;
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

float FPathFinder::GetDistSqr(const FVector2D& A, const FVector2D& B)
{
	return FMath::Square(A.X - B.X) + FMath::Square(A.Y - B.Y);
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

bool FPathFinder::IsNeighbor(const FIntPoint& A, const FIntPoint& B)
{
	if (A == B)
	{
		return true;
	}

	TArray<FIntPoint> Neighbors;
	GetNeighbors(A, Neighbors);

	for (auto& Elem : Neighbors)
	{
		if (Elem == B)
		{
			return true;
		}
	}

	return false;
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

