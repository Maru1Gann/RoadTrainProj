
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

			// adding continuous connection ( gate )
			if (EndGate.A != EndGate.B) // if it's a gate. not endpoint.
			{
				ReversePath.Add( GlobalToLocal(Chunk,EndGate.B) );
			}
			while (Current != NoConnection)
			{
				ReversePath.Add(Current);
				Current = Frontier[GetFlatIndex(Current)].CameFrom;
			}
			if (StartGate.A != StartGate.B) // if it's a gate, not startpoint.
			{
				ReversePath.Add( GlobalToLocal(Chunk, StartGate.A) );
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

// line of sight. added if thingys to make it work on gates.
void FPathFinder::SmoothPath(const FIntPoint& Chunk, TArray<FIntPoint>& Path)
{
	if (Path.IsEmpty() || Path.Num() < 3)
	{
		return;
	}

	bool IsStart = true;
	bool IsEnd = true;

	TArray<FIntPoint> SmoothPath;
	int32 CheckPoint = 0;
	int32 CurrentPoint = 1;
	if ( !IsInBoundary(Path[0]) ) // if first one is out of boundary. this means it's a gate. (not the beginning)
	{
		// start line of sight from second point.
		IsStart = false;
		SmoothPath.Add(Path[CheckPoint++]);
		CurrentPoint++;
	}
	SmoothPath.Add(Path[CheckPoint]);

	int32 LastPoint = Path.Num();
	if (!IsInBoundary(Path.Last())) // if last one is out of boundary. this means it's not the endgoal.
	{
		// end before last point.
		IsEnd = false;
		LastPoint--;
	}

	while (CurrentPoint < LastPoint)
	{
		if (IsWalkable(Chunk, Path[CheckPoint], Path[CurrentPoint]))
		{
			CurrentPoint++;
		}
		else // one point before was walkable.
		{
			CheckPoint = CurrentPoint - 1;
			SmoothPath.Add(Path[CheckPoint]);
			CurrentPoint++;
		}
	}

	if (IsEnd) // if not endgoal
	{
		SmoothPath.Add(Path[Path.Num() - 2]); // this is the last one in the chunk. (if not endgoal)
	}
	SmoothPath.Add(Path.Last()); // this is out of chunk boundary
	
	Path.Empty();

	// Set to ignore 5x5 neighbor
	TSet<FIntPoint> TwoBlock;
	for (int32 i = -2; i <= 2; i++)
	{
		for (int32 j = -2; j <= 2; j++)
		{
			TwoBlock.Add(FIntPoint(j, i));
		}
	}

	int32 Start = 0;
	int32 End = SmoothPath.Num() - 2;
	if (!IsStart)
	{
		Path.Add(SmoothPath[0]);
		Start++;
	}
	if (!IsEnd)
	{
		End--;
	}

	for (int32 i = Start; i < End; i++)
	{
		Path.Add(SmoothPath[i]);
		int32 temp = i + 1;
		while ( TwoBlock.Contains(SmoothPath[temp] - SmoothPath[i]) ) // not in 5x5 neighbors
		{
			temp++;
		}
		i = temp;
	}

	if (!IsEnd)
	{
		Path.Add(SmoothPath[SmoothPath.Num() - 2]);
	}
	Path.Add(SmoothPath.Last());

}

// returns full path made from SmoothPath. 0,0 chunk Local Coordinate(except height).
void FPathFinder::RebuildPath(const FIntPoint& Chunk, const TArray<FIntPoint>& SmoothPath, TArray<FVector>& OutPath)
{
	// make 'turn radius' turns. SmoothPath ensures 15m radius.
	// plot every x meter point. for adequate spline and road mesh. x = vertexspacing should work fine.
	// check heights.
	// check lastgate - chunk start - chunk end - nextgate

	if (SmoothPath.Num() < 2)
	{
		return;
	}

	bool IsStart = true;
	bool IsEnd = true;
	int32 Start = 0;
	int32 End = SmoothPath.Num() - 1;

	if ( !IsInBoundary(SmoothPath[0]) )
	{
		IsStart = false;
		Start = 1;
	}
	if ( !IsInBoundary( SmoothPath.Last() ) )
	{
		IsEnd = false;
		End = SmoothPath.Num() - 2;
	}

	OutPath.Empty();

	//FVector2D LastDirection = (GridToCell(SmoothPath[1]) - GridToCell(SmoothPath[0])).GetSafeNormal(); // same even if IsStart is T or F.
	FVector2D LastDirection(1, -1);
	float TurnRadius = pLM->VertexSpacing * 3 ; // should fix this later.

	for (int32 i = Start; i < End; i++) // make sure i+1 don't go out of index.
	{

		FVector2D Current = GridToCell(SmoothPath[i]);
		FVector2D Next = GridToCell(SmoothPath[i + 1]);

		// doing realistic turns
		// first find center of circle. (for turning)
		FVector2D CenterR = Current + FVector2D(-LastDirection.Y, LastDirection.X).GetSafeNormal() * TurnRadius; // 90 degree turn. cw
		FVector2D CenterL = Current + FVector2D(LastDirection.Y, -LastDirection.X).GetSafeNormal() * TurnRadius; // 90 degree turn. ccw

		// + rad -> cw in UE (tested)

		DrawDebugPoint(
			pLM->GetWorld(),
			FVector(CenterR.X, CenterR.Y, GetHeight(Chunk, CenterR)+10.f),
			8.0f,
			FColor::Red,
			true
		);

		FVector2D DirectionCheck = LastDirection * TurnRadius + Current;
		DrawDebugPoint(
			pLM->GetWorld(),
			FVector(DirectionCheck.X, DirectionCheck.Y, GetHeight(Chunk, DirectionCheck)+10.f),
			8.0f,
			FColor::Black,
			true
		);

		DrawDebugPoint(
			pLM->GetWorld(),
			FVector(CenterL.X, CenterL.Y, GetHeight(Chunk, CenterL)+10.f),
			8.0f,
			FColor::Blue,
			true
		);

		float Theta; // Next - Center - CircleEnd
		float Phi; // Current - Center - Next
		Theta = FMath::Acos( TurnRadius / FVector2D::Distance(CenterR, Next) );
		Phi = FMath::Atan2(Current.Y - CenterR.Y, Current.X - CenterR.X) - FMath::Atan2(Next.Y - CenterR.Y, Next.X - CenterR.X);
		Phi = FMath::Abs(Phi);
		UE_LOG(LogTemp, Warning, TEXT("Theta %f,  Phi %f") ,Theta, Phi);
		
		float AngleStart = FMath::Atan2(Current.Y - CenterR.Y, Current.X - CenterR.X);
		UE_LOG(LogTemp, Warning, TEXT("AngleStart %f"), AngleStart);
		float AngleEnd = Phi - Theta + AngleStart;
		float AngleStep = pLM->VertexSpacing*2 / TurnRadius; // arc length == vertexspacing.

		FVector2D CircleEnd = CenterR + FVector2D(FMath::Cos(AngleEnd), FMath::Sin(AngleEnd)) * TurnRadius;
		DrawDebugPoint(
			pLM->GetWorld(),
			LocalToGlobal(Chunk, Current),
			12.0f,
			FColor::Cyan,
			true
		);
		DrawDebugPoint(
			pLM->GetWorld(),
			LocalToGlobal(Chunk, CircleEnd),
			12.0f,
			FColor::Blue,
			true
		);

		OutPath.Add(FVector(Current.X, Current.Y, GetHeight(Chunk, Current))); // add CurrentPoint

		UE_LOG(LogTemp, Warning, TEXT("AngleStart %fPI, AngleEnd %fPI"), AngleStart / PI, AngleEnd / PI);
		for (float Angle = AngleStart + AngleStep; Angle < AngleEnd; Angle += AngleStep)
		{
			FVector2D Point = CenterR + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * TurnRadius;
			OutPath.Add(LocalToGlobal(Chunk, Point));
			DrawDebugPoint(
				pLM->GetWorld(),
				LocalToGlobal(Chunk, Point),
				10.0f,
				FColor::Red,
				true
			);
		}
		OutPath.Add(LocalToGlobal(Chunk, CircleEnd));

		// should work with stuffs below after dealing with Realistic Turns.
		float Step = pLM->VertexSpacing * 2;
		FVector2D Direction = (Next - CircleEnd).GetSafeNormal() * Step;
		FVector2D Temp = CircleEnd + Direction;
		float DistSqr = FVector2D::DistSquared(CircleEnd, Next);
		float RadSqr = FMath::Square(pLM->VertexSpacing);

		while (FVector2D::DistSquared(CircleEnd, Temp) < DistSqr && FVector2D::DistSquared(Temp , Next) >= RadSqr)
		{
			OutPath.Add( FVector( Temp.X, Temp.Y, GetHeight(Chunk, Temp) ) ); // add Pivots between Current and Next
			Temp += Direction; // go another step.
		} // this won't add the last 'Next' point

		LastDirection = Direction;
	} // we can do more work on height smoothing

	// add the last 'Next'
	FVector Last;
	if (!IsEnd) // if not endgoal, actual last one is on next chunk.
	{
		Last = FVector(SmoothPath[End-1].X + 0.5f, SmoothPath[End-1].Y + 0.5f, 0.f) * pLM->VertexSpacing;
		Last.Z = GetHeight(Chunk, FVector2D(Last.X, Last.Y));
		OutPath.Add(Last);
	}
	Last = FVector(SmoothPath.Last().X + 0.5f, SmoothPath.Last().Y + 0.5f, 0.f) * pLM->VertexSpacing;
	Last.Z = GetHeight(Chunk, FVector2D(Last.X, Last.Y));
	OutPath.Add(Last);

	UE_LOG(LogTemp, Warning, TEXT("RebuildPath Num %d"), OutPath.Num());
	
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

FVector2D FPathFinder::GridToCell(const FIntPoint& LocalGrid)
{
	return FVector2D(LocalGrid.X + 0.5f, LocalGrid.Y + 0.5f) * pLM->VertexSpacing;
}

FVector FPathFinder::LocalToGlobal(const FIntPoint& Chunk, const FVector2D& Local)
{
	return FVector( Local.X, Local.Y, GetHeight(Chunk,Local));
}

float FPathFinder::ToCWAngle(const float& Rad)
{
	float Out = FMath::Fmod(Rad, 2 * PI);
	if (Out < 0.f)
	{
		Out = 2 * PI + Out;
	}
	return Out;
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

