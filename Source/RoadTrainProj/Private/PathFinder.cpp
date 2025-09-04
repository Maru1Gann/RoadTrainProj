
#include "PathFinder.h"
#include "LandscapeManager.h"
#include "DrawDebugHelpers.h"

#include <limits>
const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles
const float SQRT2 = FMath::Sqrt(2.0f);
const FIntPoint NoConnection(-100, -100);

FPathFinder::FPathFinder( ALandscapeManager* pLM ) : pLM( pLM )
{
	MaxSlopeTanSqr = FMath::Square(pLM->MaxSlope / 100.f);
}

// temporary struct for pathfinding.
struct FNode
{
	FNode() {
		GCost = INFLOAT;
		FCost = INFLOAT;
		CameFrom = NoConnection;
		IsOpen = false;
		Prev = NoConnection;
		Next = NoConnection;
	};

	FNode(
		const float& GCost,
		const float& FCost,
		const FIntPoint& CameFrom,
		const bool& IsOpen,
		const FIntPoint& Prev,
		const FIntPoint& Next
	) : GCost(GCost), FCost(FCost), CameFrom(CameFrom), IsOpen(IsOpen), Prev(Prev), Next(Next) {
	};

	// f(n) = g(n) + h(n) -> final cost = cost + heuristic
	float GCost; // g(n). cost total to this point.
	float FCost; // f(n). g(n) + h(n).
	FIntPoint CameFrom;
	bool IsOpen;
	FIntPoint Prev; // open list doubly linked
	FIntPoint Next;
};


// find gates. TMap<FGate, TPair<FGate, float cost>> OutGates
// basically GetPath with GlobalGoal Heuristic.
void FPathFinder::GetGates(const FGate& StartGate, const FIntPoint& GlobalGoal, TMap<FIntPoint, TPair<FGate, float>>& OutGates)
{
	// add every neighboring chunk that's reachable to TMap. (as a form of FGate)
	FIntPoint Chunk = GetChunk(StartGate.B);

	// FIntPoint Chunk = GetChunk(StartGate.EndGate);
	FIntPoint Start = GlobalToLocal(Chunk, StartGate.B);
	FIntPoint Goal = GlobalToLocal(Chunk, GlobalGoal);

	TArray<FNode> Frontier;
	int32 FrontierNum = FMath::Square((pLM->VerticesPerChunk - 1)); // num of cells
	Frontier.SetNum(FrontierNum);

	TArray<bool> Visited;
	Visited.SetNum(FrontierNum);
	for (auto& Elem : Visited) { Elem = false; }
	Visited[GetFlatIndex(Start)] = true;
	Frontier[GetFlatIndex(Start)] = FNode(0, GetMoveCost(Chunk, Start, Goal), NoConnection, true, NoConnection, NoConnection);

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
			return;
		}

		// ----------if met potential gate. (chunk boundary)---------
		if ( IsOnBoundary(Current) )
		{
			// find possible edges.
			TArray<FIntPoint> NeighborsTemp;
			GetNeighbors(Current, NeighborsTemp);
			TArray<FIntPoint> Edges;
			for (auto& Neighbor : NeighborsTemp)
			{
				if ( !IsInBoundary(Neighbor) )
				{
					Edges.Add(Neighbor);
				}
			}


		}


		// ---------if didn't meet goal---------
		TArray<FIntPoint> Neighbors;
		FNode& NodeNow = Frontier[GetFlatIndex(Current)];
		GetNeighbors(Current, Neighbors);
		for (auto& Neighbor : Neighbors)
		{

			// continue if blocked (Check slope)
			if (!IsInBoundary(Neighbor) || GetTanSqr(Chunk, Current, Neighbor) > MaxSlopeTanSqr)
			{
				continue;
			}

			// continue if 'visited && lower cost'
			float NewCost = NodeNow.GCost + GetMoveCost(Chunk, Current, Neighbor);
			if (Visited[GetFlatIndex(Neighbor)] == true
				&&
				Frontier[GetFlatIndex(Neighbor)].GCost <= NewCost + 0.01) // add to ignore irrelavent difference
			{
				continue;
			}

			float Heuristic = GetMoveCost(Chunk, Neighbor, Goal);
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


	return;
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

	FVector2D LastDirection = (GridToCell(SmoothPath[1]) - GridToCell(SmoothPath[0])).GetSafeNormal(); // same even if IsStart is T or F.
	//FVector2D LastDirection(1, 0);
	float TurnRadius = pLM->VertexSpacing * 1.5 ; // should fix this later.

	for (int32 i = Start; i < End; i++) // make sure i+1 don't go out of index.
	{

		FVector2D Current = GridToCell(SmoothPath[i]);
		FVector2D Next = GridToCell(SmoothPath[i + 1]);


		// ARC PART (TURN)
		TArray<FVector2D> CurveArc;
		bool bGotCurve = GetCurve(LastDirection, Current, Next, CurveArc, TurnRadius); // curve arc contains current.

		UE_LOG(LogTemp, Warning, TEXT("Curve Num %d"), CurveArc.Num());
		for (auto& Elem : CurveArc)
		{
			DrawDebugPoint( pLM->GetWorld(), LocalToGlobal(Chunk, Elem), 10.0f, FColor::Blue, true );
			OutPath.Add(LocalToGlobal(Chunk,Elem));
		}
		
		// LINE PART
		FVector2D LineStart = Current;
		if (bGotCurve) LineStart = CurveArc.Last();

		float Step = pLM->VertexSpacing * 2;
		FVector2D Direction = (Next - LineStart).GetSafeNormal();
		FVector2D Temp = LineStart + Direction;
		float DistSqr = FVector2D::DistSquared(LineStart, Next);
		float RadSqr = FMath::Square(pLM->VertexSpacing);

		while (FVector2D::DistSquared(LineStart, Temp) < DistSqr && FVector2D::DistSquared(Temp , Next) >= RadSqr)
		{
			OutPath.Add( FVector( Temp.X, Temp.Y, GetHeight(Chunk, Temp) ) ); // add Pivots between Current and Next
			Temp += Direction*Step; // go another step.
		} // this won't add the last 'Next' point

		LastDirection = Direction;
	} // we can do more work on height smoothing. for end.

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
	float UnitHeight = ( FMath::Abs( GetCellHeight(Chunk, A) - GetCellHeight(Chunk, B) ) ) / pLM->VertexSpacing;
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

// returns arc route of shorter turn.
bool FPathFinder::GetCurve(const FVector2D& StartDirection, const FVector2D& Current, const FVector2D& Next, TArray<FVector2D>& OutRoute, const float& TurnRadius, const float& NoTurnAngle)
{
	FVector2D Dir1 = StartDirection.GetSafeNormal(); 
	FVector2D Dir2 = (Next - Current).GetSafeNormal();
	float MaxAngle = FMath::Cos( FMath::DegreesToRadians(NoTurnAngle) );
	if (FVector2D::DotProduct(Dir1, Dir2) > MaxAngle) // if turn is not needed, return false.
	{
		return false;
	}

	FVector2D CenterR = Current + FVector2D(-StartDirection.Y, StartDirection.X).GetSafeNormal() * TurnRadius; // 90 degree turn. cw
	FVector2D CenterL = Current + FVector2D(StartDirection.Y, -StartDirection.X).GetSafeNormal() * TurnRadius; // 90 degree turn. ccw

	// let's figure out which turn is better
	float RArcAngle , LArcAngle;
	RArcAngle = INFLOAT;
	LArcAngle = INFLOAT;

	RArcAngle = GetArcAngle(CenterR, Current, Next, true, TurnRadius);	// this is CW
	LArcAngle = GetArcAngle(CenterL, Current, Next, false, TurnRadius); // this should go CCW.
	
	float ArcAngle;
	FVector2D Center;
	float StepDir = 1.0f;
	if (RArcAngle < LArcAngle) // do Right Turn. CW
	{
		ArcAngle = RArcAngle;
		Center = CenterR;
	}
	else // do Left Turn. CCW
	{
		ArcAngle = LArcAngle;
		Center = CenterL;
		StepDir = -1.0f;
	}

	float AngleStart = FMath::Atan2(Current.Y - Center.Y, Current.X - Center.X);
	if (AngleStart < 0.f) AngleStart += 2 * PI;
	float AngleStep = pLM->VertexSpacing * 2 / TurnRadius; // length of arc of the step == pLM->VertexSpacing*2 
	float ActualAngle = AngleStart;
	float AngleCounter = 0 + AngleStep;

	OutRoute.Empty();
	OutRoute.Add(Current);
	while (AngleCounter < ArcAngle)
	{
		ActualAngle += StepDir * AngleStep;
		if (ActualAngle < 0.f) ActualAngle += 2 * PI;
		FVector2D Point = Center + FVector2D( FMath::Cos(ActualAngle) * TurnRadius, FMath::Sin(ActualAngle) * TurnRadius );
		OutRoute.Add( Point );
		AngleCounter += AngleStep;
	}

	return true;
}

float FPathFinder::GetArcAngle(const FVector2D& Center, const FVector2D& Current, const FVector2D& Next, const bool& IsRightTurn, const float& TurnRadius)
{
	float Distance = FVector2D::Distance(Center, Next);
	if (Distance < TurnRadius) return INFLOAT;

	float Theta, Phi; // Theta: Q - Center - Current.  Phi: Next - Center - Current.
	float ToC, ToN; 
	Theta = FMath::Acos(TurnRadius / Distance);

	ToC = FMath::Atan2(Current.Y - Center.Y, Current.X - Center.X);
	ToN = FMath::Atan2(Next.Y - Center.Y, Next.X - Center.X);
	if (ToC < 0.f) ToC += 2 * PI; // make it clockwise
	if (ToN < 0.f) ToN += 2 * PI;
	float CToN = ToN - ToC; // cw (clockwise)
	if (CToN < 0.f) CToN += 2 * PI;

	if (IsRightTurn)
	{
		Phi = CToN;
		float Out = Phi - Theta;
		if (Out < 0.f) Out += 2 * PI;
		// UE_LOG(LogTemp, Warning, TEXT("RTheta %f, RPhi %f, ROut %f"), FMath::RadiansToDegrees(Theta), FMath::RadiansToDegrees(Phi), FMath::RadiansToDegrees(Out));
		return Out;
	}
	else
	{
		Phi = 2 * PI - CToN; // make it ccw (left turn)
		float Out = Phi - Theta;
		if (Out < 0.f) Out += 2 * PI;
		// UE_LOG(LogTemp, Warning, TEXT("LTheta %f, LPhi %f, LOut %f"), FMath::RadiansToDegrees(Theta), FMath::RadiansToDegrees(Phi), FMath::RadiansToDegrees(Out));
		return Out;
	}

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

bool FPathFinder::IsInBoundary(const FIntPoint& LocalGrid)
{
	FIntPoint A = LocalGrid;
	int32 Boundary = pLM->VerticesPerChunk - 1;
	if (A.X >= 0 && A.X < Boundary 
		&& 
		A.Y >= 0 && A.Y < Boundary)
	{ return true; }
	else
	{ return false; }

}

bool FPathFinder::IsOnBoundary(const FIntPoint& LocalGrid)
{
	FIntPoint A = LocalGrid;
	int32 Boundary = pLM->VerticesPerChunk - 2; // Cell Num = VertsPerChunk - 1. Cell Last Index = CellNum - 1
	if (A.X == 0 || A.X == Boundary
		||
		A.Y == 0 || A.Y == Boundary)
	{ return true; }
	else
	{ return false; }

}