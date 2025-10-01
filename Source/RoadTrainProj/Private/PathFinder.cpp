
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
	SlopeViolationPanelty = pLM->SlopeViolationPanelty;
	MinTurnRadius = pLM->MinTurnRadius;
	UnitMinTurnRadius = MinTurnRadius / pLM->VertexSpacing;
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

// Chunk Level A*. use GetGates to find neighbors.
// basically GetPath on Chunk Level.
// StartCell & EndCell == GlobalGrid.
bool FPathFinder::GetGatePath(const FIntPoint& StartCell, const FIntPoint& EndCell, TArray<FGate>& OutGatePath)
{
	
	// +2 to make box size like below.
	// 0 0 0 0
	// 0 0 e 0
	// 0 s 0 0
	// 0 0 0 0
	// one chunk bigger.
	// S and E should be in 0 ~ + Range.
	// so we'll add offset.

	FIntPoint StartChunk = GetChunk(StartCell);
	FIntPoint EndChunk = GetChunk(EndCell);

	FIntPoint Offset = FIntPoint( -FMath::Min(StartChunk.X, EndChunk.X), -FMath::Min(StartChunk.Y, EndChunk.Y) ) + FIntPoint(1, 1);

	FIntPoint Start = StartChunk + Offset;
	FIntPoint End = EndChunk + Offset;

	FIntPoint FrontierSize( FMath::Abs(Start.X - End.X) + 1 + 2, FMath::Abs(Start.Y - End.Y) + 1 + 2 ); // may be rectangle
	// made the box.

	if (FrontierSize.X * FrontierSize.Y > 500 * 500)
	{
		UE_LOG(LogTemp, Warning, TEXT("Start-End box too big."));
		return false;
	}

	TArray<FNode> Frontier;
	int32 FrontierNum = FrontierSize.X * FrontierSize.Y;
	int32 RowNum = FrontierSize.X;
	Frontier.SetNum(FrontierNum);

	TArray<bool> Visited;
	Visited.SetNum(FrontierNum);
	for (auto& Elem : Visited) Elem = false; // init

	Visited[ GetFlatIndex(Start, RowNum) ] = true;
	Frontier[ GetFlatIndex(Start, RowNum) ] = FNode(0, GetGlobalMoveCost(StartCell, EndCell), NoConnection, true, NoConnection, NoConnection);

	TMap<FIntPoint, FGate> GateMap;			// Key is Chunk, Value is Gate. Gate is lowest GCost gate that comes into the chunk. ( startgate )
	GateMap.Add( Start, FGate(StartCell) );	// only update when cost is lower.

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
		for ( FIntPoint Finder = OpenListStart; Finder != NoConnection; Finder = Frontier[GetFlatIndex(Finder, RowNum)].Next )
		{
			FNode& FinderNode = Frontier[GetFlatIndex(Finder, RowNum)];
			if (LowestCost > FinderNode.FCost)
			{
				Current = Finder;
				LowestCost = FinderNode.FCost;
			}
		}
		// now Current == LowestCost LocalGrid in open list

	
		if ( Current == NoConnection ) // something went off.
		{ return false; }

		FGate* CurrentGate = GateMap.Find(Current);
		if (!CurrentGate) 
		{ UE_LOG(LogTemp, Warning, TEXT("CurrentGate Nullptr Error")); return false; }

		// ----------if met goal---------
		if (Current == End)
		{
			// UE_LOG(LogTemp, Warning, TEXT("Current %s Goal %s"), *Current.ToString(), *End.ToString());
			// do data passing
			TArray<FGate> ReversePath;
			while (Current != NoConnection)
			{
				CurrentGate = GateMap.Find( Current );
				ReversePath.Add( *CurrentGate );
				Current = Frontier[GetFlatIndex(Current, RowNum)].CameFrom;

			}

			OutGatePath.Empty();
			OutGatePath.SetNum(ReversePath.Num());
			for (int32 i = 0; i < ReversePath.Num(); i++)
			{
				OutGatePath[i] = ReversePath[ReversePath.Num() - 1 - i];
			}
			OutGatePath.Add(FGate(EndCell));
			return true;
		}

		// ---------if didn't meet goal---------
		FNode& NodeNow = Frontier[ GetFlatIndex(Current, RowNum) ];
		TMap< FIntPoint, TPair<FGate, float> > NeighborGates;
		GetGates( *CurrentGate, EndCell, NeighborGates);
		for (auto& NeighborGate : NeighborGates)
		{
			FIntPoint Neighbor = NeighborGate.Key + Offset;	// FIntPoint
			float GCost = NeighborGate.Value.Value;	// TPair< FGate, "float" >
			FGate Gate = NeighborGate.Value.Key;	// TPair< "FGate", float > 
			// continue if out of bounds
			if ( !IsInBoundary( Neighbor, FrontierSize ) )
			{ continue; }

			// continue if 'visited && lower cost'
			float NewCost = NodeNow.GCost + GCost;
			if ( Visited[GetFlatIndex(Neighbor, RowNum)] == true
				&&
				Frontier[GetFlatIndex(Neighbor, RowNum)].GCost <= NewCost + 0.01 ) // add to ignore irrelavent difference
			{ continue; }

			// this is lowest cost visit.
			GateMap.Add( Neighbor, Gate );
			float Heuristic = GetGlobalMoveCost(Gate.B, EndCell);
			FNode& NodeNext = Frontier[ GetFlatIndex(Neighbor, RowNum) ];
			NodeNext.GCost = NewCost;
			NodeNext.FCost = NewCost + Heuristic;
			NodeNext.CameFrom = Current;
			Visited[GetFlatIndex(Neighbor, RowNum)] = true; // visited.

			// add this to open list
			if (!NodeNext.IsOpen)
			{
				Frontier[GetFlatIndex(OpenListEnd, RowNum)].Next = Neighbor;
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
			FNode& PrevNode = Frontier[GetFlatIndex(NodeNow.Prev, RowNum)];
			PrevNode.Next = NodeNow.Next;
		}
		if (NodeNow.Next != NoConnection)
		{
			FNode& NextNode = Frontier[GetFlatIndex(NodeNow.Next, RowNum)];
			NextNode.Prev = NodeNow.Prev;
		}


	}


	return false; // no path found
}


// find gates. TMap<FIntPoint GlobalChunk, TPair<FGate, float Gcost>> OutGates
// basically GetPath with GlobalGoal Heuristic.
void FPathFinder::GetGates(const FGate& StartGate, const FIntPoint& GlobalGoal, TMap<FIntPoint, TPair<FGate, float>>& OutGates, bool DrawDebug)
{
	// let's add every neighboring chunk that's reachable to TMap. (as a form of FGate)
	OutGates.Empty();

	FIntPoint Chunk = GetChunk(StartGate.B);
	FIntPoint FromChunk = GetChunk(StartGate.A);

	if (Chunk != FromChunk) // if we came from other chunk.
	{
		OutGates.Add(FromChunk, TPair<FGate, float>(FGate(StartGate.B, StartGate.A), GetGlobalMoveCost(StartGate.B, StartGate.A)));
	}

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

		if (DrawDebug) // false by defualt
		{
			DrawDebugPoint(
				pLM->GetWorld(),
				pLM->GridToVector(LocalToGlobal(Chunk, Current)),
				3.0f,
				FColor::Blue,
				true
			);
		}

		FNode& NodeNow = Frontier[GetFlatIndex(Current)];

		// ----------if met potential gate. (chunk boundary)---------
		if ( IsOnBoundary(Current) && GetUnitDistSqr(Start, Current) >= FMath::Square(UnitMinTurnRadius*2) ) // considering turn radius.
		{
			// find possible edges.
			TArray<FIntPoint> NeighborsTemp;
			GetNeighbors(Current, NeighborsTemp);
			TArray<TPair<FIntPoint, float>> Edges;
			for (auto& Neighbor : NeighborsTemp)
			{
				if ( !IsInBoundary(Neighbor)
					/*&& !OutGates.Contains( GetChunk( LocalToGlobal( Chunk, Neighbor ) ) ) */
					/*&& GetTanSqr(Chunk, Current, Neighbor) <= MaxSlopeTanSqr*/ )
				{
					// out of boundary (gate) && Not a already found gate && slope traversable
					// this fills outgates with gates that are found first. so if already found, no need to update.
					// Total Cost + To Next Cost + Heuristic
					float MoveCost = GetMoveCost(Chunk, Current, Neighbor);
					if (GetTanSqr(Chunk, Current, Neighbor) > MaxSlopeTanSqr) MoveCost *= 2;

					float GCost = NodeNow.GCost + MoveCost;
					Edges.Add(TPair<FIntPoint, float>(Neighbor, GCost));
				}
			}

			for (auto& Edge : Edges) 
			{
				// do it only if direction of gate comes into this chunk. (for continuous chunk connection)
				
				FIntPoint GateDir = Current - Edge.Key; // this direction should come into the chunk.
				if (!IsInBoundary(Current + GateDir * 3)) continue;

				FGate GlobalGate = FGate(LocalToGlobal(Chunk, Current), LocalToGlobal(Chunk, Edge.Key));
				FIntPoint NextChunk = GetChunk(GlobalGate.B);
				const float& GCost = Edge.Value;

				TPair<FGate, float> NewOutPut(GlobalGate, GCost);

				TPair<FGate, float>* OldGate = OutGates.Find(NextChunk);
				if (OldGate)
				{
					const float& OldGCost = (*OldGate).Value;
					float OldFCost = OldGCost + GetGlobalMoveCost((*OldGate).Key.B, GlobalGoal);
					float NewFCost = GCost + GetGlobalMoveCost(NewOutPut.Key.B, GlobalGoal);

					if (OldFCost > NewFCost + 0.1f) OutGates.Add(NextChunk, NewOutPut);
				}
				else
				{
					OutGates.Add(NextChunk, NewOutPut);
				}

			}

		} // end of if is on boundary.

		// ----------------if all gates are found----------------
		if (OutGates.Num() >= 8) {

			if (DrawDebug)
			{
				for (auto& Elem : OutGates)
				{
					FIntPoint A = Elem.Value.Key.A;
					FIntPoint B = Elem.Value.Key.B;
					float Cost = Elem.Value.Value;
					UE_LOG(LogTemp, Warning, TEXT("A %s B %s Cost %f"), *A.ToString(), *B.ToString(), Cost);
					DrawDebugPoint(pLM->GetWorld(), pLM->GridToVector(A), 8.f, FColor::Red, true);
					DrawDebugPoint(pLM->GetWorld(), pLM->GridToVector(B), 8.f, FColor::Red, true);
				}
			}

			return;
		}


		// ---------if didn't meet goal---------
		TArray<FIntPoint> Neighbors;
		GetNeighbors(Current, Neighbors);
		for (auto& Neighbor : Neighbors)
		{

			// continue if blocked (Check slope)
			if (!IsInBoundary(Neighbor))
			{
				continue;
			}

			// multiply movecost if slope violated.
			float MoveCost = GetMoveCost(Chunk, Current, Neighbor);
			if (GetTanSqr(Chunk, Current, Neighbor) > MaxSlopeTanSqr) MoveCost *= SlopeViolationPanelty;

			// continue if 'visited && lower cost'
			float NewCost = NodeNow.GCost + MoveCost;
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


	} // while end.


		// if some gates are not found, It'll just end after looking every cell.
		if (DrawDebug)
		{
			for (auto& Elem : OutGates)
			{
				FIntPoint A = Elem.Value.Key.A;
				FIntPoint B = Elem.Value.Key.B;
				float Cost = Elem.Value.Value;
				UE_LOG(LogTemp, Warning, TEXT("A %s B %s Cost %f"), *A.ToString(), *B.ToString(), Cost);
				DrawDebugPoint(pLM->GetWorld(), pLM->GridToVector(A), 8.f, FColor::Red, true);
				DrawDebugPoint(pLM->GetWorld(), pLM->GridToVector(B), 8.f, FColor::Red, true);
			}
		}
		return;

}

// find path from StartGate.B to EndGate.A, but do it on (0,0) local chunk.
// returns global FIntPoint path.
// always return SGate.A, SGate.B, ~Calculated Path~ , EGate.A, EGate.B.
bool FPathFinder::GetPath(const FGate& StartGate, const FGate& EndGate, TArray<FIntPoint>& OutPath, bool DrawDebug)
{
	FIntPoint Chunk = GetChunk(StartGate.B);
	if (Chunk != GetChunk(EndGate.A))
	{
		UE_LOG(LogTemp, Warning, TEXT("GetPath input gates Chunk error, %s %s"), *Chunk.ToString(), *GetChunk(EndGate.A).ToString());
		return false;
	}

	if (StartGate.B == EndGate.A) // exception. SGate.B == EGate.A.
	{
		OutPath.Empty();
		OutPath.Add(StartGate.A);
		OutPath.Add(StartGate.B);
		OutPath.Add(EndGate.A);
		OutPath.Add(EndGate.B);
		return true;
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
			ReversePath.Add( GlobalToLocal(Chunk, EndGate.B) );
			while (Current != NoConnection)
			{
				ReversePath.Add(Current);
				Current = Frontier[GetFlatIndex(Current)].CameFrom;
			}
			ReversePath.Add( GlobalToLocal(Chunk, StartGate.A) );
			OutPath.Empty();
			OutPath.SetNum(ReversePath.Num());
			for (int32 i = 0; i < ReversePath.Num(); i++)
			{
				OutPath[i] = LocalToGlobal( Chunk, ReversePath[ReversePath.Num() - 1 - i] );
			}
			//UE_LOG(LogTemp, Warning, TEXT("GetPath WhileCount, %d"), Counter);
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

			// multiply movecost if slope violated.
			float MoveCost = GetMoveCost(Chunk, Current, Neighbor);
			if (GetTanSqr(Chunk, Current, Neighbor) > MaxSlopeTanSqr) MoveCost *= SlopeViolationPanelty;

			// continue if 'visited && lower cost'
			float NewCost = NodeNow.GCost + MoveCost;
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
// returnss global FIntPoint path
// return always has Sgate AB, EGate AB
void FPathFinder::SmoothPath(TArray<FIntPoint>& Path)
{
	if (Path.IsEmpty() || Path.Num() < 5) // at least 5 to do some skipping. (StartGateAB, path, EndgateAB)
	{
		return;
	}

	//--------------Walkable Check-----------------(skipping)

	FIntPoint Chunk = GetChunk(Path[1]);
	TArray<FIntPoint> SmoothPath;

	SmoothPath.Add(Path[0]);	// keep StartGate.A
	SmoothPath.Add(Path[1]);	// keep StartGate.B

	int32 LastIndex = Path.Num() - 1;
	int32 CheckPoint = 1;
	int32 CurrentPoint = 2;
	
	while (CurrentPoint <= LastIndex - 2)
	{
		if (IsWalkable(Chunk, GlobalToLocal(Chunk, Path[CheckPoint]), GlobalToLocal(Chunk, Path[CurrentPoint])))
		{
			CurrentPoint++;
		}
		else // one point before was walkable.
		{
			CheckPoint = CurrentPoint - 1;
			SmoothPath.Add(Path[CheckPoint]);
			CurrentPoint = CheckPoint + 1;
		}
	}

	SmoothPath.Add(Path[LastIndex - 1]);	// keep endGate.A
	SmoothPath.Add(Path[LastIndex]);		// keep endGate.B


	/*UE_LOG(LogTemp, Warning, TEXT("%s %s %s %s"), *SmoothPath[0].ToString(), *SmoothPath[1].ToString(),
		*SmoothPath[SmoothPath.Num() - 2].ToString(), *SmoothPath[SmoothPath.Num() - 1].ToString())*/


	//-----------------Radius Check-------------------
	Path.Empty();
	Path.Add(SmoothPath[0]);	// keep Startgate.A
	Path.Add(SmoothPath[1]);	// keep Startgate.B

	LastIndex = SmoothPath.Num() - 1;
	CheckPoint = 1;
	CurrentPoint = 2;
	while (CurrentPoint <= LastIndex - 2 && CheckPoint <= LastIndex - 2)
	{
		float UnitDistSqr = GetUnitDistSqr(SmoothPath[CheckPoint], SmoothPath[CurrentPoint]);
		if (UnitDistSqr < FMath::Square( UnitMinTurnRadius*2 ) ) // Square( Turning Radius*2 ) (unit)
		{
			CurrentPoint++; // skip
		}
		else
		{
			CheckPoint = CurrentPoint;
			Path.Add(SmoothPath[CheckPoint]);
			CurrentPoint++;
		}
	}

	//Path.Add(SmoothPath[LastIndex - 1]);	// keep EndGate.A
	Path.Add(SmoothPath[LastIndex]);		// keep EndGate.B

}

// returns full path made from SmoothPath.
// returns global FVector Path.
// *new* returns Final Direction.
// *new* returns SGate B ~ EGate B
FVector2D FPathFinder::RebuildPath(const TArray<FIntPoint>& SmoothPath, TArray<FVector>& OutPath, const FVector2D& StartDirection = FVector2D::ZeroVector)
{
	// from StartGate B ~ EndGate B (if both gates)
	// from StartGate B ~ EndGate A (if Endgoal)

	if (SmoothPath.Num() < 2 )
	{
		return FVector2D::ZeroVector;
	}

	FIntPoint Chunk = GetChunk(SmoothPath[1]);

	OutPath.Empty();

	// init last direction.
	FVector2D LastDirection = StartDirection;
	int32 StartIndex = 1;
	if (LastDirection == FVector2D::ZeroVector)
	{
		if (SmoothPath[0] == SmoothPath[1]) 
			LastDirection = ( GridToCell(SmoothPath[2]) - GridToCell(SmoothPath[1]) ).GetSafeNormal(); // starting point
		else 
			LastDirection = ( GridToCell(SmoothPath[1]) - GridToCell(SmoothPath[0]) ).GetSafeNormal(); // gate
	}
								
	
	float TurnRadius = MinTurnRadius ;		// should fix this later. (minimal turnradius)

	int32 LastIndex = SmoothPath.Num() - 1;
	int32 IndexEnd = LastIndex;								// do it to EndGate.A, EndGate.B
	if (SmoothPath[LastIndex - 1] == SmoothPath[LastIndex]) // if global end.
	{ IndexEnd = LastIndex - 1; }							// do it to PathLast, EndGate.A


	for (int32 i = StartIndex; i <= IndexEnd - 1; i++) // make sure i+1 don't go out of index.
	{

		FVector2D Current = GridToCell(SmoothPath[i]);
		FVector2D Next = GridToCell(SmoothPath[i + 1]);


		// ARC PART (TURN)
		TArray<FVector2D> CurveArc;
		bool bGotCurve = GetCurve(LastDirection, Current, Next, CurveArc, TurnRadius); // curve arc contains current.

		// LINE PART
		FVector2D LineStart;
		if (!bGotCurve)
		{
			LineStart = Current;
			OutPath.Add(FVector(LineStart.X, LineStart.Y, pLM->GetHeight(LineStart)));
		}
		else 
		{
			for (auto& Elem : CurveArc)
			{
				FVector Temp = FVector(Elem.X, Elem.Y, pLM->GetHeight(Elem));
				OutPath.Add(Temp);

				// DrawDebugPoint(pLM->GetWorld(), Temp, 10.0f, FColor::Blue, true);
			}
			LineStart = CurveArc.Last();
		}

		float Step = pLM->VertexSpacing * 2 / 3;
		FVector2D Direction = (Next - LineStart).GetSafeNormal();
		FVector2D Temp = LineStart + Direction * Step;
		float DistSqr = FVector2D::DistSquared(LineStart, Next);
		float RadSqr = FMath::Square(pLM->VertexSpacing);

		while (FVector2D::DistSquared(LineStart, Temp) < DistSqr && FVector2D::DistSquared(Temp , Next) >= FMath::Square(Step) )
		{
			OutPath.Add( FVector( Temp.X, Temp.Y, pLM->GetHeight(Temp) ) ); // add Pivots between Current and Next
			Temp += Direction*Step; // go another step.
		}

		// we can do more work on height smoothing.

		// this does not add the actual last SmoothPath. ( point )
		LastDirection = Direction;
	} 

	// so we add actual last.
	FVector Last;
	FVector2D Last2D = GridToCell( SmoothPath[ IndexEnd ] );
	Last = FVector(Last2D.X, Last2D.Y, GetCellHeight(SmoothPath[IndexEnd]));
	OutPath.Add(Last);

	return LastDirection;

	// UE_LOG(LogTemp, Warning, TEXT("RebuildPath Num %d"), OutPath.Num());
	
}

// Macro.
void FPathFinder::GetActualPath(const FGate& StartGate, const FGate& EndGate, TArray<FVector>& OutPath)
{

	TArray<FIntPoint> Path;
	GetPath(StartGate, EndGate, Path);
	SmoothPath(Path);

	OutPath.Empty();
	RebuildPath(Path, OutPath);
	return;
}




// ---- private -----

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

FVector2D FPathFinder::GridToCell(const FIntPoint& Grid)
{
	return FVector2D(Grid.X + 0.5f, Grid.Y + 0.5f) * pLM->VertexSpacing;
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

float FPathFinder::GetGlobalMoveCost(const FIntPoint& GlobalA, const FIntPoint& GlobalB)
{
	return GetMoveCost(GlobalA, GlobalB) + ( FMath::Abs( GetCellHeight(GlobalA) - GetCellHeight(GlobalB) ) )/ pLM->VertexSpacing;
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
	float MaxAngle = FMath::Cos( FMath::DegreesToRadians( NoTurnAngle ) );
	if (FVector2D::DotProduct(Dir1, Dir2) > MaxAngle) // if turn is not needed, return false.
	{
		return false;
	}
	float CurNextDist = FVector2D::Distance(Current, Next);
	if ( CurNextDist < TurnRadius * 2 )
	{
		return false;
	}

	float NewTurnRadius = (CurNextDist / 2) * 2 / 3;

	FVector2D CenterR = Current + FVector2D(-StartDirection.Y, StartDirection.X).GetSafeNormal() * NewTurnRadius; // 90 degree turn. cw
	FVector2D CenterL = Current + FVector2D(StartDirection.Y, -StartDirection.X).GetSafeNormal() * NewTurnRadius; // 90 degree turn. ccw

	// let's figure out which turn is better
	float RArcAngle , LArcAngle;
	RArcAngle = INFLOAT;
	LArcAngle = INFLOAT;

	RArcAngle = GetArcAngle(CenterR, Current, Next, true, NewTurnRadius);	// this is CW
	LArcAngle = GetArcAngle(CenterL, Current, Next, false, NewTurnRadius); // this should go CCW.
	
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

	if (ArcAngle >= INFLOAT)
	{ return false; }

	float AngleStart = FMath::Atan2(Current.Y - Center.Y, Current.X - Center.X);
	if (AngleStart < 0.f) AngleStart += 2 * PI;
	float AngleStep = pLM->VertexSpacing * 2 / NewTurnRadius; // length of arc of the step == pLM->VertexSpacing*2 
	AngleStep /= 3;
	float ActualAngle = AngleStart + AngleStep * StepDir;
	float AngleCounter = 0 + AngleStep;

	OutRoute.Empty();
	OutRoute.Add( Current );
	while ( AngleCounter <= ArcAngle - AngleStep * 2 / 3 )
	{
		if (ActualAngle < 0.f) ActualAngle += 2 * PI;
		FVector2D Point = Center + FVector2D(FMath::Cos(ActualAngle) * NewTurnRadius, FMath::Sin(ActualAngle) * NewTurnRadius);
		OutRoute.Add(Point);
		ActualAngle += StepDir * AngleStep;
		AngleCounter += AngleStep;
	} // this doesn't include actual CircleEnd

	FVector2D CircleEnd = Center + FVector2D( FMath::Cos(AngleStart + ArcAngle * StepDir), FMath::Sin(AngleStart + ArcAngle * StepDir) ) * NewTurnRadius;
	OutRoute.Add( CircleEnd );

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

int32 FPathFinder::GetFlatIndex(const FIntPoint& Index2D, const int32& RowNum)
{
	int32 FlatIndex = Index2D.Y * RowNum + Index2D.X;
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
	const FIntPoint& A = LocalGrid; // just changing name
	int32 Boundary = pLM->VerticesPerChunk - 1;
	if (A.X >= 0 && A.X < Boundary 
		&& 
		A.Y >= 0 && A.Y < Boundary)
	{ return true; }
	else
	{ return false; }

}

bool FPathFinder::IsInBoundary(const FIntPoint& LocalGrid, const FIntPoint& BoxSize)
{
	const FIntPoint& A = LocalGrid; // just changing name
	if (A.X >= 0 && A.X < BoxSize.X
		&&
		A.Y >= 0 && A.Y < BoxSize.Y)
	{ return true; }
	else return false;
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