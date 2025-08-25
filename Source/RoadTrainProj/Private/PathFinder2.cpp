
#include "PathFinder2.h"
#include "LandscapeManager.h"

#include <limits>
const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for obstacles

FPathFinder2::FPathFinder2(ALandscapeManager* pLM) : pLM(pLM)
{
	float RadAngle = FMath::DegreesToRadians(pLM->MaxRoadAngle);
	CosMaxAngle = FMath::Cos(RadAngle);
	MaxSlopeSquared = FMath::Square(pLM->MaxSlope);

	GetCirclePoints(pLM->CircleSampleNum);
}

// returns false if path not found
bool FPathFinder2::GetPath(const FIntPoint& Chunk, const FGate& StartGate, const FGate& EndGate, TArray<FVector2D>& OutPath)
{
	if (GetChunk(StartGate.B) != Chunk || GetChunk(EndGate.A) != Chunk)
	{
		UE_LOG(LogTemp, Warning, TEXT("GetPath InputGate Error"));
		return false;
	}

	FVector2D Offset = FVector2D(Chunk.X, Chunk.Y) * pLM->VertexSpacing * (pLM->VerticesPerChunk - 1);
	FVector2D LocalStartFrom = FVector2D(StartGate.A.X, StartGate.A.Y) - Offset;
	FVector2D LocalStart = FVector2D(StartGate.B.X, StartGate.B.Y) - Offset;
	FVector2D LocalEnd = FVector2D(EndGate.A.X, EndGate.A.Y) - Offset;
	FVector VecEnd = EndGate.A;

	TArray<FPathPoint> PathPoints;
	PathPoints.Add(FPathPoint(LocalStartFrom, 0, -1));
	FPathPoint StartPoint(LocalStart, 0, PathPoints.Num()-1 );
	PathPoints.Add(StartPoint);

	TMap<FIntPoint, int32> VisitMap;
	VisitMap.Add(SnapToGrid(StartPoint.Loc), PathPoints.Num()-1);

	TArray<TPair<float, int32>> Frontier; // key == priority. value == index of the PathPoint
	auto Predicate = [](const TPair<float, int32>& A, const TPair<float, int32>& B) { return A.Key < B.Key; }; // min heap.
	Frontier.Heapify(Predicate); // context purpose.
	Frontier.Add(TPair<float, int32>(0, PathPoints.Num()-1));

	while ( !Frontier.IsEmpty() )
	{
		TPair<float, int32> Current;
		Frontier.HeapPop(Current);

		FPathPoint PointNow = PathPoints[Current.Value];
		FPathPoint PointLast = PathPoints[PointNow.CameFrom];

		// met goal conditions ( if goal is in radius, and angle and slope ok )
		FVector VecNow = ConvertTo3D(Chunk, PointNow.Loc);
		if ( FVector::DistSquared(VecNow, VecEnd) <= FMath::Square(pLM->VertexSpacing) )
		{
			// now check angle and slope
			float SlopeSquared = GetSlopeSquared(VecNow, VecEnd);
			float CosAngle = GetCosAngle(PointLast.Loc, PointNow.Loc, LocalEnd);
			if (!(SlopeSquared > MaxSlopeSquared || CosAngle < CosMaxAngle)) // if all good
			{
				// pass data
				TArray<FVector2D> ReversePath;
				ReversePath.Add(LocalEnd);
				int32 Index = Current.Value;
				while (Index > 0) // index 0 == camefrom of starting point
				{
					ReversePath.Add(PathPoints[Index].Loc);
					Index = PathPoints[Index].CameFrom;
				}

				OutPath.Empty();
				OutPath.Reserve(ReversePath.Num());
				for (int32 i = 0; i < ReversePath.Num() - 1; i++)
				{ OutPath[i] = ReversePath[ReversePath.Num() - 1 - i]; }
				return true;
			}
		}

		// didn't meet goal.
		TArray<FVector2D> Neighbors;
		GetNeighbors(PointNow.Loc, Neighbors);
		for (auto& LocNext : Neighbors) // for all neighbors
		{
			// FVector VecNow = ConvertTo3D(Chunk, PointNow.Loc);
			FVector VecNext = ConvertTo3D(Chunk, LocNext);

			// checking slope and angle(road curvature)
			float SlopeSquared = GetSlopeSquared(VecNow, VecNext);
			float CosAngle = GetCosAngle(PointLast.Loc, PointNow.Loc, LocNext);
			if ( SlopeSquared > MaxSlopeSquared || CosAngle < CosMaxAngle ) // CosMaxAngle == -1 when MaxAngle == 180
			{ continue; }

			float NewCost = PointNow.Cost + FVector::DistSquared(VecNow, VecNext);
			FPathPoint PointNext(LocNext, NewCost, Current.Value);

			if (SnapToGrid(PointNow.Loc) != SnapToGrid(PointNext.Loc)) // if we're visiting new grid.
			{
				int32* pIndex = VisitMap.Find(SnapToGrid(PointNext.Loc));
				float OldCost = INFLOAT;
				if (pIndex)
				{ OldCost = PathPoints[*pIndex].Cost; }

				if (OldCost > NewCost) // this also covers 'if pIndex == nullptr'
				{
					PathPoints.Add(PointNext);
					VisitMap.Add(SnapToGrid(LocNext), PathPoints.Num() - 1);
				}
			}
			else // if not visiting new grid.
			{
				PathPoints.Add(PointNext);
			}

			float Priority = NewCost + FVector::DistSquared( VecNext, ConvertTo3D(Chunk, LocalEnd) ); // heuristic
			// add slope and angle bias
			float Bias = SlopeSquared * pLM->SlopePaneltyWeight + (1 - CosAngle) * pLM->DirectionPaneltyWeight;
			Frontier.HeapPush(TPair<float, int32>(Priority, PathPoints.Num() - 1));
		}
	}

	return false;
}

void FPathFinder2::GetCirclePoints(const int32& Num)
{
	if (CirclePoints.Num() == Num)
	{
		return;
	}

	CirclePoints.Empty();
	CirclePoints.Reserve(Num);
	float Rad = pLM->VertexSpacing;
	float AngleDivided = 2 * PI / Num;
	
	for (int32 i = 0; i < Num; i++)
	{
		float Angle = AngleDivided * i;
		FVector2D Point;
		Point.X = Rad * FMath::Cos(Angle);
		Point.Y = Rad * FMath::Sin(Angle);
		CirclePoints.Add(Point);
	}
}

float FPathFinder2::GetHeight(const FIntPoint& Chunk, const FVector2D& Local)
{
	FVector2D Offset = FVector2D(Chunk.X, Chunk.Y) * pLM->VertexSpacing * (pLM->VerticesPerChunk - 1);
	return pLM->GetHeight(Offset + Local);
}

FVector FPathFinder2::ConvertTo3D(const FIntPoint& Chunk, const FVector2D& Local)
{
	FVector2D Offset = FVector2D(Chunk.X, Chunk.Y) * pLM->VertexSpacing * (pLM->VerticesPerChunk - 1);
	float Height = pLM->GetHeight(Offset + Local);
	return FVector(Offset.X + Local.X, Offset.Y + Local.Y, Height);
}

FIntPoint FPathFinder2::SnapToGrid(const FVector2D& Pos)
{
	int32 X = FMath::FloorToInt32(Pos.X / pLM->VertexSpacing);
	int32 Y = FMath::FloorToInt32(Pos.Y / pLM->VertexSpacing);
	return FIntPoint(X,Y);
}

FIntPoint FPathFinder2::GetChunk(const FVector2D& Pos)
{
	float ChunkLength = (pLM->VerticesPerChunk - 1) * pLM->VertexSpacing;
	int32 X = FMath::FloorToInt32(Pos.X / ChunkLength);
	int32 Y = FMath::FloorToInt32(Pos.Y / ChunkLength);
	return FIntPoint(X,Y);
}

FIntPoint FPathFinder2::GetChunk(const FVector& Pos)
{
	return GetChunk(FVector2D(Pos.X, Pos.Y));
}

float FPathFinder2::GetSlopeSquared(const FVector& A, const FVector& B)
{
	float BaseSquared = FVector::DistSquaredXY(A, B);
	float HeightSquared = FMath::Square(A.Z - B.Z);
	return HeightSquared / BaseSquared;
}

void FPathFinder2::GetNeighbors(const FVector2D& PosNow, TArray<FVector2D>& Neighbors) 
{
	for (auto& Point : CirclePoints)
	{
		FVector2D PosNext = PosNow + Point;
		Neighbors.Add(PosNext);
	}
}

float FPathFinder2::GetCosAngle(const FVector2D& Last, const FVector2D& Now, const FVector2D& Next)
{
	FVector2D LastToNow = FVector2D(Now.X, Now.Y) - FVector2D(Last.X, Last.Y);
	FVector2D NowToNext = FVector2D(Next.X, Next.Y) - FVector2D(Now.X, Now.Y);

	if (LastToNow.IsNearlyZero())
	{
		return 1; // Initial startpoint of pathfinding. last point == now point
	}
	if (NowToNext.IsNearlyZero())
	{
		UE_LOG(LogTemp, Warning, TEXT("IsAngleValid Error."));
		return INFLOAT;
	}

	LastToNow.Normalize();
	NowToNext.Normalize();
	float CosAngle = FVector2D::DotProduct(LastToNow, NowToNext);

	return CosAngle;
}
