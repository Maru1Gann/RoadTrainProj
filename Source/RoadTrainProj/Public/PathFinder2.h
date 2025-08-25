
#pragma once

#include "CoreMinimal.h"

class ALandscapeManager;
struct FGate;
struct FPathPoint;

class FPathFinder2
{
    friend class ALandscapeManager;

public:
    FPathFinder2(ALandscapeManager* pLM);

    bool GetPath(const FIntPoint& Chunk, const FGate& Start, const FGate& End, TArray<FVector2D>& OutPath);

private:
    ALandscapeManager* pLM; // don't change member values!!
    float CosMaxAngle;
    float MaxSlopeSquared;
    float StepLength;

    TArray<FVector2D> CirclePoints;

    void GetCirclePoints(const int32& Num);
    float GetHeight(const FIntPoint& Chunk, const FVector2D& Local);
    FVector ConvertTo3D(const FIntPoint& Chunk, const FVector2D& Local);
    FIntPoint SnapToGrid(const FVector2D& Pos);
    FIntPoint GetChunk(const FVector2D& Pos);
    FIntPoint GetChunk(const FVector& Pos);
    float GetSlopeSquared(const FVector& A, const FVector& B);

    void GetNeighbors(const FVector2D& PosNow, TArray<FVector2D>& Neighbors);
    float GetCosAngle(const FVector2D& Last, const FVector2D& Now, const FVector2D& Next);

    
};

struct FGate // if end or beginning, set it  A == B
{
    FGate() {};
    FGate(const FVector& A) : A(A), B(A) {};
    FGate(const FVector& A, const FVector& B) : A(A), B(B) {};
    FVector A, B;
};

struct FPathPoint
{
    FPathPoint() { CameFrom = -1; };
    FPathPoint(const FVector2D& Loc, const float& Cost, const int32& CameFrom) : 
    Loc(Loc), Cost(Cost), CameFrom(CameFrom) {};
    FVector2D Loc;
    float Cost;
    int32 CameFrom;
};