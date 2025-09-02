
#pragma once

#include "CoreMinimal.h"

class ALandscapeManager;
struct FGate;

class FPathFinder
{

public:
    FPathFinder(ALandscapeManager* pLM);
    friend ALandscapeManager; // debug

    bool GetPath(const FGate& StartGate, const FGate& EndGate, TArray<FIntPoint>& OutPath, bool DrawDebug = false);
    void SmoothPath( const FIntPoint& Chunk, TArray<FIntPoint>& Path );
    void RebuildPath(const FIntPoint& Chunk, const TArray<FIntPoint>& SmoothPath, TArray<FVector>& OutPath);
    
private:

    ALandscapeManager* pLM; // don't change member values!!
    float MaxSlopeTanSqr;

    // -----------------tools-----------------

    bool IsWalkable(const FIntPoint& Chunk, const FIntPoint& A, const FIntPoint& B);

    float GetHeight(const FIntPoint& GlobalGrid);
    float GetHeight(const FIntPoint& Chunk, const FVector2D& Local);
    float GetCellHeight(const FIntPoint& GlobalGrid);
    float GetCellHeight(const FIntPoint& Chunk, const FIntPoint& LocalGrid);
    FIntPoint LocalToGlobal(const FIntPoint& Chunk, const FIntPoint& LocalGrid);
    FIntPoint GlobalToLocal(const FIntPoint& Chunk, const FIntPoint& GlobalGrid);
    

    int32 GetFlatIndex(const FIntPoint& Index2D);
    FIntPoint GetIndex2D(const int32& FlatIndex);

    int32 GetUnitDistSqr(const FIntPoint& A, const FIntPoint& B);
    float GetDistSqr(const FVector2D& A, const FVector2D& B);

    void GetNeighbors(const FIntPoint& A, TArray<FIntPoint>& OutNeighbors);
    bool IsNeighbor(const FIntPoint& A, const FIntPoint& B);
    bool IsInBoundary(const FIntPoint& A);
    FIntPoint GetChunk(const FIntPoint& GlobalGrid);

    float GetMoveCost(const FIntPoint& A, const FIntPoint& B);
    float GetMoveCost(const FIntPoint& Chunk, const FIntPoint& A, const FIntPoint& B);

    float GetTanSqr(const FIntPoint& Chunk, const FIntPoint& A, const FIntPoint& B);
    float GetTanSqr(const FIntPoint& Chunk, const FVector2D& LocalA, const FVector2D& LocalB);
};

struct FGate
{
    FGate() {};
    FGate(const FIntPoint& A) : A(A), B(A) {};
    FGate(const FIntPoint& A, const FIntPoint& B) : A(A), B(B) {};
    FIntPoint A, B;
};