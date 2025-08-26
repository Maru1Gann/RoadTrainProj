
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
    
private:

    ALandscapeManager* pLM; // don't change member values!!

    // -----------------tools-----------------

    float GetHeight(const FIntPoint& GlobalGrid);
    float GetCellHeight(const FIntPoint& GlobalGrid);
    float GetCellHeight(const FIntPoint& Chunk, const FIntPoint& LocalGrid);
    FIntPoint LocalToGlobal(const FIntPoint& Chunk, const FIntPoint& LocalGrid);
    FIntPoint GlobalToLocal(const FIntPoint& Chunk, const FIntPoint& GlobalGrid);
    

    int32 GetFlatIndex(const FIntPoint& Index2D);
    FIntPoint GetIndex2D(const int32& FlatIndex);
    int32 GetUnitDistSqr(const FIntPoint& A, const FIntPoint& B);
    void GetNeighbors(const FIntPoint& A, TArray<FIntPoint>& OutNeighbors);
    bool IsInBoundary(const FIntPoint& A);
    FIntPoint GetChunk(const FIntPoint& GlobalGrid);
    float GetMoveCost(const FIntPoint& A, const FIntPoint& B);
};

struct FGate
{
    FGate() {};
    FGate(const FIntPoint& A) : A(A), B(A) {};
    FGate(const FIntPoint& A, const FIntPoint& B) : A(A), B(B) {};
    FIntPoint A, B;
};