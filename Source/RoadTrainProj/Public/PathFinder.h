
#pragma once

#include "CoreMinimal.h"

class ALandscapeManager;
struct FGate;

class FPathFinder
{

public:
    FPathFinder(ALandscapeManager* pLM);
    // friend ALandscapeManager; // debug

    // HPA*
    bool GetGatePath(const FIntPoint& StartCell, const FIntPoint& EndCell, TArray<FGate>& OutGatePath);
    void GetGates(const FGate& StartGate, const FIntPoint& GlobalGoal, TMap<FIntPoint, TPair<FGate, float>>& OutGates, bool DrawDebug = false);
    bool GetPath(const FGate& StartGate, const FGate& EndGate, TArray<FIntPoint>& OutPath, bool DrawDebug = false);

    // Path Smoother
    void SmoothPath( TArray<FIntPoint>& Path );
    void RebuildPath(const TArray<FIntPoint>& SmoothPath, TArray<FVector>& OutPath);
    
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
    FVector2D GridToCell(const FIntPoint& Grid);
    FVector LocalToGlobal(const FIntPoint& Chunk, const FVector2D& Local);

    int32 GetFlatIndex(const FIntPoint& Index2D);
    int32 GetFlatIndex( const FIntPoint& Index2D, const int32& RowNum );
    FIntPoint GetIndex2D(const int32& FlatIndex);

    int32 GetUnitDistSqr(const FIntPoint& A, const FIntPoint& B);
    float GetDistSqr(const FVector2D& A, const FVector2D& B);

    void GetNeighbors(const FIntPoint& A, TArray<FIntPoint>& OutNeighbors);
    bool IsNeighbor(const FIntPoint& A, const FIntPoint& B);
    bool IsInBoundary(const FIntPoint& LocalGrid);
    bool IsInBoundary(const FIntPoint& LocalGrid, const FIntPoint& BoxSize);
    bool IsOnBoundary(const FIntPoint& LocalGrid);
    FIntPoint GetChunk(const FIntPoint& GlobalGrid);

    float GetMoveCost(const FIntPoint& A, const FIntPoint& B);
    float GetGlobalMoveCost(const FIntPoint& A, const FIntPoint& B);
    float GetMoveCost(const FIntPoint& Chunk, const FIntPoint& A, const FIntPoint& B);

    float GetTanSqr(const FIntPoint& Chunk, const FIntPoint& A, const FIntPoint& B);
    float GetTanSqr(const FIntPoint& Chunk, const FVector2D& LocalA, const FVector2D& LocalB);

    bool GetCurve(const FVector2D& StartDirection, const FVector2D& Current, const FVector2D& Next, TArray<FVector2D>& OutRoute, 
        const float& TurnRadius = 15000.0f, const float& NoTurnAngle = 20.0f);
    float GetArcAngle(const FVector2D& Center, const FVector2D& Current, const FVector2D& Next, const bool& IsRightTurn, const float& TurnRadius = 1500.0f);

};

struct FGate
{
    FGate() {};
    FGate(const FIntPoint& A) : A(A), B(A) {};
    FGate(const FIntPoint& A, const FIntPoint& B) : A(A), B(B) {};
    FIntPoint A, B;
};