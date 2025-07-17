// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

class FPathFinder : public FRunnable
{
public:
	FPathFinder(class ARMCLandscape* RMCLandscape, const FVector2D& Begin, const FVector2D& End, const float& Slope);

	virtual bool Init() override;
	virtual uint32 Run() override;
	virtual void Exit() override;
	virtual void Stop() override;

	FRunnableThread* Thread;

private:
	ARMCLandscape* RMCLandscape;
	FVector2D Begin;
	FVector2D End;
	FVector End3D;
	float SlopeSquared;
	bool ShouldGenerateHeight;

	float VertexSpacing;
	int32 VerticesPerChunk;
	float ChunkLength;
	TArray<struct FPerlinNoiseVariables> NoiseLayers;

	TArray<FVector2D> Path;

	void FindPath();
	// forward declaration of 'Node'
	struct Node GetBestGate(const FIntPoint& Chunk, const FVector2D& Start, const TSet<FVector2D>& ChunkSide);

	// tools
	float GetHeight(const FVector2D& Location);
	float GetHeight(const FVector& Location);
	float GetDistSquared(const FVector& Start, const FVector& Dest);
	float GetDistSquared(const FVector2D& Start, const FVector2D& Dest);
	FVector ConvertTo3D(const FVector2D& Vector2D);

	FIntPoint GetChunk(const FVector2D& Location);

	float Heuristic(const FVector2D& Current);
	float Heuristic(const FVector& Current);
	float GetSlopeSquared(const FVector& Current, const FVector& Next);

	bool IsInBoundary(const FIntPoint& Chunk, const FVector2D& Pos);

	TSet<FVector2D> GetDestSide(const FIntPoint& StartChunk, const FIntPoint& DestChunk);

};
