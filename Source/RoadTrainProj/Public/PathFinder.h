// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "RMCLandscape.h"


class FPathFinder : public FRunnable
{
	FPathFinder(const ARMCLandscape& RMCLandscape, FVector2D Begin, FVector2D End, float Slope)
	: Begin(Begin)
	, End(End)
	, VertexSpacing(RMCLandscape.VertexSpacing)
	, VerticesPerChunk(RMCLandscape.VerticesPerChunk)
	, SlopeSquared(Slope*Slope)
	{
		End3D = ConvertTo3D(End);
		ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;
		Thread = FRunnableThread::Create(this, TEXT("PathFinder"));
	} 

public:
	virtual bool Init() override;
	virtual uint32 Run() override;
	virtual void Exit() override;
	virtual void Stop() override;

	FRunnableThread* Thread;

private:
	const FVector2D Begin;
	const FVector2D End;
	FVector End3D;
	const float SlopeSquared;

	const float VertexSpacing;
	const int32 VerticesPerChunk;
	float ChunkLength;
	const TArray<FPerlinNoiseVariables> NoiseLayers;

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
