// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "RealtimeMeshActor.h"
#include "RealtimeMeshSimple.h"

#include "PerlinNoiseVariables.h"

// must be last
#include "RMCLandscape.generated.h"

UCLASS()
class ROADTRAINPROJ_API ARMCLandscape : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARMCLandscape();
	virtual void OnConstruction(const FTransform& Transform) override;


	// Called every frame
	virtual void Tick(float DeltaTime) override;


	UPROPERTY(EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 1, ClampMin = "10.0", Step = "10.0", Units = "cm"))
	float VertexSpacing = 300.0f;

	UPROPERTY(EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 2, ClampMin = "2", Step = "2"))
	int32 VerticesPerChunk = 10;

	UPROPERTY(EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 3, ClampMin = "0", Step = "1"))
	int32 ChunkRadius = 5;

	UPROPERTY(EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 4) )
	TArray<FPerlinNoiseVariables> PerlinNoiseLayers;



protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;



private:
	// Store all the chunks HERE
	TMap<FIntPoint, ARealtimeMeshActor*> ChunkGrid;
	
	// Sprial from center for generation priority
	TArray<FIntPoint> ChunkOrder;

	// Spiral Init.
	void GenerateChunkOrder();

	// Init in OnConstruction()
	float ChunkLength;
	float ChunkRadiusByLength;

	// Get Center point of given ChunkCoord
	FVector2D GetChunkCenter(const FIntPoint& ChunkCoord);

	// Get ChunkCoord of Player
	FIntPoint GetPlayerLoactedChunk();

	// Get if selected two chunks are in distance.
	bool IsChunkInRadius(const FIntPoint& Target, const FIntPoint& Start);

	// Generate Height with PerlinNoise
	float GenerateHeight(const FVector2D& Location);

};
