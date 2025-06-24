// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "RealtimeMeshActor.h"
#include "RealtimeMeshSimple.h"

#include "PerlinNoiseVariables.h"

#include "PCGComponent.h"
#include "PCGGraph.h"

// must be last
#include "RMCLandscape.generated.h"


UCLASS()
class ROADTRAINPROJ_API ARMCLandscape : public AActor
{
	GENERATED_BODY()

	friend class FStreamSetGenerator;

public:	
	// Sets default values for this actor's properties
	ARMCLandscape();
	
	virtual void OnConstruction(const FTransform &Transform) override;
	// Called every frame
	virtual void Tick(float DeltaTime) override;


	// -------------------Chunk Generation (RMC) ----------------------

	UPROPERTY( EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 1, ClampMin = "10.0", Step = "10.0", Units = "cm") )
	float VertexSpacing = 1000.0f;

	UPROPERTY( EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 2, ClampMin = "2", Step = "2") )
	int32 VerticesPerChunk = 128;

	UPROPERTY( EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 3, ClampMin = "0", Step = "1") )
	int32 ChunkRadius = 1;

	UPROPERTY( EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 4, ClampMin = "0.0", Step = "10.0") )
	float TextureSize = 300.0f;

	UPROPERTY( EditAnywhere, Category = "Chunks|Height", meta = (DisplayPriority = 1) )
	bool ShouldGenerateHeight = true;

	UPROPERTY( EditAnywhere, Category = "Chunks|Height", meta = (DisplayPriority = 2) )
	TArray<FPerlinNoiseVariables> PerlinNoiseLayers;

	UPROPERTY( EditAnywhere, Category = "Chunks|Update", meta = (DisplayPriority = 1) )
	bool bUseAsync = true;
	UPROPERTY( EditAnywhere, Category = "Chunks|Update", meta = (DisplayPriority = 2, ClampMin = "0.0", Step = "0.001") )
	float UpdatePeriod = 0.1f;

	UPROPERTY( EditAnywhere, Category = "Chunks|Material")
	UMaterialInterface* ChunkMaterial;


	void AsyncGenerateLandscape();

	UFUNCTION( CallInEditor, Category = "Chunks" )
	void GenerateLandscape();
	UFUNCTION( CallInEditor, Category = "Chunks" )
	void RemoveLandscape();

	// -------------------Chunk Generation (RMC) ----------------------

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;



private:
	// Store all the chunks HERE
	TMap<FIntPoint, ARealtimeMeshActor*> Chunks;
	// Chunk Generation Order
	TArray<FIntPoint> ChunkOrder;

	void AddChunk(const FIntPoint& ChunkCoord, const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet);

	void GenerateStreamSet(const FIntPoint& ChunkCoord, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet);

	void GenerateChunkOrder();

	bool IsDataReady = false;
	bool IsWorking = false;
	FIntPoint PlayerChunk = FIntPoint(0,0);
	// set true if updated
	TPair<FIntPoint, bool> RemovableChunk;
	TPair<FIntPoint, bool> NeededChunk;
	RealtimeMesh::FRealtimeMeshStreamSet MemberStreamSet;

	// for debugging purposes
	UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 5) )
	int32 ChunkCount = 0;
	// Init in OnConstruction.
	UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 6) )
	float ChunkLength;
	UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 7) )
	float HorizonDistance;

	// Get ChunkCoord of Player
	FIntPoint GetPlayerLocatedChunk();

	// Generate Height with PerlinNoise
	float GenerateHeight(const FVector2D& Location);

	void RemoveChunk(const FIntPoint& ChunkCoord);

	float GetElapsedInMs(const FDateTime& StartTime);

	FAsyncTask<FStreamSetGenerator>* StreamSetGenerator;

	
};

class FStreamSetGenerator : public FNonAbandonableTask
{
	friend class FAutoDeleteAsyncTask< FStreamSetGenerator >;

public:
	FStreamSetGenerator(ARMCLandscape* RMC) : RMC(RMC) { StartIndex = 0; };

	void DoWork();

	// Probably declares the Task to the TaskGraph
	FORCEINLINE TStatId GetStatId() const 
	{ 
		RETURN_QUICK_DECLARE_CYCLE_STAT(FLandscapeInfoTask, STATGROUP_ThreadPoolAsyncTasks);
	}


private:
	ARMCLandscape* RMC;
	int32 StartIndex = 0;
	
};