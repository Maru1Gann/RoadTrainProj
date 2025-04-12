// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ChunkInfoVariables.h"

#include "ProceduralMeshComponent.h" // FProcMeshTangent

#include "LandscapeManager.generated.h"



class FLandscapeInfoGenerator;

// we'll use this struct only in this header, so just declared it here.
USTRUCT(Atomic)
struct FPerlinNoiseVariables
{
	GENERATED_BODY()

	FPerlinNoiseVariables(
		const float& NoiseScale = 0, 
		const float& Amplitude = 0, 
		const float& Offset = 0
	) : NoiseScale(NoiseScale), Amplitude(Amplitude), Offset(Offset)
	{};

	UPROPERTY(EditAnywhere)
	float NoiseScale;
	
	UPROPERTY(EditAnywhere)
	float Amplitude;

	UPROPERTY(EditAnywhere)
	float Offset;

};


UCLASS()
class ROADTRAINPROJ_API ALandscapeManager : public AActor
{
	GENERATED_BODY()

	friend class FLandscapeInfoGenerator;
	
public:	
	// Sets default values for this actor's properties
	ALandscapeManager();


	/* Public variables */
	UPROPERTY(EditAnywhere, Category = "Landscape Manager")
	FIntPoint ChunkVertexCount;
	
	UPROPERTY(EditAnywhere, Category = "Landscape Manager")
	float CellSize;
	
	UPROPERTY(EditAnywhere, Category = "Landscape Manager")
	int RadiusByChunkCount;
	
	UPROPERTY(EditAnywhere, Category = "Landscape Manager")
	UMaterialInterface* LandscapeMaterial = nullptr;

	UPROPERTY(EditAnywhere, Category = "Landscape Manager|Height")
	TArray<FPerlinNoiseVariables> PerlinNoiseLayers;

	UPROPERTY(EditAnywhere, Category = "Landscape Manager|Height")
	bool ShouldUseHeightGeneration = true;

	UPROPERTY(EditAnywhere, Category = "Landscape Manager")
	bool ShouldDrawDebugPoint = true;


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;


	// Blueprint Callable Functions
	UFUNCTION(BlueprintCallable)
	bool TryUpdatePlayerLocatedChunk();
	
	UFUNCTION(BlueprintCallable)
	void UpdateLandscapeInfoAsync();
	UFUNCTION(BlueprintCallable)
	void UpdateLandscapeAsync();
	UFUNCTION(BlueprintCallable)
	void UpdateLandscapeByChunkAsync();
	
	UFUNCTION(BlueprintCallable)
	void UpdateLandscape();


	

	/* Editor Callable Functions */
	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void GenerateLandscape();
	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void Flush();
	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void RemoveDebugPoints();


	

// Only private sections below

private:
	class UProceduralMeshComponent* ProceduralMeshComponent;
	class ACharacter* PlayerCharacter;

private:
	int32 ChunkSectionIndex = 0;
	FIntPoint PlayerLocatedChunk = FIntPoint(0,0);

	// params for create mesh section.
	// initialize with GenerateChunkInfo.
	TArray<int32> BigTriangles;
	TArray<int32> Triangles;

	FChunkInfoVariables ChunkInfo;


	// Chunk Generation Order
	TArray<FIntPoint> ChunkOrder;

	// Chunk Status
	// <ChunkCoord, ChunkSectionIndex>
	// TMap<key, value>
	TMap<FIntPoint, int32> ChunkStatus;
	TMap<FIntPoint, int32> RemovableChunks;
	TArray<FIntPoint> NeededChunks;
	TArray<bool> NeededChunkInfoReady;

private:
	void GenerateChunkInfo(const FIntPoint ChunkCoord = FIntPoint(0,0));

	void GenerateChunkOrder(const int RadiusByCount);

	void UpdateLandscapeInfo(const FIntPoint ChunkCoord);

	void UpdateSingleChunk(const int32 SectionIndex, const FIntPoint ChunkCoord);

	void DrawSingleChunk(const FIntPoint ChunkCoord);

	/* Async Variables */

	bool IsLandscapeUpdateDone = false;
	bool IsLandscapeInfoReady = false;
	int32 LastAbandonedIndex = 0;

	/* Tools */

	bool IsChunkInRadius(const FIntPoint StartChunk, const FIntPoint ChunkCoord, const float RadiusByLength);

	FVector2D GetChunkCenter(const FIntPoint& ChunkCoord);

	FIntPoint GetPlayerLocatedChunk();
	
	void DrawDebugPoints(const FIntPoint& ChunkCoord = FIntPoint(0,0));

	float GenerateHeight(const FVector2D& Location);

	FAsyncTask<FLandscapeInfoGenerator>* AsyncInfoGenerator;
	
};


class FLandscapeInfoGenerator : public FNonAbandonableTask
{
	friend class FAsyncTask<FLandscapeInfoGenerator>;

public:
	FLandscapeInfoGenerator(ALandscapeManager* LandscapeManager):LandscapeManager(LandscapeManager) {};

	void DoWork();

	// Probably declares the Task to the TaskGraph
	FORCEINLINE TStatId GetStatId() const 
	{ 
		RETURN_QUICK_DECLARE_CYCLE_STAT(FLandscapeInfoTask, STATGROUP_ThreadPoolAsyncTasks);
	}


private:
	ALandscapeManager* LandscapeManager;
};