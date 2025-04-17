// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "ProceduralMeshComponent.h" // FProcMeshTangent

#include "LandscapeManager.generated.h"



class FLandscapeInfoGenerator;

// Structs to use.
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
	UPROPERTY(EditAnywhere, Category = "Landscape Manager")
	bool ShouldDrawDebugPoint = true;

	UPROPERTY(EditAnywhere, Category = "Landscape Manager|ChunkUpdate")
	bool ShouldUseMultithreading = true;
	UPROPERTY(EditAnywhere, Category = "Landscape Manager|ChunkUpdate")
	float ChunkUpdateFrequency = 1.0f;
	UPROPERTY(EditAnywhere, Category = "Landscape Manager|ChunkUpdate")
	float AsyncChunkInfoUpdateFrequency = 0.1f;
	UPROPERTY(EditAnywhere, Category = "Landscape Manager|ChunkUpdate")
	float AsyncChunkUpdateFrequency = 0.2f;
	


	UPROPERTY(EditAnywhere, Category = "Landscape Manager|Height")
	TArray<FPerlinNoiseVariables> PerlinNoiseLayers;
	UPROPERTY(EditAnywhere, Category = "Landscape Manager|Height")
	bool ShouldUseHeightGeneration = true;



	// FUNCTIONS

	void UpdateLandscape();

	void UpdateChunkInfoAsync();

	void UpdateLandscapeAsync();

	/* Editor Callable Functions */
	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void GenerateLandscape();
	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void FlushForEditor();
	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void RemoveDebugPoints();


	
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;


// Only private sections below

private:
	class UProceduralMeshComponent* ProceduralMeshComponent;
	class ACharacter* PlayerCharacter;

private:
	int32 ChunkSectionIndex = 0;
	FTimerHandle ChunkUpdateTimerHandle;
	FTimerHandle AsyncChunkInfoUpdateTimerHandle;
	FTimerHandle AsyncChunkUpdateTimerHandle;

	// params for create mesh section.
	// initialize with GenerateChunkInfo.
	TArray<int32> BigTriangles;
	TArray<int32> Triangles;

	TArray<FVector> Vertices;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FProcMeshTangent> Tangents;

	FIntPoint NeededChunk;
	TPair<FIntPoint, int32> RemovableChunk;


	// Chunk Generation Order
	TArray<FIntPoint> ChunkOrder;

	// Chunk Status
	// <ChunkCoord, ChunkSectionIndex>
	// TMap<key, value>
	TMap<FIntPoint, int32> ChunkStatus;

private:
	void GenerateChunkInfo(const FIntPoint ChunkCoord = FIntPoint(0,0));

	void GenerateChunkOrder(const int RadiusByCount);

	bool FindNeededChunk(FIntPoint& OutNeededChunk, const FIntPoint ChunkCoord);

	bool FindRemovableChunk(TPair<FIntPoint, int32>& OutRemovableChunk, const FIntPoint ChunkCoord);

	void UpdateSingleChunk(const int32 SectionIndex);

	void DrawSingleChunk(const FIntPoint ChunkCoord);


	// flip flop
	bool IsChunkInfoReady = false;
	bool IsChunkInfoGenerating = false;


	/* Tools */

	void ResetChunkInfo();

	bool IsChunkInRadius(const FIntPoint StartChunk, const FIntPoint ChunkCoord, const float RadiusByLength);

	FVector2D GetChunkCenter(const FIntPoint& ChunkCoord);

	FIntPoint GetPlayerLocatedChunk();
	
	void DrawDebugPoints();

	float GenerateHeight(const FVector2D& Location);

	FAsyncTask<FLandscapeInfoGenerator>* AsyncInfoTask;
	
};


class FLandscapeInfoGenerator : public FNonAbandonableTask
{
	friend class FAutoDeleteAsyncTask< FLandscapeInfoGenerator >;

public:
	FLandscapeInfoGenerator(ALandscapeManager* LandscapeManager) : LandscapeManager(LandscapeManager) {};

	void DoWork();

	// Probably declares the Task to the TaskGraph
	FORCEINLINE TStatId GetStatId() const 
	{ 
		RETURN_QUICK_DECLARE_CYCLE_STAT(FLandscapeInfoTask, STATGROUP_ThreadPoolAsyncTasks);
	}


private:
	ALandscapeManager* LandscapeManager;
};