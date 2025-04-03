// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"

#include "LandscapeManager.generated.h"


UCLASS()
class ROADTRAINPROJ_API ALandscapeManager : public AActor
{
	GENERATED_BODY()
	
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


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;


	/* Editor Callable Functions */
	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void GenerateLandscape();
	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void Flush();
	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void DrawDebugPoints();
	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void RemoveDebugPoints();



	

// Only private sections below

private:
	class UProceduralMeshComponent* ProceduralMeshComponent;


private:
	int32 ChunkSectionIndex = 0;

	// params for create mesh section.
	// initialize with GenerateChunkInfo.
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FProcMeshTangent> Tangents;

private:
	void GenerateChunkInfo(const FIntPoint ChunkCoord = FIntPoint(0,0));

	void EmptyChunkInfo();
};
