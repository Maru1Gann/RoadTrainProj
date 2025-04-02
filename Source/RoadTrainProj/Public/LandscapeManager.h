// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
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

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	/* Public Functions */

	UFUNCTION(CallInEditor, Category = "Landscape Manager")
	void EditorGenerateLandscape();

	void GenerateLandscape(FIntPoint VertexCount, float CellSize, int Radius);







// Only private sections below

private:
	UPROPERTY(VisibleDefaultsOnly)
	class UProceduralMeshComponent* ProceduralMeshComponent;


private:
	int32 ChunkSectionIndex = 0;

private:
	void GenerateChunkInfo(const FIntPoint ChunkCoord);
};
