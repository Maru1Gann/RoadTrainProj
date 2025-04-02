// Fill out your copyright notice in the Description page of Project Settings.


#include "LandscapeManager.h"
#include "ProceduralMeshComponent.h"
#include "KismetProceduralMeshLibrary.h"

// Sets default values
ALandscapeManager::ALandscapeManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	ProceduralMeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh"));
	RootComponent = ProceduralMeshComponent;

}

// Called when the game starts or when spawned
void ALandscapeManager::BeginPlay()
{
	Super::BeginPlay();


}

// Called every frame
void ALandscapeManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void ALandscapeManager::EditorGenerateLandscape()
{

}


void ALandscapeManager::GenerateChunkInfo(const FIntPoint ChunkCoord)
{

	// Offset is ChunkCoordination * ChunkVertexCount * CellSize
	FVector2D Offset = 
		FVector2D(	ChunkCoord.X * this->ChunkVertexCount.X , 
					ChunkCoord.Y * this->ChunkVertexCount.Y) * this->CellSize;


	// Declarations
	/* TArray<FVector> Vertices;
	FVector Vertex;

	TArray<FVector2D> UVs;
	FVector2D UV;

	TArray<FVector> Normals;
	TArray<FProcMeshTangent> Tangents;
 */
	//UKismetProceduralMeshLibrary::CalculateTangentsForMesh(Vertices, Triangles, UVs, Normals, Tangents);
	//CreateMeshSection(	this->ChunkSectionIndex, )
}
