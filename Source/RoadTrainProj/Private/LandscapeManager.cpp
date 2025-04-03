// Fill out your copyright notice in the Description page of Project Settings.


#include "LandscapeManager.h"
#include "ProceduralMeshComponent.h"
#include "KismetProceduralMeshLibrary.h"

#include "Kismet/KismetSystemLibrary.h"

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

	GenerateLandscape();
}

// Called every frame
void ALandscapeManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}



// Editor Callable Functions
void ALandscapeManager::GenerateLandscape()
{
	// TODO : We don't need to create collisions for most of the chunks.
	if(ProceduralMeshComponent == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT(" %s : ProceduralMeshComponent nullptr"), *GetName());
		return;
	}

	Flush();

	GenerateChunkInfo();

	// debug
	if(ShouldDrawDebugPoint)
	{
		DrawDebugPoints();
	}
	// debug

	ProceduralMeshComponent->CreateMeshSection(
		ChunkSectionIndex, 
		Vertices, 
		Triangles, 
		Normals, 
		UVs, 
		TArray<FColor>(), 
		Tangents, 
		true);
	if( LandscapeMaterial )
	{
		ProceduralMeshComponent->SetMaterial(ChunkSectionIndex, LandscapeMaterial);
	}

	return;
}

void ALandscapeManager::Flush()
{
	ProceduralMeshComponent->ClearAllMeshSections();
	FlushPersistentDebugLines(this->GetWorld());
	EmptyChunkInfo();

	return;
}

void ALandscapeManager::DrawDebugPoints()
{
	for (int i = 0; i < Vertices.Num(); i++)
	{
		DrawDebugPoint(this->GetWorld(), Vertices[i], 5, FColor::Red, true);
	}
	
	return;
}

void ALandscapeManager::RemoveDebugPoints()
{
	FlushPersistentDebugLines(this->GetWorld());

	return;
}

// Editor Callable Functions

/* 	Private 
	Section 
	From HERE ↓
*/


void ALandscapeManager::GenerateChunkInfo(const FIntPoint ChunkCoord)
{

	// Offset is ChunkCoordination * ChunkVertexCount * CellSize
	FVector Offset = 
		FVector(	ChunkCoord.X * (ChunkVertexCount.X - 1), 
					ChunkCoord.Y * (ChunkVertexCount.Y - 1), 
					0.f);
	Offset = Offset * CellSize;
	
	// First we need to make a chunk that is one vertice bigger on every side. -> length + 2
	// This is because we need to make normals that continues to another chunk.
	// else we get seams at the edge of every chunk.

	// Declarations
	TArray<FVector> BigVertices;
	FVector Vertex;

	TArray<int32> BigTriangles;

	TArray<FVector2D> BigUVs;
	FVector2D UV;


	// Calculations for BigVertices. BigUVs also since it's same.
	for (int32 iY = -1;  iY <= ChunkVertexCount.Y; iY++) // this goes from -1 ~ (length + 1) == length + 2
	{
		for (int32 iX = -1; iX <= ChunkVertexCount.X;  iX++)
		{
			Vertex = FVector( iX, iY, 0.f ) * CellSize + Offset; // getting the actual vertex coordinates in worldspace
			// TODO : we need to add height generation for z axis. HERE.
			BigVertices.Add(Vertex);
			UV = FVector2D(Vertex) / CellSize; // convert to FVector2D
			BigUVs.Add(UV);	
		}
	}


	// BigTriangles are just abstract coordinates, it should go from 0 to ChunkVertexCount + 1 ( == -1 to ChunkVertexCount )
	// !!We don't use the last vertex as startingpoint!! So (ChunkVertexCount + 1) - 1
	int32 Row = ChunkVertexCount.X + 2;
	for (int32 iY = 0; iY <= ChunkVertexCount.Y; iY++)
	{
		for (int32 iX = 0; iX <= ChunkVertexCount.X; iX++)
		{
			// counter-clockwise, mark 3 vertices for triangle generation.
			// NOTE : in UnrealEngine, positive Y goes DOWN!!
			// so { (x,y), (x, y+1), (x+1, y+1) }, { (x,y), (x+1, y+1), (x+1, y) } makes square facing upwards.

			// Column = iY * Row;
			BigTriangles.Add( iX 		+ (iY * Row) 		);
			BigTriangles.Add( iX 		+ ((iY+1) * Row) 	);
			BigTriangles.Add( (iX+1) 	+ ((iY+1) * Row) 	);

			BigTriangles.Add( iX 		+ (iY * Row) 		);
			BigTriangles.Add( (iX+1)	+ ((iY+1) * Row) 	);
			BigTriangles.Add( (iX+1) 	+ (iY * Row) 		);
		}
	}

	// With Vertices, Triangles, UVs, we calculate Normals and Tangents.
	TArray<FVector> BigNormals;
	TArray<FProcMeshTangent> BigTangents;
	// Normal & Tangent is OutParameter of function below!!
	UKismetProceduralMeshLibrary::CalculateTangentsForMesh(BigVertices, BigTriangles, BigUVs, BigNormals, BigTangents);

	// Now let's generate real infos for CreateMeshSection function.
	// Triangles should be dealt separately.
	int32 VertexIndex = 0;
	for (int32 iY = -1; iY <= ChunkVertexCount.Y; iY++)
	{
		for (int32 iX = -1; iX <= ChunkVertexCount.X; iX++)
		{
			if ( iX > -1 && iY > -1 && iX < ChunkVertexCount.X && iY < ChunkVertexCount.Y ) // we remove 1 from every side of the square;
			{
				Vertices.Add( BigVertices[VertexIndex] );
				Normals.Add( BigNormals[VertexIndex] );
				UVs.Add( BigUVs[VertexIndex] );
				Tangents.Add( BigTangents[VertexIndex] );
			}
			VertexIndex++;
		}
	}

	// Triangles
	// we don't use the last vertex. so CunkVertexCount - 1.
	// starts from 0 so ChunkVertexCount - 1.
	Row = ChunkVertexCount.X;
	for (int32 iY = 0; iY <= ChunkVertexCount.Y - 2; iY++)
	{
		for (int32 iX = 0; iX <= ChunkVertexCount.X - 2; iX++)
		{
			Triangles.Add( iX 		+ (iY * Row) 		);
			Triangles.Add( iX 		+ ((iY+1) * Row) 	);
			Triangles.Add( (iX+1) 	+ ((iY+1) * Row) 	);

			Triangles.Add( iX 		+ (iY * Row) 		);
			Triangles.Add( (iX+1) 	+ ((iY+1) * Row) 	);
			Triangles.Add( (iX+1)	+ (iY * Row) 		);
		}
	}



	UE_LOG(LogTemp, Display, TEXT("Generated Chunk Info"));
	return;
}

void ALandscapeManager::EmptyChunkInfo()
{
	Vertices.Empty();
	Triangles.Empty();
	Normals.Empty();
	UVs.Empty();
	Tangents.Empty();

	return;
}

