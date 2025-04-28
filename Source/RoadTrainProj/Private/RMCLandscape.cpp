// Fill out your copyright notice in the Description page of Project Settings.


#include "RMCLandscape.h"
#include "Mesh/RealtimeMeshAlgo.h"

// Sets default values
ARMCLandscape::ARMCLandscape()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));

}

// Called when the game starts or when spawned
void ARMCLandscape::BeginPlay()
{
	Super::BeginPlay();

	GenerateLandscape();

	return;
}

// Called every frame
void ARMCLandscape::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	return;
}


void ARMCLandscape::GenerateLandscape()
{
	ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;
	ChunkRadiusByLength = ChunkLength * ChunkRadius;
	ChunkCount = 0;

	FDateTime StartTime = FDateTime::UtcNow();
	RemoveLandscape();
	UE_LOG(LogTemp, Display, TEXT("RemoveLandscape : %f ms"), GetElapsedInMs(StartTime));


	StartTime = FDateTime::UtcNow();

	FIntPoint Center = FIntPoint(0,0);
	if( GetWorld()->GetFirstPlayerController() != nullptr )
	{
		Center = GetPlayerLoactedChunk();
	}

	int32 Extent = ChunkRadius/2 + 1;
	for( int32 iY = Center.Y - Extent; iY < Center.Y + Extent; iY++)
	{
		for (int32 iX = Center.X - Extent; iX < Center.X + Extent; iX++)
		{
			StreamSet.Empty();
			AddChunk( FIntPoint(iX,iY) );
			ChunkCount++;
		}
	}

	UE_LOG(LogTemp, Display, TEXT("AddChunks : %f ms"), GetElapsedInMs(StartTime));

	return;
}

void ARMCLandscape::RemoveLandscape()
{
	
	if(Chunks.IsEmpty())
	{
		return;
	}

	for( auto& Elem : Chunks )
	{
		if(Elem.Value)
		{
			Elem.Value->Destroy();
		}

	}
	Chunks.Empty();
	ChunkCount = 0;

	
}

void ARMCLandscape::AddChunk(const FIntPoint& ChunkCoord)
{
	if( GetWorld() == nullptr )
	{
		UE_LOG(LogTemp, Warning, TEXT("GetWorld() nullptr"));
		return;
	}

	// Spawn chunk as Actor
	ARealtimeMeshActor* RMA = GetWorld()->SpawnActor<ARealtimeMeshActor>();

	if( RMA == nullptr )
	{
		UE_LOG(LogTemp, Warning, TEXT("RMA nullptr"));
		return;
	}

	// Set Location
	FVector Offset = FVector( ChunkCoord.X , ChunkCoord.Y, 0.0f ) * ChunkLength;
	RMA->SetActorLocation(Offset);

	URealtimeMeshSimple* RealtimeMesh = RMA->GetRealtimeMeshComponent()->InitializeRealtimeMesh<URealtimeMeshSimple>();

	if( RealtimeMesh == nullptr )
	{
		UE_LOG(LogTemp, Warning, TEXT("RealtimeMesh nullptr"));
		return;
	}


	RealtimeMesh->SetupMaterialSlot(0, "PrimaryMaterial");
	RealtimeMesh->UpdateLODConfig(0, FRealtimeMeshLODConfig(1.00f));

	GenerateStreamSet(ChunkCoord);

	const FRealtimeMeshSectionGroupKey GroupKey = FRealtimeMeshSectionGroupKey::Create(0, 0);
	const FRealtimeMeshSectionKey PolyGroup0SectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey, 0);

	// this generates the mesh (chunk)
	RealtimeMesh->CreateSectionGroup(GroupKey, StreamSet);

	Chunks.Add(ChunkCoord, RMA);

	// update configuration.
	RealtimeMesh->UpdateSectionConfig( PolyGroup0SectionKey, FRealtimeMeshSectionConfig(0), true );

}

void ARMCLandscape::GenerateStreamSet(const FIntPoint& ChunkCoord)
{
	// Chunk location offset
	FVector3f Offset = FVector3f( ChunkCoord.X , ChunkCoord.Y, 0.0f ) * ChunkLength;
	
	// scale UV based on vetex spacing
	float UVScale = VertexSpacing / TextureSize;

/* 	// Big data for uv and normal continue on different chunks
	TArray<FVector3f> BigVertices, BigTangents, BigNormals;
	TArray<uint32> BigTriangles;

	for(int32 iY = -1 ; iY < VerticesPerChunk + 1; iY++)
	{
		for(int32 iX = -1; iX < VerticesPerChunk + 1; iX++)
		{
			FVector3f Vertex = FVector3f(iX, iY, 0.0f) * VertexSpacing + Offset;
			if(ShouldGenerateHeight)
			{
				Vertex.Z = GenerateHeight( FVector2D(Vertex.X + Offset.X, Vertex.Y + Offset.Y) );
			}
			BigVertices.Add(Vertex);
			
		}
	}

	for (int32 iY = -1; iY < VerticesPerChunk + 1; iY++)
	{
		for (int32 iX = -1; iX < VerticesPerChunk + 1; iX++)
		{
			int32 CurrentVertex = iX + iY * VerticesPerChunk;
			BigTriangles.Add(CurrentVertex);
			BigTriangles.Add(CurrentVertex + VerticesPerChunk);
			BigTriangles.Add(CurrentVertex + 1);

			BigTriangles.Add(CurrentVertex + VerticesPerChunk);
			BigTriangles.Add(CurrentVertex + VerticesPerChunk + 1);
			BigTriangles.Add(CurrentVertex + 1);
		}
	}

	BigTangents.SetNum( BigVertices.Num() );
	BigNormals.SetNum( BigVertices.Num() );

	// Normals and Tangents from Big datas
	RealtimeMeshAlgo::GenerateTangents(
		TConstArrayView<uint32>(BigTriangles),
		BigVertices,
		nullptr, // UV Getter. We made uv.
		[&BigNormals, &BigTangents](int index, FVector3f Tangent, FVector3f Normal) -> void
		{
			BigNormals[index] = Normal;
			BigTangents[index] = Tangent;
		},
		true
	);
	
	int32 index = 0;
	for( int32 iY = 0; iY < VerticesPerChunk + 2; iY++)
	{
		for ( int32 iX = 0; iX < VerticesPerChunk + 2; iX++)
		{
			if ( iY > 0 && iY < VerticesPerChunk + 1 
				&&
				iX > 0 && iX < VerticesPerChunk + 1 )
			{
				Tangents.Add(BigTangents[index]);
				Normals.Add(BigNormals[index]);
			}
			index++;
		}
	}

 */

	TArray<FVector3f> Vertices, Tangents, Normals;
	TArray<uint32> Triangles;
	TArray<FVector2DHalf> UVs;


	// Vertices and UVs
	for(int32 iY = 0; iY < VerticesPerChunk; iY++)
	{
		for(int32 iX = 0; iX < VerticesPerChunk; iX++)
		{
			FVector3f CurrentVertex = FVector3f(iX, iY, 0.0f) * VertexSpacing;
			if( ShouldGenerateHeight )
			{
				CurrentVertex.Z = GenerateHeight( FVector2D(CurrentVertex.X + Offset.X, CurrentVertex.Y + Offset.Y) );
			}
			Vertices.Add(CurrentVertex);

			FVector2DHalf UV;
			UV.X = (ChunkCoord.X * (VerticesPerChunk - 1) + iX) * UVScale;
			UV.Y = (ChunkCoord.Y * (VerticesPerChunk - 1) + iY) * UVScale;
			UVs.Add(UV);
		}
	}


	// Triangles
	// CounterClockwise, Right -> X+  Down -> Y+ (UE)
	for (int32 iY = 0; iY < VerticesPerChunk - 1; iY++)
	{
		for (int32 iX = 0; iX < VerticesPerChunk - 1; iX++)
		{
			int32 CurrentVertex = iX + iY * VerticesPerChunk;
			Triangles.Add(CurrentVertex);
			Triangles.Add(CurrentVertex + VerticesPerChunk);
			Triangles.Add(CurrentVertex + 1);

			Triangles.Add(CurrentVertex + VerticesPerChunk);
			Triangles.Add(CurrentVertex + VerticesPerChunk + 1);
			Triangles.Add(CurrentVertex + 1);
		}
	}

	Tangents.SetNum(Vertices.Num());
	Normals.SetNum(Vertices.Num());

	// Normals and Tangents from Big datas
	RealtimeMeshAlgo::GenerateTangents(
		TConstArrayView<uint32>(Triangles),
		Vertices,
		nullptr, // UV Getter. We made uv.
		[&Normals, &Tangents](int index, FVector3f Tangent, FVector3f Normal) -> void
		{
			Normals[index] = Normal;
			Tangents[index] = Tangent;
		},
		true
	);




	// Datas into StreamSet( class Member Var )
	RealtimeMesh::TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 1> Builder(StreamSet);
	Builder.EnableTangents();
	Builder.EnableTexCoords();
	Builder.EnableColors();

	Builder.EnablePolyGroups();

	for (int32 i = 0; i < Vertices.Num(); i++)
	{
		Builder.AddVertex( Vertices[i] )
			.SetNormalAndTangent( Normals[i], Tangents[i] )
			.SetColor(FColor::White)
			.SetTexCoord(UVs[i]);
	}

	for (int32 i = 0; i < Triangles.Num(); i+=6)
	{
		Builder.AddTriangle(
			Triangles[i],
			Triangles[i + 1],
			Triangles[i + 2],
			0
		);

		Builder.AddTriangle(
			Triangles[i + 3],
			Triangles[i + 4],
			Triangles[i + 5],
			0
		);
	}

	return;
}




/* Tools */

FVector2D ARMCLandscape::GetChunkCenter(const FIntPoint& ChunkCoord)
{

	FVector2D Offset = FVector2D( ChunkCoord.X * ChunkLength , ChunkCoord.Y * ChunkLength );
	FVector2D Center = FVector2D( ChunkLength / 2 , ChunkLength / 2 );

	return Offset + Center;
}

FIntPoint ARMCLandscape::GetPlayerLoactedChunk()
{
	FIntPoint ChunkCoord;
	FVector PlayerLocation = GetWorld()->GetFirstPlayerController()->GetPawn()->GetActorLocation();
	ChunkCoord.X = FMath::FloorToInt32( PlayerLocation.X / ChunkLength );
	ChunkCoord.Y = FMath::FloorToInt32( PlayerLocation.Y / ChunkLength );

	return ChunkCoord;
}

bool ARMCLandscape::IsChunkInRadius(const FIntPoint& Target, const FIntPoint& Start)
{
	FVector2D TargetPoint = GetChunkCenter(Target);
	FVector2D StartPoint = GetChunkCenter(Start);

	float LengthSquared = FMath::Pow(ChunkRadiusByLength, 2);
	float DistanceSquared = FVector2D::DistSquared(TargetPoint, StartPoint);
	
	
	return LengthSquared >= DistanceSquared;
}

float ARMCLandscape::GenerateHeight(const FVector2D& Location)
{
	float height = 0;

	if(PerlinNoiseLayers.Num() <= 0)
	{
		return 0;
	}

	for ( int32 i = 0; i < PerlinNoiseLayers.Num(); i++)
	{
		float NoiseScale = 1.0f / PerlinNoiseLayers[i].Frequency;
		float Amplitude = PerlinNoiseLayers[i].Amplitude;
		float Offset = PerlinNoiseLayers[i].Offset;

		height += FMath::PerlinNoise2D(Location * NoiseScale + Offset) * Amplitude;
	}

	return height;
}

void ARMCLandscape::RemoveChunk(const FIntPoint &ChunkCoord)
{
	if(Chunks.Contains(ChunkCoord))
	{
		Chunks[ChunkCoord]->Destroy();
		Chunks.Remove(ChunkCoord);
	}
	return;
}

float ARMCLandscape::GetElapsedInMs(const FDateTime &StartTime)
{
	return (FDateTime::UtcNow() - StartTime).GetTotalMilliseconds();
}
