// Fill out your copyright notice in the Description page of Project Settings.


#include "RMCLandscape.h"
#include "Mesh/RealtimeMeshAlgo.h"

// Sets default values
ARMCLandscape::ARMCLandscape()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
}

// Correspond to BP_ConstructionScript
void ARMCLandscape::OnConstruction(const FTransform &Transform)
{
	Super::OnConstruction(Transform);

	ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;
	ChunkRadiusByLength = ChunkLength * ChunkRadius;
	ChunkCount = 0;

	return;
}

// Called when the game starts or when spawned
void ARMCLandscape::BeginPlay()
{
	Super::BeginPlay();

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
	for( auto& Elem : Chunks )
	{
		Elem.Value->Destroy();
	}
	Chunks.Empty();


	GenerateChunkOrder();
	for( int32 i = 0; i < ChunkOrder.Num(); i++ )
	{
		AddChunk(ChunkOrder[i]);
	}

	return;
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

	// Set Location
	FVector Offset = FVector( ChunkCoord.X , ChunkCoord.Y, 0.0f ) * ChunkLength;
	RMA->SetActorLocation(Offset);

	URealtimeMeshSimple* RealtimeMesh = RMA->GetRealtimeMeshComponent()->InitializeRealtimeMesh<URealtimeMeshSimple>();

	RealtimeMesh->SetupMaterialSlot(0, "PrimaryMaterial");
	RealtimeMesh->UpdateLODConfig(0, FRealtimeMeshLODConfig(1.00f));

	GenerateStreamSet(ChunkCoord);

	const FRealtimeMeshSectionGroupKey GroupKey = FRealtimeMeshSectionGroupKey::Create(0, 0);
	const FRealtimeMeshSectionKey PolyGroup0SectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey, 0);

	// this generates the mesh (chunk)
	RealtimeMesh->CreateSectionGroup(GroupKey, StreamSet);

	Chunks.Add(ChunkCoord, RMA);

	// update configuration.
	RealtimeMesh->UpdateSectionConfig(PolyGroup0SectionKey, FRealtimeMeshSectionConfig(0), true);
}

void ARMCLandscape::GenerateStreamSet(const FIntPoint& ChunkCoord)
{
	// Chunk location offset
	FVector Offset = FVector( ChunkCoord.X , ChunkCoord.Y, 0.0f ) * ChunkLength;
	
	// scale UV based on vetex spacing
	float UVScale = VertexSpacing / TextureSize;


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
			UV.X = (Offset.X * (VerticesPerChunk-1) + iX) * UVScale;
			UV.Y = (Offset.Y * (VerticesPerChunk-1) + iY) * UVScale;
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

	Tangents.SetNum( Vertices.Num() );
	Normals.SetNum( Vertices.Num() );

	// Normals and Tangents
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
	RealtimeMesh::TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 2> Builder(StreamSet);
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
void ARMCLandscape::GenerateChunkOrder()
{
	ChunkOrder.Empty();

	// we make square with chunks
	// from the start point, we make it like a whirl

	/* example
		16	15	14	13	12
		17	4	3	2	11
		18	5	0	1	10
		19	6	7	8	9
		20	21	22	23	24

		this goes
		*Iterator starts from 2

		one step front
		'Iterator - 1' step upwards
		'Iterator' step backwards
		'Iterator' step downwards
		'Iterator' step forwards
		-> one square done.
		iterator++
		
		add it only when it's inside radius.
	*/

	int32 Iterator = 1;
	FIntPoint CurrentCoord = FIntPoint(0,0);
	ChunkOrder.Add(CurrentCoord);

	while ( Iterator <= ChunkRadiusByLength )
	{
		int32 step = Iterator*2;
		// one step front.
		CurrentCoord += FIntPoint(1,0);
		if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord) )
			{
				ChunkOrder.Add(CurrentCoord);
			}

		// step up
		for (int32 i = 0; i < step - 1; i++)
		{
			
			CurrentCoord += FIntPoint(0,1);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}

		// step back
		for (int32 i = 0; i < step; i++)
		{
			
			CurrentCoord += FIntPoint(-1, 0);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}

		// step down
		for (int32 i = 0; i < step; i++)
		{

			CurrentCoord += FIntPoint(0, -1);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}

		// step forward
		for (int32 i = 0; i < step; i++)
		{
			
			CurrentCoord += FIntPoint(1, 0);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}


		Iterator++;
	}


	return;
}

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
