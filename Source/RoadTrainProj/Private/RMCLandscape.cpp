// Fill out your copyright notice in the Description page of Project Settings.


#include "RMCLandscape.h"
#include "Mesh/RealtimeMeshAlgo.h"

#include "Async/Async.h"
#include "Misc/ScopeLock.h"

// Sets default values
ARMCLandscape::ARMCLandscape()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));

}

ARMCLandscape::~ARMCLandscape()
{
	RemoveLandscape();

}

// Called when the game starts or when spawned
void ARMCLandscape::BeginPlay()
{
	Super::BeginPlay();

	// TODO : Move all these to Control class later

	FTimerHandle LandscapeTimer;
	if( bUseAsync )
	{
		GetWorldTimerManager().SetTimer(
			LandscapeTimer,
			this,
			&ARMCLandscape::AsyncGenerateLandscape,
			UpdatePeriod,
			true,
			0.001f
		);
	}
	else
	{
		GetWorldTimerManager().SetTimer(
			LandscapeTimer,
			this,
			&ARMCLandscape::GenerateLandscape,
			UpdatePeriod,
			true,
			0.001f
		);
	}


	return;
}

// Called every frame
void ARMCLandscape::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	return;
}

// Public ↓

void ARMCLandscape::AsyncGenerateLandscape()
{
	AsyncAddChunksInRange();
	
	FTimerHandle RemoveTimer;
	GetWorldTimerManager().SetTimer(
		RemoveTimer,
		this,
		&ARMCLandscape::AsyncRemoveChunksOutOfRange,
		UpdatePeriod/2,
		false,
		0.0f
	);

	return;
}

void ARMCLandscape::AsyncAddChunksInRange()
{

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, 
		[this]()
		{
		// Lambda Start
			FIntPoint Center = FIntPoint(0,0);
			if( GetWorld()->GetFirstPlayerController() != nullptr )
			{
				Center = GetPlayerLocatedChunk();
			}
			
			// Adding Chunks
			int32 Extent = ChunkRadius;
			for( int32 iY = Center.Y - Extent; iY <= Center.Y + Extent; iY++)
			{
				for (int32 iX = Center.X - Extent; iX <= Center.X + Extent; iX++)
				{
					FIntPoint Coord = FIntPoint(iX, iY);
					if( !Chunks.Contains(Coord) )
					{
						RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
						GenerateStreamSet( Coord, StreamSet );
						// Do this ↓ on Main Thread. use MUTEX!!!
						AsyncTask(ENamedThreads::GameThread, [ this, &StreamSet, &Coord ]{ FScopeLock Lock(&ChunksMutex); AddChunk(Coord, StreamSet); } );
						ChunkCount++;
					}
				}
			}
		// Lambda End
		} 
	);

	return;
}

void ARMCLandscape::AsyncRemoveChunksOutOfRange()
{
	AsyncTask(ENamedThreads::AnyThread, 
		[this](){
		// Lambda Start
			FIntPoint Center = FIntPoint(0,0);
			if( GetWorld()->GetFirstPlayerController() )
			{
				Center = GetPlayerLocatedChunk();
			}
			int32 Extent = ChunkRadius;
			TArray<FIntPoint> ChunksToRemove;

			FScopeLock Lock(&ChunksMutex);	// MUTEX!!
			for ( auto &Elem : Chunks )
			{
				if( Elem.Key.X < Center.X - Extent || Elem.Key.X > Center.X + Extent
					||
					Elem.Key.Y < Center.Y - Extent || Elem.Key.Y > Center.Y + Extent)
				{
					ChunksToRemove.Add(Elem.Key);
					ARealtimeMeshActor* RMA = Elem.Value;
					AsyncTask( ENamedThreads::GameThread, [this, RMA]{ RMA->Destroy(); } );
				}
			}

			for ( int32 i = 0; i < ChunksToRemove.Num(); i++ )
			{
				Chunks.Remove(ChunksToRemove[i]);
			}
		// Lambda End
		} 
	);

}

void ARMCLandscape::GenerateLandscape()
{
	// Init some var
	ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;
	ChunkRadiusByLength = ChunkLength * ChunkRadius;
	ChunkCount = 0;


	FDateTime StartTime = FDateTime::UtcNow();
	// Adding Chunks

	FIntPoint Center = FIntPoint(0,0);
	if( GetWorld()->GetFirstPlayerController() != nullptr )
	{
		Center = GetPlayerLocatedChunk();
	}

	RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
	int32 Extent = ChunkRadius;
	for( int32 iY = Center.Y - Extent; iY <= Center.Y + Extent; iY++)
	{
		for (int32 iX = Center.X - Extent; iX <= Center.X + Extent; iX++)
		{
			FIntPoint Coord = FIntPoint(iX, iY);
			if( !Chunks.Contains(Coord) )
			{
				StreamSet.Empty();
				GenerateStreamSet( Coord, StreamSet );
				AddChunk( Coord, StreamSet );
				ChunkCount++;
			}
		}
	}

	UE_LOG(LogTemp, Display, TEXT("AddChunks : %f ms"), GetElapsedInMs(StartTime));

	StartTime = FDateTime::UtcNow();
	// Removing Chunks
	TArray<FIntPoint> ChunksToRemove;
	for( auto &Elem : Chunks )
	{

		if( Elem.Key.X < Center.X - Extent || Elem.Key.X > Center.X + Extent
			||
			Elem.Key.Y < Center.Y - Extent || Elem.Key.Y > Center.Y + Extent)
		{
			ChunksToRemove.Add(Elem.Key);
			Elem.Value->Destroy();
		}
	}

	for( int32 i = 0; i < ChunksToRemove.Num(); i++ )
	{
		FIntPoint Elem = ChunksToRemove[i];
		Chunks.Remove(Elem);
	}

	UE_LOG(LogTemp, Display, TEXT("RemoveChunks : %f ms"), GetElapsedInMs(StartTime));



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

	return;
}

// Public ↑

// Private ↓


void ARMCLandscape::AddChunk(const FIntPoint& ChunkCoord, const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet)
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

	const FRealtimeMeshSectionGroupKey GroupKey = FRealtimeMeshSectionGroupKey::Create(0, 0);
	const FRealtimeMeshSectionKey PolyGroup0SectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey, 0);

	// this generates the mesh (chunk)
	RealtimeMesh->CreateSectionGroup(GroupKey, StreamSet);

	Chunks.Add(ChunkCoord, RMA);

	// update configuration.
	RealtimeMesh->UpdateSectionConfig( PolyGroup0SectionKey, FRealtimeMeshSectionConfig(0), true );

}

void ARMCLandscape::GenerateStreamSet(const FIntPoint& ChunkCoord, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet)
{
	// Chunk location offset
	FVector2D Offset = FVector2D( ChunkCoord.X , ChunkCoord.Y ) * ChunkLength;
	
	// scale UV based on vetex spacing
	float UVScale = VertexSpacing / TextureSize;


	// actual data to use
	TArray<FVector3f> Vertices, Tangents, Normals;
	TArray<uint32> Triangles;
	TArray<FVector2DHalf> UVs;


	// Vertices and UVs
	for(int32 iY = 0; iY < VerticesPerChunk; iY++)
	{
		for(int32 iX = 0; iX < VerticesPerChunk; iX++)
		{
			FVector3f Vertex = FVector3f(iX, iY, 0.0f) * VertexSpacing;
			if( ShouldGenerateHeight )
			{
				Vertex.Z = GenerateHeight( FVector2D(Vertex.X, Vertex.Y) + Offset);
			}
			Vertices.Add(Vertex);

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

	// Big data for uv and normal continue on different chunks
	TArray<FVector3f> BigVertices;
	TArray<uint32> BigTriangles;
	

	for(int32 iY = -1 ; iY < VerticesPerChunk + 1; iY++)
	{
		for(int32 iX = -1; iX < VerticesPerChunk + 1; iX++)
		{
			FVector3f Vertex = FVector3f(iX, iY, 0.0f) * VertexSpacing;
			if(ShouldGenerateHeight)
			{
				Vertex.Z = GenerateHeight( FVector2D(Vertex.X, Vertex.Y) + Offset );
			}
			BigVertices.Add(Vertex);
		}
	}

	for (int32 iY = 0; iY < VerticesPerChunk + 2; iY++)
	{
		for (int32 iX = 0; iX < VerticesPerChunk + 2; iX++)
		{
			int32 RowLength = VerticesPerChunk + 2;
			int32 CurrentVertex = iX + iY * RowLength;
			BigTriangles.Add(CurrentVertex);
			BigTriangles.Add(CurrentVertex + RowLength);
			BigTriangles.Add(CurrentVertex + 1);

			BigTriangles.Add(CurrentVertex + RowLength);
			BigTriangles.Add(CurrentVertex + RowLength + 1);
			BigTriangles.Add(CurrentVertex + 1);
		}
	}


	// Normals and Tangents from Big datas
	int RowLength = VerticesPerChunk + 2;

	RealtimeMeshAlgo::GenerateTangents(
		TConstArrayView<uint32>(BigTriangles),
		BigVertices,
		nullptr, // UV Getter. We made uv.
		[&Normals, &Tangents, &RowLength](int index, FVector3f Tangent, FVector3f Normal) -> void
		{
			// RowLength == ColumnLength
			int iY = index / RowLength;
			int iX = index % RowLength;

			// initial index array (ex) RowLength = 4
			/* 
			0 	1 	2 	3
			4 	5 	6 	7
			8 	9 	10	11
			12 	13 	14 	15
			*/

			if( iY <= 0 || iY >= RowLength - 1  	// ignore first and last Row
				||
				iX <= 0 || iX >= RowLength - 1 ) 	// ignore first and last Column
			{
				return;
			}

			// index array now
			/* 
			x 	x 	x 	x
			x 	5 	6 	x
			x 	9 	10	x
			x 	x 	x	x
			*/

			index = index - 1;
			index = index - ( iY * RowLength );

			// index array now
			/* 
			x 	x 	x 	x
			x 	0 	1	x
			x 	0 	1	x
			x 	x 	x	x
			*/

			int VerticesPerChunk = RowLength - 2;
			index = index + (iY-1) * VerticesPerChunk;

			// index array now
			/*
				0	1
				2	3
			*/
			Normals[index] = Normal;
			Tangents[index] = Tangent;
		},
		true
	);


	// Big data done ↑


	// Datas into StreamSet( class Member Var )
	RealtimeMesh::TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 1> Builder(OutStreamSet);
	Builder.EnableTangents();
	Builder.EnableTexCoords();
	Builder.EnableColors();

	Builder.EnablePolyGroups();

	for (int32 i = 0; i < Vertices.Num(); i++)
	{
		Builder.AddVertex( Vertices[i] )
			.SetNormalAndTangent( Normals[i], Tangents[i])
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

FIntPoint ARMCLandscape::GetPlayerLocatedChunk()
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
	float height = 0.0f;

	if(PerlinNoiseLayers.Num() <= 0)
	{
		return 0.0f;
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
