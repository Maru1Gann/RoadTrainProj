// Fill out your copyright notice in the Description page of Project Settings.


#include "RMCLandscape.h"
#include "PerlinNoiseVariables.h"
#include "Mesh/RealtimeMeshAlgo.h"
#include "PathFinder.h"

#include "DrawDebugHelpers.h"

// Sets default values
ARMCLandscape::ARMCLandscape()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	PerlinNoiseLayers.Add( FPerlinNoiseVariables(10000.0f, 1000.0f, 1.0f) );

}

// Called when the game starts or when spawned
void ARMCLandscape::BeginPlay()
{
	Super::BeginPlay();

	if( DoPathFinding )
	{
		PathFinder = new FPathFinder(this, StartPos, EndPos, Slope );
		
		DrawDebugPoint(
			this->GetWorld(),
			ConvertTo3D(StartPos),
			30.f,
			FColor::Blue,
			true
		);

		DrawDebugPoint(
			this->GetWorld(),
			ConvertTo3D(EndPos),
			30.f,
			FColor::Blue,
			true
		);

	}
	

	// RMC Chunk Generation ↓
	GenerateChunkOrder();

	// TODO : Move all these to Control class later
	FTimerHandle LandscapeTimer;
	if( bUseAsync )
	{
		StreamSetGenerator = new FAsyncTask<FStreamSetGenerator>(this);

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

void ARMCLandscape::OnConstruction(const FTransform &Transform)
{
	ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;
	HorizonDistance = (ChunkLength/2 + ChunkLength * ChunkRadius) / (100*1000); // km
	GenerateChunkOrder();

	StartPos = SnapToGrid(Start);
	EndPos = SnapToGrid(End);

	StartChunk = GetChunk(StartPos);
	EndChunk = GetChunk(EndPos);
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
	if( StreamSetGenerator == nullptr )
	{
		UE_LOG(LogTemp, Warning, TEXT("StreamSetGenerator nullptr"));
		return;
	}

	if( IsDataReady == true )
	{
		IsWorking = true;

		if( NeededChunk.Value == true )
		{
			AddChunk(NeededChunk.Key, MemberStreamSet);
			NeededChunk.Value = false;
		}

		if( RemovableChunk.Value == true )
		{
			RemoveChunk(RemovableChunk.Key);
			RemovableChunk.Value = false;
		}

		IsDataReady = false;
		IsWorking = false;

	}

	if( StreamSetGenerator->IsDone() && IsWorking == false )
	{
		StreamSetGenerator->EnsureCompletion();
		StreamSetGenerator->StartBackgroundTask();
	}

	return;
}

void ARMCLandscape::GenerateLandscape()
{
	// Init some var

	FDateTime StartTime = FDateTime::UtcNow();
	int32 PrevChunkCount = ChunkCount;
	// Adding Chunks

	FIntPoint Center = GetPlayerLocatedChunk();

	float StreamSetTime = 0.f;
	float AddChunkTime = 0.f;

	RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
	// sprial generation
	for( int32 i = 0; i < ChunkOrder.Num(); i++ )
	{
		FIntPoint Coord = ChunkOrder[i] + Center;
		if( !Chunks.Contains(Coord) )
		{
				FDateTime Tmp = FDateTime::UtcNow();

			StreamSet.Empty();
			GenerateStreamSet( Coord, StreamSet );

				StreamSetTime += (FDateTime::UtcNow() - Tmp).GetTotalMilliseconds();
				Tmp = FDateTime::UtcNow();

			AddChunk( Coord, StreamSet );

				AddChunkTime += (FDateTime::UtcNow() - Tmp).GetTotalMilliseconds();

			ChunkCount++;
		}
	}

	if( PrevChunkCount != ChunkCount )
	{
		UE_LOG(LogTemp, Display, TEXT("AddChunksTotal	: %f ms"), GetElapsedInMs(StartTime));
		UE_LOG(LogTemp, Display, TEXT("StreamSetGen	: %f ms"), StreamSetTime);
		UE_LOG(LogTemp, Display, TEXT("AddingChunks	: %f ms"), AddChunkTime);
		PrevChunkCount = ChunkCount;
	}

	StartTime = FDateTime::UtcNow();
	// Removing Chunks
	TArray<FIntPoint> ChunksToRemove;
	int32 Extent = ChunkRadius;
	for( auto &Elem : Chunks )
	{

		if( Elem.Key.X < Center.X - Extent || Elem.Key.X > Center.X + Extent
			||
			Elem.Key.Y < Center.Y - Extent || Elem.Key.Y > Center.Y + Extent)
		{
			ChunksToRemove.Add(Elem.Key);
			Elem.Value->Destroy();
			ChunkCount--;
		}
	}

	for( int32 i = 0; i < ChunksToRemove.Num(); i++ )
	{
		FIntPoint Elem = ChunksToRemove[i];
		Chunks.Remove(Elem);
	}

	if(PrevChunkCount != ChunkCount)
	{
		UE_LOG(LogTemp, Display, TEXT("RemoveChunks : %f ms"), GetElapsedInMs(StartTime));
	}


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


void ARMCLandscape::SetPath(const TArray<FVector2D>& ReversePath)
{
	int32 LastIndex = ReversePath.Num() - 1;
	this->Path.Empty();
	this->Path.SetNum( LastIndex + 1);
	for( int32 i = 0; i <= LastIndex; i++ )
	{
		Path[i] = ReversePath[LastIndex - i];
	}

	return;
}
void ARMCLandscape::DrawPathDebug()
{
	UE_LOG(LogTemp, Display, TEXT("Path Num : %d"), this->Path.Num());

	for( int32 i = 0; i < Path.Num(); i++ )
	{
		FVector Point = FVector(Path[i].X, Path[i].Y, GenerateHeight(Path[i]) + 100.f );
		// UE_LOG(LogTemp, Display, TEXT("Path[%d] : %s"), i, *Point.ToString());

		DrawDebugPoint(
			this->GetWorld(),
			Point,
			10.f,
			FColor::Red,
			true
		);
	}
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

	URealtimeMeshComponent* pRMC = RMA->GetRealtimeMeshComponent();
	if( pRMC == nullptr )
	{
		UE_LOG(LogTemp, Warning, TEXT("pRMC nullptr"));
		return;
	}

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

	// set it to static
	RMA->GetRootComponent()->SetMobility(EComponentMobility::Static);

	if( ChunkMaterial != nullptr )
	{
		URealtimeMeshComponent* MeshComp = RMA->GetRealtimeMeshComponent();
		MeshComp->SetMaterial( 0, ChunkMaterial );
	}

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

void ARMCLandscape::GenerateChunkOrder()
{
	FDateTime StartTime = FDateTime::UtcNow();
	// This function never needs to be called more than once!
	// Unless the variables have changed. (when called in editor)

	ChunkOrder.Empty();
	ChunkOrder.SetNum( (ChunkRadius*2+1) * (ChunkRadius*2+1) );

	// we make square with chunks
	// from the start point, we make it like a whirl

	/* example
		20	19	18	17	16
		21	6	5	4	15
		22	7	0	3	14
		23	8	1	2	13
		24	9	10	11	12
		25	..

		x+ : right
		y+ : down
	*/

	FIntPoint Pos = FIntPoint(0,0);
	int32 Index = 0;
	ChunkOrder[Index++] = Pos;
	int32 LocalStep = 2;
	while( Index < ChunkOrder.Num() )
	{
		// one step down
		Pos.Y++;
		ChunkOrder[Index++] = Pos;

		for( int32 i = 1; i<LocalStep; i++ )
		{
			Pos.X++;
			ChunkOrder[Index++] = Pos;
		}
		for( int32 i = 0; i<LocalStep; i++)
		{
			Pos.Y--;
			ChunkOrder[Index++] = Pos;
		}
		for (int32 i = 0; i<LocalStep; i++)
		{
			Pos.X--;
			ChunkOrder[Index++] = Pos;
		}
		for (int32 i = 0; i<LocalStep; i++)
		{
			Pos.Y++;
			ChunkOrder[Index++] = Pos;
		}

		LocalStep += 2;
	}

	UE_LOG(LogTemp, Display, TEXT("ChunkOrderGeneration : %f ms"), 	GetElapsedInMs(StartTime) );
	return;
}



/* Tools */

FIntPoint ARMCLandscape::GetPlayerLocatedChunk()
{
	FIntPoint ChunkCoord;

	if( GetWorld()->GetFirstPlayerController() )
	{
		FVector PlayerLocation = GetWorld()->GetFirstPlayerController()->GetPawn()->GetActorLocation();
		ChunkCoord = GetChunk( FVector2D(PlayerLocation.X, PlayerLocation.Y) );
	}
	
	return ChunkCoord;
}

FIntPoint ARMCLandscape::GetChunk(const FVector2D& Location)
{
	FIntPoint ChunkCoord;
	ChunkCoord.X = FMath::FloorToInt32( Location.X / ChunkLength );
	ChunkCoord.Y = FMath::FloorToInt32( Location.Y / ChunkLength );

	return ChunkCoord;
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
		float Frequency = PerlinNoiseLayers[i].Frequency;
		if(Frequency == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("Perlin Noise frequency can't be 0"));
			Frequency = 0.001;
		}
		float NoiseScale = 1.0f / Frequency;
		float Amplitude = PerlinNoiseLayers[i].Amplitude;
		float Offset = PerlinNoiseLayers[i].Offset;

		height += FMath::PerlinNoise2D(Location * NoiseScale + Offset) * Amplitude;
	}

	return height;
}

FVector ARMCLandscape::ConvertTo3D(const FVector2D& Loc)
{
	return FVector(Loc.X, Loc.Y, GenerateHeight(Loc) );
}

void ARMCLandscape::RemoveChunk(const FIntPoint &ChunkCoord)
{

	if(	ARealtimeMeshActor** pRMA = Chunks.Find(ChunkCoord) )
	{
		(*pRMA)->Destroy();
		Chunks.Remove(ChunkCoord);
	}
	return;
}

float ARMCLandscape::GetElapsedInMs(const FDateTime &StartTime)
{
	return (FDateTime::UtcNow() - StartTime).GetTotalMilliseconds();
}

FVector2D ARMCLandscape::SnapToGrid(const FVector2D &Location)
{
	FVector2D Out;
	Out.X = FMath::FloorToInt(Location.X / VertexSpacing ) * VertexSpacing;
	Out.Y = FMath::FloorToInt(Location.Y / VertexSpacing ) * VertexSpacing;

    return Out;
}

// StartBackgroundTask() calls this function
void FStreamSetGenerator::DoWork()
{
	if(RMC == nullptr)
	{
		return;
	}
	
	FIntPoint CurrentChunk = RMC->GetPlayerLocatedChunk();
	if ( CurrentChunk != RMC->PlayerChunk )
	{
		StartIndex = 0;
		RMC->PlayerChunk = CurrentChunk;
	}


	for( int32 i = StartIndex; i < RMC->ChunkOrder.Num(); i++ )
	{
		FIntPoint Coord = RMC->ChunkOrder[i] + CurrentChunk;
		if( !RMC->Chunks.Contains( Coord ) )
		{
			RMC->MemberStreamSet.Empty();
			RMC->GenerateStreamSet( Coord, RMC->MemberStreamSet );
			RMC->NeededChunk.Key = Coord;
			RMC->NeededChunk.Value = true;
			StartIndex = i+1;

			// PCG add points

			break;
		}
	}

	int32 Extent = RMC->ChunkRadius;
	FIntPoint Center = CurrentChunk;
	for ( auto &Elem : RMC->Chunks )
	{
		if( Elem.Key.X < Center.X - Extent || Elem.Key.X > Center.X + Extent
			||
			Elem.Key.Y < Center.Y - Extent || Elem.Key.Y > Center.Y + Extent)
		{
			RMC->RemovableChunk.Key = Elem.Key;
			RMC->RemovableChunk.Value = true;
			break;
		}
	}

	RMC->IsDataReady = true;

	return;
}