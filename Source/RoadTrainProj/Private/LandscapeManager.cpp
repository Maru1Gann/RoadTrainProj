// Fill out your copyright notice in the Description page of Project Settings.


#include "LandscapeManager.h"

#include "ProceduralMeshComponent.h"		// FProcMeshTangent
#include "KismetProceduralMeshLibrary.h"	// Procedural Mesh Component
#include "Kismet/KismetSystemLibrary.h"		// Drawing Debug Stuffs
#include "Kismet/GameplayStatics.h"			// UGameplayStatics::GetPlayerCharacter(this, 0);
#include "GameFramework/Character.h"		// ACharacter
#include "Math/UnrealMathUtility.h"			// FMath::PerlinNoise2D


// Sets default values
ALandscapeManager::ALandscapeManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	ProceduralMeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh"));
	RootComponent = ProceduralMeshComponent;

	Triangles.Empty();
	BigTriangles.Empty();

	AsyncInfoTask = new FAsyncTask<FLandscapeInfoGenerator>(this);
}

// Called when the game starts or when spawned
void ALandscapeManager::BeginPlay()
{
	Super::BeginPlay();
	
	// get player reference
	PlayerCharacter = UGameplayStatics::GetPlayerCharacter(this, 0);
	if(PlayerCharacter == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("LandScapeManager_BeginPlay : PlayerCharacter nullptr"));
	}

	// Initial actions.

	GenerateChunkOrder(this->RadiusByChunkCount);

	GenerateLandscape();


	// Setting Timers


	// ↓ Version without multithreading
	
	if( ShouldUseMultithreading == false )
	{

		GetWorldTimerManager().SetTimer(
			ChunkUpdateTimerHandle, 
			this, 
			&ALandscapeManager::UpdateLandscape,
			ChunkUpdateFrequency,
			true,
			0.01f
		);

	}
	else // With Multithreading ↓
	{
		

		// Info Generation task
		GetWorldTimerManager().SetTimer(
			AsyncChunkInfoUpdateTimerHandle,
			this,
			&ALandscapeManager::UpdateChunkInfoAsync,
			AsyncChunkInfoUpdateFrequency,
			true,
			0.01f
		);

		// Chunk Updating task (MainThread)d
		GetWorldTimerManager().SetTimer(
			AsyncChunkUpdateTimerHandle,
			this,
			&ALandscapeManager::UpdateLandscapeAsync,
			AsyncChunkUpdateFrequency,
			true,
			0.02f
		);


	}	// END of else


	return;

}

// Called every frame
void ALandscapeManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}



void ALandscapeManager::UpdateLandscape()
{

	// --------Background Thread-------- START

	// if ( IsChunkInfoReady == true )
	// {
	// 	return;
	// }

	FIntPoint PlayerLocatedChunk = GetPlayerLocatedChunk();

	// Member variables for multithreading
	// 		FIntPoint NeededChunk;
	// 		TPair<FIntPoint, int32> RemovableChunk;

	// NeededChunk and RemovableChunk are OutParameters
	if( FindNeededChunk( NeededChunk, PlayerLocatedChunk ) == false || FindRemovableChunk( RemovableChunk, PlayerLocatedChunk ) == false )
	{
		return;
	}

	GenerateChunkInfo(NeededChunk);

	// IsChunkInfoReady = true;

	// --------Background Thread-------- END

	// if( IsChunkInfoReady == false )
	// {
	// 	return;
	// }
	

	UpdateSingleChunk(RemovableChunk.Value);

	ChunkStatus.Remove(RemovableChunk.Key);
	ChunkStatus.Add(NeededChunk, RemovableChunk.Value);
	
	// fixes bad collision issues
	ProceduralMeshComponent->ClearCollisionConvexMeshes();

	// IsChunkInfoReady = false;

	UE_LOG(LogTemp, Display, TEXT("(%d, %d) : Landscape Updated"), NeededChunk.X, NeededChunk.Y);

	return;
}


void ALandscapeManager::UpdateChunkInfoAsync()
{
	if ( IsChunkInfoGenerating == true )
	{
		UE_LOG(LogTemp, Display, TEXT("MultiThreading flag hit : ChunkInfo still Generating"));
		return;
	}
	
	if ( IsChunkInfoReady == true )
	{
		return;
	}

	// Check DoWork() at bottom
	
	if( AsyncInfoTask->IsDone() )
	{
		AsyncInfoTask->EnsureCompletion();
		AsyncInfoTask->StartBackgroundTask();
	}
	else
	{
		//UE_LOG(LogTemp, Display, TEXT(" AsyncTask still running "));
	}
	

	// when task done ↓
	// IsChunkInfoGenerating 	set to false in DoWork()
	// IsChunkInfoReady 		set to true in DoWork()

	return;
}

void ALandscapeManager::UpdateLandscapeAsync()
{
	if ( IsChunkInfoGenerating == true )
	{
		UE_LOG(LogTemp, Display, TEXT("Main Thread flag hit : ChunkInfo still generating"));
		return;
	}

	if( IsChunkInfoReady == false )
	{
		return;
	}


	UpdateSingleChunk(RemovableChunk.Value);

	ChunkStatus.Remove(RemovableChunk.Key);
	ChunkStatus.Add(NeededChunk, RemovableChunk.Value);
	
	// fixes bad collision issues
	ProceduralMeshComponent->ClearCollisionConvexMeshes();

	UE_LOG(LogTemp, Display, TEXT("(%d, %d) : Chunk Updated"), NeededChunk.X, NeededChunk.Y);

	IsChunkInfoReady = false;


	return;
}


// Editor Callable Functions
// Call this only once in Beginplay.
void ALandscapeManager::GenerateLandscape()
{

	FlushForEditor();

	GenerateChunkOrder(this->RadiusByChunkCount);

	for ( int i = 0; i < ChunkOrder.Num(); i++ )
	{
		GenerateChunkInfo( ChunkOrder[i] );
		DrawSingleChunk( ChunkOrder[i] );
	}

	return;
}

void ALandscapeManager::FlushForEditor()
{
	ProceduralMeshComponent->ClearAllMeshSections();
	FlushPersistentDebugLines(this->GetWorld());

	BigTriangles.Empty();
	Triangles.Empty();

	Vertices.Empty();
	Normals.Empty();
	UVs.Empty();
	Tangents.Empty();

	ChunkSectionIndex = 0;

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

	// Clear TArrays.
	ResetChunkInfo();

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

	TArray<FVector2D> BigUVs;
	FVector2D UV;


	// Calculations for BigVertices. BigUVs also since it's same.
	for (int32 iY = -1;  iY <= ChunkVertexCount.Y; iY++) // this goes from -1 ~ (length + 1) == length + 2
	{
		for (int32 iX = -1; iX <= ChunkVertexCount.X;  iX++)
		{
			// getting the actual vertex coordinates in worldspace
			Vertex = FVector( iX, iY, 0.f ) * CellSize + Offset; 

			// PerlinNoise Based Height Generation.
			if( ShouldUseHeightGeneration )
			{
				Vertex.Z = GenerateHeight(FVector2D(Vertex));
			}
			

			BigVertices.Add(Vertex);
			UV = FVector2D(Vertex) / CellSize; // convert to FVector2D
			BigUVs.Add(UV);	
		}
	}

	// BigTriangles are just abstract coordinates, it should go from 0 to ChunkVertexCount + 1 ( == -1 to ChunkVertexCount )
	// !!We don't use the last vertex as startingpoint!! So (ChunkVertexCount + 1) - 1


	int32 Row = ChunkVertexCount.X + 2;
	// make triangles only if it's empty. Since they are always same. No need to make it twice.
	if( BigTriangles.IsEmpty() )
	{

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
	// Only make once. Same as BigTriangles.

	if ( Triangles.IsEmpty() )
	{

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

	}


	UE_LOG(LogTemp, Display, TEXT("(%d, %d) : Generated Chunk Info"), ChunkCoord.X, ChunkCoord.Y);
	return;
}

void ALandscapeManager::GenerateChunkOrder(const int RadiusByCount)
{
	// This function never needs to be called more than once!
	// Unless the variables have changed. (when called in editor)

	ChunkOrder.Empty();


	// we make circle with x length radius. Ignore Y Length.
	float RadiusByLength = RadiusByCount * ( (ChunkVertexCount.X - 1) * CellSize );

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

	while ( Iterator <= RadiusByCount )
	{
		int32 step = Iterator*2;
		// one step front.
		CurrentCoord += FIntPoint(1,0);
		if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord, RadiusByLength) )
			{
				ChunkOrder.Add(CurrentCoord);
			}

		// step up
		for (int i = 0; i < step - 1; i++)
		{
			
			CurrentCoord += FIntPoint(0,1);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord, RadiusByLength) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}

		// step back
		for (int i = 0; i < step; i++)
		{
			
			CurrentCoord += FIntPoint(-1, 0);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord, RadiusByLength) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}

		// step down
		for (int i = 0; i < step; i++)
		{

			CurrentCoord += FIntPoint(0, -1);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord, RadiusByLength) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}

		// step forward
		for (int i = 0; i < step; i++)
		{
			
			CurrentCoord += FIntPoint(1, 0);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord, RadiusByLength) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}


		Iterator++;
	}


	return;
}

bool ALandscapeManager::FindNeededChunk(FIntPoint& OutNeededChunk, const FIntPoint ChunkCoord)
{
	// OutNeededChunk is OutParameter. Its value will change

	FIntPoint CurrentChunk = GetPlayerLocatedChunk();

	for( int i = 0; i < ChunkOrder.Num(); i++ )
	{
		FIntPoint Iterator = CurrentChunk + ChunkOrder[i];
		if( ChunkStatus.Contains( Iterator ) == false )
		{
			OutNeededChunk = Iterator;
			return true;
		}
	}

	return false;
}

bool ALandscapeManager::FindRemovableChunk(TPair<FIntPoint, int32>& OutRemovableChunk, const FIntPoint ChunkCoord)
{
	// OutSectionIndex is Out Parameter.

	float RadiusByLength = RadiusByChunkCount * ( (ChunkVertexCount.X - 1) * CellSize );

	for ( auto& Elem : ChunkStatus )
	{
		if( IsChunkInRadius(ChunkCoord, Elem.Key, RadiusByLength ) == false )
		{
			OutRemovableChunk = {Elem.Key, Elem.Value};
			return true;
		}
	}
	
	return false;
}

void ALandscapeManager::UpdateSingleChunk(const int32 SectionIndex)
{
	// We take Mesh of given SectionIndex to new ChunkCoord.
	// Need information generated.

	ProceduralMeshComponent->UpdateMeshSection(
		SectionIndex,
		Vertices, 
		Normals, 
		UVs, 
		TArray<FColor>(), 
		Tangents 
	);

	// Drawing DebugPoint
	if(ShouldDrawDebugPoint)
	{
		DrawDebugPoints();
	}
	// debug

	return;
}

void ALandscapeManager::DrawSingleChunk(const FIntPoint ChunkCoord)
{
	// Possible Optimizations : We don't need to create collisions for most of the chunks.

	if(ProceduralMeshComponent == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT(" %s : ProceduralMeshComponent nullptr"), *GetName());
		return;
	}

	// Drawing Meshes
	ProceduralMeshComponent->CreateMeshSection(
		ChunkSectionIndex,
		Vertices, 
		Triangles, 
		Normals, 
		UVs, 
		TArray<FColor>(), 
		Tangents, 
		true
	);


	// Assigning Landscape Material
	if( LandscapeMaterial )
	{
		ProceduralMeshComponent->SetMaterial(ChunkSectionIndex, LandscapeMaterial);
	}

	// Updating TMap<Key=ChunkCoord, Value=ChunkSectionIndex> ChunkStatus
	ChunkStatus.Add(ChunkCoord, ChunkSectionIndex);


	// IMPORTANT!! ChunkSectionIndex Update!
	ChunkSectionIndex++;
	// IMPORTANT!! 


	// Drawing DebugPoint
	if(ShouldDrawDebugPoint)
	{
		DrawDebugPoints();
	}
	// debug

	return;
}


/* Tools */

void ALandscapeManager::ResetChunkInfo()
{
	Vertices.Reset();
	Normals.Reset();
	UVs.Reset();
	Tangents.Reset();
	
	return;
}

bool ALandscapeManager::IsChunkInRadius(const FIntPoint StartChunk, const FIntPoint CurrentChunkCoord, const float RadiusByLength)
{
	FVector2D StartPointCenter = GetChunkCenter(StartChunk);
	FVector2D CurrentChunkCenter = GetChunkCenter(CurrentChunkCoord);

    return RadiusByLength >= FVector2d::Distance(StartPointCenter, CurrentChunkCenter);
}

FVector2D ALandscapeManager::GetChunkCenter(const FIntPoint& ChunkCoord)
{
	FVector2D ChunkLength = FVector2D( (ChunkVertexCount.X - 1), (ChunkVertexCount.Y - 1)) * CellSize;
	FVector2D Offset = FVector2D( ChunkCoord.X * ChunkLength.X , ChunkCoord.Y * ChunkLength.Y  );

	FVector2D Center = ChunkLength / 2;

	return Offset + Center;
}

FIntPoint ALandscapeManager::GetPlayerLocatedChunk()
{
	if(PlayerCharacter == nullptr)
	{
		UE_LOG(LogTemp, Display, TEXT("PlayerCharacterPointer == nullptr"));

		return FIntPoint(0,0);
	}

	// conversion to 2d Vector (we don't need Z axis)
	FVector2D PlayerLocation = FVector2D( PlayerCharacter->GetActorLocation() ); 
	// length of each side of chunk
	FVector2D Length = FVector2D( (ChunkVertexCount.X - 1) , (ChunkVertexCount.Y - 1) ) * CellSize; 
	// ChunkCoord of the chunk where player is located.
	FIntPoint PlayerLocatedChunkCoord = FIntPoint( int32(PlayerLocation.X / Length.X) , int32(PlayerLocation.Y / Length.Y) );

	if(PlayerLocation.X < 0)
	{
		PlayerLocatedChunkCoord.X -= 1;
	}
	if (PlayerLocation.Y < 0)
	{
		PlayerLocatedChunkCoord.Y -= 1;
	}

	// UE_LOG(LogTemp, Display, TEXT("Player Current ChunkCoord : ( %d , %d )"), PlayerLocatedChunkCoord.X, PlayerLocatedChunkCoord.Y);

	return PlayerLocatedChunkCoord;
}

void ALandscapeManager::DrawDebugPoints()
{
	
	for (int i = 0; i < Vertices.Num(); i++)
	{
		DrawDebugPoint(this->GetWorld(), Vertices[i], 5, FColor::Red, true);
	}
	
	return;
}

float ALandscapeManager::GenerateHeight(const FVector2D& Location)
{

	float height = 0;

	for ( int i = 0; i < PerlinNoiseLayers.Num(); i++)
	{
		float NoiseScale = PerlinNoiseLayers[i].NoiseScale;
		float Amplitude = PerlinNoiseLayers[i].Amplitude;
		float Offset = PerlinNoiseLayers[i].Offset;
		height += FMath::PerlinNoise2D(Location * NoiseScale + FVector2d(0.1f, 0.1f) + Offset) * Amplitude;
	}

	return height;
}


// FLandscapeInfoGenerator

void FLandscapeInfoGenerator::DoWork()
{
	ALandscapeManager* LMP = LandscapeManager;

	LMP->IsChunkInfoGenerating = true;

	FIntPoint& NeededChunk = LMP->NeededChunk;
	TPair<FIntPoint, int32>& RemovableChunk = LMP->RemovableChunk;

	FIntPoint PlayerLocatedChunk = LMP->GetPlayerLocatedChunk();

	// NeededChunk and RemovableChunk are OutParameters
	// try finding Needed and Removable chunk set.


	if( LMP->FindNeededChunk( NeededChunk, PlayerLocatedChunk ) == false 
		|| 
		LMP->FindRemovableChunk( RemovableChunk, PlayerLocatedChunk ) == false )
	{

		LMP->IsChunkInfoGenerating = false;
		return;
	}

	LMP->GenerateChunkInfo(NeededChunk);

	LMP->IsChunkInfoReady = true;
	LMP->IsChunkInfoGenerating = false;

	return;
}