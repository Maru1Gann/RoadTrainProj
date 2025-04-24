// Fill out your copyright notice in the Description page of Project Settings.


#include "RMCLandscape.h"


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
		for (int i = 0; i < step - 1; i++)
		{
			
			CurrentCoord += FIntPoint(0,1);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}

		// step back
		for (int i = 0; i < step; i++)
		{
			
			CurrentCoord += FIntPoint(-1, 0);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}

		// step down
		for (int i = 0; i < step; i++)
		{

			CurrentCoord += FIntPoint(0, -1);
			if ( IsChunkInRadius(FIntPoint(0,0), CurrentCoord) )
			{
				ChunkOrder.Add(CurrentCoord);
			}
		}

		// step forward
		for (int i = 0; i < step; i++)
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

	for ( int i = 0; i < PerlinNoiseLayers.Num(); i++)
	{
		float NoiseScale = 1.0f / PerlinNoiseLayers[i].Frequency;
		float Amplitude = PerlinNoiseLayers[i].Amplitude;
		float Offset = PerlinNoiseLayers[i].Offset;

		height += FMath::PerlinNoise2D(Location * NoiseScale + Offset) * Amplitude;
	}

	return height;
}