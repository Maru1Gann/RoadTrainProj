
#include "LandscapeManager.h"

#include "PerlinNoiseVariables.h"

ALandscapeManager::ALandscapeManager()
{
    PrimaryActorTick.bCanEverTick = true; // enable tick
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root")); // cannot see actor in editor bug fix

}

void ALandscapeManager::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	GetChunkOrder(ChunkRadius, ChunkOrder);
	ChunkBuilder = std::make_unique<FChunkBuilder>(this, this->Material);
	PathFinder = std::make_unique<FPathFinder>(this);
}

void ALandscapeManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void ALandscapeManager::BeginPlay()
{
	Super::BeginPlay();

}


void ALandscapeManager::GenerateLandscape()
{
}

void ALandscapeManager::RemoveLandscape()
{
}

float ALandscapeManager::GetHeight( const FVector2D& Location )
{
	return ChunkBuilder->GetHeight(Location);
}

void ALandscapeManager::AddChunk(const FIntPoint& Chunk, const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet)
{
}

void ALandscapeManager::RemoveChunk(const FIntPoint& Chunk)
{
}

// Returns 2d index of whirl. Used for chunk generation from closest point.
// Should be called only once in OnConstruction()
void ALandscapeManager::GetChunkOrder(const int32& ChunkRad, TArray<FIntPoint>& OutArray)
{
	OutArray.Empty();
	OutArray.SetNum((ChunkRad * 2 + 1) * (ChunkRad * 2 + 1));

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

	FIntPoint Pos = FIntPoint(0, 0);
	int32 Index = 0;
	OutArray[Index++] = Pos;
	int32 LocalStep = 2;
	while (Index < OutArray.Num())
	{
		// one step down
		Pos.Y++;
		OutArray[Index++] = Pos;

		for (int32 i = 1; i < LocalStep; i++)
		{
			Pos.X++;
			OutArray[Index++] = Pos;
		}
		for (int32 i = 0; i < LocalStep; i++)
		{
			Pos.Y--;
			OutArray[Index++] = Pos;
		}
		for (int32 i = 0; i < LocalStep; i++)
		{
			Pos.X--;
			OutArray[Index++] = Pos;
		}
		for (int32 i = 0; i < LocalStep; i++)
		{
			Pos.Y++;
			OutArray[Index++] = Pos;
		}

		LocalStep += 2;
	}

	return;
}