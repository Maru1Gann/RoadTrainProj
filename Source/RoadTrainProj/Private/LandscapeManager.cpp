
#include "LandscapeManager.h"
#include "PerlinNoiseVariables.h"

#include "Containers/Map.h" // MultiMap

#include "DrawDebugHelpers.h"

ALandscapeManager::ALandscapeManager()
{
    PrimaryActorTick.bCanEverTick = true; // enable tick
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root")); // cannot see actor in editor bug fix

	Start = FIntPoint(VerticesPerChunk, VerticesPerChunk) / 2;
	End = Start * 3;

	Material = nullptr;
	ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;

}

void ALandscapeManager::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	GetChunkOrder(ChunkRadius, ChunkOrder);
	ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;
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
	for (auto& Elem : ChunkOrder)
	{
		RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
		ChunkBuilder->GetStreamSet(Elem, StreamSet);
		AddChunk(Elem, StreamSet);
	}
}

void ALandscapeManager::RemoveLandscape()
{
	for (auto& Elem : Chunks)
	{
		RemoveChunk(Elem.Key);
	}

}

void ALandscapeManager::Debug()
{
	FlushPersistentDebugLines(GetWorld());
	RemoveLandscape();

	PathFinder->FindPathGates(Start, End, PathNodes);
	
	
	for (auto& Elem : PathNodes)
	{
		DrawDebugPoint
		(
			GetWorld(),
			GetNodeVector(Elem),
			15.f,
			FColor::Blue,
			true
		);
	}

	TMultiMap<FIntPoint, int32> PathMap;
	for (int32 i = 1; i < PathNodes.Num(); i++)
	{ PathMap.Add(PathNodes[i].Belong, i); }

	for (auto& Elem : ChunkOrder)
	{
		TArray<int32> Indices;
		PathMap.MultiFind(Elem, Indices);

		RealtimeMesh::FRealtimeMeshStreamSet StreamSet;

		if (!Indices.IsEmpty()) // if Chunk Contains Path
		{
			TArray<FIntPoint> Path;
			for (auto& Index : Indices) // get path
			{
				TArray<FIntPoint> TempPath;
				PathFinder->GetPath(PathNodes[Index - 1], PathNodes[Index], TempPath);
				Path.Append(TempPath);
			}
			ChunkBuilder->GetStreamSet(Elem, Path, StreamSet); // flattener version.
			DrawPathDebugPoints(Elem, Path);
		}
		else
		{ ChunkBuilder->GetStreamSet(Elem, StreamSet); } // normal version.

		AddChunk(Elem, StreamSet);
	}
}

float ALandscapeManager::GetHeight( const FVector2D& Location )
{
	return ChunkBuilder->GetHeight(Location);
}

// Add Chunk as an Actor into the world.
void ALandscapeManager::AddChunk(const FIntPoint& Chunk, const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet)
{
	UWorld* pWorld = GetWorld();
	if (!pWorld)
	{ UE_LOG(LogTemp, Warning, TEXT("GetWorld() nullptr")); 
	return; }

	// Spawn chunk as Actor
	ARealtimeMeshActor* pRMA = pWorld->SpawnActor<ARealtimeMeshActor>();
	if (!pRMA)
	{ UE_LOG(LogTemp, Warning, TEXT("RMA nullptr")); 
	return; }

	URealtimeMeshComponent* pRMC = pRMA->GetRealtimeMeshComponent();
	if (!pRMC)
	{ UE_LOG(LogTemp, Warning, TEXT("pRMC nullptr"));
	return; }

	URealtimeMeshSimple* RealtimeMesh = pRMC->InitializeRealtimeMesh<URealtimeMeshSimple>();
	if (!RealtimeMesh)
	{ UE_LOG(LogTemp, Warning, TEXT("RealtimeMesh nullptr"));
	return; }


	// Set Location
	FVector Offset = FVector( Chunk.X , Chunk.Y, 0.0f ) * ChunkLength;
	pRMA->SetActorLocation(Offset);

	// Set MaterialSlot
	RealtimeMesh->SetupMaterialSlot(0, "PrimaryMaterial");
	RealtimeMesh->UpdateLODConfig(0, FRealtimeMeshLODConfig(1.00f));

	const FRealtimeMeshSectionGroupKey GroupKey = FRealtimeMeshSectionGroupKey::Create(0, 0);
	const FRealtimeMeshSectionKey PolyGroup0SectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey, 0);

	// this generates the mesh (chunk)
	RealtimeMesh->CreateSectionGroup(GroupKey, StreamSet);

	// set it to static
	pRMA->GetRootComponent()->SetMobility(EComponentMobility::Static);

	// set up material
	if( Material )
	{ pRMC->SetMaterial(0, Material); }

	// add it to status (member)
	Chunks.Add(Chunk, pRMA);

	// update configuration.
	RealtimeMesh->UpdateSectionConfig( PolyGroup0SectionKey, FRealtimeMeshSectionConfig(0), true );

	// TODO: set collision to Query only. to specific chunks.
}

// check and remove chunk and entry from Member::Chunks TMap
void ALandscapeManager::RemoveChunk(const FIntPoint& Chunk)
{
	ARealtimeMeshActor** pRMA = Chunks.Find(Chunk);
	if (pRMA)
	{
		(*pRMA)->Destroy();
		Chunks.Remove(Chunk);
	}

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

FVector ALandscapeManager::GetNodeVector(const FPathNode& Node)
{
	FVector Out;
	Out = FVector(Node.Belong.X, Node.Belong.Y, 0.f) * ChunkLength;
	Out += FVector(Node.Pos.X, Node.Pos.Y, 0.f) * VertexSpacing;
	Out.Z = GetHeight(FVector2D(Out.X, Out.Y) );

	return Out;
}

void ALandscapeManager::DrawPathDebugPoints(const FIntPoint& Chunk, const TArray<FIntPoint>& Path)
{
	for (int32 i = 0; i < Path.Num(); i++)
	{
		DrawDebugPoint
		(
			GetWorld(),
			GetNodeVector(FPathNode(Chunk, Chunk, Path[i])),
			5.f,
			FColor::White,
			true
		);
	}
}