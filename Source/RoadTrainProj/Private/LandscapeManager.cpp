
#include "LandscapeManager.h"
#include "PerlinNoiseVariables.h"

#include "Components/SplineComponent.h" // Spline
#include "Components/SplineMeshComponent.h" // Spline Mesh

#include "DrawDebugHelpers.h"

ALandscapeManager::ALandscapeManager()
{
    PrimaryActorTick.bCanEverTick = true; // enable tick
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root")); // cannot see actor in editor bug fix

	Material = nullptr;
	RoadMesh = nullptr;
	IsPath = false;
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
	
	if (IsPath)
	{
		FVector PlayerLocation = GetPlayerLocation();
		if (GetChunk(PlayerLocation) != LastLocation)
		{
			LastLocation = GetChunk(PlayerLocation);
			GenerateChunks(PlayerLocation);
			RemoveChunks(PlayerLocation);
		}
	}
	

}

void ALandscapeManager::BeginPlay()
{
	Super::BeginPlay();

	// on construction.
	GetChunkOrder(ChunkRadius, ChunkOrder);
	ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;
	ChunkBuilder = std::make_unique<FChunkBuilder>(this, this->Material);
	PathFinder = std::make_unique<FPathFinder>(this);

	GatePath.Empty();
	ChunkGates.Empty();
	LastLocation = GetChunk(GetPlayerLocation()) + FIntPoint(-100,-100);

	IsPath = PathFinder->GetGatePath(Start, End, GatePath);
	if (!IsPath) UE_LOG(LogTemp, Warning, TEXT("No Path Error"));

	if (IsPath)
	{
		for (int32 i = 0; i < GatePath.Num() - 1; i++)
			ChunkGates.Add(GetChunk(GatePath[i].B), TPair<FGate, FGate>(GatePath[i], GatePath[i + 1]));
	}

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

void ALandscapeManager::GenerateLandscapeWithPath()
{
	FlushPersistentDebugLines(GetWorld());
	GatePath.Empty();
	IsPath = PathFinder->GetGatePath(Start, End, GatePath);
	if (!IsPath) return;

	ChunkGates.Empty();
	for (int32 i = 0; i < GatePath.Num() - 1; i++)
	{
		FGate GateA = GatePath[i];
		FGate GateB = GatePath[i + 1];
		FIntPoint Chunk = GetChunk(GateA.B);
		ChunkGates.Add(Chunk, TPair<FGate, FGate>(GateA, GateB));
	}

	TArray<FIntPoint> PriorityChunks;
	for (auto& Elem : ChunkOrder)
	{
		if (ChunkGates.Contains(Elem))
			PriorityChunks.Add(Elem);
	}

	for (auto& Elem : PriorityChunks)
	{
		if (PathChunks.Contains(Elem)) continue;

		TArray<FVector> Path;
		TArray<FVector> PathForSpline;
		for (int32 j = -1; j <= 1; j++)
			for (int32 i = -1; i <= 1; i++)
			{
				FIntPoint Target = Elem + FIntPoint(i, j);
				TPair<FGate, FGate>* FoundGates = ChunkGates.Find(Target);
				if (FoundGates)
				{
					TArray<FVector> TempPath;
					PathFinder->GetActualPath((*FoundGates).Key, (*FoundGates).Value, TempPath);
					Path.Append(TempPath);
					if (i == 0 && j == 0) PathForSpline = TempPath;
				}
			}

		RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
		ChunkBuilder->GetPathStreamSet(Elem, Path, StreamSet);
		AddChunk(Elem, StreamSet, true);

		USplineComponent* Spline;
		Spline = AddPathSpline(Elem, PathForSpline);
		MakeRoad(Spline);
	}

	for (auto& Elem : ChunkOrder)
	{
		if (Chunks.Contains(Elem)) continue;

		RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
		ChunkBuilder->GetStreamSet(Elem, StreamSet);
		AddChunk(Elem, StreamSet);
	}

}

void ALandscapeManager::RemoveLandscape()
{
	FlushPersistentDebugLines(GetWorld());

	for (auto& Elem : Chunks) RemoveChunk(Elem.Key);

	Chunks.Empty();
	PathChunks.Empty();
	Splines.Empty();
}



float ALandscapeManager::GetHeight( const FVector2D& Location )
{
	return ChunkBuilder->GetHeight(Location);
}

// Add Chunk as an Actor into the world.
void ALandscapeManager::AddChunk(const FIntPoint& Chunk, const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet, bool IsPathChunk)
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

	// set Mobility
	pRMA->GetRootComponent()->SetMobility(EComponentMobility::Movable);

	// set up material
	if( Material )
	{ pRMC->SetMaterial(0, Material); }

	// add it to status (member)
	if (IsPathChunk)	PathChunks.Add(Chunk, pRMA);
	else				Chunks.Add(Chunk, pRMA);

	// update configuration.
	RealtimeMesh->UpdateSectionConfig( PolyGroup0SectionKey, FRealtimeMeshSectionConfig(0), true );

	// TODO: set collision to Query only. to specific chunks.
}

// check and remove chunk and entry from Member::Chunks TMap
void ALandscapeManager::RemoveChunk(const FIntPoint& Chunk)
{
	ARealtimeMeshActor** ppRMA, ** ppPathRMA;
	ppRMA = Chunks.Find(Chunk);
	ppPathRMA = PathChunks.Find(Chunk);

	if(ppRMA)		(*ppRMA)->Destroy();
	if(ppPathRMA)	(*ppPathRMA)->Destroy();

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

USplineComponent* ALandscapeManager::AddPathSpline(const FIntPoint& Chunk, const TArray<FVector>& Path)
{
	ARealtimeMeshActor** ppRMA = PathChunks.Find(Chunk);
	if (!ppRMA)
	{
		UE_LOG(LogTemp, Warning, TEXT("WTF"));
		return nullptr;
	}

	ARealtimeMeshActor* pRMA = (*ppRMA);

	USplineComponent* Spline = NewObject<USplineComponent>(pRMA); // add spline component
	Spline->RegisterComponent(); // register to world.

	Spline->SetRelativeLocation(FVector::ZeroVector);
	Spline->ClearSplinePoints(false);
	
	pRMA->GetRootComponent()->SetMobility(EComponentMobility::Movable);
	Spline->SetMobility(EComponentMobility::Movable);
	Spline->AttachToComponent(pRMA->GetRootComponent(), FAttachmentTransformRules::KeepWorldTransform);


	// add spline points.
	for (auto& Pos : Path)
	{
		Spline->AddSplinePoint(Pos, ESplineCoordinateSpace::World);
	}

	USplineComponent** ppSpline = Splines.Find(Chunk);
	if (ppSpline)
	{ (*ppSpline)->DestroyComponent(); } // if already exists, remove it

	Splines.Add(Chunk, Spline); // add it to TMap.

	

	return Spline;
}

void ALandscapeManager::MakeRoad(USplineComponent* Spline)
{
	if(!this->RoadMesh || !Spline)
	{ return; }

	for (int32 i = 0; i < Spline->GetNumberOfSplinePoints() - 1; i++)
	{
		USplineMeshComponent* SplineMesh = NewObject<USplineMeshComponent>(Spline->GetOwner());
		SplineMesh->RegisterComponent();
		SplineMesh->SetMobility(EComponentMobility::Movable);
		SplineMesh->AttachToComponent(Spline, FAttachmentTransformRules::KeepWorldTransform);
		SplineMesh->SetWorldLocation(FVector::ZeroVector);
		
		FVector StartPos, StartTangent, EndPos, EndTangent;
		StartPos = Spline->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::World);
		StartTangent = Spline->GetTangentAtSplinePoint(i, ESplineCoordinateSpace::World);
		EndPos = Spline->GetLocationAtSplinePoint(i + 1, ESplineCoordinateSpace::World);
		EndTangent = Spline->GetTangentAtSplinePoint(i + 1, ESplineCoordinateSpace::World);


		SplineMesh->SetStaticMesh(RoadMesh);
		SplineMesh->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent);
		SplineMesh->SetStartScale(FVector2D(2.0f, 5.0f));
		SplineMesh->SetEndScale(FVector2D(2.0f, 5.0f));
		SplineMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	}
}

void ALandscapeManager::GenerateChunks(const FVector& PlayerLoc)
{
	FIntPoint ChunkNow = GetChunk(PlayerLoc);

	TArray<FIntPoint> PriorityChunks;
	for (auto& Elem : ChunkOrder)
	{
		FIntPoint Target = Elem + ChunkNow;
		if (ChunkGates.Contains(Target))
			PriorityChunks.Add(Target);
	}

	for (auto& Elem : PriorityChunks)
	{
		if (PathChunks.Contains(Elem)) continue;

		TArray<FVector> Path;
		TArray<FVector> PathForSpline;
		for (int32 j = -1; j <= 1; j++)
			for (int32 i = -1; i <= 1; i++)
			{
				FIntPoint Target = Elem + FIntPoint(i, j);
				TPair<FGate, FGate>* FoundGates = ChunkGates.Find(Target);
				if (FoundGates)
				{
					TArray<FVector> TempPath;
					PathFinder->GetActualPath((*FoundGates).Key, (*FoundGates).Value, TempPath);
					Path.Append(TempPath);
					if (i == 0 && j == 0) PathForSpline = TempPath;
				}
			}

		RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
		ChunkBuilder->GetPathStreamSet(Elem, Path, StreamSet);
		AddChunk(Elem, StreamSet, true);

		USplineComponent* Spline;
		Spline = AddPathSpline(Elem, PathForSpline);
		MakeRoad(Spline);
	}

	for (auto& Elem : ChunkOrder)
	{
		FIntPoint Target = Elem + ChunkNow;
		if (Chunks.Contains(Target)) continue;

		RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
		ChunkBuilder->GetStreamSet(Target, StreamSet);
		AddChunk(Target, StreamSet);
	}
}

void ALandscapeManager::RemoveChunks(const FVector& PlayerLoc)
{
	FIntPoint ChunkNow = GetChunk(PlayerLoc);
	TSet<FIntPoint> CurrentChunks;
	for (auto& Elem : ChunkOrder) CurrentChunks.Add(ChunkNow + Elem);

	TSet<FIntPoint> Removables;

	for (auto& Elem : Chunks)
	{
		if (!CurrentChunks.Contains(Elem.Key))
		{
			Elem.Value->Destroy();
			Removables.Add(Elem.Key);
		}
	}
	for (auto& Elem : Removables) Chunks.Remove(Elem);

	Removables.Empty();
	for (auto& Elem : PathChunks)
	{
		if (!CurrentChunks.Contains(Elem.Key))
		{
			Elem.Value->Destroy();
			Removables.Add(Elem.Key);
		}
	}
	for (auto& Elem : Removables)
	{
		PathChunks.Remove(Elem);
		Splines.Remove(Elem);
	}

}

FVector ALandscapeManager::GetPlayerLocation()
{
	UWorld* pWord = GetWorld();
	APlayerController* pPlayerCon = nullptr;
	APawn* pPlayerPawn = nullptr;
	if (pWord) pPlayerCon = pWord->GetFirstPlayerController();
	if (pPlayerCon) pPlayerPawn = pPlayerCon->GetPawn();

	if (pPlayerPawn) 
		return pPlayerPawn->GetActorLocation();
	else 
		return FVector(0.f, 0.f, 0.f);
}

// returns center of grid.
FVector ALandscapeManager::GridToVector(const FIntPoint& GlobalGrid)
{
	FVector Out;
	Out.X = (GlobalGrid.X + 0.5f)  * VertexSpacing;
	Out.Y = (GlobalGrid.Y + 0.5f)  * VertexSpacing;
	Out.Z = GetHeight(FVector2D(Out.X, Out.Y));
	return Out;
}

FIntPoint ALandscapeManager::GetChunk(const FIntPoint& GlobalGrid)
{
	FIntPoint Chunk;
	Chunk.X = FMath::FloorToInt32(float(GlobalGrid.X) / float(VerticesPerChunk - 1));
	Chunk.Y = FMath::FloorToInt32(float(GlobalGrid.Y) / float(VerticesPerChunk - 1));
	return Chunk;
}

FIntPoint ALandscapeManager::GetChunk(const FVector& Vector)
{
	FIntPoint Chunk;
	Chunk.X = FMath::FloorToInt32(Vector.X / ChunkLength);
	Chunk.Y = FMath::FloorToInt32(Vector.Y / ChunkLength);
	return Chunk;
}
