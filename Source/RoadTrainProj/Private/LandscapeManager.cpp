
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
	FrameCounter = 0;
	UpdateDelayFrames = 2;
	LastLocChanged = false;
	bool UseAsnyc = true;
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
	
	if (!UseAsync)
	{
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
	else
	{
		if ( ChunkQueue.IsEmpty() && ChunkRemovalQueue.IsEmpty() && ShouldDoWork() )
		{
			AsyncWork();
		}

		ProcessQueues();
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
		TSet<FIntPoint> NoBuildChunks;
		for (int32 j = -1; j <= 1; j++)
			for (int32 i = -1; i <= 1; i++)
			{
				FIntPoint Target = Elem + FIntPoint(i, j);
				TPair<FGate, FGate>* FoundGates = ChunkGates.Find(Target);
				if (FoundGates)
				{
					NoBuildChunks.Add( Target );
					TArray<FVector> TempPath;
					PathFinder->GetActualPath((*FoundGates).Key, (*FoundGates).Value, TempPath);
					Path.Append(TempPath);
					if (i == 0 && j == 0) PathForSpline = TempPath;
				}
			}

		RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
		ChunkBuilder->GetPathStreamSet(Elem, Path, NoBuildChunks, StreamSet);
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

void ALandscapeManager::Debug()
{
	FlushPersistentDebugLines(GetWorld());
	
	FIntPoint Chunk = GetChunk(Start);
	int32 DetailCount = 5;
	float DetailSpacing = VertexSpacing / DetailCount;

	for(int32 k = 0; k < VerticesPerChunk-1; k++)
		for(int32 l = 0; l < VerticesPerChunk-1; l++)
			for(int32 j = 0; j<=DetailCount; j++)
				for (int32 i = 0; i <= DetailCount; i++)
				{
					FIntPoint Offset = ( Chunk * (VerticesPerChunk-1) + FIntPoint(l,k) ) * (DetailCount);
					FIntPoint DetailGrid = Offset + FIntPoint(i, j);
					FVector ActualPos = FVector(DetailGrid.X, DetailGrid.Y, 0.f) * DetailSpacing;
					ActualPos.Z = GetHeight(FVector2D(ActualPos.X, ActualPos.Y)) + 100.f;

					FColor Color = FColor::Cyan;
					bool Draw = false;
					if (!ChunkBuilder->IsGridInChunk(Chunk, DetailGrid, DetailCount))	Color = FColor::Red, Draw = true;
					if (ChunkBuilder->IsOnBoundary(DetailGrid, DetailCount))			Color = FColor::Purple, Draw = true;
					if (Draw) DrawDebugPoint(GetWorld(), ActualPos, 15.f, Color, true);
				}
}

float ALandscapeManager::GetHeight( const FVector2D& Location )
{
	return ChunkBuilder->GetHeight(Location);
}

// returns center of grid.
FVector ALandscapeManager::GridToVector(const FIntPoint& GlobalGrid)
{
	FVector Out;
	Out.X = (GlobalGrid.X + 0.5f) * VertexSpacing;
	Out.Y = (GlobalGrid.Y + 0.5f) * VertexSpacing;
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


// private бщ

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
bool ALandscapeManager::RemoveChunk(const FIntPoint& Chunk)
{
	
	ARealtimeMeshActor** ppRMA, ** ppPathRMA;
	ppRMA = Chunks.Find(Chunk);
	ppPathRMA = PathChunks.Find(Chunk);

	bool Out = false;
	if (ppRMA || ppPathRMA) Out = true;

	if(ppRMA)		(*ppRMA)->Destroy();
	if(ppPathRMA)	(*ppPathRMA)->Destroy();

	return Out;
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
	if (!pRMA)
	{
		UE_LOG(LogTemp, Warning, TEXT("WTF"));
		return nullptr;
	}

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
		TSet<FIntPoint> NoBuildChunks;
		for (int32 j = -1; j <= 1; j++)
			for (int32 i = -1; i <= 1; i++)
			{
				FIntPoint Target = Elem + FIntPoint(i, j);
				TPair<FGate, FGate>* FoundGates = ChunkGates.Find(Target);
				if (FoundGates)
				{
					TArray<FVector> TempPath;
					NoBuildChunks.Add(Target);
					PathFinder->GetActualPath((*FoundGates).Key, (*FoundGates).Value, TempPath);
					Path.Append(TempPath);
					if (i == 0 && j == 0) PathForSpline = TempPath;
				}
			}

		RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
		ChunkBuilder->GetPathStreamSet(Elem, Path, NoBuildChunks, StreamSet);
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

// async works below.

void ALandscapeManager::AsyncWork()
{
	FIntPoint ChunkNow = GetChunk(GetPlayerLocation());

	TSet<FIntPoint> ChunksToRemove;
	TArray<FIntPoint> ChunksNeeded, RoadChunksNeeded;

	// find Chunks need to be removed.	( game thread )
	FindChunksToRemove(ChunkNow, ChunksToRemove);
	for (auto& Elem : ChunksToRemove) ChunkRemovalQueue.Enqueue(Elem);

	// find Chunks need to be made.( game thread )
	FindChunksToMake(ChunkNow, RoadChunksNeeded, ChunksNeeded);

	// make subset of ChunkGates to pass it to worker threads.
	TMap<FIntPoint, TPair<FGate, FGate>> NearGatesMap;
	FindChunkGates(ChunkNow, NearGatesMap);


	// make datas for it ( background thread )
	// need to work with all the paths first. ( for continuous chunks )

	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, RoadChunksNeeded, ChunksNeeded, NearGatesMap]()
		{
			this->UpdateDataQueue(RoadChunksNeeded, ChunksNeeded, NearGatesMap);
		}
	);

}

void ALandscapeManager::FindChunksToMake(const FIntPoint& ChunkNow, TArray<FIntPoint>& RoadChunksNeeded, TArray<FIntPoint>& ChunksNeeded)
{
	RoadChunksNeeded.Empty();
	ChunksNeeded.Empty();

	for (auto& Chunk : ChunkOrder)
	{
		FIntPoint Target = Chunk + ChunkNow;
		if ( !PathChunks.Contains(Target) && ChunkGates.Contains(Target) )
		{
			RoadChunksNeeded.Add(Target);
		}

		if (!Chunks.Contains(Target))
		{
			ChunksNeeded.Add(Target);
		}
	}

	return;
}

void ALandscapeManager::FindChunksToRemove(const FIntPoint& ChunkNow, TSet<FIntPoint>& ChunksToRemove)
{
	ChunksToRemove.Empty();
	TSet<FIntPoint> CurrentChunks;
	for (auto& Elem : ChunkOrder) CurrentChunks.Add(ChunkNow + Elem);

	for (auto& Elem : Chunks)
	{
		if (!CurrentChunks.Contains(Elem.Key))
			ChunksToRemove.Add(Elem.Key);
	}
	for (auto& Elem : PathChunks)
	{
		if (!CurrentChunks.Contains(Elem.Key))
			ChunksToRemove.Add(Elem.Key);
	}

}

void ALandscapeManager::FindChunkGates(const FIntPoint& ChunkNow, TMap<FIntPoint, TPair<FGate, FGate>>& OutGatesMap)
{
	OutGatesMap.Empty();

	for (auto& Elem : ChunkOrder)
	{
		FIntPoint TargetChunk = Elem + ChunkNow;
		TPair<FGate, FGate>* FoundGates = this->ChunkGates.Find(TargetChunk);
		if (FoundGates) OutGatesMap.Add(TargetChunk, (*FoundGates));
	}
}

void ALandscapeManager::ProcessQueues()
{
	bool Work = false;
	Work = ProcessRemovalQueue();
	if( !Work ) ProcessChunkQueue();
}

bool ALandscapeManager::ProcessRemovalQueue()
{
	bool Out = false;
	if (!ChunkRemovalQueue.IsEmpty())
	{
		FIntPoint Remover;
		ChunkRemovalQueue.Dequeue(Remover);
		Out = RemoveChunk(Remover);

		Chunks.Remove(Remover);
		PathChunks.Remove(Remover);
		Splines.Remove(Remover);
	}
	return Out;
}

bool ALandscapeManager::ProcessChunkQueue()
{
	FChunkData TempData;
	bool Out = false;
	if (!ChunkQueue.IsEmpty())
	{
		ChunkQueue.Dequeue(TempData);

		if (!TempData.ActualPath.IsEmpty() && !PathChunks.Contains(TempData.Chunk))
		{
			AddChunk(TempData.Chunk, TempData.StreamSet, true);
			USplineComponent* pSpline = AddPathSpline(TempData.Chunk, TempData.ActualPath);
			MakeRoad(pSpline);

			Out = true;
		}
		else if (TempData.ActualPath.IsEmpty() && !Chunks.Contains(TempData.Chunk)) // normalchunk && not already made.
		{
			AddChunk(TempData.Chunk, TempData.StreamSet, false);
			Out = true;
		}
	}
	return Out;
}

bool ALandscapeManager::ShouldDoWork()
{
	FIntPoint ChunkNow = GetChunk(GetPlayerLocation());
	bool DoWork = false;
	const int32& Divider = UpdateDelayFrames;
	int32& Counter = FrameCounter;

	if (ChunkNow != LastLocation)
	{
		UE_LOG(LogTemp, Warning, TEXT("LastLocChanged"));
		LastLocChanged = true;
		LastLocation = ChunkNow;
		Counter = 1;
	}
	else if (LastLocChanged)
	{
		
		Counter += 1;
		UE_LOG(LogTemp, Warning, TEXT("Counting %d"), Counter);
		if (Counter % Divider == 0)
		{
			Counter %= Divider;
			LastLocChanged = false;
			DoWork = true;
		}
	}

	return DoWork;
}



void ALandscapeManager::UpdateDataQueue(const TArray<FIntPoint> RoadChunksNeeded, const TArray<FIntPoint> ChunksNeeded, const TMap<FIntPoint, TPair<FGate, FGate>> NearGatesMap)
{

	// road(path)chunks first. (for continuous chunks)
	ParallelFor(RoadChunksNeeded.Num(), [&RoadChunksNeeded, &NearGatesMap, this] (int32 Index) 
		{ // lambda body
			FIntPoint Chunk = RoadChunksNeeded[Index];
			FChunkData TempChunkData = MakePathChunkData(Chunk, NearGatesMap);
			this->ChunkQueue.Enqueue(TempChunkData);
		}
	);
	
	ParallelFor(ChunksNeeded.Num(), [&ChunksNeeded, this](int32 Index)
		{ // lambda body
			FIntPoint Chunk = ChunksNeeded[Index];
			FChunkData TempChunkData = MakeChunkData(Chunk);
			this->ChunkQueue.Enqueue(TempChunkData);
		}
	);

}

FChunkData ALandscapeManager::MakePathChunkData(const FIntPoint TargetChunk, const TMap<FIntPoint, TPair<FGate, FGate>> NearPaths)
{
	TArray<FVector> Paths;
	TArray<FVector> PathForSpline;
	TSet<FIntPoint> NoBuildChunks;

	for(int32 j = -1; j<=1; j++)
		for (int32 i = -1; i <= 1; i++)
		{
			FIntPoint Neighbor = TargetChunk + FIntPoint(i, j);
			const TPair<FGate, FGate>* FoundGates = NearPaths.Find(Neighbor);
			if (!FoundGates) continue;

			NoBuildChunks.Add( Neighbor );

			TArray<FVector> TempPath;
			PathFinder->GetActualPath((*FoundGates).Key, (*FoundGates).Value, TempPath);
			Paths.Append(TempPath);

			if (i == 0 && j == 0) PathForSpline = TempPath;
		}

	RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
	ChunkBuilder->GetPathStreamSet(TargetChunk, Paths, NoBuildChunks, StreamSet);

	FChunkData Out(TargetChunk, StreamSet, PathForSpline);

	return Out;
}

FChunkData ALandscapeManager::MakeChunkData(const FIntPoint TargetChunk)
{
	RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
	ChunkBuilder->GetStreamSet(TargetChunk, StreamSet);

	TArray<FVector> EmptyArray;
	EmptyArray.Empty();
	FChunkData Out(TargetChunk, StreamSet, EmptyArray);
	return Out;
}
