
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
	ShouldWorkCounter = 0;

	UpdateDelayFrames = 2;
	DoWorkFrame = 3;

	UseAsync = true;

	CoverageRadius = 3;
	DetailCount = 5;

	RoadScale = FVector2D(2.0f, 5.0f);
}

void ALandscapeManager::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	GetChunkOrder(ChunkRadius, ChunkOrder);
	GetChunkOrder(ChunkRadius + 1, BigChunkOrder);

	ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;

	ChunkBuilder = std::make_unique<FChunkBuilder>(this, this->Material);
	PathFinder = std::make_unique<FPathFinder>(this);
}

void ALandscapeManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (UseAsync)
	{
		FIntPoint ChunkNow = GetChunk(GetPlayerLocation());
		if (ShouldDoWork(ChunkNow))
		{
			TryUpdatingGoal(ChunkNow);
			AsyncWork(ChunkNow);
		}

		if (FrameCounter % DoWorkFrame == 0)
		{
			FrameCounter = 0;
			Process(ChunkNow);
		}
		FrameCounter++;
	}

}

void ALandscapeManager::BeginPlay()
{
	Super::BeginPlay();

	// on construction.
	GetChunkOrder(ChunkRadius, ChunkOrder);
	GetChunkOrder(ChunkRadius + 1, BigChunkOrder);

	ChunkLength = (VerticesPerChunk - 1) * VertexSpacing;

	ChunkBuilder = std::make_unique<FChunkBuilder>(this, this->Material);
	PathFinder = std::make_unique<FPathFinder>(this);
	// on construction.

	GatePath.Empty();
	GateMap.Empty();

	FIntPoint PlayerChunk = GetChunk(GetPlayerLocation());
	LastLocation = PlayerChunk + FIntPoint(-100, -100);

	UE_LOG(LogTemp, Warning, TEXT("Overriding Start and End on BeginPlay"));
	Start = ( PlayerChunk - FIntPoint(ChunkRadius * 2, PlayerChunk.Y) ) * (VerticesPerChunk-1);
	End = (PlayerChunk + FIntPoint(ChunkRadius * 2, PlayerChunk.Y)) * (VerticesPerChunk - 1);

	IsPath = PathFinder->GetGatePath(Start, End, GatePath);
	if (!IsPath) { UE_LOG(LogTemp, Warning, TEXT("No Path Error")); }
	else { UE_LOG(LogTemp, Warning, TEXT("GatePathNum %d"), GatePath.Num()); }


	UpdateGateMap();
	UE_LOG(LogTemp, Warning, TEXT("GateMapNum %d"), GateMap.Num() );

}

void ALandscapeManager::GenerateLandscape()
{
	RemoveLandscape();

	TArray<FVector> EmptyArray;
	EmptyArray.Empty();
	for (auto& Elem : ChunkOrder)
	{
		RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
		ChunkBuilder->GetStreamSet(Elem, EmptyArray, StreamSet);
		AddChunk(Elem, StreamSet);
	}
}

void ALandscapeManager::GenerateLandscapeWithPath()
{
	RemoveLandscape();

	GatePath.Empty();
	PathFinder->GetGatePath(Start, End, GatePath);
	UpdateGateMap();

	for (auto& Chunk : ChunkOrder)
	{
		TArray<FVector> Paths;
		TArray<FVector> PathForSpline;

		for (int32 j = -1; j <= 1; j++)
			for (int32 i = -1; i <= 1; i++)
			{
				FIntPoint Target = Chunk + FIntPoint(i,j);
				TPair<FGate, FGate>* FoundGates = GateMap.Find(Target);
				if (FoundGates)
				{
					TArray<FVector> TempPath;
					FGate GateA = (*FoundGates).Key;
					FGate GateB = (*FoundGates).Value;

					PathFinder->GetActualPath(GateA, GateB, TempPath);
					if (i == 0 && j == 0) PathForSpline = TempPath;
					Paths.Append(TempPath);
				}
			}

		RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
		ChunkBuilder->GetStreamSet(Chunk, Paths, StreamSet);
		AddChunk(Chunk, StreamSet);

		if (!PathForSpline.IsEmpty())
		{
			USplineComponent* pSpline = AddPathSpline(Chunk, PathForSpline);
			if(pSpline) MakeRoad(pSpline);
			PathForSpline.Empty();
		}
	}

}


void ALandscapeManager::RemoveLandscape()
{
	FlushPersistentDebugLines(GetWorld());

	TSet<FIntPoint> RemoveSet;
	for (auto& Elem : Chunks) RemoveSet.Add(Elem.Key);
	for (auto& Elem : RemoveSet) RemoveChunk(Elem);

	Chunks.Empty();
}

void ALandscapeManager::Debug()
{
	FlushPersistentDebugLines(GetWorld());

	TMap<FIntPoint, TPair<FGate, float>> Gates;
	PathFinder->GetGates(FGate(Start), Start, Gates, true);
	
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

	// set Mobility
	pRMA->GetRootComponent()->SetMobility(EComponentMobility::Movable);

	// set up material
	if( Material )
	{ pRMC->SetMaterial(0, Material); }

	// add it to status (member)
	{ // scopelock writing.
		FRWScopeLock Lock(RWChunksMutex, FRWScopeLockType::SLT_Write);
		Chunks.Add(Chunk, pRMA); 
	}
	

	// update configuration.
	RealtimeMesh->UpdateSectionConfig( PolyGroup0SectionKey, FRealtimeMeshSectionConfig(0), true );

	// TODO: set collision to Query only. to specific chunks.
}

// check and remove chunk and entry from Member::Chunks TMap
bool ALandscapeManager::RemoveChunk(const FIntPoint& Chunk)
{
	ARealtimeMeshActor** ppRMA = Chunks.Find(Chunk);
	bool Destroyed = false;
	if ( ppRMA && (*ppRMA) )
	{
		Destroyed = (*ppRMA)->Destroy();
	}
	if (Destroyed)
	{
		// scopelock writing.
		FRWScopeLock Lock(RWChunksMutex, FRWScopeLockType::SLT_Write);
		Chunks.Remove(Chunk);
	}
	return true;
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

bool ALandscapeManager::IsChunkInRad(const FIntPoint& ChunkNow, const FIntPoint& TargetChunk)
{
	FIntPoint Dist = TargetChunk - ChunkNow;
	return (FMath::Abs(Dist.X) <= ChunkRadius && FMath::Abs(Dist.Y) <= ChunkRadius);
}
bool ALandscapeManager::IsChunkInRad(const FIntPoint& ChunkNow, const FIntPoint& TargetChunk, const int32& BoxRadius)
{
	FIntPoint Dist = TargetChunk - ChunkNow;
	return (FMath::Abs(Dist.X) <= BoxRadius && FMath::Abs(Dist.Y) <= BoxRadius);
}

USplineComponent* ALandscapeManager::AddPathSpline(const FIntPoint& Chunk, const TArray<FVector>& Path)
{
	ARealtimeMeshActor** ppRMA = Chunks.Find(Chunk);
	if (!ppRMA)
	{
		UE_LOG(LogTemp, Warning, TEXT("addPathSpline ppRMA nullptr"));
		return nullptr;
	}
	ARealtimeMeshActor* pRMA = (*ppRMA);
	if (!pRMA)
	{
		UE_LOG(LogTemp, Warning, TEXT("addPathSpline pRMA nullptr"));
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
	
	MakeRoad(Spline);
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
		SplineMesh->SetStartScale(RoadScale);
		SplineMesh->SetEndScale(RoadScale);
		SplineMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
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

void ALandscapeManager::UpdateGateMap(const int32& StartIndex)
{
	for (int32 i = StartIndex; i < GatePath.Num() - 1; i++)
	{
		FIntPoint Chunk = GetChunk(GatePath[i].B);
		GateMap.Add(Chunk, TPair<FGate,FGate>(GatePath[i], GatePath[i + 1]));
	}
}


// async works below.


// call it on game thread and it will do multithreading.
void ALandscapeManager::AsyncWork(const FIntPoint& ChunkNow)
{
	if (!ChunkQueue.IsEmpty()) return;

	// make datas for it ( background thread )
	AsyncTask(ENamedThreads::AnyBackgroundHiPriTask, [this, ChunkNow]()
		{
			TMap<FIntPoint, TPair<FGate, FGate>> NearGatesMap;
			FindNearGates(ChunkNow, NearGatesMap);

			TArray<FIntPoint> ChunksNeeded;
			FindChunksNeeded(ChunkNow, ChunksNeeded);

			this->UpdateDataQueue( ChunksNeeded, NearGatesMap );
		}
	);

}

// game thread work.
void ALandscapeManager::Process(const FIntPoint& ChunkNow)
{
	// always remove first.
	bool Removed = FindAndRemoveChunk(ChunkNow);
	if(!Removed) DequeueAndAddChunk(ChunkNow);
}

bool ALandscapeManager::FindAndRemoveChunk(const FIntPoint& ChunkNow)
{
	for (auto& Elem : Chunks)
	{
		if (!IsChunkInRad(ChunkNow, Elem.Key))
		{
			RemoveChunk(Elem.Key);
			return true;
		}
	}

	return false;
}

bool ALandscapeManager::DequeueAndAddChunk(const FIntPoint& ChunkNow)
{

	while (!ChunkQueue.IsEmpty())
	{
		FChunkData ChunkData;
		ChunkQueue.Dequeue(ChunkData);
		const FIntPoint& Chunk = ChunkData.Chunk;
		const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet = ChunkData.StreamSet;
		const TArray<FVector> Path = ChunkData.ActualPath;

		if (!Chunks.Contains(Chunk) && IsChunkInRad(ChunkNow, Chunk))
		{
			AddChunk(Chunk, StreamSet);
			if (!Path.IsEmpty()) AddPathSpline(Chunk, Path);
			return true;
		}
	}

	return false;
}


void ALandscapeManager::UpdateDataQueue(const TArray<FIntPoint> ChunksNeeded, const TMap<FIntPoint, TPair<FGate, FGate>> NearGatesMap)
{

	ParallelFor(ChunksNeeded.Num(), [ChunksNeeded, NearGatesMap, this](int32 Index)
		{ // lambda body

			FIntPoint Chunk = ChunksNeeded[Index];

			// find gates belong to neighbor chunks.
			TArray<TPair<FGate, FGate>> NearGates;
			for (int32 j = -1; j <= 1; j++)
				for (int32 i = -1; i <= 1; i++)
				{
					const TPair<FGate, FGate>* FoundGates = NearGatesMap.Find(Chunk + FIntPoint(i, j));
					if (FoundGates) NearGates.Add( (*FoundGates) );
				}

			this->ChunkQueue.Enqueue(MakeChunkData(Chunk, NearGates));
		}
	);

}


FChunkData ALandscapeManager::MakeChunkData(const FIntPoint TargetChunk, const TArray< TPair<FGate, FGate> > NearGates)
{

	TArray<FVector> Paths;
	TArray<FVector> PathForSpline;
	for (auto& Elem : NearGates)
	{
		TArray<FVector> TempPath;
		PathFinder->GetActualPath(Elem.Key, Elem.Value, TempPath);
		Paths.Append(TempPath);
		if (GetChunk(Elem.Key.B) == TargetChunk) PathForSpline = TempPath;
	}

	RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
	ChunkBuilder->GetStreamSet(TargetChunk, Paths, StreamSet);
	FChunkData Out(TargetChunk, StreamSet, PathForSpline);
	return Out;
}


// finds gates that are in BigChunkOrder radius.
void ALandscapeManager::FindNearGates(const FIntPoint& ChunkNow, TMap<FIntPoint, TPair<FGate, FGate>>& OutGatesMap)
{
	OutGatesMap.Empty();

	FScopeLock Lock(&GatesMutex);
	for (auto& Elem : this->BigChunkOrder)
	{
		FIntPoint TargetChunk = Elem + ChunkNow;
		TPair<FGate, FGate>* FoundGates = this->GateMap.Find(TargetChunk);
		if (FoundGates) OutGatesMap.Add(TargetChunk, (*FoundGates));
	}
}

void ALandscapeManager::FindChunksNeeded(const FIntPoint& ChunkNow, TArray<FIntPoint>& OutChunksNeeded)
{
	OutChunksNeeded.Empty();
	TQueue<FIntPoint, EQueueMode::Mpsc> TempQueue;

	{// scopelock read
		FRWScopeLock Lock(RWChunksMutex, FRWScopeLockType::SLT_ReadOnly);

		ParallelFor(ChunkOrder.Num(), [&TempQueue, ChunkNow, this](int32 Index)
			{
				FIntPoint Target = this->ChunkOrder[Index] + ChunkNow;
				if (!Chunks.Contains(Target)) TempQueue.Enqueue(Target);
			}
		);

	}

	while (!TempQueue.IsEmpty())
	{
		FIntPoint Target;
		TempQueue.Dequeue(Target);
		OutChunksNeeded.Add(Target);
	}
}


bool ALandscapeManager::ShouldDoWork(const FIntPoint& ChunkNow)
{
	if (!ChunkQueue.IsEmpty()) return false;

	if (LastLocation != ChunkNow)
	{
		LastLocation = ChunkNow;
		ShouldWorkCounter = 1;
	}
	else if (ShouldWorkCounter != 0)
	{
		if (ShouldWorkCounter % UpdateDelayFrames == 0)
		{
			ShouldWorkCounter = 0;
			return true;
		}
		else ShouldWorkCounter++;
	}

	return false;
}


// Infinite pathfinding. maybe we should use a dedicated thread.
// do one way first.


void ALandscapeManager::TryUpdatingGoal(const FIntPoint& ChunkNow)
{

	if (ShouldUpdateGoal(ChunkNow))
	{
		UE_LOG(LogTemp, Warning, TEXT("Attempting to Update Goal"));
		UpdateGoal();
	}
	if (UpdatedGoal)
	{
		UE_LOG(LogTemp, Warning, TEXT("Goal Updated!"));
		PathWorker.reset();
		UpdatedGoal = false;
	}

}

bool ALandscapeManager::ShouldUpdateGoal(const FIntPoint& ChunkNow)
{
	return IsChunkInRad(ChunkNow, GetChunk(End), ChunkRadius * 2);
}

void ALandscapeManager::UpdateGoal()
{
	PathWorker = std::make_unique<FPathWorker>(this);
}


// --------------pathworker.

bool FPathWorker::Init()
{
	if (!pLM) return false;

	if (pLM->GatePath.Num() - 2 < 0) return false;

	this->Start = pLM->GatePath[pLM->GatePath.Num() - 2].B; // Last one is goal, so -1. Last gate is our start.
	this->End = pLM->GatePath.Last().A + FIntPoint(pLM->ChunkRadius * 2 * (pLM->VerticesPerChunk - 1), 0);
	return true;
}

uint32 FPathWorker::Run()
{
	TArray<FGate> NewGatePath;
	bool Success = pLM->PathFinder->GetGatePath(Start, End, NewGatePath);
	if (!Success)
	{
		UE_LOG(LogTemp, Error, TEXT("INF Path Calc Error. Abort"));
		return 0;
	}

	
	{	// scopelock

		FScopeLock Lock(&pLM->GatesMutex);

		int32 LastIndex = pLM->GatePath.Num() - 1;
		// remove last element. (goal).
		pLM->GatePath.RemoveAt(LastIndex--);
		// set first. (gate to prev goal chunk).
		NewGatePath[0] = pLM->GatePath.Last();
		// remove last element. ( gate to prev goal chunk )
		pLM->GatePath.RemoveAt(LastIndex--);
		pLM->GatePath.Append(NewGatePath);
		pLM->End = this->End;
		pLM->UpdateGateMap(LastIndex);

	}	// scopelock

	return uint32(0); // success
}

void FPathWorker::Exit()
{
	pLM->UpdatedGoal = true;
}

FPathWorker::~FPathWorker()
{
	if (Thread)
	{
		Thread->Kill(true);
		delete Thread;
		Thread = nullptr;
	}

}
