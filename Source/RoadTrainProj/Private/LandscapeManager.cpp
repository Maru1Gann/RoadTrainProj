
#include "LandscapeManager.h"
#include "PerlinNoiseVariables.h"

#include "Containers/Map.h" // MultiMap
#include "Components/SplineComponent.h" // Spline
#include "Components/SplineMeshComponent.h" // Spline Mesh

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
	FlushPersistentDebugLines(GetWorld());
	for (auto& Elem : Chunks)
	{
		RemoveChunk(Elem.Key);
	}
	Chunks.Empty();
}

void ALandscapeManager::Debug()
{
	FlushPersistentDebugLines(GetWorld());
	RemoveLandscape();
	GenerateLandscape();

	FIntPoint TargetChunk = GetChunk(Start);

	DrawDebugPoint( GetWorld(), GridToVector(Start), 15.f, FColor::Cyan, true );
	DrawDebugPoint( GetWorld(), GridToVector(End), 15.f, FColor::Cyan, true );

	TArray<FGate> GatePath;
	PathFinder->GetGatePath(Start, End, GatePath);
	UE_LOG(LogTemp, Warning, TEXT("GatePathNum %d"), GatePath.Num());
	for (auto& Gate : GatePath)
	{
		UE_LOG(LogTemp, Warning, TEXT("Gate from to %s %s"), *GetChunk( Gate.A ).ToString(), *GetChunk( Gate.B ).ToString());
		DrawDebugPoint(
			GetWorld(),
			GridToVector(Gate.A),
			15.f,
			FColor::Cyan,
			true
		);
		DrawDebugPoint(
			GetWorld(),
			GridToVector(Gate.B),
			15.f,
			FColor::Blue,
			true
		);
	}

	for (int32 i = 0; i < GatePath.Num()-1; i++)
	{
		FIntPoint Chunk = GetChunk(GatePath[i].B);
		TArray<FIntPoint> Path;
		PathFinder->GetPath(GatePath[i], GatePath[i + 1], Path);
		for (auto& Point : Path)
		{	DrawDebugPoint(GetWorld(), GridToVector(Point), 3.f, FColor::White, true); }
		PathFinder->SmoothPath(Path);
		for (auto& Point : Path)
		{ DrawDebugPoint(GetWorld(), GridToVector(Point), 5.f, FColor::Black, true); }
		TArray<FVector> ActualPath;
		PathFinder->RebuildPath(Path, ActualPath);
		for (auto& Point : ActualPath)
		{ DrawDebugPoint(GetWorld(), Point, 8.f, FColor::Cyan, true); }
		AddPathSpline(Chunk, Path);
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
	RemoveChunk(Chunk);
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

void ALandscapeManager::AddPathSpline(const FIntPoint& Chunk, const TArray<FIntPoint>& Path)
{
	ARealtimeMeshActor** ppRMA = Chunks.Find(Chunk);
	if(!ppRMA)
	{ return; }
	
	ARealtimeMeshActor* pRMA = (*ppRMA);

	USplineComponent* Spline = NewObject<USplineComponent>(pRMA); // add spline component
	Spline->RegisterComponent(); // register to world.
	Spline->AttachToComponent( pRMA->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform );
	Spline->SetRelativeLocation(FVector::ZeroVector);
	Spline->ClearSplinePoints(false);
	Spline->SetMobility(EComponentMobility::Static);

	// add spline points.
	for (auto& Pos : Path)
	{
		FIntPoint WorldPos = Chunk * (VerticesPerChunk - 1) + Pos;
		FVector WorldVector = FVector(WorldPos.X + 0.5f, WorldPos.Y + 0.5f, 0.f) * VertexSpacing;
		WorldVector.Z = GetHeight(FVector2D(WorldVector.X, WorldVector.Y));
		Spline->AddSplinePoint( WorldVector, ESplineCoordinateSpace::World );
	}
	
	MakeRoad(Spline);
}

void ALandscapeManager::AddPathSpline(const FIntPoint& Chunk, const TArray<FVector>& Path)
{
	ARealtimeMeshActor** ppRMA = Chunks.Find(Chunk);
	if (!ppRMA)
	{
		UE_LOG(LogTemp, Warning, TEXT("WTF"));
		return;
	}

	ARealtimeMeshActor* pRMA = (*ppRMA);

	USplineComponent* Spline = NewObject<USplineComponent>(pRMA); // add spline component
	Spline->RegisterComponent(); // register to world.
	Spline->AttachToComponent(pRMA->GetRootComponent(), FAttachmentTransformRules::KeepWorldTransform);
	//Spline->SetRelativeLocation(FVector::ZeroVector);
	Spline->ClearSplinePoints(false);
	Spline->SetMobility(EComponentMobility::Static);

	// add spline points.
	for (auto& Pos : Path)
	{
		Spline->AddSplinePoint(Pos, ESplineCoordinateSpace::World);
	}

	MakeRoad(Spline);
}

void ALandscapeManager::MakeRoad(USplineComponent* Spline)
{
	if(!this->RoadMesh || !Spline)
	{ return; }

	for (int32 i = 1; i < Spline->GetNumberOfSplinePoints() - 2; i++)
	{
		USplineMeshComponent* SplineMesh = NewObject<USplineMeshComponent>(Spline->GetOwner());
		SplineMesh->RegisterComponent();
		SplineMesh->AttachToComponent(Spline, FAttachmentTransformRules::KeepRelativeTransform);
		SplineMesh->SetRelativeLocation(FVector::ZeroVector);
		
		FVector StartPos, StartTangent, EndPos, EndTangent;
		StartPos = Spline->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::World);
		StartTangent = Spline->GetTangentAtSplinePoint(i, ESplineCoordinateSpace::World);
		EndPos = Spline->GetLocationAtSplinePoint(i + 1, ESplineCoordinateSpace::World);
		EndTangent = Spline->GetTangentAtSplinePoint(i + 1, ESplineCoordinateSpace::World);


		SplineMesh->SetStaticMesh(RoadMesh);
		SplineMesh->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent);
		SplineMesh->SetStartScale(FVector2D(2.0f, 5.0f));
		SplineMesh->SetEndScale(FVector2D(2.0f, 5.0f));
	}
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