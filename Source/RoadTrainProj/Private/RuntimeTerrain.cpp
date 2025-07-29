
#include "RuntimeTerrain.h"
#include "PerlinNoiseVariables.h"   // NoiseLayers

#include "PathFinder2.h" // path finding
#include "PathNode.h"

#include "DrawDebugHelpers.h" // debug points


ARuntimeTerrain::ARuntimeTerrain()
{
    PrimaryActorTick.bCanEverTick = false; // disable tick
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root")); // cannot see actor in editor bug fix
    
	this->PathFinder = MakeShared<FPathFinder>( *this );
}

void ARuntimeTerrain::OnConstruction( const FTransform& Transform )
{
    Super::OnConstruction(Transform);
    
    ChunkLength = VertexSpacing * ( VerticesPerChunk - 1 );
    GetChunkOrder( this->ChunkRadius, this->ChunkOrder );

}

void ARuntimeTerrain::BeginPlay()
{
	
	Super::BeginPlay();

	// Start and End Red debug point
	DrawDebugPoint(
		this->GetWorld(),
		ConvertTo3D(Start),
		30.f,
		FColor::Red,
		true
	);
	DrawDebugPoint(
		this->GetWorld(),
		ConvertTo3D(End),
		30.f,
		FColor::Red,
		true
	);

	// High Level PathFinding Test. 
	TArray<FPathNode> Gates;
	PathFinder->FindPathGates(Start, End, Gates);

	for( auto& Elem : Gates )
	{
		FVector Current = ConvertTo3D( PathFinder->ConvertToGlobal( Elem.Belong, Elem.Pos ) );

        UE_LOG(LogTemp, Warning, 
            TEXT("Current %s, %s, %s"), *Elem.Belong.ToString(), *Elem.Next.ToString(), *Elem.Pos.ToString());

		DrawDebugPoint(
			this->GetWorld(),
			Current,
			15.f,
			FColor::Blue,
			true
		);
	}

	// Path Rebuilding Test

	for( int i = 0; i < Gates.Num() - 1; i++)
	{
		TArray<FIntPoint> Path;
		PathFinder->GetPath(Gates[i], Gates[i+1], Path);

		for( auto& Elem: Path )
		{
			
			DrawDebugPoint(
				this->GetWorld(),
				ConvertTo3D( PathFinder->ConvertToGlobal( Gates[i].Next, Elem ) ),
				5.f,
				FColor::Green,
				true
			);

		}
	}
	
}

void ARuntimeTerrain::GenerateLandscape()
{
    for( auto& Elem: this->ChunkOrder )
    {
        if( Chunks.Contains(Elem) ) // if already exists.
        {
            continue;
        }

        RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
        GetStreamSet( Elem, StreamSet );  // OutStreamSet
        AddChunk( Elem, StreamSet );
    }
    return;
}

void ARuntimeTerrain::RemoveLandscape()
{
    for( auto& Elem: this->Chunks )
    {
        RemoveChunk( Elem.Key );
    }

    return;
}




// returns StreamSet for a Chunk
void ARuntimeTerrain::GetStreamSet(const FIntPoint& Chunk, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet)
{
	// // Chunk location offset
	// FVector2D Offset = FVector2D( Chunk.X , Chunk.Y ) * ChunkLength;
	
	// scale UV based on vetex spacing
	float UVScale = VertexSpacing / TextureSize;

	// actual data to use
	TArray<FVector3f> Vertices, Tangents, Normals; 
	TArray<uint32> Triangles;
	TArray<FVector2DHalf> UVs;

	GetVertices( Chunk, 0, VerticesPerChunk, VertexSpacing, 
		Vertices ); // this line for OutParam.

	GetUVs( Chunk, 0, VerticesPerChunk, UVScale, 
		UVs );

	GetTriangles( VerticesPerChunk, 
		Triangles );

	Tangents.SetNum(Vertices.Num());
	Normals.SetNum(Vertices.Num());

	// Big data for uv and normal continue on different chunks
	TArray<FVector3f> BigVertices;
	TArray<uint32> BigTriangles;
	
	GetVertices( Chunk, -1, VerticesPerChunk + 1, VertexSpacing, 
		BigVertices );
	GetTriangles( VerticesPerChunk + 2, 
		BigTriangles );
	
	// Normals and Tangents made out of Big datas.
	GetTangents( VerticesPerChunk, BigTriangles, BigVertices, 
		Tangents, Normals );


	// Datas into StreamSet( class Member Var ) RMC syntax
	RealtimeMesh::TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 1> Builder( OutStreamSet );
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

// Adds Chunk into the World
void ARuntimeTerrain::AddChunk( const FIntPoint& Chunk, const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet )
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
	FVector Offset = FVector( Chunk.X , Chunk.Y, 0.0f ) * ChunkLength;
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

    // set up material
	if( ChunkMaterial != nullptr )
	{
		URealtimeMeshComponent* MeshComp = RMA->GetRealtimeMeshComponent();
		MeshComp->SetMaterial( 0, ChunkMaterial );
	}

    // add it to status (member)
	Chunks.Add(Chunk, RMA);

	// update configuration.
	RealtimeMesh->UpdateSectionConfig( PolyGroup0SectionKey, FRealtimeMeshSectionConfig(0), true );

}

// Removes Chunk in the world
void ARuntimeTerrain::RemoveChunk( const FIntPoint& Chunk )
{
    if(	ARealtimeMeshActor** pRMA = Chunks.Find(Chunk) )
	{
		(*pRMA)->Destroy();
		Chunks.Remove(Chunk);
	}
	return;
}



// Returns 2d index of whirl. Used for chunk generation from closest point.
// Should be called only once in OnConstruction()
void ARuntimeTerrain::GetChunkOrder( const int32& ChunkRad, TArray<FIntPoint>& OutArray )
{
    OutArray.Empty();
    OutArray.SetNum( ( ChunkRad*2 + 1 ) * ( ChunkRad*2 + 1 ) );

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
	OutArray[Index++] = Pos;
	int32 LocalStep = 2;
	while( Index < OutArray.Num() )
	{
		// one step down
		Pos.Y++;
		OutArray[Index++] = Pos;

		for( int32 i = 1; i<LocalStep; i++ )
		{
			Pos.X++;
			OutArray[Index++] = Pos;
		}
		for( int32 i = 0; i<LocalStep; i++)
		{
			Pos.Y--;
			OutArray[Index++] = Pos;
		}
		for (int32 i = 0; i<LocalStep; i++)
		{
			Pos.X--;
			OutArray[Index++] = Pos;
		}
		for (int32 i = 0; i<LocalStep; i++)
		{
			Pos.Y++;
			OutArray[Index++] = Pos;
		}

		LocalStep += 2;
	}

    return;
}

// returns PlayerLocation
FVector2D ARuntimeTerrain::GetPlayerLocation()
{
    APlayerController* PlayerCon = GetWorld()->GetFirstPlayerController();
    if( PlayerCon )
    {
        FVector Loc = PlayerCon->GetPawn()->GetActorLocation();
        return FVector2D( Loc.X, Loc.Y );
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("PlayerController nullptr"));
        return FVector2D(0,0);
    }
}

// returns the Chunk where Location belongs.
FIntPoint ARuntimeTerrain::GetChunk( const FVector2D& Location )
{
    return FIntPoint( FMath::FloorToInt32( Location.X / ChunkLength ), FMath::FloorToInt32( Location.Y / ChunkLength ) );
}

// returns Location with height
FVector ARuntimeTerrain::ConvertTo3D( const FVector2D& Location )
{
    return FVector( Location.X, Location.Y, GetHeight(Location) );
}
// Converts Global FIntPoint to FVector with Height
FVector ARuntimeTerrain::ConvertTo3D( const FIntPoint& Location )
{
	return ConvertTo3D( FVector2D( Location.X, Location.Y ) * VertexSpacing );
}

// returns height made with member noiselayers
float ARuntimeTerrain::GetHeight( const FVector2D& Location )
{
    if( ShouldGenerateHeight == false )
	{
		return 0.0f;
	}
	
    float height = 0.0f;
	if(this->NoiseLayers.Num() <= 0)
	{
		return 0.0f;
	}

	for ( int32 i = 0; i < NoiseLayers.Num(); i++)
	{
		float Frequency = NoiseLayers[i].Frequency;
		if(Frequency == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("Perlin Noise frequency can't be 0"));
			Frequency = 0.001;
		}
		float NoiseScale = 1.0f / Frequency;
		float Amplitude = NoiseLayers[i].Amplitude;
		float Offset = NoiseLayers[i].Offset;

		height += FMath::PerlinNoise2D(Location * NoiseScale + Offset) * Amplitude;
	}

	return height;
}

// returns Vertices for Streamset.
void ARuntimeTerrain::GetVertices(const FIntPoint& Chunk, const int32 & StartIndex, const int32 & EndIndex, const int32& VertexSpace, TArray<FVector3f>& OutVertices)
{

	OutVertices.Empty();

	FVector2D Offset = FVector2D( Chunk.X , Chunk.Y ) * ChunkLength; 		// ChunkLength should always be same.
	
	for( int32 iY = StartIndex; iY < EndIndex; iY++ )
	{
		for( int32 iX = StartIndex; iX < EndIndex; iX++ )
		{
			FVector3f Vertex = FVector3f(iX, iY, 0.0f) * VertexSpace; 		// VertexSpacing & VertexCount may vary later. (LoD)
			if( this->ShouldGenerateHeight )
			{
				Vertex.Z = GetHeight( FVector2D( Vertex.X, Vertex.Y ) +  Offset );
			}
			OutVertices.Add( Vertex );
		}
	}

	return;
}

// returns UVs for StreamSet
void ARuntimeTerrain::GetUVs(const FIntPoint& Chunk, const int32& StartIndex, const int32& EndIndex, const float& UVscale, TArray<FVector2DHalf>& OutUVs)
{
	
	OutUVs.Empty();

	int32 VertexCount = EndIndex - StartIndex;

	for( int32 iY = StartIndex; iY < EndIndex; iY++ )
	{
		for( int32 iX = StartIndex; iX < EndIndex; iX++ )
		{
			FVector2DHalf UV; // Mark startpoint of every uv tile
			UV.X = ( Chunk.X * (VertexCount - 1) + iX ) * UVscale;
			UV.Y = ( Chunk.Y * (VertexCount - 1) + iY ) * UVscale;

			OutUVs.Add( UV );
		}
	}

	return;
}

// returns Triangle indices for StreamSet
void ARuntimeTerrain::GetTriangles(const int32& VertexCount, TArray<uint32>& OutTriangles)
{
	OutTriangles.Empty();

	for (int32 iY = 0; iY < VertexCount - 1; iY++)
	{
		for (int32 iX = 0; iX < VertexCount - 1; iX++)
		{
			int32 CurrentVertex = iX + iY * VertexCount;
			OutTriangles.Add( CurrentVertex );
			OutTriangles.Add( CurrentVertex + VertexCount );
			OutTriangles.Add( CurrentVertex + 1 );

			OutTriangles.Add( CurrentVertex + VertexCount );
			OutTriangles.Add( CurrentVertex + VertexCount + 1 ) ;
			OutTriangles.Add( CurrentVertex + 1 );
		}
	}

	return;
}

// returns Small Tangents and Normals that are continuous along chunks.
void ARuntimeTerrain::GetTangents( const int32& VertexCount, const TArray<uint32>& BigTriangles, const TArray<FVector3f>& BigVertices, TArray<FVector3f>& OutTangents, TArray<FVector3f>& OutNormals )
{

	int32 RowLength = VertexCount + 2;

	RealtimeMeshAlgo::GenerateTangents(
		TConstArrayView<uint32>( BigTriangles ),
		BigVertices,
		nullptr, // UV Getter. We made uv.
		[&OutNormals, &OutTangents, &RowLength](int index, FVector3f Tangent, FVector3f Normal) -> void
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
			OutNormals[index] = Normal;
			OutTangents[index] = Tangent;
			},
		true
	);

	return;
}