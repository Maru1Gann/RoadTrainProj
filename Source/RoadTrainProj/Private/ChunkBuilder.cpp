
#include "ChunkBuilder.h"
#include "PerlinNoiseVariables.h"   // NoiseLayers

#include "PathFinder.h" // path finding
#include "PathNode.h"

#include "LandscapeManager.h"

#include "DrawDebugHelpers.h" // debug points


FChunkBuilder::FChunkBuilder( ALandscapeManager* pLM, UMaterialInterface* ChunkMaterial )
{
	this->VertexSpacing = pLM->VertexSpacing;
	this->VerticesPerChunk = pLM->VerticesPerChunk;
	this->ChunkRadius = pLM->ChunkRadius;
	this->TextureSize = pLM->TextureSize;
	this->ShouldGenerateHeight = pLM->ShouldGenerateHeight;
	this->NoiseLayers = pLM->NoiseLayers;

	this->ChunkMaterial = ChunkMaterial;

	// need to be updated in LandscapeManager::OnConstruction()
	ChunkLength = VertexSpacing * (VerticesPerChunk - 1); 
}

// returns StreamSet for a Chunk
void FChunkBuilder::GetStreamSet(const FIntPoint& Chunk, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet)
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
//void FChunkBuilder::AddChunk( const FIntPoint& Chunk, const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet )
//{
//    if( GetWorld() == nullptr )
//	{
//		UE_LOG(LogTemp, Warning, TEXT("GetWorld() nullptr"));
//		return;
//	}
//
//	// Spawn chunk as Actor
//	ARealtimeMeshActor* RMA = GetWorld()->SpawnActor<ARealtimeMeshActor>();
//	if( RMA == nullptr )
//	{
//		UE_LOG(LogTemp, Warning, TEXT("RMA nullptr"));
//		return;
//	}
//
//	// Set Location
//	FVector Offset = FVector( Chunk.X , Chunk.Y, 0.0f ) * ChunkLength;
//	RMA->SetActorLocation(Offset);
//
//	URealtimeMeshComponent* pRMC = RMA->GetRealtimeMeshComponent();
//	if( pRMC == nullptr )
//	{
//		UE_LOG(LogTemp, Warning, TEXT("pRMC nullptr"));
//		return;
//	}
//
//	URealtimeMeshSimple* RealtimeMesh = RMA->GetRealtimeMeshComponent()->InitializeRealtimeMesh<URealtimeMeshSimple>();
//	if( RealtimeMesh == nullptr )
//	{
//		UE_LOG(LogTemp, Warning, TEXT("RealtimeMesh nullptr"));
//		return;
//	}
//
//	RealtimeMesh->SetupMaterialSlot(0, "PrimaryMaterial");
//	RealtimeMesh->UpdateLODConfig(0, FRealtimeMeshLODConfig(1.00f));
//
//	const FRealtimeMeshSectionGroupKey GroupKey = FRealtimeMeshSectionGroupKey::Create(0, 0);
//	const FRealtimeMeshSectionKey PolyGroup0SectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey, 0);
//
//	// this generates the mesh (chunk)
//	RealtimeMesh->CreateSectionGroup(GroupKey, StreamSet);
//
//	// set it to static
//	RMA->GetRootComponent()->SetMobility(EComponentMobility::Static);
//
//    // set up material
//	if( ChunkMaterial != nullptr )
//	{
//		URealtimeMeshComponent* MeshComp = RMA->GetRealtimeMeshComponent();
//		MeshComp->SetMaterial( 0, ChunkMaterial );
//	}
//
//    // add it to status (member)
//	Chunks.Add(Chunk, RMA);
//
//	// update configuration.
//	RealtimeMesh->UpdateSectionConfig( PolyGroup0SectionKey, FRealtimeMeshSectionConfig(0), true );
//
//}

// Removes Chunk in the world
//void FChunkBuilder::RemoveChunk( const FIntPoint& Chunk )
//{
//    if(	ARealtimeMeshActor** pRMA = Chunks.Find(Chunk) )
//	{
//		(*pRMA)->Destroy();
//		Chunks.Remove(Chunk);
//	}
//	return;
//}


// returns height made with member noiselayers
float FChunkBuilder::GetHeight( const FVector2D& Location )
{
    if( ShouldGenerateHeight == false )
	{ return 0.0f; }
	
    float height = 0.0f;
	if(this->NoiseLayers.Num() <= 0)
	{ return 0.0f; }

	for ( int32 i = 0; i < NoiseLayers.Num(); i++)
	{
		float Frequency = NoiseLayers[i].Frequency;
		if( FMath::IsNearlyZero( Frequency ) )
		{ continue; }
		float NoiseScale = 1.0f / Frequency;
		float Amplitude = NoiseLayers[i].Amplitude;
		float Offset = NoiseLayers[i].Offset;

		height += FMath::PerlinNoise2D(Location * NoiseScale + Offset) * Amplitude;
	}

	return height;
}

// returns Vertices for Streamset.
void FChunkBuilder::GetVertices(const FIntPoint& Chunk, const int32 & StartIndex, const int32 & EndIndex, const int32& VertexSpace, TArray<FVector3f>& OutVertices)
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
void FChunkBuilder::GetUVs(const FIntPoint& Chunk, const int32& StartIndex, const int32& EndIndex, const float& UVscale, TArray<FVector2DHalf>& OutUVs)
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
void FChunkBuilder::GetTriangles(const int32& VertexCount, TArray<uint32>& OutTriangles)
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
void FChunkBuilder::GetTangents( const int32& VertexCount, const TArray<uint32>& BigTriangles, const TArray<FVector3f>& BigVertices, TArray<FVector3f>& OutTangents, TArray<FVector3f>& OutNormals )
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

