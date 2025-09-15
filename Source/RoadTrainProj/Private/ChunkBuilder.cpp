
#include "ChunkBuilder.h"
#include "PerlinNoiseVariables.h"   // NoiseLayers

#include "PathFinder.h" // path finding

#include "Containers/Map.h" // MultiMap

#include "LandscapeManager.h"

#include <limits.h>
const float INFLOAT = std::numeric_limits<float>::infinity(); // float INF for distance


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
	
	GetBigVertices( Chunk, Vertices,
		BigVertices );
	GetTriangles( VerticesPerChunk + 2, 
		BigTriangles );
	
	// Normals and Tangents made out of Big datas.
	GetTangents( VerticesPerChunk, BigTriangles, BigVertices, 
		Tangents, Normals );


	// Datas into StreamSet( class Member Var ) RMC syntax
	OutStreamSet.Empty();
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

void FChunkBuilder::GetStreamSet(const FIntPoint& Chunk, const TArray<FVector>& InPath, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet)
{
	// scale UV based on vetex spacing
	float UVScale = VertexSpacing / TextureSize;

	// actual data to use
	TArray<FVector3f> Vertices, Tangents, Normals;
	TArray<uint32> Triangles;
	TArray<FVector2DHalf> UVs;


	GetVertices(Chunk, 0, VerticesPerChunk, VertexSpacing,
		Vertices); // this line for OutParam.

	AdjustHeight(Chunk, InPath, Vertices);

	GetUVs(Chunk, 0, VerticesPerChunk, UVScale,
		UVs);

	GetTriangles(VerticesPerChunk,
		Triangles);

	Tangents.SetNum(Vertices.Num());
	Normals.SetNum(Vertices.Num());

	// Big data for uv and normal continue on different chunks
	TArray<FVector3f> BigVertices;
	TArray<uint32> BigTriangles;

	GetBigVertices(Chunk, Vertices,
		BigVertices);
	GetTriangles(VerticesPerChunk + 2,
		BigTriangles);

	// Normals and Tangents made out of Big datas.
	GetTangents(VerticesPerChunk, BigTriangles, BigVertices,
		Tangents, Normals);


	// Datas into StreamSet( class Member Var ) RMC syntax
	OutStreamSet.Empty();
	RealtimeMesh::TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 1> Builder(OutStreamSet);
	Builder.EnableTangents();
	Builder.EnableTexCoords();
	Builder.EnableColors();
	Builder.EnablePolyGroups();

	for (int32 i = 0; i < Vertices.Num(); i++)
	{
		Builder.AddVertex(Vertices[i])
			.SetNormalAndTangent(Normals[i], Tangents[i])
			.SetColor(FColor::White)
			.SetTexCoord(UVs[i]);
	}

	for (int32 i = 0; i < Triangles.Num(); i += 6)
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

// DetailCount == how many squares will fit in one grid. (row)
void FChunkBuilder::GetPathStreamSet(const FIntPoint& Chunk, const TArray<FVector>& InPath, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet, const int32& DetailCount)
{
	if (int32(VertexSpacing) % DetailCount != 0) // does not fit.
	{
		UE_LOG(LogTemp, Warning, TEXT("DetailCount cannot divide VertexSpacing"));
		return; 
	}

	float UVScale = VertexSpacing / TextureSize; // use same value with GetStreamSet.

	TArray<FVector3f> Vertices, Tangents, Normals;
	TArray<uint32> Triangles;
	TArray<FVector2DHalf> UVs;

	// x, y == local.

	TMultiMap<FIntPoint, int32> GridWithPath;
	TSet<FIntPoint> GridNeeded;
	for (int32 k = 0; k < InPath.Num(); k++) // init. find GridNeeded to be made, by checking GridWithPaths.
	{
		FIntPoint GlobalGrid = GetGlobalGrid( InPath[k] );

		GridWithPath.Add(GlobalGrid, k);

		for (int32 j = -2; j <= 2; j++)
		{
			for (int32 i = -2; i <= 2; i++)
			{
				FIntPoint Needed = FIntPoint(i,j) + GlobalGrid;
				if( IsGridInChunk(Chunk, Needed) ) GridNeeded.Add(Needed);
				// else, Save it to Global for usage.
			}
		}
	} // init end.
	

	auto Sorter = [](const FIntPoint& A, const FIntPoint& B) // sort by (X,Y) small to big
		{
			if (A.Y < B.Y) return true;
			else if (A.Y == B.Y)
			{
				return A.X < B.X;
			}
			else return false;
		};


	// GridNeeded.Sort(Sorter);
	float DetailSpacing = VertexSpacing / DetailCount;

	TMap<FIntPoint, int32> DetailNeeded; // Global FIntPoint, Index for Verts.
	for (auto& GlobalGrid : GridNeeded)
	{

		// add vertices needed.
		for (int32 j = 0; j <= DetailCount; j++)
		{
			for (int32 i = 0; i <= DetailCount; i++)
			{
				FIntPoint DetailGrid = FIntPoint(i, j);
				FIntPoint Offset = GlobalGrid * DetailCount;
				DetailNeeded.Add(DetailGrid + Offset); // global FIntPoint, but smaller vertexspacing.
			}
		}

	}


	DetailNeeded.KeySort(Sorter); // sort it to get right index.
	int32 Index = 0;
	for (auto& Elem : DetailNeeded)
	{
		Elem.Value = Index++; // give index to it.
		FIntPoint GlobalGrid = Elem.Key;

		FIntPoint LocalGrid = GlobalGrid - Chunk * (VerticesPerChunk - 1) * DetailCount;
		FVector3f Vertex = FVector3f(LocalGrid.X, LocalGrid.Y, 0.f) * DetailSpacing;		// local space for vertex.
		Vertex.Z = GetHeight( FVector2D(GlobalGrid.X, GlobalGrid.Y) * DetailSpacing );		// global space for height. 
		
		// find nearest roadpoint and set to it's height.
		TArray<FIntPoint> NeighborWithPath;
		int32 Rad = 2;
		for (int32 j = -Rad; j <= Rad; j++)
		{
			for (int32 i = -Rad; i <= Rad; i++)
			{
				FIntPoint BigGlobalGrid;
				BigGlobalGrid.X = FMath::FloorToInt32(GlobalGrid.X * DetailSpacing / VertexSpacing);
				BigGlobalGrid.Y = FMath::FloorToInt32(GlobalGrid.Y * DetailSpacing / VertexSpacing);
				FIntPoint Neighbor = FIntPoint(i, j) + BigGlobalGrid;
				if (GridWithPath.Contains(Neighbor))	NeighborWithPath.Add(Neighbor);
			}
		}

		TArray<int32> AllIndices;	// get all indices in neighbor grids.
		for (auto& Neighbor : NeighborWithPath) 
		{
			TArray<int32> Indices;
			GridWithPath.MultiFind(Neighbor, Indices);
			if (!Indices.IsEmpty()) AllIndices.Append(Indices);
		}


		float SumWeight = 0.f;
		float SumHeight = 0.f;
		float ClosestDist = INFLOAT;
		for (auto& PathIndex : AllIndices)	 
		{
			FVector Point = InPath[PathIndex];
			FVector2D GlobalVec = FVector2D(GlobalGrid.X, GlobalGrid.Y) * DetailSpacing;
			float Distance = FVector2D::Distance(FVector2D(Point.X, Point.Y), GlobalVec);
			if (Distance <= ClosestDist) ClosestDist = Distance;

			float Weight = 1.f / Distance + 0.001f;
			SumWeight += Weight;
			SumHeight += Weight * Point.Z;
		}

		float Alpha = 1.0f - ClosestDist / 20000.f; // 20meter
		Vertex.Z = FMath::Lerp(Vertex.Z, SumHeight / SumWeight, Alpha);
		

		Vertices.Add(Vertex);																// add it to vertices.

		// don't know if this is right way, but let's just do it.
		FVector2DHalf UV;
		UV.X = GlobalGrid.X * UVScale;
		UV.Y = GlobalGrid.Y * UVScale;
		UVs.Add(UV);
		// if points are on the edge of chunk, maybe we should update global save. (for continuous chunk)
	}

	
	for (auto& Elem : DetailNeeded) // now we have index, let's make triangles.
	{
		FIntPoint GlobalGrid = Elem.Key;
		int32 Index0 = Elem.Value;

		int32 *Index1, *Index2, *Index3; // square.
		//	0	1
		//	2	3
		Index1 = DetailNeeded.Find(GlobalGrid + FIntPoint(1, 0));
		Index2 = DetailNeeded.Find(GlobalGrid + FIntPoint(0, 1));
		Index3 = DetailNeeded.Find(GlobalGrid + FIntPoint(1, 1));

		if (Index1 && Index2 && Index3) // if all three are in the map.
		{
			// CounterClockWise.
			Triangles.Add(Index0);
			Triangles.Add(*Index2);
			Triangles.Add(*Index1);

			Triangles.Add(*Index2);
			Triangles.Add(*Index3);
			Triangles.Add(*Index1);
		}
	}
	
	Normals.SetNum(Vertices.Num());
	Tangents.SetNum(Vertices.Num());

	RealtimeMeshAlgo::GenerateTangents(
		TConstArrayView<uint32>(Triangles),
		Vertices,
		nullptr,
		[&Normals, &Tangents](int32 index, FVector3f Tangent, FVector3f Normal) -> void
		{
			Normals[index] = Normal;
			Tangents[index] = Tangent;
		},
		true
	);


	// use builder to put it into streamset.
	OutStreamSet.Empty();
	RealtimeMesh::TRealtimeMeshBuilderLocal<uint32, FPackedNormal, FVector2DHalf, 1> Builder(OutStreamSet);
	Builder.EnableTangents();
	Builder.EnableTexCoords();
	Builder.EnableColors();
	Builder.EnablePolyGroups();

	for (int32 i = 0; i < Vertices.Num(); i++)
	{
		Builder.AddVertex(Vertices[i])
			.SetNormalAndTangent(Normals[i], Tangents[i])
			.SetColor(FColor::White)
			.SetTexCoord(UVs[i]);
	}

	for (int32 i = 0; i < Triangles.Num(); i += 6)
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


// ---- private below ------

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
			{ Vertex.Z = GetHeight( FVector2D( Vertex.X, Vertex.Y ) +  Offset ); }
			OutVertices.Add( Vertex );
		}
	}

	return;
}

void FChunkBuilder::AdjustHeight(const FIntPoint& Chunk, const TArray<FVector>& InPath, TArray<FVector3f>& Vertices)
{
	TSet<FIntPoint> GridSet;
	TSet<FIntPoint> VertexSet;

	for (auto& Elem : InPath)
	{
		FIntPoint Pivot = GetGlobalGrid(Elem);
		GridSet.Add(Pivot);
	}

	for (auto& Elem : GridSet) // add 5x5 neighbors
	{

		for (int32 j = -2; j <= 2; j++)
		{
			for (int32 i = -2; i <= 2; i++)
			{
				if (IsGridInChunk(Chunk, Elem))
				{
					FIntPoint Local = Elem - Chunk * (VerticesPerChunk - 1);
					FIntPoint Pivot = Local + FIntPoint(i, j);
					VertexSet.Add(Pivot);
					VertexSet.Add(Pivot + FIntPoint(0, 1));
					VertexSet.Add(Pivot + FIntPoint(1, 1));
					VertexSet.Add(Pivot + FIntPoint(1, 0));
				}
			}
		}
	}

	for (auto& Elem : VertexSet)
	{
		if (IsIndexInChunk(Elem))
		{
			int32 Index = Elem.X + Elem.Y * VerticesPerChunk;
			Vertices[Index].Z -= 1000.f; // minus ten meters.
		}
		
	}

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

// this returns one vertex bigger square for normal calculation.
void FChunkBuilder::GetBigVertices(const FIntPoint& Chunk, const TArray<FVector3f>& SmallVertices, TArray<FVector3f>& OutVertices)
{
	OutVertices.Empty();
	FVector2D Offset = FVector2D(Chunk.X, Chunk.Y) * ChunkLength;

	for (int32 i = -1; i < VerticesPerChunk + 1; i++)
	{
		for (int32 j = -1; j < VerticesPerChunk + 1; j++)
		{
			int32 Index = GetIndex(FIntPoint(j, i));

			if (IsIndexInChunk(FIntPoint(j, i)))	// if it's in small vertices, just copy.
			{
				OutVertices.Add(SmallVertices[Index]);
			}
			else									// if it's not in small vertices, make one.
			{
				FVector3f Vertex = FVector3f(j, i, 0.0f) * VertexSpacing;
				if (this->ShouldGenerateHeight)
				{
					Vertex.Z = GetHeight(FVector2D(Vertex.X, Vertex.Y) + Offset);
				}
				OutVertices.Add(Vertex);
			}


		} // for j
	} // for i

}

void FChunkBuilder::MakeSquare(const int32& Index, const int32& CurrentVertex, const int32& VertexCount, TArray<uint32>& OutTriangles, bool Invert)
{
	if (Invert)
	{
		OutTriangles[Index + 0] = CurrentVertex;
		OutTriangles[Index + 1] = CurrentVertex + VertexCount + 1;
		OutTriangles[Index + 2] = CurrentVertex + 1;

		OutTriangles[Index + 3] = CurrentVertex;
		OutTriangles[Index + 4] = CurrentVertex + VertexCount;
		OutTriangles[Index + 5] = CurrentVertex + VertexCount + 1;
	}
	else
	{
		OutTriangles[Index + 0] = CurrentVertex;
		OutTriangles[Index + 1] = CurrentVertex + VertexCount;
		OutTriangles[Index + 2] = CurrentVertex + 1;

		OutTriangles[Index + 3] = CurrentVertex + VertexCount;
		OutTriangles[Index + 4] = CurrentVertex + VertexCount + 1;
		OutTriangles[Index + 5] = CurrentVertex + 1;
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

int32 FChunkBuilder::GetIndex(const int32& VertexCount, const FIntPoint& Pos)
{
	return Pos.Y * VertexCount + Pos.X;
}

int32 FChunkBuilder::GetIndex(const FIntPoint& Pos)
{
	return GetIndex(VerticesPerChunk, Pos);
}

FIntPoint FChunkBuilder::GetChunk(const FIntPoint& GlobalGrid)
{
	FIntPoint Out;
	Out.X = FMath::FloorToInt32(float(GlobalGrid.X) / (VerticesPerChunk - 1) );
	Out.Y = FMath::FloorToInt32(float(GlobalGrid.Y) / (VerticesPerChunk - 1) );
	return Out;
}

FIntPoint FChunkBuilder::GetGlobalGrid(const FVector& Vector)
{
	FIntPoint Out;
	Out.X = FMath::FloorToInt32(Vector.X / VertexSpacing);
	Out.Y = FMath::FloorToInt32(Vector.Y / VertexSpacing);
	return Out;
}

bool FChunkBuilder::IsIndexInChunk(const int32& VertexCount, const FIntPoint& Index)
{
	return Index.X >= 0 && Index.X < VertexCount && Index.Y >= 0 && Index.Y < VertexCount;
}

bool FChunkBuilder::IsIndexInChunk(const FIntPoint& Index)
{
	return IsIndexInChunk(VerticesPerChunk, Index);
}

bool FChunkBuilder::IsGridInChunk(const FIntPoint& Chunk, const FIntPoint& GlobalGrid)
{
	return GetChunk(GlobalGrid) == Chunk;
}

