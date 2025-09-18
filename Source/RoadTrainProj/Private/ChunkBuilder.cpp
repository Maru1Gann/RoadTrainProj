
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
	CoverageRad = 3;
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

	// change heights after getting normals.
	AdjustHeight(Chunk, Vertices); 

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

	AdjustHeight(Chunk, InPath, Vertices, CoverageRad); // change heights after getting normals.

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
	if (int32(VertexSpacing / 100) % DetailCount != 0) // does not fit. (meter)
	{
		UE_LOG(LogTemp, Warning, TEXT("DetailCount cannot divide VertexSpacing"));
		return; 
	}

	TArray<FVector3f> Vertices, Tangents, Normals;
	TArray<uint32> Triangles;
	TArray<FVector2DHalf> UVs;

	// x, y == local.
	TArray<FVector> Path;
	for (int32 i = 0; i < InPath.Num()-1; i++) // sample more points.
	{
		Path.Add(InPath[i]);
		Path.Add((InPath[i] + InPath[i + 1]) / 2);
	}
	Path.Add( InPath.Last() );


	// init. find GridNeeded to be made, by checking GridWithPaths.
	TMultiMap<FIntPoint, int32> GridWithPath;
	TSet<FIntPoint> GridNeeded;
	int32 Rad = this->CoverageRad;
	for (int32 k = 0; k < Path.Num(); k++) 
	{
		FIntPoint GlobalGrid = GetGlobalGrid( Path[k] );

		GridWithPath.Add(GlobalGrid, k);

		for (int32 j = -Rad; j <= Rad; j++)
		{
			for (int32 i = -Rad; i <= Rad; i++)
			{
				FIntPoint Needed = FIntPoint(i,j) + GlobalGrid;
				GridNeeded.Add(Needed); // add all. even if out of chunk. filter later.
			}
		}
	} // init end.
	

	float DetailSpacing = VertexSpacing / DetailCount;
	float UVScale = VertexSpacing / TextureSize;
	UVScale /= DetailCount; // same size with original chunk.

	// Global FIntPoint, Index for Verts. float for heights.
	TMap<FIntPoint, TPair<int32, float>> DetailNeeded; 
	for (auto& GlobalGrid : GridNeeded)
	{
		// add vertices needed.
		for (int32 j = 0; j <= DetailCount; j++)
			for (int32 i = 0; i <= DetailCount; i++)
			{
				FIntPoint DetailGrid = FIntPoint(i, j);
				FIntPoint Offset = GlobalGrid * DetailCount;
				DetailNeeded.Add( DetailGrid + Offset, TPair<int32,float>( -1, INFLOAT ) ); 
				// global FIntPoint, but smaller vertexspacing. (DetailSpacing), Index = -1 by default.
			}

	}

	// sort by (X,Y) small to big
	auto Sorter = [](const FIntPoint& A, const FIntPoint& B) 
		{
			if (A.Y < B.Y) return true;
			else if (A.Y == B.Y)
			{
				return A.X < B.X;
			}
			else return false;
		};

	// sort it to get right index.
	DetailNeeded.KeySort(Sorter); 
	int32 Index = 0;

	// Vertex Generation.
	for (auto& Elem : DetailNeeded)
	{

		FIntPoint SGlobalGrid = Elem.Key;

		FIntPoint LocalGrid = SGlobalGrid - Chunk * (VerticesPerChunk - 1) * DetailCount;
		FVector3f Vertex = FVector3f(LocalGrid.X, LocalGrid.Y, 0.f) * DetailSpacing;		// local space for vertex.
		Vertex.Z = GetHeight( FVector2D(SGlobalGrid.X, SGlobalGrid.Y) * DetailSpacing );		// global space for height. 
		

		// --------------------Height Adjustment-------------------------- Start

		// find all grid in radius that has path.
		// get all indices in neighbor grids.
		// find closest road point
		// lerp


		// find all grid in radius that has path.
		TArray<FIntPoint> NeighborWithPath;
		for (int32 j = -Rad-1; j <= Rad+1; j++) 
		{
			for (int32 i = -Rad-1; i <= Rad+1; i++)
			{
				FIntPoint BigGlobalGrid; // Actual GlobalGrid(VertexSpacing) Grid that this DetailGrid is in.
				BigGlobalGrid.X = FMath::FloorToInt32(SGlobalGrid.X * DetailSpacing / VertexSpacing);
				BigGlobalGrid.Y = FMath::FloorToInt32(SGlobalGrid.Y * DetailSpacing / VertexSpacing);
				FIntPoint Neighbor = FIntPoint(i, j) + BigGlobalGrid;

				if (GridWithPath.Contains(Neighbor))	NeighborWithPath.Add(Neighbor);
			}
		}

		// get all indices in neighbor grids.
		TArray<int32> AllIndices;				
		for (auto& Neighbor : NeighborWithPath) 
		{
			TArray<int32> Indices;
			GridWithPath.MultiFind(Neighbor, Indices);
			if (!Indices.IsEmpty()) AllIndices.Append(Indices);
		}

		// find closest road point
		float Closest = INFLOAT;
		float Height = 0.f;
		for (auto& PathIndex : AllIndices)	 
		{
			FVector Point = Path[PathIndex];
			FVector2D GlobalVec = FVector2D(SGlobalGrid.X, SGlobalGrid.Y) * DetailSpacing;
			float Distance = FVector2D::DistSquared(FVector2D(Point.X, Point.Y), GlobalVec);

			if (Distance <= Closest) Closest = Distance, Height = Point.Z;
		}
		
		// lerp
		Closest = FMath::Sqrt(Closest);
		float RoadHeightDist = 1500.f;	 
		float OriginalHeightDist = 3000.f;

		float Alpha = 1.0f - (Closest - RoadHeightDist) / (OriginalHeightDist - RoadHeightDist);
		Alpha = FMath::Clamp(Alpha, 0.f, 1.f);
		Vertex.Z = FMath::Lerp(Vertex.Z, Height-10.f, Alpha);


		// --------------------Height Adjustment-------------------------- End
		

		// put Height Value into DetailNeeded
		TPair<int32, float>& IndexHeight = Elem.Value;
		IndexHeight.Value = Vertex.Z; 
		TSet<FIntPoint> ChunkSet;
		GetPossibleChunks(SGlobalGrid, DetailCount, ChunkSet);
		if ( ChunkSet.Contains( Chunk ) ) // if this is inside the chunk.
		{
			IndexHeight.Key = Index++;
			Vertices.Add(Vertex);		// add it to vertices.

			// don't know if this is right way, but let's just do it.
			FVector2DHalf UV;
			UV.X = SGlobalGrid.X * UVScale;
			UV.Y = SGlobalGrid.Y * UVScale;
			UVs.Add(UV);
		}
	
		// if points are on the edge of chunk, maybe we should update global save. (for continuous chunk)
	}


	// height smoothing
	TArray<float> Heights;
	int32 HRad = 2;
	Heights.SetNum(Vertices.Num());
	for (auto& Elem : DetailNeeded)
	{
		FIntPoint Grid = Elem.Key;
		TPair<int32, float> IndexHeight = Elem.Value;
		int32 IndexNow = IndexHeight.Key;
		if (IndexNow < 0) continue;

		TArray<float> TempHeights;
		// get all indices in HRad*2+1 box radius
		for(int32 j = -HRad; j<= HRad; j++)
			for (int32 i = -HRad; i <= HRad; i++)
			{
				FIntPoint Target = Grid + FIntPoint(i, j);
				TPair<int32, float>* Found = DetailNeeded.Find(Target);
				if (Found) TempHeights.Add( (*Found).Value );
			}

		// if can't get every grid, continue. (borders)
		if (TempHeights.Num() < FMath::Square(HRad * 2 + 1))
		{
			Heights[IndexNow] = Vertices[IndexNow].Z;
			continue;
		}

		
		float Sum = 0.f;
		for (auto& Temp : TempHeights) Sum += Temp;
		Heights[IndexNow] = Sum / TempHeights.Num();
	}

	// put smoothed heights into vertices
	for (int32 i = 0; i < Heights.Num(); i++) Vertices[i].Z = Heights[i];		
		

	// now we have index, make triangles.
	for (auto& Elem : DetailNeeded) 
	{
		FIntPoint GlobalGrid = Elem.Key;
		int32 Index0 = Elem.Value.Key;
		if (Index0 < 0) continue;

		TPair<int32, float> *Index1, *Index2, *Index3; // square.
		//	0	1
		//	2	3
		Index1 = DetailNeeded.Find(GlobalGrid + FIntPoint(1, 0));
		Index2 = DetailNeeded.Find(GlobalGrid + FIntPoint(0, 1));
		Index3 = DetailNeeded.Find(GlobalGrid + FIntPoint(1, 1));

		// if not in map.
		if (!Index1 || !Index2 || !Index3) continue; 

		int32 I1 = (*Index1).Key;
		int32 I2 = (*Index2).Key;
		int32 I3 = (*Index3).Key;

		// if not in chunk.
		if (I1 < 0 || I2 < 0 || I3 < 0) continue;

		// if all three are in the map.
		if (Index1 && Index2 && Index3) 
		{
			// CounterClockWise.
			Triangles.Add( Index0 );
			Triangles.Add( I2 );
			Triangles.Add( I1 );

			Triangles.Add( I2 );
			Triangles.Add( I3 );
			Triangles.Add( I1 );
		}
	}
	

	// getting normals.
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

void FChunkBuilder::AdjustHeight(const FIntPoint& Chunk, const TArray<FVector>& InPath, TArray<FVector3f>& Vertices, const int32& Rad)
{
	TSet<FIntPoint> GridSet;
	TSet<FIntPoint> VertexSet;

	for (auto& Elem : InPath)
	{
		FIntPoint Pivot = GetGlobalGrid(Elem);
		for (int32 j = -Rad; j <= Rad; j++)
			for (int32 i = -Rad; i <= Rad; i++)
				GridSet.Add(Pivot + FIntPoint(i, j));
	}


	for (auto& Elem : GridSet)
	{
		// 0 1
		// 2 3
		FIntPoint Local = Elem - Chunk * (VerticesPerChunk - 1);
		VertexSet.Add(Local);
		VertexSet.Add(Local + FIntPoint(1, 0)); // 1
		VertexSet.Add(Local + FIntPoint(0, 1)); // 2
		VertexSet.Add(Local + FIntPoint(1, 1)); // 3
	}

	// add globally saved vertices.
	TArray<FIntPoint> SavedVertices;
	VertexLowerNeeded.MultiFind(Chunk, SavedVertices);
	for (auto& Vert : SavedVertices) VertexSet.Add(Vert);

	// filter vertices on edge of coverage.
	TSet<FIntPoint> ToRemove;
	for (auto& Elem : VertexSet)
	{
		FIntPoint Global = Elem + Chunk * (VerticesPerChunk - 1);
		if (	!GridSet.Contains(Global)
			||	!GridSet.Contains(Global + FIntPoint(-1, 0))
			||	!GridSet.Contains(Global + FIntPoint(0, -1))
			||	!GridSet.Contains(Global + FIntPoint(-1, -1)))
		{
			ToRemove.Add(Elem);
		}
	}
	// filter.
	for (auto& Remover : ToRemove) VertexSet.Remove(Remover); 


	for (auto& Elem : VertexSet)
	{
		if (IsIndexInChunk(Elem))
		{
			int32 Index = Elem.X + Elem.Y * VerticesPerChunk;
			Vertices[Index].Z -= 1000.f; // minus ten meters.
		}
		else // save it in global var.
		{
			FIntPoint Global = Elem + Chunk * (VerticesPerChunk - 1);
			TSet<FIntPoint> OtherChunks;
			GetPossibleChunks(Global, 1, OtherChunks); // detailcount = 1. it is just global grid.
			
			// add to global save. (except self)
			OtherChunks.Remove(Chunk); 
			for (auto& OtherChunk : OtherChunks)
			{
				FIntPoint Local = Global - OtherChunk * (VerticesPerChunk - 1);
				VertexLowerNeeded.Add(OtherChunk, Local);
			}
		}
	}

	// we used the global save, now we remove it.
	VertexLowerNeeded.Remove(Chunk);

}

void FChunkBuilder::AdjustHeight(const FIntPoint& Chunk, TArray<FVector3f>& Vertices)
{

	TArray<FIntPoint> SavedVertices;
	VertexLowerNeeded.MultiFind(Chunk, SavedVertices);
	if (SavedVertices.IsEmpty()) return;

	TSet<FIntPoint> VertexSet;
	for (auto& Elem : SavedVertices) VertexSet.Add(Elem);

	int32 Counter = 0;
	for (auto& Elem : VertexSet)
		if (IsIndexInChunk(Elem))
		{
			Vertices[GetIndex(Elem)].Z -= 1000.f;
			Counter++;
		}
			
	UE_LOG( LogTemp, Warning, TEXT("Chunk %s, Counter %d"), *Chunk.ToString(), Counter );

	VertexLowerNeeded.Remove(Chunk);
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

FIntPoint FChunkBuilder::GetChunk(const FVector2D& GlobalVector)
{
	FIntPoint Chunk;
	Chunk.X = FMath::FloorToInt32(GlobalVector.X / ChunkLength);
	Chunk.Y = FMath::FloorToInt32(GlobalVector.Y / ChunkLength);
	return Chunk;
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

void FChunkBuilder::GetPossibleChunks(const FIntPoint& GlobalSmallGrid, const int32& DetailCount, TSet<FIntPoint>& OutChunks)
{

	OutChunks.Empty();
	FVector2D ActualPos = GlobalSmallGrid * (VertexSpacing / DetailCount);

	FIntPoint Chunk = GetChunk(ActualPos);
	OutChunks.Add(Chunk);

	float ModX = FMath::Fmod(ActualPos.X, ChunkLength);
	float ModY = FMath::Fmod(ActualPos.Y, ChunkLength);
	float SmallValue = 0.1f;

	bool OnBoundaryX = (ModX <= SmallValue) || (ChunkLength - ModX <= SmallValue);
	bool OnBoundaryY = (ModY <= SmallValue) || (ChunkLength - ModY <= SmallValue);

	if (OnBoundaryX)				OutChunks.Add(Chunk + FIntPoint(-1, 0));
	if (OnBoundaryY)				OutChunks.Add(Chunk + FIntPoint(0, -1));
	if (OnBoundaryX && OnBoundaryY) OutChunks.Add(Chunk + FIntPoint(-1, -1));

	return;
}

bool FChunkBuilder::IsOnBoundary(const FIntPoint& GlobalSmallGrid, const int32& DetailCount)
{
	TSet<FIntPoint> Chunks;
	GetPossibleChunks(GlobalSmallGrid, DetailCount, Chunks);
	if (Chunks.Num() >= 2) return true;
	else return false;
}

