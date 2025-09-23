
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
void FChunkBuilder::GetStreamSet(const FIntPoint& Chunk, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet, int32 DetailCount)
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

	// Read Function Name. do it after normal calculation.
	ApplyVertexLowerNeeded(Chunk, Vertices);

	// Now add vertex coverage from global save.
	TMap<FIntPoint, float>* FoundMap = CoverVertices.Find(Chunk);
	int32 BaseIndex = Vertices.Num();
	if (FoundMap)
	{
		int32 Index = 0;
		float DetailSpacing = VertexSpacing / DetailCount;
		UVScale /= DetailCount;
		TArray<FVector3f> Vertices2, Tangents2, Normals2;
		TArray<uint32> Triangles2;
		TArray<FVector2DHalf> UVs2;

		TMap<FIntPoint, TPair<int32, float>> DetailNeeded;

		for (auto& Elem : *FoundMap)
		{
			FIntPoint SmallGrid = Elem.Key;
			float Height = Elem.Value;

			DetailNeeded.Add(SmallGrid, TPair<int32, float>(-1, Height));
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

		DetailNeeded.KeySort(Sorter);

		for (auto& Elem : DetailNeeded)
		{
			FIntPoint SmallGrid = Elem.Key;
			int32& IndexNow = Elem.Value.Key;
			float& Height = Elem.Value.Value;

			IndexNow = Index++;
			FIntPoint LocalGrid = SmallGrid - Chunk * (VerticesPerChunk - 1) * DetailCount;
			FVector3f Vertex = FVector3f(LocalGrid.X, LocalGrid.Y, 0.f) * DetailSpacing;
			Vertex.Z = Height;
			
			Vertices2.Add(Vertex);
			FVector2DHalf UV;
			UV.X = SmallGrid.X * UVScale;
			UV.Y = SmallGrid.Y * UVScale;
			UVs2.Add(UV);
		}

		for (auto& Elem : DetailNeeded)
		{
			FIntPoint SmallGrid = Elem.Key;
			int32 IndexNow = Elem.Value.Key;

			TPair<int32, float>* Find1, * Find2, * Find3;
			Find1 = DetailNeeded.Find(SmallGrid + FIntPoint(1, 0));
			Find2 = DetailNeeded.Find(SmallGrid + FIntPoint(0, 1));
			Find3 = DetailNeeded.Find(SmallGrid + FIntPoint(1, 1));
			// 0 1
			// 2 3
			int32 Index1, Index2, Index3;
			if (Find1 && Find2 && Find3)
			{
				Index1 = (*Find1).Key;
				Index2 = (*Find2).Key;
				Index3 = (*Find3).Key;

				Triangles2.Add(IndexNow);
				Triangles2.Add(Index2);
				Triangles2.Add(Index1);

				Triangles2.Add(Index2);
				Triangles2.Add(Index3);
				Triangles2.Add(Index1);
			}
		}

		Normals2.SetNum(Vertices2.Num());
		Tangents2.SetNum(Vertices2.Num());

		RealtimeMeshAlgo::GenerateTangents(
			TConstArrayView<uint32>(Triangles2),
			Vertices2,
			nullptr,
			[&Normals2, &Tangents2](int32 index, FVector3f Tangent, FVector3f Normal) -> void
			{
				Normals2[index] = Normal;
				Tangents2[index] = Tangent;
			},
			true
		);

		Vertices.Append(Vertices2);
		for (auto& Elem : Triangles2) Triangles.Add(Elem + BaseIndex);
		Normals.Append(Normals2);
		Tangents.Append(Tangents2);
		UVs.Append(UVs2);
		

		CoverVertices.Remove(Chunk);
	}
	


	// Datas into StreamSet
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

// DetailCount == how many squares will fit in one grid. (row)
// do this only if there's one path.
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
	TSet<FIntPoint> PathChunks;
	for (int32 i = 0; i < InPath.Num()-1; i++) // sample more points.
	{
		Path.Add(InPath[i]);
		Path.Add((InPath[i] + InPath[i + 1]) / 2);
		PathChunks.Add( GetChunk( FVector2D(InPath[i].X,InPath[i].Y) ) );
	}
	if (!InPath.IsEmpty()) Path.Add(InPath.Last());

	TSet<FIntPoint> NearChunks;
	for (int32 j = -1; j <= 1; j++)
		for (int32 i = -1; i <= 1; i++)
			NearChunks.Add(FIntPoint(i, j) + Chunk);

	NearChunks = NearChunks.Difference(PathChunks); // only neighbor chunks without paths.

	// Read Function Name.
	UpdateVertexLowerNeeded(Chunk, Path, CoverageRad);

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

	// get from globally saved vertices.
	TMap<FIntPoint, float>* FoundMap = CoverVertices.Find(Chunk);
	if (FoundMap)
	{
		for (auto& GridHeight : *FoundMap)
		{
			FIntPoint SmallGrid = GridHeight.Key;
			float SavedHeight = GridHeight.Value;
			TPair<int32, float>* FoundPair = DetailNeeded.Find(SmallGrid);
			if (!FoundPair) // only when SmallGrid does not exist.
			{
				DetailNeeded.Add(SmallGrid, TPair<int32, float>(-1, SavedHeight) );
			}	
		}

		// done using it. remove entry
		CoverVertices.Remove(Chunk);
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
		float Height = GetHeight(FVector2D(SGlobalGrid.X, SGlobalGrid.Y) * DetailSpacing); // global value for height.


		// --------------------Height Adjustment-------------------------- Start

		// find all grid in radius that has path.
		// get all indices in neighbor grids.
		// find closest road point
		// lerp


		// find all grid in radius that has path.
		TArray<FIntPoint> NeighborWithPath;
		for (int32 j = -Rad - 1; j <= Rad + 1; j++)
		{
			for (int32 i = -Rad - 1; i <= Rad + 1; i++)
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
		float RoadHeight = 0.f;
		for (auto& PathIndex : AllIndices)
		{
			FVector Point = Path[PathIndex];
			FVector2D GlobalVec = FVector2D(SGlobalGrid.X, SGlobalGrid.Y) * DetailSpacing;
			float Distance = FVector2D::DistSquared(FVector2D(Point.X, Point.Y), GlobalVec);

			if (Distance <= Closest) Closest = Distance, RoadHeight = Point.Z;
		}

		// lerp
		Closest = FMath::Sqrt(Closest);
		float RoadHeightDist = 1500.f;
		float OriginalHeightDist = 3000.f;

		float Alpha = 1.0f - (Closest - RoadHeightDist) / (OriginalHeightDist - RoadHeightDist);
		Alpha = FMath::Clamp(Alpha, 0.f, 1.f);
		Height = FMath::Lerp(Height, RoadHeight, Alpha);


		// --------------------Height Adjustment-------------------------- End


		// put Height Value into DetailNeeded
		TPair<int32, float>& IndexHeight = Elem.Value;
		IndexHeight.Value = Height;

		// figure out if it's okay to make this vertex.
		if ( IsGridInChunk(Chunk, SGlobalGrid, DetailCount) )
		{
			IndexHeight.Key = Index++;
			// set UVs.
			FVector2DHalf UV;
			UV.X = SGlobalGrid.X * UVScale;
			UV.Y = SGlobalGrid.Y * UVScale;
			UVs.Add(UV);
		}
	
	}


	// height smoothing
	TMap<FIntPoint, float> Heights;
	int32 HRad = 2;
	for (auto& Elem : DetailNeeded)
	{
		FIntPoint Grid = Elem.Key;
		int32 IndexNow = Elem.Value.Key;
		float Height = Elem.Value.Value; // we'll change this.

		TArray<float> TempHeights;
		// get all indices in HRad*2+1 box radius
		for(int32 j = -HRad; j<= HRad; j++)
			for (int32 i = -HRad; i <= HRad; i++)
			{
				FIntPoint Target = Grid + FIntPoint(i, j);
				TPair<int32, float>* Found = DetailNeeded.Find(Target);
				if (Found) TempHeights.Add( (*Found).Value );
			}

		// if can't get every grid in HRad, use original height (borders)
		if ( TempHeights.Num() >= FMath::Square(HRad * 2 + 1) )
		{
			float Sum = 0.f;
			for (auto& Temp : TempHeights) Sum += Temp;
			Height = Sum / TempHeights.Num();
		}

		Heights.Add(Grid, Height);
	}

	// Index = last one + 1 = Num.
	Vertices.SetNum(Index);
	for (auto& Elem : DetailNeeded)
	{
		FIntPoint Grid = Elem.Key;
		int32 IndexNow = Elem.Value.Key;
		
		float* FoundHeight = Heights.Find(Grid);
		if (!FoundHeight)
		{
			UE_LOG(LogTemp, Warning, TEXT("PathStreamSet Height Error!!!"));
			continue;
		}

		float Height = *FoundHeight;

		TSet<FIntPoint> OtherChunks;
		GetPossibleChunks(Grid, DetailCount, OtherChunks);
		OtherChunks.Remove(Chunk); // remove self.
		for (auto& OtherChunk : OtherChunks)
		{
			if (!NearChunks.Contains(OtherChunk)) continue;
			TMap<FIntPoint, float>* Finder = CoverVertices.Find(OtherChunk);
			if (Finder) (*Finder).Add(Grid, Height); // if found, just add.
			else // if not found, make one.
			{
				TMap<FIntPoint, float> TempMap;
				TempMap.Add(Grid, Height);
				CoverVertices.Add(OtherChunk, TempMap);
			}
		}



		FIntPoint LocalGrid = Grid - Chunk * (VerticesPerChunk - 1) * DetailCount;
		// local space for vertex.
		FVector3f Vertex = FVector3f(LocalGrid.X, LocalGrid.Y, 0.f) * DetailSpacing;
		Vertex.Z = (*FoundHeight);	// set height value.

		if (IndexNow >= 0) Vertices[IndexNow] = Vertex;	// apply it to Vertices.
	}
		

	// we have index, make triangles out of it.
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

void FChunkBuilder::UpdateVertexLowerNeeded(const FIntPoint& Chunk, const TArray<FVector>& InPath, const int32& Rad)
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
		VertexSet.Add(Elem);
		VertexSet.Add(Elem + FIntPoint(1, 0)); // 1
		VertexSet.Add(Elem + FIntPoint(0, 1)); // 2
		VertexSet.Add(Elem + FIntPoint(1, 1)); // 3
	}


	// filter vertices on edge of coverage.
	TSet<FIntPoint> ToRemove;
	for (auto& Elem : VertexSet)
	{
		FIntPoint Global = Elem;
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

	TSet<FIntPoint>* Finder = VertexLowerNeeded.Find(Chunk);
	if (Finder)
	{
		for (auto& Elem : *Finder) VertexSet.Add(Elem);
	}


	for (auto& Vertex : VertexSet)
	{
		TSet<FIntPoint> Chunks;
		GetPossibleChunks(Vertex, 1, Chunks);
		for (auto& NewChunk : Chunks)
		{
			Finder = nullptr;
			Finder = VertexLowerNeeded.Find(NewChunk);
			if (Finder) (*Finder).Add(Vertex);
			else
			{
				TSet<FIntPoint> TempVertSet;
				TempVertSet.Add(Vertex);
				VertexLowerNeeded.Add(NewChunk, TempVertSet);
			}
		}
	}

}

void FChunkBuilder::ApplyVertexLowerNeeded(const FIntPoint& Chunk, TArray<FVector3f>& Vertices)
{
	TSet<FIntPoint>* VertexSet = VertexLowerNeeded.Find(Chunk);
	if (!VertexSet) return;

	for(auto& Vertex : *VertexSet)
	{
		FIntPoint Local = Vertex - Chunk * (VerticesPerChunk - 1);
		if ( !IsIndexInChunk(Local) ) continue;
		int32 Index = GetIndex(Local);
		Vertices[Index].Z -= 1000.f;
	}

	VertexLowerNeeded.Remove(Chunk);
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

bool FChunkBuilder::IsGridInChunk(const FIntPoint& Chunk, const FIntPoint& GlobalSmallGrid, const int32& DetailCount)
{
	TSet<FIntPoint> Chunks;
	GetPossibleChunks(GlobalSmallGrid, DetailCount, Chunks);
	if (Chunks.Contains(Chunk)) return true;
	else return false;
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

