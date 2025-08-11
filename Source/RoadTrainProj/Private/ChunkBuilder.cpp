
#include "ChunkBuilder.h"
#include "PerlinNoiseVariables.h"   // NoiseLayers

#include "PathFinder.h" // path finding

#include "Containers/Map.h" // MultiMap

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

	ApplyModified(Chunk, 
		Vertices, Triangles); // this is previously modified ones by other chunks. ( continuous chunk thingy )


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

	HeightModified.Remove(Chunk);
	TriangleModified.Remove(Chunk);

	return;

}

// returns streamset for chunk. height adjustment for road included.
void FChunkBuilder::GetStreamSet(const FIntPoint& Chunk, const TArray<FIntPoint>& Path, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet)
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


	FlattenPath(Chunk, Path,
		Vertices); // Flattening path. Overloaded function for this.
	AdjustTriangles(Chunk, VerticesPerChunk, Path, 
		Triangles);

	ApplyModified(Chunk,
		Vertices, Triangles); // this is previously modified ones by other chunks. ( continuous chunk thingy )

	Tangents.SetNum(Vertices.Num());
	Normals.SetNum(Vertices.Num());

	// Big data for uv and normal continue on different chunks
	TArray<FVector3f> BigVertices;
	TArray<uint32> BigTriangles;

	GetBigVertices(Chunk, Vertices, 
		BigVertices);
	GetTriangles(VerticesPerChunk+2, 
		BigTriangles);

	// Normals and Tangents made out of Big datas.
	GetTangents(VerticesPerChunk, BigTriangles, BigVertices,
		Tangents, Normals);


	// Datas into StreamSet( class Member Var ) RMC syntax
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

// flattens landscape for road
void FChunkBuilder::FlattenPath(const FIntPoint& Chunk, const TArray<FIntPoint>& Path, TArray<FVector3f>& OutVertices)
{
	// make map for changed heights.
	TMap<FIntPoint, float> HeightReserved;
	for (auto& Elem : Path)
	{ 
		float Height = GetHeight(PosToVector2D(Chunk, Elem));
		HeightReserved.Add(Elem, Height); //occupy path.
	}

	// do for all path. (except last)
	for (int i = 0; i < Path.Num() - 1; i++)
	{
		FIntPoint PosNow, PosNext;
		PosNow = Path[i];
		PosNext = Path[i + 1];
		float HeightNow = GetHeight(PosToVector2D(Chunk, PosNow));
		float HeightNext = GetHeight(PosToVector2D(Chunk, PosNext));
		//float HeightNow = OutVertices[GetIndex(PosNow)].Z;
		//float HeightNext = OutVertices[GetIndex(PosNext)].Z;

		// flattening algorithm.
		FIntPoint Case = PosNext - PosNow;
		if (Case == FIntPoint(0, 1) || Case == FIntPoint(0, -1)) // vertical straight line.
		{
			SetHeight(Chunk, PosNow + FIntPoint(1, 0), HeightNow, HeightReserved);
			SetHeight(Chunk, PosNow + FIntPoint(-1, 0), HeightNow, HeightReserved);
		}
		else if (Case == FIntPoint(1, 0) || Case == FIntPoint(-1, 0)) // horizontal straight line.
		{
			SetHeight(Chunk, PosNow + FIntPoint(0, 1), HeightNow, HeightReserved);
			SetHeight(Chunk, PosNow + FIntPoint(0, -1), HeightNow, HeightReserved);
		}
		else if (Case == FIntPoint(-1, 1) || Case == FIntPoint(1, -1)) // diagonal 1 ¢Ø
		{
			if (Case == FIntPoint(1, -1))
			{
				SetHeight(Chunk, PosNow + FIntPoint(0, 1), HeightNow, HeightReserved);
				SetHeight(Chunk, PosNow + FIntPoint(-1, 0), HeightNow, HeightReserved);

				Swap<FIntPoint>(PosNext, PosNow);
				Swap<float>(HeightNext, HeightNow);
			}
			else
			{
				SetHeight(Chunk, PosNow + FIntPoint(0, -1), HeightNow, HeightReserved);
				SetHeight(Chunk, PosNow + FIntPoint(1, 0), HeightNow, HeightReserved);
			}

			SetHeight(Chunk, PosNow + FIntPoint(-1, 0), (HeightNext + HeightNow) / 2, HeightReserved);
			SetHeight(Chunk, PosNow + FIntPoint(0, 1), (HeightNext + HeightNow) / 2, HeightReserved);
		}
		else if (Case == FIntPoint(1, 1) || Case == FIntPoint(-1, -1)) // diagonal 2 ¢Ö
		{

			if (Case == FIntPoint(-1, -1))
			{
				SetHeight(Chunk, PosNow + FIntPoint(0, 1), HeightNow, HeightReserved);
				SetHeight(Chunk, PosNow + FIntPoint(1, 0), HeightNow, HeightReserved);
				Swap<FIntPoint>(PosNext, PosNow);
				Swap<float>(HeightNext, HeightNow);
			}
			else 
			{
				SetHeight(Chunk, PosNow + FIntPoint(0, -1), HeightNow, HeightReserved);
				SetHeight(Chunk, PosNow + FIntPoint(-1, 0), HeightNow, HeightReserved);
			}

			SetHeight(Chunk, PosNow + FIntPoint(1, 0), (HeightNext + HeightNow) / 2, HeightReserved);
			SetHeight(Chunk, PosNow + FIntPoint(0, 1), (HeightNext + HeightNow) / 2, HeightReserved);
		}

		for (auto& Elem : HeightReserved)
		{ 
			OutVertices[GetIndex(Elem.Key)].Z = Elem.Value; 
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


// Adjust triangle to fit Path.
void FChunkBuilder::AdjustTriangles(const FIntPoint& Chunk, const int32& VertexCount, const TArray<FIntPoint>& Path, TArray<uint32>& OutTriangles)
{
	// numbers of elements of Triangles == (VertexCount - 1) ^ 2 * 6
	int32 RowNum = (VertexCount - 1);

	for (int32 i = 0; i < Path.Num() - 1; i++)
	{

		FIntPoint PosNow, PosNext;
		PosNow = Path[i];
		PosNext = Path[i + 1];
		FIntPoint Case = PosNext - PosNow;

		if (Case == FIntPoint(1, 1) || Case == FIntPoint(-1, -1)) // adjust triangle for diagonal road.
		{
			TArray<FIntPoint> Targets;
			Targets.Add(PosNow + FIntPoint(0, 1));
			Targets.Add(PosNow + FIntPoint(1, 0) );
			Targets.Add(PosNow + FIntPoint(-1, 0) );
			Targets.Add(PosNow + FIntPoint(0, -1) );
			
			Targets.Add(PosNow + FIntPoint(-1, 1));
			Targets.Add(PosNow + FIntPoint(-2, 0));
			Targets.Add(PosNow + FIntPoint(0, -2));
			Targets.Add(PosNow + FIntPoint(1, -1));

			Targets.Add(PosNow + FIntPoint(2, 0));

			for (auto& Target : Targets)
			{
				if (IsIndexInChunk(RowNum, Target))
				{
					int32 Index = GetIndex(RowNum, Target);
					Index *= 6;
					int32 CurrentVertex = GetIndex(Target);
					MakeSquare(Index, CurrentVertex, VertexCount, OutTriangles);
				}
				else // continuous chunk preparation.
				{
					TArray<TPair<FIntPoint, FIntPoint>> OtherChunkPos;
					GetOtherPos(Chunk, Target, OtherChunkPos);
					for (auto& Elem : OtherChunkPos)
					{
						if (IsIndexInChunk(RowNum, Elem.Value)) // check if valid for triangle. (for the chunk)
						{ TriangleModified.Add(Elem.Key, Elem.Value); }
					}
				}

			}
		} // end of if case.
	} // end of for path.

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

// applies modified values for continuous chunk. APPLY BEFORE OTHER MODIFICATIONS!!!
void FChunkBuilder::ApplyModified(const FIntPoint& Chunk, TArray<FVector3f>& OutVertices, TArray<uint32>& OutTriangles)
{
	// modified
	TArray<TPair<FIntPoint, float>> Heights;
	TArray<FIntPoint> Triangles;

	HeightModified.MultiFind(Chunk, Heights);
	TriangleModified.MultiFind(Chunk, Triangles);

	int32 RowNum = VerticesPerChunk - 1; // Row Num for Triangle Array.

	for (auto& Elem : Heights)
	{
		int32 Index = GetIndex(Elem.Key);
		OutVertices[Index].Z = Elem.Value;
	}
	for (auto& Pos : Triangles)
	{
		if (IsIndexInChunk(RowNum, Pos))
		{
			int32 Index = GetIndex(RowNum, Pos);
			Index *= 6;
			int32 CurrentVertex = GetIndex(Pos);
			MakeSquare(Index, CurrentVertex, VerticesPerChunk, OutTriangles);
		}
	}

}


int32 FChunkBuilder::GetIndex(const int32& VertexCount, const FIntPoint& Pos)
{
	return Pos.Y * VertexCount + Pos.X;
}

int32 FChunkBuilder::GetIndex(const FIntPoint& Pos)
{
	return GetIndex(VerticesPerChunk, Pos);
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
			{ OutVertices.Add(SmallVertices[Index]); }
			else									// if it's not in small vertices, make one.
			{
				FVector3f Vertex = FVector3f(j, i, 0.0f) * VertexSpacing;
				if (this->ShouldGenerateHeight)
				{ Vertex.Z = GetHeight(FVector2D(Vertex.X, Vertex.Y) + Offset); }
				OutVertices.Add(Vertex);

				// look for modifications
				TArray<TPair<FIntPoint, FIntPoint>> OtherPos;
				GetOtherPos(Chunk, FIntPoint(j, i), OtherPos);
				for (auto& Pos : OtherPos) // for possible pos for other chunks
				{
					TArray<TPair<FIntPoint, float>> PosHeights;
					HeightModified.MultiFind(Pos.Key, PosHeights); // find exisiting ones.

					TMap<FIntPoint, int32> PosMap;
					for (int32 ih = 0; ih < PosHeights.Num(); ih++)
					{ PosMap.Add(PosHeights[ih].Key, ih); } // Position and index

					int32* Modified = PosMap.Find(Pos.Value); // if same position exists
					if (Modified)
					{ Vertex.Z = PosHeights[*Modified].Value; } // change it to modified(preoccupied) value.
				}

			}
		}
	}

}

bool FChunkBuilder::IsIndexInChunk(const int32& VertexCount, const FIntPoint& Index)
{
	return Index.X >= 0 && Index.X < VertexCount && Index.Y >= 0 && Index.Y < VertexCount;
}

bool FChunkBuilder::IsIndexInChunk(const FIntPoint& Index)
{
	return IsIndexInChunk(VerticesPerChunk, Index);
}


// if the position can be converted into other chunks point, add it to outarray.
void FChunkBuilder::GetOtherPos(const FIntPoint& DefaultChunk, const FIntPoint& DefaultPos, TArray<TPair<FIntPoint, FIntPoint>>& OutOtherPos)
{
	OutOtherPos.Empty();

	for (int32 i = -1; i <= 1; i++)
	{
		for (int32 j = -1; j <= 1; j++)
		{
			if(j == 0 && i == 0) // skip self.
			{ continue; }

			FIntPoint TargetChunk = DefaultChunk + FIntPoint(j, i);
			TPair<FIntPoint, FIntPoint> Converted = ChangeChunkPos(DefaultChunk, DefaultPos, TargetChunk);
			if (IsIndexInChunk(Converted.Value))
			{
				OutOtherPos.Add(Converted);
			}

		} // for j
	} // for i

}


TPair<FIntPoint, FIntPoint> FChunkBuilder::ChangeChunkPos(const FIntPoint& DefaultChunk, const FIntPoint& DefaultPos, const FIntPoint& TargetChunk)
{
	FIntPoint Global = DefaultChunk * (VerticesPerChunk - 1) + DefaultPos;
	FIntPoint TargetLocal = Global - TargetChunk * (VerticesPerChunk - 1);
	return TPair<FIntPoint, FIntPoint>(TargetChunk, TargetLocal);
}


FVector2D FChunkBuilder::PosToVector2D(const FIntPoint& Chunk, const FIntPoint& Pos)
{
	FVector2D Offset = FVector2D(Chunk.X, Chunk.Y) * ChunkLength;
	FVector2D Offset2 = FVector2D(Pos.X, Pos.Y) * VertexSpacing;
	return Offset + Offset2;
}

// Checks if Pos is in chunk bound internally. macro for FlattenPath.
void FChunkBuilder::SetHeight(const FIntPoint& Chunk, const FIntPoint& Pos, const float& Height, TMap<FIntPoint, float>& HeightReserved)
{
	if ( HeightReserved.Contains(Pos) ) // if this position already occupied, return.
	{ return; }

	// preparation for continous chunk
	TArray<TPair<FIntPoint, FIntPoint>> OtherChunkPos;
	GetOtherPos(Chunk, Pos, OtherChunkPos);
	for (auto& Elem : OtherChunkPos)
	{
		TPair<FIntPoint, float> PosHeight(Elem.Value, Height);
		HeightModified.Add(Elem.Key, PosHeight);
	}

	if ( !IsIndexInChunk(Pos) )
	{ return; }


	HeightReserved.Add(Pos, Height);
}

