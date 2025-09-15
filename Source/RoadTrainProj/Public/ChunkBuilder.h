
#pragma once

#include "CoreMinimal.h"                // always needed

#include "RealtimeMeshSimple.h"         // RealtimeMesh namespace
#include "Mesh/RealtimeMeshAlgo.h"      // RealtimeMeshAlgo

struct FPerlinNoiseVariables;
class ALandscapeManager;

class FChunkBuilder
{


public:
    FChunkBuilder( ALandscapeManager* pLM, UMaterialInterface* ChunkMaterial );

public:

    UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 1, ClampMin = "10.0", Step = "10.0", Units = "cm") )
	    float VertexSpacing = 1000.0f;
	UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 2, ClampMin = "2", Step = "2") )
	    int32 VerticesPerChunk = 128;   // at least two.
	UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 3, ClampMin = "0", Step = "1") )
	    int32 ChunkRadius = 1;
	UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 4, ClampMin = "0.0", Step = "10.0") )
	    float TextureSize = 300.0f;

	UPROPERTY( VisibleAnywhere, Category = "Chunks|Height", meta = (DisplayPriority = 1) )
	    bool ShouldGenerateHeight = true;
    UPROPERTY( VisibleAnywhere, Category = "Chunks|Height", meta = (DisplayPriority = 2) )
	    TArray<FPerlinNoiseVariables> NoiseLayers;

    UPROPERTY( VisibleAnywhere, Category = "Chunks|Material")
	    UMaterialInterface* ChunkMaterial;
    

    void GetStreamSet(const FIntPoint& Chunk, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet);
	void GetStreamSet(const FIntPoint& Chunk, const TArray<FVector>& InPath, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet);
	void GetPathStreamSet(const FIntPoint& Chunk, const TArray<FVector>& InPath, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet, const int32& DetailCount = 10);
    float GetHeight( const FVector2D& Location );

private:
	// for debugging & reusing purposes, shown on editor details pannel
	UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 5) )
	    int32 ChunkCount = 0;
	UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 6) )
	    float ChunkLength;



    // tools below.
    void GetVertices( const FIntPoint& Chunk, const int32& StartIndex, const int32& EndIndex, const int32& VertexSpace, TArray<FVector3f>& OutVertices );
	void AdjustHeight(const FIntPoint& Chunk, const TArray<FVector>& InPath, TArray<FVector3f>& OutVertices);
    void GetUVs( const FIntPoint& Chunk, const int32& StartIndex, const int32& EndIndex, const float& UVscale, TArray<FVector2DHalf>& OutUVs );
    void GetTriangles( const int32& VertexCount, TArray<uint32>& OutTriangles );
    void GetTangents(  const int32& VertexCount, const TArray<uint32>& BigTriangles, const TArray<FVector3f>& BigVertices, 
                            TArray<FVector3f>& OutTangents, TArray<FVector3f>& OutNormals );
	void GetBigVertices(const FIntPoint& Chunk, const TArray<FVector3f>& SmallVertices, TArray<FVector3f>& OutVertices);
	void MakeSquare(const int32& Index, const int32& CurrentVertex, const int32& VertexCount, TArray<uint32>& OutTriangles, bool Invert = true);
	
	int32 GetIndex(const int32& VertexCount, const FIntPoint& Pos);
	int32 GetIndex(const FIntPoint& Pos);
	FIntPoint GetChunk(const FIntPoint& GlobalGrid);
	FIntPoint GetGlobalGrid(const FVector& Vector);
	
	bool IsIndexInChunk(const int32& VertexCount, const FIntPoint& Index);
	bool IsIndexInChunk(const FIntPoint& Index);
	bool IsGridInChunk(const FIntPoint& Chunk, const FIntPoint& GlobalGrid);
};