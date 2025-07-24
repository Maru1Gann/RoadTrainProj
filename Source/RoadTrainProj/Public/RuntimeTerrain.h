
#pragma once

#include "CoreMinimal.h"                // always needed
#include "GameFramework/Actor.h"        // Inherits Actor 

#include "RealtimeMeshSimple.h"         // RealtimeMesh namespace
#include "RealtimeMeshActor.h"          // AReltimeMeshActor
#include "Mesh/RealtimeMeshAlgo.h"      // RealtimeMeshAlgo


#include "RuntimeTerrain.generated.h"   // must be last


UCLASS()
class ROADTRAINPROJ_API ARuntimeTerrain : public AActor
{
    GENERATED_BODY()

public:
    ARuntimeTerrain();
    virtual void OnConstruction(const FTransform& Transform) override;

    UPROPERTY( EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 1, ClampMin = "10.0", Step = "10.0", Units = "cm") )
	    float VertexSpacing = 1000.0f;
	UPROPERTY( EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 2, ClampMin = "2", Step = "2") )
	    int32 VerticesPerChunk = 128;   // at least two.
	UPROPERTY( EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 3, ClampMin = "0", Step = "1") )
	    int32 ChunkRadius = 1;
	UPROPERTY( EditAnywhere, Category = "Chunks", meta = (DisplayPriority = 4, ClampMin = "0.0", Step = "10.0") )
	    float TextureSize = 300.0f;

	UPROPERTY( EditAnywhere, Category = "Chunks|Height", meta = (DisplayPriority = 1) )
	    bool ShouldGenerateHeight = true;
    UPROPERTY( EditAnywhere, Category = "Chunks|Height", meta = (DisplayPriority = 2) )
	    TArray<struct FPerlinNoiseVariables> NoiseLayers;

    UPROPERTY( EditAnywhere, Category = "Chunks|Material")
	    UMaterialInterface* ChunkMaterial;
    

    UFUNCTION( CallInEditor, Category = "Chunks" )
        void GenerateLandscape();
    UFUNCTION( CallInEditor, Category = "Chunks" )
        void RemoveLandscape();

private:
	// for debugging & reusing purposes, shown on editor details pannel
	UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 5) )
	    int32 ChunkCount = 0;
	UPROPERTY( VisibleAnywhere, Category = "Chunks", meta = (DisplayPriority = 6) )
	    float ChunkLength;



    
	
    TArray<FIntPoint> ChunkOrder;
    // TODO: maybe FCriticalSection Mutex
    TMap<FIntPoint, ARealtimeMeshActor*> Chunks;    // Store all the chunks HERE


    void GetStreamSet( const FIntPoint& Chunk, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet );
    void AddChunk( const FIntPoint& Chunk, const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet );
    void RemoveChunk( const FIntPoint& Chunk );


    // ----------------tools----------------------
    void GetChunkOrder( const int32& ChunkRadius, TArray<FIntPoint>& OutArray );
    FVector2D GetPlayerLocation();
    FIntPoint GetChunk( const FVector2D& Location );
    FVector ConvertTo3D( const FVector2D& Location );
    float GetHeight( const FVector2D& Location );

    // tools_GetStreamset Parts
    void GetVertices( const FIntPoint& Chunk, const int32& StartIndex, const int32& EndIndex, const int32& VertexSpace, TArray<FVector3f>& OutVertices );
    void GetUVs( const FIntPoint& Chunk, const int32& StartIndex, const int32& EndIndex, const float& UVscale, TArray<FVector2DHalf>& OutUVs );
    void GetTriangles( const int32& VertexCount, TArray<uint32>& OutTriangles );
    void GetTangents(  const int32& VertexCount, const TArray<uint32>& BigTriangles, const TArray<FVector3f>& BigVertices, 
                            TArray<FVector3f>& OutTangents, TArray<FVector3f>& OutNormals );
    // tools_GetStreamset Parts

};