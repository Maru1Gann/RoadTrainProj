#pragma

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "RealtimeMeshSimple.h"         // RealtimeMesh namespace
#include "RealtimeMeshActor.h"          // AReltimeMeshActor
#include "Mesh/RealtimeMeshAlgo.h"      // RealtimeMeshAlgo
#include "Containers/Map.h"             // MultiMap

#include "PathFinder.h"
#include "ChunkBuilder.h"

#include "LandscapeManager.generated.h"


struct FPerlinNoiseVariables;
class USplineComponent;
struct FChunkData;

UCLASS()
class ROADTRAINPROJ_API ALandscapeManager : public AActor
{
    GENERATED_BODY()

public:
    ALandscapeManager();
    
    virtual void OnConstruction(const FTransform &Transform) override;
	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
    // --------------Vars for Terrain Building------------------
    UPROPERTY( EditAnywhere, Category = "Terrain", meta = (DisplayPriority = 1, ClampMin = "1000.0", Step = "1000.0", Units = "cm") )
	    float VertexSpacing = 1000.0f;
	UPROPERTY( EditAnywhere, Category = "Terrain", meta = (DisplayPriority = 2, ClampMin = "2", Step = "2") )
	    int32 VerticesPerChunk = 128;
	UPROPERTY( EditAnywhere, Category = "Terrain", meta = (DisplayPriority = 3, ClampMin = "0", Step = "1") )
	    int32 ChunkRadius = 2;
	UPROPERTY( EditAnywhere, Category = "Terrain", meta = (DisplayPriority = 4, ClampMin = "0.0", Step = "10.0") )
	    float TextureSize = 300.0f;
    UPROPERTY(EditAnywhere, Category = "Terrain|Coverage", meta = (DisplayPriority = 1, ClampMin = "0", Step = "1"))
        int32 CoverageRadius;   // cover radius of detailed layer.
    UPROPERTY( EditAnywhere, Category = "Terrain|Coverage", meta = (DisplayPriority = 2, ClampMin = "0", Step = "1"))
        int32 DetailCount;  // num of mesh squares in one side of detail layer. should divide VertexSpacing right.
    UPROPERTY( EditAnywhere, Category = "Terrain|Height", meta = (DisplayPriority = 1) )
	    bool ShouldGenerateHeight = true;
    UPROPERTY( EditAnywhere, Category = "Terrain|Height", meta = (DisplayPriority = 2) )
	    TArray<FPerlinNoiseVariables> NoiseLayers;

    UPROPERTY( EditAnywhere, Category = "Terrain|Material", meta = (DisplayPriority = 1) )
        UMaterialInterface* Material;

    UPROPERTY(EditAnywhere, Category = "Terrain|Async", meta = (DisplayPriority = 0))
        bool UseAsync;

    // Wait how many frames before updating chunks after playerlocated chunk changed.
    UPROPERTY( EditAnywhere, Category = "Terrain|Async", meta = (DisplayPriority = 1) )
        int32 UpdateDelayFrames;


    // ----------------Vars for PathFinding--------------------
    UPROPERTY( EditAnywhere, Category = "Path", meta = (DisplayPriority = 1) )
        FIntPoint Start;    // global FIntPoint
    UPROPERTY( EditAnywhere, Category = "Path", meta = (DisplayPriority = 2) )
        FIntPoint End;      // global
    UPROPERTY( EditAnywhere, Category = "Path", meta = (DisplayPriority = 3, ClampMin = "0.0", ClampMax = "100.0") )
	    float MaxSlope = 30;
    UPROPERTY(EditAnywhere, Category = "Path", meta = (DisplayPriority = 4, ClampMin = "0.0", ClampMax = "180.0") )
        float MaxRoadAngle = 15;
    UPROPERTY(EditAnywhere, Category = "Path", meta = (DisplayPriority = 5, ClampMin = "0"))
        int32 CounterHardLock = 1000;
    UPROPERTY(EditAnywhere, Category = "Path", meta = (DisplayPriority = 6))
        bool DrawPathDebug = false;
    UPROPERTY( EditAnywhere, Category = "Path|Mesh")
        UStaticMesh* RoadMesh;

    UFUNCTION(CallInEditor, Category = "Terrain")
        void GenerateLandscape();
    UFUNCTION(CallInEditor, Category = "Terrain")
        void GenerateLandscapeWithPath();
    UFUNCTION(CallInEditor, Category = "Terrain")
        void RemoveLandscape();
    UFUNCTION(CallInEditor, Category = "Terrain")
        void Debug();


    void AddChunk(const FIntPoint& Chunk, const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet);
    bool DestroyChunk(const FIntPoint& Chunk);

    // tools
    float GetHeight(const FVector2D& Location);
    FVector GridToVector(const FIntPoint& GlobalGrid);
    FIntPoint GetChunk(const FIntPoint& GlobalGrid);
    FIntPoint GetChunk(const FVector& Vector);

private:

    // ก้ use it only on game thread

    bool IsPath;
    FIntPoint LastLocation;
    bool LastLocChanged;
    int32 FrameCounter;

    TArray<FGate> GatePath;
    TMap<FIntPoint, TPair<FGate, FGate>> GateMap;

    // ก่ game thread only


    // ก้ background thread produces.
    TQueue<FChunkData, EQueueMode::Mpsc>  ChunkQueue;
    // ก่ background thread produces.
    TQueue<FIntPoint>   ChunkRemovalQueue;
    

    std::unique_ptr<FChunkBuilder> ChunkBuilder;

    TMap<FIntPoint, ARealtimeMeshActor*> Chunks;
    TMap<FIntPoint, USplineComponent*> Splines;
    float ChunkLength;

    std::unique_ptr<FPathFinder> PathFinder;

    TArray<FIntPoint> ChunkOrder;
    TArray<FIntPoint> BigChunkOrder;

    // tools
    void GetChunkOrder(const int32& ChunkRad, TArray<FIntPoint>& OutArray);

    USplineComponent* AddPathSpline(const FIntPoint& Chunk, const TArray<FVector>& Path);
    void MakeRoad(USplineComponent* Spline);

    FVector GetPlayerLocation();
    void UpdateGateMap();

   
    // async related below

    void ProcessQueue();

    void AsyncWork();

    void UpdateDataQueue(const TArray<FIntPoint> ChunksNeeded, const TMap<FIntPoint, TPair<FGate, FGate>> NearGatesMap);
    FChunkData MakeChunkData(const FIntPoint TargetChunk, const TArray< TPair<FGate, FGate> > NearGates);

    // game thread
    void FindNearGates(const FIntPoint& ChunkNow, TMap<FIntPoint, TPair<FGate, FGate>>& OutGatesMap);
    void FindChunksToRemove(const FIntPoint& ChunkNow, TSet<FIntPoint>& ChunksToRemove);
    void FindChunksToMake(const FIntPoint& ChunkNow, TArray<FIntPoint>& ChunksNeeded);
    // game thread

    bool ShouldDoWork();

};



struct FChunkData // dataset for queue.
{
    FChunkData() {};
    FChunkData(const FIntPoint& Chunk, const RealtimeMesh::FRealtimeMeshStreamSet& StreamSet, const TArray<FVector>& ActualPath) :
        Chunk(Chunk), StreamSet(StreamSet), ActualPath(ActualPath) {};
    
    FIntPoint Chunk;
    RealtimeMesh::FRealtimeMeshStreamSet StreamSet;
    TArray<FVector> ActualPath;
};