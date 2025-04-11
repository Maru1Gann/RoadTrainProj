#pragma once


#include "CoreMinimal.h"

#include "ProceduralMeshComponent.h" // FProcMeshTangent

#include "ChunkInfoVariables.generated.h"


USTRUCT(Atomic)
struct FChunkInfoVariables
{
    GENERATED_BODY()

public:
    FChunkInfoVariables(){};

    FChunkInfoVariables(
        const TArray<FVector>& Vertices,
        const TArray<FVector>& Normals,
        const TArray<FVector2D>& UVs,
        const TArray<FProcMeshTangent> Tangents
        ) : Vertices(Vertices), Normals(Normals), UVs(UVs), Tangents(Tangents)
        {};

    TArray<FVector> Vertices;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FProcMeshTangent> Tangents;
};