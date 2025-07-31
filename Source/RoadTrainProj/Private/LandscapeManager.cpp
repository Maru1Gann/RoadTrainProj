
#include "LandscapeManager.h"

#include "RuntimeTerrain.h"
#include "PerlinNoiseVariables.h"

ALandscapeManager::ALandscapeManager()
{
    PrimaryActorTick.bCanEverTick = true; // enable tick
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root")); // cannot see actor in editor bug fix

}

float ALandscapeManager::GetHeight( const FVector2D& Location )
{
    return Terrain_Large->GetHeight( Location );
}