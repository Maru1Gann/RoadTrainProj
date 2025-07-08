
#pragma once

#include "CoreMinimal.h"
#include "PerlinNoiseVariables.generated.h"


USTRUCT(Atomic)
struct FPerlinNoiseVariables
{
	GENERATED_BODY()

	FPerlinNoiseVariables(
		const float& Frequency = 0, 
		const float& Amplitude = 0, 
		const float& Offset = 0.1f
	) : Frequency(Frequency), Amplitude(Amplitude), Offset(Offset)
	{};

	UPROPERTY( EditAnywhere, meta = (ClampMin = "0.0", ClampMax="100000.0", Step = "1.0", Units="m") )
	float Frequency;
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", ClampMax = "10000.0", Step = "1.0", Units="m") )
	float Amplitude; // max wave amplitude

	UPROPERTY(EditAnywhere)
	float Offset; // seed value

};
