// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SplineComponent.h"

#include "SplinePath.generated.h"

UCLASS()
class ROADTRAINPROJ_API ASplinePath : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASplinePath();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	//Spline Component
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	USplineComponent* Spline;

	//이동할 엑터 (트럭)
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	AActor* TargetActor;

	// 속도
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float MovementSpeed = 300.f;

private:
	float DistanceTraveled;
};
