// Fill out your copyright notice in the Description page of Project Settings.


#include "SplinePath.h"

// Sets default values
ASplinePath::ASplinePath()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Root 설정
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

	// Spline Component 생성 및 Root에 부착
	Spline = CreateDefaultSubobject<USplineComponent>(TEXT("SplineComponent"));
	Spline->SetupAttachment(RootComponent);

	DistanceTraveled = 0.f;
}

// Called when the game starts or when spawned
void ASplinePath::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ASplinePath::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Spline || !TargetActor) return;

	DistanceTraveled += MovementSpeed * DeltaTime;

	float SplineLength = Spline->GetSplineLength();

	if (DistanceTraveled > SplineLength)
	{
		DistanceTraveled = SplineLength;
	}

	FVector NewLocation = Spline->GetLocationAtDistanceAlongSpline(DistanceTraveled, ESplineCoordinateSpace::World);
	FRotator NewRotation = Spline->GetRotationAtDistanceAlongSpline(DistanceTraveled, ESplineCoordinateSpace::World);

	TargetActor->SetActorLocationAndRotation(NewLocation, NewRotation);
}

