// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"


class FPathFinder : public FRunnable
{
	FMyThread() {
		Thread = FRunnableThread::Create(this, TEXT("MyThread"));
	};
public:
	FPathFinder();
	~FPathFinder();
};
