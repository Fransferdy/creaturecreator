// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "Components/ActorComponent.h"
#include "DesktopPlatform/Public/IDesktopPlatform.h"
#include "DesktopPlatform/Public/DesktopPlatformModule.h"
#include "RuntimeSkeletalMeshGenerator/RuntimeSkeletalMeshGenerator.h"
#include "SkeletalMeshLoader.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class CREATURECREATOR_API USkeletalMeshLoader : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	USkeletalMeshLoader();

	UFUNCTION(BlueprintCallable, Category = "Assimp")
	USkeletalMesh* OpenFile(USkeleton* BaseSkeleton, TArray<UMaterialInterface*> mats);

	UFUNCTION(BlueprintCallable, Category = "Assimp")
	USkeletalMesh* GenerateBlob(USkeleton* BaseSkeleton, TArray<UMaterialInterface*> mats);
	
	 
protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

		
};
