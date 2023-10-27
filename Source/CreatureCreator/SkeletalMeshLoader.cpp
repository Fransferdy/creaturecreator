// Fill out your copyright notice in the Description page of Project Settings

#include "SkeletalMeshLoader.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include "Math/UnrealMathUtility.h"

#define DEGREE_TO_RADIAN 0.01745329252

//#include "RuntimeSkeletalMeshGenerator/RuntimeSkeletalMeshGenerator.h"

// Sets default values for this component's properties
USkeletalMeshLoader::USkeletalMeshLoader()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void USkeletalMeshLoader::BeginPlay()
{
	Super::BeginPlay();

	// ...
	
}


// Called every frame
void USkeletalMeshLoader::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

TArray<FVector> getCirclePoints(int32 parts, double z, double radius)
{
	TArray<FVector> res;
	UE_LOG(LogTemp, Warning, TEXT("GET CIRCLE POINTS2"));

	for (int32 part = 0; part < parts; part++)
	{
		double x, y;
		double degreeInRadian = (360.0 / (double)parts) * (double)part * DEGREE_TO_RADIAN;

		double sin, cos;
		FMath::SinCos(&sin, &cos, degreeInRadian);
		y = sin * radius;
		x = cos * radius;
		
		res.Add(FVector(y, z, x));
		UE_LOG(LogTemp, Warning, TEXT("Point: %d %f %f %f"), part, x, y, z);
	}

	return res;
}

FVector calculateFaceNormal(FVector &a, FVector& b, FVector& c)
{
	FVector BA(b - a);
	FVector CA(c - a);
	FVector FaceNormal(FVector::CrossProduct(BA, CA));
	FaceNormal = FaceNormal / FaceNormal.Size();
	return FaceNormal;
}
void calculateFaceNormalsTangentsBitTangents(FVector& a, FVector& b, FVector& c, FVector2D &uv1, FVector2D& uv2, FVector2D& uv3, FVector& normalOut, FVector& tangentOut)//, FVector& bitTangetOut)
{
	FVector edge1(b - a);
	FVector edge2(c - a);
	FVector2D deltaUV1(uv2 - uv1);
	FVector2D deltaUV2(uv3 - uv1);

	normalOut =(FVector::CrossProduct(edge1, edge2));
	normalOut = normalOut / normalOut.Size();

	double f = 1.0f / ((deltaUV1.X * deltaUV2.Y) - (deltaUV2.X * deltaUV1.Y));
	tangentOut = FVector(0, 0, 0);
	//bitTangetOut = FVector(0, 0, 0);

	tangentOut.X = f * ((deltaUV2.Y * edge1.X) - (deltaUV1.Y * edge2.X));
	tangentOut.Y = f * ((deltaUV2.Y * edge1.Y) - (deltaUV1.Y * edge2.Y));
	tangentOut.Z = f * ((deltaUV2.Y * edge1.Z) - (deltaUV1.Y * edge2.Z));

	//bitTangetOut.X = f * ((-deltaUV2.X * edge1.X) + (deltaUV1.X * edge2.X));
	//bitTangetOut.Y = f * ((-deltaUV2.X * edge1.Y) + (deltaUV1.X * edge2.Y));
	//bitTangetOut.Z = f * ((-deltaUV2.X * edge1.Z) + (deltaUV1.X * edge2.Z));
}

void USkeletalMeshLoader::GenerateMyBlob() {

	int32 zStep = 4;
	int32 zMax = 120;

	int32 circlePoints = 120;

	FMeshSurface mSurface;

	TArray<FVector> vertices;
	int32 circlePartsSize = 20;
	double circleRadius = 50;
	double tempRad = 0;

	int32 breakOff = 20;
	double growth = circleRadius / breakOff;
	for (int32 z = 0; z < zMax; z += zStep)
	{
		int32 zIndex = z / zStep;
		tempRad = circleRadius;
		if (z < breakOff)
			tempRad = z * growth;
		if (z > zMax - breakOff)
			tempRad = (zMax - z) * growth;
		if (z == zMax - zStep)
			tempRad = 0;
		vertices.Append(getCirclePoints(circlePartsSize, z, tempRad));
	}

	meshSurfaces.Empty();
	mSurface.Vertices.Empty();
	mSurface.Normals.Empty();
	mSurface.Tangents.Empty();
	mSurface.BoneInfluences.Empty();
	mSurface.Uvs.Empty();
	mSurface.Colors.Empty();
	mSurface.FlipBinormalSigns.Empty();

	mSurface.Vertices = vertices;
	int32 nVertices = mSurface.Vertices.Num();
	mSurface.Normals.SetNum(nVertices);
	mSurface.Tangents.SetNum(nVertices);
	mSurface.BoneInfluences.SetNum(nVertices);
	mSurface.Uvs.SetNum(nVertices);
	mSurface.Colors.SetNum(nVertices);
	mSurface.FlipBinormalSigns.SetNum(nVertices);

	for (int32 v = 0; v < nVertices; v++)
	{
		mSurface.Normals[v] = FVector(0, 0, 0);
		mSurface.Tangents[v] = FVector(0, 0, 0);
		mSurface.Colors[v] = FColor(0, 0, 0);
		mSurface.FlipBinormalSigns[v] = false;

		double uvx = ((mSurface.Vertices[v].Y + circleRadius) * 0.5) / circleRadius;
		double uvy = (mSurface.Vertices[v].Z) / zMax;
		TArray<FVector2D> UVs;
		UVs.Add(FVector2D(uvx, uvy));
		mSurface.Uvs[v] = UVs;
		UE_LOG(LogTemp, Warning, TEXT("Point: %d %f %f %f"), v, mSurface.Vertices[v].X, mSurface.Vertices[v].Y, mSurface.Vertices[v].Z);

		TArray<FRawBoneInfluence> boneInfluences;
		FRawBoneInfluence boneInf;

		boneInf.BoneIndex = 0;
		boneInf.VertexIndex = v;
		boneInf.Weight = 1;

		boneInfluences.Add(boneInf);
		mSurface.BoneInfluences[v] = boneInfluences;

	}
	mSurface.Indices.Empty();
	int32 maxConectableVertices = (nVertices - 1) - circlePartsSize;
	for (int32 v = 0; v < maxConectableVertices; v++)
	{
		int32 i1, i2, i3, i4;
		i1 = v;
		i2 = v + 1;
		i3 = i1 + circlePartsSize;
		i4 = i2 + circlePartsSize;

		mSurface.Indices.Add(i1);
		mSurface.Indices.Add(i3);
		mSurface.Indices.Add(i2);

		mSurface.Indices.Add(i3);
		mSurface.Indices.Add(i4);
		mSurface.Indices.Add(i2);

		//UE_LOG(LogTemp, Warning, TEXT("Points: %d %d %d"), i1, i2, i3);

		//Normals&Tangents==========
		FVector FaceNormal, FaceTangent, FaceBiTangent;
		calculateFaceNormalsTangentsBitTangents(mSurface.Vertices[i1], mSurface.Vertices[i2], mSurface.Vertices[i3],
			mSurface.Uvs[i1][0], mSurface.Uvs[i2][0], mSurface.Uvs[i3][0],
			FaceNormal, FaceTangent);// , FaceBiTangent);
		//FVector FaceNormal = calculateFaceNormal(mSurface.Vertices[i1], mSurface.Vertices[i2], mSurface.Vertices[i3]);

		mSurface.Normals[i1] += FaceNormal;
		mSurface.Normals[i2] += FaceNormal;
		mSurface.Normals[i3] += FaceNormal;

		mSurface.Tangents[i1] += FaceTangent;
		mSurface.Tangents[i2] += FaceTangent;
		mSurface.Tangents[i3] += FaceTangent;

		calculateFaceNormalsTangentsBitTangents(mSurface.Vertices[i3], mSurface.Vertices[i4], mSurface.Vertices[i2],
			mSurface.Uvs[i3][0], mSurface.Uvs[i4][0], mSurface.Uvs[i2][0],
			FaceNormal, FaceTangent);// , FaceBiTangent);
		//FaceNormal = calculateFaceNormal(mSurface.Vertices[i3], mSurface.Vertices[i4], mSurface.Vertices[i2]);

		mSurface.Normals[i3] += FaceNormal;
		mSurface.Normals[i4] += FaceNormal;
		mSurface.Normals[i2] += FaceNormal;

		mSurface.Tangents[i3] += FaceTangent;
		mSurface.Tangents[i4] += FaceTangent;
		mSurface.Tangents[i2] += FaceTangent;
		//Normals&Tangents==========
	}

	//Calculate Normals for edge cases=====
	int32 i1, i2, i3;
	i1 = circlePartsSize - 1;
	i2 = 0;
	i3 = 0 + circlePartsSize;

	mSurface.Indices.Add(i1);
	mSurface.Indices.Add(i3);
	mSurface.Indices.Add(i2);

	FVector FaceNormal, FaceTangent;
	calculateFaceNormalsTangentsBitTangents(mSurface.Vertices[i1], mSurface.Vertices[i2], mSurface.Vertices[i3],
		mSurface.Uvs[i1][0], mSurface.Uvs[i2][0], mSurface.Uvs[i3][0],
		FaceNormal, FaceTangent);

	mSurface.Normals[i1] += FaceNormal;
	mSurface.Normals[i2] += FaceNormal;
	mSurface.Normals[i3] += FaceNormal;

	mSurface.Tangents[i1] += FaceTangent;
	mSurface.Tangents[i2] += FaceTangent;
	mSurface.Tangents[i3] += FaceTangent;

	int32 last = mSurface.Vertices.Num() - 1;
	i1 = last - circlePartsSize + 1;
	i2 = last - circlePartsSize;
	i3 = last;

	mSurface.Indices.Add(i1);
	mSurface.Indices.Add(i3);
	mSurface.Indices.Add(i2);

	calculateFaceNormalsTangentsBitTangents(mSurface.Vertices[i1], mSurface.Vertices[i2], mSurface.Vertices[i3],
		mSurface.Uvs[i1][0], mSurface.Uvs[i2][0], mSurface.Uvs[i3][0],
		FaceNormal, FaceTangent);

	mSurface.Normals[i1] += FaceNormal;
	mSurface.Normals[i2] += FaceNormal;
	mSurface.Normals[i3] += FaceNormal;

	mSurface.Tangents[i1] += FaceTangent;
	mSurface.Tangents[i2] += FaceTangent;
	mSurface.Tangents[i3] += FaceTangent;
	//Calculate Normals for edge cases=====

	//Normalize Normals/Tangents
	for (size_t v = 0; v < nVertices; ++v)
	{
		mSurface.Normals[v] = (mSurface.Normals[v] / mSurface.Normals[v].Size());
		mSurface.Tangents[v] = (mSurface.Tangents[v] / mSurface.Tangents[v].Size());
		//UE_LOG(LogTemp, Warning, TEXT("VertexNormal %d: %f %f %f"), v, mSurface.Normals[v].X, mSurface.Normals[v].Y, mSurface.Normals[v].Z);
	}
	UE_LOG(LogTemp, Warning, TEXT("Indices: %d"), mSurface.Indices.Num());


	meshSurfaces.Add(mSurface);

	std::string filepath = "C:\\dev\\projects\\monsterGame\\teste.off";
	std::ofstream file(filepath.c_str());
	file << "OFF" << std::endl;
	file << mSurface.Vertices.Num() << " " << mSurface.Indices.Num() / 3 << " 0" << std::endl;
	for (size_t v = 0; v < nVertices; ++v)
	{
		file << mSurface.Vertices[v].X << " " << mSurface.Vertices[v].Y << " " << mSurface.Vertices[v].Z << std::endl;
	}
	for (int32 f = 0; f < mSurface.Indices.Num(); f += 3)
	{
		file << "3 " << mSurface.Indices[f] << " " << mSurface.Indices[f + 1] << " " << mSurface.Indices[f + 2] << std::endl;
	}
	file.close();
}


void buildSkeleton(FReferenceSkeleton& refSkel, USkeleton* skeleton)
{
	refSkel.Empty();
	FReferenceSkeletonModifier RefSkelModifier(refSkel, skeleton);

	const FMeshBoneInfo BoneInfo("bn0_in", "bn0_ex", INDEX_NONE);
	const FTransform BoneTransform(FTransform(FVector(0, 0, 0)));

	const FMeshBoneInfo BoneInfo2("bone1_in", "bone1_ex", 0);
	const FTransform BoneTransform2(FTransform(FVector(0, 50, 0)));

	RefSkelModifier.Add(BoneInfo, BoneTransform);
	RefSkelModifier.Add(BoneInfo2, BoneTransform2);
}

USkeletalMesh* USkeletalMeshLoader::GenerateBlob(TArray<UMaterialInterface*> mats) {

	UE_LOG(LogTemp, Warning, TEXT("Meshes: %d"), meshSurfaces.Num());
	if (meshSurfaces.Num() == 0)
	{
		GenerateMyBlob();
	}


	USkeletalMesh* skeletalMesh = NewObject<USkeletalMesh>();

	USkeleton* skeleton = NewObject<USkeleton>();
	buildSkeleton((FReferenceSkeleton&)skeleton->GetReferenceSkeleton(), skeleton);

	FString name = skeleton->GetReferenceSkeleton().GetBoneName(0).ToString();
	FString name2 = skeleton->GetReferenceSkeleton().GetBoneName(1).ToString();

	UE_LOG(LogTemp, Warning, TEXT("Bone name %s , %s"), *name, *name2);

	skeletalMesh->SetRefSkeleton(skeleton->GetReferenceSkeleton());
	skeletalMesh->SetSkeleton(skeleton);


	FRuntimeSkeletalMeshGenerator::GenerateSkeletalMesh(
		skeletalMesh,
		meshSurfaces,
		mats
	);

	return skeletalMesh;
}




USkeletalMesh* USkeletalMeshLoader::OpenFile(USkeleton* BaseSkeleton, TArray<UMaterialInterface*> mats) {

	std::string filepath = "C:\\dev\\projects\\monsterGame\\voxelizer\\mesh-voxelization\\examples\\input\\teste.off";
	std::ifstream* file = new std::ifstream(filepath.c_str());
	std::string line;
	std::stringstream ss;
	int line_nb = 0;

	bool usecolor = false;

	std::getline(*file, line);
	++line_nb;

	if (line != "off" && line != "OFF") {
		std::cout << "[Error] Invalid header: \"" << line << "\", " << filepath << std::endl;
		return false;
	}

	size_t n_edges;
	std::getline(*file, line);
	++line_nb;

	int n_vertices;
	int n_faces;
	ss << line;
	ss >> n_vertices;
	ss >> n_faces;
	ss >> n_edges;

	//TArray<FMeshSurface> meshSurfaces;
	meshSurfaces.Empty();

	FMeshSurface mSurface;

	mSurface.Vertices.Empty();
	mSurface.Normals.Empty();
	mSurface.Tangents.Empty();
	mSurface.BoneInfluences.Empty();
	mSurface.Uvs.Empty();
	mSurface.Colors.Empty();

	mSurface.Vertices.SetNum(n_vertices);
	mSurface.Normals.SetNum(n_vertices);
	mSurface.Tangents.SetNum(n_vertices);
	mSurface.BoneInfluences.SetNum(n_vertices);
	mSurface.Uvs.SetNum(n_vertices);
	mSurface.Colors.SetNum(n_vertices);

	int count1 = 0, count2 = 0;
	for (size_t v = 0; v < n_vertices; ++v) {
		std::getline(*file, line);
		++line_nb;

		ss.clear();
		ss.str("");

		FVector vertex;
		double x, y, z;
		ss << line;
		ss >> x;
		ss >> y;
		ss >> z;

		mSurface.Vertices[v] = FVector(x, z, y);
		mSurface.Normals[v] = FVector(0, 0, 0);
		mSurface.Tangents[v] = FVector(0, 0, 0);
		mSurface.Colors[v] = FColor(255, 0, 255);

		mSurface.FlipBinormalSigns.Add(false);

		TArray<FVector2D> UVs;
		UVs.Add(FVector2D(count1,count2));
		int nextNum = 0;
		if (count1 == nextNum && count2 == 0)
		{
			count1 = 0;
			count2 = 0;
		}
		else
		{
			if (count2 != nextNum)
			{
				if (count1 == nextNum)
					count2 = nextNum;
				if (count1 == 0)
					count1 = nextNum;
			}
			else
			{
				if (count2 == nextNum)
				{
					count1 = nextNum;
					count2 = 0;
				}
			}
		}
		mSurface.Uvs[v] = UVs;

		

		TArray<FRawBoneInfluence> boneInfluences;
		FRawBoneInfluence boneInf;

		boneInf.BoneIndex = 0;
		boneInf.VertexIndex = v;
		boneInf.Weight = 1;

		boneInfluences.Add(boneInf);
		mSurface.BoneInfluences[v] = boneInfluences;
	}
	for (size_t f = 0; f < n_faces; ++f) {
		std::getline(*file, line);
		++line_nb;

		ss.clear();
		ss.str("");

		size_t n;
		ss << line;
		ss >> n;

		if (n != 3) {
			std::cout << "[Error] Not a triangle (" << n << " points) at " << (line_nb - 1) << std::endl;
			return false;
		}

		//std::cout << "Empty color vector: " << color(0) <<  " " << color(1) << " " << color(2) << std::endl;
		int32 i1, i2, i3;
		int color1, color2, color3;

		ss >> i1;
		ss >> i2;
		ss >> i3;

		if (usecolor == true) {
			ss >> color1;
			ss >> color2;
			ss >> color3;
		};
		//std::cout << "Color vector after reading line: " << color(0) <<  " " << color(1) << " " << color(2) << std::endl;

		mSurface.Indices.Add(i1);
		mSurface.Indices.Add(i2);
		mSurface.Indices.Add(i3);
		FVector a, b, c;
		a = mSurface.Vertices[i1];// curr1.X = curr1.X*-1;
		b = mSurface.Vertices[i2];// curr1.X = curr1.X * -1;
		c = mSurface.Vertices[i3];// curr1.X = curr1.X * -1;
		FVector BA(b - a);
		FVector CA(c - a);
		FVector FaceNormal(FVector::CrossProduct(BA,CA));
		FaceNormal = FaceNormal / FaceNormal.Size();
		//if (f==0)
		//FaceNormal = FaceNormal * -1;
		//FaceNormal.X = FaceNormal.X * -1;

		mSurface.Normals[i1] += FaceNormal;
		mSurface.Normals[i2] += FaceNormal;
		mSurface.Normals[i3] += FaceNormal;
		
		
		UE_LOG(LogTemp, Warning, TEXT("FaceNormal %d: %f %f %f"),f, FaceNormal.X, FaceNormal.Y, FaceNormal.Z);
	}
	for (size_t v = 0; v < n_vertices; ++v)
	{
		mSurface.Normals[v] = (mSurface.Normals[v] / mSurface.Normals[v].Size());
		UE_LOG(LogTemp, Warning, TEXT("VertexNormal %d: %f %f %f"), v, mSurface.Normals[v].X, mSurface.Normals[v].Y, mSurface.Normals[v].Z);
	}


	file->close();
	delete file;

	meshSurfaces.Add(mSurface);
	USkeletalMesh* skeletalMesh = NewObject<USkeletalMesh>();
	skeletalMesh->SetRefSkeleton(BaseSkeleton->GetReferenceSkeleton());
	skeletalMesh->SetSkeleton(BaseSkeleton);
	
	TArray<UMaterialInterface*> Materials;
	UMaterialInterface** matPtrArray = mats.GetData();

	for (size_t m = 0; m < mats.Num(); m++)
	{
		Materials.Add(matPtrArray[m]);
		UE_LOG(LogTemp, Warning, TEXT("Mat Loaded %d"), m);
	}


	
	FRuntimeSkeletalMeshGenerator::GenerateSkeletalMesh(
		skeletalMesh,
		meshSurfaces,
		Materials
	);

	return skeletalMesh;
}

