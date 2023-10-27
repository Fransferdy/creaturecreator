// Fill out your copyright notice in the Description page of Project Settings.


#include "CreatureBodyMaker.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include "Math/UnrealMathUtility.h"
#include "Math/Vector2D.h"
#include "TriangleBoxOverlap.h"


#define BONE_DIST 40
#define DEGREE_TO_RADIAN 0.01745329252

// Sets default values for this component's properties
UCreatureBodyMaker::UCreatureBodyMaker()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	MBone firstBodyBone = { FTransform(FVector(0,0,0)),"s0" };

	bodyBones.EmplaceAt(0, firstBodyBone);
	// ...000000000000
}


// Called when the game starts
void UCreatureBodyMaker::BeginPlay()
{
	Super::BeginPlay();

	// ...

}


// Called every frame
void UCreatureBodyMaker::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}


/*
Get 3D cube coordinate in 1D array
@x, x coordinate
@y, y coordinate
@z, z coordinate
*/
inline size_t p3d(size_t x, size_t y, size_t z, size_t size)
{
	return (x + y * size + (z * size * size));// ((x*d1)+y*d2)+z;
}

/** \brief Voxelize the given mesh into an occupancy grid.
 * \param[out] occ volume to fill
 */

bool triangle_box_intersection(const FVector& min, FVector& max, const FVector& v1, const FVector& v2, const FVector& v3) {
	float half_size[3] = {
	  (max.X - min.X) / 2.,
	  (max.Y - min.Y) / 2.,
	  (max.Z - min.Z) / 2.
	};

	float center[3] = {
	  max.X - half_size[0],
	  max.Y - half_size[1],
	  max.Z - half_size[2]
	};

	float vertices[3][3] = { {v1.X, v1.Y, v1.Z}, {v2.X, v2.Y, v2.Z}, {v3.X, v3.Y, v3.Z} };
	return triBoxOverlap(center, half_size, vertices);
}

struct voxelBoxStruct {
	bool active;
	char boneInfluences[3];
};
typedef voxelBoxStruct voxelBox;

struct voxelsInfoStruct {
	size_t size;
	size_t activeVoxels;
	voxelBox* data;
};
typedef voxelsInfoStruct voxelsInfo;
//Voxelize a model, it was supposed to be used for automatic skinning, but it is too slow.
voxelsInfo voxelize_occ(TArray<FVector> &vertices,TArray<uint32> &faceIndices, size_t size) {
	size_t halfSize = size/2;
	int height = size;
	int width = size;
	int depth = size;
	size_t totalArraySize=height * width * depth;
	int32 numberOfFaceIndices = faceIndices.Num();
	int32 verticesSize = vertices.Num();
	int32 maxExtension = 0;
	UE_LOG(LogTemp, Warning, TEXT("Counting Vertices Size"));
	for (size_t i = 0; i < verticesSize; i++)
	{
		FVector& vertex = vertices[i];
		int32 currentX = abs(FMath::CeilToInt(vertex.X));
		int32 currentY = abs(FMath::CeilToInt(vertex.Y));
		int32 currentZ = abs(FMath::CeilToInt(vertex.Z));
		if (currentX > maxExtension)
			maxExtension = currentX;
		if (currentY > maxExtension)
			maxExtension = currentY;
		if (currentZ > maxExtension)
			maxExtension = currentZ;
	}
	float growthRatio = (float)halfSize / (float)maxExtension;

	UE_LOG(LogTemp, Warning, TEXT("Growth Ratio %f "), growthRatio);
	
	voxelBox* voxelArray = (voxelBox*)calloc(totalArraySize,sizeof(voxelBox));
	UE_LOG(LogTemp, Warning, TEXT("Checking Interceptions"));
	size_t activeVoxels=0;
	voxelsInfo res;
	res.data = voxelArray;
	res.activeVoxels = 0;
	res.size = size;
	//voxelArray[p3d(halfSize, halfSize, halfSize, size)].active = true; res.activeVoxels += 1;
	
	
//#pragma omp parallel
	{
//#pragma omp for
		FVector center(halfSize, halfSize, halfSize);
		ParallelFor(totalArraySize, [&](int i) {
			int d = i % depth;
			int w = (i / depth) % width;
			int h = (i / depth) / width;

			FVector min(w, h, d);
			FVector max(w + 1, h + 1, d + 1);

			for (int32 f = 0; f < numberOfFaceIndices; f += 3) {

				FVector v1 = (vertices[faceIndices[f]] * growthRatio) + center;   // this->vertices[this->faces[f](0)];
				FVector v2 = (vertices[faceIndices[f + 1]] * growthRatio) + center; // this->vertices[this->faces[f](1)];
				FVector v3 = (vertices[faceIndices[f + 2]] * growthRatio) + center; //this->vertices[this->faces[f](2)];

				bool overlap = triangle_box_intersection(min, max, v1, v2, v3);
				if (overlap) {
					voxelArray[p3d(d, w, h, size)].active = true;
					//res.activeVoxels += 1;
					break;
				}
			}
		});
	}
	

	return res;
}


void UCreatureBodyMaker::AddBoneToBodyEnd()
{
	MBone  near = bodyBones.GetData()[bodyBones.Num() - 1];
	FVector basePos = near.transform.GetLocation();
	basePos.Y += BONE_DIST;
	FString bName = near.name + "_next";
	MBone nextBone = { FTransform(basePos),bName };
	bodyBones.Emplace(nextBone);

	UE_LOG(LogTemp, Warning, TEXT("BONE ADDED TO END"));
}
void UCreatureBodyMaker::AddBoneToBodyFront()
{
	MBone  near = bodyBones.GetData()[0];
	FVector basePos = near.transform.GetLocation();
	basePos.Y -= BONE_DIST;
	FString bName = near.name + "_back";
	MBone nextBone = { FTransform(basePos), bName };
	bodyBones.EmplaceAt(0, nextBone);
	UE_LOG(LogTemp, Warning, TEXT("BONE ADDED TO FRONT"));
}

TArray<FVector> getCirclePoints2(int32 parts, double y, double radius)
{
	TArray<FVector> res;
	//UE_LOG(LogTemp, Warning, TEXT("GET CIRCLE POINTS"));

	for (int32 part = 0; part < parts; part++)
	{
		double x, z;
		double degreeInRadian = (360.0 / (double)parts) * (double)part * DEGREE_TO_RADIAN;

		float sin, cos;
		FMath::SinCos(&sin, &cos, degreeInRadian);
		z = sin * radius;
		x = cos * radius;

		res.Add(FVector(z, y, x));
		//UE_LOG(LogTemp, Warning, TEXT("Point: %d %f %f %f"), part, x, y, z);
	}

	return res;
}

void calculateFaceNormalsTangentsBitTangents2(FVector& a, FVector& b, FVector& c, FVector2D& uv1, FVector2D& uv2, FVector2D& uv3, FVector& normalOut, FVector& tangentOut)//, FVector& bitTangetOut)
{
	FVector edge1(b - a);
	FVector edge2(c - a);
	FVector2D deltaUV1(uv2 - uv1);
	FVector2D deltaUV2(uv3 - uv1);

	normalOut = (FVector::CrossProduct(edge1, edge2));
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


void buildSkeleton2(FReferenceSkeleton& refSkel, USkeleton* skeleton, TArray<MBone>& bones)
{
	/*
	refSkel.Empty();
	FReferenceSkeletonModifier RefSkelModifier(refSkel, skeleton);

	const FMeshBoneInfo BoneInfo("bn0_in", "bn0_ex", INDEX_NONE);
	const FTransform BoneTransform(FTransform(FVector(0, 0, 0)));

	const FMeshBoneInfo BoneInfo2("bone1_in", "bone1_ex", 0);
	const FTransform BoneTransform2(FTransform(FVector(0, 50, 0)));

	RefSkelModifier.Add(BoneInfo, BoneTransform);
	RefSkelModifier.Add(BoneInfo2, BoneTransform2);
	*/
	refSkel.Empty();
	FReferenceSkeletonModifier RefSkelModifier(refSkel, skeleton);

	auto bonesData = bones.GetData();
	int32 rootBoneIndex = 0;
	float rootDistance = MAX_int32;
	FVector center(0, 0, 0);

	for (int32 bone = 0; bone < bones.Num(); bone++)
	{
		FVector diff = bonesData[bone].transform.GetLocation() - center;
		float distance = diff.Size();
		if (distance < rootDistance)
		{
			rootBoneIndex = bone;
			rootDistance = distance;
		}
	}


	FName rootBoneName(bonesData[rootBoneIndex].name + "in");
	FString rootBoneNameExt(bonesData[rootBoneIndex].name + "ext");
	const FMeshBoneInfo RootBoneInfo(rootBoneName, rootBoneNameExt, INDEX_NONE);
	RefSkelModifier.Add(RootBoneInfo, bonesData[rootBoneIndex].transform);



	UE_LOG(LogTemp, Warning, TEXT("RootBone: %s %s"), *rootBoneName.ToString(), *rootBoneNameExt);

	for (int32 bone = rootBoneIndex + 1; bone < bones.Num(); bone++)
	{
		FName boneName(bonesData[bone].name + "in");
		FString boneNameExt(bonesData[bone].name + "ext");
		const FMeshBoneInfo boneInfo(boneName, boneNameExt, 0);
		FTransform boneTransform((bonesData[bone].transform.GetLocation() - bonesData[rootBoneIndex].transform.GetLocation()));
		RefSkelModifier.Add(boneInfo, boneTransform);
		UE_LOG(LogTemp, Warning, TEXT("OtherBoneEnd: %s"), *boneNameExt);
	}
	for (int32 bone = rootBoneIndex - 1; bone >= 0; bone--)
	{
		FName boneName(bonesData[bone].name + "in");
		FString boneNameExt(bonesData[bone].name + "ext");
		const FMeshBoneInfo boneInfo(boneName, boneNameExt, 0);
		FTransform boneTransform((bonesData[bone].transform.GetLocation() - bonesData[rootBoneIndex].transform.GetLocation()));
		RefSkelModifier.Add(boneInfo, boneTransform);
		UE_LOG(LogTemp, Warning, TEXT("OtherBoneFront: %s"), *boneNameExt);
	}


	if (bones.Num() == 1)
	{
		const FMeshBoneInfo BoneInfo2("bone1_in", "bone1_ex", 0);
		const FTransform BoneTransform2(FTransform(FVector(0, 50, 0)));

		RefSkelModifier.Add(BoneInfo2, BoneTransform2);
	}

}

USkeletalMesh* UCreatureBodyMaker::Generate(TArray<UMaterialInterface*> mats) {

	USkeletalMesh* ptr = nullptr;
	TArray<FMeshSurface> meshSurfaces;
	FMeshSurface mSurface;

	int32 lastBoneIndex = bodyBones.Num() - 1;
	int32 firstBoneIndex = 0;
	UE_LOG(LogTemp, Warning, TEXT("FirstBone: %d"), firstBoneIndex);
	UE_LOG(LogTemp, Warning, TEXT("LastBone: %d"), lastBoneIndex);

	TArray<FVector> vertices;
	int32 circlePartsSize = 20;
	double circleRadius = 20;

	double yStep = BONE_DIST / 20;
	int32 yMax = BONE_DIST;

	auto bonesData = bodyBones.GetData();
	float yBase = bonesData[0].transform.GetLocation().Y - (BONE_DIST / 2);
	int32 edgeSize = yMax / 4;
	int32 yMaxBones = (yMax * bodyBones.Num());
	for (double y = 0; y < edgeSize; y += yStep)
	{
		float percent = (float)y / (float)edgeSize;
		float sin = FMath::Sin(90.0 * percent * DEGREE_TO_RADIAN);
		float ringRadius = circleRadius * sin;
		float ringDistance = y - edgeSize;
		vertices.Append(getCirclePoints2(circlePartsSize, ringDistance + yBase, ringRadius));
	}
	for (double y = 0; y < yMaxBones; y += yStep)
	{
		vertices.Append(getCirclePoints2(circlePartsSize, yBase + y, circleRadius));
	}
	for (double y = 0; y < edgeSize; y += yStep)
	{
		float percent = (float)y / (float)edgeSize;
		if (percent == 1)
			continue;
		float sin = FMath::Sin(90.0 * (1.0 - percent) * DEGREE_TO_RADIAN);
		float ringRadius = circleRadius * (sin);
		float ringDistance = y;
		UE_LOG(LogTemp, Warning, TEXT("percent: %f"), percent);
		vertices.Append(getCirclePoints2(circlePartsSize, ringDistance + yBase + yMaxBones, ringRadius));
	}
	vertices.Append(getCirclePoints2(circlePartsSize, edgeSize + yBase + yMaxBones, 0));

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

		double uvx = ((mSurface.Vertices[v].X + circleRadius) * 0.5) / circleRadius;
		double uvy = (mSurface.Vertices[v].Y) / yMax;
		TArray<FVector2D> UVs;
		UVs.Add(FVector2D(uvx, uvy));
		mSurface.Uvs[v] = UVs;
		//UE_LOG(LogTemp, Warning, TEXT("Point: %d %f %f %f"), v, mSurface.Vertices[v].X, mSurface.Vertices[v].Y, mSurface.Vertices[v].Z);

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
		calculateFaceNormalsTangentsBitTangents2(mSurface.Vertices[i1], mSurface.Vertices[i2], mSurface.Vertices[i3],
			mSurface.Uvs[i1][0], mSurface.Uvs[i2][0], mSurface.Uvs[i3][0],
			FaceNormal, FaceTangent);// , FaceBiTangent);
		//FVector FaceNormal = calculateFaceNormal(mSurface.Vertices[i1], mSurface.Vertices[i2], mSurface.Vertices[i3]);

		mSurface.Normals[i1] += FaceNormal;
		mSurface.Normals[i2] += FaceNormal;
		mSurface.Normals[i3] += FaceNormal;

		mSurface.Tangents[i1] += FaceTangent;
		mSurface.Tangents[i2] += FaceTangent;
		mSurface.Tangents[i3] += FaceTangent;

		calculateFaceNormalsTangentsBitTangents2(mSurface.Vertices[i3], mSurface.Vertices[i4], mSurface.Vertices[i2],
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
	calculateFaceNormalsTangentsBitTangents2(mSurface.Vertices[i1], mSurface.Vertices[i2], mSurface.Vertices[i3],
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

	calculateFaceNormalsTangentsBitTangents2(mSurface.Vertices[i1], mSurface.Vertices[i2], mSurface.Vertices[i3],
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

	/*
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
	*/


	USkeletalMesh* skeletalMesh = NewObject<USkeletalMesh>();

	USkeleton* skeleton = NewObject<USkeleton>();
	buildSkeleton2((FReferenceSkeleton&)skeleton->GetReferenceSkeleton(), skeleton, bodyBones);

	FString name = skeleton->GetReferenceSkeleton().GetBoneName(0).ToString();
	FString name2 = skeleton->GetReferenceSkeleton().GetBoneName(1).ToString();

	UE_LOG(LogTemp, Warning, TEXT("Bone name %s , %s"), *name, *name2);

	skeletalMesh->SetRefSkeleton(skeleton->GetReferenceSkeleton());
	skeletalMesh->SetSkeleton(skeleton);


	/* VOXELIZATION * WORKING but too slow, maybe search other options for skinning!
	size_t voxelsSize = 64;
	UE_LOG(LogTemp, Warning, TEXT("Start Voxelization"));
	voxelsInfo voxels = voxelize_occ(mSurface.Vertices, mSurface.Indices, voxelsSize);
	UE_LOG(LogTemp, Warning, TEXT("End Voxelization"));
	
	UE_LOG(LogTemp, Warning, TEXT("Saving Voxels to File"));
	std::string filepath = "C:\\dev\\projects\\monsterGame\\voxels.off";
	std::ofstream file(filepath.c_str());
	size_t totalActive=0;
	for (size_t voxelIndex = 0; voxelIndex < voxelsSize * voxelsSize * voxelsSize; voxelIndex++)
	{
		if (voxels.data[voxelIndex].active == 1)
			totalActive += 1;
	}
	file << "OFF" << std::endl;
	file << (totalActive *8) << " " << (totalActive * 12) << " 0" << std::endl;
	for (size_t voxelIndex = 0; voxelIndex < voxelsSize * voxelsSize * voxelsSize; voxelIndex++)
	{
		if (voxels.data[voxelIndex].active==1)
		{
			int d = voxelIndex % voxelsSize;
			int w = (voxelIndex / voxelsSize) % voxelsSize;
			int h = (voxelIndex / voxelsSize) / voxelsSize;
			file << d << " " << w << " " << h << std::endl;
			file << d << " " << w + 1 << " " << h << std::endl;
			file << d + 1 << " " << w << " " << h << std::endl;
			file << d + 1 << " " << w + 1 << " " << h << std::endl;

			file << d << " " << w << " " << h + 1 << std::endl;
			file << d << " " << w + 1 << " " << h + 1 << std::endl;
			file << d + 1 << " " << w << " " << h + 1 << std::endl;
			file << d + 1 << " " << w + 1 << " " << h + 1 << std::endl;
		}
	}
	size_t voxelIndex = 0;
	for (size_t i = 0; i < totalActive; i++)
	{
		voxelIndex = i * 8;
			file << "3 " << voxelIndex << " " << voxelIndex + 1 << " " << voxelIndex + 2 << std::endl;
			file << "3 " << voxelIndex + 1 << " " << voxelIndex + 2 << " " << voxelIndex + 3 << std::endl;

			file << "3 " << voxelIndex << " " << voxelIndex + 4 << " " << voxelIndex + 2 << std::endl;
			file << "3 " << voxelIndex + 6 << " " << voxelIndex + 4 << " " << voxelIndex + 2 << std::endl;

			file << "3 " << voxelIndex << " " << voxelIndex + 1 << " " << voxelIndex + 4 << std::endl;
			file << "3 " << voxelIndex + 1 << " " << voxelIndex + 4 << " " << voxelIndex + 5 << std::endl;

			file << "3 " << voxelIndex + 1 << " " << voxelIndex + 3 << " " << voxelIndex + 5 << std::endl;
			file << "3 " << voxelIndex + 3 << " " << voxelIndex + 5 << " " << voxelIndex + 7 << std::endl;

			file << "3 " << voxelIndex + 2 << " " << voxelIndex + 3 << " " << voxelIndex + 7 << std::endl;
			file << "3 " << voxelIndex + 2 << " " << voxelIndex + 6 << " " << voxelIndex + 7 << std::endl;

			file << "3 " << voxelIndex + 4 << " " << voxelIndex + 5 << " " << voxelIndex + 6 << std::endl;
			file << "3 " << voxelIndex + 6 << " " << voxelIndex + 5 << " " << voxelIndex + 7 << std::endl;
	}
	file.close();
	UE_LOG(LogTemp, Warning, TEXT("End Saving Voxels to File"));

	free(voxels.data);
	*/

	

	FRuntimeSkeletalMeshGenerator::GenerateSkeletalMesh(
		skeletalMesh,
		meshSurfaces,
		mats
	);

	return skeletalMesh;
}




