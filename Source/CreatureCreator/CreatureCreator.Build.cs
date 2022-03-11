// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class CreatureCreator : ModuleRules
{
	public CreatureCreator(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "DesktopPlatform", "RuntimeSkeletalMeshGenerator" });

		PrivateDependencyModuleNames.AddRange(new string[] {  });

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
		
		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			string PathToProject = "C:\\dev\\projects\\monsterGame\\CreatureCreator";
			PublicIncludePaths.Add(Path.Combine(PathToProject, "Source\\CreatureCreator\\include"));
			string PlatformString =  "Win64";
			string LibrariesPath = Path.Combine(PathToProject, "Binaries", PlatformString);

			PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "assimp-vc142-mt.lib"));
		}
		
	}
}
