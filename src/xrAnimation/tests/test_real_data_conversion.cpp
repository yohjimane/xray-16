#include "../stdafx.h"
#include "../OGFConverter.h"
#include "../AnimationConverter.h"
#include "../OzzAnimationSystem.h"
#include "../OzzKinematicsAnimated.h"
#include "xrCore/xrCore.h"
#include "xrCore/FS.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"

using namespace XRay::Animation;

void TestOGFConversion()
{
    Msg("=== Testing OGF to ozz Conversion ===");
    
    // Test with real stalker model
    std::string ogf_path = "../../res/gamedata/meshes/actors/stalker_bandit/stalker_bandit_1.ogf";
    
    if (!FS.exist(ogf_path.c_str())) {
        Msg("! OGF file not found: %s", ogf_path.c_str());
        return;
    }
    
    Msg("* Loading OGF file: %s", ogf_path.c_str());
    
    // Create OGF converter
    OGFConverter converter;
    
    // Load OGF file
    if (!converter.LoadFromFile(ogf_path)) {
        Msg("! Failed to load OGF file");
        return;
    }
    
    Msg("  - OGF loaded successfully");
    
    // Convert to ozz
    auto result = converter.Convert();
    if (!result.skeleton) {
        Msg("! Failed to convert skeleton");
        return;
    }
    
    Msg("* Conversion successful:");
    Msg("  - Joints: %d", result.skeleton->num_joints());
    Msg("  - Animations: %d", result.animations.size());
    
    // List first few bones
    const auto& names = result.skeleton->joint_names();
    for (int i = 0; i < std::min(10, result.skeleton->num_joints()); ++i) {
        Msg("  - Bone[%d]: %s", i, names[i]);
    }
    
    // Save skeleton for later use
    std::string skeleton_output = "test_skeleton.ozz";
    {
        ozz::io::File file(skeleton_output.c_str(), "wb");
        if (file.opened()) {
            ozz::io::OArchive archive(&file);
            archive << *result.skeleton;
            Msg("* Saved skeleton to: %s", skeleton_output.c_str());
        }
    }
}

void TestOMFConversion()
{
    Msg("\n=== Testing OMF to ozz Animation Conversion ===");
    
    // First load the skeleton we converted
    std::string skeleton_path = "test_skeleton.ozz";
    if (!FS.exist(skeleton_path.c_str())) {
        Msg("! Skeleton file not found. Run OGF conversion first.");
        return;
    }
    
    // Load skeleton
    ozz::animation::Skeleton skeleton;
    {
        ozz::io::File file(skeleton_path.c_str(), "rb");
        if (!file.opened()) {
            Msg("! Failed to open skeleton file");
            return;
        }
        ozz::io::IArchive archive(&file);
        archive >> skeleton;
    }
    
    Msg("* Loaded skeleton with %d joints", skeleton.num_joints());
    
    // Test with stalker animation
    std::string omf_path = "../../res/gamedata/meshes/actors/stalker_animation.omf";
    
    if (!FS.exist(omf_path.c_str())) {
        Msg("! OMF file not found: %s", omf_path.c_str());
        return;
    }
    
    Msg("* Loading OMF file: %s", omf_path.c_str());
    
    // Read OMF file
    IReader* reader = FS.r_open(omf_path.c_str());
    if (!reader) {
        Msg("! Failed to open OMF file");
        return;
    }
    
    // Parse OMF header
    u32 dwFlag = reader->r_u32();
    Msg("  - OMF Flags: 0x%08x", dwFlag);
    
    if (dwFlag & 0x1) { // OMF_HAS_MOTION
        u16 motion_count = reader->r_u16();
        Msg("  - Motion count: %d", motion_count);
        
        // Read first motion info
        if (motion_count > 0) {
            // Read motion name
            shared_str motion_name;
            reader->r_stringZ(motion_name);
            
            u32 motion_length = reader->r_u32();
            
            Msg("  - First motion: '%s' (length: %d)", motion_name.c_str(), motion_length);
            
            // We would need to parse the actual motion data here
            // For now, just show we can read the format
        }
    }
    
    FS.r_close(reader);
}

void TestOzzAnimationSystemWithRealData()
{
    Msg("\n=== Testing OzzAnimationSystem with Real Data ===");
    
    std::string skeleton_path = "test_skeleton.ozz";
    if (!FS.exist(skeleton_path.c_str())) {
        Msg("! Test skeleton not found. Run conversions first.");
        return;
    }
    
    // Create animation system
    auto animation_system = std::make_unique<OzzAnimationSystem>();
    
    // Load skeleton
    if (!animation_system->LoadSkeleton(skeleton_path)) {
        Msg("! Failed to load skeleton");
        return;
    }
    
    Msg("* Skeleton loaded:");
    Msg("  - Bone count: %d", animation_system->GetBoneCount());
    
    // Find specific bones
    size_t spine_idx = animation_system->FindBoneIndex("bip01_spine");
    size_t head_idx = animation_system->FindBoneIndex("bip01_head");
    size_t pelvis_idx = animation_system->FindBoneIndex("bip01_pelvis");
    
    if (spine_idx != static_cast<size_t>(-1)) {
        Msg("  - Found spine at index: %d", spine_idx);
    }
    if (head_idx != static_cast<size_t>(-1)) {
        Msg("  - Found head at index: %d", head_idx);
    }
    if (pelvis_idx != static_cast<size_t>(-1)) {
        Msg("  - Found pelvis at index: %d", pelvis_idx);
    }
    
    // Test bone transforms
    Msg("\n* Testing bone transforms:");
    for (size_t i = 0; i < std::min(size_t(5), animation_system->GetBoneCount()); ++i) {
        Fmatrix transform = animation_system->GetBoneTransform(i);
        Msg("  - Bone[%d] position: (%.3f, %.3f, %.3f)", 
            i, transform.c.x, transform.c.y, transform.c.z);
    }
}

void TestPartitionSetup()
{
    Msg("\n=== Testing Partition Setup with Real Skeleton ===");
    
    auto animation_system = std::make_unique<OzzAnimationSystem>();
    
    std::string skeleton_path = "test_skeleton.ozz";
    if (!animation_system->LoadSkeleton(skeleton_path)) {
        Msg("! Failed to load skeleton for partition test");
        return;
    }
    
    // Define partitions based on common X-Ray bone structure
    // Torso partition
    xr_vector<u16> torso_bones;
    const char* torso_bone_names[] = {
        "bip01_spine", "bip01_spine1", "bip01_spine2", 
        "bip01_neck", "bip01_head",
        "bip01_l_clavicle", "bip01_l_upperarm", "bip01_l_forearm", "bip01_l_hand",
        "bip01_r_clavicle", "bip01_r_upperarm", "bip01_r_forearm", "bip01_r_hand"
    };
    
    for (const char* bone_name : torso_bone_names) {
        size_t idx = animation_system->FindBoneIndex(bone_name);
        if (idx != static_cast<size_t>(-1)) {
            torso_bones.push_back(static_cast<u16>(idx));
        }
    }
    
    if (!torso_bones.empty()) {
        animation_system->SetPartitionMask(1, torso_bones);
        Msg("* Set torso partition with %d bones", torso_bones.size());
    }
    
    // Legs partition
    xr_vector<u16> legs_bones;
    const char* legs_bone_names[] = {
        "bip01_pelvis",
        "bip01_l_thigh", "bip01_l_calf", "bip01_l_foot", "bip01_l_toe0",
        "bip01_r_thigh", "bip01_r_calf", "bip01_r_foot", "bip01_r_toe0"
    };
    
    for (const char* bone_name : legs_bone_names) {
        size_t idx = animation_system->FindBoneIndex(bone_name);
        if (idx != static_cast<size_t>(-1)) {
            legs_bones.push_back(static_cast<u16>(idx));
        }
    }
    
    if (!legs_bones.empty()) {
        animation_system->SetPartitionMask(2, legs_bones);
        Msg("* Set legs partition with %d bones", legs_bones.size());
    }
}

void ListAvailableFiles()
{
    Msg("\n=== Available X-Ray Animation Files ===");
    
    // List OGF files
    Msg("\n* OGF (skeleton) files:");
    FS_FileSet ogf_files;
    FS.file_list(ogf_files, "../../res/gamedata/meshes/actors", FS_ListFiles | FS_RootOnly, "*.ogf");
    
    int count = 0;
    for (const auto& file : ogf_files) {
        Msg("  - %s", file.name.c_str());
        if (++count >= 10) {
            Msg("  ... and %d more", ogf_files.size() - count);
            break;
        }
    }
    
    // List OMF files
    Msg("\n* OMF (animation) files:");
    FS_FileSet omf_files;
    FS.file_list(omf_files, "../../res/gamedata/meshes/actors", FS_ListFiles | FS_RootOnly, "*.omf");
    
    for (const auto& file : omf_files) {
        Msg("  - %s", file.name.c_str());
    }
}

int main(int argc, char* argv[])
{
    // Initialize core
    Debug._initialize(false);
    Core._initialize("real_data_test", nullptr, TRUE, "fs.ltx");
    
    Msg("==================================================");
    Msg("   X-Ray ozz-animation Real Data Test");
    Msg("==================================================\n");
    
    // List available files
    ListAvailableFiles();
    
    // Run tests
    TestOGFConversion();
    TestOMFConversion();
    TestOzzAnimationSystemWithRealData();
    TestPartitionSetup();
    
    Msg("\n==================================================");
    Msg("   Test Complete");
    Msg("==================================================");
    
    // Cleanup
    Core._destroy();
    
    return 0;
}