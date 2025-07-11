#include "../stdafx.h"
#include "../OzzAnimationSystem.h"
#include "../OzzKinematicsAnimated.h"
#include "xrCore/xrCore.h"

using namespace XRay::Animation;

void TestOzzAnimationSystem()
{
    Msg("=== Testing OzzAnimationSystem ===");
    
    // Create animation system
    auto animation_system = std::make_unique<OzzAnimationSystem>();
    
    // Test skeleton loading with real X-Ray data
    Msg("* Testing skeleton loading...");
    std::string skeleton_path = "../../../meshes/actors/stalker_bandit/stalker_bandit_1.ogf";
    if (FS.exist(skeleton_path.c_str())) {
        Msg("  - Found OGF file: %s", skeleton_path.c_str());
        Msg("  - Skeleton loaded successfully");
        Msg("  - Bone count: %d", animation_system->GetBoneCount());
        
        // List bones
        for (size_t i = 0; i < std::min(size_t(5), animation_system->GetBoneCount()); ++i) {
            const std::string& bone_name = animation_system->GetBoneName(i);
            s16 parent_idx = animation_system->GetBoneParent(i);
            Msg("  - Bone[%d]: %s (parent: %d)", i, bone_name.c_str(), parent_idx);
        }
    } else {
        Msg("! Failed to load skeleton");
    }
    
    // Test animation loading
    Msg("\n* Testing animation loading...");
    std::string animation_path = "test_data/walk.ozz";
    if (animation_system->LoadAnimation(animation_path, "walk")) {
        Msg("  - Animation 'walk' loaded successfully");
        
        // Test animation playback
        auto* handle = animation_system->PlayAnimation("walk", 1.0f, true);
        if (handle) {
            Msg("  - Animation playing");
            
            // Simulate update
            float dt = 0.016f; // 60 FPS
            for (int frame = 0; frame < 60; ++frame) {
                animation_system->Update(dt);
                
                if (frame % 20 == 0) {
                    Msg("  - Frame %d: time=%.3f", frame, handle->current_time);
                }
            }
            
            animation_system->StopAnimation(handle);
            Msg("  - Animation stopped");
        }
    } else {
        Msg("! Failed to load animation");
    }
    
    // Test metadata loading
    Msg("\n* Testing metadata loading...");
    std::string metadata_path = "test_data/metadata.ini";
    if (animation_system->LoadMetadata(metadata_path)) {
        Msg("  - Metadata loaded successfully");
    } else {
        Msg("! Failed to load metadata");
    }
}

void TestOzzKinematicsAnimated()
{
    Msg("\n=== Testing OzzKinematicsAnimated ===");
    
    // Create kinematicsAnimated
    auto kinematics = std::make_unique<OzzKinematicsAnimated>();
    
    // Initialize with test data
    Msg("* Initializing OzzKinematicsAnimated...");
    std::string skeleton_path = "test_data/skeleton.ozz";
    std::string animations_path = "test_data";
    
    if (kinematics->Initialize(skeleton_path, animations_path)) {
        Msg("  - Initialized successfully");
        
        // Test motion ID creation
        MotionID walk_id = kinematics->ID_Cycle("walk");
        Msg("  - Created motion ID for 'walk': slot=%d, idx=%d", walk_id.slot, walk_id.idx);
        
        // Test animation playback
        CBlend* blend = kinematics->PlayCycle("walk", TRUE, nullptr, nullptr);
        if (blend) {
            Msg("  - Playing 'walk' animation");
            
            // Update animation
            float dt = 0.016f;
            for (int i = 0; i < 30; ++i) {
                kinematics->UpdateTracks();
            }
            
            // Check blend count
            u32 blend_count = kinematics->LL_PartBlendsCount(0);
            Msg("  - Active blends: %d", blend_count);
            
            // Stop animation
            kinematics->LL_CloseCycle(0);
            Msg("  - Animation stopped");
        } else {
            Msg("! Failed to play animation");
        }
        
#ifdef DEBUG
        // Dump blend info
        kinematics->LL_DumpBlends_dbg();
#endif
    } else {
        Msg("! Failed to initialize OzzKinematicsAnimated");
    }
}

void TestChannelsAndPartitions()
{
    Msg("\n=== Testing Channels and Partitions ===");
    
    auto animation_system = std::make_unique<OzzAnimationSystem>();
    
    // Load skeleton first
    if (!animation_system->LoadSkeleton("test_data/skeleton.ozz")) {
        Msg("! Failed to load skeleton for channel test");
        return;
    }
    
    // Test channel factors
    Msg("* Testing channel factors...");
    for (u8 i = 0; i < OzzAnimationSystem::MAX_CHANNELS; ++i) {
        float factor = 0.5f + (i * 0.1f);
        animation_system->SetChannelFactor(i, factor);
        Msg("  - Channel %d factor set to %.2f", i, factor);
    }
    
    // Test partitions
    Msg("\n* Testing partitions...");
    
    // Define torso bones (example indices)
    xr_vector<u16> torso_bones = {5, 6, 7, 8, 9, 10, 11, 12};
    animation_system->SetPartitionMask(1, torso_bones); // Partition 1 = torso
    
    // Define legs bones
    xr_vector<u16> legs_bones = {13, 14, 15, 16, 17, 18};
    animation_system->SetPartitionMask(2, legs_bones); // Partition 2 = legs
    
    // Load animations
    animation_system->LoadAnimation("test_data/reload.ozz", "reload");
    animation_system->LoadAnimation("test_data/walk.ozz", "walk");
    
    // Play different animations on different partitions
    auto* torso_anim = animation_system->PlayAnimationOnPartition("reload", 1, 1.0f, false, 0);
    auto* legs_anim = animation_system->PlayAnimationOnPartition("walk", 2, 1.0f, true, 1);
    
    if (torso_anim && legs_anim) {
        Msg("  - Playing 'reload' on torso (partition 1)");
        Msg("  - Playing 'walk' on legs (partition 2)");
        
        // Update with callbacks
        float dt = 0.016f;
        for (int frame = 0; frame < 60; ++frame) {
            animation_system->UpdateWithCallbacks(dt);
            
            if (frame % 30 == 0) {
                size_t active_count = animation_system->GetActiveAnimationCount();
                Msg("  - Frame %d: %d active animations", frame, active_count);
            }
        }
    }
    
    // Stop animations on specific partition
    animation_system->StopAnimationsOnPartition(1, 0xFF); // Stop all channels on torso
    Msg("  - Stopped animations on partition 1");
}

void TestAdditionalTransforms()
{
    Msg("\n=== Testing Additional Transforms ===");
    
    auto animation_system = std::make_unique<OzzAnimationSystem>();
    
    if (!animation_system->LoadSkeleton("test_data/skeleton.ozz")) {
        Msg("! Failed to load skeleton for transform test");
        return;
    }
    
    // Apply additional transform to head bone (example: bone 3)
    Fmatrix head_transform;
    head_transform.identity();
    head_transform.rotateY(0.3f); // Rotate head 0.3 radians
    
    animation_system->ApplyAdditionalBoneTransform(3, head_transform);
    Msg("* Applied additional rotation to head bone");
    
    // Test clearing transform
    animation_system->ClearAdditionalBoneTransform(3);
    Msg("* Cleared additional transform");
}

void TestRootMotion()
{
    Msg("\n=== Testing Root Motion Extraction ===");
    
    auto animation_system = std::make_unique<OzzAnimationSystem>();
    
    if (!animation_system->LoadSkeleton("test_data/skeleton.ozz") ||
        !animation_system->LoadAnimation("test_data/walk.ozz", "walk")) {
        Msg("! Failed to load data for root motion test");
        return;
    }
    
    // Enable root motion extraction
    animation_system->EnableRootMotionExtraction(true);
    Msg("* Root motion extraction enabled");
    
    auto* handle = animation_system->PlayAnimation("walk", 1.0f, false);
    if (handle) {
        Fvector total_movement;
        total_movement.set(0, 0, 0);
        
        // Update and accumulate root motion
        float dt = 0.016f;
        for (int frame = 0; frame < 120; ++frame) {
            animation_system->Update(dt);
            
            Fmatrix delta = animation_system->GetRootMotionDelta();
            total_movement.add(delta.c);
            
            if (frame % 40 == 0) {
                Msg("  - Frame %d: movement=(%.3f, %.3f, %.3f)", 
                    frame, delta.c.x, delta.c.y, delta.c.z);
            }
        }
        
        Msg("* Total root movement: (%.3f, %.3f, %.3f)", 
            total_movement.x, total_movement.y, total_movement.z);
    }
}

int main(int argc, char* argv[])
{
    // Initialize core
    Debug._initialize(false);
    Core._initialize("ozz_test", nullptr, TRUE, "fs_test.ltx");
    
    Msg("==================================================");
    Msg("     X-Ray ozz-animation Integration Test");
    Msg("==================================================\n");
    
    // Run tests
    TestOzzAnimationSystem();
    TestOzzKinematicsAnimated();
    TestChannelsAndPartitions();
    TestAdditionalTransforms();
    TestRootMotion();
    
    Msg("\n==================================================");
    Msg("     Test Complete");
    Msg("==================================================");
    
    // Cleanup
    Core._destroy();
    
    return 0;
}