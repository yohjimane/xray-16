#include "../stdafx.h"
#include "../OzzAnimationSystem.h"
#include "xrCore/xrCore.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"

using namespace XRay::Animation;

void TestAnimationPlayback(const std::string& skeleton_file, const std::string& animation_file)
{
    Msg("=== Testing Animation Playback Fix ===");
    Msg("* Skeleton file: %s", skeleton_file.c_str());
    Msg("* Animation file: %s", animation_file.c_str());
    
    // Create animation system
    auto animation_system = std::make_unique<OzzAnimationSystem>();
    
    // Load skeleton
    if (!animation_system->LoadSkeleton(skeleton_file.c_str())) {
        Msg("! Failed to load skeleton. Run converter first:");
        Msg("  xray_to_ozz_converter skeleton stalker_bandit_1.ogf test_skeleton.ozz");
        return;
    }
    
    // Load animation
    if (!animation_system->LoadAnimation(animation_file.c_str(), "test_anim")) {
        Msg("! Failed to load animation. Run converter first:");
        Msg("  xray_to_ozz_converter animation stalker_animation.omf test_animation.ozz test_skeleton.ozz");
        return;
    }
    
    Msg("* Skeleton loaded: %d bones", animation_system->GetBoneCount());
    
    // Show bind pose first
    Msg("\n=== Bind Pose (before animation) ===");
    for (size_t i = 0; i < std::min(size_t(10), animation_system->GetBoneCount()); ++i) {
        Fmatrix transform = animation_system->GetBoneTransform(i);
        shared_str bone_name = animation_system->GetBoneName(i);
        
        Msg("  Bone[%d] '%s' pos: (%.3f, %.3f, %.3f)", 
            i, bone_name.c_str(), transform.c.x, transform.c.y, transform.c.z);
    }
    
    // Play animation
    auto* handle = animation_system->PlayAnimation("test_anim", 1.0f, false);
    if (!handle) {
        Msg("! Failed to play animation");
        return;
    }
    
    Msg("\n=== Animation Playback (after fix) ===");
    
    // Update for several frames
    float dt = 0.033f; // ~30 FPS
    for (int frame = 0; frame < 5; ++frame) {
        animation_system->Update(dt);
        
        Msg("\n* Frame %d (time=%.3f):", frame, handle->current_time);
        
        // Check bone transforms - should NOT be all at (0,0,0) anymore
        for (size_t i = 0; i < std::min(size_t(10), animation_system->GetBoneCount()); ++i) {
            Fmatrix transform = animation_system->GetBoneTransform(i);
            shared_str bone_name = animation_system->GetBoneName(i);
            
            // Check if position is NOT at origin
            bool at_origin = (fabs(transform.c.x) < 0.001f && 
                            fabs(transform.c.y) < 0.001f && 
                            fabs(transform.c.z) < 0.001f);
            
            Msg("  Bone[%d] '%s' pos: (%.3f, %.3f, %.3f) %s", 
                i, bone_name.c_str(), 
                transform.c.x, transform.c.y, transform.c.z,
                at_origin ? "[AT ORIGIN!]" : "[OK]");
        }
    }
    
    // Test direct ozz sampling to verify keyframe data
    Msg("\n=== Direct ozz Sampling Test ===");
    
    // Load animation directly
    ozz::animation::Animation animation;
    {
        ozz::io::File file(animation_file.c_str(), "rb");
        if (file.opened()) {
            ozz::io::IArchive archive(&file);
            archive >> animation;
            
            Msg("* Animation duration: %.3f, tracks: %d", 
                animation.duration(), animation.num_tracks());
        }
    }
    
    // Load skeleton directly
    ozz::animation::Skeleton skeleton;
    {
        ozz::io::File file(skeleton_file.c_str(), "rb");
        if (file.opened()) {
            ozz::io::IArchive archive(&file);
            archive >> skeleton;
        }
    }
    
    // Sample animation
    ozz::animation::SamplingJob::Context context;
    context.Resize(skeleton.num_joints());
    
    xr_vector<ozz::math::SoaTransform> local_transforms(skeleton.num_soa_joints());
    xr_vector<ozz::math::Float4x4> model_transforms(skeleton.num_joints());
    
    ozz::animation::SamplingJob sampling_job;
    sampling_job.animation = &animation;
    sampling_job.context = &context;
    sampling_job.ratio = 0.5f; // Middle of animation
    sampling_job.output = ozz::make_span(local_transforms);
    
    if (sampling_job.Run()) {
        // Convert to model space
        ozz::animation::LocalToModelJob ltm_job;
        ltm_job.skeleton = &skeleton;
        ltm_job.input = ozz::make_span(local_transforms);
        ltm_job.output = ozz::make_span(model_transforms);
        
        if (ltm_job.Run()) {
            Msg("\n* Direct sampling result (ratio=0.5):");
            for (int i = 0; i < std::min(5, skeleton.num_joints()); ++i) {
                const auto& m = model_transforms[i];
                
                // Extract position from transform matrix (4th column)
                float px = ozz::math::GetX(m.cols[3]);
                float py = ozz::math::GetY(m.cols[3]);
                float pz = ozz::math::GetZ(m.cols[3]);
                
                Msg("  Joint[%d] '%s' pos: (%.3f, %.3f, %.3f)", 
                    i, skeleton.joint_names()[i], px, py, pz);
            }
        }
    }
}

int main(int argc, char* argv[])
{
    // Initialize core
    xrDebug::Initialize(argv[0] ? argv[0] : "animation_playback_test");
    Core.Initialize("animation_playback_test", nullptr, FALSE, nullptr);
    
    Msg("==================================================");
    Msg("   Animation Playback Fix Verification");
    Msg("==================================================");
    Msg("This test verifies the fix for Float4x4ToMatrix");
    Msg("that was incorrectly extracting translation from");
    Msg("rotation columns instead of the 4th column.");
    Msg("==================================================\n");
    
    if (argc < 3) {
        Msg("Usage: %s <skeleton.ozz> <animation.ozz>", argv[0]);
        Core._destroy();
        return 1;
    }
    
    TestAnimationPlayback(argv[1], argv[2]);
    
    Msg("\n==================================================");
    Msg("   Test Complete");
    Msg("==================================================");
    
    // Cleanup
    Core._destroy();
    
    return 0;
}