#include "stdafx.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/blending_job.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/io/archive.h"
#include "xrCore/FS.h"

namespace XRay {
namespace Animation {
namespace Tests {

class OzzBasicTest {
public:
    OzzBasicTest() {
        Msg("* Initializing OzzBasicTest");
    }
    
    ~OzzBasicTest() {
        Msg("* Destroying OzzBasicTest");
    }
    
    bool Initialize() {
        // Test 1: Load a skeleton
        if (!LoadTestSkeleton()) {
            Msg("! Failed to load test skeleton");
            return false;
        }
        
        // Test 2: Load an animation
        if (!LoadTestAnimation()) {
            Msg("! Failed to load test animation");
            return false;
        }
        
        // Test 3: Allocate runtime buffers
        if (!AllocateBuffers()) {
            Msg("! Failed to allocate runtime buffers");
            return false;
        }
        
        Msg("* OzzBasicTest initialized successfully");
        return true;
    }
    
    void Update(float dt) {
        // Update animation time
        time_ratio_ += dt * playback_speed_ / animation_duration_;
        if (time_ratio_ > 1.0f) {
            if (is_looping_) {
                time_ratio_ = fmod(time_ratio_, 1.0f);
            } else {
                time_ratio_ = 1.0f;
                is_playing_ = false;
            }
        }
        
        // Sample animation
        if (!SampleAnimation()) {
            Msg("! Animation sampling failed");
            return;
        }
        
        // Convert to model space
        if (!ComputeModelTransforms()) {
            Msg("! Model transform computation failed");
            return;
        }
        
        // Convert to X-Ray matrices
        ConvertToXRayMatrices();
        
        // Log some bone positions for debugging
        if (frame_counter_ % 60 == 0) {  // Every 60 frames
            LogBonePositions();
        }
        frame_counter_++;
    }
    
    const xr_vector<Fmatrix>& GetBoneMatrices() const {
        return bone_matrices_;
    }
    
    bool RunTests() {
        Msg("* Running OzzBasicTest suite...");
        
        // Test skeleton properties
        TestSkeletonProperties();
        
        // Test animation properties
        TestAnimationProperties();
        
        // Test sampling at different time ratios
        TestSamplingAtDifferentTimes();
        
        // Test transform conversion
        TestTransformConversion();
        
        Msg("* OzzBasicTest suite completed");
        return true;
    }
    
private:
    // ozz data
    ozz::unique_ptr<ozz::animation::Skeleton> skeleton_;
    ozz::unique_ptr<ozz::animation::Animation> animation_;
    ozz::animation::SamplingJob::Context sampling_context_;
    
    // Runtime buffers
    xr_vector<ozz::math::SoaTransform> local_transforms_;
    xr_vector<ozz::math::Float4x4> model_transforms_;
    xr_vector<Fmatrix> bone_matrices_;
    
    // Animation state
    float time_ratio_ = 0.0f;
    float playback_speed_ = 1.0f;
    float animation_duration_ = 1.0f;
    bool is_playing_ = true;
    bool is_looping_ = true;
    u32 frame_counter_ = 0;
    
    bool LoadTestSkeleton() {
        // For testing, create a simple skeleton programmatically
        // In real implementation, load from file
        
        Msg("* Creating test skeleton...");
        
        // Create a simple 3-bone skeleton: Root -> Child1 -> Child2
        ozz::animation::offline::RawSkeleton raw_skeleton;
        raw_skeleton.roots.resize(1);  // One root
        
        // Root bone
        auto& root = raw_skeleton.roots[0];
        root.name = "Root";
        root.transform.translation = ozz::math::Float3::zero();
        root.transform.rotation = ozz::math::Quaternion::identity();
        root.transform.scale = ozz::math::Float3::one();
        
        // Child 1
        root.children.resize(1);
        auto& child1 = root.children[0];
        child1.name = "Bone1";
        child1.transform.translation = ozz::math::Float3(0.0f, 1.0f, 0.0f);
        child1.transform.rotation = ozz::math::Quaternion::identity();
        child1.transform.scale = ozz::math::Float3::one();
        
        // Child 2
        child1.children.resize(1);
        auto& child2 = child1.children[0];
        child2.name = "Bone2";
        child2.transform.translation = ozz::math::Float3(0.0f, 1.0f, 0.0f);
        child2.transform.rotation = ozz::math::Quaternion::identity();
        child2.transform.scale = ozz::math::Float3::one();
        
        // Build runtime skeleton
        ozz::animation::offline::SkeletonBuilder builder;
        auto skeleton = builder(raw_skeleton);
        if (!skeleton) {
            Msg("! Failed to build skeleton");
            return false;
        }
        
        skeleton_.reset(skeleton.release());
        
        Msg("* Test skeleton created: %d joints", skeleton_->num_joints());
        
        // Initialize sampling context
        sampling_context_.Resize(skeleton_->num_joints());
        
        return true;
    }
    
    bool LoadTestAnimation() {
        // Create a simple rotation animation
        Msg("* Creating test animation...");
        
        ozz::animation::offline::RawAnimation raw_animation;
        raw_animation.duration = 2.0f;  // 2 second animation
        raw_animation.tracks.resize(skeleton_->num_joints());
        
        // Animate bone rotations
        for (int i = 0; i < skeleton_->num_joints(); ++i) {
            auto& track = raw_animation.tracks[i];
            
            // Add rotation keys
            track.rotations.resize(3);  // Start, middle, end
            
            // Start key (0 degrees)
            track.rotations[0].time = 0.0f;
            track.rotations[0].value = ozz::math::Quaternion::identity();
            
            // Middle key (180 degrees around Y)
            track.rotations[1].time = 1.0f;
            track.rotations[1].value = ozz::math::Quaternion::FromAxisAngle(
                ozz::math::Float3(0.0f, 1.0f, 0.0f), ozz::math::kPi);
            
            // End key (360 degrees = back to start)
            track.rotations[2].time = 2.0f;
            track.rotations[2].value = ozz::math::Quaternion::identity();
            
            // Keep translation constant
            track.translations.resize(1);
            track.translations[0].time = 0.0f;
            track.translations[0].value = (i == 0) ? 
                ozz::math::Float3::zero() : 
                ozz::math::Float3(0.0f, 1.0f, 0.0f);
            
            // Keep scale constant
            track.scales.resize(1);
            track.scales[0].time = 0.0f;
            track.scales[0].value = ozz::math::Float3::one();
        }
        
        // Build runtime animation
        ozz::animation::offline::AnimationBuilder builder;
        auto animation = builder(raw_animation);
        if (!animation) {
            Msg("! Failed to build animation");
            return false;
        }
        
        animation_.reset(animation.release());
        animation_duration_ = animation_->duration();
        
        Msg("* Test animation created: %.2f seconds", animation_duration_);
        
        return true;
    }
    
    bool AllocateBuffers() {
        const int num_joints = skeleton_->num_joints();
        const int num_soa_joints = skeleton_->num_soa_joints();
        
        // Allocate transform buffers
        local_transforms_.resize(num_soa_joints);
        model_transforms_.resize(num_joints);
        bone_matrices_.resize(num_joints);
        
        Msg("* Allocated buffers for %d joints (%d SoA)", num_joints, num_soa_joints);
        
        return true;
    }
    
    bool SampleAnimation() {
        ozz::animation::SamplingJob sampling_job;
        sampling_job.animation = animation_.get();
        sampling_job.context = &sampling_context_;
        sampling_job.ratio = time_ratio_;
        sampling_job.output = ozz::make_span(local_transforms_);
        
        return sampling_job.Run();
    }
    
    bool ComputeModelTransforms() {
        ozz::animation::LocalToModelJob ltm_job;
        ltm_job.skeleton = skeleton_.get();
        ltm_job.input = ozz::make_span(local_transforms_);
        ltm_job.output = ozz::make_span(model_transforms_);
        
        return ltm_job.Run();
    }
    
    void ConvertToXRayMatrices() {
        for (size_t i = 0; i < model_transforms_.size(); ++i) {
            bone_matrices_[i] = Float4x4ToMatrix(model_transforms_[i]);
        }
    }
    
    Fmatrix Float4x4ToMatrix(const ozz::math::Float4x4& ozz_matrix) const {
        Fmatrix result;
        
        // ozz uses column-major with SIMD types, X-Ray uses row-major
        // Extract values using ozz::math::GetX/Y/Z/W functions
        result._11 = ozz::math::GetX(ozz_matrix.cols[0]); 
        result._12 = ozz::math::GetX(ozz_matrix.cols[1]);
        result._13 = ozz::math::GetX(ozz_matrix.cols[2]); 
        result._14 = ozz::math::GetX(ozz_matrix.cols[3]);
        
        result._21 = ozz::math::GetY(ozz_matrix.cols[0]); 
        result._22 = ozz::math::GetY(ozz_matrix.cols[1]);
        result._23 = ozz::math::GetY(ozz_matrix.cols[2]); 
        result._24 = ozz::math::GetY(ozz_matrix.cols[3]);
        
        result._31 = ozz::math::GetZ(ozz_matrix.cols[0]); 
        result._32 = ozz::math::GetZ(ozz_matrix.cols[1]);
        result._33 = ozz::math::GetZ(ozz_matrix.cols[2]); 
        result._34 = ozz::math::GetZ(ozz_matrix.cols[3]);
        
        result._41 = ozz::math::GetW(ozz_matrix.cols[0]); 
        result._42 = ozz::math::GetW(ozz_matrix.cols[1]);
        result._43 = ozz::math::GetW(ozz_matrix.cols[2]); 
        result._44 = ozz::math::GetW(ozz_matrix.cols[3]);
        
        return result;
    }
    
    void LogBonePositions() {
        Msg("* Animation time: %.2f", time_ratio_);
        
        for (size_t i = 0; i < bone_matrices_.size(); ++i) {
            const Fmatrix& m = bone_matrices_[i];
            Fvector pos;
            pos.set(m._41, m._42, m._43);
            
            Msg("  Bone[%d]: pos=(%.3f, %.3f, %.3f)", 
                i, pos.x, pos.y, pos.z);
        }
    }
    
    void TestSkeletonProperties() {
        Msg("* Testing skeleton properties:");
        Msg("  - Joint count: %d", skeleton_->num_joints());
        Msg("  - SoA joint count: %d", skeleton_->num_soa_joints());
        
        auto joint_names = skeleton_->joint_names();
        for (int i = 0; i < skeleton_->num_joints(); ++i) {
            Msg("  - Joint[%d]: %s", i, joint_names[i]);
        }
    }
    
    void TestAnimationProperties() {
        Msg("* Testing animation properties:");
        Msg("  - Duration: %.2f seconds", animation_->duration());
        Msg("  - Tracks: %d", animation_->num_tracks());
    }
    
    void TestSamplingAtDifferentTimes() {
        Msg("* Testing sampling at different time ratios:");
        
        float test_ratios[] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
        
        for (float ratio : test_ratios) {
            time_ratio_ = ratio;
            
            if (SampleAnimation() && ComputeModelTransforms()) {
                ConvertToXRayMatrices();
                
                // Check root bone transform
                const Fmatrix& root = bone_matrices_[0];
                Fvector pos;
                pos.set(root._41, root._42, root._43);
                
                Msg("  - Ratio %.2f: Root pos=(%.3f, %.3f, %.3f)",
                    ratio, pos.x, pos.y, pos.z);
            }
        }
        
        time_ratio_ = 0.0f;  // Reset
    }
    
    void TestTransformConversion() {
        Msg("* Testing transform conversion:");
        
        // Test identity matrix
        ozz::math::Float4x4 identity = ozz::math::Float4x4::identity();
        Fmatrix xray_identity = Float4x4ToMatrix(identity);
        
        bool is_identity = true;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float expected = (i == j) ? 1.0f : 0.0f;
                float actual = xray_identity.m[i][j];
                
                if (fabs(actual - expected) > 0.0001f) {
                    is_identity = false;
                    break;
                }
            }
        }
        
        Msg("  - Identity conversion: %s", is_identity ? "PASSED" : "FAILED");
    }
};

// Test runner function
void RunOzzBasicTests() {
    Msg("=== Running ozz-animation Basic Tests ===");
    
    OzzBasicTest test;
    
    // Initialize
    if (!test.Initialize()) {
        Msg("! Test initialization failed");
        return;
    }
    
    // Run unit tests
    test.RunTests();
    
    // Simulate a few update frames
    Msg("* Simulating animation updates...");
    for (int i = 0; i < 10; ++i) {
        test.Update(0.033f);  // ~30 FPS
    }
    
    Msg("=== ozz-animation Basic Tests Completed ===");
}

} // namespace Tests
} // namespace Animation
} // namespace XRay