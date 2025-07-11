#pragma once

#include "xrCore/xrCore.h"
#include "xrCore/Animation/SkeletonMotions.hpp"
#include "xrCore/FMesh.hpp"
#include "ozz/animation/offline/raw_skeleton.h"
#include "ozz/animation/offline/raw_animation.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/animation/runtime/animation.h"
#include "xrCommon/xr_unordered_map.h"
#include "xrCommon/xr_smart_pointers.h"
#include <memory>

namespace XRay {
namespace Animation {

struct XRayMetadata {
    struct MotionParams {
        float speed = 1.0f;
        float power = 1.0f;  
        float accrue = 0.2f;
        float falloff = 0.2f;
        u16 bone_or_part = 0;
        u8 flags = 0;
        xr_vector<float> event_markers;
    };
    
    struct IKConstraints {
        enum JointType { jtRigid=0, jtCloth=1, jtJoint=2, jtWheel=3, jtSlider=5 };
        JointType type = jtRigid;
        
        struct JointLimit {
            Fvector2 limit;
            float spring_factor = 1.0f;
            float damping_factor = 1.0f;
        };
        JointLimit limits[3];
        
        float break_force = 0.0f;
        float break_torque = 0.0f;
        float friction = 0.0f;
        u32 flags = 0;
    };
    
    struct PhysicsShape {
        enum ShapeType { stNone=0, stBox=1, stSphere=2, stCylinder=3 };
        ShapeType type = stNone;
        
        union {
            struct { Fvector size; Fmatrix transform; } box;
            struct { Fvector center; float radius; } sphere;
            struct { Fvector center; float radius; float height; } cylinder;
        };
        
        float mass = 1.0f;
        Fvector center_of_mass = {0,0,0};
        u16 flags = 0;
    };
    
    xr_unordered_map<shared_str, MotionParams> motion_params;
    xr_unordered_map<shared_str, IKConstraints> ik_constraints;
    xr_unordered_map<shared_str, PhysicsShape> physics_shapes;
    
    void Save(const shared_str& metadata_path) const;
    bool Load(const shared_str& metadata_path);
};

class IFormatConverter {
public:
    virtual ~IFormatConverter() = default;
    
    struct ConversionResult {
        ozz::animation::offline::RawSkeleton skeleton;
        xr_vector<ozz::animation::offline::RawAnimation> animations;
        XRayMetadata metadata;
        shared_str error_message;
        bool success = false;
    };
    
    virtual ConversionResult Convert(const shared_str& input_path) = 0;
    virtual bool CanHandle(const shared_str& file_extension) = 0;
};

class ConverterFactory {
    xr_vector<xr_unique_ptr<IFormatConverter>> converters_;
    
public:
    void RegisterConverter(xr_unique_ptr<IFormatConverter> converter);
    xr_unique_ptr<IFormatConverter> GetConverter(const shared_str& file_path);
    
    void InitializeDefaultConverters();
};

class ConversionValidator {
public:
    struct ValidationResult {
        bool passed = true;
        xr_vector<shared_str> errors;
        xr_vector<shared_str> warnings;
        
        struct Statistics {
            size_t joint_count = 0;
            size_t animation_count = 0;
            size_t keyframe_count = 0;
            float total_duration = 0.0f;
            float compression_ratio = 0.0f;
        } stats;
    };
    
    ValidationResult ValidateConversion(
        const shared_str& original_path,
        const IFormatConverter::ConversionResult& result
    );
    
private:
    bool ValidateSkeletonHierarchy(const ozz::animation::offline::RawSkeleton& skeleton);
    bool ValidateAnimationData(const ozz::animation::offline::RawAnimation& animation);
    void ComputeCompressionMetrics(const IFormatConverter::ConversionResult& result);
};

class TransformConverter {
public:
    static ozz::math::Transform XRayToOzz(const Fmatrix& xray_matrix);
    static Fmatrix OzzToXRay(const ozz::math::Transform& ozz_transform);
    static ozz::math::Quaternion XRayQuatToOzz(const Fquaternion& xray_quat);
    static Fquaternion OzzQuatToXRay(const ozz::math::Quaternion& ozz_quat);
    static ozz::math::Float3 XRayVecToOzz(const Fvector& xray_vec);
    static Fvector OzzVecToXRay(const ozz::math::Float3& ozz_vec);
};

class OzzAssetBuilder {
public:
    struct BuildResult {
        xr_unique_ptr<ozz::animation::Skeleton> skeleton;
        xr_vector<xr_unique_ptr<ozz::animation::Animation>> animations;
        XRayMetadata preserved_metadata;
        bool success = false;
        shared_str error_message;
    };
    
    BuildResult BuildAssets(
        const ozz::animation::offline::RawSkeleton& raw_skeleton,
        const xr_vector<ozz::animation::offline::RawAnimation>& raw_animations,
        const XRayMetadata& metadata
    );
    
private:
    xr_unique_ptr<ozz::animation::Skeleton> BuildSkeleton(
        const ozz::animation::offline::RawSkeleton& raw_skeleton
    );
    
    xr_vector<xr_unique_ptr<ozz::animation::Animation>> BuildAnimations(
        const xr_vector<ozz::animation::offline::RawAnimation>& raw_animations,
        const ozz::animation::Skeleton& skeleton
    );
    
    bool ValidateBuiltAssets(const BuildResult& result);
};

} // namespace Animation
} // namespace XRay