#include "stdafx.h"
#include "AnimationConverter.h"
#include "xrCore/xrCore.h"
#include "xrCore/xr_ini.h"
#include "ozz/animation/offline/skeleton_builder.h"
#include "ozz/animation/offline/animation_builder.h"
#include "ozz/animation/offline/animation_optimizer.h"
#include <fstream>

namespace XRay {
namespace Animation {

void XRayMetadata::Save(const std::string& metadata_path) const {
    CInifile ini(metadata_path.c_str(), false, true, true);

    // Save motion parameters
    for (const auto& [name, params] : motion_params) {
        std::string section = "motion_" + name;
        ini.w_float(section.c_str(), "speed", params.speed);
        ini.w_float(section.c_str(), "power", params.power);
        ini.w_float(section.c_str(), "accrue", params.accrue);
        ini.w_float(section.c_str(), "falloff", params.falloff);
        ini.w_u16(section.c_str(), "bone_or_part", params.bone_or_part);
        ini.w_u8(section.c_str(), "flags", params.flags);

        // Save event markers
        std::string markers_str;
        for (size_t i = 0; i < params.event_markers.size(); ++i) {
            if (i > 0) markers_str += ",";
            markers_str += std::to_string(params.event_markers[i]);
        }
        if (!markers_str.empty()) {
            ini.w_string(section.c_str(), "event_markers", markers_str.c_str());
        }
    }

    // Save IK constraints
    for (const auto& [bone_name, ik] : ik_constraints) {
        std::string section = "ik_" + bone_name;
        ini.w_u32(section.c_str(), "type", static_cast<u32>(ik.type));

        for (int i = 0; i < 3; ++i) {
            std::string limit_section = section + "_limit_" + std::to_string(i);
            ini.w_fvector2(limit_section.c_str(), "limit", ik.limits[i].limit);
            ini.w_float(limit_section.c_str(), "spring_factor", ik.limits[i].spring_factor);
            ini.w_float(limit_section.c_str(), "damping_factor", ik.limits[i].damping_factor);
        }

        ini.w_float(section.c_str(), "break_force", ik.break_force);
        ini.w_float(section.c_str(), "break_torque", ik.break_torque);
        ini.w_float(section.c_str(), "friction", ik.friction);
        ini.w_u32(section.c_str(), "flags", ik.flags);
    }

    // Save physics shapes
    for (const auto& [bone_name, shape] : physics_shapes) {
        std::string section = "physics_" + bone_name;
        ini.w_u32(section.c_str(), "type", static_cast<u32>(shape.type));
        ini.w_float(section.c_str(), "mass", shape.mass);
        ini.w_fvector3(section.c_str(), "center_of_mass", shape.center_of_mass);
        ini.w_u16(section.c_str(), "flags", shape.flags);

        switch (shape.type) {
            case PhysicsShape::stBox:
                ini.w_fvector3(section.c_str(), "size", shape.box.size);
                // TODO: Implement matrix serialization
                // ini.w_fmatrix(section.c_str(), "transform", shape.box.transform);
                break;
            case PhysicsShape::stSphere:
                ini.w_fvector3(section.c_str(), "center", shape.sphere.center);
                ini.w_float(section.c_str(), "radius", shape.sphere.radius);
                break;
            case PhysicsShape::stCylinder:
                ini.w_fvector3(section.c_str(), "center", shape.cylinder.center);
                ini.w_float(section.c_str(), "radius", shape.cylinder.radius);
                ini.w_float(section.c_str(), "height", shape.cylinder.height);
                break;
        }
    }

    ini.save_as(metadata_path.c_str());
}

bool XRayMetadata::Load(const std::string& metadata_path) {
    if (!FS.exist(metadata_path.c_str())) {
        return false;
    }

    CInifile ini(metadata_path.c_str());

    // Load motion parameters
    auto root_it = *ini.sections().begin();
    auto root_end = *ini.sections().end();

    for (; root_it != root_end; ++root_it) {
        shared_str section_name = root_it->Name;
        std::string section_str = section_name.c_str();

        if (section_str.find("motion_") == 0) {
            std::string motion_name = section_str.substr(7);
            MotionParams params;

            params.speed = ini.read_if_exists<float>(section_name, "speed", 1.0f);
            params.power = ini.read_if_exists<float>(section_name, "power", 1.0f);
            params.accrue = ini.read_if_exists<float>(section_name, "accrue", 0.2f);
            params.falloff = ini.read_if_exists<float>(section_name, "falloff", 0.2f);
            params.bone_or_part = ini.read_if_exists<u16>(section_name, "bone_or_part", 0);
            params.flags = ini.read_if_exists<u8>(section_name, "flags", 0);

            // Load event markers
            if (ini.line_exist(section_name, "event_markers")) {
                std::string markers_str = ini.r_string(section_name, "event_markers");
                // Parse comma-separated float values
                std::stringstream ss(markers_str);
                std::string item;
                while (std::getline(ss, item, ',')) {
                    params.event_markers.push_back(std::stof(item));
                }
            }

            motion_params[motion_name] = params;
        }
    }

    return true;
}

void ConverterFactory::RegisterConverter(std::unique_ptr<IFormatConverter> converter) {
    converters_.push_back(std::move(converter));
}

std::unique_ptr<IFormatConverter> ConverterFactory::GetConverter(const std::string& file_path) {
    // Extract file extension
    size_t dot_pos = file_path.find_last_of('.');
    if (dot_pos == std::string::npos) {
        return nullptr;
    }

    std::string extension = file_path.substr(dot_pos);
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

    // Find suitable converter
    for (const auto& converter : converters_) {
        if (converter->CanHandle(extension)) {
            // Note: In a real implementation, we'd clone the converter
            // For now, this is a simplified approach
            return std::unique_ptr<IFormatConverter>(converter.get());
        }
    }

    return nullptr;
}

void ConverterFactory::InitializeDefaultConverters() {
    // Converters will be registered here when implemented
    // RegisterConverter(std::make_unique<OGFConverter>());
    // RegisterConverter(std::make_unique<OMFConverter>());
    // RegisterConverter(std::make_unique<ANMConverter>());
    // RegisterConverter(std::make_unique<SKLConverter>());
}

ConversionValidator::ValidationResult ConversionValidator::ValidateConversion(
    const std::string& original_path,
    const IFormatConverter::ConversionResult& result
) {
    ValidationResult validation_result;

    if (!result.success) {
        validation_result.passed = false;
        validation_result.errors.push_back("Conversion failed: " + result.error_message);
        return validation_result;
    }

    // Validate skeleton
    if (!ValidateSkeletonHierarchy(result.skeleton)) {
        validation_result.passed = false;
        validation_result.errors.push_back("Invalid skeleton hierarchy");
    }

    // Validate animations
    for (const auto& animation : result.animations) {
        if (!ValidateAnimationData(animation)) {
            validation_result.passed = false;
            // fix - need to use string buffers probably (e.g string128)
            string128 buf;
            xr_sprintf(buf, "Invalid animation data for '%s'", animation.name.c_str());
            validation_result.errors.push_back(buf);
        }
    }

    // Compute statistics
    validation_result.stats.joint_count = result.skeleton.num_joints();
    validation_result.stats.animation_count = result.animations.size();

    for (const auto& animation : result.animations) {
        validation_result.stats.total_duration += animation.duration;
        for (const auto& track : animation.tracks) {
            validation_result.stats.keyframe_count += track.translations.size();
            validation_result.stats.keyframe_count += track.rotations.size();
            validation_result.stats.keyframe_count += track.scales.size();
        }
    }

    ComputeCompressionMetrics(result);

    return validation_result;
}

bool ConversionValidator::ValidateSkeletonHierarchy(const ozz::animation::offline::RawSkeleton& skeleton) {
    if (!skeleton.Validate()) {
        return false;
    }

    if (skeleton.num_joints() == 0) {
        return false;
    }

    // Additional validation can be added here
    return true;
}

bool ConversionValidator::ValidateAnimationData(const ozz::animation::offline::RawAnimation& animation) {
    if (!animation.Validate()) {
        return false;
    }

    if (animation.duration <= 0.0f) {
        return false;
    }

    // Additional validation can be added here
    return true;
}

void ConversionValidator::ComputeCompressionMetrics(const IFormatConverter::ConversionResult& result) {
    // Compute compression metrics
    size_t original_size = 0;
    size_t compressed_size = 0;

    for (const auto& animation : result.animations) {
        original_size += animation.size();
        // Compressed size would be computed after building the runtime animation
    }

    // compression_ratio calculation would be done here
}

ozz::math::Transform TransformConverter::XRayToOzz(const Fmatrix& xray_matrix) {
    // Extract translation
    ozz::math::Float3 translation = XRayVecToOzz(xray_matrix.c);

    // Extract rotation (convert to quaternion)
    Fquaternion xray_quat;
    xray_quat.set(xray_matrix);
    ozz::math::Quaternion rotation = XRayQuatToOzz(xray_quat);

    // Extract scale
    Fvector scale_vec;
    scale_vec.x = xray_matrix.i.magnitude();
    scale_vec.y = xray_matrix.j.magnitude();
    scale_vec.z = xray_matrix.k.magnitude();
    ozz::math::Float3 scale = XRayVecToOzz(scale_vec);

    return ozz::math::Transform{translation, rotation, scale};
}

Fmatrix TransformConverter::OzzToXRay(const ozz::math::Transform& ozz_transform) {
    Fmatrix result;
    result.identity();

    // Set translation
    result.c = OzzVecToXRay(ozz_transform.translation);

    // Set rotation
    Fquaternion quat = OzzQuatToXRay(ozz_transform.rotation);
    result.rotation(quat);

    // Apply scale
    Fvector scale = OzzVecToXRay(ozz_transform.scale);
    result.i.mul(scale.x);
    result.j.mul(scale.y);
    result.k.mul(scale.z);

    return result;
}

ozz::math::Quaternion TransformConverter::XRayQuatToOzz(const Fquaternion& xray_quat) {
    return ozz::math::Quaternion(xray_quat.x, xray_quat.y, xray_quat.z, xray_quat.w);
}

Fquaternion TransformConverter::OzzQuatToXRay(const ozz::math::Quaternion& ozz_quat) {
    return Fquaternion().set(ozz_quat.x, ozz_quat.y, ozz_quat.z, ozz_quat.w);
}

ozz::math::Float3 TransformConverter::XRayVecToOzz(const Fvector& xray_vec) {
    return ozz::math::Float3(xray_vec.x, xray_vec.y, xray_vec.z);
}

Fvector TransformConverter::OzzVecToXRay(const ozz::math::Float3& ozz_vec) {
    return Fvector().set(ozz_vec.x, ozz_vec.y, ozz_vec.z);
}

OzzAssetBuilder::BuildResult OzzAssetBuilder::BuildAssets(
    const ozz::animation::offline::RawSkeleton& raw_skeleton,
    const std::vector<ozz::animation::offline::RawAnimation>& raw_animations,
    const XRayMetadata& metadata
) {
    BuildResult result;

    try {
        // Build skeleton
        result.skeleton = BuildSkeleton(raw_skeleton);
        if (!result.skeleton) {
            result.error_message = "Failed to build skeleton";
            return result;
        }

        // Build animations
        result.animations = BuildAnimations(raw_animations, *result.skeleton);
        if (result.animations.size() != raw_animations.size()) {
            result.error_message = "Failed to build all animations";
            return result;
        }

        // Preserve metadata
        result.preserved_metadata = metadata;

        // Validate built assets
        if (!ValidateBuiltAssets(result)) {
            result.error_message = "Built assets validation failed";
            return result;
        }

        result.success = true;

    } catch (const std::exception& e) {
        result.error_message = "Exception during asset building: " + std::string(e.what());
    }

    return result;
}

std::unique_ptr<ozz::animation::Skeleton> OzzAssetBuilder::BuildSkeleton(
    const ozz::animation::offline::RawSkeleton& raw_skeleton
) {
    ozz::animation::offline::SkeletonBuilder builder;
    auto skeleton = builder(raw_skeleton);

    if (!skeleton) {
        return nullptr;
    }

    return std::unique_ptr<ozz::animation::Skeleton>(skeleton.release());
}

std::vector<std::unique_ptr<ozz::animation::Animation>> OzzAssetBuilder::BuildAnimations(
    const std::vector<ozz::animation::offline::RawAnimation>& raw_animations,
    const ozz::animation::Skeleton& skeleton
) {
    std::vector<std::unique_ptr<ozz::animation::Animation>> animations;

    ozz::animation::offline::AnimationBuilder builder;
    ozz::animation::offline::AnimationOptimizer optimizer;

    for (const auto& raw_animation : raw_animations) {
        // Optimize animation first
        ozz::animation::offline::RawAnimation optimized_animation;
        if (!optimizer(raw_animation, skeleton, &optimized_animation)) {
            continue; // Skip if optimization failed
        }

        // Build animation
        auto animation = builder(optimized_animation);
        if (!animation) {
            continue;
        }


        animations.push_back(std::unique_ptr<ozz::animation::Animation>(animation.release()));
    }

    return animations;
}

bool OzzAssetBuilder::ValidateBuiltAssets(const BuildResult& result) {
    if (!result.skeleton) {
        return false;
    }

    // Validate skeleton
    if (result.skeleton->num_joints() == 0) {
        return false;
    }

    // Validate animations
    for (const auto& animation : result.animations) {
        if (!animation) {
            return false;
        }

        if (animation->duration() <= 0.0f) {
            return false;
        }

        // Check joint count compatibility
        if (animation->num_tracks() != result.skeleton->num_joints()) {
            return false;
        }
    }

    return true;
}

} // namespace Animation
} // namespace XRay
