#include "stdafx.h"
#include "AnimationConverter.h"
#include "OGFConverter.h"
#include "OMFConverter.h"
#include "xrCore/xrCore.h"
#include "xrCore/xr_ini.h"
#include "ozz/animation/offline/skeleton_builder.h"
#include "ozz/animation/offline/animation_builder.h"
#include "ozz/animation/offline/animation_optimizer.h"

namespace XRay {
namespace Animation {

void XRayMetadata::Save(const shared_str& metadata_path) const {
    CInifile ini(metadata_path.c_str(), false, true, true);

    // Save motion parameters
    for (const auto& [name, params] : motion_params) {
        string256 section;
        xr_sprintf(section, "motion_%s", name.c_str());
        ini.w_float(section, "speed", params.speed);
        ini.w_float(section, "power", params.power);
        ini.w_float(section, "accrue", params.accrue);
        ini.w_float(section, "falloff", params.falloff);
        ini.w_u16(section, "bone_or_part", params.bone_or_part);
        ini.w_u8(section, "flags", params.flags);

        // Save event markers
        string512 markers_str;
        xr_strcpy(markers_str, "");
        for (size_t i = 0; i < params.event_markers.size(); ++i) {
            if (i > 0) xr_strcat(markers_str, ",");
            string32 num_str;
            xr_sprintf(num_str, "%.3f", params.event_markers[i]);
            xr_strcat(markers_str, num_str);
        }
        if (xr_strlen(markers_str) > 0) {
            ini.w_string(section, "event_markers", markers_str);
        }
    }

    // Save IK constraints
    for (const auto& [bone_name, ik] : ik_constraints) {
        string256 section;
        xr_sprintf(section, "ik_%s", bone_name.c_str());
        ini.w_u32(section, "type", static_cast<u32>(ik.type));

        for (int i = 0; i < 3; ++i) {
            string256 limit_section;
            xr_sprintf(limit_section, "%s_limit_%d", section, i);
            ini.w_fvector2(limit_section, "limit", ik.limits[i].limit);
            ini.w_float(limit_section, "spring_factor", ik.limits[i].spring_factor);
            ini.w_float(limit_section, "damping_factor", ik.limits[i].damping_factor);
        }

        ini.w_float(section, "break_force", ik.break_force);
        ini.w_float(section, "break_torque", ik.break_torque);
        ini.w_float(section, "friction", ik.friction);
        ini.w_u32(section, "flags", ik.flags);
    }

    // Save physics shapes
    for (const auto& [bone_name, shape] : physics_shapes) {
        string256 section;
        xr_sprintf(section, "physics_%s", bone_name.c_str());
        ini.w_u32(section, "type", static_cast<u32>(shape.type));
        ini.w_float(section, "mass", shape.mass);
        ini.w_fvector3(section, "center_of_mass", shape.center_of_mass);
        ini.w_u16(section, "flags", shape.flags);

        switch (shape.type) {
            case PhysicsShape::stBox:
                ini.w_fvector3(section, "size", shape.box.size);
                // TODO: Implement matrix serialization
                // ini.w_fmatrix(section.c_str(), "transform", shape.box.transform);
                break;
            case PhysicsShape::stSphere:
                ini.w_fvector3(section, "center", shape.sphere.center);
                ini.w_float(section, "radius", shape.sphere.radius);
                break;
            case PhysicsShape::stCylinder:
                ini.w_fvector3(section, "center", shape.cylinder.center);
                ini.w_float(section, "radius", shape.cylinder.radius);
                ini.w_float(section, "height", shape.cylinder.height);
                break;
        }
    }

    ini.save_as(metadata_path.c_str());
}

bool XRayMetadata::Load(const shared_str& metadata_path) {
    if (!FS.exist(metadata_path.c_str())) {
        return false;
    }

    CInifile ini(metadata_path.c_str());

    // Load motion parameters
    auto root_it = *ini.sections().begin();
    auto root_end = *ini.sections().end();

    for (; root_it != root_end; ++root_it) {
        shared_str section_name = root_it->Name;
        xr_string section_str = section_name.c_str();

        if (section_str.find("motion_") == 0) {
            xr_string motion_name = section_str.substr(7);
            shared_str motion_name_str(motion_name.c_str());
            MotionParams params;

            params.speed = ini.read_if_exists<float>(section_name, "speed", 1.0f);
            params.power = ini.read_if_exists<float>(section_name, "power", 1.0f);
            params.accrue = ini.read_if_exists<float>(section_name, "accrue", 0.2f);
            params.falloff = ini.read_if_exists<float>(section_name, "falloff", 0.2f);
            params.bone_or_part = ini.read_if_exists<u16>(section_name, "bone_or_part", 0);
            params.flags = ini.read_if_exists<u8>(section_name, "flags", 0);

            // Load event markers
            if (ini.line_exist(section_name, "event_markers")) {
                string512 markers_str;
                xr_strcpy(markers_str, ini.r_string(section_name, "event_markers"));
                // Parse comma-separated float values
                char* token = strtok(markers_str, ",");
                while (token != nullptr) {
                    params.event_markers.push_back(static_cast<float>(atof(token)));
                    token = strtok(nullptr, ",");
                }
            }

            motion_params[motion_name_str] = params;
        }
    }

    return true;
}

void ConverterFactory::RegisterConverter(xr_unique_ptr<IFormatConverter> converter) {
    converters_.push_back(std::move(converter));
}

xr_unique_ptr<IFormatConverter> ConverterFactory::GetConverter(const shared_str& file_path) {
    // Extract file extension
    pcstr file_path_str = file_path.c_str();
    pcstr dot_pos_ptr = strrchr(file_path_str, '.');
    if (dot_pos_ptr == nullptr) {
        return nullptr;
    }

    string64 extension;
    xr_strcpy(extension, dot_pos_ptr);
    xr_strlwr(extension);
    shared_str ext_str(extension);

    // Find suitable converter
    for (const auto& converter : converters_) {
        if (converter->CanHandle(ext_str)) {
            // Note: In a real implementation, we'd clone the converter
            // For now, this is a simplified approach
            return xr_unique_ptr<IFormatConverter>(converter.get());
        }
    }

    return nullptr;
}

void ConverterFactory::InitializeDefaultConverters() {
    // Register converters
    RegisterConverter(xr_unique_ptr<IFormatConverter>(xr_new<OGFConverter>()));
    RegisterConverter(xr_unique_ptr<IFormatConverter>(xr_new<OMFConverter>()));
    // RegisterConverter(xr_make_unique<ANMConverter>());
    // RegisterConverter(xr_make_unique<SKLConverter>());
}

ConversionValidator::ValidationResult ConversionValidator::ValidateConversion(
    const shared_str& original_path,
    const IFormatConverter::ConversionResult& result
) {
    ValidationResult validation_result;

    if (!result.success) {
        validation_result.passed = false;
        string256 error_buf;
        xr_sprintf(error_buf, "Conversion failed: %s", result.error_message.c_str());
        validation_result.errors.push_back(shared_str(error_buf));
        return validation_result;
    }

    // Validate skeleton
    if (!ValidateSkeletonHierarchy(result.skeleton)) {
        validation_result.passed = false;
        validation_result.errors.push_back(shared_str("Invalid skeleton hierarchy"));
    }

    // Validate animations
    for (const auto& animation : result.animations) {
        if (!ValidateAnimationData(animation)) {
            validation_result.passed = false;
            // fix - need to use string buffers probably (e.g string128)
            string128 buf;
            xr_sprintf(buf, "Invalid animation data for '%s'", animation.name.c_str());
            validation_result.errors.push_back(shared_str(buf));
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
    const xr_vector<ozz::animation::offline::RawAnimation>& raw_animations,
    const XRayMetadata& metadata
) {
    BuildResult result;

    try {
        // Build skeleton
        result.skeleton = BuildSkeleton(raw_skeleton);
        if (!result.skeleton) {
            result.error_message = shared_str("Failed to build skeleton");
            return result;
        }

        // Build animations
        result.animations = BuildAnimations(raw_animations, *result.skeleton);
        if (result.animations.size() != raw_animations.size()) {
            result.error_message = shared_str("Failed to build all animations");
            return result;
        }

        // Preserve metadata
        result.preserved_metadata = metadata;

        // Validate built assets
        if (!ValidateBuiltAssets(result)) {
            result.error_message = shared_str("Built assets validation failed");
            return result;
        }

        result.success = true;

    } catch (...) {
        result.error_message = "Exception during asset building";
    }

    return result;
}

xr_unique_ptr<ozz::animation::Skeleton> OzzAssetBuilder::BuildSkeleton(
    const ozz::animation::offline::RawSkeleton& raw_skeleton
) {
    ozz::animation::offline::SkeletonBuilder builder;
    auto skeleton = builder(raw_skeleton);

    if (!skeleton) {
        return nullptr;
    }

    return xr_unique_ptr<ozz::animation::Skeleton>(skeleton.release());
}

xr_vector<xr_unique_ptr<ozz::animation::Animation>> OzzAssetBuilder::BuildAnimations(
    const xr_vector<ozz::animation::offline::RawAnimation>& raw_animations,
    const ozz::animation::Skeleton& skeleton
) {
    xr_vector<xr_unique_ptr<ozz::animation::Animation>> animations;

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


        animations.push_back(xr_unique_ptr<ozz::animation::Animation>(animation.release()));
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
