#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <vector>

#ifndef PROJECT_ROOT
#error "PROJECT_ROOT compile definition must be provided"
#endif

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/wait.h>
#endif

#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/animation/runtime/skeleton_utils.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/transform.h"


namespace fs = std::filesystem;

namespace
{

fs::path ProjectRoot()
{
    static const fs::path root = fs::path(PROJECT_ROOT);
    return root;
}

fs::path TestArtifactsDir()
{
    return ProjectRoot() / "asset_tests" / "test_outputs";
}

fs::path SkeletonInputPath()
{
    return ProjectRoot() / "res" / "testdata" / "stalker_hero_1.ogf";
}

fs::path AnimationInputPath()
{
    return ProjectRoot() / "res" / "testdata" / "critical_hit_grup_1.omf";
}

fs::path SkeletonOutputPath()
{
    return TestArtifactsDir() / "stalker_hero_bind_pose.ozz";
}

fs::path SkeletonCsvPath()
{
    return TestArtifactsDir() / "stalker_hero_bind_pose.csv";
}

fs::path BaselineDir()
{
    return ProjectRoot() / "src" / "xrAnimation" / "tests" / "baselines";
}

fs::path AnimationOutputPath()
{
    return TestArtifactsDir() / "critical_hit_grup_1.ozz";
}

fs::path AnimationMetadataPath()
{
    return TestArtifactsDir() / "critical_hit_grup_1.json";
}

fs::path SingleAnimationOutputPath()
{
    return TestArtifactsDir() / "critical_hit_grup_1_single.ozz";
}

fs::path MeshOutputPath()
{
    return TestArtifactsDir() / "stalker_hero_mesh.ozz";
}

const std::array<const char*, 4> kExpectedMultiMotionNames = {{
    "norm_2_critical_hit_hend_left_0",
    "norm_2_critical_hit_hend_right_0",
    "norm_2_critical_hit_torso_0",
    "norm_2_critical_hit_torso_1",
}};

std::string ReadString(ozz::io::IArchive& archive)
{
    uint32_t length = 0;
    archive >> length;
    std::string value;
    value.resize(length);
    if (length > 0)
        archive >> ozz::io::MakeArray(value.data(), length);
    return value;
}

bool ReadSerializedMotionMetadata(ozz::io::IArchive& archive, std::string* motion_name)
{
    std::string name = ReadString(archive);

    uint32_t flags = 0;
    archive >> flags;

    uint16_t bone_or_part = 0;
    archive >> bone_or_part;

    uint16_t motion_id = 0;
    archive >> motion_id;

    float speed = 0.f;
    float power = 0.f;
    float accrue = 0.f;
    float falloff = 0.f;
    archive >> speed;
    archive >> power;
    archive >> accrue;
    archive >> falloff;

    uint32_t mark_count = 0;
    archive >> mark_count;
    for (uint32_t mark_index = 0; mark_index < mark_count; ++mark_index)
    {
        std::string mark_name = ReadString(archive);

        uint32_t interval_count = 0;
        archive >> interval_count;
        for (uint32_t interval_index = 0; interval_index < interval_count; ++interval_index)
        {
            float start = 0.f;
            float end = 0.f;
            archive >> start;
            archive >> end;
        }
    }

    if (motion_name)
        *motion_name = std::move(name);

    return true;
}

bool LoadAnimationByName(const fs::path& path,
                         const std::string& motion_name,
                         ozz::animation::Animation& animation_out)
{
    ozz::io::File file(path.string().c_str(), "rb");
    if (!file.opened())
    {
        std::cerr << "failed to open animation archive: " << path << std::endl;
        return false;
    }

    ozz::io::IArchive archive(&file);

    if (archive.TestTag<ozz::animation::Animation>())
    {
        ozz::animation::Animation animation;
        archive >> animation;
        const char* name = animation.name();
        const std::string actual_name = name ? name : std::string();
        if (!motion_name.empty() && actual_name != motion_name)
        {
            std::cerr << "animation '" << motion_name
                      << "' not found in single-animation archive" << std::endl;
            return false;
        }

        animation_out = std::move(animation);
        return true;
    }

    file.Seek(0, ozz::io::File::kSet);
    archive = ozz::io::IArchive(&file);

    uint32_t animation_count = 0;
    archive >> animation_count;

    bool found = false;
    for (uint32_t i = 0; i < animation_count; ++i)
    {
        ozz::animation::Animation animation;
        archive >> animation;

        std::string metadata_name;
        ReadSerializedMotionMetadata(archive, &metadata_name);

        if (!found)
        {
            const char* animation_name_cstr = animation.name();
            const std::string animation_name = animation_name_cstr ? animation_name_cstr : std::string();
            if (animation_name == motion_name || metadata_name == motion_name)
            {
                animation_out = std::move(animation);
                found = true;
            }
        }
    }

    if (!found)
    {
        std::cerr << "animation '" << motion_name << "' not found in archive" << std::endl;
    }

    return found;
}

std::string QuoteForShell(const std::string& value)
{
#ifdef _WIN32
    std::string quoted = "\"";
    for (char ch : value)
    {
        if (ch == '\\' || ch == '"')
            quoted.push_back('\\');
        quoted.push_back(ch);
    }
    quoted.push_back('"');
    return quoted;
#else
    std::string quoted;
    quoted.reserve(value.size() + 2);
    quoted.push_back('\'');
    for (char ch : value)
    {
        if (ch == '\'')
            quoted.append("'\\''");
        else
            quoted.push_back(ch);
    }
    quoted.push_back('\'');
    return quoted;
#endif
}

fs::path ResolveBinary(const std::string& executable_name)
{
    const fs::path build_bin = ProjectRoot() / "ozz_utils" / "bin";

    const std::array<fs::path, 2> candidates = {
        build_bin / "Debug" / executable_name,
        build_bin / executable_name};

    for (const auto& path : candidates)
    {
        if (fs::exists(path))
            return path;
    }

    std::ostringstream oss;
    oss << "Unable to locate binary '" << executable_name << "'. Checked:";
    for (const auto& candidate : candidates)
        oss << '\n' << "  " << candidate.string();
    throw std::runtime_error(oss.str());
}

fs::path ResolveConverterBinary()
{
#ifdef _WIN32
    return ResolveBinary("xray_to_ozz_converter.exe");
#else
    return ResolveBinary("xray_to_ozz_converter");
#endif
}

fs::path ResolveViewerBinary()
{
#ifdef _WIN32
    return ResolveBinary("ozz_animation_viewer.exe");
#else
    return ResolveBinary("ozz_animation_viewer");
#endif
}

std::string BuildCommand(const fs::path& binary,
                         const std::vector<std::string>& args)
{
    const fs::path binary_dir = binary.parent_path();

#ifdef _WIN32
    std::string command = QuoteForShell(binary.string());
    for (const std::string& arg : args)
    {
        command.push_back(' ');
        command.append(QuoteForShell(arg));
    }
    return command;
#else
    std::string command = "LD_LIBRARY_PATH=";
    command.append(QuoteForShell(binary_dir.string()));
    command.push_back(' ');
    command.append(QuoteForShell(binary.string()));
    for (const std::string& arg : args)
    {
        command.push_back(' ');
        command.append(QuoteForShell(arg));
    }
    return command;
#endif
}

int ExecuteCommand(const fs::path& binary,
                   const std::vector<std::string>& args)
{
    const std::string command = BuildCommand(binary, args);
    const int result = std::system(command.c_str());
    if (result == -1)
        return -1;

#ifdef _WIN32
    return result;
#else
    if (WIFEXITED(result))
        return WEXITSTATUS(result);
    if (WIFSIGNALED(result))
        return 128 + WTERMSIG(result);
    return result;
#endif
}

int ExecuteConverterCommand(const std::vector<std::string>& args)
{
    return ExecuteCommand(ResolveConverterBinary(), args);
}

bool ConvertSkeleton(bool force)
{
    const fs::path output_dir = TestArtifactsDir();
    const fs::path output_file = SkeletonOutputPath();

    std::error_code ec;
    fs::create_directories(output_dir, ec);

    if (force && fs::exists(output_file))
        fs::remove(output_file);

    if (!force && fs::exists(output_file))
        return true;

    std::vector<std::string> args = {
        "skeleton",
        SkeletonInputPath().string(),
        output_file.string(),
        "--dump-bind",
        SkeletonCsvPath().string()};

    const int exit_code = ExecuteConverterCommand(args);
    if (exit_code != 0)
    {
        std::cerr << "xray_to_ozz_converter returned exit code " << exit_code << std::endl;
        return false;
    }

    return fs::exists(output_file);
}

bool ConvertMesh(bool force)
{
    const fs::path output_dir = TestArtifactsDir();
    const fs::path output_file = MeshOutputPath();

    std::error_code ec;
    fs::create_directories(output_dir, ec);

    if (force && fs::exists(output_file))
        fs::remove(output_file);

    if (!force && fs::exists(output_file))
        return true;

    std::vector<std::string> args = {
        "mesh",
        SkeletonInputPath().string(),
        output_file.string(),
    };

    const int exit_code = ExecuteConverterCommand(args);
    if (exit_code != 0)
    {
        std::cerr << "xray_to_ozz_converter returned exit code " << exit_code << std::endl;
        return false;
    }

    return fs::exists(output_file);
}

bool EnsureSkeletonGenerated()
{
    static bool cached = false;
    static bool status = false;
    if (!cached)
    {
        status = ConvertSkeleton(false);
        cached = true;
    }
    return status;
}

bool ConvertAnimation(bool force)
{
    if (!EnsureSkeletonGenerated())
        return false;

    const fs::path output_file = AnimationOutputPath();
    std::error_code ec;
    fs::create_directories(TestArtifactsDir(), ec);

    if (force && fs::exists(output_file))
        fs::remove(output_file);

    if (!force && fs::exists(output_file))
        return true;

    std::vector<std::string> args = {
        "animation",
        AnimationInputPath().string(),
        output_file.string(),
        SkeletonInputPath().string(),
        "--metadata",
        AnimationMetadataPath().string()};

    const int exit_code = ExecuteConverterCommand(args);
    if (exit_code != 0)
    {
        std::cerr << "animation conversion failed with exit code " << exit_code << std::endl;
        return false;
    }

    return fs::exists(output_file);
}

bool ConvertSpecificMotion(const std::string& motion_name, bool force)
{
    if (!EnsureSkeletonGenerated())
        return false;

    const fs::path output_file = SingleAnimationOutputPath();
    std::error_code ec;
    fs::create_directories(TestArtifactsDir(), ec);

    if (force && fs::exists(output_file))
        fs::remove(output_file);

    if (!force && fs::exists(output_file))
        return true;

    std::vector<std::string> args = {
        "animation",
        AnimationInputPath().string(),
        output_file.string(),
        SkeletonInputPath().string(),
        "--motion",
        motion_name};

    const int exit_code = ExecuteConverterCommand(args);
    if (exit_code != 0)
    {
        std::cerr << "animation conversion for motion '" << motion_name
                  << "' failed with exit code " << exit_code << std::endl;
        return false;
    }

    return fs::exists(output_file);
}

template <typename T>
bool LoadOzz(const fs::path& path, T& object)
{
    ozz::io::File file(path.string().c_str(), "rb");
    if (!file.opened())
    {
        std::cerr << "failed to open ozz archive: " << path << std::endl;
        return false;
    }

    ozz::io::IArchive archive(&file);

    if constexpr (std::is_same_v<T, ozz::animation::Animation>)
    {
        if (archive.TestTag<ozz::animation::Animation>())
        {
            archive >> object;
            return true;
        }

        file.Seek(0, ozz::io::File::kSet);
        archive = ozz::io::IArchive(&file);

        uint32_t animation_count = 0;
        archive >> animation_count;
        if (animation_count == 0)
        {
            std::cerr << "animation archive contains no animations" << std::endl;
            return false;
        }

        archive >> object;
        ReadSerializedMotionMetadata(archive, nullptr);
        return true;
    }
    else
    {
        archive >> object;
        return true;
    }
}

[[maybe_unused]] int FindJoint(const ozz::animation::Skeleton& skeleton, const std::string& name)
{
    const int index = ozz::animation::FindJoint(skeleton, name.c_str());
    if (index < 0)
        std::cerr << "joint not found: " << name << std::endl;
    return index;
}

bool SampleLocals(const ozz::animation::Animation& animation,
                  const ozz::animation::Skeleton& skeleton,
                  float time,
                  std::vector<ozz::math::Transform>& locals)
{
    if (animation.duration() <= 0.f)
        return false;

    const float clamped = std::clamp(time, 0.f, animation.duration());

    ozz::animation::SamplingJob::Context context(animation.num_tracks());
    std::vector<ozz::math::SoaTransform> soa_transforms(skeleton.num_soa_joints());

    ozz::animation::SamplingJob job;
    job.animation = &animation;
    job.context = &context;
    job.ratio = clamped / animation.duration();
    job.output = ozz::make_span(soa_transforms);
    if (!job.Run())
        return false;

    locals.resize(skeleton.num_joints());
    for (int soa_index = 0; soa_index < skeleton.num_soa_joints(); ++soa_index)
    {
        const auto& soa = soa_transforms[soa_index];

        ozz::math::SimdFloat4 translations[4];
        ozz::math::Transpose3x4(&soa.translation.x, translations);

        ozz::math::SimdFloat4 rotations[4];
        ozz::math::Transpose4x4(&soa.rotation.x, rotations);

        ozz::math::SimdFloat4 scales[4];
        ozz::math::Transpose3x4(&soa.scale.x, scales);

        for (int lane = 0; lane < 4; ++lane)
        {
            const int joint = soa_index * 4 + lane;
            if (joint >= skeleton.num_joints())
                break;

            ozz::math::Transform transform;
            ozz::math::Store3PtrU(translations[lane], &transform.translation.x);
            ozz::math::StorePtrU(rotations[lane], &transform.rotation.x);
            ozz::math::Store3PtrU(scales[lane], &transform.scale.x);
            locals[joint] = transform;
        }
    }

    return true;
}

struct ExpectedBindPose
{
    const char* joint;
    float tx;
    float ty;
    float tz;
};

const std::array<ExpectedBindPose, 5> kExpectedBindPose = {{
    {"root_stalker", 0.0f, 0.0f, 0.0f},
    {"bip01", 6.96513e-06f, 0.987438f, 4.50560e-06f},
    {"bip01_pelvis", 0.0f, 0.0f, 0.0f},
    {"bip01_spine", 0.102435f, 1.76455e-07f, 0.0213843f},
    {"bip01_head", 0.0559939f, 2.85225e-09f, 1.90456e-08f},
}};

constexpr float kTranslationTolerance = 1e-4f;
constexpr float kRotationTolerance = 1e-4f;

bool TestGenerateSkeleton()
{
    std::cout << "Generating skeleton via converter..." << std::endl;
    return ConvertSkeleton(true);
}

bool TestGenerateMesh()
{
    std::cout << "Generating mesh via converter..." << std::endl;
    return ConvertMesh(true);
}

bool TestBindPoseMatchesBlender()
{
    if (!EnsureSkeletonGenerated())
        return false;

    ozz::animation::Skeleton skeleton;
    if (!LoadOzz(SkeletonOutputPath(), skeleton))
        return false;

    bool ok = true;
    for (const auto& expected : kExpectedBindPose)
    {
        const int joint_index = FindJoint(skeleton, expected.joint);
        if (joint_index < 0)
        {
            ok = false;
            continue;
        }

        const ozz::math::Transform rest = ozz::animation::GetJointLocalRestPose(skeleton, joint_index);
        const float dx = std::fabs(rest.translation.x - expected.tx);
        const float dy = std::fabs(rest.translation.y - expected.ty);
        const float dz = std::fabs(rest.translation.z - expected.tz);
        if (dx > kTranslationTolerance || dy > kTranslationTolerance || dz > kTranslationTolerance)
        {
            std::cerr << "bind pose mismatch for joint '" << expected.joint << "'\n"
                      << "  expected: [" << expected.tx << ", " << expected.ty << ", " << expected.tz << "]\n"
                      << "  actual:   [" << rest.translation.x << ", " << rest.translation.y << ", " << rest.translation.z << "]\n";
            ok = false;
        }
    }

    return ok;
}

bool TestConvertAnimationProducesFile()
{
    const bool status = ConvertAnimation(true);
    if (!status)
        return false;
    return fs::exists(AnimationOutputPath()) && fs::exists(AnimationMetadataPath());
}

bool TestAnimationCompatibleWithSkeleton()
{
    if (!ConvertAnimation(false))
        return false;

    ozz::animation::Skeleton skeleton;
    if (!LoadOzz(SkeletonOutputPath(), skeleton))
        return false;

    ozz::animation::Animation animation;
    if (!LoadAnimationByName(AnimationOutputPath(), kExpectedMultiMotionNames[0], animation))
        return false;

    if (animation.num_tracks() != skeleton.num_joints())
    {
        std::cerr << "animation track count " << animation.num_tracks()
                  << " does not match skeleton joints " << skeleton.num_joints() << std::endl;
        return false;
    }

    return animation.duration() > 0.f;
}

bool CompareQuaternion(const ozz::math::Quaternion& actual,
                       float qx, float qy, float qz, float qw)
{
    const float dx = std::fabs(actual.x - qx);
    const float dy = std::fabs(actual.y - qy);
    const float dz = std::fabs(actual.z - qz);
    const float dw = std::fabs(actual.w - qw);
    return dx <= kRotationTolerance && dy <= kRotationTolerance && dz <= kRotationTolerance && dw <= kRotationTolerance;
}

bool TestAnimationMatchesReference()
{
    if (!ConvertAnimation(false))
        return false;

    if (!ConvertSpecificMotion(kExpectedMultiMotionNames[0], true))
        return false;

    ozz::animation::Skeleton skeleton;
    if (!LoadOzz(SkeletonOutputPath(), skeleton))
        return false;

    ozz::animation::Animation animation;
    if (!LoadAnimationByName(AnimationOutputPath(), kExpectedMultiMotionNames[0], animation))
        return false;

    ozz::animation::Animation reference_animation;
    if (!LoadOzz(SingleAnimationOutputPath(), reference_animation))
        return false;

    const float frame_duration = 1.0f / 30.0f;
    const int total_frames = static_cast<int>(std::round(reference_animation.duration() / frame_duration)) + 1;
    if (total_frames < 2)
    {
        std::cerr << "animation frame count too small" << std::endl;
        return false;
    }

    bool ok = true;

    const std::array<int, 3> frames = {0, total_frames / 2, total_frames - 1};
    const std::array<const char*, 3> tracked_joints = {
        "bip01_pelvis", "bip01_spine", "bip01_head"};

    for (const int frame : frames)
    {
        const float time = static_cast<float>(frame) * frame_duration;

        std::vector<ozz::math::Transform> locals_multi;
        std::vector<ozz::math::Transform> locals_reference;
        if (!SampleLocals(animation, skeleton, time, locals_multi))
            return false;
        if (!SampleLocals(reference_animation, skeleton, time, locals_reference))
            return false;

        for (const char* joint_name : tracked_joints)
        {
            const int joint_index = FindJoint(skeleton, joint_name);
            if (joint_index < 0)
            {
                ok = false;
                continue;
            }

            const auto& actual = locals_multi[joint_index];
            const auto& expected = locals_reference[joint_index];
            const float dx = std::fabs(actual.translation.x - expected.translation.x);
            const float dy = std::fabs(actual.translation.y - expected.translation.y);
            const float dz = std::fabs(actual.translation.z - expected.translation.z);

            if (dx > kTranslationTolerance || dy > kTranslationTolerance || dz > kTranslationTolerance)
            {
                std::cerr << "frame " << frame << " joint '" << joint_name
                          << "' translation mismatch\n"
                          << "  expected: [" << expected.translation.x << ", "
                          << expected.translation.y << ", " << expected.translation.z << "]\n"
                          << "  actual:   [" << actual.translation.x << ", " << actual.translation.y << ", "
                          << actual.translation.z << "]\n";
                ok = false;
            }

            if (!CompareQuaternion(actual.rotation,
                                   expected.rotation.x,
                                   expected.rotation.y,
                                   expected.rotation.z,
                                   expected.rotation.w))
            {
                std::cerr << "frame " << frame << " joint '" << joint_name
                          << "' rotation mismatch\n";
                ok = false;
            }
        }
    }

    return ok;
}

bool TestMultipleAnimationConversion()
{
    if (!ConvertAnimation(true))
        return false;

    ozz::io::File file(AnimationOutputPath().string().c_str(), "rb");
    if (!file.opened())
    {
        std::cerr << "failed to open animation archive: " << AnimationOutputPath() << std::endl;
        return false;
    }

    ozz::io::IArchive archive(&file);
    if (archive.TestTag<ozz::animation::Animation>())
    {
        std::cerr << "expected multi-animation archive but found single animation" << std::endl;
        return false;
    }

    file.Seek(0, ozz::io::File::kSet);
    archive = ozz::io::IArchive(&file);

    uint32_t animation_count = 0;
    archive >> animation_count;
    if (animation_count != kExpectedMultiMotionNames.size())
    {
        std::cerr << "expected " << kExpectedMultiMotionNames.size() << " animations, found "
                  << animation_count << std::endl;
        return false;
    }

    std::set<std::string> names;
    for (uint32_t i = 0; i < animation_count; ++i)
    {
        ozz::animation::Animation animation;
        archive >> animation;
        if (animation.num_tracks() == 0)
        {
            std::cerr << "animation " << i << " has zero tracks" << std::endl;
            return false;
        }

        const char* name = animation.name();
        if (!name || *name == '\0')
        {
            std::cerr << "animation " << i << " missing name" << std::endl;
            return false;
        }

        std::string metadata_name;
        ReadSerializedMotionMetadata(archive, &metadata_name);
        if (metadata_name != name)
        {
            std::cerr << "metadata name mismatch for animation '" << name
                      << "', metadata reports '" << metadata_name << "'" << std::endl;
            return false;
        }

        names.insert(std::string(name));
    }

    for (const char* expected : kExpectedMultiMotionNames)
    {
        if (names.count(expected) == 0)
        {
            std::cerr << "missing animation named '" << expected << "'" << std::endl;
            return false;
        }
    }

    return true;
}

bool TestAnimationNamesPreserved()
{
    if (!ConvertAnimation(true))
        return false;

    ozz::io::File file(AnimationOutputPath().string().c_str(), "rb");
    if (!file.opened())
    {
        std::cerr << "failed to open animation archive: " << AnimationOutputPath() << std::endl;
        return false;
    }

    ozz::io::IArchive archive(&file);
    if (archive.TestTag<ozz::animation::Animation>())
    {
        ozz::animation::Animation animation;
        archive >> animation;
        const char* name = animation.name();
        if (!name || std::string(name) != kExpectedMultiMotionNames[0])
        {
            std::cerr << "single animation archive missing expected name" << std::endl;
            return false;
        }
        return true;
    }

    file.Seek(0, ozz::io::File::kSet);
    archive = ozz::io::IArchive(&file);

    uint32_t animation_count = 0;
    archive >> animation_count;

    if (animation_count != kExpectedMultiMotionNames.size())
    {
        std::cerr << "expected " << kExpectedMultiMotionNames.size() << " animations, found "
                  << animation_count << std::endl;
        return false;
    }

    std::vector<std::string> names;
    names.reserve(animation_count);

    for (uint32_t i = 0; i < animation_count; ++i)
    {
        ozz::animation::Animation animation;
        archive >> animation;

        const char* name = animation.name();
        if (!name || *name == '\0')
        {
            std::cerr << "animation index " << i << " missing runtime name" << std::endl;
            return false;
        }

        std::string metadata_name;
        ReadSerializedMotionMetadata(archive, &metadata_name);

        if (metadata_name.empty())
        {
            std::cerr << "metadata missing name for animation index " << i << std::endl;
            return false;
        }

        if (metadata_name != name)
        {
            std::cerr << "metadata name mismatch for animation index " << i << " ('"
                      << name << "' vs '" << metadata_name << "')" << std::endl;
            return false;
        }

        names.emplace_back(name);
    }

    std::set<std::string> unique_names(names.begin(), names.end());
    if (unique_names.size() != names.size())
    {
        std::cerr << "duplicate animation names detected" << std::endl;
        return false;
    }

    for (const char* expected : kExpectedMultiMotionNames)
    {
        if (unique_names.count(expected) == 0)
        {
            std::cerr << "missing animation name '" << expected << "'" << std::endl;
            return false;
        }
    }

    return true;
}

struct TestCase
{
    const char* name;
    bool (*func)();
    bool expected_to_fail;
};

} // namespace

int main()
{
    const std::array<TestCase, 8> tests = {{
        {"GenerateSkeleton", &TestGenerateSkeleton, false},
        {"GenerateMesh", &TestGenerateMesh, false},
        {"BindPoseMatchesBlender", &TestBindPoseMatchesBlender, false},
        {"ConvertAnimationProducesFile", &TestConvertAnimationProducesFile, false},
        {"AnimationCompatibleWithSkeleton", &TestAnimationCompatibleWithSkeleton, false},
        {"AnimationMatchesReference", &TestAnimationMatchesReference, false},
        {"TestMultipleAnimationConversion", &TestMultipleAnimationConversion, false},
        {"TestAnimationNamesPreserved", &TestAnimationNamesPreserved, false},
    }};

    int failures = 0;
    for (const auto& test : tests)
    {
        std::cout << "[ RUN      ] " << test.name << std::endl;
        bool result = false;
        try
        {
            result = test.func();
        }
        catch (const std::exception& ex)
        {
            std::cerr << "Test threw exception: " << ex.what() << std::endl;
            result = false;
        }
        catch (...)
        {
            std::cerr << "Test threw unknown exception" << std::endl;
            result = false;
        }

        if (result)
        {
            std::cout << "[       OK ] " << test.name << std::endl;
        }
        else
        {
            ++failures;
            std::cout << "[  FAILED ] " << test.name;
            if (test.expected_to_fail)
                std::cout << " (expected failure)";
            std::cout << std::endl;
        }
    }

    const int passed = static_cast<int>(tests.size()) - failures;
    std::cout << "[==========] " << tests.size() << " tests run.\n";
    std::cout << "[  PASSED  ] " << passed << " tests." << std::endl;
    if (failures > 0)
        std::cout << "[  FAILED  ] " << failures << " tests." << std::endl;

    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
