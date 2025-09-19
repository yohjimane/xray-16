#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#ifndef WORKSPACE_ROOT
#error "WORKSPACE_ROOT compile definition must be provided"
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

fs::path WorkspaceRoot()
{
    static const fs::path root = fs::path(WORKSPACE_ROOT);
    return root;
}

fs::path TestArtifactsDir()
{
    return WorkspaceRoot() / "asset_tests" / "test_outputs";
}

fs::path SkeletonInputPath()
{
    return WorkspaceRoot() / "xray-16" / "res" / "testdata" / "stalker_hero_1.ogf";
}

fs::path AnimationInputPath()
{
    return WorkspaceRoot() / "xray-16" / "res" / "testdata" / "critical_hit_grup_1.omf";
}

fs::path SkeletonOutputPath()
{
    return TestArtifactsDir() / "stalker_hero_bind_pose.ozz";
}

fs::path SkeletonCsvPath()
{
    return TestArtifactsDir() / "stalker_hero_bind_pose.csv";
}

fs::path AnimationOutputPath()
{
    return TestArtifactsDir() / "critical_hit_grup_1.ozz";
}

fs::path AnimationMetadataPath()
{
    return TestArtifactsDir() / "critical_hit_grup_1.json";
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

fs::path ResolveConverterBinary()
{
#ifdef _WIN32
    const std::string executable_name = "xray_to_ozz_converter.exe";
#else
    const std::string executable_name = "xray_to_ozz_converter";
#endif

    const fs::path build_bin = WorkspaceRoot() / "xray-16" / "ozz_utils" / "bin";

    const std::array<fs::path, 2> candidates = {
        build_bin / "Debug" / executable_name,
        build_bin / executable_name};

    for (const auto& path : candidates)
    {
        if (fs::exists(path))
            return path;
    }

    std::ostringstream oss;
    oss << "Unable to locate xray_to_ozz_converter binary. Checked:";
    for (const auto& candidate : candidates)
        oss << '\n' << "  " << candidate.string();
    throw std::runtime_error(oss.str());
}

std::string BuildCommand(const std::vector<std::string>& args)
{
    const fs::path converter = ResolveConverterBinary();
    const fs::path converter_dir = converter.parent_path();

#ifdef _WIN32
    std::string command = QuoteForShell(converter.string());
    for (const std::string& arg : args)
    {
        command.push_back(' ');
        command.append(QuoteForShell(arg));
    }
    return command;
#else
    std::string command = "LD_LIBRARY_PATH=";
    command.append(QuoteForShell(converter_dir.string()));
    command.push_back(' ');
    command.append(QuoteForShell(converter.string()));
    for (const std::string& arg : args)
    {
        command.push_back(' ');
        command.append(QuoteForShell(arg));
    }
    return command;
#endif
}

int ExecuteCommand(const std::vector<std::string>& args)
{
    const std::string command = BuildCommand(args);
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

    const int exit_code = ExecuteCommand(args);
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

    const int exit_code = ExecuteCommand(args);
    if (exit_code != 0)
    {
        std::cerr << "animation conversion failed with exit code " << exit_code << std::endl;
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
    archive >> object;
    return true;
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
    if (!LoadOzz(AnimationOutputPath(), animation))
        return false;

    if (animation.num_tracks() != skeleton.num_joints())
    {
        std::cerr << "animation track count " << animation.num_tracks()
                  << " does not match skeleton joints " << skeleton.num_joints() << std::endl;
        return false;
    }

    return animation.duration() > 0.f;
}

struct JointSampleExpectation
{
    const char* joint;
    float tx;
    float ty;
    float tz;
    float qx;
    float qy;
    float qz;
    float qw;
};

struct FrameExpectation
{
    int frame;
    std::vector<JointSampleExpectation> joints;
};

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

    ozz::animation::Skeleton skeleton;
    if (!LoadOzz(SkeletonOutputPath(), skeleton))
        return false;

    ozz::animation::Animation animation;
    if (!LoadOzz(AnimationOutputPath(), animation))
        return false;

    const float frame_duration = 1.0f / 30.0f;
    const int total_frames = static_cast<int>(std::round(animation.duration() / frame_duration)) + 1;
    if (total_frames < 2)
    {
        std::cerr << "animation frame count too small" << std::endl;
        return false;
    }

    const std::array<FrameExpectation, 3> expectations = {{
        {0,
         {{"bip01_pelvis", 0.0f, 0.0f, 0.0f, 0.00000f, 0.00000f, 0.00000f, 1.00000f},
          {"bip01_spine", 0.10244f, 0.00000f, 0.02138f, 0.00000f, 0.00000f, 0.00000f, 1.00000f},
          {"bip01_head", 0.05599f, 0.00000f, 0.00000f, 0.00000f, 0.00000f, 0.00000f, 1.00000f}}},
        {total_frames / 2,
         {{"bip01_pelvis", 0.05985f, 0.92727f, -0.13289f, 0.00661f, -0.71725f, 0.69680f, -0.00127f},
          {"bip01_spine", 0.04618f, 1.02785f, -0.15830f, -0.12347f, -0.02831f, 0.69777f, 0.70458f},
          {"bip01_head", -0.09405f, 1.57454f, -0.12226f, -0.30937f, -0.14131f, 0.60664f, 0.71922f}}},
        {total_frames - 1,
         {{"bip01_pelvis", 0.05198f, 0.99550f, -0.11888f, 0.00614f, -0.78895f, 0.61445f, -0.00776f},
          {"bip01_spine", 0.02653f, 1.08829f, -0.17544f, -0.08374f, -0.03335f, 0.71219f, 0.696129f},
          {"bip01_head", -0.05817f, 1.63351f, -0.14511f, -0.27743f, -0.19140f, 0.63215f, 0.69934f}}},
    }};

    bool ok = true;

    for (const auto& frame_expectation : expectations)
    {
        const float time = static_cast<float>(frame_expectation.frame) * frame_duration;
        std::vector<ozz::math::Transform> locals;
        if (!SampleLocals(animation, skeleton, time, locals))
            return false;

        for (const auto& joint_expectation : frame_expectation.joints)
        {
            const int joint_index = FindJoint(skeleton, joint_expectation.joint);
            if (joint_index < 0)
            {
                ok = false;
                continue;
            }

            const auto& actual = locals[joint_index];
            const float dx = std::fabs(actual.translation.x - joint_expectation.tx);
            const float dy = std::fabs(actual.translation.y - joint_expectation.ty);
            const float dz = std::fabs(actual.translation.z - joint_expectation.tz);

            if (dx > kTranslationTolerance || dy > kTranslationTolerance || dz > kTranslationTolerance)
            {
                std::cerr << "frame " << frame_expectation.frame << " joint '" << joint_expectation.joint
                          << "' translation mismatch\n"
                          << "  expected: [" << joint_expectation.tx << ", " << joint_expectation.ty << ", "
                          << joint_expectation.tz << "]\n"
                          << "  actual:   [" << actual.translation.x << ", " << actual.translation.y << ", "
                          << actual.translation.z << "]\n";
                ok = false;
            }

            if (!CompareQuaternion(actual.rotation,
                                   joint_expectation.qx,
                                   joint_expectation.qy,
                                   joint_expectation.qz,
                                   joint_expectation.qw))
            {
                std::cerr << "frame " << frame_expectation.frame << " joint '" << joint_expectation.joint
                          << "' rotation mismatch\n";
                ok = false;
            }
        }
    }

    return ok;
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
    const std::array<TestCase, 5> tests = {{
        {"GenerateSkeleton", &TestGenerateSkeleton, false},
        {"BindPoseMatchesBlender", &TestBindPoseMatchesBlender, false},
        {"ConvertAnimationProducesFile", &TestConvertAnimationProducesFile, false},
        {"AnimationCompatibleWithSkeleton", &TestAnimationCompatibleWithSkeleton, false},
        {"AnimationMatchesReference", &TestAnimationMatchesReference, false},
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
