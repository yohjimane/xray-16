#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
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

#include "ozz/animation/runtime/skeleton.h"
#include "ozz/animation/runtime/skeleton_utils.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"

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
    // Prepend LD_LIBRARY_PATH so the converter can locate ozz shared objects.
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

bool GenerateSkeleton(bool force)
{
    const fs::path output_dir = TestArtifactsDir();
    const fs::path output_file = output_dir / "stalker_hero_bind_pose.ozz";
    const fs::path input_file = WorkspaceRoot() / "gamedata" / "stalker_hero" / "stalker_hero_1.ogf";

    std::error_code ec;
    fs::create_directories(output_dir, ec);
    (void)ec;

    if (force && fs::exists(output_file))
        fs::remove(output_file);

    if (!force && fs::exists(output_file))
        return true;

    std::vector<std::string> args = {
        "skeleton",
        input_file.string(),
        output_file.string()};

    const int exit_code = ExecuteCommand(args);
    if (exit_code != 0)
    {
        std::cerr << "xray_to_ozz_converter returned exit code " << exit_code << std::endl;
        return false;
    }

    if (!fs::exists(output_file))
    {
        std::cerr << "converter reported success but output file is missing: " << output_file << std::endl;
        return false;
    }

    return true;
}

bool EnsureSkeletonGenerated()
{
    static bool cached = false;
    static bool status = false;
    if (!cached)
    {
        status = GenerateSkeleton(false);
        cached = true;
    }
    return status;
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
    {"bip01", 6.96513e-06f, 0.987438f, 4.5056e-06f},
    {"bip01_pelvis", 0.0f, 0.0f, 0.0f},
    {"bip01_spine", 0.102435f, 1.76455e-07f, 0.0213843f},
    {"bip01_head", 0.0559939f, 2.85225e-09f, 1.90456e-08f},
}};

constexpr float kTranslationTolerance = 1e-4f;

fs::path SkeletonOutputPath()
{
    return TestArtifactsDir() / "stalker_hero_bind_pose.ozz";
}

bool TestGenerateSkeleton()
{
    std::cout << "Generating skeleton via converter..." << std::endl;
    return GenerateSkeleton(true);
}

bool TestBindPoseMatchesBlender()
{
    if (!EnsureSkeletonGenerated())
        return false;

    const fs::path skeleton_path = SkeletonOutputPath();
    ozz::io::File file(skeleton_path.string().c_str(), "rb");
    if (!file.opened())
    {
        std::cerr << "failed to open skeleton: " << skeleton_path << std::endl;
        return false;
    }

    ozz::io::IArchive archive(&file);
    ozz::animation::Skeleton skeleton;
    archive >> skeleton;

    bool ok = true;
    for (const auto& expected : kExpectedBindPose)
    {
        const int joint_index = ozz::animation::FindJoint(skeleton, expected.joint);
        if (joint_index < 0)
        {
            std::cerr << "joint not found in skeleton: " << expected.joint << std::endl;
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

bool TestGenerateAnimation()
{
    const fs::path input_skeleton = WorkspaceRoot() / "gamedata" / "stalker_hero" / "stalker_hero_1.ogf";
    const fs::path input_animation = WorkspaceRoot() / "gamedata" / "critical_hit_grup_1.omf";
    const fs::path output_dir = TestArtifactsDir();
    const fs::path output_animation = output_dir / "critical_hit_grup_1.ozz";

    std::vector<std::string> args = {
        "animation",
        input_animation.string(),
        output_animation.string(),
        input_skeleton.string()};

    const int exit_code = ExecuteCommand(args);
    if (exit_code != 0)
    {
        std::cerr << "[TODO] animation conversion is not yet implemented (converter exit code " << exit_code << ")" << std::endl;
        return false;
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
    const std::array<TestCase, 3> tests = {{
        {"GenerateSkeleton", &TestGenerateSkeleton, false},
        {"BindPoseMatchesBlender", &TestBindPoseMatchesBlender, false},
        {"GenerateAnimation", &TestGenerateAnimation, true},
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
                std::cout << " (expected TODO failure)";
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
