#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <unordered_map>
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

#include "../../../Externals/ozz-animation/src/animation/offline/gltf/extern/json.hpp"


namespace fs = std::filesystem;

namespace
{

using Json = nlohmann::json;

fs::path ProjectRoot()
{
    static const fs::path root = fs::path(PROJECT_ROOT);
    return root;
}

fs::path TestArtifactsDir()
{
    return ProjectRoot() / "src" / "xrAnimation" / "tests" / "testdata";
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

[[maybe_unused]] fs::path BaselineDir()
{
    return ProjectRoot() / "src" / "xrAnimation" / "tests" / "baselines";
}

fs::path BaselineCasesDir()
{
    return ProjectRoot() / "src" / "xrAnimation" / "tests" / "baseline_cases";
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

fs::path MeshSkinningOutputPath()
{
    return TestArtifactsDir() / "stalker_hero_mesh_skinning.json";
}

fs::path BlenderRestPoseBaselinePath()
{
    return BaselineCasesDir() / "stalker_hero_1_rest_pose.json";
}

constexpr float kMeshPositionTolerance = 1e-4f;
constexpr float kMeshWeightTolerance = 5e-4f;
constexpr float kMatrixTolerance = 1e-4f;
constexpr int kExpectedStalkerHeroVertexCount = 3237; // Blender export baseline

int64_t QuantizeToScaledInt(double value, double scale)
{
    return static_cast<int64_t>(std::llround(value * scale));
}

std::string BuildVertexSignature(const Json& vertex)
{
    if (!vertex.contains("position"))
        return {};

    const Json& position = vertex["position"];
    if (!position.is_array() || position.size() != 3)
        return {};

    std::ostringstream key;
    key << QuantizeToScaledInt(position[0].get<double>(), 100000.0) << ','
        << QuantizeToScaledInt(position[1].get<double>(), 100000.0) << ','
        << QuantizeToScaledInt(position[2].get<double>(), 100000.0);

    if (vertex.contains("weights") && vertex["weights"].is_array())
    {
        std::vector<std::pair<int, int64_t>> weights;
        weights.reserve(vertex["weights"].size());

        for (const auto& entry : vertex["weights"])
        {
            if (!entry.is_array() || entry.size() < 2)
                continue;

            const int joint_index = entry[0].get<int>();
            const double weight_value = entry[1].get<double>();
            weights.emplace_back(joint_index, QuantizeToScaledInt(weight_value, 1000000.0));
        }

        std::sort(weights.begin(), weights.end(), [](const auto& lhs, const auto& rhs) {
            if (lhs.first != rhs.first)
                return lhs.first < rhs.first;
            return lhs.second < rhs.second;
        });

        for (const auto& [joint, weight] : weights)
        {
            key << '|' << joint << ':' << weight;
        }
    }

    return key.str();
}

bool CollectVertexSignatures(const Json& meshes,
                             size_t& total_vertices,
                             std::unordered_map<std::string, size_t>& signatures)
{
    total_vertices = 0;
    signatures.clear();

    if (!meshes.is_array())
        return false;

    for (const auto& mesh_value : meshes)
    {
        if (!mesh_value.is_object())
            return false;

        if (!mesh_value.contains("vertices"))
            return false;

        const Json& vertices = mesh_value["vertices"];
        if (!vertices.is_array())
            return false;

        total_vertices += vertices.size();
        for (const auto& vertex : vertices)
        {
            const std::string signature = BuildVertexSignature(vertex);
            if (!signature.empty())
                signatures[signature]++;
        }
    }

    return true;
}

bool LoadJsonFile(const fs::path& path, Json& out)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        std::cerr << "failed to open json file: " << path << std::endl;
        return false;
    }

    try
    {
        stream >> out;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "failed to parse json '" << path << "': " << ex.what() << std::endl;
        return false;
    }

    return true;
}

bool NearlyEqual(float a, float b, float tolerance)
{
    return std::fabs(a - b) <= tolerance;
}

bool CompareFloatVector(const Json& actual,
                        const Json& expected,
                        float tolerance,
                        const std::string& context)
{
    if (!actual.is_array() || !expected.is_array())
    {
        std::cerr << context << ": expected arrays" << std::endl;
        return false;
    }

    if (actual.size() != expected.size())
    {
        std::cerr << context << ": size mismatch (" << actual.size() << " vs " << expected.size() << ")" << std::endl;
        return false;
    }

    bool ok = true;
    for (size_t i = 0; i < actual.size(); ++i)
    {
        const float lhs = static_cast<float>(actual[i].get<double>());
        const float rhs = static_cast<float>(expected[i].get<double>());
        if (!NearlyEqual(lhs, rhs, tolerance))
        {
            std::cerr << context << " component " << i << " mismatch: "
                      << lhs << " vs " << rhs << " (tolerance " << tolerance << ")" << std::endl;
            ok = false;
        }
    }
    return ok;
}

std::vector<std::pair<int, float>> ExtractWeights(const Json& weights)
{
    std::vector<std::pair<int, float>> result;
    if (!weights.is_array())
        return result;

    result.reserve(weights.size());
    for (const auto& entry : weights)
    {
        if (!entry.is_array() || entry.size() != 2)
            continue;

        const int joint_index = entry[0].get<int>();
        const float weight = static_cast<float>(entry[1].get<double>());
        result.emplace_back(joint_index, weight);
    }
    std::sort(result.begin(), result.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
    });
    return result;
}

bool CompareWeights(const Json& actual,
                    const Json& expected,
                    float tolerance,
                    const std::string& context)
{
    auto lhs = ExtractWeights(actual);
    auto rhs = ExtractWeights(expected);

    if (lhs.size() != rhs.size())
    {
        std::cerr << context << ": weight count mismatch (" << lhs.size()
                  << " vs " << rhs.size() << ")" << std::endl;
        return false;
    }

    bool ok = true;
    for (size_t i = 0; i < lhs.size(); ++i)
    {
        if (lhs[i].first != rhs[i].first)
        {
            std::cerr << context << ": joint index mismatch at " << i
                      << " (" << lhs[i].first << " vs " << rhs[i].first << ")" << std::endl;
            ok = false;
            continue;
        }
        if (!NearlyEqual(lhs[i].second, rhs[i].second, tolerance))
        {
            std::cerr << context << ": weight mismatch for joint " << lhs[i].first
                      << " (" << lhs[i].second << " vs " << rhs[i].second
                      << ", tolerance " << tolerance << ")" << std::endl;
            ok = false;
        }
    }
    return ok;
}

bool ValidateMatrix4x4(const Json& matrix, const std::string& context)
{
    if (!matrix.is_array() || matrix.size() != 4)
    {
        std::cerr << context << ": expected 4 rows" << std::endl;
        return false;
    }

    bool ok = true;
    for (size_t row = 0; row < 4; ++row)
    {
        const auto& row_data = matrix[row];
        if (!row_data.is_array() || row_data.size() != 4)
        {
            std::cerr << context << ": row " << row << " malformed" << std::endl;
            ok = false;
        }
    }
    return ok;
}

bool CompareMatrix4x4(const Json& actual,
                      const Json& expected,
                      float tolerance,
                      const std::string& context)
{
    if (!ValidateMatrix4x4(actual, context + ".actual") ||
        !ValidateMatrix4x4(expected, context + ".expected"))
    {
        return false;
    }

    bool ok = true;
    for (size_t row = 0; row < 4; ++row)
    {
        ok &= CompareFloatVector(actual[row], expected[row], tolerance,
                                 context + ".row" + std::to_string(row));
    }
    return ok;
}

bool ValidateJointPalette(const Json& palette,
                          const ozz::animation::Skeleton& skeleton,
                          const std::string& context)
{
    if (!palette.is_array())
    {
        std::cerr << context << ": palette not an array" << std::endl;
        return false;
    }

    bool ok = true;
    const int joint_count = skeleton.num_joints();
    for (size_t idx = 0; idx < palette.size(); ++idx)
    {
        const auto& entry = palette[idx];
        if (!entry.is_object())
        {
            std::cerr << context << ": entry " << idx << " not an object" << std::endl;
            ok = false;
            continue;
        }

        const int joint_index = entry.value("joint_index", -1);
        if (joint_index < 0 || joint_index >= joint_count)
        {
            std::cerr << context << ": palette joint index " << joint_index
                      << " out of bounds" << std::endl;
            ok = false;
        }

        if (entry.contains("inverse_bind_pose"))
        {
            ok &= ValidateMatrix4x4(entry["inverse_bind_pose"], context + ".inverse_bind_pose");
        }
        if (entry.contains("skinning_matrix"))
        {
            ok &= ValidateMatrix4x4(entry["skinning_matrix"], context + ".skinning_matrix");
        }
    }
    return ok;
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

bool TestViewerMeshMatchesBaseline()
{
    if (!EnsureSkeletonGenerated())
        return false;

    if (!ConvertMesh(false))
        return false;

    const fs::path skinning_dump = MeshSkinningOutputPath();
    std::error_code remove_error;
    fs::remove(skinning_dump, remove_error);

    std::vector<std::string> args = {
        std::string("--skeleton=") + SkeletonOutputPath().string(),
        std::string("--mesh=") + MeshOutputPath().string(),
        std::string("--dump_skinning_json=") + skinning_dump.string(),
        "--render=false",
        "--max_idle_loops=2",
    };

    const int viewer_exit = ExecuteCommand(ResolveViewerBinary(), args);
    if (viewer_exit != 0)
    {
        std::cerr << "ozz_animation_viewer returned exit code " << viewer_exit << std::endl;
        return false;
    }

    if (!fs::exists(skinning_dump))
    {
        std::cerr << "viewer did not produce skinning dump: " << skinning_dump << std::endl;
        return false;
    }

    Json viewer_json;
    if (!LoadJsonFile(skinning_dump, viewer_json))
        return false;

    Json baseline_json;
    if (!LoadJsonFile(BlenderRestPoseBaselinePath(), baseline_json))
        return false;

    ozz::animation::Skeleton skeleton;
    if (!LoadOzz(SkeletonOutputPath(), skeleton))
        return false;

    bool ok = true;

    if (!viewer_json.contains("armature") || !baseline_json.contains("armature"))
    {
        std::cerr << "armature missing from JSON data" << std::endl;
        return false;
    }

    const Json& viewer_armature = viewer_json["armature"];
    const Json& baseline_armature = baseline_json["armature"];

    if (!viewer_armature.contains("bones") || !baseline_armature.contains("bones"))
    {
        std::cerr << "bone arrays missing from armature" << std::endl;
        return false;
    }

    const Json& viewer_bones = viewer_armature["bones"];
    const Json& baseline_bones = baseline_armature["bones"];

    if (!viewer_bones.is_array() || !baseline_bones.is_array())
    {
        std::cerr << "bone data malformed" << std::endl;
        return false;
    }

    if (viewer_bones.size() != baseline_bones.size())
    {
        std::cerr << "bone count mismatch between viewer and baseline" << std::endl;
        ok = false;
    }

    const size_t bone_count = std::min(viewer_bones.size(), baseline_bones.size());
    for (size_t bone_index = 0; bone_index < bone_count; ++bone_index)
    {
        const Json& viewer_bone = viewer_bones[bone_index];
        const Json& baseline_bone = baseline_bones[bone_index];

        const std::string viewer_name = viewer_bone.value("name", std::string());
        const std::string baseline_name = baseline_bone.value("name", std::string());
        if (viewer_name != baseline_name)
        {
            std::cerr << "bone name mismatch at index " << bone_index << ": '"
                      << viewer_name << "' vs '" << baseline_name << "'" << std::endl;
            ok = false;
        }

        if (viewer_bone.contains("matrix_global") && baseline_bone.contains("matrix_global"))
        {
            const std::string ctx = "bone[" + std::to_string(bone_index) + "].matrix_global";
            ok &= CompareMatrix4x4(viewer_bone["matrix_global"],
                                   baseline_bone["matrix_global"],
                                   kMatrixTolerance,
                                   ctx);
        }
    }

    if (!viewer_json.contains("meshes") || !baseline_json.contains("meshes"))
    {
        std::cerr << "mesh arrays missing from JSON data" << std::endl;
        return false;
    }

    const Json& viewer_meshes = viewer_json["meshes"];
    const Json& baseline_meshes = baseline_json["meshes"];

    if (!viewer_meshes.is_array() || !baseline_meshes.is_array())
    {
        std::cerr << "mesh data malformed" << std::endl;
        return false;
    }

    std::unordered_map<std::string, size_t> viewer_signatures;
    std::unordered_map<std::string, size_t> baseline_signatures;
    size_t viewer_total_vertices = 0;
    size_t baseline_total_vertices = 0;

    if (!CollectVertexSignatures(viewer_meshes, viewer_total_vertices, viewer_signatures))
    {
        std::cerr << "viewer mesh vertices malformed" << std::endl;
        return false;
    }

    if (!CollectVertexSignatures(baseline_meshes, baseline_total_vertices, baseline_signatures))
    {
        std::cerr << "baseline mesh vertices malformed" << std::endl;
        return false;
    }

    if (viewer_signatures.size() != kExpectedStalkerHeroVertexCount)
    {
        std::cerr << "viewer produced " << viewer_signatures.size()
                  << " unique vertices, expected " << kExpectedStalkerHeroVertexCount << std::endl;
        ok = false;
    }

    if (baseline_signatures.size() != kExpectedStalkerHeroVertexCount)
    {
        std::cerr << "baseline recorded " << baseline_signatures.size()
                  << " unique vertices, expected " << kExpectedStalkerHeroVertexCount << std::endl;
        ok = false;
    }

    const size_t viewer_duplicate_vertices = viewer_total_vertices - viewer_signatures.size();
    if (viewer_duplicate_vertices > 0)
    {
        std::cout << "viewer mesh contains " << viewer_duplicate_vertices
                  << " duplicated vertices after deduplication" << std::endl;
    }

    if (viewer_signatures.size() != baseline_signatures.size())
    {
        std::cerr << "viewer/baseline unique vertex count mismatch ("
                  << viewer_signatures.size() << " vs "
                  << baseline_signatures.size() << ")" << std::endl;
        ok = false;
    }

    for (const auto& [signature, baseline_count] : baseline_signatures)
    {
        const auto viewer_it = viewer_signatures.find(signature);
        if (viewer_it == viewer_signatures.end())
        {
            std::cerr << "viewer missing vertex signature present in baseline" << std::endl;
            ok = false;
            break;
        }

        if (viewer_it->second < baseline_count)
        {
            std::cerr << "viewer has fewer instances of a vertex signature than baseline" << std::endl;
            ok = false;
            break;
        }
    }

    if (baseline_total_vertices != kExpectedStalkerHeroVertexCount)
    {
        std::cerr << "baseline raw vertex total " << baseline_total_vertices
                  << " differs from expected " << kExpectedStalkerHeroVertexCount << std::endl;
        ok = false;
    }

    if (viewer_meshes.size() != baseline_meshes.size())
    {
        std::cerr << "mesh count mismatch between viewer and baseline" << std::endl;
        ok = false;
    }

    const size_t mesh_count = std::min(viewer_meshes.size(), baseline_meshes.size());
    for (size_t mesh_index = 0; mesh_index < mesh_count; ++mesh_index)
    {
        const Json& viewer_mesh = viewer_meshes[mesh_index];
        const Json& baseline_mesh = baseline_meshes[mesh_index];
        const std::string mesh_ctx = "mesh[" + std::to_string(mesh_index) + "]";

        if (viewer_mesh.contains("joint_palette"))
        {
            ok &= ValidateJointPalette(viewer_mesh["joint_palette"], skeleton, mesh_ctx + ".joint_palette");
        }

        if (!viewer_mesh.contains("vertices") || !baseline_mesh.contains("vertices"))
        {
            std::cerr << mesh_ctx << ": vertex arrays missing" << std::endl;
            return false;
        }

        const Json& viewer_vertices = viewer_mesh["vertices"];
        const Json& baseline_vertices = baseline_mesh["vertices"];

        if (!viewer_vertices.is_array() || !baseline_vertices.is_array())
        {
            std::cerr << mesh_ctx << ": vertex data malformed" << std::endl;
            return false;
        }

        if (viewer_vertices.size() != baseline_vertices.size())
        {
            std::cerr << mesh_ctx << ": vertex count mismatch (" << viewer_vertices.size()
                      << " vs " << baseline_vertices.size() << ")" << std::endl;
            ok = false;
        }

        std::map<size_t, size_t> viewer_histogram;
        std::map<size_t, size_t> baseline_histogram;

        const size_t vertex_count = std::min(viewer_vertices.size(), baseline_vertices.size());
        for (size_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
        {
            const Json& viewer_vertex = viewer_vertices[vertex_index];
            const Json& baseline_vertex = baseline_vertices[vertex_index];
            const std::string vertex_ctx = mesh_ctx + ".vertex[" + std::to_string(vertex_index) + "]";

            const int viewer_idx = viewer_vertex.value("index", static_cast<int>(vertex_index));
            const int baseline_idx = baseline_vertex.value("index", static_cast<int>(vertex_index));
            if (viewer_idx != baseline_idx)
            {
                std::cerr << vertex_ctx << ": index mismatch (" << viewer_idx
                          << " vs " << baseline_idx << ")" << std::endl;
                ok = false;
            }

            if (viewer_vertex.contains("position") && baseline_vertex.contains("position"))
            {
                ok &= CompareFloatVector(viewer_vertex["position"],
                                         baseline_vertex["position"],
                                         kMeshPositionTolerance,
                                         vertex_ctx + ".position");
            }
            else
            {
                std::cerr << vertex_ctx << ": missing position data" << std::endl;
                ok = false;
            }

            if (viewer_vertex.contains("weights") && baseline_vertex.contains("weights"))
            {
                ok &= CompareWeights(viewer_vertex["weights"], baseline_vertex["weights"],
                                     kMeshWeightTolerance, vertex_ctx + ".weights");

                const auto viewer_weights = ExtractWeights(viewer_vertex["weights"]);
                const auto baseline_weights = ExtractWeights(baseline_vertex["weights"]);
                viewer_histogram[viewer_weights.size()]++;
                baseline_histogram[baseline_weights.size()]++;

                float viewer_sum = 0.f;
                for (const auto& pair : viewer_weights)
                    viewer_sum += pair.second;
                if (!viewer_weights.empty() && !NearlyEqual(viewer_sum, 1.f, 1e-2f))
                {
                    std::cerr << vertex_ctx << ": viewer weight sum " << viewer_sum << " deviates from 1.0" << std::endl;
                    ok = false;
                }
            }
            else
            {
                std::cerr << vertex_ctx << ": missing weights" << std::endl;
                ok = false;
            }

            if (viewer_vertex.contains("skinned_position") && baseline_vertex.contains("position"))
            {
                ok &= CompareFloatVector(viewer_vertex["skinned_position"],
                                         baseline_vertex["position"],
                                         1e-2f,
                                         vertex_ctx + ".skinned_position");
            }
        }

        if (viewer_histogram != baseline_histogram)
        {
            std::cerr << mesh_ctx << ": influence histogram mismatch" << std::endl;
            ok = false;
        }
    }

    return ok;
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
    const std::array<TestCase, 9> tests = {{
        {"GenerateSkeleton", &TestGenerateSkeleton, false},
        {"GenerateMesh", &TestGenerateMesh, false},
        {"ViewerMeshMatchesBaseline", &TestViewerMeshMatchesBaseline, false},
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
