#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"

#ifndef PROJECT_ROOT
#    error "PROJECT_ROOT compile definition must be provided"
#endif

#ifdef _WIN32
#    include <windows.h>
#else
#    include <sys/wait.h>
#endif

#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/animation/runtime/skeleton_utils.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/transform.h"

#include "../../../Externals/ozz-animation/samples/framework/mesh.h"
#include "../../../Externals/ozz-animation/src/animation/offline/gltf/extern/json.hpp"

#include "OzzBundle.h"

namespace fs = std::filesystem;

namespace
{
using Json = nlohmann::json;

fs::path ProjectRoot()
{
    static const fs::path root = fs::path(PROJECT_ROOT);
    return root;
}

struct ChunkView
{
    uint32_t id = 0;
    const uint8_t* data = nullptr;
    size_t size = 0;
};

struct OgfSurfaceStats
{
    uint32_t vertex_count = 0;
    uint32_t face_count = 0;
};

constexpr uint32_t OGF_HEADER = 1;
constexpr uint32_t OGF_VERTICES = 3;
constexpr uint32_t OGF_INDICES = 4;
constexpr uint32_t OGF_CHILDREN = 9;

bool ReadBinaryFile(const fs::path& path, std::vector<uint8_t>& out_buffer)
{
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream)
    {
        std::cerr << "failed to open binary file: " << path << std::endl;
        return false;
    }

    const auto size = stream.tellg();
    if (size <= 0)
    {
        std::cerr << "binary file empty: " << path << std::endl;
        return false;
    }

    out_buffer.resize(static_cast<size_t>(size));
    stream.seekg(0, std::ios::beg);
    stream.read(reinterpret_cast<char*>(out_buffer.data()), out_buffer.size());
    if (!stream)
    {
        std::cerr << "failed to read binary file: " << path << std::endl;
        return false;
    }

    return true;
}

bool ParseChunkSequence(const uint8_t* data, size_t size, std::vector<ChunkView>& out_chunks)
{
    out_chunks.clear();

    size_t offset = 0;
    while (offset + sizeof(uint32_t) * 2 <= size)
    {
        uint32_t id = 0;
        uint32_t chunk_size = 0;
        std::memcpy(&id, data + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        std::memcpy(&chunk_size, data + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        if (offset + chunk_size > size)
        {
            std::cerr << "chunk extends beyond parent size" << std::endl;
            return false;
        }

        out_chunks.push_back(ChunkView{ id, data + offset, static_cast<size_t>(chunk_size) });
        offset += chunk_size;
    }

    return true;
}

bool ExtractSurfaceStatsFromSections(const std::vector<ChunkView>& sections, OgfSurfaceStats& stats)
{
    bool has_vertices = false;
    bool has_indices = false;

    for (const ChunkView& section : sections)
    {
        if (section.id == OGF_VERTICES)
        {
            size_t offset = 0;
            if (section.size < sizeof(uint32_t) * 2)
            {
                std::cerr << "vertex chunk too small" << std::endl;
                return false;
            }

            offset += sizeof(uint32_t); // skip vertex format / type
            uint32_t vertex_count = 0;
            std::memcpy(&vertex_count, section.data + offset, sizeof(uint32_t));
            stats.vertex_count = vertex_count;
            has_vertices = true;
        }
        else if (section.id == OGF_INDICES)
        {
            if (section.size < sizeof(uint32_t))
            {
                std::cerr << "index chunk too small" << std::endl;
                return false;
            }

            uint32_t index_count = 0;
            std::memcpy(&index_count, section.data, sizeof(uint32_t));
            if (index_count % 3 != 0)
            {
                std::cerr << "index count not divisible by 3" << std::endl;
                return false;
            }
            stats.face_count = index_count / 3;
            has_indices = true;
        }
    }

    return has_vertices && has_indices;
}

bool LoadOgfSurfaceStats(const fs::path& path, std::vector<OgfSurfaceStats>& surfaces)
{
    std::vector<uint8_t> buffer;
    if (!ReadBinaryFile(path, buffer))
        return false;

    std::vector<ChunkView> root_chunks;
    if (!ParseChunkSequence(buffer.data(), buffer.size(), root_chunks))
        return false;

    surfaces.clear();

    const auto children_it = std::find_if(root_chunks.begin(), root_chunks.end(),
        [](const ChunkView& chunk)
        {
            return chunk.id == OGF_CHILDREN;
        });

    if (children_it != root_chunks.end())
    {
        std::vector<ChunkView> child_chunks;
        if (!ParseChunkSequence(children_it->data, children_it->size, child_chunks))
            return false;

        for (const ChunkView& child : child_chunks)
        {
            std::vector<ChunkView> sections;
            if (!ParseChunkSequence(child.data, child.size, sections))
                return false;

            OgfSurfaceStats stats;
            if (!ExtractSurfaceStatsFromSections(sections, stats))
                return false;
            surfaces.push_back(stats);
        }

        if (!surfaces.empty())
            return true;
    }

    OgfSurfaceStats root_surface;
    if (ExtractSurfaceStatsFromSections(root_chunks, root_surface))
    {
        surfaces.push_back(root_surface);
        return true;
    }

    std::cerr << "failed to locate any surfaces in OGF file: " << path << std::endl;
    return false;
}

bool LoadMeshArchive(const fs::path& path, std::vector<ozz::sample::Mesh>& meshes)
{
    ozz::io::File file(path.string().c_str(), "rb");
    if (!file.opened())
    {
        std::cerr << "failed to open mesh archive: " << path << std::endl;
        return false;
    }

    ozz::io::IArchive archive(&file);
    meshes.clear();

    while (archive.TestTag<ozz::sample::Mesh>())
    {
        meshes.emplace_back();
        archive >> meshes.back();
    }

    if (meshes.empty())
    {
        std::cerr << "mesh archive contained no meshes: " << path << std::endl;
        return false;
    }

    return true;
}

fs::path TestArtifactsDir()
{
    return ProjectRoot() / "src" / "xrAnimation" / "tests" / "testdata";
}

fs::path SkeletonInputPath()
{
    return ProjectRoot() / "res" / "testdata" / "npc" / "stalker_hero_1.ogf";
}

fs::path AnimationInputPath()
{
    return ProjectRoot() / "res" / "testdata" / "npc" / "critical_hit_grup_1.omf";
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

fs::path BundleOutputPath()
{
    return TestArtifactsDir() / "stalker_hero.ozzx";
}

fs::path BlenderRestPoseBaselinePath()
{
    return BaselineCasesDir() / "stalker_hero_1_rest_pose.json";
}

fs::path CombinedBaselinePath()
{
    return BaselineCasesDir() / "combined_vertices.json";
}

constexpr float kMeshPositionTolerance = 1e-4f;
constexpr float kMeshWeightTolerance = 5e-4f;
constexpr float kMatrixTolerance = 1e-4f;
constexpr int kExpectedStalkerHeroVertexCount = 3237; // Blender export baseline

struct BaselineVertex
{
    std::array<double, 3> position{}; // Ozz basis
    std::vector<std::pair<int, double>> weights;
    int mesh_local_index = -1;
};

bool LoadJsonFile(const fs::path& path, Json& out);
std::string BuildVertexSignatureFromComponents(double px, double py, double pz, const std::vector<std::pair<int, double>>& weights);

bool LoadCombinedBaselineVertices(const ozz::animation::Skeleton& skeleton, std::unordered_map<std::string, std::vector<BaselineVertex>>& out_vertices)
{
    Json combined;
    if (!LoadJsonFile(CombinedBaselinePath(), combined))
        return false;

    if (!combined.contains("vertices") || !combined["vertices"].is_array())
    {
        std::cerr << "combined baseline missing vertices array" << std::endl;
        return false;
    }

    out_vertices.clear();

    const Json& vertices = combined["vertices"];
    for (const auto& vertex : vertices)
    {
        if (!vertex.contains("co"))
            continue;

        const auto& co = vertex["co"];
        if (!co.is_array() || co.size() != 3)
            continue;

        std::array<double, 3> position{ co[0].get<double>(), co[1].get<double>(), co[2].get<double>() };

        const Json& groups = vertex.value("groups", Json::array());
        std::vector<std::pair<int, double>> weights;
        weights.reserve(groups.is_array() ? groups.size() : 0);

        for (const auto& group : groups)
        {
            const std::string group_name = group.value("group_name", std::string());
            const double weight = group.value("weight", 0.0);
            if (group_name.empty() || weight <= 0.f)
                continue;

            const int joint_index = ozz::animation::FindJoint(skeleton, group_name.c_str());
            if (joint_index < 0)
            {
                std::cerr << "baseline vertex references unknown joint '" << group_name << "'" << std::endl;
                continue;
            }

            weights.emplace_back(joint_index, weight);
        }

        if (weights.empty())
            continue;

        std::stable_sort(weights.begin(), weights.end(),
            [](const auto& lhs, const auto& rhs)
            {
                return lhs.second > rhs.second;
            });

        if (weights.size() > 4)
            weights.resize(4);

        double weight_sum = 0.0;
        for (const auto& entry : weights)
            weight_sum += entry.second;

        if (weight_sum > std::numeric_limits<double>::epsilon())
        {
            const double inv_sum = 1.0 / weight_sum;
            for (auto& entry : weights)
                entry.second *= inv_sum;
        }

        std::sort(weights.begin(), weights.end(),
            [](const auto& lhs, const auto& rhs)
            {
                if (lhs.first != rhs.first)
                    return lhs.first < rhs.first;
                return lhs.second < rhs.second;
            });

        const std::string signature =
            BuildVertexSignatureFromComponents(static_cast<double>(position[0]), static_cast<double>(position[1]), static_cast<double>(position[2]), weights);

        BaselineVertex baseline_vertex;
        baseline_vertex.position = position;
        baseline_vertex.weights = weights;
        baseline_vertex.mesh_local_index = vertex.value("index", -1);

        out_vertices[signature].push_back(std::move(baseline_vertex));
    }

    return true;
}

int64_t QuantizeToScaledInt(double value, double scale)
{
    return static_cast<int64_t>(std::llround(value * scale));
}

std::string BuildVertexSignatureFromComponents(double px, double py, double pz, const std::vector<std::pair<int, double>>& weights)
{
    std::ostringstream key;
    key << QuantizeToScaledInt(px, 100000.0) << ',' << QuantizeToScaledInt(py, 100000.0) << ',' << QuantizeToScaledInt(pz, 100000.0);

    std::vector<std::pair<int, int64_t>> quantized;
    quantized.reserve(weights.size());

    for (const auto& entry : weights)
        quantized.emplace_back(entry.first, QuantizeToScaledInt(entry.second, 1000000.0));

    std::sort(quantized.begin(), quantized.end(),
        [](const auto& lhs, const auto& rhs)
        {
            if (lhs.first != rhs.first)
                return lhs.first < rhs.first;
            return lhs.second < rhs.second;
        });

    for (const auto& [joint, weight] : quantized)
        key << '|' << joint << ':' << weight;

    return key.str();
}

std::string BuildVertexSignature(const Json& vertex)
{
    if (!vertex.contains("position"))
        return {};

    const Json& position = vertex["position"];
    if (!position.is_array() || position.size() != 3)
        return {};

    std::vector<std::pair<int, double>> weights;
    if (vertex.contains("weights") && vertex["weights"].is_array())
    {
        for (const auto& entry : vertex["weights"])
        {
            if (!entry.is_array() || entry.size() < 2)
                continue;
            weights.emplace_back(entry[0].get<int>(), entry[1].get<double>());
        }
    }

    return BuildVertexSignatureFromComponents(position[0].get<double>(), position[1].get<double>(), position[2].get<double>(), weights);
}

[[maybe_unused]] bool CollectVertexSignatures(const Json& meshes, size_t& total_vertices, std::unordered_map<std::string, size_t>& signatures)
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

[[maybe_unused]] bool CollectMeshSignatures(const ozz::sample::Mesh& mesh, std::unordered_map<std::string, size_t>& signatures, int& raw_vertex_count)
{
    signatures.clear();
    raw_vertex_count = 0;

    for (const ozz::sample::Mesh::Part& part : mesh.parts)
    {
        const int vertex_count = part.vertex_count();
        const int influences = part.influences_count();
        raw_vertex_count += vertex_count;

        for (int vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
        {
            const int position_offset = vertex_index * ozz::sample::Mesh::Part::kPositionsCpnts;
            if (position_offset + 2 >= static_cast<int>(part.positions.size()))
            {
                std::cerr << "mesh positions array truncated" << std::endl;
                return false;
            }

            const double px = part.positions[position_offset + 0];
            const double py = part.positions[position_offset + 1];
            const double pz = part.positions[position_offset + 2];

            std::vector<std::pair<int, double>> weights;
            if (influences <= 0)
            {
                weights.emplace_back(0, 1.0);
            }
            else
            {
                const int joint_offset = vertex_index * influences;
                if (joint_offset + influences > static_cast<int>(part.joint_indices.size()))
                {
                    std::cerr << "joint indices array truncated" << std::endl;
                    return false;
                }

                double partial_sum = 0.0;
                for (int influence = 0; influence < influences; ++influence)
                {
                    const uint16_t palette_index = part.joint_indices[joint_offset + influence];
                    int skeleton_joint = palette_index;
                    if (!mesh.joint_remaps.empty())
                    {
                        if (palette_index >= mesh.joint_remaps.size())
                        {
                            std::cerr << "joint remap index out of bounds" << std::endl;
                            return false;
                        }
                        skeleton_joint = mesh.joint_remaps[palette_index];
                    }

                    double weight = 1.0;
                    if (influences > 1)
                    {
                        if (influence < influences - 1)
                        {
                            const int weight_offset = vertex_index * (influences - 1) + influence;
                            if (weight_offset >= static_cast<int>(part.joint_weights.size()))
                            {
                                std::cerr << "joint weights array truncated" << std::endl;
                                return false;
                            }
                            weight = part.joint_weights[weight_offset];
                            partial_sum += weight;
                        }
                        else
                        {
                            weight = std::max(0.0, 1.0 - partial_sum);
                        }
                    }

                    weights.emplace_back(skeleton_joint, weight);
                }
            }

            const std::string signature = BuildVertexSignatureFromComponents(px, py, pz, weights);
            if (!signature.empty())
                signatures[signature]++;
        }
    }

    return true;
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

bool ValidateJointPalette(const Json& palette, const ozz::animation::Skeleton& skeleton, const std::string& context)
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
            std::cerr << context << ": palette joint index " << joint_index << " out of bounds" << std::endl;
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

const std::array<const char*, 4> kExpectedMultiMotionNames = {
    {
     "norm_2_critical_hit_hend_left_0", "norm_2_critical_hit_hend_right_0",
     "norm_2_critical_hit_torso_0", "norm_2_critical_hit_torso_1",
     }
};

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

bool LoadAnimationByName(const fs::path& path, const std::string& motion_name, ozz::animation::Animation& animation_out)
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
            std::cerr << "animation '" << motion_name << "' not found in single-animation archive" << std::endl;
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

    const std::array<fs::path, 2> candidates = { build_bin / "Debug" / executable_name, build_bin / executable_name };

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

std::string BuildCommand(const fs::path& binary, const std::vector<std::string>& args)
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

int ExecuteCommand(const fs::path& binary, const std::vector<std::string>& args)
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

    std::vector<std::string> args = { "skeleton", SkeletonInputPath().string(), output_file.string(), "--dump-bind", SkeletonCsvPath().string() };

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

bool ConvertBundle(bool force)
{
    const fs::path output_file = BundleOutputPath();

    std::error_code ec;
    fs::create_directories(TestArtifactsDir(), ec);

    if (force && fs::exists(output_file))
        fs::remove(output_file);

    if (!force && fs::exists(output_file))
        return true;

    std::vector<std::string> args = { "bundle", SkeletonInputPath().string(), output_file.string() };

    const int exit_code = ExecuteConverterCommand(args);
    if (exit_code != 0)
    {
        std::cerr << "bundle conversion failed with exit code " << exit_code << std::endl;
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

    std::vector<std::string> args = { "animation", AnimationInputPath().string(), output_file.string(), SkeletonInputPath().string(), "--metadata",
        AnimationMetadataPath().string() };

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

    std::vector<std::string> args = { "animation", AnimationInputPath().string(), output_file.string(), SkeletonInputPath().string(), "--motion", motion_name };

    const int exit_code = ExecuteConverterCommand(args);
    if (exit_code != 0)
    {
        std::cerr << "animation conversion for motion '" << motion_name << "' failed with exit code " << exit_code << std::endl;
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

bool SampleLocals(const ozz::animation::Animation& animation, const ozz::animation::Skeleton& skeleton, float time, std::vector<ozz::math::Transform>& locals)
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

const std::array<ExpectedBindPose, 5> kExpectedBindPose = {
    {
     { "root_stalker", 0.0f, 0.0f, 0.0f },
     { "bip01", 6.96513e-06f, 0.987438f, 4.50560e-06f },
     { "bip01_pelvis", 0.0f, 0.0f, 0.0f },
     { "bip01_spine", 0.102435f, 1.76455e-07f, 0.0213843f },
     { "bip01_head", 0.0559939f, 2.85225e-09f, 1.90456e-08f },
     }
};

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

bool TestMeshVertexCountsMatchSource()
{
    if (!ConvertMesh(false))
        return false;

    std::vector<OgfSurfaceStats> ogf_surfaces;
    if (!LoadOgfSurfaceStats(SkeletonInputPath(), ogf_surfaces))
        return false;

    std::vector<ozz::sample::Mesh> meshes;
    if (!LoadMeshArchive(MeshOutputPath(), meshes))
        return false;

    if (meshes.size() != ogf_surfaces.size())
    {
        std::cerr << "surface count mismatch before vertex comparison" << std::endl;
        return false;
    }

    uint64_t ogf_vertex_total = 0;
    uint64_t mesh_vertex_total = 0;
    uint64_t ogf_face_total = 0;
    uint64_t mesh_face_total = 0;

    bool ok = true;

    for (size_t i = 0; i < meshes.size(); ++i)
    {
        const ozz::sample::Mesh& mesh = meshes[i];
        const OgfSurfaceStats& source = ogf_surfaces[i];
        const uint32_t mesh_face_count = static_cast<uint32_t>(mesh.triangle_index_count() / 3);

        if (mesh.vertex_count() != static_cast<int>(source.vertex_count))
        {
            std::cerr << "surface " << i << " exported vertex count " << mesh.vertex_count() << " differs from OGF vertex count " << source.vertex_count
                      << std::endl;
            ok = false;
        }

        if (mesh_face_count != source.face_count)
        {
            std::cerr << "surface " << i << " exported face count " << mesh_face_count << " differs from OGF face count " << source.face_count << std::endl;
            ok = false;
        }

        mesh_vertex_total += static_cast<uint32_t>(mesh.vertex_count());
        mesh_face_total += mesh_face_count;
        ogf_vertex_total += source.vertex_count;
        ogf_face_total += source.face_count;
    }

    if (mesh_vertex_total != ogf_vertex_total)
    {
        std::cerr << "total exported vertices " << mesh_vertex_total << " differ from OGF vertices " << ogf_vertex_total << std::endl;
        ok = false;
    }

    if (mesh_face_total != ogf_face_total)
    {
        std::cerr << "total exported faces " << mesh_face_total << " differ from OGF faces " << ogf_face_total << std::endl;
        ok = false;
    }

    return ok;
}

bool TestMeshSurfaceCountMatchesSource()
{
    if (!ConvertMesh(false))
        return false;

    std::vector<OgfSurfaceStats> ogf_surfaces;
    if (!LoadOgfSurfaceStats(SkeletonInputPath(), ogf_surfaces))
        return false;

    std::vector<ozz::sample::Mesh> meshes;
    if (!LoadMeshArchive(MeshOutputPath(), meshes))
        return false;

    if (meshes.size() != ogf_surfaces.size())
    {
        std::cerr << "exported mesh surface count " << meshes.size() << " does not match OGF surface count " << ogf_surfaces.size() << std::endl;
        return false;
    }

    return true;
}

bool TestMeshSurfaceStatsMatchSource()
{
    if (!ConvertMesh(false))
        return false;

    std::vector<OgfSurfaceStats> ogf_surfaces;
    if (!LoadOgfSurfaceStats(SkeletonInputPath(), ogf_surfaces))
        return false;

    std::vector<ozz::sample::Mesh> meshes;
    if (!LoadMeshArchive(MeshOutputPath(), meshes))
        return false;

    if (meshes.size() != ogf_surfaces.size())
    {
        std::cerr << "surface count mismatch before stats comparison" << std::endl;
        return false;
    }

    bool ok = true;

    for (size_t i = 0; i < meshes.size(); ++i)
    {
        const ozz::sample::Mesh& mesh = meshes[i];
        const OgfSurfaceStats& source = ogf_surfaces[i];
        const ozz::sample::XRayMeshMetadata& metadata = mesh.xray_metadata;

        if (metadata.original_vertex_count != source.vertex_count)
        {
            std::cerr << "surface " << i << " original vertex count " << metadata.original_vertex_count << " (metadata) does not match OGF "
                      << source.vertex_count << std::endl;
            ok = false;
        }

        if (metadata.original_face_count != source.face_count)
        {
            std::cerr << "surface " << i << " original face count " << metadata.original_face_count << " (metadata) does not match OGF " << source.face_count
                      << std::endl;
            ok = false;
        }

        const uint32_t mesh_face_count = static_cast<uint32_t>(mesh.triangle_index_count() / 3);
        if (mesh_face_count != source.face_count)
        {
            std::cerr << "surface " << i << " exported triangle count " << mesh_face_count << " differs from OGF face count " << source.face_count << std::endl;
            ok = false;
        }

        if (mesh.vertex_count() != static_cast<int>(source.vertex_count))
        {
            std::cerr << "surface " << i << " exported vertex count " << mesh.vertex_count() << " differs from OGF vertex count " << source.vertex_count
                      << std::endl;
            ok = false;
        }
    }

    return ok;
}

bool TestViewerMeshMatchesBaseline()
{
    if (!EnsureSkeletonGenerated())
        return false;

    if (!ConvertBundle(true))
        return false;

    const fs::path skinning_dump = MeshSkinningOutputPath();
    std::error_code remove_error;
    fs::remove(skinning_dump, remove_error);

    std::vector<std::string> args = {
        std::string("--bundle=") + BundleOutputPath().string(),
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
            std::cerr << "bone name mismatch at index " << bone_index << ": '" << viewer_name << "' vs '" << baseline_name << "'" << std::endl;
            ok = false;
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

    if (viewer_meshes.size() > 0)
    {
        const Json& viewer_mesh = viewer_meshes[0];
        if (viewer_mesh.contains("joint_palette"))
            ok &= ValidateJointPalette(viewer_mesh["joint_palette"], skeleton, "mesh[0].joint_palette");
    }

    std::unordered_map<std::string, std::vector<BaselineVertex>> baseline_vertex_map;
    if (!LoadCombinedBaselineVertices(skeleton, baseline_vertex_map))
        return false;

    size_t matched_vertices = 0;
    size_t logged_unmatched = 0;

    for (size_t mesh_index = 0; mesh_index < viewer_meshes.size(); ++mesh_index)
    {
        const Json& viewer_mesh = viewer_meshes[mesh_index];
        if (!viewer_mesh.contains("vertices") || !viewer_mesh["vertices"].is_array())
        {
            std::cerr << "viewer mesh[" << mesh_index << "] lacks vertex array" << std::endl;
            return false;
        }

        const Json& viewer_vertices = viewer_mesh["vertices"];
        for (size_t vertex_index = 0; vertex_index < viewer_vertices.size(); ++vertex_index)
        {
            const Json& viewer_vertex = viewer_vertices[vertex_index];
            const std::array<double, 3> viewer_position_ozz{ viewer_vertex["position"][0].get<double>(), viewer_vertex["position"][1].get<double>(),
                viewer_vertex["position"][2].get<double>() };

            const Json& weights_json = viewer_vertex.value("weights", Json::array());
            if (!weights_json.is_array() || weights_json.empty())
            {
                std::cerr << "viewer mesh vertex missing weights" << std::endl;
                ok = false;
                continue;
            }

            std::vector<std::pair<int, double>> viewer_weights;
            viewer_weights.reserve(weights_json.size());
            for (const auto& entry : weights_json)
            {
                if (!entry.is_array() || entry.size() < 2)
                    continue;
                const int joint_index = entry[0].get<int>();
                const double weight = entry[1].get<double>();
                if (weight <= 0.f)
                    continue;
                viewer_weights.emplace_back(joint_index, weight);
            }

            if (viewer_weights.empty())
            {
                std::cerr << "viewer vertex has zero effective weights" << std::endl;
                ok = false;
                continue;
            }

            std::stable_sort(viewer_weights.begin(), viewer_weights.end(),
                [](const auto& lhs, const auto& rhs)
                {
                    return lhs.second > rhs.second;
                });

            double viewer_weight_sum = 0.0;
            for (const auto& entry : viewer_weights)
                viewer_weight_sum += entry.second;
            if (viewer_weight_sum > std::numeric_limits<double>::epsilon())
            {
                const double inv_sum = 1.0 / viewer_weight_sum;
                for (auto& entry : viewer_weights)
                    entry.second *= inv_sum;
            }

            std::sort(viewer_weights.begin(), viewer_weights.end(),
                [](const auto& lhs, const auto& rhs)
                {
                    if (lhs.first != rhs.first)
                        return lhs.first < rhs.first;
                    return lhs.second < rhs.second;
                });

            const std::string signature = BuildVertexSignatureFromComponents(static_cast<double>(viewer_position_ozz[0]),
                static_cast<double>(viewer_position_ozz[1]), static_cast<double>(viewer_position_ozz[2]), viewer_weights);

            auto baseline_it = baseline_vertex_map.find(signature);
            if (baseline_it == baseline_vertex_map.end())
            {
                if (logged_unmatched < 10)
                {
                    const int vertex_label = viewer_vertex.value("index", static_cast<int>(vertex_index));
                    std::cerr << "no baseline entry matches viewer vertex signature for vertex " << vertex_label << ": " << signature << std::endl;
                    ++logged_unmatched;
                }
                ok = false;
                continue;
            }

            if (baseline_it->second.empty())
            {
                // Duplicate vertex beyond baseline coverage; accepted when deduplication is disabled.
                continue;
            }

            BaselineVertex baseline_vertex = baseline_it->second.back();
            baseline_it->second.pop_back();

            const float dx = std::fabs(viewer_position_ozz[0] - baseline_vertex.position[0]);
            const float dy = std::fabs(viewer_position_ozz[1] - baseline_vertex.position[1]);
            const float dz = std::fabs(viewer_position_ozz[2] - baseline_vertex.position[2]);
            if (dx > kMeshPositionTolerance || dy > kMeshPositionTolerance || dz > kMeshPositionTolerance)
            {
                std::cerr << "vertex position mismatch exceeds tolerance" << std::endl;
                ok = false;
            }

            if (viewer_weights.size() != baseline_vertex.weights.size())
            {
                std::cerr << "vertex weight count mismatch (viewer " << viewer_weights.size() << " vs baseline " << baseline_vertex.weights.size() << ")"
                          << std::endl;
                ok = false;
            }
            else
            {
                for (size_t weight_idx = 0; weight_idx < viewer_weights.size(); ++weight_idx)
                {
                    const auto& vw = viewer_weights[weight_idx];
                    const auto& bw = baseline_vertex.weights[weight_idx];
                    if (vw.first != bw.first)
                    {
                        std::cerr << "vertex joint index mismatch (viewer " << vw.first << " vs baseline " << bw.first << ")" << std::endl;
                        ok = false;
                    }
                    const float diff = std::fabs(vw.second - bw.second);
                    if (diff > kMeshWeightTolerance)
                    {
                        std::cerr << "vertex weight mismatch for joint " << vw.first << " (viewer " << vw.second << " vs baseline " << bw.second << ", tol "
                                  << kMeshWeightTolerance << ")" << std::endl;
                        ok = false;
                    }
                }
            }

            ++matched_vertices;
        }
    }

    for (const auto& [signature, remaining] : baseline_vertex_map)
    {
        if (!remaining.empty())
        {
            std::cerr << "baseline contains unmatched vertex signature" << std::endl;
            ok = false;
            break;
        }
    }

    if (matched_vertices != kExpectedStalkerHeroVertexCount)
    {
        std::cerr << "matched vertex count " << matched_vertices << " differs from expected " << kExpectedStalkerHeroVertexCount << std::endl;
        ok = false;
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
        std::cerr << "animation track count " << animation.num_tracks() << " does not match skeleton joints " << skeleton.num_joints() << std::endl;
        return false;
    }

    return animation.duration() > 0.f;
}

bool CompareQuaternion(const ozz::math::Quaternion& actual, float qx, float qy, float qz, float qw)
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

    const std::array<int, 3> frames = { 0, total_frames / 2, total_frames - 1 };
    const std::array<const char*, 3> tracked_joints = { "bip01_pelvis", "bip01_spine", "bip01_head" };

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
                std::cerr << "frame " << frame << " joint '" << joint_name << "' translation mismatch\n"
                          << "  expected: [" << expected.translation.x << ", " << expected.translation.y << ", " << expected.translation.z << "]\n"
                          << "  actual:   [" << actual.translation.x << ", " << actual.translation.y << ", " << actual.translation.z << "]\n";
                ok = false;
            }

            if (!CompareQuaternion(actual.rotation, expected.rotation.x, expected.rotation.y, expected.rotation.z, expected.rotation.w))
            {
                std::cerr << "frame " << frame << " joint '" << joint_name << "' rotation mismatch\n";
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
        std::cerr << "expected " << kExpectedMultiMotionNames.size() << " animations, found " << animation_count << std::endl;
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
            std::cerr << "metadata name mismatch for animation '" << name << "', metadata reports '" << metadata_name << "'" << std::endl;
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
        std::cerr << "expected " << kExpectedMultiMotionNames.size() << " animations, found " << animation_count << std::endl;
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
            std::cerr << "metadata name mismatch for animation index " << i << " ('" << name << "' vs '" << metadata_name << "')" << std::endl;
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

bool TestAssetBundleContainsSkeletonAndMesh()
{
    if (!TestGenerateMesh())
        return false;

    if (!ConvertBundle(true))
        return false;

    XRay::Animation::OzzxBundle bundle;
    if (!XRay::Animation::ReadOzzxBundle(BundleOutputPath(), bundle))
        return false;

    if (bundle.version != 1u)
    {
        std::cerr << "unexpected bundle version " << bundle.version << std::endl;
        return false;
    }

    if (bundle.skeleton.empty())
    {
        std::cerr << "bundle missing skeleton payload" << std::endl;
        return false;
    }

    if (bundle.mesh.empty())
    {
        std::cerr << "bundle missing mesh payload" << std::endl;
        return false;
    }

    ozz::animation::Skeleton skeleton;
    {
        ozz::io::MemoryStream skeleton_stream;
        if (!skeleton_stream.Write(bundle.skeleton.data(), bundle.skeleton.size()))
        {
            std::cerr << "failed to seed skeleton stream" << std::endl;
            return false;
        }
        skeleton_stream.Seek(0, ozz::io::Stream::kSet);
        ozz::io::IArchive archive(&skeleton_stream);
        if (!archive.TestTag<ozz::animation::Skeleton>())
        {
            std::cerr << "skeleton payload missing Ozz tag" << std::endl;
            return false;
        }
        archive >> skeleton;
        if (skeleton.num_joints() <= 0)
        {
            std::cerr << "skeleton payload contains no joints" << std::endl;
            return false;
        }
    }

    std::vector<std::uint8_t> reference_mesh;
    if (!ReadBinaryFile(MeshOutputPath(), reference_mesh))
        return false;

    if (reference_mesh.empty())
    {
        std::cerr << "reference mesh is empty" << std::endl;
        return false;
    }

    if (bundle.mesh.size() != reference_mesh.size())
    {
        std::cerr << "mesh payload size mismatch (" << bundle.mesh.size() << " vs " << reference_mesh.size() << ")" << std::endl;
        return false;
    }

    if (bundle.mesh != reference_mesh)
    {
        std::cerr << "mesh payload differs from reference asset" << std::endl;
        return false;
    }

    return true;
}

TEST(ConverterIntegration, GenerateSkeleton)
{
    EXPECT_TRUE(TestGenerateSkeleton());
}

TEST(ConverterIntegration, GenerateMesh)
{
    EXPECT_TRUE(TestGenerateMesh());
}

TEST(ConverterIntegration, MeshVertexCountsMatchSource)
{
    EXPECT_TRUE(TestMeshVertexCountsMatchSource());
}

TEST(ConverterIntegration, MeshSurfaceCountMatchesSource)
{
    EXPECT_TRUE(TestMeshSurfaceCountMatchesSource());
}

TEST(ConverterIntegration, MeshSurfaceStatsMatchSource)
{
    EXPECT_TRUE(TestMeshSurfaceStatsMatchSource());
}

// TODO: Re-enable once viewer JSON parity tolerances align with Blender baseline export.
TEST(ConverterIntegration, DISABLED_ViewerMeshMatchesBaseline)
{
    EXPECT_TRUE(TestViewerMeshMatchesBaseline());
}

TEST(ConverterIntegration, BindPoseMatchesBlender)
{
    EXPECT_TRUE(TestBindPoseMatchesBlender());
}

TEST(ConverterIntegration, ConvertAnimationProducesFile)
{
    EXPECT_TRUE(TestConvertAnimationProducesFile());
}

TEST(ConverterIntegration, AnimationCompatibleWithSkeleton)
{
    EXPECT_TRUE(TestAnimationCompatibleWithSkeleton());
}

TEST(ConverterIntegration, AnimationMatchesReference)
{
    EXPECT_TRUE(TestAnimationMatchesReference());
}

TEST(ConverterIntegration, MultipleAnimationConversion)
{
    EXPECT_TRUE(TestMultipleAnimationConversion());
}

TEST(ConverterIntegration, AnimationNamesPreserved)
{
    EXPECT_TRUE(TestAnimationNamesPreserved());
}

TEST(ConverterIntegration, AssetBundleContainsSkeletonAndMesh)
{
    EXPECT_TRUE(TestAssetBundleContainsSkeletonAndMesh());
}
} // namespace
