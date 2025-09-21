//----------------------------------------------------------------------------//
//                                                                            //
// ozz-animation is hosted at http://github.com/guillaumeblanc/ozz-animation  //
// and distributed under the MIT License (MIT).                               //
//                                                                            //
// Copyright (c) Guillaume Blanc                                              //
//                                                                            //
// Permission is hereby granted, free of charge, to any person obtaining a    //
// copy of this software and associated documentation files (the "Software"), //
// to deal in the Software without restriction, including without limitation  //
// the rights to use, copy, modify, merge, publish, distribute, sublicense,   //
// and/or sell copies of the Software, and to permit persons to whom the      //
// Software is furnished to do so, subject to the following conditions:       //
//                                                                            //
// The above copyright notice and this permission notice shall be included in //
// all copies or substantial portions of the Software.                        //
//                                                                            //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR //
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   //
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    //
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER //
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    //
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        //
// DEALINGS IN THE SOFTWARE.                                                  //
//                                                                            //
//----------------------------------------------------------------------------//
#include "stdafx.h"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>
#include "../Externals/ozz-animation/samples/framework/application.h"
#include "../Externals/ozz-animation/samples/framework/mesh.h"
#include "../Externals/ozz-animation/samples/framework/renderer.h"
#include "../Externals/ozz-animation/samples/framework/utils.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/ik_two_bone_job.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/log.h"
#include "ozz/base/maths/box.h"
#include "ozz/base/maths/quaternion.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/simd_quaternion.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/vec_float.h"
#include "ozz/options/options.h"
#if defined(__APPLE__)
#    include <OpenGL/gl3.h>
#else
#    include <GL/gl.h>
#endif
#include "../../../Externals/imgui/backends/imgui_impl_glfw.h"
#include "../../../Externals/imgui/backends/imgui_impl_opengl3.h"
#include "../../../Externals/imgui/imgui.h"
#include "../../../Externals/imgui/imgui_internal.h"
#include "ozz/base/span.h"

// Skeleton archive can be specified as an option.
OZZ_OPTIONS_DECLARE_STRING(skeleton, "Path to the skeleton (ozz archive format).", "media/skeleton.ozz", false)

// Animation archive can be specified as an option.
OZZ_OPTIONS_DECLARE_STRING(animation, "Path to the animation (ozz archive format).", "media/animation.ozz", false)

// Mesh archive can be specified as an option.
OZZ_OPTIONS_DECLARE_STRING(mesh, "Path to the skinned mesh (ozz archive format).", "", false)

// Optional JSON export for sampled animation data.
OZZ_OPTIONS_DECLARE_STRING(dump_animation_json, "Path to write sampled animation data as JSON.", "", false)

// Optional JSON export for bind-pose skinning baseline data.
OZZ_OPTIONS_DECLARE_STRING(dump_skinning_json, "Path to write bind-pose skinning data as JSON.", "", false)

OZZ_OPTIONS_DECLARE_STRING(texture_root, "Optional root directory to resolve texture metadata paths.", "", false)

namespace
{
namespace fs = std::filesystem;

constexpr uint32_t MakeFourCC(char a, char b, char c, char d)
{
    return (static_cast<uint32_t>(a)) | (static_cast<uint32_t>(b) << 8) | (static_cast<uint32_t>(c) << 16) | (static_cast<uint32_t>(d) << 24);
}

struct DdsPixelFormat
{
    uint32_t size;
    uint32_t flags;
    uint32_t fourCC;
    uint32_t rgbBitCount;
    uint32_t rBitMask;
    uint32_t gBitMask;
    uint32_t bBitMask;
    uint32_t aBitMask;
};

struct DdsHeader
{
    uint32_t size;
    uint32_t flags;
    uint32_t height;
    uint32_t width;
    uint32_t pitchOrLinearSize;
    uint32_t depth;
    uint32_t mipMapCount;
    uint32_t reserved1[11];
    DdsPixelFormat ddspf;
    uint32_t caps;
    uint32_t caps2;
    uint32_t caps3;
    uint32_t caps4;
    uint32_t reserved2;
};

static_assert(sizeof(DdsPixelFormat) == 32, "Unexpected DDS pixel format header size.");
static_assert(sizeof(DdsHeader) == 124, "Unexpected DDS header size.");

std::vector<float> CollectRecordSamples(const ozz::sample::Record* record)
{
    std::vector<float> samples;
    if (!record)
    {
        return samples;
    }

    const float* begin = record->record_begin();
    const float* end = record->record_end();
    const float* cursor = record->cursor();
    if (!begin || !end || begin == end || !cursor || cursor == end)
    {
        return samples;
    }

    const float* current = cursor;
    const float* segment_end = end;
    while (current < segment_end)
    {
        samples.push_back(*current);
        ++current;
        if (current == end)
        {
            segment_end = cursor;
            current = begin;
        }
    }

    std::reverse(samples.begin(), samples.end());
    return samples;
}

GLuint LoadDdsTexture(const fs::path& path)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        ozz::log::Err() << "Failed to open DDS texture: " << path << std::endl;
        return 0;
    }

    uint32_t magic = 0;
    stream.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    if (!stream || magic != MakeFourCC('D', 'D', 'S', ' '))
    {
        ozz::log::Err() << "Invalid DDS magic for texture: " << path << std::endl;
        return 0;
    }

    DdsHeader header{};
    stream.read(reinterpret_cast<char*>(&header), sizeof(DdsHeader));
    if (!stream || header.size != sizeof(DdsHeader))
    {
        ozz::log::Err() << "Invalid DDS header for texture: " << path << std::endl;
        return 0;
    }

    const uint32_t fourcc = header.ddspf.fourCC;
    GLenum gl_format = 0;
    uint32_t block_size = 0;
    if (fourcc == MakeFourCC('D', 'X', 'T', '1'))
    {
        gl_format = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT;
        block_size = 8;
    }
    else if (fourcc == MakeFourCC('D', 'X', 'T', '3'))
    {
        gl_format = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT;
        block_size = 16;
    }
    else if (fourcc == MakeFourCC('D', 'X', 'T', '5'))
    {
        gl_format = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
        block_size = 16;
    }
    else
    {
        ozz::log::Err() << "Unsupported DDS FourCC in texture: " << path << std::endl;
        return 0;
    }

    const uint32_t mip_count = header.mipMapCount > 0 ? header.mipMapCount : 1;
    const size_t data_offset = static_cast<size_t>(stream.tellg());
    stream.seekg(0, std::ios::end);
    const size_t file_size = static_cast<size_t>(stream.tellg());
    const size_t data_size = file_size - data_offset;
    stream.seekg(data_offset, std::ios::beg);

    std::vector<uint8_t> data(data_size);
    if (!stream.read(reinterpret_cast<char*>(data.data()), data_size))
    {
        ozz::log::Err() << "Failed to read DDS texture data: " << path << std::endl;
        return 0;
    }

    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, mip_count > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    uint32_t width = header.width;
    uint32_t height = header.height;
    const uint8_t* bytes = data.data();

    for (uint32_t level = 0; level < mip_count; ++level)
    {
        const uint32_t ww = std::max(1u, width);
        const uint32_t hh = std::max(1u, height);
        const size_t size = std::max<size_t>(1, ((ww + 3) / 4) * ((hh + 3) / 4) * block_size);

        glCompressedTexImage2D(GL_TEXTURE_2D, level, gl_format, ww, hh, 0, static_cast<GLsizei>(size), bytes);

        bytes += size;
        width = width > 1 ? width / 2 : 1;
        height = height > 1 ? height / 2 : 1;
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    return texture;
}

GLuint LoadTextureFromFile(const fs::path& path)
{
    const std::string extension = path.extension().string();
    std::string lowercase;
    lowercase.resize(extension.size());
    std::transform(extension.begin(), extension.end(), lowercase.begin(),
        [](unsigned char c)
        {
            return static_cast<char>(std::tolower(c));
        });

    if (lowercase == ".dds")
    {
        return LoadDdsTexture(path);
    }

    ozz::log::LogV() << "Unsupported texture format for path: " << path << std::endl;
    return 0;
}

fs::path SanitizeTexturePath(const std::string& raw)
{
    std::string sanitized = raw;
    for (char& ch : sanitized)
    {
        if (ch == '\\')
        {
            ch = fs::path::preferred_separator;
        }
    }
    return fs::path(sanitized);
}

std::vector<fs::path> BuildTextureCandidates(const fs::path& texture, const fs::path& mesh_dir)
{
    std::vector<fs::path> candidates;
    auto add_candidate = [&candidates](const fs::path& candidate)
    {
        if (candidate.empty())
        {
            return;
        }
        for (const fs::path& existing : candidates)
        {
            if (existing == candidate)
            {
                return;
            }
        }
        candidates.push_back(candidate);
    };

    const bool has_extension = texture.has_extension();

    if (texture.is_absolute())
    {
        add_candidate(texture);
    }
    else
    {
        add_candidate(texture);
        add_candidate(mesh_dir / texture);
        add_candidate(fs::current_path() / texture);
    }

    if (!has_extension)
    {
        const fs::path with_dds = texture.string() + ".dds";
        if (texture.is_absolute())
        {
            add_candidate(with_dds);
        }
        else
        {
            add_candidate(with_dds);
            add_candidate(mesh_dir / with_dds);
            add_candidate(fs::current_path() / with_dds);
        }
    }

    return candidates;
}

std::string EscapeJsonString(const char* input)
{
    std::string escaped;
    if (!input)
    {
        return escaped;
    }
    escaped.reserve(std::strlen(input));
    for (const unsigned char ch : std::string_view(input))
    {
        switch (ch)
        {
        case '"':  escaped += "\\\""; break;
        case '\\': escaped += "\\\\"; break;
        case '\b': escaped += "\\b"; break;
        case '\f': escaped += "\\f"; break;
        case '\n': escaped += "\\n"; break;
        case '\r': escaped += "\\r"; break;
        case '\t': escaped += "\\t"; break;
        default:
            if (ch < 0x20)
            {
                std::ostringstream oss;
                oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(ch);
                escaped += oss.str();
            }
            else
            {
                escaped += static_cast<char>(ch);
            }
            break;
        }
    }
    return escaped;
}

std::string FormatFloat(float value, int precision = 6)
{
    std::ostringstream oss;
    oss.imbue(std::locale::classic());
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

std::string GetFileStem(const char* path)
{
    if (path == nullptr)
    {
        return std::string();
    }
    std::string value(path);
    const size_t separator = value.find_last_of("/\\");
    if (separator != std::string::npos)
    {
        value.erase(0, separator + 1);
    }
    const size_t dot = value.find_last_of('.');
    if (dot != std::string::npos)
    {
        value.erase(dot);
    }
    return value;
}

std::array<std::array<float, 4>, 4> MatrixToRows(const ozz::math::Float4x4& matrix)
{
    std::array<std::array<float, 4>, 4> rows{};
    for (int column = 0; column < 4; ++column)
    {
        float values[4];
        ozz::math::StorePtrU(matrix.cols[column], values);
        for (int row = 0; row < 4; ++row)
        {
            rows[row][column] = values[row];
        }
    }
    return rows;
}

std::array<float, 3> ConvertOzzToBlender(float x, float y, float z)
{
    return { x, -z, y };
}

std::string BuildMeshDisplayName(const std::string& base_label, size_t mesh_index, size_t mesh_count)
{
    std::ostringstream oss;
    oss.imbue(std::locale::classic());
    if (!base_label.empty())
    {
        oss << base_label;
        if (mesh_count > 1)
        {
            oss << '_' << mesh_index;
        }
        return oss.str();
    }

    oss << "mesh_" << mesh_index;
    return oss.str();
}
} // namespace

struct MotionMarkData
{
    std::string name;
    std::vector<std::pair<float, float>> intervals;
};

struct MotionMetadataData
{
    std::string name;
    uint32_t flags = 0;
    uint16_t bone_or_part = 0;
    uint16_t motion_id = 0;
    float speed = 0.f;
    float power = 0.f;
    float accrue = 0.f;
    float falloff = 0.f;
    std::vector<MotionMarkData> marks;
};

class DearImGuiLayer
{
public:
    DearImGuiLayer() = default;

    ~DearImGuiLayer()
    {
        Shutdown();
    }

    bool Initialize(GLFWwindow* window)
    {
        if (!window)
        {
            return false;
        }
        if (initialized_)
        {
            window_ = window;
            return true;
        }

        IMGUI_CHECKVERSION();
        context_ = ImGui::CreateContext();
        if (!context_)
        {
            return false;
        }

        ImGui::StyleColorsDark();
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

        if (!ImGui_ImplGlfw_InitForOpenGL(window, true))
        {
            ImGui::DestroyContext(context_);
            context_ = nullptr;
            return false;
        }

        if (!ImGui_ImplOpenGL3_Init(kGlslVersion))
        {
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext(context_);
            context_ = nullptr;
            return false;
        }

        window_ = window;
        initialized_ = true;
        return true;
    }

    void BeginFrame()
    {
        if (!initialized_)
        {
            return;
        }
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }

    void Render()
    {
        if (!initialized_)
        {
            return;
        }
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    void Shutdown()
    {
        if (!initialized_)
        {
            return;
        }
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        if (context_)
        {
            ImGui::DestroyContext(context_);
            context_ = nullptr;
        }
        initialized_ = false;
        window_ = nullptr;
    }

private:
    static constexpr const char* kGlslVersion = "#version 150";

    GLFWwindow* window_ = nullptr;
    ImGuiContext* context_ = nullptr;
    bool initialized_ = false;
};

class PlaybackSampleApplication : public ozz::sample::Application
{
public:
    ~PlaybackSampleApplication() override = default;

protected:
    static constexpr int kNoAnimationIndex = -1;

    struct ViewerUiState
    {
        int current_animation = kNoAnimationIndex;
        bool draw_skeleton = true;
        bool draw_mesh = false;
        ozz::sample::Renderer::Options renderer_options;
        bool log_bones_each_frame = false;
        int log_bone_limit = 0;
        bool show_bone_debug = true;
        int bone_display_limit = 16;
        bool foot_ik_enabled = false;
        float foot_ik_ground_height = 0.f;
        float foot_ik_weight = 1.f;
        float foot_ik_soften = 0.97f;
        float foot_ik_twist_angle_deg = 0.f;
        std::vector<uint8_t> mesh_visibility;
    };

    bool HasAnimationSelected() const
    {
        return ui_state_.current_animation >= 0 && ui_state_.current_animation < static_cast<int>(animations_.size());
    }

    std::string GetAnimationName(int index) const
    {
        if (index < 0 || index >= static_cast<int>(animations_.size()))
        {
            return std::string();
        }
        if (index < static_cast<int>(animation_metadata_.size()))
        {
            const std::string& meta_name = animation_metadata_[index].name;
            if (!meta_name.empty())
            {
                return meta_name;
            }
        }
        const char* runtime_name = animations_[index].name();
        if (runtime_name && runtime_name[0] != '\0')
        {
            return runtime_name;
        }
        return std::string();
    }

    std::string ReadString(ozz::io::IArchive& archive)
    {
        uint32_t length = 0;
        archive >> length;
        std::string value;
        value.resize(length);
        if (length > 0)
        {
            archive >> ozz::io::MakeArray(value.data(), length);
        }
        return value;
    }

    // Read serialized metadata chunk that follows each animation in converter output.
    MotionMetadataData ReadSerializedMetadata(ozz::io::IArchive& archive)
    {
        MotionMetadataData metadata;
        metadata.name = ReadString(archive);
        archive >> metadata.flags;
        archive >> metadata.bone_or_part;
        archive >> metadata.motion_id;
        archive >> metadata.speed;
        archive >> metadata.power;
        archive >> metadata.accrue;
        archive >> metadata.falloff;

        uint32_t mark_count = 0;
        archive >> mark_count;
        metadata.marks.resize(mark_count);
        for (uint32_t mark_index = 0; mark_index < mark_count; ++mark_index)
        {
            MotionMarkData& mark = metadata.marks[mark_index];
            mark.name = ReadString(archive);

            uint32_t interval_count = 0;
            archive >> interval_count;
            mark.intervals.resize(interval_count);
            for (uint32_t interval_index = 0; interval_index < interval_count; ++interval_index)
            {
                float start = 0.f;
                float end = 0.f;
                archive >> start;
                archive >> end;
                mark.intervals[interval_index] = { start, end };
            }
        }

        return metadata;
    }

    // Load multiple animations from a single file
    bool LoadMultipleAnimations(const char* filename)
    {
        ozz::io::File file(filename, "rb");
        if (!file.opened())
        {
            ozz::log::Err() << "Failed to open animation file: " << filename << std::endl;
            return false;
        }

        ozz::io::IArchive archive(&file);

        animation_metadata_.clear();

        // First try to read a single animation (standard format)
        if (archive.TestTag<ozz::animation::Animation>())
        {
            // Single animation file
            animations_.resize(1);
            archive >> animations_[0];
            MotionMetadataData metadata;
            const char* name = animations_[0].name();
            metadata.name = name ? name : "";
            animation_metadata_.push_back(std::move(metadata));
            return true;
        }

        // Reset file position
        file.Seek(0, ozz::io::File::kSet);
        archive = ozz::io::IArchive(&file);

        // Try to read multi-animation format (count + animations)
        uint32_t anim_count = 0;
        archive >> anim_count;

        if (anim_count == 0)
        {
            ozz::log::Err() << "Invalid animation count: " << anim_count << std::endl;
            return false;
        }

        animations_.resize(anim_count);
        animation_metadata_.resize(anim_count);
        for (uint32_t i = 0; i < anim_count; ++i)
        {
            archive >> animations_[i];
            animation_metadata_[i] = ReadSerializedMetadata(archive);
            if (animation_metadata_[i].name.empty())
            {
                const char* animation_name = animations_[i].name();
                if (animation_name)
                {
                    animation_metadata_[i].name = animation_name;
                }
            }
        }

        ozz::log::LogV() << "Loaded " << anim_count << " animations from " << filename << std::endl;
        return true;
    }

    void PrintPoseTable()
    {
        const auto joint_names = skeleton_.joint_names();
        const int joint_count = skeleton_.num_joints();

        size_t name_width_sz = std::strlen("Bone");
        for (int joint = 0; joint < joint_count; ++joint)
        {
            name_width_sz = std::max(name_width_sz, std::strlen(joint_names[joint]));
        }
        const int name_width = static_cast<int>(name_width_sz);

        const std::array<const char*, 6> headers = { "PosX", "PosY", "PosZ", "RotX", "RotY", "RotZ" };
        std::array<int, 6> column_widths{};
        for (size_t i = 0; i < headers.size(); ++i)
        {
            column_widths[i] = std::max<int>(10, std::strlen(headers[i]));
        }

        auto format_value = [](float value, int precision)
        {
            if (!std::isfinite(value))
            {
                return std::string("--");
            }
            if (std::fabs(value) < 1e-6f)
            {
                value = 0.0f;
            }
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(precision);
            if (value >= 0.f)
            {
                oss << ' ';
            }
            oss << value;
            return oss.str();
        };

        std::cout << "\n=== OZZ BIND POSE TABLE ===" << std::endl;
        std::cout << std::left << std::setw(name_width) << "Bone" << "  ";
        for (size_t i = 0; i < headers.size(); ++i)
        {
            std::cout << std::left << std::setw(column_widths[i]) << headers[i];
            if (i + 1 != headers.size())
            {
                std::cout << "  ";
            }
        }
        std::cout << std::endl;

        std::cout << std::string(name_width, '-') << "  ";
        for (size_t i = 0; i < headers.size(); ++i)
        {
            std::cout << std::string(column_widths[i], '-');
            if (i + 1 != headers.size())
            {
                std::cout << "  ";
            }
        }
        std::cout << std::endl;

        for (int joint = 0; joint < joint_count; ++joint)
        {
            const ozz::math::Float4x4& matrix = models_[joint];
            const float tx = ozz::math::GetX(matrix.cols[3]);
            const float ty = ozz::math::GetY(matrix.cols[3]);
            const float tz = ozz::math::GetZ(matrix.cols[3]);

            float rx = std::numeric_limits<float>::quiet_NaN();
            float ry = std::numeric_limits<float>::quiet_NaN();
            float rz = std::numeric_limits<float>::quiet_NaN();
            if (ozz::math::AreAllTrue1(ozz::math::IsOrthogonal(matrix)))
            {
                const ozz::math::SimdFloat4 quat_simd = ozz::math::ToQuaternion(matrix);
                float quat_values[4];
                ozz::math::StorePtrU(quat_simd, quat_values);
                const ozz::math::Quaternion quat(quat_values[0], quat_values[1], quat_values[2], quat_values[3]);
                const ozz::math::Float3 euler = ozz::math::ToEuler(quat);
                rx = euler.x * ozz::math::kRadianToDegree;
                ry = euler.y * ozz::math::kRadianToDegree;
                rz = euler.z * ozz::math::kRadianToDegree;
            }

            std::cout << std::left << std::setw(name_width) << joint_names[joint] << "  " << std::right << std::setw(column_widths[0]) << format_value(tx, 6)
                      << "  " << std::setw(column_widths[1]) << format_value(ty, 6) << "  " << std::setw(column_widths[2]) << format_value(tz, 6) << "  "
                      << std::setw(column_widths[3]) << format_value(rx, 3) << "  " << std::setw(column_widths[4]) << format_value(ry, 3) << "  "
                      << std::setw(column_widths[5]) << format_value(rz, 3) << std::endl;
        }

        std::cout << std::endl;
    }

protected:
    // Updates current animation time and skeleton pose.
    virtual bool OnUpdate(float _dt, float)
    {
        GLFWwindow* window = nullptr;
        if (ozz::sample::Application::GetCurrent())
        {
            window = ozz::sample::Application::GetCurrent()->GetWindow();
        }
        const bool space_down = window && glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
        if (space_down && !space_was_down_ && HasAnimationSelected() && animations_[ui_state_.current_animation].duration() > 0.0f)
        {
            controller_.TogglePlay();
        }
        space_was_down_ = space_down;

        if (HasAnimationSelected() && animations_[ui_state_.current_animation].duration() > 0.0f)
        {
            // Updates current animation time.
            controller_.Update(animations_[ui_state_.current_animation], _dt);

            // Samples optimized animation at t = animation_time_.
            ozz::animation::SamplingJob sampling_job;
            sampling_job.animation = &animations_[ui_state_.current_animation];
            sampling_job.context = &context_;
            sampling_job.ratio = controller_.time_ratio();
            sampling_job.output = make_span(locals_);
            if (!sampling_job.Run())
            {
                return false;
            }
        }
        else
        {
            // No animation selected - use bind pose
            for (int i = 0; i < skeleton_.num_soa_joints(); ++i)
            {
                locals_[i] = skeleton_.joint_rest_poses()[i];
            }
        }

        // Converts from local space to model space matrices.
        ozz::animation::LocalToModelJob ltm_job;
        ltm_job.skeleton = &skeleton_;
        ltm_job.input = make_span(locals_);
        ltm_job.output = make_span(models_);
        if (!ltm_job.Run())
        {
            return false;
        }

        if (!ApplyFootIKAfterLocal())
        {
            return false;
        }

        if (ui_state_.log_bones_each_frame)
        {
            LogBoneTransformsForFrame();
        }

        if (skinning_dump_pending_ && !skinning_dump_path_.empty())
        {
            if (!ExportSkinningToJson(skinning_dump_path_.c_str()))
            {
                return false;
            }
            skinning_dump_pending_ = false;
        }

        return true;
    }

    virtual bool OnDisplay(ozz::sample::Renderer* _renderer)
    {
        bool success = true;
        const ozz::math::Float4x4 transform = ozz::math::Float4x4::identity();

        imgui_frame_ready_ = false;

        if (ui_state_.draw_skeleton)
        {
            success &= _renderer->DrawPosture(skeleton_, make_span(models_), transform);
        }

        if (ui_state_.draw_mesh && !meshes_.empty())
        {
            const bool visibility_missing = ui_state_.mesh_visibility.size() != meshes_.size();
            for (size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index)
            {
                if (!visibility_missing && ui_state_.mesh_visibility[mesh_index] == 0)
                {
                    continue;
                }
                const ozz::sample::Mesh& mesh = meshes_[mesh_index];
                const size_t palette_size = mesh.joint_remaps.size();
                if (palette_size > skinning_matrices_.size())
                {
                    continue;
                }

                for (size_t palette_index = 0; palette_index < palette_size; ++palette_index)
                {
                    const uint16_t joint = mesh.joint_remaps[palette_index];
                    skinning_matrices_[palette_index] = models_[joint] * mesh.inverse_bind_poses[palette_index];
                }

                auto skinning_span = ozz::make_span(skinning_matrices_);
                skinning_span = skinning_span.first(palette_size);

                ozz::sample::Renderer::Options draw_options = ui_state_.renderer_options;
                if (draw_options.texture && mesh_index < mesh_textures_.size())
                {
                    draw_options.texture_override = mesh_textures_[mesh_index];
                }

                success &= _renderer->DrawSkinnedMesh(mesh, skinning_span, transform, draw_options);
            }
        }

        if (imgui_layer_)
        {
            imgui_layer_->BeginFrame();
            DrawDearImGuiPanels();
            imgui_frame_ready_ = true;
        }

        return success;
    }

    bool OnRenderUiOverlay() override
    {
        if (imgui_layer_ && imgui_frame_ready_)
        {
            imgui_layer_->Render();
            imgui_frame_ready_ = false;
        }
        return true;
    }

    virtual bool OnInitialize()
    {
        SetUseSampleGui(false);

        if (!imgui_layer_)
        {
            imgui_layer_ = std::make_unique<DearImGuiLayer>();
        }
        GLFWwindow* window = nullptr;
        if (ozz::sample::Application::GetCurrent())
        {
            window = ozz::sample::Application::GetCurrent()->GetWindow();
        }
        if (window && imgui_layer_)
        {
            if (!imgui_layer_->Initialize(window))
            {
                ozz::log::Err() << "Failed to initialize Dear ImGui layer." << std::endl;
            }
        }

        // Reading skeleton.
        if (!ozz::sample::LoadSkeleton(OPTIONS_skeleton, &skeleton_))
        {
            return false;
        }
        skeleton_label_ = GetFileStem(OPTIONS_skeleton);

        // Try reading animation(s), but allow failure for bind pose testing
        bool has_animation = LoadMultipleAnimations(OPTIONS_animation);

        if (has_animation && !animations_.empty())
        {
            // Skeleton and animation needs to match.
            if (skeleton_.num_joints() != animations_[0].num_tracks())
            {
                return false;
            }
        }

        // Debug: output skeleton information
        std::cout << "\n=== DEBUG SKELETON INFO ===" << std::endl;
        std::cout << "Skeleton joints: " << skeleton_.num_joints() << std::endl;
        if (has_animation && !animations_.empty())
        {
            std::cout << "Loaded " << animations_.size() << " animations" << std::endl;
            for (size_t i = 0; i < animations_.size(); ++i)
            {
                const char* animation_name = animations_[i].name();
                const std::string display_name = animation_name && animation_name[0] != '\0' ? animation_name :
                    animation_metadata_.size() > i                                           ? animation_metadata_[i].name :
                                                                                               std::string();
                std::cout << "  Animation " << i << " ('" << display_name << "'): tracks=" << animations_[i].num_tracks()
                          << ", duration=" << animations_[i].duration() << " seconds" << std::endl;
            }
        }
        else
        {
            std::cout << "No animation loaded - using bind pose" << std::endl;
        }

        // Output all bone names
        const ozz::span<const char* const> joint_names = skeleton_.joint_names();
        std::cout << "Bone names:" << std::endl;
        for (int i = 0; i < skeleton_.num_joints(); ++i)
        {
            std::cout << "  [" << i << "] " << joint_names[i] << std::endl;
        }

        ui_state_.current_animation = animations_.empty() ? kNoAnimationIndex : 0;

        // Allocates runtime buffers.
        const int num_soa_joints = skeleton_.num_soa_joints();
        locals_.resize(num_soa_joints);
        const int num_joints = skeleton_.num_joints();
        models_.resize(num_joints);

        // Allocates a context that matches animation requirements.
        context_.Resize(num_joints);

        const bool mesh_requested = OPTIONS_mesh && OPTIONS_mesh[0] != '\0';
        if (mesh_requested)
        {
            if (!ozz::sample::LoadMeshes(OPTIONS_mesh, &meshes_))
            {
                return false;
            }
            mesh_label_ = GetFileStem(OPTIONS_mesh);
            size_t skinning_count = 0;
            for (const ozz::sample::Mesh& mesh : meshes_)
            {
                skinning_count = std::max(skinning_count, mesh.joint_remaps.size());
            }
            skinning_matrices_.resize(skinning_count);
            for (const ozz::sample::Mesh& mesh : meshes_)
            {
                if (num_joints < mesh.highest_joint_index())
                {
                    ozz::log::Err() << "The provided mesh doesn't match skeleton"
                                    << " (joint count mismatch)." << std::endl;
                    return false;
                }
            }
            ui_state_.draw_mesh = true;
            LoadMeshTextures(OPTIONS_mesh);
            RefreshMeshDisplayState();
        }
        else
        {
            meshes_.clear();
            skinning_matrices_.clear();
            ui_state_.draw_mesh = false;
            mesh_names_.clear();
            ui_state_.mesh_visibility.clear();
            ReleaseMeshTextures();
        }

        if (!ComputeBindPoseModelMatrices())
        {
            return false;
        }

        InitializeFootIKChains();

        ui_state_.log_bone_limit = num_joints > 0 ? num_joints : 0;
        ui_state_.bone_display_limit = std::min(num_joints, 16);

        if (OPTIONS_dump_animation_json && OPTIONS_dump_animation_json[0] != '\0')
        {
            if (!ExportAnimationToJson(OPTIONS_dump_animation_json))
            {
                return false;
            }
        }

        skinning_dump_path_.clear();
        skinning_dump_pending_ = false;
        if (OPTIONS_dump_skinning_json && OPTIONS_dump_skinning_json[0] != '\0')
        {
            skinning_dump_path_ = OPTIONS_dump_skinning_json;
            skinning_dump_pending_ = true;
        }

        // Optionally set a forced animation time ratio from environment.
        const char* ratio_env = std::getenv("OZZ_SAMPLE_RATIO");
        const char* time_env = std::getenv("OZZ_SAMPLE_TIME");
        if (ratio_env)
        {
            forced_time_ratio_ = std::clamp(static_cast<float>(std::atof(ratio_env)), 0.f, 1.f);
            use_forced_time_ratio_ = true;
        }
        else if (time_env && !animations_.empty())
        {
            const float duration = animations_[ui_state_.current_animation].duration();
            if (duration > 0.f)
            {
                const float sample_time = static_cast<float>(std::atof(time_env));
                forced_time_ratio_ = std::clamp(sample_time / duration, 0.f, 1.f);
                use_forced_time_ratio_ = true;
            }
        }

        if (use_forced_time_ratio_)
        {
            controller_.set_playback_speed(0.f);
            controller_.set_time_ratio(forced_time_ratio_);
        }

        return true;
    }

    void OnDestroy() override
    {
        if (imgui_layer_)
        {
            imgui_layer_->Shutdown();
            imgui_layer_.reset();
        }
        ReleaseMeshTextures();
    }

    virtual bool OnGui(ozz::sample::ImGui*)
    {
        return true;
    }

    virtual void GetSceneBounds(ozz::math::Box* _bound) const
    {
        ozz::sample::ComputePostureBounds(make_span(models_), ozz::math::Float4x4::identity(), _bound);
    }

private:
    // Playback animation controller. This is a utility class that helps with
    // controlling animation playback time.
    ozz::sample::PlaybackController controller_;

    // Runtime skeleton.
    ozz::animation::Skeleton skeleton_;

    // Runtime animations (support multiple).
    ozz::vector<ozz::animation::Animation> animations_;

    // Metadata per animation loaded from converter archives.
    std::vector<MotionMetadataData> animation_metadata_;

    struct FootIkChain
    {
        std::string label;
        int hip = -1;
        int knee = -1;
        int ankle = -1;
        ozz::math::SimdFloat4 mid_axis;
        ozz::math::Float3 target_offset = { 0.f, 0.f, 0.f };
        bool enabled = true;
        bool reached = false;
        std::string hip_name;
        std::string knee_name;
        std::string ankle_name;
    };

    // UI state shared between rendering and presentation layers.
    ViewerUiState ui_state_{};

    std::unique_ptr<DearImGuiLayer> imgui_layer_;
    bool imgui_frame_ready_ = false;

    // Sampling context.
    ozz::animation::SamplingJob::Context context_;

    // Buffer of local transforms as sampled from animation_.
    ozz::vector<ozz::math::SoaTransform> locals_;

    // Buffer of model space matrices.
    ozz::vector<ozz::math::Float4x4> models_;
    ozz::vector<ozz::math::Float4x4> bind_pose_models_;

    // Optional meshes used for skinning validation / rendering.
    ozz::vector<ozz::sample::Mesh> meshes_;
    ozz::vector<ozz::math::Float4x4> skinning_matrices_;

    // Cached labels for reporting/exporting.
    std::string skeleton_label_;
    std::string mesh_label_;
    std::vector<std::string> mesh_names_;
    std::vector<GLuint> mesh_textures_;
    std::vector<std::string> mesh_texture_paths_;

    // Optional dump controls.
    std::string skinning_dump_path_;
    bool skinning_dump_pending_ = false;

    // Optional externally forced time ratio for sampling animations.
    float forced_time_ratio_ = 0.f;
    bool use_forced_time_ratio_ = false;

    bool space_was_down_ = false;

    // Cached lowercase joint names for lookup.
    std::vector<std::string> joint_names_lower_;

    // Foot IK configuration/state.
    bool foot_ik_initialized_ = false;
    bool foot_ik_available_ = false;
    FootIkChain left_foot_chain_{};
    FootIkChain right_foot_chain_{};

    void ReleaseMeshTextures()
    {
        if (!mesh_textures_.empty())
        {
            for (GLuint texture : mesh_textures_)
            {
                if (texture != 0)
                {
                    glDeleteTextures(1, &texture);
                }
            }
        }
        mesh_textures_.clear();
        mesh_texture_paths_.clear();
    }

    void LoadMeshTextures(const char* mesh_path)
    {
        ReleaseMeshTextures();
        mesh_textures_.resize(meshes_.size(), 0);
        mesh_texture_paths_.assign(meshes_.size(), std::string());

        if (!mesh_path)
        {
            return;
        }

        const fs::path mesh_file = fs::absolute(fs::path(mesh_path));
        const fs::path mesh_dir = mesh_file.parent_path();
        fs::path override_root;
        if (OPTIONS_texture_root && OPTIONS_texture_root[0] != '\0')
        {
            override_root = fs::absolute(fs::path(std::string(OPTIONS_texture_root)));
        }

        for (size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index)
        {
            const ozz::sample::XRayMeshMetadata& metadata = meshes_[mesh_index].xray_metadata;
            if (metadata.texture_path.empty())
            {
                continue;
            }

            const fs::path relative = SanitizeTexturePath(metadata.texture_path);
            std::vector<fs::path> candidates = BuildTextureCandidates(relative, mesh_dir);
            if (!override_root.empty())
            {
                std::vector<fs::path> override_candidates = BuildTextureCandidates(relative, override_root);
                candidates.insert(candidates.end(), override_candidates.begin(), override_candidates.end());
            }

            GLuint texture = 0;
            std::string last_candidate;
            for (const fs::path& candidate : candidates)
            {
                const std::string abs_candidate = fs::absolute(candidate).string();
                last_candidate = abs_candidate;

                if (!fs::exists(candidate))
                {
                    continue;
                }

                texture = LoadTextureFromFile(candidate);
                if (texture != 0)
                {
                    ozz::log::Out() << "Loaded texture for mesh " << mesh_index << " from " << abs_candidate << std::endl;
                    mesh_texture_paths_[mesh_index] = abs_candidate;
                    break;
                }
            }

            if (texture == 0)
            {
                ozz::log::Out() << "Unable to load texture '" << metadata.texture_path << "' for mesh " << mesh_index;
                if (!last_candidate.empty())
                {
                    ozz::log::Out() << " (last candidate: " << last_candidate << ")";
                }
                ozz::log::Out() << std::endl;
                mesh_texture_paths_[mesh_index] = last_candidate;
            }

            mesh_textures_[mesh_index] = texture;
        }
    }

    void RefreshMeshDisplayState()
    {
        mesh_names_.clear();
        ui_state_.mesh_visibility.assign(meshes_.size(), 1);

        mesh_names_.reserve(meshes_.size());
        const size_t mesh_count = meshes_.size();
        for (size_t mesh_index = 0; mesh_index < mesh_count; ++mesh_index)
        {
            const ozz::sample::Mesh& mesh = meshes_[mesh_index];
            const ozz::sample::XRayMeshMetadata& metadata = mesh.xray_metadata;

            std::string display_name;
            if (!metadata.texture_path.empty())
            {
                display_name = metadata.texture_path;
            }
            else if (!metadata.shader_name.empty())
            {
                display_name = metadata.shader_name;
            }

            if (!display_name.empty())
            {
                if (mesh_count > 1)
                {
                    display_name += "_" + std::to_string(mesh_index);
                }
            }
            else
            {
                display_name = BuildMeshDisplayName(mesh_label_, mesh_index, mesh_count);
            }

            mesh_names_.push_back(std::move(display_name));
        }
    }

    void SetCurrentAnimation(int index)
    {
        int resolved = index;
        if (animations_.empty())
        {
            resolved = kNoAnimationIndex;
        }
        else
        {
            const int max_index = static_cast<int>(animations_.size()) - 1;
            resolved = std::clamp(index, kNoAnimationIndex, max_index);
        }

        if (resolved == ui_state_.current_animation)
        {
            return;
        }

        ui_state_.current_animation = resolved;
        controller_.Reset();
        context_.Invalidate();
        if (!skinning_dump_path_.empty())
        {
            skinning_dump_pending_ = true;
        }
    }

    void DrawDearImGuiPanels()
    {
        ImGuiIO& io = ImGui::GetIO();
        if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable)
        {
            ImGuiViewport* viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(viewport->WorkPos);
            ImGui::SetNextWindowSize(viewport->WorkSize);
            ImGui::SetNextWindowViewport(viewport->ID);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

            const ImGuiWindowFlags host_flags = ImGuiWindowFlags_NoDocking |
                ImGuiWindowFlags_NoTitleBar |
                ImGuiWindowFlags_NoCollapse |
                ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoBringToFrontOnFocus |
                ImGuiWindowFlags_NoNavFocus |
                ImGuiWindowFlags_NoBackground;

            if (ImGui::Begin("##ViewerDockHost", nullptr, host_flags))
            {
                const ImGuiDockNodeFlags dock_flags = ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode;
                const ImGuiID dockspace_id = ImGui::GetID("ViewerDockSpace");
                ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dock_flags);

                static bool dock_built = false;
                if (!dock_built)
                {
                    dock_built = true;
                    ImGui::DockBuilderRemoveNode(dockspace_id);
                    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
                    ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);

                    ImGuiID dock_main_id = dockspace_id;
                    ImGuiID dock_right_id = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Right, 0.32f, nullptr, &dock_main_id);
                    ImGuiID dock_left_id = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Left, 0.23f, nullptr, &dock_main_id);
                    ImGuiID dock_right_bottom_id = ImGui::DockBuilderSplitNode(dock_right_id, ImGuiDir_Down, 0.42f, nullptr, &dock_right_id);

                    ImGui::DockBuilderDockWindow("Performance", dock_left_id);
                    ImGui::DockBuilderDockWindow("Animation", dock_right_id);
                    ImGui::DockBuilderDockWindow("Inverse Kinematics", dock_right_id);
                    ImGui::DockBuilderDockWindow("Rendering", dock_right_id);
                    ImGui::DockBuilderDockWindow("Metadata", dock_right_id);
                    ImGui::DockBuilderDockWindow("Logging", dock_right_bottom_id);
                    ImGui::DockBuilderDockWindow("Bone Transforms", dock_right_bottom_id);

                    ImGui::DockBuilderFinish(dockspace_id);
                }
            }
            ImGui::End();
            ImGui::PopStyleVar(3);
        }

        DrawPerformancePanel();
        DrawAnimationPanel();
        DrawRenderingPanel();
        DrawFootIkPanel();
        DrawLoggingPanel();
        DrawBoneTransformsPanel();
        DrawMetadataPanel();
    }

    void DrawPerformancePanel()
    {
        if (!ImGui::Begin("Performance"))
        {
            ImGui::End();
            return;
        }

        const auto draw_timing_section = [&](const char* label, const char* plot_label, ozz::sample::Record* record, bool show_fps)
        {
            if (!record)
            {
                return;
            }

            std::vector<float> samples = CollectRecordSamples(record);
            if (samples.empty())
            {
                return;
            }

            ozz::sample::Record::Statistics stats = record->GetStatistics();

            ImGui::SeparatorText(label);

            if (show_fps)
            {
                const float fps = stats.mean > 0.f ? 1000.f / stats.mean : 0.f;
                ImGui::Text("Average: %.2f ms  (%.0f fps)", stats.mean, fps);
            }
            else
            {
                ImGui::Text("Average: %.2f ms", stats.mean);
            }
            ImGui::Text("Latest: %.2f ms     Min: %.2f ms     Max: %.2f ms", stats.latest, stats.min, stats.max);

            const auto max_it = std::max_element(samples.begin(), samples.end());
            float y_max = max_it != samples.end() ? *max_it : stats.max;
            y_max = std::max(y_max, stats.max);
            if (!std::isfinite(y_max) || y_max <= 0.f)
            {
                y_max = 1.f;
            }

            ImGui::PlotLines(plot_label, samples.data(), static_cast<int>(samples.size()), 0, nullptr, 0.f, y_max * 1.1f, ImVec2(-1.0f, 72.0f));
        };

        draw_timing_section("Frame", "Frame time (ms)", GetFpsRecord(), true);
        draw_timing_section("Update", "Update (ms)", GetUpdateTimeRecord(), false);
        draw_timing_section("Render", "Render (ms)", GetRenderTimeRecord(), false);

        ImGui::End();
    }

    void DrawAnimationPanel()
    {
        if (!ImGui::Begin("Animation"))
        {
            ImGui::End();
            return;
        }

        const int animation_count = static_cast<int>(animations_.size());
        const bool has_animations = animation_count > 0;

        if (ImGui::Button("Previous"))
        {
            if (has_animations)
            {
                int target = ui_state_.current_animation;
                if (target == kNoAnimationIndex)
                {
                    target = animation_count - 1;
                }
                else if (target > 0)
                {
                    --target;
                }
                else
                {
                    target = kNoAnimationIndex;
                }
                SetCurrentAnimation(target);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Next"))
        {
            if (has_animations)
            {
                int target = ui_state_.current_animation;
                if (target == kNoAnimationIndex)
                {
                    target = 0;
                }
                else if (target < animation_count - 1)
                {
                    ++target;
                }
                else
                {
                    target = kNoAnimationIndex;
                }
                SetCurrentAnimation(target);
            }
        }

        if (!has_animations)
        {
            ImGui::Separator();
            ImGui::TextUnformatted("No animations loaded. Displaying bind pose.");
        }
        else
        {
            if (ImGui::RadioButton("Bind pose", ui_state_.current_animation == kNoAnimationIndex))
            {
                SetCurrentAnimation(kNoAnimationIndex);
            }

            if (ImGui::BeginListBox("##AnimationList"))
            {
                for (int i = 0; i < animation_count; ++i)
                {
                    const std::string name = GetAnimationName(i);
                    const bool selected = ui_state_.current_animation == i;
                    const std::string display_name = name.empty() ? ("Animation " + std::to_string(i + 1)) : name;
                    const std::string label = display_name + "##anim" + std::to_string(i);
                    if (ImGui::Selectable(label.c_str(), selected))
                    {
                        SetCurrentAnimation(i);
                    }
                    if (selected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndListBox();
            }
        }

        ImGui::Separator();

        if (HasAnimationSelected())
        {
            const int index = ui_state_.current_animation;
            const ozz::animation::Animation& animation = animations_[index];
            const MotionMetadataData* metadata = animation_metadata_.size() > static_cast<size_t>(index) ? &animation_metadata_[index] : nullptr;

            DrawPlaybackControls(animation);

            if (metadata)
            {
                ImGui::Spacing();
                ImGui::Text("Flags: 0x%08X", metadata->flags);
                ImGui::Text("Motion ID: %u", metadata->motion_id);
                ImGui::Text("Speed: %.3f  Power: %.3f", metadata->speed, metadata->power);
                ImGui::Text("Accrue: %.3f  Falloff: %.3f", metadata->accrue, metadata->falloff);
                if (!metadata->marks.empty())
                {
                    ImGui::SeparatorText("Marks");
                    for (const MotionMarkData& mark : metadata->marks)
                    {
                        std::ostringstream mark_stream;
                        mark_stream.imbue(std::locale::classic());
                        mark_stream << mark.name << " [";
                        for (size_t idx = 0; idx < mark.intervals.size(); ++idx)
                        {
                            mark_stream << std::fixed << std::setprecision(3) << mark.intervals[idx].first << " - " << mark.intervals[idx].second;
                            if (idx + 1 < mark.intervals.size())
                            {
                                mark_stream << ", ";
                            }
                        }
                        mark_stream << "]";
                        ImGui::BulletText("%s", mark_stream.str().c_str());
                    }
                }
            }
        }
        else
        {
            ImGui::TextUnformatted("Bind pose (no animation).");
        }

        ImGui::End();
    }

    void DrawPlaybackControls(const ozz::animation::Animation& animation)
    {
        const float duration = animation.duration();

        bool playing = controller_.playing();
        if (ImGui::Checkbox("Play", &playing))
        {
            controller_.set_play(playing);
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop"))
        {
            controller_.Reset();
            controller_.set_play(false);
        }

        bool loop = controller_.loop();
        if (ImGui::Checkbox("Loop", &loop))
        {
            controller_.set_loop(loop);
        }

        float speed = controller_.playback_speed();
        if (ImGui::SliderFloat("Speed", &speed, -4.f, 4.f, "%.2fx"))
        {
            controller_.set_playback_speed(speed);
        }

        float ratio = controller_.time_ratio();
        if (ImGui::SliderFloat("Time", &ratio, 0.f, 1.f, "%.3f"))
        {
            controller_.set_time_ratio(ratio);
        }

        ImGui::Text("Duration: %.3f s", duration);
        ImGui::Text("Current time: %.3f s", duration * controller_.time_ratio());
    }

    void DrawRenderingPanel()
    {
        if (!ImGui::Begin("Rendering"))
        {
            ImGui::End();
            return;
        }

        ImGui::Checkbox("Draw skeleton", &ui_state_.draw_skeleton);
        if (!meshes_.empty())
        {
            ImGui::Checkbox("Draw mesh", &ui_state_.draw_mesh);

            ImGui::Checkbox("Show triangles", &ui_state_.renderer_options.triangles);
            if (ImGui::Checkbox("Show texture", &ui_state_.renderer_options.texture))
            {
                if (ui_state_.renderer_options.texture)
                {
                    ozz::log::Out() << "Texture rendering enabled." << std::endl;
                    for (size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index)
                    {
                        const auto& metadata = meshes_[mesh_index].xray_metadata;
                        const std::string resolved = mesh_index < mesh_texture_paths_.size() ? mesh_texture_paths_[mesh_index] : std::string();
                        const bool loaded = mesh_index < mesh_textures_.size() && mesh_textures_[mesh_index] != 0;
                        ozz::log::Out() << "mesh[" << mesh_index << "] metadata='" << metadata.texture_path << "' resolved='"
                                        << (resolved.empty() ? "" : resolved) << "' loaded=" << (loaded ? "yes" : "no") << std::endl;
                    }
                }
                else
                {
                    ozz::log::Out() << "Texture rendering disabled." << std::endl;
                }
            }
            ImGui::Checkbox("Show vertices", &ui_state_.renderer_options.vertices);
            ImGui::Checkbox("Show normals", &ui_state_.renderer_options.normals);
            ImGui::Checkbox("Show tangents", &ui_state_.renderer_options.tangents);
            ImGui::Checkbox("Show binormals", &ui_state_.renderer_options.binormals);
            ImGui::Checkbox("Show colors", &ui_state_.renderer_options.colors);
            ImGui::Checkbox("Wireframe", &ui_state_.renderer_options.wireframe);
            ImGui::Checkbox("Skip skinning", &ui_state_.renderer_options.skip_skinning);

            if (mesh_names_.size() != meshes_.size() || ui_state_.mesh_visibility.size() != meshes_.size())
            {
                RefreshMeshDisplayState();
            }

            if (ImGui::CollapsingHeader("Mesh Visibility", ImGuiTreeNodeFlags_DefaultOpen))
            {
                for (size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index)
                {
                    bool visible = ui_state_.mesh_visibility[mesh_index] != 0;
                    if (ImGui::Checkbox(mesh_names_[mesh_index].c_str(), &visible))
                    {
                        ui_state_.mesh_visibility[mesh_index] = visible ? 1 : 0;
                    }
                }
            }
        }

        ImGui::End();
    }

    void DrawFootIkPanel()
    {
        if (!ImGui::Begin("Inverse Kinematics"))
        {
            ImGui::End();
            return;
        }

        if (!foot_ik_initialized_)
        {
            ImGui::TextUnformatted("Foot IK initializes after a skeleton is loaded.");
            ImGui::End();
            return;
        }

        if (!foot_ik_available_)
        {
            ImGui::TextUnformatted("No leg chains detected for foot IK on this skeleton.");
            ImGui::End();
            return;
        }

        ImGui::Checkbox("Enable foot IK", &ui_state_.foot_ik_enabled);
        if (ui_state_.foot_ik_enabled)
        {
            ui_state_.foot_ik_weight = std::clamp(ui_state_.foot_ik_weight, 0.f, 1.f);
            const float soften_max = 0.999f;
            ui_state_.foot_ik_soften = std::clamp(ui_state_.foot_ik_soften, 0.f, soften_max);

            ImGui::SliderFloat("Ground height", &ui_state_.foot_ik_ground_height, -2.f, 2.f);
            ImGui::SliderFloat("IK weight", &ui_state_.foot_ik_weight, 0.f, 1.f);
            ImGui::SliderFloat("IK soften", &ui_state_.foot_ik_soften, 0.f, soften_max);
            ImGui::SliderFloat("Twist (deg)", &ui_state_.foot_ik_twist_angle_deg, -180.f, 180.f);

            if (ImGui::CollapsingHeader("Left chain", ImGuiTreeNodeFlags_DefaultOpen))
            {
                DrawFootIkChainPanel(left_foot_chain_);
            }
            if (ImGui::CollapsingHeader("Right chain", ImGuiTreeNodeFlags_DefaultOpen))
            {
                DrawFootIkChainPanel(right_foot_chain_);
            }
        }

        ImGui::End();
    }

    void DrawFootIkChainPanel(FootIkChain& chain)
    {
        if (!FootChainValid(chain))
        {
            ImGui::TextUnformatted("Chain unavailable for this skeleton.");
            return;
        }

        ImGui::PushID(chain.label.c_str());
        ImGui::Checkbox("Enabled", &chain.enabled);
        ImGui::Text("Hip: %s", chain.hip_name.c_str());
        ImGui::Text("Knee: %s", chain.knee_name.c_str());
        ImGui::Text("Ankle: %s", chain.ankle_name.c_str());
        ImGui::Text("Status: %s", chain.reached ? "target reached" : "solving");

        ImGui::SliderFloat("Vertical offset", &chain.target_offset.z, -0.5f, 0.5f);
        ImGui::SliderFloat("Forward offset", &chain.target_offset.x, -0.5f, 0.5f);
        ImGui::SliderFloat("Lateral offset", &chain.target_offset.y, -0.5f, 0.5f);
        ImGui::PopID();
    }

    void DrawLoggingPanel()
    {
        if (!ImGui::Begin("Logging"))
        {
            ImGui::End();
            return;
        }

        ImGui::Checkbox("Log bones every frame", &ui_state_.log_bones_each_frame);
        if (ui_state_.log_bones_each_frame)
        {
            const int max_joints = skeleton_.num_joints();
            if (max_joints > 0)
            {
                ui_state_.log_bone_limit = std::clamp(ui_state_.log_bone_limit, 1, max_joints);
                ImGui::SliderInt("Bones printed", &ui_state_.log_bone_limit, 1, max_joints);
            }
            else
            {
                ui_state_.log_bone_limit = 0;
            }
        }

        ImGui::End();
    }

    void DrawBoneTransformsPanel()
    {
        if (!ImGui::Begin("Bone Transforms"))
        {
            ImGui::End();
            return;
        }

        ImGui::Checkbox("Show bone transforms", &ui_state_.show_bone_debug);

        const int max_joints = skeleton_.num_joints();
        if (max_joints > 0)
        {
            ui_state_.bone_display_limit = std::clamp(ui_state_.bone_display_limit, 1, max_joints);
            ImGui::SliderInt("Bones shown", &ui_state_.bone_display_limit, 1, max_joints);
        }
        else
        {
            ui_state_.bone_display_limit = 0;
        }

        if (ui_state_.show_bone_debug && max_joints > 0)
        {
            const auto joint_names = skeleton_.joint_names();
            const int display_count = std::min(ui_state_.bone_display_limit, max_joints);

            if (HasAnimationSelected())
            {
                const float duration = animations_[ui_state_.current_animation].duration();
                const float ratio = controller_.time_ratio();
                const float time = duration * ratio;
                ImGui::Text("Time %.3fs / %.3fs (ratio %.3f)", time, duration, ratio);
            }
            else
            {
                ImGui::TextUnformatted("Bind pose (no animation)");
            }

            if (ImGui::BeginTable("BoneTransformsTable", 8, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_RowBg))
            {
                ImGui::TableSetupColumn("Bone");
                ImGui::TableSetupColumn("Pos X");
                ImGui::TableSetupColumn("Pos Y");
                ImGui::TableSetupColumn("Pos Z");
                ImGui::TableSetupColumn("Rot W");
                ImGui::TableSetupColumn("Rot X");
                ImGui::TableSetupColumn("Rot Y");
                ImGui::TableSetupColumn("Rot Z");
                ImGui::TableHeadersRow();

                for (int joint = 0; joint < display_count; ++joint)
                {
                    const ozz::math::Float4x4& matrix = models_[joint];
                    const float tx = ozz::math::GetX(matrix.cols[3]);
                    const float ty = ozz::math::GetY(matrix.cols[3]);
                    const float tz = ozz::math::GetZ(matrix.cols[3]);

                    const ozz::math::SimdFloat4 quat_simd = ozz::math::ToQuaternion(matrix);
                    float quat[4];
                    ozz::math::StorePtrU(quat_simd, quat);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(joint_names[joint]);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.3f", tx);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.3f", ty);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.3f", tz);
                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("%.3f", quat[3]);
                    ImGui::TableSetColumnIndex(5);
                    ImGui::Text("%.3f", quat[0]);
                    ImGui::TableSetColumnIndex(6);
                    ImGui::Text("%.3f", quat[1]);
                    ImGui::TableSetColumnIndex(7);
                    ImGui::Text("%.3f", quat[2]);
                }

                ImGui::EndTable();
            }

            if (display_count < max_joints)
            {
                ImGui::Text("Showing %d of %d bones", display_count, max_joints);
            }
        }

        ImGui::End();
    }

    void DrawMetadataPanel()
    {
        if (!ImGui::Begin("Metadata"))
        {
            ImGui::End();
            return;
        }

        ImGui::Text("Skeleton: %s", skeleton_label_.c_str());
        ImGui::Text("Joints: %d", skeleton_.num_joints());
        if (!meshes_.empty())
        {
            ImGui::Separator();
            ImGui::Text("Mesh file: %s", mesh_label_.c_str());
            ImGui::Text("Loaded meshes: %zu", meshes_.size());
        }

        if (!skinning_dump_path_.empty())
        {
            ImGui::Separator();
            ImGui::Text("Skinning dump: %s", skinning_dump_path_.c_str());
            ImGui::Text("Pending: %s", skinning_dump_pending_ ? "yes" : "no");
        }

        ImGui::End();
    }

    bool ComputeBindPoseModelMatrices()
    {
        if (skeleton_.num_joints() == 0)
        {
            models_.clear();
            bind_pose_models_.clear();
            return true;
        }

        const auto rest_poses = skeleton_.joint_rest_poses();
        if (rest_poses.size() != locals_.size())
        {
            locals_.resize(rest_poses.size());
        }
        for (size_t i = 0; i < rest_poses.size(); ++i)
        {
            locals_[i] = rest_poses[i];
        }

        if (models_.size() != static_cast<size_t>(skeleton_.num_joints()))
        {
            models_.resize(skeleton_.num_joints());
        }

        ozz::animation::LocalToModelJob ltm_job;
        ltm_job.skeleton = &skeleton_;
        ltm_job.input = make_span(locals_);
        ltm_job.output = make_span(models_);
        if (!ltm_job.Run())
        {
            ozz::log::Err() << "Failed to build bind-pose model matrices." << std::endl;
            return false;
        }
        bind_pose_models_ = models_;
        return true;
    }

    static std::string ToLowerString(std::string_view value)
    {
        std::string lowered;
        lowered.reserve(value.size());
        for (char ch : value)
        {
            lowered.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
        }
        return lowered;
    }

    int FindJointIndexByNames(std::initializer_list<const char*> candidates) const
    {
        if (joint_names_lower_.empty())
        {
            return -1;
        }
        for (const char* candidate : candidates)
        {
            if (!candidate)
            {
                continue;
            }
            const std::string candidate_lower = ToLowerString(candidate);
            for (size_t index = 0; index < joint_names_lower_.size(); ++index)
            {
                if (joint_names_lower_[index] == candidate_lower)
                {
                    return static_cast<int>(index);
                }
            }
        }
        return -1;
    }

    bool FootChainValid(const FootIkChain& chain) const
    {
        return chain.hip >= 0 && chain.knee >= 0 && chain.ankle >= 0;
    }

    void InitializeFootIKChains()
    {
        const bool previously_initialized = foot_ik_initialized_;
        const bool previous_left_enabled = left_foot_chain_.enabled;
        const bool previous_right_enabled = right_foot_chain_.enabled;

        left_foot_chain_ = FootIkChain{};
        right_foot_chain_ = FootIkChain{};
        left_foot_chain_.label = "Left foot";
        right_foot_chain_.label = "Right foot";

        joint_names_lower_.clear();
        const int joint_count = skeleton_.num_joints();
        joint_names_lower_.reserve(joint_count);
        const auto joint_names = skeleton_.joint_names();
        for (int joint = 0; joint < joint_count; ++joint)
        {
            joint_names_lower_.push_back(ToLowerString(joint_names[joint]));
        }

        auto populate_chain = [&](FootIkChain& chain, std::initializer_list<const char*> hip_candidates, std::initializer_list<const char*> knee_candidates,
                                  std::initializer_list<const char*> ankle_candidates)
        {
            chain.hip = FindJointIndexByNames(hip_candidates);
            chain.knee = FindJointIndexByNames(knee_candidates);
            chain.ankle = FindJointIndexByNames(ankle_candidates);
            chain.hip_name = chain.hip >= 0 ? joint_names[chain.hip] : std::string();
            chain.knee_name = chain.knee >= 0 ? joint_names[chain.knee] : std::string();
            chain.ankle_name = chain.ankle >= 0 ? joint_names[chain.ankle] : std::string();
            chain.mid_axis = ozz::math::simd_float4::z_axis();
            if (FootChainValid(chain) && chain.knee >= 0 && static_cast<size_t>(chain.knee) < bind_pose_models_.size())
            {
                ozz::math::SimdFloat4 axis_candidate = bind_pose_models_[chain.knee].cols[2];
                const ozz::math::SimdFloat4 axis_len_sq = ozz::math::Length3Sqr(axis_candidate);
                const ozz::math::SimdFloat4 min_len = ozz::math::simd_float4::Load1(1e-6f);
                if (ozz::math::AreAllTrue1(ozz::math::CmpGt(axis_len_sq, min_len)))
                {
                    chain.mid_axis = ozz::math::Normalize3(axis_candidate);
                }
            }
            chain.target_offset = { 0.f, 0.f, 0.f };
            chain.reached = false;
        };

        populate_chain(left_foot_chain_, { "bip01_l_thigh", "l_thigh", "left_thigh" }, { "bip01_l_calf", "l_calf", "left_calf", "bip01_l_knee" },
            { "bip01_l_foot", "l_foot", "left_foot", "bip01_l_ankle" });
        populate_chain(right_foot_chain_, { "bip01_r_thigh", "r_thigh", "right_thigh" }, { "bip01_r_calf", "r_calf", "right_calf", "bip01_r_knee" },
            { "bip01_r_foot", "r_foot", "right_foot", "bip01_r_ankle" });

        foot_ik_available_ = FootChainValid(left_foot_chain_) || FootChainValid(right_foot_chain_);

        if (!previously_initialized)
        {
            ui_state_.foot_ik_enabled = false;
            ui_state_.foot_ik_ground_height = 0.f;
            ui_state_.foot_ik_weight = 1.f;
            ui_state_.foot_ik_soften = 0.97f;
            ui_state_.foot_ik_twist_angle_deg = 0.f;
        }
        else if (!foot_ik_available_)
        {
            ui_state_.foot_ik_enabled = false;
        }

        left_foot_chain_.enabled = FootChainValid(left_foot_chain_) && (previously_initialized ? previous_left_enabled : true);
        right_foot_chain_.enabled = FootChainValid(right_foot_chain_) && (previously_initialized ? previous_right_enabled : true);

        if (!FootChainValid(left_foot_chain_))
        {
            left_foot_chain_.enabled = false;
        }
        if (!FootChainValid(right_foot_chain_))
        {
            right_foot_chain_.enabled = false;
        }

        left_foot_chain_.reached = false;
        right_foot_chain_.reached = false;
        foot_ik_initialized_ = true;
    }

    bool ApplyFootIKAfterLocal()
    {
        if (!ui_state_.foot_ik_enabled || !foot_ik_available_)
        {
            left_foot_chain_.reached = false;
            right_foot_chain_.reached = false;
            return true;
        }

        bool applied = false;
        if (left_foot_chain_.enabled && FootChainValid(left_foot_chain_))
        {
            applied |= ApplyFootChainIk(left_foot_chain_);
        }
        else
        {
            left_foot_chain_.reached = false;
        }

        if (right_foot_chain_.enabled && FootChainValid(right_foot_chain_))
        {
            applied |= ApplyFootChainIk(right_foot_chain_);
        }
        else
        {
            right_foot_chain_.reached = false;
        }

        if (applied)
        {
            ozz::animation::LocalToModelJob ltm_job;
            ltm_job.skeleton = &skeleton_;
            ltm_job.input = make_span(locals_);
            ltm_job.output = make_span(models_);
            if (!ltm_job.Run())
            {
                return false;
            }
        }
        return true;
    }

    bool ApplyFootChainIk(FootIkChain& chain)
    {
        chain.reached = false;
        if (!FootChainValid(chain) || !chain.enabled)
        {
            return false;
        }

        const size_t model_count = models_.size();
        if (chain.hip < 0 ||
            chain.knee < 0 ||
            chain.ankle < 0 ||
            static_cast<size_t>(chain.hip) >= model_count ||
            static_cast<size_t>(chain.knee) >= model_count ||
            static_cast<size_t>(chain.ankle) >= model_count)
        {
            return false;
        }

        const ozz::math::Float4x4& ankle_matrix = models_[chain.ankle];
        ozz::math::Float3 target = { ozz::math::GetX(ankle_matrix.cols[3]) + chain.target_offset.x,
            ozz::math::GetY(ankle_matrix.cols[3]) + chain.target_offset.y, ui_state_.foot_ik_ground_height + chain.target_offset.z };

        const ozz::math::SimdFloat4 target_ms = ozz::math::simd_float4::Load3PtrU(&target.x);

        ozz::math::SimdFloat4 pole_vector_ms = models_[chain.knee].cols[1];
        const ozz::math::SimdFloat4 pole_len_sq = ozz::math::Length3Sqr(pole_vector_ms);
        const ozz::math::SimdFloat4 min_len = ozz::math::simd_float4::Load1(1e-6f);
        if (ozz::math::AreAllTrue1(ozz::math::CmpGt(pole_len_sq, min_len)))
        {
            pole_vector_ms = ozz::math::Normalize3(pole_vector_ms);
        }

        ozz::animation::IKTwoBoneJob ik_job;
        ik_job.target = target_ms;
        ik_job.pole_vector = pole_vector_ms;
        ik_job.mid_axis = chain.mid_axis;
        const float clamped_weight = std::clamp(ui_state_.foot_ik_weight, 0.f, 1.f);
        const float soften_max = 0.999f;
        const float clamped_soften = std::clamp(ui_state_.foot_ik_soften, 0.f, soften_max);
        ik_job.weight = clamped_weight;
        ik_job.soften = clamped_soften;
        ik_job.twist_angle = ui_state_.foot_ik_twist_angle_deg * ozz::math::kDegreeToRadian;
        ik_job.start_joint = &models_[chain.hip];
        ik_job.mid_joint = &models_[chain.knee];
        ik_job.end_joint = &models_[chain.ankle];
        ozz::math::SimdQuaternion start_correction;
        ik_job.start_joint_correction = &start_correction;
        ozz::math::SimdQuaternion mid_correction;
        ik_job.mid_joint_correction = &mid_correction;
        ik_job.reached = &chain.reached;

        if (!ik_job.Run())
        {
            chain.reached = false;
            return false;
        }

        ozz::sample::MultiplySoATransformQuaternion(chain.hip, start_correction, make_span(locals_));
        ozz::sample::MultiplySoATransformQuaternion(chain.knee, mid_correction, make_span(locals_));

        return true;
    }

    bool ExportSkinningToJson(const char* path)
    {
        if (path == nullptr || path[0] == '\0')
        {
            return true;
        }
        if (meshes_.empty())
        {
            ozz::log::Err() << "No mesh loaded; cannot export skinning JSON." << std::endl;
            return false;
        }

        const int joint_count = skeleton_.num_joints();
        if (joint_count == 0)
        {
            ozz::log::Err() << "Skeleton has no joints; cannot export skinning JSON." << std::endl;
            return false;
        }

        if (models_.size() != static_cast<size_t>(joint_count))
        {
            if (!ComputeBindPoseModelMatrices())
            {
                return false;
            }
        }

        std::ofstream file(path, std::ios::binary);
        if (!file)
        {
            ozz::log::Err() << "Failed to open skinning JSON export path: " << path << std::endl;
            return false;
        }
        file.imbue(std::locale::classic());

        const auto joint_names = skeleton_.joint_names();

        file << "{\n";
        file << "  \"armature\": {\n";
        file << "    \"name\": \"" << EscapeJsonString(skeleton_label_.c_str()) << "\",\n";
        file << "    \"bones\": [\n";

        for (int joint = 0; joint < joint_count; ++joint)
        {
            const auto rows = MatrixToRows(models_[joint]);
            ozz::math::SimdInt4 invertible;
            const ozz::math::Float4x4 inverse = ozz::math::Invert(models_[joint], &invertible);
            const auto inverse_rows = MatrixToRows(inverse);

            file << "      {\n";
            file << "        \"index\": " << joint << ",\n";
            file << "        \"name\": \"" << EscapeJsonString(joint_names[joint]) << "\",\n";
            file << "        \"matrix_global\": [\n";
            for (int row = 0; row < 4; ++row)
            {
                file << "          [" << FormatFloat(rows[row][0]) << ", " << FormatFloat(rows[row][1]) << ", " << FormatFloat(rows[row][2]) << ", "
                     << FormatFloat(rows[row][3]) << "]";
                file << (row < 3 ? ",\n" : "\n");
            }
            file << "        ],\n";
            file << "        \"matrix_global_inverse\": [\n";
            for (int row = 0; row < 4; ++row)
            {
                file << "          [" << FormatFloat(inverse_rows[row][0]) << ", " << FormatFloat(inverse_rows[row][1]) << ", "
                     << FormatFloat(inverse_rows[row][2]) << ", " << FormatFloat(inverse_rows[row][3]) << "]";
                file << (row < 3 ? ",\n" : "\n");
            }
            file << "        ]\n";
            file << "      }";
            file << (joint + 1 < joint_count ? ",\n" : "\n");
        }

        file << "    ]\n";
        file << "  },\n";
        file << "  \"meshes\": [\n";

        for (size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index)
        {
            const ozz::sample::Mesh& mesh = meshes_[mesh_index];

            const size_t palette_size = mesh.joint_remaps.size();
            if (palette_size > 0)
            {
                skinning_matrices_.resize(palette_size);
                if (mesh.inverse_bind_poses.size() != palette_size)
                {
                    ozz::log::Err() << "Mesh palette size mismatch: remaps=" << mesh.joint_remaps.size()
                                    << " inverse_bind_poses=" << mesh.inverse_bind_poses.size() << std::endl;
                }
                for (size_t palette_index = 0; palette_index < palette_size; ++palette_index)
                {
                    const uint16_t joint_index = mesh.joint_remaps[palette_index];
                    if (joint_index >= models_.size())
                    {
                        ozz::log::Err() << "Palette index " << palette_index << " references joint " << joint_index << " beyond loaded skeleton." << std::endl;
                        continue;
                    }
                    if (mesh.inverse_bind_poses.size() > palette_index)
                    {
                        skinning_matrices_[palette_index] = models_[joint_index] * mesh.inverse_bind_poses[palette_index];
                    }
                    else
                    {
                        skinning_matrices_[palette_index] = models_[joint_index];
                    }
                }
            }
            else
            {
                skinning_matrices_.clear();
            }

            file << "    {\n";
            const std::string mesh_name =
                mesh_names_.size() > mesh_index ? mesh_names_[mesh_index] : BuildMeshDisplayName(mesh_label_, mesh_index, meshes_.size());
            file << "      \"name\": \"" << EscapeJsonString(mesh_name.c_str()) << "\",\n";
            file << "      \"vertex_count\": " << mesh.vertex_count() << ",\n";
            const ozz::sample::XRayMeshMetadata& xray = mesh.xray_metadata;
            file << "      \"xray\": {\n";
            file << "        \"ogf_type\": " << static_cast<int>(xray.ogf_type) << ",\n";
            file << "        \"original_vertex_count\": " << xray.original_vertex_count << ",\n";
            file << "        \"original_face_count\": " << xray.original_face_count << ",\n";
            file << "        \"texture_path\": ";
            if (!xray.texture_path.empty())
            {
                file << "\"" << EscapeJsonString(xray.texture_path.c_str()) << "\"";
            }
            else
            {
                file << "null";
            }
            file << ",\n";
            file << "        \"shader_name\": ";
            if (!xray.shader_name.empty())
            {
                file << "\"" << EscapeJsonString(xray.shader_name.c_str()) << "\"";
            }
            else
            {
                file << "null";
            }
            file << ",\n";
            file << "        \"texture_link\": ";
            if (xray.texture_link_present)
            {
                file << xray.texture_link;
            }
            else
            {
                file << "null";
            }
            file << ",\n";
            file << "        \"shader_link\": ";
            if (xray.shader_link_present)
            {
                file << xray.shader_link;
            }
            else
            {
                file << "null";
            }
            file << ",\n";
            file << "        \"lod_visuals\": [";
            for (size_t lod_index = 0; lod_index < xray.lod_visuals.size(); ++lod_index)
            {
                file << (lod_index == 0 ? "" : ", ") << "\"" << EscapeJsonString(xray.lod_visuals[lod_index].c_str()) << "\"";
            }
            file << "],\n";
            file << "        \"lod_data_bytes\": " << xray.lod_data.size() << ",\n";
            file << "        \"progressive_collapse_count\": " << xray.progressive_collapse_count << ",\n";
            file << "        \"progressive_data_bytes\": " << xray.progressive_data.size() << ",\n";
            file << "        \"child_links\": [";
            for (size_t link_index = 0; link_index < xray.child_visual_links.size(); ++link_index)
            {
                file << (link_index == 0 ? "" : ", ") << xray.child_visual_links[link_index];
            }
            file << "]\n";
            file << "      },\n";
            file << "      \"joint_palette\": [\n";
            for (size_t palette_index = 0; palette_index < palette_size; ++palette_index)
            {
                file << "        {\n";
                file << "          \"palette_index\": " << palette_index << ",\n";
                file << "          \"joint_index\": " << mesh.joint_remaps[palette_index] << ",\n";
                if (mesh.inverse_bind_poses.size() > palette_index)
                {
                    const auto inverse_rows = MatrixToRows(mesh.inverse_bind_poses[palette_index]);
                    file << "          \"inverse_bind_pose\": [\n";
                    for (int row = 0; row < 4; ++row)
                    {
                        file << "            [" << FormatFloat(inverse_rows[row][0]) << ", " << FormatFloat(inverse_rows[row][1]) << ", "
                             << FormatFloat(inverse_rows[row][2]) << ", " << FormatFloat(inverse_rows[row][3]) << "]";
                        file << (row < 3 ? ",\n" : "\n");
                    }
                    file << "          ],\n";
                }
                else
                {
                    file << "          \"inverse_bind_pose\": [],\n";
                }
                if (skinning_matrices_.size() > palette_index)
                {
                    const auto skin_rows = MatrixToRows(skinning_matrices_[palette_index]);
                    file << "          \"skinning_matrix\": [\n";
                    for (int row = 0; row < 4; ++row)
                    {
                        file << "            [" << FormatFloat(skin_rows[row][0]) << ", " << FormatFloat(skin_rows[row][1]) << ", "
                             << FormatFloat(skin_rows[row][2]) << ", " << FormatFloat(skin_rows[row][3]) << "]";
                        file << (row < 3 ? ",\n" : "\n");
                    }
                    file << "          ]\n";
                }
                else
                {
                    file << "          \"skinning_matrix\": []\n";
                }
                file << "        }";
                file << (palette_index + 1 < palette_size ? ",\n" : "\n");
            }
            file << "      ],\n";
            file << "      \"vertices\": [\n";

            int global_vertex = 0;
            for (const ozz::sample::Mesh::Part& part : mesh.parts)
            {
                const int influences = part.influences_count();
                const int vertex_count = part.vertex_count();
                const bool has_normals = part.normals.size() == static_cast<size_t>(vertex_count * ozz::sample::Mesh::Part::kNormalsCpnts);
                for (int v = 0; v < vertex_count; ++v, ++global_vertex)
                {
                    file << "        {\n";
                    file << "          \"index\": " << global_vertex << ",\n";
                    const int pos_offset = v * ozz::sample::Mesh::Part::kPositionsCpnts;
                    const float px = part.positions[pos_offset + 0];
                    const float py = part.positions[pos_offset + 1];
                    const float pz = part.positions[pos_offset + 2];
                    const auto blender_pos = ConvertOzzToBlender(px, py, pz);
                    file << "          \"position\": [" << FormatFloat(blender_pos[0]) << ", " << FormatFloat(blender_pos[1]) << ", "
                         << FormatFloat(blender_pos[2]) << "],\n";

                    float nx = 0.f;
                    float ny = 0.f;
                    float nz = 0.f;
                    if (has_normals)
                    {
                        const int normal_offset = v * ozz::sample::Mesh::Part::kNormalsCpnts;
                        nx = part.normals[normal_offset + 0];
                        ny = part.normals[normal_offset + 1];
                        nz = part.normals[normal_offset + 2];
                        const auto blender_normal = ConvertOzzToBlender(nx, ny, nz);
                        file << "          \"normal\": [" << FormatFloat(blender_normal[0]) << ", " << FormatFloat(blender_normal[1]) << ", "
                             << FormatFloat(blender_normal[2]) << "],\n";
                    }

                    std::vector<std::pair<uint16_t, float>> weights;
                    weights.reserve(influences > 0 ? influences : 1);
                    const bool skinning_available = !skinning_matrices_.empty();
                    const ozz::math::SimdFloat4 rest_position = ozz::math::simd_float4::Load(px, py, pz, 1.f);
                    const ozz::math::SimdFloat4 rest_normal =
                        has_normals ? ozz::math::simd_float4::Load(nx, ny, nz, 0.f) : ozz::math::simd_float4::Load(0.f, 0.f, 0.f, 0.f);
                    float skinned_pos[3] = { 0.f, 0.f, 0.f };
                    float skinned_nrm[3] = { 0.f, 0.f, 0.f };
                    float weight_sum = 0.f;

                    if (influences > 0)
                    {
                        const int joint_base = v * influences;
                        const int weight_base = v * std::max(0, influences - 1);
                        float accum = 0.f;
                        for (int influence = 0; influence < influences; ++influence)
                        {
                            const uint16_t palette_index = part.joint_indices[joint_base + influence];
                            if (palette_index >= mesh.joint_remaps.size())
                            {
                                ozz::log::Err() << "Vertex " << global_vertex << " references out-of-range joint index" << std::endl;
                                continue;
                            }
                            float weight = 0.f;
                            if (influence < influences - 1)
                            {
                                weight = part.joint_weights[weight_base + influence];
                                accum += weight;
                            }
                            else
                            {
                                weight = std::max(0.f, 1.f - accum);
                            }
                            weight = std::clamp(weight, 0.f, 1.f);
                            weight_sum += weight;

                            const uint16_t joint_index = mesh.joint_remaps[palette_index];
                            weights.emplace_back(joint_index, weight);

                            if (skinning_available && weight > 0.f && palette_index < skinning_matrices_.size())
                            {
                                const ozz::math::Float4x4& skin_matrix = skinning_matrices_[palette_index];
                                float transformed_position[4];
                                ozz::math::StorePtrU(ozz::math::TransformPoint(skin_matrix, rest_position), transformed_position);
                                skinned_pos[0] += transformed_position[0] * weight;
                                skinned_pos[1] += transformed_position[1] * weight;
                                skinned_pos[2] += transformed_position[2] * weight;

                                if (has_normals)
                                {
                                    float transformed_normal[4];
                                    ozz::math::StorePtrU(ozz::math::TransformVector(skin_matrix, rest_normal), transformed_normal);
                                    skinned_nrm[0] += transformed_normal[0] * weight;
                                    skinned_nrm[1] += transformed_normal[1] * weight;
                                    skinned_nrm[2] += transformed_normal[2] * weight;
                                }
                            }
                        }
                    }

                    std::sort(weights.begin(), weights.end(),
                        [](const auto& lhs, const auto& rhs)
                        {
                            return lhs.first < rhs.first;
                        });

                    file << "          \"weights\": [";
                    for (size_t idx = 0; idx < weights.size(); ++idx)
                    {
                        file << "[" << weights[idx].first << ", " << FormatFloat(weights[idx].second) << "]";
                        if (idx + 1 < weights.size())
                        {
                            file << ", ";
                        }
                    }
                    file << "],\n";

                    const bool wrote_skinned = skinning_available && !weights.empty();
                    if (wrote_skinned)
                    {
                        const auto blender_skinned = ConvertOzzToBlender(skinned_pos[0], skinned_pos[1], skinned_pos[2]);
                        file << "          \"skinned_position\": [" << FormatFloat(blender_skinned[0]) << ", " << FormatFloat(blender_skinned[1]) << ", "
                             << FormatFloat(blender_skinned[2]) << "]";
                        if (has_normals)
                        {
                            const float length = std::sqrt(skinned_nrm[0] * skinned_nrm[0] + skinned_nrm[1] * skinned_nrm[1] + skinned_nrm[2] * skinned_nrm[2]);
                            if (length > 0.f)
                            {
                                skinned_nrm[0] /= length;
                                skinned_nrm[1] /= length;
                                skinned_nrm[2] /= length;
                            }
                            else
                            {
                                skinned_nrm[0] = nx;
                                skinned_nrm[1] = ny;
                                skinned_nrm[2] = nz;
                            }
                            const auto blender_skinned_normal = ConvertOzzToBlender(skinned_nrm[0], skinned_nrm[1], skinned_nrm[2]);
                            file << ",\n";
                            file << "          \"skinned_normal\": [" << FormatFloat(blender_skinned_normal[0]) << ", "
                                 << FormatFloat(blender_skinned_normal[1]) << ", " << FormatFloat(blender_skinned_normal[2]) << "]";
                        }
                        file << ",\n";
                    }
                    else
                    {
                        file << "          \"skinned_position\": [" << FormatFloat(blender_pos[0]) << ", " << FormatFloat(blender_pos[1]) << ", "
                             << FormatFloat(blender_pos[2]) << "],\n";
                        if (has_normals)
                        {
                            const auto blender_normal_fallback = ConvertOzzToBlender(nx, ny, nz);
                            file << "          \"skinned_normal\": [" << FormatFloat(blender_normal_fallback[0]) << ", "
                                 << FormatFloat(blender_normal_fallback[1]) << ", " << FormatFloat(blender_normal_fallback[2]) << "],\n";
                        }
                    }

                    file << "          \"weight_sum\": " << FormatFloat(weight_sum) << "\n";
                    file << "        }";
                    if (global_vertex + 1 < mesh.vertex_count())
                    {
                        file << ",";
                    }
                    file << "\n";
                }
            }

            file << "      ]\n";
            file << "    }";
            file << (mesh_index + 1 < meshes_.size() ? ",\n" : "\n");
        }

        file << "  ]\n";
        file << "}\n";

        ozz::log::LogV() << "Exported skinning JSON to " << path << std::endl;
        return true;
    }

    void LogBoneTransformsForFrame() const
    {
        const int joint_count = skeleton_.num_joints();
        if (joint_count == 0)
        {
            return;
        }

        const auto joint_names = skeleton_.joint_names();
        const int max_joints = ui_state_.log_bone_limit > 0 ? std::min(ui_state_.log_bone_limit, joint_count) : joint_count;

        std::ostringstream oss;
        oss.imbue(std::locale::classic());

        for (int joint = 0; joint < max_joints; ++joint)
        {
            const ozz::math::Float4x4& matrix = models_[joint];
            const float tx = ozz::math::GetX(matrix.cols[3]);
            const float ty = ozz::math::GetY(matrix.cols[3]);
            const float tz = ozz::math::GetZ(matrix.cols[3]);

            const ozz::math::SimdFloat4 quat_simd = ozz::math::ToQuaternion(matrix);
            float quat[4];
            ozz::math::StorePtrU(quat_simd, quat);

            oss << "  Bone[" << joint << "] '" << joint_names[joint] << "': pos(" << FormatFloat(tx, 6) << ", " << FormatFloat(ty, 6) << ", "
                << FormatFloat(tz, 6) << ") rot(" << FormatFloat(quat[3], 6) << ", " << FormatFloat(quat[0], 6) << ", " << FormatFloat(quat[1], 6) << ", "
                << FormatFloat(quat[2], 6) << ")" << std::endl;
        }

        oss << std::endl;
        std::cout << oss.str();
    }

    bool ExportAnimationToJson(const char* path)
    {
        if (path == nullptr || path[0] == '\0')
        {
            return true;
        }

        if (animations_.empty())
        {
            ozz::log::Err() << "No animation available for JSON export." << std::endl;
            return false;
        }

        if (!HasAnimationSelected())
        {
            ozz::log::Err() << "No animation selected for JSON export." << std::endl;
            return false;
        }

        const ozz::animation::Animation& animation = animations_[ui_state_.current_animation];
        const float duration = animation.duration();
        const ozz::span<const float> timepoints = animation.timepoints();

        ozz::vector<float> samples;
        samples.reserve(timepoints.size() + 2);
        constexpr float kEpsilon = 1e-5f;
        if (timepoints.empty() || timepoints.front() > kEpsilon)
        {
            samples.push_back(0.f);
        }
        samples.insert(samples.end(), timepoints.begin(), timepoints.end());
        if (!samples.empty())
        {
            std::sort(samples.begin(), samples.end());
            samples.erase(std::unique(samples.begin(), samples.end(),
                              [](float a, float b)
                              {
                                  return std::fabs(a - b) <= kEpsilon;
                              }),
                samples.end());
        }
        if (duration > kEpsilon)
        {
            if (samples.empty() || std::fabs(samples.back() - duration) > kEpsilon)
            {
                samples.push_back(duration);
            }
        }
        else if (samples.empty())
        {
            samples.push_back(0.f);
        }

        std::ofstream file(path, std::ios::binary);
        if (!file)
        {
            ozz::log::Err() << "Failed to open JSON export path: " << path << std::endl;
            return false;
        }
        file.imbue(std::locale::classic());

        const int joint_count = skeleton_.num_joints();
        const ozz::span<const char* const> joint_names = skeleton_.joint_names();

        ozz::animation::SamplingJob sampling_job;
        sampling_job.animation = &animation;
        sampling_job.context = &context_;
        sampling_job.output = make_span(locals_);

        ozz::animation::LocalToModelJob ltm_job;
        ltm_job.skeleton = &skeleton_;
        ltm_job.input = make_span(locals_);
        ltm_job.output = make_span(models_);

        const int frame_count = static_cast<int>(samples.size());
        const float frame_rate = duration > kEpsilon && frame_count > 1 ? static_cast<float>(frame_count - 1) / std::max(duration, kEpsilon) : 0.f;

        file << "{\n";
        file << "  \"skeleton\": \"" << EscapeJsonString(OPTIONS_skeleton) << "\",\n";
        file << "  \"animation\": \"" << EscapeJsonString(OPTIONS_animation) << "\",\n";
        file << "  \"duration\": " << FormatFloat(duration) << ",\n";
        file << "  \"frame_rate\": " << FormatFloat(frame_rate) << ",\n";
        file << "  \"frame_count\": " << frame_count << ",\n";
        file << "  \"frame_start\": 0,\n";
        file << "  \"frame_end\": " << (frame_count > 0 ? frame_count - 1 : 0) << ",\n";
        file << "  \"frames\": [\n";

        for (int frame = 0; frame < frame_count; ++frame)
        {
            const float sample_time = samples[frame];
            const float ratio = duration > kEpsilon ? sample_time / duration : 0.f;
            sampling_job.ratio = std::clamp(ratio, 0.f, 1.f);
            if (!sampling_job.Run())
            {
                ozz::log::Err() << "SamplingJob failed at frame " << frame << std::endl;
                return false;
            }
            if (!ltm_job.Run())
            {
                ozz::log::Err() << "LocalToModelJob failed at frame " << frame << std::endl;
                return false;
            }

            file << "    {\n";
            file << "      \"frame\": " << frame << ",\n";
            file << "      \"time\": " << FormatFloat(sample_time) << ",\n";
            file << "      \"ratio\": " << FormatFloat(sampling_job.ratio) << ",\n";
            file << "      \"bones\": {\n";

            for (int joint = 0; joint < joint_count; ++joint)
            {
                const ozz::math::Float4x4& matrix = models_[joint];
                const float tx = ozz::math::GetX(matrix.cols[3]);
                const float ty = ozz::math::GetY(matrix.cols[3]);
                const float tz = ozz::math::GetZ(matrix.cols[3]);

                const ozz::math::SimdFloat4 quat_simd = ozz::math::ToQuaternion(matrix);
                float quat_values[4];
                ozz::math::StorePtrU(quat_simd, quat_values);

                const float sx = ozz::math::GetX(ozz::math::Length3(matrix.cols[0]));
                const float sy = ozz::math::GetX(ozz::math::Length3(matrix.cols[1]));
                const float sz = ozz::math::GetX(ozz::math::Length3(matrix.cols[2]));

                file << "        \"" << EscapeJsonString(joint_names[joint]) << "\": {\n";
                file << "          \"location\": [" << FormatFloat(tx) << ", " << FormatFloat(ty) << ", " << FormatFloat(tz) << "],\n";
                file << "          \"rotation\": [" << FormatFloat(quat_values[0]) << ", " << FormatFloat(quat_values[1]) << ", " << FormatFloat(quat_values[2])
                     << ", " << FormatFloat(quat_values[3]) << "],\n";
                file << "          \"scale\": [" << FormatFloat(sx) << ", " << FormatFloat(sy) << ", " << FormatFloat(sz) << "]\n";
                file << "        }";
                if (joint + 1 != joint_count)
                {
                    file << ",";
                }
                file << "\n";
            }

            file << "      }\n";
            file << "    }";
            if (frame + 1 != frame_count)
            {
                file << ",";
            }
            file << "\n";
        }

        file << "  ]\n";
        file << "}\n";

        ozz::log::LogV() << "Exported animation JSON to " << path << std::endl;
        return true;
    }
};

int main(int _argc, const char** _argv)
{
    const char* title = "Ozz-animation sample: Binary animation/skeleton playback";
    return PlaybackSampleApplication().Run(_argc, _argv, "1.0", title);
}
