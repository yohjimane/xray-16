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
#include "../Externals/ozz-animation/samples/framework/application.h"
#include "../Externals/ozz-animation/samples/framework/imgui.h"
#include "../Externals/ozz-animation/samples/framework/renderer.h"
#include "../Externals/ozz-animation/samples/framework/mesh.h"
#include "../Externals/ozz-animation/samples/framework/utils.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/log.h"
#include "ozz/base/maths/box.h"
#include "ozz/base/maths/quaternion.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/vec_float.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/io/archive.h"
#include "ozz/options/options.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <locale>
#include <cctype>
#include <cstdint>
#include <vector>
#include "GL/glfw.h"
#include "ozz/base/span.h"


// Skeleton archive can be specified as an option.
OZZ_OPTIONS_DECLARE_STRING(skeleton,
                           "Path to the skeleton (ozz archive format).",
                           "media/skeleton.ozz", false)

// Animation archive can be specified as an option.
OZZ_OPTIONS_DECLARE_STRING(animation,
                           "Path to the animation (ozz archive format).",
                           "media/animation.ozz", false)

// Mesh archive can be specified as an option.
OZZ_OPTIONS_DECLARE_STRING(mesh,
                           "Path to the skinned mesh (ozz archive format).",
                           "", false)

// Optional JSON export for sampled animation data.
OZZ_OPTIONS_DECLARE_STRING(dump_animation_json,
                           "Path to write sampled animation data as JSON.",
                           "", false)

// Optional JSON export for bind-pose skinning baseline data.
OZZ_OPTIONS_DECLARE_STRING(dump_skinning_json,
                           "Path to write bind-pose skinning data as JSON.",
                           "", false)

OZZ_OPTIONS_DECLARE_STRING(texture_root,
                           "Optional root directory to resolve texture metadata paths.",
                           "", false)

namespace {

namespace fs = std::filesystem;

constexpr uint32_t MakeFourCC(char a, char b, char c, char d) {
  return (static_cast<uint32_t>(a)) | (static_cast<uint32_t>(b) << 8) |
         (static_cast<uint32_t>(c) << 16) | (static_cast<uint32_t>(d) << 24);
}

struct DdsPixelFormat {
  uint32_t size;
  uint32_t flags;
  uint32_t fourCC;
  uint32_t rgbBitCount;
  uint32_t rBitMask;
  uint32_t gBitMask;
  uint32_t bBitMask;
  uint32_t aBitMask;
};

struct DdsHeader {
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

static_assert(sizeof(DdsPixelFormat) == 32,
              "Unexpected DDS pixel format header size.");
static_assert(sizeof(DdsHeader) == 124,
              "Unexpected DDS header size.");

GLuint LoadDdsTexture(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    ozz::log::Err() << "Failed to open DDS texture: " << path << std::endl;
    return 0;
  }

  uint32_t magic = 0;
  stream.read(reinterpret_cast<char*>(&magic), sizeof(magic));
  if (!stream || magic != MakeFourCC('D', 'D', 'S', ' ')) {
    ozz::log::Err() << "Invalid DDS magic for texture: " << path << std::endl;
    return 0;
  }

  DdsHeader header{};
  stream.read(reinterpret_cast<char*>(&header), sizeof(DdsHeader));
  if (!stream || header.size != sizeof(DdsHeader)) {
    ozz::log::Err() << "Invalid DDS header for texture: " << path << std::endl;
    return 0;
  }

  const uint32_t fourcc = header.ddspf.fourCC;
  GLenum gl_format = 0;
  uint32_t block_size = 0;
  if (fourcc == MakeFourCC('D', 'X', 'T', '1')) {
    gl_format = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT;
    block_size = 8;
  } else if (fourcc == MakeFourCC('D', 'X', 'T', '3')) {
    gl_format = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT;
    block_size = 16;
  } else if (fourcc == MakeFourCC('D', 'X', 'T', '5')) {
    gl_format = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
    block_size = 16;
  } else {
    ozz::log::Err() << "Unsupported DDS FourCC in texture: " << path
                    << std::endl;
    return 0;
  }

  const uint32_t mip_count = header.mipMapCount > 0 ? header.mipMapCount : 1;
  const size_t data_offset = static_cast<size_t>(stream.tellg());
  stream.seekg(0, std::ios::end);
  const size_t file_size = static_cast<size_t>(stream.tellg());
  const size_t data_size = file_size - data_offset;
  stream.seekg(data_offset, std::ios::beg);

  std::vector<uint8_t> data(data_size);
  if (!stream.read(reinterpret_cast<char*>(data.data()), data_size)) {
    ozz::log::Err() << "Failed to read DDS texture data: " << path << std::endl;
    return 0;
  }

  GLuint texture = 0;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                  mip_count > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  uint32_t width = header.width;
  uint32_t height = header.height;
  const uint8_t* bytes = data.data();

  for (uint32_t level = 0; level < mip_count; ++level) {
    const uint32_t ww = std::max(1u, width);
    const uint32_t hh = std::max(1u, height);
    const size_t size =
        std::max<size_t>(1, ((ww + 3) / 4) * ((hh + 3) / 4) * block_size);

    glCompressedTexImage2D(GL_TEXTURE_2D, level, gl_format, ww, hh, 0,
                           static_cast<GLsizei>(size), bytes);

    bytes += size;
    width = width > 1 ? width / 2 : 1;
    height = height > 1 ? height / 2 : 1;
  }

  glBindTexture(GL_TEXTURE_2D, 0);
  return texture;
}

GLuint LoadTextureFromFile(const fs::path& path) {
  const std::string extension = path.extension().string();
  std::string lowercase;
  lowercase.resize(extension.size());
  std::transform(extension.begin(), extension.end(), lowercase.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (lowercase == ".dds") {
    return LoadDdsTexture(path);
  }

  ozz::log::LogV() << "Unsupported texture format for path: " << path
                   << std::endl;
  return 0;
}

fs::path SanitizeTexturePath(const std::string& raw) {
  std::string sanitized = raw;
  for (char& ch : sanitized) {
    if (ch == '\\') {
      ch = fs::path::preferred_separator;
    }
  }
  return fs::path(sanitized);
}

std::vector<fs::path> BuildTextureCandidates(const fs::path& texture,
                                             const fs::path& mesh_dir) {
  std::vector<fs::path> candidates;
  auto add_candidate = [&candidates](const fs::path& candidate) {
    if (candidate.empty()) {
      return;
    }
    for (const fs::path& existing : candidates) {
      if (existing == candidate) {
        return;
      }
    }
    candidates.push_back(candidate);
  };

  const bool has_extension = texture.has_extension();

  if (texture.is_absolute()) {
    add_candidate(texture);
  } else {
    add_candidate(texture);
    add_candidate(mesh_dir / texture);
    add_candidate(fs::current_path() / texture);
  }

  if (!has_extension) {
    const fs::path with_dds = texture.string() + ".dds";
    if (texture.is_absolute()) {
      add_candidate(with_dds);
    } else {
      add_candidate(with_dds);
      add_candidate(mesh_dir / with_dds);
      add_candidate(fs::current_path() / with_dds);
    }
  }

  return candidates;
}

std::string EscapeJsonString(const char* input) {
  std::string escaped;
  if (!input) {
    return escaped;
  }
  escaped.reserve(std::strlen(input));
  for (const unsigned char ch : std::string_view(input)) {
    switch (ch) {
      case '"':
        escaped += "\\\"";
        break;
      case '\\':
        escaped += "\\\\";
        break;
      case '\b':
        escaped += "\\b";
        break;
      case '\f':
        escaped += "\\f";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        if (ch < 0x20) {
          std::ostringstream oss;
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(ch);
          escaped += oss.str();
        } else {
          escaped += static_cast<char>(ch);
        }
        break;
    }
  }
  return escaped;
}

std::string FormatFloat(float value, int precision = 6) {
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

std::string GetFileStem(const char* path) {
  if (path == nullptr) {
    return std::string();
  }
  std::string value(path);
  const size_t separator = value.find_last_of("/\\");
  if (separator != std::string::npos) {
    value.erase(0, separator + 1);
  }
  const size_t dot = value.find_last_of('.');
  if (dot != std::string::npos) {
    value.erase(dot);
  }
  return value;
}

std::array<std::array<float, 4>, 4> MatrixToRows(
    const ozz::math::Float4x4& matrix) {
  std::array<std::array<float, 4>, 4> rows{};
  for (int column = 0; column < 4; ++column) {
    float values[4];
    ozz::math::StorePtrU(matrix.cols[column], values);
    for (int row = 0; row < 4; ++row) {
      rows[row][column] = values[row];
    }
  }
  return rows;
}

std::array<float, 3> ConvertOzzToBlender(float x, float y, float z) {
  return {x, -z, y};
}

std::string BuildMeshDisplayName(const std::string& base_label,
                                 size_t mesh_index,
                                 size_t mesh_count) {
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  if (!base_label.empty()) {
    oss << base_label;
    if (mesh_count > 1) {
      oss << '_' << mesh_index;
    }
    return oss.str();
  }

  oss << "mesh_" << mesh_index;
  return oss.str();
}

}  // namespace

struct MotionMarkData {
  std::string name;
  std::vector<std::pair<float, float>> intervals;
};

struct MotionMetadataData {
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

class PlaybackSampleApplication : public ozz::sample::Application {
 public:
  ~PlaybackSampleApplication() override { ReleaseMeshTextures(); }

 protected:
  static constexpr int kNoAnimationIndex = -1;

  bool HasAnimationSelected() const {
    return current_animation_ >= 0 &&
           current_animation_ < static_cast<int>(animations_.size());
  }

  std::string GetAnimationName(int index) const {
    if (index < 0 || index >= static_cast<int>(animations_.size())) {
      return std::string();
    }
    if (index < static_cast<int>(animation_metadata_.size())) {
      const std::string& meta_name = animation_metadata_[index].name;
      if (!meta_name.empty()) {
        return meta_name;
      }
    }
    const char* runtime_name = animations_[index].name();
    if (runtime_name && runtime_name[0] != '\0') {
      return runtime_name;
    }
    return std::string();
  }

  std::string ReadString(ozz::io::IArchive& archive) {
    uint32_t length = 0;
    archive >> length;
    std::string value;
    value.resize(length);
    if (length > 0) {
      archive >> ozz::io::MakeArray(value.data(), length);
    }
    return value;
  }

  // Read serialized metadata chunk that follows each animation in converter output.
  MotionMetadataData ReadSerializedMetadata(ozz::io::IArchive& archive) {
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
    for (uint32_t mark_index = 0; mark_index < mark_count; ++mark_index) {
      MotionMarkData& mark = metadata.marks[mark_index];
      mark.name = ReadString(archive);

      uint32_t interval_count = 0;
      archive >> interval_count;
      mark.intervals.resize(interval_count);
      for (uint32_t interval_index = 0; interval_index < interval_count;
           ++interval_index) {
        float start = 0.f;
        float end = 0.f;
        archive >> start;
        archive >> end;
        mark.intervals[interval_index] = {start, end};
      }
    }

    return metadata;
  }

  // Load multiple animations from a single file
  bool LoadMultipleAnimations(const char* filename) {
    ozz::io::File file(filename, "rb");
    if (!file.opened()) {
      ozz::log::Err() << "Failed to open animation file: " << filename << std::endl;
      return false;
    }

    ozz::io::IArchive archive(&file);

    animation_metadata_.clear();

    // First try to read a single animation (standard format)
    if (archive.TestTag<ozz::animation::Animation>()) {
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

    if (anim_count == 0 || anim_count > 100) { // Sanity check
      ozz::log::Err() << "Invalid animation count: " << anim_count << std::endl;
      return false;
    }

    animations_.resize(anim_count);
    animation_metadata_.resize(anim_count);
    for (uint32_t i = 0; i < anim_count; ++i) {
      archive >> animations_[i];
      animation_metadata_[i] = ReadSerializedMetadata(archive);
      if (animation_metadata_[i].name.empty()) {
        const char* animation_name = animations_[i].name();
        if (animation_name) {
          animation_metadata_[i].name = animation_name;
        }
      }
    }

    ozz::log::LogV() << "Loaded " << anim_count << " animations from " << filename << std::endl;
    return true;
  }

  void PrintPoseTable() {
    const auto joint_names = skeleton_.joint_names();
    const int joint_count = skeleton_.num_joints();

    size_t name_width_sz = std::strlen("Bone");
    for (int joint = 0; joint < joint_count; ++joint) {
      name_width_sz = std::max(name_width_sz, std::strlen(joint_names[joint]));
    }
    const int name_width = static_cast<int>(name_width_sz);

    const std::array<const char*, 6> headers = {
        "PosX", "PosY", "PosZ", "RotX", "RotY", "RotZ"};
    std::array<int, 6> column_widths{};
    for (size_t i = 0; i < headers.size(); ++i) {
      column_widths[i] = std::max<int>(10, std::strlen(headers[i]));
    }

    auto format_value = [](float value, int precision) {
      if (!std::isfinite(value)) {
        return std::string("--");
      }
      if (std::fabs(value) < 1e-6f) {
        value = 0.0f;
      }
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(precision);
      if (value >= 0.f) {
        oss << ' ';
      }
      oss << value;
      return oss.str();
    };

    std::cout << "\n=== OZZ BIND POSE TABLE ===" << std::endl;
    std::cout << std::left << std::setw(name_width) << "Bone" << "  ";
    for (size_t i = 0; i < headers.size(); ++i) {
      std::cout << std::left << std::setw(column_widths[i]) << headers[i];
      if (i + 1 != headers.size()) {
        std::cout << "  ";
      }
    }
    std::cout << std::endl;

    std::cout << std::string(name_width, '-') << "  ";
    for (size_t i = 0; i < headers.size(); ++i) {
      std::cout << std::string(column_widths[i], '-');
      if (i + 1 != headers.size()) {
        std::cout << "  ";
      }
    }
    std::cout << std::endl;

    for (int joint = 0; joint < joint_count; ++joint) {
      const ozz::math::Float4x4& matrix = models_[joint];
      const float tx = ozz::math::GetX(matrix.cols[3]);
      const float ty = ozz::math::GetY(matrix.cols[3]);
      const float tz = ozz::math::GetZ(matrix.cols[3]);

      float rx = std::numeric_limits<float>::quiet_NaN();
      float ry = std::numeric_limits<float>::quiet_NaN();
      float rz = std::numeric_limits<float>::quiet_NaN();
      if (ozz::math::AreAllTrue1(ozz::math::IsOrthogonal(matrix))) {
        const ozz::math::SimdFloat4 quat_simd = ozz::math::ToQuaternion(matrix);
        float quat_values[4];
        ozz::math::StorePtrU(quat_simd, quat_values);
        const ozz::math::Quaternion quat(quat_values[0], quat_values[1],
                                         quat_values[2], quat_values[3]);
        const ozz::math::Float3 euler = ozz::math::ToEuler(quat);
        rx = euler.x * ozz::math::kRadianToDegree;
        ry = euler.y * ozz::math::kRadianToDegree;
        rz = euler.z * ozz::math::kRadianToDegree;
      }

      std::cout << std::left << std::setw(name_width) << joint_names[joint]
                << "  " << std::right
                << std::setw(column_widths[0]) << format_value(tx, 6) << "  "
                << std::setw(column_widths[1]) << format_value(ty, 6) << "  "
                << std::setw(column_widths[2]) << format_value(tz, 6) << "  "
                << std::setw(column_widths[3]) << format_value(rx, 3) << "  "
                << std::setw(column_widths[4]) << format_value(ry, 3) << "  "
                << std::setw(column_widths[5]) << format_value(rz, 3)
                << std::endl;
    }

    std::cout << std::endl;
  }

 protected:
 // Updates current animation time and skeleton pose.
  virtual bool OnUpdate(float _dt, float) {
    const bool space_down = glfwGetKey(GLFW_KEY_SPACE) == GLFW_PRESS;
    if (space_down && !space_was_down_ && HasAnimationSelected() &&
        animations_[current_animation_].duration() > 0.0f) {
      controller_.TogglePlay();
    }
    space_was_down_ = space_down;

    if (HasAnimationSelected() &&
        animations_[current_animation_].duration() > 0.0f) {
      // Updates current animation time.
      controller_.Update(animations_[current_animation_], _dt);

      // Samples optimized animation at t = animation_time_.
      ozz::animation::SamplingJob sampling_job;
      sampling_job.animation = &animations_[current_animation_];
      sampling_job.context = &context_;
      sampling_job.ratio = controller_.time_ratio();
      sampling_job.output = make_span(locals_);
      if (!sampling_job.Run()) {
        return false;
      }
    } else {
      // No animation selected - use bind pose
      for (int i = 0; i < skeleton_.num_soa_joints(); ++i) {
        locals_[i] = skeleton_.joint_rest_poses()[i];
      }
    }

    // Converts from local space to model space matrices.
    ozz::animation::LocalToModelJob ltm_job;
    ltm_job.skeleton = &skeleton_;
    ltm_job.input = make_span(locals_);
    ltm_job.output = make_span(models_);
    if (!ltm_job.Run()) {
      return false;
    }

    if (log_bones_each_frame_) {
      LogBoneTransformsForFrame();
    }

    if (skinning_dump_pending_ && !skinning_dump_path_.empty()) {
      if (!ExportSkinningToJson(skinning_dump_path_.c_str())) {
        return false;
      }
      skinning_dump_pending_ = false;
    }

    return true;
  }

  virtual bool OnDisplay(ozz::sample::Renderer* _renderer) {
    bool success = true;

    // Calculate ground offset to prevent clipping
    float min_z = std::numeric_limits<float>::max();
    for (int i = 0; i < skeleton_.num_joints(); ++i) {
      float z = ozz::math::GetZ(models_[i].cols[3]);
      if (z < min_z) {
        min_z = z;
      }
    }

    const ozz::math::Float4x4 transform = ozz::math::Float4x4::identity();

    if (draw_skeleton_) {
      success &=
          _renderer->DrawPosture(skeleton_, make_span(models_), transform);
    }

    if (draw_mesh_ && !meshes_.empty()) {
      const bool visibility_missing = mesh_visibility_.size() != meshes_.size();
      for (size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index) {
        if (!visibility_missing && mesh_visibility_[mesh_index] == 0) {
          continue;
        }
        const ozz::sample::Mesh& mesh = meshes_[mesh_index];
        const size_t palette_size = mesh.joint_remaps.size();
        if (palette_size > skinning_matrices_.size()) {
          continue;
        }

        for (size_t palette_index = 0; palette_index < palette_size;
             ++palette_index) {
          const uint16_t joint = mesh.joint_remaps[palette_index];
          skinning_matrices_[palette_index] =
              models_[joint] * mesh.inverse_bind_poses[palette_index];
        }

        auto skinning_span = ozz::make_span(skinning_matrices_);
        skinning_span = skinning_span.first(palette_size);

        ozz::sample::Renderer::Options draw_options = renderer_options_;
        if (draw_options.texture && mesh_index < mesh_textures_.size()) {
          draw_options.texture_override = mesh_textures_[mesh_index];
        }

        success &=
            _renderer->DrawSkinnedMesh(mesh, skinning_span, transform, draw_options);
      }
    }

    return success;
  }

  virtual bool OnInitialize() {
    // Reading skeleton.
    if (!ozz::sample::LoadSkeleton(OPTIONS_skeleton, &skeleton_)) {
      return false;
    }
    skeleton_label_ = GetFileStem(OPTIONS_skeleton);

    // Try reading animation(s), but allow failure for bind pose testing
    bool has_animation = LoadMultipleAnimations(OPTIONS_animation);

    if (has_animation && !animations_.empty()) {
      // Skeleton and animation needs to match.
      if (skeleton_.num_joints() != animations_[0].num_tracks()) {
        return false;
      }
    }

    // Debug: output skeleton information
    std::cout << "\n=== DEBUG SKELETON INFO ===" << std::endl;
    std::cout << "Skeleton joints: " << skeleton_.num_joints() << std::endl;
    if (has_animation && !animations_.empty()) {
      std::cout << "Loaded " << animations_.size() << " animations" << std::endl;
      for (size_t i = 0; i < animations_.size(); ++i) {
        const char* animation_name = animations_[i].name();
        const std::string display_name = animation_name && animation_name[0] != '\0'
                                             ? animation_name
                                             : animation_metadata_.size() > i
                                                   ? animation_metadata_[i].name
                                                   : std::string();
        std::cout << "  Animation " << i << " ('" << display_name
                  << "'): tracks=" << animations_[i].num_tracks()
                  << ", duration=" << animations_[i].duration() << " seconds" << std::endl;
      }
    } else {
      std::cout << "No animation loaded - using bind pose" << std::endl;
    }

    // Output all bone names
    const ozz::span<const char* const> joint_names = skeleton_.joint_names();
    std::cout << "Bone names:" << std::endl;
    for (int i = 0; i < skeleton_.num_joints(); ++i) {
      std::cout << "  [" << i << "] " << joint_names[i] << std::endl;
    }

    current_animation_ = animations_.empty() ? kNoAnimationIndex : 0;

    // Allocates runtime buffers.
    const int num_soa_joints = skeleton_.num_soa_joints();
    locals_.resize(num_soa_joints);
    const int num_joints = skeleton_.num_joints();
    models_.resize(num_joints);

    // Allocates a context that matches animation requirements.
    context_.Resize(num_joints);

    const bool mesh_requested = OPTIONS_mesh && OPTIONS_mesh[0] != '\0';
    if (mesh_requested) {
      if (!ozz::sample::LoadMeshes(OPTIONS_mesh, &meshes_)) {
        return false;
      }
      mesh_label_ = GetFileStem(OPTIONS_mesh);
      size_t skinning_count = 0;
      for (const ozz::sample::Mesh& mesh : meshes_) {
        skinning_count = std::max(skinning_count, mesh.joint_remaps.size());
      }
      skinning_matrices_.resize(skinning_count);
      for (const ozz::sample::Mesh& mesh : meshes_) {
        if (num_joints < mesh.highest_joint_index()) {
          ozz::log::Err() << "The provided mesh doesn't match skeleton"
                          << " (joint count mismatch)." << std::endl;
          return false;
        }
      }
      draw_mesh_ = true;
      LoadMeshTextures(OPTIONS_mesh);
      RefreshMeshDisplayState();
    } else {
      meshes_.clear();
      skinning_matrices_.clear();
      draw_mesh_ = false;
      mesh_names_.clear();
      mesh_visibility_.clear();
      ReleaseMeshTextures();
    }

    if (!ComputeBindPoseModelMatrices()) {
      return false;
    }

    log_bone_limit_ = num_joints > 0 ? num_joints : 0;
    bone_display_limit_ = std::min(num_joints, 16);

    if (OPTIONS_dump_animation_json &&
        OPTIONS_dump_animation_json[0] != '\0') {
      if (!ExportAnimationToJson(OPTIONS_dump_animation_json)) {
        return false;
      }
    }

    skinning_dump_path_.clear();
    skinning_dump_pending_ = false;
    if (OPTIONS_dump_skinning_json && OPTIONS_dump_skinning_json[0] != '\0') {
      skinning_dump_path_ = OPTIONS_dump_skinning_json;
      skinning_dump_pending_ = true;
    }

    // Optionally set a forced animation time ratio from environment.
    const char* ratio_env = std::getenv("OZZ_SAMPLE_RATIO");
    const char* time_env = std::getenv("OZZ_SAMPLE_TIME");
    if (ratio_env) {
      forced_time_ratio_ = std::clamp(static_cast<float>(std::atof(ratio_env)), 0.f, 1.f);
      use_forced_time_ratio_ = true;
    } else if (time_env && !animations_.empty()) {
      const float duration = animations_[current_animation_].duration();
      if (duration > 0.f) {
        const float sample_time = static_cast<float>(std::atof(time_env));
        forced_time_ratio_ = std::clamp(sample_time / duration, 0.f, 1.f);
        use_forced_time_ratio_ = true;
      }
    }

    if (use_forced_time_ratio_) {
      controller_.set_playback_speed(0.f);
      controller_.set_time_ratio(forced_time_ratio_);
    }

    return true;
  }

  virtual bool OnGui(ozz::sample::ImGui* _im_gui) {
    // Animation selection
    {
      static bool select_open = true;
      ozz::sample::ImGui::OpenClose select_oc(_im_gui, "Animation selection", &select_open);
      if (select_open) {
        if (animations_.empty()) {
          _im_gui->DoLabel("No animations loaded. Displaying bind pose.");
        } else {
          const int animation_count = static_cast<int>(animations_.size());
          int selected_animation = std::clamp(current_animation_, kNoAnimationIndex,
                                              animation_count - 1);

          _im_gui->DoLabel(
              "Choose an animation to preview or select the bind pose.");

          if (_im_gui->DoButton("Previous")) {
            if (selected_animation == kNoAnimationIndex) {
              selected_animation = animation_count - 1;
            } else if (selected_animation > 0) {
              --selected_animation;
            } else {
              selected_animation = kNoAnimationIndex;
            }
          }

          if (_im_gui->DoButton("Next")) {
            if (selected_animation == kNoAnimationIndex) {
              selected_animation = 0;
            } else if (selected_animation < animation_count - 1) {
              ++selected_animation;
            } else {
              selected_animation = kNoAnimationIndex;
            }
          }

          char label[64];
          _im_gui->DoRadioButton(kNoAnimationIndex, "Bind pose (no animation)",
                                 &selected_animation);
          for (int i = 0; i < animation_count; ++i) {
            const std::string motion_name = GetAnimationName(i);
            if (!motion_name.empty()) {
              std::snprintf(label, sizeof(label), "%d: %s", i + 1,
                            motion_name.c_str());
            } else {
              std::snprintf(label, sizeof(label), "Animation %d of %d", i + 1,
                            animation_count);
            }

            _im_gui->DoRadioButton(i, motion_name.c_str(), &selected_animation);
          }

          if (selected_animation != current_animation_) {
            current_animation_ = selected_animation;
            controller_.Reset();
            context_.Invalidate();
            if (!skinning_dump_path_.empty()) {
              skinning_dump_pending_ = true;
            }
          }

          if (HasAnimationSelected()) {
            const MotionMetadataData* metadata =
                animation_metadata_.size() > static_cast<size_t>(current_animation_)
                    ? &animation_metadata_[current_animation_]
                    : nullptr;
            if (metadata) {
              std::snprintf(label, sizeof(label), "Flags: 0x%08X", metadata->flags);
              _im_gui->DoLabel(label);
              std::snprintf(label, sizeof(label), "Motion ID: %u", metadata->motion_id);
              _im_gui->DoLabel(label);
              std::snprintf(label, sizeof(label), "Speed: %.3f  Power: %.3f",
                            metadata->speed, metadata->power);
              _im_gui->DoLabel(label);
              std::snprintf(label, sizeof(label), "Accrue: %.3f  Falloff: %.3f",
                            metadata->accrue, metadata->falloff);
              _im_gui->DoLabel(label);
            }
            std::snprintf(label, sizeof(label), "Duration: %.2f seconds",
                          animations_[current_animation_].duration());
            _im_gui->DoLabel(label);
            if (metadata && !metadata->marks.empty()) {
              _im_gui->DoLabel("Marks:");
              metadata_labels_.clear();
              for (const MotionMarkData& mark : metadata->marks) {
                std::ostringstream mark_stream;
                mark_stream.imbue(std::locale::classic());
                mark_stream << "  " << mark.name << " [";
                for (size_t idx = 0; idx < mark.intervals.size(); ++idx) {
                  mark_stream << std::fixed << std::setprecision(3)
                              << mark.intervals[idx].first << " - "
                              << mark.intervals[idx].second;
                  if (idx + 1 < mark.intervals.size()) {
                    mark_stream << ", ";
                  }
                }
                mark_stream << "]";
                metadata_labels_.push_back(mark_stream.str());
              }
              for (const std::string& mark_label : metadata_labels_) {
                _im_gui->DoLabel(mark_label.c_str());
              }
              metadata_labels_.clear();
            }


            // Exposes animation runtime playback controls.
            {
                static bool open = true;
                ozz::sample::ImGui::OpenClose oc(_im_gui, "Animation control", &open);
                if (open) {
                    if (HasAnimationSelected()) {
                        controller_.OnGui(animations_[current_animation_], _im_gui);
                    }
                    else {
                        _im_gui->DoLabel("Select an animation to enable playback controls.");
                    }
                }
            }
          }
        }
      }
    }

    {
      static bool render_open = true;
      ozz::sample::ImGui::OpenClose render_oc(_im_gui, "Rendering", &render_open);
      if (render_open) {
        _im_gui->DoCheckBox("Draw skeleton", &draw_skeleton_);
        if (!meshes_.empty()) {
          _im_gui->DoCheckBox("Draw mesh", &draw_mesh_);
          _im_gui->DoCheckBox("Show triangles", &renderer_options_.triangles);
          if (_im_gui->DoCheckBox("Show texture", &renderer_options_.texture)) {
            if (renderer_options_.texture) {
              ozz::log::Out() << "Texture rendering enabled." << std::endl;
              for (size_t mesh_index = 0; mesh_index < meshes_.size();
                   ++mesh_index) {
                const auto& metadata = meshes_[mesh_index].xray_metadata;
                const std::string resolved =
                    mesh_index < mesh_texture_paths_.size()
                        ? mesh_texture_paths_[mesh_index]
                        : std::string();
                const bool loaded =
                    mesh_index < mesh_textures_.size() &&
                    mesh_textures_[mesh_index] != 0;
                ozz::log::Out()
                    << "mesh[" << mesh_index << "] metadata='"
                    << metadata.texture_path << "' resolved='"
                    << (resolved.empty() ? "" : resolved)
                    << "' loaded=" << (loaded ? "yes" : "no")
                    << std::endl;
              }
            } else {
              ozz::log::Out() << "Texture rendering disabled." << std::endl;
            }
          }
          _im_gui->DoCheckBox("Show vertices", &renderer_options_.vertices);
          _im_gui->DoCheckBox("Show normals", &renderer_options_.normals);
          _im_gui->DoCheckBox("Show tangents", &renderer_options_.tangents);
          _im_gui->DoCheckBox("Show binormals", &renderer_options_.binormals);
          _im_gui->DoCheckBox("Show colors", &renderer_options_.colors);
          _im_gui->DoCheckBox("Wireframe", &renderer_options_.wireframe);
          _im_gui->DoCheckBox("Skip skinning", &renderer_options_.skip_skinning);

          if (mesh_names_.size() != meshes_.size() ||
              mesh_visibility_.size() != meshes_.size()) {
            RefreshMeshDisplayState();
          }

          if (!meshes_.empty()) {
            _im_gui->DoLabel("Mesh visibility:");
          }

          for (size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index) {
            const std::string& mesh_name = mesh_names_[mesh_index];
            bool visible = mesh_visibility_[mesh_index] != 0;
            if (_im_gui->DoCheckBox(mesh_name.c_str(), &visible)) {
              mesh_visibility_[mesh_index] = visible ? 1 : 0;
            }
          }
        }
      }
    }


    // Logging options
    {
      static bool logging_open = false;
      ozz::sample::ImGui::OpenClose oc(_im_gui, "Logging options", &logging_open);
      if (logging_open) {
        _im_gui->DoCheckBox("Log bones every frame", &log_bones_each_frame_);
        if (log_bones_each_frame_) {
          const int max_joints = skeleton_.num_joints();
          if (max_joints > 0) {
            if (log_bone_limit_ < 1) {
              log_bone_limit_ = 1;
            }
            _im_gui->DoSlider("Bones printed", 1, max_joints, &log_bone_limit_,
                              1.f, true);
          } else {
            log_bone_limit_ = 0;
          }
        }
      }
    }

    // On-screen bone transform display
    {
      static bool display_open = true;
      ozz::sample::ImGui::OpenClose oc(_im_gui, "Bone transforms", &display_open);
      if (display_open) {
        _im_gui->DoCheckBox("Show bone transforms", &show_bone_debug_);
        const int max_joints = skeleton_.num_joints();
        if (max_joints > 0) {
          if (bone_display_limit_ < 1) {
            bone_display_limit_ = 1;
          }
          if (bone_display_limit_ > max_joints) {
            bone_display_limit_ = max_joints;
          }
          _im_gui->DoSlider("Bones shown", 1, max_joints, &bone_display_limit_, 1.f,
                            show_bone_debug_);
        } else {
          bone_display_limit_ = 0;
        }

        if (show_bone_debug_ && max_joints > 0) {
          const auto joint_names = skeleton_.joint_names();
          const int display_count = std::min(bone_display_limit_, max_joints);

          if (HasAnimationSelected()) {
            const float duration = animations_[current_animation_].duration();
            const float ratio = controller_.time_ratio();
            const float time = duration * ratio;
            char header[128];
            std::snprintf(header, sizeof(header),
                          "Time %.3fs / %.3fs (ratio %.3f)", time, duration, ratio);
            _im_gui->DoLabel(header);
          } else {
            _im_gui->DoLabel("Bind pose (no animation)");
          }

          int name_width = 4;
          for (int joint = 0; joint < display_count; ++joint) {
            name_width =
                std::max<int>(name_width, static_cast<int>(std::strlen(joint_names[joint])));
          }

          constexpr std::array<const char*, 7> column_headers = {
              "Pos X", "Pos Y", "Pos Z", "Rot W", "Rot X", "Rot Y", "Rot Z"};
          constexpr int numeric_width = 11;

          std::ostringstream header_stream;
          header_stream.imbue(std::locale::classic());
          header_stream << std::left << std::setw(name_width) << "Bone" << "  ";
          for (const char* col : column_headers) {
            header_stream << std::right << std::setw(numeric_width) << col << ' ';
          }
          _im_gui->DoLabel(header_stream.str().c_str());

          const int separator_width =
              name_width + 2 + static_cast<int>(column_headers.size()) * (numeric_width + 1);
          std::string separator(separator_width, '-');
          _im_gui->DoLabel(separator.c_str());

          for (int joint = 0; joint < display_count; ++joint) {
            const ozz::math::Float4x4& matrix = models_[joint];
            const float tx = ozz::math::GetX(matrix.cols[3]);
            const float ty = ozz::math::GetY(matrix.cols[3]);
            const float tz = ozz::math::GetZ(matrix.cols[3]);

            const ozz::math::SimdFloat4 quat_simd = ozz::math::ToQuaternion(matrix);
            float quat[4];
            ozz::math::StorePtrU(quat_simd, quat);

            std::array<std::string, 7> value_strings = {
                FormatFloat(tx), FormatFloat(ty), FormatFloat(tz),
                FormatFloat(quat[3]), FormatFloat(quat[0]),
                FormatFloat(quat[1]), FormatFloat(quat[2])};

            std::ostringstream row_stream;
            row_stream.imbue(std::locale::classic());
            row_stream << std::left << std::setw(name_width) << joint_names[joint] << "  ";
            for (const std::string& value : value_strings) {
              row_stream << std::right << std::setw(numeric_width) << value << ' ';
            }
            _im_gui->DoLabel(row_stream.str().c_str(), ozz::sample::ImGui::kLeft, false);
          }

          if (display_count < max_joints) {
            char footer[96];
            std::snprintf(footer, sizeof(footer), "Showing %d of %d bones",
                          display_count, max_joints);
            _im_gui->DoLabel(footer);
          }
        }
      }
    }
    return true;
  }

  virtual void GetSceneBounds(ozz::math::Box* _bound) const {
    ozz::sample::ComputePostureBounds(make_span(models_),
                                      ozz::math::Float4x4::identity(), _bound);
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

  // Temporary strings used to display mark information.
  std::vector<std::string> metadata_labels_;

  // Current animation index (-1 selects bind pose)
  int current_animation_ = kNoAnimationIndex;

  // Sampling context.
  ozz::animation::SamplingJob::Context context_;

  // Buffer of local transforms as sampled from animation_.
  ozz::vector<ozz::math::SoaTransform> locals_;

  // Buffer of model space matrices.
  ozz::vector<ozz::math::Float4x4> models_;

  // Optional meshes used for skinning validation / rendering.
  ozz::vector<ozz::sample::Mesh> meshes_;
  ozz::vector<ozz::math::Float4x4> skinning_matrices_;

  // Cached labels for reporting/exporting.
  std::string skeleton_label_;
  std::string mesh_label_;
  std::vector<std::string> mesh_names_;
  std::vector<uint8_t> mesh_visibility_;
  std::vector<GLuint> mesh_textures_;
  std::vector<std::string> mesh_texture_paths_;

  // Optional dump controls.
  std::string skinning_dump_path_;
  bool skinning_dump_pending_ = false;

  // Rendering toggles and options.
  bool draw_skeleton_ = true;
  bool draw_mesh_ = false;
  ozz::sample::Renderer::Options renderer_options_;

  // Optional externally forced time ratio for sampling animations.
  float forced_time_ratio_ = 0.f;
  bool use_forced_time_ratio_ = false;

  bool log_bones_each_frame_ = false;
  int log_bone_limit_ = 0;
  bool show_bone_debug_ = true;
  int bone_display_limit_ = 16;
  bool space_was_down_ = false;

  void ReleaseMeshTextures() {
    if (!mesh_textures_.empty()) {
      for (GLuint texture : mesh_textures_) {
        if (texture != 0) {
          glDeleteTextures(1, &texture);
        }
      }
    }
    mesh_textures_.clear();
    mesh_texture_paths_.clear();
  }

  void LoadMeshTextures(const char* mesh_path) {
    ReleaseMeshTextures();
    mesh_textures_.resize(meshes_.size(), 0);
    mesh_texture_paths_.assign(meshes_.size(), std::string());

    if (!mesh_path) {
      return;
    }

    const fs::path mesh_file = fs::absolute(fs::path(mesh_path));
    const fs::path mesh_dir = mesh_file.parent_path();
    fs::path override_root;
    if (OPTIONS_texture_root && OPTIONS_texture_root[0] != '\0') {
      override_root = fs::absolute(fs::path(std::string(OPTIONS_texture_root)));
    }

    for (size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index) {
      const ozz::sample::XRayMeshMetadata& metadata =
          meshes_[mesh_index].xray_metadata;
      if (metadata.texture_path.empty()) {
        continue;
      }

      const fs::path relative = SanitizeTexturePath(metadata.texture_path);
      std::vector<fs::path> candidates =
          BuildTextureCandidates(relative, mesh_dir);
      if (!override_root.empty()) {
        std::vector<fs::path> override_candidates =
            BuildTextureCandidates(relative, override_root);
        candidates.insert(candidates.end(), override_candidates.begin(),
                          override_candidates.end());
      }

      GLuint texture = 0;
      std::string last_candidate;
      for (const fs::path& candidate : candidates) {
        const std::string abs_candidate = fs::absolute(candidate).string();
        last_candidate = abs_candidate;

        if (!fs::exists(candidate)) {
          continue;
        }

        texture = LoadTextureFromFile(candidate);
        if (texture != 0) {
          ozz::log::Out() << "Loaded texture for mesh " << mesh_index
                          << " from " << abs_candidate << std::endl;
          mesh_texture_paths_[mesh_index] = abs_candidate;
          break;
        }
      }

      if (texture == 0) {
        ozz::log::Out() << "Unable to load texture '" << metadata.texture_path
                        << "' for mesh " << mesh_index;
        if (!last_candidate.empty()) {
          ozz::log::Out() << " (last candidate: " << last_candidate << ")";
        }
        ozz::log::Out() << std::endl;
        mesh_texture_paths_[mesh_index] = last_candidate;
      }

      mesh_textures_[mesh_index] = texture;
    }
  }

  void RefreshMeshDisplayState() {
    mesh_names_.clear();
    mesh_visibility_.assign(meshes_.size(), 1);

    mesh_names_.reserve(meshes_.size());
    const size_t mesh_count = meshes_.size();
    for (size_t mesh_index = 0; mesh_index < mesh_count; ++mesh_index) {
      const ozz::sample::Mesh& mesh = meshes_[mesh_index];
      const ozz::sample::XRayMeshMetadata& metadata = mesh.xray_metadata;

      std::string display_name;
      if (!metadata.texture_path.empty()) {
        display_name = metadata.texture_path;
      } else if (!metadata.shader_name.empty()) {
        display_name = metadata.shader_name;
      }

      if (!display_name.empty()) {
        if (mesh_count > 1) {
          display_name += "_" + std::to_string(mesh_index);
        }
      } else {
        display_name = BuildMeshDisplayName(mesh_label_, mesh_index, mesh_count);
      }

      mesh_names_.push_back(std::move(display_name));
    }
  }

  bool ComputeBindPoseModelMatrices() {
    if (skeleton_.num_joints() == 0) {
      models_.clear();
      return true;
    }

    const auto rest_poses = skeleton_.joint_rest_poses();
    if (rest_poses.size() != locals_.size()) {
      locals_.resize(rest_poses.size());
    }
    for (size_t i = 0; i < rest_poses.size(); ++i) {
      locals_[i] = rest_poses[i];
    }

    if (models_.size() != static_cast<size_t>(skeleton_.num_joints())) {
      models_.resize(skeleton_.num_joints());
    }

    ozz::animation::LocalToModelJob ltm_job;
    ltm_job.skeleton = &skeleton_;
    ltm_job.input = make_span(locals_);
    ltm_job.output = make_span(models_);
    if (!ltm_job.Run()) {
      ozz::log::Err() << "Failed to build bind-pose model matrices." << std::endl;
      return false;
    }
    return true;
  }

  bool ExportSkinningToJson(const char* path) {
    if (path == nullptr || path[0] == '\0') {
      return true;
    }
    if (meshes_.empty()) {
      ozz::log::Err() << "No mesh loaded; cannot export skinning JSON." << std::endl;
      return false;
    }

    const int joint_count = skeleton_.num_joints();
    if (joint_count == 0) {
      ozz::log::Err() << "Skeleton has no joints; cannot export skinning JSON." << std::endl;
      return false;
    }

    if (models_.size() != static_cast<size_t>(joint_count)) {
      if (!ComputeBindPoseModelMatrices()) {
        return false;
      }
    }

    std::ofstream file(path, std::ios::binary);
    if (!file) {
      ozz::log::Err() << "Failed to open skinning JSON export path: " << path
                      << std::endl;
      return false;
    }
    file.imbue(std::locale::classic());

    const auto joint_names = skeleton_.joint_names();

    file << "{\n";
    file << "  \"armature\": {\n";
    file << "    \"name\": \"" << EscapeJsonString(skeleton_label_.c_str())
         << "\",\n";
    file << "    \"bones\": [\n";

    for (int joint = 0; joint < joint_count; ++joint) {
      const auto rows = MatrixToRows(models_[joint]);
      ozz::math::SimdInt4 invertible;
      const ozz::math::Float4x4 inverse =
          ozz::math::Invert(models_[joint], &invertible);
      const auto inverse_rows = MatrixToRows(inverse);

      file << "      {\n";
      file << "        \"index\": " << joint << ",\n";
      file << "        \"name\": \""
           << EscapeJsonString(joint_names[joint]) << "\",\n";
      file << "        \"matrix_global\": [\n";
      for (int row = 0; row < 4; ++row) {
        file << "          [" << FormatFloat(rows[row][0]) << ", "
             << FormatFloat(rows[row][1]) << ", "
             << FormatFloat(rows[row][2]) << ", "
             << FormatFloat(rows[row][3]) << "]";
        file << (row < 3 ? ",\n" : "\n");
      }
      file << "        ],\n";
      file << "        \"matrix_global_inverse\": [\n";
      for (int row = 0; row < 4; ++row) {
        file << "          [" << FormatFloat(inverse_rows[row][0]) << ", "
             << FormatFloat(inverse_rows[row][1]) << ", "
             << FormatFloat(inverse_rows[row][2]) << ", "
             << FormatFloat(inverse_rows[row][3]) << "]";
        file << (row < 3 ? ",\n" : "\n");
      }
      file << "        ]\n";
      file << "      }";
      file << (joint + 1 < joint_count ? ",\n" : "\n");
    }

    file << "    ]\n";
    file << "  },\n";
    file << "  \"meshes\": [\n";

    for (size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index) {
      const ozz::sample::Mesh& mesh = meshes_[mesh_index];

      const size_t palette_size = mesh.joint_remaps.size();
      if (palette_size > 0) {
        skinning_matrices_.resize(palette_size);
        if (mesh.inverse_bind_poses.size() != palette_size) {
          ozz::log::Err() << "Mesh palette size mismatch: remaps="
                          << mesh.joint_remaps.size()
                          << " inverse_bind_poses="
                          << mesh.inverse_bind_poses.size() << std::endl;
        }
        for (size_t palette_index = 0; palette_index < palette_size;
             ++palette_index) {
          const uint16_t joint_index = mesh.joint_remaps[palette_index];
          if (joint_index >= models_.size()) {
            ozz::log::Err() << "Palette index " << palette_index
                            << " references joint " << joint_index
                            << " beyond loaded skeleton." << std::endl;
            continue;
          }
          if (mesh.inverse_bind_poses.size() > palette_index) {
            skinning_matrices_[palette_index] =
                models_[joint_index] * mesh.inverse_bind_poses[palette_index];
          } else {
            skinning_matrices_[palette_index] = models_[joint_index];
          }
        }
      } else {
        skinning_matrices_.clear();
      }

      file << "    {\n";
      const std::string mesh_name =
          mesh_names_.size() > mesh_index
              ? mesh_names_[mesh_index]
              : BuildMeshDisplayName(mesh_label_, mesh_index, meshes_.size());
      file << "      \"name\": \""
           << EscapeJsonString(mesh_name.c_str()) << "\",\n";
      file << "      \"vertex_count\": " << mesh.vertex_count() << ",\n";
      const ozz::sample::XRayMeshMetadata& xray = mesh.xray_metadata;
      file << "      \"xray\": {\n";
      file << "        \"ogf_type\": " << static_cast<int>(xray.ogf_type) << ",\n";
      file << "        \"original_vertex_count\": " << xray.original_vertex_count << ",\n";
      file << "        \"original_face_count\": " << xray.original_face_count << ",\n";
      file << "        \"texture_path\": ";
      if (!xray.texture_path.empty()) {
        file << "\"" << EscapeJsonString(xray.texture_path.c_str()) << "\"";
      } else {
        file << "null";
      }
      file << ",\n";
      file << "        \"shader_name\": ";
      if (!xray.shader_name.empty()) {
        file << "\"" << EscapeJsonString(xray.shader_name.c_str()) << "\"";
      } else {
        file << "null";
      }
      file << ",\n";
      file << "        \"texture_link\": ";
      if (xray.texture_link_present) {
        file << xray.texture_link;
      } else {
        file << "null";
      }
      file << ",\n";
      file << "        \"shader_link\": ";
      if (xray.shader_link_present) {
        file << xray.shader_link;
      } else {
        file << "null";
      }
      file << ",\n";
      file << "        \"lod_visuals\": [";
      for (size_t lod_index = 0; lod_index < xray.lod_visuals.size(); ++lod_index) {
        file << (lod_index == 0 ? "" : ", ")
             << "\"" << EscapeJsonString(xray.lod_visuals[lod_index].c_str())
             << "\"";
      }
      file << "],\n";
      file << "        \"lod_data_bytes\": " << xray.lod_data.size() << ",\n";
      file << "        \"progressive_collapse_count\": "
           << xray.progressive_collapse_count << ",\n";
      file << "        \"progressive_data_bytes\": "
           << xray.progressive_data.size() << ",\n";
      file << "        \"child_links\": [";
      for (size_t link_index = 0; link_index < xray.child_visual_links.size();
           ++link_index) {
        file << (link_index == 0 ? "" : ", ") << xray.child_visual_links[link_index];
      }
      file << "]\n";
      file << "      },\n";
      file << "      \"joint_palette\": [\n";
      for (size_t palette_index = 0; palette_index < palette_size;
           ++palette_index) {
        file << "        {\n";
        file << "          \"palette_index\": " << palette_index << ",\n";
        file << "          \"joint_index\": "
             << mesh.joint_remaps[palette_index] << ",\n";
        if (mesh.inverse_bind_poses.size() > palette_index) {
          const auto inverse_rows =
              MatrixToRows(mesh.inverse_bind_poses[palette_index]);
          file << "          \"inverse_bind_pose\": [\n";
          for (int row = 0; row < 4; ++row) {
            file << "            [" << FormatFloat(inverse_rows[row][0]) << ", "
                 << FormatFloat(inverse_rows[row][1]) << ", "
                 << FormatFloat(inverse_rows[row][2]) << ", "
                 << FormatFloat(inverse_rows[row][3]) << "]";
            file << (row < 3 ? ",\n" : "\n");
          }
          file << "          ],\n";
        } else {
          file << "          \"inverse_bind_pose\": [],\n";
        }
        if (skinning_matrices_.size() > palette_index) {
          const auto skin_rows = MatrixToRows(skinning_matrices_[palette_index]);
          file << "          \"skinning_matrix\": [\n";
          for (int row = 0; row < 4; ++row) {
            file << "            [" << FormatFloat(skin_rows[row][0]) << ", "
                 << FormatFloat(skin_rows[row][1]) << ", "
                 << FormatFloat(skin_rows[row][2]) << ", "
                 << FormatFloat(skin_rows[row][3]) << "]";
            file << (row < 3 ? ",\n" : "\n");
          }
          file << "          ]\n";
        } else {
          file << "          \"skinning_matrix\": []\n";
        }
        file << "        }";
        file << (palette_index + 1 < palette_size ? ",\n" : "\n");
      }
      file << "      ],\n";
      file << "      \"vertices\": [\n";

      int global_vertex = 0;
      for (const ozz::sample::Mesh::Part& part : mesh.parts) {
        const int influences = part.influences_count();
        const int vertex_count = part.vertex_count();
        const bool has_normals =
            part.normals.size() ==
            static_cast<size_t>(vertex_count * ozz::sample::Mesh::Part::kNormalsCpnts);
        for (int v = 0; v < vertex_count; ++v, ++global_vertex) {
          file << "        {\n";
          file << "          \"index\": " << global_vertex << ",\n";
          const int pos_offset = v * ozz::sample::Mesh::Part::kPositionsCpnts;
          const float px = part.positions[pos_offset + 0];
          const float py = part.positions[pos_offset + 1];
          const float pz = part.positions[pos_offset + 2];
          const auto blender_pos = ConvertOzzToBlender(px, py, pz);
          file << "          \"position\": [" << FormatFloat(blender_pos[0]) << ", "
               << FormatFloat(blender_pos[1]) << ", " << FormatFloat(blender_pos[2]) << "],\n";

          float nx = 0.f;
          float ny = 0.f;
          float nz = 0.f;
          if (has_normals) {
            const int normal_offset =
                v * ozz::sample::Mesh::Part::kNormalsCpnts;
            nx = part.normals[normal_offset + 0];
            ny = part.normals[normal_offset + 1];
            nz = part.normals[normal_offset + 2];
            const auto blender_normal = ConvertOzzToBlender(nx, ny, nz);
            file << "          \"normal\": [" << FormatFloat(blender_normal[0]) << ", "
                 << FormatFloat(blender_normal[1]) << ", " << FormatFloat(blender_normal[2]) << "],\n";
          }

          std::vector<std::pair<uint16_t, float>> weights;
          weights.reserve(influences > 0 ? influences : 1);
          const bool skinning_available = !skinning_matrices_.empty();
          const ozz::math::SimdFloat4 rest_position =
              ozz::math::simd_float4::Load(px, py, pz, 1.f);
          const ozz::math::SimdFloat4 rest_normal =
              has_normals ? ozz::math::simd_float4::Load(nx, ny, nz, 0.f)
                          : ozz::math::simd_float4::Load(0.f, 0.f, 0.f, 0.f);
          float skinned_pos[3] = {0.f, 0.f, 0.f};
          float skinned_nrm[3] = {0.f, 0.f, 0.f};
          float weight_sum = 0.f;

          if (influences > 0) {
            const int joint_base = v * influences;
            const int weight_base = v * std::max(0, influences - 1);
            float accum = 0.f;
            for (int influence = 0; influence < influences; ++influence) {
              const uint16_t palette_index =
                  part.joint_indices[joint_base + influence];
              if (palette_index >= mesh.joint_remaps.size()) {
                ozz::log::Err() << "Vertex " << global_vertex
                                << " references out-of-range joint index"
                                << std::endl;
                continue;
              }
              float weight = 0.f;
              if (influence < influences - 1) {
                weight = part.joint_weights[weight_base + influence];
                accum += weight;
              } else {
                weight = std::max(0.f, 1.f - accum);
              }
              weight = std::clamp(weight, 0.f, 1.f);
              weight_sum += weight;

              const uint16_t joint_index = mesh.joint_remaps[palette_index];
              weights.emplace_back(joint_index, weight);

              if (skinning_available && weight > 0.f &&
                  palette_index < skinning_matrices_.size()) {
                const ozz::math::Float4x4& skin_matrix =
                    skinning_matrices_[palette_index];
                float transformed_position[4];
                ozz::math::StorePtrU(
                    ozz::math::TransformPoint(skin_matrix, rest_position),
                    transformed_position);
                skinned_pos[0] += transformed_position[0] * weight;
                skinned_pos[1] += transformed_position[1] * weight;
                skinned_pos[2] += transformed_position[2] * weight;

                if (has_normals) {
                  float transformed_normal[4];
                  ozz::math::StorePtrU(
                      ozz::math::TransformVector(skin_matrix, rest_normal),
                      transformed_normal);
                  skinned_nrm[0] += transformed_normal[0] * weight;
                  skinned_nrm[1] += transformed_normal[1] * weight;
                  skinned_nrm[2] += transformed_normal[2] * weight;
                }
              }
            }
          }

          std::sort(weights.begin(), weights.end(),
                    [](const auto& lhs, const auto& rhs) {
                      return lhs.first < rhs.first;
                    });

          file << "          \"weights\": [";
          for (size_t idx = 0; idx < weights.size(); ++idx) {
            file << "[" << weights[idx].first << ", "
                 << FormatFloat(weights[idx].second) << "]";
            if (idx + 1 < weights.size()) {
              file << ", ";
            }
          }
          file << "],\n";

          const bool wrote_skinned = skinning_available && !weights.empty();
          if (wrote_skinned) {
            const auto blender_skinned = ConvertOzzToBlender(skinned_pos[0], skinned_pos[1], skinned_pos[2]);
            file << "          \"skinned_position\": ["
                 << FormatFloat(blender_skinned[0]) << ", "
                 << FormatFloat(blender_skinned[1]) << ", "
                 << FormatFloat(blender_skinned[2]) << "]";
            if (has_normals) {
              const float length =
                  std::sqrt(skinned_nrm[0] * skinned_nrm[0] +
                            skinned_nrm[1] * skinned_nrm[1] +
                            skinned_nrm[2] * skinned_nrm[2]);
              if (length > 0.f) {
                skinned_nrm[0] /= length;
                skinned_nrm[1] /= length;
                skinned_nrm[2] /= length;
              } else {
                skinned_nrm[0] = nx;
                skinned_nrm[1] = ny;
                skinned_nrm[2] = nz;
              }
              const auto blender_skinned_normal = ConvertOzzToBlender(skinned_nrm[0], skinned_nrm[1], skinned_nrm[2]);
              file << ",\n";
              file << "          \"skinned_normal\": ["
                   << FormatFloat(blender_skinned_normal[0]) << ", "
                   << FormatFloat(blender_skinned_normal[1]) << ", "
                   << FormatFloat(blender_skinned_normal[2]) << "]";
            }
            file << ",\n";
          } else {
            file << "          \"skinned_position\": [" << FormatFloat(blender_pos[0])
                 << ", " << FormatFloat(blender_pos[1]) << ", "
                 << FormatFloat(blender_pos[2]) << "],\n";
            if (has_normals) {
              const auto blender_normal_fallback = ConvertOzzToBlender(nx, ny, nz);
              file << "          \"skinned_normal\": [" << FormatFloat(blender_normal_fallback[0])
                   << ", " << FormatFloat(blender_normal_fallback[1]) << ", "
                   << FormatFloat(blender_normal_fallback[2]) << "],\n";
            }
          }

          file << "          \"weight_sum\": " << FormatFloat(weight_sum)
               << "\n";
          file << "        }";
          if (global_vertex + 1 < mesh.vertex_count()) {
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

  void LogBoneTransformsForFrame() const {
    const int joint_count = skeleton_.num_joints();
    if (joint_count == 0) {
      return;
    }

    const auto joint_names = skeleton_.joint_names();
    const int max_joints = log_bone_limit_ > 0
                               ? std::min(log_bone_limit_, joint_count)
                               : joint_count;

    std::ostringstream oss;
    oss.imbue(std::locale::classic());

    for (int joint = 0; joint < max_joints; ++joint) {
      const ozz::math::Float4x4& matrix = models_[joint];
      const float tx = ozz::math::GetX(matrix.cols[3]);
      const float ty = ozz::math::GetY(matrix.cols[3]);
      const float tz = ozz::math::GetZ(matrix.cols[3]);

      const ozz::math::SimdFloat4 quat_simd = ozz::math::ToQuaternion(matrix);
      float quat[4];
      ozz::math::StorePtrU(quat_simd, quat);

      oss << "  Bone[" << joint << "] '" << joint_names[joint]
          << "': pos(" << FormatFloat(tx, 6) << ", " << FormatFloat(ty, 6)
          << ", " << FormatFloat(tz, 6) << ") rot(" << FormatFloat(quat[3], 6)
          << ", " << FormatFloat(quat[0], 6) << ", "
          << FormatFloat(quat[1], 6) << ", " << FormatFloat(quat[2], 6)
          << ")" << std::endl;
    }

    oss << std::endl;
    std::cout << oss.str();
  }

  bool ExportAnimationToJson(const char* path) {
    if (path == nullptr || path[0] == '\0') {
      return true;
    }

    if (animations_.empty()) {
      ozz::log::Err() << "No animation available for JSON export." << std::endl;
      return false;
    }

    if (!HasAnimationSelected()) {
      ozz::log::Err() << "No animation selected for JSON export." << std::endl;
      return false;
    }

    const ozz::animation::Animation& animation = animations_[current_animation_];
    const float duration = animation.duration();
    const ozz::span<const float> timepoints = animation.timepoints();

    ozz::vector<float> samples;
    samples.reserve(timepoints.size() + 2);
    constexpr float kEpsilon = 1e-5f;
    if (timepoints.empty() || timepoints.front() > kEpsilon) {
      samples.push_back(0.f);
    }
    samples.insert(samples.end(), timepoints.begin(), timepoints.end());
    if (!samples.empty()) {
      std::sort(samples.begin(), samples.end());
      samples.erase(std::unique(samples.begin(), samples.end(),
                                [](float a, float b) {
                                  return std::fabs(a - b) <= kEpsilon;
                                }),
                    samples.end());
    }
    if (duration > kEpsilon) {
      if (samples.empty() || std::fabs(samples.back() - duration) > kEpsilon) {
        samples.push_back(duration);
      }
    } else if (samples.empty()) {
      samples.push_back(0.f);
    }

    std::ofstream file(path, std::ios::binary);
    if (!file) {
      ozz::log::Err() << "Failed to open JSON export path: " << path
                      << std::endl;
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
    const float frame_rate = duration > kEpsilon && frame_count > 1
                                 ? static_cast<float>(frame_count - 1) /
                                       std::max(duration, kEpsilon)
                                 : 0.f;

    file << "{\n";
    file << "  \"skeleton\": \"" << EscapeJsonString(OPTIONS_skeleton)
         << "\",\n";
    file << "  \"animation\": \"" << EscapeJsonString(OPTIONS_animation)
         << "\",\n";
    file << "  \"duration\": " << FormatFloat(duration) << ",\n";
    file << "  \"frame_rate\": " << FormatFloat(frame_rate) << ",\n";
    file << "  \"frame_count\": " << frame_count << ",\n";
    file << "  \"frame_start\": 0,\n";
    file << "  \"frame_end\": " << (frame_count > 0 ? frame_count - 1 : 0)
         << ",\n";
    file << "  \"frames\": [\n";

    for (int frame = 0; frame < frame_count; ++frame) {
      const float sample_time = samples[frame];
      const float ratio = duration > kEpsilon ? sample_time / duration : 0.f;
      sampling_job.ratio = std::clamp(ratio, 0.f, 1.f);
      if (!sampling_job.Run()) {
        ozz::log::Err() << "SamplingJob failed at frame " << frame
                        << std::endl;
        return false;
      }
      if (!ltm_job.Run()) {
        ozz::log::Err() << "LocalToModelJob failed at frame " << frame
                        << std::endl;
        return false;
      }

      file << "    {\n";
      file << "      \"frame\": " << frame << ",\n";
      file << "      \"time\": " << FormatFloat(sample_time) << ",\n";
      file << "      \"ratio\": " << FormatFloat(sampling_job.ratio)
           << ",\n";
      file << "      \"bones\": {\n";

      for (int joint = 0; joint < joint_count; ++joint) {
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

        file << "        \"" << EscapeJsonString(joint_names[joint])
             << "\": {\n";
        file << "          \"location\": [" << FormatFloat(tx) << ", "
             << FormatFloat(ty) << ", " << FormatFloat(tz) << "],\n";
        file << "          \"rotation\": [" << FormatFloat(quat_values[0])
             << ", " << FormatFloat(quat_values[1]) << ", "
             << FormatFloat(quat_values[2]) << ", "
             << FormatFloat(quat_values[3]) << "],\n";
        file << "          \"scale\": [" << FormatFloat(sx) << ", "
             << FormatFloat(sy) << ", " << FormatFloat(sz) << "]\n";
        file << "        }";
        if (joint + 1 != joint_count) {
          file << ",";
        }
        file << "\n";
      }

      file << "      }\n";
      file << "    }";
      if (frame + 1 != frame_count) {
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

int main(int _argc, const char** _argv) {
  const char* title =
      "Ozz-animation sample: Binary animation/skeleton playback";
  return PlaybackSampleApplication().Run(_argc, _argv, "1.0", title);
}
