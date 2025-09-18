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
#ifdef _MSC_VER
#include "stdafx.h"
#endif
#include "../Externals/ozz-animation/samples/framework/application.h"
#include "../Externals/ozz-animation/samples/framework/imgui.h"
#include "../Externals/ozz-animation/samples/framework/renderer.h"
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
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include "ozz/base/span.h"

// Skeleton archive can be specified as an option.
OZZ_OPTIONS_DECLARE_STRING(skeleton,
                           "Path to the skeleton (ozz archive format).",
                           "media/skeleton.ozz", false)

// Animation archive can be specified as an option.
OZZ_OPTIONS_DECLARE_STRING(animation,
                           "Path to the animation (ozz archive format).",
                           "media/animation.ozz", false)

class PlaybackSampleApplication : public ozz::sample::Application {
 protected:
  // Load multiple animations from a single file
  bool LoadMultipleAnimations(const char* filename) {
    ozz::io::File file(filename, "rb");
    if (!file.opened()) {
      ozz::log::Err() << "Failed to open animation file: " << filename << std::endl;
      return false;
    }

    ozz::io::IArchive archive(&file);

    // First try to read a single animation (standard format)
    if (archive.TestTag<ozz::animation::Animation>()) {
      // Single animation file
      animations_.resize(1);
      archive >> animations_[0];
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
    for (uint32_t i = 0; i < anim_count; ++i) {
      archive >> animations_[i];
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
    // Debug: count frames and output debug info for first few frames
    debug_frame_count_++;

    if (!animations_.empty() && animations_[current_animation_].duration() > 0.0f) {
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
      // No animation - use bind pose
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

    // Debug: output bone transforms for first few frames
    if (debug_frame_count_ <= 3) {
      std::cout << "\n=== DEBUG FRAME " << debug_frame_count_ << " ===" << std::endl;
      std::cout << "Animation time ratio: " << controller_.time_ratio() << std::endl;

      // Show skeleton bounds
      ozz::math::Box bounds;
      GetSceneBounds(&bounds);
      std::cout << "Scene bounds: min(" << bounds.min.x << ", "
                << bounds.min.y << ", " << bounds.min.z
                << ") max(" << bounds.max.x << ", "
                << bounds.max.y << ", " << bounds.max.z << ")" << std::endl;

      // Show first 10 bone transforms
      const ozz::span<const char* const> joint_names = skeleton_.joint_names();
      for (int i = 0; i < std::min(10, static_cast<int>(models_.size())); ++i) {
        const ozz::math::Float4x4& matrix = models_[i];
        // Extract translation from the last column
        float x = ozz::math::GetX(matrix.cols[3]);
        float y = ozz::math::GetY(matrix.cols[3]);
        float z = ozz::math::GetZ(matrix.cols[3]);
        std::cout << "  Bone[" << i << "] '" << joint_names[i] << "': pos("
                  << std::fixed << std::setprecision(3) << x << ", " << y << ", " << z << ")" << std::endl;
      }

      // Show all bone positions in a compact format
      std::cout << "\nAll bone positions:" << std::endl;
      for (int i = 0; i < static_cast<int>(models_.size()); ++i) {
        const ozz::math::Float4x4& matrix = models_[i];
        float x = ozz::math::GetX(matrix.cols[3]);
        float y = ozz::math::GetY(matrix.cols[3]);
        float z = ozz::math::GetZ(matrix.cols[3]);
        if (i % 5 == 0) std::cout << std::endl; // New line every 5 bones
        std::cout << "(" << std::setw(7) << std::setprecision(2) << x << ","
                  << std::setw(7) << std::setprecision(2) << y << ","
                  << std::setw(7) << std::setprecision(2) << z << ") ";
      }
      std::cout << std::endl;

      if (debug_frame_count_ == 1) {
        PrintPoseTable();
      }
    }

    return true;
  }

  virtual bool OnDisplay(ozz::sample::Renderer* _renderer) {
    // Calculate ground offset to prevent clipping
    float min_z = std::numeric_limits<float>::max();
    for (int i = 0; i < skeleton_.num_joints(); ++i) {
      float z = ozz::math::GetZ(models_[i].cols[3]);
      if (z < min_z) {
        min_z = z;
      }
    }

    // Create transform matrix with vertical offset
    ozz::math::Float4x4 transform = ozz::math::Float4x4::identity();
    if (min_z < 0.0f) {
      // Offset character up so lowest bone is at ground level
      transform.cols[3] = ozz::math::simd_float4::Load(0.0f, 0.0f, -min_z + 0.05f, 1.0f); // +0.05 for small buffer
    }

    return _renderer->DrawPosture(skeleton_, make_span(models_), transform);
  }

  virtual bool OnInitialize() {
    // Initialize debug counter
    debug_frame_count_ = 0;

    // Reading skeleton.
    if (!ozz::sample::LoadSkeleton(OPTIONS_skeleton, &skeleton_)) {
      return false;
    }

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
        std::cout << "  Animation " << i << ": tracks=" << animations_[i].num_tracks()
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

    // Allocates runtime buffers.
    const int num_soa_joints = skeleton_.num_soa_joints();
    locals_.resize(num_soa_joints);
    const int num_joints = skeleton_.num_joints();
    models_.resize(num_joints);

    // Allocates a context that matches animation requirements.
    context_.Resize(num_joints);

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
    if (animations_.size() > 1) {
      static bool select_open = true;
      ozz::sample::ImGui::OpenClose select_oc(_im_gui, "Animation selection", &select_open);
      if (select_open) {
        _im_gui->DoLabel("Current animation:");
        char label[64];
        snprintf(label, sizeof(label), "Animation %d of %zu", current_animation_ + 1, animations_.size());
        _im_gui->DoLabel(label);

        // Previous/Next buttons
        bool changed = false;
        if (_im_gui->DoButton("Previous") && current_animation_ > 0) {
          current_animation_--;
          changed = true;
        }
        // _im_gui->DoSameLine(); // Not available in this ImGui wrapper
        if (_im_gui->DoButton("Next") && current_animation_ < (int)animations_.size() - 1) {
          current_animation_++;
          changed = true;
        }

        // Reset animation when changed
        if (changed) {
          controller_.Reset();
          context_.Invalidate();
        }

        // Animation info
        if (current_animation_ < (int)animations_.size()) {
          snprintf(label, sizeof(label), "Duration: %.2f seconds", animations_[current_animation_].duration());
          _im_gui->DoLabel(label);
        }
      }
    }

    // Exposes animation runtime playback controls.
    {
      static bool open = true;
      ozz::sample::ImGui::OpenClose oc(_im_gui, "Animation control", &open);
      if (open && !animations_.empty()) {
        controller_.OnGui(animations_[current_animation_], _im_gui);
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

  // Current animation index
  int current_animation_ = 0;

  // Sampling context.
  ozz::animation::SamplingJob::Context context_;

  // Buffer of local transforms as sampled from animation_.
  ozz::vector<ozz::math::SoaTransform> locals_;

  // Buffer of model space matrices.
  ozz::vector<ozz::math::Float4x4> models_;

  // Debug frame counter
  int debug_frame_count_;

  // Optional externally forced time ratio for sampling animations.
  float forced_time_ratio_ = 0.f;
  bool use_forced_time_ratio_ = false;
};

int main(int _argc, const char** _argv) {
  const char* title =
      "Ozz-animation sample: Binary animation/skeleton playback";
  return PlaybackSampleApplication().Run(_argc, _argv, "1.0", title);
}
