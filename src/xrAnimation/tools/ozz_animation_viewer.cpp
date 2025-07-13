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

#include "framework/application.h"
#include "framework/imgui.h"
#include "framework/renderer.h"
#include "framework/utils.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/log.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/vec_float.h"
#include "ozz/base/maths/box.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/io/archive.h"
#include <cstdio>
#include "ozz/options/options.h"
#include <iostream>
#include <iomanip>
#include <limits>

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
};

int main(int _argc, const char** _argv) {
  const char* title =
      "Ozz-animation sample: Binary animation/skeleton playback";
  return PlaybackSampleApplication().Run(_argc, _argv, "1.0", title);
}
