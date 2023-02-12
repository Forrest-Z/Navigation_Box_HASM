/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_LADYBUG_H
#define NIE_FORMATS_LADYBUG_H

// Standard includes
#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// Glog includes
#include <glog/logging.h>

// Boost includes
#include <boost/filesystem.hpp>

// OpenCV includes
#include <opencv2/core/mat.hpp>

// Ladybug includes
#include <ladybug/ladybug.h>
#include <ladybug/ladybugrenderer.h>
#include <ladybug/ladybugstream.h>

namespace nie {
namespace io {
namespace ladybug {

class Exception : public std::exception {
public:
    explicit Exception(LadybugError const& error) : error_(error) {}

    char const* what() const noexcept override { return ladybugErrorToString(error_); }

    LadybugError error() const { return error_; }

private:
    LadybugError error_;
};

namespace detail {

// Simple function to replace ladybug error reporting with custom C++ exception
inline void Check(LadybugError const& error) {
    if (error != LADYBUG_OK) {
        throw Exception(error);
    }
}

// RAII class for ladybug library contexts
template <typename Context, LadybugError createFunc(Context*), LadybugError destroyFunc(Context*)>
class ContextRAII {
public:
    ContextRAII() { Check(createFunc(&context_)); }

    ~ContextRAII() {
        try {
            Check(destroyFunc(&context_));
        } catch (Exception const& e) {
            LOG(ERROR) << "Ladybug error #" << e.error() << ": " << e.what();
        }
    }

    // Implicit conversion operator
    operator Context() { return context_; }

private:
    Context context_;
};

}  // namespace detail

// Explicit instantiation of known ladybug contexts
using Context = detail::ContextRAII<LadybugContext, &ladybugCreateContext, &ladybugDestroyContext>;
using StreamContext =
    detail::ContextRAII<LadybugStreamContext, &ladybugCreateStreamContext, &ladybugDestroyStreamContext>;

// Convenience alias
using Image = LadybugImage;

class StreamReader {
public:
    explicit StreamReader(boost::filesystem::path const& stream_file);
    ~StreamReader();

    unsigned frame_id() const { return frame_id_; }

    unsigned frame_count() const { return frame_count_; }

    float frame_rate() const { return frame_rate_; }

    void SaveCalibrationFile(boost::filesystem::path const& calibration_file);

    // Read the image and move to the next frame
    // NOTE: The frame_id will not move beyond the last frame
    Image Read();

    // Seek to the frame with the given index
    void Seek(unsigned frame_id);

private:
    StreamContext stream_context_;
    unsigned stream_version_;

    unsigned frame_id_;
    unsigned frame_count_;
    float frame_rate_;
};

class ImageProcessor {
public:
    explicit ImageProcessor(
        boost::filesystem::path const& calibration_file,
        bool falloff_correction_enabled = true,
        float falloff_correction_value = 1.0f,
        unsigned blending_width = 100,
        bool enable_anti_aliasing = false);

    void SetViewParameters(float fov, float rot_x, float rot_y, float rot_z, float pos_x, float pos_y, float pos_z);

    cv::Mat Process(
        Image const& image,
        LadybugOutputImage output_image_type = LADYBUG_PANORAMIC,
        unsigned output_image_width = 2048,
        unsigned output_image_height = 1024,
        LadybugPixelFormat output_image_format = LADYBUG_BGR,
        LadybugColorProcessingMethod color_processing_method = LADYBUG_HQLINEAR);

private:
    using Buffers = std::array<std::vector<unsigned char>, LADYBUG_NUM_CAMERAS>;

    static std::vector<unsigned char*> Convert(Buffers& buffers);

    // Ladybug library context
    Context context_;

    // Texture buffers
    Buffers texture_buffers_;
    unsigned texture_width_;
    unsigned texture_height_;

    LadybugPixelFormat pixel_format_;
};

}  // namespace ladybug
}  // namespace io
}  // namespace nie

#endif  // NIE_FORMATS_LADYBUG_H
