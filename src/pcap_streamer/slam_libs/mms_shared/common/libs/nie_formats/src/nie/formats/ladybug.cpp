/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "ladybug.hpp"

// Standard includes
#include <iostream>
#include <stdexcept>
#include <unordered_map>

// Ladybug includes
#include <ladybug/ladybugGPS.h>

// NIE includes
#include <nie/core/string.hpp>

namespace nie {
namespace io {
namespace ladybug {

int ConvertToOpenCV(LadybugPixelFormat format) {
    // Provides an (incomplete) mapping from Ladybug format types to OpenCV format types
    // NOTE: OpenCV does currently not provide a half float type (16-bit), but is expected to do so in the future

    // clang-format off
    static std::unordered_map<LadybugPixelFormat, unsigned> mapping =
    {
        {LADYBUG_MONO8,  CV_8UC1},   // 8 bit mono
        {LADYBUG_MONO16, CV_16UC1},  // 16 bit mono

        {LADYBUG_RAW8,   CV_8UC1},   // 8 bit raw data  (Bayer)
        {LADYBUG_RAW16,  CV_16UC1},  // 16 bit raw data (Bayer)

        {LADYBUG_BGR,    CV_8UC3},   // 24 bit BGR
        {LADYBUG_BGRU,   CV_8UC4},   // 32 bit BGRU
        {LADYBUG_BGR16,  CV_16UC3},  // 48 bit BGR  (16 bit int per channel)
        {LADYBUG_BGRU16, CV_16UC4},  // 64 bit BGRU (16 bit int per channel)
//      {LADYBUG_BGR16F, ...},       // 48 bit BGR  (16 bit float per channel)
        {LADYBUG_BGR32F, CV_32FC3},  // 96 bit BGR  (32 bit float per channel)

        {LADYBUG_RGB,    CV_8UC3},   // 24 bit RBG
        {LADYBUG_RGBU,   CV_8UC4},   // 32 bit RGBU
        {LADYBUG_RGB16,  CV_16UC3},  // 48 bit RGB  (16 bit int per channel)
        {LADYBUG_RGBU16, CV_16UC4},  // 64 bit RGBU (16 bit int per channel)
//      {LADYBUG_RGB16F, ...},       // 48 bit RGB  (16 bit float per channel)
        {LADYBUG_RGB32F, CV_32FC3}   // 96 bit RGB  (32 bit float per channel)
    };
    // clang-format on

    // Search the ladybug format
    auto format_it = mapping.find(format);
    if (format_it == mapping.end()) {
        throw std::runtime_error(
            "Unable to derive OpenCV pixel format from Ladybug pixel format (0x" + ToHex(format) + ")");
    }

    // Return the associated OpenCV pixel format
    return format_it->second;
}

unsigned BytesPerPixel(LadybugImage const& image) {
    // clang-format off
    return image.dataFormat == LADYBUG_DATAFORMAT_RAW12                        ||
           image.dataFormat == LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW12            ||
           image.dataFormat == LADYBUG_DATAFORMAT_COLOR_SEP_JPEG12             ||
           image.dataFormat == LADYBUG_DATAFORMAT_COLOR_SEP_HALF_HEIGHT_JPEG12 ||
           image.dataFormat == LADYBUG_DATAFORMAT_RAW16                        ||
           image.dataFormat == LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW16 ? 8 : 4;
    // clang-format on
}

StreamReader::StreamReader(boost::filesystem::path const& stream_path) : frame_id_(0), frame_count_(0) {
    // Initialise the stream for reading (with async buffers to optimize performance)
    detail::Check(ladybugInitializeStreamForReading(stream_context_, stream_path.c_str(), true));

    // Need to query the stream information here, because calling it later will reset the stream reading position
    LadybugStreamHeadInfo headerInfo;
    detail::Check(ladybugGetStreamHeader(stream_context_, &headerInfo));

    // Save stream version
    stream_version_ = headerInfo.ulLadybugStreamVersion;

    // Derive frame rate
    frame_rate_ = stream_version_ < 7 ? static_cast<float>(headerInfo.ulFrameRate) : headerInfo.frameRate;

    // Obtain number of frames
    detail::Check(ladybugGetStreamNumOfImages(stream_context_, &frame_count_));
}

StreamReader::~StreamReader() {
    try {
        // NOTE: The API prescribes that all streams (R/W) should be closed, but the library actually crashes on this
        // detail::check(ladybugStopStream(&stream_context_));
    } catch (Exception const& e) {
        LOG(ERROR) << "Ladybug error #" << e.error() << ": " << e.what();
    }
}

void StreamReader::SaveCalibrationFile(boost::filesystem::path const& calibration_path) {
    // Write file with camera calibration parameters
    detail::Check(ladybugGetStreamConfigFile(stream_context_, calibration_path.c_str()));
}

void StreamReader::Seek(unsigned frame_id) {
    // Check if the frame index should change (Optimisation)
    if (frame_id == frame_id_) return;

    // Seek to the given frame in the stream
    detail::Check(ladybugGoToImage(stream_context_, frame_id));

    // Remember the new frame index
    frame_id_ = frame_id;
}

Image StreamReader::Read() {
    LadybugImage image;

    detail::Check(ladybugReadImageFromStream(stream_context_, &image));

    if (frame_id_ < frame_count_ - 1) ++frame_id_;

    return image;
}

ImageProcessor::ImageProcessor(
    boost::filesystem::path const& calibration_path,
    bool falloff_correction_enabled,
    float falloff_correction_value,
    unsigned blending_width,
    bool enable_anti_aliasing)
    : texture_width_(0), texture_height_(0) {
    // Load configuration file
    detail::Check(ladybugLoadConfig(context_, calibration_path.c_str()));

    // Set falloff correction value and flag
    detail::Check(ladybugSetFalloffCorrectionFlag(context_, falloff_correction_enabled));
    detail::Check(ladybugSetFalloffCorrectionAttenuation(context_, falloff_correction_value));

    // Set blending width
    detail::Check(ladybugSetBlendingParams(context_, blending_width));

    // Make the rendering engine use the alpha mask
    detail::Check(ladybugSetAlphaMasking(context_, true));

    // Set image sampling anti-aliasing disabled / enabled
    detail::Check(ladybugSetAntiAliasing(context_, enable_anti_aliasing));

    // Set default view parameters
    SetViewParameters(60.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void ImageProcessor::SetViewParameters(
    float fov, float rot_x, float rot_y, float rot_z, float pos_x, float pos_y, float pos_z) {
    detail::Check(ladybugSetSphericalViewParams(context_, fov, rot_x, rot_y, rot_z, pos_x, pos_y, pos_z));
}

cv::Mat ImageProcessor::Process(
    Image const& image,
    LadybugOutputImage output_image_type,
    unsigned output_image_width,
    unsigned output_image_height,
    LadybugPixelFormat output_image_format,
    LadybugColorProcessingMethod color_processing_method) {
    // Allocate texture buffers based on image size
    if (texture_width_ != image.uiCols || texture_height_ != image.uiRows) {
        // Cache image size
        texture_width_ = image.uiCols;
        texture_height_ = image.uiRows;

        if (color_processing_method == LADYBUG_DOWNSAMPLE4 || color_processing_method == LADYBUG_MONO) {
            texture_width_ /= 2;
            texture_height_ /= 2;
        }

        // Use the proper output pixel format
        pixel_format_ = BytesPerPixel(image) == 8 ? LADYBUG_BGRU16 : LADYBUG_BGRU;

        // Allocate buffers to hold the color-processed images
        for (unsigned i = 0; i < LADYBUG_NUM_CAMERAS; ++i) {
            texture_buffers_[i].resize(texture_width_ * texture_height_ * BytesPerPixel(image));
        }

        // Initialize alpha masks; NOTE: Can take long time if masks are not yet present in current directory
        detail::Check(ladybugInitializeAlphaMasks(context_, texture_width_, texture_height_));
    }

    // Convert RAII safe C++ style texture buffers to C Style nested pointer for library
    std::vector<unsigned char*> buffers = Convert(texture_buffers_);

    // Set color processing (debayering) method
    detail::Check(ladybugSetColorProcessingMethod(context_, color_processing_method));

    // Configure output image type
    detail::Check(ladybugConfigureOutputImages(context_, output_image_type));

    // Configure off-screen rendering size
    detail::Check(ladybugSetOffScreenImageSize(context_, output_image_type, output_image_width, output_image_height));

    // Convert the image to texture buffers
    detail::Check(ladybugConvertImage(context_, &image, buffers.data(), pixel_format_));

    // Update the textures on graphics card
    detail::Check(ladybugUpdateTextures(
        context_, LADYBUG_NUM_CAMERAS, const_cast<const unsigned char**>(buffers.data()), pixel_format_));

    // Render and obtain the image in off-screen buffer
    LadybugProcessedImage processed_image;
    detail::Check(ladybugRenderOffScreenImage(context_, output_image_type, output_image_format, &processed_image));

    // Convert ladybug image to an OpenCV image; Image data is cloned to avoid issues with memory allocation
    // NOTE: Both Ladybug and OpenCV use BGR channel order; ladybug API specifies that RGB is not even supported
    cv::Mat result = cv::Mat(
                         static_cast<int>(processed_image.uiRows),
                         static_cast<int>(processed_image.uiCols),
                         ConvertToOpenCV(processed_image.pixelFormat),
                         processed_image.pData)
                         .clone();

    // Because we cloned the image data, we can now release the memory that is allocated by the ladybug library
    detail::Check(ladybugReleaseOffScreenImage(context_, output_image_type));

    return result;
}

std::vector<unsigned char*> ImageProcessor::Convert(ImageProcessor::Buffers& buffers) {
    std::vector<unsigned char*> result;

    for (unsigned i = 0; i < LADYBUG_NUM_CAMERAS; ++i) {
        result.push_back(buffers[i].data());
    }

    return result;
}

}  // namespace ladybug
}  // namespace io
}  // namespace nie
