/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "basler_camera.hpp"

#include <chrono>

#include <glog/logging.h>
#include <functional>

namespace nie {

std::vector<std::string> BaslerCamera::GetDeviceIds() {
    Pylon::PylonAutoInitTerm pylon_auto_init_term;
    Pylon::DeviceInfoList_t devices;
    Pylon::CTlFactory::GetInstance().EnumerateDevices(devices);
    std::vector<std::string> ids;

    for (auto&& device : devices) {
        ids.push_back(device.GetSerialNumber().c_str());
    }

    return ids;
}

BaslerCamera::BaslerCamera()
    : BaslerCamera({}, {[]() { return Pylon::CTlFactory::GetInstance().CreateFirstDevice(); }}) {}

BaslerCamera::BaslerCamera(const std::string& id)
    : BaslerCamera(
          {}, {[id]() {
              return Pylon::CTlFactory::GetInstance().CreateDevice(Pylon::CDeviceInfo().SetSerialNumber(id.c_str()));
          }}) {}

BaslerCamera::BaslerCamera(const BaslerParameters& parameters)
    : BaslerCamera(parameters, {[]() { return Pylon::CTlFactory::GetInstance().CreateFirstDevice(); }}) {}

BaslerCamera::BaslerCamera(const std::string& id, const BaslerParameters& parameters)
    : BaslerCamera(
          parameters, {[id]() {
              return Pylon::CTlFactory::GetInstance().CreateDevice(Pylon::CDeviceInfo().SetSerialNumber(id.c_str()));
          }}) {}

BaslerCamera::BaslerCamera(
    const BaslerParameters& parameters, std::function<Pylon::IPylonDevice*(void)> device_creator) try
    : pylon_auto_init_term_(),
      camera_(device_creator()) {
    // NOTE: According to their default configuration, the cameras are set up for free-running continuous acquisition.

    camera_.Open();
    camera_.PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_BayerRG8);
    camera_.GainAuto.SetValue(Basler_UsbCameraParams::GainAuto_Continuous);
    // The default uses full resolution image and strives for a target exposure value of 0.5 (around 128 pixel value for
    // 8 bits).
    camera_.ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Continuous);
    // This as a default gives the cleanest (meaning least amount of noise) images.
    camera_.AutoFunctionProfile.SetValue(Basler_UsbCameraParams::AutoFunctionProfile_MinimizeGain);
    camera_.AutoTargetBrightness.SetValue(parameters.auto_exposure_brightness);
    camera_.AutoFunctionROISelector.SetValue(Basler_UsbCameraParams::AutoFunctionROISelector_ROI1);
    camera_.AutoFunctionROIUseBrightness.SetValue(true);
    // Setting the ROI to a 1,1 region on 0,0 ensures that the error checking in SetValue for the various
    // properties is not order dependent.
    camera_.AutoFunctionROIOffsetX.SetValue(0);
    camera_.AutoFunctionROIOffsetY.SetValue(0);
    camera_.AutoFunctionROIWidth.SetValue(1);
    camera_.AutoFunctionROIHeight.SetValue(1);
    // Set the actual ROI
    camera_.AutoFunctionROIOffsetX.SetValue(parameters.auto_exposure_roi.x);
    camera_.AutoFunctionROIOffsetY.SetValue(parameters.auto_exposure_roi.y);
    camera_.AutoFunctionROIWidth.SetValue(parameters.auto_exposure_roi.width);
    camera_.AutoFunctionROIHeight.SetValue(parameters.auto_exposure_roi.height);

    // Extra data chunks to be inserted
    if (GenApi::IsWritable(camera_.ChunkModeActive)) {
        camera_.ChunkModeActive.SetValue(true);
    } else {
        // The only reason this is kept as a basler runtime exception is because of the catch below that
        // translates everything already. An extra catch would be needed if we threw a default std::exception
        // derived one
        throw RUNTIME_EXCEPTION("The camera doesn't support chunk features");
    }

    camera_.ChunkSelector.SetValue(Basler_UsbCameraParams::ChunkSelector_Timestamp);
    camera_.ChunkEnable.SetValue(true);

    // Triggering
    camera_.TriggerSelector.SetValue(parameters.trigger_selector);
    camera_.TriggerMode.SetValue(parameters.trigger_mode);
    camera_.TriggerSource.SetValue(parameters.trigger_source);
    camera_.TriggerActivation.SetValue(parameters.trigger_activation);

    VLOG(2) << "Using device: " << camera_.GetDeviceInfo().GetSerialNumber();
} catch (const Pylon::GenericException& e) {
    throw std::runtime_error("BaslerCamera::BaslerCamera(): " + std::string(e.what()));
}

BaslerCamera::~BaslerCamera() {
    if (IsGrabbing()) StopGrabbing();
}

void BaslerCamera::StartGrabbing(GrabStrategy strategy) {
    try {
        switch (strategy) {
            case GrabStrategy::kOneByOne:
                camera_.StartGrabbing(Pylon::GrabStrategy_OneByOne);
                break;
            case GrabStrategy::kLatestImageOnly:
                camera_.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
                break;
        }
    } catch (const Pylon::GenericException& e) {
        throw std::runtime_error("BaslerCamera::StartGrabbing(): " + std::string(e.what()));
    }
}

void BaslerCamera::StopGrabbing() { camera_.StopGrabbing(); }

bool BaslerCamera::IsGrabbing() const { return camera_.IsGrabbing(); }

// 6.4.3.1. Basler USB 3.0 Ace manual
// When the camera receives a hardware trigger signal and begins exposure, it will exit the "waiting
// for frame start trigger" acquisition status because at that point, it cannot react to a new frame start
// trigger signal. As soon as the camera is capable of reacting to a new frame start trigger signal, it
// will automatically return to the "waiting for frame start trigger" acquisition status.
bool BaslerCamera::WaitForTriggerReady(const std::uint32_t& timeout_ms, bool throws_on_timeout) {
    try {
        return camera_.WaitForFrameTriggerReady(
            timeout_ms, (throws_on_timeout) ? Pylon::TimeoutHandling_ThrowException : Pylon::TimeoutHandling_Return);
    } catch (const Pylon::GenericException& e) {
        throw std::runtime_error("BaslerCamera::WaitForTriggerReady(): " + std::string(e.what()));
    }
}

// TODO(jbr) Perhaps this timeout should be configurable
bool BaslerCamera::WaitForGrabResult(const std::uint32_t& timeout_ms) {
    return camera_.GetGrabResultWaitObject().Wait(timeout_ms);
}

// Get a single frame while grabbing.
// Outputs an image and its corresponding time of capture as system time in nanoseconds.

bool BaslerCamera::GetFrame(
    const std::uint32_t& timeout_ms, RawFrame* frame, std::int64_t* system_time, const bool& throws_on_timeout) {
    Pylon::CBaslerUsbGrabResultPtr grab_result;

    if (!RetrieveResult(timeout_ms, throws_on_timeout, &grab_result)) return false;

    GetFromGrabResult(grab_result, frame, system_time);
    return true;
}

bool BaslerCamera::GetFrame(
    const std::uint32_t& timeout_ms, cv::Mat* frame, std::int64_t* system_time, const bool& throws_on_timeout) {
    Pylon::CBaslerUsbGrabResultPtr grab_result;

    if (!RetrieveResult(timeout_ms, throws_on_timeout, &grab_result)) return false;

    GetFromGrabResult(grab_result, frame, system_time);
    return true;
}

bool BaslerCamera::GetFrame(
    const std::uint32_t& timeout_ms,
    RawFrame* frame_raw,
    cv::Mat* frame_cv,
    std::int64_t* system_time,
    const bool& throws_on_timeout) {
    Pylon::CBaslerUsbGrabResultPtr grab_result;

    if (!RetrieveResult(timeout_ms, throws_on_timeout, &grab_result)) return false;

    GetFromGrabResult(grab_result, frame_raw, frame_cv, system_time);
    return true;
}

// Get a single frame without the use of grabbing.
// Outputs an image and its corresponding time of capture as system time in nanoseconds.
bool BaslerCamera::GetSingleFrame(
    const std::uint32_t& timeout_ms, RawFrame* frame, std::int64_t* system_time, const bool& throws_on_timeout) {
    Pylon::CBaslerUsbGrabResultPtr grab_result;

    if (!GrabOne(timeout_ms, throws_on_timeout, &grab_result)) return false;

    GetFromGrabResult(grab_result, frame, system_time);
    return true;
}

bool BaslerCamera::GetSingleFrame(
    const std::uint32_t& timeout_ms, cv::Mat* frame, std::int64_t* system_time, const bool& throws_on_timeout) {
    Pylon::CBaslerUsbGrabResultPtr grab_result;

    if (!GrabOne(timeout_ms, throws_on_timeout, &grab_result)) return false;

    GetFromGrabResult(grab_result, frame, system_time);
    return true;
}

bool BaslerCamera::GetSingleFrame(
    const std::uint32_t& timeout_ms,
    RawFrame* frame_raw,
    cv::Mat* frame_cv,
    std::int64_t* system_time,
    const bool& throws_on_timeout) {
    Pylon::CBaslerUsbGrabResultPtr grab_result;

    if (!GrabOne(timeout_ms, throws_on_timeout, &grab_result)) return false;

    GetFromGrabResult(grab_result, frame_raw, frame_cv, system_time);
    return true;
}

std::string BaslerCamera::id() const { return std::string(camera_.GetDeviceInfo().GetSerialNumber().c_str()); }

cv::Size BaslerCamera::image_size() const {
    return cv::Size(static_cast<int>(camera_.Width.GetValue()), static_cast<int>(camera_.Height.GetValue()));
}

Pylon::CBaslerUsbInstantCamera& BaslerCamera::camera() { return camera_; }

bool BaslerCamera::RetrieveResult(
    const std::uint32_t& timeout_ms, const bool& throws_on_timeout, Pylon::CBaslerUsbGrabResultPtr* grab_result) {
    bool success;

    try {
        success = camera_.RetrieveResult(
            timeout_ms,
            *grab_result,
            (throws_on_timeout) ? Pylon::TimeoutHandling_ThrowException : Pylon::TimeoutHandling_Return);

        if (success) CheckGrabResult(*grab_result);
    } catch (const Pylon::GenericException& e) {
        throw std::runtime_error("BaslerCamera::RetrieveResult(): " + std::string(e.what()));
    }

    return success;
}

bool BaslerCamera::GrabOne(
    const std::uint32_t& timeout_ms, const bool& throws_on_timeout, Pylon::CBaslerUsbGrabResultPtr* grab_result) {
    bool success;

    try {
        success = camera_.GrabOne(
            timeout_ms,
            *grab_result,
            (throws_on_timeout) ? Pylon::TimeoutHandling_ThrowException : Pylon::TimeoutHandling_Return);

        if (success) CheckGrabResult(*grab_result);
    } catch (const Pylon::GenericException& e) {
        throw std::runtime_error("BaslerCamera::GrabOne(): " + std::string(e.what()));
    }

    return success;
}

void BaslerCamera::CheckGrabResult(const Pylon::CBaslerUsbGrabResultPtr& grab_result) const {
    // Image grabbed successfully?
    if (!grab_result->GrabSucceeded()) {
        throw std::runtime_error(
            "BaslerCamera::CheckGrabResult(): " + std::to_string(grab_result->GetErrorCode()) + " " +
            std::string(grab_result->GetErrorDescription()));
    }
}

void BaslerCamera::GetFromGrabResult(const Pylon::CBaslerUsbGrabResultPtr& grab_result, RawFrame* frame) const {
    frame->Create(grab_result->GetWidth(), grab_result->GetHeight());

    std::size_t size = grab_result->GetWidth() * grab_result->GetHeight();
    std::uint8_t* src_begin = static_cast<std::uint8_t*>(grab_result->GetBuffer());
    std::uint8_t* src_end = src_begin + size;
    std::uint8_t* dst_begin = frame->ptr<std::uint8_t>();

    std::copy(src_begin, src_end, dst_begin);
}

void BaslerCamera::GetFromGrabResult(const Pylon::CBaslerUsbGrabResultPtr& grab_result, cv::Mat* frame) const {
    frame->create(grab_result->GetHeight(), grab_result->GetWidth(), CV_8UC3);

    // The format converter to convert to the opencv bgr format.
    Pylon::CImageFormatConverter format_converter;
    format_converter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    // Convert is non-const. So we create one per image.
    format_converter.Convert(
        frame->ptr<void>(),
        // TODO(jbr) OpenCV doesn't always give continuous buffer?
        frame->total() * frame->elemSize(),
        grab_result);
}

void BaslerCamera::GetFromGrabResult(
    const Pylon::CBaslerUsbGrabResultPtr& grab_result, std::int64_t* system_time) const {
    // Is this really needed?
    if (Pylon::PayloadType_ChunkData != grab_result->GetPayloadType()) {
        throw std::runtime_error("BaslerCamera::GetFromGrabResult(): Unexpected payload type received.");
    }

    // Check is nonsense...
    // if (IsReadable(grab_result_->ChunkTimestamp))
    // TODO(jbr) We could check the time it takes to get the time on the camera and can use half of that to compensate?
    camera_.TimestampLatch.Execute();

    std::int64_t frame_time = grab_result->ChunkTimestamp.GetValue();
    std::int64_t camera_time = camera_.TimestampLatchValue.GetValue();

    *system_time = std::chrono::system_clock::now().time_since_epoch().count() - (camera_time - frame_time);
}

void BaslerCamera::GetFromGrabResult(
    const Pylon::CBaslerUsbGrabResultPtr& grab_result, RawFrame* frame, std::int64_t* system_time) const {
    GetFromGrabResult(grab_result, system_time);
    GetFromGrabResult(grab_result, frame);
}

void BaslerCamera::GetFromGrabResult(
    const Pylon::CBaslerUsbGrabResultPtr& grab_result, cv::Mat* frame, std::int64_t* system_time) const {
    GetFromGrabResult(grab_result, system_time);
    GetFromGrabResult(grab_result, frame);
}

void BaslerCamera::GetFromGrabResult(
    const Pylon::CBaslerUsbGrabResultPtr& grab_result,
    RawFrame* frame_raw,
    cv::Mat* frame_cv,
    std::int64_t* system_time) const {
    GetFromGrabResult(grab_result, system_time);
    GetFromGrabResult(grab_result, frame_raw);
    GetFromGrabResult(grab_result, frame_cv);
}

}  // namespace nie