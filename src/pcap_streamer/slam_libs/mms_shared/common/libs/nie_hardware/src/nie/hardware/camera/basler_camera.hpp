/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_CAMERA_BASLER_CAMERA_HPP
#define NIE_HARDWARE_CAMERA_BASLER_CAMERA_HPP

#include <cstdint>

#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <opencv2/core.hpp>

#include "camera.hpp"
#include "raw_frame.hpp"

namespace nie {

// TODO(jbr) Perhaps add padding and bit depth choice, etc., in the future
class BaslerParameters {
public:
    BaslerParameters()
        : auto_exposure_brightness(0.5),
          auto_exposure_roi(0, 0, 1920, 1200),
          trigger_mode(Basler_UsbCameraParams::TriggerMode_Off),
          trigger_selector(Basler_UsbCameraParams::TriggerSelector_FrameStart),
          trigger_source(Basler_UsbCameraParams::TriggerSource_Software),
          trigger_activation(Basler_UsbCameraParams::TriggerActivation_RisingEdge) {}

    double auto_exposure_brightness;
    cv::Rect auto_exposure_roi;
    Basler_UsbCameraParams::TriggerModeEnums trigger_mode;
    Basler_UsbCameraParams::TriggerSelectorEnums trigger_selector;
    Basler_UsbCameraParams::TriggerSourceEnums trigger_source;
    Basler_UsbCameraParams::TriggerActivationEnums trigger_activation;
};

// TODO(jbr) The camera now always starts in continuous mode
class BaslerCamera final : public Camera {
public:
    static std::vector<std::string> GetDeviceIds();

    BaslerCamera();
    explicit BaslerCamera(const std::string& id);
    explicit BaslerCamera(const BaslerParameters& parameters);
    BaslerCamera(const std::string& id, const BaslerParameters& parameters);
    ~BaslerCamera();

    void StartGrabbing(GrabStrategy strategy = GrabStrategy::kOneByOne) override;
    void StopGrabbing() override;
    bool IsGrabbing() const override;
    //
    bool WaitForTriggerReady(const std::uint32_t& timeout_ms, bool throws_on_timeout = false) override;
    // Returns false in case there was no grab result available after the timeout.
    bool WaitForGrabResult(const std::uint32_t& timeout_ms);

    // Get a single frame while grabbing.
    // Outputs an image and its corresponding time of capture as system time in nanoseconds.
    bool GetFrame(
            const std::uint32_t& timeout_ms,
            RawFrame* frame,
            std::int64_t* system_time,
            const bool& throws_on_timeout = false);
    bool GetFrame(
            const std::uint32_t& timeout_ms,
            cv::Mat* frame,
            std::int64_t* system_time,
            const bool& throws_on_timeout = false) override;
    bool GetFrame(
            const std::uint32_t& timeout_ms,
            RawFrame* frame_raw,
            cv::Mat* frame_cv,
            std::int64_t* system_time,
            const bool& throws_on_timeout = false);

    // Get a single frame without the use of grabbing.
    // Outputs an image and its corresponding time of capture as system time in nanoseconds.
    bool GetSingleFrame(
            const std::uint32_t& timeout_ms,
            RawFrame* frame,
            std::int64_t* system_time,
            const bool& throws_on_timeout = false);
    bool GetSingleFrame(
            const std::uint32_t& timeout_ms,
            cv::Mat* frame,
            std::int64_t* system_time,
            const bool& throws_on_timeout = false);
    bool GetSingleFrame(
            const std::uint32_t& timeout_ms,
            RawFrame* frame_raw,
            cv::Mat* frame_cv,
            std::int64_t* system_time,
            const bool& throws_on_timeout = false);

    std::string id() const;
    cv::Size image_size() const;
    const Pylon::CBaslerUsbInstantCamera& camera() const;
    Pylon::CBaslerUsbInstantCamera& camera();

private:
    BaslerCamera(const BaslerParameters& parameters, std::function<Pylon::IPylonDevice*(void)> device_creator);

    bool RetrieveResult(
            const std::uint32_t& timeout_ms,
            const bool& throws_on_timeout,
            Pylon::CBaslerUsbGrabResultPtr* grab_result);
    bool GrabOne(
            const std::uint32_t& timeout_ms,
            const bool& throws_on_timeout,
            Pylon::CBaslerUsbGrabResultPtr* grab_result);
    void CheckGrabResult(const Pylon::CBaslerUsbGrabResultPtr& grab_result) const;
    void GetFromGrabResult(const Pylon::CBaslerUsbGrabResultPtr& grab_result, RawFrame* frame) const;
    void GetFromGrabResult(const Pylon::CBaslerUsbGrabResultPtr& grab_result, cv::Mat* frame) const;
    void GetFromGrabResult(const Pylon::CBaslerUsbGrabResultPtr& grab_result, std::int64_t* system_time) const;

    void GetFromGrabResult(
            const Pylon::CBaslerUsbGrabResultPtr& grab_result, RawFrame* frame, std::int64_t* system_time) const;
    void GetFromGrabResult(
            const Pylon::CBaslerUsbGrabResultPtr& grab_result, cv::Mat* frame, std::int64_t* system_time) const;
    void GetFromGrabResult(
            const Pylon::CBaslerUsbGrabResultPtr& grab_result,
            RawFrame* frame_raw,
            cv::Mat* frame_cv,
            std::int64_t* system_time) const;

    // RAII style auto initialize and terminate. Nice of them we don't have to make it.
    // Has reference counting in case of multiple cameras.
    // Keep as the first variable.
    Pylon::PylonAutoInitTerm pylon_auto_init_term_;
    Pylon::CBaslerUsbInstantCamera camera_;
};

}  // namespace nie

#endif  // NIE_HARDWARE_CAMERA_BASLER_CAMERA_HPP