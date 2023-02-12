/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_CAMERA_FILE_CAMERA_HPP
#define NIE_HARDWARE_CAMERA_FILE_CAMERA_HPP

#include <glog/logging.h>

#include <atomic>  // for std::atomic_bool
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>  // for std::thread

#include <boost/filesystem.hpp>
#include <nie/core/container/mt_circular_buffer.hpp>
#include <nie/core/string.hpp>
#include <opencv2/opencv.hpp>

#include "camera.hpp"

namespace nie {

//
//
/**
 * Usage:
 *  1. call StartGrabbing()
 *  2. repeatedly call GetFrame() an arbitrary number of times.
 *  3. call StopGrabbing()
 * GetFrame() should only be called after StartGrabbing() and before StopGrabbing().
 * FileCamera implements the ICamera interface by reading image files from a given directory path.
 * The images are read in a separate thread and the communication among threads is done via a circular queue
 */
class FileCamera : public Camera {
public:
    FileCamera() = delete;
    ~FileCamera() override { StopGrabbing(); }

    /**
     * The directory of image files is scanned for filenames which are sorted lexicographically.
     * Images are then read in order one-by-one and stored in nie::CircularBuffer.
     *
     * This class cannot be copied or moved because nie::CircularBuffer cannot be copied or moved.
     * Use std::unique_ptr<> in order pass this class around.
     *
     * @param images_path  Path to directory containing images.
     */
    FileCamera(std::string const& images_path)
        : is_frame_grabbing_thread_running_{false},
          grabbing_done_{false},
          images_path_{images_path},
          image_size_{{0, 0}},
          number_of_frames_(0),
          first_image_(0) {}
    /**
     * @param images_path  Path to directory containing images.
     * @param number_of_frames  Number of images to read.
     */
    FileCamera(std::string const& images_path, size_t number_of_frames)
        : is_frame_grabbing_thread_running_{false},
          grabbing_done_{false},
          images_path_{images_path},
          image_size_{{0, 0}},
          number_of_frames_(number_of_frames),
          first_image_(0) {}
    /**
     * @param images_path  Path to directory containing images.
     * @param number_of_frames  Number of images to read.
     * @param first_image  Index in sorted filename list where we start reading images.
     */
    FileCamera(std::string const& images_path, size_t number_of_frames, size_t first_image)
        : is_frame_grabbing_thread_running_{false},
          grabbing_done_{false},
          images_path_{images_path},
          image_size_{{0, 0}},
          number_of_frames_(number_of_frames),
          first_image_(first_image) {}
    /**
     * @param images_path
     * @param image_size  Expected size of images (legacy code).
     */
    FileCamera(std::string const& images_path, cv::Size const& image_size)
        : is_frame_grabbing_thread_running_{false},
          grabbing_done_{false},
          images_path_{images_path},
          image_size_{image_size},
          number_of_frames_(0),
          first_image_(0) {}
    // explicitly delete copying (nie::mt::MtCircularBuffer cannot be copied)
    FileCamera(FileCamera const&) = delete;
    FileCamera& operator=(FileCamera const&) = delete;
    // explicitly delete moving (nie::mt::MtCircularBuffer cannot be moved)
    FileCamera(FileCamera&& other) = delete;
    FileCamera& operator=(FileCamera&& other) = delete;

    bool GetFrame(
            std::uint32_t const& timeout_ms,
            cv::Mat* frame,
            std::int64_t* system_time,
            bool const& throws_on_timeout) override {
        cv::Mat frame_grabbing_output;
        bool const pop_succeeded = frame_buffer_.WaitForPopFront(&frame_grabbing_output, timeout_ms);

        if (!pop_succeeded && throws_on_timeout) {
            throw std::runtime_error(
                    "FileCamera::GetFrame(): [timeout] exceeded the threshold of " + std::to_string(timeout_ms) +
                    " ms");
        }

        *frame = frame_grabbing_output;
        *system_time = std::chrono::system_clock::now().time_since_epoch().count();

        return pop_succeeded;
    }

    void StartGrabbing(GrabStrategy strategy) override {
        // Like basler_camera and the basler SDK, calling StartGrabbing while grabbing
        // switches the GrabStrategy -- this is not super cool but we want to ensure
        // uniformity across all the realizations of the Camera interface. If we are
        // already grabbing, of course, we return early.
        SwitchStrategy(strategy);
        if (IsGrabbing()) {
            return;
        }

        // We do this here and not inside the thread because it is technically a very
        // bad idea to control the thread from inside the thread (as I realized while
        // I tried to fix some threading edge cases). The downside is that if the
        // client asks really quickly whether or not we are grabbing frames, we might
        // lie, since we sort the image paths before reading the files and that may
        // take some time
        is_frame_grabbing_thread_running_ = true;
        // Spawning a new thread that keeps reading images from the files in a directory
        frame_grabbing_thread_ = std::thread(&FileCamera::RunGrabbingThread, this);
    }

    /**
     * Buffer shall be cleared.
     */
    void StopGrabbing() override {
        if (!is_frame_grabbing_thread_running_) {
            return;
        }

        // We are no longer grabbing frames
        grabbing_done_ = true;

        // Stop the frame grabbing loop in the child thread
        is_frame_grabbing_thread_running_ = false;

        if (frame_grabbing_thread_.joinable()) {
            if (frame_buffer_.Full()) {
                // If the buffer is full then the child thread might be blocked by BlockingPushBack()
                // Pop the buffer to make sure that the child thread is not blocked by BlockingPushBack().
                cv::Mat frame;
                frame_buffer_.PopFront(&frame);
            }

            frame_grabbing_thread_.join();
        }
        // Clear the buffer to make sure it is empty.
        frame_buffer_.Clear();
    }

    bool IsGrabbing() const override { return is_frame_grabbing_thread_running_; }
    bool IsGrabbingDone() const { return grabbing_done_; }

    bool WaitForTriggerReady(std::uint32_t const&, bool) override { throw std::runtime_error("Not implemented"); };

    std::string id() const override {
        // The id for FileCamera is the path to the directory containing the images
        return images_path_;
    }

    cv::Size2i image_size() const override { return image_size_; }

private:
    // The actual frame grabbing thread
    void RunGrabbingThread() {
        // boolean for detecting if the
        grabbing_done_ = false;
        // we use this to avoid a deadlock when stopping the thread whilst
        // maintaining the behavior we expect from a file data provider
        boost::filesystem::directory_iterator image_folder_iterator;
        try {
            image_folder_iterator = boost::filesystem::directory_iterator{images_path_};
        } catch (boost::filesystem::filesystem_error e) {
            std::cerr << e.what();
            grabbing_done_ = true;
            return;
        }
        std::vector<boost::filesystem::path> image_paths;
        for (auto const& item : image_folder_iterator) {
            image_paths.push_back(item.path());
        }

        // std::filesystem::directory_iterator does not guarantee any ordering,
        // so we have to sort the paths ourselves - should work fine as long as
        // the naming/numbering scheme of the folder is consistent
        std::sort(image_paths.begin(), image_paths.end());
        image_paths.erase(image_paths.begin(), image_paths.begin() + first_image_);
        if (number_of_frames_ > 0) {
            image_paths.erase(image_paths.begin() + number_of_frames_, image_paths.end());
        }

        auto image_paths_iterator = image_paths.begin();

        size_t count = 0;
        while (is_frame_grabbing_thread_running_ && (image_paths_iterator != image_paths.end())) {
            auto const image_name = image_paths_iterator->string();
            cv::Mat const frame = cv::imread(image_name);
            frame_buffer_.BlockingPushBack(frame);
            ++image_paths_iterator;
            VLOG(6) << "Grabbed frame " << count;
            ++count;
        }
        // We are no longer grabbing frames
        grabbing_done_ = true;
    }

    // Implements GrabStrategy switching -- which basler apparently does but is not super clear about
    void SwitchStrategy(nie::GrabStrategy strategy) {
        switch (strategy) {
            case GrabStrategy::kLatestImageOnly:
                frame_buffer_.Resize(1);
                break;
            case GrabStrategy::kOneByOne:
                frame_buffer_.Resize(kMaxFrameBufferLength_);
                break;
        }
    }

    // Size of the circular buffer used in the frame grabbing thread
    static constexpr int kMaxFrameBufferLength_ = 10;
    // Used for buffering the frame stream
    mt::MtCircularBuffer<cv::Mat> frame_buffer_;
    // Thread-safe bool to signal the grabbing thread to stop
    std::atomic_bool is_frame_grabbing_thread_running_;
    // Thread-safe bool so that the grabbing thread can signal us that it is done
    std::atomic_bool grabbing_done_;
    // The frame grabbing thread can be seen as a frame stream
    std::thread frame_grabbing_thread_;
    // Path to directory containing images
    std::string images_path_;
    // Size of the images
    cv::Size image_size_;
    // Number of frames to load, zero is all
    size_t number_of_frames_;
    // First image to load
    size_t first_image_;
};

}  // namespace nie

#endif  // NIE_HARDWARE_CAMERA_FILE_CAMERA_HPP
