/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_POSITIONING_DETAIL_POSITION_READER_HPP_
#define NIE_HARDWARE_POSITIONING_DETAIL_POSITION_READER_HPP_

#include <atomic>  // atomic_bool

#include <nie/core/container/mt_circular_buffer.hpp>
#include <nie/core/geometry/isometry3.hpp>

#include "nie/hardware/timing/time_defs.hpp"
#include "nie/hardware/timing/time_synchronizer.hpp"
#include "raw_position_data.hpp"
#include "timestamped_isometry.hpp"

namespace nie {

/// Hardware interface for any device that provides position and orientation data.
/// This class can be used inside a worker thread by first calling StartGrabbing(), then Run() which will enter a while
/// loop until there is no data left to read or until StopGrabbing() is called.
class PositionReader {
public:
    using Scalar = double;

public:
    PositionReader(size_t const buffer_size, Scalar const alpha = 0.5)
        : position_buffer_{buffer_size}, do_grabbing_{false}, time_sync_{alpha} {}

    virtual ~PositionReader() = default;

    virtual void Run() = 0;

    /// StartGrabbing must be called before calling Run
    virtual void StartGrabbing() { do_grabbing_ = true; }

    /// Stop reading data
    virtual void StopGrabbing() { do_grabbing_ = false; }

    bool IsGrabbing() const { return do_grabbing_; }

    /// Passes data to CircluarBuffer
    void OnPosition(detail::RawPositionData<Isometry3qd>&& raw_position_data) {
        // Do time syncing to convert RawPositionData to TimestampedIsometry
        position_buffer_.BlockingPushBack(
            {raw_position_data.isometry, time_sync_.Update(raw_position_data.system_time, raw_position_data.gps_time)});
    }

    mt::MtCircularBuffer<TimestampedIsometry>& position_buffer() { return position_buffer_; }
    mt::MtCircularBuffer<TimestampedIsometry> const& position_buffer() const { return position_buffer_; }

private:
    mt::MtCircularBuffer<TimestampedIsometry> position_buffer_;
    std::atomic_bool do_grabbing_;
    TimeSynchronizer<ExponentialMovingAverage> time_sync_;
};

}  // end namespace nie

#endif  // NIE_HARDWARE_POSITIONING_DETAIL_POSITION_READER_HPP_
