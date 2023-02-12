/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/core/container/mt_circular_buffer.hpp>

#include "circular_buffer_test_templates.hpp"

TEST(MtCircularBuffer, Async) { nie::mt::MtCircularBuffer<size_t> cb{5}; }

TEST(MtCircularBuffer, Size) { cb_test::TestSize<nie::mt::MtCircularBuffer>(); }

TEST(MtCircularBuffer, Count) { cb_test::TestCount<nie::mt::MtCircularBuffer>(); }

TEST(MtCircularBuffer, PopFront) { cb_test::TestPopFront<nie::mt::MtCircularBuffer>(); }

TEST(MtCircularBuffer, Back) { cb_test::TestBack<nie::mt::MtCircularBuffer>(); }

TEST(MtCircularBuffer, Front) { cb_test::TestFront<nie::mt::MtCircularBuffer>(); }

TEST(MtCircularBuffer, Empty) { cb_test::TestEmpty<nie::mt::MtCircularBuffer>(); }

TEST(MtCircularBuffer, Full) { cb_test::TestFull<nie::mt::MtCircularBuffer>(); }

TEST(MtCircularBuffer, Clear) { cb_test::TestClear<nie::mt::MtCircularBuffer>(); }

TEST(MtCircularBuffer, ResizeGrow) { cb_test::TestResizeGrow<nie::mt::MtCircularBuffer>(); }

TEST(MtCircularBuffer, ResizeShrink) { cb_test::TestResizeShrink<nie::mt::MtCircularBuffer>(); }
