/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <gtest/gtest.h>

///
/// Tests for common interface of CircularBuffer and CircularBufferSync
///
namespace cb_test {

template <template <class...> class C>
void TestSize() {
    for (size_t k = 0; k < 33; ++k) {
        C<int> circular_buffer{k};

        ASSERT_EQ(circular_buffer.Size(), k);
    }
}

template <template <class...> class C>
void TestCount() {
    C<int> circular_buffer{5};

    EXPECT_EQ(circular_buffer.Count(), 0);
    circular_buffer.PushBack(0);
    EXPECT_EQ(circular_buffer.Count(), 1);
    circular_buffer.PushBack(0);
    EXPECT_EQ(circular_buffer.Count(), 2);
    circular_buffer.PushBack(0);
    EXPECT_EQ(circular_buffer.Count(), 3);
    circular_buffer.PushBack(0);
    EXPECT_EQ(circular_buffer.Count(), 4);
    circular_buffer.PushBack(0);
    EXPECT_EQ(circular_buffer.Count(), 5);
    for (size_t i = 0; i < 6; ++i) {
        circular_buffer.PushBack(0);
        EXPECT_EQ(circular_buffer.Count(), 5);
    }

    for (size_t i = circular_buffer.Size(); i > 0; --i) {
        EXPECT_EQ(circular_buffer.Count(), i);
        int x;
        circular_buffer.PopFront(&x);
    }
    EXPECT_EQ(circular_buffer.Count(), 0);
}

template <template <class...> class C>
void TestPopFront() {
    C<size_t> circular_buffer{5};

    for (size_t i = 0; i < circular_buffer.Size(); ++i) {
        circular_buffer.PushBack(i);
    }

    for (size_t i = 0; i < circular_buffer.Size(); ++i) {
        size_t x;
        circular_buffer.PopFront(&x);
        ASSERT_EQ(x, i);
    }

    for (size_t i = 0; i < 1234; ++i) {
        circular_buffer.PushBack(i);
    }

    size_t t;
    size_t expect = 1234 - circular_buffer.Size();
    while (circular_buffer.PopFront(&t)) {
        ASSERT_EQ(t, expect);
        ++expect;
    }
}

template <template <class...> class C>
void TestPopBack() {
    C<size_t> circular_buffer{5};

    for (size_t i = 0; i < 5; ++i) {
        circular_buffer.PushBack(i);
    }

    for (int i = 4; i >= 0; --i) {
        size_t x;
        circular_buffer.PopBack(&x);
        ASSERT_EQ(x, static_cast<size_t>(i));
    }

    for (size_t i = 0; i < 1234; ++i) {
        circular_buffer.PushBack(i);
    }

    size_t t;
    size_t expect = 1234 - 1;
    while (circular_buffer.PopBack(&t)) {
        ASSERT_EQ(t, expect);
        --expect;
    }
}

template <template <class...> class C>
void TestBack() {
    C<size_t> circular_buffer{5};

    // This loop can have a arbitrary number of iterations and still be correct
    for (size_t i = 0; i < 33; ++i) {
        circular_buffer.PushBack(i);
        ASSERT_EQ(circular_buffer.Back(), i);
    }
}

template <template <class...> class C>
void TestFront() {
    C<size_t> circular_buffer{5};

    // This loop can have an arbitrary number of iterations and still be correct
    for (size_t i = 0; i < 33; ++i) {
        circular_buffer.PushBack(i);
        if (i < circular_buffer.Size()) {
            ASSERT_EQ(circular_buffer.Front(), 0);
        } else {
            ASSERT_EQ(circular_buffer.Front(), i - circular_buffer.Size() + 1);
        }
    }
}

template <template <class...> class C>
void TestEmpty() {
    C<size_t> circular_buffer{5};

    ASSERT_TRUE(circular_buffer.Empty());

    circular_buffer.PushBack(0);
    ASSERT_FALSE(circular_buffer.Empty());

    size_t x;

    circular_buffer.PopFront(&x);
    ASSERT_TRUE(circular_buffer.Empty());

    // This loop can have a arbitrary number of iterations and still be correct
    for (size_t i = 0; i < 33; ++i) {
        circular_buffer.PushBack(i);
        ASSERT_FALSE(circular_buffer.Empty());
    }

    for (size_t i = 0; i < circular_buffer.Size() - 1; ++i) {
        circular_buffer.PopFront(&x);
        ASSERT_FALSE(circular_buffer.Empty());
    }
    circular_buffer.PopFront(&x);
    ASSERT_TRUE(circular_buffer.Empty());
}

template <template <class...> class C>
void TestFull() {
    C<size_t> circular_buffer{5};

    ASSERT_FALSE(circular_buffer.Full());

    for (size_t i = 0; i < circular_buffer.Size() - 1; ++i) {
        circular_buffer.PushBack(i);
        ASSERT_FALSE(circular_buffer.Full());
    }

    // This loop can have a arbitrary number of iterations and still be correct
    for (size_t i = 0; i < 33; ++i) {
        circular_buffer.PushBack(i);
        ASSERT_TRUE(circular_buffer.Full());
    }

    for (size_t i = 0; i < circular_buffer.Size(); ++i) {
        size_t x;
        circular_buffer.PopFront(&x);
        ASSERT_FALSE(circular_buffer.Full());
    }
}

template <template <class...> class C>
void TestClear() {
    nie::CircularBuffer<size_t> circular_buffer{5};

    // This loop can have a arbitrary number of iterations and still be correct
    for (size_t k = 0; k < 33; ++k) {
        // This loop can have a arbitrary number of iterations and still be correct
        for (size_t i = 0; i < 13; ++i) {
            circular_buffer.PushBack(i);
        }
        circular_buffer.Clear();
        ASSERT_TRUE(circular_buffer.Empty());
    }
}

template <template <class...> class C>
void TestResizeGrow() {
    for (size_t size = 1; size < 33; ++size) {
        for (size_t grow = 0; grow < 21; ++grow) {
            C<size_t> circular_buffer{size};

            // Fill buffer with ascending values
            for (size_t i = 0; i < size; ++i) {
                circular_buffer.PushBack(i);
            }

            ASSERT_EQ(circular_buffer.Count(), size);
            circular_buffer.Resize(size + grow);
            ASSERT_EQ(circular_buffer.Size(), size + grow);
            ASSERT_EQ(circular_buffer.Count(), size);

            for (size_t i = 0; i < size; ++i) {
                size_t x;
                circular_buffer.PopFront(&x);
                ASSERT_EQ(x, i);
            }
        }
    }
}

template <template <class...> class C>
void TestResizeShrink() {
    for (size_t size = 1; size < 33; ++size) {
        for (size_t shrink = 0; shrink < size; ++shrink) {
            C<size_t> circular_buffer{size};

            // Fill buffer with ascending values
            for (size_t i = 0; i < size; ++i) {
                circular_buffer.PushBack(i);
            }

            ASSERT_EQ(circular_buffer.Count(), size);
            circular_buffer.Resize(size - shrink);
            ASSERT_EQ(circular_buffer.Size(), size - shrink);
            ASSERT_EQ(circular_buffer.Count(), size - shrink);

            for (size_t i = shrink; i < size; ++i) {
                size_t x;
                circular_buffer.PopFront(&x);
                ASSERT_EQ(x, i);
            }
        }
    }
}

}  // namespace cb_test
