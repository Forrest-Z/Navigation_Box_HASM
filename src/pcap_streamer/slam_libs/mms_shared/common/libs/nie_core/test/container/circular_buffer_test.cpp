/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/core/container/circular_buffer.hpp>

#include "circular_buffer_test_templates.hpp"

nie::CircularBuffer<size_t> CreateCircularBuffer(size_t const& n = 7) {
    nie::CircularBuffer<size_t> cb{5};
    for (size_t k = 0; k < n; ++k) {
        cb.PushBack(k + 1);
    }
    return cb;
}

TEST(CircularArithmeticSimple, Increment) {
    constexpr size_t size = 5;
    size_t cidx = 0;

    EXPECT_EQ(cidx, 0);
    // This loop can have arbitrary number of iterations and still be correct
    for (size_t i = 0; i < 3; ++i) {
        cidx = nie::detail::CircularArithmeticSimple::Increment(cidx, size);
        EXPECT_EQ(static_cast<size_t>(cidx), 1);
        cidx = nie::detail::CircularArithmeticSimple::Increment(cidx, size);
        EXPECT_EQ(static_cast<size_t>(cidx), 2);
        cidx = nie::detail::CircularArithmeticSimple::Increment(cidx, size);
        EXPECT_EQ(static_cast<size_t>(cidx), 3);
        cidx = nie::detail::CircularArithmeticSimple::Increment(cidx, size);
        EXPECT_EQ(static_cast<size_t>(cidx), 4);
        cidx = nie::detail::CircularArithmeticSimple::Increment(cidx, size);
        EXPECT_EQ(static_cast<size_t>(cidx), 0);
    }
}

TEST(CircularArithmeticSimple, Decrement) {
    constexpr size_t size = 5;
    size_t cidx = 0;

    EXPECT_EQ(cidx, 0);
    // This loop can have arbitrary number of iterations and still be correct
    for (size_t i = 0; i < 3; ++i) {
        cidx = nie::detail::CircularArithmeticSimple::Decrement(cidx, size);
        EXPECT_EQ(static_cast<size_t>(cidx), 4);
        cidx = nie::detail::CircularArithmeticSimple::Decrement(cidx, size);
        EXPECT_EQ(static_cast<size_t>(cidx), 3);
        cidx = nie::detail::CircularArithmeticSimple::Decrement(cidx, size);
        EXPECT_EQ(static_cast<size_t>(cidx), 2);
        cidx = nie::detail::CircularArithmeticSimple::Decrement(cidx, size);
        EXPECT_EQ(static_cast<size_t>(cidx), 1);
        cidx = nie::detail::CircularArithmeticSimple::Decrement(cidx, size);
        EXPECT_EQ(static_cast<size_t>(cidx), 0);
    }
}

TEST(CircularArithmeticSimple, Plus) {
    constexpr size_t size = 5;
    size_t cidx = 0;

    cidx = nie::detail::CircularArithmeticSimple::Plus(cidx, 0, size);
    EXPECT_EQ(cidx, 0);
    cidx = nie::detail::CircularArithmeticSimple::Plus(cidx, 1, size);
    EXPECT_EQ(cidx, 1);
    cidx = nie::detail::CircularArithmeticSimple::Plus(cidx, 2, size);
    EXPECT_EQ(cidx, 3);
    cidx = nie::detail::CircularArithmeticSimple::Plus(cidx, 3, size);
    EXPECT_EQ(cidx, 1);
    cidx = nie::detail::CircularArithmeticSimple::Plus(cidx, 4, size);
    EXPECT_EQ(cidx, 0);
    cidx = nie::detail::CircularArithmeticSimple::Plus(cidx, 5, size);
    EXPECT_EQ(cidx, 0);
    cidx = nie::detail::CircularArithmeticSimple::Plus(cidx, 6, size);
    EXPECT_EQ(cidx, 1);
    cidx = nie::detail::CircularArithmeticSimple::Plus(cidx, 7, size);
    EXPECT_EQ(cidx, 3);
    cidx = nie::detail::CircularArithmeticSimple::Plus(cidx, 8, size);
    EXPECT_EQ(cidx, 1);
    cidx = nie::detail::CircularArithmeticSimple::Plus(cidx, 9, size);
    EXPECT_EQ(cidx, 0);

    cidx = nie::detail::CircularArithmeticSimple::Plus(0, 4, size);
    EXPECT_EQ(cidx, 4);
    cidx = nie::detail::CircularArithmeticSimple::Plus(0, 5, size);
    EXPECT_EQ(cidx, 0);
    cidx = nie::detail::CircularArithmeticSimple::Plus(0, 6, size);
    EXPECT_EQ(cidx, 1);
    cidx = nie::detail::CircularArithmeticSimple::Plus(0, 7, size);
    EXPECT_EQ(cidx, 2);
}

TEST(CircularArithmeticSimple, Minus) {
    constexpr size_t size = 5;
    size_t cidx = 0;

    cidx = nie::detail::CircularArithmeticSimple::Minus(cidx, 0, size);
    EXPECT_EQ(cidx, 0);
    cidx = nie::detail::CircularArithmeticSimple::Minus(cidx, 1, size);
    EXPECT_EQ(cidx, 4);
    cidx = nie::detail::CircularArithmeticSimple::Minus(cidx, 2, size);
    EXPECT_EQ(cidx, 2);
    cidx = nie::detail::CircularArithmeticSimple::Minus(cidx, 3, size);
    EXPECT_EQ(cidx, 4);
    cidx = nie::detail::CircularArithmeticSimple::Minus(cidx, 4, size);
    EXPECT_EQ(cidx, 0);
    cidx = nie::detail::CircularArithmeticSimple::Minus(cidx, 5, size);
    EXPECT_EQ(cidx, 0);
    cidx = nie::detail::CircularArithmeticSimple::Minus(cidx, 6, size);
    EXPECT_EQ(cidx, 4);

    cidx = nie::detail::CircularArithmeticSimple::Minus(0, 4, size);
    EXPECT_EQ(cidx, 1);
    cidx = nie::detail::CircularArithmeticSimple::Minus(0, 5, size);
    EXPECT_EQ(cidx, 0);
    cidx = nie::detail::CircularArithmeticSimple::Minus(0, 6, size);
    EXPECT_EQ(cidx, 4);
    cidx = nie::detail::CircularArithmeticSimple::Minus(0, 7, size);
    EXPECT_EQ(cidx, 3);
}

TEST(SaturatingCounter, Basic) {
    EXPECT_EQ(nie::detail::SaturatingCounter::Increment(0, 10), 1);
    EXPECT_EQ(nie::detail::SaturatingCounter::Increment(1, 10), 2);
    EXPECT_EQ(nie::detail::SaturatingCounter::Increment(2, 10), 3);
    EXPECT_EQ(nie::detail::SaturatingCounter::Increment(3, 10), 4);

    size_t i = 0;
    i = nie::detail::SaturatingCounter::Increment(i, 10);
    EXPECT_EQ(i, 1);
    i = nie::detail::SaturatingCounter::Increment(i, 10);
    EXPECT_EQ(i, 2);
    i = nie::detail::SaturatingCounter::Increment(i, 10);
    EXPECT_EQ(i, 3);

    // This loop can have arbitrary number of iterations and still be correct
    for (size_t k = 1; k < 99; ++k) {
        for (size_t i = 0; i < k; ++i) {
            ASSERT_LE(nie::detail::SaturatingCounter::Increment(i, k), k);
        }
    }
}

TEST(CircularBuffer, Size) { cb_test::TestSize<nie::CircularBuffer>(); }

TEST(CircularBuffer, Count) { cb_test::TestCount<nie::CircularBuffer>(); }

TEST(CircularBuffer, PopFront) { cb_test::TestPopFront<nie::CircularBuffer>(); }

TEST(CircularBuffer, PopBack) { cb_test::TestPopBack<nie::CircularBuffer>(); }

TEST(CircularBuffer, ArraySubscriptOperatorHalfFullBuffer) {
    nie::CircularBuffer<size_t> cb = CreateCircularBuffer(3);
    // cb looks like {1,2,3} logically

    EXPECT_EQ(cb[-4], 3);

    EXPECT_EQ(cb[-3], 1);
    EXPECT_EQ(cb[-2], 2);
    EXPECT_EQ(cb[-1], 3);

    EXPECT_EQ(cb[0], 1);
    EXPECT_EQ(cb[1], 2);
    EXPECT_EQ(cb[2], 3);

    EXPECT_EQ(cb[3], 1);
    EXPECT_EQ(cb[4], 2);
    EXPECT_EQ(cb[5], 3);

    EXPECT_EQ(cb[6], 1);
}

TEST(CircularBuffer, ArraySubscriptOperatorFullBuffer) {
    nie::CircularBuffer<size_t> cb = CreateCircularBuffer(5);
    // cb looks like {1,2,3,4,5} logically

    EXPECT_EQ(cb[-6], 5);

    EXPECT_EQ(cb[-5], 1);
    EXPECT_EQ(cb[-4], 2);
    EXPECT_EQ(cb[-3], 3);
    EXPECT_EQ(cb[-2], 4);
    EXPECT_EQ(cb[-1], 5);

    EXPECT_EQ(cb[0], 1);
    EXPECT_EQ(cb[1], 2);
    EXPECT_EQ(cb[2], 3);
    EXPECT_EQ(cb[3], 4);
    EXPECT_EQ(cb[4], 5);

    EXPECT_EQ(cb[5], 1);
    EXPECT_EQ(cb[6], 2);
    EXPECT_EQ(cb[7], 3);
    EXPECT_EQ(cb[8], 4);
    EXPECT_EQ(cb[9], 5);

    EXPECT_EQ(cb[10], 1);
}

TEST(CircularBuffer, ArraySubscriptOperatorReducedFullBuffer) {
    nie::CircularBuffer<size_t> cb = CreateCircularBuffer(5);
    {
        size_t x;
        cb.PopFront(&x);
        EXPECT_EQ(x, 1);
        cb.PopFront(&x);
        EXPECT_EQ(x, 2);
        cb.PopBack(&x);
        EXPECT_EQ(x, 5);
    }
    // cb looks like {3,4} logically

    EXPECT_EQ(cb[-3], 4);

    EXPECT_EQ(cb[-2], 3);
    EXPECT_EQ(cb[-1], 4);

    EXPECT_EQ(cb[0], 3);
    EXPECT_EQ(cb[1], 4);

    EXPECT_EQ(cb[2], 3);
    EXPECT_EQ(cb[3], 4);

    EXPECT_EQ(cb[4], 3);
}

TEST(CircularBuffer, ArraySubscriptOperatorOverFullBuffer) {
    nie::CircularBuffer<size_t> cb = CreateCircularBuffer();
    // cb looks like {3,4,5,6,7} logically

    EXPECT_EQ(cb[-6], 7);

    EXPECT_EQ(cb[-5], 3);
    EXPECT_EQ(cb[-4], 4);
    EXPECT_EQ(cb[-3], 5);
    EXPECT_EQ(cb[-2], 6);
    EXPECT_EQ(cb[-1], 7);

    EXPECT_EQ(cb[0], 3);
    EXPECT_EQ(cb[1], 4);
    EXPECT_EQ(cb[2], 5);
    EXPECT_EQ(cb[3], 6);
    EXPECT_EQ(cb[4], 7);

    EXPECT_EQ(cb[5], 3);
    EXPECT_EQ(cb[6], 4);
    EXPECT_EQ(cb[7], 5);
    EXPECT_EQ(cb[8], 6);
    EXPECT_EQ(cb[9], 7);

    EXPECT_EQ(cb[10], 3);
}

TEST(CircularBuffer, Back) { cb_test::TestBack<nie::CircularBuffer>(); }

TEST(CircularBuffer, Front) { cb_test::TestFront<nie::CircularBuffer>(); }

TEST(CircularBuffer, Begin) { EXPECT_EQ(*CreateCircularBuffer().begin(), 3); }

TEST(CircularBuffer, EndMinus1) { EXPECT_EQ(*(CreateCircularBuffer().end() - 1), 7); }

TEST(CircularBuffer, End) { EXPECT_EQ(*CreateCircularBuffer().end(), 3); }

// Test for non-const-ness of iterators, after example at https://stackoverflow.com/a/29830322
TEST(CircularBuffer, NonConstnessIterator) {
    using iter = nie::CircularBuffer<size_t>::iterator;
    using pointer = std::iterator_traits<iter>::pointer;
    using iterator_type = std::remove_pointer<pointer>::type;
    EXPECT_FALSE(std::is_const<iterator_type>::value);
}

// Test for const-ness of iterators, after example at https://stackoverflow.com/a/29830322
TEST(CircularBuffer, ConstnessIterator) {
    using iter = nie::CircularBuffer<size_t>::const_iterator;
    using pointer = std::iterator_traits<iter>::pointer;
    using iterator_type = std::remove_pointer<pointer>::type;
    EXPECT_TRUE(std::is_const<iterator_type>::value);
}

TEST(CircularBuffer, NonConstToConstIterator) {
    nie::CircularBuffer<size_t> cb = CreateCircularBuffer();
    nie::CircularBuffer<size_t>::const_iterator iter1(cb.begin());
    nie::CircularBuffer<size_t>::const_iterator iter2 = cb.begin();
    iter2 = cb.end();
}

TEST(CircularBuffer, IteratorLoop) {
    nie::CircularBuffer<size_t> cb = CreateCircularBuffer();
    // cb looks like {3,4,5,6,7} logically

    std::size_t expected_value = 2;
    for (nie::CircularBuffer<size_t>::iterator iter = cb.begin(); iter != cb.end(); ++iter) {
        EXPECT_EQ(*iter, ++expected_value);
    }
}

TEST(CircularBuffer, RangeBasedLoop) {
    nie::CircularBuffer<size_t> cb = CreateCircularBuffer();
    // cb looks like {3,4,5,6,7} logically

    std::size_t expected_value = 2;
    for (auto const& i : cb) {
        EXPECT_EQ(i, ++expected_value);
    }
}

TEST(CircularBuffer, Empty) { cb_test::TestEmpty<nie::CircularBuffer>(); }

TEST(CircularBuffer, Full) { cb_test::TestFull<nie::CircularBuffer>(); }

TEST(CircularBuffer, Clear) { cb_test::TestClear<nie::CircularBuffer>(); }

TEST(CircularBuffer, Reorder) {
    nie::CircularBuffer<size_t> circular_buffer{5};

    // This loop can have a arbitrary number of iterations and still be correct
    for (size_t k = 0; k < 11; ++k) {
        // push some zeros
        for (size_t i = 0; i < k; ++i) {
            circular_buffer.PushBack(0);
        }
        // fill buffer with known set of numbers which will be "shifted"
        for (size_t i = 0; i < circular_buffer.Size(); ++i) {
            circular_buffer.PushBack(i);
        }

        // Reorder the internet buffer so that it is no longer "shifted"
        circular_buffer.Reorder();

        // Check size of resulting vector
        auto vec = circular_buffer.Data();
        ASSERT_EQ(vec.size(), 5);

        // Check if the elements are in the correct order
        vec = circular_buffer.Data();
        for (size_t i = 0; i < circular_buffer.Size(); ++i) {
            size_t x;
            circular_buffer.PopFront(&x);
            EXPECT_EQ(x, vec[i]);
        }
    }

    for (size_t i = 0; i < circular_buffer.Size(); ++i) {
        circular_buffer.PushBack(0);
    }
    circular_buffer.Clear();

    auto& vec = circular_buffer.Data();
    ASSERT_EQ(vec[0], 0);
    ASSERT_EQ(vec[1], 0);
    ASSERT_EQ(vec[2], 0);
    ASSERT_EQ(vec[3], 0);
    ASSERT_EQ(vec[4], 0);

    circular_buffer.PushBack(0);
    circular_buffer.PushBack(11);
    circular_buffer.PushBack(22);
    circular_buffer.PushBack(33);

    size_t x;
    circular_buffer.PopFront(&x);

    circular_buffer.Reorder();

    vec = circular_buffer.Data();
    std::cout << std::endl;
    ASSERT_EQ(vec[0], 11);
    ASSERT_EQ(vec[1], 22);
    ASSERT_EQ(vec[2], 33);
    ASSERT_EQ(vec[3], 0);
    ASSERT_EQ(vec[4], 0);
}

TEST(CircularBuffer, ResizeGrow) { cb_test::TestResizeGrow<nie::CircularBuffer>(); }

TEST(CircularBuffer, ResizeShrink) { cb_test::TestResizeShrink<nie::CircularBuffer>(); }
