/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/core/algorithm.hpp>

TEST(Algorithm, CopyIfIndicesEmpty) {
    std::vector<int> input{0, 1, 2, 3, 4};
    std::vector<int> indices;
    std::vector<int> output;
    auto it = nie::CopyIf(input.begin(), input.end(), output.begin(), indices.begin(), indices.end());

    EXPECT_EQ(output.size(), indices.size());
    EXPECT_EQ(it, output.end());
}

TEST(Algorithm, CopyIfIndicesTooLong) {
    std::vector<int> input{0, 1, 2, 3, 4};
    std::vector<int> indices{1, 2, 3};
    std::size_t amount = 2;
    std::vector<int> output(amount);
    auto it = nie::CopyIf(input.begin(), input.end(), output.begin(), indices.begin(), indices.begin() + amount);

    EXPECT_EQ(output.size(), amount);
    EXPECT_EQ(it, output.end());
    for (std::size_t index = 0; index < output.size(); ++index) {
        EXPECT_EQ(output[index], indices[index]);
    }
}

TEST(Algorithm, CopyIfIndices) {
    std::vector<int> input{0, 1, 2, 3, 4, 5};
    std::vector<int> indices{1, 2, 4};
    std::vector<int> output(indices.size());
    auto it = nie::CopyIf(input.begin(), input.end(), output.begin(), indices.begin(), indices.end());

    EXPECT_EQ(output.size(), indices.size());
    EXPECT_EQ(it, output.end());
    for (std::size_t index = 0; index < output.size(); ++index) {
        EXPECT_EQ(output[index], indices[index]);
    }
}

TEST(Algorithm, CopyIfIndicesAll) {
    std::vector<int> input{0, 1, 2, 3, 4};
    std::vector<int> indices{0, 1, 2, 3, 4};
    std::vector<int> output(indices.size());
    auto it = nie::CopyIf(input.begin(), input.end(), output.begin(), indices.begin(), indices.end());

    EXPECT_EQ(output.size(), indices.size());
    EXPECT_EQ(it, output.end());
    for (std::size_t index = 0; index < output.size(); ++index) {
        EXPECT_EQ(output[index], indices[index]);
    }
}

TEST(Algorithm, RemoveIfIndicesTooLong) {
    std::vector<int> v{0, 1, 2, 3, 4};
    int indices[] = {1, 2};
    auto it = nie::RemoveIf(indices, indices + 1, &v);
    EXPECT_EQ(v[0], 0);
    EXPECT_EQ(v[1], 2);
    EXPECT_EQ(v[2], 3);
    EXPECT_EQ(it, v.begin() + 4);
}

TEST(Algorithm, RemoveIfIndices1) {
    std::vector<int> v{0, 1, 2, 3, 4};
    int indices[] = {1};
    auto it = nie::RemoveIf(indices, indices + 1, &v);
    EXPECT_EQ(v[0], 0);
    EXPECT_EQ(v[1], 2);
    EXPECT_EQ(v[2], 3);
    EXPECT_EQ(it, v.begin() + 4);
}

TEST(Algorithm, RemoveIfIndices2) {
    std::vector<int> v{0, 1, 2, 3, 4};
    int indices[] = {1, 3};
    auto it = nie::RemoveIf(indices, indices + 2, &v);
    EXPECT_EQ(v[0], 0);
    EXPECT_EQ(v[1], 2);
    EXPECT_EQ(v[2], 4);
    EXPECT_EQ(it, v.begin() + 3);
}

TEST(Algorithm, RemoveIfIndices4) {
    std::vector<int> v{0, 1, 2, 3, 4};
    int indices[] = {0, 1, 3, 4};
    auto it = nie::RemoveIf(indices, indices + 4, &v);
    EXPECT_EQ(v[0], 2);
    EXPECT_EQ(it, v.begin() + 1);
}

TEST(Algorithm, RemoveIfIndices5) {
    std::vector<int> v{0, 1, 2, 3, 4};
    int indices[] = {0, 1, 2, 3, 4};
    auto it = nie::RemoveIf(indices, indices + 5, &v);
    EXPECT_EQ(it, v.begin());
}

TEST(Algorithm, RemoveIfVectorBoolTooShort1) {
    std::vector<int> v{0, 1, 2, 3, 4};
    std::vector<bool> f(3, false);
    auto const it = nie::RemoveIf(f, &v);
    EXPECT_EQ(v[0], 0);
    EXPECT_EQ(v[1], 1);
    EXPECT_EQ(v[2], 2);
    EXPECT_EQ(v[3], 3);
    EXPECT_EQ(v[4], 4);
    EXPECT_EQ(it, v.end());
}

TEST(Algorithm, RemoveIfVectorBoolTooShort2) {
    std::vector<int> v{0, 1, 2, 3, 4};
    std::vector<bool> f(3, true);
    auto const it = nie::RemoveIf(f, &v);
    EXPECT_EQ(it, v.begin() + 2);
}

TEST(Algorithm, RemoveIfVectorBoolTooLong) {
    std::vector<int> v{0, 1, 2, 3, 4};
    std::vector<bool> f(6, false);
    auto const it = nie::RemoveIf(f, &v);
    EXPECT_EQ(v[0], 0);
    EXPECT_EQ(v[1], 1);
    EXPECT_EQ(v[2], 2);
    EXPECT_EQ(v[3], 3);
    EXPECT_EQ(v[4], 4);
    EXPECT_EQ(it, v.end());
}

TEST(Algorithm, RemoveIfVectorBool1) {
    std::vector<int> v{0, 1, 2, 3, 4};
    std::vector<bool> f = {false, true, false, true, false};
    auto it = nie::RemoveIf(f, &v);
    EXPECT_EQ(v[0], 0);
    EXPECT_EQ(v[1], 2);
    EXPECT_EQ(v[2], 4);
    EXPECT_EQ(it, v.begin() + 3);
}

TEST(Algorithm, RemoveIfVectorBool2) {
    std::vector<int> v{0, 1, 2, 3, 4};
    std::vector<bool> f = {true, false, false, false, true};
    auto it = nie::RemoveIf(f, &v);
    EXPECT_EQ(v[0], 1);
    EXPECT_EQ(v[1], 2);
    EXPECT_EQ(v[2], 3);
    EXPECT_EQ(it, v.begin() + 3);
}

TEST(Algorithm, ForEachTupleEmpty) {
    std::tuple<> t;

    std::size_t counter = 0;
    nie::ForEach(t, [&counter](auto const&) { ++counter; });
    EXPECT_EQ(counter, 0);
    EXPECT_EQ(counter, std::tuple_size<decltype(t)>::value);
}

TEST(Algorithm, ForEachTuple) {
    struct X {
        int number;
        std::string string;
    };
    std::tuple<std::size_t, std::string, X> t = {0, "test", {-1, "string"}};

    std::size_t counter = 0;
    nie::ForEach(t, [&counter](auto const&) { ++counter; });
    EXPECT_EQ(counter, 3);
    EXPECT_EQ(counter, std::tuple_size<decltype(t)>::value);
}
