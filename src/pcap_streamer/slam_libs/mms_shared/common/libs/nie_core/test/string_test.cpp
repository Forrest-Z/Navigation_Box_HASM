/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/core/string.hpp>

class StringSplitJoin : public ::testing::Test {
protected:
    std::vector<int> const vector_list = {-1, 0, 1, 2, 3};
    std::string const string_list = "-1,0,1,2,3";
    char const list_separator = ',';
};

TEST_F(StringSplitJoin, Join) {
    auto s = nie::Join(vector_list, list_separator);
    EXPECT_EQ(s, string_list);
}
TEST_F(StringSplitJoin, Split) {
    auto v = nie::Split<int>(string_list, list_separator);
    EXPECT_EQ(v, vector_list);
}
TEST_F(StringSplitJoin, SplitIntoStrings) {
    auto v = nie::Split<std::string>(string_list, list_separator);
    ASSERT_EQ(v.size(), vector_list.size());
    for (size_t i = 0; i < v.size(); ++i) {
        ASSERT_EQ(v[i], std::to_string(vector_list[i]));
    }
}

TEST(StringSplit, SplitIntoStringsEmpty) {
    struct Test{
        std::string string_list;
        std::vector<std::string> vector_list;
    };
    std::vector<Test> const tests = {
            {{"1,2,3"}, {"1","2","3"}},
            {{",2,3"},  {"","2","3"}},
            {{"1,,3"},  {"1","","3"}},
            {{"1,2,"},  {"1","2",""}}
    };
    char const list_separator = ',';

    for (Test const& test : tests) {
        auto v = nie::Split<std::string>(test.string_list, list_separator);
        ASSERT_EQ(v.size(), test.vector_list.size());
        for (size_t i = 0; i < v.size(); ++i) {
            ASSERT_EQ(v[i], test.vector_list[i]);
        }
    }
}

class StringTrim : public ::testing::Test {
protected:
    std::string s = " a ";
    std::string t = "  a  a  ";

    // Expected results for Trim, LeftTrim and RightTrim respectively
    std::string const s_c = "a";
    std::string const t_c = "a  a";
    std::string const s_c_l = "a ";
    std::string const t_c_l = "a  a  ";
    std::string const s_c_r = " a";
    std::string const t_c_r = "  a  a";

    // Manipulated variables
    std::string s_e{};
    std::string t_e{};
};

TEST_F(StringTrim, TrimInplace) {
    nie::Trim(&s);
    nie::Trim(&t);

    EXPECT_EQ(s_c, s);
    EXPECT_EQ(t_c, t);
}
TEST_F(StringTrim, Trim) {
    s_e = nie::Trim(s);
    t_e = nie::Trim(t);

    EXPECT_EQ(s_c, s_e);
    EXPECT_EQ(t_c, t_e);
}

TEST_F(StringTrim, TrimLeftInplace) {
    nie::LeftTrim(&s);
    nie::LeftTrim(&t);

    EXPECT_EQ(s_c_l, s);
    EXPECT_EQ(t_c_l, t);
}
TEST_F(StringTrim, TrimLeft) {
    s_e = nie::LeftTrim(s);
    t_e = nie::LeftTrim(t);

    EXPECT_EQ(s_c_l, s_e);
    EXPECT_EQ(t_c_l, t_e);
}

TEST_F(StringTrim, TrimRightInplace) {
    nie::RightTrim(&s);
    nie::RightTrim(&t);

    EXPECT_EQ(s_c_r, s);
    EXPECT_EQ(t_c_r, t);
}
TEST_F(StringTrim, TrimRight) {
    s_e = nie::RightTrim(s);
    t_e = nie::RightTrim(t);

    EXPECT_EQ(s_c_r, s_e);
    EXPECT_EQ(t_c_r, t_e);
}

class StringStrip : public ::testing::Test {
protected:
    std::string const stripped = "abcdefghijklmnopqrstuvwxyz\n";
    std::string const unstripped_single = "a   bcde f g h  i jklmnop q r s t u v w x yz  \n ";
    std::string const unstripped_multi = "a _;, _ bcde f _g h ;, i jklmnop q r; s; t u; v ;___w x yz  \n __";

    char const strip_token_single = ' ';
    std::string const strip_token_multi = " ,_;";
};

TEST_F(StringStrip, StripInplaceSingleChar) {
    std::string s = unstripped_single;

    nie::Strip(&s, strip_token_single);
    EXPECT_EQ(s, stripped);
    nie::Strip(&s, strip_token_single);
    EXPECT_EQ(s, stripped);
}
TEST_F(StringStrip, StripSingleChar) {
    std::string s = nie::Strip(unstripped_single, strip_token_single);
    EXPECT_EQ(s, stripped);
    s = nie::Strip(s, strip_token_single);
    EXPECT_EQ(s, stripped);
}

TEST_F(StringStrip, StripInplaceMultiChar) {
    std::string s = unstripped_multi;

    nie::Strip(&s, strip_token_multi);
    EXPECT_EQ(s, stripped);
    nie::Strip(&s, strip_token_multi);
    EXPECT_EQ(s, stripped);
}
TEST_F(StringStrip, StripMultiChar) {
    std::string s = nie::Strip(unstripped_multi, strip_token_multi);
    EXPECT_EQ(s, stripped);
    s = nie::Strip(s, strip_token_multi);
    EXPECT_EQ(s, stripped);
}

class StringReduce : public ::testing::Test {
protected:
    std::string const reduced = "-1,0,1,2,3";
    std::string const unreduced_single = "-1,0,1,,2,3";
    std::string const unreduced_multi = "-1,0,1,,2,3";

    char const reduce_token_single = ',';
    std::string const reduce_token_multi = ",-";
};

TEST_F(StringReduce, ReduceInplaceSingleChar) {
    std::string s = unreduced_single;

    nie::Reduce(&s, reduce_token_single);
    EXPECT_EQ(s, reduced);
    nie::Reduce(&s, reduce_token_single);
    EXPECT_EQ(s, reduced);
}
TEST_F(StringReduce, ReduceSingleChar) {
    std::string s = nie::Reduce(unreduced_single, reduce_token_single);
    EXPECT_EQ(s, reduced);
    s = nie::Reduce(s, reduce_token_single);
    EXPECT_EQ(s, reduced);
}

TEST_F(StringReduce, ReduceInplaceMultiChar) {
    std::string s = unreduced_multi;

    nie::Reduce(&s, reduce_token_multi);
    EXPECT_EQ(s, reduced);
    nie::Reduce(&s, reduce_token_multi);
    EXPECT_EQ(s, reduced);
}
TEST_F(StringReduce, ReduceMultiChar) {
    std::string s = nie::Reduce(unreduced_multi, reduce_token_multi);
    EXPECT_EQ(s, reduced);
    s = nie::Reduce(s, reduce_token_multi);
    EXPECT_EQ(s, reduced);
}

class StringCase : public ::testing::Test {
protected:
    std::string const lower_case = "abcdefghijklmnopqrstuvwxyz\n0123456789\n";
    std::string const mixed_case = "ABcDEFGhIJklmnopQrsTUvWxYZ\n0123456789\n";
    std::string const upper_case = "ABCDEFGHIJKLMNOPQRSTUVWXYZ\n0123456789\n";
};

TEST_F(StringCase, ToLowerInplace) {
    std::string s = mixed_case;

    nie::ToLower(&s);
    EXPECT_EQ(s, lower_case);
    nie::ToLower(&s);
    EXPECT_EQ(s, lower_case);
}
TEST_F(StringCase, ToLower) {
    std::string s = nie::ToLower(mixed_case);
    EXPECT_EQ(s, lower_case);
    s = nie::ToLower(s);
    EXPECT_EQ(s, lower_case);
}

TEST_F(StringCase, ToUpperInplace) {
    std::string s = mixed_case;

    nie::ToUpper(&s);
    EXPECT_EQ(s, upper_case);
    nie::ToUpper(&s);
    EXPECT_EQ(s, upper_case);
}
TEST_F(StringCase, ToUpper) {
    std::string s = nie::ToUpper(mixed_case);
    EXPECT_EQ(s, upper_case);
    s = nie::ToUpper(s);
    EXPECT_EQ(s, upper_case);
}

TEST(StringStartEnd, StartsWith) {
    std::string s = "-x-";
    std::string m = "---x---";
    EXPECT_TRUE(nie::StartsWith(s, "-"));
    EXPECT_TRUE(nie::StartsWith(m, "-"));

    EXPECT_FALSE(nie::StartsWith(s, "--"));
    EXPECT_TRUE(nie::StartsWith(m, "--"));
}
TEST(StringStartEnd, EndsWith) {
    std::string s = "-x-";
    std::string m = "---x---";
    EXPECT_TRUE(nie::EndsWith(s, "-"));
    EXPECT_TRUE(nie::EndsWith(m, "-"));

    EXPECT_FALSE(nie::EndsWith(s, "--"));
    EXPECT_TRUE(nie::EndsWith(m, "--"));
}
