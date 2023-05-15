/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include <nie/cv/subdiv_2d.hpp>

class Subdiv2DTest : public ::testing::Test {
protected:
    Subdiv2DTest() : triangulation_(cv::Rect(0, 0, 10, 10)) {}

    void DoTest(std::vector<cv::Point> const& features, std::vector<std::pair<int, int>> const& expectations) {
        std::map<int, int> vertex_ids;
        for (std::size_t index = 0; index < features.size(); ++index) {
            int vertex_id = triangulation_.insert(features[index]);
            vertex_ids[vertex_id] = index;
        }

        // Process triangulation
        std::vector<std::pair<int, int>> edges;
        triangulation_.getConnectionList(&edges);

        // Filter out the edges returned that are not connected to supplied points
        edges.erase(
            remove_if(
                edges.begin(),
                edges.end(),
                [&vertex_ids](std::pair<int, int> const& e) -> bool {
                    return (
                        vertex_ids.find(e.first) == vertex_ids.end() or vertex_ids.find(e.second) == vertex_ids.end());
                }),
            edges.end());
        // Translate the id's
        for (std::pair<int, int>& edge : edges) {
            edge.first = vertex_ids[edge.first];
            edge.second = vertex_ids[edge.second];
        }

        // Check result
        ASSERT_EQ(edges.size(), expectations.size()) << "Mismatch in the number of triangulation connections.";

        sort(edges.begin(), edges.end());
        // Copy expectations to be able to sort them
        auto expectations_sorted = expectations;
        sort(expectations_sorted.begin(), expectations_sorted.end());
        for (std::size_t i = 0; i < edges.size(); ++i) {
            ASSERT_EQ(edges[i].first, expectations_sorted[i].first);
            ASSERT_EQ(edges[i].second, expectations_sorted[i].second);
        }
    }

private:
    nie::Subdiv2D triangulation_;
};

TEST_F(Subdiv2DTest, GenericTest) {
    /*
     * . . . . . 0 . . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . 1 . . . . . . . 2
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . . . . 3 . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . . 4 . . . . .
     */
    std::vector<cv::Point> features;
    features.emplace_back(5, 1);
    features.emplace_back(1, 3);
    features.emplace_back(9, 3);
    features.emplace_back(6, 6);
    features.emplace_back(4, 9);

    std::vector<std::pair<int, int>> expectations = {{0, 1}, {0, 2}, {0, 3}, {1, 3}, {1, 4}, {2, 3}, {2, 4}, {3, 4}};

    DoTest(features, expectations);
}

/*
 * Exactly the same test as GenericTest, but with the addition that one point is
 * added twice, which has no effect on the triangulation. Although the id
 * translation mapping needs to be handled properly.
 */
TEST_F(Subdiv2DTest, DuplicatePointTest) {
    /*
     * . . . . . 0 . . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . 1 . . . . . . . 2/5
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . . . . 3 . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . . 4 . . . . .
     */
    std::vector<cv::Point> features;
    features.emplace_back(5, 1);
    features.emplace_back(1, 3);
    features.emplace_back(9, 3);
    features.emplace_back(6, 6);
    features.emplace_back(4, 9);
    features.emplace_back(9, 3);  // duplicate of 2

    // Because of the duplicate point and the test using a map to translate the
    // triangulation vertex ids to the original feature ids, the last id will be
    // kept in case of a duplicate. So feature id 2 becomes 5 compared to the
    // GenericTest.
    std::vector<std::pair<int, int>> expectations = {{0, 1}, {0, 5}, {0, 3}, {1, 3}, {1, 4}, {5, 3}, {5, 4}, {3, 4}};

    DoTest(features, expectations);
}
