/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#define TRANSLATION_NEAR(TYPE, v1, v2, precision) \
    do {                                          \
        TYPE##_NEAR(v1[0], v2[0], precision);     \
        TYPE##_NEAR(v1[1], v2[1], precision);     \
        TYPE##_NEAR(v1[2], v2[2], precision);     \
    } while (0)

#define ASSERT_TRANSLATION_NEAR(v1, v2, precision) TRANSLATION_NEAR(ASSERT, v1, v2, precision)
#define EXPECT_TRANSLATION_NEAR(v1, v2, precision) TRANSLATION_NEAR(EXPECT, v1, v2, precision)

#define QUATERNION_NEAR(TYPE, q1, q2, precision)               \
    do {                                                       \
        TYPE##_NEAR(q1.w(), q2.w(), precision);                \
        TRANSLATION_NEAR(TYPE, q1.vec(), q2.vec(), precision); \
    } while (0)

#define ASSERT_QUATERNION_NEAR(q1, q2, precision) QUATERNION_NEAR(ASSERT, q1, q2, precision)
#define EXPECT_QUATERNION_NEAR(q1, q2, precision) QUATERNION_NEAR(EXPECT, q1, q2, precision)

#define ISOMETRY_NEAR(TYPE, isom1, isom2, precision)                                 \
    do {                                                                             \
        QUATERNION_NEAR(TYPE, isom1.rotation(), isom2.rotation(), precision);        \
        TRANSLATION_NEAR(TYPE, isom1.translation(), isom2.translation(), precision); \
    } while (0)

#define ASSERT_ISOMETRY_NEAR(isom1, isom2, precision) ISOMETRY_NEAR(ASSERT, isom1, isom2, precision)
#define EXPECT_ISOMETRY_NEAR(isom1, isom2, precision) ISOMETRY_NEAR(EXPECT, isom1, isom2, precision)
