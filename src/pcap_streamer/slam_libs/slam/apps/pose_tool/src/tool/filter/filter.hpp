/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

// The filter option of the pose_tool will be filter the given pose file based on the pose id's supplied in the csv
// file. All pose and related records that reference a pose id in the csv file, will be not be copied from the input to
// the output.
void Filter();
