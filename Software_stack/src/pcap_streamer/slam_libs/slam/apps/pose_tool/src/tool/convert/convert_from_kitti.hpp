/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

// The kitti files that will be read are the raw data oxts files, not the ground truth pose file. The timestamp file
// belonging to the oxts files should be supplied and the actual oxts files will be searched for the in the data folder
// besides the timestamp file.
void ConvertFromKitti();
