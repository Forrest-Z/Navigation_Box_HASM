/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

/// Temporary conversion function that allows quick and dirty handling of Tue g2o format
//
// The g2o file contains the complete bundle adjustment problem. Almost all poses represent an image, however only for
// keyframes/images the pose was determined and for the other images the pose was interpolated between the key poses.
// The csv file contains the keyframe ids with the corresponding left image path.
//
// This function will:
//  * The g2o file will be converted to a pose, objt and kpnt file.
//  * The csv will be used to create an iref file with records for both left and right frames (note that the latter one
//    is searched for in this app).
//  * Then the pose, objt and kpnt files are filtered to only contain the keyframe poses (in the iref file).
//  * Finally, from the image file name the timestamps are parsed and added to the relevant poses.
void ConvertFromG2oTue();
