/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
// Standard C++ includes
#include <iostream>

// Ladybug includes
#include <ladybug/ladybug.h>


/**
 * The sole purpose of this application is to retrieve the version of the ladybug library that it is linked against
 *
 * @return The version of the ladybug library
 */
int main()
{
    // Create the ladybug library context
    LadybugContext context;
    LadybugError error = ladybugCreateContext(&context);
    if (error != LADYBUG_OK) { std::cerr << "Error ladybugCreateContext - " << ladybugErrorToString(error); return EXIT_FAILURE; }

    // Request the library version
    unsigned version_major;
    unsigned version_minor;
    unsigned version_type;
    unsigned version_build;
    error = ladybugGetLibraryVersion(context,
                                     &version_major,
                                     &version_minor,
                                     &version_type,
                                     &version_build);
    if (error != LADYBUG_OK) { std::cerr << "Error ladybugGetLibraryVersion - " << ladybugErrorToString(error); return EXIT_FAILURE; }

    // Output version in standard format: Major.Minor.Patch.Build
    std::cout << version_major << "."
              << version_minor << "."
              << version_type  << "."
              << version_build;

    // Cleanup
    error = ladybugDestroyContext(&context);
    if (error != LADYBUG_OK) { std::cerr << "Error ladybugDestroyContext - " << ladybugErrorToString(error); return EXIT_FAILURE; }

    return EXIT_SUCCESS;
}
