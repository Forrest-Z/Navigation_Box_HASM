/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
// Standard C++ includes
#include <iostream>

// Libpcap includes
#include <pcap/pcap.h>


/**
 * The sole purpose of this application is to retrieve the version of the pcap library that it is linked against
 *
 * @return The version of the pcap library
 */
int main()
{
    // Obtain library version
    std::string version = pcap_lib_version();

    // Output version in standard format: Major[.Minor[.Patch[.Build]]]; Strip the non-numeric prefix off
    std::cout << version.substr(version.find_first_of("0123456789"));

    return EXIT_SUCCESS;
}
