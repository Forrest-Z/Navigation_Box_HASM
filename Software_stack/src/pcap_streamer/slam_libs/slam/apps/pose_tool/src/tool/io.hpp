/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <vector>

#include <gflags/gflags.h>
#include <boost/filesystem.hpp>
#include <nie/formats/ba_graph.hpp>

DECLARE_string(in_paths);
DECLARE_string(out_paths);

bool GetAndCheckInPathsForExtension(std::string const& extension, std::vector<boost::filesystem::path>* paths);
void GetAndCheckInPathsForExtensionOrFatal(std::string const& extension, boost::filesystem::path* path);
boost::filesystem::path GetAndCheckInPathsForExtensionOrFatal(std::string const& extension);

bool InPathExists(std::string const& extension);
void InPathExistsOrFatal(std::string const& extension);
void InPathExistsOrFatal();

// Check if a given directory is available
void OutDirAvailableOrFatal();

// Check if the parent directory of the given files are available
void CheckOutPathsLocationsOrFatal();

void GetAndCheckOutPathsForExtensionOrFatal(std::string const& extension, boost::filesystem::path* path);
boost::filesystem::path GetAndCheckOutPathsForExtensionOrFatal(std::string const& extension);

// TODO(jbr): Very annoying data type, two vectors is better as the pair makes it very hard to reuse things without
// copying
template <typename T>
bool ReadData(std::vector<std::pair<std::string, T>>* collections);
template <typename T>
bool ReadData(std::vector<T>* collections);
template <typename T>
bool ReadData(T* collection);

// Assumes that the collection has a corresponding output filename based on the extensions tied to the collection type.
template <typename T>
void WriteData(T const& collection);

template <typename T>
void CheckAllHeadersAreIdentical(std::vector<T> const& collections);

#include "io.inl"
