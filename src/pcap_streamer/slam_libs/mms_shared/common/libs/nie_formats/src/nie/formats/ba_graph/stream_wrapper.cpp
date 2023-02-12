/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "stream_wrapper.hpp"

namespace nie {

namespace io {

IStreamWrapper::IStreamWrapper(std::istream* stream) : stream_(stream) {}

OStreamWrapper::OStreamWrapper(std::ostream* stream) : stream_(stream) {}

}  // namespace io

}  // namespace nie