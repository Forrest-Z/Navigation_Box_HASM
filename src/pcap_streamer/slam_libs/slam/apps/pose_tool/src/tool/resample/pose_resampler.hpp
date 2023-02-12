#pragma once

#include <nie/formats/ba_graph.hpp>

nie::io::PoseCollection ResampleDistance(
        nie::io::PoseCollection const& c, double const interval, double const max_delta);
