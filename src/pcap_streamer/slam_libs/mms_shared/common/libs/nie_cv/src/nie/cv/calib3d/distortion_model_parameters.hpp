/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CALIB3D_DISTORTION_MODEL_PARAMETERS_HPP
#define NIE_CV_CALIB3D_DISTORTION_MODEL_PARAMETERS_HPP

#include <string>

#include <nie/core/glog.hpp>

namespace nie {

enum class ParameterFocalLength { SINGLE_F, XY_F };

enum class ParameterSkew { SKEW, NO_SKEW };

enum class ParameterDistortionRadial { K3, K6, NO_DISTORTION };

enum class ParameterDistortionTangential { P2, NO_DISTORTION };

enum class ParameterDistortionThinPrism { S4, NO_DISTORTION };

struct DistortionModelParameters {
    ParameterFocalLength focal_length;
    ParameterSkew skew;
    ParameterDistortionRadial distortion_radial;
    ParameterDistortionTangential distortion_tangential;
    ParameterDistortionThinPrism distortion_thin_prism;
};

// TODO: [EDD] Most of the code in this file is boilerplate to convert enums from / to std::string representation
//             When using C++17 we may consider using the magic_enum header-only library (MIT license), which provides
//             static reflection for enums based on compiler specific hacks (Clang >= 5, MSVC >= 15.3, and GCC >= 9)
//             See: https://github.com/Neargye/magic_enum

inline std::string ParameterFocalLengthToString(nie::ParameterFocalLength focal_length) {
    switch (focal_length) {
        case nie::ParameterFocalLength::SINGLE_F:
            return "SINGLE_F";
        case nie::ParameterFocalLength::XY_F:
            return "XY_F";
        default:
            LOG(FATAL) << "Unknown enum nie::ParameterFocalLength";
    }
}

inline std::string ParameterSkewToString(nie::ParameterSkew skew) {
    switch (skew) {
        case nie::ParameterSkew::SKEW:
            return "SKEW";
        case nie::ParameterSkew::NO_SKEW:
            return "NO_SKEW";
        default:
            LOG(FATAL) << "Unknown enum nie::ParameterSkew";
    }
}

inline std::string ParameterDistortionRadialToString(nie::ParameterDistortionRadial distortion_radial) {
    switch (distortion_radial) {
        case nie::ParameterDistortionRadial::K3:
            return "K3";
        case nie::ParameterDistortionRadial::K6:
            return "K6";
        case nie::ParameterDistortionRadial::NO_DISTORTION:
            return "NO_DISTORTION";
        default:
            LOG(FATAL) << "Unknown enum nie::ParameterDistortionRadial";
    }
}

inline std::string ParameterDistortionTangentialToString(nie::ParameterDistortionTangential distortion_tangential) {
    switch (distortion_tangential) {
        case nie::ParameterDistortionTangential::P2:
            return "P2";
        case nie::ParameterDistortionTangential::NO_DISTORTION:
            return "NO_DISTORTION";
        default:
            LOG(FATAL) << "Unknown enum nie::ParameterDistortionTangential";
    }
}

inline std::string ParameterDistortionThinPrismToString(nie::ParameterDistortionThinPrism distortion_thin_prism) {
    switch (distortion_thin_prism) {
        case nie::ParameterDistortionThinPrism::S4:
            return "S4";
        case nie::ParameterDistortionThinPrism::NO_DISTORTION:
            return "NO_DISTORTION";
        default:
            LOG(FATAL) << "Unknown enum nie::ParameterDistortionThinPrism";
    }
}

inline nie::ParameterFocalLength ParameterFocalLengthToEnum(std::string const& focal_length) {
    if (focal_length == "SINGLE_F") {
        return nie::ParameterFocalLength::SINGLE_F;
    } else if (focal_length == "XY_F") {
        return nie::ParameterFocalLength::XY_F;
    } else {
        LOG(FATAL) << "Unable to convert '" << focal_length << "' to nie::ParameterFocalLength enum";
    }
}

inline nie::ParameterSkew ParameterSkewToEnum(std::string const& skew) {
    if (skew == "SKEW") {
        return nie::ParameterSkew::SKEW;
    } else if (skew == "NO_SKEW") {
        return nie::ParameterSkew::NO_SKEW;
    } else {
        LOG(FATAL) << "Unable to convert '" << skew << "' to nie::ParameterSkew enum";
    }
}

inline nie::ParameterDistortionRadial ParameterDistortionRadialToEnum(std::string const& distortion_radial) {
    if (distortion_radial == "K3") {
        return nie::ParameterDistortionRadial::K3;
    } else if (distortion_radial == "K6") {
        return nie::ParameterDistortionRadial::K6;
    } else if (distortion_radial == "NO_DISTORTION") {
        return nie::ParameterDistortionRadial::NO_DISTORTION;
    } else {
        LOG(FATAL) << "Unable to convert '" << distortion_radial << "' to nie::ParameterDistortionRadial enum";
    }
}

inline nie::ParameterDistortionTangential ParameterDistortionTangentialToEnum(
    std::string const& distortion_tangential) {
    if (distortion_tangential == "P2") {
        return nie::ParameterDistortionTangential::P2;
    } else if (distortion_tangential == "NO_DISTORTION") {
        return nie::ParameterDistortionTangential::NO_DISTORTION;
    } else {
        LOG(FATAL) << "Unable to convert '" << distortion_tangential << "' to nie::ParameterDistortionTangential enum";
    }
}

inline nie::ParameterDistortionThinPrism ParameterDistortionThinPrismToEnum(std::string const& distortion_thin_prism) {
    if (distortion_thin_prism == "S4") {
        return nie::ParameterDistortionThinPrism::S4;
    } else if (distortion_thin_prism == "NO_DISTORTION") {
        return nie::ParameterDistortionThinPrism::NO_DISTORTION;
    } else {
        LOG(FATAL) << "Unable to convert '" << distortion_thin_prism << "' to nie::ParameterDistortionThinPrism enum";
    }
}

namespace detail {

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class DistortionModelParametersTemplated {
public:
    static constexpr ParameterFocalLength kParameterFocalLength = FL;
    static constexpr ParameterSkew kParameterSkew = SK;
    static constexpr ParameterDistortionRadial kParameterDistortionRadial = RA;
    static constexpr ParameterDistortionTangential kParameterDistortionTangential = TA;
    static constexpr ParameterDistortionThinPrism kParameterDistortionThinPrism = TP;
};

}  // namespace detail

}  // namespace nie

#endif  // NIE_CV_CALIB3D_DISTORTION_MODEL_PARAMETERS_HPP
