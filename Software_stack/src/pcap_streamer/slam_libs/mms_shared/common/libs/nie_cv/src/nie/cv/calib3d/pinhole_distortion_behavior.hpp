/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CALIB3D_PINHOLE_DISTORTION_BEHAVIOR_HPP
#define NIE_CV_CALIB3D_PINHOLE_DISTORTION_BEHAVIOR_HPP

#include "distortion_model_parameters.hpp"

namespace nie {

namespace detail {

// Started with the following models
// https://docs.opencv.org/3.3.1/d9/d0c/group__calib3d.html

template <typename T>
inline void DistortThinPrism(
    T const& r2, T const& r4, T const& s_1, T const& s_2, T const& s_3, T const& s_4, T* d_u, T* d_v) {
    *d_u += s_1 * r2 + s_2 * r4;
    *d_v += s_3 * r2 + s_4 * r4;
}

template <typename T>
inline void DistortTangential(T const& r2, T const& p_1, T const& p_2, T const& u_u, T const& u_v, T* d_u, T* d_v) {
    *d_u += 2.0 * p_1 * u_u * u_v + p_2 * (r2 + 2.0 * u_u * u_u);
    *d_v += p_1 * (r2 + 2.0 * u_v * u_v) + 2.0 * p_2 * u_u * u_v;
}

template <typename T>
inline void DistortRadial_K3(
    T const& r2, T const& r4, T const& r6, T const& k_1, T const& k_2, T const& k_3, T* d_u, T* d_v) {
    T rd = 1.0 + k_1 * r2 + k_2 * r4 + k_3 * r6;
    *d_u *= rd;
    *d_v *= rd;
}

template <typename T>
inline void DistortRadial_K6(
    T const& r2,
    T const& r4,
    T const& r6,
    T const& k_1,
    T const& k_2,
    T const& k_3,
    T const& k_4,
    T const& k_5,
    T const& k_6,
    T* d_u,
    T* d_v) {
    T rd = (1.0 + k_1 * r2 + k_2 * r4 + k_3 * r6) / (1.0 + k_4 * r2 + k_5 * r4 + k_6 * r6);
    *d_u *= rd;
    *d_v *= rd;
}

template <ParameterFocalLength FL>
class BehaviorFocalLength {};

template <>
class BehaviorFocalLength<ParameterFocalLength::SINGLE_F> {
public:
    static constexpr int kParameters = 1;

    template <typename T>
    inline static void Apply(const T* const intrinsics, T const& v_u, T const& v_v, T* p_u, T* p_v) {
        *p_u = v_u * intrinsics[0] + intrinsics[1];
        *p_v = v_v * intrinsics[0] + intrinsics[2];
    }

    template <typename T>
    inline static void Undo(const T* const intrinsics, T const& p_u, T const& p_v, T* v_u, T* v_v) {
        *v_u = (p_u - intrinsics[1]) / intrinsics[0];
        *v_v = (p_v - intrinsics[2]) / intrinsics[0];
    }
};

template <>
class BehaviorFocalLength<ParameterFocalLength::XY_F> {
public:
    static constexpr int kParameters = 2;

    template <typename T>
    inline static void Apply(const T* const intrinsics, T const& d_u, T const& d_v, T* p_u, T* p_v) {
        *p_u = d_u * intrinsics[0] + intrinsics[2];
        *p_v = d_v * intrinsics[1] + intrinsics[3];
    }

    template <typename T>
    inline static void Undo(const T* const intrinsics, T const& p_u, T const& p_v, T* v_u, T* v_v) {
        *v_u = (p_u - intrinsics[2]) / intrinsics[0];
        *v_v = (p_v - intrinsics[3]) / intrinsics[1];
    }
};

template <ParameterSkew SK>
class BehaviorSkew {};

template <>
class BehaviorSkew<ParameterSkew::NO_SKEW> {
public:
    static constexpr int kParameters = 0;

    template <typename T>
    inline static void Apply(const T* const, T const&, T const&, T*, T*) {}

    template <typename T>
    inline static void Undo(const T* const, T const&, T const&, T*, T*) {}
};

template <>
class BehaviorSkew<ParameterSkew::SKEW> {
public:
    static constexpr int kParameters = 1;

    template <typename T>
    inline static void Apply(const T* const intrinsics, T const&, T const& d_v, T* p_u, T*) {
        *p_u += d_v * intrinsics[0];
    }

    template <typename T>
    inline static void Undo(const T* const intrinsics, T const&, T const& p_v, T* v_u, T*) {
        *v_u -= p_v * intrinsics[0];
    }
};

template <ParameterDistortionRadial RA>
class BehaviorDistortionRadial {};

template <>
class BehaviorDistortionRadial<ParameterDistortionRadial::NO_DISTORTION> {
public:
    static constexpr int kParameters = 0;

    template <typename T>
    inline static void Apply(const T* const, T const&, T const&, T const&, T*, T*) {}
};

template <>
class BehaviorDistortionRadial<ParameterDistortionRadial::K3> {
public:
    static constexpr int kParameters = 3;

    template <typename T>
    inline static void Apply(const T* const intrinsics, T const& r2, T const& r4, T const& r6, T* d_u, T* d_v) {
        DistortRadial_K3(r2, r4, r6, intrinsics[0], intrinsics[1], intrinsics[2], d_u, d_v);
    }
};

template <>
class BehaviorDistortionRadial<ParameterDistortionRadial::K6> {
public:
    static constexpr int kParameters = 6;

    template <typename T>
    inline static void Apply(const T* const intrinsics, T const& r2, T const& r4, T const& r6, T* d_u, T* d_v) {
        DistortRadial_K6(
            r2,
            r4,
            r6,
            intrinsics[0],
            intrinsics[1],
            intrinsics[2],
            intrinsics[3],
            intrinsics[4],
            intrinsics[5],
            d_u,
            d_v);
    }
};

template <ParameterDistortionTangential TA>
class BehaviorDistortionTangential {};

template <>
class BehaviorDistortionTangential<ParameterDistortionTangential::NO_DISTORTION> {
public:
    static constexpr int kParameters = 0;

    template <typename T>
    inline static void Apply(const T* const, T const&, T const&, T const&, T*, T*) {}
};

template <>
class BehaviorDistortionTangential<ParameterDistortionTangential::P2> {
public:
    static constexpr int kParameters = 2;

    template <typename T>
    inline static void Apply(const T* const intrinsics, T const& r2, T const& u_u, T const& u_v, T* d_u, T* d_v) {
        DistortTangential(r2, intrinsics[0], intrinsics[1], u_u, u_v, d_u, d_v);
    }
};

template <ParameterDistortionThinPrism TP>
class BehaviorDistortionThinPrism {};

template <>
class BehaviorDistortionThinPrism<ParameterDistortionThinPrism ::NO_DISTORTION> {
public:
    static constexpr int kParameters = 0;

    template <typename T>
    inline static void Apply(const T* const, T const&, T const&, T*, T*) {}
};

template <>
class BehaviorDistortionThinPrism<ParameterDistortionThinPrism::S4> {
public:
    static constexpr int kParameters = 4;

    template <typename T>
    inline static void Apply(const T* const intrinsics, T const& r2, T const& r4, T* d_u, T* d_v) {
        DistortThinPrism(r2, r4, intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3], d_u, d_v);
    }
};

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class PinholeDistortionModelHelper {
public:
    static constexpr ParameterFocalLength kParameterFocalLength = FL;
    static constexpr ParameterSkew kParameterSkew = SK;
    static constexpr ParameterDistortionRadial kParameterDistortionRadial = RA;
    static constexpr ParameterDistortionTangential kParameterDistortionTangential = TA;
    static constexpr ParameterDistortionThinPrism kParameterDistortionThinPrism = TP;

    static constexpr int kIndexPrincipalPoint = BehaviorFocalLength<FL>::kParameters;

    static constexpr int kIndexDistortionSkew = kIndexPrincipalPoint + 2;  // Principal point parameter count

    static constexpr int kIndexDistortionRadial = kIndexDistortionSkew + BehaviorSkew<SK>::kParameters;

    static constexpr int kIndexDistortionTangential =
        kIndexDistortionRadial + BehaviorDistortionRadial<RA>::kParameters;

    static constexpr int kIndexDistortionThinPrism =
        kIndexDistortionTangential + BehaviorDistortionTangential<TA>::kParameters;

    static constexpr int kParametersK = detail::BehaviorFocalLength<FL>::kParameters + 2 +  // Principal point
                                        detail::BehaviorSkew<SK>::kParameters;

    static constexpr int kParametersIntrinsics = kParametersK + detail::BehaviorDistortionRadial<RA>::kParameters +
                                                 detail::BehaviorDistortionTangential<TA>::kParameters +
                                                 detail::BehaviorDistortionThinPrism<TP>::kParameters;
};

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP,
    typename T>
inline void Distort(const T* const intrinsics, T const& u_u, T const& u_v, T* p_u, T* p_v) {
    using I = PinholeDistortionModelHelper<FL, SK, RA, TA, TP>;

    T r2 = u_u * u_u + u_v * u_v;
    T r4 = r2 * r2;
    T r6 = r4 * r2;

    T d_u = u_u;
    T d_v = u_v;

    BehaviorDistortionRadial<RA>::Apply(intrinsics + I::kIndexDistortionRadial, r2, r4, r6, &d_u, &d_v);
    BehaviorDistortionTangential<TA>::Apply(intrinsics + I::kIndexDistortionTangential, r2, u_u, u_v, &d_u, &d_v);
    BehaviorDistortionThinPrism<TP>::Apply(intrinsics + I::kIndexDistortionThinPrism, r2, r4, &d_u, &d_v);

    // Get image coordinates
    // NOTE: slightly dirty. The same variables are used for input and output. This gives no side effects.
    BehaviorSkew<SK>::Apply(intrinsics + I::kIndexDistortionSkew, d_u, d_v, &d_u, &d_v);
    BehaviorFocalLength<FL>::Apply(intrinsics, d_u, d_v, p_u, p_v);
}

}  // namespace detail

}  // namespace nie

#endif  // NIE_CV_CALIB3D_PINHOLE_DISTORTION_BEHAVIOR_HPP
