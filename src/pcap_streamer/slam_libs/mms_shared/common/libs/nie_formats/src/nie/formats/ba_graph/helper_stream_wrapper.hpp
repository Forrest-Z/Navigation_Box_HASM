/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <cstdint>

#include "ba_graph_types.hpp"
#include "stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

/// Generic helper functions

void ReadChunkHeader(std::int32_t* length, ChunkType* type, IStreamWrapper* wrapper);

void WriteChunkHeader(const std::int32_t& length, const ChunkType& type, OStreamWrapper* wrapper);

SignatureType ReadSignature(IStreamWrapper* wrapper);

void WriteSignature(SignatureType const& type, OStreamWrapper* wrapper);

// Return the length property of the header
std::int32_t ReadHeaderChunkHeader(IStreamWrapper* wrapper);

void WriteFooter(OStreamWrapper* wrapper);

/// Generic functions to be implemented for the specific formats which will be used by the collection read and write
/// functions and the stream object

// Helper function definitions to calculate chunk length of an object, possibly also using the information from the
// header object
template <typename Object>
std::int32_t GetChunkLength(Object const& object);
template <typename Header, typename Object>
std::int32_t GetChunkLength(Header const& header, Object const& object);

// IO function definitions for the header and collection objects
template <typename Object>
void Read(IStreamWrapper* wrapper, Object* object);
template <typename Object>
void Write(Object const& object, OStreamWrapper* wrapper);

template <typename Header, typename... Records>
void ReadRecords(IStreamWrapper*, Header const&, std::function<void(Records)> const&...);

// IO function definitions for an object, possibly also using the information from the header object
template <typename Header, typename Record>
void Read(Header const& header, IStreamWrapper* wrapper, Record* record);
template <typename Header, typename Record>
void Write(Header const& header, Record const& record, OStreamWrapper* wrapper);

template <typename T, class Enabled = void>
struct CollectionSupportTraits {
    static constexpr bool is_collection = false;
    static constexpr bool is_header = false;
};

template <typename T>
struct CollectionSupportTraits<
    T,
    std::enable_if_t<std::is_same<std::remove_const_t<decltype(T::Signature)>, SignatureType>::value>> {
    static constexpr bool is_collection = true;
    static constexpr bool is_header = false;
    using Type = T;
    using Collection = T;
    using Header = typename Collection::Header;
    static const SignatureType Signature;
};

template <typename T>
const detail::SignatureType CollectionSupportTraits<
    T,
    std::enable_if_t<std::is_same<std::remove_const_t<decltype(T::Signature)>, SignatureType>::value>>::Signature =
    T::Signature;

template <typename T>
struct CollectionSupportTraits<
    T,
    std::enable_if_t<std::is_same<std::remove_const_t<decltype(T::Collection::Signature)>, SignatureType>::value>> {
    static constexpr bool is_collection = false;
    static constexpr bool is_header = true;
    using Type = T;
    using Collection = typename T::Collection;
    using Header = T;
    static const SignatureType Signature;
};

template <typename T>
const detail::SignatureType CollectionSupportTraits<
    T,
    std::enable_if_t<std::is_same<std::remove_const_t<decltype(T::Collection::Signature)>, SignatureType>::value>>::
    Signature = T::Collection::Signature;

template <typename T>
static constexpr bool is_collection_v = CollectionSupportTraits<T>::is_collection;

template <typename T>
static constexpr bool is_collection_header_v = CollectionSupportTraits<T>::is_header;

}  // namespace detail

}  // namespace io

}  // namespace nie

