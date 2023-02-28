// Copyright (c) Facebook, Inc. and its affiliates
// SPDX-License-Identifier: MIT OR Apache-2.0

#pragma once

#include <cstdint>
#include <limits>

#include "binary.hpp"
#include "serde.hpp"

// Maximum length supported in practice (e.g. Java).
constexpr size_t Postcard_MAX_LENGTH = (1ull << 31) - 1;

namespace serde {

class PostcardSerializer : public BinarySerializer<PostcardSerializer> {
    using Parent = BinarySerializer<PostcardSerializer>;

    template<typename T>
    void serialize_unsigned_as_uleb128(T);

    template<typename T>
    T zigzag(T);

  public:
    PostcardSerializer() : Parent(SIZE_MAX) {}

    void serialize_u16(uint16_t value);
    void serialize_u32(uint32_t value);
    void serialize_u64(uint64_t value);

    void serialize_i16(int16_t value);
    void serialize_i32(int32_t value);
    void serialize_i64(int64_t value);

    void serialize_f32(float value);
    void serialize_f64(double value);
    void serialize_len(size_t value);
    void serialize_variant_index(uint32_t value);

    static constexpr bool enforce_strict_map_ordering = false;
};

class PostcardDeserializer : public BinaryDeserializer<PostcardDeserializer> {
    using Parent = BinaryDeserializer<PostcardDeserializer>;

    template<typename T>
    T deserialize_uleb128();

    template<typename U, typename I>
    I de_zigzag(U);

  public:
    PostcardDeserializer(std::vector<uint8_t> bytes)
        : Parent(std::move(bytes), SIZE_MAX) {}

    uint64_t deserialize_u64();
    uint32_t deserialize_u32();
    uint16_t deserialize_u16();

    int64_t deserialize_i64();
    int32_t deserialize_i32();
    int16_t deserialize_i16();


    float deserialize_f32();
    double deserialize_f64();
    size_t deserialize_len();
    uint32_t deserialize_variant_index();

    static constexpr bool enforce_strict_map_ordering = false;
};

// Native floats and doubles must be IEEE-754 values of the expected size.
static_assert(std::numeric_limits<float>::is_iec559);
static_assert(std::numeric_limits<double>::is_iec559);
static_assert(sizeof(float) == sizeof(uint32_t));
static_assert(sizeof(double) == sizeof(uint64_t));

template<typename T>
inline void PostcardSerializer::serialize_unsigned_as_uleb128(T value) {
  while (value >= 0x80) {
    bytes_.push_back((uint8_t)((value & 0x7F) | 0x80));
    value = value >> 7;
  }
  bytes_.push_back((uint8_t)value);
}

inline void PostcardSerializer::serialize_u64(uint64_t value) {
  PostcardSerializer::serialize_unsigned_as_uleb128(value);
}

inline void PostcardSerializer::serialize_u32(uint32_t value) {
  PostcardSerializer::serialize_unsigned_as_uleb128(value);
}

inline void PostcardSerializer::serialize_u16(uint16_t value) {
  PostcardSerializer::serialize_unsigned_as_uleb128(value);
}

inline void PostcardSerializer::serialize_i64(int64_t value) {
  PostcardSerializer::serialize_unsigned_as_uleb128((uint64_t)zigzag(value));
}

inline void PostcardSerializer::serialize_i32(int32_t value) {
  PostcardSerializer::serialize_unsigned_as_uleb128((uint32_t)zigzag(value));
}

inline void PostcardSerializer::serialize_i16(int16_t value) {
  PostcardSerializer::serialize_unsigned_as_uleb128((uint16_t)zigzag(value));
}

inline void PostcardSerializer::serialize_f32(float value) {
    Parent::serialize_u32(*reinterpret_cast<uint32_t *>(&value));
}

inline void PostcardSerializer::serialize_f64(double value) {
    Parent::serialize_u64(*reinterpret_cast<uint64_t *>(&value));
}

inline void PostcardSerializer::serialize_len(size_t value) {
    if (value > Postcard_MAX_LENGTH) {
        throw serde::serialization_error("Length is too large");
    }
    PostcardSerializer::serialize_unsigned_as_uleb128((uint64_t)value);
}

inline void PostcardSerializer::serialize_variant_index(uint32_t value) {
    PostcardSerializer::serialize_unsigned_as_uleb128(value);
}

template<typename T>
inline T PostcardSerializer::zigzag(T n) {
  return ((n << 1) ^ (n >> ((sizeof(T) * 8) - 1)));
}


template<typename T>
inline T PostcardDeserializer::deserialize_uleb128() {
  uint64_t value = 0;
  for (int shift = 0; shift < sizeof(T) * 8; shift += 7) {
    auto byte = read_byte();
    auto digit = byte & 0x7F;
    value |= (uint64_t)digit << shift;
    if (value > std::numeric_limits<T>::max()) {
      throw serde::deserialization_error(
          "Overflow while parsing uleb128-encoded uint32 value");
    }
    if (digit == byte) {
      if (shift > 0 && digit == 0) {
        throw serde::deserialization_error(
            "Invalid uleb128 number (unexpected zero digit)");
      }
      return (T)value;
    }
  }
  throw serde::deserialization_error(
      "Overflow while parsing uleb128-encoded uint32 value");
}

template<typename U, typename I>
inline I PostcardDeserializer::de_zigzag(U n) {
  return ((I)(n >> 1)) ^ (-( (I) (n & 0b1)));
}

inline uint64_t PostcardDeserializer::deserialize_u64() {
  return PostcardDeserializer::deserialize_uleb128<uint64_t>();
}

inline uint32_t PostcardDeserializer::deserialize_u32() {
  return PostcardDeserializer::deserialize_uleb128<uint32_t>();
}

inline uint16_t PostcardDeserializer::deserialize_u16() {
  return PostcardDeserializer::deserialize_uleb128<uint16_t>();
}

inline int64_t PostcardDeserializer::deserialize_i64() {
  auto value = PostcardDeserializer::deserialize_uleb128<uint64_t>();
  return de_zigzag<uint64_t, int64_t>(value);
}

inline int32_t PostcardDeserializer::deserialize_i32() {
  auto value = PostcardDeserializer::deserialize_uleb128<uint32_t>();
  return de_zigzag<uint32_t, int32_t>(value);
}

inline int16_t PostcardDeserializer::deserialize_i16() {
  auto value = PostcardDeserializer::deserialize_uleb128<uint16_t>();
  return de_zigzag<uint16_t, int16_t>(value);
}

inline float PostcardDeserializer::deserialize_f32() {
    auto value = Parent::deserialize_u32();
    return *reinterpret_cast<float *>(&value);
}

inline double PostcardDeserializer::deserialize_f64() {
    auto value = Parent::deserialize_u64();
    return *reinterpret_cast<double *>(&value);
}

inline size_t PostcardDeserializer::deserialize_len() {
    auto value = (size_t)PostcardDeserializer::deserialize_u64();
    if (value > Postcard_MAX_LENGTH) {
        throw serde::deserialization_error("Length is too large");
    }
    return (size_t)value;
}

inline uint32_t PostcardDeserializer::deserialize_variant_index() {
    return PostcardDeserializer::deserialize_u32();
}



} // end of namespace serde
