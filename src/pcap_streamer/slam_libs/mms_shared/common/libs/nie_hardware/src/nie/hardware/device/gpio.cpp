/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "gpio.hpp"

#include <nie/core/filesystem.hpp>

std::string to_string(const std::string& value) { return value; }

namespace nie {

// Reason: You have to provide the definition of the static member as well as the declaration. The declaration and
// the initializer go inside the class definition, but the member definition has to be separate.
// These can't be in the hpp because otherwise every include of gpio causes them to be defined multiple times.
constexpr char Gpio::kPathGpioControllerExport[];
constexpr char Gpio::kPathGpioControllerUnexport[];
constexpr char Gpio::kPathGpioBaseNoId[];
constexpr char Gpio::kPathGpioDirection[];
constexpr char Gpio::kPathGpioValue[];
constexpr char Gpio::kPathGpioActiveLow[];
constexpr char Gpio::kDirectionIn[];
constexpr char Gpio::kDirectionOut[];

Gpio::Gpio(const std::uint32_t& id) : id_(id) {
    Export();
    direction(GpioDirection::kOut);
    value(GpioValue::kLow);
    active_low(GpioActiveLow::kFalse);
}

Gpio::~Gpio() {
    try {
        Unexport();
    } catch (const std::exception& e) {
    }
}

std::uint32_t Gpio::id() const { return id_; }

GpioDirection Gpio::direction() const {
    std::string v;
    ReadFromFile(GetGpioDirectionPath(), &v);

    if (v == kDirectionOut) return GpioDirection::kOut;

    return GpioDirection::kIn;
}

void Gpio::direction(GpioDirection v) const {
    switch (v) {
        case GpioDirection::kOut:
            WriteToFile(GetGpioDirectionPath(), kDirectionOut);
            break;
        case GpioDirection::kIn:
            WriteToFile(GetGpioDirectionPath(), kDirectionIn);
            break;
    }
}

GpioValue Gpio::value() const {
    int v;
    ReadFromFile(GetGpioValuePath(), &v);

    return static_cast<GpioValue>(v);
}

void Gpio::value(GpioValue v) const { WriteToFile(GetGpioValuePath(), static_cast<int>(v)); }

GpioActiveLow Gpio::active_low() const {
    int v;
    ReadFromFile(GetGpioActiveLowPath(), &v);

    return static_cast<GpioActiveLow>(v);
}

void Gpio::active_low(GpioActiveLow v) const { WriteToFile(GetGpioActiveLowPath(), static_cast<int>(v)); }

std::string Gpio::GetGpioDirectionPath() const { return kPathGpioBaseNoId + std::to_string(id_) + kPathGpioDirection; }

std::string Gpio::GetGpioValuePath() const { return kPathGpioBaseNoId + std::to_string(id_) + kPathGpioValue; }

std::string Gpio::GetGpioActiveLowPath() const { return kPathGpioBaseNoId + std::to_string(id_) + kPathGpioActiveLow; }

template <typename T>
void Gpio::WriteToFile(const std::string& path, const T& value) const {
    std::fstream file = OpenFile(path, std::ios::out);

    // Here we use the namespace to obtain an overload for to_string(string)
    using namespace std;
    file << to_string(value);
    // Flushing ensures something is (attempted to get) written to the respective file.
    // It means an exception may be thrown if there are no rights.
    // If nothing is flushed there is no direct interaction with the gpio interface and
    // things only get flushed when the fstream is destructed (destructors catch
    // exceptions) making the problem invisible.
    file.flush();

    if (file.fail()) {
        throw std::runtime_error("Unable to write value to file: " + path + " " + to_string(value));
    }
}

template <typename T>
void Gpio::ReadFromFile(const std::string& path, T* value) const {
    std::fstream file = OpenFile(path, std::ios::in);

    file >> *value;

    if (file.fail()) {
        throw std::runtime_error("Unable to read from file: " + path);
    }
}

void Gpio::Export() const { WriteToFile(kPathGpioControllerExport, id_); }

void Gpio::Unexport() const { WriteToFile(kPathGpioControllerUnexport, id_); }

}  // namespace nie