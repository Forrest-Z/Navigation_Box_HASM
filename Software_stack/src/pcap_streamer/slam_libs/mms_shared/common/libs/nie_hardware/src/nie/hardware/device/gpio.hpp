/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_DEVICE_GPIO_HPP
#define NIE_HARDWARE_DEVICE_GPIO_HPP

#include <cstdint>
#include <string>

namespace nie {

enum class GpioDirection {
    // Gpio can be used for reading.
    kIn = 0,
    // Gpio can be used for writing.
    kOut
};

enum class GpioValue { kLow = 0, kHigh };

enum class GpioActiveLow {
    // Do not invert the interpretation of GpioValue.
    kFalse = 0,
    // Invert the interpretation of GpioValue.
    kTrue
};

// Linux only.
//
// Simple interface class to communicate with gpio signals through sysfs. Each gpio is considered a stand alone
// signal. The sysfs interface for gpios can be found in directory:
//      /sys/class/gpio
//
// Here one can check which gpios are available through their respective controllers. A controller corresponds to a
// directory. These are named like:
//      gpiochip<n>
//
// Here <n> is the first gpio available for this controller. The amount of gpios from this controller can be read from
// the ngpio file in the gpiochip<n> directory.
//
// Before a gpio signal can be used, it has to be exported. This class exports a gpio on creation. On creation, it is
// expected that the respective gpio has not been exported yet. The destructor unexports the gpio. The class doesn't
// lock the files associated with each gpio.
//
// More information on gpio:
//      https://www.kernel.org/doc/Documentation/gpio/sysfs.txt
// TODO(jbr) This gpio could be returned through a GpioManager.
// TODO(jbr) Controller can check availability.
// TODO(jbr) Perhaps keep the gpio locked?
class Gpio {
public:
    // TODO(jbr) now we assume the gpio is not in use and we always export and release on destruction.
    // TODO(jbr) perhaps lock the value file if the process is slow to write values?
    // TODO(jbr) do we need to set the default values below?
    // Each gpio defaults to having their properties set to:
    //      direction = in
    //      value = 0
    //      active_low = 0
    explicit Gpio(const std::uint32_t& id);
    ~Gpio();

    // The id of the gpio.
    std::uint32_t id() const;

    // "direction" ... reads as either "in" or "out". This value may
    //      normally be written. Writing as "out" defaults to
    //      initializing the value as low. To ensure glitch free
    //      operation, values "low" and "high" may be written to
    //      configure the GPIO as an output with that initial value.
    //
    //      Note that this attribute *will not exist* if the kernel
    //      doesn't support changing the direction of a GPIO, or
    //      it was exported by kernel code that didn't explicitly
    //      allow userspace to reconfigure this GPIO's direction.
    GpioDirection direction() const;
    void direction(GpioDirection v) const;

    // "value" ... reads as either 0 (low) or 1 (high). If the GPIO
    //      is configured as an output, this value may be written;
    //      any nonzero value is treated as high.
    //
    //      If the pin can be configured as interrupt-generating interrupt
    //      and if it has been configured to generate interrupts (see the
    //      description of "edge"), you can poll(2) on that file and
    //      poll(2) will return whenever the interrupt was triggered_toggle. If
    //      you use poll(2), set the events POLLPRI and POLLERR. If you
    //      use select(2), set the file descriptor in exceptfds. After
    //      poll(2) returns, either lseek(2) to the beginning of the sysfs
    //      file and read the new value or close the file and re-open it
    //      to read the value.
    GpioValue value() const;
    void value(GpioValue v) const;

    // "active_low" ... reads as either 0 (false) or 1 (true). Write
    //      any nonzero value to invert the value attribute both
    //      for reading and writing. Existing and subsequent
    //      poll(2) support configuration via the edge attribute
    //      for "rising" and "falling" edges will follow this
    //      setting.
    GpioActiveLow active_low() const;
    void active_low(GpioActiveLow v) const;

private:
    static constexpr char kPathGpioControllerExport[] = "/sys/class/gpio/export";
    static constexpr char kPathGpioControllerUnexport[] = "/sys/class/gpio/unexport";
    static constexpr char kPathGpioBaseNoId[] = "/sys/class/gpio/gpio";
    static constexpr char kPathGpioDirection[] = "/direction";
    static constexpr char kPathGpioValue[] = "/value";
    static constexpr char kPathGpioActiveLow[] = "/active_low";
    static constexpr char kDirectionIn[] = "in";
    static constexpr char kDirectionOut[] = "out";

    std::string GetGpioDirectionPath() const;
    std::string GetGpioValuePath() const;
    std::string GetGpioActiveLowPath() const;

    template <typename T>
    void WriteToFile(const std::string& path, const T& value) const;
    template <typename T>
    void ReadFromFile(const std::string& path, T* value) const;
    // Export this with the gpio controller.
    void Export() const;
    // Unxport this with the gpio controller.
    void Unexport() const;

    const std::uint32_t id_;
};

}  // namespace nie

#endif  // NIE_HARDWARE_DEVICE_GPIO_HPP
