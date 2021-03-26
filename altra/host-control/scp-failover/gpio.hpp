#pragma once

#include <systemd/sd-event.h>

#include <gpioplus/chip.hpp>
#include <gpioplus/event.hpp>
#include <gpioplus/handle.hpp>

#include <map>
#include <memory>
#include <sdbusplus/message.hpp>
#include <string>

namespace ampere
{
struct EventDeleter
{
    void operator()(sd_event* event) const
    {
        event = sd_event_unref(event);
    }
};
using EventPtr = std::unique_ptr<sd_event, EventDeleter>;
class Gpio
{
  public:
    Gpio(uint32_t line);
    Gpio(uint32_t line, bool output);
    void GpioReleaseEvent(void);
    void GpioReleaseHandle(void);
    void setValue(uint8_t v);
    int getValue(void);
    int getValue(bool output);
    int getEventFd();
    int eventRead();

    enum GPIO
    {
        LOW = 0,
        HIGH = 1
    };

  private:
    std::unique_ptr<gpioplus::Handle> gpioHandle;
    std::unique_ptr<gpioplus::Event> gpioEvent;

    std::unique_ptr<gpioplus::Handle> buildGpioHandle(uint32_t chip,
                                                      uint32_t offset);
    std::unique_ptr<gpioplus::Event> buildGpioEvent(uint32_t chip,
                                                    uint32_t offset);
};
} // namespace ampere
