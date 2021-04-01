#include "gpio.hpp"

#include <phosphor-logging/log.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

namespace ampere
{
static constexpr auto consumer_label = "ampere-scp-failover";

Gpio::Gpio(uint32_t line)
{
    gpioEvent = buildGpioEvent(0, line);
}

Gpio::Gpio(uint32_t line, [[maybe_unused]] bool output)
{
    gpioHandle = buildGpioHandle(0, line);
}

void Gpio::GpioReleaseEvent()
{
    gpioEvent->~EventInterface();
}

void Gpio::GpioReleaseHandle()
{
    gpioHandle->~HandleInterface();
}

std::unique_ptr<gpioplus::Handle> Gpio::buildGpioHandle(uint32_t id,
                                                        uint32_t line)
{
    try
    {
        gpioplus::Chip chip(id);
        gpioplus::HandleFlags flags(chip.getLineInfo(line).flags);
        flags.output = true;
        std::vector<gpioplus::Handle::Line> lines = {{line, 0}};
        return std::make_unique<gpioplus::Handle>(chip, lines, flags,
                                                  consumer_label);
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
        return nullptr;
    }
}

std::unique_ptr<gpioplus::Event> Gpio::buildGpioEvent(uint32_t id,
                                                      uint32_t line)
{
    try
    {
        gpioplus::Chip chip(id);
        gpioplus::HandleFlags handleflags(chip.getLineInfo(line).flags);
        handleflags.output = false;
        gpioplus::EventFlags eventflags;
        eventflags.falling_edge = true;
        eventflags.rising_edge = true;
        return std::make_unique<gpioplus::Event>(
            chip, line, handleflags, eventflags, consumer_label);
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
        return nullptr;
    }
}

void Gpio::setValue(uint8_t v)
{
    gpioHandle->setValues({v});
}

int Gpio::getValue()
{
    return gpioEvent->getValue();
}

int Gpio::getValue([[maybe_unused]] bool output)
{
    std::vector<uint8_t> value;

    value = gpioHandle->getValues();
    return value[0];
}

int Gpio::getEventFd()
{
    using namespace gpioplus::internal;
    return *gpioEvent->getFd();
}

int Gpio::eventRead()
{
    return gpioEvent->read()->id;
}

} // namespace ampere
