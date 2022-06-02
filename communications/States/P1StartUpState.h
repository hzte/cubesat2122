#include <iostream>
#include <pigpio.h>

#include "../Controller.h"
#include "../State.h"
#include "../CONSTANTS.h"
#include "P2DetectionState.h"

class P1StartUpState : public State
{
private:
    // used to hold current I2C connection
    int handle;
    // False if still calibrating
    // True if subsystem is ready for next phase
    bool payloadsConfirmation = false;
    bool AOCSConfirmation = false;

public:
    P1StartUpState(Controller *controller)
        : State(controller)
    {
    }

    void entryEvent() override;
    void updateState() override;
    void transmitGroundStation();
    void transmitSubsystems();

    // optional: void entryEvent() override
};

void P1StartUpState::entryEvent()
{
    // initialise GPIO
    // if (gpioInitialise() < 0) return 1;
    // set gpio pin modes
}

void P1StartUpState::updateState()
{
    std::cout << "P1StartUpState is working" << std::endl;
    transmitSubsystems();
    transmitGroundStation();
    // change state condition
    if (payloadsConfirmation & AOCSConfirmation)
    {
        controller->setState<P2DetectionState>(controller);
    }
}

void P1StartUpState::transmitGroundStation()
{
    // Transmit startup confirmation
    // controller->setState<P2DetectionState>(controller);
}

// Receive confirmation signals from subsystems
//     Payloads
//     AOCS
void P1StartUpState::transmitSubsystems()
{
    // Payloads
    if (!payloadsConfirmation)
    {
        handle = i2cOpen(BUS, PAYLOADS_I2C_ADDR, 0);
        int message = i2cReadByte(handle);
        if (message == 0x1)
        {
            payloadsConfirmation = true;
        }
        i2cClose(handle);
    }
    // AOCS
    if (!AOCSConfirmation)
    {
        handle = i2cOpen(BUS, AOCS_I2C_ADDR, 0);
        int message = i2cReadByte(handle);
        if (message == 0x1)
        {
            AOCSConfirmation = true;
        }
        i2cClose(handle);
    }
}