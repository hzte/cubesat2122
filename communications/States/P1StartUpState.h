#include <iostream>
#include "../Controller.h"
#include "../State.h"
#include "../CONSTANTS.h"
#include "P2DetectionState.h"

class P1StartUpState : public State
{
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
    controller->setState<P2DetectionState>(controller);
}

void P1StartUpState::transmitGroundStation()
{
    // Transmit startup confirmation
    // controller->setState<P2DetectionState>(controller);
}
void P1StartUpState::transmitSubsystems()
{
    // Receive confirmation signals from subsystems
    //     Payloads
    //     AOCS
}