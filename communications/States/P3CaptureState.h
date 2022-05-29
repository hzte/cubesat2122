#include <iostream>
#include "../Controller.h"
#include "../State.h"
#include "../CONSTANTS.h"
#include "P4ContainmentState.h"

class P3CaptureState : public State
{
public:
    P3CaptureState(Controller *controller)
        : State(controller)
    {
    }

    void entryEvent() override;
    void updateState() override;
    void transmitGroundStation();
    void transmitSubsystems();

    // optional: void entryEvent() override
};

void P3CaptureState::entryEvent()
{
    // reset gpio pin values
    // set gpio pin modes
}

void P3CaptureState::updateState()
{
    std::cout << "P3CaptureState is working" << std::endl;
    transmitSubsystems();
    transmitGroundStation();
    // if debris capture success
    // --> controller->setState<P4ContainmentState>(controller);
    // else if debris capture failed
    // --> controller->setState<P2DetectionState>(controller);
    controller->setState<P4ContainmentState>(controller);
}

void P3CaptureState::transmitGroundStation()
{
    /*
    Communications --> Ground Station
        Payloads
            Image/video data
            Range data (Sound Sensor)
        AOCS
            Attitude data
            Angular velocity (CubeSat)
    */
}
void P3CaptureState::transmitSubsystems()
{
    /*
    Payloads --> AOCS
        Debris velocity
        Range data (Sound Sensor)
    Payloads --> Communications
        Image/video data
        Range data (Sound Sensor)
    AOCS  --> Payloads
        Attitude data
    AOCS --> Communications
        Attitude data
        Angular velocity (CubeSat)
    */
}