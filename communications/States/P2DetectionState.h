#include <iostream>
#include "../Controller.h"
#include "../State.h"
#include "../CONSTANTS.h"
#include "P3CaptureState.h"

class P2DetectionState : public State
{
public:
    P2DetectionState(Controller *controller)
        : State(controller)
    {
    }

    void entryEvent() override;
    void updateState() override;
    void transmitGroundStation();
    void transmitSubsystems();

    // optional: void entryEvent() override
};

void P2DetectionState::entryEvent()
{
    // reset gpio pin values
    // set gpio pin modes
}

void P2DetectionState::updateState()
{
    std::cout << "P2DetectionState is working" << std::endl;
    transmitSubsystems();
    transmitGroundStation();
    // if debris detected by payloads
    // --> controller->setState<P3CaptureState>(controller);
    controller->setState<P3CaptureState>(controller);
}

void P2DetectionState::transmitGroundStation()
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
void P2DetectionState::transmitSubsystems()
{
    /*
    Payloads --> AOCS
        (No data)
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