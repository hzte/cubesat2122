#include <iostream>
#include "../Controller.h"
#include "../State.h"
#include "../CONSTANTS.h"

class P4ContainmentState : public State
{
public:
    P4ContainmentState(Controller *controller)
        : State(controller)
    {
    }

    void entryEvent() override;
    void updateState() override;
    void transmitGroundStation();
    void transmitSubsystems();

    // optional: void entryEvent() override
};

void P4ContainmentState::entryEvent()
{
    // reset gpio pin values
    // set gpio pin modes
}

void P4ContainmentState::updateState()
{
    std::cout << "P4ContainmentState is working" << std::endl;
    transmitSubsystems();
    transmitGroundStation();
}

void P4ContainmentState::transmitGroundStation()
{
}
void P4ContainmentState::transmitSubsystems()
{
}