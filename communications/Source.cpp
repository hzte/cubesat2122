#include <iostream>
#include "State.h"
#include "Controller.h"
#include "States/P1StartUpState.h"

int main()
{
    Controller controller;
    controller.setState<P1StartUpState>(&controller);

    for (int i = 0; i < 4; i++)
    {
        controller.updateState();
    }

    // controller.updateState();

    return 0;
}