#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "State.h"

class Controller
{
    State *currentState = nullptr;
    State *nextState = nullptr;

public:
    // Controller object constructor
    Controller()
    {
    }

    /* Set the current state
       The first template argument T should be specified as a concrete state class
       Parameters are passed bounded by paranthesis.
       E.g. controller.setState<P1StartUpState>(&controller);
            controller.setState<P1StartUpState>(&controller, 5, "Hello");
     */
    template <class T, class... Types>
    void setState(Types... args)
    {
        delete nextState;
        nextState = new T(args...);
    }

    /* Execute the current state's update function and change states
       if it is requested.
     */
    void updateState()
    {
        if (nextState != nullptr)
        {
            delete currentState;
            currentState = nextState;
            nextState = nullptr;

            currentState->entryEvent();
        }

        if (currentState != nullptr)
            currentState->updateState();
    }
};

#endif
