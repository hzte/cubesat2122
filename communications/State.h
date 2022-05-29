#ifndef STATE_H
#define STATE_H

class Controller;

class State
{
protected:
  Controller *const controller;

  State(Controller *_controller)
      : controller(_controller)
  {
  }

  virtual ~State() {}

  /* Update the current state
     remain in this state or change it.
     The concrete state class has to override the method.
   */
  virtual void updateState() = 0;

  /* Optional function
     Is only called once when the state updates for the first time
     The concrete state class has to override the method.
   */
  virtual void entryEvent() {}

  /*Optional function
    The concrete state class has to override the method.
   */
  virtual const char *stateName()
  {
    return "";
  }

  friend class Controller;
};

#endif