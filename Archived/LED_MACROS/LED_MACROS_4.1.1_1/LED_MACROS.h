//LED_MACROS REV 4.1.1_1
//.h
#ifndef LED_MACROS_h
#define LED_MACROS_h

#include "Arduino.h"

class MACROS {
  public:
    bool Run();
    bool Ready()
    {
      return !_MacroRun;
    }

    void RST() {
      _MacroRun = false;
    }

    void Fade(uint8_t Target, uint8_t Frames);
    void Set(uint8_t Target, uint16_t Delay);
    void SetDelay(uint16_t Delay);
    void SetVal(uint8_t Val);
    void SetFPS(uint16_t FPS);
    uint8_t Val() {
      return _Val;
    }

  private:
    bool _MacroRun = false;

    //Which Macro is Running: false = Set, high = Fade
    uint32_t _Timer;
    uint16_t _Delay;

    //Rate For Frames. Default is 33ms for 30 FPS:
    uint16_t _Rate = 33;
    uint8_t _Val;
    uint8_t _Target;
    uint8_t _Increment;
};

class MACROS_BUILD {
  public:
    bool MacroChange() {
      bool Change = false;
      if(PrevMacro != Macro) {
        Change = true;
        PrevMacro = Macro;
      }
      return Change;
    }
    uint8_t PrevMacro = 0;
    uint8_t Macro = 0;
    uint8_t MacroStage = 0;
};

//.cpp
//#include "LED_MACROS.h"
//#include "Arduino.h"

void MACROS::SetFPS(uint16_t FPS) {
  if (FPS) {
    if (FPS > 1000) {
      _Rate = 1;
    }
    else {
      _Rate = 1000 / FPS;
    }
  }
  else {
    _Rate = 0;
  }
}

void MACROS::SetVal(uint8_t Val) {
  _MacroRun = false;
  if (Val != _Val)
    _Val = Val;
  _Target = _Val;
}

void MACROS::SetDelay(uint16_t Delay) {
  if (Delay) {
    _Delay = Delay;
    _Timer = millis();
    _MacroRun = true;
  }
  else {
    _MacroRun = false;
  }
}

void MACROS::Set(uint8_t Target, uint16_t Delay) {
  if (Delay) {
    _Delay = Delay;
    _Timer = millis();
    _MacroRun = true;
  }
  else {
    _MacroRun = false;
  }
  if (Target != _Val)
    _Val = Target;
  _Target = _Val;
}

void MACROS::Fade(uint8_t Target, uint8_t Frames) {
  if (Frames) {
    if (Target > _Val) {
      _MacroRun = true;
      _Delay = _Rate;
      _Increment = (Target - _Val) / Frames;
      if (!_Increment) _Increment = 1;
      _Target = Target;
      _Timer = millis();
    }
    else if (Target < _Val) {
      _MacroRun = true;
      _Delay = _Rate;
      _Increment = (_Val - Target) / Frames;
      if (!_Increment) _Increment = 1;
      _Target = Target;
      _Timer = millis();
    }
    else {
      _MacroRun = false;
    }
  }
}

bool MACROS::Run() {
  if (_MacroRun) {
    if ((millis() - _Timer) >= _Delay) {
      if (_Val < _Target) {
        //Fading Up
        uint8_t ValHold = _Val + _Increment;
        if (ValHold >= _Target || ValHold <= _Val) {
          // Done Fading
          ValHold = _Target;
          _MacroRun = false;
        }
        _Val = ValHold;
      }
      else if (_Val > _Target) {
        //Fading Down
        uint8_t ValHold = _Val - _Increment;
        if (ValHold <= _Target || ValHold >= _Val) {
          // Done Fading
          ValHold = _Target;
          _MacroRun = false;
        }
        _Val = ValHold;
      }
      else {
        //PWM Set Macro Done
        _MacroRun = false;
      }
      _Timer = millis();
    }
  }
  return !_MacroRun;
}
#endif
