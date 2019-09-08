#include "Joint.h"
#include <Arduino.h>

void Joint::Initialize(int pin, int min, int max, int startPos)
{
    _pin = pin;
    _min = min;
    _max = max;
    _pos = _Limit(startPos);
    _currentPos = _Limit(startPos);
    
    _servo.attach(_pin);
}
     
void Joint::Write(int pos)
{
    _pos = _Limit(pos);
}

int Joint::Update()
{
    int status = 0;
    
    if(_currentPos < _pos)
    {
        _currentPos += 1;
    }
    else if(_currentPos == _pos)
    {
        _currentPos = _pos;
        status = 1;
    }
    else if(_currentPos > _pos)
    {
        _currentPos -= 1;
    }
    
    _servo.write(_currentPos);

    return status;
}
        
int Joint::_Limit(int value)
{
    return int(constrain(value, _min, _max));
}
