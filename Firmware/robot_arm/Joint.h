#include <Servo.h>

class Joint
{
    public:
        void Initialize(int pin, int min, int max, int startPos);
     
        void Write(int pos);
        int Update();
        
    private:
        Servo _servo;
        int _pin;
        int _min;
        int _max;
        int _speedMs;
        int _pos;
        int _currentPos;
        
        int _Limit(int value);
};
