#include "Joint.h"

#define J0_CORRECTION(q1) (((180-q1)-80)*2)
#define J1_CORRECTION(q2) ((q2*0.888889)+100)

//Board Settings
const int baud_rate = 9600;

//Pin Mappings
const int joint0_pin = 9;
const int joint1_pin = 10;
const int joint2_pin = 11;
const int joint3_pin = 12;

//Joint Settings
int delayMs = 20;
int jointReady[4] = {0,0,0,0};

//Servo declarations
Joint joint0 = Joint();
Joint joint1 = Joint();
Joint joint2 = Joint();
Joint joint3 = Joint();

//Global variables
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

//Function Definitions
String getValue(String data, char separator, int index);
int ready(int joint0, int joint1, int joint2, int joint3);

void setup() {
    Serial.begin(baud_rate);
    inputString.reserve(200);

    joint0.Initialize(joint0_pin, 0, 180, J0_CORRECTION(90)); //156
    joint1.Initialize(joint1_pin, 0, 180, J1_CORRECTION(90)); //85
    joint2.Initialize(joint2_pin, 0, 180, 70); //70
    joint3.Initialize(joint3_pin, 0, 150, 0); //0
}

void loop() {
    int armReady = 0;

    armReady = ready(joint0.Update(), joint1.Update(), joint2.Update(), joint3.Update());
    delay(delayMs);
    
    if (stringComplete) 
    {
        String joint0Str, joint1Str, joint2Str, joint3Str, delayMsStr;

        if(inputString.equalsIgnoreCase("status\n") or inputString.equalsIgnoreCase("status"))
        {
            Serial.println(armReady);
        }
        else
        {
            joint0Str = getValue(inputString, ',', 0);
            joint1Str = getValue(inputString, ',', 1);
            joint2Str = getValue(inputString, ',', 2);
            joint3Str = getValue(inputString, ',', 3);
            delayMsStr = getValue(inputString, ',', 4);

            
            joint0.Write(J0_CORRECTION(joint0Str.toInt()));
            joint1.Write(J1_CORRECTION(joint1Str.toInt()));
            joint2.Write(joint2Str.toInt());
            joint3.Write(joint3Str.toInt());
            delayMs = delayMsStr.toInt();
             
            Serial.print(joint0Str); Serial.print(" -- ");
            Serial.print(joint1Str); Serial.print(" -- ");
            Serial.print(joint2Str); Serial.print(" -- ");
            Serial.print(joint3Str); Serial.print(" -- ");
            Serial.println(delayMs);
        }
        
        // clear the string:
        inputString = "";
        stringComplete = false;
    }
}

void serialEvent() {
    while (Serial.available()) {
        // get the new byte:
        char inChar = (char)Serial.read();
        // add it to the inputString:
        inputString += inChar;
        // if the incoming character is a newline, set a flag
        // so the main loop can do something about it:
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length()-1;

    for(int i=0; i<=maxIndex && found<=index; i++){
        if(data.charAt(i)==separator || i==maxIndex){
            found++;
            strIndex[0] = strIndex[1]+1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }

    return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

int ready(int joint0, int joint1, int joint2, int joint3)
{
    return (joint0 && joint1 && joint2 && joint3);
}

