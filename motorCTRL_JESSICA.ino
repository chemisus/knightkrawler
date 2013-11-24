#include<Math.h>

/*
    I still need to fix some of this code since we took out the Upper Limit switch
    The Pin numbers will also probably be changed whenever we get everything together.
    anywhere you see LS that just means Limit Switch. 
    Don't judge me for bad code. I haven't had a chance to sit down and finish it :-P 
*/
    
int joystick_UPDN = 0;  //Up/Down Joystick (0-499==Move Down, 501-1023==Move UP);
int motorSpeed = 0;    //TX Output to motor controller
int LSUP = 0;
int LSDN = 0;
int BTN1 = 6;      //Button for raising/lowering the platform
int LSUP_PIN = 2;  //Limit Switch on top of platform
int LSDN_PIN = 3;  //Limit Switch on bottom of platform
int RELAY_UP = 4;  //Relay for Raising the platform
int RELAY_DN = 5;  //Relay for lowering the platform
int joystick_HOME = 500; //Middle point for joystick


void setup1 () {
  Serial.begin(9600);    //Initiate Morgan Freeman's Voice
  pinMode(LSUP_PIN, INPUT_PULLUP);  //use PULLUP to avoid external resistors (because were lazy) 
  pinMode(LSDN_PIN, INPUT_PULLUP);
  pinMode(RELAY_UP, OUTPUT);
  pinMode(RELAY_DN, OUTPUT);
  
  //Shut off relays on startup for safety 
  digitalWrite(RELAY_UP, HIGH);
  digitalWrite(RELAY_DN, HIGH);
}

void loop1 ()
{
  /************* Forward/Reverse Controls ******************/
  
  if (!LSUP && LSDN) //only allow movement if the platform is UP 
  {
    joystick_UPDN = analogRead(A0);
    motorSpeed = map(joystick_UPDN, 0, 1023, 0, 255);
    Serial.write(motorSpeed);    //Send TittySprinkles to the motor controller 
                                //(0-149 Reverse, 150 Stop, 151-255 Forward) 
  }
  
  
  /****************** Raise/Lower Platform **************/
//Lower Platform
    if (!LSUP && LSDN && !BTN1)
    {
       while(LSDN) //continue lowering until bottom limit switch is hit
        {
          LSDN = digitalRead(LSDN_PIN);
          digitalWrite(RELAY_UP, LOW);
          digitalWrite(RELAY_DN, HIGH);
        }
    }
    
//Raise Platform
    else if (LSUP && !LSDN && !BTN1)
    {  
      while(LSUP) //continue raising until top limit switch is hit
      {
        LSUP = digitalRead(LSUP_PIN);
        digitalWrite(RELAY_UP, HIGH);
        digitalWrite(RELAY_DN, LOW);
      }
    }
    
//Shut off Relays for safety purposes
    else
    {
      digitalWrite(RELAY_UP, HIGH);
      digitalWrite(RELAY_DN, HIGH);
    }
           
    delay(100);
    Serial.flush();
}

//If there is one thing in life I learned that I enjoy
//    ...It's walking in a womens underwear (preferably my girlfriends)
//                --Terrence Howard 
//                         -- Brandon Frazer
//                                                                                                                                                                                                          -- Michael Scott
