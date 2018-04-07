/*
 * This program will calculate the velocity of a bullet using 2 IR beams as triggers.
 * It will then trigger a camera when the bullet will impact the target
 * The program assumes constance zero acceleration and neglects air resistance
 */

// Pin numbers
// Inputs
#define BUTTON 2 // #define works as shown, there is no = or ; in it.
#define TRIG_1 A0
#define TRIG_2 A1
// Outputs
#define LED 13
#define CAM 10


// Constants
#define TRIG_HIGH 0

void setup() {
  // Inputs
  pinMode(BUTTON,INPUT);
  pinMode(TRIG_1,INPUT);
  pinMode(TRIG_2,INPUT);

  // Outputs
  pinMode(LED,OUTPUT);
  pinMode(CAM,OUTPUT);
}

void loop() {

  float distTriggers = 0; // Distance between trig1 and trig2 in ft
  float distTarget = 0;   // Distance between end of muzzle to target in ft
  float distTrig1 = 0;    // Distance between muzzle and trig1 in ft

  float velocity = 0;     // Velocity of the bullet 

  unsigned long timeBetTrig = 0; // Time between triggers in microseconds
  unsigned long timeToTarget = 0; // Time from trig2 till the bullet hits the target

  bool armed = false;
  
  while(armed == false)
  {
    digitalWrite(LED,1); // keep led on while not armed
    if (digitalRead(BUTTON)== 1)
    {
      armed = true;
    }
  }

  digitalWrite(LED,0);  // Ready to fire.

  while(analogRead(TRIG_1) > TRIG_HIGH) // Wait for bullet to pass by trig1
    {
      timeBetTrig = 0;
    }
  // Trig 1 was triggered

  while(analogRead(TRIG_2) > TRIG_HIGH) // Wait for bullet to pass trig2
    {
      delayMicroseconds(1); // Wait 1 microsecond
      timeBetTrig++; // 1 microsecond passed
    }
  // Trig 2 was triggered

  velocity = calcVelocity(distTriggers,timeBetTrig); // Calculate velocity of bullet

  timeToTarget = calcTTT(velocity, distTriggers, distTrig1, distTarget); // Calculate the time left till the bullet hits the target in microseconds
  
  delayMicroseconds(timeToTarget); // Wait till the bullet hits the target

  shoot(); // Take the picture
}

float calcVelocity(float distance,unsigned long timeMicro) // takes a distance in feet and the time in microseconds to calculate the velocity in ft/sec
  {
    float timeSec = (float)timeMicro * (float)pow(10,-6);
    float velocity = distance / timeSec;
    return velocity;
  }

unsigned long calcTTT(float velocity, float distTriggers, float distTrig1, float distTarget) // Takes velocity in ft/sec, distance between triggers in ft, and distance between the muzzle to the target in ft,
                                                                           // and calculates the time left till impact in microseconds
  {
    float distLeft = distTarget - (distTriggers + distTrig1);
    float timeSec =  distLeft / velocity;
    unsigned long timeToTarget = (unsigned long)timeSec * (unsigned long)pow(10,6);
    return timeToTarget;
  }
void shoot()
  {
    digitalWrite(CAM, HIGH); // fire flash
    delay(50);
    digitalWrite(CAM, LOW);
  }

