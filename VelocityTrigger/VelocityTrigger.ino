/*
 * Ethan Grey
 * This program will calculate the velocity of a bullet using 2 IR beams as triggers.
 * It will then trigger a camera when the projectile will impact the target
 * The program assumes constance zero acceleration and neglects air resistance
 */

/////////// Note: Parts of code used from other sources
// Portions of code used from EEEnthusiast / MPU-6050 Implementation: https://github.com/VRomanov89/EEEnthusiast/blob/master/MPU-6050%20Implementation/MPU6050_Implementation/MPU6050_Implementation.ino
// Video explaining the code: https://www.youtube.com/watch?v=M9lZ5Qy5S2s&t=274s
// Was used to open I2C connection with accelerometer, set the settings, and retrive and convert data


/*
LCD Circuit:
LCD RS pin to digital pin 12
LCD Enable pin to digital pin 11
LCD D4 pin to digital pin 5
LCD D5 pin to digital pin 4
LCD D6 pin to digital pin 3
LCD D7 pin to digital pin 2
Additionally, wire a 10k pot to +5V and GND, with it's wiper (output) to LCD screens VO pin (pin3). 
A 220 ohm resistor is used to power the backlight of the display, usually on pin 15 and 16 of the LCD connector
*/

// I2C uses A5 as SCL and A4 as SDA on arduino UNO
// Accelerometer wiring: Blk - grnd yel-5V wht- SCL blu- SDA

#include <LiquidCrystal.h> // Used to interface with LCD
#include <IRremote.h> // Used to read IR remote signal
#include <Wire.h> // Used for I2C communication with accelerometer

// Pin numbers
// Inputs
#define REMOTE 8 // #define works as shown, there is no = or ; in it.
#define TRIG_1 A0
#define TRIG_2 A1
// Outputs
#define CAM 13


// Constants
#define DIST_TRIGGERS 0.25f // Distance between trig1 and trig2 in ft, determined by 3D print

// Global Variables
bool fire = false; // Determines if the chronograph is in shooting or options mode
short menu = 0; // Used for determining which menu page to show
short trigHigh = 1024; // Defualt value is high to prevent missfires
short trim = 0; // used as an offset 

float distTarget = 0;   // Distance between end of muzzle to target in ft
float distTrig1 = 0;    // Distance between muzzle and trig1 in ft
// Muzzle is about 0.1541667 ft from sensor 1 if flush with last rest
float velocity = 0;     // Velocity of the bullet 

unsigned long timeBetTrig = 0; // Time between triggers in microseconds
unsigned long timeToTarget = 0; // Time from trig2 till the projectile hits the target

#define AVE_SIZE 10 // Determines how many shots to save for calculaing the average velocity
float aveArray[AVE_SIZE];
float average = 0.0f;
short shotsFired = 0;
short avePos = 0;

// IR remote
IRrecv irrecv(REMOTE);
decode_results results;
short remIn = -1;

// LCD
#define rs 12
#define en 11
#define d4 5
#define d5 4
#define d6 3
#define d7 2

LiquidCrystal lcd(rs,en,d4,d5,d6,d7); // create LiquidCrystal variable using specified pins

// Accelerometer data
// Note: Works when pins are pointing to the ground
long accelX, accelY, accelZ; // Raw data from accelerometer
float gForceX, gForceY, gForceZ, rad; // Data converted to g-force and angle in radians

// Character arrays for storing LCD input
#define SIZE 4
char lcdIn0[SIZE]; // Dist to target
char lcdIn1[SIZE]; // Dist trig 1
char lcdIn2[SIZE]; // trigger when < val
char lcdIn3[SIZE]; // trim val for adjusting the time of the shot


short row = 0; // used to select the row of LCD display
short col = 12; // used to select the column of LCD display

void setup() {
  // Inputs
  pinMode(REMOTE,INPUT);
  pinMode(TRIG_1,INPUT);
  pinMode(TRIG_2,INPUT);
  
  // Outputs
  pinMode(CAM,OUTPUT);
  
  // IR remote
  irrecv.enableIRIn(); // Start the receiver
  
  // LCD
  lcd.begin(16,2); // columns,rows
  
  lcd.home();
  lcd.print("Chronograph");
  lcd.setCursor(0,1);
  lcd.print("By Ethan Grey");
  delay(5000);
  
  empty(lcdIn0,SIZE); // Fill input arrays with spaces
  empty(lcdIn1,SIZE);
  empty(lcdIn2,SIZE);
  empty(lcdIn3,SIZE);
  lcdIn3[3] = '0'; // Set initial value to 0
  
  Serial.begin(9600); // Open serial monitor (For debugging)
  Wire.begin();
  setupMPU(); // Sets the desired settings for power managment and accelerometer, gyroscope was commented out as it is not needed
}

void loop() {
  menu = 0;
  while(!fire)  // Enter options while not ready to fire
  {
    while (menu == 0 && !fire) // First page of menu, show sensor vals
      {
        short trig1Val = analogRead(TRIG_1); // Get sensor vals
        short trig2Val = analogRead(TRIG_2);
        
        lcd.clear();
        lcd.noCursor(); // Hide cursor
        lcd.home();
        lcd.print("Trig 1: ");
        lcd.print(trig1Val); 
        lcd.setCursor(0,1);
        lcd.print("Trig 2: ");
        lcd.print(trig2Val);
        delay(500); // wait 0.5 sec
        if (readRemote() == 12) // If remote reads next, go to the next page
          {
            menu++;
          } 
        else if (readRemote() == 19)
          {
            if (checkParameters())
              {
                fire = true;
              }
            else
              {
                printError();
              }
          }
      }
    // reset input cursor pos
    row = 0;
    col = 15;
    
    while (menu == 1 && !fire) // Second page of menu, set distTarget and distTrig1
      {
        lcd.clear();
        lcd.cursor(); // Show cursor
        lcd.home();
        String tmpStr = charToString(lcdIn0,SIZE);
        lcd.print("Dist Target:");
        lcd.print(tmpStr);
        lcd.setCursor(0,1);
        tmpStr = charToString(lcdIn1,SIZE);
        lcd.print("Dist Trig 1:");
        lcd.print(tmpStr);
        lcd.setCursor(col,row); // Set cursor to user input
        getLCDInput(); // Handles user input to LCD)
        delay(500);
      }
    // reset input cursor pos
    row = 0;
    col = 15;
    
    while (menu == 2 && !fire) // Third page of menu, set trigger val
      {
        lcd.clear();
        lcd.cursor(); // Show cursor
        lcd.home();
        String tmpStr = charToString(lcdIn2,SIZE);
        lcd.print("Trigger Val:");
        lcd.print(tmpStr);
        lcd.setCursor(0,1);
        lcd.print("Trim value :");
        tmpStr = charToString(lcdIn3,SIZE);
        lcd.print(tmpStr);
        lcd.setCursor(col,row); // Set cursor to user input
        getLCDInput(); // Handles user input to LCD)
        delay(500);
      }
      
    while (menu == 3 && !fire)
      {
        if (shotsFired < AVE_SIZE) // If we havn't filled the average array yet
          {
            float sum = 0;
            for (short c = 0; c < shotsFired; c++) // Add all the shots taken so far
              {
                sum = sum + aveArray[c];
              }
            average = (sum / shotsFired); // average them
          }
        else
          {
            float sum = 0;
            for (short c = 0; c < AVE_SIZE; c++) // Add all the shots saved
              {
                sum = sum + aveArray[c];
              }
            average = (sum / AVE_SIZE); // average them
          }
          
        lcd.clear();
        lcd.noCursor();
        lcd.home();
        lcd.print("Shots Fired:"); // Show how many shots have been fired
        lcd.print(shotsFired);
        lcd.setCursor(0,1);
        lcd.print("Average Vel:"); // Show the average velocity
        lcd.print(average);
        
        if (readRemote() == 11) // Prev
          {
            menu--;
          }
        else if (readRemote() == 19)
          {
            if (checkParameters())
              {
                fire = true;
              }
            else
              {
                printError();
              }
          }
        delay(500);
      }
  }
  // Fire mode
  timeBetTrig = 0;
  unsigned long time1 = 0;
  unsigned long time2 = 0;
  lcd.clear();
  lcd.home();
  lcd.print("Fire mode on");
  
  Serial.print("trigHigh: ");
  Serial.println(trigHigh);
  
  RESTART:
  while(fire && analogRead(TRIG_1) > trigHigh) // Wait for projectile to pass by trig1
    {
      if(readRemote() == 10) // If EQ (menu button) was pressed, go to options
        {
          fire = false;
        }  
      else
        {
          recordAccelRegisters(); // Read accelerometer to get the angle of the shot
        }
    }
  
  if (fire)
  {
  // Trig 1 was triggered
  time1 = micros(); // Record the time trig1 was triggered
  
  //lcd.clear();
  //lcd.home();
  //lcd.print("Trig 1");
  //lcd.setCursor(0,1);
  //lcd.print("Triggered");
  
  while(analogRead(TRIG_2) > trigHigh) // Wait for projectile to pass trig2
    {
      // Do nothing while waiting
    }
  // Trig 2 was triggered
  time2 = micros(); // Record the time that trig 2 was triggered
  timeBetTrig = time2 - time1; // Take the difference between the triggers 

  //Serial.print("timeBetTrig(micro seconds): ");
  //Serial.println(timeBetTrig);
  
  //lcd.clear();
  //lcd.home();
  //lcd.print("Trig 2");
  //lcd.setCursor(0,1);
  //lcd.print("Triggered");

  velocity = calcVelocity(DIST_TRIGGERS,timeBetTrig); // Calculate velocity of projectile
  if (velocity < 1)
    {
      goto RESTART;
    }
  velocity = velocity * cos(rad); // Convert velocity to its horizontal component

  timeToTarget = calcTTT(velocity, DIST_TRIGGERS, distTrig1, distTarget); // Calculate the time left till the projectile hits the target in microseconds
  //Serial.print("timeToTarget (microSeconds): ");
  //Serial.println(timeToTarget);
  //Serial.print("trim: ");
  //Serial.println(trim);
  unsigned long impactTime = micros() + timeToTarget + trim; // Set the impact time
  //unsigned long endTime = 0;
  //Serial.print("impactTime: ");
  //Serial.println(impactTime);
  while (micros() < impactTime) // Wait until impact time
    {
      // Wait for impact
    }
  //endTime = micros();
  //Serial.print("time: ");
  //Serial.println(endTime);
  //Serial.print("Difference = ");
  //Serial.println(endTime - impactTime);
  
  shoot(); // Take the picture
  
  shotsFired++;
  
  if (avePos == AVE_SIZE - 1) // Check if we met the size limit for the average array
    {
      avePos = 0;  // If we did, set the position to the begging
    }
  aveArray[avePos] = velocity; // store the last shot's velocity for later use
  
  avePos++;
  
  if(readRemote() == 10) // If EQ (menu button) was pressed, go to options
  // Placed again here so if trigVal was set too low, you can still get to the menu
      {
        fire = false;
      }  
      
   lcd.clear();
   lcd.home();
   lcd.print("Velocity:");
   lcd.setCursor(0,1);
   lcd.print(velocity);
   lcd.print(" ft/s");
   delay(3000); // Wait 3 seconds
  }
}

float calcVelocity(float distance,unsigned long timeMicro) // takes a distance in feet and the time in microseconds to calculate the velocity in ft/sec
  {
    float timeSec = (float)timeMicro * (float)pow(10,-6);
    Serial.print("timeSec: ");
    Serial.println(timeSec);
    float velocity = distance / timeSec;
    Serial.print("distance: ");
    Serial.println(distance);
    Serial.print("velocity: ");
    Serial.println(velocity);  
    return velocity;  
  }

unsigned long calcTTT(float velocity, float dist_triggers, float distTrig1, float distTarget) // Takes velocity in ft/sec, distance between triggers in ft, and distance between the muzzle to the target in ft,
                                                                           // and calculates the time left till impact in microseconds
  {
    float distLeft = distTarget - (dist_triggers + distTrig1);
    float timeSec =  distLeft / velocity;
    unsigned long timeToTarget = (unsigned long)timeSec * (unsigned long)pow(10,6);
    return timeToTarget;
  }
void shoot()
  {
    
    digitalWrite(CAM, HIGH); // fire flash
    delay(500);
    digitalWrite(CAM, LOW);
  }
  
short readRemote()
  {
    short input = -1; // -1 indicates a value we wern't looking for
    
    if (irrecv.decode(&results)) // if we recive an IR signal
    {
     Serial.println(results.value, HEX);
     switch (results.value) // Check what button was pressed
     {
       case 0xFF6897: // 0
       input = 0;
         break;
       case 0xFF30CF: // 1
       input = 1;
         break;
       case 0xFF18E7: // 2
       input = 2;
         break;
       case 0xFF7A85: // 3
       input = 3;
         break;
       case 0xFF10EF: // 4
       input = 4;
         break;
       case 0xFF38C7: // 5
       input = 5;
         break;
       case 0xFF5AA5: // 6
       input = 6;
         break;
       case 0xFF42BD: // 7
       input = 7;
         break;
       case 0xFF4AB5: // 8
       input = 8;
         break;
       case 0xFF52AD: // 9
       input = 9;
         break;
       case 0xFF906F: // EQ
       input = 10;
         break;
       case 0xFF22DD: // Prev
       input = 11;
         break;
       case 0xFF02FD: // Next
       input = 12;
         break;
       case 0xFF629D: // CH
       input = 13;
         break;
       case 0xFFE01F: // -
       input = 14;
         break;
       case 0xFFA857: // +
       input = 15;
         break;
       case 0xFF9867: // 100+
       input = 16;
         break;
       case 0xFFA25D: // CH-
       input = 17;
         break;
       case 0xFFE21D: // CH+
       input = 18;
         break;
       case 0xFFC23D: // Play/Pause
       input = 19;
         break;
       case 0xFFB04F:
       input = 20;
         break;
     }
     
     irrecv.resume(); // Receive the next value
    }
     return input; // Return the button that was pressed
  }
  
void getLCDInput()
  {
    short val = readRemote();
    Serial.println("val = " + val);
    switch (val)
          {
            case 0:
              if (menu == 1)
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn0, SIZE, col - 12, '0');
                        break;
                      case 1:
                        insert(lcdIn1, SIZE, col - 12, '0');
                        break;
                    }
                }
                  else
                    switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '0');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '0');
                        break;
                    }
                
              break;
            case 1:
              if (menu == 1)
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn0, SIZE, col - 12, '1');
                        break;
                      case 1:
                        insert(lcdIn1, SIZE, col - 12, '1');
                        break;
                    }
                }
              else
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '1');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '1');
                        break;
                    }
                }
              break;
            case 2:
              if (menu == 1)
                {                
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn0, SIZE, col - 12, '2');
                        break;
                      case 1:
                        insert(lcdIn1, SIZE, col - 12, '2');
                        break;
                    }
                }
              else
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '2');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '2');
                        break;
                    }
                }
              break;
            case 3:
              if (menu == 1)
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn0, SIZE, col - 12, '3');
                        break;
                      case 1:
                        insert(lcdIn1, SIZE, col - 12, '3');
                        break;
                    }
                }
              else
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '3');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '3');
                        break;
                    }
                }
              break;
            case 4:
              if (menu == 1)
              {
                switch (row)
                  {
                    case 0:
                      insert(lcdIn0, SIZE, col - 12, '4');
                      break;
                    case 1:
                      insert(lcdIn1, SIZE, col - 12, '4');
                      break;
                  }
              }
            else
              {
                switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '4');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '4');
                        break;
                    }
              }
              break;
            case 5:
              if (menu == 1)
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn0, SIZE, col - 12, '5');
                        break;
                      case 1:
                        insert(lcdIn1, SIZE, col - 12, '5');
                        break;
                    }
                }
              else
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '5');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '5');
                        break;
                    }
                }
              break;
            case 6:
              if (menu == 1)
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn0, SIZE, col - 12, '6');
                        break;
                      case 1:
                        insert(lcdIn1, SIZE, col - 12, '6');
                        break;
                    }
                }
              else
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '6');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '6');
                        break;
                    }
                }
              break;
            case 7:
              if (menu == 1)
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn0, SIZE, col - 12, '7');
                        break;
                      case 1:
                        insert(lcdIn1, SIZE, col - 12, '7');
                        break;
                    }
                }
              else
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '7');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '7');
                        break;
                    }
                }
              break;
            case 8:
              if (menu == 1)
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn0, SIZE, col - 12, '8');
                        break;
                      case 1:
                        insert(lcdIn1, SIZE, col - 12, '8');
                        break;
                    }
                }
              else
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '8');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '8');
                        break;
                    }
                }
              break;
            case 9:
              if (menu == 1)
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn0, SIZE, col - 12, '9');
                        break;
                      case 1:
                        insert(lcdIn1, SIZE, col - 12, '9');
                        break;
                    }
                }
              else
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '9');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '9');
                        break;
                    }
                }
              break;
            case 11: // Prev
              menu--;
              break;
            case 12: // Next
              menu++; 
              break;
            case 13: // CH
              if (menu == 1)
                {
                  switch (row)
                    {
                      case 0:
                        clearChar(lcdIn0, col - 12, SIZE);
                        break;
                      case 1:
                        clearChar(lcdIn1, col - 12, SIZE);
                        break;
                    }
                }
              else
                {
                  switch (row)
                    {
                      case 0:
                        clearChar(lcdIn2, col - 12, SIZE);
                        break;
                      case 1:
                        clearChar(lcdIn3, col - 12, SIZE);
                        break;
                    }
                }
              break;
            /*
            case 14: // - (move cursor left)
              if (col > 12 ) // Make sure user dosn't pass input area
                {
                  col--;
                }
              break;
            case 15: // + (move cursor right)
              if (col < 15) // Make sure user dosn't pass input area
                {
                  col++;
                }
              break;
            */ // Removed changing cursor position
            case 16: // 100+ (add decimal point)
              if (menu == 1)
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn0, SIZE, col - 12, '.');
                        break;
                      case 1:
                        insert(lcdIn1, SIZE, col - 12, '.');
                        break;
                    }
                }
              else
                {
                  switch (row)
                    {
                      case 0:
                        insert(lcdIn2, SIZE, col - 12, '.');
                        break;
                      case 1:
                        insert(lcdIn3, SIZE, col - 12, '.');
                        break;
                    }
                }
              break;
            case 17: // CH-  (move cursor down)
              if (row == 0) // Make sure user dosn't pass input area
                {
                  row++;
                  col = 15;
                }
              break;
            case 18: // CH+ (move cursor up)
              if (row == 1) // Make sure user dosn't pass input area
                {
                  row--;
                  col = 15;
                }
              break;  
            case 19: // Play/Pause (enter fire mode)
              if (checkParameters()) // If we have all needed variables
                {
                  fire = true; // Set fire mode
                }          
              else // We don't have needed variables
                {
                  printError();
                }
                break;
            case 20:
              if (menu == 2 && row == 1)
                {
                  insert(lcdIn3, SIZE, col - 12, '-');
                }
          }
  }
  
void clearChar(char array[], short pos, short length)
    {
       for (short c = pos; c >= 0; c--)
         {
           if (c != 0) // if c isn't the first value in the array
             {
               array[c] = array[c-1]; // copy the value to the left of the array
             }
           else // c is the first value in the array
             {
               array[c] = ' '; // clear the value
             }
         }
       
    }

String charToString (char array[], short length)
  {
    String str;
    for (short c = 0; c < length; c++)
      {
        str = str + array[c];
      }
    return str;
    
  }

void empty (char array[], short length)
  {
    for (int c = 0; c < length; c++)
      {
        array[c] = ' ';
      }
  }

short charToShort (char array[], short length)
  {
    bool negative = false;
    bool hasVal = false; // used to determine if we have a value
    bool error = false;
    short num = 0;
    short results = 0;
    for (short c = 0; c < length; c++)
      {
        results = charToNum(array[c]);
        if (results == -1 || results == 10) // If char is a not digit or is a .
          {
            Serial.print("results: ");
            Serial.println(results);
            
            Serial.print("hasVal: ");
            Serial.println(hasVal);
            
            //Serial.print("finished: ");
            //Serial.println(finished);
            
            error = true;
          }
        else if (results == 11) // If it is a space
          {
            // Do nothing
          }
        else if (results == 20) // If it is a -
          {
            negative = true;
          }
        else
          {
            num = num + (results * pow(10,(length - 1) - c)); // add the number to the sum
            hasVal = true; // We have at least one value
            Serial.println("hasVal = true");
          }
      }
    if (!error && hasVal) // If there was no error
      {
        if (negative)
          {
            num = num * -1;
          }
        return num; // Return the number
      }
    else
      {
        return -1; // return error code
      }
  }
  
float charToFloat (char array[], short length)
  {
    bool hasVal = false; // used to determine if we have a value
    bool error = false;
    bool decimal = false;
    short decimalPos = 0;
    float num = 0;
    short results = 0;
    
    for (short c = 0; c < length; c++)
      {
        results = charToNum(array[c]);
        if ((results == -1) || (results == 10 && decimal)) // If char is a not digit, another . is recived
          {
            Serial.print("results: ");
            Serial.println(results);
            
            Serial.print("hasVal: ");
            Serial.println(hasVal);
            
            //Serial.print("finished: ");
            //Serial.println(finished);
            
            error = true;
          }
        else if ((results == 10)) // if we get a .
          {
            decimal = true;
            decimalPos = c;
          }
        else if (decimal) // If we have recived a . and are not done
          {
            num = num + (results * pow(10,-1 *(c - decimalPos))); // add the number to the sum
            hasVal = true; // We have at least one digit
            Serial.println("hasVal = true");
          }
        else if (results == 11) // We recived a space
          {
            // Do nothing
          }
        else
          {
            if (c !=0)
            {
              num = num * 10 + results; // add the number to the sum
            }
            else
            {
              num = num + results;
            }
            hasVal = true; // We have at least one digit 
            Serial.println("hasVal = true");
          }
      }
    if (!error && hasVal) // If there was no error and we had a value
      {
        return num; // Return the number
      }
    else
      {
        return -1; // return error code
      }
  }

short charToNum(char character)
  // Converts character to digit or . 
  // -1 indicates neither
  // 10 indicates .
  // 11 indicates ' '
  {
    short num = -1; // Set num to -1 to indicate not a number
    if ((character >= 48 && character <= 57) || character == 46 || character == ' ' || character == '-') // If character is a digit, a . , or ' '
      {
        if (character == 46)
          {
            num = 10;
          }
        else if (character == ' ')
          {
            num = 11;
          }
        else if (character == '-')
          {
            num = 20;
          }
        else 
          {
            num = character - 48; // Convert ASCII to digit
          }
      }
    else // character is not a digit or a .
      {
        num = -1;
        Serial.print(character);
        Serial.println(" is not a number, . , or ' '");
      }
    Serial.print("num: ");
    Serial.println(num);
    return num;
  }
  
bool checkParameters()
  {
    Serial.println("Start check");
    bool params = true; // have all paramaters default
    // temp values so the globals arn't affected
    float distTar = charToFloat(lcdIn0,SIZE);
    float distTrig = charToFloat(lcdIn1,SIZE);
    short trigVal = charToShort(lcdIn2,SIZE);
    short trimVal = charToShort(lcdIn3,SIZE); 
 
    //short trigHigh = 1024; // Defualt value is high to prevent missfires
    //short trim = 0; // used as an offset 
    
    //float distTarget = 0;   // Distance between end of muzzle to target in ft
    //float distTrig1 = 0;    // Distance between muzzle and trig1 in ft   
    
    if(distTar != -1) // Check Dist Target
      {
        distTarget = distTar;
      }
    else
      {
        params = false;
      }
      
    if(distTrig != -1) // Check Dist Trig 1
      {
        distTrig1 = distTrig;
      }
    else
      {
        params = false;
      }
      
    if (trigVal != -1) // Check Trig Val
      {
        trigHigh = trigVal + 1; // When read, trigVal is always 1 short for some reason. This is a temp fix until the cause is determined.
      }
    else
      {
        params = false;
      }
      
    if (trimVal != -1) // Check Trim
      {
        trim = trimVal;
      }
    else
      {
        params = false;
      }
    Serial.println("End check");
    
    Serial.print("distTar: ");
    Serial.println(distTar);
    
    Serial.print("distTrig: ");
    Serial.println(distTrig);
    
    Serial.print("trigVal: ");
    Serial.println(trigVal);
    
    Serial.print("trimVal: ");
    Serial.println(trimVal);
    
    Serial.print("params = ");
    Serial.println(params);
    
    return params;
  }
void printError()
  {
    Serial.println("Start error");
    lcd.clear(); // Clear the lcd
    lcd.noCursor();
    lcd.home(); // Set cursor to home position
    lcd.print("Error in values"); // Print error message
    Serial.println("Start delay");
    delay(3000); // Wair for 3 seconds
    Serial.println("End error");
    return;
  }
  
void insert(char array[], short length, short pos, char charToInsert)
  {
    if (array[0] == ' ') // If the first item in the array is empty (array is not full)
      {
        for (short c = 0; c < length; c++)
          {
            if (c != length - 1) // if we are not at the end of the array
              {
                array[c] = array[c+1]; // copy the character to the right
              }
            else // we are at the end of the array
              {
                array[c] = charToInsert; // insert the character
              } 
          }
      }
    else
      {
        lcd.clear();
        lcd.home();
        lcd.print("Value full!");
        delay(2000);
      }
  }
unsigned long microToMilli(unsigned long num)
  {
    unsigned long result = num * pow(10,-3);
    return result;
  }

void setupMPU() // taken from EEEnthusiast  
{
  //// Set power managment settings
  // Start transmission with MPU-6050
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  // Select register to write to
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  // Write the data to the register
  Wire.write((byte)0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9) // Code will not compile without (byte)
  // Indicate the end of data transmission
  Wire.endTransmission(); 

  /*
  //// Setup gyroscpe
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  */
  
  //// Setup accelerometer
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write((byte)0b00000000); //Setting the accel to +/- 2g // Code will not compile without (byte)
  Wire.endTransmission(); 
}

void recordAccelRegisters() // Gets the sensor value for all 3 axis, taken from EEEnthusiast
{
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData() // Converts sensor reading into g-force, taken from EEEnthusiast
{
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;

  rad = atan(gForceY/gForceX);
}
