 /*
 * Pin layout used:
 * ------------------------------------------------
 *             MFRC522      Arduino       Arduino
 *             Reader/PCD   Uno/101       Mega
 * Signal      Pin          Pin           Pin
 * ------------------------------------------------
 * RST/Reset   RST          9             5
 * SPI SS 1    SDA(SS)      ** custom, take a unused pin, only HIGH/LOW required **
 * SPI SS 2    SDA(SS)      ** custom, take a unused pin, only HIGH/LOW required **
 * SPI MOSI    MOSI         11 / ICSP-4   51
 * SPI MISO    MISO         12 / ICSP-1   50
 * SPI SCK     SCK          13 / ICSP-3   52
 */
 
#include "Wire.h"
#include <SPI.h>
#include <MFRC522.h>

#define DS3231_I2C_ADDRESS 0x68   // I2C adress for RTC

#define RST_PIN        49         // Configurable, see typical pin layout above
#define SS_1_PIN       46         // Configurable, Data pin for RFID1
#define SS_2_PIN       48         // Configurable, Data pin for RFID2

#define SWITCH         42         //limit switch input pin (in the middle on the switch)

byte ssPins[] = {SS_1_PIN, SS_2_PIN};
MFRC522 mfrc522[2];               // Create MFRC522 instance.
String uidString;                 // RFID card ID
String card_ID = "prazno";
bool flag_l=0;                    //flag lezaljka
bool flag_s=0;                    //flag sef
bool flag_switch=0;
bool flag_motor=0;
int time_spent=0;

int enable1 = 10;   //pin 10 - bijela
int in11 = 9;       //pin 9 - zelena
int in12 = 6;       //pin 6 - plava

int enable2 = 8;   //pin 8 - zelena
int in21 = 5;       //pin 5 - plava 
int in22 = 7;       //pin 7 - ružičasta

int zadano = 0;
float degree1 = 0.0;
float degree2 = 0.0;
int encoder1PinA = 2;   // right (labeled DT on our decoder, yellow wire)
int encoder1PinB = 3;  // left (labeled CLK on our decoder, red wire)s

int encoder2PinA = 18;   // right (labeled DT on our decoder, yellow wire)
int encoder2PinB = 19;  // left (labeled CLK on our decoder, red wire)s

long int encoderPos1 = 0;  // a counter for the dial
static boolean rotating1=false;      // debounce management

long int encoderPos2 = 0;  // a counter for the dial
static boolean rotating2=false;      // debounce management

// interrupt service routine vars
boolean A_set1 = false;              
boolean B_set1 = false;

boolean A_set2 = false;              
boolean B_set2 = false;


//SETUP ***************************************************************************
void setup() {
pinMode(enable1, OUTPUT);
pinMode(in11, OUTPUT);
pinMode(in12, OUTPUT);
pinMode(enable1, OUTPUT);
pinMode(encoder1PinA, INPUT_PULLUP); // new method of enabling pullups
pinMode(encoder1PinB, INPUT_PULLUP);

pinMode(enable2, OUTPUT);
pinMode(in21, OUTPUT);
pinMode(in22, OUTPUT);
pinMode(enable2, OUTPUT);
pinMode(encoder2PinA, INPUT_PULLUP); // new method of enabling pullups
pinMode(encoder2PinB, INPUT_PULLUP); 

// ENKODER1 PREKID PIN A
attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
// ENKODER1 PREKID PIN B
attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);

// ENKODER2 PREKID PIN A
attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder2A, CHANGE);
// ENKODER2 PREKID PIN B
attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoder2B, CHANGE);
  Wire.begin();                   // Begin I2C communication
  setDS3231time(0,0,0);           // DS3231 seconds, minutes, hours
  
  pinMode(OUTPUT, SWITCH);
  
  Serial.begin(115200);           // Initialize serial communications with the PC
  while (!Serial);                // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  SPI.begin();                    // Init SPI bus

  for (uint8_t reader = 0; reader < 2; reader++) {
    mfrc522[reader].PCD_Init(ssPins[reader], RST_PIN); // Init each MFRC522 card
    Serial.print(F("Reader "));
    Serial.print(reader);
    Serial.print(F(": "));
    mfrc522[reader].PCD_DumpVersionToSerial();
  }
Serial.println("setup done");
}


//loop ******************************************************************************************************************
void loop() {

  if (Serial.available() > 0){
    zadano=Serial.parseInt();
    Serial.print("zadano = ");
    Serial.println(zadano);
  }

  for (uint8_t reader = 0; reader < 2; reader++) {
    // Look for new cards

    if (mfrc522[reader].PICC_IsNewCardPresent() && mfrc522[reader].PICC_ReadCardSerial()) {
      Serial.print(F("Reader "));
      Serial.print(reader);
      // Show some details of the PICC (that is: the tag/card)
      Serial.print(F(": Card UID:"));
      uidString = String(mfrc522[reader].uid.uidByte[0])+" "+String(mfrc522[reader].uid.uidByte[1])+" "+String(mfrc522[reader].uid.uidByte[2])+ " "+String(mfrc522[reader].uid.uidByte[3]);
      Serial.print(" ");
      Serial.println(uidString);
      //Serial.print(F("PICC type: "));
      //MFRC522::PICC_Type piccType = mfrc522[reader].PICC_GetType(mfrc522[reader].uid.sak);
      //Serial.println(mfrc522[reader].PICC_GetTypeName(piccType));

      if (reader == 0) {                    //RFID od lezaljke, ako je prazna
        if (flag_l==0){
          motor_control(zadano);
          //call_xbee()
          //open_l()
          flag_l=true;
          card_ID=uidString;
          setDS3231time(0,0,0);           //za početak ležanja postavi nule
          Serial.print("zauzetost ležaljke: ");
          Serial.println(flag_l);
          Serial.print("card_ID: ");
          Serial.println(card_ID);
          Serial.println("");
          delay(100);

        }
        else if (card_ID==uidString) {    //RFID od lezaljke, kraj koristenja
          motor_control(0);
          flag_l=0;
          flag_s=0;
          displayTime();   
          card_ID="prazno";
          Serial.println("sef se otvara..");
          Serial.print("zauzetost ležaljke: ");
          Serial.println(flag_l);
          Serial.print("card_ID: ");
          Serial.println(card_ID);
          Serial.println("");
          delay(10);
        }
        else {
          Serial.println("Ležaljka je već zauzeta");
          Serial.println("");
        }
      }
      
      if (reader == 1){
        if (card_ID==uidString && flag_s==0){
          //open_safe();
          flag_s=1;
          Serial.print("sef se otvara, flag: ");
          Serial.println(flag_s);
          Serial.print("card_ID: ");
          Serial.println(card_ID);
        }
        else if(card_ID!=uidString)
          Serial.println("neautorizirano pokušaj otvaranja sefa!");
      }
      // Halt PICC
      mfrc522[reader].PICC_HaltA();
      // Stop encryption on PCD
      mfrc522[reader].PCD_StopCrypto1();
    } //if (mfrc522[reader].PICC_IsNewC
  } //for(uint8_t reader
  if (flag_l == 1){
    flag_switch = digitalRead(SWITCH);
    Serial.print(flag_switch);
    if (flag_switch){
    //motor()
    flag_motor=1;
    }
  }

delay(5);
}



//kontrola motora **************************************************************************************
void motor_control(int zadano){
while (degree1 > (zadano+5) || degree1 < (zadano-5) || degree2 > (zadano+5) || degree2 < (zadano-5)){     //ako jedan od 2 nije odradio posao
   if(degree1 > (zadano+5) || degree1 < (zadano-5)){                    //ako prvi nije odradio
      if (degree1<zadano){
          digitalWrite(in11, HIGH);
          digitalWrite(in12, LOW);
          analogWrite(enable1, 250);
      }
      else if (degree1>zadano){
          digitalWrite(in11, LOW);
          digitalWrite(in12, HIGH);
          analogWrite(enable1, 250);
      }
  }
  else{
      digitalWrite(in11, LOW);
      digitalWrite(in12, HIGH);
      analogWrite(enable1, 0);
  }
  if(degree2 > (zadano+5) || degree2 < (zadano-5)){                     //ako drugi nije odradio
  
     if (degree2<zadano){
        digitalWrite(in21, LOW);
        digitalWrite(in22, HIGH);
        analogWrite(enable2, 250);
    }
    else if (degree2>zadano){
        digitalWrite(in21, HIGH);
        digitalWrite(in22, LOW);
        analogWrite(enable2, 250);
    }
  }
  else{
      digitalWrite(in21, LOW);
      digitalWrite(in22, HIGH);
      analogWrite(enable2, 0);
  }
delay(2);
}                                                                             //gasi oba motora
      digitalWrite(in11, LOW);
      digitalWrite(in12, HIGH);
      analogWrite(enable1, 0);
      
      digitalWrite(in21, LOW);
      digitalWrite(in22, HIGH);
      analogWrite(enable2, 0);
rotating1 = true;
rotating2 = true;
}



////funkcije RTC ************************************************************************
//
void setDS3231time(byte second, byte minute, byte hour) {
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(0)); // set day of week
  Wire.write(decToBcd(0)); // set date
  Wire.write(decToBcd(0)); // set month
  Wire.write(decToBcd(0)); // set year
  Wire.endTransmission();
}


int readDS3231time(byte *second, byte *minute, byte *hour){
  int spent;
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  spent=*minute+*hour*60;
  Serial.print("Vrijeme provedeno (u min): ");
  Serial.println(spent);
}


void displayTime(){
  byte second, minute, hour, time_spent;
  // retrieve data from DS3231
  time_spent=readDS3231time(&second, &minute, &hour);
  // send it to the serial monitor
  Serial.print("vrijeme provedeno (h: min: s)");
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  /*if (minute<10)
  {
    Serial.print("0");
  }*/
  Serial.print(minute, DEC);
  Serial.print(":");
  /*if (second<10)
  {
    Serial.print("0");
  }*/
  Serial.println(second, DEC);
}


// Convert binary coded decimal to normal decimal numbers and inverse
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}


// Encoder interrupts *******************************************************************
//prekidi za enkoder motora A
void doEncoder1A(){

  // Test transition, did things really change? 
  if( digitalRead(encoder1PinA) != A_set1 ) {  // debounce once more
    A_set1 = !A_set1;

    // adjust counter + if A leads B
    if ( A_set1 && !B_set1 ) 
      encoderPos1++;
      Serial.print("encoderPos1 raste ");
      Serial.println(encoderPos1);
  }
      degree1 = 6.25*float(encoderPos1);
      rotating1 = false;  // no more debouncing until loop() hits again
}
// PREKIDNI POTPROGRAM PINA  B
void doEncoder1B(){

  if( digitalRead(encoder1PinB) != B_set1 ) {
    B_set1 = !B_set1;
    //  adjust counter - 1 if B leads A
    if( B_set1 && !A_set1 ){
      encoderPos1--;
      Serial.print("encoderPos1 pada ");
      Serial.println(encoderPos1);
    }
      rotating1 = false;
      degree1 = 6.25*float(encoderPos1);

  }
}

void doEncoder2A(){

  // Test transition, did things really change? 
  if( digitalRead(encoder2PinA) != A_set2 ) {  // debounce once more
    A_set2 = !A_set2;

    // adjust counter + if A leads B
    if ( A_set2 && !B_set2 ) 
      encoderPos2++;
      Serial.print("encoderPos2 raste ");
      Serial.println(encoderPos2);
  }
      degree2 = -6.25*float(encoderPos2);
      rotating2 = false;  // no more debouncing until loop() hits again
}
// PREKIDNI POTPROGRAM PINA  B
void doEncoder2B(){
  
  if( digitalRead(encoder2PinB) != B_set2 ) {
    B_set2 = !B_set2;
    //  adjust counter - 1 if B leads A
    if( B_set2 && !A_set2 ){
      encoderPos2--;
      Serial.print("encoderPos2 pada ");
      Serial.println(encoderPos2);
    }
      rotating2 = false;
      degree2 = -6.25*float(encoderPos2);

  }
}
