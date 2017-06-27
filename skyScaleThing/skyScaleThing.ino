
/*
 * 
 * Start towing ir Stop towing naudoti du atskirus mygtukus
 * Auto retrieve veikia iki paklaidos (pvz -100m +- dar paklaida nuo isvynioto ilgio) nuo likusio sniuro iki prdzios...
 * Po to retrievui naudoti manual buttona, kol paspaustas vyniojimas veikia, atleistas - vyniojimas stabdomas 
 * - retrievui - inputai skirtingi (auto/manual), outputas i rele - tas pats
 * 
1. svarstykles  (CLK/DT) - 2, 3  pin, 3=>17/ spalvos: apkorovos celes iejimas: zalias-A+, baltas-A-, juodas-E-, raudonas-E+/ Raudonas, juodas, baltas, zalias
2. sistemos startui/calibravimui 4 pin
3. speedometrui - 5 pin, startas uzduodamas  gavus pirma signala, pradedamas skaiciuoti greitis ir sniuro issyvyniojimo atstumas
4. traukos relei 4 IN is mygtuko +/- bus 6/7 pin  - dedati pvz i scetch'a pasijungus ant reles ir isitikinti kad veikia, esanst paspaudimui i cosole printinti rezultata
5. traukos relei OUT is arduino IN i rele +/- bus 8/9 pin  - dedati pvz i scetch'a pasijungus ant reles ir isitikinti kad veikia, esanst paspaudimui i cosole printinti rezultata
6. sniuro vyniojimui mygtukas IN i arduino bus 11 pin
7. sniuro vyniojimui OUT is qruino IN i rele 10 pin
8. sniuro vyniojimo priverstinis sustabdymas - mygtukas 12 pin // bus pajungtas veliau
9. sistemos restartui mygtukas - arduino reset
10. bugno sukimosi INPUT PIN 12 is gerkono nuo skriemulio

* 11. bugno sukimuisi INPUT - o OUTPUT buzzer/sviesdiodis , LED output pin 13 (situos du galima pajungti ne prie qrduino o tiesiog atskirai)

*** Paspaudus stop, traukos nuemimas prasideda iki minimumo,  kol cele rodo <= 0 arba kol gaunamas signalas is variklio kad jis krastineje padetyje
*** bus du galiniai input mygtukai ant traukos variklio, reikia rezervuoti inputus i arduino, kad nebeleisti didinti/mazinti traukos, 

*** TODO: nedaleisti programiskai kad vienu metu trauka butu ir didinama ir mazinama

rezultato pvz:


Trauka: 21.1 kg, virves greitis: 10 km/h, isivyniojo: +243 m, (liko: 1350 m), Trauka: (neutrali/didina/mazina), Virves vyniojimas: neutralus/Ijungtas ()

 */

#include "HX711.h"
#include <LiquidCrystal.h>
 
/* Segment byte maps for numbers 0 to 9 */
//const byte SEGMENT_MAP[] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0X80,0X90};
/* Byte maps to select digit 1 to 4 */
//const byte SEGMENT_SELECT[] = {0xF1,0xF2,0xF4,0xF8};
 

#define DOUT  3 //17 //3 = > 17
#define CLK  2
#define BTN_START_PIN 14 // 4
#define SPEED_PIN  16 //15 // 5 => 15 sfor sparkfun
#define BTN_TRACK_INCREASE_PIN  12 // 6 //36 // 6 =>36
#define BTN_TRACK_DECREASE_PIN  13 // 7 //37 // 7 =>37
//#define BTN_TRACK_MIN_MAX_PIN  38 // traukos varikliuko galinukai
#define OUTPUT_RELAY_TRACK_INCREASE_PIN  18 // 8 //32 // in current scetch output is directly without resitors connected to PIN 8 =>34
#define OUTPUT_RELAY_TRACK_DECREASE_PIN  19 // 9 //33 // in current scetch output is directly without resitors connected to PIN 9 => 35

//#define OUTPUT_RELAY_REEL_START_PIN 17 // 27     // in current scetch output is directly without resitors connected to PIN 10
//#define BTN_REEL_START_AUTO_PIN 25 // starts auto retrieve on click
//#define BTN_REEL_START_MANUAL_PIN 15 // 26  // if auto is on - stops retrieve, else starts retrieve while is pressed

//#define BTN_REEL_SPIN_PIN_LED 12 //18 // 12 => 18 - bugno sukimosi gerkonas
//#define LED_REEL_SPIN_PIN_BUZZER 13 //23 // 13=>23

#define longPressTimes 15
#define maxWireLength 1500
#define rollDiameterCm 10
#define minimalDelayForSpeedReel 2000

#define weightDelimiter 0.453592

#define weight 1.312345

#define PI 3.1415926535897932384626433832795

#define printOutDelay 500

#define tractionWorkingTime 50 // 100
#define timeForTractionOff 500 // 300

#define autoRetrievePressTime 5000

HX711 scale(DOUT, CLK);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

float calibration_factor = -7050; //-7050 worked for my 440lb max scale setup

boolean startBtnState = LOW;
long startBtnStateChangeTime = 0;
boolean previousStartBtnState = LOW;

int towingSymbol = 0;

boolean speedBtnState = LOW;
boolean previousSpeedBtnState = LOW;
 
int towingOn = 0;
unsigned long towingStartTime;
unsigned long statePrintOutTime;

unsigned long distanceStartTime;
unsigned long reelSpinLastTime;
long reelRotationCount = 0;

long tractionState = 0;

boolean tracIncreaseOn = LOW;
long tracIncreaseOnTime = 0;
boolean tracDecreaseOn = LOW;
long tracDecreaseOnTime = 0;
boolean tracOff = LOW;
long tracOffTime = 0;

double wireSpeed = 0;
double wireLength = 0;

double lastTraction = 0;

long reelRetrieveState = 0;
long reelRetrieveAutoState = 0;

boolean autoRetrieveState = LOW;
long autoRetrievePressTimeOn = 0;

void setup() {
  pinMode(BTN_START_PIN, INPUT);
  pinMode(SPEED_PIN, INPUT);
  pinMode(BTN_TRACK_INCREASE_PIN, INPUT);
  pinMode(BTN_TRACK_DECREASE_PIN, INPUT);
  pinMode(OUTPUT_RELAY_TRACK_INCREASE_PIN, OUTPUT);
  pinMode(OUTPUT_RELAY_TRACK_DECREASE_PIN, OUTPUT);
//  pinMode(OUTPUT_RELAY_REEL_START_PIN, OUTPUT);
//  pinMode(BTN_REEL_START_MANUAL_PIN, INPUT); // start/stop button can't be used the same one, we can use only internal state for it

  digitalWrite(BTN_START_PIN, LOW);
//  digitalWrite(BTN_REEL_START_MANUAL_PIN, LOW);
  digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, LOW);
  digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, LOW);
//  digitalWrite(OUTPUT_RELAY_REEL_START_PIN, LOW);
  digitalWrite(BTN_TRACK_INCREASE_PIN, LOW);
  digitalWrite(BTN_TRACK_DECREASE_PIN, LOW);
  digitalWrite(SPEED_PIN, LOW);
  
  Serial.begin(115200); // 9600 => 115200 for soarkfun 

  // set up the LCD's number of colum5ns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.setCursor(0,1);
  lcd.print(" SkyScale v1.0.1");
  lcd.setCursor(2,0);
  lcd.print("Waiting start!");

  startBtnStateChangeTime = millis();
  
  // scale with HX711 calibration on each start, which makes scale to ZERO possition
  resetScale();
  Serial.println("Waiting for your input");
}

void loop() {
//  tractionControlState();  
  towingOn = 1;
//  tractionControlStateSimple();
//  startButtonState();
//  wireRetrieveState();
  readSpeed(); // maybe it's better to read wire speed and state always, not according to towing is On/Off...
  
  if ((millis() - statePrintOutTime) > printOutDelay) {
    readScale();
    
    if (towingOn == 1) {
      printSpeed();
    }
    
    statePrintOutTime = millis();
    Serial.println();
  }
  
//  delay(500); // this will be removed when getting quite enough accuracy and button state management in code
}

void startButtonState()
{
  startBtnState = digitalRead(BTN_START_PIN);
  
  if (startBtnState != previousStartBtnState && startBtnState == HIGH && ((millis() - startBtnStateChangeTime) > 3000)) {
    towingOn = !towingOn;
    startBtnStateChangeTime = millis();
    if (towingOn == 1) {
      towingStartTime = millis();
      distanceStartTime = towingStartTime;
      reelSpinLastTime = distanceStartTime;
      Serial.print("Towing has started!");
      Serial.print(" Time:");
      printTime();

      lcd.setCursor(0,0);
      lcd.print (">               ");
      
      Serial.println();
    } else {
      Serial.println("Towing has stopped!");
      towingStartTime = 0;
      lcd.setCursor(0,0);
      lcd.print ("X");
    }
//    previousStartBtnState = startBtnState;
  } else {
//    previousStartBtnState = LOW;
  }
  previousStartBtnState = startBtnState;
}

void readSpeed()
{
  long currentReelSignalTime = millis();
  long timeHasPassedAfterLastSignal = currentReelSignalTime - reelSpinLastTime;

  speedBtnState = digitalRead(SPEED_PIN);
//  if (speedBtnState = HIGH) {
//    Serial.println("Sniuras")
//  }

    if (speedBtnState != previousSpeedBtnState && speedBtnState == HIGH) {

      float ropeCirmcumferenceCm = (rollDiameterCm * PI); // 2 * PI * R = PI * D
      wireLength = wireLength + (ropeCirmcumferenceCm * 0.01);
      
      if (timeHasPassedAfterLastSignal > 0) {
        // SPEED Km/h = m * 1000 / min * 60  = cm * 100 * 1000 / s * 60 * 60 = cm * 100 * 1000 / ms * 1000 * 60 * 60 = cm * 100 / ms * 3600
//        float ropeSpeed = (ropeCirmcumferenceCm * 100) / (timeHasPassedAfterLastSignal * 3600);
        float ropeSpeed = 91.4 * ropeCirmcumferenceCm / timeHasPassedAfterLastSignal;
        
        wireSpeed = ropeSpeed;  

        reelSpinLastTime = currentReelSignalTime;
      }
    } 
  else if (timeHasPassedAfterLastSignal > minimalDelayForSpeedReel) {    
     wireSpeed = 0;
  }
}

void printSpeed()
{
  Serial.print(", virves greitis: ");
  Serial.print(wireSpeed);

  lcd.setCursor(0,1);
  lcd.print ("                ");

  lcd.setCursor(10,1);
  lcd.print (wireSpeed, DEC);
  
  Serial.print(" km/h, isivyniojo: ");
  
  if (wireLength > 0.0) {
    Serial.print("+");  
  } else {
    Serial.print("-");
  }
  Serial.print(wireLength);

  lcd.setCursor(0,1);
  lcd.print (wireLength);
//  lcd.print ("Force:       ");
//  lcd.setCursor(9,0);
//  float sss = scale.get_units();
//  lcd.print (sss, 0.453592);
//  lcd.print ("150");
//  lcd.setCursor(14,0);
//  lcd.print ("kg");
  
  Serial.print("m, (liko: ");
  Serial.print(maxWireLength - wireLength);
  Serial.print(" m)");
}

void printTime()
{
  long towingTime = millis() - towingStartTime;

  long sekundes = towingTime / printOutDelay; 
  if (sekundes < 60) {
    Serial.print(sekundes);
    Serial.print(" s");  
  } else if (sekundes > 60 && sekundes < 3600) {
    long minutes = sekundes / 60;
    Serial.print(minutes);
    Serial.print(" min ");
    Serial.print(sekundes - minutes * 60);
    Serial.print(" s");
  }
}

void resetScale()
{
  Serial.println("Remove all weight from scale");
  scale.set_scale();
  scale.tare();  //Reset the scale to 0

  long zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
}

void readScale()
{
  scale.set_scale(calibration_factor); //Adjust to this calibration factor

  Serial.print("Laikas nuo starto: ");
  printTime();
  double traction = weight * scale.get_units();

  if (traction < 0) {
    Serial.print(", Neigiama:");
    Serial.println(traction);
    traction = lastTraction;
  } else {
    lastTraction = traction;
  }
  
  Serial.print(", Trauka: ");
  Serial.print(traction, weightDelimiter); // used default lbs to kg fraction (0.453592), later we need to calculate it more acuratly to the phisical error we want to have for accuracy/precision
  // NOTE: at this moment 2.5 kgs is showing 3 kg even when wheight is swinging - it shows 4 kg...
  lcd.setCursor(2,0);
  lcd.print ("Force:     ");
  lcd.setCursor(9,0);
  lcd.print ("        ");
  lcd.setCursor(9,0);
//  float sss = -weight * scale.get_units();
  lcd.print (traction, weightDelimiter);
//  lcd.print ("150");
  lcd.setCursor(14,0);
  lcd.print ("kg");
//  showTraction(scale.get_units(), 0.453592);

  
  Serial.print(" kg"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  Serial.println();
}

//void wireRetrieveState()
//{
////  Serial.print(", Virves vyniojimas:"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
//  int currentReelRetrieveState = reelRetrieveState;
//
////  if (digitalRead(BTN_REEL_START_AUTO_PIN) == HIGH && reelRetrieveState == 0) {
////    reelRetrieveAutoState = 1;
////    reelRetrieveState = 1;
////  } else vynioja
//
////#define autoRetrievePressTime 5000
////boolean autoRetrieveState = LOW;
////autoRetrievePressTimeOn
//
////
//  if (digitalRead(BTN_REEL_START_MANUAL_PIN) == HIGH && autoRetrieveState == LOW && autoRetrievePressTimeOn == 0) {
//    autoRetrievePressTimeOn = millis();
//  }
//  
//  if (reelRetrieveState == 1 && millis() - autoRetrievePressTimeOn >= autoRetrievePressTime && autoRetrievePressTimeOn > 0) {
//    autoRetrieveState = HIGH;
//    autoRetrievePressTimeOn = -1;
//  }
//
//  if (digitalRead(BTN_REEL_START_MANUAL_PIN) == LOW && autoRetrieveState == LOW) {
//    autoRetrieveState = LOW;
//    reelRetrieveState = 0;
////    Serial.print(" neutralus ");
//  } else if (digitalRead(BTN_REEL_START_MANUAL_PIN) == LOW && autoRetrieveState == HIGH) {
//      reelRetrieveState = 1;
//      autoRetrievePressTimeOn = 0;
//  } else if (digitalRead(BTN_REEL_START_MANUAL_PIN) == HIGH) {
////    Serial.print(" vynioja ");
//    if (autoRetrieveState == HIGH && autoRetrievePressTimeOn == 0) {
//      autoRetrieveState = LOW;
//      reelRetrieveState = 0;
//    } else {
//      reelRetrieveState = 1;  
//    }
//  }
//  
//  if (reelRetrieveState == 1) {
//    digitalWrite(OUTPUT_RELAY_REEL_START_PIN, HIGH);
////    Serial.print("neutralus");
//  } else {
//    digitalWrite(OUTPUT_RELAY_REEL_START_PIN, LOW);
////    Serial.print("IJUNGTAS");
//  }
//}

void tractionControlStateSimple()
{
  long currentTractionState = tractionState;

  if (!(digitalRead(BTN_TRACK_INCREASE_PIN) == HIGH && digitalRead(BTN_TRACK_DECREASE_PIN) == HIGH)) {

      if (digitalRead(BTN_TRACK_INCREASE_PIN) == HIGH && digitalRead(BTN_TRACK_DECREASE_PIN) == LOW)
      {
      Serial.println(" reikia didinati ");
        currentTractionState++;   
      } else 
      if (digitalRead(BTN_TRACK_DECREASE_PIN) == HIGH && digitalRead(BTN_TRACK_INCREASE_PIN) == LOW)
      {
      Serial.println(" reikia mazinti ");
        currentTractionState--;    
      }

      if (currentTractionState > tractionState) {
        digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, LOW);
        digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, HIGH);
      } else if (currentTractionState < tractionState){
        digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, HIGH);
        digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, LOW);
      } 
//      else {
//        digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, LOW);
//        digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, LOW);
//      }
//    Serial.println("asasasas");
  } else if (digitalRead(BTN_TRACK_INCREASE_PIN) == HIGH && digitalRead(BTN_TRACK_DECREASE_PIN) == HIGH) {
      Serial.print("ATJUGTA DEL DVIGUBO PASPAUDIMO");
      digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, LOW);
      digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, HIGH);
//      Serial.println("dvigubas");
      lcd.setCursor(1,0);
      lcd.print ("V");
  }
}

void tractionControlState()
{
  long currentTractionState = tractionState;

  if (!(digitalRead(BTN_TRACK_INCREASE_PIN) == HIGH && digitalRead(BTN_TRACK_DECREASE_PIN) == HIGH)) {

    if (tracOff == LOW) {
      
      if (digitalRead(BTN_TRACK_INCREASE_PIN) == HIGH && digitalRead(BTN_TRACK_DECREASE_PIN) == LOW)
      {
//      Serial.println(" reikia didinati ");
        currentTractionState++;   
      } else 
      if (digitalRead(BTN_TRACK_DECREASE_PIN) == HIGH && digitalRead(BTN_TRACK_INCREASE_PIN) == LOW)
      {
//      Serial.println(" reikia mazinti ");
        currentTractionState--;    
      }
      if (currentTractionState > tractionState && tracIncreaseOn == LOW && tracDecreaseOn == LOW) {
        tracIncreaseOn = HIGH;
        tracIncreaseOnTime = millis();
      
        tracDecreaseOn = LOW;
        tracDecreaseOnTime = 0;
        Serial.print(" didinama ");
      } else 
      if (currentTractionState < tractionState && tracIncreaseOn == LOW && tracDecreaseOn == LOW) {
        tracIncreaseOn = LOW;
        tracIncreaseOnTime = 0;
      
        tracDecreaseOn = HIGH;
        tracDecreaseOnTime = millis();
        Serial.print(" mazinama ");
      } else if (tracIncreaseOn == LOW && tracDecreaseOn == LOW) {
//      Serial.print(" nesikeicia ");
        tracIncreaseOn = LOW;
        tracIncreaseOnTime = 0;
        tracDecreaseOn = LOW;
        tracDecreaseOnTime = 0;
      }
    }
//    Serial.println();
  } else if (digitalRead(BTN_TRACK_INCREASE_PIN) == HIGH && digitalRead(BTN_TRACK_DECREASE_PIN) == HIGH) {
//      Serial.print("ATJUGTA DEL DVIGUBO PASPAUDIMO");
      digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, LOW);
      digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, HIGH);
      tracIncreaseOn = LOW;
      tracIncreaseOnTime = 0;
      
      tracDecreaseOn = LOW;
      tracDecreaseOnTime = 0;
//      Serial.println();
      lcd.setCursor(1,0);
      lcd.print ("V");
  }
  doTraction();
}

void doTraction()
{
  if (tracOff == HIGH && tracOffTime != 0 && millis() - tracOffTime > timeForTractionOff) {
      Serial.println();
      Serial.print(" Praejo ");
      Serial.print(millis()-tracOffTime);
      Serial.print(" laiko, galima vel reguliuoti trauka ");
      Serial.println();
      tracOffTime = 0;
      tracOff = LOW;
  }
    
  if (tracOff == LOW) {
    if (tracIncreaseOn == HIGH && millis() - tracIncreaseOnTime < tractionWorkingTime)
    {
      digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, LOW);
      digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, HIGH);

      lcd.setCursor(1,0);
      lcd.print ("+");
    } else 
    if (tracDecreaseOn == HIGH && millis() - tracDecreaseOnTime < tractionWorkingTime)
    {
      digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, HIGH);
      digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, LOW);

      lcd.setCursor(1,0);
      lcd.print ("-");
    } 
    if ( (tracIncreaseOn == HIGH && millis() - tracIncreaseOnTime >= tractionWorkingTime) || (tracDecreaseOn == HIGH && millis() - tracDecreaseOnTime >= tractionWorkingTime))
    {
      tracIncreaseOn = LOW;
      tracDecreaseOn = LOW;

      tracIncreaseOnTime = 0;
      tracDecreaseOnTime = 0;
      
      digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, LOW);
      digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, LOW);

      lcd.setCursor(1,0);
      lcd.print (" ");
      
      tracOffTime = millis();
      tracOff = HIGH;
      Serial.println (" nunulinom trauka ");
    }  
  }
}

void ShowTraction()
{
  int number = weight * scale.get_units() * weightDelimiter;
  uint8_t ones,tens,hundreds;

//  hundreds = number/100;
//  number = number-hundreds*100;
//
//  Serial.print("a: ");
//  Serial.print(number);
//  Serial.print(", ");
//  tens = number/10;
//  ones = number-tens*10;
//  /* Update the display with the current counter value */
////  WriteNumberToSegment(0 , 0);
//  WriteNumberToSegment(1 , hundreds);
//  WriteNumberToSegment(2 , tens);
//  WriteNumberToSegment(3 , ones);
}

///* Write a decimal number between 0 and 9 to one of the 4 digits of the display */
//void WriteNumberToSegment(byte Segment, byte Value)
//{
//digitalWrite(LATCH_DIO,LOW);
//shiftOut(DATA_DIO, CLK_DIO, MSBFIRST, SEGMENT_MAP[Value]);
//shiftOut(DATA_DIO, CLK_DIO, MSBFIRST, SEGMENT_SELECT[Segment] );
//digitalWrite(LATCH_DIO,HIGH);
//}

