
/*
 * 
 * Start towing ir Stop towing naudoti du atskirus mygtukus
 * Auto retrieve veikia iki paklaidos (pvz -100m +- dar paklaida nuo isvynioto ilgio) nuo likusio sniuro iki prdzios...
 * Po to retrievui naudoti manual buttona, kol paspaustas vyniojimas veikia, atleistas - vyniojimas stabdomas 
 * - retrievui - inputai skirtingi (auto/manual), outputas i rele - tas pats
 * 
1. svarstykles  (CLK/DT) - 2, 3  pin, 3=>17/ spalvos: apkorovos celes iejimas: zalias-A+, baltas-A-, juodas-E-, raudonas-E+
2. sistemos startui/calibravimui 4 pin
3. speedometrui - 5 pin, startas uzduodamas  gavus pirma signala, pradedamas skaiciuoti greitis ir sniuro issyvyniojimo atstumas
4. traukos relei 4 IN is mygtuko +/- bus 6/7 pin  - dedati pvz i scetch'a pasijungus ant reles ir isitikinti kad veikia, esanst paspaudimui i cosole printinti rezultata
5. traukos relei OUT is arduino IN i rele +/- bus 8/9 pin  - dedati pvz i scetch'a pasijungus ant reles ir isitikinti kad veikia, esanst paspaudimui i cosole printinti rezultata
6. sniuro vyniojimui mygtukas IN i arduino bus 11 pin
7. sniuro vyniojimui OUT is qruino IN i rele 10 pin
8. sniuro vyniojimo priverstinis sustabdymas - mygtukas 12 pin // bus pajungtas veliau
9. sistemos restartui mygtukas - arduino reset
10. bugno sukimosi INPUT PIN 12 is gerkono nuo bugno

* 11. bugno sukimuisi INPUT - o OUTPUT buzzer/sviesdiodis , LED output pin 13

*** Paspaudus stop, traukos nuemimas prasideda iki minimumo,  kol cele rodo <= 0 arba kol gaunamas signalas is variklio kad jis krastineje padetyje
*** bus du galiniai input mygtukai ant traukos variklio, reikia rezervuoti inputus i arduino, kad nebeleisti didinti/mazinti traukos, 

*** TODO: nedaleisti programiskai kad vienu metu trauka butu ir didinama ir mazinama

rezultato pvz:


Trauka: 21.1 kg, virves greitis: 10 km/h, isivyniojo: +243 m, (liko: 1350 m), Trauka: (neutrali/didina/mazina), Virves vyniojimas: neutralus/Ijungtas ()

 */

#include "HX711.h"

#define DOUT  17 //3 = > 17
#define CLK  2
#define BTN_START_PIN  4
#define SPEED_PIN  5
#define BTN_TRACK_INCREASE_PIN  6
#define BTN_TRACK_DECREASE_PIN  7
#define OUTPUT_RELAY_TRACK_INCREASE_PIN  8 // in current scetch output is directly without resitors connected to PIN 8
#define OUTPUT_RELAY_TRACK_DECREASE_PIN  9 // in current scetch output is directly without resitors connected to PIN 9
#define OUTPUT_RELAY_REEL_START_PIN 10     // in current scetch output is directly without resitors connected to PIN 10
#define BTN_REEL_START_PIN 11
#define BTN_REEL_STOP_PIN 12

#define BTN_REEL_SPIN_PIN_LED 12
#define LED_REEL_SPIN_PIN_BUZZER 13

#define longPressTimes 15
#define maxWireLength 1500
#define rollDiameterCm 10
#define minimalDelayForSpeedReel 2000
#define PI 3.1415926535897932384626433832795


HX711 scale(DOUT, CLK);

float calibration_factor = -7050; //-7050 worked for my 440lb max scale setup
boolean startBtnState = LOW;
boolean previousStartBtnState = LOW;

boolean speedBtnState = LOW;
boolean previousSpeedBtnState = LOW;

int towingOn = 0;
unsigned long towingStartTime;
unsigned long statePrintOutTime;

unsigned long distanceStartTime;
unsigned long reelSpinLastTime;
long reelRotationCount = 0;

int pressState = 0;

int tractionState = 0;

double wireSpeed = 0;
double wireLength = 0;

int reelRetrieveState = 0;

void setup() {
  pinMode(BTN_START_PIN, INPUT);
//  digitalWrite(BTN_START_PIN, HIGH); // turns on pull-up resistor after input

  pinMode(SPEED_PIN, INPUT);
  pinMode(BTN_TRACK_INCREASE_PIN, INPUT);
  pinMode(BTN_TRACK_DECREASE_PIN, INPUT);
  pinMode(OUTPUT_RELAY_TRACK_INCREASE_PIN, OUTPUT); // in current scetch output is directly without resitors connected to PIN 8
  pinMode(OUTPUT_RELAY_TRACK_DECREASE_PIN, OUTPUT); // in current scetch output is directly without resitors connected to PIN 9
  pinMode(OUTPUT_RELAY_REEL_START_PIN, OUTPUT); // in current scetch output is directly without resitors connected to PIN 10
  pinMode(BTN_REEL_START_PIN, INPUT);
  pinMode(BTN_REEL_STOP_PIN, INPUT); // start/stop button can't be used the same one, we can use only internal state for it

  pinMode(BTN_REEL_SPIN_PIN_LED, INPUT); // in current scetch led output is working on speed input
  pinMode(LED_REEL_SPIN_PIN_BUZZER, OUTPUT); // in current scetch led output is working on speed input or BTN_REEL_SPIN_PIN_LED
  
  digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, HIGH);
  digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, HIGH);
  digitalWrite(OUTPUT_RELAY_REEL_START_PIN, HIGH);

  Serial.begin(9600);

  // scale with HX711 calibration on each start, which makes scale to ZERO possition
  resetScale();
  Serial.println("Waiting for your input");
}

void loop() {
  
  startButtonState();
  reelState();
  readSpeed(); // maybe it's better to read wire speed and state always, not according to towing is On/Off...
//  readScale();
  tractionControlState();
  
  if (towingOn == 1 && (millis() - statePrintOutTime) > 1000) {
    readScale();
    printSpeed();

    wireRetrieveState();
    tractionControlState();  
  
    statePrintOutTime = millis();
    Serial.println();
  }
  
//  delay(500); // this will be removed when getting quite enough accuracy and button state management in code
}

void startButtonState()
{
  startBtnState = digitalRead(BTN_START_PIN);

  if (startBtnState != previousStartBtnState && startBtnState == HIGH) {
    towingOn = !towingOn;
    if (towingOn == 1) {
      towingStartTime = millis();
      distanceStartTime = towingStartTime;
      reelSpinLastTime = distanceStartTime;
      Serial.print("Towing has started!");
      Serial.print(" Time:");
      printTime();
      
      Serial.println();
    } else {
      Serial.println("Towing has stopped!");
      towingStartTime = 0;
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


    if (speedBtnState != previousSpeedBtnState && speedBtnState == HIGH) {

      double ropeCirmcumferenceCm = (rollDiameterCm * PI); // 2 * PI * R = PI * D
      wireLength = wireLength + (ropeCirmcumferenceCm *0.01);
      
      if (timeHasPassedAfterLastSignal > 0) {
        // SPEED Km/h = m * 1000 / min * 60  = cm * 100 * 1000 / s * 60 * 60 = cm * 100 * 1000 / ms * 1000 * 60 * 60 = cm * 100 / ms * 3600
        float ropeSpeed = (ropeCirmcumferenceCm * 100) / (timeHasPassedAfterLastSignal * 3600);
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
  Serial.print(" km/h, isivyniojo: ");
  
  if (wireLength > 0.0) {
    Serial.print("+");  
  } else {
    Serial.print("-");
  }
  Serial.print(wireLength);
  Serial.print("m, (liko: ");
  Serial.print(maxWireLength - wireLength);
  Serial.print(" m)");
}

void printTime()
{
  long towingTime = millis() - towingStartTime;

  long sekundes = towingTime / 1000; 
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
//  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
//  Serial.println(zero_factor);
}

void readScale()
{
  scale.set_scale(calibration_factor); //Adjust to this calibration factor

  Serial.print("Laikas nuo starto: ");
  printTime();
  
  Serial.print(", Trauka: ");
  Serial.print(scale.get_units(), 0.453592); // used default lbs to kg fraction (0.453592), later we need to calculate it more acuratly to the phisical error we want to have for accuracy/precision
  // NOTE: at this moment 2.5 kgs is showing 3 kg even when wheight is swinging - it shows 4 kg...
  
  Serial.print(" kg"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
//  Serial.println();
}

void wireRetrieveState()
{
  Serial.print(", Virves vyniojimas:"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  int currentReelRetrieveState = reelRetrieveState;
  
  if (digitalRead(BTN_REEL_STOP_PIN) == HIGH) {
    reelRetrieveState = 0;
  } else if (digitalRead(BTN_REEL_START_PIN) == HIGH) {
    reelRetrieveState = 1;
  } 
  
  if (reelRetrieveState == 0) {
    digitalWrite(OUTPUT_RELAY_REEL_START_PIN, HIGH);
    Serial.print("neutralus");
  } else {
    digitalWrite(OUTPUT_RELAY_REEL_START_PIN, LOW);
    Serial.print("IJUNGTAS");
  }
}
 
void tractionControlState()
{
  int currentTractionState = tractionState;
  if (digitalRead(BTN_TRACK_INCREASE_PIN) == HIGH)
  {
    currentTractionState++;   
  } else 
  if (digitalRead(BTN_TRACK_DECREASE_PIN) == HIGH)
  {
    currentTractionState--;    
  }
  Serial.print(", Traukos pokytis:");
  if (currentTractionState > tractionState) {
    digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, LOW);
    Serial.print("didinama");
  } else if (currentTractionState < tractionState) {
    digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, LOW);
    Serial.print("mazinama");
  } else {
    Serial.print("nesikeicia");
    digitalWrite(OUTPUT_RELAY_TRACK_INCREASE_PIN, HIGH);
    digitalWrite(OUTPUT_RELAY_TRACK_DECREASE_PIN, HIGH);
  }
  Serial.println();
}

void reelState()
{
    if (digitalRead(SPEED_PIN) == HIGH || digitalRead(BTN_REEL_SPIN_PIN_LED) == HIGH) {
    digitalWrite(LED_REEL_SPIN_PIN_BUZZER, HIGH);
//    Serial.print(", BUGNAS SUKASI ");
  } else {
    digitalWrite(LED_REEL_SPIN_PIN_BUZZER, LOW);
//    Serial.print(", bugnas nesisuka ");
  }
}

