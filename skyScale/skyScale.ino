
/*
 * 
 * Start towing ir Stop towing naudoti du atskirus mygtukus
 * Auto retrieve veikia iki paklaidos (pvz -100m +- dar paklaida nuo isvynioto ilgio) nuo likusio sniuro iki prdzios...
 * Po to retrievui naudoti manual buttona, kol paspaustas vyniojimas veikia, atleistas - vyniojimas stabdomas 
 * - retrievui - inputai skirtingi (auto/manual), outputas i rele - tas pats
 * 
1. svarstykles  (CLK/DT) - 2, 3  pin
2. sistemos startui/calibravimui 4 pin
3. speedometrui - 5 pin, startas uzduodamas  gavus pirma signala, pradedamas skaiciuoti greitis ir sniuro issyvyniojimo atstumas
4. traukos relei IN is mygtuko +/- bus 6/7 pin  - dedati pvz i scetch'a pasijungus ant reles ir isitikinti kad veikia, esanst paspaudimui i cosole printinti rezultata
5. traukos relei OUT is arduino IN i rele +/- bus 8/9 pin  - dedati pvz i scetch'a pasijungus ant reles ir isitikinti kad veikia, esanst paspaudimui i cosole printinti rezultata
6. sniuro vyniojimui mygtukas IN i arduino bus 10 pin
7. sniuro vyniojimui OUT is qruino IN i rele 11 pin
8. sistemos veikimo pradziai mygtukas, optional, jei 2 neuztektu 12 pin
9. sistemos restartui mygtukas - arduino reset

* 10. bugno sukimuisi INPUT - o OUTPUT buzzer/sviesdiodis

rezultato pvz:


Trauka: 21.1 kg, virves greitis: 10 km/h, isivyniojo: +243 m, (liko: 1350 m), Trauka: (neutrali/didina/mazina), Virves vyniojimas: neutralus/Ijungtas ()

 */

#include "HX711.h"

#define DOUT  3
#define CLK  2
#define BTN_START_PIN  4
#define SPEED_PIN  5
#define BTN_TRACK_INCREASE_PIN  6
#define BTN_TRACK_DECREASE_PIN  7
#define OUTPUT_RELAY_TRACK_INCREASE_PIN  8
#define OUTPUT_RELAY_TRACK_DECREASE_PIN  9
#define BTN_REEL_START_PIN 10
#define OUTPUT_RELAY_REEL_START_PIN 11
#define BTN_REEL_STOP_PIN 12
#define longPressTimes 15
#define maxWireLength 1500

HX711 scale(DOUT, CLK);

float calibration_factor = -7050; //-7050 worked for my 440lb max scale setup
boolean startBtnState = LOW;
boolean previousStartBtnState = LOW;
int towingOn = 0;
int pressState = 0;

int tractionState = 0;

int wireSpeed = 0;
int wireLength = 0;

int reelRetrieveState = 0;

void setup() {
  pinMode(BTN_START_PIN, INPUT);
//  digitalWrite(BTN_START_PIN, HIGH); // turns on pull-up resistor after input

  pinMode(SPEED_PIN, INPUT);
  pinMode(BTN_TRACK_INCREASE_PIN, INPUT);
  pinMode(BTN_TRACK_DECREASE_PIN, INPUT);
  pinMode(OUTPUT_RELAY_TRACK_INCREASE_PIN, OUTPUT);
  pinMode(OUTPUT_RELAY_TRACK_DECREASE_PIN, OUTPUT);
  pinMode(BTN_REEL_START_PIN, INPUT);
  pinMode(BTN_REEL_STOP_PIN, INPUT); // start/stop button can't be used the same one, we can use only internal state for it
  pinMode(OUTPUT_RELAY_REEL_START_PIN, OUTPUT);

  Serial.begin(9600);

  // scale with HX711 calibration on each start, which makes scale to ZERO possition
  resetScale();
  Serial.println("Waiting for your input");
}

void loop() {
  startButtonState();
  
  if (towingOn == 1) {
    readScale();
    readSpeed(); // maybe it's better to read wire speed and state always, not according to towing is On/Off...
    wireRetrieveState();
    tractionControlState();
    Serial.println();
  }
  
  delay(500); // this will be removed when getting quite enough accuracy and button state management in code
}

void startButtonState()
{
  startBtnState = digitalRead(BTN_START_PIN);
  if (startBtnState == previousStartBtnState && startBtnState == HIGH) {
    towingOn = !towingOn;
    if (towingOn == 1) {
      Serial.println("Towing has started!");
    } else {
      Serial.println("Towing has stopped!");
    }
//    previousStartBtnState = startBtnState;
//      pressState = 0;
  } else {
//     pressState++;
  }
  previousStartBtnState = startBtnState;
}

void readSpeed()
{
  Serial.print(", virves greitis: ");
  Serial.print(wireSpeed);
  Serial.print(" km/h, isivyniojo: ");
  
  if (wireLength > 0) {
    Serial.print("+");  
  } else {
    Serial.print("-");
  }
  Serial.print(wireLength);
  Serial.print("m, (liko: ");
  Serial.print(maxWireLength - wireLength);
  Serial.print(" m)");
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

  Serial.print("Trauka: ");
  Serial.print(scale.get_units(), 0.453592); // used default lbs to kg fraction (0.453592), later we need to calculate it more acuratly to the phisical error we want to have for accuracy/precision
  // NOTE: at this moment 2.5 kgs is showing 3 kg even when wheight is swinging - it shows 4 kg...
  
  Serial.print(" kg"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
}

void wireRetrieveState()
{
  Serial.print(", Virves vyniojimas:"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  if (reelRetrieveState == 0) {
    Serial.print("neutralus");
  } else {
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
    Serial.print("didinama");
  } else if (currentTractionState < tractionState) {
    Serial.print("mazinama");
  } else {
    Serial.print("nesikeicia");
  }
}

