
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
  

#define DOUT  3 //17 //3 = > 17
#define CLK  2
#define SPEED_PIN  12 //16 //15 // 5 => 15 sfor sparkfun

#define longPressTimes 15
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

double rollDiameterCm = 10;
double maxWireLength = 1500;

boolean startBtnState = LOW;
long startBtnStateChangeTime = 0;
boolean previousStartBtnState = LOW;

int towingSymbol = 0;

boolean speedBtnState = LOW;
boolean previousSpeedBtnState = HIGH;

boolean speedSignalStarted = false;
 
int towingOn = 0;
unsigned long towingStartTime;
long statePrintOutTime = 0;

long distanceStartTime = 0;
unsigned long reelSpinLastTime = 0;
long reelRotationCount = 0;

double wireSpeed = 0;
double wireLength = 0;

double lastTraction = 0;

void setup() {
  pinMode(SPEED_PIN, INPUT);
  digitalWrite(SPEED_PIN, HIGH);
  
  Serial.begin(115200); // 9600 => 115200 for soarkfun 

  // set up the LCD's number of colum5ns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.setCursor(0,1);
  lcd.print(" SkyScale v1.2.1");
  lcd.setCursor(2,0);
  lcd.print("Waiting start!");

  startBtnStateChangeTime = millis();
  
  // scale with HX711 calibration on each start, which makes scale to ZERO possition
  resetScale();
}

void loop() {

  towingOn = 1;
  readSpeed(); // maybe it's better to read wire speed and state always, not according to towing is On/Off...

  if ((millis() - statePrintOutTime) > printOutDelay) {
    if (statePrintOutTime == 0) 
    {
      lcd.setCursor(0,1);
      lcd.print ("               ");
    }
    readScale();
    
    if (towingOn == 1) {
      printSpeed();
    }
    statePrintOutTime = millis();
    Serial.println();
  }
//  delay(500); // this will be removed when getting quite enough accuracy and button state management in code
}

void readSpeed()
{
  unsigned long currentReelSignalTime = millis();
  unsigned long timeHasPassedAfterLastSignal = currentReelSignalTime - reelSpinLastTime;

  speedBtnState = digitalRead(SPEED_PIN);

    if (speedBtnState == LOW && previousSpeedBtnState == HIGH) {
        speedSignalStarted = true;
    }

    if (speedSignalStarted && previousSpeedBtnState == LOW) {  
      speedSignalStarted = false;
  
      double ropeCirmcumferenceCm = (rollDiameterCm * PI); // 2 * PI * R = PI * D
      wireLength = wireLength + ropeCirmcumferenceCm;
      
      if (timeHasPassedAfterLastSignal > 0) {
        // SPEED Km/h = m * 1000 / min * 60  = cm * 100 * 1000 / s * 60 * 60 = cm * 100 * 1000 / ms * 1000 * 60 * 60 = cm * 100000 / ms * 3600 * 1000 = cm * 10 / ms * 3600
        double ropeSpeed = (ropeCirmcumferenceCm * 1000) / (timeHasPassedAfterLastSignal * 36);
        
        wireSpeed = ropeSpeed;  

        reelSpinLastTime = currentReelSignalTime;
        Serial.print("Praejo: ");
        Serial.print(timeHasPassedAfterLastSignal);
        Serial.print(" ms, issisuko:");
        Serial.print(ropeCirmcumferenceCm);
        Serial.print("cm, greitis: ");
        Serial.print(wireSpeed);
        Serial.println();
      }
      
    } 
  else if (timeHasPassedAfterLastSignal > minimalDelayForSpeedReel) {    
     wireSpeed = 0;
     previousSpeedBtnState = HIGH;
  }

  if (speedBtnState != previousSpeedBtnState) {
    previousSpeedBtnState = speedBtnState;
  }
}

void printSpeed()
{
  Serial.print(", virves greitis: ");
  Serial.print((long)wireSpeed);

  lcd.setCursor(0,1);
  lcd.print ("     ");

  lcd.setCursor(12,1);
  lcd.print ("    ");
  lcd.setCursor(12,1);
  lcd.print ((long)wireSpeed);
  
  Serial.print(" km/h, isivyniojo: ");
  
  if (wireLength > 0.0) {
    Serial.print("+");  
  } else {
    Serial.print("-");
  }
  Serial.print((long)(wireLength / 100));

  if ((wireLength / 100) < 10) {
    lcd.setCursor(3,1);
  } else if ((wireLength / 100) < 100) {
    lcd.setCursor(2,1);
  } else if ((wireLength / 100) < 1000) {
    lcd.setCursor(1,1);
  } else {
    lcd.setCursor(0,1);
  }
  lcd.print ((long)(wireLength / 100));
  lcd.print ("m");
  
  Serial.print("m, (liko: ");
  Serial.print((long)(maxWireLength - wireLength / 100));
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
  long tracForce = (long)traction;

  lcd.setCursor(0,0);
  lcd.print ("             ");
  lcd.setCursor(0,0);
  long analogTrac = round(traction/10);
  long phase = 0;
  while (phase < analogTrac && analogTrac > 0)
  {
    lcd.print ("@");
    phase = phase + 1;
  }
  
  Serial.print(", Trauka long: ");
  Serial.print(tracForce);
  
  Serial.print(", Trauka: ");
  Serial.print(traction, weightDelimiter); // used default lbs to kg fraction (0.453592), later we need to calculate it more acuratly to the phisical error we want to have for accuracy/precision
  Serial.print("(");
  Serial.print(traction);
  Serial.print(")");
  
  lcd.setCursor(13,0);
  lcd.print ("   ");

  if (traction < 10) {
    lcd.setCursor(15,0);  
  } else if (traction < 100) {
    lcd.setCursor(14,0);  
  } else {
    lcd.setCursor(13,0);
  }
  
  lcd.print (traction, weightDelimiter);

  Serial.print(" kg"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  Serial.println();
}

