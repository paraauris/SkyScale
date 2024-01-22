#include "U8glib.h"

U8GLIB_PCD8544 u8g(13, 11, 10, 9, 8);    // SPI Com: SCK(CLK) = 13, MOSI(DIN) = 11, CS(CE) = 10, A0(DC) = 9, Reset(RST) = 8

#include "HX711.h"
HX711 scale;

#define SPEED_PIN 5 // prijungtas prie D2 su interuptu 
float apsilgis =0.251; // ilgis metrais skritulio ilgis apsitimo ilgis = apskritimo diametras * pi 8.00 * pi 2512 cm 0,25m
float ilgis = 0.00; //virves ilgis

volatile int rpmcount;

 int a=0; // apssauga del dubliavimo
//btn
 uint32_t ms;             //     
 uint32_t m_lastChange;   //    reikalinga pokycio skaiciavimams
 uint32_t m_dbTime = 10; //  apsauga nuo virpejimo siui metu nereikalinga  
 unsigned long pokytis=0; // pokytis tarp ratuko sens suveikimų  
 double greitis=0.00;   
 double ababa;            // greičio tarpiniams skaiciavimams  
 float Tkof;              // jegos koeficiantas iš jo dauginama išmatuota jėga. Jis reguliuojamas su pot. 
 double jega;

void setup() 
    {
     rpmcount =0;         // apsisukimų skaiciuoklė ilgio matavimui  D2
     Serial.begin(115200);
     pinMode(SPEED_PIN, INPUT);
     Serial.begin(38400);
     scale.begin(A1, A0);
     scale.set_scale(-6900);
     scale.tare();
    
    }

void loop() 
    {
   //serial isvestis testams
     Serial.print("pokytis "); Serial.print(pokytis, 5); Serial.println("ms ");
     Serial.print("ababa "); Serial.print(ababa, 5);  Serial.println("h ");
     Serial.print("greitis "); Serial.print(greitis, 5); Serial.println("km/h ");
     Serial.print("tkof"); Serial.println(Tkof, 3);  
     Serial.print("jega "); Serial.print(jega, 3); Serial.println("kg ");
     Serial.print("ilgis "); Serial.print(ilgis, 3); Serial.println("m ");
     Serial.print("rpmcount "); Serial.println(rpmcount); 
   //
     Tkof = analogRead(2); // jėgos koef. vidurinė koja prijungta prire A2 šoninės prie Vcc ir Gnd
     Tkof = map(Tkof, 0, 1023, 0, 3); //(value, fromLow, fromHigh, toLow, toHigh)
  
     jega= Tkof*scale.get_units(5);
  
     attachInterrupt(0, rpm, RISING);  //suveikia kai iš 0 i 1 pasikeicia

     ilgis=rpmcount*apsilgis;
 
           
   //greitis                              // apsk ilgis m . kad padaryt km *1000   
     ababa=(double)pokytis/3600; //pokytis yra ms. kad padaryt h reikia /3600 000    supaprastinus /3600      
     greitis = (double)apsilgis/(double)ababa;
   
  // picture loop
     u8g.firstPage();  
     do {
     u8g.setContrast(140);
     //u8g.drawRFrame(0, 0, 63, 17, 3);   //Data1 frame
    // u8g.drawRFrame(0, 31, 63, 17, 3);  //Data2 frame
    // u8g.drawRFrame(0, 16, 63, 16, 3);   //Data3 frame
    //u8g.drawRFrame(62, 0, 22, 48, 3);   //Graph frame
    

     u8g.setFont(u8g_font_fub30r);
     u8g.setPrintPos(0, 31);
     u8g.print((int)jega);

     u8g.setFont(u8g_font_courB08r);
     u8g.setPrintPos(71, 27);
     u8g.print("Kg");
     
     u8g.setFont(u8g_font_courB10);
     u8g.setPrintPos(0, 47);
     u8g.print((int)ilgis);

     u8g.setFont(u8g_font_courB08r);
     u8g.setPrintPos(35, 47);
     u8g.print("m");
     
     u8g.setFont(u8g_font_courB14r);
     u8g.setPrintPos(42, 47);
     u8g.print((int)greitis);

     u8g.setFont(u8g_font_courB08r);
     u8g.setPrintPos(65, 47);
     u8g.print("KmH");
        } while( u8g.nextPage() );
  
   }


void rpm()
    { 
     if (a==1 && (micros() - m_lastChange) > m_dbTime)
       {
        pokytis= (micros() - m_lastChange);
        rpmcount++;
        a=0;
       }    
     a=1; m_lastChange = micros();
    }


///// pot<>koef skaičiavimui
 float map(float x, float in_min, float in_max, float out_min, float out_max)
      {
       return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
      }
