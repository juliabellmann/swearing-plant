//Assignment MCS40-LB - Thema: Bewegunssensor (erweitert mit Feuchtigkeitssensor, DF-Player und Lautsprecher)

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"  //Download hier zu finden: https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299

SoftwareSerial mySoftwareSerial(10, 11); // DFPlayer an Pin 10+11
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);



//Variablen fuer den Bewegungssensor
int bewegung=7; //Bewegungssensor an Pin 7 
int bewegungsstatus=0;

//Variablen fuer den Feuchtigkeitssensor
int messwert=0; //unter dieser Variable wird später der Messwert gespeichert

void setup() {
  mySoftwareSerial.begin(9600);
  Serial.begin(9600);
  pinMode(bewegung, INPUT);
 
  //DF Player Code - kopiert von folgender Quelle: 
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  if (!myDFPlayer.begin(mySoftwareSerial)) {  
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!")); while (true);
  }
  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.volume(30);      //Lautstärke von 0-30
  myDFPlayer.play (1);        // spiele ersten Song

}

//Hier wird die Feuchtigkeit und Bewegung gemessen und daraufhin ggf. ein Sound ausgegeben - im Loop
void loop() {
  messwert = analogRead(A0); //hier wird geprüft, was der Bodenfeuchtigkeitssensor sagt
  Serial.print("Messwert:");
  Serial.println(messwert);   //Zeig Messwert im seriellen Monitor!
  bewegungsstatus = digitalRead(bewegung); //hier wird der Pin 7 ausgelesen. "HIGH" für 5V oder "LOW" für 0V

  if (messwert < 400 && bewegungsstatus == HIGH) {  //ACHTUNG Werte für Bodenfeuchte je nach Pflanzenbedarf anpassen 
                                                    // im Idealfall gucken, was die "Standard"-Werte des Sensor sind, indem man einmal an der Luft und einmal im Wasserglas misst, bzw. Trockene und feuchte Erde
    Serial.println("Bewegung erkannt und Wert schlecht");
    myDFPlayer.next();        //Wenn Bewegung erkannt und Erde trocken, dann spiele den nächsten Sound
    delay(5000);             //Pause, damit das nicht konstant triggert
  }
  else {
    Serial.println("Keine Bewegung oder Wert ok");      
    delay(5000);
  }
  
  delay(5000);
}
