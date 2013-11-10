

//Port Mapping:
// D0=Pin20; D1=Pin21; D2=Pin19; D3=Pin18; D4=Pin25; D5=Pin31; D6=Pin27; D7=Pin1;
// D8=Pin28; D9=Pin29; D10=Pin30; D11=Pin12; D12=Pin26; D13=Pin32; 
// A0=Pin36; A1=Pin37; A2=Pin38; A3=Pin39; A4=Pin40; A5=Pin41; 

// Lightblock
// Kompatibel zu Teensy 2.0 mit Teensy Boatloader, 
// Kompatibel zu Leonardo mit Leonardo Boatloader und DtrEnable im Terminalprogramm.
// nicht Kompatibel zu Teensy 1.0:

// Wenn Controller nicht mehr erkannt wird: Masse an Pin13(Reset) während Arduino IDE nach compilieren nach Controller sucht.

// serielle Befehle ##############################
// Eingehend: W, P, N, O, K, Y, D, G, E
// Ausgehend: J, L, U, R, Q, F, A, B, C, X, M, S, H
// Frei:  V
// IMPLEMENTIERT ######################################''''''
// Version 1.7 - PID Funktion für Lüftersteuerung.
// Lightblock15 - Arduino 1.0 kompatible Befehle.
// NTC Temperaturfühler an F0 geht.
// Lüfter an C7 geht, eventuell PID nachstellen, bei 35% Leistung geht Lüfter aus.
// Mondlicht an PD3 geht.
// Zeit zeitdifferenz implementieren.

// Lightblock23:
// EEPROM Speicherung umgestellt, EEPROMAnything rausgenommen, verursacht Fehler?
// Konsolen Befehle implementiert: stop, start, lesen, lesenlong.
// Zeit wird korekt manuell gespeichert.

// Lightblock25:
// Funktionen extra.

// Lightblock26: +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// delay's entfernt.
// Variablenname VB.NET und Controller angeglichen.
// Phototransistor an Analogeingang.  0 - 1023 

// Lightblock27: +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* IR Remote eingebaut.
 * IRremote: IRrecvDemo - demonstrates receiving IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 *http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html
 */
// Lightblock28: +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Konsolenbefehle: 
// "stop" serieller Ausgang wird gestopt.
// start" serieller Ausgang wird gestartet.
// "eeprom" EEPROM Speicherinhalt wird strukturiert gelesen.
// "lesen" EEPROM Speicherinhalt wird unformatiert als byte gelesen.
// "lesenlong" EEPROM Speicherinhalt wird unformatiert als long gelesen.
// "loeschen"  EEPROM Speicherinhalt wird gelöscht.

// IR Empfang kann getestet werden.

// aktuelle VB Software: Lightblock_VB 1.9.6

// Lightblock29: +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// IR Status wird gesendet mit "T".
// IR Befehle implementiert: IR AN/AUS, Lampe AN/AUS, max.Hellingkeit +/-........
// Bugs in Variablenlängen korrigiert.

// Lightblock32: +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// "F" Modifier in Strings eingebaut.
//By using the F() macro, the string is only copied 1 byte at a time, leaving the full string (character array) in FLASH.

// Lightblock36: +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Code reduziert, wire.h für I2C Timermodul implementiert.
//By using the F() macro, the string is only copied 1 byte at a time, leaving the full string (character array) in FLASH.

// Lightblock37: +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Float/Double eleminiert: PID Library von Double in Long umgeschrieben, MULTIMAP Funktion von Float in Int umgeschrieben. Test steht noch aus!!!!!!!!!!!!!!!!!!
// Thermister Funktion hat noch Float!!!!!!!!


// BUGS ###########################################
// Betriebszeit wird nicht gespeichert bei Stromausfall
// delay's hier noch nötig?


// zu tun: ########################################
// Messenger durch CMDMessenger ersetzen.
// "Freiblasfunktion" gegen Staub in den Lüftungskanälen.
// Softstart?
// einige Funktionen in den Sekundentakt legen, spart Rechenleistung.

// Includes: *******************************************************************
//#include <PID_v1_Long.h>  // PID Lüftersteuerung. Umgeschrieben auf Long Variablen!    http://playground.arduino.cc/Code/PIDLibrary
                                                                                  //     http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
#include <Time.h>  // Interne Controlleruhr, basierend auf Quarztakt.   http://playground.arduino.cc/Code/time
//#include <math.h>
#include <EEPROMEx.h>  // EEPROM, Werte speichern/lesen.  http://playground.arduino.cc/Code/EEPROMex
#include <DigitalToggle.h>
#include <Messenger.h>  // senden/empfangen serielle Befehle parsen.   http://playground.arduino.cc/code/messenger
#include <IRremote.h>  // Infrarot Fernsteuerung.    http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html
#include <Wire.h> // I2C Library. 
//#include <Streaming.h>  // Serial Streaming.       http://arduiniana.org/libraries/streaming/

//  #include "memdebug.h"
// #define DEBUG 1

#define firmware "37>VB1.9.9"  // Firmware String.

Messenger message = Messenger(','); // MESSENGER initialisierung, use this for ',' delimiter

//static char tmp1[55];

// Variablen Port Ausgänge: *******************************************************************
//int fan = 13;  // Ausgang Lüfter an Arduino Pin13 = C7.
int fan = 5;  // Ausgang Lüfter an Arduino Pin13 = C7.
int LEDmondlicht = 1;  // Ausgang blaue LEDs an Arduino PinD1 = PD3.
int LEDpin = 6; // die kontroll LED, Arduino Port6 = PD7.
int LEDhpled = 9;  // PWM Ausgang für High Power LED, Arduino Pin9 = PB5
byte hpled = 0;  // High Power LED Variable

// Variablen Port Eingänge: *******************************************************************
int ntc = 5;    // Eingang NTC Temperaturfühler Arduino PinA5 = F0.
int photo = 4;    // Eingang Phototransistor Arduino PinA4 = F1.


// Variablen: *********************************************************************************
int ResValue= 10000; // Der Thermistor Widerstand bei 20°C
byte mondlicht;  //geparster Wert vom PC wird hier gespeichert. (0-255)
char incoming; // incoming serial data
int i=0;  // Hilfsvariable.
//int q=0;  // wozu das?
unsigned long betriebsminuten; // 1 Tag= 1440 Min., 10 Jahre= 5.256.000 Min. / long=2.147.483.647
byte minutex;  // Hilfsvariable Betriebsminuten. (0-59)
unsigned long daysec; // die Sekunden des Tages.  // 1 Tag= 86.400 Sec. / long=2.147.483.647
//float dimmer = 0;  // Dimmervariable, aktueller Dimmerwert. FmultiMap will Float!
long dimmer = 0;  // Dimmervariable, aktueller Dimmerwert. FmultiMap will Float!

byte outbound = 0; // Maximalwert der High Power LED (0-255). Dieser Wert kommt eigentlich nur von VBNet Prog und wird zurückgespiegelt.
unsigned long prevtime;  // Zeit Hilfsvariable. prevtime ist UNIX Timestamp Format.
unsigned long pctime; // Die Realtime vom PC, ist UNIX Timestamp Format.
//int zeitkorrektur; // Zeitkorrekturwert für time.h in Sekunden
byte zeitkorrektur; // Zeitkorrekturwert für time.h in Sekunden
byte secondx;  // Hilfsvariable Sekunden 0-60.
//unsigned long kallibriersekundenEnd = 43200; // Zeitbasis inerhalb der die Zeitkorrektur vorgenommen wird. muß dieselbe sein, wie in VB Programm.= Eingestellt: 12 Std. Messzeit. Max 65.535
//unsigned long kallibriersekunden; // Kallibriersekunden hochzählen.
unsigned int kallibriersekundenEnd = 43200; // Zeitbasis inerhalb der die Zeitkorrektur vorgenommen wird. muß dieselbe sein, wie in VB Programm.= Eingestellt: 12 Std. Messzeit. Max 65.535
unsigned int kallibriersekunden; // Kallibriersekunden hochzählen.
boolean gesendet= false;  // Hilfsvariable zur Steuerung Datensenden serielle Schnittstelle.
int befehle = 0;
int photowert;  // Variable für Phototransistor.
int eepromControl; // kontrolle, ob EEPROM beschrieben wurde.

//boolean stopflag = false;  // sekundenzählen AN/AUS.
boolean StopSerial = false;  // Datensenden serielle schnittstelle AN/AUS.

int stelle;   // Hilfsvariablen für EEPROM Test.
int lesen;
unsigned long lesen_long;
//unsigned long testlong;

const int maxAllowedWrites = 5000;  // für EEPROMEx max. anzahl Schreibzyklen
const int memBase          = 350;

unsigned int looptime; 
unsigned int milliloop;
//unsigned long looptime; 
//unsigned long milliloop;

// Variablen Infrarot library: ************************************************************************

boolean IRflag = false;  // Datensenden serielle schnittstelle AN/AUS
int RECV_PIN = 4;  // Arduino Port D4.
int RELAY_PIN = 10;  // NUR TEST (LED) !!!!!!!!!!!!!!!!!!!!!!!
IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long last = millis();  // Hilfsvariable für IR Empfang, nimmt millis() auf.


// +++++++++++++++++++++ Infrarot Codes bei denen geschaltet wird: +++++++++++++++++++++++++++++++

#define ENABLE_CODE 0xFFA25D                // IR Modus einschalten/ausschalten (EIN/AUS-Taste auf China Fernbedienung).
int enable = 0;                           // dazugehörige Variable.
#define LED_TOGGLE_CODE 0xFF629D            // Lampe einschalten/ausschalten  (Mode-Taste auf China Fernbedienung).
int on = 0;                               // dazugehörige Variable.
#define DIMM_PLUS_CODE 0xFF906F             // Lampenhelligkeit erhöhen    (+ Taste auf China Fernbedienung).
#define DIMM_MINUS_CODE 0xFFA857            // Lampenhelligkeit verringern (- Taste auf China Fernbedienung).
byte max_manuell = 0;                      // Maximalwert der High Power LED (0-255).
#define TIMER_HOCHDIMMEN_CODE 0xFFC25D      // Timer schaltet die Lampe gedimmt ein. (>> Taste auf China Fernbedienung).
unsigned long timer_on_manuell;           // Timervariable überimmt die daysec, zu diesem Zeitpunkt wird eingeschaltet.
#define TIMER_RUNTERDIMMEN_CODE 0xFF02FD   // Timer schaltet die Lampe gedimmt aus. (<< Taste auf China Fernbedienung).
unsigned long timer_off_manuell;           // Timervariable überimmt die daysec, zu diesem Zeitpunkt wird ausgeschaltet.

// Variablen PID library: ************************************************************************
//double setpoint, Input, Output;  // double=float, float funktioniert hier aber nicht! EINZIGE FLOAT VARIABLE; VERSUCHEN ZU ELIMINIEREN!

//long setpoint, Input, Output;  // double=float, float funktioniert hier aber nicht! EINZIGE FLOAT VARIABLE; VERSUCHEN ZU ELIMINIEREN!
byte pidwert;
long Kpx, Kix, Kdx, Inputx, setpointx, SampleTimex;

int temperatur;  // Therminstor Temperatur Variable

unsigned long t;   // Zeitvariable für EEPROM Speicherung. 

byte pidFan;

// Input = P = Nachstellwert, Vertärkung
// Output = I = Nachstellzeit
// Setpoint = D = Reaktion auf Änderungsgescheindgkeit

//PID myPID(&Input, &Output, &Setpoint,1,0.05,0.25, REVERSE); //sehr langsame Reaktion
//PID myPID(&Input, &Output, &Setpoint,2,5,1, REVERSE); //sehr schnelle reaktion! (zu schnell)

//PID myPID(&Input, &Output, &setpoint, 1, 1, 0.5, REVERSE); //ganz gut.
//PID myPID(&Input, &Output, &setpoint, 1, 1, 5, REVERSE); //ganz gut. +++++ 0.5 auf 5 geändert wegen umschreibung der PID Library auf Long Variablen.



// ********************************* Variablen Multimap library: *********************************************
// float out[8]; // 8 Stellen, Dimmerwert.
// float in[8]; // 8 Stellen, Zeitwert, Tagessekunden. 
long out[8]; // 8 Stellen, Dimmerwert. könnte Int sein, 0-255
long in[8]; // 8 Stellen, Zeitwert, Tagessekunden. muss mindestens Long sein, 1 Tag= 86.400 Sec.

void setup(){   // %%%%%%%%%%%%%%%%%%%%%%%%%%% SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Infrarot Setup *********************************************************
  pinMode(RELAY_PIN, OUTPUT);
  irrecv.enableIRIn(); // Start the receiver

  // ******************************************************************************
  EEPROM.setMaxAllowedWrites(maxAllowedWrites);
  EEPROM.setMemPool(memBase, EEPROMSizeATmega1280);
  
  // Pin konfiguration **************************************************************************** 
  pinMode(LEDhpled, OUTPUT);
  analogWrite(LEDhpled, 0);   // initialwert LED 0% 
  pinMode(mondlicht, OUTPUT);
  pinMode(fan, OUTPUT);
  pinMode(LEDpin, OUTPUT); // we flash the LED each second

  // die in EEPROM gespeicherten Werte auslesen! **************************************************

  pctime = EEPROM.readLong(1);   //getPCtime();
  setTime(pctime);   // Sync Arduino clock to the time received on the serial port

  outbound = EEPROM.readInt(5);
  zeitkorrektur = EEPROM.read(120);
  mondlicht = EEPROM.read(7);
  max_manuell = EEPROM.read(8);  // IR Manueller Maximalwert der Lampe. BYTE
  eepromControl = EEPROM.read(9);  // Kontrolle ob EEPROM mit Werten beschrieben wurde. BYTE

  in[0] = EEPROM.readLong(20);
  in[1] = EEPROM.readLong(25);
  in[2] = EEPROM.readLong(30);
  in[3] = EEPROM.readLong(35);
  in[4] = EEPROM.readLong(40);
  in[5] = EEPROM.readLong(45);
  in[6] = EEPROM.readLong(50);
  in[7] = EEPROM.readLong(55);

  out[0] = EEPROM.readLong(60);
  out[1] = EEPROM.readLong(65);
  out[2] = EEPROM.readLong(70);
  out[3] = EEPROM.readLong(75);
  out[4] = EEPROM.readLong(80);
  out[5] = EEPROM.readLong(85);
  out[6] = EEPROM.readLong(90);
  out[7] = EEPROM.readLong(95);

  setpointx = EEPROM.readLong(100);

  Serial.print(F("Betriebsminuten= "));
  betriebsminuten = EEPROM.readLong(110); 

  if (eepromControl != 111) {  // Wenn der EEEPROM Kontrollwert nicht 111 ist...
    setpointx = 35;
    zeitkorrektur = 0;
  }


  //t = EEPROM.readLong(1);
  //setTime(t);  // Time mit Daten aus EEPROM laden.

  //attachInterrupt(1, ExtInterrupt, RISING);    // Interrupt1 is on pin 19
  //attachInterrupt(2, ExtInterrupt, RISING);    // Interrupt2 is on pin 20 

    //attachInterrupt(1, ExtInterrupt, FALLING);    // Interrupt1 is on pin 19
  //attachInterrupt(3, ExtInterrupt, FALLING);    // Interrupt3 -is 1 on PD3/pin21
  attachInterrupt(0, ExtInterrupt, FALLING);    // Interrupt0 -is 3 on PD0/pin18 ....  0+1 geht nicht, weil für serial Funktion reserviert!!!!!!

  // interrupt 2 = PortDigital0 = PD2, hoffentlich....!!!!!!!!!!!!!!!!


  // ******************************************* PID konfiguration ***************************************************
 // Input = temperatur; // PID Eingangsvariable.
  //  setpoint = 30;
 // myPID.SetMode(AUTOMATIC); //turn the PID on
  
 //setpointx = 25;
 Kpx = -10;  // kleinster Wert = 1. Höchster Wert = 1000.
 Kix = -10;
 Kdx = -5;
 SampleTimex = 500;

  // ************************************************ Seriell ********************************************************

  //Serial.begin(115200); // Serielle Übertragungsgeschwindigkeit über USB.
  Serial.begin(38400); // Serielle Übertragungsgeschwindigkeit über USB.
  //while (! Serial); // Wait until Serial is ready - Leonardo
  message.attach(messageCompleted);   

}


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void  loop() { 

  looptime = micros()- milliloop;
  milliloop = micros();


  // ####### Messenger Aufruf, eingehende Strings werden geparst #############
  // The following line is the most effective way of 
  // feeding the serial data to Messenger
  while ( Serial.available( ) ) message.process(Serial.read( ) );  

  // ############################################### Infrarot Receiver #######################################################  
  if (irrecv.decode(&results)) {
    // If it's been at least 1/4 second since the last
    // IR received, toggle the relay
    if (millis() - last > 250) {   

      //   dump(&results);                          // -------------- hier auskommentieren löschen zum IFR Empfang test ------------------------------- 
      //     IRflag == true;  // nicht benutzt zur Zeit.     

      if (results.value == ENABLE_CODE) {  // IR Modus ein/ausschalten   ßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßß
        enable = !enable;                                  // Toggle Variable
        digitalWrite(RELAY_PIN, enable ? HIGH : LOW);  // die Test LED zum Funktionstest, kann auskommentiert werden.
        // wenn(?) ... wahr, dann X ansonsten(:) Y.  
        lampeBlink();
      }                                                                     



      if (results.value == LED_TOGGLE_CODE) {  // Lanpe ein/ausschalten   ßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßß
        on = !on;                                  // Toggle Variable
        //      digitalWrite(RELAY_PIN, on ? HIGH : LOW);  // die Test LED zum Funktionstest, kann auskommentiert werden.
      }      

      if (results.value == DIMM_PLUS_CODE) {  // Lampenhelligkeit erhöhen   ßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßß
        if (max_manuell < 255) {
          max_manuell = max_manuell + 5;                                  // Variable erhöhen.
          //      digitalWrite(RELAY_PIN, on ? HIGH : LOW);  // die Test LED zum Funktionstest, kann auskommentiert werden.
        }  
      }
      if (results.value == DIMM_MINUS_CODE) {  // Lampenhelligkeit verringern   ßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßß
        if (max_manuell > 10) {
          max_manuell = max_manuell - 5;                                  // Variable verringern.
          //      digitalWrite(RELAY_PIN, on ? HIGH : LOW);  // die Test LED zum Funktionstest, kann auskommentiert werden.

          EEPROM.writeInt(8, max_manuell);  // Wert speichern.
        }
      }     
      if (results.value == TIMER_HOCHDIMMEN_CODE) {  // Timer anschalten   ßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßß
        timer_on_manuell = daysec;                                  // Variable erhöhen.
        //      digitalWrite(RELAY_PIN, on ? HIGH : LOW);  // die Test LED zum Funktionstest, kann auskommentiert werden.
      }   
      if (results.value == TIMER_RUNTERDIMMEN_CODE) {  // Timer auschalten   ßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßßß
        timer_off_manuell = daysec;                                  // Variable erhöhen.
        //      digitalWrite(RELAY_PIN, on ? HIGH : LOW);  // die Test LED zum Funktionstest, kann auskommentiert werden.
      }   



      //   Serial.print("IR Sender value: ");  Serial.println(results.value, HEX);  // Überprüfen des IR Sendercodes.

    }
    last = millis();      
    irrecv.resume(); // Receive the next value  
  }

  // #################################### Mondlicht einschalten oder nicht ###########################################

  if (dimmer == 0) {     // Mondlicht wird in Abhängigkeit vom Dimmerwert(muss 0 sein) geschaltet.
    analogWrite(LEDmondlicht, mondlicht);
  }
  else {
    analogWrite(LEDmondlicht, 0);
  }

  // ################################## Die Zeitkorrektursekunden werden hochgezählt #################################
  if (secondx != second()) {     // ist eine Sekunde vergangen?
    kallibriersekunden  = kallibriersekunden + 1;     // dann einen hochzählen!
    secondx = second();    // Sekundencouter wieder zurücksetzen.
  }

  if (kallibriersekunden == kallibriersekundenEnd) {     // sind 43200 Sekunden vergangen? Dann Zeit neu kallibrieren.
    adjustTime(zeitkorrektur); // time.h Variable, adjust system time by adding the zeitdifferenz value.
    kallibriersekunden = 0;  // Wert zurücksetzen.
  }


  // #################################################  Thermistor   ##################################################
  temperatur = Thermister(analogRead(ntc));  // NTC am Analogen Eingang wird ausgelesen.

  // ################################################  PID Funktion  ##################################################
//  Input = temperatur;
//  myPID.Compute();
 // analogWrite(fan,Output);  // Lüfter wird PWM angesteuert.
 // pidwert = Output;

  
    pidwert = Compute(temperatur, setpointx, Kpx, Kix, Kdx, SampleTimex);  // PID Funktionsaufruf.
    pidFan = pidwert;
    analogWrite(fan,pidFan);  // Lüfter wird PWM angesteuert.
      

  // ############### Multimap Aufruf, der aktuelle Dimmerwert (dimmer) wird in Zeitabhängigkeit ermittelt. ############
  //dimmer = FmultiMap(daysec, in, out, 8); // Multimap: Dimmerwert ermitteln.  // für FLOAT
  dimmer = multiMap(daysec, in, out, 8); // Multimap: Dimmerwert ermitteln.  // für FLOAT

  // ####################### Die Betriebsminuten (Gesamtlaufzeit der Lampe) wird hochgezählt ##########################
  if (minutex != minute()) {     // ist eine Minute vergangen?
    betriebsminuten = betriebsminuten + 1;     // dann einen hochzählen!
    minutex = minute();    // Minutencouter wieder zurücksetzen.
  }


  // ######################### her werden aktuelle Datenstrings jede Sekunde an die serielle Schnittstelle gesendet ####################### 

  if(prevtime != now()){  // wenn eine Sekunde vergangen ist, den Code ausführen. prevtime und now ist UNIX Timestamp.

    daysec = (hour()*3600L)+(minute()*60L)+second();   // the seconds of the day, max. ist 86400 sec. Zeitbasis für die Zeitschaltuhr.  
    digitalToggle(LEDpin);   // ### Funktionskontrolle Programm LED alle Sekunde blinken lassen ###########
    prevtime = now();  // prevtime zurücksetzen
    gesendet = false;    //  gesendet flag auf false......
  }


  if (gesendet == false && StopSerial == false)  {  // ... und Daten senden wenn noch nicht gesendet und Stopflag nicht gesetzt.

    //   Serial.print('&'); // this is the Start header.   

    Serial.print("&J");   // binary data senden.
    Serial.print(now());  // alle Sekunde die UNIX Timestring an die serielle Schnittstelle senden.    
    Serial.print("$J"); // this is the End header mit carriage return

    Serial.print("&B"); // this is the header for the Betriebsminuten
    Serial.print(betriebsminuten);
    Serial.print("$B");  // ein Zeilenumbruch mehr vermeidet einen Fehler in VB. (oder auch nicht....)

    Serial.print("&A"); // Header Tag für die aktuellen Tagessekunden.
    Serial.print(daysec);
    Serial.print("$A"); // this is the End header.

    Serial.print("&D"); // // Header Tag für den aktuellen Dimmerwert der LED. 
    Serial.print(dimmer);
    Serial.print("$D"); // this is the End header.   

    Serial.print("&F"); // // Header Tag..
    Serial.print(F(firmware));
    Serial.print("$F"); // this is the End header.

    // ######## Thermistor Temperatur #############
    Serial.print("&Q"); // // Header Tag..
    Serial.print(temperatur);
    Serial.print("$Q"); // this is the End header.

    // ######## Lüfter PID Wert #############
    Serial.print("&R"); // // Header Tag..
    Serial.print(pidwert);  
    Serial.print("$R"); // this is the End header.

    // ######## LED maximal Helligleit Anzeigen #############
    Serial.print("&Y"); // // Header Tag..
    //Serial.print((int)outbound);  //outbound(float) in int umgewandelt gesendet.
    Serial.print(outbound);  //max. Helligkeitswert (0-255)
    Serial.print("$Y"); // this is the End header.

    // ######## Mondlicht Anzeigen #############
    Serial.print("&N"); // // Header Tag..
    Serial.print(mondlicht);  //mondlichtwert senden, 0 oder 255.
    Serial.print("$N"); // this is the End header.

    // ######## Temperaturvorgabe Anzeigen #############
    Serial.print("&O"); // // Header Tag..
    Serial.print((int)setpointx);  //
    Serial.print("$O"); // this is the End header.
    //delay(20);

    // ######## Phototransistor Wert Anzeigen #############
    photowert = analogRead(photo); 
    Serial.print("&H"); // // Header Tag..
    Serial.print(photowert);  //
    Serial.print("$H"); // this is the End header.
    //delay(20);

    // ######## Zeitdifferenz Anzeigen #############
    Serial.print("&C"); // // Header Tag..
    // Serial.print((int)zeitkorrektur);  //
    Serial.print(zeitkorrektur);  //
    Serial.print("$C"); // this is the End header.    

    // ######## Infrarotmodus AN/AUS #############
    Serial.print("&V"); // // Header Tag..
    // Serial.print((int)zeitkorrektur);  //
    Serial.print(enable);  //
    Serial.print("$V"); // this is the End header.    

    // ######## Message Anzeigen #############
    //    Serial.print("&M"); // // Header Tag..
    //  Serial.print(meldung);  //
    //   Serial.print("$M"); // this is the End header.  

    // ######## Durchlaufzeit Loop #############
    Serial.print("&Z"); // // Header Tag..
    Serial.print(looptime);   // Durchlaufzeit Loop wird gesendet.
    Serial.print("$Z"); // // Header Tag..   

    // ######## aktuelle Lampenhelligkeit #############
    Serial.print("&I"); // // Header Tag..
    Serial.print(hpled);   // aktuelle Lampenhelligkeit wird gesendet.
    Serial.print("$I"); // // Header Tag..   

    Serial.print("&!"); // // Header Tag..
    //  Serial.print(availableMemory());   // freier RAM wird gesendet.
    Serial.print(freeRam());   // freier RAM wird gesendet.
    //Serial.print(getFreeMemory());   // freier RAM wird gesendet.
    Serial.print("$!"); // // Header Tag

    // Serial.print("&?"); // // Header Tag..
    // Serial.print(string); // Echo the string
    // Serial.print("$?"); // // Header Tag

    //  if (on == 1) {    // das hier nur zum Testen der Funktion! 
    //  Serial.print("Infrarot high");  
    Serial.println('#');  // Ende Daten senden.............................................................


    gesendet = true; // gesendet flag setzen.

  }



  // ##############################################  Lampe schalten, dimmen  ###################################################


  if (enable == 0) {                                         // wenn IR ausgeschaltet, VBNet Dimmerwerte nehmen..................
    hpled = ((dimmer * outbound) / 255 );
    analogWrite(LEDhpled, hpled);      // Daten an High Power LED schreiben. (Wert: 0-255) = 0.00-255.00 / 255 * 0-255
  }



  else   {                                        // wenn IR eingeschaltet............................................
    if (on == 0) {                                    // wenn IR Code Lampe AUS gesetzt, 
      hpled = 0;
      analogWrite(LEDhpled, hpled); // Daten an High Power LED schreiben: Lampe AUS.
    }  

    if (on == 1) {                                    // wenn IR Code Lampe EIN gesetzt, 
      hpled = max_manuell;     
      analogWrite(LEDhpled, hpled); // Daten an High Power LED schreiben: Lampe EIN.
    }     


  }

}  // ##############################################  Loop zuende  ###################################################




// ############################################################## ENDE #########################################################



