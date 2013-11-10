


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ab hier FUNKTIONEN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// *********************************** PID *******************************************************************

 byte Compute(long Input, long Setpoint, long kp, long ki, long kd, long SampleTime)
{
  static byte ergebnisX;
  static unsigned long lastTime; 
  
  unsigned long jetzt = millis();              
   
   if(jetzt - lastTime > SampleTime)   
  {
    static long ITerm, lastInput, ergebnis;
    long  error, dInput;
    static long minErgebnis = 0; 
    static long maxErgebnis = 255;
    //long minITerm = 100*minErgebnis; maxITerm = 100*maxErgebnis;
    static long minITerm = 0; 
    static long maxITerm = 25500;

    
        
  
   error = Setpoint - Input;
   
    ITerm += (ki * error);
    if(ITerm > maxITerm) ITerm = maxITerm;          // faulty coding.............
    else if(ITerm < minITerm) ITerm = minITerm;       // faulty coding.............
     dInput = (Input - lastInput);

  ergebnis = ((kp * error) + ITerm - (kd * dInput))/100;
  
 if(ergebnis > maxErgebnis) ergebnis = maxErgebnis;
      else if(ergebnis < minErgebnis) ergebnis = minErgebnis;
      ergebnisX = ergebnis;

   lastInput = Input;
   lastTime = jetzt;
  
  return ergebnis;
}
else 
{ 
return ergebnisX; 
}
}


// ############################################# Dimmerwerte an serielle Schnittstelle ###################################################

int dimmerSchreiben() {
  
   char trenner = ','; 
   
    Serial.print("&L");  // Header Tag..    28.546 bytes
    Serial.print(in[0]);  //  ,0 = Nachkommastelle weglassen.
    Serial.print(trenner); 
    Serial.print(in[1]);
    Serial.print(trenner);  
    Serial.print(in[2]);
    Serial.print(trenner);     
    Serial.print(in[3]);
    Serial.print(trenner);    
    Serial.print(in[4]);
    Serial.print(trenner);     
    Serial.print(in[5]);
    Serial.print(trenner);    
    Serial.print(in[6]);
    Serial.print(trenner);     
    Serial.print(in[7]);
         Serial.print(trenner); 
    Serial.print(out[0]);  //  ,0 = Nachkommastelle weglassen.
    Serial.print(trenner); 
    Serial.print(out[1]);
    Serial.print(trenner);  
    Serial.print(out[2]);
    Serial.print(trenner);     
    Serial.print(out[3]);
    Serial.print(trenner);      
    Serial.print(out[4]);
    Serial.print(trenner);     
    Serial.print(out[5]);
    Serial.print(trenner);      
    Serial.print(out[6]);
    Serial.print(trenner);      
    Serial.print(out[7]);
    Serial.println("$L");
   
    
 //   sprintf(res, “%d\n%g\n%g”, a, b, d);    //  http://www.kriwanek.de/arduino/sprachreferenz/173-sprintf-ausgabeformatierung.html
    
   //     Serial.print(out[0],trenner,out[1],trenner,out[2],trenner,out[3],trenner,out[4],trenner,out[5],trenner,out[6],trenner,out[7],trenner);
  
  
// Serial.print(out[0],trenner);  
}



// ############################################# blink Routine ###################################################

int lampeBlink() {                  // Lampe zur Funktionsbestätigung blinken lassen.
  int toggleflag = 0;
  unsigned long lastmil = 0;
  boolean ena = false;
    lastmil = millis();    // Timervariable wird gestartet.
    analogWrite(LEDhpled, 0);      // Lampen Pin wird 0% Leistung.
    
    while (ena == false) {   // das jetzt wird gemacht, bis ena == true.
    
            if (millis() - lastmil > 300) {  // wenn 1500mS vergangen, 
   // analogWrite(LEDhpled, hpled);      // Lampen Pin wird wieder normale Leistung.
       ena = true;
        }
          else if (millis() - lastmil > 200) {  // wenn 1000mS vergangen, 
    analogWrite(LEDhpled, 0);      // Lampen Pin wird 0% Leistung.            
      }  
         else if (millis() - lastmil > 100) {  // wenn 500mS vergangen, 
    analogWrite(LEDhpled, 255);      // Lampen Pin wird 100% Leistung.
        }
     }
  }




/*    int availableMemory()
    {
     int size = 2560;
     byte *buf;
     while ((buf = (byte *) malloc(--size)) == NULL);
     free(buf);
     return size;
    }
    
    */
//Serial.print("free memory = ");
 //Serial.print(availableMemory());
// Serial.print(" - memory used = ");
 //Serial.println(2560-availableMemory());
 
/* int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
*/

int freeRam() {
  int byteCounter = 0; // initialize a counter
  byte *byteArray; // create a pointer to a byte array
  // More on pointers here: http://en.wikipedia.org/wiki/Pointer#C_pointers

  // use the malloc function to repeatedly attempt allocating a certain number of bytes to memory
  // More on malloc here: http://en.wikipedia.org/wiki/Malloc
  while ( (byteArray = (byte*) malloc (byteCounter * sizeof(byte))) != NULL ) {
    byteCounter++; // if allocation was successful, then up the count for the next try
    free(byteArray); // free memory after allocating it
  }
 
  free(byteArray); // also free memory after the function finishes
  return byteCounter; // send back the highest number of bytes successfully allocated
}

// ############################################# IR Routine, nur zum testen ###################################################
/*
void dump(decode_results *results) {
  int count = results->rawlen;
  if (results->decode_type == UNKNOWN) {
    Serial.println(F("Could not decode message"));
  } 
  else {
    if (results->decode_type == NEC) {
      Serial.print(F("Decoded NEC: ")); //By using the F() macro, the string is only copied 1 byte at a time, leaving the full string (character array) in FLASH.
    } 
    else if (results->decode_type == SONY) {
      Serial.print(F("Decoded SONY: "));
    } 
    else if (results->decode_type == RC5) {
      Serial.print(F("Decoded RC5: "));
    } 
    else if (results->decode_type == RC6) {
      Serial.print(F("Decoded RC6: "));
    }
    Serial.print(results->value, HEX);
    Serial.print(" (");
    Serial.print(results->bits, DEC);
    Serial.println(F(" bits)"));
  }
  Serial.print(F("Raw ("));
  Serial.print(count, DEC);
  Serial.print(F("): "));

  for (int i = 0; i < count; i++) {
    if ((i % 2) == 1) {
      Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
    } 
    else {
      Serial.print(-(int)results->rawbuf[i]*USECPERTICK, DEC);
    }
    Serial.print(" ");
  }
  Serial.println("");
}
*/
// ############################################# Konsolen Routinen ###################################################
void stop() {          // stopt serielle Kommunikation.
StopSerial = true;   
}

void start() {          // startet serielle Kommunikation.
StopSerial = false;   
}


void auslesen() {       // leist alle EEPROM Speicherstellen als byte bis 1000 und gibt sie auf der setiellen Schnittstelle aus.
  for (stelle = 0; stelle < 1000; stelle++){
      //delay(1);
      lesen = EEPROM.read(stelle);
           Serial.print(stelle);
           Serial.print(F(" > "));
           Serial.println(lesen);
 
           if (stelle == 1000)
         {
           stelle = 0;
           break;
         }
  }
}
/*
void auslesen_long() {         // ######################### entbehrliche Funktion!!! #######################################
  for (stelle = 0; stelle < 1000; stelle = stelle+4){
      //delay(1);
      lesen_long = EEPROM.readLong(stelle);
           Serial.print(stelle);
           Serial.print(F(" > "));
           Serial.println(lesen_long);
 
           if (stelle == 1000)
         {
           stelle = 0;
           break;
    }
  }
}
*/
void loeschen(){            // löscht alle EEPROM Speicherstellen (überschreiben mit 0) bis 1000 und gibt sie auf der setiellen Schnittstelle aus.
  for (stelle = 0; stelle < 1000; stelle++){
    EEPROM.write(stelle, 0);
      //delay(1);
      lesen = EEPROM.read(stelle);
           Serial.print(stelle);
           Serial.print(F(" > "));
           Serial.print(0);
           Serial.print(F(" > "));
           Serial.print(lesen);
           
           if (lesen == 0)       // Löschergebnis wird getestet.
           {
             Serial.println(F("OK!"));
           }
           else 
           {
             Serial.println(F("ER!"));
             break;
           }
           if (stelle == 1000)
         {
           stelle = 0;
      //     schreiben = 1;
           break;
     }
  }
}


// ########################### TIME Funktion für die Zeitsynchronistation mit dem PC #################################

boolean getPCtime() {
  // if time sync available from serial port, update time and return true
      setTime(pctime);   // Sync Arduino clock to the time received on the serial port
      return true;   // return true if time message received on the serial port           
  }

// ################################## Thermistor Funktion zur Temperaturmessung #######################################

double Thermister(int RawADC) {
 double Temp;
 Temp = log(((10240000/RawADC) - ResValue));
 Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
 Temp = Temp - 273.15;            // Convert Kelvin to Celcius
 return Temp;
 }
 
 

// ############################# Interruptroutine für EEPROM Speicherung bei Stromabschaltung. ########################

void ExtInterrupt(){ 

//fan = LOW;
//LEDmondlicht = LOW;
//hpled = LOW;
//LEDpin = LOW;
//ntc = LOW;

//PORTD = B00000100;
//PORTB = B00000000;
//PORTC = B00000000;
//sprintf(meldung, "Zeit gespeichert");

  Serial.println(F("&MZeit safe$M"));

 //time_t t = now(); // store the current time in time variable t 

//EEPROM.writeLong(1, t);    // Zeit in EEPROM.
//EEPROM.writeLong(105, betriebsminuten);   // Einschaltdauer in EEPROM.
}

// ############################# Interruptroutine für IR Modus. ########################
void IRtoggle(){
  enable = 1; 
}

// #############################  EEPROM Speicherung Daten ########################

void safe(){                     // wird duch Taste in VBNet Programm ausgelöst, sollte auch bei Stromabschaltung gehen.

EEPROM.writeLong(1, now());  // Zeit speichern, UNIX Timestring. Speicherstelle 6 defekt??
EEPROM.writeInt(5, outbound);

EEPROM.write(7, mondlicht);
EEPROM.write(8, max_manuell);
EEPROM.write(9, 111);            // Kontrollbyte: 111 wird geschrieben, um Speicherung zu bestätigen.

EEPROM.writeLong(20, in[0]);
EEPROM.writeLong(25, in[1]);
EEPROM.writeLong(30, in[2]);
EEPROM.writeLong(35, in[3]);
EEPROM.writeLong(40, in[4]);
EEPROM.writeLong(45, in[5]);
EEPROM.writeLong(50, in[6]);
EEPROM.writeLong(55, in[7]);

EEPROM.writeLong(60, out[0]);
EEPROM.writeLong(65, out[1]);
EEPROM.writeLong(70, out[2]);
EEPROM.writeLong(75, out[3]);
EEPROM.writeLong(80, out[4]);
EEPROM.writeLong(85, out[5]);
EEPROM.writeLong(90, out[6]);
EEPROM.writeLong(95, out[7]);
  
EEPROM.updateLong(100, setpointx); // 8 Stellen, Dimmerwert speichern.     

EEPROM.writeLong(110, betriebsminuten);
//EEPROM.writeInt(120, zeitkorrektur);  // Zeitkorrekturspeicherung wird extra ausgelöst!

//sprintf(meldung, "Daten gespeichert");

  Serial.println(F("&MDatSafe$M"));


}

// ##################################### Multimap funktion für die Zeitschaltuhr #######################################
/*
float FmultiMap(float val, float * _in, float * _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}
*/
int multiMap(long val, long* _in, long* _out, uint8_t size)  // Int Funktion in Long umgeschrieben, spart etwa 300 bytes! ***************************************
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];                   // Float-Int gleich!
  if (val >= _in[size-1]) return _out[size-1];         // Float-Int gleich!

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested          // Float-Int gleich!
  while(val > _in[pos]) pos++;                         // Float-Int gleich!

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];               // Float-Int gleich!

  // interpolate in the right segment for the rest
  return map(val, _in[pos-1], _in[pos], _out[pos-1], _out[pos]);
}

// #################### MESSENGER, hier werden die eingehenden Strings der ser. Schnittstelle geparst ##################
// Die Werte werden durch ein "," getrennt, oben in der Initialisierung festgelegt.

// ************************************************** Parser für serielle Befehle ************************************************************ 

// ############################# Konsolen Befehle ##################################################

void messageCompleted() {
  while (message.available())  {
    
 // message.copyString(string,MAXSIZE);
  // Serial.println(string); // Echo the string
    
  if (message.checkString("dimmw")) // Dimmerwerte für vb.net schreiben.  // ############### wenn serieller Befehl kommt.... #################
  {
    dimmerSchreiben();    
  }
  
  if (message.checkString("stop")) // serieller Ausgang wird gestopt.  // ############### wenn serieller Befehl kommt.... #################
  {
    stop();    
  }
  
  if (message.checkString("start")) // serieller Ausgang wird gestartet.
  {
    start();    
  }
  
  
  if (message.checkString("lesen")) // EEPROM Speicherinhalt wird unformatiert als byte gelesen.
  {
    auslesen();    
  }
/*  
  if (message.checkString("lesenlong")) // EEPROM Speicherinhalt wird unformatiert als long gelesen.
  {
    auslesen_long();    
  }    
*/
  if (message.checkString("loeschen")) // EEPROM Speicherinhalt löschen.
  {
    loeschen();    
  }  
  
  if (message.checkString("safe")) // EEPROM Speicher manuell schreiben.
  {
    safe();  
     Serial.println(F("EEsafe"));  
  } 
  
/*  if (message.checkString("dimmer")) // Dimmerwerte lesen.
  {
   stop();                                // senden ausschalten  
Serial.println(F("Dimmerwerte:"));
Serial.print(in[0]);
Serial.print(F(" >> "));
Serial.println(out[0]);
Serial.print(in[1]);
Serial.print(F(" >> "));
Serial.println(out[1]);
Serial.print(in[2]);
Serial.print(F(" >> "));
Serial.println(out[2]);
Serial.print(in[3]);
Serial.print(F(" >> "));
Serial.println(out[3]);
Serial.print(in[4]);
Serial.print(F(" >> "));
Serial.println(out[4]);
Serial.print(in[5]);
Serial.print(F(" >> "));
Serial.println(out[5]);
Serial.print(in[6]);
Serial.print(F(" >> "));
Serial.println(out[6]);
Serial.print(in[7]);
Serial.print(F(" >> "));
Serial.println(out[7]);
  } 
  
*/  
 
 if (message.checkString("eeprom")) // EEPROM Speicherinhalt wird strukturiert gelesen.
  {
    stop();                                // senden ausschalten
  
Serial.print(F("EEbyte= "));
Serial.println(EEPROM.read(9));

Serial.print(F("0= "));
Serial.println(EEPROM.readLong(60));
Serial.print(F("1= "));
Serial.println(EEPROM.readLong(65));
Serial.print(F("2= "));
Serial.println(EEPROM.readLong(70));
Serial.print(F("3= "));
Serial.println(EEPROM.readLong(75));
Serial.print(F("4= "));
Serial.println(EEPROM.readLong(80));
Serial.print(F("5= "));
Serial.println(EEPROM.readLong(85));
Serial.print(F("6= "));
Serial.println(EEPROM.readLong(90));
Serial.print(F("7= "));
Serial.println(EEPROM.readLong(95));
Serial.println();

Serial.print(F("i0= "));
Serial.println(EEPROM.readLong(20));
Serial.print(F("i1= "));
Serial.println(EEPROM.readLong(25));
Serial.print(F("i2= "));
Serial.println(EEPROM.readLong(30));
Serial.print(F("i3= "));
Serial.println(EEPROM.readLong(35));
Serial.print(F("i4= "));
Serial.println(EEPROM.readLong(40));
Serial.print(F("i5= "));
Serial.println(EEPROM.readLong(45));
Serial.print(F("i6= "));
Serial.println(EEPROM.readLong(50));
Serial.print(F("i7= "));
Serial.println(EEPROM.readLong(55));
Serial.println();

t = EEPROM.readLong(1);
Serial.print(F("UNIX-t= "));
Serial.println(EEPROM.readLong(1));

Serial.print(F("mond= "));
Serial.println(EEPROM.read(7));

Serial.print(F("max.LED= "));
Serial.println(EEPROM.readInt(5));

Serial.print(F("PIDfan= "));
Serial.println(EEPROM.readLong(100)); 

Serial.print(F("KorrZeit= "));
Serial.println(EEPROM.readInt(120)); 

Serial.print(F("Betr.min= "));
Serial.println(EEPROM.readLong(110)); 

EEPROM.writeLong(110, betriebsminuten);

Serial.print(F("IR MaxLampe= "));
Serial.println(EEPROM.read(8));  // IR Manueller Maximalwert der Lampe.
  
  }   
 
 // ********************************* Parser für Dimmerwerte senden ******************************************    

  else if (message.checkString("&H"))
    {
      dimmerSchreiben();
     
       Serial.println(F("&MDimmSend$M"));
    }   
  
// ******************************************* Parser für Time sync ************************************************* 
    
 if (message.checkString("&J")) // if time sync available from serial port, update time and return true
  {
 //   long pctime = message.readLong();   
     pctime = message.readLong();   //getPCtime();
      setTime(pctime);   // Sync Arduino clock to the time received on the serial port
 
// sprintf(meldung, "Controller Zeit aktualisiert");    // meldung wird in VBNet Programm angezeigt.
 
   Serial.println(F("&MyCZeit upd$M"));

  }

// ************************************ Parser für Lampen Mximalhelligkeit ******************************************    

 if (message.checkString("&Y"))
    {
      outbound = message.readInt(); 
    
       Serial.println(F("&Mmax.Lamp upd$M"));
  
    } 

// ********************************* Parser für Betriebsminuten rücksetzen ******************************************    

  else if (message.checkString("&K"))
    {
      betriebsminuten = message.readInt();
     
       Serial.println(F("&MBetriebsmin. upd$M"));
    }     

// ************************************* Parser für PID Temperaturvorgabe *******************************************    

 if (message.checkString("&O"))
    {
      setpointx = message.readInt();
    
     Serial.println("&MTemp upd$M");
    } 
// ******************************************* Parser Zeitkorrektur *************************************************    

 if (message.checkString("&C"))
    {
      zeitkorrektur = message.readInt();
       
EEPROM.writeInt(120, zeitkorrektur);

    adjustTime(zeitkorrektur); // time.h hier einmal den Wert erneuern!
      kallibriersekunden = 0;  // hier einmal die Kallibrierzeit zurücksetzen für akkurate kallibrierung.

// sprintf(meldung, "Zeitkorrektur aktualisiert");  // meldung wird in VBNet Programm angezeigt.

     Serial.println(F("&MZeitkor.upd$M"));

   
      //delay(10);    // Dealay nötig, damit VB den String erkennt.
    }    
    
// ********************************************* Parser für Mondlicht ***********************************************    

 if (message.checkString("&N"))
    {
      mondlicht = message.readInt();
      analogWrite(LEDmondlicht, mondlicht);
               
 //sprintf(meldung, "Mondlicht aktualisiert");
 
      Serial.println(F("&MMondUpd$M"));

      //delay(10);    // Dealay nötig, damit VB den String erkennt.
    }

// ********************************************** Parser für Steuerbefehle ******************************************    

 if (message.checkString("&G"))
    {
      befehle = message.readInt();
      
if (befehle == 1) {  // ...
  ExtInterrupt();
}
      //delay(10);    // Dealay nötig, damit VB den String erkennt.
    }
    
//********************************************** Parser für Zeitschaltuhr, Datum **************************************** 

  if (message.checkString("&P"))  {

      for (int i=0;i<8;i++) { 
        long varlong2 = message.readLong();    
        in[i] = varlong2;
      }

      i = 0;
      
// sprintf(meldung, "Schaltuhr Zeit aktualisiert");  // meldung wird in VBNet Programm angezeigt.

      Serial.println(F("&MTimerZeitUpd$M"));


 //delay(20);    // Dealay nötig, damit VB den String erkennt.
 
    }   

// **************************************** Parser für Zeitschaltuhr, Lichthelligkeit **************************************** 

  if (message.checkString("&W"))  {

      for (int i=0;i<8;i++) { 
        long varlong = message.readInt();    
        out[i] = varlong;
      }

      i = 0;


 //sprintf(meldung, "Schaltuhr Wert aktualisiert");  // meldung wird in VBNet Programm angezeigt.
  Serial.println("&MTimerWertUpd$M");

 
 //     Serial.println("M Schaltuhr Leistung aktualisiert$"); // Statusmeldung an VB, wenn zu lang. yC stüzt ab!!
 //     Serial.println(); // Terminate message
       //delay(20);    // Dealay nötig, damit VB den String erkennt.     
      //analogWrite(hpled, atoi(outbound)); // atoi wandelt zeichenkette in Zahl    
    }   
    
// ********************************************* Parser Setup im EEPROM speichern  ***********************************************    

 if (message.checkString("&E")) {
  safe();   
               
               
 //sprintf(meldung, "Daten gespeichert");
 
   Serial.println(F("&MDaten safe$M"));

      //delay(10);    // Dealay nötig, damit VB den String erkennt.
    }
    
    // ********************************************* Parser Infrarotmodus  ***********************************************    

 if (message.checkString("&V")) {
 // IRtoggle();   
 // enable = message.readInt();
int tempe = message.readInt();
if (tempe == 1) {
   enable = 1;
  digitalWrite(RELAY_PIN, HIGH); 
      }
      
      else {
   enable = 0;
  digitalWrite(RELAY_PIN, LOW); 
      }
      
      Serial.println(F("&MIR tog$M"));
               
 //sprintf(meldung, "Daten gespeichert");
      //delay(10);    // Dealay nötig, damit VB den String erkennt.
    }
    
// ********************************************* kein Befehl erkannt ***********************************************      
      else {
  // sprintf(meldung, "Befehl nicht erkannt");  // meldung wird in VBNet Programm angezeigt.
  // Serial.println("&MBefehl nicht erkannt$M");

   
   break;

    }   
 
  } 
}    
  
  

