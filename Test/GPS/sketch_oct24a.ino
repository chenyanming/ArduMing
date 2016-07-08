//---------------------------//
//----integration Piksi------//
//----Sven Burkart-----------//
//----Christian Leder--------//
//----Projekt HS 2014--------//
//---------------------------//

const unsigned int MAX_INPUT = 200;                    // max anzhalt message bytes (längere messages werden nicht gespeichert)

void setup() {
                                                       // put your setup code here, to run once:
  
  Serial.begin(115200);                                //usb TX0
  Serial2.begin (115200);                              //GPS TX2
  

}

void loop() {
                                                      // put your main code here, to run repeatedly:

  while (Serial2.available () > 0)                   //wenn von piksi auf Arduinos TX 2 etwas empfangen wird, 
  {
    processIncomingByte (Serial2.read ());           //führe funktion process incomings aus. 
  }
}
                                                    // Protokoll auswerten  - Funktion um Aufzuschlüsseln
void processIncomingByte (const byte inByte)        //funktion um SBP zu filtern und nur die benötigte msg zu verwenden
{
  //Serial.println(inByte , HEX);
  
  static byte input_msg [MAX_INPUT];
  static unsigned int input_pos = 0;               // nicht negativer int und immer der gleiche int
  
  if (inByte == 0x55)
  {
    if (input_pos == 29)
    {
      for( int i = 0 ; i < input_pos ; i++)
      {
        //Serial.print(input_msg [i],HEX);       // wenn hex 55 print gesammter puffer [input_msg)
        //Serial.print(" ");
      }    
      //Serial.println();                       // neue Zeile nach kpl. msg print
      
      if((input_msg[0] == 0x03) && (input_msg[1] == 0x02))
      {
        msg_analyse (input_msg);              //funktion um die herausgefilterte msg zu analysieren
      }
    }
    input_pos = 0;                            // und setzte input_pos wieder auf 0
  }
  else
  {
    if (input_pos < (MAX_INPUT - 1))         // nur wenn pufferlänge nicht erreicht ist (in unserem Falle, wird das nicht der Fall sein)        
    {
      input_msg [input_pos] = inByte;        // schreibt byte in puffer an position 0
      input_pos = input_pos + 1;             // erhöht position im Puffer um 1
    }
  }
  
}

void msg_analyse (byte byte_msg[29])        //funktion um die herausgefilterte msg zu analysieren
{
  double east = 0;
  double north = 0;
  double dist=0;
  north = bytesToInt (byte_msg[12], byte_msg[11], byte_msg[10], byte_msg[9]);        //vertikale abweichung von basis in  (aufruf der Funktion und welche bytes dazu verwendet werden)
  east = bytesToInt (byte_msg[16], byte_msg[15], byte_msg[14], byte_msg[13]);        //horizontale abweichung von basis in mm (aufruf der Funktion und welche bytes dazu verwendet werden)

  Serial.print("north ");
  Serial.print(north);
  Serial.print(" ; east ");
  Serial.print(east);
  
  dist=sqrt(north*north+east*east);                // calc direct distance to base
  Serial.print(" ; direkte Distanz zur Basis ");            //direct distance to base in mm
  Serial.println(dist);
  
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
  //----calculating perpendicular distance to a line (point A to Point B)----//
  //----point A must be the Base-----Point be must be manualy definied-------//
  //----bei negativer perp_distance nach rechts korrigieren------------------//
  //----bei postitiver perp_distance nach links korrigieren------------------//
  
  int B_east = 5543.00;                                                     //east koordinate in mm von Basis für Punkt B    manuell definiert
  int B_north = -7989.00;                                                    //north koordinate in mm von Basis für Punkt B   manuell definiert
  int A_east = 1826.00;                                                      //east koordinate in mm von Basis für Punkt A    manuell definiert
  int A_north = -2329.00;                                                    //north koordinate in mm von Basis für Punkt A   manuell definiert
  int A = 0;                                                         // A aus allgemeiner Geradengleichung = Steigung
  int B = -1;                                                        // B aus allgemeiner Geradengleichung Ax + Bx + C 
  int C = 0;                                                         // C aus allgemeiner Geradengleichung (C = Y-Achsenabschnitt)
  int perp_distance = 0;
  
  A=(B_north-A_north)/(B_east-A_east);                                //berechnen der Steigung
  C = B_north-(A*B_east);                                            //Berechnen der Y-Achsenabschnitt
  perp_distance = ((A*east)+(B*north)+C)/sqrt(A*A+B*B);              //Formel für perpendicular Distance to a line
  Serial.print("perpendicular Distance to line from Point A ( "); Serial.print(A_east);Serial.print(","); Serial.print(A_north); Serial.print(") to Point B ("); Serial.print(B_east);Serial.print(",");Serial.print(B_north);Serial.print("): ");
  Serial.println(perp_distance);
  Serial.println();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
}


int bytesToInt (int b4, int b3, int b2, int b1)    //4 bytes verschieben, zusammensetzen und zu dezimalzahl wandeln
{
  int resultat=0;
  resultat=(b4 << 24) | (b3<<16) | (b2<<8) | b1;   //bytes in resultatvariable nach links verschieben bis aus 4 bytes eine zahl wird
  return resultat;
}
