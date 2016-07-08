/*!
* \file       onlyserial.ino
*
*
* \date       2015/03/15
*
* \author     Matthias Schibli (based on sketch from Sven Burkart and Christian Leder)
* 
* \brief      decode Piksi position, baseline and time

\details
- 
*/




#include <SPI.h>
#include <Wire.h>
#include <Time.h>


const unsigned int MAX_INPUT = 2000;
const int chipSelect = 10;
const float pi = 3.14159265359;

void setup()
{

    Serial.begin(115200);                                // serial to pc
    Serial2.begin (115200);                              // GPS
    Serial.println("init");

 
}


// Definition of SBP Protocol here: https://github.com/swift-nav/libswiftnav/raw/master/docs/sbp.pdf

#define		EndofMessage	                        0x55
#define 	Byte0LatLon 	                        0x01
#define 	Byte1LatLon				0x02
#define 	Byte0NED				0x03
#define		Byte1NED				0x02
#define		Byte0Time				0x00
#define		Byte1Time				0x01

#define		LengthLatLon	                        41
#define		LengthNED				29
#define		LengthTime				18


void loop()
{
    static double north=0;
    static double east=0;
    static double latitude=0;
    static double longitude=0;
    static long timestamp=0;
    int newdata=0;
    float heading=0;


    while (Serial2.available () > 0)
    {
        processIncomingByte (Serial2.read (),timestamp,north,east,latitude,longitude, newdata);
    }



    if (newdata==true)
    {
        Serial2.flush();

        Serial.print("timestamp ");
        Serial.print(timestamp);
        Serial.print(" north ");
        Serial.print(north);
        Serial.print(" east ");
        Serial.print(east);
        Serial.print(" latitude ");
        Serial.print(latitude,7);
        Serial.print(" longitude ");
        Serial.print(longitude,7);
        Serial.println();


    }
}







void processIncomingByte (const byte inByte,long& timestamp,double& north, double& east,double& latitude,double& longitude, int&newdata)
{
// Serial.println(inByte , HEX);

    static byte input_msg [MAX_INPUT];
    static unsigned int input_pos = 0;               // nicht negativer int und immer der gleiche int

    static int newdataTime=0;
    static int newdataNED=0;
    static int newdataLatLon=0;

    if (inByte == EndofMessage)
    {

        /*
        //print serial data from Piksi for debugging
        for( int i = 0 ; i < input_pos ; i++)
        {
                Serial.print(input_msg [i],HEX);       // wenn hex 55 print gesammter puffer [input_msg)
                Serial.print(" ");
        }
         Serial.println();
         */



        if (input_pos >=LengthTime || input_pos >=LengthNED || input_pos >=LengthLatLon)
        {

            // NED Message
            if((input_msg[0] == Byte0NED) && (input_msg[1] == Byte1NED) && input_pos == LengthNED)
            {
                msg_analyse (input_msg,north,east);
                newdataNED=1;
                input_pos = 0;
            }
            // LatLon Message
            else if((input_msg[0] == Byte0LatLon) && (input_msg[1] == Byte1LatLon)&& input_pos == LengthLatLon)
            {
                msg_analyse_latlon (input_msg,latitude,longitude);
                newdataLatLon=1;
                input_pos = 0;
            }

            // Time Message
            else if((input_msg[0] == Byte0Time) && (input_msg[1] == Byte1Time)&& input_pos == LengthTime)
            {
                msg_analyse_time (input_msg,timestamp);
                newdataTime=1;
                input_pos = 0;
            }

            else {}



        }
        // after message is parsed, set back to beginning
        input_pos = 0;

        if (newdataTime == true && newdataLatLon==true && newdataNED ==true)
        {
            newdata=true;
            newdataTime =false;
            newdataLatLon==false;
            newdataNED =false;
        }
    }

    // if no complete message in buffer, read out again and fill up input_msg
    else
    {

        if (input_pos < (MAX_INPUT - 1))
        {
            input_msg [input_pos] = inByte;
            input_pos = input_pos + 1;
        }
    }

}

void msg_analyse_latlon (byte byte_msg[41],double& latitude, double& longitude)
{

    latitude = Bytes2Double (byte_msg[16], byte_msg[15], byte_msg[14], byte_msg[13],byte_msg[12], byte_msg[11], byte_msg[10], byte_msg[9]);
    longitude = Bytes2Double (byte_msg[24], byte_msg[23], byte_msg[22], byte_msg[21],byte_msg[20], byte_msg[19], byte_msg[18], byte_msg[17]);
}


void msg_analyse (byte byte_msg[40],double& north, double& east)
{
    north = Bytes2Intu32 (byte_msg[12], byte_msg[11], byte_msg[10], byte_msg[9]);
    east = Bytes2Intu32 (byte_msg[16], byte_msg[15], byte_msg[14], byte_msg[13]);
}

void msg_analyse_time (byte byte_msg[40],long& timestamp)
{
    int nrofweek=0;
    int timeofweek=0;

    nrofweek = Bytes2Intu16 ( byte_msg[6], byte_msg[5]);
    timeofweek = Bytes2Intu32 (byte_msg[10], byte_msg[9], byte_msg[8], byte_msg[7]);

    timestamp=timeofweek+nrofweek*7*24*3600;
}



int Bytes2Intu32 (int b4, int b3, int b2, int b1)
{
    int result=0;
    result=(b4 << 24) | (b3<<16) | (b2<<8) | b1;
    return result;
}

// convert 2 bytes to int
int Bytes2Intu16 (int b2, int b1)
{
    int result=0;
    result=(b2<<8) | b1;
    return result;
}


// convert 8 bytes to double
double Bytes2Double (int b8,int b7,int b6,int b5,int b4, int b3, int b2, int b1)
{

    union u_tag
    {
        byte b[8];
        double fval;
    } u;

    u.b[0] = b1;
    u.b[1] = b2;
    u.b[2] = b3;
    u.b[3] = b4;
    u.b[4] = b5;
    u.b[5] = b6;
    u.b[6] = b7;
    u.b[7] = b8;

    return u.fval;
}


