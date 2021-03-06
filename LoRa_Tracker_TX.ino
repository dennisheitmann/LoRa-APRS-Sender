/*
  LoRa APRS Sender/Tracker for Dragino LoRa/GPS 433 MHz shield

  changes:
  - use PINs for Dragino shield
  - remove battery/voltage
  - works together with LoRa APRS Gateway
  - use Serial1 on Arduino Mega used instead of SoftwareSerial
*/

//Hardware definitions
const byte lora_PNSS = 10;      //pin number where the NSS line for the LoRa device is connected.
const byte PLED1 = 8;           //pin number for LED on Tracker
const byte lora_PReset = 9;     //pin where LoRa device reset line is connected
const byte lora_PPWMCH = 7;     //pin number for tone generation, connects to LoRa device DIO2.
static const uint32_t GPSBaud = 9600;     //GPS
const byte loraPower = 17;      //power in dBm

String Tcall = "DO7DH-12";     //your Call Sign
String InputString = "";        //data on buff is copied to this string
String Outputstring = "";       //data for output is copied to this string
String outString = "";          //The new Output String with GPS Conversion RAW
String sSymbol = ">";           //Symbol Code

#include <SPI.h>
#include "LoRaTX.h"
#include <TinyGPS++.h>
#include <math.h>

TinyGPSPlus gps;                  // The TinyGPS++ object


///////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
    {
      displayInfo();
    }

  if (millis() > 200000 && gps.charsProcessed() < 10)
  {
    digitalWrite(PLED1, HIGH);
    Serial.println("No GPS detected!");
    while (true);
  }

}
//////////////////////////////////////////////////////////////////////////////////////
//@APA calc encoded GPS Position
void recalcEncodedGPS() {
  Serial.println("------------------Encode Start---");
  String Ns, Ew;
  float Tlat, Tlon;
  int Talt;
  long Lat;
  long Lon;
  int Alt1, Alt2;
  Tlat = gps.location.lat();
  Tlon = gps.location.lng();
  Talt = gps.altitude.feet();
  Serial.print("Tlat is ");
  Serial.println(Tlat, 5);
  Serial.print("Tlon is ");
  Serial.println(Tlon, 5);
  Serial.print("Talt is ");
  Serial.println(Talt);
  if (Tlat < 0) {
    Ns = "S";
  } else {
    Ns = "N";
  }
  if (Tlon < 0) {
    Ew = "W";
  } else {
    Ew = "E";
  }
  Lat = 380926 * (90 - Tlat);
  Lon = 190463 * (180 + Tlon);
  outString = (Tcall);
  outString += ">APRS:!";
  outString += "/";
  //Calc Alt
  if (Talt > 0) {
    double ALT = log(Talt) / log(1.002);
    //Serial.println(ALT);
    Alt1 = ALT / 91;
    Alt2 = (int)ALT % 91;
  } else {
    Alt1 = 0;
    Alt2 = 0;
  }
  //Calc Lan
  int ilanFirst = Lat / pow(91, 3);
  //Serial.println(ilanFirst);
  long iTemp = Lat % (long)pow(91, 3);
  //Serial.println(iTemp);
  int ilanSecond = iTemp / pow(91, 2);
  //Serial.println(ilanSecond);
  iTemp = Lat % (long)pow(91, 2);
  //Serial.println(iTemp);
  int ilanThird = iTemp / 91;
  //Serial.println(ilanThird);
  int ilanFour = Lat % (long)91;
  outString += char(ilanFirst + 33);
  outString += char(ilanSecond + 33);
  outString += char(ilanThird + 33);
  outString += char(ilanFour + 33);


  //Calc Lon
  int ilonFirst = Lon / pow(91, 3);
  //Serial.println(ilonFirst);
  iTemp = Lon % (long)pow(91, 3);
  //Serial.println(iTemp);
  int ilonSecond = iTemp / pow(91, 2);
  //Serial.println(ilonSecond);
  iTemp = Lon % (long)pow(91, 2);
  //Serial.println(iTemp);
  int ilonThird = iTemp / 91;
  //Serial.println(ilonThird);
  int ilonFour = Lon % (long)91;
  outString += char(ilonFirst + 33);
  outString += char(ilonSecond + 33);
  outString += char(ilonThird + 33);
  outString += char(ilonFour + 33);
  outString += sSymbol;
  //outString +=" ";  //No Speed and Course
  outString += char(Alt1 + 33); //Altitude 1
  outString += char(Alt2 + 33); //Altitude 2
  outString += char(0x30 + 33);

  Serial.println("------------------Encode End---");
}

//////////////////////////////////////////////////////////////////////////////////////
//@APA Recalc GPS Position
void recalcGPS() {

  String Ns, Ew;
  float Tlat, Tlon;
  int Talt;
  float Lat;
  float Lon;
  Tlat = gps.location.lat();
  Tlon = gps.location.lng();
  Talt = gps.altitude.meters();
  Serial.print("Tlat is ");
  Serial.println(Tlat, 5);
  Serial.print("Tlon is ");
  Serial.println(Tlon, 5);
  Serial.print("Talt is ");
  Serial.println(Talt);
  if (Tlat < 0) {
    Ns = "S";
  } else {
    Ns = "N";
  }
  if (Tlon < 0) {
    Ew = "W";
  } else {
    Ew = "E";
  }
  if (Tlat < 0) {
    Tlat = -Tlat;
  }
  unsigned int Deg_Lat = Tlat;
  Lat = 100 * (Deg_Lat) + (Tlat - Deg_Lat) * 60;

  if (Tlon < 0) {
    Tlon = -Tlon;
  }
  unsigned int Deg_Lon = Tlon;
  Lon = 100 * (Deg_Lon) + (Tlon - Deg_Lon) * 60;
  outString = (Tcall);
  outString += ">APRS:!";
  if (Tlat < 10) {
    outString += "0";
  }
  outString += String(Lat, 2);
  outString += Ns;
  outString += char(92);
  if (Tlon < 100) {
    outString += "0";
  }
  if (Tlon < 10) {
    outString += "0";
  }
  outString += String(Lon, 2);
  outString += Ew;
  outString += sSymbol;
  outString += " /A=";
  outString += Talt;
  outString += "m";

  Serial.print("New Position ");
  Serial.println(outString);
}

/////////////////////////////////////////////////////////////////////////////////////////
void displayInfo()
{
  //Serial.print("Location: ");
  byte i;
  byte ltemp;
  Outputstring = "";

  if (gps.location.isValid())
  {
    //New System
    recalcEncodedGPS();
    Outputstring = outString;
    // at this point Outputstring has the LoRa Telemetry data to send
    Serial.print("OutputString is ");
    Serial.println(Outputstring);
    ltemp = Outputstring.length();
    lora_SetModem(lora_BW125, lora_SF12, lora_CR4_5, lora_Explicit, lora_LowDoptON);    //Setup the LoRa modem parameters
    lora_PrintModem();                //Print the modem parameters
    lora_TXStart = 0;
    lora_TXEnd = 0;

    for (i = 0; i <= ltemp; i++)
    {
      lora_TXBUFF[i] = Outputstring.charAt(i);
    }
    i--;
    lora_TXEnd = i;

    digitalWrite(PLED1, HIGH);        //LED on during packet
    lora_Send(lora_TXStart, lora_TXEnd, 60, 255, 1, 10, loraPower);	//send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
    digitalWrite(PLED1, LOW);
    lora_TXPKTInfo();			            //print packet information
    lora_TXBuffPrint(0);
    Serial.println();
    delay(20000);
  }
  else
  {
    // at this point Outputstring has the LoRa Telemetry data to send
    digitalWrite(PLED1, HIGH);
    Outputstring = "No GPS-Fix";
    Serial.println(Outputstring);

    ltemp = Outputstring.length();
    lora_SetModem(lora_BW125, lora_SF12, lora_CR4_5, lora_Explicit, lora_LowDoptON);    //Setup the LoRa modem parameters
    lora_PrintModem();                //Print the modem parameters
    lora_TXStart = 0;
    lora_TXEnd = 0;

    for (i = 0; i <= ltemp; i++)
    {
      lora_TXBUFF[i] = Outputstring.charAt(i);
    }
    i--;
    lora_TXEnd = i;

    digitalWrite(PLED1, HIGH);        //LED on during packet
    lora_Send(lora_TXStart, lora_TXEnd, 60, 255, 1, 10, loraPower);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
    digitalWrite(PLED1, LOW);
    lora_TXPKTInfo();                 //print packet information
    lora_TXBuffPrint(0);
    Serial.println();
    delay(20000);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
void addtostring(double lFloat, byte lmin, byte lprecision, String Stuff) // from dtostrf3
{
  char charVal[10];                             //temporarily holds data from vals
  memset(charVal, 0, sizeof(charVal));          //clear array
  InputString = "";
  dtostrf(lFloat, lmin, lprecision, charVal);   //lmin is mininum width, lprecision is precision
  for (int i = 0; i < sizeof(charVal); i++)
  {
    if  (charVal[i] == 0)
    {
      break;
    }
    InputString += charVal[i];
  }
  //return InputString;
  Outputstring = Outputstring + InputString + Stuff;
}

/////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);                           //Serial console ouput
  Serial.println("ProMiniLoRaTracker_V1");
  Serial.println(Tcall);
  Serial.println();
  pinMode(lora_PReset, OUTPUT);			            //RFM98 reset line
  digitalWrite(lora_PReset, LOW);		            //Reset RFM98
  pinMode (lora_PNSS, OUTPUT);			            //set the slaveSelectPin as an output:
  digitalWrite(lora_PNSS, HIGH);
  pinMode(PLED1, OUTPUT);			                  // for shield LED
  SPI.begin();					                        // initialize SPI:
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  lora_ResetDev();			                        //Reset the device
  lora_Setup();				                          //Do the initial LoRa Setup
  float LoRa_LFreq = 433.775;
  byte LoRa_LFMsb, LoRa_LFMid, LoRa_LFLsb;
  long LoRa_LLongFreq;
  LoRa_LLongFreq = ((LoRa_LFreq * 1000000) / 61.03515625);
  LoRa_LFMsb =  LoRa_LLongFreq >> 16;
  LoRa_LFMid = (LoRa_LLongFreq & 0x00FF00) >> 8;
  LoRa_LFLsb = (LoRa_LLongFreq & 0x0000FF);
  lora_SetFreq(LoRa_LFMsb, LoRa_LFMid, LoRa_LFLsb);
  Serial.println(LoRa_LFreq, 3);
  Serial.print("lora_SetFreq(");
  Serial.print(LoRa_LFMsb);
  Serial.print(",");
  Serial.print(LoRa_LFMid);
  Serial.print(",");
  Serial.print(LoRa_LFLsb);
  Serial.println(")");
  //lora_SetFreq(108, 113, 153);                  //Set the LoRa frequency, (433.775 Mhz = 108/113/153) lora_SetFreq(byte lora_LFMsb, byte lora_LFMid, byte lora_LFLsb)
  lora_Tone(1000, 1000, loraPower);                    //Transmit an FM tone, 1000hz, 100ms
  Serial1.begin(GPSBaud);                            //Startup soft serial for GPS
}
