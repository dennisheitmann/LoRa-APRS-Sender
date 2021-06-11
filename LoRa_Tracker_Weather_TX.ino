/*
  LoRa APRS Sender/Tracker for Dragino LoRa/GPS 433 MHz shield

  changes:
  - use PINs for Dragino shield
  - remove battery/voltage
  - works together with LoRa APRS Gateway
  - use Serial1 on Arduino Mega used instead of SoftwareSerial
  - Send BME280 Weather Data
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
String sSymbol = "D";           //Symbol Code
String wSymbol = "_";           //Weather Symbol Code

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
float temp = (-99.0);
float baro = (-99.0);
float humi = (-99.0);

#include <SPI.h>
#include "LoRaTX.h"
#include <TinyGPS++.h>
#include <math.h>

TinyGPSPlus gps;                  // The TinyGPS++ object
uint16_t year;
uint8_t month;
uint8_t day;
uint8_t hour;
uint8_t minute;
uint8_t second;

///////////////////////////////////////////////////////////////////////////////////////
static void updateDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (d.isValid()) {
    year = d.year() + 2000;
    month = d.month();
    day = d.day();
  }

  if (t.isValid()) {
    second = t.second();
    minute = t.minute();
    hour = t.hour();
  }
}
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
String weather() {
  String weatherString = "";
  updateDateTime(gps.date, gps.time);
  char mon_buf[3];
  char day_buf[3];
  char hour_buf[3];
  char minute_buf[3];
  sprintf(mon_buf, "%02d", month);
  sprintf(day_buf, "%02d", day);
  sprintf(hour_buf, "%02d", hour);
  sprintf(minute_buf, "%02d", minute);
  temp = bme.readTemperature();
  int temp_f = (float)temp * 1.8 + 32;
  char temp_f_buf[4];
  sprintf(temp_f_buf, "%03d", temp_f);
  baro = bme.readPressure();
  char baro_buf[6];
  sprintf(baro_buf, "%05d", (int)(baro / 10));
  humi = bme.readHumidity();
  if (humi > 99) {
    humi = 99;
  }
  char humi_buf[3];
  sprintf(humi_buf, "%02d", (int)humi);
  String Ns, Ew;
  float Tlat, Tlon;
  int Talt;
  float Lat;
  float Lon;
  Tlat = gps.location.lat();
  Tlon = gps.location.lng();
  Talt = gps.altitude.meters();
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
  weatherString = (Tcall);
  weatherString += ">APRS:!";
  if (Tlat < 10) {
    weatherString += "0";
  }
  weatherString += String(Lat, 2);
  weatherString += Ns;
  weatherString += char(47);
  if (Tlon < 100) {
    weatherString += "0";
  }
  if (Tlon < 10) {
    weatherString += "0";
  }
  weatherString += String(Lon, 2);
  weatherString += Ew;
  weatherString += wSymbol;
  weatherString += ".../...g...t";
  weatherString += temp_f_buf;
  weatherString += "r...p...P...h";
  weatherString += humi_buf;
  weatherString += "b";
  weatherString += baro_buf;
  weatherString += "LoRa";
  return weatherString;
}
//////////////////////////////////////////////////////////////////////////////////////
//@APA Recalc GPS Position
String recalcGPS() {
  String outString = "";
  String Ns, Ew;
  float Tlat, Tlon;
  int Talt;
  float Lat;
  float Lon;
  Tlat = gps.location.lat();
  Tlon = gps.location.lng();
  Talt = gps.altitude.meters();
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
  outString += char(47);
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

  return outString;
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
    Outputstring = recalcGPS();
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
    delay(25000);
    // temp,pres,baro
    Outputstring = weather();
    // at this point Outputstring has the LoRa Telemetry data to send
    Serial.print("Outputstring is ");
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
    delay(25000);
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
  Serial.println("DennisLoRaAPRSWeatherTX");
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
  while (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
}
