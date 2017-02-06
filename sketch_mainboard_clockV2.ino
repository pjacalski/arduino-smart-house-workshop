#include <DallasTemperature.h>
#include <DS3231.h>
//#include <IRremote.h>
#include <OneWire.h>
#include <Wire.h>
#include "LedControl.h"

//---------------------------------------------------------------------------------------------------------------------------------
//I/O definitions

#define irPin 9
#define ONE_WIRE_BUS 8
#define DataIn 12
#define CLK 10
#define LOAD 11
#define BME280_ADDRESS 0x76


//Initializations
//IRrecv irrecv(irPin);
//decode_results results;
DS3231 clock;
RTCDateTime dt;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

LedControl lc = LedControl(DataIn, CLK, LOAD, 1);

//Variables
unsigned long int previousTime = 0;
unsigned long int previousTime2 = 0;
unsigned long int previousTime3 = 0;

bool FLAG_NEGATIVE = false;

float temp_out;
int i;
unsigned short int currentScreen = 0;
unsigned short int previousScreen = 0;

unsigned long int hum_raw, temp_raw, pres_raw;
signed long int t_fine;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;

uint16_t sensorValue = 0;
uint16_t sensorValues[5];
uint16_t sensorSum = 0;
int8_t outputValue = 4;
int8_t counter = 0;

int yy, mm, dd, ss, mon, hh;

bool relayState[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int macierz[8][8] = {{0, 0, 0, 0, 0, 0, 0, 0}, //time
  {0, 0, 0, 0, 0, 0, 0, 0}, //date
  {B00000110, B00010101, 0, 0, 0, 0, B01100011, B01001110}, //tin
  {B01111110, B00001111, 0, 0, 0, 0, B01100011, B01001110}, //tout
  {0, 0, 0, 0, 0, B00010111, B01100111, B00011101}, //press
  {0, 0, 0, 0, 0, 0, B01110011, B00011101}, //hum
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};

//---------------------------------------------------------------------------------------------------------------------------------


float liczbaa = 0;
int *modco(float liczbaa) {
  int czesci = 0;
  int jednosci = 0;
  int dziesiatki = 0;
  int setki = 0;
  int tysiace = 0;
  int liczba2 = 0;
  int liczba3 = 0;
  int liczba4 = 0;
  int liczba5 = 0;
  int a = 0;

  float liczbab = liczbaa * 10;
  int liczba = liczbab;

  czesci = liczba % 10;
  liczba2 = liczba / 10;
  jednosci = liczba2 % 10;
  liczba3 = liczba2 / 10;
  dziesiatki = liczba3 % 10;
  liczba4 = liczba3 / 10;
  setki = liczba4 % 10;
  liczba5 = liczba4 / 10;
  tysiace = liczba5 % 10;

  int tab[5];
  tab[0] = czesci;
  tab[1] = jednosci;
  tab[2] = dziesiatki;
  tab[3] = setki;
  tab[4] = tysiace;

  delay(1);
  return tab;
}


//---------------------------------------------------------------------------------------------------------------------------------

void setup() {

  uint8_t osrs_t = 1;             //Temperature oversampling x 1
  uint8_t osrs_p = 1;             //Pressure oversampling x 1
  uint8_t osrs_h = 1;             //Humidity oversampling x 1
  uint8_t mode = 3;               //Normal mode
  uint8_t t_sb = 5;               //Tstandby 1000ms
  uint8_t filter = 0;             //Filter off
  uint8_t spi3w_en = 0;           //3-wire SPI Disable

  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg  = osrs_h;

  Wire.begin();

  writeReg(0xF2, ctrl_hum_reg);
  writeReg(0xF4, ctrl_meas_reg);
  writeReg(0xF5, config_reg);
  readTrim();                    //
  // put your setup code here, to run once:
  clock.begin();
  sensors.begin();

  //irrecv.enableIRIn();


  lc.shutdown(0, false);
  lc.setIntensity(0, 4);
  lc.clearDisplay(0);

  Serial.begin(9600);
  Serial.println("initialisation successful");
}



//---------------------------------------------------------------------------------------------------------------------------------

void loop() {
  double temp_act = 0.0, press_act = 0.0, hum_act = 0.0;
  signed long int temp_cal;
  unsigned long int press_cal, hum_cal;

  if (millis() - previousTime2 > 1000) {

    yy = dt.year;
    mon = dt.month;
    dd = dt.day;
    hh = dt.hour;
    mm = dt.minute;
    ss = dt.second;

    readData();
    temp_cal = calibration_T(temp_raw);
    press_cal = calibration_P(pres_raw);
    hum_cal = calibration_H(hum_raw);
    temp_act = (double)temp_cal / 100.0;
    press_act = (double)press_cal / 100.0;
    hum_act = (double)hum_cal / 1024.0;
    previousTime2 = millis();

    sensors.requestTemperatures();
    temp_out = sensors.getTempCByIndex(0);

    int *wsk;

    Serial.print(dt.hour); Serial.print(":"); Serial.print(dt.minute); Serial.print(":"); Serial.println(dt.second);
    Serial.print(dt.day); Serial.print("-"); Serial.print(dt.month); Serial.print("-"); Serial.println(dt.year);
    Serial.print("Temp IN:  "); Serial.println(temp_act);
    Serial.print("Temp OUT: "); Serial.println(temp_out);
    Serial.print("Pressure: "); Serial.println(press_act);
    Serial.print("Humidity: "); Serial.println(hum_act);
    Serial.print("\n\n\n\n");

    if (temp_out < 0.0) {
      FLAG_NEGATIVE = true;
      temp_out = abs(temp_out);
    }
    else FLAG_NEGATIVE = false;


    /*
       Reverse engineering of the code:
                    O t   1 0.0 * C
       7segDisplay  7 6 5 4 3 2 1 0
       macierz      0 1 2 3 4 5 6 7
       tab            4 3 2 1 0
    */

    wsk = modco(temp_out);
    int tab[5];
    tab[0] = *wsk; tab[1] = *(wsk + 1); tab[2] = *(wsk + 2); tab[3] = *(wsk + 3); tab[4] = *(wsk + 4);
    macierz[3][5] = tab[0];
    macierz[3][4] = tab[1];
    if (FLAG_NEGATIVE && temp_out < 10.0)
      macierz[3][3] = 1;
    else
      macierz[3][3] = tab[2];
    if (FLAG_NEGATIVE && temp_out >= 10.0)
      macierz[3][2] = 1;
    else
      macierz[3][2] = tab[3];
    //macierz[3][1]=tab[4];

    wsk = modco(temp_act);
    tab[0] = *wsk; tab[1] = *(wsk + 1); tab[2] = *(wsk + 2); tab[3] = *(wsk + 3); tab[4] = *(wsk + 4);
    macierz[2][5] = tab[0];
    macierz[2][4] = tab[1];
    macierz[2][3] = tab[2];
    macierz[2][2] = tab[3];
    macierz[2][1] = tab[4];

    wsk = modco(hh);
    tab[0] = *wsk; tab[1] = *(wsk + 1); tab[2] = *(wsk + 2); tab[3] = *(wsk + 3); tab[4] = *(wsk + 4);
    macierz[0][2] = tab[2];
    macierz[0][3] = tab[1];

    wsk = modco(mm);
    tab[0] = *wsk; tab[1] = *(wsk + 1); tab[2] = *(wsk + 2); tab[3] = *(wsk + 3); tab[4] = *(wsk + 4);
    macierz[0][4] = tab[2];
    macierz[0][5] = tab[1];

    wsk = modco(ss);
    tab[0] = *wsk; tab[1] = *(wsk + 1); tab[2] = *(wsk + 2); tab[3] = *(wsk + 3); tab[4] = *(wsk + 4);
    macierz[0][6] = tab[2];
    macierz[0][7] = tab[1];

    wsk = modco(press_act);
    tab[0] = *wsk; tab[1] = *(wsk + 1); tab[2] = *(wsk + 2); tab[3] = *(wsk + 3); tab[4] = *(wsk + 4);
    macierz[4][4] = tab[0];
    macierz[4][3] = tab[1];
    macierz[4][2] = tab[2];
    macierz[4][1] = tab[3];
    macierz[4][0] = tab[4];

    wsk = modco(dd);
    tab[0] = *wsk; tab[1] = *(wsk + 1); tab[2] = *(wsk + 2); tab[3] = *(wsk + 3); tab[4] = *(wsk + 4);
    macierz[1][0] = tab[2];
    macierz[1][1] = tab[1];

    wsk = modco(mon);
    tab[0] = *wsk; tab[1] = *(wsk + 1); tab[2] = *(wsk + 2); tab[3] = *(wsk + 3); tab[4] = *(wsk + 4);
    macierz[1][2] = tab[2];
    macierz[1][3] = tab[1];

    wsk = modco(yy);
    tab[0] = *wsk; tab[1] = *(wsk + 1); tab[2] = *(wsk + 2); tab[3] = *(wsk + 3); tab[4] = *(wsk + 4);
    macierz[1][4] = tab[4];
    macierz[1][5] = tab[3];
    macierz[1][6] = tab[2];
    macierz[1][7] = tab[1];

    wsk = modco(hum_act);
    tab[0] = *wsk; tab[1] = *(wsk + 1); tab[2] = *(wsk + 2); tab[3] = *(wsk + 3); tab[4] = *(wsk + 4);
    macierz[5][3] = tab[2];
    macierz[5][4] = tab[1];
    macierz[5][5] = tab[0];
  }


  // put your main code here, to run repeatedly:
  dt = clock.getDateTime();
  //sensors.requestTemperatures();
  //tempout = sensors.getTempCByIndex(0);

  if (ss >= 5 && ss < 55) {
    if (ss >= 5 && ss < 10) currentScreen = 1;
    if (ss >= 10 && ss < 15) currentScreen = 0;
    if (ss >= 15 && ss < 20) currentScreen = 2;
    if (ss >= 20 && ss < 25) currentScreen = 3;
    if (ss >= 25 && ss < 35) currentScreen = 0;
    if (ss >= 35 && ss < 40) currentScreen = 4;
    if (ss >= 40 && ss < 50) currentScreen = 0;
    if (ss >= 50 && ss < 55) currentScreen = 5;
  }
  else currentScreen = 0;

  if (currentScreen != previousScreen) {
    previousScreen = currentScreen;
    lc.clearDisplay(0);
  }


  switch (currentScreen) {
    case 0: //time
      //lc.clearDisplay(0);
      lc.setDigit(0, 0, macierz[0][7], false);
      lc.setDigit(0, 1, macierz[0][6], false);
      lc.setDigit(0, 2, macierz[0][5], true);
      lc.setDigit(0, 3, macierz[0][4], false);
      lc.setDigit(0, 4, macierz[0][3], true);
      lc.setDigit(0, 5, macierz[0][2], false);
      break;

    case 1: //date
      //lc.clearDisplay(0);
      lc.setDigit(0, 0, macierz[1][7], false);
      lc.setDigit(0, 1, macierz[1][6], false);
      lc.setDigit(0, 2, macierz[1][5], false);
      lc.setDigit(0, 3, macierz[1][4], false);
      lc.setDigit(0, 4, macierz[1][3], true);
      lc.setDigit(0, 5, macierz[1][2], false);
      lc.setDigit(0, 6, macierz[1][1], true);
      lc.setDigit(0, 7, macierz[1][0], false);
      break;

    case 2: //tin
      //lc.clearDisplay(0);
      lc.setRow(0, 0, macierz[2][7]);
      lc.setRow(0, 1, macierz[2][6]);
      lc.setDigit(0, 2, macierz[2][5], false);
      lc.setDigit(0, 3, macierz[2][4], true);
      lc.setDigit(0, 4, macierz[2][3], false);
      lc.setRow(0, 6, B00010101);
      lc.setRow(0, 7, macierz[2][0]);
      break;

    case 3: //tout
      //lc.clearDisplay(0);
      lc.setRow(0, 0, macierz[3][7]);
      lc.setRow(0, 1, macierz[3][6]);
      lc.setDigit(0, 2, macierz[3][5], false);
      lc.setDigit(0, 3, macierz[3][4], true);
      if (FLAG_NEGATIVE && temp_out < 10.0)
        lc.setRow(0, 4, macierz[3][3]);
      else if (temp_out > 10.0)
        lc.setDigit(0, 4, macierz[3][3], false);
      if (FLAG_NEGATIVE && temp_out >= 10.0)
        lc.setRow(0, 5, macierz[3][2]);
      lc.setRow(0, 6, B00001111);
      lc.setRow(0, 7, macierz[3][0]);
      break;

    case 4: //press
      //lc.clearDisplay(0);
      lc.setRow(0, 0, macierz[4][7]);
      lc.setRow(0, 1, macierz[4][6]);
      lc.setRow(0, 2, macierz[4][5]);
      lc.setDigit(0, 3, macierz[4][4], false);
      lc.setDigit(0, 4, macierz[4][3], true);
      lc.setDigit(0, 5, macierz[4][2], false);
      lc.setDigit(0, 6, macierz[4][1], false);
      if (macierz[4][0] > 0)
        lc.setDigit(0, 7, macierz[4][0], false);
      break;

    case 5: //hum
      //lc.clearDisplay(0);
      lc.setRow(0, 0, macierz[5][7]);
      lc.setRow(0, 1, macierz[5][6]);
      lc.setDigit(0, 2, macierz[5][5], false);
      lc.setDigit(0, 3, macierz[5][4], true);
      lc.setDigit(0, 4, macierz[5][3], false);
      break;

  }

  if (previousTime3 - millis() > 100) {
    sensorValue = analogRead(A0);

    sensorValues[counter] = sensorValue;

    if (counter < 4)  counter++;
    else {
      counter = 0;
      for (i = 0; i < 4; i++) {
        sensorSum = sensorSum + sensorValues[i];
      }
      sensorSum = sensorSum / 5;
      outputValue = map(sensorSum, 0, 1023, 0, 8);
      lc.setIntensity(0, outputValue);
    }
  }

}


void readTrim()
{
  uint8_t data[32], i = 0;
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 24);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xA1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 1);
  data[i] = Wire.read();
  i++;

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xE1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 7);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  dig_H1 = data[24];
  dig_H2 = (data[26] << 8) | data[25];
  dig_H3 = data[27];
  dig_H4 = (data[28] << 4) | (0x0F & data[29]);
  dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
  dig_H6 = data[31];
}
void writeReg(uint8_t reg_address, uint8_t data)
{
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg_address);
  Wire.write(data);
  Wire.endTransmission();
}


void readData()
{
  int i = 0;
  uint32_t data[8];
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 8);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  hum_raw  = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T)
{

  signed long int var1, var2, T;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

unsigned long int calibration_P(signed long int adc_P)
{
  signed long int var1, var2;
  unsigned long int P;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
  if (var1 == 0)
  {
    return 0;
  }
  P = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (P < 0x80000000)
  {
    P = (P << 1) / ((unsigned long int) var1);
  }
  else
  {
    P = (P / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(P >> 2)) * ((signed long int)dig_P8)) >> 13;
  P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
  return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
  signed long int v_x1;

  v_x1 = (t_fine - ((signed long int)76800));
  v_x1 = (((((adc_H << 14) - (((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
            ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
                (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
                ((signed long int) dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (unsigned long int)(v_x1 >> 12);
}

