
//Определение программно реализуемых хостов шин I2C. Каждый хост работает в режиме
//Master. Число хостов может быть объявлено несколько, работа программного хоста
//не зависит от наличия физического модуля I2C в применяемом МК.

#include <SoftWire.h>
SoftWire SWire_CH1(PB8, PB9, SOFT_FAST);
SoftWire SWire_CH2(PB10, PB11, SOFT_FAST);

#include <usb_serial.h>
USBSerial myUserial;
char buf[100];

//================================
  uint16_t C[8];
  uint32_t Ds1, Ds2;
  int32_t TEMP,TEPM1,TEMP2;
  int32_t P,P1,P2;
  uint8_t _model = 1; //MS5837_02BA
  uint8_t crc4(uint16_t n_prom[]);
  
#define MS5837_ADDR               0x76  
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

const float Pa = 100.0f;
const float bar = 0.001f;
const float mbar = 1.0f;


//Начальная инициализация и считывание каллибровочных коэффициентов
bool inits(SoftWire SWire) 
{
 // Reset the MS5837, per datasheet
 SWire.beginTransmission(MS5837_ADDR);
 SWire.write(MS5837_RESET);
 SWire.endTransmission();

  // Wait for reset to complete
  delay(10);

  // Read calibration values and CRC
  for ( uint8_t i = 0 ; i < 7 ; i++ ) {
   SWire.beginTransmission(MS5837_ADDR);
   SWire.write(MS5837_PROM_READ+i*2);
   SWire.endTransmission();

   SWire.requestFrom(MS5837_ADDR,2);
    C[i] = (SWire.read() << 8) |SWire.read();
  }

  // Verify that data is correct with CRC
  uint8_t crcRead = C[0] >> 12;
  uint8_t crcCalculated = crc4(C);

  if ( crcCalculated == crcRead ) {
    return true; // Initialization success
  }
  
  return false; // CRC fail
};

//Чтение данных с датчика давления
void read_data(SoftWire SWire)
{
 // Request Ds1 conversion
 SWire.beginTransmission(MS5837_ADDR);
 SWire.write(MS5837_CONVERT_D1_8192);
 SWire.endTransmission();

 delay(20); // Max conversion time per datasheet
  
 SWire.beginTransmission(MS5837_ADDR);
 SWire.write(MS5837_ADC_READ);
 SWire.endTransmission();

 SWire.requestFrom(MS5837_ADDR,3);
  Ds1 = 0;
  Ds1 =SWire.read();
  Ds1 = (Ds1 << 8) | SWire.read();
  Ds1 = (Ds1 << 8) | SWire.read();
  
  // Request D2 conversion
 SWire.beginTransmission(MS5837_ADDR);
 SWire.write(MS5837_CONVERT_D2_8192);
 SWire.endTransmission();

 delay(20); // Max conversion time per datasheet
  
 SWire.beginTransmission(MS5837_ADDR);
 SWire.write(MS5837_ADC_READ);
 SWire.endTransmission();

 SWire.requestFrom(MS5837_ADDR,3);
  Ds2 = 0;
  Ds2 =SWire.read();
  Ds2 = (Ds2 << 8) |SWire.read();
  Ds2 = (Ds2 << 8) |SWire.read();

  calculate();
}

float pressure(float conversion) {
    if ( _model == 1 ) { //MS5837_02BA
        return P*conversion/100.0f;
    }
    else {
        return P*conversion/10.0f;
    }
}

float temperature() {
  return TEMP/100.0f;
}

uint8_t crc4(uint16_t n_prom[]) {
  uint16_t n_rem = 0;

  n_prom[0] = ((n_prom[0]) & 0x0FFF);
  n_prom[7] = 0;

  for ( uint8_t i = 0 ; i < 16; i++ ) {
    if ( i%2 == 1 ) {
      n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
    } else {
      n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
    }
    for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
      if ( n_rem & 0x8000 ) {
        n_rem = (n_rem << 1) ^ 0x3000;
      } else {
        n_rem = (n_rem << 1);
      }
    }
  }
  
  n_rem = ((n_rem >> 12) & 0x000F);

  return n_rem ^ 0x00;
}


void calculate() {
 // Given C1-C6 and Ds1, Ds2, calculated TEMP and P
  // Do conversion first and then second order temp compensation
  
  int32_t dT = 0;
  int64_t SENS = 0;
  int64_t OFF = 0;
  int32_t SENSi = 0;
  int32_t OFFi = 0;  
  int32_t Ti = 0;    
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;
  
  // Terms called
  dT = Ds2-uint32_t(C[5])*256l;
  if ( _model == 1 ) { //MS5837_02BA
    SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l;
    OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l;
    P = (Ds1*SENS/(2097152l)-OFF)/(32768l);
  } else {
    SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
    OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
    P = (Ds1*SENS/(2097152l)-OFF)/(8192l);
  }
  
  // Temp conversion
  TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;
  
  //Second order compensation
  if ( _model == 1 ) { //MS5837_02BA
    if((TEMP/100)<20){         //Low temp
      Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
      OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
      SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
    }
  } else {
    if((TEMP/100)<20){         //Low temp
      Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
      OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
      SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
      if((TEMP/100)<-15){    //Very low temp
        OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
        SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
      }
    }
    else if((TEMP/100)>=20){    //High temp
      Ti = 2*(dT*dT)/(137438953472LL);
      OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
      SENSi = 0;
    }
  }
  
  OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
  SENS2 = SENS-SENSi;
  
  TEMP = (TEMP-Ti);
  
  if ( _model == 1 ) { //MS5837_02BA
    P = (((Ds1*SENS2)/2097152l-OFF2)/32768l); 
  } else {
    P = (((Ds1*SENS2)/2097152l-OFF2)/8192l);
  }
};
//================================

void toggle_led()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
};


void setup() 
{
  //Инициализация библиотек и интерфейсов
  myUserial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  SWire_CH1.begin();
  SWire_CH2.begin();

  delay(1000);
}

void loop()
{
char par = 0;
//В главном цикле поочередно производим инициализацию и опрос датчик давления
     if(myUserial.available()>0)
     {
     par = myUserial.read();

     if(par=='g')
     {
        inits(SWire_CH1);
        read_data(SWire_CH1);  
        sprintf(buf,"P1 %f T1 %f ",pressure(Pa),temperature()); 
        myUserial.write(buf);

        inits(SWire_CH2);
        read_data(SWire_CH2);  
        sprintf(buf,"P2 %f T2 %f \n",pressure(Pa),temperature()); 
        myUserial.write(buf);
                
        toggle_led();
     };
     
     }; 
        delay(100);
       
  }
     
