#include <USBComposite.h>
#define DEFAULT_SERIAL_STRING "00000000001A"
#define PRODUCT "STM32 DAQ BOARD"
#define MANUF "STMicroelectronics"
#define TXSIZE 64
#define RXSIZE 64
#define VID 0x0483
#define PID 0x5750

#define BOARD_LED PC13

#include <SoftWire.h>
SoftWire SWire_CH1(PB8, PB9, SOFT_FAST);


const uint8_t reportDescription[] = {
      0x06, 0x00, 0xff,              //   USAGE_PAGE (Generic Desktop)
      0x09, 0x01,                    //   USAGE (Vendor Usage 1)
      // System Parameters
    //Прием в мк
      0xa1, 0x01,                    //   COLLECTION (Application)
      0x85, 0x01,                    //   REPORT_ID (2)
      0x75, 0x08,                    //   REPORT_SIZE (8)
      0x95, 64,                        //   REPORT_COUNT (4)
      0x09, 0x01,                    //   USAGE (Vendor Usage 1)
      0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)
      0xC0    /*     END_COLLECTION              */
};

USBHIDDevice STM_HID;
uint8 buf_in[RXSIZE];
uint8 buf_out[TXSIZE];
char buf_[TXSIZE];
HIDReporter STM_HID_REP(buf_out,TXSIZE);

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

void setup(){
 // (reportDescription, sizeof(reportDescription));  
  STM_HID.begin(reportDescription, sizeof(reportDescription),PID,VID,MANUF ,PRODUCT ,DEFAULT_SERIAL_STRING);
  
  SWire_CH1.begin();
  inits(SWire_CH1);
  pinMode(BOARD_LED, OUTPUT);
   digitalWrite(BOARD_LED, HIGH);
   delay(1000);digitalWrite(BOARD_LED, LOW);
   delay(1000);digitalWrite(BOARD_LED, HIGH);

}


void loop() {
  buf_out[0]=1; 
  
  read_data(SWire_CH1);  
  int p1 = pressure(Pa);
  int t1 = temperature();
    
  int p2 = 0;
  int t2 = 0;
  sprintf(buf_," P1 %d T1 %d P2 %d T2 %d ",p1,t1,p2,t2); 
  for(int i(1);i<TXSIZE;i++)buf_out[i]=buf_[i];             
        toggle_led();
  STM_HID_REP.sendReport();

}

