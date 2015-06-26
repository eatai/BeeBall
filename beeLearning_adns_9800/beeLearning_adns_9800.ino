// Eatai Roth 2015
// This script adapted from Gigi Butbaia http://www.instructables.com/id/Arduino-Tutorial-ADNS-9800-Laser-Mouse-Traveled-Di/

#include <SPI.h>
#include <avr/pgmspace.h>


// Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

//volatile byte sampRateBuff[4];
//volatile byte durationBuff[4];

volatile uint32_t sampRate;
volatile uint32_t dT;
volatile uint32_t duration;

volatile unsigned long lastTime;
volatile unsigned long startTime;

volatile int xdat[2];
volatile int ydat[2];

volatile float e1_dist = 0;
volatile float e2_dist = 0;

const int ncs0 = 8;
const int ncs1 = 9;
const float countsPerMM = 1800/25.4; // 1800 = cpi

extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

void setup() {
  Serial.begin(115200);
  
  pinMode (ncs0, OUTPUT);
  pinMode (ncs1, OUTPUT);

    
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(8);

  performStartup(ncs0); 
  adns_com_end(ncs0);
  performStartup(ncs1);
  adns_com_end(ncs1);
  delay(200);
  
  getTimeVar();
}

void loop() {
    // If trial is over, wait for new trial trigger
    if ((millis()-startTime) > duration){
      getTimeVar();
    }
    // If statement triggers reading if time since start is close to an integer mult of dT (max 2 ms, conservative) 
    // AND interval since last measurement is greater than dT - 2ms. These two conditions prevent sampling drift.
    if ((millis()-startTime)%dT <= 2 & (millis()-lastTime) >= (dT-2)){
      lastTime = millis();
      Serial.print(float(lastTime - startTime)/1000.0, DEC);  
      printDist(ncs0);
//      Serial.print(" ------ ");
      printDist(ncs1);
      Serial.println("");
    }
}

void printDist(int ss){  
    adns_read_motion(ss);
    e2_dist = -1*float((xdat[1]<<8) | xdat[0])/countsPerMM;
    e1_dist = -1*float((ydat[1]<<8) | ydat[0])/countsPerMM;
    Serial.print(",");
    Serial.print(e1_dist, 6);
    Serial.print(",");
    Serial.print(e2_dist, 6);
}

void getTimeVar(){
  byte sampRateBuff[4];
  byte durationBuff[4];
    // Wait for 4 bytes (integer) to be transmitted
  while (Serial.available()<4){
  }
  Serial.readBytes(sampRateBuff, 4);
  sampRate = bytes2int(sampRateBuff);
  dT = 1000/sampRate;
  
  while (Serial.available()<4){
  }
  Serial.readBytes(durationBuff, 4);
  duration = 1000*bytes2int(durationBuff);
  
  startTime = millis();
  lastTime = millis()-dT;
      
  Serial.print("Sample rate is set to: ");
  Serial.print(sampRate, DEC);
  Serial.println(" Hz");
  Serial.print("Duration is set to: ");
  Serial.print(duration/1000, DEC);
  Serial.println(" s");
}
  
void adns_com_begin(int ss){
  digitalWrite(ss, LOW);
}


void adns_com_end(int ss){
  digitalWrite(ss, HIGH);
}


byte adns_read_reg(int ss, byte reg_addr){
  adns_com_begin(ss);
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end(ss);
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}


void adns_write_reg(int ss, byte reg_addr, byte data){
  adns_com_begin(ss);
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end(ss);
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}


void adns_upload_firmware(int ss){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(ss, REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(ss, REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(ss, REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin(ss);
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end(ss);
}


void performStartup(int ss){
  adns_com_end(ss); // ensure that the spi is reset
//  adns_com_begin(ss); // ensure that the spi is reset
//  adns_com_end(ss); // ensure that the serial port is reset
  adns_write_reg(ss, REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_motion(ss);
  // upload the firmware
  adns_upload_firmware(ss);
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(ss, REG_LASER_CTRL0);
  adns_write_reg(ss, REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );  
  delay(1);

  Serial.println("Optical Chip Initialized");
}


void adns_read_motion(int ss){
    adns_read_reg(ss, REG_Motion);
    xdat[0] = (int)adns_read_reg(ss, REG_Delta_X_L);
    xdat[1] = (int)adns_read_reg(ss, REG_Delta_X_H);
    ydat[0] = (int)adns_read_reg(ss, REG_Delta_Y_L);
    ydat[1] = (int)adns_read_reg(ss, REG_Delta_Y_H);
}


void dispRegisters(int ss){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
  char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","Motion"  };
  byte regres;

  digitalWrite(ss,LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delay(1);
  }
  digitalWrite(ss,HIGH);
}


int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
}

long bytes2int(byte b[4]){
   long p = 0;
   p = ((long )b[0]) << 24;
   p |= ((long )b[1]) << 16;
   p |= ((long )b[2]) << 8;
   p |= b[3];
   return p;
}
  
  
