#define DEBUG_MODULE "amgdeck"

#include "debug.h"
#include "i2cdev.h"
#include "deck.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"

#include "amg8833.h"
static AMG8833_Dev_t kontol;
float temp;

void AMG8833_init(AMG8833_Dev_t* b, uint8_t address, I2C_Dev *i2cperipheral)
{
  b->I2Cx = i2cperipheral;
  b->devAddr = address;
}

// struct pctl _pctl;
// struct rst _rst;
// struct fpsc _fpsc;
// struct intc _intc;
// struct stat _stat;
// struct sclr _sclr;
// struct ave _ave;
// struct inthl _inthl;
// struct inthh _inthh;
// struct intll _intll;
// struct intlh _intlh;
// struct ihysl _ihysl;
// struct ihysh _ihysh;
// struct tthl _tthl;
// struct tthh _tthh;

// uint8_t getPCTL(void){ return _pctl.PCTL; }
// uint8_t getRST(void){	return _rst.RST; }
// uint8_t getFPSC(void){ return _fpsc.FPS & 0x01; }
// uint8_t getINTC(void){ return (_intc.INTMOD << 1 | _intc.INTEN) & 0x03; }
// uint8_t getSTAT(void){ return ( (_stat.OVF_THS << 3) | (_stat.OVF_IRS << 2) | (_stat.INTF << 1) ) & 0x07; }
// uint8_t getSCLR(void){ return ((_sclr.OVT_CLR << 3) | (_sclr.OVS_CLR << 2) | (_sclr.INTCLR << 1)) & 0x07; }
// uint8_t getAVE(void){ return (_ave.MAMOD << 5); }
// uint8_t getINTHL(void){ return _inthl.INT_LVL_H; }
// uint8_t getINTHH(void){ return _inthh.INT_LVL_H; }
// uint8_t getINTLL(void){ return _intll.INT_LVL_L; }
// uint8_t getINTLH(void){ return (_intlh.INT_LVL_L & 0xF); }
// uint8_t getIHYSL(void){ return _ihysl.INT_HYS; }
// uint8_t getIHYSH(void){ return (_ihysh.INT_HYS & 0xF); }
// uint8_t getTTHL(void){ return _tthl.TEMP; }
// uint8_t getTTHH(void){ return ( (_tthh.SIGN << 3) | _tthh.TEMP) & 0xF; }			
// // uint8_t min(uint8_t a, uint8_t b){ return a < b ? a : b; }
// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// // static bool isInit;
void amg8833Task(void* arg);
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint8_t new_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
// uint8_t rawArray[128];


// int SensorInit(void)
// {
// 	//enter normal mode
// 	_pctl.PCTL = AMG88xx_NORMAL_MODE;
// 	// write8(AMG88xx_PCTL, getPCTL());
//     i2cdevWrite16(I2C1_DEV, (uint8_t)AMG88xx_ADDRESS, AMG88xx_PCTL, 1, getPCTL());
	
// 	//software reset
// 	_rst.RST = AMG88xx_INITIAL_RESET;
// 	// write8(AMG88xx_RST, getRST());
//     i2cdevWrite16(I2C// i2cdevInit(I2C1_DEV);1_DEV, (uint8_t)AMG88xx_ADDRESS, AMG88xx_RST, getRST());
	
// 	//disable interrupts by default
// 	_intc.INTEN = 0;
//     i2cdevWrite16(I2C1_DEV, (uint8_t)AMG88xx_ADDRESS, AMG88xx_INTC, getINTC());
//     // write8(AMG88xx_INTC, getINTC());
//     // disableInterrupt();
	
// 	//set to 10 FPS
// 	_fpsc.FPS = AMG88xx_FPS_10;
// 	// write8(AMG88xx_FPSC, getFPSC());
//     i2cdevWrite16(I2C1_DEV, (uint8_t)AMG88xx_ADDRESS, AMG88xx_FPSC, getFPSC());
//     vTaskDelay(M2T(10));

//     DEBUG_PRINT("Sensor AMG88XX init...!\n");

// 	return 0;
// }

// void readPixel(float *buf, uint8_t size)
// {
// 	// uint16_t recast;
// 	// float converted;
//   // uint8_t bytesToRead = (uint8_t)(size << 1) < (uint8_t)(AMG88xx_PIXEL_ARRAY_SIZE << 1) ? (uint8_t)(size << 1) : (uint8_t)(AMG88xx_PIXEL_ARRAY_SIZE << 1);
// 	// uint8_t bytesToRead = min((uint8_t)(size << 1), (uint8_t)(AMG88xx_PIXEL_ARRAY_SIZE << 1));

// 	// uint8_t rawArray[bytesToRead];
// 	// read(AMG88xx_PIXEL_OFFSET, rawArray, bytesToRead);
// 	i2cdevReadReg16(I2C1_DEV, (uint8_t)AMG88xx_ADDRESS, AMG88xx_PIXEL_OFFSET, 128, rawArray);
//   DEBUG_PRINT("Reading pixel ...!\n");
// 	// for(int i=0; i<size; i++){
// 		// uint8_t pos = i << 1;
// 		// recast = ((uint16_t)rawArray[pos + 1] << 8) | ((uint16_t)rawArray[pos]);
// 		// 
// 		// converted = signedMag12ToFloat(recast) * AMG88xx_PIXEL_TEMP_CONVERSION;
// 		// buf[i] = converted;
// 	// }
// }

static void amg8833Init()
{
  // if (isInit)
  //   return;

  DEBUG_PRINT("Initializing AMG8833...\n");
  AMG8833_init(&kontol, AMG88xx_ADDRESS, I2C1_DEV);

  i2cdevInit(I2C1_DEV);
  begin(&kontol, I2C1_DEV);
  
  xTaskCreate(amg8833Task, "AMG8833", (8 * configMINIMAL_STACK_SIZE), NULL, 3, NULL);
  // isInit = true;

  DEBUG_PRINT("AMG8833 initialization complete!\n");
}

void amg8833Task(void* arg)
{
  systemWaitStart();

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  // SensorInit();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(120));
    // temp = readThermistor(&kontol);
   
    // Read two bytes from the sensor
    readPixels(&kontol, pixels, AMG88xx_PIXEL_ARRAY_SIZE);
    for (int i=0; i<64; i++){
      new_pixels[i] = (uint8_t) pixels[i];
    }
    // DEBUG_PRINT("Light reading is: %f\n", intensity);
  }
}

static bool amg8833Test()
{
  DEBUG_PRINT("AMG8833 test passed!\n");
  return true;
}


static const DeckDriver amg8833Driver = {
  .vid = 0,
  .pid = 0,
  .name = "amgdeck",
  .init = amg8833Init,
  .test = amg8833Test,

  .usedPeriph = DECK_USING_I2C,
};

DECK_DRIVER(amg8833Driver);
LOG_GROUP_START(amgdeck)
// LOG_ADD(LOG_FLOAT, temp, &temp)
// LOG_ADD(LOG_UINT8, y, &rawArray[2])
// LOG_ADD(LOG_UINT8, z, &rawArray[3])
// LOG_ADD(LOG_UINT8, a, &rawArray[4])
LOG_ADD(LOG_UINT8, new_pixels[0], &new_pixels[0])
LOG_ADD(LOG_UINT8, new_pixels[1], &new_pixels[1])
LOG_ADD(LOG_UINT8, new_pixels[2], &new_pixels[2])
LOG_ADD(LOG_UINT8, new_pixels[3], &new_pixels[3])
LOG_ADD(LOG_UINT8, new_pixels[4], &new_pixels[4])
LOG_ADD(LOG_UINT8, new_pixels[5], &new_pixels[5])
LOG_ADD(LOG_UINT8, new_pixels[6], &new_pixels[6])
LOG_ADD(LOG_UINT8, new_pixels[7], &new_pixels[7])
LOG_ADD(LOG_UINT8, new_pixels[8], &new_pixels[8])
LOG_ADD(LOG_UINT8, new_pixels[9], &new_pixels[9])
LOG_ADD(LOG_UINT8, new_pixels[10], &new_pixels[10])
LOG_ADD(LOG_UINT8, new_pixels[11], &new_pixels[11])
LOG_ADD(LOG_UINT8, new_pixels[12], &new_pixels[12])
LOG_ADD(LOG_UINT8, new_pixels[13], &new_pixels[13])
LOG_ADD(LOG_UINT8, new_pixels[14], &new_pixels[14])
LOG_ADD(LOG_UINT8, new_pixels[15], &new_pixels[15])
LOG_ADD(LOG_UINT8, new_pixels[16], &new_pixels[16])
LOG_ADD(LOG_UINT8, new_pixels[17], &new_pixels[17])
LOG_ADD(LOG_UINT8, new_pixels[18], &new_pixels[18])
LOG_ADD(LOG_UINT8, new_pixels[19], &new_pixels[19])
LOG_ADD(LOG_UINT8, new_pixels[20], &new_pixels[20])
LOG_ADD(LOG_UINT8, new_pixels[21], &new_pixels[21])
LOG_ADD(LOG_UINT8, new_pixels[22], &new_pixels[22])
LOG_ADD(LOG_UINT8, new_pixels[23], &new_pixels[23])
LOG_ADD(LOG_UINT8, new_pixels[24], &new_pixels[24])
LOG_ADD(LOG_UINT8, new_pixels[25], &new_pixels[25])
LOG_ADD(LOG_UINT8, new_pixels[26], &new_pixels[26])
LOG_ADD(LOG_UINT8, new_pixels[27], &new_pixels[27])
LOG_ADD(LOG_UINT8, new_pixels[28], &new_pixels[28])
LOG_ADD(LOG_UINT8, new_pixels[29], &new_pixels[29])
LOG_ADD(LOG_UINT8, new_pixels[30], &new_pixels[30])
LOG_ADD(LOG_UINT8, new_pixels[31], &new_pixels[31])
LOG_ADD(LOG_UINT8, new_pixels[32], &new_pixels[32])
LOG_ADD(LOG_UINT8, new_pixels[33], &new_pixels[33])
LOG_ADD(LOG_UINT8, new_pixels[34], &new_pixels[34])
LOG_ADD(LOG_UINT8, new_pixels[35], &new_pixels[35])
LOG_ADD(LOG_UINT8, new_pixels[36], &new_pixels[36])
LOG_ADD(LOG_UINT8, new_pixels[37], &new_pixels[37])
LOG_ADD(LOG_UINT8, new_pixels[38], &new_pixels[38])
LOG_ADD(LOG_UINT8, new_pixels[39], &new_pixels[39])
LOG_ADD(LOG_UINT8, new_pixels[40], &new_pixels[40])
LOG_ADD(LOG_UINT8, new_pixels[41], &new_pixels[41])
LOG_ADD(LOG_UINT8, new_pixels[42], &new_pixels[42])
LOG_ADD(LOG_UINT8, new_pixels[43], &new_pixels[43])
LOG_ADD(LOG_UINT8, new_pixels[44], &new_pixels[44])
LOG_ADD(LOG_UINT8, new_pixels[45], &new_pixels[45])
LOG_ADD(LOG_UINT8, new_pixels[46], &new_pixels[46])
LOG_ADD(LOG_UINT8, new_pixels[47], &new_pixels[47])
LOG_ADD(LOG_UINT8, new_pixels[48], &new_pixels[48])
LOG_ADD(LOG_UINT8, new_pixels[49], &new_pixels[49])
LOG_ADD(LOG_UINT8, new_pixels[50], &new_pixels[50])
LOG_ADD(LOG_UINT8, new_pixels[51], &new_pixels[51])
LOG_ADD(LOG_UINT8, new_pixels[52], &new_pixels[52])
LOG_ADD(LOG_UINT8, new_pixels[53], &new_pixels[53])
LOG_ADD(LOG_UINT8, new_pixels[54], &new_pixels[54])
LOG_ADD(LOG_UINT8, new_pixels[55], &new_pixels[55])
LOG_ADD(LOG_UINT8, new_pixels[56], &new_pixels[56])
LOG_ADD(LOG_UINT8, new_pixels[57], &new_pixels[57])
LOG_ADD(LOG_UINT8, new_pixels[58], &new_pixels[58])
LOG_ADD(LOG_UINT8, new_pixels[59], &new_pixels[59])
LOG_ADD(LOG_UINT8, new_pixels[60], &new_pixels[60])
LOG_ADD(LOG_UINT8, new_pixels[61], &new_pixels[61])
LOG_ADD(LOG_UINT8, new_pixels[62], &new_pixels[62])
LOG_ADD(LOG_UINT8, new_pixels[63], &new_pixels[63])
// LOG_ADD(LOG_FLOAT, pixels[0], &pixels[0])
// LOG_ADD(LOG_FLOAT, pixels[1], &pixels[1])
// LOG_ADD(LOG_FLOAT, pixels[2], &pixels[2])
// LOG_ADD(LOG_FLOAT, pixels[3], &pixels[3])
// LOG_ADD(LOG_FLOAT, pixels[4], &pixels[4])
// LOG_ADD(LOG_FLOAT, pixels[5], &pixels[5])
// LOG_ADD(LOG_FLOAT, pixels[6], &pixels[6])
// LOG_ADD(LOG_FLOAT, pixels[7], &pixels[7])
// LOG_ADD(LOG_FLOAT, pixels[8], &pixels[8])
// LOG_ADD(LOG_FLOAT, pixels[9], &pixels[9])
// LOG_ADD(LOG_FLOAT, pixels[10], &pixels[10])
// LOG_ADD(LOG_FLOAT, pixels[11], &pixels[11])
// LOG_ADD(LOG_FLOAT, pixels[12], &pixels[12])
// LOG_ADD(LOG_FLOAT, pixels[13], &pixels[13])
// LOG_ADD(LOG_FLOAT, pixels[14], &pixels[14])
// LOG_ADD(LOG_FLOAT, pixels[15], &pixels[15])
// LOG_ADD(LOG_FLOAT, pixels[16], &pixels[16])
// LOG_ADD(LOG_FLOAT, pixels[17], &pixels[17])
// LOG_ADD(LOG_FLOAT, pixels[18], &pixels[18])
// LOG_ADD(LOG_FLOAT, pixels[19], &pixels[19])
// LOG_ADD(LOG_FLOAT, pixels[20], &pixels[20])
// LOG_ADD(LOG_FLOAT, pixels[21], &pixels[21])
// LOG_ADD(LOG_FLOAT, pixels[22], &pixels[22])
// LOG_ADD(LOG_FLOAT, pixels[23], &pixels[23])
// LOG_ADD(LOG_FLOAT, pixels[24], &pixels[24])
// LOG_ADD(LOG_FLOAT, pixels[25], &pixels[25])
// LOG_ADD(LOG_FLOAT, pixels[26], &pixels[26])
// LOG_ADD(LOG_FLOAT, pixels[27], &pixels[27])
// LOG_ADD(LOG_FLOAT, pixels[28], &pixels[28])
// LOG_ADD(LOG_FLOAT, pixels[29], &pixels[29])
// LOG_ADD(LOG_FLOAT, pixels[30], &pixels[30])
// LOG_ADD(LOG_FLOAT, pixels[31], &pixels[31])
// LOG_ADD(LOG_FLOAT, pixels[32], &pixels[32])
// LOG_ADD(LOG_FLOAT, pixels[33], &pixels[33])
// LOG_ADD(LOG_FLOAT, pixels[34], &pixels[34])
// LOG_ADD(LOG_FLOAT, pixels[35], &pixels[35])
// LOG_ADD(LOG_FLOAT, pixels[36], &pixels[36])
// LOG_ADD(LOG_FLOAT, pixels[37], &pixels[37])
// LOG_ADD(LOG_FLOAT, pixels[38], &pixels[38])
// LOG_ADD(LOG_FLOAT, pixels[39], &pixels[39])
// LOG_ADD(LOG_FLOAT, pixels[40], &pixels[40])
// LOG_ADD(LOG_FLOAT, pixels[41], &pixels[41])
// LOG_ADD(LOG_FLOAT, pixels[42], &pixels[42])
// LOG_ADD(LOG_FLOAT, pixels[43], &pixels[43])
// LOG_ADD(LOG_FLOAT, pixels[44], &pixels[44])
// LOG_ADD(LOG_FLOAT, pixels[45], &pixels[45])
// LOG_ADD(LOG_FLOAT, pixels[46], &pixels[46])
// LOG_ADD(LOG_FLOAT, pixels[47], &pixels[47])
// LOG_ADD(LOG_FLOAT, pixels[48], &pixels[48])
// LOG_ADD(LOG_FLOAT, pixels[49], &pixels[49])
// LOG_ADD(LOG_FLOAT, pixels[50], &pixels[50])
// LOG_ADD(LOG_FLOAT, pixels[51], &pixels[51])
// LOG_ADD(LOG_FLOAT, pixels[52], &pixels[52])
// LOG_ADD(LOG_FLOAT, pixels[53], &pixels[53])
// LOG_ADD(LOG_FLOAT, pixels[54], &pixels[54])
// LOG_ADD(LOG_FLOAT, pixels[55], &pixels[55])
// LOG_ADD(LOG_FLOAT, pixels[56], &pixels[56])
// LOG_ADD(LOG_FLOAT, pixels[57], &pixels[57])
// LOG_ADD(LOG_FLOAT, pixels[58], &pixels[58])
// LOG_ADD(LOG_FLOAT, pixels[59], &pixels[59])
// LOG_ADD(LOG_FLOAT, pixels[60], &pixels[60])
// LOG_ADD(LOG_FLOAT, pixels[61], &pixels[61])
// LOG_ADD(LOG_FLOAT, pixels[62], &pixels[62])
// LOG_ADD(LOG_FLOAT, pixels[63], &pixels[63])
LOG_GROUP_STOP(amgdeck)