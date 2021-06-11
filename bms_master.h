
#ifndef bms_master_H
#define bms_master_H

#include "mbed.h"
#include <cstdio>
#include <stdint.h>

#define IC_LTC6811

#define MD_422HZ_1KHZ 0
#define MD_27KHZ_14KHZ 1
#define MD_7KHZ_3KHZ 2
#define MD_26HZ_2KHZ 3

#define ADC_OPT_ENABLED 1
#define ADC_OPT_DISABLED 0

#define CELL_CH_ALL 0 //12
#define CELL_CH_1and7 1
#define CELL_CH_2and8 2
#define CELL_CH_3and9 3
#define CELL_CH_4and10 4
#define CELL_CH_5and11 5
#define CELL_CH_6and12 6

#define CELL_CHANNELS 12

#define SELFTEST_1 1
#define SELFTEST_2 2

#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6

#define STAT_CH_ALL 0
#define STAT_CH_SOC 1
#define STAT_CH_ITEMP 2
#define STAT_CH_VREGA 3
#define STAT_CH_VREGD 4

#define DCP_DISABLED 0
#define DCP_ENABLED 1

#define PULL_UP_CURRENT 1
#define PULL_DOWN_CURRENT 0

#define REG_ALL 0
#define REG_1 1
#define REG_2 2
#define REG_3 3
#define REG_4 4
#define REG_5 5
#define REG_6 6

#define NUM_RX_BYT 8
#define CELL 1
#define AUX 2
#define STAT 3
#define CFGR 0
#define CFGRB 4
DigitalOut led(LED1);

DigitalOut CS_PIN(PA_15); //chip select

SPI spi(PB_5,PB_4,PB_3); //mosi,miso,sck
//Serial pc(USBTX,USBRX);

#define UI_BUFFER_SIZE 64

//! Cell Voltage data structure.
typedef struct
{
  uint16_t c_codes[18];//!< Cell Voltage Codes
  uint8_t pec_match[6];//!< If a PEC error was detected during most recent read cmd
} cv;

//! AUX Reg Voltage Data
typedef struct
{
  uint16_t a_codes[9];//!< Aux Voltage Codes
  uint8_t pec_match[4];//!< If a PEC error was detected during most recent read cmd
} ax;

typedef struct
{
  uint16_t stat_codes[4];//!< A two dimensional array of the stat voltage codes.
  uint8_t flags[3]; //!< byte array that contains the uv/ov flag data
  uint8_t mux_fail[1]; //!< Mux self test status flag
  uint8_t thsd[1]; //!< Thermal shutdown status
  uint8_t pec_match[2];//!< If a PEC error was detected during most recent read cmd
} st;

typedef struct
{
  uint8_t tx_data[6];
  uint8_t rx_data[8];
  uint8_t rx_pec_match;//!< If a PEC error was detected during most recent read cmd
} ic_register;

typedef struct
{
  uint16_t pec_count;
  uint16_t cfgr_pec;
  uint16_t cell_pec[6];
  uint16_t aux_pec[4];
  uint16_t stat_pec[2];
} pec_counter;

typedef struct
{
  uint8_t cell_channels;
  uint8_t stat_channels;
  uint8_t aux_channels;
  uint8_t num_cv_reg;
  uint8_t num_gpio_reg;
  uint8_t num_stat_reg;
} register_cfg;

typedef struct
{
  ic_register config;
  ic_register configb;
  cv   cells;
  ax   aux;
  st   stat;
  ic_register  com;
  ic_register pwm;
  ic_register pwmb;
  ic_register sctrl;
  ic_register sctrlb;
  bool isospi_reverse;
  pec_counter crc_count;
  register_cfg ic_reg;
  long system_open_wire;
} cell_asic;




/*!   calculates  and returns the CRC15
  @returns The calculated pec15 as an unsigned int
*/
uint16_t pec15_calc(uint8_t len, //!< the length of the data array being passed to the function
                    uint8_t *data //!<  the array of data that the PEC will be generated from
                   );

/*!  Wake isoSPI up from idle state */
void wakeup_idle(uint8_t total_ic);//!< number of ICs in the daisy chain

/*!  Wake the LTC6813 from the sleep state */
void wakeup_sleep(uint8_t total_ic); //!< number of ICs in the daisy chain

/*! Sense a command to the bms IC. This code will calculate the PEC code for the transmitted command*/
void cmd_68(uint8_t tx_cmd[2]); //!< 2 Byte array containing the BMS command to be sent

//! Writes an array of data to the daisy chain
void write_68(uint8_t total_ic , //!< number of ICs in the daisy chain
              uint8_t tx_cmd[2], //!< 2 Byte array containing the BMS command to be sent
              uint8_t data[] //!< Array containing the data to be written to the BMS ICs
             );
//! Issues a command onto the daisy chain and reads back 6*total_ic data in the rx_data array
int8_t read_68( uint8_t total_ic, //!< number of ICs in the daisy chain
                uint8_t tx_cmd[2], //!< 2 Byte array containing the BMS command to be sent
                uint8_t *rx_data); //!< Array that the read back data will be stored.

/*! Starts the Mux Decoder diagnostic self test

 Running this command will start the Mux Decoder Diagnostic Self Test
 This test takes roughly 1mS to complete. The MUXFAIL bit will be updated,
 the bit will be set to 1 for a failure and 0 if the test has been passed.
 */
void LTC681x_diagn();

//! Sends the poll adc command
//! @returns 1 byte read back after a pladc command. If the byte is not 0xFF ADC conversion has completed
uint8_t LTC681x_pladc();

//! This function will block operation until the ADC has finished it's conversion
//! @returns the approximate time it took for the ADC function to complete.
uint32_t LTC681x_pollAdc();

/*! Starts cell voltage conversion

  Starts ADC conversions of the LTC6811 Cpin inputs.
  The type of ADC conversion executed can be changed by setting the following parameters:
*/
void LTC681x_adcv(uint8_t MD, //!< ADC Conversion Mode
                  uint8_t DCP, //!< Controls if Discharge is permitted during conversion
                  uint8_t CH //!< Sets which Cell channels are converted
                 );

/*!  Starts cell voltage  and GPIO 1&2 conversion
*/
void LTC681x_adcvax(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t DCP //!< Controls if Discharge is permitted during conversion
);


/*!  Starts cell voltage self test conversion
*/
void LTC681x_cvst(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t ST //!< Self Test Mode
);

/*!  Starts cell voltage and SOC conversion
*/
void LTC681x_adcvsc(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t DCP //!< Controls if Discharge is permitted during conversion
);
/*!  Starts cell voltage overlap conversion
*/
void LTC681x_adol(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t DCP //!< Discharge permitted during conversion
);

/*!  Start an open wire Conversion
*/
void LTC681x_adow(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t PUP //!< Controls if Discharge is permitted during conversion
);


/*!  Start a GPIO and Vref2 Conversion
*/
void LTC681x_adax(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t CHG //!< Sets which GPIO channels are converted
);

/*!  Start an GPIO Redundancy test
*/
void LTC681x_adaxd(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t CHG //!< Sets which GPIO channels are converted
);

/*!  Start an Auxiliary Register Self Test Conversion
*/
void LTC681x_axst(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t ST //!< Sets if self test 1 or 2 is run
);



/*!  Start a Status ADC Conversion
*/
void LTC681x_adstat(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t CHST //!< Sets which Stat channels are converted
);

/*!   Start a Status register redundancy test Conversion
*/
void LTC681x_adstatd(
  uint8_t MD, //!< ADC Mode
  uint8_t CHST //!< Sets which Status channels are converted
);


/*!  Start a Status Register Self Test Conversion
*/
void LTC681x_statst(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t ST //!< Sets if self test 1 or 2 is run
);

void LTC681x_rdcv_reg(uint8_t reg, //!<Determines which cell voltage register is read back
                      uint8_t total_ic, //!<the number of ICs in the
                      uint8_t *data //!<An array of the unparsed cell codes
                     );
/*! helper function that parses voltage measurement registers
*/
int8_t parse_cells(uint8_t current_ic,
                   uint8_t cell_reg,
                   uint8_t cell_data[],
                   uint16_t *cell_codes,
                   uint8_t *ic_pec);

/*!  Read the raw data from the LTC681x auxiliary register

 The function reads a single GPIO voltage register and stores thre read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC681x_rdaux() command.
 */
void LTC681x_rdaux_reg(  uint8_t reg, //Determines which GPIO voltage register is read back
                         uint8_t total_ic, //The number of ICs in the system
                         uint8_t *data //Array of the unparsed auxiliary codes
                      );
/*!  Read the raw data from the LTC681x stat register

 The function reads a single GPIO voltage register and stores thre read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC681x_rdstat() command.
 */
void LTC681x_rdstat_reg(uint8_t reg, //Determines which stat register is read back
                        uint8_t total_ic, //The number of ICs in the system
                        uint8_t *data //Array of the unparsed stat codes
                       );

/*!  Clears the LTC681x cell voltage registers

The command clears the cell voltage registers and initializes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.
*/
void LTC681x_clrcell();
/*! Clears the LTC681x Auxiliary registers

The command clears the Auxiliary registers and initializes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.
*/
void LTC681x_clraux();

/*!  Clears the LTC681x Stat registers

The command clears the Stat registers and initializes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.
*/
void LTC681x_clrstat();

/*!  Clears the LTC681x SCTRL registers

The command clears the SCTRL registers and initializes
all values to 0. The register will read back hexadecimal 0x00
after the command is sent.
*/
void LTC681x_clrsctrl();

/*! Starts the Mux Decoder diagnostic self test

Running this command will start the Mux Decoder Diagnostic Self Test
This test takes roughly 1mS to complete. The MUXFAIL bit will be updated,
the bit will be set to 1 for a failure and 0 if the test has been passed.
*/
void LTC681x_diagn();

/*!  Reads and parses the LTC681x cell voltage registers.

 The function is used to read the cell codes of the LTC6811.
 This function will send the requested read commands parse the data
 and store the cell voltages in the cell_asic structure.
 */
uint8_t LTC681x_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // the number of ICs in the system
                     cell_asic ic[] // Array of the parsed cell codes
                    );

/*!  Reads and parses the LTC681x auxiliary registers.

 The function is used to read the  parsed GPIO codes of the LTC6811. This function will send the requested
 read commands parse the data and store the gpio voltages in the cell_asic structure.
*/
int8_t LTC681x_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
                     uint8_t total_ic,//the number of ICs in the system
                     cell_asic ic[]//!< Measurement Data Structure
                    );

/*!  Reads and parses the LTC681x stat registers.

 The function is used to read the  parsed status codes of the LTC6811. This function will send the requested
 read commands parse the data and store the status voltages in the cell_asic structure
 */
int8_t LTC681x_rdstat(  uint8_t reg, //!<Determines which Stat  register is read back.
                        uint8_t total_ic,//!<the number of ICs in the system
                        cell_asic ic[]//!< Measurement Data Structure
                     );
/*!  Write the LTC681x CFGRA

 This command will write the configuration registers of the LTC681xs
 connected in a daisy chain stack. The configuration is written in descending
 order so the last device's configuration is written first.
 */
void LTC681x_wrcfg(uint8_t total_ic, //The number of ICs being written to
                   cell_asic ic[] //A two dimensional array of the configuration data that will be written
                  );

/*!  Reads the LTC681x CFGRA register
*/
int8_t LTC681x_rdcfg(uint8_t total_ic, //Number of ICs in the system
                     cell_asic ic[] //A two dimensional array that the function stores the read configuration data.
                    );

/*! Selft Test Helper Function*/
uint16_t LTC681x_st_lookup(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
);

/*! Helper Function to clear DCC bits in the CFGR Registers*/
void clear_discharge(uint8_t total_ic,
                     cell_asic ic[]);

/*! Helper function that runs the ADC Self Tests*/
int16_t LTC681x_run_cell_adc_st(uint8_t adc_reg,
                                uint8_t total_ic,
                                cell_asic ic[]);

/*! Helper function that runs the ADC Digital Redudancy commands and checks output for errors*/
int16_t LTC681x_run_adc_redundancy_st(uint8_t adc_mode,
                                      uint8_t adc_reg,
                                      uint8_t total_ic,
                                      cell_asic ic[]);

/*! Helper function that runs the datasheet open wire algorithm*/
void LTC681x_run_openwire(uint8_t total_ic,
                          cell_asic ic[]);

/*! Helper Function that runs the ADC Overlap test*/
uint16_t LTC681x_run_adc_overlap(uint8_t total_ic,
                                 cell_asic ic[]);
/*! Helper Function that counts overall PEC errors and register/IC PEC errors*/
void LTC681x_check_pec(uint8_t total_ic,
                       uint8_t reg,
                       cell_asic ic[]);

/*! Helper Function that resets the PEC error counters */
void LTC681x_reset_crc_count(uint8_t total_ic,
                             cell_asic ic[]);

/*! Helper Function to initialize the CFGR data structures*/
void LTC681x_init_cfg(uint8_t total_ic,
                      cell_asic ic[]);

/*! Helper function to set appropriate bits in CFGR register based on bit function*/
void LTC681x_set_cfgr(uint8_t nIC,
                      cell_asic ic[],
                      bool refon,
                      bool adcopt,
                      bool gpio[5],
                      bool dcc[12]);

/*! Helper function to turn the refon bit HIGH or LOW*/
void LTC681x_set_cfgr_refon(uint8_t nIC,
                            cell_asic ic[],
                            bool refon);

/*! Helper function to turn the ADCOPT bit HIGH or LOW*/
void LTC681x_set_cfgr_adcopt(uint8_t nIC,
                             cell_asic ic[],
                             bool adcopt);

/*! Helper function to turn the GPIO bits HIGH or LOW*/
void LTC681x_set_cfgr_gpio(uint8_t nIC,
                           cell_asic ic[],
                           bool gpio[]);

/*! Helper function to turn the DCC bits HIGH or LOW*/
void LTC681x_set_cfgr_dis(uint8_t nIC,
                          cell_asic ic[],
                          bool dcc[]);



//This needs a PROGMEM =  when using with a LINDUINO
const uint16_t crc15Table[256]  {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
                                0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
                                0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
                                0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
                                0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
                                0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
                                0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
                                0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
                                0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
                                0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
                                0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
                                0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
                                0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
                                0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
                                0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
                                0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
                                0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
                                0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
                                0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
                                0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
                                0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
                                0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
                                0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
                               };
                               
//********************************** LTC6811 ******************************************//                               
void LTC6811_init_reg_limits(uint8_t total_ic, cell_asic ic[]);

void LTC6811_set_discharge(int Cell,
                           uint8_t total_ic,
                           cell_asic ic[]);

//********************************** SPI ******************************************//
/*
Writes an array of bytes out of the SPI port
*/
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    );
/*
 Writes and read a set number of bytes using the SPI port.

*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   );

uint8_t spi_read_byte(uint8_t tx_dat);//name conflicts with linduino also needs to take a byte as a parameter

//********************************** UserInterface ******************************************//
// io buffer
extern char ui_buffer[UI_BUFFER_SIZE];

// Read data from the serial interface into the ui_buffer buffer
uint8_t read_data();

// Read an integer from the serial interface.
// The routine can recognize Hex, Decimal, Octal, or Binary
// Example:
// Hex:     0x11 (0x prefix)
// Decimal: 17
// Octal:   O21 (leading letter O prefix)
// Binary:  B10001 (leading letter B prefix)
int32_t read_int();

// Read a character from the serial interface
int8_t read_char();

//********************************** UserInterface ******************************************//
#endif


//////////////////////////

void wakeup_idle(uint8_t total_ic)
{ //dökümana bakılarak daisy chain için optimize edilecek
  for (int i =0; i<total_ic; i++)
  {
    CS_PIN = 0;
    //delayMicroseconds(2); 
    //Guarantees the isoSPI will be in ready mode
    wait_us(2);
    spi_read_byte(0xff);
    
    CS_PIN = 1;
    //wait_us(10);
    printf("uyandı IDLE\n");
  }
}

//Generic wakeup commannd to wake the LTC6811 from sleep
void wakeup_sleep(uint8_t total_ic)
{
  for (int i =0; i<total_ic; i++) //total_ic yerine 220 yazılmış
  {
    CS_PIN = 0;
    wait_us(300e3); // Guarantees the LTC6811 will be in standby
    //spi_read_byte(0xff); //ben ekledim eksik gibi duruyor (değilmiş)
    spi.write(0x00);
    CS_PIN = 1;
    wait_us(10e3);
    printf("UYANDI SLEEP\n");
    
  }
}


//Generic function to write 68xx commands. Function calculated PEC for tx_cmd data
void cmd_68(uint8_t tx_cmd[2])
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  cmd[0] = tx_cmd[0];
  cmd[1] =  tx_cmd[1];
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  CS_PIN = 0;
  spi_write_array(4,cmd);
  printf("Written config");
  printf("%d   %d   %d   %d \n",cmd[0],cmd[1],cmd[2],cmd[3]);
  CS_PIN = 1;
}

//Generic function to write 68xx commands and write payload data. Function calculated PEC for tx_cmd data
void write_68(uint8_t total_ic , uint8_t tx_cmd[2], uint8_t data[])
{
  const uint8_t BYTES_IN_REG = 6;
  const uint8_t CMD_LEN = 4+(8*total_ic);
  uint8_t *cmd;
  uint16_t data_pec;
  uint16_t cmd_pec;
  uint8_t cmd_index;

  cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));
  cmd[0] = tx_cmd[0];
  cmd[1] = tx_cmd[1];
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  cmd_index = 4;
  for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each LTC681x in daisy chain, this loops starts with
  {
    // the last IC on the stack. The first configuration written is
    // received by the last IC in the daisy chain

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      cmd[cmd_index] = data[((current_ic-1)*6)+current_byte];
      cmd_index = cmd_index + 1;
    }

    data_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &data[(current_ic-1)*6]);    // calculating the PEC for each Iss configuration register data
    cmd[cmd_index] = (uint8_t)(data_pec >> 8);
    cmd[cmd_index + 1] = (uint8_t)data_pec;
    cmd_index = cmd_index + 2;
  }


  CS_PIN = 0;
  spi_write_array(CMD_LEN, cmd);
  CS_PIN = 1;
  free(cmd);
}

//Generic function to write 68xx commands and read data. Function calculated PEC for tx_cmd data
int8_t read_68( uint8_t total_ic, uint8_t tx_cmd[2], uint8_t *rx_data)
{
  const uint8_t BYTES_IN_REG = 8;
  uint8_t cmd[4];
  uint8_t data[256];
  int8_t pec_error = 0;
  uint16_t cmd_pec;
  uint16_t data_pec;
  uint16_t received_pec;

  // data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t)); // This is a problem because it can fail

  cmd[0] = tx_cmd[0];
  cmd[1] = tx_cmd[1];
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);


  CS_PIN = 0;
  spi_write_read(cmd, 4, data, (BYTES_IN_REG*total_ic));         //Read the configuration data of all ICs on the daisy chain into
  CS_PIN = 1;                          //rx_data[] array

  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)       //executes for each LTC681x in the daisy chain and packs the data
  {
    //into the r_comm array as well as check the received Config data
    //for any bit errors
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      rx_data[(current_ic*8)+current_byte] = data[current_byte + (current_ic*BYTES_IN_REG)];
    }
    received_pec = (rx_data[(current_ic*8)+6]<<8) + rx_data[(current_ic*8)+7];
    data_pec = pec15_calc(6, &rx_data[current_ic*8]);
    if (received_pec != data_pec)
    {
      pec_error = -1;
    }
  }


  return(pec_error);
}

// Calculates  and returns the CRC15

uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
                   )
{
  uint16_t remainder,addr;

  remainder = 16;//initialize the PEC
  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  {
    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
    
    remainder = (remainder<<8)^crc15Table[addr];
  }
  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

//Starts cell voltage conversion
void LTC681x_adcv(
  uint8_t MD, //ADC Mode
  uint8_t DCP, //Discharge Permit
  uint8_t CH //Cell Channels to be measured
)
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + 0x60 + (DCP<<4) + CH;
 
  cmd_68(cmd);
  
}

//Starts cell voltage overlap conversion
void LTC681x_adol(
  uint8_t MD, //ADC Mode
  uint8_t DCP //Discharge Permit
)
{
  uint8_t cmd[4];
  uint8_t md_bits;
  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + (DCP<<4) +0x01;
  cmd_68(cmd);
}

//Starts cell voltage self test conversion
void LTC681x_cvst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t cmd[2];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + ((ST)<<5) +0x07;
  cmd_68(cmd);

}

//Start an Auxiliary Register Self Test Conversion
void LTC681x_axst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + ((ST&0x03)<<5) +0x07;
  cmd_68(cmd);

}

//Start a Status Register Self Test Conversion
void LTC681x_statst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t cmd[2];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + ((ST&0x03)<<5) +0x0F;
  cmd_68(cmd);

}
uint8_t LTC681x_pladc()
{
    uint8_t cmd[4];
	uint8_t adc_state = 0xFF;
	uint16_t cmd_pec;
	
	cmd[0] = 0x07;
	cmd[1] = 0x14;
	cmd_pec = pec15_calc(2, cmd);
	cmd[2] = (uint8_t)(cmd_pec >> 8);
	cmd[3] = (uint8_t)(cmd_pec);

    CS_PIN = 0;
    spi_write_array(4, cmd);
    adc_state = spi_read_byte(0xFF);
    CS_PIN = 1;
    return adc_state;
}
//This function will block operation until the ADC has finished it's conversion
uint32_t LTC681x_pollAdc()
{
  uint32_t counter = 0;
  uint8_t finished = 0;
  uint8_t current_time = 0;
  uint8_t cmd[4];
  uint16_t cmd_pec;


  cmd[0] = 0x07;
  cmd[1] = 0x14;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  CS_PIN = 0;
  spi_write_array(4,cmd);

  while ((counter<200000)&&(finished == 0))
  {
    current_time = spi_read_byte(0xff);
    if (current_time>0)
    {
      finished = 1;
    }
    else
    {
      counter = counter + 10;
    }
  }

  CS_PIN = 1;


  return(counter);
}

//Start a GPIO and Vref2 Conversion
void LTC681x_adax(
  uint8_t MD, //ADC Mode
  uint8_t CHG //GPIO Channels to be measured)
)
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + 0x60 + CHG ;
  cmd_68(cmd);

}

//Start an GPIO Redundancy test
void LTC681x_adaxd(
  uint8_t MD, //ADC Mode
  uint8_t CHG //GPIO Channels to be measured)
)
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + CHG ;
  cmd_68(cmd);
}

//Start a Status ADC Conversion
void LTC681x_adstat(
  uint8_t MD, //ADC Mode
  uint8_t CHST //GPIO Channels to be measured
)
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + 0x68 + CHST ;
  cmd_68(cmd);
}

// Start a Status register redundancy test Conversion
void LTC681x_adstatd(
  uint8_t MD, //ADC Mode
  uint8_t CHST //GPIO Channels to be measured
)
{
  uint8_t cmd[2];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + 0x08 + CHST ;
  cmd_68(cmd);

}


// Start an open wire Conversion
void LTC681x_adow(
  uint8_t MD, //ADC Mode
  uint8_t PUP //Discharge Permit
)
{
  uint8_t cmd[2];
  uint8_t md_bits;
  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + 0x28 + (PUP<<6) ;//+ CH;
  cmd_68(cmd);
}

// Reads the raw cell voltage register data
void LTC681x_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
                      uint8_t total_ic, //the number of ICs in the
                      uint8_t *data //An array of the unparsed cell codes
                     )
{
  const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;

  if (reg == 1)     //1: RDCVA
  {
    cmd[1] = 0x04;
    cmd[0] = 0x00;
  }
  else if (reg == 2) //2: RDCVB
  {
    cmd[1] = 0x06;
    cmd[0] = 0x00;
  }
  else if (reg == 3) //3: RDCVC
  {
    cmd[1] = 0x08;
    cmd[0] = 0x00;
  }
  else if (reg == 4) //4: RDCVD
  {
    cmd[1] = 0x0A;
    cmd[0] = 0x00;
  }
  else if (reg == 5) //4: RDCVE
  {
    cmd[1] = 0x09;
    cmd[0] = 0x00;
  }
  else if (reg == 6) //4: RDCVF
  {
    cmd[1] = 0x0B;
    cmd[0] = 0x00;
  }


  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  CS_PIN = 0;
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  CS_PIN = 1;

}

/*
The function reads a single GPIO voltage register and stores thre read data
in the *data point as a byte array. This function is rarely used outside of
the LTC6811_rdaux() command.
*/
void LTC681x_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
                       uint8_t total_ic, //The number of ICs in the system
                       uint8_t *data //Array of the unparsed auxiliary codes
                      )
{
  const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;


  if (reg == 1)     //Read back auxiliary group A
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }
  else if (reg == 2)  //Read back auxiliary group B
  {
    cmd[1] = 0x0e;
    cmd[0] = 0x00;
  }
  else if (reg == 3)  //Read back auxiliary group C
  {
    cmd[1] = 0x0D;
    cmd[0] = 0x00;
  }
  else if (reg == 4)  //Read back auxiliary group D
  {
    cmd[1] = 0x0F;
    cmd[0] = 0x00;
  }
  else          //Read back auxiliary group A
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  CS_PIN = 0;
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  CS_PIN = 1;

}

/*
The function reads a single stat  register and stores the read data
in the *data point as a byte array. This function is rarely used outside of
the LTC6811_rdstat() command.
*/
void LTC681x_rdstat_reg(uint8_t reg, //Determines which stat register is read back
                        uint8_t total_ic, //The number of ICs in the system
                        uint8_t *data //Array of the unparsed stat codes
                       )
{
  const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;


  if (reg == 1)     //Read back statiliary group A
  {
    cmd[1] = 0x10;
    cmd[0] = 0x00;
  }
  else if (reg == 2)  //Read back statiliary group B
  {
    cmd[1] = 0x12;
    cmd[0] = 0x00;
  }

  else          //Read back statiliary group A
  {
    cmd[1] = 0x10;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  CS_PIN = 0;
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  CS_PIN = 1;

}

/*
The command clears the cell voltage registers and intiallizes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.
*/
void LTC681x_clrcell()
{
  uint8_t cmd[2]= {0x07 , 0x11};
  cmd_68(cmd);
}

/*
The command clears the Auxiliary registers and initializes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.
*/
void LTC681x_clraux()
{
  uint8_t cmd[2]= {0x07 , 0x12};
  cmd_68(cmd);
}


/*
The command clears the Stat registers and intiallizes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.

*/
void LTC681x_clrstat()
{
  uint8_t cmd[2]= {0x07 , 0x13};
  cmd_68(cmd);
}

//Starts the Mux Decoder diagnostic self test
void LTC681x_diagn()
{
  uint8_t cmd[2] = {0x07 , 0x15};
  cmd_68(cmd);
}

//Reads and parses the LTC681x cell voltage registers.
uint8_t LTC681x_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // the number of ICs in the system
                     cell_asic *ic // Array of the parsed cell codes
                    )
{

  int8_t pec_error = 0;
  uint8_t *cell_data;
  uint8_t c_ic = 0;
  cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));
 
  if (reg == 0)
  {
    for (uint8_t cell_reg = 1; cell_reg<ic[0].ic_reg.num_cv_reg+1; cell_reg++)                   //executes once for each of the LTC6811 cell voltage registers
    {
      LTC681x_rdcv_reg(cell_reg, total_ic,cell_data );
      for (int current_ic = 0; current_ic<total_ic; current_ic++)
      {
        if (ic->isospi_reverse == false)
        {
          c_ic = current_ic;
        }
        else
        {
          c_ic = total_ic - current_ic - 1;
        }
        pec_error = pec_error + parse_cells(current_ic,cell_reg, cell_data,
                                            &ic[c_ic].cells.c_codes[0],
                                            &ic[c_ic].cells.pec_match[0]);
      }
    }
  }
 
  else
  {
    LTC681x_rdcv_reg(reg, total_ic,cell_data);
 
    for (int current_ic = 0; current_ic<total_ic; current_ic++)
    {
      if (ic->isospi_reverse == false)
      {
        c_ic = current_ic;

      }
      else
      {
        c_ic = total_ic - current_ic - 1;
      }
      pec_error = pec_error + parse_cells(current_ic,reg, &cell_data[8*c_ic],
                                          &ic[c_ic].cells.c_codes[0],
                                          &ic[c_ic].cells.pec_match[0]);
    }
  }
  LTC681x_check_pec(total_ic,CELL,ic);
  free(cell_data);
  return(pec_error);
}

//helper function that parses voltage measurement registers
int8_t parse_cells(uint8_t current_ic, uint8_t cell_reg, uint8_t cell_data[], uint16_t *cell_codes, uint8_t *ic_pec)
{

  const uint8_t BYT_IN_REG = 6;
  const uint8_t CELL_IN_REG = 3;
  int8_t pec_error = 0;
  uint16_t parsed_cell;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter = current_ic*NUM_RX_BYT; //data counter


  for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
  {
    // loops once for each of the 3 cell voltage codes in the register

    parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each cell code is received as two bytes and is combined to
    // create the parsed cell voltage code
    cell_codes[current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
    data_counter = data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
                                                        //must increment by two for each parsed cell code
  }

  received_pec = (cell_data[data_counter] << 8) | cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
                                                                            //after the 6 cell voltage data bytes
  data_pec = pec15_calc(BYT_IN_REG, &cell_data[(current_ic) * NUM_RX_BYT]);

  if (received_pec != data_pec)
  {
    pec_error = 1;                             //The pec_error variable is simply set negative if any PEC errors
    ic_pec[cell_reg-1]=1;
  }
  else
  {
    ic_pec[cell_reg-1]=0;
  }
  data_counter=data_counter+2;
  return(pec_error);
}

/*
The function is used
to read the  parsed GPIO codes of the LTC6811. This function will send the requested
read commands parse the data and store the gpio voltages in aux_codes variable
*/
int8_t LTC681x_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
                     uint8_t total_ic,//the number of ICs in the system
                     cell_asic ic[]//A two dimensional array of the gpio voltage codes.
                    )
{
  uint8_t *data;
  int8_t pec_error = 0;
  uint8_t c_ic =0;
  data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

  if (reg == 0)
  {
    for (uint8_t gpio_reg = 1; gpio_reg<ic[0].ic_reg.num_gpio_reg+1; gpio_reg++)                 //executes once for each of the LTC6811 aux voltage registers
    {
      LTC681x_rdaux_reg(gpio_reg, total_ic,data);                 //Reads the raw auxiliary register data into the data[] array
      for (int current_ic = 0; current_ic<total_ic; current_ic++)
      {
        if (ic->isospi_reverse == false)
        {
          c_ic = current_ic;
        }
        else
        {
          c_ic = total_ic - current_ic - 1;
        }
        pec_error = parse_cells(current_ic,gpio_reg, data,
                                &ic[c_ic].aux.a_codes[0],
                                &ic[c_ic].aux.pec_match[0]);

      }
    }
  }
  else
  {
    LTC681x_rdaux_reg(reg, total_ic, data);

    for (int current_ic = 0; current_ic<total_ic; current_ic++)
    {
      if (ic->isospi_reverse == false)
      {
        c_ic = current_ic;
      }
      else
      {
        c_ic = total_ic - current_ic - 1;
      }
      pec_error = parse_cells(current_ic,reg, data,
                              &ic[c_ic].aux.a_codes[0],
                              &ic[c_ic].aux.pec_match[0]);
    }

  }
  LTC681x_check_pec(total_ic,AUX,ic);
  free(data);
  return (pec_error);
}

// Reads and parses the LTC681x stat registers.
int8_t LTC681x_rdstat(uint8_t reg, //Determines which Stat  register is read back.
                      uint8_t total_ic,//the number of ICs in the system
                      cell_asic ic[]
                     )

{

  const uint8_t BYT_IN_REG = 6;
  const uint8_t GPIO_IN_REG = 3;

  uint8_t *data;
  uint8_t data_counter = 0;
  int8_t pec_error = 0;
  uint16_t parsed_stat;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t c_ic = 0;
  data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

  if (reg == 0)
  {

    for (uint8_t stat_reg = 1; stat_reg< 3; stat_reg++)                      //executes once for each of the LTC6811 stat voltage registers
    {
      data_counter = 0;
      LTC681x_rdstat_reg(stat_reg, total_ic,data);                            //Reads the raw statiliary register data into the data[] array

      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every LTC6811 in the daisy chain
      {
        if (ic->isospi_reverse == false)
        {
          c_ic = current_ic;
        }
        else
        {
          c_ic = total_ic - current_ic - 1;
        }
        // current_ic is used as the IC counter
        if (stat_reg ==1)
        {
          for (uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
          {
            // loops once for each of the 3 gpio voltage codes in the register

            parsed_stat = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
            ic[c_ic].stat.stat_codes[current_gpio] = parsed_stat;
            data_counter=data_counter+2;                                               //Because gpio voltage codes are two bytes the data counter

          }
        }
        else if (stat_reg == 2)
        {
          parsed_stat = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
          data_counter = data_counter +2;
          ic[c_ic].stat.stat_codes[3] = parsed_stat;
          ic[c_ic].stat.flags[0] = data[data_counter++];
          ic[c_ic].stat.flags[1] = data[data_counter++];
          ic[c_ic].stat.flags[2] = data[data_counter++];
          ic[c_ic].stat.mux_fail[0] = (data[data_counter] & 0x02)>>1;
          ic[c_ic].stat.thsd[0] = data[data_counter++] & 0x01;
        }

        received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 gpio voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);

        if (received_pec != data_pec)
        {
          pec_error = -1; //The pec_error variable is simply set negative if any PEC errors
          ic[c_ic].stat.pec_match[stat_reg-1]=1;
          //are detected in the received serial data
        }
        else
        {
          ic[c_ic].stat.pec_match[stat_reg-1]=0;
        }

        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs gpio voltage data
      }


    }

  }
  else
  {

    LTC681x_rdstat_reg(reg, total_ic, data);
    for (int current_ic = 0 ; current_ic < total_ic; current_ic++)            // executes for every LTC6811 in the daisy chain
    {
      // current_ic is used as an IC counter
      if (ic->isospi_reverse == false)
      {
        c_ic = current_ic;
      }
      else
      {
        c_ic = total_ic - current_ic - 1;
      }
      if (reg ==1)
      {
        for (uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
        {
          // loops once for each of the 3 gpio voltage codes in the register
          parsed_stat = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
          // create the parsed gpio voltage code

          ic[c_ic].stat.stat_codes[current_gpio] = parsed_stat;
          data_counter=data_counter+2;                        //Because gpio voltage codes are two bytes the data counter
          //must increment by two for each parsed gpio voltage code

        }
      }
      else if (reg == 2)
      {
        parsed_stat = data[data_counter+1] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
        ic[c_ic].stat.stat_codes[3] = parsed_stat;
        ic[c_ic].stat.flags[0] = data[data_counter++];
        ic[c_ic].stat.flags[1] = data[data_counter++];
        ic[c_ic].stat.flags[2] = data[data_counter++];
        ic[c_ic].stat.mux_fail[0] = (data[data_counter] & 0x02)>>1;
        ic[c_ic].stat.thsd[0] = data[data_counter++] & 0x01;
      }


      received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 gpio voltage data bytes
      data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
      if (received_pec != data_pec)
      {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        ic[c_ic].stat.pec_match[reg-1]=1;

      }

      data_counter=data_counter+2;
    }
  }
  LTC681x_check_pec(total_ic,STAT,ic);
  free(data);
  return (pec_error);
}

//Write the LTC681x CFGRA
void LTC681x_wrcfg(uint8_t total_ic, //The number of ICs being written to
                   cell_asic ic[]
                  )
{
  uint8_t cmd[2] = {0x00 , 0x01} ;
  uint8_t write_buffer[256];
  uint8_t write_count = 0;
  uint8_t c_ic = 0;
  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    if (ic->isospi_reverse == true)
    {
      c_ic = current_ic;
    }
    else
    {
      c_ic = total_ic - current_ic - 1;
    }

    for (uint8_t data = 0; data<6; data++)
    {
      write_buffer[write_count] = ic[c_ic].config.tx_data[data];
      write_count++;
    }
  }
  write_68(total_ic, cmd, write_buffer);
}

//Read CFGA
int8_t LTC681x_rdcfg(uint8_t total_ic, //Number of ICs in the system
                     cell_asic *ic
                    )
{
  uint8_t cmd[2]= {0x00 , 0x02};
  uint8_t read_buffer[256];
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t calc_pec;
  uint8_t c_ic = 0;
  pec_error = read_68(total_ic, cmd, read_buffer);
  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    if (ic->isospi_reverse == false)
    {
      c_ic = current_ic;
    }
    else
    {
      c_ic = total_ic - current_ic - 1;
    }

    for (int byte=0; byte<8; byte++)
    {
      ic[c_ic].config.rx_data[byte] = read_buffer[byte+(8*current_ic)];
    }
    calc_pec = pec15_calc(6,&read_buffer[8*current_ic]);
    data_pec = read_buffer[7+(8*current_ic)] | (read_buffer[6+(8*current_ic)]<<8);
    if (calc_pec != data_pec )
    {
      ic[c_ic].config.rx_pec_match = 1;
    }
    else ic[c_ic].config.rx_pec_match = 0;
  }
  LTC681x_check_pec(total_ic,CFGR,ic);
  return(pec_error);
}

//Looks up the result pattern for digital filter self test
uint16_t LTC681x_st_lookup(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint16_t test_pattern = 0;
  if (MD == 1)
  {
    if (ST == 1)
    {
      test_pattern = 0x9565;
    }
    else
    {
      test_pattern = 0x6A9A;
    }
  }
  else
  {
    if (ST == 1)
    {
      test_pattern = 0x9555;
    }
    else
    {
      test_pattern = 0x6AAA;
    }
  }
  return(test_pattern);
}

//Clears all of the DCC bits in the configuration registers
void clear_discharge(uint8_t total_ic, cell_asic ic[])
{
  for (int i=0; i<total_ic; i++)
  {
    ic[i].config.tx_data[4] = 0;
    ic[i].config.tx_data[5] = 0;
  }
}

// Runs the Digital Filter Self Test
int16_t LTC681x_run_cell_adc_st(uint8_t adc_reg,uint8_t total_ic, cell_asic ic[])
{
  int16_t error = 0;
  uint16_t expected_result = 0;
  for (int self_test = 1; self_test<3; self_test++)
  {

    expected_result = LTC681x_st_lookup(2,self_test);
    wakeup_idle(total_ic);
    switch (adc_reg)
    {
      case CELL:
        wakeup_idle(total_ic);
        LTC681x_clrcell();
        LTC681x_cvst(2,self_test);
        LTC681x_pollAdc();//this isn't working
        wakeup_idle(total_ic);
       // error = LTC681x_rdcv(0, total_ic,ic);
        for (int cic = 0; cic < total_ic; cic++)
        {
          for (int channel=0; channel< ic[cic].ic_reg.cell_channels; channel++)
          {
            if (ic[cic].cells.c_codes[channel] != expected_result)
            {
              error = error+1;
            }
          }
        }
        break;
      case AUX:
        error = 0;
        wakeup_idle(total_ic);
        LTC681x_clraux();
        LTC681x_axst(2,self_test);
        LTC681x_pollAdc();
        wait_us(10000);
        wakeup_idle(total_ic);
        LTC681x_rdaux(0, total_ic,ic);
        for (int cic = 0; cic < total_ic; cic++)
        {
          for (int channel=0; channel< ic[cic].ic_reg.aux_channels; channel++)
          {
            if (ic[cic].aux.a_codes[channel] != expected_result)
            {
              error = error+1;
            }
          }
        }
        break;
      case STAT:
        wakeup_idle(total_ic);
        LTC681x_clrstat();
        LTC681x_statst(2,self_test);
        LTC681x_pollAdc();
        wakeup_idle(total_ic);
        error = LTC681x_rdstat(0,total_ic,ic);
        for (int cic = 0; cic < total_ic; cic++)
        {
          for (int channel=0; channel< ic[cic].ic_reg.stat_channels; channel++)
          {
            if (ic[cic].stat.stat_codes[channel] != expected_result)
            {
              error = error+1;
            }
          }
        }
        break;

      default:
        error = -1;
        break;
    }
  }
  return(error);
}

//runs the redundancy self test
int16_t LTC681x_run_adc_redundancy_st(uint8_t adc_mode, uint8_t adc_reg, uint8_t total_ic, cell_asic ic[])
{
  int16_t error = 0;
  for (int self_test = 1; self_test<3; self_test++)
  {
    wakeup_idle(total_ic);
    switch (adc_reg)
    {
      case AUX:
        LTC681x_clraux();
        LTC681x_adaxd(adc_mode,AUX_CH_ALL);
        LTC681x_pollAdc();
        wakeup_idle(total_ic);
        error = LTC681x_rdaux(0, total_ic,ic);
        for (int cic = 0; cic < total_ic; cic++)
        {
          for (int channel=0; channel< ic[cic].ic_reg.aux_channels; channel++)
          {
            if (ic[cic].aux.a_codes[channel] >= 65280)
            {
              error = error+1;
            }
          }
        }
        break;
      case STAT:
        LTC681x_clrstat();
        LTC681x_adstatd(adc_mode,STAT_CH_ALL);
        LTC681x_pollAdc();
        wakeup_idle(total_ic);
        error = LTC681x_rdstat(0,total_ic,ic);
        for (int cic = 0; cic < total_ic; cic++)
        {
          for (int channel=0; channel< ic[cic].ic_reg.stat_channels; channel++)
          {
            if (ic[cic].stat.stat_codes[channel] >= 65280)
            {
              error = error+1;
            }
          }
        }
        break;

      default:
        error = -1;
        break;
    }
  }
  return(error);
}

//Runs the datasheet algorithm for open wire
void LTC681x_run_openwire(uint8_t total_ic, cell_asic ic[])
{
  uint16_t OPENWIRE_THRESHOLD = 4000;
  const uint8_t  N_CHANNELS = ic[0].ic_reg.cell_channels;

  cell_asic pullUp_cell_codes[total_ic];
  cell_asic pullDwn_cell_codes[total_ic];
  cell_asic openWire_delta[total_ic];
  int8_t error;

  wakeup_sleep(total_ic);
  LTC681x_adow(MD_7KHZ_3KHZ,PULL_UP_CURRENT);
  LTC681x_pollAdc();
  wakeup_idle(total_ic);
  LTC681x_adow(MD_7KHZ_3KHZ,PULL_UP_CURRENT);
  LTC681x_pollAdc();
  wakeup_idle(total_ic);
  //error = LTC681x_rdcv(0, total_ic,pullUp_cell_codes);

  wakeup_idle(total_ic);
  LTC681x_adow(MD_7KHZ_3KHZ,PULL_DOWN_CURRENT);
  LTC681x_pollAdc();
  wakeup_idle(total_ic);
  LTC681x_adow(MD_7KHZ_3KHZ,PULL_DOWN_CURRENT);
  LTC681x_pollAdc();
  wakeup_idle(total_ic);
  //error = LTC681x_rdcv(0, total_ic,pullDwn_cell_codes);

  for (int cic=0; cic<total_ic; cic++)
  {
    ic[cic].system_open_wire =0;
    for (int cell=0; cell<N_CHANNELS; cell++)
    {
      if (pullDwn_cell_codes[cic].cells.c_codes[cell]>pullUp_cell_codes[cic].cells.c_codes[cell])
      {
        openWire_delta[cic].cells.c_codes[cell] = pullDwn_cell_codes[cic].cells.c_codes[cell] - pullUp_cell_codes[cic].cells.c_codes[cell]  ;
      }
      else
      {
        openWire_delta[cic].cells.c_codes[cell] = 0;
      }

    }
  }
  for (int cic=0; cic<total_ic; cic++)
  {
    for (int cell=1; cell<N_CHANNELS; cell++)
    {

      if (openWire_delta[cic].cells.c_codes[cell]>OPENWIRE_THRESHOLD)
      {
        ic[cic].system_open_wire += (1<<cell);

      }
    }
    if (pullUp_cell_codes[cic].cells.c_codes[0] == 0)
    {
      ic[cic].system_open_wire += 1;
    }
    if (pullUp_cell_codes[cic].cells.c_codes[N_CHANNELS-1] == 0)
    {
      ic[cic].system_open_wire += (1<<(N_CHANNELS));
    }
  }
}

// Runs the ADC overlap test for the IC
//RUNlı bütün fonksiyonlar değiştirildi.
uint16_t LTC681x_run_adc_overlap(uint8_t total_ic, cell_asic ic[])
{
  uint16_t error = 0;
  int32_t measure_delta =0;
  int16_t failure_pos_limit = 20;
  int16_t failure_neg_limit = -20;
  wakeup_idle(total_ic);
  LTC681x_adol(MD_7KHZ_3KHZ,DCP_DISABLED);
  LTC681x_pollAdc();
  wakeup_idle(total_ic);
  //error = LTC681x_rdcv(0, total_ic,ic);
  for (int cic = 0; cic<total_ic; cic++)
  {
    measure_delta = (int32_t)ic[cic].cells.c_codes[6]-(int32_t)ic[cic].cells.c_codes[7];
    if ((measure_delta>failure_pos_limit) || (measure_delta<failure_neg_limit))
    {
      error = error | (1<<(cic-1));
    }
  }
  return(error);
}

//Helper function that increments PEC counters
void LTC681x_check_pec(uint8_t total_ic,uint8_t reg, cell_asic ic[])
{
  switch (reg)
  {
    case CFGR:
      for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
      {
        ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].config.rx_pec_match;
        ic[current_ic].crc_count.cfgr_pec = ic[current_ic].crc_count.cfgr_pec + ic[current_ic].config.rx_pec_match;
      }
      break;

    case CFGRB:
      for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
      {
        ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].configb.rx_pec_match;
        ic[current_ic].crc_count.cfgr_pec = ic[current_ic].crc_count.cfgr_pec + ic[current_ic].configb.rx_pec_match;
      }
      break;
    case CELL:
      for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
      {
        for (int i=0; i<ic[0].ic_reg.num_cv_reg; i++)
        {
          ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].cells.pec_match[i];
          ic[current_ic].crc_count.cell_pec[i] = ic[current_ic].crc_count.cell_pec[i] + ic[current_ic].cells.pec_match[i];
        }
      }
      break;
    case AUX:
      for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
      {
        for (int i=0; i<ic[0].ic_reg.num_gpio_reg; i++)
        {
          ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + (ic[current_ic].aux.pec_match[i]);
          ic[current_ic].crc_count.aux_pec[i] = ic[current_ic].crc_count.aux_pec[i] + (ic[current_ic].aux.pec_match[i]);
        }
      }

      break;
    case STAT:
      for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
      {

        for (int i=0; i<ic[0].ic_reg.num_stat_reg-1; i++)
        {
          ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].stat.pec_match[i];
          ic[current_ic].crc_count.stat_pec[i] = ic[current_ic].crc_count.stat_pec[i] + ic[current_ic].stat.pec_match[i];
        }
      }
      break;
    default:
      break;
  }
}

//Helper Function to reset PEC counters
void LTC681x_reset_crc_count(uint8_t total_ic, cell_asic ic[])
{
  for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
  {
    ic[current_ic].crc_count.pec_count = 0;
    ic[current_ic].crc_count.cfgr_pec = 0;
    for (int i=0; i<6; i++)
    {
      ic[current_ic].crc_count.cell_pec[i]=0;

    }
    for (int i=0; i<4; i++)
    {
      ic[current_ic].crc_count.aux_pec[i]=0;
    }
    for (int i=0; i<2; i++)
    {
      ic[current_ic].crc_count.stat_pec[i]=0;
    }
  }
}

//Helper function to intialize CFG variables.
void LTC681x_init_cfg(uint8_t total_ic, cell_asic ic[])
{
  bool REFON = true;
  bool ADCOPT = false;
  bool gpioBits[5] = {true,true,true,true,true};
  bool dccBits[12] = {false,false,false,false,false,false,false,false,false,false,false,false};
  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    for (int j =0; j<6; j++)
    {
      ic[current_ic].config.tx_data[j] = 0;
      ic[current_ic].configb.tx_data[j] = 0;
    }
    LTC681x_set_cfgr(current_ic ,ic,REFON,ADCOPT,gpioBits,dccBits);

  }
}

//Helper function to set CFGR variable
void LTC681x_set_cfgr(uint8_t nIC, cell_asic ic[], bool refon, bool adcopt, bool gpio[5],bool dcc[12])
{
  LTC681x_set_cfgr_refon(nIC,ic,refon);
  LTC681x_set_cfgr_adcopt(nIC,ic,adcopt);
  LTC681x_set_cfgr_gpio(nIC,ic,gpio);
  LTC681x_set_cfgr_dis(nIC,ic,dcc);
}



//Helper function to set the REFON bit
void LTC681x_set_cfgr_refon(uint8_t nIC, cell_asic ic[], bool refon)
{
  if (refon) ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|0x04;
  else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&0xFB;
}

//Helper function to set the adcopt bit
void LTC681x_set_cfgr_adcopt(uint8_t nIC, cell_asic ic[], bool adcopt)
{
  if (adcopt) ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|0x01;
  else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&0xFE;
}

//Helper function to set GPIO bits
void LTC681x_set_cfgr_gpio(uint8_t nIC, cell_asic ic[],bool gpio[5])
{
  for (int i =0; i<5; i++)
  {
    if (gpio[i])ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|(0x01<<(i+3));
    else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&(~(0x01<<(i+3)));
  }
}

//Helper function to control discharge
void LTC681x_set_cfgr_dis(uint8_t nIC, cell_asic ic[],bool dcc[12])
{
  for (int i =0; i<8; i++)
  {
    if (dcc[i])ic[nIC].config.tx_data[4] = ic[nIC].config.tx_data[4]|(0x01<<i);
    else ic[nIC].config.tx_data[4] = ic[nIC].config.tx_data[4]& (~(0x01<<i));
  }
  for (int i =0; i<4; i++)
  {
    if (dcc[i+8])ic[nIC].config.tx_data[5] = ic[nIC].config.tx_data[5]|(0x01<<i);
    else ic[nIC].config.tx_data[5] = ic[nIC].config.tx_data[5]&(~(0x01<<i));
  }
}

//*************************** LTC 6811 ****************************************//
void LTC6811_init_reg_limits(uint8_t total_ic, cell_asic ic[])
{
  for (uint8_t cic=0; cic<total_ic; cic++)
  {
    ic[cic].ic_reg.cell_channels=12;
    ic[cic].ic_reg.stat_channels=4;
    ic[cic].ic_reg.aux_channels=6;
    ic[cic].ic_reg.num_cv_reg=4;
    ic[cic].ic_reg.num_gpio_reg=2;
    ic[cic].ic_reg.num_stat_reg=3;
  }
}

//Helper function to set discharge bit in CFG register
void LTC6811_set_discharge(int Cell, uint8_t total_ic, cell_asic ic[])
{
  for (int i=0; i<total_ic; i++)
  {
    if (Cell<9)
    {
      ic[i].config.tx_data[4] = ic[i].config.tx_data[4] | (1<<(Cell-1));
    }
    else if (Cell < 13)
    {
      ic[i].config.tx_data[5] = ic[i].config.tx_data[5] | (1<<(Cell-9));
    }
  }
}

//****************************** SPI **************************************//
/*
Writes an array of bytes out of the SPI port
*/
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
  for (uint8_t i = 0; i < len; i++)
  {
    spi.write((int8_t)data[i]);
  }

}

/*
 Writes and read a set number of bytes using the SPI port.

*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
  for (uint8_t i = 0; i < tx_len; i++)
  {
    spi.write(tx_Data[i]);
  }

  for (uint8_t i = 0; i < rx_len; i++)
  {

    rx_data[i] = (uint8_t)spi.write(0xFF);
  }

}

uint8_t spi_read_byte(uint8_t tx_dat)
{
  uint8_t data;
  data = (uint8_t)spi.write(0xFF);
  return(data);
}

//****************************** UserInterface **************************************//
char ui_buffer[UI_BUFFER_SIZE];

// Read data from the serial interface into the ui_buffer
uint8_t read_data()
{
  uint8_t index = 0; //index to hold current location in ui_buffer
  int c; // single character used to store incoming keystrokes
  while (index < UI_BUFFER_SIZE-1)
  {
    //c = pc.getc(); //read one character
    c = getchar(); 
    if (((char) c == '\r') || ((char) c == '\n')) break; // if carriage return or linefeed, stop and return data
    if ( ((char) c == '\x7F') || ((char) c == '\x08') )   // remove previous character (decrement index) if Backspace/Delete key pressed      index--;
    {
      if (index > 0) index--;
    }
    else if (c >= 0)
    {
      ui_buffer[index++]=(char) c; // put character into ui_buffer
    }
  }
  ui_buffer[index]='\0';  // terminate string with NULL

  if ((char) c == '\r')    // if the last character was a carriage return, also clear linefeed if it is next character
  {
    wait_us(10000);  // allow 10ms for linefeed to appear on serial pins
    //if (pc.peek() == '\n') 
    
    //pc.getc(); // if linefeed appears, read it and throw it away
  }

  return index; // return number of characters, not including null terminator
}

// Read an integer from the serial interface.
// The routine can recognize Hex, Decimal, Octal, or Binary
// Example:
// Hex:     0x11 (0x prefix)
// Decimal: 17
// Octal:   021 (leading zero prefix)
// Binary:  B10001 (leading B prefix)
int32_t read_int()
{
  int32_t data;
  read_data();
  if (ui_buffer[0] == 'm')
    return('m');
  if ((ui_buffer[0] == 'B') || (ui_buffer[0] == 'b'))
  {
    data = strtol(ui_buffer+1, NULL, 2);
  }
  else
    data = strtol(ui_buffer, NULL, 0);
  return(data);
}

// Read a character from the serial interface
int8_t read_char()
{
  read_data();
  return(ui_buffer[0]);
}

