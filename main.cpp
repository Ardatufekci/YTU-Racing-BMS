#include "mbed.h"
#include "bms_master.h"
#include <cstdint>
#include <cstdio>
#include <stdint.h>
#include "MCP23017.h"
/*
Yapılacaklar 
Delayler güncellenecek voltaj ölçümündeki yerlerde 
watchdog entegrasyonu yapılacak

*/
#define ENABLED 1
#define DISABLED 0

#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

////////////// DEFINE PINS
#define I2C_SDA                 PB_9
#define I2C_SCL                 PB_8
#define CAN1_RX_PIN             PD_0
#define CAN1_TX_PIN             PD_1
#define Thermistor_PIN_1        PA_5
#define Thermistor_PIN_2        PA_6
#define Thermistor_PIN_3        PA_7
#define Thermistor_PIN_4        PB_1
#define Thermistor_PIN_5        PC_2
#define Thermistor_PIN_6        PB_0
#define Thermistor_PIN_7        PA_0
#define Thermistor_PIN_8        PF_9
#define Thermistor_PIN_9        PC_3
#define Thermistor_PIN_10       PF_6
#define Thermistor_PIN_11       PF_7
#define Thermistor_PIN_12       PC_1
#define AIR_P_Pin              D10
#define AIR_N_Pin              D9
#define PreCharge_Pin          D8
#define Shutdown_Pin           D6 
#define Discharge_Pin          D5     
#define MCP23017_DEFAULT_ADDR 0x40
#define MCP23017_DEFAULT_ADDR2 0x42
#define MCP23017_DEFAULT_ADDR3 0x44
#define MCP23017_PORTA 0x00
#define MCP23017_PORTB 0x01

DigitalOut Discharge(Discharge_Pin);
DigitalOut Air_P(AIR_P_Pin);
DigitalOut Air_N(AIR_N_Pin);
DigitalOut PreCharge(PreCharge_Pin);
DigitalOut Shutdown(Shutdown_Pin);

float olcum1;
float olcum2;
float olcum3;
float olcum4;
float olcum5;
float olcum6;
float olcum7;
float olcum8;
float olcum9;
float olcum10;
float olcum11;
float olcum12;

//10k thermistorler için
float A = -0.9098880323e-3;
float B = 2.148732041e-4;
float C = 1.064658112e-7;

//100k thermistorler için
float a1 = 0.8268883559e-3;
float b1 = 2.088318209e-4 ;
float c1 = 0.8051409251e-7; 

float R1 = 1000;
float logR2,R2,T,Tc,Tf;


AnalogIn ain1(Thermistor_PIN_1,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain2(Thermistor_PIN_2,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain3(Thermistor_PIN_3,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain4(Thermistor_PIN_4,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain5(Thermistor_PIN_5,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain6(Thermistor_PIN_6,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain7(Thermistor_PIN_7,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain8(Thermistor_PIN_8,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain9(Thermistor_PIN_9,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain10(Thermistor_PIN_10,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain11(Thermistor_PIN_11,MBED_CONF_TARGET_DEFAULT_ADC_VREF);
AnalogIn ain12(Thermistor_PIN_12,MBED_CONF_TARGET_DEFAULT_ADC_VREF);



#define TRANSMIT_OBJECT_0_ADRESS 0x04

#define CAN1_FREQUENCY          1000000 // 1 Mb/s

typedef struct {
    int     id = TRANSMIT_OBJECT_0_ADRESS;
    bool    uv;      // 1 bit
    bool    ov;         // 1 bit
    bool    precharge;        // 1 bit
    bool    charger;          // 1 bit
    bool    Discharge;        // 1 bit
    bool    Fan;            // 1 bit
    bool    status[2];      // 2 bit

    int TotalVoltage;
    int Sıcaklık;
    int Akim;

/*
    bool    Fan;            // 1 bit
    bool    PWMEnable;      // 1 bit
    bool    MCUOperational; // 1 bit
    bool    MCUPreOperational;// 1 bit
    bool    BMSEnable;
    bool    DisplayEnable;
    bool    TSMasterSwitch;
    bool    HeartBeat;
*/


    char    data[7];
    void calcData(){
        data[0] = (uv << 0) | (ov << 1) | (precharge << 2) | (charger << 3) | (Discharge << 4) | (Fan << 5) | (status[0] << 6) | (status[1] << 7);
        data[1] = ((TotalVoltage%256));
        data[2] = (TotalVoltage/256)%256;
        data[3] = ((Sıcaklık%256));
        data[4] = (Sıcaklık/256)%256;
        data[5] = ((Akim%256));
        data[6] = (Akim/256)%256;

        //data[1] = (Fan << 0) | (PWMEnable << 1) | (MCUOperational << 2) | (MCUPreOperational << 3) | (BMSEnable << 4) | (DisplayEnable << 5) | (TSMasterSwitch << 6) | (HeartBeat << 7);
    }

} CANTransmitObject0;

//Değişkenler
const uint8_t slave_sayi = 1; //daisy chaindeki slave sayısı


//ADC command conf
//const uint8_t ADC_OPT = ADC_OPT_ENABLED; //ADCOPT İLE DEĞİŞTİRİLECEK
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;
const uint8_t ADC_DCP = DCP_DISABLED;
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; 
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;
const uint8_t NO_OF_REG = REG_ALL;
//UNDER VOLTAGE VE OVER VOLTAGE LİMİTLERİ
const uint16_t OverVoltTh = 27000; //450000
const uint16_t UnderVoltTh = 10000; //325000
const uint8_t  OverTemp = 60;
//Loop Measurement config
const uint8_t WRITE_CONFIG = ENABLED; 
const uint8_t READ_CONFIG = ENABLED; 
const uint8_t MEASURE_CELL = ENABLED; 
const uint8_t MEASURE_AUX = DISABLED; 
const uint8_t MEASURE_STAT = DISABLED;
const uint8_t MEASURE_MUX1 = DISABLED;
const uint8_t MEASURE_MUX2 = DISABLED;
const uint8_t MEASURE_MUX3 = DISABLED;
bool GPIOBITS_A[5] = {true,true,true,true,true}; // OUTPUT = {false,false,false,false,false}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
////////////////// GPIOBITS_A lardandan birisi true olursa rdcfg ne döndürür ona bakılmalı
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool DCCBITS_A[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
cell_asic cell_a[slave_sayi]; // Her bir slave'in ayrı objesinin listesi

//Bizim kısım
CAN can1(CAN1_RX_PIN,CAN1_TX_PIN);
I2C i2c(I2C_SDA,I2C_SCL);
MCP23017::MCP23017 io(i2c,MCP23017_DEFAULT_ADDR);
MCP23017::MCP23017 io2(i2c,MCP23017_DEFAULT_ADDR2);
MCP23017::MCP23017 io3(i2c,MCP23017_DEFAULT_ADDR3);
CANMessage Hellomsg = 0x00;
CANMessage msg;
CANMessage preChargeRcvMsg;
CANTransmitObject0 BMS;
//AnalogIn ain(PA_7_ALT2,5);
int zaman;
bool PreChargeState = false;
float meas;
//static BufferedSerial Serial(USBTX,USBRX);
int hata = 0;

#define MOSFET_STATUS_PIN 7 // ????
#define VCU_ID 0x6B // BUNLARI VCU İLE KONUŞUP DEĞİŞTİRECEĞİZ
#define KELLY_KDHE_RESPONSE_CAN_ID 0x73
#define AMS_CAN_ID 0x69
#define CCP_A2D_BATCH_READ1 0x1b


void print_pec()
{
    for (int current_ic=0; current_ic<slave_sayi; current_ic++) {
        printf("\n%d", cell_a[current_ic].crc_count.pec_count);
        printf(" : PEC Errors Detected on IC");

        printf("%d\n", current_ic+1);
    }
}
void print_cells(uint8_t datalog_en)
{

  float total = 0.00;
  for (int current_ic = 0 ; current_ic < slave_sayi; current_ic++)
  {
    if (datalog_en == 1)
    {
      printf(" IC ");
      printf("%d \n",current_ic+1);
      printf(", ");
      for (int i=0; i<CELL_CHANNELS; i++)
        {
        
        printf(" C");
        printf("%d \n",i+1);
        printf(":");
        //printf("%f,,,%d  \n",cell_codes[current_ic][i]*0.001,4);
        printf("%f",cell_a[current_ic].cells.c_codes[i]*0.001);
        total = total + cell_a[current_ic].cells.c_codes[i]*0.001;
       
        printf(",");
        
        
      }
     
    }
    else
    {
      printf("Cells, ");
      for (int i=0; i<CELL_CHANNELS; i++)
      {
      //printf("%f,,,%d \n ",cell_codes[current_ic][i]*0.001,4);
        printf("%f",cell_a[current_ic].cells.c_codes[i]*0.001);
        
              
      }
    }
  }
  
  printf("Pack Voltage: %f",total);
  
}
void check_error(int hata)
{
    if (hata == -1) {
    printf("PEC HATASI !!!");
    }
}
void serial_print_hex(uint8_t data)
{ //Tekrar bakılacak
  if (data< 16)
  {
    printf("0");
    printf("0x0%X",data);
  }
  else
    printf("0x%X",data);
}
void print_config()
{   //üsttekine göre baştan yazıldı
    int cfg_pec;
    printf("Written Config");
    for (int current_ic = 0; current_ic<slave_sayi; current_ic++)
    {
    printf("IC %d \n",current_ic+1);
    //printf("0 = %d \n",tx_cfg[current_ic][0]);
    printf("0 = %d \n",cell_a[current_ic].config.tx_data[0]);
    serial_print_hex(cell_a[current_ic].config.tx_data[0]);
    //printf("1 = %d \n",tx_cfg[current_ic][1]);
    printf("1 = %d \n",cell_a[current_ic].config.tx_data[1]);
    serial_print_hex(cell_a[current_ic].config.tx_data[1]);
    //printf("2 = %d \n",tx_cfg[current_ic][2]);
    printf("2 = %d \n",cell_a[current_ic].config.tx_data[2]);
    serial_print_hex(cell_a[current_ic].config.tx_data[2]);
    //printf("3 = %d \n",tx_cfg[current_ic][3]);
    printf("3 = %d \n",cell_a[current_ic].config.tx_data[3]);
    serial_print_hex(cell_a[current_ic].config.tx_data[3]);
    //printf("4 = %d \n",tx_cfg[current_ic][4]);
    printf("4 = %d \n",cell_a[current_ic].config.tx_data[4]);
    serial_print_hex(cell_a[current_ic].config.tx_data[4]);
    //printf("5 = %d \n",tx_cfg[current_ic][5]);
    printf("5 = %d \n",cell_a[current_ic].config.tx_data[5]);
    serial_print_hex(cell_a[current_ic].config.tx_data[5]);
    printf("--------- \n");
    printf("Calculated PEC: ");
    cfg_pec = pec15_calc(6,&cell_a[current_ic].config.tx_data[0]);
    printf("%d \n",(uint8_t)(cfg_pec>>8));
    serial_print_hex(cfg_pec>>8);
    printf("--- \n");
    printf("%d \n",(uint8_t)(cfg_pec));
    serial_print_hex(cfg_pec);
    printf("--- \n");
    printf("Print Config OK !! \n");
    
    printf("Tekrar");
    }
}


void print_rxconfig()
{
    // ÜSTEKKİ cell_asic tipinde baştan yazıldı
    int cfg_pec;
    printf("Received Config\n");
    for (int current_ic = 0; current_ic<slave_sayi; current_ic++)
    {
    printf("IC %d \n",current_ic+1);
    printf("0 = %d \n",cell_a[current_ic].config.rx_data[0]);
    serial_print_hex(cell_a[current_ic].config.rx_data[0]);
    printf("1 = %d \n",cell_a[current_ic].config.rx_data[1]);
    serial_print_hex(cell_a[current_ic].config.rx_data[1]);
    printf("2 = %d \n",cell_a[current_ic].config.rx_data[2]);
    serial_print_hex(cell_a[current_ic].config.rx_data[2]);
    printf("3 = %d \n",cell_a[current_ic].config.rx_data[3]);
    serial_print_hex(cell_a[current_ic].config.rx_data[3]);
    printf("4 = %d \n",cell_a[current_ic].config.rx_data[4]);
    serial_print_hex(cell_a[current_ic].config.rx_data[4]);
    printf("5 = %d \n",cell_a[current_ic].config.rx_data[5]);
    serial_print_hex(cell_a[current_ic].config.rx_data[5]);
    printf("---------\n");
    printf("Receive Config OK !!\n");
    }
    
}
void print_aux()
{
    // baştan yazıldı önceki hali duruyor
    for (int current_ic =0 ; current_ic < slave_sayi; current_ic++)
  {
    if (ENABLED)
    {
      printf(" IC %d",current_ic+1);
      
      for (int i=0; i < 5; i++)
      {
        printf(" GPIO-");
        printf("%d",i+1);
        printf(":");
        //printf("%f,,,,%d",aux_codes[current_ic][i]*0.0001,4);
        printf("%f,,,,%d",cell_a[current_ic].aux.a_codes[i]*0.0001,4);
        //printf("%f",cell_a[current_ic][i].aux.a_codes);
       // printf("%f,,",cell_a[current_ic][i].aux.pec_match[0]*0.001);
        printf(",");
      }
      printf(" Vref2");
      printf(":");
      printf("%f,,,,%d",cell_a[current_ic].aux.a_codes[5]*0.0001,4);
      printf("\n");
    }
    else
    {
      printf("AUX, ");

      for (int i=0; i < 6; i++)
      {
        printf("%f,,,,%d",cell_a[current_ic].aux.a_codes[i]*0.0001,4);
        printf(",");
      }
    }
    printf("\n");
}
}
void print_stat()
{
    // BAŞTAN ÜSTTEKİNE GÖRE cell_asic tipinde yazıldı
    for (int current_ic =0 ; current_ic < slave_sayi; current_ic++)
    {
    printf("IC %d",current_ic+1);
    printf(" SOC:");
    //printf("%f,,,,%d",stat_codes[current_ic][0]*0.0001*20,4);
    printf("%f,,,%d",cell_a[current_ic].stat.stat_codes[0]*0.0001,4);
    printf(",");
    printf(" Itemp:");
    printf("%f,,,,%d",cell_a[current_ic].stat.stat_codes[1]*0.0001,4);
    printf(",");
    printf(" VregA:");
    printf("%f,,,,%d",cell_a[current_ic].stat.stat_codes[2]*0.0001,4);
    printf(",");
    printf(" VregD:");
    printf("%f,,,,%d",cell_a[current_ic].stat.stat_codes[3]*0.0001,4);
    printf("\n");
    }
    
}

void gpio_out()
{   //GPIO MUX İLE KULLANMAK İÇİN FONKSİYON 
    //10k ohm ile pull-up atılacak output olarak kullanılacak GPIO'lara
    bool GPIOBITS_A[5] = {false,false,false,false,true} ;
    // ADCOPT = true; KONTROL EDİLMELİ
    wakeup_sleep(slave_sayi);
    
    for (uint8_t current_ic = 0; current_ic<slave_sayi;current_ic++)
    {   
        for (uint8_t current_gpio; current_gpio<5; current_gpio++) {
        GPIOBITS_A[current_gpio] = false;
        if (current_gpio>1) {
        GPIOBITS_A[current_gpio-1] = true;
        }
        LTC681x_set_cfgr(current_ic, cell_a, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A);
        
        wakeup_idle(slave_sayi);
        LTC681x_wrcfg(slave_sayi,cell_a);
        printf("GPIO Out OK !!");
    
        printf("GPIO Sıcaklık okuması başlanıyor");
        wakeup_sleep(slave_sayi);
        LTC681x_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
        wakeup_idle(slave_sayi);
        hata = LTC681x_rdaux(0,slave_sayi, cell_a);
        check_error(hata);
        zaman = LTC681x_pollAdc();
        printf("Sıcaklık ölçümü süresi: %d",zaman);

        } 
    }
    

}

void setup()
{   
    printf("başladım");
    CS_PIN = 1; ///// GENEL AYARLAMA
    
    spi.format(8,3); //spi.format(11); 11 bit diye düşünmüştük ama değil
    spi.frequency(100000); //default 1MHz olduğu için

    CS_PIN = 0;
    for (int current_ic = 0; current_ic < slave_sayi; current_ic++) {
        
        LTC681x_set_cfgr(current_ic, cell_a, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A);
    }

    LTC681x_reset_crc_count(slave_sayi, cell_a);
    
    LTC6811_init_reg_limits(slave_sayi, cell_a);
    CS_PIN=1;
    wakeup_idle(slave_sayi);
    LTC681x_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    wait_us(500);
   // LTC681x_rdcv(0, uint8_t total_ic, cell_asic *ic)
    
    can1.frequency(CAN1_FREQUENCY);
    BMS.status[0] = 1;
    BMS.precharge = PreChargeState;
    BMS.calcData();
    can1.write(CANMessage(BMS.id,BMS.data,sizeof(BMS.data)));
    if (true)//can1.read(CANMessage()))
    {
        printf("CAN init OK");
        PreChargeState = true;
        /*
        unsigned char preChargeSendMsg[1] = { CCP_A2D_BATCH_READ1 };
        unsigned char preChargeRcvMsg[5];
        unsigned char len = 0;
        int8_t error = 0;
        */
        BMS.precharge = PreChargeState;
        BMS.calcData();
        can1.write(CANMessage(BMS.id,BMS.data,sizeof(BMS.data)));
        printf("PreCharge'a başlanıyor");
    }
    else 
    {
    printf("CAN init FAIL");
    setup();
    }
    while (PreChargeState) {
    //can1.write();
    if (can1.read(preChargeRcvMsg)) {
    float kellyVolt = preChargeRcvMsg.data[4]/1.39;
    printf("Kelly volt is %f",kellyVolt); // 
    LTC681x_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    zaman = LTC681x_pollAdc();
    printf("Süre = %d",zaman);
    wakeup_idle(slave_sayi);
    hata = LTC681x_rdcv(0, slave_sayi, cell_a);
    check_error(hata);
    float total = 0.00;
        for (int current_ic = 0 ; current_ic < slave_sayi; current_ic++)
        {
          for (int i=0; i<=CELL_CHANNELS; i++)
          {
            //total = total + (cell_codes[current_ic][i]*0.0001);
            total = total + cell_a[current_ic].cells.c_codes[i]*0.0001;
          }
        }
        printf("Pack Voltage = %f",total);

        float thresholdVoltage = total*(.95);

        if (thresholdVoltage >= UnderVoltTh & thresholdVoltage < OverVoltTh) {
        printf("PreCharge done");
        PreChargeState = false;
        BMS.precharge = PreChargeState;
        BMS.status[0] = 0;
        BMS.status[1] = 1;
        BMS.calcData();
        can1.write(CANMessage(BMS.id,BMS.data,sizeof(BMS.data)));
        break;
        }
        else {
        BMS.status[0] = 1;
        BMS.status[0] = 1;
        BMS.calcData();
        can1.write(CANMessage(BMS.id,BMS.data,sizeof(BMS.data)));
        }
    }
    }
    wait_us(50000);
}

void loop()
{
    
    unsigned char len = 0;
    unsigned char buf[8];
    int8_t error = 0;
    wakeup_sleep(slave_sayi);
    LTC681x_wrcfg(slave_sayi, cell_a);
    /*
    Bu kısımda charging yapmak için gerekli eylemler ve aksiyonlar belirlenecek
    if (can1.read(msg)) {
    if (msg.id = VCU_ID){
        //Charging kısmı buraya
    }
    }
    */
    LTC681x_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    zaman = LTC681x_pollAdc();
    printf("Süre = %d",zaman);
    wakeup_idle(slave_sayi);
    hata = LTC681x_rdcv(0, slave_sayi, cell_a);
    //LTC681x_rdcv(uint8_t reg, uint8_t total_ic, uint16_t (*cell_codes)[12])
    check_error(hata);

    float total = 0.00;
    for (int current_ic = 0 ; current_ic < slave_sayi; current_ic++)
    {
      printf("Cells, ");
      for (int i=0; i<=CELL_CHANNELS; i++)
      { 
        for (int j = 0; j<=CELL_CHANNELS; j++) {
        
        
        if (cell_a[current_ic].cells.c_codes[i]*0.001 < 1)//cell_codes[current_ic][i]*0.0001 < 1)
        {
          //digitalWrite(MOSFET_STATUS_PIN, LOW); 
          printf("UNDER VOLTAGE FOR CELL: %d \n",i);
        }
        printf("cell codes %f,,,%d SLAVE ,,, %d CELL   \n",cell_a[current_ic].cells.c_codes[i]*0.001,i,j);
        total = total + (cell_a[current_ic].cells.c_codes[i]*0.001);
        HAL_Delay(500);
        }
      }
    }
}

void olcum()
{
    hata = 0;
    
    if (WRITE_CONFIG == ENABLED) {
    wakeup_idle(slave_sayi);
    wakeup_sleep(slave_sayi);
    LTC681x_wrcfg(slave_sayi, cell_a);
    print_config();
    printf("Write config OK!! \n");
    }
    
    
    if (READ_CONFIG == ENABLED)
    {
    wakeup_idle(slave_sayi);
    wakeup_sleep(slave_sayi);
    hata = LTC681x_rdcfg(slave_sayi, cell_a);
    check_error(hata);
    print_rxconfig();
    printf("Read Config OK!! \n");
    }
    
    
    if (MEASURE_CELL == ENABLED)
    {
        wakeup_idle(slave_sayi); // iki örnektede de bu kullanılmış sleep hatalı
       // wakeup_sleep(slave_sayi);
        LTC681x_adcv(ADC_CONVERSION_MODE,ADC_DCP ,CELL_CH_TO_CONVERT);
        //wait_us(10000);
        
        zaman = LTC681x_pollAdc(); //Delay kesinleştirilecek
        wakeup_idle(slave_sayi);
        hata = LTC681x_rdcv(NO_OF_REG,slave_sayi, cell_a);
        printf("Ölçüm süresi = %d \n",zaman);
        //check_error(hata);
        print_cells(ENABLED);
        int total;
        for (int current_ic = 0; current_ic == slave_sayi; current_ic++) {
            for (int cell_sayi = 0; cell_sayi == 12 ; cell_sayi++) {
                if(cell_a[current_ic].cells.c_codes[cell_sayi]*0.0001 <= UnderVoltTh)
                {
                    BMS.uv = 1;
                    BMS.status[0] = 1;
                    BMS.status[1] = 1;
                   // BMS.calcData();
                   // can1.write(CANMessage(BMS.id,BMS.data,sizeof(BMS.data)));            
                    Shutdown = 1;
                }
                else if (cell_a[current_ic].cells.c_codes[cell_sayi] == 0xffff) {
                    printf("bağlantı hatası");
                    BMS.status[0] = 1;
                    BMS.status[1] = 1;
                    //BMS.calcData();
                    //can1.write(CANMessage(BMS.id,BMS.data,sizeof(BMS.data)));
                }
                else 
                {

                    total = total + cell_a[current_ic].cells.c_codes[cell_sayi]*0.0001;
                    printf("Cell Measure OK!! \n");
                }
            }
            BMS.TotalVoltage = total;
            BMS.status[0] = 0;
            BMS.status[1] = 1;
            //BMS.calcData();
            //can1.write(CANMessage(BMS.id,BMS.data,sizeof(BMS.data)));
        }
        
    }

    if (MEASURE_AUX == ENABLED)  
    {
    wakeup_idle(slave_sayi);
    LTC681x_adax(ADC_CONVERSION_MODE,AUX_CH_ALL);
    LTC681x_pollAdc(); //Delay kesinleştirilecek
    wakeup_idle(slave_sayi);
    hata = LTC681x_rdaux(0,slave_sayi, cell_a);
    check_error(hata);
    for (int current_ic = 0; current_ic == slave_sayi; current_ic++) {
            for (int aux_sayi = 0; aux_sayi == 4 ; aux_sayi++) {
                if(cell_a[current_ic].aux.a_codes[aux_sayi]*0.0001 >= OverTemp)
                {
                    Shutdown = 1;
                    BMS.Sıcaklık = (int)cell_a[current_ic].aux.a_codes[aux_sayi]*0.0001 ;
                    BMS.status[0] = 1;
                    BMS.status[1] = 1;
                    BMS.calcData();
                    can1.write(CANMessage(BMS.id,BMS.data,sizeof(BMS.data)));            
                }
                else if (cell_a[current_ic].aux.a_codes[aux_sayi] == 0xffff) {
                    printf("bağlantı hatası");
                    BMS.status[0] = 1;
                    BMS.status[1] = 1;
                    BMS.calcData();
                    can1.write(CANMessage(BMS.id,BMS.data,sizeof(BMS.data)));
                }
                else 
                {
                    printf("Aux Measure OK!! \n");
                }
            }
        }
    }
    if (MEASURE_MUX1 == ENABLED) 
    {
    io.setDirrection(MCP23017_PORTB, 0x00); //PORT B output ayarlanması
    io.setDirrection(MCP23017_PORTA, 0x00); //PORT A output ayarlanması
    char data[2];
    data[0] = 0x01;
    data[1] = 0x00;
    char dongu[16] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
    i2c.write(0x20<<1,data,2,0);
    data[0] = 0x13; //PORT B için 
    for (int i = 0; i<=15; i++) {
        data[1] = dongu[i];
        i2c.write(0x20<<1,data,2,0);
        olcum1 = ain1.read();
        olcum2 = ain2.read();
        R2 = R1 * (1023.0/olcum1 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > OverTemp) {
        //SHUTDOWN
        printf("shutdownnn");
        Shutdown = 1;
        }
        R2 = R1 * (1023.0/olcum2 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > OverTemp) {
        //SHUTDOWN
        printf("shutdownnn");
        Shutdown = 1;
        }
        printf("%d.Sıcaklık = %f \n",i,Tc);
        printf("--------\n");
    }
    data[0] = 0x12; //PORT A için
    for (int i = 0; i<=15; i++) {
        data[1] = dongu[i];
        i2c.write(0x20<<1,data,2,0);
        olcum3 = ain1.read();
        olcum4 = ain2.read();
        R2 = R1 * (1023.0/olcum3 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > 70) {
        //SHUTDOWN
        printf("SHUTDOWN");
        Shutdown = 1;
        }
        R2 = R1 * (1023.0/olcum4 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > 70) {
        //SHUTDOWN
        printf("SHUTDOWN");
        Shutdown = 1;
        }
        printf("%d.Sıcaklık = %f \n",i,Tc);
        printf("--------\n");
    }
    
    }
    if (MEASURE_MUX2 == ENABLED) 
    {
    io2.setDirrection(MCP23017_PORTB, 0x00); //PORT B output ayarlanması
    io2.setDirrection(MCP23017_PORTA, 0x00); //PORT A output ayarlanması
    char data[2];
    data[0] = 0x01;
    data[1] = 0x00;
    char dongu[16] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
    i2c.write(0x21<<1,data,2,0);
    data[0] = 0x13; //PORT B için 
    for (int i = 0; i<=15; i++) {
        data[1] = dongu[i];
        i2c.write(0x20<<1,data,2,0);
        olcum5 = ain5.read();
        olcum6 = ain6.read();
        R2 = R1 * (1023.0/olcum1 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > 70) {
        //SHUTDOWN
        printf("SHUTDOWN");
        Shutdown = 1;
        }
        R2 = R1 * (1023.0/olcum2 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > 70) {
        //SHUTDOWN
        printf("SHUTDOWN");
        Shutdown = 1;
        }
        printf("%d.Sıcaklık = %f \n",i,Tc);
        printf("--------\n");
    }
    data[0] = 0x12; //PORT A için
    for (int i = 0; i<=15; i++) {
        data[1] = dongu[i];
        i2c.write(0x21<<1,data,2,0);
        olcum7 = ain7.read();
        olcum8 = ain8.read();
        R2 = R1 * (1023.0/olcum3 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > 70) {
        //SHUTDOWN
        printf("SHUTDOWN");
        Shutdown = 1;
        }
        R2 = R1 * (1023.0/olcum4 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > 70) {
        //SHUTDOWN
        printf("SHUTDOWN");
        Shutdown = 1;
        }
        printf("%d.Sıcaklık = %f \n",i,Tc);
        printf("--------\n");
    }
    
    }
    if (MEASURE_MUX3 == ENABLED) 
    {
    io3.setDirrection(MCP23017_PORTB, 0x00); //PORT B output ayarlanması
    io3.setDirrection(MCP23017_PORTA, 0x00); //PORT A output ayarlanması
    char data[2];
    data[0] = 0x01;
    data[1] = 0x00;
    char dongu[16] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
    i2c.write(0x22<<1,data,2,0);
    data[0] = 0x13; //PORT B için 
    for (int i = 0; i<=15; i++) {
        data[1] = dongu[i];
        i2c.write(0x20<<1,data,2,0);
        olcum9 = ain9.read();
        olcum10 = ain10.read();
        R2 = R1 * (1023.0/olcum1 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > OverTemp) {
        //SHUTDOWN
        printf("shutdownnn");
        Shutdown = 1;
        }
        R2 = R1 * (1023.0/olcum2 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > OverTemp) {
        //SHUTDOWN
        printf("shutdownnn");
        Shutdown = 1;
        }
        printf("%d.Sıcaklık = %f \n",i,Tc);
        printf("--------\n");
    }
    data[0] = 0x12; //PORT A için
    for (int i = 0; i<=15; i++) {
        data[1] = dongu[i];
        i2c.write(0x22<<1,data,2,0);
        olcum11 = ain11.read();
        olcum12 = ain12.read();
        R2 = R1 * (1023.0/olcum3 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > 70) {
        //SHUTDOWN
        printf("SHUTDOWN");
        Shutdown = 1;
        }
        R2 = R1 * (1023.0/olcum4 -1.0);
        logR2 = log(R2);
        T = (1.0/(A+B*logR2+C*logR2*logR2*logR2));
        Tc = T-273.15;
        if (Tc > 70) {
        //SHUTDOWN
        printf("SHUTDOWN");
        Shutdown = 1;
        }
        printf("%d.Sıcaklık = %f \n",i,Tc);
        printf("--------\n");
    }
    }

    if (MEASURE_STAT == ENABLED)
    {
    wakeup_idle(slave_sayi);
    LTC681x_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
    LTC681x_pollAdc();
    wakeup_idle(slave_sayi);
    hata = LTC681x_rdstat(0, slave_sayi, cell_a);
    check_error(hata);
    printf("measure stat OK!!");
    }

}

// main() runs in its own thread in the OS
int main()
{  

    printf("basladim");
    

    /*
    spi.frequency(1000000); //Frekans değiştirilebilir Fast 1000KHz'ye göre ayarlandı
    spi.format(11); // Commandler datasheet'ten kontrol edildi 11 bit gittiği için 11 bit ayarlandı 60.sayfa
    */
    while (true) {
      
        olcum();
       
        }
        }