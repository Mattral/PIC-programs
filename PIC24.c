/* 
 * File:   PIC24.c
 * Author: Utente
 *
 * Created on 6 marzo 2013, 15.36
 */

#include <string.h>
#include <stdlib.h>
#include <xc.h>

#pragma config BWRP = OFF, SOSCSRC = DIG, IESO = OFF
#pragma config POSCMOD = HS, OSCIOFNC = OFF, POSCFREQ = MS, FWDTEN = OFF
#pragma config BOREN = BOR3, LVRCFG = ON, PWRTEN = ON, BORV = V27
#pragma config MCLRE = ON, ICS = PGx1
#pragma config FNOSC = PRI


#define led LATBbits.LATB9
#define data_add_0_w LATAbits.LATA7
#define data_add_1_w LATCbits.LATC9
#define data_add_2_w LATCbits.LATC8
#define data_add_3_w LATAbits.LATA10
#define data_add_4_w LATBbits.LATB2
#define data_add_5_w LATBbits.LATB3
#define data_add_6_w LATCbits.LATC0
#define data_add_7_w LATCbits.LATC1
#define data_add_0_r PORTAbits.RA7
#define data_add_1_r PORTCbits.RC9
#define data_add_2_r PORTCbits.RC8
#define data_add_3_r PORTAbits.RA10
#define data_add_4_r PORTBbits.RB2
#define data_add_5_r PORTBbits.RB3
#define data_add_6_r PORTCbits.RC0
#define data_add_7_r PORTCbits.RC1
#define data_en LATBbits.LATB5
#define add_latch LATBbits.LATB6
#define sw0 PORTCbits.RC5
#define sw1 PORTCbits.RC4
#define sw2 PORTCbits.RC3
#define sw3 PORTAbits.RA9
#define sw4 PORTAbits.RA4
#define p0 PORTAbits.RA1
#define p1 PORTCbits.RC2
#define p2 PORTAbits.RA8
#define p3 PORTBbits.RB4

//display command
#define rs_dpy LATAbits.LATA7
#define en_dpy LATBbits.LATB3
#define db4_dpy LATBbits.LATB2
#define db5_dpy LATAbits.LATA10
#define db6_dpy LATCbits.LATC8
#define db7_dpy LATCbits.LATC9

#define rdp 120




typedef union{              //structured variable
     struct {
        unsigned a0:1;
        unsigned a1:1;
        unsigned a2:1;
        unsigned a3:1;
        unsigned a4:1;
        unsigned a5:1;
        unsigned a6:1;
        unsigned a7:1;
        unsigned a8:1;
        unsigned a9:1;
        unsigned a10:1;
        unsigned a11:1;
        unsigned a12:1;
        unsigned a13:1;
        unsigned a14:1;
        unsigned a15:1;
    } one;
    int whole;
}composed;

typedef union{
     struct {
        unsigned a0:1;
        unsigned a1:1;
        unsigned a2:1;
        unsigned a3:1;
        unsigned a4:1;
        unsigned a5:1;
        unsigned a6:1;
        unsigned a7:1;
        unsigned a8:1;
        unsigned a9:1;
        unsigned a10:1;
        unsigned a11:1;
        unsigned a12:1;
        unsigned a13:1;
        unsigned a14:1;
        unsigned a15:1;
    } one;
    struct{
        char byte_0;
        char byte_1;
    }two;
    unsigned int whole;
}composed_1;

struct{                //for memory module only
        unsigned a0:1; //quiet state
        unsigned a1:1; //set address and data
        unsigned a2:1; //read eeprom
        unsigned a3:1; //write eeprom
        unsigned a4:1; //read s-ram
        unsigned a5:1; //write s_ram
        unsigned a6:1;
        unsigned a7:1;
}mem_0;

struct{                //analog module selector
        unsigned a0:1;
        unsigned a1:1;
        unsigned a2:1;
        unsigned a3:1;
        unsigned a4:1;
        unsigned a5:1;
        unsigned a6:1;
        unsigned a7:1;
}analog_sel;

struct{                     //for auxiliary operations
        unsigned old_p0:1;  //for read P0
        unsigned exe:1;     //for read/write memory executed
        unsigned old_p1:1;  //for read P1
        unsigned old_p2:1;  //for read P2
        unsigned old_p3:1;  //for read P3
        unsigned dac_1_time:1;      //for DAC module, for comand disply only first time
        unsigned data_add6r:1;      //for read data_add_6_r
        unsigned data_add7r:1;      //for read data_add_7_r
}aux_0;

struct{                //auxiliary for opto module
        unsigned first:1;
        unsigned old_data_add_0_r:1;
        unsigned old_data_add_1_r:1;
        unsigned old_data_add_4_r:1;
        unsigned change:1;
        unsigned a5:1;
        unsigned a6:1;
        unsigned a7:1;
}opto_0;

struct{                //auxiliary for opto module
        unsigned first:1;
        unsigned spi_in:1;
        unsigned spi_out:1;
        unsigned uart_tx:1;
        unsigned uart_rx:1;
        unsigned ex_rx:1;
        unsigned refresh:1;
        unsigned a6:1;
}spi_uart_0;

struct{                //auxiliary for pwm passo passo module
        unsigned first:1;
        unsigned pwm:1;
        unsigned stepper:1;
        unsigned one_time:1;
        unsigned pwm_on:1;      //for pwm modulation
        unsigned pwm_off:1;
        unsigned forward:1;
        unsigned back:1;
        unsigned stepper_change_phase:1;
        unsigned stepper_0:1;
        unsigned stepper_1:1;
        unsigned stepper_2:1;
        unsigned stepper_3:1;
}pwm_pp;

char st_x_dpy[16], st_ad[16], st_mem[16], old_st_x_dpy[16], st_spi_uart[16];

int ad_conv_buf, mem_data, add_data, spi_buf, uart_tx, uart_rx, pwm_on, pwm_off, stepper_time, dummy;
double mem_address_long;
composed letter_address, letter, sw_selector, old_sw_selector, aux_1;
composed_1 spi_buffer, mem_address;


void initialize(void);
void disable_module(void);
void enable_module(void);
void selection_module(short int);
void write_display(short int);
void write_letter_add(void);
void write_letter_char(char);
void set_bus_input(void);
void set_bus_output(void);
void display_init(void);
void update_switch_button(void);
void vis_add_data(void);

void __delay_ms(int);
void __delay_us(int);
void __attribute__((__interrupt__)) _T2Interrupt(void);

//**********************************************************************************************
 
int main(int argc, char** argv) {

        __delay_ms(500);
        initialize();

        strcpy(st_x_dpy, "DELORENZO spa   "); //first message on display
        write_display(1);
        strcpy(st_x_dpy, "BRS PIC24FV32   "); //first message on display
        write_display(2);
        __delay_ms(100);
        __delay_ms(100);
        __delay_ms(100);
        __delay_ms(100);
        __delay_ms(100);
        __delay_ms(100);
        __delay_ms(100);
        __delay_ms(100);
        __delay_ms(100);
        __delay_ms(100);


     while(1){
        //sw selection


        sw_selector.one.a0 = sw0;
        sw_selector.one.a1 = sw1;
        sw_selector.one.a2 = sw2;
        sw_selector.one.a3 = sw3;
        sw_selector.one.a4 = sw4;

        if(sw_selector.whole != old_sw_selector.whole){
            disable_module();
            mem_0.a0 = 1; //for memory module only
            aux_0.dac_1_time = 1;
            opto_0.first = 1;
            spi_uart_0.first = 1;
            spi_uart_0.refresh = 1;
            pwm_pp.first = 1;
            IEC0bits.T2IE = 0; //timer 2 interrupt off
            T2CONbits.TON = 0; //timer 2 off
        }
        old_sw_selector.whole = sw_selector.whole;

//************************I/O MODULE input******************************************************

        if(sw_selector.whole == 0){     //I/O module (input)

          strcpy(st_x_dpy, "I/O module (in) "); //first message on display
          write_display(1);
          selection_module(0);          //i/o mod (input) selected
          set_bus_input();
          enable_module();
          strcpy(st_x_dpy, "                "); //first message on display
          if(data_add_0_r){
              st_x_dpy[0] = '1';
          }else{
              st_x_dpy[0] = '0';
          }
          if(data_add_1_r){
              st_x_dpy[1] = '1';
          }else{
              st_x_dpy[1] = '0';
          }
          if(data_add_2_r){
              st_x_dpy[2] = '1';
          }else{
              st_x_dpy[2] = '0';
          }
          if(data_add_3_r){
              st_x_dpy[3] = '1';
          }else{
              st_x_dpy[3] = '0';
          }
          if(data_add_4_r){
              st_x_dpy[4] = '1';
          }else{
              st_x_dpy[4] = '0';
          }
          if(data_add_5_r){
              st_x_dpy[5] = '1';
          }else{
              st_x_dpy[5] = '0';
          }
          if(data_add_6_r){
              st_x_dpy[6] = '1';
          }else{
              st_x_dpy[6] = '0';
          }
          if(data_add_7_r){
              st_x_dpy[7] = '1';
          }else{
              st_x_dpy[7] = '0';
          }
          disable_module();

          write_display(2);

        }

//********************I/O MODULE output******************************************************
        if(sw_selector.whole == 1){     //I/O module (output)
          strcpy(st_x_dpy, "I/O module (out)"); //first message on display
          write_display(1);
          strcpy(st_x_dpy, "push P0-P1-P2-P3");
          write_display(2);
          selection_module(1);          //i/o mod (output) selected
          set_bus_output();
          enable_module();

          ANSBbits.ANSB4 = 0;

          TRISBbits.TRISB4 = 1;



          if(p0){
              data_add_0_w = 1;
              data_add_1_w = 1;
          }else{
              data_add_0_w = 0;
              data_add_1_w = 0;
          }
          if(p1){
              data_add_2_w = 1;
              data_add_3_w = 1;
          }else{
              data_add_2_w = 0;
              data_add_3_w = 0;
          }
           if(p2){
              data_add_4_w = 1;
              data_add_5_w = 1;
          }else{
              data_add_4_w = 0;
              data_add_5_w = 0;
          }
          if(p3){
              data_add_6_w = 1;
              data_add_7_w = 1;
          }else{
              data_add_6_w = 0;
              data_add_7_w = 0;
          }

          disable_module();

        }


     



//***************************ANALOGIC MODULE        **************************************************
        if(sw_selector.whole == 2){     //ANALOGIC module
          strcpy(st_x_dpy, "A/D module      "); //first message on display
          write_display(1);
          if(!analog_sel.a0&&!analog_sel.a1&&!analog_sel.a2&&!analog_sel.a3){ //if nothing selected
            strcpy(st_x_dpy, "select P0-P1...."); //first message on display
            write_display(2);
          }
          selection_module(2);
          set_bus_output();
          enable_module();


          if(p0&&!aux_0.old_p0){                   //analog selector
             update_switch_button();
             analog_sel.a0 = 1;
             analog_sel.a1 = 0;
             analog_sel.a2 = 0;
             analog_sel.a3 = 0;
          }
          if(p1&&!aux_0.old_p1){
             update_switch_button();
             analog_sel.a0 = 0;
             analog_sel.a1 = 1;
             analog_sel.a2 = 0;
             analog_sel.a3 = 0;
          }
          if(p2&&!aux_0.old_p2){
             update_switch_button();
             analog_sel.a0 = 0;
             analog_sel.a1 = 0;
             analog_sel.a2 = 1;
             analog_sel.a3 = 0;
          }
          if(p3&&!aux_0.old_p3){
             update_switch_button();
             analog_sel.a0 = 0;
             analog_sel.a1 = 0;
             analog_sel.a2 = 0;
             analog_sel.a3 = 1;
          }
          update_switch_button();



          if(analog_sel.a0){              //A/D SPI throw MCP3201
            __delay_ms(197);
            __delay_ms(197);
            data_add_0_w = 1;  //MCP3201 in shut down
            //A/D converter with SPI
            //set up
            SPI1CON1bits.SMP = 0; //sample in the middle of data out
            SPI1CON1bits.CKE = 1; //data transmitted on falling edge of ck
            SPI1CON1bits.CKP = 0; //in idle state CK is low
            //primary and secondary prescaler as by reset (always 0)
            SPI1CON1bits.MSTEN = 1; //master mode
            SPI1STATbits.SPIEN = 1; //enable SCK, SDI, ecc. as SPI port

            //read throw SPI of MCP3201
            data_add_0_w = 0; //MCP3201 "on"
            __delay_us(100);
            SPI1BUF = 0b10101010;  //dummy transmission (first)
            while(!SPI1STATbits.SPIRBF){

            }
            spi_buffer.whole = 0;
            spi_buffer.two.byte_1 = SPI1BUF;
            SPI1BUF = 0b10101010;  //dummy transmission (second)
            while(!SPI1STATbits.SPIRBF){

            }
            data_add_0_w = 1;
            spi_buffer.two.byte_0 = SPI1BUF;

            SPI1STATbits.SPIEN = 0;

            spi_buffer.whole = spi_buffer.whole/2; //see MCP3201 datasheet fig. 6-1 pag 15
            itoa(st_ad, spi_buffer.whole,10);  //convertion of integer in char string
           

            st_x_dpy[0] = 'S';
            st_x_dpy[1] = 'P';
            st_x_dpy[2] = 'I';
            st_x_dpy[3] = ' ';
            st_x_dpy[4] = 'A';
            st_x_dpy[5] = '/';
            st_x_dpy[6] = 'D';
            st_x_dpy[7] = ' ';
            st_x_dpy[8] = 'v';
            st_x_dpy[9] = 'a';
            st_x_dpy[10] = 'l';
            st_x_dpy[11] = ':';

          
            st_x_dpy[12] = st_ad[0];
            st_x_dpy[13] = st_ad[1];
            st_x_dpy[14] = st_ad[2];
            st_x_dpy[15] = st_ad[3];
            if(spi_buffer.whole < 10){
               st_x_dpy[13] = ' ';
               st_x_dpy[14] = ' ';
               st_x_dpy[15] = ' ';
            }
            if(spi_buffer.whole < 100){
               st_x_dpy[14] = ' ';
               st_x_dpy[15] = ' ';
            }
            if(spi_buffer.whole < 1000){
               st_x_dpy[15] = ' ';
            }
            write_display(2);
          }

          if(analog_sel.a1){  //MCP9700 temp. sensor

              ANSBbits.ANSB15 = 1; //input as analog
              TRISBbits.TRISB15 = 1;

              AD1CHSbits.CH0SA4 = 0;    //select AN9 channel
              AD1CHSbits.CH0SA3 = 1;
              AD1CHSbits.CH0SA2 = 0;
              AD1CHSbits.CH0SA1 = 0;
              AD1CHSbits.CH0SA0 = 1;

              AD1CON1bits.ADON = 1;  //module on

              AD1CON1bits.SAMP = 1;  //start sample
              __delay_us(100);
              AD1CON1bits.SAMP = 0;  //start conversion

              while(!AD1CON1bits.DONE){

              }

              ad_conv_buf = (ADC1BUF0/2) - 50;  //convert MCP9700 output in volt
             

              itoa(st_ad, ad_conv_buf, 10);  //convertion of integer in char string

              st_x_dpy[0] = 't';
              st_x_dpy[1] = 'e';
              st_x_dpy[2] = 'm';
              st_x_dpy[3] = 'p';
              st_x_dpy[4] = 'e';
              st_x_dpy[5] = 'r';
              st_x_dpy[6] = 'a';
              st_x_dpy[7] = 't';
              st_x_dpy[8] = 'u';
              st_x_dpy[9] = 'r';
              st_x_dpy[10] = 'e';
              st_x_dpy[11] = '^';
              st_x_dpy[12] = 'c';
              st_x_dpy[13] = ' ';

              st_x_dpy[14] = st_ad[0];
              st_x_dpy[15] = st_ad[1];

              write_display(2);


          }

          if(analog_sel.a2){      //VPOT

              ANSBbits.ANSB12 = 1; //input as analog
              TRISBbits.TRISB12 = 1;

              AD1CHSbits.CH0SA4 = 0;    //select AN12 channel
              AD1CHSbits.CH0SA3 = 1;
              AD1CHSbits.CH0SA2 = 1;
              AD1CHSbits.CH0SA1 = 0;
              AD1CHSbits.CH0SA0 = 0;

             
              AD1CON1bits.ADON = 1;  //module on

              AD1CON1bits.SAMP = 1;  //start sample
              __delay_us(100);
              AD1CON1bits.SAMP = 0;  //start conversion
       
              while(!AD1CON1bits.DONE){

              }

              ad_conv_buf = (ADC1BUF0*5)/102;  //10 bit a/d to voltage conversion
             

              itoa(st_ad, ad_conv_buf, 10);  //convertion of integer in char string
        
              st_x_dpy[0] = 'L';
              st_x_dpy[1] = 'i';
              st_x_dpy[2] = 'g';
              st_x_dpy[3] = 'h';
              st_x_dpy[4] = 't';
              st_x_dpy[5] = ' ';
              st_x_dpy[6] = 's';
              st_x_dpy[7] = 'e';
              st_x_dpy[8] = 'n';
              st_x_dpy[9] = 's';
              st_x_dpy[10] = 'o';
              st_x_dpy[11] = 'r';
              st_x_dpy[12] = ':';
            
              st_x_dpy[13] = st_ad[0];
              st_x_dpy[14] = '.';
              st_x_dpy[15] = st_ad[1];
              if(ad_conv_buf < 10){
                  st_x_dpy[13] = '0';
                  st_x_dpy[15] = st_ad[0];
              }
              write_display(2);

          }

          
          if(analog_sel.a3){      //PTC

              ANSBbits.ANSB14 = 1; //input as analog
              TRISBbits.TRISB14 = 1;

              AD1CHSbits.CH0SA4 = 0;    //select AN10 channel
              AD1CHSbits.CH0SA3 = 1;
              AD1CHSbits.CH0SA2 = 0;
              AD1CHSbits.CH0SA1 = 1;
              AD1CHSbits.CH0SA0 = 0;


              AD1CON1bits.ADON = 1;  //module on

              AD1CON1bits.SAMP = 1;  //start sample
              __delay_us(100);
              AD1CON1bits.SAMP = 0;  //start conversion

              while(!AD1CON1bits.DONE){

              }

              ad_conv_buf = ADC1BUF0/2;  //to convert in Volt x100
           

              itoa(st_ad, ad_conv_buf, 10);  //convertion of integer in char string

         
              st_x_dpy[0] = 'P';
              st_x_dpy[1] = 'T';
              st_x_dpy[2] = 'C';
              st_x_dpy[3] = ' ';
              st_x_dpy[4] = 'V';
              st_x_dpy[5] = 'O';
              st_x_dpy[6] = 'L';
              st_x_dpy[7] = 'T';
              st_x_dpy[8] = ' ';
              st_x_dpy[9] = ' ';
              st_x_dpy[10] = ' ';
              st_x_dpy[11] = ' ';
        
              st_x_dpy[12] = st_ad[0];
              st_x_dpy[13] = '.';
              st_x_dpy[14] = st_ad[1];
              st_x_dpy[15] = st_ad[2];

              if(ad_conv_buf < 100){
                st_x_dpy[12] = '0';
                st_x_dpy[14] = st_ad[0];
                st_x_dpy[15] = st_ad[1];
              }

              write_display(2);


          }

          disable_module();

        }


     




//*************************DAC MODULE******************************************************
        if(sw_selector.whole == 3){     //DAC module
          if(aux_0.dac_1_time){         //for a correct read of BC3 point voltage is necessary to keep
                                        //always on the abilitation module
            strcpy(st_x_dpy, "D/A module      "); //first message on display
            write_display(1);
            selection_module(8);
            set_bus_output();
            aux_0.dac_1_time = 0;
            data_add_1_w = 1;  //CS high
            data_add_0_w = 1;  //LDAC high

             //set up
            SPI1CON1bits.SMP = 0; //sample in the middle of data out
            SPI1CON1bits.CKE = 1; //data transmitted on falling edge of ck
            SPI1CON1bits.CKP = 0; //in idle state CK is low
            //primary and secondary prescaler as by reset (always 0)
            SPI1CON1bits.MSTEN = 1; //master mode
            SPI1STATbits.SPIEN = 1; //enable SCK, SDI, ecc. as SPI port

            enable_module();
          }

          //read data from trimmer

          ANSAbits.ANSA0 = 1; //input as analog
          TRISAbits.TRISA0 = 1;

          AD1CHSbits.CH0SA4 = 0;    //select AN0 channel
          AD1CHSbits.CH0SA3 = 0;
          AD1CHSbits.CH0SA2 = 0;
          AD1CHSbits.CH0SA1 = 0;
          AD1CHSbits.CH0SA0 = 0;


          AD1CON1bits.ADON = 1;  //module on

          __delay_ms(194);  //delay to charge capacitors on RB4 pin

          AD1CON1bits.SAMP = 1;  //start sample
          __delay_us(100);
          AD1CON1bits.SAMP = 0;  //start conversion

          while(!AD1CON1bits.DONE){

          }

          //voltage on display
          ad_conv_buf = ( ADC1BUF0*5)/102;  //10 bit a/d to voltage conversion
          itoa(st_ad, ad_conv_buf, 10);  //convertion of integer in char string
          st_x_dpy[0] = 'T';
          st_x_dpy[1] = 'r';
          st_x_dpy[2] = 'i';
          st_x_dpy[3] = 'm';
          st_x_dpy[4] = 'm';
          st_x_dpy[5] = 'e';
          st_x_dpy[6] = 'r';
          st_x_dpy[7] = ' ';
          st_x_dpy[8] = 'v';
          st_x_dpy[9] = 'o';
          st_x_dpy[10] = 'l';
          st_x_dpy[11] = 't';
          st_x_dpy[12] = ':';
          st_x_dpy[13] = st_ad[0];
          st_x_dpy[14] = '.';
          st_x_dpy[15] = st_ad[1];
          if(ad_conv_buf < 10){
                st_x_dpy[13] = '0';
                st_x_dpy[15] = st_ad[0];
          }

          disable_module();
          write_display(2);
          selection_module(8);
          set_bus_output();
          data_add_1_w = 1;  //CS high
          data_add_0_w = 1;  //LDAC high
          enable_module();
          //voltage on display

          spi_buffer.whole = ADC1BUF0*97/20;  //for convert 10 bit a/d to 12 bit DAC
 //the conversion facto 49/50 is for: convert the read from adc PIC 10 bit in 12 bit dac,
//convert the maximum output of dac (4.096V, setted with bit 13 to zero) to 5V;
//the value is: 4*5/4.096= 4.88, so 97/20

          spi_buffer.one.a13 = 0;        //output gain *2;
          spi_buffer.one.a12 = 1;        //SHDN "on"         

          //send data to DAC MCP4821
          data_add_1_w = 0;  //CS low
       
          __delay_us(100);

          SPI1BUF = spi_buffer.two.byte_1;   //first byte transmission
          while(!SPI1STATbits.SPIRBF){

          }
          dummy = SPI1BUF;
        

          SPI1BUF = spi_buffer.two.byte_0;   //second byte transmission
          while(!SPI1STATbits.SPIRBF){

          }
          dummy = SPI1BUF;
    
          __delay_us(10);
          data_add_1_w = 1;

          __delay_us(10);
          data_add_0_w = 0;
          __delay_us(10);
          data_add_0_w = 1;

        

        }

     


//************************MEMORY MODULE*****************************************************
        if(sw_selector.whole == 4){     //MEMORY module

            if(mem_0.a0){
                strcpy(st_x_dpy, "EEPROM & S-RAM  "); //first message on display
                write_display(1);
                strcpy(st_x_dpy, "push P0 to start"); //first message on display
                write_display(2);

                 //set up spi

                 SPI1CON1bits.SMP = 0; //sample in the middle of data out
                 SPI1CON1bits.CKE = 1; //data transmitted on falling edge of ck
                 SPI1CON1bits.CKP = 0; //in idle state CK is low
                 //primary and secondary prescaler as by reset (always 0)
                 SPI1CON1bits.MSTEN = 1; //master mode
                 SPI1STATbits.SPIEN = 1; //enable SCK, SDI, ecc. as SPI port

                 mem_0.a1 = 0;
                 mem_0.a2 = 0;
                 mem_0.a3 = 0;
                 mem_0.a4 = 0;
                 mem_0.a5 = 0;
                if(mem_0.a0&&p0&&!aux_0.old_p0){
                    mem_0.a0 = 0;
                    mem_0.a1 = 1;
                }
                aux_0.old_p0 = p0;
            }

            if(mem_0.a1){               //set address and data
                                        //po = inc, p1 = exe, p3 clear
                selection_module(7);  //to read setup
                set_bus_input();
                enable_module();

                if(!data_add_5_r){        //set address
                     if(p3){
                        mem_address_long = 0;
                     }
                     if(p0&&!aux_0.old_p0){
                         if(data_add_4_r){
                            mem_address_long++;
                         }
                         if(data_add_3_r){
                            mem_address_long = mem_address_long + 10;
                         }
                         if(data_add_2_r){
                            mem_address_long = mem_address_long + 100;
                         }
                         if(data_add_1_r){
                            mem_address_long = mem_address_long + 1000;
                         }
                         if(data_add_0_r){
                            mem_address_long = mem_address_long + 10000;
                         }
                     }
                     aux_0.old_p0 = p0;

                     if(!data_add_7_r){  //if eeprom
                         if(mem_address_long > 255){
                             mem_address.whole = 255;
                         }else{
                             mem_address.whole = mem_address_long;
                         }
                     }
                     if(data_add_7_r){  //if eeprom
                         if(mem_address_long > 32767){
                             mem_address.whole = 32767;
                         }else{
                             mem_address.whole = mem_address_long;
                         }
                     }
                }

                 if(data_add_5_r&&data_add_6_r){        //set data (only if write)
                     if(p3){
                        mem_data = 0;
                     }
                     if(p0&&!aux_0.old_p0){
                         if(data_add_4_r){
                            mem_data++;
                         }
                         if(data_add_3_r){
                            mem_data = mem_data + 10;
                         }
                         if(data_add_2_r){
                            mem_data = mem_data + 100;
                         }
                         if(data_add_1_r){
                            mem_data = mem_data + 1000;
                         }
                         if(data_add_0_r){
                            mem_data = mem_data + 10000;
                         }
                     }
                     aux_0.old_p0 = p0;

                     if(mem_data > 255){
                         mem_data = 255;
                     }

                 }


                if(p1&&!aux_0.old_p1){                 
                    mem_0.a1 = 0;
                    aux_0.exe = 0;
                    if(!data_add_6_r&&!data_add_7_r){   //read eeprom
                        mem_0.a2 = 1;
                    }
                    if(data_add_6_r&&!data_add_7_r){   //write eeprom
                        mem_0.a3 = 1;
                    }
                    if(!data_add_6_r&&data_add_7_r){   //read s-ram
                        mem_0.a4 = 1;
                    }
                    if(data_add_6_r&&data_add_7_r){   //write s-ram
                        mem_0.a5 = 1;
                    }
                }
                aux_0.old_p1 = p1;  //update of p1



                aux_0.data_add6r = data_add_6_r;   //auxiliary before change to display write (bus change)
                aux_0.data_add7r = data_add_7_r;
                if(aux_0.data_add6r){              //if write
                  itoa(st_mem, mem_data, 10);  //convertion of integer in char string
                  if(aux_0.data_add7r){
                    strcpy(st_x_dpy, "RAM data        ");
                  }else{
                    strcpy(st_x_dpy, "EE data         ");
                  }
                  add_data = mem_data;
                  vis_add_data();
                }else{
                     strcpy(st_x_dpy, "                ");
                }
                write_display(2);

                itoa(st_mem, mem_address.whole, 10);  //convertion of integer in char string
                if(aux_0.data_add7r){
                    strcpy(st_x_dpy, "RAM address     ");
                }else{
                    strcpy(st_x_dpy, "EE address      ");
                }
                add_data = mem_address.whole;
                vis_add_data();
                write_display(1);

            }

            if(mem_0.a2){        //read eeprom (p2 back)
                if(!aux_0.exe){
                    selection_module(6);
                    set_bus_output();
                    data_add_1_w = 1;  //deselect s-ram
                    data_add_0_w = 0;  //select eeprom
                    enable_module();

                    __delay_us(100);

                    SPI1BUF = 0b00000011;   //instruction trasmission (read)
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = mem_address.whole;   //address trasmission
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = 0b01010101;   //dummy trasmission for read
                    while(!SPI1STATbits.SPIRBF){

                    }
                    __delay_us(100);
                    data_add_0_w = 1;

                    aux_0.exe = 1;
                }

                if(p2&&!aux_0.old_p2){      //back
                    aux_0.exe = 0;
                    mem_0.a1 = 1;
                    mem_0.a2 = 0;
                    disable_module();
                }
                aux_0.old_p2 = p2;  //update of p1

                itoa(st_mem, SPI1BUF, 10);  //convertion of integer in char string
                strcpy(st_x_dpy, "EE data         ");
                add_data = SPI1BUF;
                vis_add_data();
                write_display(2);

            }
            if(mem_0.a3){           //write eeprom (p2 back)
                 if(!aux_0.exe){
                    data_add_1_w = 1;  //deselect s-ram
                    data_add_0_w = 0;  //select eeprom
                    selection_module(6);
                    set_bus_output();
                    enable_module();

                    __delay_us(100);
                    SPI1BUF = 0b00000110;   //write enable (WREN)
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    __delay_us(100);
                    data_add_0_w = 1;
                    __delay_us(100);
                    data_add_0_w = 0;
                    __delay_us(100);
                    SPI1BUF = 0b00000010;   //byte write sequence
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = mem_address.whole;   //address trasmission
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = mem_data;      //data trasmission
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    __delay_us(100);
                    data_add_0_w = 1;
                    aux_0.exe = 1;
                }

                if(p2&&!aux_0.old_p2){      //back
                    aux_0.exe = 0;
                    mem_0.a1 = 1;
                    mem_0.a3 = 0;
                    disable_module();
                }
                aux_0.old_p2 = p2;  //update of p2

                strcpy(st_x_dpy, "EE write done!! ");
                write_display(1);
                strcpy(st_x_dpy, "                ");
                write_display(2);


            }
            if(mem_0.a4){           //read s-ram (p2 back)
                 if(!aux_0.exe){
                    selection_module(6);
                    set_bus_output();
                    data_add_0_w = 1;  //deselect eeprom
                    data_add_1_w = 0;  //select s-ram
                    enable_module();

                    __delay_us(100);
                    SPI1BUF = 0b00000011;   //instruction trasmission (read)
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = 0;             //first byte address address trasmission
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = mem_address.two.byte_0; //second byte address address trasmission
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = mem_address.two.byte_1; //third byte address address trasmission
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = 0b01010101;   //dummy trasmission for read
                    while(!SPI1STATbits.SPIRBF){

                    }

                    __delay_us(100);
                    data_add_1_w = 1;

                    aux_0.exe = 1;
                }


                 if(p2&&!aux_0.old_p2){      //back
                    aux_0.exe = 0;
                    mem_0.a1 = 1;
                    mem_0.a4 = 0;
                    disable_module();
                }
                aux_0.old_p2 = p2;  //update of p2

                itoa(st_mem, SPI1BUF, 10);  //convertion of integer in char string
                strcpy(st_x_dpy, "RAM data        ");
                add_data = SPI1BUF;
                vis_add_data();
                write_display(2);

            }
            if(mem_0.a5){           //write s-ram
                 if(!aux_0.exe){
                    selection_module(6);
                    set_bus_output();
                    data_add_0_w = 1;  //deselect eeprom
                    data_add_1_w = 0;  //select s-ram
                    enable_module();

                    __delay_us(100);
                    SPI1BUF = 0b00000010;   //instruction trasmission (write)
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = 0;             //first byte address address trasmission
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = mem_address.two.byte_0; //second byte address address trasmission
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = mem_address.two.byte_1; //third byte address address trasmission
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    SPI1BUF = mem_data;
                    while(!SPI1STATbits.SPIRBF){

                    }
                    dummy = SPI1BUF;

                    __delay_us(100);
                    data_add_1_w = 1;  //select s-ram

                    aux_0.exe = 1;
                }

                if(p2&&!aux_0.old_p2){      //back
                    aux_0.exe = 0;
                    mem_0.a1 = 1;
                    mem_0.a5 = 0;
                    disable_module();
                }
                aux_0.old_p2 = p2;  //update of p2

                strcpy(st_x_dpy, "RAM write done!!");
                write_display(1);
                strcpy(st_x_dpy, "                ");
                write_display(2);
            }


        }
     



        if(sw_selector.whole == 5){     //OPTO module

          if(opto_0.first){
            opto_0.first = 0;
            strcpy(st_x_dpy, "OPTO module     "); //first message on display
            write_display(1);
            strcpy(st_x_dpy, "IN1/IN2=00 INF=0");
            write_display(2);
            selection_module(5);
            TRISAbits.TRISA7 = 1; //data_add_0 as input
            TRISCbits.TRISC9 = 1; //data_add_1 as input
            data_add_2_w = 0;
            data_add_3_w = 0;
            TRISCbits.TRISC8 = 0; //data_add_2 as output
            TRISAbits.TRISA10 = 0; //data_add_3 as output
            TRISBbits.TRISB2 = 1; //data_add_4 as input
            enable_module();
          }

          if(p0){
              data_add_2_w = 1;
          }else{
              data_add_2_w = 0;
          }
          if(p1){
              data_add_3_w = 1;
          }else{
              data_add_3_w = 0;
          }


           if(data_add_1_r!=opto_0.old_data_add_1_r){
              opto_0.old_data_add_1_r = data_add_1_r;
              if(data_add_1_r){
                st_x_dpy[8] = '1';
              }else{
                st_x_dpy[8] = '0';
              }
              opto_0.change = 1;
          }
        
         
          
          if(data_add_0_r!=opto_0.old_data_add_0_r){
              opto_0.old_data_add_0_r = data_add_0_r;
              if(data_add_0_r){
                st_x_dpy[9] = '1';
              }else{
                st_x_dpy[9] = '0';
              }
              opto_0.change = 1;
          }
         
          

          if(data_add_4_r!=opto_0.old_data_add_4_r){
              opto_0.old_data_add_4_r = data_add_4_r;
              if(data_add_4_r){
                st_x_dpy[15] = '1';
              }else{
                st_x_dpy[15] = '0';
              }
              opto_0.change = 1;
          }


          if(opto_0.change){

            opto_0.change = 0;

            st_x_dpy[0] = 'I';
            st_x_dpy[1] = 'N';
            st_x_dpy[2] = '1';
            st_x_dpy[3] = '/';
            st_x_dpy[4] = 'I';
            st_x_dpy[5] = 'N';
            st_x_dpy[6] = '2';
            st_x_dpy[7] = '=';


            st_x_dpy[10] = ' ';
            st_x_dpy[11] = 'I';
            st_x_dpy[12] = 'N';
            st_x_dpy[13] = 'F';
            st_x_dpy[14] = '=';

            write_display(2);

            selection_module(5);
            TRISAbits.TRISA7 = 1; //data_add_0 as input
            TRISCbits.TRISC9 = 1; //data_add_1 as input
            data_add_2_w = 0;
            data_add_3_w = 0;
            TRISCbits.TRISC8 = 0; //data_add_2 as output
            TRISAbits.TRISA10 = 0; //data_add_3 as output
            TRISBbits.TRISB2 = 1; //data_add_4 as input
            enable_module();

          }



        }


     


        if(sw_selector.whole == 6){     //SPI UART


           if(spi_uart_0.first){

                spi_uart_0.spi_in = 0;
                spi_uart_0.spi_out = 0;
                spi_uart_0.uart_tx = 0;
                spi_uart_0.uart_rx = 0;
                spi_buf = 0;
                uart_tx = 48;  //ASCII = 0;
                uart_rx = 48;

                //set up spi
                SPI1CON1bits.SMP = 0; //sample in the middle of data out
                SPI1CON1bits.CKE = 1; //data transmitted on falling edge of ck
                SPI1CON1bits.CKP = 0; //in idle state CK is low
                //primary and secondary prescaler as by reset (always 0)
                SPI1CON1bits.MSTEN = 1; //master mode
                SPI1STATbits.SPIEN = 1; //enable SCK, SDI, ecc. as SPI port


                if(spi_uart_0.refresh){
                    spi_uart_0.refresh = 0;
                    strcpy(st_x_dpy, "SPI-UART module "); //first message on display
                    write_display(1);
                    strcpy(st_x_dpy, "0/1=SPI 2/3=UART"); //first message on display
                    write_display(2);
                }

            if(p0&&!aux_0.old_p0){
                spi_uart_0.first = 0;
                spi_uart_0.spi_in = 1;
                spi_uart_0.spi_out = 0;
                spi_uart_0.uart_tx = 0;
                spi_uart_0.uart_rx = 0;
            }
            aux_0.old_p0 = p0;

            if(p1&&!aux_0.old_p1){
                spi_uart_0.first = 0;
                spi_uart_0.spi_in = 0;
                spi_uart_0.spi_out = 1;
                spi_uart_0.uart_tx = 0;
                spi_uart_0.uart_rx = 0;
             }
             aux_0.old_p1 = p1;

             if(p2&&!aux_0.old_p2){
               spi_uart_0.first = 0;
               spi_uart_0.spi_in = 0;
               spi_uart_0.spi_out = 0;
               spi_uart_0.uart_tx = 1;
               spi_uart_0.uart_rx = 0;
             }
             aux_0.old_p2 = p2;

             if(p3&&!aux_0.old_p3){
               spi_uart_0.first = 0;
               spi_uart_0.spi_in = 0;
               spi_uart_0.spi_out = 0;
               spi_uart_0.uart_tx = 0;
               spi_uart_0.uart_rx = 1;
               strcpy(st_x_dpy, "UART RX:        ");
               write_display(2);
             }
             aux_0.old_p3 = p3;

           }


           if(spi_uart_0.spi_in){  //p3= back

               selection_module(3);
               data_add_0_w = 0;
               data_add_1_w = 1;
               TRISAbits.TRISA7 = 0;
               TRISCbits.TRISC9 = 0;
               enable_module();
               __delay_us(100);
               data_add_1_w = 0;  //input load in 74hc166
               __delay_us(100);

               SPI1BUF = 0b10101010;  //dummy cycle to load shift register
               while(!SPI1STATbits.SPIRBF){

               }
               dummy = SPI1BUF;
               __delay_us(100);
               data_add_1_w = 1;
               __delay_us(100);



               SPI1BUF = 0b10101010;  //read date
               while(!SPI1STATbits.SPIRBF){

               }
               __delay_us(100);
               data_add_1_w = 0;  //

               spi_buffer.two.byte_0 = SPI1BUF;
               strcpy(st_x_dpy, "SPI IN=          ");
               if(spi_buffer.one.a0){
                   st_x_dpy[8] = '1';
               }else{
                   st_x_dpy[8] = '0';
               }
               if(spi_buffer.one.a1){
                   st_x_dpy[9] = '1';
               }else{
                   st_x_dpy[9] = '0';
               }
               if(spi_buffer.one.a2){
                   st_x_dpy[10] = '1';
               }else{
                   st_x_dpy[10] = '0';
               }
               if(spi_buffer.one.a3){
                   st_x_dpy[11] = '1';
               }else{
                   st_x_dpy[11] = '0';
               }
                if(spi_buffer.one.a4){
                   st_x_dpy[12] = '1';
               }else{
                   st_x_dpy[12] = '0';
               }
               if(spi_buffer.one.a5){
                   st_x_dpy[13] = '1';
               }else{
                   st_x_dpy[13] = '0';
               }
               if(spi_buffer.one.a6){
                   st_x_dpy[14] = '1';
               }else{
                   st_x_dpy[14] = '0';
               }
               if(spi_buffer.one.a7){
                   st_x_dpy[15] = '1';
               }else{
                   st_x_dpy[15] = '0';
               }

               if(p3&&!aux_0.old_p3){
                   spi_uart_0.first = 1;
                   spi_uart_0.refresh = 1;
               }
               aux_0.old_p3 = p3;

               write_display(2);
           }

           if(spi_uart_0.spi_out){          //po= inc, p1= dec, p3=back

              selection_module(3);
              data_add_0_w = 0;
              data_add_1_w = 1;
              TRISAbits.TRISA7 = 0;
              TRISCbits.TRISC9 = 0;
              enable_module();

              if(p0&&!aux_0.old_p0&&(spi_buf < 254)) {
                  spi_buf++;
              }
              aux_0.old_p0 = p0;

              if(p1&&!aux_0.old_p1&&(spi_buf > 0)) {
                  spi_buf--;
              }
              aux_0.old_p1 = p1;

              if(p3&&!aux_0.old_p3){
                   spi_uart_0.first = 1;
                   spi_uart_0.refresh = 1;
              }
              aux_0.old_p3 = p3;

              SPI1BUF = spi_buf;
              while(!SPI1STATbits.SPIRBF){

              }
              dummy = SPI1BUF;

              __delay_us(100);
              data_add_0_w = 1;
              __delay_us(100);
              data_add_0_w = 0;

              strcpy(st_x_dpy, "SPI OUT =        ");

              itoa(st_spi_uart, spi_buf, 10);  //convertion of integer in char string

              if(spi_buf < 10){
                  st_x_dpy[11] = st_spi_uart[0];
              }else if(spi_buf < 100){
                  st_x_dpy[11] = st_spi_uart[0];
                  st_x_dpy[12] = st_spi_uart[1];
              }else{
                  st_x_dpy[11] = st_spi_uart[0];
                  st_x_dpy[12] = st_spi_uart[1];
                  st_x_dpy[13] = st_spi_uart[2];
              }

              write_display(2);

           }

//Hyperterminal configuration: 9600 bits/sec, 1 stop bits, no parity bit, no flow control,
// ASCII set up: "echo typed character locally
           if(spi_uart_0.uart_tx){  //p0 inc, p1 dec, p2 tx, p3 clear

               selection_module(3);
               data_add_0_w = 0;
               data_add_1_w = 1;
               TRISAbits.TRISA7 = 0;
               TRISCbits.TRISC9 = 0;
               enable_module();


               U1BRG = 12;         //setting 9600 baud
               U1MODEbits.UARTEN = 1;  //uart enable
               U1STAbits.UTXEN = 1;  //TX enable

               if(p0&&!aux_0.old_p0){
                   uart_tx++;
               }
               aux_0.old_p0 = p0;

               if(p1&&!aux_0.old_p1){
                   uart_tx--;
               }
               aux_0.old_p1 = p1;

               if(p3&&!aux_0.old_p3){
                   spi_uart_0.first = 1;
                   spi_uart_0.refresh = 1;
               }
               aux_0.old_p3 = p3;

               if(p2&&!aux_0.old_p2){
                  U1TXREG = uart_tx;
                  __delay_ms(100);
               }
               aux_0.old_p2 = p2;

               strcpy(st_x_dpy, "UART TX:        ");
               st_x_dpy[9] = (char)uart_tx;
               write_display(2);

           }


            if(spi_uart_0.uart_rx){  // p3 clear

                selection_module(3);
                enable_module();

                U1BRG = 12;         //setting 9600 baud
                U1MODEbits.UARTEN = 1;  //uart enable

                spi_uart_0.ex_rx = 0;
                while(!spi_uart_0.ex_rx){

                    if(p3&&!aux_0.old_p3){
                        spi_uart_0.ex_rx = 1;
                    }
                    aux_0.old_p3 = p3;

                    if(U1STAbits.URXDA){
                        uart_rx = U1RXREG;
                        strcpy(st_x_dpy, "UART RX:        ");
                        st_x_dpy[9] = (char)uart_rx;
                        write_display(2);

                        selection_module(3);
                        enable_module();

                    }
                }

                spi_uart_0.first = 1;
                spi_uart_0.refresh = 1;

            }




        }


     


        if(sw_selector.whole == 7){     //PWM stepper, p0 pwm p1 stepper

            if(pwm_pp.first){
                pwm_pp.pwm = 0;
                pwm_pp.stepper = 0;
                if(p0&&!aux_0.old_p0){
                    pwm_pp.first = 0;
                    pwm_pp.pwm = 1;
                    pwm_pp.one_time = 1;
                }
                aux_0.old_p0 = p0;
                if(p1&&!aux_0.old_p1){
                    pwm_pp.first = 0;
                    pwm_pp.stepper = 1;
                    pwm_pp.one_time = 1;
                }
                aux_0.old_p1 = p1;

                IEC0bits.T2IE = 0;
                T2CONbits.TON = 0;

                strcpy(st_x_dpy, "PWM & STEPPER   ");
                write_display(1);
                strcpy(st_x_dpy, "P0/1=PWM/stepper");
                write_display(2);

                data_add_0_w = 0;
                data_add_1_w = 0;
                data_add_2_w = 0;
                data_add_3_w = 0;
                data_add_4_w = 0;
                data_add_5_w = 0;
                data_add_6_w = 0;
                data_add_7_w = 0;
            }

            while(pwm_pp.pwm){             //p0 inc, p1 dec, p2 inv, p3 exit
                if(pwm_pp.one_time){
                    pwm_pp.one_time = 0;
                    strcpy(st_x_dpy, "P0=inc P1=dec   ");
                    write_display(1);
                    strcpy(st_x_dpy, "P2=inv P3=exit  ");
                    write_display(2);

                    //used timer 2 for pwm generation
                 
                    pwm_on = 50;
                    pwm_off = 205;

                    T2CONbits.TCKPS0 = 1; //prescaler 8  freq= 2000000/8= 250KHz
                    T2CONbits.TCKPS1 = 0;

                    PR2 = pwm_on;

                    IEC0bits.T2IE = 1;  //interrupt on
                    T2CONbits.TON = 1;

                    pwm_pp.pwm_on = 1;
                    pwm_pp.pwm_off = 0;
                    pwm_pp.forward = 1;
                    pwm_pp.back = 0;
                    selection_module(4);
                    set_bus_output();
                    enable_module();

                }

                if(pwm_pp.forward){  //forward rotation
                    data_add_3_w = 0;
                    data_add_0_w = 0;
                    data_add_1_w = 1;
                }
                if(pwm_pp.back){        //back rotation
                    data_add_2_w = 0;
                    data_add_0_w = 1;
                    data_add_1_w = 0;
                }



                if(pwm_pp.pwm_on&&pwm_pp.forward){  //forward pwm
                    data_add_2_w = 1;
                }
                if(pwm_pp.pwm_off&&pwm_pp.forward){
                    data_add_2_w = 0;
                }

                if(pwm_pp.pwm_on&&pwm_pp.back){  //back pwm
                    data_add_3_w = 1;
                }
                if(pwm_pp.pwm_off&&pwm_pp.back){
                    data_add_3_w = 0;
                }

                if(p0&&!aux_0.old_p0&&(pwm_on < 240)){  //inc pwm
                    pwm_on = pwm_on + 10;
                }
                aux_0.old_p0 = p0;

                if(p1&&!aux_0.old_p1&&(pwm_on > 30)){   //dec pwm
                    pwm_on = pwm_on - 10;
                }
                aux_0.old_p1 = p1;

                if(p2&&!aux_0.old_p2){                  //to invert rotation
                    if(pwm_pp.forward){
                        pwm_pp.forward = 0;
                        pwm_pp.back = 1;
                        pwm_on = 50;
                        pwm_off = 205;
                        data_add_2_w = 0;
                        data_add_1_w = 0;
                    }else{
                        pwm_pp.back = 0;
                        pwm_pp.forward = 1;
                        pwm_on = 50;
                        pwm_off = 205;
                        data_add_3_w = 0;
                        data_add_0_w = 0;
                    }
                    __delay_ms(100);
                    __delay_ms(100);
                    __delay_ms(100);
                    __delay_ms(100);
                    __delay_ms(100);
                }
                aux_0.old_p2 = p2;

                pwm_off = 255 - pwm_on;

                if(p3&&!aux_0.old_p3){          //exit
                    pwm_pp.pwm = 0;
                    pwm_pp.first = 1;
                }
                aux_0.old_p3 = p3;

            }

            if(pwm_pp.stepper){             //p0 inc speed, p1 dec speed, p3 exit
               if(pwm_pp.one_time){
                    pwm_pp.one_time = 0;
                    strcpy(st_x_dpy, "P0=inc P1=dec   ");
                    write_display(1);
                    strcpy(st_x_dpy, "P3=exit         ");
                    write_display(2);

                    //used timer 2 for pwm generation
                    //freq: FOSC/2 = 2MHz

                    T2CONbits.TCKPS0 = 1; //prescaler 256  freq= 2000000/256= 7800 Hz
                    T2CONbits.TCKPS1 = 1;
                   

                    stepper_time = 1000; //first time
                    PR2 = stepper_time;

                    IEC0bits.T2IE = 1;  //interrupt on
                    T2CONbits.TON = 1;

                    pwm_pp.stepper_0 = 1;
                    pwm_pp.stepper_1 = 0;
                    pwm_pp.stepper_2 = 0;
                    pwm_pp.stepper_3 = 0;

                    data_add_4_w = 1;
                    data_add_5_w = 0;
                    data_add_6_w = 0;
                    data_add_7_w = 0;

                    selection_module(4);
                    set_bus_output();
                    enable_module();
                }


               if(pwm_pp.stepper_0){
                   data_add_7_w = 0;
                   data_add_4_w = 1;
               }
               if(pwm_pp.stepper_1){
                   data_add_4_w = 0;
                   data_add_5_w = 1;
               }
               if(pwm_pp.stepper_2){
                   data_add_5_w = 0;
                   data_add_6_w = 1;
               }
               if(pwm_pp.stepper_3){
                   data_add_6_w = 0;
                   data_add_7_w = 1;
               }

               if(pwm_pp.stepper_0&&pwm_pp.stepper_change_phase){
                   pwm_pp.stepper_0 = 0;
                   pwm_pp.stepper_1 = 1;
                   pwm_pp.stepper_change_phase = 0;
               }
               if(pwm_pp.stepper_1&&pwm_pp.stepper_change_phase){
                   pwm_pp.stepper_1 = 0;
                   pwm_pp.stepper_2 = 1;
                   pwm_pp.stepper_change_phase = 0;
               }
                if(pwm_pp.stepper_2&&pwm_pp.stepper_change_phase){
                   pwm_pp.stepper_2 = 0;
                   pwm_pp.stepper_3 = 1;
                   pwm_pp.stepper_change_phase = 0;
               }
               if(pwm_pp.stepper_3&&pwm_pp.stepper_change_phase){
                   pwm_pp.stepper_3 = 0;
                   pwm_pp.stepper_0 = 1;
                   pwm_pp.stepper_change_phase = 0;
               }




                if(p0&&!aux_0.old_p0&&(stepper_time > 100)){  //inc speed
                    stepper_time = stepper_time - 100;
                }
                aux_0.old_p0 = p0;

                if(p1&&!aux_0.old_p1&&(stepper_time < 2000)){   //dec speed
                    stepper_time = stepper_time + 100;
                }
                aux_0.old_p1 = p1;


                if(p3&&!aux_0.old_p3){          //exit
                    pwm_pp.stepper_change_phase = 0;
                    pwm_pp.first = 1;
                }
                aux_0.old_p3 = p3;

            }
        }

    }

    return (EXIT_SUCCESS);
}

//***************************************************************************************************
//***************************************************************************************************
//**************************************************************************************************
 void __delay_ms(int time){  //delay in millisec
     T1CONbits.TCKPS0 = 0;  //prescaler /64, freq (4000000/2)/64 = 31250 Hz, PR1 = 31
     T1CONbits.TCKPS1 = 1;
     T1CONbits.TCS = 0;
     PR1 = 31;
     TMR1 = 0;

     T1CONbits.TON = 1;

     while(time != 0){
         while(!IFS0bits.T1IF){

         }
         time = time - 1;
         T1CONbits.TON = 0;
         TMR1 = 0;
         IFS0bits.T1IF = 0;
         T1CONbits.TON = 1;
     }       
    }


 void __delay_us(int time){  // microdelay, it isn't a calculated value

     while(time!=0){
         time = time - 1;
     }

    }


 void initialize(void){     //initialize function

   TRISBbits.TRISB9 = 0;  //for led

    //set data_en and add_latch as output
    data_en = 1;
    add_latch = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB6 = 0;


    ANSBbits.ANSB2 = 0;     //mixed analog I/O  as input
    ANSBbits.ANSB3 = 0;
    ANSCbits.ANSC0 = 0;
    ANSCbits.ANSC1 = 0;
    ANSAbits.ANSA1 = 0;
    ANSCbits.ANSC2 = 0;
    ANSBbits.ANSB4 = 0;

    TRISBbits.TRISB11 = 0;    //set CK and output SPI as output
    TRISBbits.TRISB13 = 0;

   
    //initialize display to 4-bits interface, 2 lines
    selection_module(9);
    set_bus_output();

    enable_module();

    aux_1.whole = 0b0011;       //8 bits interface, 2 line
    display_init();

    __delay_ms(100);

    aux_1.whole = 0b0011;       //8 bits interface, 2 line
    display_init();

    __delay_ms(5);

    aux_1.whole = 0b0011;       //8 bits interface, 2 line
    display_init();

     __delay_ms(100);

    aux_1.whole = 0b0010;       //4 bits interface, 2 line
    display_init();



    aux_1.whole = 0b0010;       //4 bits interface, 2 line
    display_init();
    aux_1.whole = 0b1000;
    display_init();

    aux_1.whole = 0b0000;       //dsp off
    display_init();
    aux_1.whole = 0b1000;
    display_init();

    aux_1.whole = 0b0000;       //clear
    display_init();
    aux_1.whole = 0b0001;
    display_init();

    aux_1.whole = 0b0000;       //entry mode set
    display_init();
    aux_1.whole = 0b0100;
    display_init();

    aux_1.whole = 0b0000;       //dsp on
    display_init();
    aux_1.whole = 0b1100;
    display_init();


    disable_module();
}

void enable_module(void){
    data_en = 0;
}

void disable_module(void){
   data_en = 1;
}


void selection_module(short int add){
    composed ind;

    data_en = 1;

    ind.whole = add;

    data_add_0_w = ind.one.a0;
    data_add_1_w = ind.one.a1;
    data_add_2_w = ind.one.a2;
    data_add_3_w = ind.one.a3;
    TRISAbits.TRISA7 = 0;     //pin micro for selection setup as output
    TRISCbits.TRISC9 = 0;
    TRISCbits.TRISC8 = 0;
    TRISAbits.TRISA10 = 0;

    asm("NOP");
    asm("NOP");
    add_latch = 1;  //address latched
    asm("NOP");
    asm("NOP");
    add_latch = 0;
    asm("NOP");
    asm("NOP");



}


void write_display(short int line){

    short int i;
    selection_module(9);
    set_bus_output();
    for (i = 0; i < 16; i++){
        db7_dpy = 1;            //always high to send address
        if(line == 1){          //if first line
            db6_dpy = 0;
        }
        if(line == 2){          //if second line
            db6_dpy = 1;
        }
        //letter address
        letter_address.whole = i;
        write_letter_add();
        //letter
        write_letter_char(st_x_dpy[i]);
        disable_module();
    }

}


void write_letter_add(void){

     en_dpy = 0;
     rs_dpy = 0;         //first four bits
     enable_module();
     __delay_us(rdp);
     en_dpy = 1;
     __delay_us(rdp);
     db5_dpy = 0;
     db4_dpy = 0;
     __delay_us(rdp);
     en_dpy = 0;

     __delay_us(rdp) ;    //second four bits
     en_dpy = 1;
     __delay_us(rdp);
     if(letter_address.one.a3){
         db7_dpy = 1;
     }else{
         db7_dpy = 0;
     }
     if(letter_address.one.a2){
         db6_dpy = 1;
     }else{
         db6_dpy = 0;
     }
     if(letter_address.one.a1){
         db5_dpy = 1;
     }else{
         db5_dpy = 0;
     }
     if(letter_address.one.a0){
         db4_dpy = 1;
     }else{
         db4_dpy = 0;
     }
     __delay_us(rdp);
     en_dpy = 0;
     __delay_us(rdp);
}

void write_letter_char(char ch){
    letter.whole = ch;

    __delay_us(rdp);
     rs_dpy = 1;         //first four bits
     __delay_us(rdp);
     en_dpy = 1;
     __delay_us(rdp);
     if(letter.one.a7){
         db7_dpy = 1;
     }else{
         db7_dpy = 0;
     }
     if(letter.one.a6){
         db6_dpy = 1;
     }else{
         db6_dpy = 0;
     }
     if(letter.one.a5){
         db5_dpy = 1;
     }else{
         db5_dpy = 0;
     }
     if(letter.one.a4){
         db4_dpy = 1;
     }else{
         db4_dpy = 0;
     }
     __delay_us(rdp);
     en_dpy = 0;

     __delay_us(rdp) ;    //second four bits
     en_dpy = 1;
     __delay_us(rdp);
     if(letter.one.a3){
         db7_dpy = 1;
     }else{
         db7_dpy = 0;
     }
     if(letter.one.a2){
         db6_dpy = 1;
     }else{
         db6_dpy = 0;
     }
     if(letter.one.a1){
         db5_dpy = 1;
     }else{
         db5_dpy = 0;
     }
     if(letter.one.a0){
         db4_dpy = 1;
     }else{
         db4_dpy = 0;
     }
     __delay_us(rdp);
     en_dpy = 0;
     rs_dpy = 0;
     __delay_us(rdp);

}

void set_bus_input(void){

    TRISAbits.TRISA7 = 1;
    TRISCbits.TRISC9 = 1;
    TRISCbits.TRISC8 = 1;
    TRISAbits.TRISA10 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;
    TRISCbits.TRISC0 = 1;
    TRISCbits.TRISC1 = 1;
}

void set_bus_output(void){
    data_add_0_w = 0;
    data_add_1_w = 0;
    data_add_2_w = 0;
    data_add_3_w = 0;
    data_add_4_w = 0;
    data_add_5_w = 0;
    data_add_6_w = 0;
    data_add_7_w = 0;
    rs_dpy = 0;
    en_dpy = 0;
    db4_dpy = 0;
    db5_dpy = 0;
    db6_dpy = 0;
    db7_dpy = 0;
    TRISAbits.TRISA7 = 0;
    TRISCbits.TRISC9 = 0;
    TRISCbits.TRISC8 = 0;
    TRISAbits.TRISA10 = 0;
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB3 = 0;
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
}


void display_init(void){

    __delay_us(rdp);
    rs_dpy = 0;         // four bits
    __delay_us(rdp);
    en_dpy = 1;
    __delay_us(rdp);
    if(aux_1.one.a3){
        db7_dpy = 1;
    }else{
        db7_dpy = 0;
    }
    if(aux_1.one.a2){
        db6_dpy = 1;
    }else{
        db6_dpy = 0;
    }
    if(aux_1.one.a1){
        db5_dpy = 1;
    }else{
        db5_dpy = 0;
    }
    if(aux_1.one.a0){
        db4_dpy = 1;
    }else{
        db4_dpy = 0;
    }

    __delay_us(rdp);
    en_dpy = 0;
    __delay_us(rdp);

}

void update_switch_button(void){
     aux_0.old_p0 = p0;  //update pushbutton front
     aux_0.old_p1 = p1;
     aux_0.old_p2 = p2;
     aux_0.old_p3 = p3;
}

void vis_add_data(void){
    if(add_data < 10){
        st_x_dpy[15] = st_mem[0];
    }else if(add_data < 100){
        st_x_dpy[14] = st_mem[0];
        st_x_dpy[15] = st_mem[1];
    }else if(add_data < 1000){
        st_x_dpy[13] = st_mem[0];
        st_x_dpy[14] = st_mem[1];
        st_x_dpy[15] = st_mem[2];
    }else if(add_data < 10000){
        st_x_dpy[12] = st_mem[0];
        st_x_dpy[13] = st_mem[1];
        st_x_dpy[14] = st_mem[2];
        st_x_dpy[15] = st_mem[3];
    }else{
        st_x_dpy[11] = st_mem[0];
        st_x_dpy[12] = st_mem[1];
        st_x_dpy[13] = st_mem[2];
        st_x_dpy[14] = st_mem[3];
        st_x_dpy[15] = st_mem[4];
    }
}


void __attribute__((__interrupt__)) _T2Interrupt(void){

        T2CONbits.TON = 0;
        TMR2 = 0;
        //for pwm
        if(pwm_pp.pwm){
            if(pwm_pp.pwm_on){
                pwm_pp.pwm_on = 0;
                pwm_pp.pwm_off = 1;
                PR2 = pwm_off;
            }else{
                pwm_pp.pwm_on = 1;
                pwm_pp.pwm_off = 0;
                PR2 = pwm_on;
            }
        }
        //for stepper

        if(pwm_pp.stepper){
            pwm_pp.stepper_change_phase = 1;
            PR2 = stepper_time;
        }
        IFS0bits.T2IF = 0;
        T2CONbits.TON = 1;
}