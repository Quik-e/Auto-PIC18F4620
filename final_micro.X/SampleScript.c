/*
 * File:   main.c
 * Author: kike
 *
 * Created on January 10, 2019, 5:19 PM
 */

// PIC18LF4520 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator,if I write HSPLL PLL enabled (Clock Frequency = 4 x FOSC1),INTIO67 internal OSC with RA6 and RA7 as IO pins)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (PORTC=CCP2 input/output is multiplexed with RC1 - PORTBE=CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF
#include <xc.h>

//#define _XTAL_FREQ 250000 //250kHz - Internal
#define _XTAL_FREQ 20000000 //20MHz - External

/******LCD DEFINES******/
#define RS LATBbits.LB0     //PIN 0 of PORTB is assigned for register select Pin of LCD
#define EN LATBbits.LB1     //PIN 1 of PORTB is assigned for enable Pin of LCD 
#define LATdata LATB        //PORTB(PB2-PB5) is assigned for LCD Data Output 
#define LCD_TRIS TRISB      //Define macros for PORTB Direction Register

/******ULTRASONIC SENSOR DEFINES******/
#define Trig LATDbits.LD0
#define Echo PORTDbits.RD1
#define Trigio TRISDbits.RD0
#define Echoio TRISDbits.RD1
#define T0start 26472       //Overflow en 2.5ms con PS=32

/******LIGHTS DEFINES******/
#define Front LATCbits.LC3
#define RearL LATCbits.LC4
#define RearR LATCbits.LC5
#define Frontio TRISCbits.RC3
#define RearLio TRISCbits.RC4
#define RearRio TRISCbits.RC5

/******MOTORS DEFINES******/
#define LENio TRISCbits.RC2     //Left motor Enable IO
#define RENio TRISCbits.RC1     //Right motor Enable IO
#define LFio TRISDbits.RD4      //Left motor Foward direction IO
#define LBio TRISDbits.RD5      //Left motor Backward direction IO
#define RFio TRISDbits.RD6      //Right motor Foward direction IO
#define RBio TRISDbits.RD7      //Right motor backward direction IO
#define LEN LATCbits.LC2        //Left motor Enable (CCP1)
#define REN LATCbits.LC1        //Right motor Enable (CCP2)
#define LF LATDbits.LD4         //Left motor Foward direction
#define LB LATDbits.LD5         //Left motor Backward direction
#define RF LATDbits.LD6         //Right motor Foward direction
#define RB LATDbits.LD7         //Right motor backward direction

/******BLUETOOTH DEFINES******/
#define Txio TRISCbits.RC6      //Transmitter IO
#define Rxio TRISCbits.RC7      //Receiver IO
#define State PORTCbits.RC5     //Bluetooth State:
#define Stateio TRISCbits.RC5     //Bluetooth State IO
#define BaudRate 9600

/******LCD PROTOTYPES******/
void LCD_init(void);                            //Initialize LCD
void LCD_hcommand(unsigned char cmd );           //Send half command to LCD
void LCD_command(unsigned char );               //Send command to LCD
void LCD_char(unsigned char dat);               //Send data to LCD
void LCD_string(const char *);                  //Display data string on LCD
void LCD_string_xy(char, char , const char *);  //Lines, Column, String
void LCD_clear(void);                            //Clear LCD Screen
volatile unsigned char nibble=0x00;

/******ULTRASONIC SENSOR PROTOTYPES******/
void US_init(void);
void US_send_pulse(void);
void US_print_distance(unsigned int dist);
volatile unsigned int distance=0;
volatile unsigned int ParkingDistance=45;

/******LIGHTS PROTOTYPES******/
void L_init(void);
void L_turn_on(void);
void L_turn_off(void);

/******MOTORS PROTOTYPES******/
void M_init(void);
void M_foward_direction(void);
void M_backward_direction(void);
void M_clockwise_direction(void);
void M_anticlockwise_direction(void);
void M_stop(void);
void M_set_DC(unsigned int DutyCycle, unsigned char motor);

/******BLUETOOTH PROTOTYPES******/ //It uses the USART module to communicate via bluetooth with another device
void BT_init(void);
void BT_tx_char(unsigned char Tx);
void BT_tx_string(const unsigned char *);
unsigned char BT_rx_char(void);
void BT_rx_string(unsigned char RxStr[]);

/******EXTRAS******/
void IOSC_init(void);
void itoa4D(char arr[], unsigned int num);
unsigned int atoi4D(char arr[]);

void main(void)
{
    BT_init();
    LCD_init();
    L_init();
    BT_tx_char('\n');
    LCD_string_xy(1,1,"LCD connected");
    BT_tx_string("LCD connected\n");
    __delay_ms(1000);
    M_init();
    LCD_string_xy(2,1,"Motor connected");
    BT_tx_string("Motor connected\n");
    US_init();
    LCD_string_xy(1,1,"US connected");
    BT_tx_string("US connected\n");
    __delay_ms(1000);
    LCD_command(0x01);
    while(1);
    return;
}

/******LCD FUNCTIONS******/
void LCD_init()
{
    LATdata=0x00;
    LCD_TRIS=0x00;
    __delay_ms(50);
    LCD_hcommand(0x30);
    LCD_command(0x28);  //Function set: 4 bit, 5x8, 2 lines
    LCD_command(0x28);  //Function set: 4 bit, 5x8, 2 lines
    LCD_command(0x0C);  //Display ON, Cursor OFF, Blink OFF
    LCD_command(0x01);  //Display clear
    LCD_command(0x06);  //Increasing cursor, Shift OFF
    LCD_command(0x02);  //Returns cursor to first cell, unshifts display
    return;
}

void LCD_hcommand(unsigned char cmd )
{
    nibble=cmd;
    LATdata = (0xF0 & nibble)>>2;   //Send higher nibble of command first to PORT 
    RS = 0;                      //Command Register is selected i.e.RS=0 
    NOP();
    EN = 1;                      //High-to-low pulse on Enable pin to latch data 
    NOP();
    EN = 0;
    __delay_ms(3);
    return;
}

void LCD_command(unsigned char cmd )
{
    nibble=cmd;
    LATdata = (0xF0 & nibble)>>2;   //Send higher nibble of command first to PORT 
    RS = 0;                      //Command Register is selected i.e.RS=0 
    NOP();
    EN = 1;                      //High-to-low pulse on Enable pin to latch data 
    NOP();
    EN = 0;
    __delay_ms(1);
    nibble=cmd;
    LATdata = (nibble<<4)>>2;     //Send lower nibble of command to PORT 
    RS=0;
    NOP();
    EN = 1;
    NOP();
    EN = 0;
    __delay_ms(3);
    return;
}

void LCD_char(unsigned char dat)
{
    nibble=dat;
    LATdata = (0xF0 & nibble)>>2;   //Send higher nibble of data first to PORT
    RS = 1;                         //Data Register is selected
    NOP();
    EN = 1;                         //High-to-low pulse on Enable pin to latch data
    NOP();
    EN = 0;
    __delay_ms(1);
    nibble=dat;
    LATdata = (nibble<<4)>>2;       //Send lower nibble of data to PORT
    RS=1;
    NOP();
    EN = 1;                         //High-to-low pulse on Enable pin to latch data
    NOP();
    EN = 0;
    __delay_ms(2);
    return;
}

void LCD_string(const char *msg)
{
    unsigned char i=0;
    while((*msg)!=0)
    {		
        LCD_char(*msg);
        msg++;
        i++;
        if(i==16) LCD_command(0xC0);
    }
    return;
}

void LCD_string_xy(char row,char pos,const char *msg)
{
    char location=0;
    if(row<=1)
    {
        location=((0x80) | ((pos) & 0x0f))-1;      //Print message on 1st row and desired location
        LCD_command(location);
    }
    else
    {
        location=((0xC0) | ((pos) & 0x0f))-1;      //Print message on 2nd row and desired location
        LCD_command(location);    
    }  
    LCD_string(msg);
    return;
}

void LCD_clear()
{
    LCD_command(0x01);     //Clear display screen
    return;
}

/******ULTRASONIC FUNCTIONS******/
//Maximum distance is 4m, so sound will travel for 23ms,TMR0 will be set to last 2.5s
//Minimum distance is 2cm

void US_init(void)
{
    RBIE=1;
    RBIF=0;
    Trig=0;
    Trigio=0;
    Echoio=1;
    TMR0ON=0;   //Not counting
    T08BIT=0;   //16 bit accuracy
    T0SE=1;     //Increment on LOW-TO-HIGH transition
    PSA=0;      //Uses prescaler
    T0PS2=1;    //Prescaler=256
    T0PS1=0;
    T0PS0=0;
    T0CS=0;
    TMR0IF=0;
    RBIF=0;
    GIE=1;
    PEIE=1;
    TMR0IE=1;
    INT0IE=0;
    TMR0IF=0;
    TMR0=T0start;   //***IMPORTANT
    return;
}

void US_send_pulse(void)
{
    char dist[5],i=0;
    distance=0;
    Trig=1;
    __delay_us(12);
    Trig=0;
    while(!Echo);
    TMR0ON=1;
    while(Echo);
    TMR0ON=0;
    distance+=(100000000/_XTAL_FREQ)*(TMR0-T0start)*343*4*32/(100000*2); //mm
    TMR0=T0start;
    __delay_us(15);
    distance/=1;
    US_print_distance(distance);
    itoa4D(dist,distance);
    __delay_ms(20);
    return;
}

void US_print_distance(unsigned int dist)
{
    char dist4D[5];
    itoa4D(dist4D,dist);
    LCD_command(0x01);
    LCD_string_xy(1,1,"Distance (mm):");
    LCD_string_xy(2,7,dist4D);
    BT_tx_string("Distance (mm):");    
    BT_tx_string(dist4D);
    BT_tx_char('\n');
    return;
}

/******LIGHTS FUNCTIONS******/
void L_init(void)
{
    Frontio=0;
    RearLio=0;
    RearRio=0;
    return;
}

void L_turn_on(void)
{
    Front=1;
    RearL=1;
    RearR=1;
    return;
}

void L_turn_off(void)
{
    Front=0;
    RearL=0;
    RearR=0;
    return;
}

/******MOTORS FUNCTIONS******/
void M_init(void) //PAGE 161 PIC DATASHEET
{
    LF=0;LB=0;RF=1;RB=0; //Sets direction of the motors 
    LENio=1;RENio=1;LFio=0;LBio=0;RFio=0;RBio=0; //Sets all PWM pins as INPUT and ENABLE pins as OUTPUT 
    PR2=0xFF;
    CCP1M3=1;
    CCP1M2=1;
    CCP1M1=0;
    CCP1M0=0;
    CCP2M3=1;
    CCP2M2=1;
    CCP2M1=0;
    CCP2M0=0;
    CCPR1L=0xFF;    //Duty Cycle Assignment
    DC1B1=1;        //Duty Cycle is not assigned until a match between PR2 and TMR2 (when a period is completed)
    DC1B0=1;        //Duty cycle= 10bits*Tosc*TMR2Prescaler (DUTY CYCLE CAN'T BE GREATER THAN THE PWM PERIOD) - About 0.01% each step with this configuration
    CCPR2L=0xFF;    //Duty Cycle Assignment
    DC2B1=1;        
    DC2B0=1;        
    TMR2IF=0;
    T2CKPS1=1;      //Prescaller=16 - PWM Period=[(PR2+1)]*4*Tosc*TMR2Prescaler=819.2 us - PWM Frequency=1220.7 Hz
    T2CKPS0=1;      //PWM doesn't use Postscaler
    TMR2ON=1;
    while(!TMR2IF); //Wait so that the Duty Cycle is assigned
    LENio=0;RENio=0;//Modulated pins are activated and deactivated through the TRISCON
    M_stop();
 }

void M_foward_direction(void)
{
    if(CCPR1L!=CCPR2L)
    {
        CCPR2L=CCPR1L;
        DC2B1=DC1B1;
        DC2B0=DC1B0;
    }
    LF=1;RF=1;LB=0;RB=0;
}

void M_backward_direction(void)
{
    if(CCPR1L!=CCPR2L)
    {
        CCPR2L=CCPR1L;
        DC2B1=DC1B1;
        DC2B0=DC1B0;
    }
    LF=0;RF=0;LB=1;RB=1;
}

void M_clockwise_direction(void)        //Seen from above
{
    if(CCPR1L!=CCPR2L)
    {
        CCPR2L=CCPR1L;
        DC2B1=DC1B1;
        DC2B0=DC1B0;
    }
    LF=1;RF=0;LB=0;RB=1;
}

void M_anticlockwise_direction(void)    //Seen from above
{
    if(CCPR1L!=CCPR2L)
    {
        CCPR2L=CCPR1L;
        DC2B1=DC1B1;
        DC2B0=DC1B0;
    }
    LF=0;RF=1;LB=1;RB=0;
}

void M_stop(void)
{
    LF=0;RF=0;LB=0;RB=0;
    if(CCPR1L!=CCPR2L)
    {
        CCPR2L=CCPR1L;
        DC2B1=DC1B1;
        DC2B0=DC1B0;
    }
}

void M_set_DC(unsigned int DutyCycle, unsigned char motor) //1:CCP1 - 2:CCP2 - 3:CCP1&CCP2, Assuming PR2=0xFF
{
    if(motor<1||motor>3||DutyCycle<=0||DutyCycle>1023)return;
    switch(motor)
    {
        case 1:
            CCPR1L=DutyCycle>>2;
            DC1B1=(DutyCycle&2)>>1;        
            DC1B0=DutyCycle&1;
            break;
        case 2:
            CCPR2L=DutyCycle>>2;
            DC2B1=(DutyCycle&2)>>1;        
            DC2B0=DutyCycle&1;
            break;
        case 3:
            CCPR1L=DutyCycle>>2;
            DC1B1=(DutyCycle&2)>>1;        
            DC1B0=DutyCycle&1;
            CCPR2L=DutyCycle>>2;
            DC2B1=(DutyCycle&2)>>1;        
            DC2B0=DutyCycle&1;
            break;
    }
}

/******BLUETOOTH FUNCTIONS******/
void BT_init(void)
{
    Txio=0;
    Rxio=1;
    //TXSTA REGISTER
    TX9=0;      //8 bit transmission
    TXEN=1;     //Enable transmission
    SYNC=0;     //Asychronous mode
    SENDB=0;    //Send brake transmission completed
    BRGH=1;     //High BAUDRate
    //RCSTA REGISTER
    SPEN=1;     //Serial port enabled (Tx and Rx)
    RX9=0;
    CREN=1;     //Enables receiver
    ADDEN=1;    //1 = Enables address detection, enables interrupt and loads the receive buffer when RSR<8> is set 0 = Disables address detection, all bytes are received and ninth bit can be used as parity bit
    //BAUDCON REGISTER
    BRG16=0;    //Baud rate accuracy (1=16 bit,0=8bit)
    WUE=0;      //Wake up Enable Bit
    ABDEN=0;    //Auto Baud detect Enable Bit
    //SPBRG=(unsigned char)((( (float) (_XTAL_FREQ) / (float) (64*BaudRate)) - 1)); //SPBRG = (Fosc /(64*9600))-1 - Due to it's configuration, if you change SYNC, BRGH or BRG16 check DATASHEET
    SPBRG=130;
    PEIE=1;
    RCIE=1;
    GIE=1;
}

void BT_tx_char(unsigned char Tx)
{
    while(!TXIF);
    TXREG=Tx;
}

void BT_tx_string(const unsigned char *msg)
{
    while((*msg)!=0)
    {		
        BT_tx_char(*msg);
	msg++;	
    }
    return;
}

void BT_rx_string(unsigned char RxStr[])
{
    unsigned char i=0;
    do
    {
        RxStr[i]=BT_rx_char();
        i++;
    }while(RxStr[i-1]!='\0'&&RxStr[i-1]!='.'&&i<31);
    RxStr[i]='\0';
    return;
}

unsigned char BT_rx_char(void)
{
    while(!RCIF);
    if(OERR)
    {
        CREN=0;
        NOP();
        CREN=1;
    }
    return RCREG;
}

void __interrupt(low_priority) Rx_ISR()
{
    RCIE=0;
    unsigned char RxMessage[32]={},DuCyT[5]={};
    unsigned char motor=0;
    unsigned long int DuCy=0;
    BT_rx_string(RxMessage);
    switch(RxMessage[0])
    {
        case 'P':
            BT_tx_string("Parking\n");
            M_foward_direction();
            M_set_DC(60,3);
            while(distance!=ParkingDistance)
            {
                if(RCIF)
                {
                    BT_rx_string(RxMessage);
                    if(RxMessage[0]=='S')
                    {
                        BT_tx_string("Parking stopped\n");
                        break;
                    }
                }
                if(ParkingDistance>distance)M_foward_direction();
                US_send_pulse();
                if(distance<=ParkingDistance)break;
            }
            M_stop();
            LCD_clear();
            if(distance<=ParkingDistance)
            {
                LCD_string_xy(1,7,"DONE");
                BT_tx_string("DONE!!\n");
            }
            break;
        case 'F':
            M_foward_direction();
            break;
        case 'B':
            M_backward_direction();
            break;
        case 'S':
            M_stop();
            break;
        case 'D':
            US_send_pulse();
            break;
        case 'T':
            M_stop();
            BT_tx_string("Write what you want to see on the LCD\n");
            BT_rx_string(RxMessage);
            LCD_clear();
            LCD_string_xy(1,1,RxMessage);
            break;
        case 'I':
            Reset();
            break;
        case 'C':
            M_clockwise_direction();
            break;
        case 'A':
            M_anticlockwise_direction();
            break;
        case '1':
            M_set_DC(102,3);    //10%
            break;
        case '2':
            M_set_DC(205,3);    //20%
            break;
        case '3':
            M_set_DC(307,3);    //30%
            break;
        case '4':
            M_set_DC(409,3);    //40%
            break;
        case '5':
            M_set_DC(512,3);    //50%
            break;
        case '6':
            M_set_DC(614,3);    //60%
            break;
        case '7':
            M_set_DC(716,3);    //70%
            break;
        case '8':
            M_set_DC(818,3);    //80%
            break;
        case '9':
            M_set_DC(921,3);    //90%
            break;
        case '0':
            M_set_DC(1023,3);   //100%
            break;
        case 'W':
            M_stop();
            M_foward_direction();
            M_set_DC(512,1);
            M_set_DC(1023,2);
            break;
        case 'X':
            M_stop();
            M_foward_direction();
            M_set_DC(1023,1);
            M_set_DC(512,2);
            break;
        case 'Y':
            M_stop();
            M_backward_direction();
            M_set_DC(512,1);
            M_set_DC(1023,2);
            break;
        case 'Z':
            M_stop();
            M_backward_direction();
            M_set_DC(1023,1);
            M_set_DC(512,2);
            break;
        case 'L':
            if(!(Front&&RearL&&RearR))L_turn_on();
            else L_turn_off();
            break;
        case 'd':
            M_stop();
            BT_tx_string("Which motor do you want to set DC? Motor 1 (1), Motor 2 (2) or both (3)\n");
            BT_rx_string(RxMessage);
            if(RxMessage[0]=='1'||RxMessage[0]=='2'||RxMessage[0]=='3')
            {
                motor=RxMessage[0]&0x0F;
                BT_tx_string("Please enter DC value (%)\n");
                BT_rx_string(RxMessage);
                DuCy=atoi4D(RxMessage);
                DuCy=DuCy*1023/100;
                M_set_DC(DuCy,motor);
                itoa4D(DuCyT,DuCy);
                BT_tx_string(DuCyT);
            }
            else BT_tx_string("Wrong Answer!!\n");
            break;
    }
    RCIE=1;
}

/******EXTRAS******/
void IOSC_init(void)
{
    //OSCCON REGISTER
    IDLEN=1;    //Device enters IDLE mode on SLEEP instruction
    IRCF2=0;    //IOSC Frequency=250kHz
    IRCF1=1;
    IRCF0=0;
    while(!IOFS);//Internal Oscillator Frequency Stable bit (1 STABLE - 0 UNSTABLE)
}

void itoa4D(char arr[], unsigned int num)
{
    arr[0]=((num/1000)%10)|0x30;
    arr[1]=((num/100)%10)|0x30;;
    arr[2]=((num/10)%10)|0x30;
    arr[3]=(num%10)|0x30;
    arr[4]='\0';
    if(arr[0]=='0')
    {
        arr[0]=' ';
        if(arr[1]=='0')
        {
            arr[1]=' ';
            if(arr[2]=='0')arr[2]=' ';
        }
    }
    return;
}

unsigned int atoi4D(char arr[])
{
    unsigned int num=0;
    num=(arr[0]&0x0F)*1000;
    num+=(arr[1]&0x0F)*100;
    num+=(arr[2]&0x0F)*10;
    num+=arr[3]&0x0F;
    return num;
}


