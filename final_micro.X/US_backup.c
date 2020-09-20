/******ULTRASONIC SENSOR DEFINES******/
#define Trig LATBbits.LB3
#define Echo PORTBbits.RB4
#define Trigio TRISBbits.RB3
#define Echoio TRISBbits.RB4
#define T0start 16706

/******ULTRASONIC SENSOR PROTOTYPES******/
void US_init(void);
void US_send_pulse(void);
void US_print_distance(unsigned int dist);
volatile unsigned int distance=0;
void itoa4D(char arr[], unsigned int num);

/******ULTRASONIC FUNCTIONS******/
//Maximum distance is 4m, so sound will travel for 23ms,TMR0 will be set to last 2.5s
void US_init(void)
{
    RBIE=0;
    RBIF=0;
    Trig=0;
    Trigio=0;
    Echoio=1;
    TMR0ON=0;   //Not counting
    T08BIT=0;   //16 bit accuracy
    T0SE=1;     //Increment on LOW-TO-HIGH transition
    PSA=0;      //Uses prescaler
    T0PS2=1;
    T0PS1=1;
    T0PS0=1;
    T0CS=0;
    TMR0IF=0;
    RBIF=0;
    GIE=1;
    PEIE=1;
    TMR0IE=1;
    INT0IE=1;
    TMR0IF=0;
    TMR0=T0start;   //***IMPORTANT
    return;
}

void US_send_pulse(void)
{
    LCD_command(0x01);
    LCD_string_xy(1,1,"Triggering...");
    Trig=1;
    __delay_ms(1000);
    __delay_ms(1000);
    __delay_ms(1000);
    __delay_ms(1000);
    Trig=0;
    LCD_string_xy(2,1,"Triggered");
    while(!Echo);
    TMR0ON=1;
    LCD_command(0x01);
    LCD_string_xy(1,1,"Echo ON");
    return;
}

void US_print_distance(unsigned int dist)
{
    char dist4D[5];
    itoa4D(dist4D,dist);
    LCD_command(0x01);
    LCD_string_xy(1,1,"Distance (mm):");
    LCD_string_xy(2,6,dist4D);
    return;
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
        }
        if(arr[2]=='0')arr[2]=' ';
    }
    return;
}

/*void __interrupt(high_priority) IOC_ISR(void)
{
    GIE=0;
    LCD_clear();
    LCD_string("RBINT");
    __delay_ms(1000);
    __delay_ms(1000);
    __delay_ms(1000);
    __delay_ms(1000);
    LATB=PORTB;
    if(!RBIF){GIE=1;return;}
    if(!Echo)
    {
        TMR0ON=0;
        distance=(TMR0-T0start)*4/_XTAL_FREQ*343000;
        US_print_distance(distance);
        __delay_ms(1000);
        __delay_ms(1000);
        __delay_ms(1000);
        __delay_ms(1000);
        __delay_ms(1000);
        TMR0=0;
    }
    RBIF=0;GIE=1;return;
}*/

//La distancia maxima del US es 4 m, por ende la ida y vuelta del sonido no puede superar los 23ms, el TMR0 no superará los 25 ms
void __interrupt(low_priority) TMR0_ISR(void)
{
    LCD_clear();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    TMR0ON=0;
    LCD_string("TMR0INT");
    __delay_ms(1000);
    if(!TMR0IF){return;}
    GIE=0;
    distance=TMR0;
    US_print_distance(distance);
    __delay_ms(1000);
    __delay_ms(1000);
    TMR0ON=0;
    TMR0=0;
    TMR0IF=0;
    GIE=1;
    TMR0ON=1;
    return;
}
