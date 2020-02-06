//atmega32u4 control
#include<avr/io.h>					//HEADER FILE FOR AVR INPUT OUTPUT
#include<compat/deprecated.h>		//HEADER FILE FOR FUNCTIONS LIKE SBI AND CBI				//HEADER FILE FOR DELAY
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>

//define easy preprosseors
#define TIMERSIZE 255 // size of timer
#define TIMER_CNT TCNT0 //current timer value
#define MIN_CUTOFF_TIMERVAL 215 //smallest possible time interval
#define VEL_COMPUTE_NUMERATOR 2142500 // caluculate this using the angle and freq of timer

#define LedDDR DDRB
#define LedPort PORTB
#define LedPin  4


#define PWMF OCR1A
#define PWML OCR1B
#define PWMB OCR1C
#define PWMR OCR3A

#define MUtoPWM 0.0625 //1250(TOP)*1e-3/20 .... so this * microseconds gives a pwm value
#define PWMmin 63
#define PWMmax 156

#define KP 1
#define KD 0.5
// define volatile variables
volatile uint8_t buff[8];
volatile int process_it;
int posF = 1000;
volatile unsigned int vrF = 900;
volatile unsigned int velF = 0;
volatile float vdF = 0;
volatile unsigned int vpF = 0;
volatile unsigned int stTimeF=0;
volatile unsigned int curntTimeF=0;
int posL = 1000;
volatile unsigned int vrL = 900;
volatile unsigned int velL = 0;
volatile float vdL = 0;
volatile unsigned int vpL = 0;
volatile unsigned int stTimeL=0;
volatile unsigned int curntTimeL=0;
int posB = 1000;
volatile unsigned int vrB = 900;
volatile unsigned int velB = 0;
volatile float vdB = 0;
volatile unsigned int vpB = 0;
volatile unsigned int stTimeB=0;
volatile unsigned int curntTimeB=0;
int posR = 1000;
volatile unsigned int vrR = 900;
volatile unsigned int velR = 0;
volatile float vdR = 0;
volatile unsigned int vpR = 0;
volatile unsigned int stTimeR=0;
volatile unsigned int curntTimeR=0;
char buf[10];
int temp=0;

void Toggle_led()
{
	if (bit_is_clear(LedPort, LedPin))		//BLINK LED3 ON TIMER INTERRUPT
			sbi(LedPort,LedPin);				//LED3 ON
		else							//ELSE
			cbi(LedPort,LedPin);
}

int main (void)
{
 // Serial.begin (115200);   // debugging

  // have to send on master in, *slave out*
 sbi(DDRB,3);

  // turn on SPI in slave mode
  SPCR =0b11001000;

  // turn on the input interrupts and set them low

  // turn the led for debug
  sbi(LedDDR,LedPin);
  cbi(LedPort,LedPin);



  //turn on the vel timer (using 8 bit counter)
  TCCR0A=0;
  TCCR0B=0b00000011;
  TIMSK0=0x01;

  // setup four pwms(2 16 bit timers) FAST PWM
  TCCR1A=0b10101010;
  TCCR3A=0b10000010;
  TCCR1B=0b00011100;
  TCCR3B=0b00011100;
  ICR1=1249;
  ICR3=1249;
  sbi(DDRB,5);
  sbi(DDRB,6);
  sbi(DDRB,7);
  sbi(DDRC,6);

  // get ready for an interrupt
  process_it = 0;

  // now set interrupts
  EICRA=0xFF;
  EICRB=0;
  EIMSK=0x0F;
  cbi(DDRD,0);sbi(PORTD,0); //Front
  cbi(DDRD,1);sbi(PORTD,1);//Left
  cbi(DDRD,2);sbi(PORTD,2);//Back
  cbi(DDRD,3);sbi(PORTD,3);//Right

  // turn all interrupts on
  sei();


//Arm the ESCs
  PWMF=PWMmin;
  PWML=PWMmin;
  PWMB=PWMmin;
  PWMR=PWMmin;
  _delay_ms(100);

  PWMF=PWMmax;
  PWML=PWMmax;
  PWMB=PWMmax;
  PWMR=PWMmax;
    _delay_ms(100);

    PWMF=PWMmin;
    PWML=PWMmin;
    PWMB=PWMmin;
    PWMR=PWMmin;
    _delay_ms(100);

    PWMF=140;
       PWML=140;
       PWMB=140;
      PWMR=140;

       while(1) {
    	   PWMF=140;
    	          PWML=14;
    	          PWMB=14;
    	         PWMR=14;
         _delay_ms(100); // LED OFF
         PWMF=140;
            	          PWML=1140;
            	          PWMB=1140;
            	         PWMR=1140;
       }


// main loop - wait for flag set in interrupt routine
while (1)
{
  //feedback Control Front
 posF=posF+KP*(vrF-velF)+KD*(0-vdF); //vel here is in rps...so multiply by 60 to make into rpm
 temp=MUtoPWM*posF;
 if(temp<PWMmin)
{temp=PWMmin;}
 if(temp>PWMmax)
{temp=PWMmax;}
PWMF=temp;


  //feedback Control Left
 posL=posL+KP*(vrL-velL)+KD*(0-vdL); //vel here is in rps...so multiply by 60 to make into rpm
 temp=MUtoPWM*posL;
 if(temp<PWMmin)
{temp=PWMmin;}
 if(temp>PWMmax)
{temp=PWMmax;}
PWML=temp;

  //feedback Control Back
 posB=posB+KP*(vrB-velB)+KD*(0-vdB); //vel here is in rps...so multiply by 60 to make into rpm
 temp=MUtoPWM*posB;
 if(temp<PWMmin)
{temp=PWMmin;}
 if(temp>PWMmax)
{temp=PWMmax;}
PWMB=temp;

  //feedback Control Right
 posR=posR+KP*(vrR-velR)+KD*(0-vdR); //vel here is in rps...so multiply by 60 to make into rpm
 temp=MUtoPWM*posR;
 if(temp<PWMmin)
{temp=PWMmin;}
 if(temp>PWMmax)
{temp=PWMmax;}
PWMR=temp;

// SPI testing
  if (process_it == 1)
    {

       vrF=100*buff[0]+buff[1];
       vrL=100*buff[2]+buff[3];
       vrB=100*buff[4]+buff[5];
       vrR=100*buff[6]+buff[7];

     process_it = 0;
   }

//feedback delay
//delay(90);
}
return 0;
}
// SPI interrupt routine
ISR (SPI_STC_vect)
{
  int i;
  buff[0] = SPDR;
  for(i=1;i<8;i++)
  {
  while(!(SPSR & (1<<SPIF))){}
  buff[i] = SPDR;
  }
process_it = 1;
}

//Front interrup
SIGNAL(INT0_vect)
{
  curntTimeF=curntTimeF+TIMER_CNT;
  curntTimeF=curntTimeF-stTimeF;

  if (curntTimeF<MIN_CUTOFF_TIMERVAL)
  {return;}

   velF=VEL_COMPUTE_NUMERATOR/(curntTimeF);
   vdF=(velF-vpF)/(curntTimeF);
   vpF=velF;

   curntTimeF=0;
   stTimeF=TIMER_CNT;

   Toggle_led();

}

//Left interrup
SIGNAL(INT1_vect)
{
  curntTimeL=curntTimeL+TIMER_CNT;
  curntTimeL=curntTimeL-stTimeL;

  if (curntTimeL<MIN_CUTOFF_TIMERVAL)
  {return;}

   velL=VEL_COMPUTE_NUMERATOR/(curntTimeL);
   vdL=(velL-vpL)/(curntTimeL);
   vpL=velL;

   curntTimeL=0;
   stTimeL=TIMER_CNT;

   Toggle_led();

}

//BACK interrup
SIGNAL(INT2_vect)
{
  curntTimeB=curntTimeB+TIMER_CNT;
  curntTimeB=curntTimeB-stTimeB;

  if (curntTimeB<MIN_CUTOFF_TIMERVAL)
  {return;}

   velB=VEL_COMPUTE_NUMERATOR/(curntTimeB);
   vdB=(velB-vpB)/(curntTimeB);
   vpB=velB;

   curntTimeB=0;
   stTimeB=TIMER_CNT;

   Toggle_led();

}

//Right interrup
SIGNAL(INT3_vect)
{
  curntTimeR=curntTimeR+TIMER_CNT;
  curntTimeR=curntTimeR-stTimeR;

  if (curntTimeR<MIN_CUTOFF_TIMERVAL)
  {return;}

   velR=VEL_COMPUTE_NUMERATOR/(curntTimeR);
   vdR=(velR-vpR)/(curntTimeR);
   vpR=velR;

   curntTimeR=0;
   stTimeR=TIMER_CNT;

   Toggle_led();

}
// Common timer overflow interrupt
SIGNAL(TIMER0_OVF_vect)
{
  curntTimeF=curntTimeF+TIMERSIZE;
  curntTimeL=curntTimeL+TIMERSIZE;
  curntTimeB=curntTimeB+TIMERSIZE;
  curntTimeR=curntTimeR+TIMERSIZE;
}


