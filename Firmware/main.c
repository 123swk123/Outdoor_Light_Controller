/************************************************************************
* Copyright (C) 2016 SwK(123swk123@gmail.com)
*
* main.c is part of Outdoor Light Controller.
*
* Outdoor Light Controller is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Outdoor Light Controller is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Outdoor Light Controller.  If not, see <http://www.gnu.org/licenses/>.
************************************************************************/

#ifdef __CODEVISIONAVR__
#include <io.h>
#include <delay.h>
#else
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <avr/sleep.h>
#endif

#include <stdbool.h>
#include <stdint.h>

#define TIMER_INC_IN_HRS  2
#define TIMER_MAX         12
#define TIMER_INC_STEPS   (TIMER_MAX/TIMER_INC_IN_HRS)
#define TIMER_NOOF_SECS_IN_1HR  3612/*(3600 / 0.99669)*/

#define TIMER_ADC_STEPS   (1023 / TIMER_INC_STEPS)

#define TIMEOUT_IN_SECS (((gu16TimerValue / TIMER_ADC_STEPS) + 1) * TIMER_INC_IN_HRS * TIMER_NOOF_SECS_IN_1HR)

#define OUTPUT_PIN  PORTB0

#define INPUT_LDR_DARK_MAX  200
#define INPUT_LDR_BRIGHT_MIN  30

volatile uint16_t gu16CurrTimeInSecs = 0;
volatile uint16_t gu16LDRValue = 0;
volatile uint16_t gu16TimerValue = 0;

// Timer 0 overflow interrupt service routine
#ifdef __CODEVISIONAVR__
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
#else
ISR(TIM0_OVF_vect)
#endif
{
  // Reinitialize Timer 0 value
  TCNT0=0x6E;
  gu16CurrTimeInSecs++;

}

// Bandgap Voltage Reference: Off
#define ADC_VREF_TYPE ((0<<REFS0) | (0<<ADLAR))

// ADC interrupt service routine
// with auto input scanning
#ifdef __CODEVISIONAVR__
interrupt [ADC_INT] void adc_isr(void)
#else
ISR(ADC_vect)
#endif
{
  static bool bWhich_ADC_Ch = 0;
  
  if (bWhich_ADC_Ch)
  {
    gu16TimerValue = ADCW;
    ADMUX = 3;
    bWhich_ADC_Ch = 0;
  } 
  else
  {
    gu16LDRValue = ADCW;
    ADMUX = 0;
    bWhich_ADC_Ch = 1;
  }

  // Delay needed for the stabilization of the ADC input voltage   
  #ifdef __CODEVISIONAVR__
  delay_us(10);          
  #else
  _delay_us(10);
  #endif
}

void Output_State_On(void)
{
  PORTB |= (1 << OUTPUT_PIN);
  gu16CurrTimeInSecs = 0;
}

void Output_State_Off(void)
{
  PORTB &= ~(1 << OUTPUT_PIN);
}

void main(void)
{
  // Crystal Oscillator division factor: 64
  CLKPR=(1<<CLKPCE);
  CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (1<<CLKPS2) | (1<<CLKPS1) | (0<<CLKPS0);

  DDRB=(0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (1<<DDB0);
  PORTB=(0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

  // Timer/Counter 0 initialization
  // Clock source: System Clock
  // Clock value: 0.146 kHz
  // Mode: Normal top=0xFF
  // OC0A output: Disconnected
  // OC0B output: Disconnected
  // Timer Period: 0.99669 s
  TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
  TCCR0B=(0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);
  TCNT0=0x6E;
  OCR0A=0x00;
  OCR0B=0x00;

  // Timer/Counter 0 Interrupt(s) initialization
  TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);

  // External Interrupt(s) initialization
  // INT0: Off
  // Interrupt on any change on pins PCINT0-5: Off
  GIMSK=(0<<INT0) | (0<<PCIE);
  MCUCR=(0<<ISC01) | (0<<ISC00);

  // Analog Comparator initialization
  // Analog Comparator: Off
  // The Analog Comparator's positive input is
  // connected to the AIN0 pin
  // The Analog Comparator's negative input is
  // connected to the AIN1 pin
  ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIS1) | (0<<ACIS0);
  // Digital input buffer on AIN0: On
  // Digital input buffer on AIN1: On
  DIDR0=(0<<AIN0D) | (0<<AIN1D);

  // ADC initialization
  // ADC Clock frequency: 75.000 kHz
  // ADC Bandgap Voltage Reference: Off
  // ADC Auto Trigger Source: Timer0 Overflow
  // Digital input buffers on ADC0: On, ADC1: Off, ADC2: Off, ADC3: On
  DIDR0|=(1<<ADC0D) | (0<<ADC2D) | (1<<ADC3D) | (0<<ADC1D);
  ADMUX=3;
  ADCSRB=(1<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);
  ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (1<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (1<<ADPS0);

  while (1)
  {
    #if 1
    uint16_t u16TimeOutInSecs;
    uint16_t u16LDRValue;
    uint16_t u16CurrTime;
    #endif

    #ifdef __CODEVISIONAVR__
    #asm("sei")
    /*sleep*/
    #asm("cli")
    #else
    //set_sleep_mode(SLEEP_MODE_IDLE);
    //sleep_enable();
    //sleep_bod_disable();
    //sei();
    //sleep_cpu();
    //sleep_disable();
    //cli();
    #endif

    #if 1
      cli();
      u16LDRValue = gu16LDRValue;
      u16TimeOutInSecs = TIMEOUT_IN_SECS;
      u16CurrTime = gu16CurrTimeInSecs;
      sei();
    #endif
    
    if ((PORTB & (1 << OUTPUT_PIN)) == 0) 
    {
      if (u16LDRValue > INPUT_LDR_DARK_MAX)
      {
        Output_State_On();
      }
    } 
    else
    {
      if ((u16LDRValue < INPUT_LDR_BRIGHT_MIN) || (u16CurrTime >= u16TimeOutInSecs))
      {
        Output_State_Off();
      }
    }    
  }
}
