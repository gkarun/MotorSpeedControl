// Timer and Counter example for Mega2560
// Author: Nick Gammon
// Date: 24th April 2012

// input on pin D47 (T5)

// these are checked for in the main program
volatile unsigned long timerCounts;
volatile boolean counterReady;

// internal to counting routine
unsigned long overflowCount;
unsigned int timerTicks;
unsigned int timerPeriod;

void startCounting (unsigned int ms) 
  {

  counterReady = false;         // time not up yet
  timerPeriod = ms;             // how many 1 ms counts to do
  timerTicks = 0;               // reset interrupt counter
  overflowCount = 0;            // no overflows yet

  // reset Timer 2 and Timer 5
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR5A = 0;             
  TCCR5B = 0;  

  // Timer 5 - counts events on pin D47
  TIMSK5 = bit (TOIE1);   // interrupt on Timer 5 overflow

  // Timer 2 - gives us our 1 ms counting interval
  // 16 MHz clock (62.5 ns per tick) - prescaled by 128
  //  counter increments every 8 µs. 
  // So we count 125 of them, giving exactly 1000 µs (1 ms)
  TCCR2A = bit (WGM21) ;   // CTC mode
  OCR2A  = 124;            // count up to 125  (zero relative!!!!)

  // Timer 2 - interrupt on match (ie. every 1 ms)
  TIMSK2 = bit (OCIE2A);   // enable Timer2 Interrupt

  TCNT2 = 0;     
  TCNT5 = 0;      // Both counters to zero

  // Reset prescalers
  GTCCR = bit (PSRASY);        // reset prescaler now
  // start Timer 2
  TCCR2B =  bit (CS20) | bit (CS22) ;  // prescaler of 128
  // start Timer 5
  // External clock source on T4 pin (D47). Clock on rising edge.
  TCCR5B =  bit (CS50) | bit (CS51) | bit (CS52);

}  // end of startCounting

ISR (TIMER5_OVF_vect)
{
  ++overflowCount;               // count number of Counter1 overflows  
}  // end of TIMER5_OVF_vect


//******************************************************************
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR (TIMER2_COMPA_vect) 
{
  // grab counter value before it changes any more
  unsigned int timer5CounterValue;
  timer5CounterValue = TCNT5;  // see datasheet, (accessing 16-bit registers)

  // see if we have reached timing period
  if (++timerTicks < timerPeriod) 
    return;  // not yet

  // if just missed an overflow
  if (TIFR5 & TOV5)
    overflowCount++;

  // end of gate time, measurement ready

  TCCR5A = 0;    // stop timer 5
  TCCR5B = 0;    

  TCCR2A = 0;    // stop timer 2
  TCCR2B = 0;    

  TIMSK2 = 0;    // disable Timer2 Interrupt
  TIMSK5 = 0;    // disable Timer5 Interrupt

  // calculate total count
  timerCounts = (overflowCount << 16) + timer5CounterValue;  // each overflow is 65536 more
  counterReady = true;              // set global flag for end count period
}  // end of TIMER2_COMPA_vect


void setup () {
  Serial.begin(115200);       
  Serial.println("Frequency Counter");
} // end of setup


void loop () {

  // stop Timer 0 interrupts from throwing the count out
  byte oldTCCR0A = TCCR0A;
  byte oldTCCR0B = TCCR0B;
  TCCR0A = 0;    // stop timer 0
  TCCR0B = 0;    
  
  startCounting (500);  // how many ms to count for

  while (!counterReady) 
     { }  // loop until count over

  // adjust counts by counting interval to give frequency in Hz
  float frq = (timerCounts *  1000.0) / timerPeriod;

  // restart timer 0
  TCCR0A = oldTCCR0A;
  TCCR0B = oldTCCR0B;

  Serial.print ("Frequency: ");
  Serial.println ((unsigned long) frq);
  
  // let serial stuff finish
  delay(200);

}   // end of loop