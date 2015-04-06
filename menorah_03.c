/*************************************************************************************

   4-channel musical menorah program for the EvilMadScientist Menorah kit
      
    http://www.evilmadscientist.com/2009/new-led-hanukkah-menorah-kit/
    
To connect a speaker, install one end of each of four 220-ohm resistors
to the following holes in the PC board:
    J2 pin 4 (output PD0)
    J2 pin 5 (output PD1)
    OPT1 pin closest to the CPU chip (output PA1)
    OPT2 pin closest to the CPU chip (output PA0)
 Connect the other ends of the four resistors together and to one speaker wire.
 Connect the other speaker wire to a ground, like J2 pin 1.
 
The loudest sound will come from a medium-size speaker (4", say) with a big magnet.
It will also be louder if you use three AA batteries (4.5 volts) instead of two (3 volts)
for the menorah's power. You could also connect the output to an amplifier.

Note that the "option 1" jumper for moving the shamash (lighter candle) to the end can
no longer be used, because we use that pin as one of the speaker outputs.

I had originally intended this to work for the ATtiny2313 with 2K of program memory
that comes with the Menorah kit, but I couldn't get the program to fit with musical
scores of a reasonable length. So you have to upgrade to the pin-compatible ATtiny4313, 
which has 4K of program memory.

You also need to change the processor to run at 8Mhz, by programming its LFUSE to E4.
Here are the avrdude commands to program the fuse and the flash program memory, if you're
using the adafruit USBtinyISP AVR Programmer Kit (http://www.adafruit.com/products/46):

   avrdude -c usbtiny -p t4313 -U lfuse:w:0xe4:m
   avrdude -c usbtiny -p t4313 -U flash:w:menorah.hex

See a video of the music-playing menorah in action here:
  https://www.youtube.com/watch?v=-ckyKqKwPhU
  
I've posted both the source code and the generated hex file here:
  https://drive.google.com/folderview?id=0B1ZOnb_w5lfBd1ZoSS0wZ0ZpejA
I use the free Atmel Studio development environment, but others should be fine too.

------------------------------------------------------------------------------------------

This uses an much-modified version of the "Playtune" tone generator library
that I originally wrote in 2011 for Arduinos, and which is available as open source
at https://code.google.com/p/arduino-playtune/

The companion program for converting MIDI files into bytestreams that
can be played by Playtune is at https://code.google.com/p/miditones/.

(C) Copyright 2013, Len Shustek; see below for licensing terms.

Change log for Menorah version

V2.0, 16 November 2013, L. Shustek
  - Initial release for ATtiny; 2 channels, each using one of the ATTiny timers.
  
V2.1, 23 November 2013, L. Shustek
  - Change to polling scheme instead of variable-rate interrupts. The frequencies
    are less accurate, but we can play more polyphonic tones than the number of
    hardware timers.
 - Do no multiplication or division at runtime, and do mostly 8-bit arithmetic.

V2.2, 27 November 2013, L. Shustek
  - Minor changes, mostly cosmetic.
  - Avoid static initialization of global variables in the data space.

***********************************************************************************/
/*
*   (C) Copyright 2013, Len Shustek
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of version 3 of the GNU General Public License as
*   published by the Free Software Foundation at http://www.gnu.org/licenses,
*   with Additional Permissions under term 7(b) that the original copyright
*   notice and author attribution must be preserved and under term 7(c) that
*   modified versions be marked as different from the original.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*/
/**********************************************************************************
 *
 * Playtune: An Arduino Tune Generator library
 *
 * Plays a polyphonic musical score.
 *
 *   This was inspired by and adapted from "A Tone Generator Library",
 *   written by Brett Hagman, http://www.roguerobotics.com/
 *
 * Playtune change log
 *  19 January 2011, L.Shustek, V1.0
 *     - Initial release.
 *  23 February 2011, L. Shustek, V1.1
 *     - prevent hang if delay rounds to count of 0
 *   4 December 2011, L. Shustek, V1.2
 *     - add special TESLA_COIL mods
 *  10 June 2013, L. Shustek, V1.3
 *     - change to be compatible with Arduino IDE version 1.0.5
 *  12 November 2013, L. Shustek, code branch for the EvilMadScientist Menorah
 *     - This is a much simplified version for ATtiny, using fast polling
 *       to toggle notes instead of timers programmed for each channel.
 *	   - We do no multiplication or division at runtime, and do as much 
 *       in 8-bit as possible.
 */

/*---------------------------------------------------------------------------------
 *
 *
 *                              About Playtune
 *
 *  Playtune interprets a bytestream of commands that represent a polyphonic musical
 *  score.  It uses the microcontroller counters for generating tones, so the number of
 *  simultaneous notes that can be played depends on which processor you have.
 *
 *  Once a score starts playing, all of the processing happens in interrupt routines,
 *  so any other "real" program can be running at the same time, as long as it doesn't
 *  use the timers or output pins that Playtune is using.  Playtune generates a lot of
 *  interrupts because the toggling of the output bits is done in software, not by the
 *  timer hardware.  But measurements I've made on a NANO show that Playtune uses less
 *  than 10% of the available processor cycles even when playing all three channels at
 *  pretty high frequencies.
 *
 *  The easiest way to hear the music is to connect each of the output pins to a resistor
 *  (500 ohms, say).  Connect other ends of the resistors together and then to one
 *  terminal of an 8-ohm speaker.  The other terminal of the speaker is connected to
 *  ground.  No other hardware is needed!  If you are going to connect to an amplifier,
 *  you should DC-isolate the signal using a capacitor.
 *
 *  There is no volume modulation.  All tones are played at the same volume, which
 *  makes some scores sound strange.  This is definitely not a high-quality synthesizer.
 *
 *
 *  *****  The score bytestream  *****
 *
 *  The bytestream is a series of commands that can turn notes on and off, and can
 *  start a waiting period until the next note change.  Here are the details, with
 *  numbers shown in hexadecimal.
 *
 *  If the high-order bit of the byte is 1, then it is one of the following commands:
 *
 *    9t nn  Start playing note nn on tone generator t.  Generators are numbered
 *           starting with 0.  The notes numbers are the MIDI numbers for the chromatic
 *           scale, with decimal 60 being Middle C, and decimal 69 being Middle A
 *           at 440 Hz.  The highest note is decimal 127 at about 12,544 Hz.
 *
 *    8t     Stop playing the note on tone generator t.
 *
 *    F0     End of score: stop playing.
 *
 *    E0     End of score: start playing again from the beginning.
 *
 *  If the high-order bit of the byte is 0, it is a command to wait.  The other 7 bits
 *  and the 8 bits of the following byte are interpreted as a 15-bit big-endian integer
 *  that is the number of milliseconds to wait before processing the next command.
 *  For example,
 *
 *    07 D0
 *
 *  would cause a wait of 0x07d0 = 2000 decimal millisconds or 2 seconds.  Any tones
 *  that were playing before the wait command will continue to play.
 *
 *  The score is stored in Flash memory ("PROGMEM") along with the program, because
 *  there's a lot more of that than data memory.
 *
 *
 *  *****  Where does the score data come from?  *****
 *
 *  Well, you can write the score by hand from the instructions above, but that's
 *  pretty hard.  An easier way is to translate MIDI files into these score commands,
 *  and I've written a program called "miditones" to do that.  See the separate
 *  documentation for that program, which is also open source.
 *
 *
 *  *****  Nostalgia from me  *****
 *
 *  Writing Playtune was a lot of fun, because it essentially duplicates what I did
 *  as a graduate student at Stanford University in about 1973.  That project used the
 *  then-new Intel 8008 microprocessor, plus three hardware square-wave generators that
 *  I built out of 7400-series TTL.  The music compiler was written in Pascal and read
 *  scores that were hand-written in a notation I made up that looked something like
 *      C  Eb  4G  8G+  2R  +  F  D#
 *  This was done was before MIDI had been invented, and anyway I wasn't a pianist so I
 *  would not have been able to record my own playing.  I could barely read music well
 *  enough to transcribe scores, but I created, slowly, quite a few of them.
 *
 *  Len Shustek, 4 Feb 2011
 *
 -------------------------------------------------------------------------------------*/

#define DBUG 0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#define WDTCSR WDTCR	// There's a typo in the AtmelStudio iotn4313.h file!

#define NUM_CHANS 4		// number of speaker outputs

#define CPU_MHZ	8		// ATtiny CKSEL 0100: 8 Mhz, 1x prescale (lfuse=E4)

/*  Menorah I/O port assignments */

#define SPEAKER0 PD0  // output, speaker 1 (also USB-TTL RX)
#define SPEAKER1 PD1  // output, speaker 2 (also USB-TTL TX)
#define LED0     PD2  // output, LED 0
#define LED1     PD3  // output, LED 1
#define LED2     PD4  // output, LED 2
#define LED3     PD5  // output, LED 3
#define LED4     PD6  // output, LED 4

#define LED5     PB0  // output, LED 5
#define LED6     PB1  // output, LED 6
#define LED7     PB2  // output, LED 7
#define LED8     PB3  // output, LED 8
#define BUTTON   PB4  // input, pushbutton
//               PB5  // USB MOSI
//               PB6  // USB MISO
//               PB7  // USB USCK

#define SPEAKER2 PA0  // output, speaker 1 (was input, option jumper 2)
#define SPEAKER3 PA1  // output, speaker 3 (was input, option jumper 1)
//               PA2  // input, USB reset

#define ADDRA	0x1b	// address of PORTA
#define ADDRB	0x18	// address of PORTB
#define ADDRD	0x12	// address of PORTD
#define PORT(x)	_SFR_IO8(x)
#define BUTTON_DOWN ((PINB & (1<<BUTTON))==0)  // true if button is down

#define EEPROM_addr  1	// second byte (why not?) of EEPROM holds day number

// speaker registers
// We use constants for speaker ports because we don't want any overhead
// in the interrupt routines; the loop through the speakers there is unrolled.

#define SPEAKER0_REG PORTD	//  data register
#define SPEAKER0_DIR DDRD	//  direction register

#define SPEAKER1_REG PORTD	//  data register
#define SPEAKER1_DIR DDRD	//  direction register

#define SPEAKER2_REG PORTA	//  data register
#define SPEAKER2_DIR DDRA	//  direction register

#define SPEAKER3_REG PORTA	//  data register
#define SPEAKER3_DIR DDRA	//  direction register

typedef uint8_t byte;
typedef uint8_t boolean;
#define false 0
#define true 1
#define noInterrupts cli
#define Interrupts sei

// variable for timing

volatile unsigned int scorewait_interrupt_count;
volatile unsigned int delaywait_interrupt_count;

// variables for music-playing

volatile byte *score_start;
volatile byte *score_cursor;
volatile boolean tune_playing;

volatile long accumulator [NUM_CHANS];
volatile long decrement [NUM_CHANS];
volatile boolean playing [NUM_CHANS];

/* Table of accumulator decrement values, generated by a companion Excel spreadsheet.
   These depend on the polling frequency and the accumulator restart value.
   We basically do incremental division for each channel in the polling interrupt routine:
        accum -= decrement
        if (accum < 0) {
            toggle speaker output
            accum += ACCUM_RESTART
        }
*/

#define POLLTIME_USEC 50		// polling interval in microseconds
#define ACCUM_RESTART 4194304L	// 2^22 allows 1-byte addition on 3- or 4-byte numbers
#define MAX_NOTE 123

const long decrement_PGM[MAX_NOTE+1] PROGMEM = {
    3429L, 3633L, 3849L, 4078L, 4320L, 4577L, 4850L, 5138L, 5443L, 5767L, 6110L, 6473L, 
    6858L, 7266L, 7698L, 8156L, 8641L, 9155L, 9699L, 10276L, 10887L, 11534L, 12220L, 
    12947L, 13717L, 14532L, 15396L, 16312L, 17282L, 18310L, 19398L, 20552L, 21774L, 
    23069L, 24440L, 25894L, 27433L, 29065L, 30793L, 32624L, 34564L, 36619L, 38797L, 
    41104L, 43548L, 46137L, 48881L, 51787L, 54867L, 58129L, 61586L, 65248L, 69128L, 
    73238L, 77593L, 82207L, 87096L, 92275L, 97762L, 103575L, 109734L, 116259L, 123172L, 
    130496L, 138256L, 146477L, 155187L, 164415L, 174191L, 184549L, 195523L, 207150L, 
    219467L, 232518L, 246344L, 260992L, 276512L, 292954L, 310374L, 328830L, 348383L, 
    369099L, 391047L, 414299L, 438935L, 465035L, 492688L, 521984L, 553023L, 585908L, 
    620748L, 657659L, 696766L, 738198L, 782093L, 828599L, 877870L, 930071L, 985375L, 
    1043969L, 1106047L, 1171815L, 1241495L, 1315318L, 1393531L, 1476395L, 1564186L, 
    1657197L, 1755739L, 1860141L, 1970751L, 2087938L, 2212093L, 2343631L, 2482991L, 
    2630637L, 2787063L, 2952790L, 3128372L, 3314395L, 3511479L, 3720282L, 3941502L, 
    4175876L
};

void tune_playnote (byte chan, byte note);
void tune_stopnote (byte chan);
void tune_stepscore (void);


//--------------------------------------------------------------------------
// Initialize the timers
//--------------------------------------------------------------------------

void init_timers () {
    
    // We use the 8 bit timer to generate the polling interrupt for notes.
    // It should interrupt often, like every 50 microseconds.
    
    TCCR0A = (1 << WGM01);	// mode 010: CTC   
#if CPU_MHZ==4
    TCCR0B = 1 << CS00;		// clock select 001: no prescaling
    OCR0A = CPU_MHZ * POLLTIME_USEC;
#elif CPU_MHZ==8
    TCCR0B = 1 << CS01;		// clock select 010: clk/8 prescaling
    OCR0A = CPU_MHZ/8 * POLLTIME_USEC;
#else
 unusual frequency
#endif
    
    // We use the 16 bit timer both for timing scores from the interrupt routine
    // and doing mainline code waits. It interrupts once a millisecond.
    
    TCCR1A = 0;  // mode 0100: CTC
    TCCR1B = (1 << WGM12) | (1<<CS10); // clock select 001: no prescaling
    OCR1A = CPU_MHZ * 1000;

    tune_playing = false;    
    scorewait_interrupt_count = 0;
    delaywait_interrupt_count = 0;
    
    TIMSK =(1<<OCIE0A) | (1<<OCIE1A); // turn on match A interrupts for both timers
    Interrupts();		 // enable interrupts

}


//--------------------------------------------------------------------------
// Start playing a note on a particular channel
//--------------------------------------------------------------------------

void tune_playnote (byte chan, byte note) {

    if (chan < NUM_CHANS) {
        if (note>MAX_NOTE) note=MAX_NOTE;
        decrement[chan] = pgm_read_dword(decrement_PGM + note);
        accumulator[chan] = ACCUM_RESTART;
        playing[chan]=true;
    }
}


//--------------------------------------------------------------------------
// Stop playing a note on a particular channel
//--------------------------------------------------------------------------

void tune_stopnote (byte chan) {
    playing[chan]= false;
}


//--------------------------------------------------------------------------
//   Play a score
//--------------------------------------------------------------------------

void tune_stopscore();

void tune_playscore (byte *score) {
    if (tune_playing) tune_stopscore();
    score_start = score;
    score_cursor = score;
    tune_stepscore();	/* execute initial commands */
    tune_playing = true;  /* release the interrupt routine */
}

/* Do score commands until a "wait" is found, or the score is stopped.
 This is called initially from tune_playcore, but then is called
 from the interrupt routine when waits expire.
 */

#define CMD_PLAYNOTE	0x90	/* play a note: low nibble is generator #, note is next byte */
#define CMD_STOPNOTE	0x80	/* stop a note: low nibble is generator # */
#define CMD_RESTART		0xe0	/* restart the score from the beginning */
#define CMD_STOP		0xf0	/* stop playing */
/* if CMD < 0x80, then the other 7 bits and the next byte are a 15-bit big-endian number of msec to wait */

void tune_stepscore (void) {
  byte cmd, opcode, chan;

  while (1) {
    cmd = pgm_read_byte(score_cursor++);
    if (cmd < 0x80) {
      /* wait count is in msec. */
      scorewait_interrupt_count = ((unsigned)cmd << 8) | (pgm_read_byte(score_cursor++));
      break;
    }
    opcode = cmd & 0xf0;
    chan = cmd & 0x0f;
    if (opcode == CMD_STOPNOTE) {
      /* stop note */
      tune_stopnote (chan);
    }
    else if (opcode == CMD_PLAYNOTE) {
      /* play note */
      tune_playnote (chan, pgm_read_byte(score_cursor++));
    }
    else if (opcode == CMD_RESTART) {
      /* restart score */
      score_cursor = score_start;
    }
    else if (opcode == CMD_STOP) {
      /* stop score */
      tune_playing = false;
      break;
    }
  }
}

//--------------------------------------------------------------------------
// Stop playing a score
//--------------------------------------------------------------------------

void tune_stopscore (void) {
  tune_stopnote(0);
  tune_stopnote(1);
  tune_stopnote(2);
  tune_stopnote(3);  // depends on NUM_CHANS==4
  tune_playing = false;
}

//--------------------------------------------------------------------------
// Delay a specified number of milliseconds, up to about 30 seconds.
//--------------------------------------------------------------------------

void tune_delay (unsigned duration) {
    boolean notdone;
    
    delaywait_interrupt_count = duration;
    do {
        // wait until the interrupt routines decrements the toggle count to zero
        noInterrupts();
        notdone = delaywait_interrupt_count != 0;  /* interrupt-safe test */
        Interrupts();
    }  while (notdone);
}

//--------------------------------------------------------------------------
// Stop all channels
//--------------------------------------------------------------------------

void tune_stopchans(void) {

  TIMSK &= ~(1 << OCIE0A);  // disable all timer interrupts
  TIMSK &= ~(1 << OCIE1A);
}

//--------------------------------------------------------------------------
//  Timer interrupt Service Routines
//--------------------------------------------------------------------------

ISR(TIMER0_COMPA_vect) { //******* 8-bit timer: 50 microsecond interrupts
    
// We unroll code with a macro to avoid loop overhead.
// For even greater efficiency, we could write this in assembly code
// and do 3-byte instead of 4-byte arithmetic.

  #define dospeaker(spkr) if (playing[spkr]) {	 \
      accumulator[spkr] -= decrement[spkr];		 \
  if (accumulator[spkr]<0) {					 \
      SPEAKER##spkr##_REG ^= (1<<SPEAKER##spkr); \
      accumulator[spkr] += ACCUM_RESTART;		 \
    }											 \
  }	
  
  dospeaker(0);
  dospeaker(1);
  dospeaker(2);
  dospeaker(3);  // Depends on NUM_CHANS==4
}

ISR(TIMER1_COMPA_vect) { //******* 16-bit timer: millisecond interrupts
    
    // decrement score wait counter
    if (tune_playing && scorewait_interrupt_count && --scorewait_interrupt_count == 0) {
        // end of a score wait, so execute more score commands
        tune_stepscore ();  // execute commands
    }
    
    // decrement delay wait counter
    if (delaywait_interrupt_count)
    --delaywait_interrupt_count;	// countdown for tune_delay()
}

//-----------------------------------------------------------------------------------------------------
// The music we play, generated by "miditones" from MIDI files.
//-----------------------------------------------------------------------------------------------------

const

/// Playtune bytestream for file "maoz_4chan.mid" created by MIDITONES V1.6 on Mon Nov 25 14:36:59 2013
// command line: c:\data\projects\midi\miditones\lcc\miditones maoz_4chan
byte PROGMEM score [] = {
    0x90,74, 0x91,62, 0x92,54, // control track
    2,33, 0x80, 0x81, 0x90,69, 0x91,57, 1,116, 0x82, 0,173, 0x80, 0x81,
    0x90,54, 0x91,74, 0x92,62, 2,33, 0x80, 0x81, 0x82, 0x90,47, 0x91,79, 0x92,62, 0x93,67, 2,33,
    0x80, 0x81, 0x82, 0x83, 0x90,57, 0x91,78, 0x92,66, 0x93,62, 2,33, 0x80, 0x81, 0x82, 0x83, 0x90,55, 0x91,76,
    0x92,61, 0x93,64, 2,33, 0x80, 0x81, 0x82, 0x83, 0x90,54, 0x91,74, 0x92,62, 2,247, 0x81, 0,19,
    0x82, 0,1, 0x80, 0,124, 0x90,81, 0,159, 0x80, 0,25, 0x90,69, 0x91,50, 0x92,62, 0,2,
    0x93,81, 2,30, 0x80, 0x83, 0x90,71, 0x93,83, 2,33, 0x82, 0x80, 0x81, 0x83, 0x90,64, 0x91,52, 0x92,76,
    0x93,62, 2,33, 0x81, 0x82, 0x80, 0x83, 0x90,52, 0x91,78, 0x92,62, 0x93,67, 1,16, 0x81, 0x91,79,
    1,16, 0x82, 0x83, 0x80, 0x81, 0x90,62, 0x91,57, 0x92,78, 0x93,66, 2,23, 0x81, 0,9, 0x82, 0x80,
    0x83, 0x90,45, 0x91,76, 0x92,61, 0x93,64, 2,33, 0x80, 0x81, 0x82, 0x83, 0x90,54, 0x91,74, 0x92,62,
    2,247, 0x80, 0,2, 0x81, 0,2, 0x82, 1,69, 0x90,54, 0x91,74, 0x92,62, 2,33, 0x81, 0x82,
    0x91,69, 0x92,57, 1,16, 0x93,49, 0,17, 0x80, 0,255, 0x81, 0x82, 0x83, 0x90,74, 0x91,62, 0x92,47,
    2,33, 0x80, 0x81, 0x82, 0x90,79, 0x91,59, 0x92,52, 0x93,67, 2,33, 0x82, 0x80, 0x81, 0x83, 0x90,45,
    0x91,78, 0x92,62, 0x93,66, 2,33, 0x80, 0x81, 0x82, 0x83, 0x90,46, 0x91,76, 0x92,64, 0x93,61, 2,33,
    0x80, 0x81, 0x82, 0x83, 0x90,47, 0x91,74, 0x92,62, 3,85, 0x81, 0,9, 0x80, 0,1, 0x82, 0,225,
    0x90,50, 0x91,81, 0x92,62, 0x93,69, 2,33, 0x81, 0x82, 0x83, 0x91,83, 0x92,62, 0x93,71, 2,33,
    0x80, 0x81, 0x82, 0x83, 0x90,52, 0x91,76, 0x92,64, 2,33, 0x80, 0x81, 0x82, 0x90,43, 0x91,78, 0x92,59,
    0x93,67, 1,16, 0x81, 0x91,79, 1,16, 0x82, 0x83, 0x80, 0x81, 0x90,62, 0x91,45, 0x92,78, 0x93,66,
    2,33, 0x82, 0x80, 0x83, 0x90,76, 0x92,64, 0x93,61, 1,16, 0x81, 0,5, 0x91,49, 1,9, 0x81,
    0,1, 0x80, 0x82, 0x83, 0x90,50, 0x91,74, 0x92,62, 3,168, 0x80, 0x81, 0,154, 0x82, 0x90,50, 0x91,81,
    0x92,66, 0x93,69, 2,215, 0x81, 0,35, 0x80, 0,55, 0x90,81, 0x82, 0x83, 0x91,69, 0x92,66, 0,178,
    0x93,50, 0,93, 0x80, 0x81, 0x82, 0x90,83, 0x91,67, 0x92,71, 2,33, 0x80, 0x81, 0x82, 0x90,85, 0x91,69,
    0x92,73, 1,80, 0x83, 0,208, 0x80, 0x81, 0x82, 0x90,50, 0x91,86, 0x92,66, 0x93,69, 3,158, 0x80,
    0,164, 0x81, 0x82, 0x83, 0x90,50, 0x91,81, 0x92,69, 0x93,66, 3,131, 0x80, 0,191, 0x81, 0x82, 0x83,
    0x90,54, 0x91,86, 0x92,62, 0x93,69, 2,33, 0x81, 0x91,85, 2,33, 0x82, 0x83, 0x80, 0x81, 0x90,62,
    0x91,55, 0x92,83, 2,33, 0x80, 0x81, 0x82, 0x90,69, 0x91,54, 0x92,81, 0x93,62, 2,33, 0x81, 0x82,
    0x80, 0x83, 0x90,57, 0x91,81, 0x92,66, 0x93,69, 1,16, 0x81, 0x82, 0x83, 0x91,79, 0x92,64, 0x93,67,
    1,16, 0x81, 0x82, 0x83, 0x91,78, 0x92,62, 0x93,66, 1,16, 0x81, 0x82, 0x83, 0x91,79, 0x92,64, 0x93,67,
    1,16, 0x81, 0x82, 0x83, 0x91,76, 0x92,64, 0x93,61, 2,33, 0x80, 0x90,57, 2,33, 0x81, 0x82, 0x83,
    0x80, 0x90,78, 0x91,62, 0x92,50, 0x93,57, 3,50, 0x80, 0x82, 0x90,79, 0x92,52, 1,16, 0x80, 0x81,
    0x82, 0x90,81, 0x91,62, 0x92,54, 2,33, 0x83, 0x82, 0x92,57, 0x93,54, 2,33, 0x80, 0x81, 0x82, 0x83,
    0x90,76, 0x91,61, 0x92,57, 0x93,64, 2,14, 0x82, 0,18, 0x92,55, 1,16, 0x80, 0x81, 0x83, 0x82,
    0x90,78, 0x91,63, 0x92,54, 1,16, 0x80, 0x81, 0x82, 0x90,79, 0x91,64, 0x92,52, 0x93,59, 2,33,
    0x82, 0x92,50, 1,16, 0x80, 0x82, 0x90,79, 0x92,49, 1,16, 0x80, 0x81, 0x83, 0x82, 0x90,78, 0x91,62,
    0x92,50, 0x93,57, 2,33, 0x80, 0x81, 0x82, 0x83, 0x90,73, 0x91,57, 0x92,52, 0x93,55, 2,33, 0x80,
    0x81, 0x82, 0x83, 0x90,74, 0x91,59, 0x92,47, 0x93,54, 2,33, 0x80, 0x81, 0x82, 0x83, 0x90,79, 0x91,62,
    0x92,43, 0x93,59, 1,16, 0x82, 0x92,40, 1,16, 0x80, 0x81, 0x83, 0x82, 0x90,78, 0x91,57, 0x92,45,
    0x93,62, 2,33, 0x80, 0x81, 0x83, 0x90,79, 0x91,57, 0x93,62, 2,33, 0x82, 0x80, 0x81, 0x83, 0x90,45,
    0x91,81, 0x92,64, 0x93,61, 2,33, 0x80, 0x90,57, 1,16, 0x80, 0x90,55, 1,16, 0x81, 0x82, 0x83,
    0x80, 0x90,78, 0x91,62, 0x92,50, 0x93,57, 3,50, 0x80, 0x82, 0x90,79, 0x92,52, 1,16, 0x80, 0x81,
    0x82, 0x90,81, 0x91,62, 0,5, 0x92,54, 2,27, 0x82, 0x83, 0x92,57, 0x93,54, 2,33, 0x80, 0x81,
    0x82, 0x83, 0x90,76, 0x91,61, 0x92,57, 2,33, 0x93,55, 1,16, 0x80, 0x81, 0x83, 0x90,78, 0x91,63,
    0x93,54, 1,16, 0x80, 0x81, 0x82, 0x83, 0x90,79, 0x91,64, 0x92,59, 0x93,52, 2,33, 0x83, 0x93,50,
    1,16, 0x80, 0x83, 0x90,79, 0x93,49, 1,16, 0x80, 0x81, 0x82, 0x83, 0x90,78, 0x91,62, 0x92,50, 0x93,57,
    2,33, 0x80, 0x81, 0x82, 0x83, 0x90,73, 0x91,57, 0x92,55, 0x93,52, 2,33, 0x80, 0x81, 0x82, 0x83, 0x90,74,
    0x91,59, 0x92,54, 0x93,47, 2,140, 0x80, 0x81, 0x82, 0x83, 0x90,79, 0x91,62, 0x92,43, 0x93,59, 1,84,
    0x82, 0x92,40, 1,84, 0x80, 0x81, 0x83, 0x82, 0x90,78, 0x91,57, 0x92,45, 0x93,62, 3,232, 0x80, 0x81,
    0x83, 0x90,76, 0x91,55, 0x93,61, 3,232, 0x82, 0x80, 0x81, 0x83, 0x90,38, 0x91,74, 0x92,54, 0x93,50,
7,208, 0x81, 0x82, 0x80, 0x83, 0xf0};
// This score contains 1004 bytes, and 4 tone generators are used.


// Playtune bytestream for file "match_01.mid" created by MIDITONES V1.6 on Sat Nov 16 13:07:41 2013
// command line: c:\data\projects\midi\miditones\lcc\miditones match_01
const byte PROGMEM match_sound [] = {
    0x90,109, 0,10, 0x91,110, 0,36, 0x80, 0x90,111, 0,29, 0x81, 0,1, 0x91,112, 0,43, 0x80,
    0,17, 0x81, 0xf0};
// This score contains 24 bytes, and 2 tone generators are used.

// Playtune bytestream for file "light_01.mid" created by MIDITONES V1.6 on Sat Nov 16 13:07:35 2013
// command line: c:\data\projects\midi\miditones\lcc\miditones light_01
const byte PROGMEM light_sound [] = {
    0,14, 0x90,72, 0,60, 0x91,75, 0,157, 0x80, 0,10, 0x90,79, 0,71, 0x81, 0,34, 0x91,84,
    0,129, 0x80, 0,9, 0x90,87, 0,182, 0x81, 0,43, 0x80, 0xf0};
// This score contains 35 bytes, and 2 tone generators are used.


//-----------------------------------------------------------------------------------------------------
// info about the LEDs
//
//   The arrays could be reconfigured as one array of structures, but that
//   generates less efficient code for an 8-bit processor than byte arrays.
//-----------------------------------------------------------------------------------------------------

const byte led_port[9] = {ADDRD, ADDRD, ADDRD, ADDRD, ADDRD, ADDRB, ADDRB, ADDRB, ADDRB};
const byte led_mask[9] = {1<<LED0, 1<<LED1, 1<<LED2, 1<<LED3, 1<<LED4, 1<<LED5, 1<<LED6, 1<<LED7, 1<<LED8};
boolean led_on[9];
    
byte led_lighting = 0xff;	  // which led we're currently lighting, ie is gradually getting brighter
byte lighting_skips = 0;	  // how many cycles to skip for the one that is lighting
byte lighting_skipcount = 0;  // count the skips for the led that is lighting

// parameters that control the lighting algorithm

#define ON_TIME 5	// milliseconds of on time in a minor cycle (for maximum brightness)
#define OFF_TIME 10	// milliseconds of off time in a minor cycle (for maximum brightness)
#define CYCLES_PER_BRIGHTNESS_INCREMENT 5	//number of minor cycles before next brightness increment
#define MAX_BRIGHTNESS_CYCLE_SKIPS 10  // how many minor cycles to skip for minimum brightness

// The time it takes to light a candle is approximately
// (ON_TIME+OFF_TIME) * CYCLES_PER_BRIGHTNESS_INCREMENT * MAX_BRIGHTNESS_CYCLE_SKIP
// (5+10) * 5 * 10 =  3/4 second

//-----------------------------------------------------------------------------------------------------
// Do one cycle of lighting LEDs for a period of time proportional to their brightness.
//
// We have to do this manually -- not with PWM output from a timer --
// because both timers are in use for playing music.
//
// We also check if the pushbutton is firmly pushed, and return true if so.
//-----------------------------------------------------------------------------------------------------

boolean light_cycle () {
    byte led;
    boolean pushbutton_warned;
    
    pushbutton_warned = BUTTON_DOWN;
    
    for (led=0; led<9; ++led) {
        // turn on LEDs that have any brightness at all.
        // skip the one that's lighting if its skipcount is non-zero
        if (led_on[led]) {
            if (led==led_lighting && lighting_skipcount>0) {
                if (++lighting_skipcount >= lighting_skips)
                    lighting_skipcount = 0;  // end of skips: will light next time
            }
            else {
                PORT(led_port[led]) |= led_mask[led];  // turn it on
                if (led==led_lighting) lighting_skipcount=1; // start skips again
            }
        }
    }
    tune_delay(ON_TIME);	// delay for on time
    
    for (led=0; led<9; ++led)
        if (led_on[led])  PORT(led_port[led]) &= ~led_mask[led];  // turn it off
    tune_delay(OFF_TIME);	//delay for off time
    
    // if the pushbutton is still on after the delays, it's debounced and really pushed
    return pushbutton_warned && BUTTON_DOWN;
}

//-----------------------------------------------------------------------------------------------------
// flicker-light one led, leaving others on that were on
// return true if the button is being pushed
//-----------------------------------------------------------------------------------------------------

boolean light (byte led) {
    int i;
    
    tune_playscore (light_sound);
    led_lighting = led;
    led_on[led] = true;
    lighting_skips = MAX_BRIGHTNESS_CYCLE_SKIPS; // start with maximum minor cycle skips
    lighting_skipcount = 0;
    do {
        for (i=0; i<CYCLES_PER_BRIGHTNESS_INCREMENT; ++i) 
            if (light_cycle()) return true;  // stop if button pushed
        --lighting_skips; //gradually reduce minor cycle skips
    } while (lighting_skips>0);
    led_lighting = 0xff; // nothing lighting up now
    return false;
}

//-----------------------------------------------------------------------------------------------------
// wait while keeping the lights lit
//-----------------------------------------------------------------------------------------------------

boolean delay_with_lights (int cycles) {
// callers should use this macro, so that division is done at compile time
    #define MSEC_TO_CYCLES(msec) msec/(ON_TIME+OFF_TIME)
    while (cycles--) if (light_cycle()) return true;
    return false;
}


//-----------------------------------------------------------------------------------------------------
//                    **************   main logic   ***********************
//-----------------------------------------------------------------------------------------------------

int main (void) {
    byte led, night, i;
    boolean center_shamas; // candle lighter is LED 8, not LED 4
    unsigned long sleepcounter;
    
    // Clear watchdog timer-- this can prevent several things from going wrong.
    MCUSR &= 0xF7;		//Clear WDRF Flag
    WDTCSR = 0x18;		//Set stupid bits so we can clear timer...
    WDTCSR = 0x00;
    MCUCR &= 0xCF;		//Disable sleep mode

    init_timers();      // initialize both timers, for music and for delays
    
    // configure I/O ports

restart:
    DDRA = (1<<SPEAKER2)+(1<<SPEAKER3);    
    PORTA = 0;
    DDRB = (1<<LED5)+(1<<LED6)+(1<<LED7)+(1<<LED8);
    PORTB = (1<<BUTTON);     // pushbutton has a pullup
    DDRD = (1<<SPEAKER1)+(1<<SPEAKER2)+(1<<LED0)+(1<<LED1)+(1<<LED2)+(1<<LED3)+(1<<LED4);
    PORTD = 0; 

    for (led=0; led<9; ++led)
        led_on[led] = false;
    for (i=0; i<NUM_CHANS; ++i)
        playing[i] = false;
        
    while (BUTTON_DOWN) ; // wait for button to be released
    

#if 0	//	test all notes
{byte note;
    for (note=21; note<109; ++note) {
        tune_playnote(0, note);
        tune_delay(200);
        tune_stopnote(0);
        tune_delay(50);
    }
    tune_delay(1000);
}
#endif
#if 0	// test wait timer accuracy
{byte i;
    for (i=0; i<10; ++i){
        tune_playnote(0,60);
        tune_delay(100);
        tune_stopnote(0);
        tune_delay(900);
    }
}
#endif

    // the current night is stored in non-volatile EPROM; go to the next night.

    night = eeprom_read_byte((byte *)EEPROM_addr);
    if (night > 7) night = 7;	// uninitialized? start with 0
    if (++night > 7) night = 0; // next night
    eeprom_write_byte((byte *)EEPROM_addr,night);
        
    // light the shamas
    
    center_shamas = true;	// no longer an option: PORTA & (1<<JMP1); // jumper 1 means shamas is on the end
    tune_delay(2000);		// wait two seconds
    led = center_shamas ? 4 : 8;
    for (i=0; i<4; ++i) {  // simulate striking a match, sort of
        tune_playscore (match_sound);
        PORT(led_port[led]) |= led_mask[led];  // turn it on
        while (tune_playing) ;
        PORT(led_port[led]) &= ~led_mask[led];  // turn it off
        tune_delay (300);
    }
    if (light(led)) goto restart;	// now light it
    
    // light one or more other candles, depending on the night
    
    led = 7-night;  // start with the newest light
    if (center_shamas && led>3) ++led;
    do {
        delay_with_lights (MSEC_TO_CYCLES(1500));
        if (light (led++)) goto restart;
        if (center_shamas && led==4) ++led;
    } while (night--);
    
    // play the music while keeping the lights lit
    
    delay_with_lights (MSEC_TO_CYCLES(2000));
    tune_playscore (score);	
    while (tune_playing) {
        if (light_cycle()) {
            tune_stopscore();
            goto restart;
        }
    }
    
    // Wait half an hour with the lights on, then turn off
    
    #define SLEEP_MINUTES 30
    for (sleepcounter = SLEEP_MINUTES*60UL*1000UL/(ON_TIME+OFF_TIME);	sleepcounter; --sleepcounter) {
        if (light_cycle()) goto restart;
    }
    PORTA = 0;
    PORTB = 0;
    PORTD = 0;
    DDRD = 0;
    DDRB = 0;
    DDRA = 0;
    MCUCR &= 0xCF;	// ensure power-down mode
    MCUCR |= 0x30;	// enable sleep & power-down modes
    asm("sleep");	//Go to sleep!
}

