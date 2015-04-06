      A polyphonic music generator for the ATtiny, 
  specifically for the Evil Mad Scientist Hanukkah menorah

PLAYTUNE interprets a sequence of simple commands ("note on", "note off", and "wait")
that represents a polyphonic musical score without volume modulation. I've posted a
general interrupt-driven version for Arduinos at https://github.com/LenShustek/arduino-playtune.
There is a companion program for turning MIDI files into score commands at 
https://github.com/LenShustek/miditones.

This much-simplified version of PLAYTUNE for the ATtiny processor uses polling 
instead of timer-driven interrupts, avoids multiplication or division at runtime, 
and mostly does 8-bit arithmetic. The frequencies are less accurate, but we can
play more simultaneous tones than the number of hardware timers.

I wrote this version for the Evil Mad Scientist menorah kit
  http://www.evilmadscientist.com/2009/new-led-hanukkah-menorah-kit/
and posted on their forum about it.
  http://www.evilmadscientist.com/2013/musical-menorah/
I used the free Atmel "Studio" development environment, although it might
also be possible to use the Arduino IDE. I'm posting this mostly as a 
source of ideas; you will have to make changes for other hardware environments.

-- Len
