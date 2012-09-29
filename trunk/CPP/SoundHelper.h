// A fun sketch to demonstrate the use of the Tone library.

// To mix the output of the signals to output to a small speaker (i.e. 8 Ohms or higher),
// simply use 1K Ohm resistors from each output pin and tie them together at the speaker.
// Don't forget to connect the other side of the speaker to ground!

// You can get more RTTTL (RingTone Text Transfer Language) songs from
// http://code.google.com/p/rogue-code/wiki/ToneLibraryDocumentation

#define OCTAVE_OFFSET 0


#ifndef SoundHelper_h
#define SoundHelper_h

void playRtttl(char *p);
void playStartSequence();
void initSpeaker(uint8_t tonePin);
void play(uint16_t freq, uint32_t dur);
void playRtttl(char *p);
void playAlarm();
void longBeep();
void shortBeep();
void resetBeepState();
void longBeepAsync();

#endif
