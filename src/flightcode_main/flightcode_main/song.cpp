// -------------------------------------------------
// Copyright (c) 2024 HiBit <https://www.hibit.dev>
// -------------------------------------------------

#include "pitches.h"
#include "songs.h"
#include <Arduino.h>

int melody[] = {
  NOTE_B4, REST,
  NOTE_FS4, NOTE_FS4, NOTE_B4, NOTE_FS4, NOTE_E4, REST, NOTE_B3,

  NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_B3, NOTE_B3, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_B3, NOTE_B3,
  NOTE_CS4, NOTE_CS4, NOTE_CS4, NOTE_CS4, NOTE_AS3, NOTE_AS3, NOTE_CS4, NOTE_CS4, NOTE_CS4, NOTE_CS4, NOTE_AS3, NOTE_B3,
  NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_B3, NOTE_B3, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_B3, NOTE_B3,
  NOTE_CS4, NOTE_CS4, NOTE_CS4, NOTE_CS4, NOTE_AS3, NOTE_AS3, NOTE_CS4, NOTE_CS4, NOTE_CS4, NOTE_CS4, NOTE_AS3,

  NOTE_B4, NOTE_A4, NOTE_G4, NOTE_D4, NOTE_FS4, NOTE_E4, NOTE_B4,
  NOTE_B4, NOTE_A4, NOTE_G4, NOTE_D4, NOTE_FS4, NOTE_AS4,

  REST, NOTE_E4, NOTE_FS4, NOTE_E4, NOTE_D4, NOTE_B3,
  NOTE_D4, NOTE_E4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_E4, NOTE_B4,
  REST, NOTE_E4, NOTE_FS4, NOTE_E4, NOTE_D4, NOTE_B3,
  NOTE_D4, NOTE_E4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_E4, NOTE_B4, REST, NOTE_B3, NOTE_B3, NOTE_B3,
  NOTE_D4, NOTE_CS4, NOTE_B3, NOTE_FS3, NOTE_E3, NOTE_FS3, NOTE_FS4, NOTE_B4, NOTE_FS4, NOTE_E4,
  
  REST
};

int durations[] = {
  4, 2,
  4, 8, 4, 8, 4, 2, 8,

  4, 4, 8, 8, 2, 8, 4, 4, 8, 8, 2, 8,
  4, 4, 8, 8, 2, 8, 4, 4, 8, 8, 2, 8,
  4, 4, 8, 8, 2, 8, 4, 4, 8, 8, 2, 8,
  4, 4, 8, 8, 2, 8, 4, 4, 8, 8, 2,

  2, 2, 2, 4, 1, 1, 8,
  2, 2, 2, 4, 1, 1,

  2, 2, 8, 8, 8, 2,
  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 2,
  2, 2, 8, 8, 8, 2,
  8, 8, 8, 8, 8, 8, 2, 2, 8, 8, 8,
  2, 2, 2, 8, 4, 4, 8, 4, 8, 2,

  1
};

void whatisthatmelody()
{
  int size = sizeof(durations) / sizeof(int);

  for (int note = 0; note < size; note++) {
    //to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int duration = 1000 / durations[note];
    tone(BUZZER_PIN, melody[note], duration);

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);

    //stop the tone playing:
    noTone(BUZZER_PIN);
  }
}