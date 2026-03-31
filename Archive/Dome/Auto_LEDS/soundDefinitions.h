#include <ArduinoPins.h>
#include <FatReader.h>
#include <FatStructs.h>
#include <mcpDac.h>
#include <SdInfo.h>
#include <SdReader.h>
#include <WaveHC.h>
#include <Wavemainpage.h>
#include <WavePinDefs.h>
#include <WaveUtil.h>

#ifndef soundDefinitions_h
#define soundDefinitions_h

SdReader card;
FatVolume vol;
FatReader root;
FatReader file;
WaveHC wave;

void playfile(char *name) {
  if (wave.isplaying) {
    //already playing something, so stop it!
      wave.stop();
  }
  //look in root directory and open the file
  if (!file.open(root, name)) {
    putstring("Couldn't Open File"); Serial.print(name); return;
  }
  if (!wave.create(file)) {
    putstring_nl("Not a valid WAV"); return;
  }

  wave.play();
}

void playcomplete(const char *name) {
  //call out helper to find and play this name
  playfile(name);
  while (wave.isplaying) {
    //do nothing  
  }
    //now done playing
}

int button1 = 1;
int button2 = 2;
int button3 = 3;
int button4 = 4;

int button1state = 0;
int button2state = 0;
int button3state = 0;
int button4state = 0;

void angry() {
  playcomplete("angry.wav");
}

void giddy() {
  playcomplete("giddy.wav");
}

void happyStatic() {
  playcomplete("happyStatic.wav");
}

void yes() {
  playcomplete("yes.wav");
  psiYes();
}

void yahoo() {
  playcomplete("yahoo.wav");
}
void firmware() {
  playcomplete("firmware.wav");
}

void headturn() {
  playcomplete("headturn.wav");
}

void poke() {
  playcomplete("poke.wav");
}

void newFirmware() {
  playcomplete("newFirmware.wav");
}

void proximity() {
  playcomplete("proximity.wav");
}

void readyy() {
  playcomplete("readyy.wav");
}

void upset() {
  playcomplete("upset.wav");
}

void waggle() {
  playcomplete("waggle.wav");
}

#endif
