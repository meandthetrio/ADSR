### 1/6/25 10:57am
Cuz revamped "CHORUS" section and changed it to Modulation section since added a TAPE setting.
- Modulation section = CHORUS and TAPE
- CHORUS = DEPTH & SPEED
- TAPE = DROPOUTS & RATE
- encoder_r toggles between the two settings
- It seems like if modulation is active and switch to TAPE - CHORUS settings remain and both can be used.
- If Tape settings are set first, and switch to CHORUS, Chorus seems like the only active effect. (not sure if we should fix this or just leave it)

Revamped SATURATION setting
- Tape = SAT and BUMP. Sat is Edge's original setting and BUMP mimics a transformer being pushed at the output stage.
- BIT = RESOULTION and SAMPLERATE. RESO has three different bit modes called CRUSH, STATIC, and HISS for more/less subtle changes.

### 1/5/25 2:57pm
Cuz revamped entire Reverb Algo
- Added SHM - octave up feedback loop
- WET - goes 100% wet at 100% fader
- DCY - Infinite Freeze at 100%
- DMP - Max HIGHs at 0% - massive HIGH cut on a quick sweep when fader increases.
- \\PRE - At 0% PRE fader is bypassed, 100% should be 1000m delay (1sec) - at that logic 50% on fader should be 500ms.

*****
- Edited Drawfader section to give it "mixer" style
- Updated Green LED blinking logic on some screen where Preview/Play is available
- Edited "LOAD TO" screen to horizontal triangle view
- Cleaned up text thickness in the FX engines

### 2/3/26 2:13am

- Load a constant/repeating/solid/loud sample from SD to PERFORM
- Left Encoder Rotation selects which section is highlighted
- Hold Shift the Rotate L encoder to highlight a parameter
- rotate Right encoder to change value of highlighted parameter
- release shift, move encoder Left rotation to highlight a different section

-AMP
	Attack = A
	Release = R
	
	decay and sustain not yet working

-FLT
	C = cutoff
	R = Resonance

-FX
	S = Tape saturation
	C = Chorus
	D = Ping Pong Delay
	R = Reverb





### 2/2/26 2:08am
# Post TUNE/ADSR removal updates

___CREATED A NEW BRANCH TO WORK ON THE BASICS OF THE BAKE FEATURE, MAINLY CREATING MULTIPLE SAMPLES FROM A SINGLE SAMPLE, REPITCHING THEM AND PLACING THEM ON THEIR RELATIVE NOTE #_______

BUGS:
1.) THERE IS NO BAKE PROGRESS WHEN BAKING, CURRENTLY TAKES AROUND 20 SECONDS AND TRANSPORTS DIRECTLY TO PERFORM SUBMENU
2.) EVERYTHING FROM A3 AND HIGHER SOUNDS OUT OF TUNE?
3.) THE ADDED FADE INS / FADE OUTS WHICH CAN BE CONTROLLED WITH THE ENCODERS ON THE PERFORM SUBMENU WILL BE DELETED FOR SOMETHING BETTER

-Removed TUNE and ADSR SELECT screens.

-Added CONFIRM BAKE screen (ENCODER R) with YES/NO boxes from START/END SELECT.

-BAKE now uses only the selected START/END window; PERFORM shows and plays only that window.

-BAKE runs offline once and stores a per-note bank in SDRAM for C2-C4: C3 is the original window, other notes are phase-vocoder pitch-shifted with duration preserved (phase locking + transient handling to reduce artifacts).

-Perform is MIDI-only for baked content: only notes C2-C4 play from the baked bank.

-Added 5-voice polyphony for baked Perform playback.

-Added global attack/release fades in Perform (L/R encoders; Shift = fine). The waveform overlay shows the fade lines moving inward as fades increase.

# 1/1/26 EDGE


-Added ADSR select window

-Changed PodButton2 to the Save/Delete Menu

-Changed the shift button to function as a fine-adjust for moving START/END times on the START/END select screen.  I.e. HOLD SHIFT and turn encoder will move the line 1/64th of sample vs 1/32nd of sample





feature/Record2 Branch
# 1/1/2026

## Author

<!-- EDGE -->

## Description

// Added BAKE feature


// Updated main menu to Load, Record, Perform, Play. Perform is meant to be used with Midi, and Play will eventually be the sequencer.

// Add "A:Preview" to load screen with Blinking Green LED indicator to know what the file sounds like before you load it.

// Implemented wrap‑around scrolling in the LOAD menu. Turning left from the first item now jumps to the last
// item, and vice versa.

// DrawSaveScreen now shows a progress bar (based on save_frames_written / sample_length) below the dots.

//** Added DELETE function in the Shift Menu with confirmation screen.(R= YES, L=NO)

// EDGE's edit of monitoring in mono both headphones applied.

// MOVED MIC AND LINE SOURCES OUT OF SHIFT MENU AND PLACED AFTER RECORD IS SELECTED

// Merged Polyphony waveform visual for both LOAD and RECORD sections into WaveContV3 branch. (Branch of Edge's ADSR original script). This includes waveform size, color shading, and angled trim points.

// Added Mic In ** Using TRS cable TIP = LEFT(LINE IN) AND RING = RIGHT (MIC IN) responding to Daisy Seed Codec pins 16 and 17 respectively. (Final Product only needs mono cable for the Line In. Mic is wired directly to R pin 17).

// Made .wav file names smaller in LOAD menu

// cleaned up button logic (some issues with Play originally not opening Playback screen after Load or Record) - STILL MORE TO DO HERE ALWAYS ??

// Kept .wav file name in Playback screen - had to shrink down waveform and trim points slightly

// Removed DrawWavePlayMenu from objects - kept getting compile Warnings that this was declared but never actually used. This must have been something from Edge's original ADSR draw wave form script.

// Edited encoder travel on both trim points and sine wave knobs. Animations move more naturally and cover more ground with less of a turn. 1/32. Can be changed back to 1/64 if we want less traction.

// REVIST SINE WAVE SECTION - DO WE REALLY NEED THIS??

// Added normalization to both playback waveform and recorded playback waveform - (increase waveform size regardless of signal strength. Affects only visual and not actual playback volume or gain).

// Add 4 bar pre roll with old school movie animation
//** NEED TO ADD OPTION TO DISABLE THIS FEATURE

// Added SHIFT key menu with SAVE 
//** files current save with an auto-generated file name, but cannot be reloaded. Need to fix this
//** ADD delete function
//** add naming capabilities for saved .wav files

// Added kSaveChunkFrames = 2048 to save files faster to the SD card. Originally took a minute or so. (still takes too long - need to fix this)

// BUGS: 
// **CANNOT ACCESS SINE WAVE AGAIN AFTER BACKING OUT INITIALLY - COULD BE PROBLEMATIC IF NEEDING TO REVIST, THE ONLY WAY TO ACCESS IT AGAIN IS TO RE-RECORD SOMETHING
// ** SD CARD DOESN'T LOAD IF REMOVED WITHOUT REFLASHING. I tested removing the SD card after initially loading files, but get a "NO WAV FILES" error. Even when reinserting the SD card and pressing LOAD, the error persists. The only way to reload is to flash or reset the seed. (very bad for users).
//** SD card removal feature revoked. Only have internal SD for saving patches.




