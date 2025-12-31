# WaveContV3 Branch

## Author

<!-- CUZ -->

## Description

// Merged Polyphony waveform visual for both LOAD and RECORD sections into WaveContV3 branch. (Branch of Edge's ADSR original script). This includes waveform size, color shading, and angled trim points.

// Added Mic In ** Using TRS cable TIP = LEFT(LINE IN) AND RING = RIGHT (MIC IN) responding to Daisy Seed Codec pins 16 and 17 respectively.

// Made .wav file names smaller in LOAD menu

// cleaned up button logic (some issues with Play originally not opening Playback screen after Load or Record) - STILL MORE TO DO HERE ALWAYS ??

// Kept .wav file name in Playback screen - had to shrink down waveform and trim points slightly

// Removed DrawWavePlayMenu from objects - kept getting compile Warnings that this was declared but never actually used. This must have been something from Edge's original ADSR draw wave form script.

// Edited encoder travel on both trim points and sine wave knobs. Animations move more natually and cover more groun with less of a turn. 1/32. Can be changed back to 1/64 if we want less traction.

// REVIST SINE WAVE SECTION - DO WE REALLY NEED THIS??

// Added normalization to both playback waveform and recorded playback waveform - (increase waveform size regardless of signal strength. Affects only visual and not actual playback volume or gain).

// Add 4 bar pre roll with old school movie animation

// Added SHIFT key menu with SAVE, LINE IN, and MICROPHONE
// **can now select between which audio source is being recorded
// **this essentially selects either L or R from the audio codec pin 16/17
// ** THIS NEEDS TO BE INVESTIGATED FURTHER to possible mute channels when recording.
// ** Currently in RECORD ANIMATION - both can be monitored at the same time (which is actually nice)
// ** Summed to mono on the output

// BUGS: 
// **CANNOT ACCESS SINE WAVE AGAIN AFTER BACKING OUT INITIALLY - COULD BE PROBLEMATIC IF NEEDING TO REVIST, THE ONLY WAY TO ACCESS IT AGAIN IS TO RE-RECORD SOMETHING
// ** SD CARD DOESN'T LOAD IF REMOVED WITHOUT REFLASHING. I tested removing the SD card after initially loading files, but get a "NO WAV FILES" error. Even when reinserting the SD card and pressing LOAD, the error persists. The only way to reload is to flash or reset the seed. (very bad for users).





