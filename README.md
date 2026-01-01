# WaveContV3 Branch

## Author

<!-- CUZ -->

## Description

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





