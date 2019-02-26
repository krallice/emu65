# emu65
6502 emulator running BASIC!

This is a cycle accurate 6502 vCPU emulator, with supplied bootstrap code to load and run the Emulation Enhanced Basic ROM. I tell you what, I was pretty damn excited when I first saw that EhBASIC boot line ...

```
# bin/emu65 
Loading EH BASIC into 0xC000. Maximum Bytes Loadable: 0x4000.
Loaded 0x4000 (16384) Bytes into RAM.
PC is 0xFF80. Staring Program Execution.

6502 EhBASIC [C]old/[W]arm ?

Memory size ? 1024

255 Bytes free

Enhanced BASIC 2.22

Ready

10 FOR X = 5 TO 1 STEP -1
20 PRINT "SKYNET ACTIVATING IN"; X; " ..."
30 NEXT

RUN
SKYNET ACTIVATING IN 5 ...
SKYNET ACTIVATING IN 4 ...
SKYNET ACTIVATING IN 3 ...
SKYNET ACTIVATING IN 2 ...
SKYNET ACTIVATING IN 1 ...

Ready
```
