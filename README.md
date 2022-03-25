# The Challenge
The challenge is based on the Hack CPU of the Nand2Tetris project at https://www.nand2tetris.org/
specifically you want to go over the Hack architeture and ISA here: 
https://www.coursera.org/learn/nand2tetris2/lecture/rlxAQ/unit-0-3-the-hack-computer-and-machine-language

You will be given a basic CPU design embedded in a full system containing:


- VGA module 		– For displaying Part of the RAM 24 lines, for debug and review.
- RAM module 		– like any modern based CPU system for memory store & load.
- ROM module 		– to contain the software the CPU should run.
- Performance counter 	– to analyze the software runtime.
- Hex display 		– to write the Hex runtime of the system on the FPGA 7Seg display.


The challenge is to make the system written in the Verilog files  to run the ROM software as fast as possible – the fastest code which runs without changing the functionality of the software nor the runtime checker wins 😊.
The system will work as classic computing machine, reading the ROM from the first instruction decoding it and finely executing it before heading to the next instruction. (First instruction 16’h0).

The system halts when the CPU reaches line 4095 (the last one). Thus, you must not add an infinite loop to your code, or else the performance counter will not stop. Simply ensure the CPU will reach the last line natively.
Block Diagram Of the System
 
 ![image](https://user-images.githubusercontent.com/94902187/159119399-dae95d9c-5831-47f6-9cd0-7e10e2acc34a.png)

 
notice the Performance counter & seven segment & VGA modules.
The performance counter is for analyzing the run time of the code inside the machine – do not modify this block.
The seven segment is to represent the runtime in Hex Decimal clock cycles count. Again, don’t modify this block. Same for the VGA.

You may rewrite almost anything, including

- alu.sv
- ram.sv  
- rom.sv 
- cpu.sv 

for the following System Verilog file definitions.sv make sure you modify only what is related to the CPU / ROM / RAM blocks and not the seven segment nor the runtime checker or the VGA.

Few Guidelines:

1.	Don’t touch the checking Verilog and its clock 50 MHz, for the rest of the design you can split the clock and change it as you like, I’ll suggest do it carefully.
2.	Don’t change the code sequence – make sure no jumps or any other hardware preventing the CPU to run every line of the software code.
3.	Make sure the CPU isn’t “fixed” for the given software and can be Generalized for any HACKcpu software.

Your CPU will be tested by running a program similar to the given source software.
After programming the device, you'll see on the screen the first few words of the data memory, in binary and in hex. In the bottom of the screen, you'll see a clock count (50MHz) in hex, and time count.

Seven Segment Display

In case you don't have a screen, you can see the clock count on the 7 segment displays. Switch the rightmost switch to get the 3 least significant hex digits of the clock count. The next switch will show the next 3 digits. There are 8 digits in total.
To avoid confusion, each switch also turns on an LED. The rightmost LED corresponds to the 3 least significant digits, the second LED corresponds to the next 3 digits, and so on.
When the CPU finishes executing the program, the leftmost LED will turn on.


The modules mentioned above : 
ROM
The ROM is slightly different from the one described in the course book. It contains 4096 words, 16 bit each, and is synchronous, i.e.:
RAM
The RAM is also different from the course. It is the same dimensions as the ROM, and is synchronous too. Writing is defined in this way:
if (writeEnable(t) == 1)
    RAM[address(t)](t+1) = OutM(t)
The RAM is implemented in dual channel mode. The CPU interacts with the first channel only. The second channel is used for the VGA and should not concern you.
VGA
The screen shows 12 lines, 2 words per line, begining at offset 0 in the RAM.
CPU
The CPU's functionality is exactly as described in the nand2tetris course book. But as we decided to use synchronous RAM, input data from it will lag until the next clock cycle. To workaround this issue, the CPU stalls every second clock cycle. By stalling we mean the A, D, and PC registers are not updated. This way, when the CPU is not stalled, the input data is up to date.
The CPU block diagram, as described in the course book:
 
 ![image](https://user-images.githubusercontent.com/94902187/159119425-ab7bc7e6-ef86-4a48-b2f9-62794634f5e5.png)

![image](https://user-images.githubusercontent.com/94902187/159119483-143ed737-cbf7-4717-b73b-3e83f02bfdc8.png)


 
Run a Sample Program
The Sample Programs folder contains some assembly programs you can run on the CPU. These programs have already been assembled for you and the assembled versions are in the hack folder. Intel Hex versions were also prepared for you in the hex folder.
Run Your Own Program
You can also write your own programs, according to the specification of the nand2tetris course: https://www.nand2tetris.org/.
In order to run your program, you'll have to generate the hex file yourself. Follow these steps:
•	Download the official software suite from https://www.nand2tetris.org/software.
•	Assemble your program using the assembler in the suite.
•	Copy the resulting binary code into the hack directory.
•	Run the script at python/hack_to_hex.py. Your program will be converted to hex format and put in the hex directory.
•	Write the resulting hex file to the memory using the ISMCE as explained above.
Rtl viewer can shoe the block diagram of the system: 
 
 ![image](https://user-images.githubusercontent.com/94902187/159119447-2c1b9dd8-5783-4d79-92cb-ff5f1a8fdacf.png)

