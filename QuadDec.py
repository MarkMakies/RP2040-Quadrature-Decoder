################################################################################
# Roverling Mk II - Quadrature Decoder
# Count transitions using RP2040 PIO based state machine, no CPU cycles and then
# use timer based interrupt to read accumulated counts, aka velocity. 
# Motor quadrature optical encoder: 100 CPT, 17.83:1, 1783 counts per revolution
# v102 Apr 2024 improved stability, more accurate timing, use of different emiters
#      tested up to 22kHz, 20ms sample time (full free wheeling speed 11 m/s)

from machine import Pin
from rp2 import asm_pio, StateMachine

REncApin = Pin(6, Pin.IN)
REncBpin = Pin(7, Pin.IN)  
LEncApin = Pin(8, Pin.IN)
LEncBpin = Pin(9, Pin.IN)  

LE: int = 0
RE: int = 0
LO: int = 0
RO: int = 0
prevLE: int = 0
prevRE: int = 0
Lfirst, Rfirst = True, True

@asm_pio()
def encoder():       
    # adapted from https://github.com/adamgreen/QuadratureDecoder
    # In C/C++ can define base, which needs to be 0 for jump table to work
    # in python not working, but padding out to exactly 32 instructions fixes it
    # 16 element jump table based on 4-bit encoder last state and current state.

    jmp('delta0')     # 00-00
    jmp('delta0')     # 00-01
    jmp('delta0')     # 00-10
    jmp('delta0')     # 00-11

    jmp('plus1')      # 01-00
    jmp('delta0')     # 01-01
    jmp('delta0')     # 01-10
    jmp('minus1')     # 01-11

    jmp('minus1')     # 10-00
    jmp('delta0')     # 10-01
    jmp('delta0')     # 10-10
    jmp('plus1')      # 10-11

    jmp('delta0')     # 11-00
    jmp('delta0')     # 11-01
    jmp('delta0')     # 11-10
    jmp('delta0')     # 11-11
                        
    label('delta0')     # Program actually starts here.
    mov(isr, null)      # Make sure that the input shift register is cleared when table jumps to delta0.
    in_(y, 2)           # Upper 2-bits of address are formed from previous encoder pin readings Y -> ISR[3,2]
    mov(y, pins)        # Lower 2-bits of address are formed from current encoder pin readings. PINS -> Y
    in_(y, 2)           # Y -> ISR[1,0]
    mov(pc, isr)        # Jump into jump table which will then jump to delta0, minus1, o/ COUNTS_PER_M / Period_sr plus1 labels.

    label('minus1')
    jmp(x_dec,'output') # Decrement x
    jmp('output')

    label('plus1')
    mov(x, invert(x))   # Increment x by calculating x=~(~x - 1)
    jmp(x_dec,'next2')
    label('next2')
    mov(x, invert(x))

    label('output')
    mov(isr, x)         # Push out updated counter.
    push(noblock)
    jmp('delta0')

    nop()                #need to pad out to exactly 32 instructions
    nop()
    nop()

@micropython.viper
def twos_comp(v, b: int) -> int:
    val = int(v)
    bits = int(b)
    if (val & (1 << (bits - 1))) != 0:          # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)                 # compute negative value
    return val                                  # return positive value as is

@micropython.native
def QueryEncoders() -> int:
    global LE, RE, LO, RO
    global prevLE, prevRE
    global Lfirst, Rfirst

    # if taking first sample, more reliable timing, however reading from 2 periods old
    # if taking last: up to 30 pops before clear @ 45us each (22kHz) delays reading by 1.3m/s
    # which means that value will be out by 0-3% depeding on speed

    if LeftMotorEncoder.rx_fifo():
        LE = twos_comp(LeftMotorEncoder.get(), 32)      # pop first value & convert to int32
        while LeftMotorEncoder.rx_fifo():               # clear remaining
            LeftMotorEncoder.get()                 
        LO = prevLE - LE                                # calc delta
        prevLE = LE
    else:
        LO = 0                                          # means no pulse & empty FIFO

    if RightMotorEncoder.rx_fifo():
        RE = twos_comp(RightMotorEncoder.get(), 32)
        while RightMotorEncoder.rx_fifo():    
            RightMotorEncoder.get()     
        RO = prevRE - RE
        prevRE = RE
    else:
        RO = 0

    if Lfirst and abs(LE) > 0:                      # ensure very first sample is ignored
        LO = 0
        Lfirst = False

    if Rfirst and abs(RE) > 0:
        RO = 0
        Rfirst = False

    return (LO, -RO)

LeftMotorEncoder = StateMachine(0, encoder, in_base=LEncApin)
RightMotorEncoder = StateMachine(1, encoder, in_base=REncApin)

LeftMotorEncoder.active(1)
RightMotorEncoder.active(1)
