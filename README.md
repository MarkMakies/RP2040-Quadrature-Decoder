# RP2040-Quadrature-Decoder
Designed specifically to measure velocity up to 22kHz on two channels on a mobile platform.

I borrowed from many of the micropython designs already out there.  For me
many couldn't measure velocity well, but total counts.  The outside loop just needs to 
QueryEncoders() every 20ms - exactly (as much as possible), use a timer.  You also need to ensure a gc.collect() doesn't occur at the wrong
time.  I schedule it in the main loop to be sure and predictable.

I've tested it reliably up to 22kHz.  My biggest problem was noisy signals, make sure
you condition / shield them well.

It works very well for me as inputs into a PID controller, I hope it helps you.
