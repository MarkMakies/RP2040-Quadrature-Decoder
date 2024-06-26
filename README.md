# RP2040-Quadrature-Decoder
Designed specifically to measure the velocity of two geared DC motors with 100 CPT encoders.  Up to 22kHz inputs on two channels, which corresponds to a vehicle speed of around 10 m/s for my application.

I borrowed from many of the micropython designs already out there.  However, many couldn't measure velocity well, but total counts.  The outside loop just needs to 
QueryEncoders() every 20ms - exactly (as much as possible), use a timer.  You also need to ensure a gc.collect() doesn't occur at the wrong time.  I schedule it in the main loop to be sure and predictable.

I've tested it reliably up to 22kHz.  My biggest problem was noisy signals, make sure you condition / shield / terminate them well.

It works very well for me as inputs into a PID controller for a mobile robotic platform, I hope it helps you.
