dcf77
======

_dcf77_ contains a state machine to identify a [DCF77][] signal fed in to a
regular GPIO pin. It also contains a decoder for the received signal into
date/time values.

The driver is hardware independent and can e.g. be used with microcontrollers and
any implementation of an [embedded-hal][] crate.

The input signal of the dcf77 statemachine shall look like this

```text
DCF77 RF signal:

   |     |||||||||||||||||||||||||||||||||||||        |||||||||||||||||||||||||||||||    |||
   |     |||||||||||||||||||||||||||||||||||||        |||||||||||||||||||||||||||||||    |||
   |     |||||||||||||||||||||||||||||||||||||        |||||||||||||||||||||||||||||||    |||
...|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||...

DCF77 receiver output that can be fed into this state machine:
     ___                                      _______                                ___
..._|   |____________________________________|       |______________________________|   |___...
    100ms               900ms                  200ms              800ms             100ms      
```

[embedded-hal]: https://github.com/japaric/embedded-hal.git
[DCF77]: https://en.wikipedia.org/wiki/DCF77

License
-------

[0-clause BSD license](LICENSE-0BSD.txt).
