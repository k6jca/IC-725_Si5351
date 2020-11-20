# IC-725_Si5351

Arduino code to setup a Silicon Labs Si5351 clock generator to create an Icom IC-725's BFO clock (whose frequency depends upon mode).

Note that although this code can also generate the IC-725's 30.72 MHz oscillator signal, I have disabled this feature and I am instead generating the 30.72 MHz signal with a VCTCXO.
