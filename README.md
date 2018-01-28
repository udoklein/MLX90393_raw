# MLX90393_raw
Arduino library for the MLX90393 magnetometer

This is a simple and lightweight Arduino library for accessing the
MLX90393 magnetometer.

## Known Issues
If you get compiler errors with regards to name mangling / ABI
version then you need to add the following to your
`platform.txt` or `platform.local.txt` file:

    compiler.c.extra_flags=-fabi-version=0
    compiler.cpp.extra_flags=-fabi-version=0

The issue is that according to the [manual](https://gcc.gnu.org/onlinedocs/libstdc++/manual/abi.html)
gcc 4.* defaults to the (buggy) ABI version 2.
Setting version=0 defaults to the most up to date ABI version. In theory
this can break code. However in practice Arduino
will recompile everything everytime anyway thus it does not break
anything and just avoid the nasty name mangling bug.


## Credits
This library was inspired by the following project by Ted Yapo:

[https://github.com/tedyapo/arduino-MLX90393](https://github.com/tedyapo/arduino-MLX90393)

His library provides more features than mine. In particular it
is capable to deliver readouts converted into Teslas. His library
also supports operation without DRDY pin.

My library is more lightweight because I do not require these
advanced capabilities. If you need more than my library
provide please do not ask for extensions, use Ted's library.