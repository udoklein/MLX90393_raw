//
//  www.blinkenlight.net
//
//  Copyright 2018 Udo Klein
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see http://www.gnu.org/licenses/


// Attention: if you get compiler errors with regards to name mangling
//            then please have a look into the README

#include <MLX90393_raw.h>
using namespace MLX90393;

const int8_t drdy_pin = 10;
const uint32_t baudrate = 57600;

// I2C
// Arduino A4 = SDA
// Arduino A5 = SCL
typedef MLX90393_raw<0, 0, Wire, drdy_pin> MLX;


#define print(...)   Serial.print( __VA_ARGS__ )
#define println(...) Serial.println( __VA_ARGS__ )


struct output_mode_t {
    typedef enum { HEX_RAW = 0, HEX_CSV = 1, DEC_CSV = 2} style_t;

    // QUIET:        suppress status
    // NORMAL:       show status on error
    // VERBOSE:      always show status (HEX)
    typedef enum { QUIET = 0, NORMAL = 1, VERBOSE = 2, VERY_VERBOSE = 3 } verbosity_t;

    // ON_DEMAND:  reading must be explicitly triggered
    // AUTO:       any measurement will trigger output (except WOC w/o DRDY pin)
    // CONTINUOUS: read even if there might be nothing to read
    typedef enum { ON_DEMAND = 0, AUTO = 1, CONTINUOUS = 2 } read_behaviour_t;

    typedef enum { IDLE = 0, READ = 1, START_MEASUREMENT = 2, BURST = 3, WAKE_ON_CHANGE = 4 } chip_state_t;

    style_t style;
    verbosity_t verbosity;
    read_behaviour_t read_behaviour;
    uint8_t timestamps;

    chip_state_t state;
};

output_mode_t output_mode = {output_mode_t::HEX_CSV,
                             output_mode_t::NORMAL,
                             output_mode_t::ON_DEMAND,
                             output_mode_t::IDLE };

uint8_t axis_mask = AXIS_MASK::ALL;

namespace timer {
    static auto old_ms = millis();
    static auto new_ms = millis();
    void set_old_ms() { old_ms = millis(); }
    void set_new_ms() { new_ms = millis(); }
    uint32_t elapsed() { return new_ms-old_ms; }
}

void space() {
    print(' ');
}

namespace dump {
    void nibble(uint8_t data) {
        print(data, HEX);
    }

    void hex(uint8_t data) {
        nibble(data >> 4);
        nibble(data & 0xf);
    }

    void hex(uint16_t data) {
        hex(highByte(data));
        hex(lowByte(data));
    }

    void bin(uint8_t data) {
        for (int8_t i=7; i>=0; --i) {
            print((data >> i) & 1);
        }
    }

    void bin(uint16_t data) {
        bin(highByte(data));
        space();
        bin(lowByte(data));
    }

    void value(uint16_t data, uint8_t base) {
        switch (base) {
            case  2: bin(data);   break;
            case 10: print(data); break;
            case 16: hex(data);   break;
            default: print(F("wrong base"));
        }
    }

    void status(uint8_t s) {
        for (int8_t i=7; i>=0; --i) {
            if (i == 4) {
                print(' ');
            }
            print((s >> i) & 1);
            if (i == 4) {
                print(' ');
            }
        }
        if ((s >> 4) & 1) {
            print(F(" (ERROR)"));
        } else {
            print(F(" (OK)"));
        }
        print(F("  burst mode, woc mode, sm mode, error, single bit error, rs, d1, d0"));
        if (s == 0xff) {
            print(F(" -- this might indicate an i2c communication failure!"));
        }
    }

    void reg(uint8_t adr, uint16_t reg) {
        print(F("0x"));
        hex(adr);
        space();
        space();

        hex(reg);
        space();
        space();
        bin(reg);
        space();
        space();
    }

    void register_0(uint16_t reg) {
        dump::reg(0, reg);
        print(F("[15:9] ANA_RESERVED_LOW, [8] BIST, [7] Z_SERIES, [6:4] GAIN_SEL, [3:0] HALLCONF"));
    }

    void register_1(uint16_t reg) {
        dump::reg(1, reg);
        print(F("[15] TRIG_INT, [14:13] COMM_MODE, [12] WOC_DIFF, [11] EXT_TRIG, [10] TCMP_EN, [9:6] BURST_SEL (zyxt), [5-0] Burst data rate"));
    }

    void register_2(uint16_t reg) {
        dump::reg(2, reg);
        print(F("[15:13] -, [12:11] OSR2, [10:5] RES_XYZ, [4:2] DIG_FILT, [1:0] OSR"));
    }

    void register_3(uint16_t reg) {
        dump::reg(3, reg);
        print(F("[15:8] SENS_TC_HT, [7:0] SENS_TC_LT"));
    }

    void register_4(uint16_t reg) {
        dump::reg(4, reg);
        print(F("Offset X"));
    }

    void register_5(uint16_t reg) {
        dump::reg(5, reg);
        print(F("Offset Y"));
    }

    void register_6(uint16_t reg) {
        dump::reg(6, reg);
        print(F("Offset Z"));
    }

    void register_7(uint16_t reg) {
        dump::reg(7, reg);
        print(F("WOXY Threshold"));
    }

    void register_8(uint16_t reg) {
        dump::reg(8, reg);
        print(F("WOZ Threshold"));
    }

    void register_9(uint16_t reg) {
        dump::reg(9, reg);
        print(F("WOT Threshold"));
    }
}


namespace explain {
    void status(status_t status) {

        print(F("burst mode: "));
        println(status.burst_mode);
        print(F("woc mode:   "));
        println(status.woc_mode);
        print(F("sm mode:    "));
        println(status.sm_mode);
        print(F("error: "));
        println(status.err);
        print(F("single bit error detected: "));
        println(status.sed);
        print(F("rs: "));
        println(status.rs);
        print(F("d1: "));
        println(status.d1);
        print(F("d0: "));
        println(status.d0);
        if (uint8_t(status) == 0xff) {
            println(F("This might indicate an i2c communication failure!"));
        }
    }

    void register_0(uint16_t reg) {
        uint16_t data = uint16_t(reg);

        print(F("ANA_RESERVED_LOW: "));
        println(data >> 9);
        print(F("BIST: "));
        println((data >> 8) & 1);
        print(F("Z_SERIES: "));
        println((data >> 7) & 1);
        print(F("GAIN_SEL: "));
        println((data >> 4) & 7);
        print(F("HALLCONF: "));
        println(data & 0xf);
    }

    void register_1(uint16_t reg) {
        uint16_t data = uint16_t(reg);

        print(F("TRIG_INT: "));
        println(data >> 15);
        print(F("COMM_MODE: "));
        println((data >> 13) & 3);
        print(F("WOC_DIFF: "));
        println((data >> 12) & 1);
        print(F("EXT_TRIG: "));
        println((data >> 11) & 1);
        print(F("TCMP_EN: "));
        println(data >> 10 & 1);
        print(F("BURST_SEL (zyxt): "));
        print((data >> 9) & 1);
        print((data >> 8) & 1);
        print((data >> 7) & 1);
        println((data >> 6) & 1);
        print(F("Burst data rate: "));
        println(data & 0x1f);
    }

    void register_2(uint16_t reg) {
        uint16_t data = uint16_t(reg);

        print(F("unused: "));
        println((data >> 13) & 0x7);
        print(F("Temperature Oversampling OSR2: "));
        println((data >> 11) & 3);
        print(F("Resolution X Y Z: "));
        print((data >> 5) & 0x3); space();
        print((data >> 7) & 0x3); space();
        println((data >> 9) & 0x3);
        print(F("Digital Filtering: "));
        println((data >> 2) & 0x7);
        print(F("Oversampling: "));
        println(data & 3);
    }

    void register_3(uint16_t reg) {
        print(F("Sensitivity drift compensation factor for T > Tref: "));
        println(highByte(reg));
        print(F("Sensitivity drift compensation factor for T < Tref: "));
        println(lowByte(reg));
    }

    void register_4(uint16_t reg) {
        print(F("Offset X:"));
        println(reg);
    }

    void register_5(uint16_t reg) {
        print(F("Offset Y:"));
        println(reg);
    }

    void register_6(uint16_t reg) {
        print(F("Offset Z:"));
        println(reg);
    }

    void register_7(uint16_t reg) {
        print(F("WOXY Threshold:"));
        println(reg);
    }

    void register_8(uint16_t reg) {
        print(F("WOZ Threshold:"));
        println(reg);
    }

    void register_9(uint16_t reg) {
        print(F("WOT Threshold:"));
        println(reg);
    }

    void separator_as_needed(boolean has_predecessor) {
        if (has_predecessor and output_mode.style != output_mode_t::style_t::HEX_RAW) {
            print(',');
        }
    };

    void axis(uint8_t zyxt) {
        boolean has_predecessor = false;

        if (zyxt & AXIS_MASK::X) {
            print('X');
            has_predecessor = true;
        }
        if (zyxt & AXIS_MASK::Y) {
            separator_as_needed(has_predecessor);
            print('Y');
            has_predecessor = true;
        }
        if (zyxt & AXIS_MASK::Z) {
            separator_as_needed(has_predecessor);
            print('Z');
            has_predecessor = true;
        }
        if (zyxt & AXIS_MASK::T) {
            separator_as_needed(has_predecessor);
            print('T');
            has_predecessor = true;
        }
        if (has_predecessor) {
            println();
        } else {
            println('-');
        }
    }

    void data(uint8_t zyxt, zyxt_t d) {
        const uint8_t base = output_mode.style == output_mode_t::DEC_CSV  ? 10 :
                                                                            16;
        boolean has_predecessor = false;
        if (zyxt & AXIS_MASK::X) {
            dump::value(d.x, base);
            has_predecessor = true;
        }
        if (zyxt & AXIS_MASK::Y) {
            separator_as_needed(has_predecessor);
            dump::value(d.y, base);
            has_predecessor = true;
        }
        if (zyxt & AXIS_MASK::Z) {
            separator_as_needed(has_predecessor);
            dump::value(d.z, base);
            has_predecessor = true;
        }
        if (zyxt & AXIS_MASK::T) {
            separator_as_needed(has_predecessor);
            dump::value(d.t, base);
            has_predecessor = true;
        }
        if (has_predecessor) {
            println();
        } else {
            println('-');
        }
    }
}

namespace show {
    void status(status_t s, boolean newline = true) {
        // Notice that newline will only be output IF AND ONLY IF
        // status prints output. So this is NOT the same as
        // calling status() and then println()

        if (output_mode.verbosity == output_mode_t::QUIET) { return; }

        if (output_mode.verbosity == output_mode_t::VERBOSE || s.error()) {
            dump::status(uint8_t(s));
            if (newline) { println(); }
        }
    }

    void registers(uint8_t lo, uint8_t hi, bool dump = true, bool explain = false) {
        // dump registers (Datasheet section 9)

        for (uint8_t i=lo; i <= hi and i < 0x40; ++i) {
            uint16_t reg;
            status_t status;
            status = MLX::readRegister(i, reg);
            if (status.ok()) {
                if (dump) {
                    switch (i) {
                        case 0: dump::register_0(reg); break;
                        case 1: dump::register_1(reg); break;
                        case 2: dump::register_2(reg); break;
                        case 3: dump::register_3(reg); break;
                        case 4: dump::register_4(reg); break;
                        case 5: dump::register_5(reg); break;
                        case 6: dump::register_6(reg); break;
                        case 7: dump::register_7(reg); break;
                        case 8: dump::register_8(reg); break;
                        case 9: dump::register_9(reg); break;
                        default: dump::reg(i, reg);
                    }
                    // no, we will not dump the status
                    // this clutters the display to much
                }
                if (explain) {
                    switch (i) {
                        case 0: explain::register_0(reg); break;
                        case 1: explain::register_1(reg); break;
                        case 2: explain::register_2(reg); break;
                        case 3: explain::register_3(reg); break;
                        case 4: explain::register_4(reg); break;
                        case 5: explain::register_5(reg); break;
                        case 6: explain::register_6(reg); break;
                        case 7: explain::register_7(reg); break;
                        case 8: explain::register_8(reg); break;
                        case 9: explain::register_9(reg); break;
                        default: if (!dump) dump::reg(i, reg);  println();
                    }
                    show::status(status);
                }
            } else {
                dump::reg(i, reg);
                print(F("read failure  "));
                show::status(status, false);
            }
            println();
        }
    }

    void measurement(uint8_t axis_mask) {
        zyxt_t data;
        if (output_mode.state == output_mode_t::IDLE) { return; }

        if (output_mode.state == output_mode_t::READ ||
            output_mode.read_behaviour == output_mode_t::CONTINUOUS ||
            output_mode.read_behaviour == output_mode_t::AUTO &&
            output_mode.state != output_mode_t::IDLE &&
            (  drdy_pin < 0 ||
            drdy_pin >= 0 && digitalRead(drdy_pin))
            ) {
            // attention if WOC is selected AND drdy_pin < 0 and
            // float output is selected we have to ensure that this is not going to block
            // however this is close to impossible without a drdy pin.
            // workaround: if AUTO / WOC is selected withoput drdy pin
            // then read the data but only output it if there is no error.

            if (drdy_pin < 0 &&
                output_mode.read_behaviour == output_mode_t::AUTO &&
                output_mode.state != output_mode_t::READ) {
                // should be good enough for our purposes
                delay(500);
            } else {
                if (output_mode.read_behaviour == output_mode_t::AUTO ||
                    output_mode.read_behaviour != output_mode_t::CONTINUOUS &&
                    output_mode.state != output_mode_t::IDLE ) {
                    delayMicroseconds(100);
                    for (uint32_t ms = millis(); (millis()-1000 <= ms) && !MLX::dataReady(); );
                }
            }

            zyxt_t data;
            const status_t status = MLX::readMeasurement(axis_mask, data);

            if (output_mode.timestamps == 1) {
                timer::set_new_ms();
                const uint32_t elapsed = timer::elapsed();
                timer::set_old_ms();

                print(millis());
                print(F(", "));
                println(elapsed);
            }

            show::status(status);
            explain::axis(axis_mask & 0x0f);
            explain::data(axis_mask, data);

            if (output_mode.state == output_mode_t::READ ||
                output_mode.state == output_mode_t::START_MEASUREMENT &&
                output_mode.read_behaviour != output_mode_t::CONTINUOUS) {
                output_mode.state = output_mode_t::IDLE;
            }
        }
    }
}

void help() {
    println();
    println(F("H, ?:  this help function\n"));

    println(F("D  Dump Registers 0x0 - 0x9"));
    println(F("Dc Dump Customer Area (0x00 - 0x1F)"));
    println(F("Dm Dump Melexis Area (0x20 - 0x3F)"));
    println(F("Da All (0x00 - 0x3F)"));
    println(F("E  Explain Registers 0x0 - 0x9\n"));

    println(F("S [xyzt*]  Single Measurement"));
    println(F("B [xyzt*]  Burst Measurement"));
    println(F("W [xyzt*]  Wake on Change"));
    println(F("R [xyzt*]  Read Measurement Result"));
    println(F("X  eXit Burst or Wake on Change Mode\n"));

    println(F("@  MLX90393 Software Reset"));
    println(F(".  NOP / read status\n"));

    println(F("$aa=XXxx set register at address aa value XXxx. XX = high byte.\n"));

    println(F("<   Memory Recall"));
    println(F(">!  Memory Store\n"));

    println(F("The following commands will set or get parameters"));
    println(F("+G [0-7]   Gain"));
    println(F("+O [0-3]   OverSampling"));
    println(F("+T [0-3]   Temperature oversampling"));
    println(F("+F [0-7]   digital Filtering"));
    println(F("+N [0-3]{3} resolutioN  (xyz)"));
    println(F("+C [0-1]   temperature Compensation"));
    println(F("+H [0-f]   Hall configuration"));
    println(F("+A [xyzt]* Axis default choice for triggered reads"));
    println(F("+E [0-1]   External trigger (1 = enable)"));
    println(F("+I [0-1]   trigger / Interrupt (0 = trigger, 1 = interrupt)\n"));

    println(F("#V [0-2] Output Verbosity"));
    println(F("#S [0-3] Output Style HEX_RAW=0, HEX_CSV=1, DEC_CSV=2, FLOAT_CSV=3"));
    println(F("#B [0-2] Output Behaviour ON_DEMAND = 0, AUTO = 1, CONTINUOUS = 2\n"));
    println(F("#T [0-1] Timestamps\n"));
}

namespace parser {
    const char command_separator = ';';
    const char option_separator = ',';

    boolean is_joker(const char c) {
        return (c == '*');
    }

    uint8_t parse_joker(const char c) {
        return 0xf;
    }

    boolean is_quarternary_digit(const char c) {
        return (('0' <= c) && (c < '4'));
    }

    boolean is_octal_digit(const char c) {
        return (('0' <= c) && (c < '8'));
    }

    boolean is_decimal_digit(const char c) {
        return (('0' <= c) && (c <= '9'));
    }

    uint8_t parse_digit(const char c) {
        return
            is_decimal_digit(c)      ? c - '0' :
            ('a' <= c) && (c <= 'f') ? c - 'a' + 10
                                     : c - 'A' + 10;
    }

    boolean is_decimal_digit_or_joker(const char c) {
        return (('0' <= c) && (c <= '9')) || (c == '*');
    }

    uint8_t parse_digit_or_joker(const char c) {
        return
            is_joker(c) ? parse_joker(c)
                        : parse_digit(c);
    }

    boolean is_hexadecimal_digit(const char c) {
        return ((('0' <= c) && (c <= '9')) ||
                (('a' <= c) && (c <= 'f')) ||
                (('A' <= c) && (c <= 'F')));
    }

    boolean is_ternary_digit(const char c) {
        return (('0' <= c) && (c < '3'));
    }

    boolean is_binary_digit(const char c) {
        return (('0' <= c) && (c < '2'));
    }

    void static_parse(const char c, const char previous_char) {
        using namespace MLX90393;
        // static_parse is a co-routine for parsing.
        // That is all variables will be declared static
        // and the program counter will be stored in
        // static void * parser_state.
        // The idea is that the YIELD_NEXT_CHAR
        // macro can be used to return control to the
        // caller while keeping the internal state.
        // The caller in turn will push the
        // next available character into static_parse
        // whenever it has one character ready.
        // This is a means of cooperative multi tasking

        // For more background on what is going on you might want to
        // follow the two links below.
        //     https://gcc.gnu.org/onlinedocs/gcc/Labels-as-Values.html
        //     http://www.chiark.greenend.org.uk/~sgtatham/coroutines.html

        static void * parser_state = &&l_parser_start;

        if (parser_state < &&l_generic_error) {
            print(c);
        }

        if (c == ' ') {
            return;
        }

        if (c == command_separator) {
            println();
        }

        goto *parser_state;
        #define LABEL(N) label_ ## N
        #define XLABEL(N) LABEL(N)
        #define YIELD_NEXT_CHAR                                               \
            do {                                                              \
                parser_state = &&XLABEL(__LINE__); return; XLABEL(__LINE__):; \
            } while (0)

        l_parser_start: ;

        if (c == ' ') {
            // ignore leading space
            YIELD_NEXT_CHAR;
        }

        uint8_t zyxt = 0;
        static bool start_detetermine_axis;
        switch (c) {
            case '?':
            case 'h': {
                help();
                // ignore all characters till command end
                while (c != command_separator) {
                    YIELD_NEXT_CHAR;
                }
                goto l_done;
            }

            case 'd': {
                YIELD_NEXT_CHAR;
                static uint8_t dump_lo;
                static uint8_t dump_hi;
                switch (c) {
                    case 'c': dump_lo = 0x00; dump_hi = 0x1F; YIELD_NEXT_CHAR; break;
                    case 'm': dump_lo = 0x20; dump_hi = 0x3F; YIELD_NEXT_CHAR; break;
                    case 'a': dump_lo = 0x00; dump_hi = 0x3F; YIELD_NEXT_CHAR; break;
                    case command_separator: dump_lo = 0x00; dump_hi = 0x9;     break;
                    default: goto l_unknown_command;
                }
                if (c != command_separator) goto l_separator_error;
                show::registers(dump_lo, dump_hi);
                goto l_done;
            }

            case 'e': {
                YIELD_NEXT_CHAR;
                if (c != command_separator) goto l_separator_error;
                show::registers(0, 9, false, true);
                goto l_done;
            }

            case '@': {
                YIELD_NEXT_CHAR;
                if (c != command_separator) goto l_separator_error;
                println(F("Software reset"));
                explain::status(MLX::reset());
                output_mode.state = output_mode_t::IDLE;
                goto l_done;
            }

            case '.': {
                YIELD_NEXT_CHAR;
                if (c != command_separator) goto l_separator_error;
                println(F("nop"));
                explain::status(MLX::nop());
                goto l_done;
            }

            case 'x': {
                YIELD_NEXT_CHAR;
                if (c != command_separator) goto l_separator_error;
                println(F("exit mode"));
                explain::status(MLX::exit());
                output_mode.state = output_mode_t::IDLE;
                goto l_done;
            }

            case '<': {
                YIELD_NEXT_CHAR;
                if (c != command_separator) goto l_separator_error;
                println(F("Memory Recall"));
                show::status(MLX::memoryRecall());
                goto l_done;
            }

            case '>': {
                YIELD_NEXT_CHAR;
                if (c != '!') goto l_unknown_command;
                YIELD_NEXT_CHAR;
                if (c != command_separator) goto l_separator_error;
                println(F("Memory Store"));
                show::status(MLX::memoryStore());
                goto l_done;
            }

            static uint8_t  adr;
            static uint16_t val;
            case '$': {
                YIELD_NEXT_CHAR;  // consume first address nibble
                if (!is_hexadecimal_digit(c)) goto l_hexadecimal_digit_expected;
                YIELD_NEXT_CHAR;  // consume second, address nible or = or ;
                if (is_hexadecimal_digit(c)) {
                    adr = ((parse_digit(previous_char))<<4) |
                            parse_digit(c);
                    if (adr > 0x3f) goto l_address_out_of_range;
                    YIELD_NEXT_CHAR;  // consume = or ;
                    if (c == command_separator) {
                        goto l_read_register;
                    } else if (c != '=') {
                        goto  l_assignment_expected;
                    }
                } else {  // single digit case, treat = or ;
                    adr = parse_digit(previous_char);
                    if (c == command_separator) {
                        goto l_read_register;
                    } else if (c != '=') {
                        goto l_hexadecimal_digit_expected;
                    }
                }
                // writing to MLX range is not supported by the chip,
                // but maybe we want to try anyway
                if (adr > 0x1f) goto l_address_out_of_range;
                YIELD_NEXT_CHAR;  // consume highest value nibble
                val = 0;
                if (!is_hexadecimal_digit(c))
                    if (c == command_separator) goto l_write_register;
                    else goto l_hexadecimal_digit_expected;
                val = parse_digit(c);
                YIELD_NEXT_CHAR;
                if (!is_hexadecimal_digit(c))
                    if (c == command_separator) goto l_write_register;
                    else goto l_hexadecimal_digit_expected;
                val = (val << 4) | parse_digit(c);
                YIELD_NEXT_CHAR;
                if (!is_hexadecimal_digit(c))
                    if (c == command_separator) goto l_write_register;
                    else goto l_hexadecimal_digit_expected;
                val = (val << 4) | parse_digit(c);
                YIELD_NEXT_CHAR;
                if (!is_hexadecimal_digit(c))
                    if (c == command_separator) goto l_write_register;
                    else goto l_hexadecimal_digit_expected;
                val = (val << 4) | parse_digit(c);
                YIELD_NEXT_CHAR;
                if (c != command_separator) goto l_separator_error;

                l_write_register:
                show::status(MLX::writeRegister(adr, val));
                goto l_done;

                l_read_register:
                show::status(MLX::readRegister(adr, val));
                dump::reg(adr, val);
                goto l_done;
            }

            case '+': {
                YIELD_NEXT_CHAR;
                switch(c) {
                    case 'g': {
                        YIELD_NEXT_CHAR;
                        if (c==command_separator) {
                            print(F("Gain: "));
                            uint8_t val;
                            show::status(MLX::read<GAIN_SEL>(val));
                            println(val);
                        } else {
                            if (!is_octal_digit(c)) goto l_octal_digit_expected;
                            YIELD_NEXT_CHAR;
                            if (c != command_separator) goto l_separator_error;
                            print(F("Gain: "));
                            println(previous_char);
                            show::status(MLX::write<GAIN_SEL>(parse_digit(previous_char)));
                        }
                        goto l_done;
                    }

                    case 'o': {
                        YIELD_NEXT_CHAR;
                        if (c==command_separator) {
                            print(F("Oversampling: "));
                            uint8_t val;
                            show::status(MLX::read<OSR>(val));
                            println(val);
                        } else {
                            if (!is_quarternary_digit(c)) goto l_quarternary_digit_expected;
                            YIELD_NEXT_CHAR;
                            if (c != command_separator) goto l_separator_error;
                            print(F("Oversampling: "));
                            println(previous_char);
                            show::status(MLX::write<OSR>(parse_digit(previous_char)));
                        }
                        goto l_done;
                    }

                    case 't': {
                        YIELD_NEXT_CHAR;
                        if (c==command_separator) {
                            print(F("Temperature oversampling: "));
                            uint8_t val;
                            show::status(MLX::read<OSR2>(val));
                            println(val);
                        } else {
                            if (!is_quarternary_digit(c)) goto l_quarternary_digit_expected;
                            YIELD_NEXT_CHAR;
                            if (c != command_separator) goto l_separator_error;
                            print(F("Temperature oversampling: "));
                            println(previous_char);
                            show::status(MLX::write<OSR2>(parse_digit(previous_char)));
                        }
                        goto l_done;
                    }

                    case 'f': {
                        YIELD_NEXT_CHAR;
                        if (c==command_separator) {
                            print(F("digital Filtering: "));
                            uint8_t val;
                            show::status(MLX::read<DIG_FLT>(val));
                            println(val);
                        } else {
                            if (!is_octal_digit(c))  goto l_octal_digit_expected;
                            YIELD_NEXT_CHAR;
                            if (c != command_separator) goto l_separator_error;
                            print(F("digital Filtering: "));
                            println(previous_char);
                            show::status(MLX::write<DIG_FLT>(previous_char));
                        }
                        goto l_done;
                    }

                    case 'n': {
                        uint8_t x;
                        uint8_t y;
                        uint8_t z;

                        YIELD_NEXT_CHAR;
                        if (c == command_separator) {
                            show::status(MLX::readResolution(x, y, z));
                        } else {
                            static uint8_t val;
                            if (!is_quarternary_digit(c)) goto l_quarternary_digit_expected;
                            val = parse_digit(c);
                            YIELD_NEXT_CHAR;
                            if (!is_quarternary_digit(c)) goto l_quarternary_digit_expected;
                            val = (val<<2) | parse_digit(c);
                            YIELD_NEXT_CHAR;
                            if (!is_quarternary_digit(c)) goto l_quarternary_digit_expected;
                            val = (val<<2) | parse_digit(c);
                            YIELD_NEXT_CHAR;
                            if (c != command_separator) goto l_separator_error;

                            z = val & 0x3;
                            y = (val >> 2) & 0x3;
                            x = (val >> 4) & 0x3;
                            show::status(MLX::writeResolution(x, y, z));
                        }
                        print(F("resolutionN: "));
                        print(x); space();
                        print(y); space();
                        println(z);

                        goto l_done;
                    }

                    case 'c': {
                        YIELD_NEXT_CHAR;
                        if (c==command_separator) {
                            print(F("Temperature Compensation: "));
                            uint8_t enabled;
                            show::status(MLX::read<TCMP_EN>(enabled));
                            println(enabled);
                        } else {
                            if (!is_binary_digit(c)) goto l_binary_digit_expected;
                            YIELD_NEXT_CHAR;
                            if (c != command_separator) goto l_separator_error;
                            print(F("Temperature Compensation: "));
                            println(previous_char);
                            show::status(MLX::write<TCMP_EN>(parse_digit(previous_char)));
                        }
                        goto l_done;
                    }

                    case 'h': {
                        YIELD_NEXT_CHAR;
                        if (c==command_separator) {
                            print(F("Hall configuration: "));
                            uint8_t hallconf;
                            show::status(MLX::read<HALLCONF>(hallconf));
                            println(hallconf,16);
                        } else {
                            if (!is_hexadecimal_digit(c)) goto l_hexadecimal_digit_expected;
                            YIELD_NEXT_CHAR;
                            if (c != command_separator) goto l_separator_error;
                            print(F("Hall configuration: "));
                            println(previous_char);
                            show::status(MLX::write<HALLCONF>(parse_digit(previous_char)));
                        }
                        goto l_done;
                    }

                    case 'a': {
                        static uint8_t burst_sel;
                        YIELD_NEXT_CHAR;
                        if (c==command_separator) {
                            uint8_t burst_sel;
                            print(F("Axis default: "));
                            show::status(MLX::read<BURST_SEL>(burst_sel));
                            explain::axis(burst_sel);
                            println();
                        } else {
                            burst_sel = 0;
                            goto l_det_axis_start;
                            l_det_axis:
                            YIELD_NEXT_CHAR;
                            l_det_axis_start:
                            if (start_detetermine_axis and (c != command_separator)) {
                                burst_sel = 0;
                            }
                            start_detetermine_axis = false;
                            switch (c) {
                                case 'x': burst_sel |= AXIS_MASK::X; goto l_det_axis;
                                case 'y': burst_sel |= AXIS_MASK::Y; goto l_det_axis;
                                case 'z': burst_sel |= AXIS_MASK::Z; goto l_det_axis;
                                case 't': burst_sel |= AXIS_MASK::T; goto l_det_axis;
                                case '*': burst_sel |= AXIS_MASK::ALL; goto l_det_axis;
                                case command_separator: break;
                                default: goto l_zyxt_expected;
                            }
                            print(F("Axis default: "));
                            explain::axis(burst_sel);
                            show::status(MLX::write<BURST_SEL>(burst_sel));
                        }
                        goto l_done;
                    }

                    case 'e': {
                        YIELD_NEXT_CHAR;
                        if (c==command_separator) {
                            print(F("External trigger: "));
                            uint8_t ext_trig;
                            show::status(MLX::read<EXT_TRIG>(ext_trig));
                            println(ext_trig);
                        } else {
                            if (!is_binary_digit(c)) goto l_binary_digit_expected;
                            YIELD_NEXT_CHAR;
                            if (c != command_separator) goto l_separator_error;
                            print(F("External trigger: "));
                            println(previous_char);
                            show::status(MLX::write<EXT_TRIG>(parse_digit(previous_char)));
                        }
                        goto l_done;
                    }

                    case 'i': {
                        YIELD_NEXT_CHAR;
                        if (c==command_separator) {
                            print(F("trigger / Interrupt: "));
                            uint8_t trig_int_sel;
                            show::status(MLX::read<TRIG_INT_SEL>(trig_int_sel));
                            println(trig_int_sel);
                        } else {
                            if (!is_binary_digit(c)) goto l_binary_digit_expected;
                            YIELD_NEXT_CHAR;
                            if (c != command_separator) goto l_separator_error;
                            print(F("trigger / Interrupt: "));
                            println(previous_char);
                            show::status(MLX::write<TRIG_INT_SEL>(parse_digit(previous_char)));
                        }
                        goto l_done;
                    }

                }
                goto l_unknown_command;
            }

            case '#': {
                YIELD_NEXT_CHAR;
                switch (c) {
                    case 'v': {
                        YIELD_NEXT_CHAR;
                        if (c == command_separator) goto l_output_verbosity;
                        if (!is_ternary_digit(c)) goto l_ternary_digit_expected;
                        YIELD_NEXT_CHAR;
                        if (c != command_separator) goto l_separator_error;
                        output_mode.verbosity = parse_digit(previous_char);
                        l_output_verbosity:
                        print(F("output Verbosity: "));
                        println(output_mode.verbosity);
                        break;
                    }
                    case 's': {
                        YIELD_NEXT_CHAR;
                        if (c == command_separator) goto l_output_style;
                        if (!is_quarternary_digit(c)) goto l_quarternary_digit_expected;
                        YIELD_NEXT_CHAR;
                        if (c != command_separator) goto l_separator_error;
                        output_mode.style = parse_digit(previous_char);
                        l_output_style:
                        print(F("output Style: "));
                        println(output_mode.style);
                        break;
                    }
                    case 'b': {
                        YIELD_NEXT_CHAR;
                        if (c == command_separator) goto l_output_behaviour;
                        if (!is_ternary_digit(c)) goto l_ternary_digit_expected;
                        YIELD_NEXT_CHAR;
                        if (c != command_separator) goto l_separator_error;
                        output_mode.read_behaviour = parse_digit(previous_char);
                        l_output_behaviour:
                        print(F("output Behaviour: "));
                        println(output_mode.read_behaviour);
                        break;
                    }
                    case 't': {
                        YIELD_NEXT_CHAR;
                        if (c == command_separator) goto l_output_timestamps;
                        if (!is_binary_digit(c)) goto l_binary_digit_expected;
                        YIELD_NEXT_CHAR;
                        if (c != command_separator) goto l_separator_error;
                        output_mode.timestamps = parse_digit(previous_char);
                        l_output_timestamps:
                        print(F("Timestamps: "));
                        println(output_mode.timestamps);
                        break;
                    }
                    default: goto l_unknown_command;
                }
                goto l_done;
            }

            static uint8_t cmd;
            case 'r': cmd = CMD::READ_MEASUREMENT;  goto l_start_determine_axis;
            case 'w': cmd = CMD::WAKE_ON_CHANGE;    goto l_start_determine_axis;
            case 'b': cmd = CMD::START_BURST;       goto l_start_determine_axis;
            case 's': cmd = CMD::START_MEASUREMENT; goto l_start_determine_axis;

            l_start_determine_axis:
                start_detetermine_axis = true;
            l_determine_axis:  //  default: start_detetermine_axis = false;
                YIELD_NEXT_CHAR;
                if (start_detetermine_axis and (c != command_separator)) {
                    // wipe outdated axis information
                    cmd &= 0xf0;
                }
                start_detetermine_axis = false;
                switch (c) {
                    case 'x': zyxt = AXIS_MASK::X; break;
                    case 'y': zyxt = AXIS_MASK::Y; break;
                    case 'z': zyxt = AXIS_MASK::Z; break;
                    case 't': zyxt = AXIS_MASK::T; break;
                    case '*': zyxt = AXIS_MASK::ALL; break;
                    case command_separator: {
                        if ((cmd & 0xf0) != CMD::READ_MEASUREMENT) {
                            show::status(MLX::sendCommand(cmd));
                        }

                        timer::set_old_ms();
                        switch (cmd & 0xf0) {
                            case CMD::READ_MEASUREMENT:
                                output_mode.state = output_mode_t::READ;
                                break;
                            case CMD::START_MEASUREMENT:
                                output_mode.state = output_mode_t::START_MEASUREMENT;
                                break;
                            case CMD::START_BURST:
                                output_mode.state = output_mode_t::BURST;
                                break;
                            case CMD::WAKE_ON_CHANGE:
                                output_mode.state = output_mode_t::WAKE_ON_CHANGE;
                                break;
                        }

                        axis_mask = cmd & AXIS_MASK::ALL;
                        show::measurement(axis_mask);
                        goto l_done;
                    }
                    default: goto l_zyxt_expected;
                }

                cmd |= zyxt;
                goto l_determine_axis;

            default: goto l_unknown_command;
        }

        l_unknown_command:
        print(F("\n Unknown command"));
        goto l_generic_error;

        l_address_out_of_range:
        print(F("\n Address out of range"));
        goto l_generic_error;

        l_value_out_of_range:
        print(F("\n Value out of range"));
        goto l_generic_error;

        l_assignment_expected:
        print(F(" \n Assignment '=' expected"));
        goto l_generic_error;

        l_zyxt_expected:
        print(F("\nAxis [txyz] or * expected"));
        goto l_generic_error;

        l_digit_or_joker_expected:
        print(F("\n Digit or * expected"));
        goto l_generic_error;

        l_digit_expected:
        print(F("\n Digit expected"));
        goto l_generic_error;

        l_octal_digit_expected:
        print(F("\n Octal digit expected"));
        goto l_generic_error;

        l_quarternary_digit_expected:
        print(F("\n Quarternary digit expected"));
        goto l_generic_error;

        l_ternary_digit_expected:
        print(F("\n Quarternary digit expected"));
        goto l_generic_error;

        l_binary_digit_expected:
        print(F("\n Binary digit expected"));
        goto l_generic_error;

        l_hexadecimal_digit_expected:
        print(F("\n Hexadecimal digit expected"));
        goto l_generic_error;

        l_separator_error:
        print(F("\n Unexpected or missing separator"));
        goto l_generic_error;

        l_out_of_range_error:
        print(F("\n Digit out of range error"));
        goto l_generic_error;

        l_syntax_error:
        println();
        goto l_generic_error;

        l_generic_error:

        print(F(" error at '"));
        print(c);
        if (c== command_separator) {
            println('\'');
        } else {
            print(F("' before "));
            do {
                YIELD_NEXT_CHAR;
                print(c);
            } while (c != command_separator);

            println();
        }
        println();
        println(F("Use h for help!"));
        goto l_done;

        l_done:
        println();
        parser_state = &&l_parser_start;
        return;
    }


    boolean parse() {  // deliver true if some character was available
        static char previous_char;
        if (Serial.available()) {
            char c = (char) Serial.read();

            // interpret tab as whitespace
            if (c == 0x09) {
                c = ' ';
            }

            // map newline and carriage return to command separator
            if (c==0x0A || c==0x0D) {
                c = command_separator;
            }

            // ignore successive separators
            if ((c == option_separator || c == command_separator) && (c == previous_char)) {
                return true;
            }

            // map everything to lower case
            if (('A' <= c) && (c <= 'Z')) {
                c+= 'a' - 'A';
            }

            static_parse(c, previous_char);
            if (c != ' ') {
                previous_char = c;
            }

            return true;
        }
        return false;
    }
}

void boilerplate() {
    print(F("\nMelexis 90393 debug helper for MLX90393_raw V"));
    println(F(MLX90393_VERSION_STRING));
    println(F("\n(C) Udo Klein 2018"));
    println(F("License: GPL v3\n"));

    //help();

    println();
    explain::status(MLX::nop());
    println();
    show::registers(0, 9);

    println(F("\n\nUse h for help!"));
}

void setup(){
    Serial.begin(baudrate);

    MLX::begin();
    boilerplate();
}


void loop() {
    while (parser::parse()) {
        /* consume all available input characters */
    }

    show::measurement(axis_mask);
}
