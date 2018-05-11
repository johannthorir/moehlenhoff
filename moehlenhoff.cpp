// Copyright (c) 2016 Jóhann Þórir Jóhannsson. All rights reserved.

// This is a FSK demodulator. It's hardcoded to demodulate a FSK
// signal from a Möhlenhoff AR 4070 KF-2 room thermostat sampled
// froom 868.1 Mhz at 2046000 samples pr sec with rtl_sdr.
// Specifically, the parameters for rtl_sdr are:
//    rtl_sdr -f 868.3e1 -d 0 -s 2048000 -g 33.8 -
// whose output can be piped directly to stdin of this program.
//
// Operation is as follows, either I or Q (doesn't matter which
// since the signal isn't phase modulated) is multiplied by a
// delayed copy of itself and then lowpass filtered to remove the
// harmonics added by the multiplication.
//
// The tuning of the feedback delay is found by finding local
// maximum of the absolute difference of the phase of the cosines
// of the two frequencies - low and high. At this sample rate and
// tuning, this happens at a feedback of 4 samples. The harmonics
// filters are a couple of bi-quad Butterworths at 0.025 the
// sample rate in series. Also incorporated is a noise gate.
//
// The output is a square wave which is positive for a high
// frequency and negative for low frequency with a power related
// to the original signal strength. This signal can then be
// imported into Audacity as raw data, single channel 8 bit
// unsigned pcm, or further processed to extract the original
// digital signal.
//
// Only dependency is the rtl-sdr library (librtlsdr-dev) and
// builds with just
//    g++ moehlenhoff.cpp -lrtlsdr -o moehlenhoff

// TODO:
//   1)  get under source control.
//   2)  create cmake environment
//   3)  split into multiple files - split stuff up for easier
//       maintenance.
//   4)  add options for samplerate, device, gain etc...
//   5)  add options for dumping raw data at various stages.
//   6)  add way to get the data to a store which is accessible
//       to a web client.
//   7)  update description and comments.
//   8)  code cleanup - c++
//   9)  fault tolerant daemon.
//  10)  figure out the checksum.
//  11)  figure out extra bits in the flags byte - some must be
//       for low battery
//  ???  ---------.   .---------- ???? battery?
//  ???  -------. :   : .-------- Switch in low position
//  ???  -----. : :   : : .------ Switch in high position
//  ???  ---. : : :   : : : .---- "set" button was pushed on the transmitter, all others are 0.
//          x x x x - x x x x
//  12)  Make service scripts and parameterize the output file, pid file
//      etc


#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <map>

#include <rtl-sdr.h>

static const uint32_t DEFAULT_FREQUENCY   = 868100000;
static const uint32_t DEFAULT_SAMPLE_RATE = 2048000;
static const uint32_t DEFAULT_BUF_LENGTH  = (16 * 16384);
static const uint32_t MINIMAL_BUF_LENGTH  = 512;
static const uint32_t MAXIMAL_BUF_LENGTH  = (256 * 16384);
static const uint32_t MAX_FIR_BUFFER_LENGTH = 1024;
static bool do_exit = false;

static const int  symbol_size = 212;  // size of symbol in samples

static const char output_file[] = "/usr/share/moehlenhoff/log_%x.csv";
static const char pid_file[]    = "/var/run/moehlenhoff.pid";

// -----------------------------------------------------------------------------
//!
void unlink_pid() {
   unlink(pid_file);
}

// -----------------------------------------------------------------------------

//!
void save_pid() {
    FILE *pidfile = fopen(pid_file, "w");
    fprintf(pidfile, "%i\n", getpid());
    fclose(pidfile);
}


// -----------------------------------------------------------------------------

static const char pattern[] = "01111100000111110101010101001111110";
static const int  circular_bufmask = 0x3f;

char circular_buffer[circular_bufmask + 1];
int  circular_head = 0; // next position to write in

void circular_clear_buffer() {
    for(int i=0; i<=circular_bufmask; i++)
        circular_buffer[i] = 0;
}

// add one character to the circular buffer and check if we have the
// preamble.
bool add_one_bit(char bit) {
    circular_buffer[circular_head] = bit;
    circular_head++;
    circular_head &= circular_bufmask;

    int start = ((circular_bufmask + 1) + circular_head - 35) & circular_bufmask;
    for(int i=0 ; i<35; i++)
        if(pattern[i] != circular_buffer[(start+i) & circular_bufmask])
            return false;
    circular_clear_buffer();
    return true;
}

// -----------------------------------------------------------------------------
/*
    A message is a series of five bit groups where the first bit
    is always 0 and the remaining four bits are a reversed nibble,
    i.e. the highest order bit is last.
    Total message is:
        0 : preamble    : two nibbles,
        1 : id          : four nibbles,
        2 : temperature : two nibbles,
        3 : setting     : two nibbles,
        4 : external    : two nibbles
        5 : flags       : two nibbles
        6 : checksum    : four nibbles;
    or 18 groups of five bits = 90 bits
    It looks like the temperature and settings are scaled like this:

       T : temperature in C°
       v  :=  (T - 6.6) * 10

    so to get the temperature setting from the value, divide the value
    with 10 and add 6.6.
*/

struct message_t {
    uint8_t  preamble;
    uint16_t id;
    uint8_t  temp;
    uint8_t  setting;
    uint8_t  external;
    uint8_t  flags;
    uint16_t checksum;
    inline float temperature(uint8_t v) { return (float)(v)*0.1f + 6.6f; }
    void print(FILE *file) {
        time_t now = time(0);
        struct tm * t = gmtime(&now);
        fprintf(file, "%02d.%02d.%4d\t"
               "%02d:%02d:%02d\t"
               "%ld\t0x%02d\t0x%04x\t"
               "%3d\t%.1f\t"
               "%3d\t%.1f\t"
               "%3d\t%.1f\t"
               "%c%c%c%c%c%c%c%c\t"
               "0x%04x\n",
            t->tm_mday,  t->tm_mon + 1, t->tm_year + 1900,
            t->tm_hour,  t->tm_min, t->tm_sec,
            now, preamble, id,
            temp,     temperature(temp),
            setting,  temperature(setting),
            external, temperature(external),
            (flags & 0x80 ? '1' : '0'), (flags & 0x40 ? '1' : '0'), (flags & 0x20 ? '1' : '0'), (flags & 0x10 ? '1' : '0'),
            (flags & 0x08 ? '1' : '0'), (flags & 0x04 ? '1' : '0'), (flags & 0x02 ? '1' : '0'), (flags & 0x01 ? '1' : '0'),
            checksum
        );
    }
 };


class Parser {
public:
    Parser() { reset(); }

    void closeFiles() {
        for(auto it : files) {
            fclose(it.second);
        }
    }

    void reset() {
        parsed  = 0;
        state   = 0;
        nibbles = 0;
        bitno   = 0;
        nibble  = 0;
    }

    void parseNibble(uint8_t nibble) {
        // 0:preamble, two nibbles,
        // 1:id : four nibbles,
        // 2:temp : two nibbles,
        // 3:setting : two nibbles,
        // 4:external : two nibbles
        // 5:flags : two nibbles
        // 6:checksum : four nibbles;
        switch(state) {
            case 0:
                parsed = (parsed >> 4) | (nibble << 4);
                nibbles++;
                if(nibbles == 2) {
                    message.preamble = parsed;
                    state++;
                    parsed = 0;
                    nibbles = 0;
                }
                break;
            case 1:
                parsed = (parsed >> 4) | (nibble << 12);
                nibbles++;

                if(nibbles == 4) {
                    message.id = parsed;
                    state++;
                    parsed = 0;
                    nibbles = 0;
                }
                break;
            case 2:
                parsed = (parsed >> 4) | (nibble << 4);
                nibbles++;
                if(nibbles == 2) {
                    message.temp = parsed;
                    state++;
                    parsed = 0;
                    nibbles = 0;
                }
                break;
            case 3:
                parsed = (parsed >> 4) | (nibble << 4);
                nibbles++;
                if(nibbles == 2) {
                    message.setting = parsed;
                    state++;
                    parsed = 0;
                    nibbles = 0;
                }
                break;
            case 4:
                parsed = (parsed >> 4) | (nibble << 4);
                nibbles++;
                if(nibbles == 2) {
                    message.external = parsed;
                    state++;
                    parsed = 0;
                    nibbles = 0;
                }
                break;
            case 5:
                parsed = (parsed >> 4) | (nibble << 4);
                nibbles++;
                if(nibbles == 2) {
                    message.flags = parsed;
                    state++;
                    parsed = 0;
                    nibbles = 0;
                }

                break;
            case 6:
                parsed = (parsed >> 4) | (nibble << 12);
                nibbles++;

                if(nibbles == 4) {
                    // find the file handle to use
                    FILE *file;
                    auto it = files.find(message.id);
                    if(it == files.end()) {
                        char filename[256];
                        sprintf(filename, output_file, message.id);
                        FILE *output = fopen(filename, "a");
                        if(output == 0) {
                            fprintf(stderr, "Failed to open output file %s for appending\n", filename);
                            unlink_pid();
                            exit(1);
                        }
                        setvbuf(output, NULL, _IONBF, 0);
                        files.insert(std::make_pair(message.id, output));
                        file = output;
                    }
                    else {
                        file = it->second;
                    }
                    message.checksum = parsed;
                    message.print(file);
                    reset();
                }
                break;
        }
    }

    void parseBit(char bit) {
        if(bitno == 0) {
            if(bit == '0') {
                // start of a nibble
                nibble = 0;
            }
            else {
                // deal with format error...
                fprintf(stderr, "FORMAT ERROR\n");
                reset();
            }
        }
        else {
            nibble = (nibble >> 1) | (bit == '1' ? 0x08 : 0x00);
        }

        bitno++;

        if(bitno == 5) {
            parseNibble(nibble);
            bitno = 0;
            nibble = 0;
        }
    }

private:
    std::map<uint16_t, FILE *> files;
    message_t message;
    uint16_t parsed;
    int state;
    int nibbles;
    int bitno;
    uint8_t nibble;
};


// -----------------------------------------------------------------------------

static Parser parser;

void processSample(uint8_t byte) {
    static int  last_positive = 0;      // if true, then last zero crossing was from neg to pos.
    static int  samples_since_last = 0; // number of samples read since last zero crossing
    static int  in_message = 0;         // 0 if outside message, 1 if inside, 2 if waitforterm..
    static int  message_bits_read = 0;
    static char lastbit = '.';

    samples_since_last++;
    signed char v = (byte - 127);

    if(( v > 1 && !last_positive)||( v < -1 && last_positive)) {
        // we are seeing a positive edge
        last_positive = 1-last_positive;

        // how many?
        int n   = samples_since_last / symbol_size;
        int rem = samples_since_last % symbol_size;

        if(rem > symbol_size / 2)
            n++;

        for(int i=0; i<n; i++) {
            char thisbit = last_positive ? '1' : '0';
            if(in_message == 2) {
                // we are after the message and next bit
                if(thisbit != '1')
                    fprintf(stderr, " NOT TERMINATED PROPERLY\n");

                parser.reset();
                in_message = 0;
            }
            else if(in_message == 1) {
                // we are inside the message.
                message_bits_read++;

                if(message_bits_read % 2 == 0) {
                    if(thisbit == lastbit) {
                        fprintf(stderr, " TRANSMISSION ERROR\n");
                        parser.reset();
                        in_message = 0;
                    }
                    else {
                        char bit = (lastbit < thisbit ? '1' : '0');

                        parser.parseBit(bit);
                    }
                }

                lastbit = thisbit;

                if(message_bits_read == 180) {
                    in_message = 2;
                    parser.reset();
                }
            }
            else {
                // add the bit to the queue and see if we have the beginning of a message.
                if(add_one_bit('0'+last_positive)) {
                    in_message = 1;
                    message_bits_read = 0;
                    parser.reset();
                }
            }
        }

        samples_since_last = 0;
    }
}

// -----------------------------------------------------------------------------
inline float saturate(float x) { return x < -1.0f ? -1.0f : x > 1.0f ? 1.0f : x; }

class BiQuad {
public:
    // be nice with the freq parameter (0..0.5) or it will not
    // work...
    BiQuad(float freq) {

        x[0] = x[1]= y[0] = y[1] = 0.0f;
        // biquad butterworth lowpass:
        // don't bother with warping ....
        float omega_c = freq;
        float omega_c_2 = omega_c * omega_c;
        float c = 1.0 / M_PI;

        float tcy0 = c * (c + M_SQRT2 * omega_c) + omega_c_2;
        float tcx = omega_c_2 / tcy0;

        g  = tcx;
        cx[0] = 2 * tcx;
        cx[1] = tcx;
        cy[0] = (2.0 * (omega_c_2 - c * c ))/tcy0;
        cy[1] = (c * ( c -  M_SQRT2 * omega_c) + omega_c_2)/tcy0;
    };

    float processSample(float nx)  {
        float ny = g * nx + cx[0] * x[0] + cx[1] * x[1] - cy[0] * y[0] - cy[1] * y[1];
        x[1] = x[0];
        x[0] = nx;
        y[1] = y[0];
        y[0] = ny;
        return saturate(ny);
    };

    void processInline(float data[], int len) {
        float *p = data;
        for(int i=0; i<len; i++) {
            float ny = g * (*p) + cx[0] * x[0] + cx[1] * x[1] - cy[0] * y[0] - cy[1] * y[1];
            x[1] = x[0];
            x[0] = *p;
            y[1] = y[0];
            y[0] = ny;
            *p++ = saturate(ny);
        }
    }


    // ...threw away unneeded stuff like parameter ramping and
    // block level processing
private:
    float x[2];
    float y[2];  // delays
    float g;
    float cx[2];
    float cy[2];  // coefficients
};


// -----------------------------------------------------------------------------


static const float amp            = 8.0f;  // how much to amplify in order to saturate the result.
static const int   max_noise_gate = 128;   // maximum number of samples to keep of zeros...
static const float thresholdsqr   = 0.001f;

int    BUFSIZE = 4;
float  firbuffer[MAX_FIR_BUFFER_LENGTH];
int    firpos = 0;
int    noise_gate = 0;

BiQuad f1(0.0125);
BiQuad f2(0.0225);


// -----------------------------------------------------------------------------

static void sighandler(int signum) {
    fprintf(stderr, "Signal caught, exiting!\n");
    do_exit = true;
}

// -----------------------------------------------------------------------------

// callback for reading asynchronously.
static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx) {
    if (!ctx)
        return;

    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)ctx;

    if (do_exit) {
        rtlsdr_cancel_async(dev);
        return;
    }

    // loop over the interleaved byte samples.
    for(uint32_t i = 0; i<len; i += 2) {
        float I = (buf[  i  ] - 127) / 128.0f;
        float Q = (buf[i + 1] - 127) / 128.0f;

        float argsqr = I*I + Q*Q;

        if(argsqr > thresholdsqr) {
            noise_gate = max_noise_gate;
        }

        if(noise_gate > 0) {
            firbuffer[firpos++] = I;
            firpos %= BUFSIZE;
            float value = saturate((I * firbuffer[firpos])/0.707);
            float sample = f2.processSample(f1.processSample(value)*amp);
            unsigned char byte = sample * 127.0 + 127.0;
            processSample(byte);
//             fwrite(&byte, 1, 1, stdout);
            noise_gate--;
        }
    }
}

// -----------------------------------------------------------------------------

void install_sighandler() {
    struct sigaction sigact;

    sigact.sa_handler = sighandler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGPIPE, &sigact, NULL);
}

// -----------------------------------------------------------------------------

void set_frequency(rtlsdr_dev_t *dev, uint32_t frequency) {
    if (rtlsdr_set_center_freq(dev, frequency) < 0)
        fprintf(stderr, "WARNING: Failed to set center freq.\n");
    else
        fprintf(stderr, "Tuned to %u Hz.\n", frequency);
}

// -----------------------------------------------------------------------------

void set_sample_rate(rtlsdr_dev_t *dev, uint32_t samp_rate) {
    if (rtlsdr_set_sample_rate(dev, samp_rate) < 0)
        fprintf(stderr, "WARNING: Failed to set sample rate.\n");
    else
        fprintf(stderr, "Sampling at %u S/s.\n", samp_rate);
}

// -----------------------------------------------------------------------------
void set_auto_gain(rtlsdr_dev_t *dev) {
    if (rtlsdr_set_tuner_gain_mode(dev, 0)!= 0)
        fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
    else
        fprintf(stderr, "Tuner gain set to automatic.\n");
}

// -----------------------------------------------------------------------------
// must be one of the supported gains * 10
int set_manual_gain(rtlsdr_dev_t *dev, int gain) {
    int r = rtlsdr_set_tuner_gain_mode(dev, 1);
    if (r < 0) {
        fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
        return r;
    }

    r = rtlsdr_set_tuner_gain(dev, gain);
    if (r != 0)
        fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
    else
        fprintf(stderr, "Tuner gain set to %0.2f dB.\n", gain/10.0);

    return r;
}





int main(int argc, char **argv) {
    daemon(0,1);
    save_pid();



    uint32_t frequency      = DEFAULT_FREQUENCY;
    uint32_t samp_rate      = DEFAULT_SAMPLE_RATE;
    uint32_t out_block_size = DEFAULT_BUF_LENGTH;
    int      gain           = 402; // 386; // 38.6 dB
    uint32_t dev_index = 0;

    rtlsdr_dev_t *dev = 0;

    setvbuf(stdout, NULL, _IONBF, 0);
   //  setvbuf(stdin , NULL, _IONBF, 0);


    for(int i=0; i<BUFSIZE; i++)
        firbuffer[i] = 0.0f;

    install_sighandler();

    int r;
    do {
        r  = rtlsdr_open(&dev, dev_index);
        if (r < 0) {
            fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
            unlink_pid();
            exit(1);
        }

        uint8_t *buffer = (uint8_t*)malloc(out_block_size * sizeof(uint8_t));

        set_sample_rate(dev, samp_rate);
        set_frequency(dev,   frequency);
        set_manual_gain(dev, gain);

        if (rtlsdr_reset_buffer(dev) < 0)
            fprintf(stderr, "WARNING: Failed to reset buffers.\n");

        fprintf(stderr, "Reading samples in async mode...\n");
        r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)dev, 0, out_block_size);
        rtlsdr_close(dev);
        free(buffer);


        if (do_exit) {
            fprintf(stderr, "\nUser cancel, exiting...\n");
        }
        else {
            fprintf(stderr, "\nLibrary error %d - retrying in 5 seconds...\n", r);
            sleep(5);
        }
    } while(!do_exit);
    parser.closeFiles();
    unlink_pid();
    return r >= 0 ? r : -r;
}
