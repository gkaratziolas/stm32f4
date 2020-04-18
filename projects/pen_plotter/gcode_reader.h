#ifndef GCODE_READER_H_
#define GCODE_READER_H_

#include "fifo.h"

#define GCODE_MAX_STRING_LEN 256
#define GCODE_MAX_CODES_PER_LINE 100

#define GCODE_MAX_VARS 7

// Supported gcodes
enum gcode_code {
        gcode_NONE = 0,
        // G codes
        gcode_G00,
        gcode_G01,
        gcode_G02,
        gcode_G03,
        gcode_G21,
        gcode_GXX,
        // M codes
        gcode_M02,
        gcode_M03,
        gcode_M05,
        gcode_MXX,
};

struct gcode_word {
        char name;
        float value;
};

struct gcode_command {
        enum gcode_code code;
        struct gcode_word vars[GCODE_MAX_VARS];
};

int gcode_read_line(struct fifo *gcode_command_fifo,
                    char *gcode_string, int str_length);
int gcode_read_chunk(struct gcode_word *gword, char *gcode_string, int i0, int i1);

int gcode_process_codes(struct fifo *gcode_command_fifo,
                        struct fifo *gcode_code_fifo);

int gcode_command_read_var(struct gcode_command *gcommand, char name, float *value);

#endif // GCODE_READER_H_
