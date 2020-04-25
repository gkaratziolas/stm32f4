#include "gcode_reader.h"

enum gcode_read_state {
        IDLE,
        IN_COMMENT,
        IN_COMMAND,
        END
};

void gcode_command_reset(struct gcode_command *gcommand);
int gcode_command_add_var(struct gcode_command *gcommand, char name, float value);
enum gcode_code gcode_word_to_code(struct gcode_word *gword);

char is_letter(char x);
char is_numerical(char x);
char upper(char x);

int gcode_read_line(struct fifo *gcode_command_fifo,
                    char *gcode_string, int str_length)
{
        char c = '\0';
        int i, i0, i1;

        int syntax_error = 0;

        enum gcode_read_state state = IDLE;

        struct gcode_word gword;
        struct gcode_word gwords[GCODE_MAX_CODES_PER_LINE];
        struct fifo gword_fifo = fifo_init(gwords,
                                           sizeof(struct gcode_word),
                                           GCODE_MAX_CODES_PER_LINE);

        int gcommand_count = 0;
        for (i = 0; i < str_length; i++) {
                if ((c == 'M') || (c == 'G'))
                        gcommand_count++;
        }
        if (gcommand_count == 0) {
                // no commands found in string
                return READ_SUCCESS;
        }
        if (gcommand_count > fifo_space(gcode_command_fifo)) {
                return READ_ERR_NO_SPACE;
        }

        for (i = 0; i < str_length; i++) {
                c = gcode_string[i];

                switch (state) {
                case IDLE:
                        if (is_letter(c)) {
                                state = IN_COMMAND;
                                i0 = i;
                        } else if (c == '(') {
                                state = IN_COMMENT;
                        } else if (c == ';') {
                                state = END;
                        }
                        break;
                case IN_COMMENT:
                        if (c == ')') {
                                state = IDLE;
                        }
                        break;
                case IN_COMMAND:
                        if ((i == (str_length - 1)) || (!is_numerical(gcode_string[i+1]))){
                                // new command found
                                i1 = i;
                                syntax_error = gcode_read_chunk(&gword, gcode_string, i0, i1);
                                fifo_push(&gword_fifo, &gword);
                                state = IDLE;
                        }
                        break;
                case END:
                default:
                        break;
                }
                if (syntax_error != 0) {
                        return READ_ERR_SYNTAX * syntax_error;
                }
        }
        gcode_process_codes(gcode_command_fifo, &gword_fifo);
        return READ_SUCCESS;
}

/*
 * Takes a chunk of gcode (e.g "G 01.2 412 ") fills in a word struct
 * Returns 0 for sucess, otherwise returns 1 + location of syntax error
 */
int gcode_read_chunk(struct gcode_word *gword, char *gcode_string, int i0, int i1)
{
        int i = i0;
        float sign = 1;

        gword->name  = gcode_string[i0];
        gword->value = 0.f;

        i++;

        // Ignore any leading whitespace
        while (gcode_string[i] == ' ') {
                i++;
        }

        // Look for exactly one leading plus or minus symbol
        if (gcode_string[i] == '+') {
                i++;
        } else if (gcode_string[i] == '-') {
                sign = -1;
                i++;
        }

        int val = 0;
        int div = 0;
        while (i < i1+1) {
                // allow for one or fewer decimal points
                if (gcode_string[i] == '.') {
                        if (div == 0) {
                                div = 1;
                        } else {
                                return i+1;
                        }
                // allow many [0-9]
                } else if (gcode_string[i] >= '0' && gcode_string[i] <= '9') {
                        val = val * 10;
                        val += (int)(gcode_string[i] - '0');
                        if (div > 0) {
                                div *= 10;
                        }
                // ignore any spaces
                } else if (gcode_string[i] == ' ') {
                        ;;
                // anything else is illegal
                } else {
                        return i+1;
                }
                i++;
        }
        if (div == 0)
                div = 1;
        gword->value = (float) (sign * val) / (float) div;
        return 0;
}

enum gcode_code gcode_word_to_code(struct gcode_word *gword)
{
        int num = (int) (gword->value + 0.1f);
        if (gword->name == 'G') {
                switch (num) {
                case 0:
                        return gcode_G00;
                case 1:
                        return gcode_G01;
                case 2:
                        return gcode_G02;
                case 3:
                        return gcode_G03;
                case 21:
                        return gcode_G21;
                default:
                        return gcode_GXX;
                }
        }
        if (gword->name == 'M') {
                switch (num) {
                case 2:
                        return gcode_M02;
                case 3:
                        return gcode_M03;
                case 5:
                        return gcode_M05;
                default:
                        return gcode_MXX;
                }
        }
        return gcode_NONE;
}

int gcode_process_codes(struct fifo *gcommand_fifo, struct fifo *gword_fifo)
{
        struct gcode_command gcommand;
        struct gcode_word    gword;

        gcode_command_reset(&gcommand);

        // Decode first code
        if (fifo_pop(gword_fifo, &gword) == FIFO_ERR_EMPTY) {
                return 0;
        }
        // ignore line numbers
        // look for next code
        if (gword.name == 'N') {
                if (fifo_pop(gword_fifo, &gword) == FIFO_ERR_EMPTY) {
                        return 0;
                }
        }
        // if M or G, start new command
        if ((gword.name == 'G') || (gword.name == 'M')) {
                gcommand.code = gcode_word_to_code(&gword);
        } else {
                // if the line starts with a variable, dump everything
                return -1;
        }

        // At this point, tmp_instruction contains a valid code
        while (!fifo_empty(gword_fifo)) {
                fifo_pop(gword_fifo, &gword);
                switch(gword.name) {
                case 'G':
                case 'M':
                        fifo_push(gcommand_fifo, &gcommand);
                        gcode_command_reset(&gcommand);
                        gcommand.code = gcode_word_to_code(&gword);
                        break;
                case 'N':
                        // syntax error, can't have line number in middle of line!
                        return -1;
                case 'F':
                case 'I':
                case 'J':
                case 'X':
                case 'Y':
                case 'Z':
                        gcode_command_add_var(&gcommand, gword.name, gword.value);
                        break;
                default:
                        // unsupported variable
                        return -1;
                }
        }
        fifo_push(gcommand_fifo, &gcommand);
        return 0;
}

void gcode_command_reset(struct gcode_command *gcommand)
{
        int i;
        gcommand->code = gcode_NONE;
        for (i=0; i<GCODE_MAX_VARS; i++) {
                gcommand->vars[i].name  = 0;
                gcommand->vars[i].value = 0.f;
        }
}

int gcode_command_add_var(struct gcode_command *gcommand, char name, float value)
{
        int i;
        for (i=0; i<GCODE_MAX_VARS; i++) {
                if (gcommand->vars[i].name == name) {
                        // variable already defined
                        return -1;
                }
                if (gcommand->vars[i].name == 0) {
                        // first empty slot found
                        break;        
                }
        }
        if (i == GCODE_MAX_VARS)  {
                // no space left
                return -1;
        }
        gcommand->vars[i].name  = name;
        gcommand->vars[i].value = value;
        return 0;
}

int gcode_command_read_var(struct gcode_command *gcommand, char name, float *value)
{
        int i;
        for (i=0; i<GCODE_MAX_VARS; i++) {
                if (gcommand->vars[i].name == name) {
                        *value = gcommand->vars[i].value;
                        return 0;
                }
        }
        // var not found
        return -1;
}

// Returns 1 for ascii letter [a-zA-Z], 0 otherwise
char is_letter(char x)
{
        if ((x >= 'A') && (x <= 'Z'))
                return 1;
        if ((x >= 'a') && (x <= 'z'))
                return 1;
        return 0;
}

// Returns 1 for ascii symbols used for writing numbers [0-1\+\-\.], 0 otherwise
char is_numerical(char x)
{
        if ((x >= '0') && (x <= '9'))
                return 1;
        if ((x == '-') || (x == '+') || (x == '.') || (x == ' '))
                return 1;
        return 0;
}

char upper(char x)
{
        if ((x >= 'A') && (x <= 'Z'))
                return x;
        if ((x >= 'a') && (x <= 'z'))
                return (x - 'a') + 'A';
        return x;    
}
