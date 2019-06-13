#ifndef COMMAND_USART_H_
#define COMMAND_USART_H_

struct command_packet = {
        uint8_t command;
        uint8_t data_length;
        uint8_t data[MAX_DATA_LENGTH];
        uint8_t crc;
};


bool read_command_buffer(struct command_packet *packet_copy);
bool check_packet_dropped(void);


#endif // COMMAND_USART_H_
