#ifndef COMMAND_USART_H_
#define COMMAND_USART_H_

#define MAX_DATA_LENGTH 255

struct command_packet = {
        uint8_t command;
        uint8_t data_length;
        uint8_t data[MAX_DATA_LENGTH];
        uint8_t crc;
};

bool command_usart_receive(struct command_packet *packet_copy);
bool command_usart_transmit(struct command_packet *rx_packet);
bool command_usart_check_packet_dropped(void);

#endif // COMMAND_USART_H_
