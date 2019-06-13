#include "stm32f4xx_usart.h"
#include "command_usart.h"

#define MAX_DATA_LENGTH 255

const uint8_t STARTA = 0x12;
const uint8_t STARTB = 0x34;

static bool rx_buffer_empty = false;
static bool rx_command_dropped = false;

static enum {
        WAIT_STARTA_STATE,
        WAIT_STARTB_STATE,
        WAIT_COMMAND_STATE,
        WAIT_LENGTH_STATE,
        WAIT_DATA_STATE,
        WAIT_CRC_STATE
} rx_state = WAIT_STARTA_STATE;

static struct command_packet rx_command_packet;

void irq()
{
        // TODO: Check if data available
        uint8_t rx_byte = USART_ReceiveData(debug_usart);
        uint8_t rx_data_count;

        switch (state) {
        case WAIT_STARTA_STATE:
                if (rx_byte == STARTA) {
                        if (!rx_buffer_empty) {
                                rx_command_dropped = true;
                        } else {
                                rx_state = WAIT_STARTB_STATE;
                        }
                }
                break;

        case WAIT_STARTB_STATE:
                if (rx_byte == STARTB)
                        rx_state = WAIT_COMMAND_STATE;
                break;

        case WAIT_COMMAND_STATE:
                rx_command_packet.command = rx_byte;
                rx_state = WAIT_LENGTH_STATE;
                break;

        case WAIT_LENGTH_STATE:
                rx_command_packet.data_length = rx_byte;
                rx_data_count = 0;

                if (rx_command_packet.data_length == 0)
                        rx_state = WAIT_CRC_STATE;
                else
                        rx_state = WAIT_DATA_STATE;
                break;

        case WAIT_DATA_STATE:
                rx_command_packet.data[rx_data_count] = rx_byte;
                rx_data_count++;
                if (rx_data_count == rx_command_packet.data_length)
                        rx_state = WAIT_CRC_STATE;
                break;

        case WAIT_CRC_STATE:
                rx_command_packet.crc = rx_byte;
                rx_buffer_empty = false;
                state = WAIT_STARTA_STATE;
                break;
        }
}

/*
 * If a packet is available, copy the packet into a passed-in struct and return
 * true. Else return false.
 */
bool read_command_buffer(struct command_packet *packet_copy)
{
        if (rx_buffer_empty) {
                return false;
        }
        packet_copy->command     = rx_command_packet.command;
        packet_copy->data_length = rx_command_packet.data_length;
        packet_copy->crc         = rx_command_packet.crc;

        int i, j;
        for (i=0; i<data_length; i++) {
                packet_copy->data[i] = rx_command_packet.data[i];
        }
        // Ensure top of recieved data buffer is empty
        for (j=i; i<MAX_DATA_LENGTH+1; j++) {
                packet_copy->data[j] = 0;
        }

        rx_buffer_empty = true;
        return true;
}

bool check_packet_dropped(void)
{
        if (rx_command_dropped) {
                rx_command_dropped = false;
                return true;
        }
        return false;
}
