#include "s31_protocol.h"

#include <string.h>
#include "../common/crc8.h"

uint8_t s31_frame_encode(uint8_t command, const uint8_t *payload, uint8_t length,
                         uint8_t frame[S31_MAX_FRAME])
{
    if (length > S31_MAX_PAYLOAD || (length != 0u && payload == NULL))
    {
        return 0u;
    }

    frame[0] = S31_SOF1;
    frame[1] = S31_SOF2;
    frame[2] = length;
    frame[3] = command;
    if (length != 0u)
    {
        memcpy(&frame[4], payload, length);
    }
    frame[4u + length] = c30d_crc8(&frame[2], (uint8_t)(length + 2u));
    return (uint8_t)(length + 5u);
}

s31_frame_result_t s31_frame_feed(s31_frame_parser_t *parser, uint8_t byte)
{
    switch (parser->state)
    {
        case S31_RX_HEADER_FIRST:
            if (byte == S31_SOF1)
            {
                parser->state = S31_RX_HEADER_SECOND;
            }
            break;

        case S31_RX_HEADER_SECOND:
            if (byte == S31_SOF2)
            {
                parser->state = S31_RX_LENGTH;
            }
            else if (byte != S31_SOF1)
            {
                parser->state = S31_RX_HEADER_FIRST;
            }
            break;

        case S31_RX_LENGTH:
            parser->length = byte;
            parser->index = 0u;
            if (byte > S31_MAX_PAYLOAD)
            {
                parser->state = S31_RX_HEADER_FIRST;
            }
            else
            {
                parser->crc = c30d_crc8_step(0u, byte);
                parser->state = S31_RX_COMMAND;
            }
            break;

        case S31_RX_COMMAND:
            parser->command = byte;
            parser->crc = c30d_crc8_step(parser->crc, byte);
            parser->state = S31_RX_DATA;
            break;

        case S31_RX_DATA:
            if (parser->index < parser->length)
            {
                parser->payload[parser->index++] = byte;
                parser->crc = c30d_crc8_step(parser->crc, byte);
                break;
            }
            parser->state = S31_RX_HEADER_FIRST;
            return byte == parser->crc ? S31_FRAME_READY : S31_FRAME_BAD_CRC;

        default:
            parser->state = S31_RX_HEADER_FIRST;
            break;
    }
    return S31_FRAME_NONE;
}
