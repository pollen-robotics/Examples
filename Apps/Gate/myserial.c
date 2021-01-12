#include "myserial.h"
#include "main.h"

int serial_write(uint8_t *data, int len)
{
    for (int i=0; i < len; i++)
    {
        while (!LL_USART_IsActiveFlag_TXE(USART3))
            ;
        LL_USART_TransmitData8(USART3, data[i]);
    }
    return 0;
}