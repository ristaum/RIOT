#include <stdio.h>
#include "periph/i2c.h"

int main(void)
{
    puts("Start");

    i2c_t dev = 0;
    int error = 0;

    i2c_init(dev);
    i2c_acquire(dev);

    uint8_t data = 0;
    // if ((error = i2c_read_reg(dev, 0x76, 0xD0, &data, 0)) != 0)
    //     printf("error: %d\r\n", error);
    // else
    //     printf("data: %d\r\n", data);

    if ((error = i2c_read_reg(dev, 0x76, 0x01, &data, 0)) != 0)
        printf("error: %d TT\r\n", error);
    else
        printf("data: %d TT\r\n", data);

    i2c_release(dev);

    return 0;
}
