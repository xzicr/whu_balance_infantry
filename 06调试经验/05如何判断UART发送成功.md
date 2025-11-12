# 如何判断UART发送成功

可以尝试采用temp= HAL+UART_Transmit()返回值判断

HAL_OK   0

HAL_ERROR  1

HAL_BUSY    2

HAL_TIMEOUT  3