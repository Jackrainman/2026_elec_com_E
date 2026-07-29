# Raspberry Pi serial receiver

The Raspberry Pi sends one ASCII command per line:

```text
x,y,th,a\r\n
```

Example:

```text
1.23,4.56,7.89,10.11\r\n
```

Initialize the UART DMA and `uart_ex` first, then start the receiver:

```c
raspi_serial_init(&huart4);
```

Read the latest command in the robot-arm task:

```c
raspi_serial_data_t command;
static uint32_t last_sequence;

if (raspi_serial_get_latest(&command) &&
    command.sequence != last_sequence) {
    last_sequence = command.sequence;

    /* Use command.x, command.y, command.th and command.a here. */
}
```

`sequence` increases after every valid command, so the motion task only acts
once on each received command.
