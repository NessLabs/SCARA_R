#ifndef _SIMPLELCD_H
#define _SIMPLELCD_H

#include "SimpleGPIO.h" // Clase que controla pines
#include <FreeRTOS/FreeRTOS.h>

#define RS _gpio[8]
#define RW _gpio[9]
#define EN _gpio[10]

#define CMD_CLEAR 0b00000001
#define CMD_SECOND_LINE 0xC0
#define CMD_HOME 0b00000010
#define CMD_ENTRY_MODE 0b00000110
#define CMD_DISPLAY_ON 0b00001100
#define CMD_DISPLAY_OFF 0b00001000
#define CMD_DISP_SHIFT 0b00010100
#define CMD_FUNC_SET 0b00111110

class SimpleLCD
{
public:
    SimpleLCD();
    ~SimpleLCD();
    void setup(const uint8_t pins[]);
    void writeCommand(uint8_t command);
    void print(char character);

private:
    void setData(uint8_t data);
    SimpleGPIO _gpio[11];
};

#endif // _SIMPLELCD_H