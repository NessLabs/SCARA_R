#include "SimpleLCD.h"

SimpleLCD::SimpleLCD()
{
}

SimpleLCD::~SimpleLCD()
{
}

void SimpleLCD::setup(const uint8_t pins[])
{
    for (int i = 0; i < 11; i++)
        _gpio[i].setup(pins[i], GPIO);
    EN.set(1);
    vTaskDelay(10);
    writeCommand(CMD_DISPLAY_ON);
    writeCommand(CMD_ENTRY_MODE);
    writeCommand(CMD_FUNC_SET);
    writeCommand(CMD_DISP_SHIFT);
    writeCommand(CMD_CLEAR);
    writeCommand(CMD_HOME);
}

void SimpleLCD::writeCommand(uint8_t command) 
{
    EN.set(1);
    RS.set(0);
    RW.set(0);
    setData(command);
}

void SimpleLCD::print(char character)
{
    EN.set(1);
    RS.set(1);
    RW.set(0);
    setData(character);
}

void SimpleLCD::setData(uint8_t data)
{
    for (int i = 0; i < 8; i++)
        _gpio[i].set((data >> i) & 1);
    EN.set(0);
    vTaskDelay(1);
    EN.set(1);
}
