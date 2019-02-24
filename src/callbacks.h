#ifndef CALLBACKS_H
#define CALLBACKS_H

// Callbacks that get registered with the core 6502 
// to perform specific emu65/EHBASIC functions:
void emu65_write_callback(const core_t *core, const uint16_t addy);
void emu65_read_callback(const core_t *core, const uint16_t addy);

#endif
