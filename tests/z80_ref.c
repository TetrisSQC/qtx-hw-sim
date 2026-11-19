#define CHIPS_IMPL
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "z80.h"
#include "zx48-rom.h"
static uint8_t mem[65536];
int main(int argc, char **argv) {
    long n = argc > 1 ? atol(argv[1]) : 4000000;
    z80_t cpu;
    memcpy(mem, zx48_rom, 16384);
    for (int i = 16384; i < 65536; i++) mem[i] = (uint8_t)(i * 7);
    uint64_t pins = z80_init(&cpu);
    uint32_t h = 0;
    for (long i = 0; i < n; i++) {
        pins ^= Z80_CLK;
        if ((i % 140000) < 64) pins |= Z80_INT; else pins &= ~Z80_INT;
        pins = z80_tick(&cpu, pins);
        uint16_t a = Z80_GET_ADDR(pins);
        uint8_t d = 0;
        if ((pins & Z80_MREQ) && (pins & Z80_RD)) { d = mem[a]; Z80_SET_DATA(pins, d); }
        else if ((pins & Z80_MREQ) && (pins & Z80_WR)) { if (a >= 16384) mem[a] = Z80_GET_DATA(pins); d = Z80_GET_DATA(pins); }
        else if ((pins & Z80_IORQ) && (pins & Z80_RD)) { d = (uint8_t)(0xBF ^ (i >> 8)); Z80_SET_DATA(pins, d); }
        else if (pins & Z80_IORQ) { d = 0xFF; Z80_SET_DATA(pins, 0xFF); }
        uint32_t ctl = (pins & Z80_M1 ? 1 : 0) | (pins & Z80_MREQ ? 2 : 0) | (pins & Z80_IORQ ? 4 : 0) |
                       (pins & Z80_RD ? 8 : 0) | (pins & Z80_WR ? 16 : 0) | (pins & Z80_HALT ? 32 : 0) |
                       (pins & Z80_RFSH ? 64 : 0);
        h = h * 31 + a + d * 65536 + ctl * 0x1000000 + cpu.pc * 3 + cpu.af * 5 + cpu.bc * 7 + cpu.de * 11 + cpu.hl * 13 + cpu.sp * 17;
        if ((i & 0x7FFFF) == 0x7FFFF) printf("%ld PC=%04X AF=%04X BC=%04X DE=%04X HL=%04X SP=%04X IX=%04X IY=%04X h=%08X\n",
            i, cpu.pc, cpu.af, cpu.bc, cpu.de, cpu.hl, cpu.sp, cpu.ix, cpu.iy, h);
    }
    printf("final h=%08X\n", h);
    return 0;
}
