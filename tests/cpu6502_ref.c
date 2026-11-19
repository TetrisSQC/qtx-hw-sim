#define CHIPS_IMPL
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "m6502.h"
#include "c64-roms.h"
static uint8_t mem[65536];
static uint8_t in_cb(void *u) { (void)u; return 0x3F; }
static void out_cb(uint8_t d, void *u) { (void)d; (void)u; }
int main(int argc, char **argv) {
    long n = argc > 1 ? atol(argv[1]) : 2000000;
    m6502_t cpu;
    for (int i = 0; i < 65536; i++) mem[i] = (i & 64) ? 0xFF : 0;
    memcpy(mem + 0xA000, dump_c64_basic_bin, 8192);
    memcpy(mem + 0xE000, dump_c64_kernalv3_bin, 8192);
    uint64_t pins = m6502_init(&cpu, &(m6502_desc_t){ .m6510_in_cb = in_cb, .m6510_out_cb = out_cb, .m6510_io_floating = 0xC0 });
    uint32_t h = 0;
    for (long i = 0; i < n; i++) {
        if (i == 5) pins &= ~M6502_RES;   /* release */
        if ((i % 20000) == 10000) pins |= M6502_IRQ; else pins &= ~M6502_IRQ;
        pins = m6502_tick(&cpu, pins);
        uint16_t a = M6502_GET_ADDR(pins);
        if (M6510_CHECK_IO(pins)) pins = m6510_iorq(&cpu, pins);
        else if (pins & M6502_RW) {
            uint8_t v = mem[a];
            if (a >= 0xD000 && a < 0xE000) v = (uint8_t)(a ^ (i >> 6));   /* fake IO: varying */
            M6502_SET_DATA(pins, v);
        } else if (a < 0xA000 || (a >= 0xC000 && a < 0xE000)) mem[a] = M6502_GET_DATA(pins);
        h = h * 31 + (uint32_t)(pins & 0x1FFFFFF) + cpu.A * 7 + cpu.X * 13 + cpu.Y * 17 + cpu.P * 19 + cpu.S;
        if ((i & 0x3FFFF) == 0x3FFFF) printf("%ld PC=%04X A=%02X X=%02X Y=%02X S=%02X P=%02X h=%08X\n", i, cpu.PC, cpu.A, cpu.X, cpu.Y, cpu.S, cpu.P, h);
    }
    printf("final h=%08X\n", h);
    return 0;
}
