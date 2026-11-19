"""Generate hwsim.m6502.pas from chips/m6502.h (floooh's cycle-stepped 6502/6510).
The instruction decoder (a switch over (opcode<<3)|cycle) and the tick
prologue are translated mechanically by cxlate.py; the ALU helpers, init and
the 6510 port are written by hand below. Pin layout is the same as the C
code, but only bits 0..30 exist (the 6510 port pins P0-P5 of the C pin mask
are not used by the board: it reads io_ddr / io_out directly)."""
import re, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cxlate

root, out = sys.argv[1], sys.argv[2]
src = open(os.path.join(root, "chips", "m6502.h"), encoding="utf-8").read()


def strip_comments(s):
    s = re.sub(r"/\*.*?\*/", "", s, flags=re.S)
    return re.sub(r"//[^\n]*", "", s)


fm, om = cxlate.parse_defines(src)
fm["M6510_SET_PORT"] = (["p", "d"], "")      # port pins are not in our pin mask
for k in list(om):
    if not (k.startswith("M6502_") or k.startswith("M6510_")):
        del om[k]
for k in ("M6502_PIN_MASK", "M6510_PORT_BITS", "M6510_P0", "M6510_P1", "M6510_P2", "M6510_P3",
          "M6510_P4", "M6510_P5"):
    om.pop(k, None)

body = src[src.index("uint64_t m6502_tick(m6502_t* c, uint64_t pins) {"):]
body = body[body.index("{") + 1:]
pro, rest = body.split("switch (c->IR++) {", 1)
sw = rest[:rest.index("\n    }\n    M6510_SET_PORT")]
pro = strip_comments(pro).replace("_RD();", "")
assert "_RD()" not in pro

FIELDS16 = ["IR", "PC", "AD", "irq_pip", "nmi_pip"]
FIELDS8 = ["A", "X", "Y", "S", "P", "brk_flags", "bcd_enabled", "io_ddr", "io_inp", "io_out", "io_pins",
           "io_pullup", "io_floating", "io_drive"]
widths = {f: 16 for f in FIELDS16}
widths.update({f: 8 for f in FIELDS8})
widths["FPINS"] = 0
widths["pins"] = 0


def rename(e):
    if e[0] == "field" and e[1] == ("id", "c"):
        return "FPINS" if e[2] == "PINS" else e[2]
    if e[0] == "id":
        return e[1]
    return None


funcs = {
    "_m6502_adc": ("Adc", False, 1), "_m6502_sbc": ("Sbc", False, 1), "_m6502_cmp": ("Cmp", False, 1),
    "_m6502_asl": ("Asl", False, 1), "_m6502_lsr": ("Lsr", False, 1), "_m6502_rol": ("Rol", False, 1),
    "_m6502_ror": ("Ror", False, 1), "_m6502_bit": ("Bit", False, 1), "_m6502_arr": ("Arr", False, 1),
    "_m6502_sbx": ("Sbx", False, 1),
}


def tr(code, indent):
    code = cxlate.expand_macros(code, fm, om)
    em = cxlate.Emitter(widths, rename, funcs)
    return [indent + l for l in cxlate.translate(code, em)]


L = []
for line in tr(pro, "  "):
    L.append(line)
L.append("  irv := IR;")
L.append("  IR := (IR + 1) and $FFFF;")
L.append("  pins := pins or M6502_RW;")
L.append("  case irv of")
cases = re.findall(r"case \(0x([0-9A-F]{2})<<3\)\|(\d): (.*?)break;\n", sw)
assert len(cases) == 2048, len(cases)
for op, cyc, code in cases:
    code = strip_comments(code)
    if code.strip().startswith("assert(false)"):
        continue
    stm = tr(code, "      ")
    L.append("    $%03X: begin { %s/%s }" % ((int(op, 16) << 3) | int(cyc), op, cyc))
    L.extend(stm)
    L.append("    end;")
L.append("  end;")

decoder = "\n".join(L)

consts = []
for k, v in om.items():
    val = cxlate.expand_macros(v, fm, om)
    val = re.sub(r"(\d+)ULL|(\d+)ULL", r"\1", val)
    try:
        ev = eval(val.replace("ULL", "").replace("<<", "<<"))
    except Exception as ex:
        continue
    if ev >= 1 << 32:
        continue
    consts.append("  %s = %d;" % (k, ev))

tmpl = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "m6502.pas.tmpl"), encoding="utf-8").read()
res = tmpl.replace("{%CONSTS%}", "\n".join(consts)).replace("{%DECODER%}", decoder)
open(os.path.join(out, "hwsim.m6502.pas"), "w", encoding="utf-8", newline="\n").write(res)
print("m6502: %d cases, %d lines" % (len(cases), res.count("\n")))
