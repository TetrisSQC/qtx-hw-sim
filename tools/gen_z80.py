"""Generate hwsim.z80.pas from chips/z80.h (floooh's Z80, altered by c64-hw-sim to
run on the edges of its clock).

- Registers: one 20 byte buffer seen as Uint8Array and Uint16Array, exactly
  like the C unions (little endian: pc = pcl | pch << 8, af = f | a << 8...).
- Pins: the address bus is not in the pin mask but in the field 'ab' (it is
  an output only); the control pins are bits 0..15, D0-D7 bits 16..23.
- The helper functions and the decoder are translated by cxlate.py; the
  gotos of the decoder become a label variable and a break out of a
  'repeat ... until True' around the case statement.
Usage: python gen_z80.py <c-src-root> <out-dir>"""
import re, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cxlate

root, out = sys.argv[1], sys.argv[2]
src = open(os.path.join(root, "chips", "z80.h"), encoding="utf-8").read()


def strip_comments(s):
    s = re.sub(r"/\*.*?\*/", "", s, flags=re.S)
    return re.sub(r"//[^\n]*", "", s)


# --- pins (our layout) ------------------------------------------------------
PINS = ["M1", "MREQ", "IORQ", "RD", "WR", "HALT", "INT", "RES", "NMI", "WAIT", "RFSH", "BUSRQ", "BUSAK",
        "CLK", "DOE", "AOE"]
pin_val = {name: 1 << i for i, name in enumerate(PINS)}
pin_val["IEIO"] = 1 << 24
pin_val["RETI"] = 1 << 25

fm, om = cxlate.parse_defines(src)
om = {k: v for k, v in om.items() if not k.startswith("Z80_PIN_") and k not in ("Z80_PIN_MASK",)}
for name, v in pin_val.items():
    om["Z80_" + name] = str(v)
CTRL = pin_val["M1"] | pin_val["MREQ"] | pin_val["IORQ"] | pin_val["RD"] | pin_val["WR"] | pin_val["RFSH"]
om["Z80_CTRL_PIN_MASK"] = str(CTRL)
OUTMASK = CTRL | pin_val["HALT"] | pin_val["BUSAK"] | pin_val["RETI"] | pin_val["DOE"] | pin_val["AOE"]
om["Z80_OUTPUT_PIN_MASK"] = str(OUTMASK)
om["Z80_INPUT_PIN_MASK"] = str(0xFF0000 | pin_val["INT"] | pin_val["RES"] | pin_val["NMI"] | pin_val["WAIT"] |
                                pin_val["BUSRQ"] | pin_val["CLK"])
om.pop("_Z80_UNREACHABLE", None)
fm["_Z80_UNLIKELY"] = (["x"], "(x)")
for k in list(om):
    if k.startswith("Z80_") and ("ULL" in om[k] and "<<" in om[k] and "PIN" in om[k]):
        del om[k]

# --- registers ---------------------------------------------------------------
SLOT = {"pc": 0, "af": 1, "bc": 2, "de": 3, "hl": 4, "ix": 5, "iy": 6, "wz": 7, "sp": 8, "ir": 9}
BYTE = {}
for name, lo, hi in [("pc", "pcl", "pch"), ("af", "f", "a"), ("bc", "c", "b"), ("de", "e", "d"),
                     ("hl", "l", "h"), ("ix", "ixl", "ixh"), ("iy", "iyl", "iyh"), ("wz", "wzl", "wzh"),
                     ("sp", "spl", "sph"), ("ir", "r", "i")]:
    BYTE[lo] = SLOT[name] * 2
    BYTE[hi] = SLOT[name] * 2 + 1

FIELDS = {"step": 16, "brq_step": 16, "addr": 16, "dlatch": 8, "opcode": 8, "hlx_idx": 8,
          "pins": 0, "int_bits": 0, "nmi_latch": 0, "af2": 16, "bc2": 16, "de2": 16, "hl2": 16, "im": 8,
          "prefix_active": 1, "iff1": 1, "iff2": 1}
FIELD_NAME = {"pins": "FPINS"}
BOOLS = {"prefix_active", "iff1", "iff2"}
LOCAL_RENAME = {"in": "inp"}

func_names = {}     # c name -> pascal name


def rename(e, em):
    if e[0] == "field":
        base, f = e[1], e[2]
        if base == ("id", "cpu"):
            if f in SLOT:
                return "r16[%d]" % SLOT[f]
            if f in BYTE:
                return "r8[%d]" % BYTE[f]
            if f in FIELDS:
                return FIELD_NAME.get(f, f)
            return None
        # cpu->hlx[idx].hl / .h / .l
        if base[0] == "index" and base[1] == ("field", ("id", "cpu"), "hlx"):
            idx = em.as_int(em.ex(base[2]))
            if f == "hl":
                return "r16[4 + %s]" % idx
            if f == "l":
                return "r8[(4 + %s) * 2]" % idx
            if f == "h":
                return "r8[(4 + %s) * 2 + 1]" % idx
        return None
    if e[0] == "id":
        n = e[1]
        if n == "_z80_szp_flags":
            return "SZP_FLAGS"
        if n == "_z80_indirect_table":
            return "INDIRECT_TABLE"
        return LOCAL_RENAME.get(n, n)
    return None


widths = {}
for f, w in FIELDS.items():
    widths[FIELD_NAME.get(f, f)] = 0 if w == 1 else w
bool_ids = {FIELD_NAME.get(f, f) for f in BOOLS}

TYPES = {"uint8_t": 8, "uint16_t": 16, "uint32_t": 32, "uint64_t": 0, "int": 0, "bool": 1, "void": None}

# --- helper functions --------------------------------------------------------
SKIP = {"_z80_set_ab_ctl", "_z80_res"}     # hand-written in the template
fstart = src.index("static inline uint64_t _z80_halt")
fend = src.index("uint64_t z80_prefetch(", fstart)
fsrc = strip_comments(src[fstart:fend])
funcs = []
for m in re.finditer(r"static (?:inline )?(\w+) (_z80_\w+)\(([^)]*)\) \{", fsrc):
    rtype, name, params = m.group(1), m.group(2), m.group(3)
    i = m.end() - 1
    depth = 0
    for j in range(i, len(fsrc)):
        if fsrc[j] == "{":
            depth += 1
        elif fsrc[j] == "}":
            depth -= 1
            if depth == 0:
                break
    body = fsrc[i + 1:j]
    plist = []
    drop = 0
    for k, prm in enumerate([x.strip() for x in params.split(",") if x.strip()]):
        if prm.startswith("z80_t*"):
            drop = 1
            continue
        ty, pn = prm.rsplit(" ", 1)
        plist.append((TYPES[ty.replace("const ", "")], LOCAL_RENAME.get(pn, pn)))
    pname = "Z" + name[len("_z80"):]
    func_names[name] = pname
    funcs.append((name, pname, rtype, plist, drop, body))

func_table = {}
for name, pname, rtype, plist, drop, body in funcs:
    func_table[name] = (pname, rtype == "bool", drop)
func_table["_z80_set_ab_ctl"] = ("Z_set_ab_ctl", False, 0)
func_table["_z80_res"] = ("Z_res", False, 1)
func_table["_z80_reset_regs"] = ("ResetRegs", False, 1)


def make_em(ret_width=0, ret_bool=False, gotos=None):
    return cxlate.Emitter(widths, rename, func_table, bool_ids=bool_ids, gotos=gotos,
                          ret_width=ret_width, ret_bool=ret_bool)


decls, impls = [], []
for name, pname, rtype, plist, drop, body in funcs:
    if name in SKIP:
        continue
    rw = TYPES[rtype]
    params = "; ".join("%s: %s" % (pn, "Boolean" if w == 1 else "Integer") for w, pn in plist)
    if rtype == "void":
        sig = "procedure %s(%s)" % (pname, params)
    else:
        sig = "function %s(%s): %s" % (pname, params, "Boolean" if rtype == "bool" else "Integer")
    decls.append("    " + sig.replace("()", "") + ";")
    em = make_em(ret_width=rw if rw not in (None, 1) else 0, ret_bool=(rtype == "bool"))
    for w, pn in plist:
        if w == 1:
            em.bool_ids.add(pn)
    code = cxlate.expand_macros(body, fm, om)
    lines = cxlate.translate(code, em)
    locals_ = dict(em.locals)
    pre = []
    for w, pn in plist:
        if w in (8, 16):
            pre.append("  %s := %s;" % (pn, cxlate.Emitter.mask(pn, w)))
    var = ""
    if locals_:
        var = "var\n" + "".join("  %s: %s;\n" % (ln, "Boolean" if lw == 1 else "Integer")
                                for ln, lw in locals_.items())
    impls.append("%s;\n%sbegin\n%s\nend;\n" % (sig.replace("procedure ", "procedure TZ80.").replace(
        "function ", "function TZ80.").replace("()", ""), var,
        "\n".join(pre + ["  " + l for l in lines])))

# --- the decoder ---------------------------------------------------------------
tick = src[src.index("uint64_t z80_tick(z80_t* cpu, uint64_t in) {"):]
sw_start = tick.index("switch (cpu->step) {")
sw_end = tick.index("fetch_next:")
switch_code = strip_comments(tick[sw_start:sw_end])
gotos = {
    "fetch_next": ["lbl := 1;"],
    "rise_done": ["lbl := 2;"],
    "rise_nosample": ["lbl := 3;"],
    "edge_done": ["lbl := 4;"],
}
em = make_em(gotos=gotos)
code = cxlate.expand_macros(switch_code, fm, om)
dec_lines = cxlate.translate(code, em)
tickvars = "".join("  %s: %s;\n" % (n, "Boolean" if w == 1 else "Integer") for n, w in em.locals.items())
decoder = "\n".join("    " + l for l in dec_lines)

# --- constants -----------------------------------------------------------------
consts = []
for k, v in om.items():
    if not (k.startswith("Z80_") or k.startswith("_Z80_CTRL")):
        continue
    val = cxlate.expand_macros(v, fm, om)
    try:
        ev = cxlate.eval_const(cxlate.Parser(cxlate.tokenize(val)).expr())
    except Exception:
        continue
    if ev >= 1 << 32 or ev < 0:
        continue
    consts.append("  %s = %d;" % (k.lstrip("_"), ev))


def c_table(name):
    m = re.search(r"static const uint8_t %s\[256\] = \{(.*?)\};" % name, src, re.S)
    vals = [int(x, 0) for x in re.findall(r"0x[0-9a-fA-F]+|\b\d+\b", strip_comments(m.group(1)))]
    assert len(vals) == 256, (name, len(vals))
    return ", ".join(("\n    " if i % 16 == 0 else "") + "$%02X" % v for i, v in enumerate(vals))


tmpl = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "z80.pas.tmpl"), encoding="utf-8").read()
res = (tmpl.replace("{%CONSTS%}", "\n".join(consts))
       .replace("{%DECLS%}", "\n".join(decls))
       .replace("{%IMPLS%}", "\n".join(impls))
       .replace("{%DECODER%}", decoder)
       .replace("{%TICKVARS%}", tickvars)
       .replace("{%SZP%}", c_table("_z80_szp_flags"))
       .replace("{%INDIRECT%}", c_table("_z80_indirect_table")))
open(os.path.join(out, "hwsim.z80.pas"), "w", encoding="utf-8", newline="\n").write(res)
print("z80: %d helper functions, %d decoder lines" % (len(impls), len(dec_lines)))
