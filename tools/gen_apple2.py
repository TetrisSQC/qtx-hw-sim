"""gen_apple2.py - Apple II+ board data from the KiCad netlist (qtx/data/apple2p/apple2p.net.xml).

The schematic (captured from the Apple II Reference Manual) has no PCB, so the
geometry is synthesized: every chip sits at its Apple grid location (rows A..J
front to back, columns 1..15 left to right; chips lie horizontally, pin 1 at
the lower left), the six ROMs in row F, the 6502 at H6, the slots along the
back edge, passives in a strip on the right. Pads get real DIP spacing.

Fixes of the schematic capture (see data/apple2p/README.md):
- B5+B50 and B8+B80 are one 74LS174 each (spare flip-flops drawn apart);
- labels that differ only in case are one net (SOFT5/Soft5, ~{IO-SELECT-n});
- TTL chips whose power unit was not placed get VCC/GND tied to the rails.

Usage: python gen_apple2.py <netlist.xml> <out-dir>"""
import sys, os, re, collections
import xml.etree.ElementTree as ET

src, out = sys.argv[1], sys.argv[2]
root = ET.parse(src).getroot()

MERGE_PARTS = {"B50": "B5", "B80": "B8"}

# --- components ---------------------------------------------------------------
comps = {}
for c in root.iter("comp"):
    ref = MERGE_PARTS.get(c.get("ref"), c.get("ref"))
    val = c.findtext("value")
    part = c.find("libsource").get("part")
    if ref not in comps or c.get("ref") in MERGE_PARTS.values():
        comps.setdefault(ref, {"value": val, "part": part})
    if c.get("ref") in MERGE_PARTS:            # the full symbol names the part
        comps[ref] = {"value": val, "part": part}

# --- nets: union-find over net names -----------------------------------------------
parent = {}


def find(a):
    parent.setdefault(a, a)
    while parent[a] != a:
        parent[a] = parent[parent[a]]
        a = parent[a]
    return a


def union(a, b):
    ra, rb = find(a), find(b)
    if ra != rb:
        # keep the nicer name as root: not "Net-(", not "unconnected-", short
        def score(n):
            return (n.startswith("unconnected-"), n.startswith("Net-("), "/" in n, len(n))
        if score(rb) < score(ra):
            ra, rb = rb, ra
        parent[rb] = ra


def clean(n):
    n = n.split("/")[-1] if n.startswith("/") else n
    return n


pin_net = {}            # (ref, pin) -> raw net name
for n in root.iter("net"):
    name = n.get("name")
    find(name)
    for nd in n.findall("node"):
        ref = MERGE_PARTS.get(nd.get("ref"), nd.get("ref"))
        key = (ref, nd.get("pin"))
        if key in pin_net:
            a, b = pin_net[key], name
            # the same physical pin drawn twice: one of them is usually unconnected
            if a.startswith("unconnected-"):
                pin_net[key] = b
            elif not b.startswith("unconnected-"):
                union(a, b)
        else:
            pin_net[key] = name

# labels that differ only in case
by_lower = collections.defaultdict(list)
for name in list(parent):
    by_lower[clean(name).lower()].append(name)
for names in by_lower.values():
    for n in names[1:]:
        union(names[0], n)

# The RAM sheet was drawn by copy and paste: in rows C, D, E only the first chip
# (C3, D3, E3) has its row's RA6 and /CAS, the others got CRA6 and ~{DCAS}. On the
# board every chip of row X has XRA6 on pin 13 and ~{XCAS} on pin 15.
for ref, c in comps.items():
    m = re.match(r"^([CDE])(\d+)$", ref)
    if m and c["part"] == "4116":
        row = m.group(1)
        pin_net[(ref, "13")] = row + "RA6"
        pin_net[(ref, "15")] = "~{" + row + "CAS}"

# C13 (74LS51) makes SYNC = /(B14-1 B11-8 B14-13 + H2 H3 HBL) (Gayler, The Apple
# II Circuit Description, ch. 4): the capture has H3 on both pins 10 and 11.
pin_net[("C13", "11")] = "H2"

# B8 (the high nibble of the RAM output latch) was copied from B5 with the
# outputs unchanged: it drives LD0-LD3 like B5. It latches RDO4-RDO7 to LD4-LD7.
pin_net[("B8", "5")] = "LD6"
pin_net[("B8", "7")] = "LD7"
pin_net[("B8", "10")] = "LD4"
pin_net[("B8", "12")] = "LD5"

# ~{INH} (the slots' ROM inhibit, CS2 of the six ROMs) has no pull-up in the
# capture, and pin 7 (R6) of the RA1 pull-up pack is left open: that is it.
pin_net[("RA1", "7")] = "~{INH}"

# --- power pins of TTL chips -------------------------------------------------------
PINS = {"74LS00": 14, "74LS02": 14, "74LS04": 14, "74LS08": 14, "74LS11": 14, "74LS20": 14, "74LS32": 14,
        "74LS51": 14, "74LS74": 14, "74LS86": 14, "74LS138": 16, "74LS139": 16, "74LS151": 16, "74LS153": 16,
        "74LS161": 16, "74LS166": 16, "74LS174": 16, "74LS175": 16, "74LS194": 16, "74LS195": 16,
        "74LS251": 16, "74LS257": 16, "74LS259": 16, "74LS283": 16, "8T97": 16, "8T28": 16, "4116": 16,
        "ROM_Chip": 24, "2513": 24, "MOS6502": 40, "555": 8, "555_alt1": 8, "NE558": 16,
        "AppII_MemorySelectBlock": 14}
gnd_net = "GND"
vcc_net = "+5V"
for ref, c in comps.items():
    n = PINS.get(c["part"])
    if not n or c["part"] in ("MOS6502", "ROM_Chip", "2513", "4116", "AppII_MemorySelectBlock", "555",
                              "555_alt1", "NE558"):
        continue
    gnd, vcc = (7, 14) if n == 14 else (8, 16)
    for p, net in ((gnd, gnd_net), (vcc, vcc_net)):
        k = (ref, str(p))
        if k not in pin_net or pin_net[k].startswith("unconnected-"):
            pin_net[k] = net

# --- final nets ----------------------------------------------------------------------
roots = {}
for k, n in pin_net.items():
    r = find(n)
    roots.setdefault(r, None)
net_names = sorted(roots, key=lambda r: (r.startswith("unconnected-"), r))
net_index = {r: i for i, r in enumerate(net_names)}


def display(r):
    if r.startswith("unconnected-"):
        return r
    return clean(r)


# --- geometry ------------------------------------------------------------------------
PITCH = 2.54
COLW = 22.0          # column spacing (mm)
ROWH = 19.0          # row spacing (mm)
ROWS = "ABCDEFGHJ"   # A at the front (bottom of the view)
X0, Y0 = 20.0, 20.0


def grid_xy(ref):
    m = re.match(r"^([A-HJ])(\d+)$", ref)
    if m:
        row, col = ROWS.index(m.group(1)), int(m.group(2))
        return X0 + col * COLW, Y0 + (len(ROWS) - row) * ROWH + 30
    m = re.match(r"^ROM-([DEF])([08])$", ref)
    if m:
        # 24 pin parts are ~30 mm long: wider spacing than the 22 mm grid,
        # between F2 (col 2) and F12 (col 12)
        k = "DEF".index(m.group(1)) * 2 + (m.group(2) == "8")
        return X0 + 3.3 * COLW + k * 33.0, Y0 + (len(ROWS) - ROWS.index("F")) * ROWH + 30
    if ref == "U3":
        return X0 + 7.5 * COLW, Y0 + (len(ROWS) - ROWS.index("H")) * ROWH + 30
    m = re.match(r"^SLOT(\d)$", ref)
    if m:
        return X0 + (2 + int(m.group(1)) * 1.6) * COLW, Y0 + 12
    return None


def dip_pads(n, cx, cy, wide):
    """Horizontal DIP: pins 1..n/2 along the bottom left to right, n/2+1..n along the top right to left."""
    half = n // 2
    rowsep = 15.24 if wide else 7.62
    x0 = cx - (half - 1) * PITCH / 2
    pads = []
    for i in range(n):
        if i < half:
            pads.append((str(i + 1), x0 + i * PITCH, cy + rowsep / 2))
        else:
            pads.append((str(i + 1), x0 + (n - 1 - i) * PITCH, cy - rowsep / 2))
    return pads


def edge_pads(n, cx, cy):
    """A 50 pin slot: 2 x 25, like the edge connector of a card."""
    pads = []
    for i in range(n):
        side, k = divmod(i, n // 2)
        pads.append((str(i + 1), cx - 12 * 1.27 + (k if side == 0 else n // 2 - 1 - k) * 1.27, cy + (2 if side == 0 else -2)))
    return pads


pins_of = collections.defaultdict(dict)
for (ref, pin), n in pin_net.items():
    pins_of[ref][pin] = net_index[find(n)]

layout = []      # (ref, value, footprint, pads[(name, x, y, w, h, net)])
MISC_X0 = X0 + 15.6 * COLW   # strip on the right, packed row by row
MISC_W = 75.0                 # strip width (mm)
MISC_ROWH = 6.0
CONN_ROWH = 10.0              # connectors: one per row, room for the name above
CONNECTORS = ["A7", "J14", "CP0", "J18", "K14", "J0", "J2", "SPKR1"]
misc_x, misc_y = MISC_X0, Y0 + 30
def misc_key(r):
    if r in CONNECTORS:
        return (0, CONNECTORS.index(r), r)
    return (1, -len(pins_of.get(r, {})), r)


def on_grid(r):
    return grid_xy(r) is not None and r not in CONNECTORS


order = sorted(comps, key=lambda r: (not on_grid(r), (0, 0, r) if on_grid(r) else misc_key(r)))
for ref in order:
    c = comps[ref]
    pins = pins_of.get(ref, {})
    n = PINS.get(c["part"])
    pos = grid_xy(ref) if on_grid(ref) or ref.startswith("SLOT") else None
    if ref.startswith("SLOT"):
        pads = edge_pads(50, *pos)
        w, h = 0.9, 2.5
        fp = "SLOT_50"
    elif n and pos:
        if max([int(p) for p in pins if p.isdigit()] + [0]) > n:
            n = max(int(p) for p in pins if p.isdigit())
            n += n & 1
        pads = dip_pads(n, pos[0], pos[1], n >= 24)
        w, h = 1.6, 1.6
        fp = "DIP-%d" % n
    else:
        # passives and other parts: a row of pads in the strip on the right
        names = sorted(pins, key=lambda p: (len(p), p)) or ["1", "2"]
        width = (len(names) - 1) * PITCH
        if ref in CONNECTORS:
            pads = [(p, MISC_X0 + i * PITCH, misc_y) for i, p in enumerate(names)]
            misc_y += CONN_ROWH
            misc_x = MISC_X0
            w, h = 1.4, 1.4
            fp = "MISC-%d" % len(pads)
            full = [(name, x, y, w, h, pins.get(name, -1)) for name, x, y in pads]
            layout.append((ref, c["value"], fp, full))
            continue
        if misc_x > MISC_X0 and misc_x + width > MISC_X0 + MISC_W:
            misc_x, misc_y = MISC_X0, misc_y + MISC_ROWH
        pads = [(p, misc_x + i * PITCH, misc_y) for i, p in enumerate(names)]
        misc_x += width + 6.0
        w, h = 1.4, 1.4
        fp = "MISC-%d" % len(pads)
    full = [(name, x, y, w, h, pins.get(name, -1)) for name, x, y in pads]
    layout.append((ref, c["value"], fp, full))

xs = [p[1] for _, _, _, pads in layout for p in pads]
ys = [p[2] for _, _, _, pads in layout for p in pads]
outline = (min(xs) - 8, min(ys) - 8, max(xs) + 8, max(ys) + 8)


def pstr(s):
    return "'" + s.replace("'", "''") + "'"


L = ["unit hwsim.bd_apple2p;", "",
     "{ GENERATED by tools/gen_apple2.py from data/apple2p/apple2p.net.xml - do not edit.",
     "  Apple II+ motherboard: netlist captured from the Apple II Reference Manual (1979),",
     "  geometry synthesized from the Apple grid locations. }", "",
     "interface", "", "uses hwsim.boarddata;", "", "function BoardApple2p: TBoardData;", "",
     "implementation", "", "const"]
L.append("  NETS: array[0..%d] of String = [" % (len(net_names) - 1))
L.append(",\n".join("    " + pstr(display(n)) for n in net_names))
L.append("  ];")
allpads = [(ref, p) for ref, _, _, pads in layout for p in pads]
L.append("  PADS: array[0..%d] of TBdPad = [" % (len(allpads) - 1))
L.append(",\n".join("    (name: %s; x: %.3f; y: %.3f; w: %.3f; h: %.3f; net: %d)" % (pstr(p[0]), p[1], p[2], p[3], p[4], p[5])
                    for _, p in allpads))
L.append("  ];")
L.append("  COMPS: array[0..%d] of TBdComp = [" % (len(layout) - 1))
first = 0
rows = []
for ref, val, fp, pads in layout:
    rows.append("    (ref: %s; value: %s; footprint: %s; first_pad: %d; npads: %d)" % (pstr(ref), pstr(val), pstr(fp), first, len(pads)))
    first += len(pads)
L.append(",\n".join(rows))
L.append("  ];")
L += ["", "var FData: TBoardData;", "", "function BoardApple2p: TBoardData;", "var i: Integer;", "begin",
      "  if FData = nil then begin", "    FData := TBoardData.Create;",
      "    FData.outline[0] := %.3f; FData.outline[1] := %.3f; FData.outline[2] := %.3f; FData.outline[3] := %.3f;" % outline,
      "    FData.nnets := %d; FData.npads := %d; FData.ncomps := %d;" % (len(net_names), len(allpads), len(layout)),
      "    FData.nets.SetLength(FData.nnets);", "    for i := 0 to FData.nnets - 1 do FData.nets[i] := NETS[i];",
      "    FData.pads.SetLength(FData.npads);", "    for i := 0 to FData.npads - 1 do FData.pads[i] := PADS[i];",
      "    FData.comps.SetLength(FData.ncomps);", "    for i := 0 to FData.ncomps - 1 do FData.comps[i] := COMPS[i];",
      "  end;", "  Result := FData;", "end;", "", "end."]
open(os.path.join(out, "hwsim.bd_apple2p.pas"), "w", encoding="utf-8", newline="\n").write("\n".join(L) + "\n")

# a report for checking by hand
rep = []
for ref, val, fp, pads in layout:
    rep.append("%-7s %-24s %s" % (ref, val, " ".join("%s=%s" % (p[0], display(net_names[p[5]]) if p[5] >= 0 else "-") for p in pads)))
open(os.path.join(os.path.dirname(src), "apple2p.pins.txt"), "w", encoding="utf-8").write("\n".join(rep) + "\n")
multi = collections.Counter(pin_net[k] for k in pin_net)
print("apple2p: %d parts, %d pads, %d nets (%d of them unconnected pins)" %
      (len(layout), len(allpads), len(net_names), sum(1 for n in net_names if n.startswith("unconnected-"))))
