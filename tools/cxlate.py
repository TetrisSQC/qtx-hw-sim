"""cxlate.py - a small C -> Pascal (Quartex/DWScript) statement translator,
made for the code-generated instruction decoders of floooh's chips
(m6502.h, z80.h). It handles the subset of C used there:

  statements: expr; if (c) {..} else {..}  {..}  break; assert(..);
              case labels are handled by the caller
  expressions: full C precedence, ++/-- (pre/post), compound assignment,
              ternary, casts to (u)int8/16/32, calls.

Every value is an Integer (JS number). Fields and locals have a declared
bit width: an assignment to an N-bit lvalue is masked to N bits, like a C
store to uint8_t/uint16_t. Boolean context is tracked so that C's int/bool
mixing becomes valid Pascal (Ord(), <> 0).

Function-like and object-like macros are expanded first from a dict of
C definitions (taken from the header, or overridden by the caller).
"""
import re

# ---------------------------------------------------------------------------
# Macro expansion
# ---------------------------------------------------------------------------

def split_args(s):
    """Split a macro argument list at top-level commas."""
    args, depth, cur = [], 0, ""
    for ch in s:
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        if ch == "," and depth == 0:
            args.append(cur)
            cur = ""
        else:
            cur += ch
    args.append(cur)
    return [a.strip() for a in args]


def find_close(s, i):
    """s[i] == '(' -> index of the matching ')'."""
    depth = 0
    for j in range(i, len(s)):
        if s[j] == "(":
            depth += 1
        elif s[j] == ")":
            depth -= 1
            if depth == 0:
                return j
    raise ValueError("unbalanced: " + s[i:i + 80])


def expand_macros(text, fmacros, omacros, max_pass=20):
    """fmacros: name -> (params, body); omacros: name -> body."""
    ident = re.compile(r"[A-Za-z_]\w*")
    for _ in range(max_pass):
        out, i, changed = [], 0, False
        while i < len(text):
            m = ident.match(text, i)
            if not m or (i > 0 and (text[i - 1].isalnum() or text[i - 1] == "_")):
                out.append(text[i])
                i += 1
                continue
            name = m.group(0)
            j = m.end()
            if name in fmacros:
                k = j
                while k < len(text) and text[k] == " ":
                    k += 1
                if k < len(text) and text[k] == "(":
                    e = find_close(text, k)
                    args = split_args(text[k + 1:e]) if text[k + 1:e].strip() else []
                    params, body = fmacros[name]
                    if len(params) == 1 and params[0] == "" and not args:
                        args = [""]
                    if len(args) != len(params):
                        raise ValueError("macro %s args %r" % (name, args))
                    rep = body
                    for p, a in zip(params, args):
                        if p:
                            rep = re.sub(r"\b%s\b" % re.escape(p), lambda _m, a=a: "(" + a + ")", rep)
                    out.append(rep)
                    i = e + 1
                    changed = True
                    continue
            if name in omacros:
                out.append("(" + omacros[name] + ")")
                i = j
                changed = True
                continue
            out.append(name)
            i = j
        text = "".join(out)
        if not changed:
            return text
    raise ValueError("macro expansion does not terminate")


def parse_defines(header):
    """Collect '#define NAME(args) body' and '#define NAME body' from a header."""
    f, o = {}, {}
    for line in header.splitlines():
        m = re.match(r"\s*#define\s+(\w+)\(([^)]*)\)\s+(.*)$", line)
        if m:
            body = re.sub(r"//.*$|/\*.*?\*/", "", m.group(3)).strip()
            f[m.group(1)] = ([p.strip() for p in m.group(2).split(",")], body)
            continue
        m = re.match(r"\s*#define\s+(\w+)\s+(.*)$", line)
        if m:
            body = re.sub(r"//.*$|/\*.*?\*/", "", m.group(2)).strip()
            if body:
                o[m.group(1)] = body
    return f, o

# ---------------------------------------------------------------------------
# Tokenizer
# ---------------------------------------------------------------------------

TOK = re.compile(r"""
  (?P<ws>\s+)|
  (?P<num>0[xX][0-9a-fA-F]+[uUlL]*|\d+[uUlL]*)|
  (?P<id>[A-Za-z_]\w*)|
  (?P<op>->|\+\+|--|<<=|>>=|<<|>>|<=|>=|==|!=|&&|\|\||\+=|-=|\*=|/=|%=|&=|\|=|\^=|[-+*/%&|^~!<>=?:;,(){}\[\].])
""", re.X)


def tokenize(s):
    toks, i = [], 0
    while i < len(s):
        m = TOK.match(s, i)
        if not m:
            raise ValueError("bad char at: " + s[i:i + 40])
        i = m.end()
        if m.lastgroup == "ws":
            continue
        toks.append((m.lastgroup, m.group(0)))
    toks.append(("eof", ""))
    return toks

# ---------------------------------------------------------------------------
# Parser -> AST
# ---------------------------------------------------------------------------

BINPREC = [
    ["||"], ["&&"], ["|"], ["^"], ["&"], ["==", "!="], ["<", ">", "<=", ">="],
    ["<<", ">>"], ["+", "-"], ["*", "/", "%"],
]
ASSIGN_OPS = ["=", "+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "<<=", ">>="]
CAST_TYPES = {"uint8_t": 8, "uint16_t": 16, "uint32_t": 32, "int8_t": -8, "int16_t": -16,
              "int": 0, "unsigned": 32, "uint64_t": 64, "bool": 1, "int32_t": 0}


class Parser:
    def __init__(self, toks):
        self.t, self.i = toks, 0

    def peek(self, k=0):
        return self.t[self.i + k]

    def next(self):
        tok = self.t[self.i]
        self.i += 1
        return tok

    def expect(self, v):
        tok = self.next()
        if tok[1] != v:
            raise ValueError("expected %r got %r near %r" % (v, tok, self.t[self.i - 5:self.i + 5]))
        return tok

    # statements
    def stmts(self, end=None):
        out = []
        while True:
            tok = self.peek()
            if tok[0] == "eof" or tok[1] == end:
                return out
            st = self.stmt()
            if st is not None:
                out.append(st)

    def stmt(self):
        tok = self.peek()
        if tok[1] == ";":
            self.next()
            return None
        if tok[1] == "{":
            self.next()
            body = self.stmts("}")
            self.expect("}")
            return ("block", body)
        if tok[1] == "if":
            self.next()
            self.expect("(")
            cond = self.expr()
            self.expect(")")
            then = self.stmt()
            els = None
            if self.peek()[1] == "else":
                self.next()
                els = self.stmt()
            return ("if", cond, then, els)
        if tok[1] == "break":
            self.next()
            self.expect(";")
            return ("break",)
        if tok[1] == "goto":
            self.next()
            lab = self.next()[1]
            self.expect(";")
            return ("goto", lab)
        if tok[1] == "switch":
            self.next()
            self.expect("(")
            sel = self.expr()
            self.expect(")")
            self.expect("{")
            groups = []          # [(labels, stmts)]
            while self.peek()[1] != "}":
                labels = []
                while self.peek()[1] in ("case", "default"):
                    if self.next()[1] == "case":
                        labels.append(self.cond())
                    else:
                        labels.append("default")
                    self.expect(":")
                body = []
                while self.peek()[1] not in ("case", "default", "}"):
                    st = self.stmt()
                    if st is not None:
                        body.append(st)
                groups.append((labels, body))
            self.expect("}")
            return ("switch", sel, groups)
        if tok[1] == "const" and self.peek(1)[0] == "id" and self.peek(1)[1] in CAST_TYPES:
            self.next()
            return self.stmt()
        if tok[1] == "return":
            self.next()
            e = None if self.peek()[1] == ";" else self.expr()
            self.expect(";")
            return ("return", e)
        if tok[1] in ("assert", "CHIPS_ASSERT", "_Z80_UNREACHABLE"):
            self.next()
            if self.peek()[1] == "(":
                self.next()
                depth = 1
                while depth:
                    v = self.next()[1]
                    depth += (v == "(") - (v == ")")
            if self.peek()[1] == ";":
                self.next()
            return ("assert",)
        # declaration: type name = expr;
        if tok[0] == "id" and tok[1] in CAST_TYPES and self.peek(1)[0] == "id":
            self.next()
            decls = []
            while True:
                name = self.next()[1]
                init = None
                if self.peek()[1] == "=":
                    self.next()
                    init = self.assign()
                decls.append(("decl", CAST_TYPES[tok[1]], name, init))
                if self.peek()[1] == ",":
                    self.next()
                    continue
                break
            self.expect(";")
            return decls[0] if len(decls) == 1 else ("block", decls)
        e = self.expr()
        self.expect(";")
        return ("expr", e)

    # expressions
    def expr(self):
        e = self.assign()
        while self.peek()[1] == ",":
            self.next()
            e2 = self.assign()
            e = ("comma", e, e2)
        return e

    def assign(self):
        lhs = self.cond()
        if self.peek()[1] in ASSIGN_OPS:
            op = self.next()[1]
            rhs = self.assign()
            return ("assign", op, lhs, rhs)
        return lhs

    def cond(self):
        c = self.binary(0)
        if self.peek()[1] == "?":
            self.next()
            a = self.expr()
            self.expect(":")
            b = self.cond()
            return ("tern", c, a, b)
        return c

    def binary(self, lvl):
        if lvl == len(BINPREC):
            return self.unary()
        e = self.binary(lvl + 1)
        while self.peek()[1] in BINPREC[lvl]:
            op = self.next()[1]
            e2 = self.binary(lvl + 1)
            e = ("bin", op, e, e2)
        return e

    def unary(self):
        tok = self.peek()
        if tok[1] in ("++", "--"):
            self.next()
            return ("pre", tok[1], self.unary())
        if tok[1] in ("-", "~", "!", "+"):
            self.next()
            return ("un", tok[1], self.unary())
        if tok[1] == "(" and self.peek(1)[0] == "id" and self.peek(1)[1] in CAST_TYPES and self.peek(2)[1] == ")":
            self.next()
            ty = self.next()[1]
            self.next()
            return ("cast", CAST_TYPES[ty], self.unary())
        return self.postfix()

    def postfix(self):
        e = self.primary()
        while True:
            tok = self.peek()
            if tok[1] in ("++", "--"):
                self.next()
                e = ("post", tok[1], e)
            elif tok[1] == "(":
                self.next()
                args = []
                if self.peek()[1] != ")":
                    args.append(self.assign())
                    while self.peek()[1] == ",":
                        self.next()
                        args.append(self.assign())
                self.expect(")")
                e = ("call", e, args)
            elif tok[1] in ("->", "."):
                self.next()
                name = self.next()[1]
                e = ("field", e, name)
            elif tok[1] == "[":
                self.next()
                idx = self.expr()
                self.expect("]")
                e = ("index", e, idx)
            else:
                return e

    def primary(self):
        tok = self.next()
        if tok[0] == "num":
            v = tok[1].rstrip("uUlL")
            return ("num", int(v, 16) if v.lower().startswith("0x") else int(v))
        if tok[0] == "id":
            if tok[1] in ("true", "false"):
                return ("bool", tok[1] == "true")
            return ("id", tok[1])
        if tok[1] == "(":
            e = self.expr()
            self.expect(")")
            return ("paren", e)
        raise ValueError("unexpected %r near %r" % (tok, self.t[self.i - 6:self.i + 6]))

# ---------------------------------------------------------------------------
# Emitter
# ---------------------------------------------------------------------------

class Emitter:
    """widths: lvalue name (as emitted) -> bit width (0 = no mask, 32 = and $FFFFFFFF shr 0).
    rename: function(ast of field/id) -> pascal name or None.
    funcs: c function name -> (pascal name, returns_bool, drop_first_arg)."""

    def __init__(self, widths, rename, funcs, bool_ids=(), gotos=None, ret_width=0, ret_bool=False,
                 const_eval=None):
        self.widths = widths
        self.rename = rename
        self.funcs = funcs
        self.bool_ids = set(bool_ids)
        self.pre, self.post = [], []
        self.locals = {}
        self.gotos = gotos or {}
        self.ret_width = ret_width
        self.ret_bool = ret_bool
        self.const_eval = const_eval

    # helpers
    def name_of(self, e):
        while e[0] == "paren":
            e = e[1]
        try:
            n = self.rename(e, self)
        except TypeError:
            n = self.rename(e)
        if n is None:
            raise ValueError("cannot name lvalue %r" % (e,))
        return n

    def width(self, name):
        if name in self.locals:
            return self.locals[name]
        return self.widths.get(name, 0)

    @staticmethod
    def mask(code, w):
        if w in (0, 64, 1):
            return code
        if w == 32:
            return "((%s) shr 0)" % code
        if w > 0:
            return "((%s) and $%X)" % (code, (1 << w) - 1)
        w = -w
        return "((((%s) and $%X) xor $%X) - $%X)" % (code, (1 << w) - 1, 1 << (w - 1), 1 << (w - 1))

    def as_int(self, r):
        code, kind = r
        return code if kind == "int" else "Ord(%s)" % code

    def as_bool(self, r):
        code, kind = r
        return code if kind == "bool" else "((%s) <> 0)" % code

    # expressions -> (code, kind)
    def ex(self, e):
        t = e[0]
        if t == "num":
            v = e[1]
            if v > 0xFFFFFFFF:
                raise ValueError("64-bit constant %x" % v)
            return ("$%X" % v if v > 9 else str(v), "int")
        if t == "bool":
            return ("True" if e[1] else "False", "bool")
        if t == "paren":
            code, kind = self.ex(e[1])
            return ("(%s)" % code, kind)
        if t in ("id", "field"):
            n = self.name_of(e)
            return (n, "bool" if n in self.bool_ids else "int")
        if t == "index":
            return ("%s[%s]" % (self.name_of(e[1]), self.as_int(self.ex(e[2]))), "int")
        if t == "cast":
            r = self.ex(e[2])
            w = e[1]
            if w == 1:
                return (self.as_bool(r), "bool")
            if w in (0, 64):
                return (self.as_int(r), "int")
            return (self.mask(self.as_int(r), w), "int")
        if t == "un":
            op = e[1]
            r = self.ex(e[2])
            if op == "!":
                return ("(not %s)" % self.as_bool(r), "bool")
            if op == "~":
                return ("(not %s)" % self.as_int(r), "int")
            if op == "-":
                return ("(-%s)" % self.as_int(r), "int")
            return (self.as_int(r), "int")
        if t == "bin":
            op, a, b = e[1], self.ex(e[2]), self.ex(e[3])
            if op in ("&&", "||"):
                return ("(%s %s %s)" % (self.as_bool(a), "and" if op == "&&" else "or", self.as_bool(b)), "bool")
            if op in ("==", "!=", "<", ">", "<=", ">="):
                pop = {"==": "=", "!=": "<>"}.get(op, op)
                if a[1] == "bool" and b[1] == "bool":
                    return ("(%s %s %s)" % (a[0], pop, b[0]), "bool")
                return ("(%s %s %s)" % (self.as_int(a), pop, self.as_int(b)), "bool")
            pop = {"|": "or", "&": "and", "^": "xor", "<<": "shl", ">>": "shr", "/": "div", "%": "mod"}.get(op, op)
            return ("(%s %s %s)" % (self.as_int(a), pop, self.as_int(b)), "int")
        if t == "tern":
            c, a, b = self.as_bool(self.ex(e[1])), self.ex(e[2]), self.ex(e[3])
            if a[1] == "bool" and b[1] == "bool":
                return ("IfB(%s, %s, %s)" % (c, a[0], b[0]), "bool")
            return ("IfI(%s, %s, %s)" % (c, self.as_int(a), self.as_int(b)), "int")
        if t == "post" or t == "pre":
            n = self.name_of(e[2])
            w = self.width(n)
            upd = "%s := %s;" % (n, self.mask("%s %s 1" % (n, "+" if e[1] == "++" else "-"), w))
            (self.post if t == "post" else self.pre).append(upd)
            return (n, "int")
        if t == "assign":
            # assignment used as a value: hoist it before
            st = self.assign_stmt(e)
            self.pre.extend(st)
            n = self.name_of(e[2])
            return (n, "bool" if n in self.bool_ids else "int")
        if t == "call":
            fname = e[1][1] if e[1][0] == "id" else None
            if fname not in self.funcs:
                raise ValueError("unknown function %r" % (e[1],))
            pname, rbool, drop = self.funcs[fname]
            args = e[2][drop:]
            code = "%s(%s)" % (pname, ", ".join(self.as_int(self.ex(a)) for a in args))
            return (code, "bool" if rbool else "int")
        if t == "comma":
            self.pre.extend(self.stmt_list([("expr", e[1])]))
            return self.ex(e[2])
        raise ValueError("expr %r" % (e,))

    def assign_stmt(self, e):
        op, lhs, rhs = e[1], e[2], e[3]
        n = self.name_of(lhs)
        w = self.width(n)
        r = self.ex(rhs)
        if op == "=":
            if n in self.bool_ids:
                return ["%s := %s;" % (n, self.as_bool(r))]
            val = self.as_int(r)
        else:
            bop = op[:-1]
            pop = {"|": "or", "&": "and", "^": "xor", "<<": "shl", ">>": "shr", "/": "div", "%": "mod"}.get(bop, bop)
            val = "%s %s %s" % (n, pop, "(" + self.as_int(r) + ")")
        if w and op in ("&=", ">>=") and op != "=":
            return ["%s := %s;" % (n, val)]       # can't grow
        return ["%s := %s;" % (n, self.mask(val, w))]

    def stmt_list(self, stmts, indent=""):
        """Statements in sequence. After a statement that never falls through
        the rest is unreachable; an 'if' without else whose branch never falls
        through gets the rest of the sequence as its else branch (so gotos
        need no break: Pascal case statements stay JS switches)."""
        out = []
        stmts = list(stmts)
        i = 0
        while i < len(stmts):
            s = stmts[i]
            rest = stmts[i + 1:]
            if s[0] == "if" and s[3] is None and rest and terminates(s[2]):
                s = ("if", s[1], s[2], ("block", rest))
                out.extend(self.stmt(s, indent))
                break
            out.extend(self.stmt(s, indent))
            if terminates(s) and s[0] != "break":
                break
            i += 1
        return out

    def stmt(self, s, indent=""):
        t = s[0]
        if t == "expr":
            e = s[1]
            while e[0] == "paren":
                e = e[1]
            self.pre, self.post = [], []
            if e[0] == "assign":
                body = self.assign_stmt(e)
            elif e[0] in ("post", "pre"):
                n = self.name_of(e[2])
                w = self.width(n)
                body = ["%s := %s;" % (n, self.mask("%s %s 1" % (n, "+" if e[1] == "++" else "-"), w))]
            elif e[0] == "call":
                body = [self.ex(e)[0] + ";"]
            elif e[0] == "comma":
                body = self.stmt_list([("expr", e[1]), ("expr", e[2])], "")
            else:
                body = ["{ %s }" % self.ex(e)[0]]
            pre, post = self.pre, self.post
            self.pre, self.post = [], []
            return [indent + x for x in pre + body + post]
        if t == "block":
            return self.stmt_list(s[1], indent)
        if t == "if":
            self.pre, self.post = [], []
            c = self.as_bool(self.ex(s[1]))
            pre, post = self.pre, self.post
            self.pre, self.post = [], []
            if post:
                raise ValueError("post-increment in condition")
            out = [indent + x for x in pre]
            out.append(indent + "if %s then begin" % c)
            out.extend(self.stmt(s[2], indent + "  "))
            if s[3] is not None:
                out.append(indent + "end else begin")
                out.extend(self.stmt(s[3], indent + "  "))
            out.append(indent + "end;")
            return out
        if t == "break" or t == "assert":
            return []
        if t == "goto":
            return [indent + x for x in self.gotos[s[1]]]
        if t == "switch":
            self.pre, self.post = [], []
            sel = self.as_int(self.ex(s[1]))
            pre = self.pre
            self.pre = []
            out = [indent + x for x in pre]
            out.append(indent + "case %s of" % sel)
            dflt = None
            for labels, body in s[2]:
                body = list(body)
                term = bool(body) and terminates(body[-1])
                if body and body[-1][0] == "break":
                    body = body[:-1]
                if not term and (labels, body) != s[2][-1] and body:
                    raise ValueError("switch fall-through not supported: %r" % (labels,))
                if any(isinstance(l, tuple) and False for l in labels):
                    pass
                lab_codes = [self.case_label(l) for l in labels if l != "default"]
                code = self.stmt_list(body, indent + "    ")
                if "default" in labels:
                    dflt = code
                    if not lab_codes:
                        continue
                out.append(indent + "  %s: begin" % ", ".join(lab_codes))
                out.extend(code)
                out.append(indent + "  end;")
            if dflt:
                out.append(indent + "else begin")
                out.extend(dflt)
                out.append(indent + "end;")
            out.append(indent + "end;")
            return out
        if t == "return":
            if s[1] is None:
                return [indent + "exit;"]
            self.pre, self.post = [], []
            rr = self.ex(s[1])
            if self.ret_bool:
                r = self.as_bool(rr)
            else:
                r = self.mask(self.as_int(rr), self.ret_width) if self.ret_width not in (0, 64) else self.as_int(rr)
            pre, post = self.pre, self.post
            self.pre, self.post = [], []
            if post:
                return ([indent + x for x in pre] + [indent + "Result := %s;" % r] +
                        [indent + x for x in post] + [indent + "exit;"])
            return [indent + x for x in pre] + [indent + "exit(%s);" % r]
        if t == "decl":
            w, name, init = s[1], s[2], s[3]
            self.locals[name] = w if w != 0 else 0
            if init is None:
                return []
            return self.stmt(("expr", ("assign", "=", ("id", name), init)), indent)
        raise ValueError("stmt %r" % (s,))

    def case_label(self, e):
        v = eval_const(e)
        return str(v)


def translate(code, emitter):
    toks = tokenize(code)
    p = Parser(toks)
    stmts = p.stmts()
    return emitter.stmt_list(stmts)


def eval_const(e):
    """Evaluate a constant expression AST (after macro expansion)."""
    t = e[0]
    if t == "num":
        return e[1]
    if t == "paren":
        return eval_const(e[1])
    if t == "un":
        v = eval_const(e[2])
        return {"-": -v, "~": ~v, "!": int(not v), "+": v}[e[1]]
    if t == "bin":
        a, b = eval_const(e[2]), eval_const(e[3])
        return {"+": a + b, "-": a - b, "*": a * b, "/": a // b, "%": a % b, "<<": a << b, ">>": a >> b,
                "&": a & b, "|": a | b, "^": a ^ b}[e[1]]
    raise ValueError("not a constant: %r" % (e,))


def terminates(st):
    """Does a statement never fall through (break, return, goto on every path)?"""
    t = st[0]
    if t in ("break", "return", "goto", "assert"):
        return True
    if t == "block":
        return bool(st[1]) and terminates(st[1][-1])
    if t == "if":
        return st[3] is not None and terminates(st[2]) and terminates(st[3])
    return False
