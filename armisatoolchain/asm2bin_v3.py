#!/usr/bin/env python3
"""
asm2bin_v2.py — ARM32 Assembler (GCC Output Compatible)
EE 533 Lab 6 - Team 10

Handles real arm-none-eabi-gcc -O1 output including:
  - Conditional execution on ANY instruction (strgt, suble, subne, bxle ...)
  - Pre/post-indexed addressing with writeback:  [Rn,#off]!  [Rn],#off
  - PC-relative literal pool loads:  ldr r3, .L14
  - Register-shifted-register:       add r0, r0, r3, lsl #2
  - push/pop {reglist}
  - bl (branch with link), bx (branch exchange)
  - subs, adds  (S flag variants)
  - Separate .text and .data sections → I-Mem + D-Mem output files

Usage:
  python3 asm2bin_v2.py sort_gcc.s
  python3 asm2bin_v2.py sort_gcc.s --dmem-base 0 --format coe
  python3 asm2bin_v2.py sort_gcc.s --dump
"""

import sys, re, struct, argparse
from collections import OrderedDict

# ─────────────────────────── Tables ─────────────────────────────────────────

COND = {
    'EQ':0x0,'NE':0x1,'CS':0x2,'HS':0x2,'CC':0x3,'LO':0x3,
    'MI':0x4,'PL':0x5,'VS':0x6,'VC':0x7,'HI':0x8,'LS':0x9,
    'GE':0xA,'LT':0xB,'GT':0xC,'LE':0xD,'AL':0xE,'':0xE,
}

DP = {
    'AND':0x0,'EOR':0x1,'SUB':0x2,'RSB':0x3,
    'ADD':0x4,'ADC':0x5,'SBC':0x6,'RSC':0x7,
    'TST':0x8,'TEQ':0x9,'CMP':0xA,'CMN':0xB,
    'ORR':0xC,'MOV':0xD,'BIC':0xE,'MVN':0xF,
}
NO_RD = {'CMP','CMN','TST','TEQ'}
NO_RN = {'MOV','MVN'}

REGS = {
    **{f'R{i}':i for i in range(16)},
    'SP':13,'LR':14,'PC':15,'FP':11,'IP':12,
}

SHIFT_TYPE = {'LSL':0,'LSR':1,'ASR':2,'ROR':3}

# ─────────────────────────── Helpers ─────────────────────────────────────────

def parse_reg(s):
    k = s.strip().upper().rstrip(',').rstrip(']').rstrip('!')
    if k not in REGS:
        raise ValueError(f"Unknown register '{s}'")
    return REGS[k]

def parse_imm(s):
    s = s.strip().lstrip('#').rstrip(',').rstrip(']').rstrip('!')
    try:
        return int(s, 0)
    except ValueError:
        raise ValueError(f"Bad immediate '{s}'")

def encode_rot_imm(val):
    """Encode 32-bit constant as ARM 12-bit rotated immediate."""
    val &= 0xFFFFFFFF
    for rot in range(0, 32, 2):
        v = ((val << rot) | (val >> (32 - rot))) & 0xFFFFFFFF
        if v <= 0xFF:
            return ((rot >> 1) << 8) | v
    raise ValueError(f"Cannot encode {val:#010x} as ARM rotated immediate")

def reg_list(s):
    """Parse '{r4, lr}' → 16-bit bitmask."""
    s = s.strip().lstrip('{').rstrip('}')
    mask = 0
    for tok in re.split(r'[\s,]+', s):
        tok = tok.strip()
        if not tok: continue
        if '-' in tok:
            a, b = tok.split('-')
            for i in range(REGS[a.upper()], REGS[b.upper()]+1):
                mask |= (1 << i)
        else:
            mask |= (1 << REGS[tok.upper()])
    return mask

def smart_split(line):
    """Split on whitespace/commas but keep bracket contents intact."""
    tokens, cur, depth = [], '', 0
    for ch in line:
        if ch == '[': depth += 1; cur += ch
        elif ch == ']': depth -= 1; cur += ch
        elif ch == '{': depth += 1; cur += ch
        elif ch == '}': depth -= 1; cur += ch
        elif ch in (' ','\t',',') and depth == 0:
            if cur: tokens.append(cur); cur = ''
        else:
            cur += ch
    if cur: tokens.append(cur)
    return tokens

def norm_label(s):
    """Strip leading dots and trailing colons from label names."""
    return s.lstrip('.').rstrip(':').strip()

def signed24(v):
    return v & 0xFFFFFF

# ─────────────────────────── Encoders ────────────────────────────────────────

def enc_dp_imm(cond, op, s, rn, rd, imm12):
    return (cond<<28)|(1<<25)|(op<<21)|(s<<20)|(rn<<16)|(rd<<12)|(imm12&0xFFF)

def enc_dp_reg(cond, op, s, rn, rd, sh_imm, sh_type, rm):
    return (cond<<28)|(op<<21)|(s<<20)|(rn<<16)|(rd<<12)|(sh_imm<<7)|(sh_type<<5)|rm

def enc_dp_reg_rs(cond, op, s, rn, rd, rs, sh_type, rm):
    return (cond<<28)|(op<<21)|(s<<20)|(rn<<16)|(rd<<12)|(rs<<8)|(sh_type<<5)|(1<<4)|rm

def enc_mem(cond, load, rn, rd, offset, pre=1, byte=0, writeback=0):
    up = 1 if offset >= 0 else 0
    abs_off = abs(offset)
    if abs_off > 4095:
        raise ValueError(f"Offset {offset} out of range")
    return (cond<<28)|(0b01<<26)|(pre<<24)|(up<<23)|(byte<<22)|(writeback<<21)|(load<<20)|(rn<<16)|(rd<<12)|abs_off

def enc_mem_reg(cond, load, rn, rd, rm, shift_imm=0, shift_type=0, pre=1, up=1, byte=0, writeback=0):
    """LDR/STR with register offset: [Rn, Rm, LSL #n]"""
    return (cond<<28)|(0b01<<26)|(1<<25)|(pre<<24)|(up<<23)|(byte<<22)|(writeback<<21)|(load<<20)|(rn<<16)|(rd<<12)|(shift_imm<<7)|(shift_type<<5)|rm

def enc_branch(cond, off_words, link=0):
    return (cond<<28)|(0b101<<25)|(link<<24)|signed24(off_words)

def enc_bx(cond, rm):
    return (cond<<28)|0x012FFF10|rm

def enc_ldm_stm(cond, load, rn, regmask, pre, up, writeback):
    return (cond<<28)|(0b100<<25)|(pre<<24)|(up<<23)|(writeback<<21)|(load<<20)|(rn<<16)|(regmask&0xFFFF)

# ─────────────────────────── Mnemonic Parser ─────────────────────────────────

# Branch mnemonics
BRANCH_MAP = {
    'B':('B','AL'),'BL':('BL','AL'),'BX':('BX','AL'),
    'BEQ':('B','EQ'),'BNE':('B','NE'),'BLT':('B','LT'),
    'BLE':('B','LE'),'BGT':('B','GT'),'BGE':('B','GE'),
    'BLEQ':('BL','EQ'),'BLNE':('BL','NE'),
    'BXLE':('BX','LE'),'BXEQ':('BX','EQ'),'BXNE':('BX','NE'),
    'BXGE':('BX','GE'),'BXLT':('BX','LT'),
}

def split_mnem(raw):
    """
    'STRGT' → ('STR','GT',False)
    'SUBS'  → ('SUB','AL',True)
    'SUBNE' → ('SUB','NE',False)
    'ADDEQS'→ ('ADD','EQ',True)
    Returns (base, cond_str, s_flag)
    """
    raw = raw.upper()
    if raw in BRANCH_MAP:
        base, cond = BRANCH_MAP[raw]
        return base, cond, False
    if raw in ('PUSH','POP','NOP'):
        return raw, 'AL', False

    # Try S suffix
    s_flag = False
    work = raw
    if work.endswith('S') and work[:-1] in DP:
        return work[:-1], 'AL', True

    # Try cond suffix (2 chars), then S before it
    for cond_str in sorted(COND.keys(), key=len, reverse=True):
        if not cond_str: continue
        if work.endswith(cond_str):
            base = work[:-len(cond_str)]
            if base in DP or base in ('LDR','STR','LDRB','STRB'):
                return base, cond_str, False
            # Maybe there's an S before the cond
            if base.endswith('S') and base[:-1] in DP:
                return base[:-1], cond_str, True

    # No cond
    if work in DP or work in ('LDR','STR','LDRB','STRB'):
        return work, 'AL', False

    return raw, 'AL', False


# ─────────────────────────── Memory Operand Parser ───────────────────────────

def parse_mem_op(full_args):
    """
    Parse memory operands from token list or string.
    Handles:
      '[Rn]'              → (rn, 0,   pre=1, wb=0, rm=None, sh=0, st=0)
      '[Rn, #off]'        → (rn, off, pre=1, wb=0, rm=None, sh=0, st=0)
      '[Rn, #off]!'       → (rn, off, pre=1, wb=1, rm=None, sh=0, st=0)
      '[Rn], #off'        → (rn, off, pre=0, wb=0, rm=None, sh=0, st=0)
      '[Rn, Rm, LSL #n]'  → (rn, 0,   pre=1, wb=0, rm=Rm,  sh=n, st=0)
    Returns (rn, offset, pre, writeback, rm, shift_imm, shift_type)
    For backward compat, callers that expect 4 values still work via [:4]
    """
    if isinstance(full_args, list):
        s = ', '.join(full_args)
    else:
        s = full_args
    s = s.strip()

    # Post-indexed: [Rn], #off
    post = re.match(r'\[(\w+)\]\s*[,\s]\s*#(-?\d+)', s)
    if post:
        rn  = REGS[post.group(1).upper()]
        off = int(post.group(2))
        return rn, off, 0, 0, None, 0, 0

    wb = 1 if s.endswith('!') else 0
    s  = s.rstrip('!')

    # Register-shifted: [Rn, Rm, LSL #n]
    reg_sh = re.match(r'\[(\w+)\s*,\s*(\w+)\s*,\s*(LSL|LSR|ASR|ROR)\s*#(\d+)\]', s, re.I)
    if reg_sh:
        rn = REGS[reg_sh.group(1).upper()]
        rm = REGS[reg_sh.group(2).upper()]
        st = SHIFT_TYPE[reg_sh.group(3).upper()]
        sh = int(reg_sh.group(4))
        return rn, 0, 1, wb, rm, sh, st

    # Register offset: [Rn, Rm]
    reg_off = re.match(r'\[(\w+)\s*,\s*(\w+)\]', s)
    if reg_off:
        rn = REGS[reg_off.group(1).upper()]
        rm = REGS[reg_off.group(2).upper()]
        return rn, 0, 1, wb, rm, 0, 0

    # Pre-indexed: [Rn] or [Rn, #off]
    pre = re.match(r'\[(\w+)(?:\s*,\s*#(-?\d+))?\]', s)
    if pre:
        rn  = REGS[pre.group(1).upper()]
        off = int(pre.group(2)) if pre.group(2) else 0
        return rn, off, 1, wb, None, 0, 0

    raise ValueError(f"Bad memory operand: '{s}'")


# ─────────────────────────── Assembler ───────────────────────────────────────

class Assembler:
    def __init__(self, dmem_base=0):
        self.dmem_base = dmem_base   # byte address where D-Mem starts
        self.labels = {}             # name → byte address (in respective section)
        self.text_words = []         # (byte_addr, word, src)
        self.data_words = []         # (byte_addr, word, src)   (D-Mem content)
        self.errors = []
        self._section = 'text'
        self._text_pc = 0
        self._data_pc = 0

    # ── Pass 1: Collect labels ────────────────────────────────────────────

    def _clean(self, line):
        return re.split(r'[@;]', line)[0].strip()

    def first_pass(self, lines):
        sect = 'text'
        tpc, dpc = 0, 0
        for raw in lines:
            line = self._clean(raw)
            if not line: continue
            # Local labels like .L3: start with dot — handle before directive check
            import re as _re
            if line.endswith(':') and _re.match(r'[A-Za-z_.]', line):
                lbl = norm_label(line)
                if sect == 'text':
                    self.labels[lbl] = tpc
                else:
                    self.labels[lbl] = self.dmem_base + dpc
                continue
            if line.startswith('.'):
                d = line.lower()
                if d.startswith('.text'):   sect = 'text'
                elif d.startswith('.data'): sect = 'data'
                elif d.startswith('.set'):
                    m = _re.match(r'\.set\s+(\S+)\s*,\s*\.\s*\+\s*(\d+)', line)
                    if m:
                        lbl = norm_label(m.group(1))
                        self.labels[lbl] = self.dmem_base + dpc + int(m.group(2))
                elif d.startswith('.word') and sect == 'data':
                    dpc += 4
                continue
            if line.endswith(':'):
                lbl = norm_label(line)
                if sect == 'text':
                    self.labels[lbl] = tpc
                else:
                    self.labels[lbl] = self.dmem_base + dpc
                continue
            # Instruction or .word in text section
            if sect == 'text':
                tpc += 4
            elif sect == 'data':
                dpc += 4

    # ── Pass 2: Emit words ────────────────────────────────────────────────

    def second_pass(self, lines):
        sect = 'text'
        tpc, dpc = 0, 0
        for lineno, raw in enumerate(lines, 1):
            line = self._clean(raw)
            if not line: continue
            import re as _re
            if line.endswith(':') and _re.match(r'[A-Za-z_.]', line):
                continue  # labels already resolved in first pass
            if line.startswith('.'):
                d = line.lower()
                if d.startswith('.text'):   sect = 'text'
                elif d.startswith('.data'): sect = 'data'
                elif d.startswith('.word'):
                    # Emit word (literal pool or data)
                    val_str = line.split(None, 1)[1].strip()
                    val = self._resolve_word(val_str, tpc)
                    if sect == 'text':
                        self.text_words.append((tpc, val & 0xFFFFFFFF, raw.rstrip()))
                        tpc += 4
                    else:
                        self.data_words.append((dpc, val & 0xFFFFFFFF, raw.rstrip()))
                        dpc += 4
                continue
            if line.endswith(':'):
                continue
            if sect != 'text':
                continue
            try:
                word = self.asm_one(line, tpc)
                self.text_words.append((tpc, word, raw.rstrip()))
                tpc += 4
            except Exception as e:
                self.errors.append(f"Line {lineno}: {e}\n  >>> {raw.rstrip()}")

    def _resolve_word(self, s, pc):
        """Resolve a .word value (label, integer, or label expression)."""
        s = s.strip()
        # Label with possible offset: .LANCHOR0  or  .LC0 + 4
        m = re.match(r'\.?(\w+)\s*([+-]\s*\d+)?', s)
        if m:
            lbl = m.group(1)
            off = int(m.group(2).replace(' ','')) if m.group(2) else 0
            if lbl in self.labels:
                return self.labels[lbl] + off
            # Maybe it's just a number
            try:
                return int(s, 0)
            except ValueError:
                pass
        try:
            return int(s, 0)
        except ValueError:
            raise ValueError(f"Cannot resolve .word value: '{s}'")

    def assemble(self, source):
        lines = source.splitlines()
        self.first_pass(lines)
        self.second_pass(lines)
        if self.errors:
            for e in self.errors:
                print(f"[ERROR] {e}", file=sys.stderr)
            sys.exit(1)

    # ── Single instruction assembler ──────────────────────────────────────

    def asm_one(self, line, pc):
        toks = smart_split(line.strip())
        toks = [t for t in toks if t]
        base, cond_str, s_flag = split_mnem(toks[0])
        c    = COND[cond_str]
        s    = 1 if s_flag else 0
        args = toks[1:]

        # ── NOP ──────────────────────────────────────────────────────────
        if base == 'NOP':
            return enc_dp_reg(0xE, DP['MOV'], 0, 0, 0, 0, 0, 0)

        # ── PUSH / POP ────────────────────────────────────────────────────
        if base == 'PUSH':
            mask = reg_list(' '.join(args))
            # STMDB SP!, {reglist}  P=1 U=0 W=1 L=0
            return enc_ldm_stm(c, 0, 13, mask, pre=1, up=0, writeback=1)
        if base == 'POP':
            mask = reg_list(' '.join(args))
            # LDMIA SP!, {reglist}  P=0 U=1 W=1 L=1
            return enc_ldm_stm(c, 1, 13, mask, pre=0, up=1, writeback=1)

        # ── BX ────────────────────────────────────────────────────────────
        if base == 'BX':
            return enc_bx(c, parse_reg(args[0]))

        # ── B / BL ────────────────────────────────────────────────────────
        if base in ('B','BL'):
            link = 1 if base == 'BL' else 0
            lbl = norm_label(args[0])
            if lbl not in self.labels:
                raise ValueError(f"Undefined label '{lbl}'")
            off = (self.labels[lbl] - (pc + 8)) // 4
            return enc_branch(c, off, link)

        # ── LDR Rd, .label  (PC-relative literal pool load) ──────────────
        # 支持 .L4, .L4+4, .L4+8 等带偏移的 literal pool 引用
        if base == 'LDR' and len(args) == 2 and args[1].startswith('.'):
            rd  = parse_reg(args[0])
            lbl_expr = args[1]
            # 解析可能的偏移 .L4+4 → lbl='L4', extra_off=4
            m = re.match(r'\.?(\w+)\s*([+-]\s*\d+)?$', lbl_expr)
            if not m:
                raise ValueError(f"Bad label expression '{lbl_expr}'")
            lbl_name = m.group(1)
            extra_off = int(m.group(2).replace(' ','')) if m.group(2) else 0
            if lbl_name not in self.labels:
                raise ValueError(f"Undefined label '{lbl_expr}'")
            target = self.labels[lbl_name] + extra_off
            off    = target - (pc + 8)
            # Encode as LDR Rd, [PC, #off]
            return enc_mem(c, 1, 15, rd, off, pre=1, writeback=0)

        # ── LDR / STR / LDRB / STRB ──────────────────────────────────────
        if base in ('LDR','STR','LDRB','STRB'):
            load = 1 if base in ('LDR','LDRB') else 0
            byte = 1 if base in ('LDRB','STRB') else 0
            rd   = parse_reg(args[0])
            result = parse_mem_op(args[1:])
            rn, off, pre, wb = result[0], result[1], result[2], result[3]
            rm = result[4] if len(result) > 4 else None
            sh_imm = result[5] if len(result) > 5 else 0
            sh_type = result[6] if len(result) > 6 else 0
            if rm is not None:
                # Register-offset or register-shifted encoding
                up = 1  # assume positive (GCC always uses positive)
                return enc_mem_reg(c, load, rn, rd, rm, sh_imm, sh_type, pre=pre, up=up, byte=byte, writeback=wb)
            return enc_mem(c, load, rn, rd, off, pre=pre, byte=byte, writeback=wb)

        # ── MOV / MVN ─────────────────────────────────────────────────────
        if base in ('MOV','MVN'):
            rd  = parse_reg(args[0])
            src = args[1]
            if src.startswith('#'):
                imm = parse_imm(src)
                if imm < 0:
                    # Use MVN with bitwise NOT
                    imm12 = encode_rot_imm((~imm) & 0xFFFFFFFF)
                    return enc_dp_imm(c, DP['MVN'], s, 0, rd, imm12)
                return enc_dp_imm(c, DP[base], s, 0, rd, encode_rot_imm(imm))
            else:
                rm, sh_imm, sh_type = self._parse_shifted_reg(args[1:])
                return enc_dp_reg(c, DP[base], s, 0, rd, sh_imm, sh_type, rm)

        # ── CMP / CMN / TST / TEQ ─────────────────────────────────────────
        if base in NO_RD:
            rn = parse_reg(args[0])
            return self._dp_op2(c, DP[base], 1, rn, 0, args[1:])

        # ── ADD / SUB / AND / ORR / EOR / RSB / ADC / SBC / RSC ──────────
        if base in DP and base not in NO_RD and base not in NO_RN:
            rd = parse_reg(args[0])
            rn = parse_reg(args[1])
            return self._dp_op2(c, DP[base], s, rn, rd, args[2:])

        # ── LSL / LSR / ASR / ROR (as shift instructions) ─────────────────
        if base in SHIFT_TYPE:
            rd  = parse_reg(args[0])
            rm  = parse_reg(args[1])
            st  = SHIFT_TYPE[base]
            src = args[2]
            if src.startswith('#'):
                sh_imm = parse_imm(src)
                return enc_dp_reg(c, DP['MOV'], s, 0, rd, sh_imm, st, rm)
            else:
                rs = parse_reg(src)
                return enc_dp_reg_rs(c, DP['MOV'], s, 0, rd, rs, st, rm)

        raise ValueError(f"Unsupported: '{toks[0]}' (base='{base}' cond='{cond_str}')")

    def _parse_shifted_reg(self, args):
        """
        Parse: Rm  or  Rm, LSL #n  or  Rm, LSL Rn
        Returns (rm, shift_imm, shift_type)  or raises.
        """
        rm = parse_reg(args[0])
        if len(args) < 2:
            return rm, 0, 0
        sh_name = args[1].upper()
        if sh_name not in SHIFT_TYPE:
            return rm, 0, 0
        st = SHIFT_TYPE[sh_name]
        if len(args) > 2:
            src = args[2]
            if src.startswith('#'):
                return rm, parse_imm(src), st
            else:
                # Register shift — for now emit as immediate 0 (simplification)
                return rm, 0, st
        return rm, 0, st

    def _dp_op2(self, cond, op, s, rn, rd, rest):
        """
        Encode data processing with flexible Op2.
        rest is list of remaining tokens after Rn (or after Rd,Rn).
        """
        if not rest:
            raise ValueError("Missing Op2")
        src = rest[0]
        if src.startswith('#'):
            imm = parse_imm(src)
            # Handle negative immediate: ADD ↔ SUB flip
            if imm < 0:
                flip_map = {DP['ADD']:DP['SUB'], DP['SUB']:DP['ADD'],
                            DP['CMN']:DP['CMP'], DP['CMP']:DP['CMN']}
                op_flip = flip_map.get(op, op)
                return enc_dp_imm(cond, op_flip, s, rn, rd, encode_rot_imm((-imm)&0xFFFFFFFF))
            return enc_dp_imm(cond, op, s, rn, rd, encode_rot_imm(imm))
        else:
            rm, sh_imm, sh_type = self._parse_shifted_reg(rest)
            return enc_dp_reg(cond, op, s, rn, rd, sh_imm, sh_type, rm)


# ─────────────────────────── Output Formatters ───────────────────────────────

def to_coe(words_list, depth):
    data = {a//4: w for a, w, _ in words_list}
    vals = [f"{data.get(i,0):08X}" for i in range(depth)]
    return "memory_initialization_radix=16;\nmemory_initialization_vector=\n" + \
           ',\n'.join(vals) + ';'

def to_hex(words_list):
    return '\n'.join(f'{w:08X}' for _, w, _ in words_list)

def to_mif(words_list, depth, width=32):
    body = [f"DEPTH={depth};", f"WIDTH={width};",
            "ADDRESS_RADIX=HEX;", "DATA_RADIX=HEX;", "CONTENT BEGIN"]
    used = {a//4 for a,_,_ in words_list}
    d    = {a//4:w for a,w,_ in words_list}
    for i in range(depth):
        body.append(f"\t{i:04X} : {d.get(i,0):08X};")
    body.append("END;")
    return '\n'.join(body)

def disasm(word):
    """Mini disassembler for verification."""
    CN = ['EQ','NE','CS','CC','MI','PL','VS','VC',
          'HI','LS','GE','LT','GT','LE','',  '']
    DN = ['AND','EOR','SUB','RSB','ADD','ADC','SBC','RSC',
          'TST','TEQ','CMP','CMN','ORR','MOV','BIC','MVN']
    RN = [f'R{i}' for i in range(13)]+['SP','LR','PC']

    cond= (word>>28)&0xF; cs=CN[cond]
    b27 = (word>>27)&0x1F

    if (word>>25)&0x7 == 0b101:          # Branch
        L=(word>>24)&1; off=word&0xFFFFFF
        if off&0x800000: off-=0x1000000
        return f"B{'L' if L else ''}{cs} #{off*4+8:+d}"

    if (word>>24)&0xFF == 0x12 and (word>>4)&0xF == 1:  # BX
        return f"BX{cs} {RN[word&0xF]}"

    if (word>>25)&0x7 == 0b100:          # Block transfer
        L=(word>>20)&1; P=(word>>24)&1; U=(word>>23)&1
        W=(word>>21)&1; rn=(word>>16)&0xF; rl=word&0xFFFF
        regs=','.join(RN[i] for i in range(16) if rl&(1<<i))
        op='LDM' if L else 'STM'
        mode=('IB','IA','DB','DA')[(P<<1)|U] if not L else ('IB','IA','DB','DA')[(P<<1)|U]
        return f"{op}{mode}{cs} {RN[rn]}{'!' if W else ''},{{{regs}}}"

    if (word>>26)&0x3 == 0b01:           # LDR/STR
        L=(word>>20)&1; B=(word>>22)&1
        rn=(word>>16)&0xF; rd=(word>>12)&0xF
        off=word&0xFFF; U=(word>>23)&1; P=(word>>24)&1; W=(word>>21)&1
        sign='+' if U else '-'
        wb='!' if W else ''
        return f"{'LDR' if L else 'STR'}{'B' if B else ''}{cs} {RN[rd]},[{RN[rn]},#{sign}{off}]{wb}"

    if (word>>26)&0x3 == 0b00:           # DP
        I=(word>>25)&1; op=(word>>21)&0xF; S=(word>>20)&1
        rn=(word>>16)&0xF; rd=(word>>12)&0xF
        ss='S' if S and DN[op] not in ('CMP','CMN','TST','TEQ') else ''
        op2=word&0xFFF
        if I:
            rot=(op2>>8)&0xF; v=op2&0xFF
            v=((v>>(rot*2))|(v<<(32-rot*2)))&0xFFFFFFFF
            o2=f'#{v}'
        else:
            rm=op2&0xF; sha=(op2>>7)&0x1F; sht=(op2>>5)&0x3
            sn=['LSL','LSR','ASR','ROR'][sht]
            o2=RN[rm] if sha==0 else f'{RN[rm]},{sn}#{sha}'
        if DN[op]=='MOV': return f"MOV{cs}{ss} {RN[rd]},{o2}"
        if DN[op] in ('CMP','CMN','TST','TEQ'): return f"{DN[op]}{cs} {RN[rn]},{o2}"
        return f"{DN[op]}{cs}{ss} {RN[rd]},{RN[rn]},{o2}"

    return f"???  {word:08X}"


# ─────────────────────────── Main ────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='ARM32 Assembler v2 – gcc output compatible')
    ap.add_argument('input')
    ap.add_argument('--format','-f', default='hex',
                    choices=['hex','coe','mif'],
                    help='Output format for I-Mem (default: hex)')
    ap.add_argument('--out','-o', default=None, help='I-Mem output file')
    ap.add_argument('--dmem-out', default=None, help='D-Mem output file')
    ap.add_argument('--dmem-base', type=lambda x:int(x,0), default=0,
                    help='Byte address of D-Mem start (default: 0)')
    ap.add_argument('--imem-depth', type=int, default=512)
    ap.add_argument('--dmem-depth', type=int, default=256)
    ap.add_argument('--dump', action='store_true',
                    help='Print annotated disassembly')
    args = ap.parse_args()

    src = open(args.input).read()
    asm = Assembler(dmem_base=args.dmem_base)
    asm.assemble(src)

    # Print labels
    print("=== Labels (I-Mem) ===", file=sys.stderr)
    for lbl, addr in sorted(asm.labels.items(), key=lambda x:x[1]):
        print(f"  {lbl:25s} byte={addr:#06x}  word={addr//4}", file=sys.stderr)

    print(f"\nI-Mem: {len(asm.text_words)} words", file=sys.stderr)
    print(f"D-Mem: {len(asm.data_words)} words", file=sys.stderr)

    # Dump
    if args.dump:
        print("\n=== I-Mem (Instructions + Literal Pool) ===", file=sys.stderr)
        for addr, word, src in asm.text_words:
            print(f"  {addr//4:04X}({addr:04X})  {word:08X}  {disasm(word):38s}  ; {src.strip()}",
                  file=sys.stderr)
        if asm.data_words:
            print("\n=== D-Mem (Data Section) ===", file=sys.stderr)
            for addr, word, src in asm.data_words:
                sv = word if word < 0x80000000 else word - 0x100000000
                print(f"  word={addr//4:03d}  {word:08X}  (={sv:d})  ; {src.strip()}", file=sys.stderr)

    # Write I-Mem
    if args.format == 'coe':
        text = to_coe(asm.text_words, args.imem_depth)
    elif args.format == 'mif':
        text = to_mif(asm.text_words, args.imem_depth)
    else:
        text = to_hex(asm.text_words)

    if args.out:
        open(args.out,'w').write(text)
        print(f"I-Mem written to {args.out}", file=sys.stderr)
    else:
        print(text)

    # Write D-Mem
    if args.dmem_out and asm.data_words:
        if args.format == 'coe':
            dtxt = to_coe(asm.data_words, args.dmem_depth)
        elif args.format == 'mif':
            dtxt = to_mif(asm.data_words, args.dmem_depth)
        else:
            dtxt = to_hex(asm.data_words)
        open(args.dmem_out,'w').write(dtxt)
        print(f"D-Mem written to {args.dmem_out}", file=sys.stderr)
    elif asm.data_words:
        print("\n=== D-Mem hex (use --dmem-out to save) ===", file=sys.stderr)
        for addr, word, _ in asm.data_words:
            sv = word if word < 0x80000000 else word - 0x100000000
            print(f"  D-Mem[{addr//4:3d}] = {word:08X} ({sv:d})", file=sys.stderr)

if __name__ == '__main__':
    main()
