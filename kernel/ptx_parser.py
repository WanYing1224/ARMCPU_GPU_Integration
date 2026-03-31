import re
import sys

OPCODES = {
    'READ_TID': '000001',
    'LDI':      '000010',
    'LD64':     '000011',
    'ST64':     '000100',
    'ADD_I16':  '010000',
    'SUB_I16':  '010001',
    'BF_MUL':   '100000',
    'BF_MAC':   '100001',
    'RELU':     '100010'
}

class PTXCompiler:
    def __init__(self):
        self.reg_map = {}
        self.next_hw_reg = 2 
        self.machine_code = []
        self.mem_offset = 0

    def allocate_reg(self, ptx_reg):
        if ptx_reg not in self.reg_map:
            self.reg_map[ptx_reg] = self.next_hw_reg
            self.next_hw_reg += 1
        return self.reg_map[ptx_reg]

    def emit_r_type(self, opcode_str, rd, rs1, rs2, rs3=0):
        opcode = OPCODES[opcode_str]
        rd_bin = format(rd, '04b')
        rs1_bin = format(rs1, '04b')
        rs2_bin = format(rs2, '04b')
        rs3_bin = format(rs3, '04b')
        instr_bin = f"{opcode}{rd_bin}{rs1_bin}{rs2_bin}{rs3_bin}0000000000"
        self.machine_code.append(format(int(instr_bin, 2), '08X'))

    def emit_i_type(self, opcode_str, rd, rs_base, imm):
        opcode = OPCODES[opcode_str]
        rd_bin = format(rd, '04b')
        rs_base_bin = format(rs_base, '04b')
        imm_bin = format(imm & 0x3FFFF, '018b')
        instr_bin = f"{opcode}{rd_bin}{rs_base_bin}{imm_bin}"
        self.machine_code.append(format(int(instr_bin, 2), '08X'))

    def parse_file(self, filename):
        self.emit_r_type('READ_TID', 1, 0, 0)
        in_target_kernel = False

        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                
                # TARGET LOCK: Only parse the vector addition kernel
                '''
                if line.startswith('.visible .entry') and 'int16_vector_add' in line:
                    in_target_kernel = True
                    continue
                
                # If we hit another kernel, stop parsing to prevent overflow
                if line.startswith('.visible .entry') and 'int16_vector_add' not in line:
                    in_target_kernel = False
                    continue
                '''    
                
                if line.startswith('.visible .entry') and 'bf16_fma' in line:
                    in_target_kernel = True
                    continue
                
                # If we hit another kernel, stop parsing to prevent overflow
                if line.startswith('.visible .entry') and 'bf16_fma' not in line:
                    in_target_kernel = False
                    continue
                
                if not in_target_kernel:
                    continue
                
                clean_line = line.replace(';', '').replace('}', '').replace('{', '').replace('[', '').replace(']', '').replace('(', '').replace(')', '')
                parts = re.split(r'[\s,]+', clean_line)
                
                if not parts or not parts[0]:
                    continue
                    
                instr = parts[0]

                try:
                    # Memory Load (Strictly global data)
                    if instr.startswith('ld.'):
                        if 'global' in instr:
                            rd = self.allocate_reg(parts[1])
                            self.emit_i_type('LD64', rd, 0, self.mem_offset) 
                            self.mem_offset += 8 
                    
                    # Memory Store (Strictly global data)
                    elif instr.startswith('st.'):
                        if 'global' in instr:
                            rs_src = self.allocate_reg(parts[2])
                            self.emit_i_type('ST64', rs_src, 1, 0)
                    
                    # BFloat16 FMA (fma.rn.bf16)
                    elif instr.startswith('fma.'):
                        rd = self.allocate_reg(parts[1])
                        rs1 = self.allocate_reg(parts[2])
                        rs2 = self.allocate_reg(parts[3])
                        rs3 = self.allocate_reg(parts[4])
                        self.emit_r_type('BF_MAC', rd, rs1, rs2, rs3)
                    
                    # Integer Addition (Allowing s32 in case C++ promoted the int16)
                    elif instr.startswith('add.s16') or instr.startswith('add.u16') or instr.startswith('add.s32'):
                        rd = self.allocate_reg(parts[1])
                        rs1 = self.allocate_reg(parts[2])
                        rs2 = self.allocate_reg(parts[3])
                        self.emit_r_type('ADD_I16', rd, rs1, rs2)

                except Exception as e:
                    print(f"Error parsing line: '{line}' - {e}")

        with open('gpu_program.hex', 'w') as out_f:
            for hex_code in self.machine_code:
                out_f.write(f"{hex_code}\n")
        print("Compilation successful: Isolated int16_vector_add.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python ptx_parser.py <kernel.ptx>")
        sys.exit(1)
    compiler = PTXCompiler()
    compiler.parse_file(sys.argv[1])