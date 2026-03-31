import struct

def float32_to_bfloat16(f32_val):
    """Converts a float32 to a 16-bit integer representing a BFloat16"""
    # Pack float to bytes, unpack as 32-bit int
    packed = struct.pack('>f', f32_val)
    int32_rep = struct.unpack('>I', packed)[0]
    
    # BFloat16 is literally just the top 16 bits of a Float32
    bf16_int = (int32_rep >> 16) & 0xFFFF
    return bf16_int

def bfloat16_to_float32(bf16_int):
    """Converts a 16-bit BFloat16 integer back to float32"""
    # Pad the bottom 16 bits with zeros
    int32_rep = (bf16_int & 0xFFFF) << 16
    packed = struct.pack('>I', int32_rep)
    f32_val = struct.unpack('>f', packed)[0]
    return f32_val

# Quick Test
f_val = 3.14159
bf16 = float32_to_bfloat16(f_val)
restored = bfloat16_to_float32(bf16)
print(f"Original: {f_val}, BFloat16 Hex: {hex(bf16)}, Restored: {restored}")