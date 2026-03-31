#include <stdint.h>
#include <cuda_bf16.h> 

// Vector Addition: Compute element-wise addition 
__global__ void int16_vector_add(int16_t *a, int16_t *b, int16_t *c) {
    int idx = threadIdx.x; 
    c[idx] = a[idx] + b[idx];
}

// Vector Subtraction: Compute element-wise subtraction
__global__ void int16_vector_sub(int16_t *a, int16_t *b, int16_t *c) {
    int idx = threadIdx.x;
    c[idx] = a[idx] - b[idx];
}

// BFloat16 Vector Multiply
__global__ void bf16_vector_mul(__nv_bfloat16 *a, __nv_bfloat16 *b, __nv_bfloat16 *c) { 
    int idx = threadIdx.x; 
    c[idx] = __hmul(a[idx], b[idx]); 
}

// BFloat16 Fused Multiply-Accumulate 
__global__ void bf16_fma(__nv_bfloat16 *a, __nv_bfloat16 *b, __nv_bfloat16 *c, __nv_bfloat16 *d) { 
    int idx = threadIdx.x; 
    d[idx] = __hfma(a[idx], b[idx], c[idx]); 
}

// ReLU Activation: out[i] = max(0, in[i])
__global__ void int16_relu(int16_t *in, int16_t *out) {
    int idx = threadIdx.x;
    out[idx] = (in[idx] > 0) ? in[idx] : 0;
}