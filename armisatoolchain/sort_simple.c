/*
 * sort_simple.c  —  EE 533 Lab 6  Team 10
 *
 * Simplified bubble sort designed to compile into a minimal
 * ARM instruction subset (no stack frames, no ldmia/stmia).
 *
 * Compile to assembly:
 *   arm-none-eabi-gcc -O1 -S -march=armv4t -marm sort_simple.c
 *
 * The -O1 flag avoids the complex stack-frame code that -O0 generates
 * (no ldmia/stmia/push/pop), while -O2/-O3 may unroll loops.
 * -O1 is the sweet spot for a minimal instruction subset.
 *
 * Expected instruction subset in output:
 *   MOV, ADD, SUB, LSL, LDR, STR, CMP, BGE/BLE/BLT/B
 */

/* Array lives in a fixed memory region (word-addressed, base = 0).
 * We use a global so the compiler doesn't put it on the stack. */
int N = 10;
int array[10] = {323, 123, -455, 2, 98, 125, 10, 65, -56, 0};

void bubble_sort(void) {
    int i, j, tmp;
    for (i = 0; i < N - 1; i++) {
        for (j = 0; j < N - 1 - i; j++) {
            if (array[j] > array[j + 1]) {
                tmp         = array[j];
                array[j]    = array[j + 1];
                array[j + 1] = tmp;
            }
        }
    }
}

int main(void) {
    bubble_sort();
    return 0;
}
