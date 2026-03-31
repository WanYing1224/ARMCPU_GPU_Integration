	.arch armv4t
	.fpu softvfp
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 1
	.eabi_attribute 34, 0
	.eabi_attribute 18, 4
	.file	"sort_simple.c"
	.text
	.align	2
	.global	bubble_sort
	.syntax unified
	.arm
	.type	bubble_sort, %function
bubble_sort:
	@ Function supports interworking.
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L14
	ldr	r3, [r3]
	cmp	r3, #1
	bxle	lr
	str	lr, [sp, #-4]!
	sub	ip, r3, #1
	ldr	r0, .L14
	add	lr, r0, #4
	add	r0, r0, r3, lsl #2
.L3:
	cmp	ip, #0
	suble	ip, ip, #1
	suble	r0, r0, #4
	ble	.L3
	mov	r3, lr
.L5:
	ldr	r2, [r3]
	ldr	r1, [r3, #4]!
	cmp	r2, r1
	strgt	r1, [r3, #-4]
	strgt	r2, [r3]
	cmp	r3, r0
	bne	.L5
	subs	ip, ip, #1
	subne	r0, r0, #4
	bne	.L3
.L1:
	ldr	lr, [sp], #4
	bx	lr
.L15:
	.align	2
.L14:
	.word	.LANCHOR0
	.size	bubble_sort, .-bubble_sort
	.align	2
	.global	main
	.syntax unified
	.arm
	.type	main, %function
main:
	@ Function supports interworking.
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	bl	bubble_sort
	mov	r0, #0
	pop	{r4, lr}
	bx	lr
	.size	main, .-main
	.global	array
	.global	N
	.data
	.align	2
	.set	.LANCHOR0,. + 0
	.type	N, %object
	.size	N, 4
N:
	.word	10
	.type	array, %object
	.size	array, 40
array:
	.word	323
	.word	123
	.word	-455
	.word	2
	.word	98
	.word	125
	.word	10
	.word	65
	.word	-56
	.word	0
	.ident	"GCC: (Arm GNU Toolchain 15.2.Rel1 (Build arm-15.86)) 15.2.1 20251203"
