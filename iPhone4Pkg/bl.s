.code 32
.text
.globl _start
_start:
_base:
	@ stupid hack relocator quick thing i hate this bad bad bad
	@ works for iBoot since we have zero control over where it jumps
	@ into images...

	@ Disable interrupts
	cpsid	if, #0x13
	@ Disable MMU
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #1
	mcr	p15, 0, r0, c1, c0, 0
	isb	sy

	@ Load base
	adr	r0, _start
	ldr	r1, =0x41008000
	cmp r1, r0
	beq .L_relocate_efi
	
	mov	r4, r1
	mov	r2, #(2 * 1024 * 1024)
.L_relocate_loop:
	ldr	r3, [r0], #4
	str	r3, [r1], #4
	subs	r2, r2, #4
	bgt	.L_relocate_loop
	
	@ Relocated bootloader. Now we can use immvals.
.L_relocate_efi:
	ldr	r0, =___Build_iPhone4_DEBUG_ARMGCC_FV_IPHONE4_EFI_fd
	ldr	r1, =0x42008000
	mov	r4, r1
	mov	r2, #(2 * 1024 * 1024)
.L_relocate_loop2:
	ldr	r3, [r0], #4
	str	r3, [r1], #4
	subs	r2, r2, #4
	bgt	.L_relocate_loop2

	@ Done relocating, jump.
	bx	r4
	

