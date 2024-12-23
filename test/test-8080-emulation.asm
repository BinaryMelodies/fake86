
%macro	brkem	1
	db	0x0f, 0xff, %1
%endmacro

LOAD_ES	equ	0x80
STORE_FAR	equ	0x81
LOAD_FAR	equ	0x82
OUT_FAR	equ	0x83
IN_FAR	equ	0x84
SYSTEM	equ	0x8F

	org	0x7c00

	cli

	xor	ax, ax
	mov	ds, ax

	;;;; Native (8086) mode stack should be separate from 8080 emulation mode stack
	mov	ax, 0x1000
	mov	ss, ax
	xor	sp, sp

	;;;; Set up interrupt vectors to handle far address from 8080 emulation mode
	mov	word [4 * LOAD_ES], load_es
	mov	word [4 * LOAD_ES + 2], cs

	mov	word [4 * STORE_FAR], store_far
	mov	word [4 * STORE_FAR + 2], cs

	mov	word [4 * LOAD_FAR], load_far
	mov	word [4 * LOAD_FAR + 2], cs

	mov	word [4 * OUT_FAR], out_far
	mov	word [4 * OUT_FAR + 2], cs

	mov	word [4 * IN_FAR], in_far
	mov	word [4 * IN_FAR + 2], cs

	mov	word [4 * SYSTEM], start8080
	mov	word [4 * SYSTEM + 2], cs

	brkem	SYSTEM

.0:
	hlt
	jmp	.0

load_es:
	mov	es, bx
	iret

store_far:
	mov	[es:bx], al
	iret

load_far:
	mov	al, [es:bx]
	iret

out_far:
	out	dx, al
	iret

in_far:
	in	al, dx
	iret

start8080:
	;;;; Set up stack pointer
	; lxi	sp, 0x7c00
	db	0x31
	dw	0x7c00

	;;;; Load VGA screen buffer into 8086 mode ES for accessing via far calls
	; lxi	h, 0xb800
	db	0x21
	dw	0xb800

	; calln	LOAD_ES
	db	0xed, 0xed, LOAD_ES

	;;;; Clear VGA screen with blue background and yellow foreground
	; lxi	h, 0
	db	0x21
	dw	0

	; lxi	b, 80 * 25
	db	0x01
	dw	80 * 25

.2:
	; mvi	a, 0x20
	db	0x3e, 0x20

	; calln	STORE_FAR
	db	0xed, 0xed, STORE_FAR

	; inx	h
	db	0x23

	; mvi	a, 0x1e
	db	0x3e, 0x1e

	; calln	STORE_FAR
	db	0xed, 0xed, STORE_FAR

	; inx	h
	db	0x23

	; dcx	b
	db	0x0b

	; mov	a, b
	db	0x78

	; ora	c
	db	0xb1

	; jnz	.2
	db	0xc2
	dw	.2

	;;;; Display message
	; lxi	d, message
	db	0x11
	dw	message

	; lxi	h, 0
	db	0x21
	dw	0

	; mvi	c, message_length
	db	0x0e, message_length

.1:
	; ldax	d
	db	0x1a

	; inx	d
	db	0x13

	;calln	STORE_FAR
	db	0xed, 0xed, STORE_FAR

	; inx	h
	db	0x23

	; mvi	a, 0x1e
	db	0x3e, 0x1e

	; calln	STORE_FAR
	db	0xed, 0xed, STORE_FAR

	; inx	h
	db	0x23

	; dcr	c
	db	0x0d

	; jnz	.1
	db	0xc2
	dw	.1

	;;;; Move cursor to end of screen
	; lxi	d, 0x03d4
	db	0x11
	dw	0x03d4

	; mvi	a, 0x0f
	db	0x3e, 0x0f

	; calln	OUT_FAR
	db	0xed, 0xed, OUT_FAR

	; inx	d
	db	0x13

	; mvi	a, (80 * 24 + 79) & 0xff
	db	0x3e, (80 * 24 + 79) & 0xff

	; calln	OUT_FAR
	db	0xed, 0xed, OUT_FAR

	; lxi	d, 0x03d4
	db	0x11
	dw	0x03d4

	; mvi	a, 0x0e
	db	0x3e, 0x0e

	; calln	OUT_FAR
	db	0xed, 0xed, OUT_FAR

	; inx	d
	db	0x13

	; mvi	a, (80 * 24 + 79) >> 8
	db	0x3e, (80 * 24 + 79) >> 8

	; calln	OUT_FAR
	db	0xed, 0xed, OUT_FAR

.0:
	; hlt
	db	0x76

	; jmp	.0
	db	0xc3
	dw	.0

message:
	db	"This code is executed from 8080 emulation mode"
message_length	equ	$ - message

	times	512-2-($-$$) db 0
	db	0x55, 0xaa

