
  .global read_c28_value
  
  .global read_c29_value

  .global read_c30_value

  .global enable_rx_isr

read_c28_value:
    lbco &R14, c28, 0, 4   ; read constant table entry C28
	JMP		r3.w2

read_c29_value:
    lbco &R14, c29, 0, 4   ; read constant table entry C29
	JMP		r3.w2

read_c30_value:
    lbco &R14, c30, 0, 4   ; read constant table entry C30
	JMP		r3.w2

enable_rx_isr:
	mov r31.b0, r14.b0
	JMP r3.w2
