# mark_description "Intel(R) C Intel(R) 64 Compiler for applications running on Intel(R) 64, Version 18.0.1.163 Build 20171018";
# mark_description "-S -xHost -qopenmp -fno-alias";
	.file "MatMatMultiplyBlockHelper.cpp"
	.text
..TXTST0:
.L_2__routine_start__Z25MatMatMultiplyBlockHelperRA32_A32_KfS2_RA32_A32_f_0:
# -- Begin  _Z25MatMatMultiplyBlockHelperRA32_A32_KfS2_RA32_A32_f
	.text
# mark_begin;
       .align    16,0x90
	.globl _Z25MatMatMultiplyBlockHelperRA32_A32_KfS2_RA32_A32_f
# --- MatMatMultiplyBlockHelper(const float (&)[32][32], const float (&)[32][32], float (&)[32][32])
_Z25MatMatMultiplyBlockHelperRA32_A32_KfS2_RA32_A32_f:
# parameter 1: %rdi
# parameter 2: %rsi
# parameter 3: %rdx
..B1.1:                         # Preds ..B1.0
                                # Execution count [1.00e+00]
	.cfi_startproc
	.cfi_personality 0x3,__gxx_personality_v0
..___tag_value__Z25MatMatMultiplyBlockHelperRA32_A32_KfS2_RA32_A32_f.1:
..L2:
                                                          #5.1
        pushq     %rbp                                          #5.1
	.cfi_def_cfa_offset 16
        movq      %rsp, %rbp                                    #5.1
	.cfi_def_cfa 6, 16
	.cfi_offset 6, -16
        andq      $-64, %rsp                                    #5.1
        pushq     %r14                                          #5.1
        pushq     %r15                                          #5.1
        pushq     %rbx                                          #5.1
        subq      $168, %rsp                                    #5.1
        movq      %rdx, %rcx                                    #5.1
        xorb      %dl, %dl                                      #21.5
        xorl      %eax, %eax                                    #21.5
	.cfi_escape 0x10, 0x03, 0x0e, 0x38, 0x1c, 0x0d, 0xc0, 0xff, 0xff, 0xff, 0x1a, 0x0d, 0xe8, 0xff, 0xff, 0xff, 0x22
	.cfi_escape 0x10, 0x0e, 0x0e, 0x38, 0x1c, 0x0d, 0xc0, 0xff, 0xff, 0xff, 0x1a, 0x0d, 0xf8, 0xff, 0xff, 0xff, 0x22
	.cfi_escape 0x10, 0x0f, 0x0e, 0x38, 0x1c, 0x0d, 0xc0, 0xff, 0xff, 0xff, 0x1a, 0x0d, 0xf0, 0xff, 0xff, 0xff, 0x22
                                # LOE rax rcx rsi rdi r12 r13 dl
..B1.2:                         # Preds ..B1.6 ..B1.1
                                # Execution count [2.00e+00]
        xorb      %r10b, %r10b                                  #22.5
        lea       (%rdi,%rax), %r9                              #30.40
        xorl      %ebx, %ebx                                    #22.5
        lea       (%rcx,%rax), %r8                              #30.17
                                # LOE rax rcx rbx rsi rdi r8 r9 r12 r13 dl r10b
..B1.3:                         # Preds ..B1.5 ..B1.2
                                # Execution count [4.00e+00]
        xorl      %r15d, %r15d                                  #23.5
        lea       (%rsi,%rbx), %r14                             #30.62
        vmovups   1952(%rbx,%r8), %ymm30                        #30.17
        xorl      %r11d, %r11d                                  #23.5
        vmovups   1792(%rbx,%r8), %ymm1                         #30.17
        vmovups   1056(%rbx,%r8), %ymm0                         #30.17
        vmovups   (%rbx,%r8), %ymm31                            #30.17
        vmovups   32(%rbx,%r8), %ymm2                           #30.17
        vmovups   1920(%rbx,%r8), %ymm3                         #30.17
        vmovups   1824(%rbx,%r8), %ymm4                         #30.17
        vmovups   1696(%rbx,%r8), %ymm5                         #30.17
        vmovups   1664(%rbx,%r8), %ymm6                         #30.17
        vmovups   1568(%rbx,%r8), %ymm7                         #30.17
        vmovups   1536(%rbx,%r8), %ymm8                         #30.17
        vmovups   1440(%rbx,%r8), %ymm9                         #30.17
        vmovups   1408(%rbx,%r8), %ymm10                        #30.17
        vmovups   1312(%rbx,%r8), %ymm11                        #30.17
        vmovups   1280(%rbx,%r8), %ymm12                        #30.17
        vmovups   1184(%rbx,%r8), %ymm13                        #30.17
        vmovups   1152(%rbx,%r8), %ymm14                        #30.17
        vmovups   1024(%rbx,%r8), %ymm15                        #30.17
        vmovups   928(%rbx,%r8), %ymm16                         #30.17
        vmovups   896(%rbx,%r8), %ymm17                         #30.17
        vmovups   800(%rbx,%r8), %ymm18                         #30.17
        vmovups   768(%rbx,%r8), %ymm19                         #30.17
        vmovups   672(%rbx,%r8), %ymm20                         #30.17
        vmovups   640(%rbx,%r8), %ymm21                         #30.17
        vmovups   544(%rbx,%r8), %ymm22                         #30.17
        vmovups   512(%rbx,%r8), %ymm23                         #30.17
        vmovups   416(%rbx,%r8), %ymm24                         #30.17
        vmovups   384(%rbx,%r8), %ymm25                         #30.17
        vmovups   288(%rbx,%r8), %ymm26                         #30.17
        vmovups   256(%rbx,%r8), %ymm27                         #30.17
        vmovups   160(%rbx,%r8), %ymm28                         #30.17
        vmovups   128(%rbx,%r8), %ymm29                         #30.17
        vmovups   %ymm31, 32(%rsp)                              #23.5[spill]
        vmovups   %ymm0, 64(%rsp)                               #23.5[spill]
        vmovups   %ymm1, 96(%rsp)                               #23.5[spill]
        vmovups   %ymm30, (%rsp)                                #23.5[spill]
                                # LOE rax rcx rbx rsi rdi r8 r9 r11 r12 r13 r14 r15 dl r10b ymm2 ymm3 ymm4 ymm5 ymm6 ymm7 ymm8 ymm9 ymm10 ymm11 ymm12 ymm13 ymm14 ymm15 ymm16 ymm17 ymm18 ymm19 ymm20 ymm21 ymm22 ymm23 ymm24 ymm25 ymm26 ymm27 ymm28 ymm29
..B1.4:                         # Preds ..B1.4 ..B1.3
                                # Execution count [1.28e+02]
        vmovups   (%r11,%r14), %ymm1                            #30.62
        vmovups   32(%rsp), %ymm30                              #30.17[spill]
        vmovups   32(%r11,%r14), %ymm0                          #30.62
        vbroadcastss (%r9,%r15,4), %ymm31                       #30.40
        vfmadd231ps %ymm31, %ymm1, %ymm30                       #30.17
        addq      $128, %r11                                    #23.5
        vfmadd231ps %ymm31, %ymm0, %ymm2                        #30.17
        vbroadcastss 256(%r9,%r15,4), %ymm31                    #30.40
        vmovups   %ymm30, 32(%rsp)                              #30.17[spill]
        vmovups   %ymm2, 32(%rbx,%r8)                           #30.17
        vfmadd231ps %ymm31, %ymm1, %ymm27                       #30.17
        vfmadd231ps %ymm31, %ymm0, %ymm26                       #30.17
        vbroadcastss 128(%r9,%r15,4), %ymm30                    #30.40
        vbroadcastss 512(%r9,%r15,4), %ymm31                    #30.40
        vmovups   %ymm27, 256(%rbx,%r8)                         #30.17
        vmovups   %ymm26, 288(%rbx,%r8)                         #30.17
        vfmadd231ps %ymm30, %ymm1, %ymm29                       #30.17
        vfmadd231ps %ymm30, %ymm0, %ymm28                       #30.17
        vfmadd231ps %ymm31, %ymm1, %ymm23                       #30.17
        vfmadd231ps %ymm31, %ymm0, %ymm22                       #30.17
        vbroadcastss 384(%r9,%r15,4), %ymm30                    #30.40
        vbroadcastss 768(%r9,%r15,4), %ymm31                    #30.40
        vmovups   %ymm29, 128(%rbx,%r8)                         #30.17
        vmovups   %ymm28, 160(%rbx,%r8)                         #30.17
        vfmadd231ps %ymm30, %ymm1, %ymm25                       #30.17
        vfmadd231ps %ymm30, %ymm0, %ymm24                       #30.17
        vfmadd231ps %ymm31, %ymm1, %ymm19                       #30.17
        vfmadd231ps %ymm31, %ymm0, %ymm18                       #30.17
        vmovups   %ymm23, 512(%rbx,%r8)                         #30.17
        vmovups   %ymm25, 384(%rbx,%r8)                         #30.17
        vmovups   %ymm24, 416(%rbx,%r8)                         #30.17
        vmovups   %ymm22, 544(%rbx,%r8)                         #30.17
        vmovups   %ymm19, 768(%rbx,%r8)                         #30.17
        vmovups   %ymm18, 800(%rbx,%r8)                         #30.17
        vbroadcastss 640(%r9,%r15,4), %ymm30                    #30.40
        vbroadcastss 1024(%r9,%r15,4), %ymm31                   #30.40
        vfmadd231ps %ymm30, %ymm1, %ymm21                       #30.17
        vfmadd231ps %ymm30, %ymm0, %ymm20                       #30.17
        vfmadd231ps %ymm31, %ymm1, %ymm15                       #30.17
        vbroadcastss 896(%r9,%r15,4), %ymm30                    #30.40
        vmovups   %ymm21, 640(%rbx,%r8)                         #30.17
        vmovups   %ymm20, 672(%rbx,%r8)                         #30.17
        vmovups   %ymm15, 1024(%rbx,%r8)                        #30.17
        vfmadd231ps %ymm30, %ymm1, %ymm17                       #30.17
        vfmadd231ps %ymm30, %ymm0, %ymm16                       #30.17
        vmovups   64(%rsp), %ymm30                              #30.17[spill]
        vmovups   %ymm17, 896(%rbx,%r8)                         #30.17
        vmovups   %ymm16, 928(%rbx,%r8)                         #30.17
        vfmadd231ps %ymm31, %ymm0, %ymm30                       #30.17
        vbroadcastss 1152(%r9,%r15,4), %ymm31                   #30.40
        vmovups   %ymm30, 64(%rsp)                              #30.17[spill]
        vmovups   %ymm30, 1056(%rbx,%r8)                        #30.17
        vfmadd231ps %ymm31, %ymm1, %ymm14                       #30.17
        vfmadd231ps %ymm31, %ymm0, %ymm13                       #30.17
        vbroadcastss 1280(%r9,%r15,4), %ymm30                   #30.40
        vbroadcastss 1408(%r9,%r15,4), %ymm31                   #30.40
        vmovups   %ymm14, 1152(%rbx,%r8)                        #30.17
        vmovups   %ymm13, 1184(%rbx,%r8)                        #30.17
        vfmadd231ps %ymm30, %ymm1, %ymm12                       #30.17
        vfmadd231ps %ymm30, %ymm0, %ymm11                       #30.17
        vfmadd231ps %ymm31, %ymm1, %ymm10                       #30.17
        vfmadd231ps %ymm31, %ymm0, %ymm9                        #30.17
        vbroadcastss 1536(%r9,%r15,4), %ymm30                   #30.40
        vbroadcastss 1664(%r9,%r15,4), %ymm31                   #30.40
        vmovups   %ymm12, 1280(%rbx,%r8)                        #30.17
        vmovups   %ymm11, 1312(%rbx,%r8)                        #30.17
        vfmadd231ps %ymm30, %ymm1, %ymm8                        #30.17
        vfmadd231ps %ymm30, %ymm0, %ymm7                        #30.17
        .byte     144                                           #30.17
        vmovups   96(%rsp), %ymm30                              #30.17[spill]
        vfmadd231ps %ymm31, %ymm1, %ymm6                        #30.17
        vfmadd231ps %ymm31, %ymm0, %ymm5                        #30.17
        vmovups   %ymm10, 1408(%rbx,%r8)                        #30.17
        vmovups   %ymm9, 1440(%rbx,%r8)                         #30.17
        vmovups   %ymm8, 1536(%rbx,%r8)                         #30.17
        vmovups   %ymm7, 1568(%rbx,%r8)                         #30.17
        vmovups   %ymm6, 1664(%rbx,%r8)                         #30.17
        vmovups   %ymm5, 1696(%rbx,%r8)                         #30.17
        vbroadcastss 1792(%r9,%r15,4), %ymm31                   #30.40
        vfmadd231ps %ymm31, %ymm1, %ymm30                       #30.17
        vfmadd231ps %ymm31, %ymm0, %ymm4                        #30.17
        vmovups   %ymm30, 96(%rsp)                              #30.17[spill]
        vmovups   %ymm30, 1792(%rbx,%r8)                        #30.17
        vmovups   %ymm4, 1824(%rbx,%r8)                         #30.17
        vbroadcastss 1920(%r9,%r15,4), %ymm30                   #30.40
        vfmadd231ps %ymm30, %ymm1, %ymm3                        #30.17
        incq      %r15                                          #23.5
        vmovups   (%rsp), %ymm1                                 #30.17[spill]
        vmovups   %ymm3, 1920(%rbx,%r8)                         #30.17
        vfmadd231ps %ymm30, %ymm0, %ymm1                        #30.17
        vmovups   %ymm1, (%rsp)                                 #30.17[spill]
        cmpq      $32, %r15                                     #23.5
        jb        ..B1.4        # Prob 96%                      #23.5
                                # LOE rax rcx rbx rsi rdi r8 r9 r11 r12 r13 r14 r15 dl r10b ymm1 ymm2 ymm3 ymm4 ymm5 ymm6 ymm7 ymm8 ymm9 ymm10 ymm11 ymm12 ymm13 ymm14 ymm15 ymm16 ymm17 ymm18 ymm19 ymm20 ymm21 ymm22 ymm23 ymm24 ymm25 ymm26 ymm27 ymm28 ymm29
..B1.5:                         # Preds ..B1.4
                                # Execution count [4.00e+00]
        vmovups   32(%rsp), %ymm31                              #[spill]
        incb      %r10b                                         #22.5
        vmovaps   %ymm1, %ymm30                                 #
        vmovups   %ymm31, (%rbx,%r8)                            #30.17
        vmovups   %ymm30, 1952(%rbx,%r8)                        #30.17
        addq      $64, %rbx                                     #22.5
        cmpb      $2, %r10b                                     #22.5
        jb        ..B1.3        # Prob 49%                      #22.5
                                # LOE rax rcx rbx rsi rdi r8 r9 r12 r13 dl r10b
..B1.6:                         # Preds ..B1.5
                                # Execution count [2.00e+00]
        incb      %dl                                           #21.5
        addq      $2048, %rax                                   #21.5
        cmpb      $2, %dl                                       #21.5
        jb        ..B1.2        # Prob 50%                      #21.5
                                # LOE rax rcx rsi rdi r12 r13 dl
..B1.7:                         # Preds ..B1.6
                                # Execution count [1.00e+00]
        vzeroupper                                              #32.1
        addq      $168, %rsp                                    #32.1
	.cfi_restore 3
        popq      %rbx                                          #32.1
	.cfi_restore 15
        popq      %r15                                          #32.1
	.cfi_restore 14
        popq      %r14                                          #32.1
        movq      %rbp, %rsp                                    #32.1
        popq      %rbp                                          #32.1
	.cfi_def_cfa 7, 8
	.cfi_restore 6
        ret                                                     #32.1
        .align    16,0x90
                                # LOE
	.cfi_endproc
# mark_end;
	.type	_Z25MatMatMultiplyBlockHelperRA32_A32_KfS2_RA32_A32_f,@function
	.size	_Z25MatMatMultiplyBlockHelperRA32_A32_KfS2_RA32_A32_f,.-_Z25MatMatMultiplyBlockHelperRA32_A32_KfS2_RA32_A32_f
..LN_Z25MatMatMultiplyBlockHelperRA32_A32_KfS2_RA32_A32_f.0:
	.data
# -- End  _Z25MatMatMultiplyBlockHelperRA32_A32_KfS2_RA32_A32_f
	.data
	.section .note.GNU-stack, ""
// -- Begin DWARF2 SEGMENT .eh_frame
	.section .eh_frame,"a",@progbits
.eh_frame_seg:
	.align 8
# End
