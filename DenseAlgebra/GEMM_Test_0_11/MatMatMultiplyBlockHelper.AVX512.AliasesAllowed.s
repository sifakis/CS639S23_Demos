# mark_description "Intel(R) C Intel(R) 64 Compiler for applications running on Intel(R) 64, Version 18.0.1.163 Build 20171018";
# mark_description "-S -qopenmp -xCOMMON-AVX512 -o MatMatMultiplyBlockHelper.AVX512.AliasesAllowed.s";
	.file "MatMatMultiplyBlockHelper.cpp"
	.text
..TXTST0:
.L_2__routine_start__Z25MatMatMultiplyBlockHelperRA64_A64_KfS2_RA64_A64_f_0:
# -- Begin  _Z25MatMatMultiplyBlockHelperRA64_A64_KfS2_RA64_A64_f
	.text
# mark_begin;
       .align    16,0x90
	.globl _Z25MatMatMultiplyBlockHelperRA64_A64_KfS2_RA64_A64_f
# --- MatMatMultiplyBlockHelper(const float (&)[64][64], const float (&)[64][64], float (&)[64][64])
_Z25MatMatMultiplyBlockHelperRA64_A64_KfS2_RA64_A64_f:
# parameter 1: %rdi
# parameter 2: %rsi
# parameter 3: %rdx
..B1.1:                         # Preds ..B1.0
                                # Execution count [1.00e+00]
	.cfi_startproc
	.cfi_personality 0x3,__gxx_personality_v0
..___tag_value__Z25MatMatMultiplyBlockHelperRA64_A64_KfS2_RA64_A64_f.1:
..L2:
                                                          #5.1
        xorb      %cl, %cl                                      #6.5
        xorl      %eax, %eax                                    #6.5
                                # LOE rax rdx rbx rbp rsi rdi r12 r13 r14 r15 cl
..B1.2:                         # Preds ..B1.4 ..B1.1
                                # Execution count [6.40e+01]
        xorl      %r10d, %r10d                                  #7.9
        lea       (%rdi,%rax), %r9                              #10.27
        vmovups   192(%rax,%rdx), %zmm3                         #10.13
        xorl      %r8d, %r8d                                    #7.9
        vmovups   128(%rax,%rdx), %zmm2                         #10.13
        vmovups   64(%rax,%rdx), %zmm1                          #10.13
        vmovups   (%rax,%rdx), %zmm0                            #10.13
        .align    16,0x90
                                # LOE rax rdx rbx rbp rsi rdi r8 r9 r10 r12 r13 r14 r15 cl zmm0 zmm1 zmm2 zmm3
..B1.3:                         # Preds ..B1.3 ..B1.2
                                # Execution count [4.10e+03]
        vbroadcastss (%r9,%r10,4), %zmm4                        #10.27
        vfmadd231ps (%r8,%rsi), %zmm4, %zmm0                    #10.13
        vmovups   %zmm0, (%rax,%rdx)                            #10.13
        vbroadcastss (%r9,%r10,4), %zmm5                        #10.27
        vfmadd231ps 64(%r8,%rsi), %zmm5, %zmm1                  #10.13
        vmovups   %zmm1, 64(%rax,%rdx)                          #10.13
        vbroadcastss (%r9,%r10,4), %zmm6                        #10.27
        vfmadd231ps 128(%r8,%rsi), %zmm6, %zmm2                 #10.13
        vmovups   %zmm2, 128(%rax,%rdx)                         #10.13
        vbroadcastss (%r9,%r10,4), %zmm7                        #10.27
        incq      %r10                                          #7.9
        vfmadd231ps 192(%r8,%rsi), %zmm7, %zmm3                 #10.13
        addq      $256, %r8                                     #7.9
        vmovups   %zmm3, 192(%rax,%rdx)                         #10.13
        cmpq      $64, %r10                                     #7.9
        jb        ..B1.3        # Prob 98%                      #7.9
                                # LOE rax rdx rbx rbp rsi rdi r8 r9 r10 r12 r13 r14 r15 cl zmm0 zmm1 zmm2 zmm3
..B1.4:                         # Preds ..B1.3
                                # Execution count [6.40e+01]
        incb      %cl                                           #6.5
        addq      $256, %rax                                    #6.5
        cmpb      $64, %cl                                      #6.5
        jb        ..B1.2        # Prob 98%                      #6.5
                                # LOE rax rdx rbx rbp rsi rdi r12 r13 r14 r15 cl
..B1.5:                         # Preds ..B1.4
                                # Execution count [1.00e+00]
        vzeroupper                                              #12.1
        ret                                                     #12.1
        .align    16,0x90
                                # LOE
	.cfi_endproc
# mark_end;
	.type	_Z25MatMatMultiplyBlockHelperRA64_A64_KfS2_RA64_A64_f,@function
	.size	_Z25MatMatMultiplyBlockHelperRA64_A64_KfS2_RA64_A64_f,.-_Z25MatMatMultiplyBlockHelperRA64_A64_KfS2_RA64_A64_f
..LN_Z25MatMatMultiplyBlockHelperRA64_A64_KfS2_RA64_A64_f.0:
	.data
# -- End  _Z25MatMatMultiplyBlockHelperRA64_A64_KfS2_RA64_A64_f
	.data
	.section .note.GNU-stack, ""
// -- Begin DWARF2 SEGMENT .eh_frame
	.section .eh_frame,"a",@progbits
.eh_frame_seg:
	.align 8
# End
