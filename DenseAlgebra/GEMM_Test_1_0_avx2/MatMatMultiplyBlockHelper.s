# mark_description "Intel(R) C Intel(R) 64 Compiler for applications running on Intel(R) 64, Version 19.1.0.166 Build 20191121";
# mark_description "-S -xavx2 -fno-alias";
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
                                                          #6.1
        xorb      %cl, %cl                                      #9.5
        xorl      %eax, %eax                                    #9.5
                                # LOE rax rdx rbx rbp rsi rdi r12 r13 r14 r15 cl
..B1.2:                         # Preds ..B1.4 ..B1.1
                                # Execution count [6.40e+01]
        xorl      %r10d, %r10d                                  #10.5
        lea       (%rdi,%rax), %r9                              #13.40
        vmovups   224(%rax,%rdx), %ymm7                         #14.41
        xorl      %r8d, %r8d                                    #10.5
        vmovups   192(%rax,%rdx), %ymm6                         #14.41
        vmovups   160(%rax,%rdx), %ymm5                         #14.41
        vmovups   128(%rax,%rdx), %ymm4                         #14.41
        vmovups   96(%rax,%rdx), %ymm3                          #14.41
        vmovups   64(%rax,%rdx), %ymm2                          #14.41
        vmovups   32(%rax,%rdx), %ymm1                          #14.41
        vmovups   (%rax,%rdx), %ymm0                            #14.41
        .align    16,0x90
                                # LOE rax rdx rbx rbp rsi rdi r8 r9 r10 r12 r13 r14 r15 cl ymm0 ymm1 ymm2 ymm3 ymm4 ymm5 ymm6 ymm7
..B1.3:                         # Preds ..B1.3 ..B1.2
                                # Execution count [4.10e+03]
        vbroadcastss (%r9,%r10,4), %ymm8                        #13.25
        incq      %r10                                          #10.5
        vfmadd231ps (%r8,%rsi), %ymm8, %ymm0                    #15.18
        vfmadd231ps 32(%r8,%rsi), %ymm8, %ymm1                  #15.18
        vfmadd231ps 64(%r8,%rsi), %ymm8, %ymm2                  #15.18
        vfmadd231ps 96(%r8,%rsi), %ymm8, %ymm3                  #15.18
        vfmadd231ps 128(%r8,%rsi), %ymm8, %ymm4                 #15.18
        vfmadd231ps 160(%r8,%rsi), %ymm8, %ymm5                 #15.18
        vfmadd231ps 192(%r8,%rsi), %ymm8, %ymm6                 #15.18
        vfmadd231ps 224(%r8,%rsi), %ymm8, %ymm7                 #15.18
        addq      $256, %r8                                     #10.5
        cmpq      $64, %r10                                     #10.5
        jb        ..B1.3        # Prob 98%                      #10.5
                                # LOE rax rdx rbx rbp rsi rdi r8 r9 r10 r12 r13 r14 r15 cl ymm0 ymm1 ymm2 ymm3 ymm4 ymm5 ymm6 ymm7
..B1.4:                         # Preds ..B1.3
                                # Execution count [6.40e+01]
        incb      %cl                                           #9.5
        lea       (%rax,%rdx), %r8                              #16.30
        vmovups   %ymm5, 160(%r8)                               #16.30
        vmovups   %ymm4, 128(%r8)                               #16.30
        vmovups   %ymm3, 96(%r8)                                #16.30
        vmovups   %ymm2, 64(%r8)                                #16.30
        vmovups   %ymm1, 32(%r8)                                #16.30
        vmovups   %ymm0, (%r8)                                  #16.30
        vmovups   %ymm7, 224(%r8)                               #16.30
        vmovups   %ymm6, 192(%r8)                               #16.30
        addq      $256, %rax                                    #9.5
        cmpb      $64, %cl                                      #9.5
        jb        ..B1.2        # Prob 98%                      #9.5
                                # LOE rax rdx rbx rbp rsi rdi r12 r13 r14 r15 cl
..B1.5:                         # Preds ..B1.4
                                # Execution count [1.00e+00]
        vzeroupper                                              #18.1
        ret                                                     #18.1
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
# End
