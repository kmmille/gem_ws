\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 11 C0 - 11 C1 - 11 C2 - 11 C3 - 11 C4 - 11 C5 + 181.5 Constant + [
   2 C0 ^2 + 2 C1 ^2 + 2 C2 ^2 + 2 C3 ^2 + 2 C4 ^2 + 2 C5 ^2 ] / 2 
Subject To
 R0: C0 = 0.5
 R1: C1 = 0.5
 R2: C4 = 5.5
 R3: C5 = 5.5
 R4: - C0 + 1e+06 alpha_ob0_seg0[0] <= 999994.8
 R5: - C2 + 1e+06 alpha_ob0_seg0[0] <= 999994.8
 R6: - C1 + 1e+06 alpha_ob0_seg0[1] <= 999994.8
 R7: - C3 + 1e+06 alpha_ob0_seg0[1] <= 999994.8
 R8: C0 + 1e+06 alpha_ob0_seg0[2] <= 1.0000018e+06
 R9: C2 + 1e+06 alpha_ob0_seg0[2] <= 1.0000018e+06
 R10: C1 + 1e+06 alpha_ob0_seg0[3] <= 1.0000018e+06
 R11: C3 + 1e+06 alpha_ob0_seg0[3] <= 1.0000018e+06
 R12: alpha_ob0_seg0[0] + alpha_ob0_seg0[1] + alpha_ob0_seg0[2]
   + alpha_ob0_seg0[3] >= 1
 R13: - C2 + 1e+06 alpha_ob0_seg1[0] <= 999994.8
 R14: - C4 + 1e+06 alpha_ob0_seg1[0] <= 999994.8
 R15: - C3 + 1e+06 alpha_ob0_seg1[1] <= 999994.8
 R16: - C5 + 1e+06 alpha_ob0_seg1[1] <= 999994.8
 R17: C2 + 1e+06 alpha_ob0_seg1[2] <= 1.0000018e+06
 R18: C4 + 1e+06 alpha_ob0_seg1[2] <= 1.0000018e+06
 R19: C3 + 1e+06 alpha_ob0_seg1[3] <= 1.0000018e+06
 R20: C5 + 1e+06 alpha_ob0_seg1[3] <= 1.0000018e+06
 R21: alpha_ob0_seg1[0] + alpha_ob0_seg1[1] + alpha_ob0_seg1[2]
   + alpha_ob0_seg1[3] >= 1
 R22: C0 >= 0
 R23: C1 >= 0
 R24: C0 <= 85
 R25: C1 <= 85
 R26: C2 >= 0
 R27: C3 >= 0
 R28: C2 <= 85
 R29: C3 <= 85
 R30: C4 >= 0
 R31: C5 >= 0
 R32: C4 <= 85
 R33: C5 <= 85
Bounds
 Constant = 1
Binaries
 alpha_ob0_seg0[0] alpha_ob0_seg0[1] alpha_ob0_seg0[2] alpha_ob0_seg0[3]
 alpha_ob0_seg1[0] alpha_ob0_seg1[1] alpha_ob0_seg1[2] alpha_ob0_seg1[3]
End
