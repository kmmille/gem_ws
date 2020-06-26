\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
 
Subject To
 R0: C0 = 0.5
 R1: C1 = 0.5
 R2: C4 = 5.5
 R3: C5 = 5.5
 R4: - C2 + 1e+06 alpha_ob0_seg1[0] <= 999995.9
 R5: - C4 + 1e+06 alpha_ob0_seg1[0] <= 999995.9
 R6: alpha_ob0_seg1[0] + alpha_ob0_seg1[1] + alpha_ob0_seg1[2]
   + alpha_ob0_seg1[3] >= 1
 R7: - C3 + 1e+06 alpha_ob0_seg1[1] <= 999995.9
 R8: - C5 + 1e+06 alpha_ob0_seg1[1] <= 999995.9
 R9: alpha_ob0_seg1[0] + alpha_ob0_seg1[1] + alpha_ob0_seg1[2]
   + alpha_ob0_seg1[3] >= 1
 R10: C2 + 1e+06 alpha_ob0_seg1[2] <= 1.0000029e+06
 R11: C4 + 1e+06 alpha_ob0_seg1[2] <= 1.0000029e+06
 R12: alpha_ob0_seg1[0] + alpha_ob0_seg1[1] + alpha_ob0_seg1[2]
   + alpha_ob0_seg1[3] >= 1
 R13: C3 + 1e+06 alpha_ob0_seg1[3] <= 1.0000029e+06
 R14: C5 + 1e+06 alpha_ob0_seg1[3] <= 1.0000029e+06
 R15: alpha_ob0_seg1[0] + alpha_ob0_seg1[1] + alpha_ob0_seg1[2]
   + alpha_ob0_seg1[3] >= 1
 R16: C0 >= 0.1
 R17: C1 >= 0.1
 R18: C0 <= 10
 R19: C1 <= 10
 R20: C2 >= 0.1
 R21: C3 >= 0.1
 R22: C2 <= 10
 R23: C3 <= 10
 R24: C4 >= 0.1
 R25: C5 >= 0.1
 R26: C4 <= 10
 R27: C5 <= 10
Bounds
Binaries
 alpha_ob0_seg1[0] alpha_ob0_seg1[1] alpha_ob0_seg1[2] alpha_ob0_seg1[3]
End
