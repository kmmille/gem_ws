\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 20 C0 - 40 C1 - 20 C2 - 40 C3 + 1000 Constant + [ 2 C0 ^2 + 2 C1 ^2
   + 2 C2 ^2 + 2 C3 ^2 ] / 2 
Subject To
 R0: C0 = 9.832686974308469
 R1: C1 = 18.54607978562129
 R2: C2 = 10
 R3: C3 = 20
 R4: - C0 + 1e+06 alpha_ob0_seg0[0] <= 999989.847606
 R5: - C2 + 1e+06 alpha_ob0_seg0[0] <= 999989.847606
 R6: alpha_ob0_seg0[0] + alpha_ob0_seg0[1] + alpha_ob0_seg0[2]
   + alpha_ob0_seg0[3] >= 1
 R7: - C1 + 1e+06 alpha_ob0_seg0[1] <= 999990.873805
 R8: - C3 + 1e+06 alpha_ob0_seg0[1] <= 999990.873805
 R9: alpha_ob0_seg0[0] + alpha_ob0_seg0[1] + alpha_ob0_seg0[2]
   + alpha_ob0_seg0[3] >= 1
 R10: C0 + 1e+06 alpha_ob0_seg0[2] <= 1.0000044947940001e+06
 R11: C2 + 1e+06 alpha_ob0_seg0[2] <= 1.0000044947940001e+06
 R12: alpha_ob0_seg0[0] + alpha_ob0_seg0[1] + alpha_ob0_seg0[2]
   + alpha_ob0_seg0[3] >= 1
 R13: C1 + 1e+06 alpha_ob0_seg0[3] <= 1.000006973805e+06
 R14: C3 + 1e+06 alpha_ob0_seg0[3] <= 1.000006973805e+06
 R15: alpha_ob0_seg0[0] + alpha_ob0_seg0[1] + alpha_ob0_seg0[2]
   + alpha_ob0_seg0[3] >= 1
 R16: - C0 + 1e+06 alpha_ob1_seg0[0] <= 999989.856
 R17: - C2 + 1e+06 alpha_ob1_seg0[0] <= 999989.856
 R18: alpha_ob1_seg0[0] + alpha_ob1_seg0[1] + alpha_ob1_seg0[2]
   + alpha_ob1_seg0[3] >= 1
 R19: - C1 + 1e+06 alpha_ob1_seg0[1] <= 1.000006973805e+06
 R20: - C3 + 1e+06 alpha_ob1_seg0[1] <= 1.000006973805e+06
 R21: alpha_ob1_seg0[0] + alpha_ob1_seg0[1] + alpha_ob1_seg0[2]
   + alpha_ob1_seg0[3] >= 1
 R22: C0 + 1e+06 alpha_ob1_seg0[2] <= 999989.856
 R23: C2 + 1e+06 alpha_ob1_seg0[2] <= 999989.856
 R24: alpha_ob1_seg0[0] + alpha_ob1_seg0[1] + alpha_ob1_seg0[2]
   + alpha_ob1_seg0[3] >= 1
 R25: C1 + 1e+06 alpha_ob1_seg0[3] <= 999990.873805
 R26: C3 + 1e+06 alpha_ob1_seg0[3] <= 999990.873805
 R27: alpha_ob1_seg0[0] + alpha_ob1_seg0[1] + alpha_ob1_seg0[2]
   + alpha_ob1_seg0[3] >= 1
 R28: - C0 + 1e+06 alpha_ob2_seg0[0] <= 999989.8476059999
 R29: - C2 + 1e+06 alpha_ob2_seg0[0] <= 999989.8476059999
 R30: alpha_ob2_seg0[0] + alpha_ob2_seg0[1] + alpha_ob2_seg0[2]
   + alpha_ob2_seg0[3] >= 1
 R31: - C1 + 1e+06 alpha_ob2_seg0[1] <= 999991.0228125
 R32: - C3 + 1e+06 alpha_ob2_seg0[1] <= 999991.0228125
 R33: alpha_ob2_seg0[0] + alpha_ob2_seg0[1] + alpha_ob2_seg0[2]
   + alpha_ob2_seg0[3] >= 1
 R34: C0 + 1e+06 alpha_ob2_seg0[2] <= 1.0000080000039999e+06
 R35: C2 + 1e+06 alpha_ob2_seg0[2] <= 1.0000080000039999e+06
 R36: alpha_ob2_seg0[0] + alpha_ob2_seg0[1] + alpha_ob2_seg0[2]
   + alpha_ob2_seg0[3] >= 1
 R37: C1 + 1e+06 alpha_ob2_seg0[3] <= 999991.0228125
 R38: C3 + 1e+06 alpha_ob2_seg0[3] <= 999991.0228125
 R39: alpha_ob2_seg0[0] + alpha_ob2_seg0[1] + alpha_ob2_seg0[2]
   + alpha_ob2_seg0[3] >= 1
 R40: C0 >= -11
 R41: C1 >= -11
 R42: C0 <= 25
 R43: C1 <= 25
 R44: C2 >= -11
 R45: C3 >= -11
 R46: C2 <= 25
 R47: C3 <= 25
Bounds
 Constant = 1
Binaries
 alpha_ob0_seg0[0] alpha_ob0_seg0[1] alpha_ob0_seg0[2] alpha_ob0_seg0[3]
 alpha_ob1_seg0[0] alpha_ob1_seg0[1] alpha_ob1_seg0[2] alpha_ob1_seg0[3]
 alpha_ob2_seg0[0] alpha_ob2_seg0[1] alpha_ob2_seg0[2] alpha_ob2_seg0[3]
End
