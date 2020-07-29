\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 60 C0 - 20 C1 - 60 C2 - 20 C3 - 60 C4 - 20 C5 - 60 C6 - 20 C7
   + 4000 Constant + [ 2 C0 ^2 + 2 C1 ^2 + 2 C2 ^2 + 2 C3 ^2 + 2 C4 ^2
   + 2 C5 ^2 + 2 C6 ^2 + 2 C7 ^2 ] / 2 
Subject To
 R0: C0 = 0
 R1: C1 = 1
 R2: C6 = 30
 R3: C7 = 10
 R4: - C0 + 1e+06 alpha_ob0_seg0[0] <= 999992.49102
 R5: - C2 + 1e+06 alpha_ob0_seg0[0] <= 999992.49102
 R6: - C1 + 1e+06 alpha_ob0_seg0[1] <= 999957.1016000001
 R7: - C3 + 1e+06 alpha_ob0_seg0[1] <= 999957.1016000001
 R8: C0 + 1e+06 alpha_ob0_seg0[2] <= 999986.35898
 R9: C2 + 1e+06 alpha_ob0_seg0[2] <= 999986.35898
 R10: C1 + 1e+06 alpha_ob0_seg0[3] <= 1.0000403484e+06
 R11: C3 + 1e+06 alpha_ob0_seg0[3] <= 1.0000403484e+06
 R12: alpha_ob0_seg0[0] + alpha_ob0_seg0[1] + alpha_ob0_seg0[2]
   + alpha_ob0_seg0[3] >= 1
 R13: - C0 + 1e+06 alpha_ob1_seg0[0] <= 999981.3909999999
 R14: - C2 + 1e+06 alpha_ob1_seg0[0] <= 999981.3909999999
 R15: - C1 + 1e+06 alpha_ob1_seg0[1] <= 999957.1016000001
 R16: - C3 + 1e+06 alpha_ob1_seg0[1] <= 999957.1016000001
 R17: C0 + 1e+06 alpha_ob1_seg0[2] <= 1.000004959e+06
 R18: C2 + 1e+06 alpha_ob1_seg0[2] <= 1.000004959e+06
 R19: C1 + 1e+06 alpha_ob1_seg0[3] <= 1.0000403484e+06
 R20: C3 + 1e+06 alpha_ob1_seg0[3] <= 1.0000403484e+06
 R21: alpha_ob1_seg0[0] + alpha_ob1_seg0[1] + alpha_ob1_seg0[2]
   + alpha_ob1_seg0[3] >= 1
 R22: - C0 + 1e+06 alpha_ob2_seg0[0] <= 999982.236799801
 R23: - C2 + 1e+06 alpha_ob2_seg0[0] <= 999982.236799801
 R24: - C1 + 1e+06 alpha_ob2_seg0[1] <= 999970.1095291895
 R25: - C3 + 1e+06 alpha_ob2_seg0[1] <= 999970.1095291895
 R26: C0 + 1e+06 alpha_ob2_seg0[2] <= 999993.3631998011
 R27: C2 + 1e+06 alpha_ob2_seg0[2] <= 999993.3631998011
 R28: C1 + 1e+06 alpha_ob2_seg0[3] <= 1.0000273405291894e+06
 R29: C3 + 1e+06 alpha_ob2_seg0[3] <= 1.0000273405291894e+06
 R30: alpha_ob2_seg0[0] + alpha_ob2_seg0[1] + alpha_ob2_seg0[2]
   + alpha_ob2_seg0[3] >= 1
 R31: - C0 + 1e+06 alpha_ob3_seg0[0] <= 1.0000040868238758e+06
 R32: - C2 + 1e+06 alpha_ob3_seg0[0] <= 1.0000040868238758e+06
 R33: - C1 + 1e+06 alpha_ob3_seg0[1] <= 999957.2594997246
 R34: - C3 + 1e+06 alpha_ob3_seg0[1] <= 999957.2594997246
 R35: C0 + 1e+06 alpha_ob3_seg0[2] <= 999993.3632238759
 R36: C2 + 1e+06 alpha_ob3_seg0[2] <= 999993.3632238759
 R37: C1 + 1e+06 alpha_ob3_seg0[3] <= 1.0000273404997246e+06
 R38: C3 + 1e+06 alpha_ob3_seg0[3] <= 1.0000273404997246e+06
 R39: alpha_ob3_seg0[0] + alpha_ob3_seg0[1] + alpha_ob3_seg0[2]
   + alpha_ob3_seg0[3] >= 1
 R40: - C0 + 1e+06 alpha_ob4_seg0[0] <= 999997.6803502565
 R41: - C2 + 1e+06 alpha_ob4_seg0[0] <= 999997.6803502565
 R42: - C1 + 1e+06 alpha_ob4_seg0[1] <= 999972.7268002755
 R43: - C3 + 1e+06 alpha_ob4_seg0[1] <= 999972.7268002755
 R44: C0 + 1e+06 alpha_ob4_seg0[2] <= 999999.7696102565
 R45: C2 + 1e+06 alpha_ob4_seg0[2] <= 999999.7696102565
 R46: C1 + 1e+06 alpha_ob4_seg0[3] <= 1.0000141232002755e+06
 R47: C3 + 1e+06 alpha_ob4_seg0[3] <= 1.0000141232002755e+06
 R48: alpha_ob4_seg0[0] + alpha_ob4_seg0[1] + alpha_ob4_seg0[2]
   + alpha_ob4_seg0[3] >= 1
 R49: - C0 + 1e+06 alpha_ob5_seg0[0] <= 1.0000038341502565e+06
 R50: - C2 + 1e+06 alpha_ob5_seg0[0] <= 1.0000038341502565e+06
 R51: - C1 + 1e+06 alpha_ob5_seg0[1] <= 999972.6012002755
 R52: - C3 + 1e+06 alpha_ob5_seg0[1] <= 999972.6012002755
 R53: C0 + 1e+06 alpha_ob5_seg0[2] <= 999993.6158102566
 R54: C2 + 1e+06 alpha_ob5_seg0[2] <= 999993.6158102566
 R55: C1 + 1e+06 alpha_ob5_seg0[3] <= 1.0000142488002755e+06
 R56: C3 + 1e+06 alpha_ob5_seg0[3] <= 1.0000142488002755e+06
 R57: alpha_ob5_seg0[0] + alpha_ob5_seg0[1] + alpha_ob5_seg0[2]
   + alpha_ob5_seg0[3] >= 1
 R58: - C0 + 1e+06 alpha_ob6_seg0[0] <= 1.0000066737e+06
 R59: - C2 + 1e+06 alpha_ob6_seg0[0] <= 1.0000066737e+06
 R60: - C1 + 1e+06 alpha_ob6_seg0[1] <= 999952.1958
 R61: - C3 + 1e+06 alpha_ob6_seg0[1] <= 999952.1958
 R62: C0 + 1e+06 alpha_ob6_seg0[2] <= 999948.4263000001
 R63: C2 + 1e+06 alpha_ob6_seg0[2] <= 999948.4263000001
 R64: C1 + 1e+06 alpha_ob6_seg0[3] <= 1.0000452542e+06
 R65: C3 + 1e+06 alpha_ob6_seg0[3] <= 1.0000452542e+06
 R66: alpha_ob6_seg0[0] + alpha_ob6_seg0[1] + alpha_ob6_seg0[2]
   + alpha_ob6_seg0[3] >= 1
 R67: - C0 + 1e+06 alpha_ob7_seg0[0] <= 1.0000070091e+06
 R68: - C2 + 1e+06 alpha_ob7_seg0[0] <= 1.0000070091e+06
 R69: - C1 + 1e+06 alpha_ob7_seg0[1] <= 1.0000598843e+06
 R70: - C3 + 1e+06 alpha_ob7_seg0[1] <= 1.0000598843e+06
 R71: C0 + 1e+06 alpha_ob7_seg0[2] <= 999965.0909
 R72: C2 + 1e+06 alpha_ob7_seg0[2] <= 999965.0909
 R73: C1 + 1e+06 alpha_ob7_seg0[3] <= 999937.5657
 R74: C3 + 1e+06 alpha_ob7_seg0[3] <= 999937.5657
 R75: alpha_ob7_seg0[0] + alpha_ob7_seg0[1] + alpha_ob7_seg0[2]
   + alpha_ob7_seg0[3] >= 1
 R76: - C0 + 1e+06 alpha_ob8_seg0[0] <= 1.000006992114031e+06
 R77: - C2 + 1e+06 alpha_ob8_seg0[0] <= 1.000006992114031e+06
 R78: - C1 + 1e+06 alpha_ob8_seg0[1] <= 1.0000588068002755e+06
 R79: - C3 + 1e+06 alpha_ob8_seg0[1] <= 1.0000588068002755e+06
 R80: C0 + 1e+06 alpha_ob8_seg0[2] <= 999990.457874031
 R81: C2 + 1e+06 alpha_ob8_seg0[2] <= 999990.457874031
 R82: C1 + 1e+06 alpha_ob8_seg0[3] <= 999935.5432002755
 R83: C3 + 1e+06 alpha_ob8_seg0[3] <= 999935.5432002755
 R84: alpha_ob8_seg0[0] + alpha_ob8_seg0[1] + alpha_ob8_seg0[2]
   + alpha_ob8_seg0[3] >= 1
 R85: - C0 + 1e+06 alpha_ob9_seg0[0] <= 1.0000033775063743e+06
 R86: - C2 + 1e+06 alpha_ob9_seg0[0] <= 1.0000033775063743e+06
 R87: - C1 + 1e+06 alpha_ob9_seg0[1] <= 1.0000348666997246e+06
 R88: - C3 + 1e+06 alpha_ob9_seg0[1] <= 1.0000348666997246e+06
 R89: C0 + 1e+06 alpha_ob9_seg0[2] <= 999994.0725863742
 R90: C2 + 1e+06 alpha_ob9_seg0[2] <= 999994.0725863742
 R91: C1 + 1e+06 alpha_ob9_seg0[3] <= 999937.4832997246
 R92: C3 + 1e+06 alpha_ob9_seg0[3] <= 999937.4832997246
 R93: alpha_ob9_seg0[0] + alpha_ob9_seg0[1] + alpha_ob9_seg0[2]
   + alpha_ob9_seg0[3] >= 1
 R94: - C0 + 1e+06 alpha_ob10_seg0[0] <= 999990.2670277582
 R95: - C2 + 1e+06 alpha_ob10_seg0[0] <= 999990.2670277582
 R96: - C1 + 1e+06 alpha_ob10_seg0[1] <= 1.0000347702002756e+06
 R97: - C3 + 1e+06 alpha_ob10_seg0[1] <= 1.0000347702002756e+06
 R98: C0 + 1e+06 alpha_ob10_seg0[2] <= 1.0000071828877581e+06
 R99: C2 + 1e+06 alpha_ob10_seg0[2] <= 1.0000071828877581e+06
 R100: C1 + 1e+06 alpha_ob10_seg0[3] <= 999939.8298002756
 R101: C3 + 1e+06 alpha_ob10_seg0[3] <= 999939.8298002756
 R102: alpha_ob10_seg0[0] + alpha_ob10_seg0[1] + alpha_ob10_seg0[2]
   + alpha_ob10_seg0[3] >= 1
 R103: - C0 + 1e+06 alpha_ob11_seg0[0] <= 999990.267069801
 R104: - C2 + 1e+06 alpha_ob11_seg0[0] <= 999990.267069801
 R105: - C1 + 1e+06 alpha_ob11_seg0[1] <= 1.0000576202069657e+06
 R106: - C3 + 1e+06 alpha_ob11_seg0[1] <= 1.0000576202069657e+06
 R107: C0 + 1e+06 alpha_ob11_seg0[2] <= 1.000002082929801e+06
 R108: C2 + 1e+06 alpha_ob11_seg0[2] <= 1.000002082929801e+06
 R109: C1 + 1e+06 alpha_ob11_seg0[3] <= 999939.8298069657
 R110: C3 + 1e+06 alpha_ob11_seg0[3] <= 999939.8298069657
 R111: alpha_ob11_seg0[0] + alpha_ob11_seg0[1] + alpha_ob11_seg0[2]
   + alpha_ob11_seg0[3] >= 1
 R112: - C0 + 1e+06 alpha_ob12_seg0[0] <= 999991.073732
 R113: - C2 + 1e+06 alpha_ob12_seg0[0] <= 999991.073732
 R114: - C1 + 1e+06 alpha_ob12_seg0[1] <= 999952.2125
 R115: - C3 + 1e+06 alpha_ob12_seg0[1] <= 999952.2125
 R116: C0 + 1e+06 alpha_ob12_seg0[2] <= 999990.776268
 R117: C2 + 1e+06 alpha_ob12_seg0[2] <= 999990.776268
 R118: C1 + 1e+06 alpha_ob12_seg0[3] <= 1.0000452375e+06
 R119: C3 + 1e+06 alpha_ob12_seg0[3] <= 1.0000452375e+06
 R120: alpha_ob12_seg0[0] + alpha_ob12_seg0[1] + alpha_ob12_seg0[2]
   + alpha_ob12_seg0[3] >= 1
 R121: - C0 + 1e+06 alpha_ob13_seg0[0] <= 999985.657099801
 R122: - C2 + 1e+06 alpha_ob13_seg0[0] <= 999985.657099801
 R123: - C1 + 1e+06 alpha_ob13_seg0[1] <= 1.0000576202063023e+06
 R124: - C3 + 1e+06 alpha_ob13_seg0[1] <= 1.0000576202063023e+06
 R125: C0 + 1e+06 alpha_ob13_seg0[2] <= 1.000007192899801e+06
 R126: C2 + 1e+06 alpha_ob13_seg0[2] <= 1.000007192899801e+06
 R127: C1 + 1e+06 alpha_ob13_seg0[3] <= 999939.8298063023
 R128: C3 + 1e+06 alpha_ob13_seg0[3] <= 999939.8298063023
 R129: alpha_ob13_seg0[0] + alpha_ob13_seg0[1] + alpha_ob13_seg0[2]
   + alpha_ob13_seg0[3] >= 1
 R130: - C0 + 1e+06 alpha_ob14_seg0[0] <= 999994.771829801
 R131: - C2 + 1e+06 alpha_ob14_seg0[0] <= 999994.771829801
 R132: - C1 + 1e+06 alpha_ob14_seg0[1] <= 1.0000615983063023e+06
 R133: - C3 + 1e+06 alpha_ob14_seg0[1] <= 1.0000615983063023e+06
 R134: C0 + 1e+06 alpha_ob14_seg0[2] <= 999998.078169801
 R135: C2 + 1e+06 alpha_ob14_seg0[2] <= 999998.078169801
 R136: C1 + 1e+06 alpha_ob14_seg0[3] <= 999935.8517063023
 R137: C3 + 1e+06 alpha_ob14_seg0[3] <= 999935.8517063023
 R138: alpha_ob14_seg0[0] + alpha_ob14_seg0[1] + alpha_ob14_seg0[2]
   + alpha_ob14_seg0[3] >= 1
 R139: - C0 + 1e+06 alpha_ob15_seg0[0] <= 999999.3718445294
 R140: - C2 + 1e+06 alpha_ob15_seg0[0] <= 999999.3718445294
 R141: - C1 + 1e+06 alpha_ob15_seg0[1] <= 1.0000527482997245e+06
 R142: - C3 + 1e+06 alpha_ob15_seg0[1] <= 1.0000527482997245e+06
 R143: C0 + 1e+06 alpha_ob15_seg0[2] <= 999998.0781885295
 R144: C2 + 1e+06 alpha_ob15_seg0[2] <= 999998.0781885295
 R145: C1 + 1e+06 alpha_ob15_seg0[3] <= 999935.8516997246
 R146: C3 + 1e+06 alpha_ob15_seg0[3] <= 999935.8516997246
 R147: alpha_ob15_seg0[0] + alpha_ob15_seg0[1] + alpha_ob15_seg0[2]
   + alpha_ob15_seg0[3] >= 1
 R148: - C0 + 1e+06 alpha_ob16_seg0[0] <= 999999.371829801
 R149: - C2 + 1e+06 alpha_ob16_seg0[0] <= 999999.371829801
 R150: - C1 + 1e+06 alpha_ob16_seg0[1] <= 1.0000527483059706e+06
 R151: - C3 + 1e+06 alpha_ob16_seg0[1] <= 1.0000527483059706e+06
 R152: C0 + 1e+06 alpha_ob16_seg0[2] <= 999993.7281698009
 R153: C2 + 1e+06 alpha_ob16_seg0[2] <= 999993.7281698009
 R154: C1 + 1e+06 alpha_ob16_seg0[3] <= 999944.7017059706
 R155: C3 + 1e+06 alpha_ob16_seg0[3] <= 999944.7017059706
 R156: alpha_ob16_seg0[0] + alpha_ob16_seg0[1] + alpha_ob16_seg0[2]
   + alpha_ob16_seg0[3] >= 1
 R157: - C0 + 1e+06 alpha_ob17_seg0[0] <= 1.0000016470956831e+06
 R158: - C2 + 1e+06 alpha_ob17_seg0[0] <= 1.0000016470956831e+06
 R159: - C1 + 1e+06 alpha_ob17_seg0[1] <= 1.0000598666742218e+06
 R160: - C3 + 1e+06 alpha_ob17_seg0[1] <= 1.0000598666742218e+06
 R161: C0 + 1e+06 alpha_ob17_seg0[2] <= 999994.0767956832
 R162: C2 + 1e+06 alpha_ob17_seg0[2] <= 999994.0767956832
 R163: C1 + 1e+06 alpha_ob17_seg0[3] <= 999937.4792742218
 R164: C3 + 1e+06 alpha_ob17_seg0[3] <= 999937.4792742218
 R165: alpha_ob17_seg0[0] + alpha_ob17_seg0[1] + alpha_ob17_seg0[2]
   + alpha_ob17_seg0[3] >= 1
 R166: - C0 + 1e+06 alpha_ob18_seg0[0] <= 999999.3718601406
 R167: - C2 + 1e+06 alpha_ob18_seg0[0] <= 999999.3718601406
 R168: - C1 + 1e+06 alpha_ob18_seg0[1] <= 1.0000353982997246e+06
 R169: - C3 + 1e+06 alpha_ob18_seg0[1] <= 1.0000353982997246e+06
 R170: C0 + 1e+06 alpha_ob18_seg0[2] <= 999998.0782041405
 R171: C2 + 1e+06 alpha_ob18_seg0[2] <= 999998.0782041405
 R172: C1 + 1e+06 alpha_ob18_seg0[3] <= 999944.7016997245
 R173: C3 + 1e+06 alpha_ob18_seg0[3] <= 999944.7016997245
 R174: alpha_ob18_seg0[0] + alpha_ob18_seg0[1] + alpha_ob18_seg0[2]
   + alpha_ob18_seg0[3] >= 1
 R175: - C0 + 1e+06 alpha_ob19_seg0[0] <= 999956.839399801
 R176: - C2 + 1e+06 alpha_ob19_seg0[0] <= 999956.839399801
 R177: - C1 + 1e+06 alpha_ob19_seg0[1] <= 999925.3963434525
 R178: - C3 + 1e+06 alpha_ob19_seg0[1] <= 999925.3963434525
 R179: C0 + 1e+06 alpha_ob19_seg0[2] <= 1.0000080105998011e+06
 R180: C2 + 1e+06 alpha_ob19_seg0[2] <= 1.0000080105998011e+06
 R181: C1 + 1e+06 alpha_ob19_seg0[3] <= 1.0000720537434525e+06
 R182: C3 + 1e+06 alpha_ob19_seg0[3] <= 1.0000720537434525e+06
 R183: alpha_ob19_seg0[0] + alpha_ob19_seg0[1] + alpha_ob19_seg0[2]
   + alpha_ob19_seg0[3] >= 1
 R184: - C0 + 1e+06 alpha_ob20_seg0[0] <= 999983.6053459151
 R185: - C2 + 1e+06 alpha_ob20_seg0[0] <= 999983.6053459151
 R186: - C1 + 1e+06 alpha_ob20_seg0[1] <= 999987.7722497246
 R187: - C3 + 1e+06 alpha_ob20_seg0[1] <= 999987.7722497246
 R188: C0 + 1e+06 alpha_ob20_seg0[2] <= 1.000013844745915e+06
 R189: C2 + 1e+06 alpha_ob20_seg0[2] <= 1.000013844745915e+06
 R190: C1 + 1e+06 alpha_ob20_seg0[3] <= 999984.8277497246
 R191: C3 + 1e+06 alpha_ob20_seg0[3] <= 999984.8277497246
 R192: alpha_ob20_seg0[0] + alpha_ob20_seg0[1] + alpha_ob20_seg0[2]
   + alpha_ob20_seg0[3] >= 1
 R193: - C0 + 1e+06 alpha_ob21_seg0[0] <= 999958.8717898987
 R194: - C2 + 1e+06 alpha_ob21_seg0[0] <= 999958.8717898987
 R195: - C1 + 1e+06 alpha_ob21_seg0[1] <= 999987.7837602755
 R196: - C3 + 1e+06 alpha_ob21_seg0[1] <= 999987.7837602755
 R197: C0 + 1e+06 alpha_ob21_seg0[2] <= 1.0000385781898986e+06
 R198: C2 + 1e+06 alpha_ob21_seg0[2] <= 1.0000385781898986e+06
 R199: C1 + 1e+06 alpha_ob21_seg0[3] <= 1.0000043162402755e+06
 R200: C3 + 1e+06 alpha_ob21_seg0[3] <= 1.0000043162402755e+06
 R201: alpha_ob21_seg0[0] + alpha_ob21_seg0[1] + alpha_ob21_seg0[2]
   + alpha_ob21_seg0[3] >= 1
 R202: - C0 + 1e+06 alpha_ob22_seg0[0] <= 999939.2718
 R203: - C2 + 1e+06 alpha_ob22_seg0[0] <= 999939.2718
 R204: - C1 + 1e+06 alpha_ob22_seg0[1] <= 999993.13376
 R205: - C3 + 1e+06 alpha_ob22_seg0[1] <= 999993.13376
 R206: C0 + 1e+06 alpha_ob22_seg0[2] <= 1.0000385782e+06
 R207: C2 + 1e+06 alpha_ob22_seg0[2] <= 1.0000385782e+06
 R208: C1 + 1e+06 alpha_ob22_seg0[3] <= 1.00000431624e+06
 R209: C3 + 1e+06 alpha_ob22_seg0[3] <= 1.00000431624e+06
 R210: alpha_ob22_seg0[0] + alpha_ob22_seg0[1] + alpha_ob22_seg0[2]
   + alpha_ob22_seg0[3] >= 1
 R211: - C0 + 1e+06 alpha_ob23_seg0[0] <= 999939.2718101013
 R212: - C2 + 1e+06 alpha_ob23_seg0[0] <= 999939.2718101013
 R213: - C1 + 1e+06 alpha_ob23_seg0[1] <= 999987.7837597246
 R214: - C3 + 1e+06 alpha_ob23_seg0[1] <= 999987.7837597246
 R215: C0 + 1e+06 alpha_ob23_seg0[2] <= 1.0000581782101013e+06
 R216: C2 + 1e+06 alpha_ob23_seg0[2] <= 1.0000581782101013e+06
 R217: C1 + 1e+06 alpha_ob23_seg0[3] <= 1.0000043162397245e+06
 R218: C3 + 1e+06 alpha_ob23_seg0[3] <= 1.0000043162397245e+06
 R219: alpha_ob23_seg0[0] + alpha_ob23_seg0[1] + alpha_ob23_seg0[2]
   + alpha_ob23_seg0[3] >= 1
 R220: - C0 + 1e+06 alpha_ob24_seg0[0] <= 999939.271799801
 R221: - C2 + 1e+06 alpha_ob24_seg0[0] <= 999939.271799801
 R222: - C1 + 1e+06 alpha_ob24_seg0[1] <= 999987.7838262042
 R223: - C3 + 1e+06 alpha_ob24_seg0[1] <= 999987.7838262042
 R224: C0 + 1e+06 alpha_ob24_seg0[2] <= 1.000038578199801e+06
 R225: C2 + 1e+06 alpha_ob24_seg0[2] <= 1.000038578199801e+06
 R226: C1 + 1e+06 alpha_ob24_seg0[3] <= 1.0000096662262043e+06
 R227: C3 + 1e+06 alpha_ob24_seg0[3] <= 1.0000096662262043e+06
 R228: alpha_ob24_seg0[0] + alpha_ob24_seg0[1] + alpha_ob24_seg0[2]
   + alpha_ob24_seg0[3] >= 1
 R229: - C0 + 1e+06 alpha_ob25_seg0[0] <= 999994.9164998011
 R230: - C2 + 1e+06 alpha_ob25_seg0[0] <= 999994.9164998011
 R231: - C1 + 1e+06 alpha_ob25_seg0[1] <= 999925.4284782809
 R232: - C3 + 1e+06 alpha_ob25_seg0[1] <= 999925.4284782809
 R233: C0 + 1e+06 alpha_ob25_seg0[2] <= 999943.6834998011
 R234: C2 + 1e+06 alpha_ob25_seg0[2] <= 999943.6834998011
 R235: C1 + 1e+06 alpha_ob25_seg0[3] <= 1.0000720216782809e+06
 R236: C3 + 1e+06 alpha_ob25_seg0[3] <= 1.0000720216782809e+06
 R237: alpha_ob25_seg0[0] + alpha_ob25_seg0[1] + alpha_ob25_seg0[2]
   + alpha_ob25_seg0[3] >= 1
 R238: - C0 + 1e+06 alpha_ob26_seg0[0] <= 999914.52327842
 R239: - C2 + 1e+06 alpha_ob26_seg0[0] <= 999914.52327842
 R240: - C1 + 1e+06 alpha_ob26_seg0[1] <= 999993.6552362755
 R241: - C3 + 1e+06 alpha_ob26_seg0[1] <= 999993.6552362755
 R242: C0 + 1e+06 alpha_ob26_seg0[2] <= 1.0000829266784199e+06
 R243: C2 + 1e+06 alpha_ob26_seg0[2] <= 1.0000829266784199e+06
 R244: C1 + 1e+06 alpha_ob26_seg0[3] <= 999992.1947642756
 R245: C3 + 1e+06 alpha_ob26_seg0[3] <= 999992.1947642756
 R246: alpha_ob26_seg0[0] + alpha_ob26_seg0[1] + alpha_ob26_seg0[2]
   + alpha_ob26_seg0[3] >= 1
 R247: - C0 + 1e+06 alpha_ob27_seg0[0] <= 999912.265199801
 R248: - C2 + 1e+06 alpha_ob27_seg0[0] <= 999912.265199801
 R249: - C1 + 1e+06 alpha_ob27_seg0[1] <= 999987.7842112777
 R250: - C3 + 1e+06 alpha_ob27_seg0[1] <= 999987.7842112777
 R251: C0 + 1e+06 alpha_ob27_seg0[2] <= 1.000076834799801e+06
 R252: C2 + 1e+06 alpha_ob27_seg0[2] <= 1.000076834799801e+06
 R253: C1 + 1e+06 alpha_ob27_seg0[3] <= 1.0000096658112778e+06
 R254: C3 + 1e+06 alpha_ob27_seg0[3] <= 1.0000096658112778e+06
 R255: alpha_ob27_seg0[0] + alpha_ob27_seg0[1] + alpha_ob27_seg0[2]
   + alpha_ob27_seg0[3] >= 1
 R256: - C0 + 1e+06 alpha_ob28_seg0[0] <= 999920.6152119379
 R257: - C2 + 1e+06 alpha_ob28_seg0[0] <= 999920.6152119379
 R258: - C1 + 1e+06 alpha_ob28_seg0[1] <= 999981.4341997246
 R259: - C3 + 1e+06 alpha_ob28_seg0[1] <= 999981.4341997246
 R260: C0 + 1e+06 alpha_ob28_seg0[2] <= 1.0000768348119379e+06
 R261: C2 + 1e+06 alpha_ob28_seg0[2] <= 1.0000768348119379e+06
 R262: C1 + 1e+06 alpha_ob28_seg0[3] <= 1.0000096657997245e+06
 R263: C3 + 1e+06 alpha_ob28_seg0[3] <= 1.0000096657997245e+06
 R264: alpha_ob28_seg0[0] + alpha_ob28_seg0[1] + alpha_ob28_seg0[2]
   + alpha_ob28_seg0[3] >= 1
 R265: - C0 + 1e+06 alpha_ob29_seg0[0] <= 1.0000081732920931e+06
 R266: - C2 + 1e+06 alpha_ob29_seg0[0] <= 1.0000081732920931e+06
 R267: - C1 + 1e+06 alpha_ob29_seg0[1] <= 999925.4331002756
 R268: - C3 + 1e+06 alpha_ob29_seg0[1] <= 999925.4331002756
 R269: C0 + 1e+06 alpha_ob29_seg0[2] <= 999989.2766720931
 R270: C2 + 1e+06 alpha_ob29_seg0[2] <= 999989.2766720931
 R271: C1 + 1e+06 alpha_ob29_seg0[3] <= 1.0000624169002755e+06
 R272: C3 + 1e+06 alpha_ob29_seg0[3] <= 1.0000624169002755e+06
 R273: alpha_ob29_seg0[0] + alpha_ob29_seg0[1] + alpha_ob29_seg0[2]
   + alpha_ob29_seg0[3] >= 1
 R274: - C0 + 1e+06 alpha_ob30_seg0[0] <= 1.0000058189e+06
 R275: - C2 + 1e+06 alpha_ob30_seg0[0] <= 1.0000058189e+06
 R276: - C1 + 1e+06 alpha_ob30_seg0[1] <= 999940.5469
 R277: - C3 + 1e+06 alpha_ob30_seg0[1] <= 999940.5469
 R278: C0 + 1e+06 alpha_ob30_seg0[2] <= 999985.5311
 R279: C2 + 1e+06 alpha_ob30_seg0[2] <= 999985.5311
 R280: C1 + 1e+06 alpha_ob30_seg0[3] <= 1.0000569031e+06
 R281: C3 + 1e+06 alpha_ob30_seg0[3] <= 1.0000569031e+06
 R282: alpha_ob30_seg0[0] + alpha_ob30_seg0[1] + alpha_ob30_seg0[2]
   + alpha_ob30_seg0[3] >= 1
 R283: - C0 + 1e+06 alpha_ob31_seg0[0] <= 1.0000058188284199e+06
 R284: - C2 + 1e+06 alpha_ob31_seg0[0] <= 1.0000058188284199e+06
 R285: - C1 + 1e+06 alpha_ob31_seg0[1] <= 999940.5469002755
 R286: - C3 + 1e+06 alpha_ob31_seg0[1] <= 999940.5469002755
 R287: C0 + 1e+06 alpha_ob31_seg0[2] <= 999991.6311284199
 R288: C2 + 1e+06 alpha_ob31_seg0[2] <= 999991.6311284199
 R289: C1 + 1e+06 alpha_ob31_seg0[3] <= 1.0000453031002756e+06
 R290: C3 + 1e+06 alpha_ob31_seg0[3] <= 1.0000453031002756e+06
 R291: alpha_ob31_seg0[0] + alpha_ob31_seg0[1] + alpha_ob31_seg0[2]
   + alpha_ob31_seg0[3] >= 1
 R292: - C0 + 1e+06 alpha_ob32_seg0[0] <= 999996.9688499999
 R293: - C2 + 1e+06 alpha_ob32_seg0[0] <= 999996.9688499999
 R294: - C1 + 1e+06 alpha_ob32_seg0[1] <= 999940.5469
 R295: - C3 + 1e+06 alpha_ob32_seg0[1] <= 999940.5469
 R296: C0 + 1e+06 alpha_ob32_seg0[2] <= 999991.63115
 R297: C2 + 1e+06 alpha_ob32_seg0[2] <= 999991.63115
 R298: C1 + 1e+06 alpha_ob32_seg0[3] <= 1.0000569031e+06
 R299: C3 + 1e+06 alpha_ob32_seg0[3] <= 1.0000569031e+06
 R300: alpha_ob32_seg0[0] + alpha_ob32_seg0[1] + alpha_ob32_seg0[2]
   + alpha_ob32_seg0[3] >= 1
 R301: - C0 + 1e+06 alpha_ob33_seg0[0] <= 999991.83683
 R302: - C2 + 1e+06 alpha_ob33_seg0[0] <= 999991.83683
 R303: - C1 + 1e+06 alpha_ob33_seg0[1] <= 999940.589
 R304: - C3 + 1e+06 alpha_ob33_seg0[1] <= 999940.589
 R305: C0 + 1e+06 alpha_ob33_seg0[2] <= 1.00000401317e+06
 R306: C2 + 1e+06 alpha_ob33_seg0[2] <= 1.00000401317e+06
 R307: C1 + 1e+06 alpha_ob33_seg0[3] <= 1.000056861e+06
 R308: C3 + 1e+06 alpha_ob33_seg0[3] <= 1.000056861e+06
 R309: alpha_ob33_seg0[0] + alpha_ob33_seg0[1] + alpha_ob33_seg0[2]
   + alpha_ob33_seg0[3] >= 1
 R310: - C0 + 1e+06 alpha_ob34_seg0[0] <= 999991.8368084199
 R311: - C2 + 1e+06 alpha_ob34_seg0[0] <= 999991.8368084199
 R312: - C1 + 1e+06 alpha_ob34_seg0[1] <= 999940.5890002755
 R313: - C3 + 1e+06 alpha_ob34_seg0[1] <= 999940.5890002755
 R314: C0 + 1e+06 alpha_ob34_seg0[2] <= 1.0000056131484199e+06
 R315: C2 + 1e+06 alpha_ob34_seg0[2] <= 1.0000056131484199e+06
 R316: C1 + 1e+06 alpha_ob34_seg0[3] <= 1.0000452610002755e+06
 R317: C3 + 1e+06 alpha_ob34_seg0[3] <= 1.0000452610002755e+06
 R318: alpha_ob34_seg0[0] + alpha_ob34_seg0[1] + alpha_ob34_seg0[2]
   + alpha_ob34_seg0[3] >= 1
 R319: - C2 + 1e+06 alpha_ob0_seg1[0] <= 999992.49102
 R320: - C4 + 1e+06 alpha_ob0_seg1[0] <= 999992.49102
 R321: - C3 + 1e+06 alpha_ob0_seg1[1] <= 999957.1016000001
 R322: - C5 + 1e+06 alpha_ob0_seg1[1] <= 999957.1016000001
 R323: C2 + 1e+06 alpha_ob0_seg1[2] <= 999986.35898
 R324: C4 + 1e+06 alpha_ob0_seg1[2] <= 999986.35898
 R325: C3 + 1e+06 alpha_ob0_seg1[3] <= 1.0000403484e+06
 R326: C5 + 1e+06 alpha_ob0_seg1[3] <= 1.0000403484e+06
 R327: alpha_ob0_seg1[0] + alpha_ob0_seg1[1] + alpha_ob0_seg1[2]
   + alpha_ob0_seg1[3] >= 1
 R328: - C2 + 1e+06 alpha_ob1_seg1[0] <= 999981.3909999999
 R329: - C4 + 1e+06 alpha_ob1_seg1[0] <= 999981.3909999999
 R330: - C3 + 1e+06 alpha_ob1_seg1[1] <= 999957.1016000001
 R331: - C5 + 1e+06 alpha_ob1_seg1[1] <= 999957.1016000001
 R332: C2 + 1e+06 alpha_ob1_seg1[2] <= 1.000004959e+06
 R333: C4 + 1e+06 alpha_ob1_seg1[2] <= 1.000004959e+06
 R334: C3 + 1e+06 alpha_ob1_seg1[3] <= 1.0000403484e+06
 R335: C5 + 1e+06 alpha_ob1_seg1[3] <= 1.0000403484e+06
 R336: alpha_ob1_seg1[0] + alpha_ob1_seg1[1] + alpha_ob1_seg1[2]
   + alpha_ob1_seg1[3] >= 1
 R337: - C2 + 1e+06 alpha_ob2_seg1[0] <= 999982.236799801
 R338: - C4 + 1e+06 alpha_ob2_seg1[0] <= 999982.236799801
 R339: - C3 + 1e+06 alpha_ob2_seg1[1] <= 999970.1095291895
 R340: - C5 + 1e+06 alpha_ob2_seg1[1] <= 999970.1095291895
 R341: C2 + 1e+06 alpha_ob2_seg1[2] <= 999993.3631998011
 R342: C4 + 1e+06 alpha_ob2_seg1[2] <= 999993.3631998011
 R343: C3 + 1e+06 alpha_ob2_seg1[3] <= 1.0000273405291894e+06
 R344: C5 + 1e+06 alpha_ob2_seg1[3] <= 1.0000273405291894e+06
 R345: alpha_ob2_seg1[0] + alpha_ob2_seg1[1] + alpha_ob2_seg1[2]
   + alpha_ob2_seg1[3] >= 1
 R346: - C2 + 1e+06 alpha_ob3_seg1[0] <= 1.0000040868238758e+06
 R347: - C4 + 1e+06 alpha_ob3_seg1[0] <= 1.0000040868238758e+06
 R348: - C3 + 1e+06 alpha_ob3_seg1[1] <= 999957.2594997246
 R349: - C5 + 1e+06 alpha_ob3_seg1[1] <= 999957.2594997246
 R350: C2 + 1e+06 alpha_ob3_seg1[2] <= 999993.3632238759
 R351: C4 + 1e+06 alpha_ob3_seg1[2] <= 999993.3632238759
 R352: C3 + 1e+06 alpha_ob3_seg1[3] <= 1.0000273404997246e+06
 R353: C5 + 1e+06 alpha_ob3_seg1[3] <= 1.0000273404997246e+06
 R354: alpha_ob3_seg1[0] + alpha_ob3_seg1[1] + alpha_ob3_seg1[2]
   + alpha_ob3_seg1[3] >= 1
 R355: - C2 + 1e+06 alpha_ob4_seg1[0] <= 999997.6803502565
 R356: - C4 + 1e+06 alpha_ob4_seg1[0] <= 999997.6803502565
 R357: - C3 + 1e+06 alpha_ob4_seg1[1] <= 999972.7268002755
 R358: - C5 + 1e+06 alpha_ob4_seg1[1] <= 999972.7268002755
 R359: C2 + 1e+06 alpha_ob4_seg1[2] <= 999999.7696102565
 R360: C4 + 1e+06 alpha_ob4_seg1[2] <= 999999.7696102565
 R361: C3 + 1e+06 alpha_ob4_seg1[3] <= 1.0000141232002755e+06
 R362: C5 + 1e+06 alpha_ob4_seg1[3] <= 1.0000141232002755e+06
 R363: alpha_ob4_seg1[0] + alpha_ob4_seg1[1] + alpha_ob4_seg1[2]
   + alpha_ob4_seg1[3] >= 1
 R364: - C2 + 1e+06 alpha_ob5_seg1[0] <= 1.0000038341502565e+06
 R365: - C4 + 1e+06 alpha_ob5_seg1[0] <= 1.0000038341502565e+06
 R366: - C3 + 1e+06 alpha_ob5_seg1[1] <= 999972.6012002755
 R367: - C5 + 1e+06 alpha_ob5_seg1[1] <= 999972.6012002755
 R368: C2 + 1e+06 alpha_ob5_seg1[2] <= 999993.6158102566
 R369: C4 + 1e+06 alpha_ob5_seg1[2] <= 999993.6158102566
 R370: C3 + 1e+06 alpha_ob5_seg1[3] <= 1.0000142488002755e+06
 R371: C5 + 1e+06 alpha_ob5_seg1[3] <= 1.0000142488002755e+06
 R372: alpha_ob5_seg1[0] + alpha_ob5_seg1[1] + alpha_ob5_seg1[2]
   + alpha_ob5_seg1[3] >= 1
 R373: - C2 + 1e+06 alpha_ob6_seg1[0] <= 1.0000066737e+06
 R374: - C4 + 1e+06 alpha_ob6_seg1[0] <= 1.0000066737e+06
 R375: - C3 + 1e+06 alpha_ob6_seg1[1] <= 999952.1958
 R376: - C5 + 1e+06 alpha_ob6_seg1[1] <= 999952.1958
 R377: C2 + 1e+06 alpha_ob6_seg1[2] <= 999948.4263000001
 R378: C4 + 1e+06 alpha_ob6_seg1[2] <= 999948.4263000001
 R379: C3 + 1e+06 alpha_ob6_seg1[3] <= 1.0000452542e+06
 R380: C5 + 1e+06 alpha_ob6_seg1[3] <= 1.0000452542e+06
 R381: alpha_ob6_seg1[0] + alpha_ob6_seg1[1] + alpha_ob6_seg1[2]
   + alpha_ob6_seg1[3] >= 1
 R382: - C2 + 1e+06 alpha_ob7_seg1[0] <= 1.0000070091e+06
 R383: - C4 + 1e+06 alpha_ob7_seg1[0] <= 1.0000070091e+06
 R384: - C3 + 1e+06 alpha_ob7_seg1[1] <= 1.0000598843e+06
 R385: - C5 + 1e+06 alpha_ob7_seg1[1] <= 1.0000598843e+06
 R386: C2 + 1e+06 alpha_ob7_seg1[2] <= 999965.0909
 R387: C4 + 1e+06 alpha_ob7_seg1[2] <= 999965.0909
 R388: C3 + 1e+06 alpha_ob7_seg1[3] <= 999937.5657
 R389: C5 + 1e+06 alpha_ob7_seg1[3] <= 999937.5657
 R390: alpha_ob7_seg1[0] + alpha_ob7_seg1[1] + alpha_ob7_seg1[2]
   + alpha_ob7_seg1[3] >= 1
 R391: - C2 + 1e+06 alpha_ob8_seg1[0] <= 1.000006992114031e+06
 R392: - C4 + 1e+06 alpha_ob8_seg1[0] <= 1.000006992114031e+06
 R393: - C3 + 1e+06 alpha_ob8_seg1[1] <= 1.0000588068002755e+06
 R394: - C5 + 1e+06 alpha_ob8_seg1[1] <= 1.0000588068002755e+06
 R395: C2 + 1e+06 alpha_ob8_seg1[2] <= 999990.457874031
 R396: C4 + 1e+06 alpha_ob8_seg1[2] <= 999990.457874031
 R397: C3 + 1e+06 alpha_ob8_seg1[3] <= 999935.5432002755
 R398: C5 + 1e+06 alpha_ob8_seg1[3] <= 999935.5432002755
 R399: alpha_ob8_seg1[0] + alpha_ob8_seg1[1] + alpha_ob8_seg1[2]
   + alpha_ob8_seg1[3] >= 1
 R400: - C2 + 1e+06 alpha_ob9_seg1[0] <= 1.0000033775063743e+06
 R401: - C4 + 1e+06 alpha_ob9_seg1[0] <= 1.0000033775063743e+06
 R402: - C3 + 1e+06 alpha_ob9_seg1[1] <= 1.0000348666997246e+06
 R403: - C5 + 1e+06 alpha_ob9_seg1[1] <= 1.0000348666997246e+06
 R404: C2 + 1e+06 alpha_ob9_seg1[2] <= 999994.0725863742
 R405: C4 + 1e+06 alpha_ob9_seg1[2] <= 999994.0725863742
 R406: C3 + 1e+06 alpha_ob9_seg1[3] <= 999937.4832997246
 R407: C5 + 1e+06 alpha_ob9_seg1[3] <= 999937.4832997246
 R408: alpha_ob9_seg1[0] + alpha_ob9_seg1[1] + alpha_ob9_seg1[2]
   + alpha_ob9_seg1[3] >= 1
 R409: - C2 + 1e+06 alpha_ob10_seg1[0] <= 999990.2670277582
 R410: - C4 + 1e+06 alpha_ob10_seg1[0] <= 999990.2670277582
 R411: - C3 + 1e+06 alpha_ob10_seg1[1] <= 1.0000347702002756e+06
 R412: - C5 + 1e+06 alpha_ob10_seg1[1] <= 1.0000347702002756e+06
 R413: C2 + 1e+06 alpha_ob10_seg1[2] <= 1.0000071828877581e+06
 R414: C4 + 1e+06 alpha_ob10_seg1[2] <= 1.0000071828877581e+06
 R415: C3 + 1e+06 alpha_ob10_seg1[3] <= 999939.8298002756
 R416: C5 + 1e+06 alpha_ob10_seg1[3] <= 999939.8298002756
 R417: alpha_ob10_seg1[0] + alpha_ob10_seg1[1] + alpha_ob10_seg1[2]
   + alpha_ob10_seg1[3] >= 1
 R418: - C2 + 1e+06 alpha_ob11_seg1[0] <= 999990.267069801
 R419: - C4 + 1e+06 alpha_ob11_seg1[0] <= 999990.267069801
 R420: - C3 + 1e+06 alpha_ob11_seg1[1] <= 1.0000576202069657e+06
 R421: - C5 + 1e+06 alpha_ob11_seg1[1] <= 1.0000576202069657e+06
 R422: C2 + 1e+06 alpha_ob11_seg1[2] <= 1.000002082929801e+06
 R423: C4 + 1e+06 alpha_ob11_seg1[2] <= 1.000002082929801e+06
 R424: C3 + 1e+06 alpha_ob11_seg1[3] <= 999939.8298069657
 R425: C5 + 1e+06 alpha_ob11_seg1[3] <= 999939.8298069657
 R426: alpha_ob11_seg1[0] + alpha_ob11_seg1[1] + alpha_ob11_seg1[2]
   + alpha_ob11_seg1[3] >= 1
 R427: - C2 + 1e+06 alpha_ob12_seg1[0] <= 999991.073732
 R428: - C4 + 1e+06 alpha_ob12_seg1[0] <= 999991.073732
 R429: - C3 + 1e+06 alpha_ob12_seg1[1] <= 999952.2125
 R430: - C5 + 1e+06 alpha_ob12_seg1[1] <= 999952.2125
 R431: C2 + 1e+06 alpha_ob12_seg1[2] <= 999990.776268
 R432: C4 + 1e+06 alpha_ob12_seg1[2] <= 999990.776268
 R433: C3 + 1e+06 alpha_ob12_seg1[3] <= 1.0000452375e+06
 R434: C5 + 1e+06 alpha_ob12_seg1[3] <= 1.0000452375e+06
 R435: alpha_ob12_seg1[0] + alpha_ob12_seg1[1] + alpha_ob12_seg1[2]
   + alpha_ob12_seg1[3] >= 1
 R436: - C2 + 1e+06 alpha_ob13_seg1[0] <= 999985.657099801
 R437: - C4 + 1e+06 alpha_ob13_seg1[0] <= 999985.657099801
 R438: - C3 + 1e+06 alpha_ob13_seg1[1] <= 1.0000576202063023e+06
 R439: - C5 + 1e+06 alpha_ob13_seg1[1] <= 1.0000576202063023e+06
 R440: C2 + 1e+06 alpha_ob13_seg1[2] <= 1.000007192899801e+06
 R441: C4 + 1e+06 alpha_ob13_seg1[2] <= 1.000007192899801e+06
 R442: C3 + 1e+06 alpha_ob13_seg1[3] <= 999939.8298063023
 R443: C5 + 1e+06 alpha_ob13_seg1[3] <= 999939.8298063023
 R444: alpha_ob13_seg1[0] + alpha_ob13_seg1[1] + alpha_ob13_seg1[2]
   + alpha_ob13_seg1[3] >= 1
 R445: - C2 + 1e+06 alpha_ob14_seg1[0] <= 999994.771829801
 R446: - C4 + 1e+06 alpha_ob14_seg1[0] <= 999994.771829801
 R447: - C3 + 1e+06 alpha_ob14_seg1[1] <= 1.0000615983063023e+06
 R448: - C5 + 1e+06 alpha_ob14_seg1[1] <= 1.0000615983063023e+06
 R449: C2 + 1e+06 alpha_ob14_seg1[2] <= 999998.078169801
 R450: C4 + 1e+06 alpha_ob14_seg1[2] <= 999998.078169801
 R451: C3 + 1e+06 alpha_ob14_seg1[3] <= 999935.8517063023
 R452: C5 + 1e+06 alpha_ob14_seg1[3] <= 999935.8517063023
 R453: alpha_ob14_seg1[0] + alpha_ob14_seg1[1] + alpha_ob14_seg1[2]
   + alpha_ob14_seg1[3] >= 1
 R454: - C2 + 1e+06 alpha_ob15_seg1[0] <= 999999.3718445294
 R455: - C4 + 1e+06 alpha_ob15_seg1[0] <= 999999.3718445294
 R456: - C3 + 1e+06 alpha_ob15_seg1[1] <= 1.0000527482997245e+06
 R457: - C5 + 1e+06 alpha_ob15_seg1[1] <= 1.0000527482997245e+06
 R458: C2 + 1e+06 alpha_ob15_seg1[2] <= 999998.0781885295
 R459: C4 + 1e+06 alpha_ob15_seg1[2] <= 999998.0781885295
 R460: C3 + 1e+06 alpha_ob15_seg1[3] <= 999935.8516997246
 R461: C5 + 1e+06 alpha_ob15_seg1[3] <= 999935.8516997246
 R462: alpha_ob15_seg1[0] + alpha_ob15_seg1[1] + alpha_ob15_seg1[2]
   + alpha_ob15_seg1[3] >= 1
 R463: - C2 + 1e+06 alpha_ob16_seg1[0] <= 999999.371829801
 R464: - C4 + 1e+06 alpha_ob16_seg1[0] <= 999999.371829801
 R465: - C3 + 1e+06 alpha_ob16_seg1[1] <= 1.0000527483059706e+06
 R466: - C5 + 1e+06 alpha_ob16_seg1[1] <= 1.0000527483059706e+06
 R467: C2 + 1e+06 alpha_ob16_seg1[2] <= 999993.7281698009
 R468: C4 + 1e+06 alpha_ob16_seg1[2] <= 999993.7281698009
 R469: C3 + 1e+06 alpha_ob16_seg1[3] <= 999944.7017059706
 R470: C5 + 1e+06 alpha_ob16_seg1[3] <= 999944.7017059706
 R471: alpha_ob16_seg1[0] + alpha_ob16_seg1[1] + alpha_ob16_seg1[2]
   + alpha_ob16_seg1[3] >= 1
 R472: - C2 + 1e+06 alpha_ob17_seg1[0] <= 1.0000016470956831e+06
 R473: - C4 + 1e+06 alpha_ob17_seg1[0] <= 1.0000016470956831e+06
 R474: - C3 + 1e+06 alpha_ob17_seg1[1] <= 1.0000598666742218e+06
 R475: - C5 + 1e+06 alpha_ob17_seg1[1] <= 1.0000598666742218e+06
 R476: C2 + 1e+06 alpha_ob17_seg1[2] <= 999994.0767956832
 R477: C4 + 1e+06 alpha_ob17_seg1[2] <= 999994.0767956832
 R478: C3 + 1e+06 alpha_ob17_seg1[3] <= 999937.4792742218
 R479: C5 + 1e+06 alpha_ob17_seg1[3] <= 999937.4792742218
 R480: alpha_ob17_seg1[0] + alpha_ob17_seg1[1] + alpha_ob17_seg1[2]
   + alpha_ob17_seg1[3] >= 1
 R481: - C2 + 1e+06 alpha_ob18_seg1[0] <= 999999.3718601406
 R482: - C4 + 1e+06 alpha_ob18_seg1[0] <= 999999.3718601406
 R483: - C3 + 1e+06 alpha_ob18_seg1[1] <= 1.0000353982997246e+06
 R484: - C5 + 1e+06 alpha_ob18_seg1[1] <= 1.0000353982997246e+06
 R485: C2 + 1e+06 alpha_ob18_seg1[2] <= 999998.0782041405
 R486: C4 + 1e+06 alpha_ob18_seg1[2] <= 999998.0782041405
 R487: C3 + 1e+06 alpha_ob18_seg1[3] <= 999944.7016997245
 R488: C5 + 1e+06 alpha_ob18_seg1[3] <= 999944.7016997245
 R489: alpha_ob18_seg1[0] + alpha_ob18_seg1[1] + alpha_ob18_seg1[2]
   + alpha_ob18_seg1[3] >= 1
 R490: - C2 + 1e+06 alpha_ob19_seg1[0] <= 999956.839399801
 R491: - C4 + 1e+06 alpha_ob19_seg1[0] <= 999956.839399801
 R492: - C3 + 1e+06 alpha_ob19_seg1[1] <= 999925.3963434525
 R493: - C5 + 1e+06 alpha_ob19_seg1[1] <= 999925.3963434525
 R494: C2 + 1e+06 alpha_ob19_seg1[2] <= 1.0000080105998011e+06
 R495: C4 + 1e+06 alpha_ob19_seg1[2] <= 1.0000080105998011e+06
 R496: C3 + 1e+06 alpha_ob19_seg1[3] <= 1.0000720537434525e+06
 R497: C5 + 1e+06 alpha_ob19_seg1[3] <= 1.0000720537434525e+06
 R498: alpha_ob19_seg1[0] + alpha_ob19_seg1[1] + alpha_ob19_seg1[2]
   + alpha_ob19_seg1[3] >= 1
 R499: - C2 + 1e+06 alpha_ob20_seg1[0] <= 999983.6053459151
 R500: - C4 + 1e+06 alpha_ob20_seg1[0] <= 999983.6053459151
 R501: - C3 + 1e+06 alpha_ob20_seg1[1] <= 999987.7722497246
 R502: - C5 + 1e+06 alpha_ob20_seg1[1] <= 999987.7722497246
 R503: C2 + 1e+06 alpha_ob20_seg1[2] <= 1.000013844745915e+06
 R504: C4 + 1e+06 alpha_ob20_seg1[2] <= 1.000013844745915e+06
 R505: C3 + 1e+06 alpha_ob20_seg1[3] <= 999984.8277497246
 R506: C5 + 1e+06 alpha_ob20_seg1[3] <= 999984.8277497246
 R507: alpha_ob20_seg1[0] + alpha_ob20_seg1[1] + alpha_ob20_seg1[2]
   + alpha_ob20_seg1[3] >= 1
 R508: - C2 + 1e+06 alpha_ob21_seg1[0] <= 999958.8717898987
 R509: - C4 + 1e+06 alpha_ob21_seg1[0] <= 999958.8717898987
 R510: - C3 + 1e+06 alpha_ob21_seg1[1] <= 999987.7837602755
 R511: - C5 + 1e+06 alpha_ob21_seg1[1] <= 999987.7837602755
 R512: C2 + 1e+06 alpha_ob21_seg1[2] <= 1.0000385781898986e+06
 R513: C4 + 1e+06 alpha_ob21_seg1[2] <= 1.0000385781898986e+06
 R514: C3 + 1e+06 alpha_ob21_seg1[3] <= 1.0000043162402755e+06
 R515: C5 + 1e+06 alpha_ob21_seg1[3] <= 1.0000043162402755e+06
 R516: alpha_ob21_seg1[0] + alpha_ob21_seg1[1] + alpha_ob21_seg1[2]
   + alpha_ob21_seg1[3] >= 1
 R517: - C2 + 1e+06 alpha_ob22_seg1[0] <= 999939.2718
 R518: - C4 + 1e+06 alpha_ob22_seg1[0] <= 999939.2718
 R519: - C3 + 1e+06 alpha_ob22_seg1[1] <= 999993.13376
 R520: - C5 + 1e+06 alpha_ob22_seg1[1] <= 999993.13376
 R521: C2 + 1e+06 alpha_ob22_seg1[2] <= 1.0000385782e+06
 R522: C4 + 1e+06 alpha_ob22_seg1[2] <= 1.0000385782e+06
 R523: C3 + 1e+06 alpha_ob22_seg1[3] <= 1.00000431624e+06
 R524: C5 + 1e+06 alpha_ob22_seg1[3] <= 1.00000431624e+06
 R525: alpha_ob22_seg1[0] + alpha_ob22_seg1[1] + alpha_ob22_seg1[2]
   + alpha_ob22_seg1[3] >= 1
 R526: - C2 + 1e+06 alpha_ob23_seg1[0] <= 999939.2718101013
 R527: - C4 + 1e+06 alpha_ob23_seg1[0] <= 999939.2718101013
 R528: - C3 + 1e+06 alpha_ob23_seg1[1] <= 999987.7837597246
 R529: - C5 + 1e+06 alpha_ob23_seg1[1] <= 999987.7837597246
 R530: C2 + 1e+06 alpha_ob23_seg1[2] <= 1.0000581782101013e+06
 R531: C4 + 1e+06 alpha_ob23_seg1[2] <= 1.0000581782101013e+06
 R532: C3 + 1e+06 alpha_ob23_seg1[3] <= 1.0000043162397245e+06
 R533: C5 + 1e+06 alpha_ob23_seg1[3] <= 1.0000043162397245e+06
 R534: alpha_ob23_seg1[0] + alpha_ob23_seg1[1] + alpha_ob23_seg1[2]
   + alpha_ob23_seg1[3] >= 1
 R535: - C2 + 1e+06 alpha_ob24_seg1[0] <= 999939.271799801
 R536: - C4 + 1e+06 alpha_ob24_seg1[0] <= 999939.271799801
 R537: - C3 + 1e+06 alpha_ob24_seg1[1] <= 999987.7838262042
 R538: - C5 + 1e+06 alpha_ob24_seg1[1] <= 999987.7838262042
 R539: C2 + 1e+06 alpha_ob24_seg1[2] <= 1.000038578199801e+06
 R540: C4 + 1e+06 alpha_ob24_seg1[2] <= 1.000038578199801e+06
 R541: C3 + 1e+06 alpha_ob24_seg1[3] <= 1.0000096662262043e+06
 R542: C5 + 1e+06 alpha_ob24_seg1[3] <= 1.0000096662262043e+06
 R543: alpha_ob24_seg1[0] + alpha_ob24_seg1[1] + alpha_ob24_seg1[2]
   + alpha_ob24_seg1[3] >= 1
 R544: - C2 + 1e+06 alpha_ob25_seg1[0] <= 999994.9164998011
 R545: - C4 + 1e+06 alpha_ob25_seg1[0] <= 999994.9164998011
 R546: - C3 + 1e+06 alpha_ob25_seg1[1] <= 999925.4284782809
 R547: - C5 + 1e+06 alpha_ob25_seg1[1] <= 999925.4284782809
 R548: C2 + 1e+06 alpha_ob25_seg1[2] <= 999943.6834998011
 R549: C4 + 1e+06 alpha_ob25_seg1[2] <= 999943.6834998011
 R550: C3 + 1e+06 alpha_ob25_seg1[3] <= 1.0000720216782809e+06
 R551: C5 + 1e+06 alpha_ob25_seg1[3] <= 1.0000720216782809e+06
 R552: alpha_ob25_seg1[0] + alpha_ob25_seg1[1] + alpha_ob25_seg1[2]
   + alpha_ob25_seg1[3] >= 1
 R553: - C2 + 1e+06 alpha_ob26_seg1[0] <= 999914.52327842
 R554: - C4 + 1e+06 alpha_ob26_seg1[0] <= 999914.52327842
 R555: - C3 + 1e+06 alpha_ob26_seg1[1] <= 999993.6552362755
 R556: - C5 + 1e+06 alpha_ob26_seg1[1] <= 999993.6552362755
 R557: C2 + 1e+06 alpha_ob26_seg1[2] <= 1.0000829266784199e+06
 R558: C4 + 1e+06 alpha_ob26_seg1[2] <= 1.0000829266784199e+06
 R559: C3 + 1e+06 alpha_ob26_seg1[3] <= 999992.1947642756
 R560: C5 + 1e+06 alpha_ob26_seg1[3] <= 999992.1947642756
 R561: alpha_ob26_seg1[0] + alpha_ob26_seg1[1] + alpha_ob26_seg1[2]
   + alpha_ob26_seg1[3] >= 1
 R562: - C2 + 1e+06 alpha_ob27_seg1[0] <= 999912.265199801
 R563: - C4 + 1e+06 alpha_ob27_seg1[0] <= 999912.265199801
 R564: - C3 + 1e+06 alpha_ob27_seg1[1] <= 999987.7842112777
 R565: - C5 + 1e+06 alpha_ob27_seg1[1] <= 999987.7842112777
 R566: C2 + 1e+06 alpha_ob27_seg1[2] <= 1.000076834799801e+06
 R567: C4 + 1e+06 alpha_ob27_seg1[2] <= 1.000076834799801e+06
 R568: C3 + 1e+06 alpha_ob27_seg1[3] <= 1.0000096658112778e+06
 R569: C5 + 1e+06 alpha_ob27_seg1[3] <= 1.0000096658112778e+06
 R570: alpha_ob27_seg1[0] + alpha_ob27_seg1[1] + alpha_ob27_seg1[2]
   + alpha_ob27_seg1[3] >= 1
 R571: - C2 + 1e+06 alpha_ob28_seg1[0] <= 999920.6152119379
 R572: - C4 + 1e+06 alpha_ob28_seg1[0] <= 999920.6152119379
 R573: - C3 + 1e+06 alpha_ob28_seg1[1] <= 999981.4341997246
 R574: - C5 + 1e+06 alpha_ob28_seg1[1] <= 999981.4341997246
 R575: C2 + 1e+06 alpha_ob28_seg1[2] <= 1.0000768348119379e+06
 R576: C4 + 1e+06 alpha_ob28_seg1[2] <= 1.0000768348119379e+06
 R577: C3 + 1e+06 alpha_ob28_seg1[3] <= 1.0000096657997245e+06
 R578: C5 + 1e+06 alpha_ob28_seg1[3] <= 1.0000096657997245e+06
 R579: alpha_ob28_seg1[0] + alpha_ob28_seg1[1] + alpha_ob28_seg1[2]
   + alpha_ob28_seg1[3] >= 1
 R580: - C2 + 1e+06 alpha_ob29_seg1[0] <= 1.0000081732920931e+06
 R581: - C4 + 1e+06 alpha_ob29_seg1[0] <= 1.0000081732920931e+06
 R582: - C3 + 1e+06 alpha_ob29_seg1[1] <= 999925.4331002756
 R583: - C5 + 1e+06 alpha_ob29_seg1[1] <= 999925.4331002756
 R584: C2 + 1e+06 alpha_ob29_seg1[2] <= 999989.2766720931
 R585: C4 + 1e+06 alpha_ob29_seg1[2] <= 999989.2766720931
 R586: C3 + 1e+06 alpha_ob29_seg1[3] <= 1.0000624169002755e+06
 R587: C5 + 1e+06 alpha_ob29_seg1[3] <= 1.0000624169002755e+06
 R588: alpha_ob29_seg1[0] + alpha_ob29_seg1[1] + alpha_ob29_seg1[2]
   + alpha_ob29_seg1[3] >= 1
 R589: - C2 + 1e+06 alpha_ob30_seg1[0] <= 1.0000058189e+06
 R590: - C4 + 1e+06 alpha_ob30_seg1[0] <= 1.0000058189e+06
 R591: - C3 + 1e+06 alpha_ob30_seg1[1] <= 999940.5469
 R592: - C5 + 1e+06 alpha_ob30_seg1[1] <= 999940.5469
 R593: C2 + 1e+06 alpha_ob30_seg1[2] <= 999985.5311
 R594: C4 + 1e+06 alpha_ob30_seg1[2] <= 999985.5311
 R595: C3 + 1e+06 alpha_ob30_seg1[3] <= 1.0000569031e+06
 R596: C5 + 1e+06 alpha_ob30_seg1[3] <= 1.0000569031e+06
 R597: alpha_ob30_seg1[0] + alpha_ob30_seg1[1] + alpha_ob30_seg1[2]
   + alpha_ob30_seg1[3] >= 1
 R598: - C2 + 1e+06 alpha_ob31_seg1[0] <= 1.0000058188284199e+06
 R599: - C4 + 1e+06 alpha_ob31_seg1[0] <= 1.0000058188284199e+06
 R600: - C3 + 1e+06 alpha_ob31_seg1[1] <= 999940.5469002755
 R601: - C5 + 1e+06 alpha_ob31_seg1[1] <= 999940.5469002755
 R602: C2 + 1e+06 alpha_ob31_seg1[2] <= 999991.6311284199
 R603: C4 + 1e+06 alpha_ob31_seg1[2] <= 999991.6311284199
 R604: C3 + 1e+06 alpha_ob31_seg1[3] <= 1.0000453031002756e+06
 R605: C5 + 1e+06 alpha_ob31_seg1[3] <= 1.0000453031002756e+06
 R606: alpha_ob31_seg1[0] + alpha_ob31_seg1[1] + alpha_ob31_seg1[2]
   + alpha_ob31_seg1[3] >= 1
 R607: - C2 + 1e+06 alpha_ob32_seg1[0] <= 999996.9688499999
 R608: - C4 + 1e+06 alpha_ob32_seg1[0] <= 999996.9688499999
 R609: - C3 + 1e+06 alpha_ob32_seg1[1] <= 999940.5469
 R610: - C5 + 1e+06 alpha_ob32_seg1[1] <= 999940.5469
 R611: C2 + 1e+06 alpha_ob32_seg1[2] <= 999991.63115
 R612: C4 + 1e+06 alpha_ob32_seg1[2] <= 999991.63115
 R613: C3 + 1e+06 alpha_ob32_seg1[3] <= 1.0000569031e+06
 R614: C5 + 1e+06 alpha_ob32_seg1[3] <= 1.0000569031e+06
 R615: alpha_ob32_seg1[0] + alpha_ob32_seg1[1] + alpha_ob32_seg1[2]
   + alpha_ob32_seg1[3] >= 1
 R616: - C2 + 1e+06 alpha_ob33_seg1[0] <= 999991.83683
 R617: - C4 + 1e+06 alpha_ob33_seg1[0] <= 999991.83683
 R618: - C3 + 1e+06 alpha_ob33_seg1[1] <= 999940.589
 R619: - C5 + 1e+06 alpha_ob33_seg1[1] <= 999940.589
 R620: C2 + 1e+06 alpha_ob33_seg1[2] <= 1.00000401317e+06
 R621: C4 + 1e+06 alpha_ob33_seg1[2] <= 1.00000401317e+06
 R622: C3 + 1e+06 alpha_ob33_seg1[3] <= 1.000056861e+06
 R623: C5 + 1e+06 alpha_ob33_seg1[3] <= 1.000056861e+06
 R624: alpha_ob33_seg1[0] + alpha_ob33_seg1[1] + alpha_ob33_seg1[2]
   + alpha_ob33_seg1[3] >= 1
 R625: - C2 + 1e+06 alpha_ob34_seg1[0] <= 999991.8368084199
 R626: - C4 + 1e+06 alpha_ob34_seg1[0] <= 999991.8368084199
 R627: - C3 + 1e+06 alpha_ob34_seg1[1] <= 999940.5890002755
 R628: - C5 + 1e+06 alpha_ob34_seg1[1] <= 999940.5890002755
 R629: C2 + 1e+06 alpha_ob34_seg1[2] <= 1.0000056131484199e+06
 R630: C4 + 1e+06 alpha_ob34_seg1[2] <= 1.0000056131484199e+06
 R631: C3 + 1e+06 alpha_ob34_seg1[3] <= 1.0000452610002755e+06
 R632: C5 + 1e+06 alpha_ob34_seg1[3] <= 1.0000452610002755e+06
 R633: alpha_ob34_seg1[0] + alpha_ob34_seg1[1] + alpha_ob34_seg1[2]
   + alpha_ob34_seg1[3] >= 1
 R634: - C4 + 1e+06 alpha_ob0_seg2[0] <= 999992.49102
 R635: - C6 + 1e+06 alpha_ob0_seg2[0] <= 999992.49102
 R636: - C5 + 1e+06 alpha_ob0_seg2[1] <= 999957.1016000001
 R637: - C7 + 1e+06 alpha_ob0_seg2[1] <= 999957.1016000001
 R638: C4 + 1e+06 alpha_ob0_seg2[2] <= 999986.35898
 R639: C6 + 1e+06 alpha_ob0_seg2[2] <= 999986.35898
 R640: C5 + 1e+06 alpha_ob0_seg2[3] <= 1.0000403484e+06
 R641: C7 + 1e+06 alpha_ob0_seg2[3] <= 1.0000403484e+06
 R642: alpha_ob0_seg2[0] + alpha_ob0_seg2[1] + alpha_ob0_seg2[2]
   + alpha_ob0_seg2[3] >= 1
 R643: - C4 + 1e+06 alpha_ob1_seg2[0] <= 999981.3909999999
 R644: - C6 + 1e+06 alpha_ob1_seg2[0] <= 999981.3909999999
 R645: - C5 + 1e+06 alpha_ob1_seg2[1] <= 999957.1016000001
 R646: - C7 + 1e+06 alpha_ob1_seg2[1] <= 999957.1016000001
 R647: C4 + 1e+06 alpha_ob1_seg2[2] <= 1.000004959e+06
 R648: C6 + 1e+06 alpha_ob1_seg2[2] <= 1.000004959e+06
 R649: C5 + 1e+06 alpha_ob1_seg2[3] <= 1.0000403484e+06
 R650: C7 + 1e+06 alpha_ob1_seg2[3] <= 1.0000403484e+06
 R651: alpha_ob1_seg2[0] + alpha_ob1_seg2[1] + alpha_ob1_seg2[2]
   + alpha_ob1_seg2[3] >= 1
 R652: - C4 + 1e+06 alpha_ob2_seg2[0] <= 999982.236799801
 R653: - C6 + 1e+06 alpha_ob2_seg2[0] <= 999982.236799801
 R654: - C5 + 1e+06 alpha_ob2_seg2[1] <= 999970.1095291895
 R655: - C7 + 1e+06 alpha_ob2_seg2[1] <= 999970.1095291895
 R656: C4 + 1e+06 alpha_ob2_seg2[2] <= 999993.3631998011
 R657: C6 + 1e+06 alpha_ob2_seg2[2] <= 999993.3631998011
 R658: C5 + 1e+06 alpha_ob2_seg2[3] <= 1.0000273405291894e+06
 R659: C7 + 1e+06 alpha_ob2_seg2[3] <= 1.0000273405291894e+06
 R660: alpha_ob2_seg2[0] + alpha_ob2_seg2[1] + alpha_ob2_seg2[2]
   + alpha_ob2_seg2[3] >= 1
 R661: - C4 + 1e+06 alpha_ob3_seg2[0] <= 1.0000040868238758e+06
 R662: - C6 + 1e+06 alpha_ob3_seg2[0] <= 1.0000040868238758e+06
 R663: - C5 + 1e+06 alpha_ob3_seg2[1] <= 999957.2594997246
 R664: - C7 + 1e+06 alpha_ob3_seg2[1] <= 999957.2594997246
 R665: C4 + 1e+06 alpha_ob3_seg2[2] <= 999993.3632238759
 R666: C6 + 1e+06 alpha_ob3_seg2[2] <= 999993.3632238759
 R667: C5 + 1e+06 alpha_ob3_seg2[3] <= 1.0000273404997246e+06
 R668: C7 + 1e+06 alpha_ob3_seg2[3] <= 1.0000273404997246e+06
 R669: alpha_ob3_seg2[0] + alpha_ob3_seg2[1] + alpha_ob3_seg2[2]
   + alpha_ob3_seg2[3] >= 1
 R670: - C4 + 1e+06 alpha_ob4_seg2[0] <= 999997.6803502565
 R671: - C6 + 1e+06 alpha_ob4_seg2[0] <= 999997.6803502565
 R672: - C5 + 1e+06 alpha_ob4_seg2[1] <= 999972.7268002755
 R673: - C7 + 1e+06 alpha_ob4_seg2[1] <= 999972.7268002755
 R674: C4 + 1e+06 alpha_ob4_seg2[2] <= 999999.7696102565
 R675: C6 + 1e+06 alpha_ob4_seg2[2] <= 999999.7696102565
 R676: C5 + 1e+06 alpha_ob4_seg2[3] <= 1.0000141232002755e+06
 R677: C7 + 1e+06 alpha_ob4_seg2[3] <= 1.0000141232002755e+06
 R678: alpha_ob4_seg2[0] + alpha_ob4_seg2[1] + alpha_ob4_seg2[2]
   + alpha_ob4_seg2[3] >= 1
 R679: - C4 + 1e+06 alpha_ob5_seg2[0] <= 1.0000038341502565e+06
 R680: - C6 + 1e+06 alpha_ob5_seg2[0] <= 1.0000038341502565e+06
 R681: - C5 + 1e+06 alpha_ob5_seg2[1] <= 999972.6012002755
 R682: - C7 + 1e+06 alpha_ob5_seg2[1] <= 999972.6012002755
 R683: C4 + 1e+06 alpha_ob5_seg2[2] <= 999993.6158102566
 R684: C6 + 1e+06 alpha_ob5_seg2[2] <= 999993.6158102566
 R685: C5 + 1e+06 alpha_ob5_seg2[3] <= 1.0000142488002755e+06
 R686: C7 + 1e+06 alpha_ob5_seg2[3] <= 1.0000142488002755e+06
 R687: alpha_ob5_seg2[0] + alpha_ob5_seg2[1] + alpha_ob5_seg2[2]
   + alpha_ob5_seg2[3] >= 1
 R688: - C4 + 1e+06 alpha_ob6_seg2[0] <= 1.0000066737e+06
 R689: - C6 + 1e+06 alpha_ob6_seg2[0] <= 1.0000066737e+06
 R690: - C5 + 1e+06 alpha_ob6_seg2[1] <= 999952.1958
 R691: - C7 + 1e+06 alpha_ob6_seg2[1] <= 999952.1958
 R692: C4 + 1e+06 alpha_ob6_seg2[2] <= 999948.4263000001
 R693: C6 + 1e+06 alpha_ob6_seg2[2] <= 999948.4263000001
 R694: C5 + 1e+06 alpha_ob6_seg2[3] <= 1.0000452542e+06
 R695: C7 + 1e+06 alpha_ob6_seg2[3] <= 1.0000452542e+06
 R696: alpha_ob6_seg2[0] + alpha_ob6_seg2[1] + alpha_ob6_seg2[2]
   + alpha_ob6_seg2[3] >= 1
 R697: - C4 + 1e+06 alpha_ob7_seg2[0] <= 1.0000070091e+06
 R698: - C6 + 1e+06 alpha_ob7_seg2[0] <= 1.0000070091e+06
 R699: - C5 + 1e+06 alpha_ob7_seg2[1] <= 1.0000598843e+06
 R700: - C7 + 1e+06 alpha_ob7_seg2[1] <= 1.0000598843e+06
 R701: C4 + 1e+06 alpha_ob7_seg2[2] <= 999965.0909
 R702: C6 + 1e+06 alpha_ob7_seg2[2] <= 999965.0909
 R703: C5 + 1e+06 alpha_ob7_seg2[3] <= 999937.5657
 R704: C7 + 1e+06 alpha_ob7_seg2[3] <= 999937.5657
 R705: alpha_ob7_seg2[0] + alpha_ob7_seg2[1] + alpha_ob7_seg2[2]
   + alpha_ob7_seg2[3] >= 1
 R706: - C4 + 1e+06 alpha_ob8_seg2[0] <= 1.000006992114031e+06
 R707: - C6 + 1e+06 alpha_ob8_seg2[0] <= 1.000006992114031e+06
 R708: - C5 + 1e+06 alpha_ob8_seg2[1] <= 1.0000588068002755e+06
 R709: - C7 + 1e+06 alpha_ob8_seg2[1] <= 1.0000588068002755e+06
 R710: C4 + 1e+06 alpha_ob8_seg2[2] <= 999990.457874031
 R711: C6 + 1e+06 alpha_ob8_seg2[2] <= 999990.457874031
 R712: C5 + 1e+06 alpha_ob8_seg2[3] <= 999935.5432002755
 R713: C7 + 1e+06 alpha_ob8_seg2[3] <= 999935.5432002755
 R714: alpha_ob8_seg2[0] + alpha_ob8_seg2[1] + alpha_ob8_seg2[2]
   + alpha_ob8_seg2[3] >= 1
 R715: - C4 + 1e+06 alpha_ob9_seg2[0] <= 1.0000033775063743e+06
 R716: - C6 + 1e+06 alpha_ob9_seg2[0] <= 1.0000033775063743e+06
 R717: - C5 + 1e+06 alpha_ob9_seg2[1] <= 1.0000348666997246e+06
 R718: - C7 + 1e+06 alpha_ob9_seg2[1] <= 1.0000348666997246e+06
 R719: C4 + 1e+06 alpha_ob9_seg2[2] <= 999994.0725863742
 R720: C6 + 1e+06 alpha_ob9_seg2[2] <= 999994.0725863742
 R721: C5 + 1e+06 alpha_ob9_seg2[3] <= 999937.4832997246
 R722: C7 + 1e+06 alpha_ob9_seg2[3] <= 999937.4832997246
 R723: alpha_ob9_seg2[0] + alpha_ob9_seg2[1] + alpha_ob9_seg2[2]
   + alpha_ob9_seg2[3] >= 1
 R724: - C4 + 1e+06 alpha_ob10_seg2[0] <= 999990.2670277582
 R725: - C6 + 1e+06 alpha_ob10_seg2[0] <= 999990.2670277582
 R726: - C5 + 1e+06 alpha_ob10_seg2[1] <= 1.0000347702002756e+06
 R727: - C7 + 1e+06 alpha_ob10_seg2[1] <= 1.0000347702002756e+06
 R728: C4 + 1e+06 alpha_ob10_seg2[2] <= 1.0000071828877581e+06
 R729: C6 + 1e+06 alpha_ob10_seg2[2] <= 1.0000071828877581e+06
 R730: C5 + 1e+06 alpha_ob10_seg2[3] <= 999939.8298002756
 R731: C7 + 1e+06 alpha_ob10_seg2[3] <= 999939.8298002756
 R732: alpha_ob10_seg2[0] + alpha_ob10_seg2[1] + alpha_ob10_seg2[2]
   + alpha_ob10_seg2[3] >= 1
 R733: - C4 + 1e+06 alpha_ob11_seg2[0] <= 999990.267069801
 R734: - C6 + 1e+06 alpha_ob11_seg2[0] <= 999990.267069801
 R735: - C5 + 1e+06 alpha_ob11_seg2[1] <= 1.0000576202069657e+06
 R736: - C7 + 1e+06 alpha_ob11_seg2[1] <= 1.0000576202069657e+06
 R737: C4 + 1e+06 alpha_ob11_seg2[2] <= 1.000002082929801e+06
 R738: C6 + 1e+06 alpha_ob11_seg2[2] <= 1.000002082929801e+06
 R739: C5 + 1e+06 alpha_ob11_seg2[3] <= 999939.8298069657
 R740: C7 + 1e+06 alpha_ob11_seg2[3] <= 999939.8298069657
 R741: alpha_ob11_seg2[0] + alpha_ob11_seg2[1] + alpha_ob11_seg2[2]
   + alpha_ob11_seg2[3] >= 1
 R742: - C4 + 1e+06 alpha_ob12_seg2[0] <= 999991.073732
 R743: - C6 + 1e+06 alpha_ob12_seg2[0] <= 999991.073732
 R744: - C5 + 1e+06 alpha_ob12_seg2[1] <= 999952.2125
 R745: - C7 + 1e+06 alpha_ob12_seg2[1] <= 999952.2125
 R746: C4 + 1e+06 alpha_ob12_seg2[2] <= 999990.776268
 R747: C6 + 1e+06 alpha_ob12_seg2[2] <= 999990.776268
 R748: C5 + 1e+06 alpha_ob12_seg2[3] <= 1.0000452375e+06
 R749: C7 + 1e+06 alpha_ob12_seg2[3] <= 1.0000452375e+06
 R750: alpha_ob12_seg2[0] + alpha_ob12_seg2[1] + alpha_ob12_seg2[2]
   + alpha_ob12_seg2[3] >= 1
 R751: - C4 + 1e+06 alpha_ob13_seg2[0] <= 999985.657099801
 R752: - C6 + 1e+06 alpha_ob13_seg2[0] <= 999985.657099801
 R753: - C5 + 1e+06 alpha_ob13_seg2[1] <= 1.0000576202063023e+06
 R754: - C7 + 1e+06 alpha_ob13_seg2[1] <= 1.0000576202063023e+06
 R755: C4 + 1e+06 alpha_ob13_seg2[2] <= 1.000007192899801e+06
 R756: C6 + 1e+06 alpha_ob13_seg2[2] <= 1.000007192899801e+06
 R757: C5 + 1e+06 alpha_ob13_seg2[3] <= 999939.8298063023
 R758: C7 + 1e+06 alpha_ob13_seg2[3] <= 999939.8298063023
 R759: alpha_ob13_seg2[0] + alpha_ob13_seg2[1] + alpha_ob13_seg2[2]
   + alpha_ob13_seg2[3] >= 1
 R760: - C4 + 1e+06 alpha_ob14_seg2[0] <= 999994.771829801
 R761: - C6 + 1e+06 alpha_ob14_seg2[0] <= 999994.771829801
 R762: - C5 + 1e+06 alpha_ob14_seg2[1] <= 1.0000615983063023e+06
 R763: - C7 + 1e+06 alpha_ob14_seg2[1] <= 1.0000615983063023e+06
 R764: C4 + 1e+06 alpha_ob14_seg2[2] <= 999998.078169801
 R765: C6 + 1e+06 alpha_ob14_seg2[2] <= 999998.078169801
 R766: C5 + 1e+06 alpha_ob14_seg2[3] <= 999935.8517063023
 R767: C7 + 1e+06 alpha_ob14_seg2[3] <= 999935.8517063023
 R768: alpha_ob14_seg2[0] + alpha_ob14_seg2[1] + alpha_ob14_seg2[2]
   + alpha_ob14_seg2[3] >= 1
 R769: - C4 + 1e+06 alpha_ob15_seg2[0] <= 999999.3718445294
 R770: - C6 + 1e+06 alpha_ob15_seg2[0] <= 999999.3718445294
 R771: - C5 + 1e+06 alpha_ob15_seg2[1] <= 1.0000527482997245e+06
 R772: - C7 + 1e+06 alpha_ob15_seg2[1] <= 1.0000527482997245e+06
 R773: C4 + 1e+06 alpha_ob15_seg2[2] <= 999998.0781885295
 R774: C6 + 1e+06 alpha_ob15_seg2[2] <= 999998.0781885295
 R775: C5 + 1e+06 alpha_ob15_seg2[3] <= 999935.8516997246
 R776: C7 + 1e+06 alpha_ob15_seg2[3] <= 999935.8516997246
 R777: alpha_ob15_seg2[0] + alpha_ob15_seg2[1] + alpha_ob15_seg2[2]
   + alpha_ob15_seg2[3] >= 1
 R778: - C4 + 1e+06 alpha_ob16_seg2[0] <= 999999.371829801
 R779: - C6 + 1e+06 alpha_ob16_seg2[0] <= 999999.371829801
 R780: - C5 + 1e+06 alpha_ob16_seg2[1] <= 1.0000527483059706e+06
 R781: - C7 + 1e+06 alpha_ob16_seg2[1] <= 1.0000527483059706e+06
 R782: C4 + 1e+06 alpha_ob16_seg2[2] <= 999993.7281698009
 R783: C6 + 1e+06 alpha_ob16_seg2[2] <= 999993.7281698009
 R784: C5 + 1e+06 alpha_ob16_seg2[3] <= 999944.7017059706
 R785: C7 + 1e+06 alpha_ob16_seg2[3] <= 999944.7017059706
 R786: alpha_ob16_seg2[0] + alpha_ob16_seg2[1] + alpha_ob16_seg2[2]
   + alpha_ob16_seg2[3] >= 1
 R787: - C4 + 1e+06 alpha_ob17_seg2[0] <= 1.0000016470956831e+06
 R788: - C6 + 1e+06 alpha_ob17_seg2[0] <= 1.0000016470956831e+06
 R789: - C5 + 1e+06 alpha_ob17_seg2[1] <= 1.0000598666742218e+06
 R790: - C7 + 1e+06 alpha_ob17_seg2[1] <= 1.0000598666742218e+06
 R791: C4 + 1e+06 alpha_ob17_seg2[2] <= 999994.0767956832
 R792: C6 + 1e+06 alpha_ob17_seg2[2] <= 999994.0767956832
 R793: C5 + 1e+06 alpha_ob17_seg2[3] <= 999937.4792742218
 R794: C7 + 1e+06 alpha_ob17_seg2[3] <= 999937.4792742218
 R795: alpha_ob17_seg2[0] + alpha_ob17_seg2[1] + alpha_ob17_seg2[2]
   + alpha_ob17_seg2[3] >= 1
 R796: - C4 + 1e+06 alpha_ob18_seg2[0] <= 999999.3718601406
 R797: - C6 + 1e+06 alpha_ob18_seg2[0] <= 999999.3718601406
 R798: - C5 + 1e+06 alpha_ob18_seg2[1] <= 1.0000353982997246e+06
 R799: - C7 + 1e+06 alpha_ob18_seg2[1] <= 1.0000353982997246e+06
 R800: C4 + 1e+06 alpha_ob18_seg2[2] <= 999998.0782041405
 R801: C6 + 1e+06 alpha_ob18_seg2[2] <= 999998.0782041405
 R802: C5 + 1e+06 alpha_ob18_seg2[3] <= 999944.7016997245
 R803: C7 + 1e+06 alpha_ob18_seg2[3] <= 999944.7016997245
 R804: alpha_ob18_seg2[0] + alpha_ob18_seg2[1] + alpha_ob18_seg2[2]
   + alpha_ob18_seg2[3] >= 1
 R805: - C4 + 1e+06 alpha_ob19_seg2[0] <= 999956.839399801
 R806: - C6 + 1e+06 alpha_ob19_seg2[0] <= 999956.839399801
 R807: - C5 + 1e+06 alpha_ob19_seg2[1] <= 999925.3963434525
 R808: - C7 + 1e+06 alpha_ob19_seg2[1] <= 999925.3963434525
 R809: C4 + 1e+06 alpha_ob19_seg2[2] <= 1.0000080105998011e+06
 R810: C6 + 1e+06 alpha_ob19_seg2[2] <= 1.0000080105998011e+06
 R811: C5 + 1e+06 alpha_ob19_seg2[3] <= 1.0000720537434525e+06
 R812: C7 + 1e+06 alpha_ob19_seg2[3] <= 1.0000720537434525e+06
 R813: alpha_ob19_seg2[0] + alpha_ob19_seg2[1] + alpha_ob19_seg2[2]
   + alpha_ob19_seg2[3] >= 1
 R814: - C4 + 1e+06 alpha_ob20_seg2[0] <= 999983.6053459151
 R815: - C6 + 1e+06 alpha_ob20_seg2[0] <= 999983.6053459151
 R816: - C5 + 1e+06 alpha_ob20_seg2[1] <= 999987.7722497246
 R817: - C7 + 1e+06 alpha_ob20_seg2[1] <= 999987.7722497246
 R818: C4 + 1e+06 alpha_ob20_seg2[2] <= 1.000013844745915e+06
 R819: C6 + 1e+06 alpha_ob20_seg2[2] <= 1.000013844745915e+06
 R820: C5 + 1e+06 alpha_ob20_seg2[3] <= 999984.8277497246
 R821: C7 + 1e+06 alpha_ob20_seg2[3] <= 999984.8277497246
 R822: alpha_ob20_seg2[0] + alpha_ob20_seg2[1] + alpha_ob20_seg2[2]
   + alpha_ob20_seg2[3] >= 1
 R823: - C4 + 1e+06 alpha_ob21_seg2[0] <= 999958.8717898987
 R824: - C6 + 1e+06 alpha_ob21_seg2[0] <= 999958.8717898987
 R825: - C5 + 1e+06 alpha_ob21_seg2[1] <= 999987.7837602755
 R826: - C7 + 1e+06 alpha_ob21_seg2[1] <= 999987.7837602755
 R827: C4 + 1e+06 alpha_ob21_seg2[2] <= 1.0000385781898986e+06
 R828: C6 + 1e+06 alpha_ob21_seg2[2] <= 1.0000385781898986e+06
 R829: C5 + 1e+06 alpha_ob21_seg2[3] <= 1.0000043162402755e+06
 R830: C7 + 1e+06 alpha_ob21_seg2[3] <= 1.0000043162402755e+06
 R831: alpha_ob21_seg2[0] + alpha_ob21_seg2[1] + alpha_ob21_seg2[2]
   + alpha_ob21_seg2[3] >= 1
 R832: - C4 + 1e+06 alpha_ob22_seg2[0] <= 999939.2718
 R833: - C6 + 1e+06 alpha_ob22_seg2[0] <= 999939.2718
 R834: - C5 + 1e+06 alpha_ob22_seg2[1] <= 999993.13376
 R835: - C7 + 1e+06 alpha_ob22_seg2[1] <= 999993.13376
 R836: C4 + 1e+06 alpha_ob22_seg2[2] <= 1.0000385782e+06
 R837: C6 + 1e+06 alpha_ob22_seg2[2] <= 1.0000385782e+06
 R838: C5 + 1e+06 alpha_ob22_seg2[3] <= 1.00000431624e+06
 R839: C7 + 1e+06 alpha_ob22_seg2[3] <= 1.00000431624e+06
 R840: alpha_ob22_seg2[0] + alpha_ob22_seg2[1] + alpha_ob22_seg2[2]
   + alpha_ob22_seg2[3] >= 1
 R841: - C4 + 1e+06 alpha_ob23_seg2[0] <= 999939.2718101013
 R842: - C6 + 1e+06 alpha_ob23_seg2[0] <= 999939.2718101013
 R843: - C5 + 1e+06 alpha_ob23_seg2[1] <= 999987.7837597246
 R844: - C7 + 1e+06 alpha_ob23_seg2[1] <= 999987.7837597246
 R845: C4 + 1e+06 alpha_ob23_seg2[2] <= 1.0000581782101013e+06
 R846: C6 + 1e+06 alpha_ob23_seg2[2] <= 1.0000581782101013e+06
 R847: C5 + 1e+06 alpha_ob23_seg2[3] <= 1.0000043162397245e+06
 R848: C7 + 1e+06 alpha_ob23_seg2[3] <= 1.0000043162397245e+06
 R849: alpha_ob23_seg2[0] + alpha_ob23_seg2[1] + alpha_ob23_seg2[2]
   + alpha_ob23_seg2[3] >= 1
 R850: - C4 + 1e+06 alpha_ob24_seg2[0] <= 999939.271799801
 R851: - C6 + 1e+06 alpha_ob24_seg2[0] <= 999939.271799801
 R852: - C5 + 1e+06 alpha_ob24_seg2[1] <= 999987.7838262042
 R853: - C7 + 1e+06 alpha_ob24_seg2[1] <= 999987.7838262042
 R854: C4 + 1e+06 alpha_ob24_seg2[2] <= 1.000038578199801e+06
 R855: C6 + 1e+06 alpha_ob24_seg2[2] <= 1.000038578199801e+06
 R856: C5 + 1e+06 alpha_ob24_seg2[3] <= 1.0000096662262043e+06
 R857: C7 + 1e+06 alpha_ob24_seg2[3] <= 1.0000096662262043e+06
 R858: alpha_ob24_seg2[0] + alpha_ob24_seg2[1] + alpha_ob24_seg2[2]
   + alpha_ob24_seg2[3] >= 1
 R859: - C4 + 1e+06 alpha_ob25_seg2[0] <= 999994.9164998011
 R860: - C6 + 1e+06 alpha_ob25_seg2[0] <= 999994.9164998011
 R861: - C5 + 1e+06 alpha_ob25_seg2[1] <= 999925.4284782809
 R862: - C7 + 1e+06 alpha_ob25_seg2[1] <= 999925.4284782809
 R863: C4 + 1e+06 alpha_ob25_seg2[2] <= 999943.6834998011
 R864: C6 + 1e+06 alpha_ob25_seg2[2] <= 999943.6834998011
 R865: C5 + 1e+06 alpha_ob25_seg2[3] <= 1.0000720216782809e+06
 R866: C7 + 1e+06 alpha_ob25_seg2[3] <= 1.0000720216782809e+06
 R867: alpha_ob25_seg2[0] + alpha_ob25_seg2[1] + alpha_ob25_seg2[2]
   + alpha_ob25_seg2[3] >= 1
 R868: - C4 + 1e+06 alpha_ob26_seg2[0] <= 999914.52327842
 R869: - C6 + 1e+06 alpha_ob26_seg2[0] <= 999914.52327842
 R870: - C5 + 1e+06 alpha_ob26_seg2[1] <= 999993.6552362755
 R871: - C7 + 1e+06 alpha_ob26_seg2[1] <= 999993.6552362755
 R872: C4 + 1e+06 alpha_ob26_seg2[2] <= 1.0000829266784199e+06
 R873: C6 + 1e+06 alpha_ob26_seg2[2] <= 1.0000829266784199e+06
 R874: C5 + 1e+06 alpha_ob26_seg2[3] <= 999992.1947642756
 R875: C7 + 1e+06 alpha_ob26_seg2[3] <= 999992.1947642756
 R876: alpha_ob26_seg2[0] + alpha_ob26_seg2[1] + alpha_ob26_seg2[2]
   + alpha_ob26_seg2[3] >= 1
 R877: - C4 + 1e+06 alpha_ob27_seg2[0] <= 999912.265199801
 R878: - C6 + 1e+06 alpha_ob27_seg2[0] <= 999912.265199801
 R879: - C5 + 1e+06 alpha_ob27_seg2[1] <= 999987.7842112777
 R880: - C7 + 1e+06 alpha_ob27_seg2[1] <= 999987.7842112777
 R881: C4 + 1e+06 alpha_ob27_seg2[2] <= 1.000076834799801e+06
 R882: C6 + 1e+06 alpha_ob27_seg2[2] <= 1.000076834799801e+06
 R883: C5 + 1e+06 alpha_ob27_seg2[3] <= 1.0000096658112778e+06
 R884: C7 + 1e+06 alpha_ob27_seg2[3] <= 1.0000096658112778e+06
 R885: alpha_ob27_seg2[0] + alpha_ob27_seg2[1] + alpha_ob27_seg2[2]
   + alpha_ob27_seg2[3] >= 1
 R886: - C4 + 1e+06 alpha_ob28_seg2[0] <= 999920.6152119379
 R887: - C6 + 1e+06 alpha_ob28_seg2[0] <= 999920.6152119379
 R888: - C5 + 1e+06 alpha_ob28_seg2[1] <= 999981.4341997246
 R889: - C7 + 1e+06 alpha_ob28_seg2[1] <= 999981.4341997246
 R890: C4 + 1e+06 alpha_ob28_seg2[2] <= 1.0000768348119379e+06
 R891: C6 + 1e+06 alpha_ob28_seg2[2] <= 1.0000768348119379e+06
 R892: C5 + 1e+06 alpha_ob28_seg2[3] <= 1.0000096657997245e+06
 R893: C7 + 1e+06 alpha_ob28_seg2[3] <= 1.0000096657997245e+06
 R894: alpha_ob28_seg2[0] + alpha_ob28_seg2[1] + alpha_ob28_seg2[2]
   + alpha_ob28_seg2[3] >= 1
 R895: - C4 + 1e+06 alpha_ob29_seg2[0] <= 1.0000081732920931e+06
 R896: - C6 + 1e+06 alpha_ob29_seg2[0] <= 1.0000081732920931e+06
 R897: - C5 + 1e+06 alpha_ob29_seg2[1] <= 999925.4331002756
 R898: - C7 + 1e+06 alpha_ob29_seg2[1] <= 999925.4331002756
 R899: C4 + 1e+06 alpha_ob29_seg2[2] <= 999989.2766720931
 R900: C6 + 1e+06 alpha_ob29_seg2[2] <= 999989.2766720931
 R901: C5 + 1e+06 alpha_ob29_seg2[3] <= 1.0000624169002755e+06
 R902: C7 + 1e+06 alpha_ob29_seg2[3] <= 1.0000624169002755e+06
 R903: alpha_ob29_seg2[0] + alpha_ob29_seg2[1] + alpha_ob29_seg2[2]
   + alpha_ob29_seg2[3] >= 1
 R904: - C4 + 1e+06 alpha_ob30_seg2[0] <= 1.0000058189e+06
 R905: - C6 + 1e+06 alpha_ob30_seg2[0] <= 1.0000058189e+06
 R906: - C5 + 1e+06 alpha_ob30_seg2[1] <= 999940.5469
 R907: - C7 + 1e+06 alpha_ob30_seg2[1] <= 999940.5469
 R908: C4 + 1e+06 alpha_ob30_seg2[2] <= 999985.5311
 R909: C6 + 1e+06 alpha_ob30_seg2[2] <= 999985.5311
 R910: C5 + 1e+06 alpha_ob30_seg2[3] <= 1.0000569031e+06
 R911: C7 + 1e+06 alpha_ob30_seg2[3] <= 1.0000569031e+06
 R912: alpha_ob30_seg2[0] + alpha_ob30_seg2[1] + alpha_ob30_seg2[2]
   + alpha_ob30_seg2[3] >= 1
 R913: - C4 + 1e+06 alpha_ob31_seg2[0] <= 1.0000058188284199e+06
 R914: - C6 + 1e+06 alpha_ob31_seg2[0] <= 1.0000058188284199e+06
 R915: - C5 + 1e+06 alpha_ob31_seg2[1] <= 999940.5469002755
 R916: - C7 + 1e+06 alpha_ob31_seg2[1] <= 999940.5469002755
 R917: C4 + 1e+06 alpha_ob31_seg2[2] <= 999991.6311284199
 R918: C6 + 1e+06 alpha_ob31_seg2[2] <= 999991.6311284199
 R919: C5 + 1e+06 alpha_ob31_seg2[3] <= 1.0000453031002756e+06
 R920: C7 + 1e+06 alpha_ob31_seg2[3] <= 1.0000453031002756e+06
 R921: alpha_ob31_seg2[0] + alpha_ob31_seg2[1] + alpha_ob31_seg2[2]
   + alpha_ob31_seg2[3] >= 1
 R922: - C4 + 1e+06 alpha_ob32_seg2[0] <= 999996.9688499999
 R923: - C6 + 1e+06 alpha_ob32_seg2[0] <= 999996.9688499999
 R924: - C5 + 1e+06 alpha_ob32_seg2[1] <= 999940.5469
 R925: - C7 + 1e+06 alpha_ob32_seg2[1] <= 999940.5469
 R926: C4 + 1e+06 alpha_ob32_seg2[2] <= 999991.63115
 R927: C6 + 1e+06 alpha_ob32_seg2[2] <= 999991.63115
 R928: C5 + 1e+06 alpha_ob32_seg2[3] <= 1.0000569031e+06
 R929: C7 + 1e+06 alpha_ob32_seg2[3] <= 1.0000569031e+06
 R930: alpha_ob32_seg2[0] + alpha_ob32_seg2[1] + alpha_ob32_seg2[2]
   + alpha_ob32_seg2[3] >= 1
 R931: - C4 + 1e+06 alpha_ob33_seg2[0] <= 999991.83683
 R932: - C6 + 1e+06 alpha_ob33_seg2[0] <= 999991.83683
 R933: - C5 + 1e+06 alpha_ob33_seg2[1] <= 999940.589
 R934: - C7 + 1e+06 alpha_ob33_seg2[1] <= 999940.589
 R935: C4 + 1e+06 alpha_ob33_seg2[2] <= 1.00000401317e+06
 R936: C6 + 1e+06 alpha_ob33_seg2[2] <= 1.00000401317e+06
 R937: C5 + 1e+06 alpha_ob33_seg2[3] <= 1.000056861e+06
 R938: C7 + 1e+06 alpha_ob33_seg2[3] <= 1.000056861e+06
 R939: alpha_ob33_seg2[0] + alpha_ob33_seg2[1] + alpha_ob33_seg2[2]
   + alpha_ob33_seg2[3] >= 1
 R940: - C4 + 1e+06 alpha_ob34_seg2[0] <= 999991.8368084199
 R941: - C6 + 1e+06 alpha_ob34_seg2[0] <= 999991.8368084199
 R942: - C5 + 1e+06 alpha_ob34_seg2[1] <= 999940.5890002755
 R943: - C7 + 1e+06 alpha_ob34_seg2[1] <= 999940.5890002755
 R944: C4 + 1e+06 alpha_ob34_seg2[2] <= 1.0000056131484199e+06
 R945: C6 + 1e+06 alpha_ob34_seg2[2] <= 1.0000056131484199e+06
 R946: C5 + 1e+06 alpha_ob34_seg2[3] <= 1.0000452610002755e+06
 R947: C7 + 1e+06 alpha_ob34_seg2[3] <= 1.0000452610002755e+06
 R948: alpha_ob34_seg2[0] + alpha_ob34_seg2[1] + alpha_ob34_seg2[2]
   + alpha_ob34_seg2[3] >= 1
 R949: C0 >= 0
 R950: C1 >= 0
 R951: C0 <= 85
 R952: C1 <= 85
 R953: C2 >= 0
 R954: C3 >= 0
 R955: C2 <= 85
 R956: C3 <= 85
 R957: C4 >= 0
 R958: C5 >= 0
 R959: C4 <= 85
 R960: C5 <= 85
 R961: C6 >= 0
 R962: C7 >= 0
 R963: C6 <= 85
 R964: C7 <= 85
Bounds
 Constant = 1
Binaries
 alpha_ob0_seg0[0] alpha_ob0_seg0[1] alpha_ob0_seg0[2] alpha_ob0_seg0[3]
 alpha_ob1_seg0[0] alpha_ob1_seg0[1] alpha_ob1_seg0[2] alpha_ob1_seg0[3]
 alpha_ob2_seg0[0] alpha_ob2_seg0[1] alpha_ob2_seg0[2] alpha_ob2_seg0[3]
 alpha_ob3_seg0[0] alpha_ob3_seg0[1] alpha_ob3_seg0[2] alpha_ob3_seg0[3]
 alpha_ob4_seg0[0] alpha_ob4_seg0[1] alpha_ob4_seg0[2] alpha_ob4_seg0[3]
 alpha_ob5_seg0[0] alpha_ob5_seg0[1] alpha_ob5_seg0[2] alpha_ob5_seg0[3]
 alpha_ob6_seg0[0] alpha_ob6_seg0[1] alpha_ob6_seg0[2] alpha_ob6_seg0[3]
 alpha_ob7_seg0[0] alpha_ob7_seg0[1] alpha_ob7_seg0[2] alpha_ob7_seg0[3]
 alpha_ob8_seg0[0] alpha_ob8_seg0[1] alpha_ob8_seg0[2] alpha_ob8_seg0[3]
 alpha_ob9_seg0[0] alpha_ob9_seg0[1] alpha_ob9_seg0[2] alpha_ob9_seg0[3]
 alpha_ob10_seg0[0] alpha_ob10_seg0[1] alpha_ob10_seg0[2]
 alpha_ob10_seg0[3] alpha_ob11_seg0[0] alpha_ob11_seg0[1]
 alpha_ob11_seg0[2] alpha_ob11_seg0[3] alpha_ob12_seg0[0]
 alpha_ob12_seg0[1] alpha_ob12_seg0[2] alpha_ob12_seg0[3]
 alpha_ob13_seg0[0] alpha_ob13_seg0[1] alpha_ob13_seg0[2]
 alpha_ob13_seg0[3] alpha_ob14_seg0[0] alpha_ob14_seg0[1]
 alpha_ob14_seg0[2] alpha_ob14_seg0[3] alpha_ob15_seg0[0]
 alpha_ob15_seg0[1] alpha_ob15_seg0[2] alpha_ob15_seg0[3]
 alpha_ob16_seg0[0] alpha_ob16_seg0[1] alpha_ob16_seg0[2]
 alpha_ob16_seg0[3] alpha_ob17_seg0[0] alpha_ob17_seg0[1]
 alpha_ob17_seg0[2] alpha_ob17_seg0[3] alpha_ob18_seg0[0]
 alpha_ob18_seg0[1] alpha_ob18_seg0[2] alpha_ob18_seg0[3]
 alpha_ob19_seg0[0] alpha_ob19_seg0[1] alpha_ob19_seg0[2]
 alpha_ob19_seg0[3] alpha_ob20_seg0[0] alpha_ob20_seg0[1]
 alpha_ob20_seg0[2] alpha_ob20_seg0[3] alpha_ob21_seg0[0]
 alpha_ob21_seg0[1] alpha_ob21_seg0[2] alpha_ob21_seg0[3]
 alpha_ob22_seg0[0] alpha_ob22_seg0[1] alpha_ob22_seg0[2]
 alpha_ob22_seg0[3] alpha_ob23_seg0[0] alpha_ob23_seg0[1]
 alpha_ob23_seg0[2] alpha_ob23_seg0[3] alpha_ob24_seg0[0]
 alpha_ob24_seg0[1] alpha_ob24_seg0[2] alpha_ob24_seg0[3]
 alpha_ob25_seg0[0] alpha_ob25_seg0[1] alpha_ob25_seg0[2]
 alpha_ob25_seg0[3] alpha_ob26_seg0[0] alpha_ob26_seg0[1]
 alpha_ob26_seg0[2] alpha_ob26_seg0[3] alpha_ob27_seg0[0]
 alpha_ob27_seg0[1] alpha_ob27_seg0[2] alpha_ob27_seg0[3]
 alpha_ob28_seg0[0] alpha_ob28_seg0[1] alpha_ob28_seg0[2]
 alpha_ob28_seg0[3] alpha_ob29_seg0[0] alpha_ob29_seg0[1]
 alpha_ob29_seg0[2] alpha_ob29_seg0[3] alpha_ob30_seg0[0]
 alpha_ob30_seg0[1] alpha_ob30_seg0[2] alpha_ob30_seg0[3]
 alpha_ob31_seg0[0] alpha_ob31_seg0[1] alpha_ob31_seg0[2]
 alpha_ob31_seg0[3] alpha_ob32_seg0[0] alpha_ob32_seg0[1]
 alpha_ob32_seg0[2] alpha_ob32_seg0[3] alpha_ob33_seg0[0]
 alpha_ob33_seg0[1] alpha_ob33_seg0[2] alpha_ob33_seg0[3]
 alpha_ob34_seg0[0] alpha_ob34_seg0[1] alpha_ob34_seg0[2]
 alpha_ob34_seg0[3] alpha_ob0_seg1[0] alpha_ob0_seg1[1] alpha_ob0_seg1[2]
 alpha_ob0_seg1[3] alpha_ob1_seg1[0] alpha_ob1_seg1[1] alpha_ob1_seg1[2]
 alpha_ob1_seg1[3] alpha_ob2_seg1[0] alpha_ob2_seg1[1] alpha_ob2_seg1[2]
 alpha_ob2_seg1[3] alpha_ob3_seg1[0] alpha_ob3_seg1[1] alpha_ob3_seg1[2]
 alpha_ob3_seg1[3] alpha_ob4_seg1[0] alpha_ob4_seg1[1] alpha_ob4_seg1[2]
 alpha_ob4_seg1[3] alpha_ob5_seg1[0] alpha_ob5_seg1[1] alpha_ob5_seg1[2]
 alpha_ob5_seg1[3] alpha_ob6_seg1[0] alpha_ob6_seg1[1] alpha_ob6_seg1[2]
 alpha_ob6_seg1[3] alpha_ob7_seg1[0] alpha_ob7_seg1[1] alpha_ob7_seg1[2]
 alpha_ob7_seg1[3] alpha_ob8_seg1[0] alpha_ob8_seg1[1] alpha_ob8_seg1[2]
 alpha_ob8_seg1[3] alpha_ob9_seg1[0] alpha_ob9_seg1[1] alpha_ob9_seg1[2]
 alpha_ob9_seg1[3] alpha_ob10_seg1[0] alpha_ob10_seg1[1] alpha_ob10_seg1[2]
 alpha_ob10_seg1[3] alpha_ob11_seg1[0] alpha_ob11_seg1[1]
 alpha_ob11_seg1[2] alpha_ob11_seg1[3] alpha_ob12_seg1[0]
 alpha_ob12_seg1[1] alpha_ob12_seg1[2] alpha_ob12_seg1[3]
 alpha_ob13_seg1[0] alpha_ob13_seg1[1] alpha_ob13_seg1[2]
 alpha_ob13_seg1[3] alpha_ob14_seg1[0] alpha_ob14_seg1[1]
 alpha_ob14_seg1[2] alpha_ob14_seg1[3] alpha_ob15_seg1[0]
 alpha_ob15_seg1[1] alpha_ob15_seg1[2] alpha_ob15_seg1[3]
 alpha_ob16_seg1[0] alpha_ob16_seg1[1] alpha_ob16_seg1[2]
 alpha_ob16_seg1[3] alpha_ob17_seg1[0] alpha_ob17_seg1[1]
 alpha_ob17_seg1[2] alpha_ob17_seg1[3] alpha_ob18_seg1[0]
 alpha_ob18_seg1[1] alpha_ob18_seg1[2] alpha_ob18_seg1[3]
 alpha_ob19_seg1[0] alpha_ob19_seg1[1] alpha_ob19_seg1[2]
 alpha_ob19_seg1[3] alpha_ob20_seg1[0] alpha_ob20_seg1[1]
 alpha_ob20_seg1[2] alpha_ob20_seg1[3] alpha_ob21_seg1[0]
 alpha_ob21_seg1[1] alpha_ob21_seg1[2] alpha_ob21_seg1[3]
 alpha_ob22_seg1[0] alpha_ob22_seg1[1] alpha_ob22_seg1[2]
 alpha_ob22_seg1[3] alpha_ob23_seg1[0] alpha_ob23_seg1[1]
 alpha_ob23_seg1[2] alpha_ob23_seg1[3] alpha_ob24_seg1[0]
 alpha_ob24_seg1[1] alpha_ob24_seg1[2] alpha_ob24_seg1[3]
 alpha_ob25_seg1[0] alpha_ob25_seg1[1] alpha_ob25_seg1[2]
 alpha_ob25_seg1[3] alpha_ob26_seg1[0] alpha_ob26_seg1[1]
 alpha_ob26_seg1[2] alpha_ob26_seg1[3] alpha_ob27_seg1[0]
 alpha_ob27_seg1[1] alpha_ob27_seg1[2] alpha_ob27_seg1[3]
 alpha_ob28_seg1[0] alpha_ob28_seg1[1] alpha_ob28_seg1[2]
 alpha_ob28_seg1[3] alpha_ob29_seg1[0] alpha_ob29_seg1[1]
 alpha_ob29_seg1[2] alpha_ob29_seg1[3] alpha_ob30_seg1[0]
 alpha_ob30_seg1[1] alpha_ob30_seg1[2] alpha_ob30_seg1[3]
 alpha_ob31_seg1[0] alpha_ob31_seg1[1] alpha_ob31_seg1[2]
 alpha_ob31_seg1[3] alpha_ob32_seg1[0] alpha_ob32_seg1[1]
 alpha_ob32_seg1[2] alpha_ob32_seg1[3] alpha_ob33_seg1[0]
 alpha_ob33_seg1[1] alpha_ob33_seg1[2] alpha_ob33_seg1[3]
 alpha_ob34_seg1[0] alpha_ob34_seg1[1] alpha_ob34_seg1[2]
 alpha_ob34_seg1[3] alpha_ob0_seg2[0] alpha_ob0_seg2[1] alpha_ob0_seg2[2]
 alpha_ob0_seg2[3] alpha_ob1_seg2[0] alpha_ob1_seg2[1] alpha_ob1_seg2[2]
 alpha_ob1_seg2[3] alpha_ob2_seg2[0] alpha_ob2_seg2[1] alpha_ob2_seg2[2]
 alpha_ob2_seg2[3] alpha_ob3_seg2[0] alpha_ob3_seg2[1] alpha_ob3_seg2[2]
 alpha_ob3_seg2[3] alpha_ob4_seg2[0] alpha_ob4_seg2[1] alpha_ob4_seg2[2]
 alpha_ob4_seg2[3] alpha_ob5_seg2[0] alpha_ob5_seg2[1] alpha_ob5_seg2[2]
 alpha_ob5_seg2[3] alpha_ob6_seg2[0] alpha_ob6_seg2[1] alpha_ob6_seg2[2]
 alpha_ob6_seg2[3] alpha_ob7_seg2[0] alpha_ob7_seg2[1] alpha_ob7_seg2[2]
 alpha_ob7_seg2[3] alpha_ob8_seg2[0] alpha_ob8_seg2[1] alpha_ob8_seg2[2]
 alpha_ob8_seg2[3] alpha_ob9_seg2[0] alpha_ob9_seg2[1] alpha_ob9_seg2[2]
 alpha_ob9_seg2[3] alpha_ob10_seg2[0] alpha_ob10_seg2[1] alpha_ob10_seg2[2]
 alpha_ob10_seg2[3] alpha_ob11_seg2[0] alpha_ob11_seg2[1]
 alpha_ob11_seg2[2] alpha_ob11_seg2[3] alpha_ob12_seg2[0]
 alpha_ob12_seg2[1] alpha_ob12_seg2[2] alpha_ob12_seg2[3]
 alpha_ob13_seg2[0] alpha_ob13_seg2[1] alpha_ob13_seg2[2]
 alpha_ob13_seg2[3] alpha_ob14_seg2[0] alpha_ob14_seg2[1]
 alpha_ob14_seg2[2] alpha_ob14_seg2[3] alpha_ob15_seg2[0]
 alpha_ob15_seg2[1] alpha_ob15_seg2[2] alpha_ob15_seg2[3]
 alpha_ob16_seg2[0] alpha_ob16_seg2[1] alpha_ob16_seg2[2]
 alpha_ob16_seg2[3] alpha_ob17_seg2[0] alpha_ob17_seg2[1]
 alpha_ob17_seg2[2] alpha_ob17_seg2[3] alpha_ob18_seg2[0]
 alpha_ob18_seg2[1] alpha_ob18_seg2[2] alpha_ob18_seg2[3]
 alpha_ob19_seg2[0] alpha_ob19_seg2[1] alpha_ob19_seg2[2]
 alpha_ob19_seg2[3] alpha_ob20_seg2[0] alpha_ob20_seg2[1]
 alpha_ob20_seg2[2] alpha_ob20_seg2[3] alpha_ob21_seg2[0]
 alpha_ob21_seg2[1] alpha_ob21_seg2[2] alpha_ob21_seg2[3]
 alpha_ob22_seg2[0] alpha_ob22_seg2[1] alpha_ob22_seg2[2]
 alpha_ob22_seg2[3] alpha_ob23_seg2[0] alpha_ob23_seg2[1]
 alpha_ob23_seg2[2] alpha_ob23_seg2[3] alpha_ob24_seg2[0]
 alpha_ob24_seg2[1] alpha_ob24_seg2[2] alpha_ob24_seg2[3]
 alpha_ob25_seg2[0] alpha_ob25_seg2[1] alpha_ob25_seg2[2]
 alpha_ob25_seg2[3] alpha_ob26_seg2[0] alpha_ob26_seg2[1]
 alpha_ob26_seg2[2] alpha_ob26_seg2[3] alpha_ob27_seg2[0]
 alpha_ob27_seg2[1] alpha_ob27_seg2[2] alpha_ob27_seg2[3]
 alpha_ob28_seg2[0] alpha_ob28_seg2[1] alpha_ob28_seg2[2]
 alpha_ob28_seg2[3] alpha_ob29_seg2[0] alpha_ob29_seg2[1]
 alpha_ob29_seg2[2] alpha_ob29_seg2[3] alpha_ob30_seg2[0]
 alpha_ob30_seg2[1] alpha_ob30_seg2[2] alpha_ob30_seg2[3]
 alpha_ob31_seg2[0] alpha_ob31_seg2[1] alpha_ob31_seg2[2]
 alpha_ob31_seg2[3] alpha_ob32_seg2[0] alpha_ob32_seg2[1]
 alpha_ob32_seg2[2] alpha_ob32_seg2[3] alpha_ob33_seg2[0]
 alpha_ob33_seg2[1] alpha_ob33_seg2[2] alpha_ob33_seg2[3]
 alpha_ob34_seg2[0] alpha_ob34_seg2[1] alpha_ob34_seg2[2]
 alpha_ob34_seg2[3]
End
