\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 60 C0 - 20 C1 - 60 C2 - 20 C3 + 2000 Constant + [ 2 C0 ^2 + 2 C1 ^2
   + 2 C2 ^2 + 2 C3 ^2 ] / 2 
Subject To
 R0: C0 = 0
 R1: C1 = 1
 R2: C2 = 30
 R3: C3 = 10
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
 R184: - C0 + 1e+06 alpha_ob20_seg0[0] <= 999958.8717898987
 R185: - C2 + 1e+06 alpha_ob20_seg0[0] <= 999958.8717898987
 R186: - C1 + 1e+06 alpha_ob20_seg0[1] <= 999987.7837602755
 R187: - C3 + 1e+06 alpha_ob20_seg0[1] <= 999987.7837602755
 R188: C0 + 1e+06 alpha_ob20_seg0[2] <= 1.0000385781898986e+06
 R189: C2 + 1e+06 alpha_ob20_seg0[2] <= 1.0000385781898986e+06
 R190: C1 + 1e+06 alpha_ob20_seg0[3] <= 1.0000043162402755e+06
 R191: C3 + 1e+06 alpha_ob20_seg0[3] <= 1.0000043162402755e+06
 R192: alpha_ob20_seg0[0] + alpha_ob20_seg0[1] + alpha_ob20_seg0[2]
   + alpha_ob20_seg0[3] >= 1
 R193: - C0 + 1e+06 alpha_ob21_seg0[0] <= 999939.2718
 R194: - C2 + 1e+06 alpha_ob21_seg0[0] <= 999939.2718
 R195: - C1 + 1e+06 alpha_ob21_seg0[1] <= 999993.13376
 R196: - C3 + 1e+06 alpha_ob21_seg0[1] <= 999993.13376
 R197: C0 + 1e+06 alpha_ob21_seg0[2] <= 1.0000385782e+06
 R198: C2 + 1e+06 alpha_ob21_seg0[2] <= 1.0000385782e+06
 R199: C1 + 1e+06 alpha_ob21_seg0[3] <= 1.00000431624e+06
 R200: C3 + 1e+06 alpha_ob21_seg0[3] <= 1.00000431624e+06
 R201: alpha_ob21_seg0[0] + alpha_ob21_seg0[1] + alpha_ob21_seg0[2]
   + alpha_ob21_seg0[3] >= 1
 R202: - C0 + 1e+06 alpha_ob22_seg0[0] <= 999939.2718101013
 R203: - C2 + 1e+06 alpha_ob22_seg0[0] <= 999939.2718101013
 R204: - C1 + 1e+06 alpha_ob22_seg0[1] <= 999987.7837597246
 R205: - C3 + 1e+06 alpha_ob22_seg0[1] <= 999987.7837597246
 R206: C0 + 1e+06 alpha_ob22_seg0[2] <= 1.0000581782101013e+06
 R207: C2 + 1e+06 alpha_ob22_seg0[2] <= 1.0000581782101013e+06
 R208: C1 + 1e+06 alpha_ob22_seg0[3] <= 1.0000043162397245e+06
 R209: C3 + 1e+06 alpha_ob22_seg0[3] <= 1.0000043162397245e+06
 R210: alpha_ob22_seg0[0] + alpha_ob22_seg0[1] + alpha_ob22_seg0[2]
   + alpha_ob22_seg0[3] >= 1
 R211: - C0 + 1e+06 alpha_ob23_seg0[0] <= 999994.9164998011
 R212: - C2 + 1e+06 alpha_ob23_seg0[0] <= 999994.9164998011
 R213: - C1 + 1e+06 alpha_ob23_seg0[1] <= 999925.4284782809
 R214: - C3 + 1e+06 alpha_ob23_seg0[1] <= 999925.4284782809
 R215: C0 + 1e+06 alpha_ob23_seg0[2] <= 999943.6834998011
 R216: C2 + 1e+06 alpha_ob23_seg0[2] <= 999943.6834998011
 R217: C1 + 1e+06 alpha_ob23_seg0[3] <= 1.0000720216782809e+06
 R218: C3 + 1e+06 alpha_ob23_seg0[3] <= 1.0000720216782809e+06
 R219: alpha_ob23_seg0[0] + alpha_ob23_seg0[1] + alpha_ob23_seg0[2]
   + alpha_ob23_seg0[3] >= 1
 R220: - C0 + 1e+06 alpha_ob24_seg0[0] <= 999914.52327842
 R221: - C2 + 1e+06 alpha_ob24_seg0[0] <= 999914.52327842
 R222: - C1 + 1e+06 alpha_ob24_seg0[1] <= 999993.6552362755
 R223: - C3 + 1e+06 alpha_ob24_seg0[1] <= 999993.6552362755
 R224: C0 + 1e+06 alpha_ob24_seg0[2] <= 1.0000829266784199e+06
 R225: C2 + 1e+06 alpha_ob24_seg0[2] <= 1.0000829266784199e+06
 R226: C1 + 1e+06 alpha_ob24_seg0[3] <= 999992.1947642756
 R227: C3 + 1e+06 alpha_ob24_seg0[3] <= 999992.1947642756
 R228: alpha_ob24_seg0[0] + alpha_ob24_seg0[1] + alpha_ob24_seg0[2]
   + alpha_ob24_seg0[3] >= 1
 R229: - C0 + 1e+06 alpha_ob25_seg0[0] <= 1.0000081732920931e+06
 R230: - C2 + 1e+06 alpha_ob25_seg0[0] <= 1.0000081732920931e+06
 R231: - C1 + 1e+06 alpha_ob25_seg0[1] <= 999925.4331002756
 R232: - C3 + 1e+06 alpha_ob25_seg0[1] <= 999925.4331002756
 R233: C0 + 1e+06 alpha_ob25_seg0[2] <= 999989.2766720931
 R234: C2 + 1e+06 alpha_ob25_seg0[2] <= 999989.2766720931
 R235: C1 + 1e+06 alpha_ob25_seg0[3] <= 1.0000624169002755e+06
 R236: C3 + 1e+06 alpha_ob25_seg0[3] <= 1.0000624169002755e+06
 R237: alpha_ob25_seg0[0] + alpha_ob25_seg0[1] + alpha_ob25_seg0[2]
   + alpha_ob25_seg0[3] >= 1
 R238: - C0 + 1e+06 alpha_ob26_seg0[0] <= 1.0000058189e+06
 R239: - C2 + 1e+06 alpha_ob26_seg0[0] <= 1.0000058189e+06
 R240: - C1 + 1e+06 alpha_ob26_seg0[1] <= 999940.5469
 R241: - C3 + 1e+06 alpha_ob26_seg0[1] <= 999940.5469
 R242: C0 + 1e+06 alpha_ob26_seg0[2] <= 999985.5311
 R243: C2 + 1e+06 alpha_ob26_seg0[2] <= 999985.5311
 R244: C1 + 1e+06 alpha_ob26_seg0[3] <= 1.0000569031e+06
 R245: C3 + 1e+06 alpha_ob26_seg0[3] <= 1.0000569031e+06
 R246: alpha_ob26_seg0[0] + alpha_ob26_seg0[1] + alpha_ob26_seg0[2]
   + alpha_ob26_seg0[3] >= 1
 R247: - C0 + 1e+06 alpha_ob27_seg0[0] <= 1.0000058188284199e+06
 R248: - C2 + 1e+06 alpha_ob27_seg0[0] <= 1.0000058188284199e+06
 R249: - C1 + 1e+06 alpha_ob27_seg0[1] <= 999940.5469002755
 R250: - C3 + 1e+06 alpha_ob27_seg0[1] <= 999940.5469002755
 R251: C0 + 1e+06 alpha_ob27_seg0[2] <= 999991.6311284199
 R252: C2 + 1e+06 alpha_ob27_seg0[2] <= 999991.6311284199
 R253: C1 + 1e+06 alpha_ob27_seg0[3] <= 1.0000453031002756e+06
 R254: C3 + 1e+06 alpha_ob27_seg0[3] <= 1.0000453031002756e+06
 R255: alpha_ob27_seg0[0] + alpha_ob27_seg0[1] + alpha_ob27_seg0[2]
   + alpha_ob27_seg0[3] >= 1
 R256: - C0 + 1e+06 alpha_ob28_seg0[0] <= 999996.9688499999
 R257: - C2 + 1e+06 alpha_ob28_seg0[0] <= 999996.9688499999
 R258: - C1 + 1e+06 alpha_ob28_seg0[1] <= 999940.5469
 R259: - C3 + 1e+06 alpha_ob28_seg0[1] <= 999940.5469
 R260: C0 + 1e+06 alpha_ob28_seg0[2] <= 999991.63115
 R261: C2 + 1e+06 alpha_ob28_seg0[2] <= 999991.63115
 R262: C1 + 1e+06 alpha_ob28_seg0[3] <= 1.0000569031e+06
 R263: C3 + 1e+06 alpha_ob28_seg0[3] <= 1.0000569031e+06
 R264: alpha_ob28_seg0[0] + alpha_ob28_seg0[1] + alpha_ob28_seg0[2]
   + alpha_ob28_seg0[3] >= 1
 R265: - C0 + 1e+06 alpha_ob29_seg0[0] <= 999991.83683
 R266: - C2 + 1e+06 alpha_ob29_seg0[0] <= 999991.83683
 R267: - C1 + 1e+06 alpha_ob29_seg0[1] <= 999940.589
 R268: - C3 + 1e+06 alpha_ob29_seg0[1] <= 999940.589
 R269: C0 + 1e+06 alpha_ob29_seg0[2] <= 1.00000401317e+06
 R270: C2 + 1e+06 alpha_ob29_seg0[2] <= 1.00000401317e+06
 R271: C1 + 1e+06 alpha_ob29_seg0[3] <= 1.000056861e+06
 R272: C3 + 1e+06 alpha_ob29_seg0[3] <= 1.000056861e+06
 R273: alpha_ob29_seg0[0] + alpha_ob29_seg0[1] + alpha_ob29_seg0[2]
   + alpha_ob29_seg0[3] >= 1
 R274: - C0 + 1e+06 alpha_ob30_seg0[0] <= 999991.8368084199
 R275: - C2 + 1e+06 alpha_ob30_seg0[0] <= 999991.8368084199
 R276: - C1 + 1e+06 alpha_ob30_seg0[1] <= 999940.5890002755
 R277: - C3 + 1e+06 alpha_ob30_seg0[1] <= 999940.5890002755
 R278: C0 + 1e+06 alpha_ob30_seg0[2] <= 1.0000056131484199e+06
 R279: C2 + 1e+06 alpha_ob30_seg0[2] <= 1.0000056131484199e+06
 R280: C1 + 1e+06 alpha_ob30_seg0[3] <= 1.0000452610002755e+06
 R281: C3 + 1e+06 alpha_ob30_seg0[3] <= 1.0000452610002755e+06
 R282: alpha_ob30_seg0[0] + alpha_ob30_seg0[1] + alpha_ob30_seg0[2]
   + alpha_ob30_seg0[3] >= 1
 R283: C0 >= 0
 R284: C1 >= 0
 R285: C0 <= 85
 R286: C1 <= 85
 R287: C2 >= 0
 R288: C3 >= 0
 R289: C2 <= 85
 R290: C3 <= 85
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
End
