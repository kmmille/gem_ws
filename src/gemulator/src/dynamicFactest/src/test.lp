\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 100 C0 - 160 C1 - 100 C2 - 160 C3 - 100 C4 - 160 C5 + 26700 Constant + [
   2 C0 ^2 + 2 C1 ^2 + 2 C2 ^2 + 2 C3 ^2 + 2 C4 ^2 + 2 C5 ^2 ] / 2 
Subject To
 R0: C0 = 44.99981069295919
 R1: C1 = 46.6170323131715
 R2: C4 = 50
 R3: C5 = 80
 R4: - C0 + 1e+06 alpha_ob0_seg0[0] <= 1.0000284171e+06
 R5: - C2 + 1e+06 alpha_ob0_seg0[0] <= 1.0000284171e+06
 R6: - C1 + 1e+06 alpha_ob0_seg0[1] <= 999957.0843
 R7: - C3 + 1e+06 alpha_ob0_seg0[1] <= 999957.0843
 R8: C0 + 1e+06 alpha_ob0_seg0[2] <= 999957.1829
 R9: C2 + 1e+06 alpha_ob0_seg0[2] <= 999957.1829
 R10: C1 + 1e+06 alpha_ob0_seg0[3] <= 1.0000403657e+06
 R11: C3 + 1e+06 alpha_ob0_seg0[3] <= 1.0000403657e+06
 R12: alpha_ob0_seg0[0] + alpha_ob0_seg0[1] + alpha_ob0_seg0[2]
   + alpha_ob0_seg0[3] >= 1
 R13: - C0 + 1e+06 alpha_ob1_seg0[0] <= 1.0000284170499525e+06
 R14: - C2 + 1e+06 alpha_ob1_seg0[0] <= 1.0000284170499525e+06
 R15: - C1 + 1e+06 alpha_ob1_seg0[1] <= 999957.0843002755
 R16: - C3 + 1e+06 alpha_ob1_seg0[1] <= 999957.0843002755
 R17: C0 + 1e+06 alpha_ob1_seg0[2] <= 999969.0328499526
 R18: C2 + 1e+06 alpha_ob1_seg0[2] <= 999969.0328499526
 R19: C1 + 1e+06 alpha_ob1_seg0[3] <= 1.0000132657002755e+06
 R20: C3 + 1e+06 alpha_ob1_seg0[3] <= 1.0000132657002755e+06
 R21: alpha_ob1_seg0[0] + alpha_ob1_seg0[1] + alpha_ob1_seg0[2]
   + alpha_ob1_seg0[3] >= 1
 R22: - C0 + 1e+06 alpha_ob2_seg0[0] <= 1.0000400171500475e+06
 R23: - C2 + 1e+06 alpha_ob2_seg0[0] <= 1.0000400171500475e+06
 R24: - C1 + 1e+06 alpha_ob2_seg0[1] <= 999957.0842997245
 R25: - C3 + 1e+06 alpha_ob2_seg0[1] <= 999957.0842997245
 R26: C0 + 1e+06 alpha_ob2_seg0[2] <= 999957.4329500474
 R27: C2 + 1e+06 alpha_ob2_seg0[2] <= 999957.4329500474
 R28: C1 + 1e+06 alpha_ob2_seg0[3] <= 1.0000132656997246e+06
 R29: C3 + 1e+06 alpha_ob2_seg0[3] <= 1.0000132656997246e+06
 R30: alpha_ob2_seg0[0] + alpha_ob2_seg0[1] + alpha_ob2_seg0[2]
   + alpha_ob2_seg0[3] >= 1
 R31: - C0 + 1e+06 alpha_ob3_seg0[0] <= 1.000022266175665e+06
 R32: - C2 + 1e+06 alpha_ob3_seg0[0] <= 1.000022266175665e+06
 R33: - C1 + 1e+06 alpha_ob3_seg0[1] <= 999957.1681002756
 R34: - C3 + 1e+06 alpha_ob3_seg0[1] <= 999957.1681002756
 R35: C0 + 1e+06 alpha_ob3_seg0[2] <= 999975.183775665
 R36: C2 + 1e+06 alpha_ob3_seg0[2] <= 999975.183775665
 R37: C1 + 1e+06 alpha_ob3_seg0[3] <= 1.0000271819002755e+06
 R38: C3 + 1e+06 alpha_ob3_seg0[3] <= 1.0000271819002755e+06
 R39: alpha_ob3_seg0[0] + alpha_ob3_seg0[1] + alpha_ob3_seg0[2]
   + alpha_ob3_seg0[3] >= 1
 R40: - C0 + 1e+06 alpha_ob4_seg0[0] <= 1.0000139162e+06
 R41: - C2 + 1e+06 alpha_ob4_seg0[0] <= 1.0000139162e+06
 R42: - C1 + 1e+06 alpha_ob4_seg0[1] <= 999957.1679999999
 R43: - C3 + 1e+06 alpha_ob4_seg0[1] <= 999957.1679999999
 R44: C0 + 1e+06 alpha_ob4_seg0[2] <= 999975.1838
 R45: C2 + 1e+06 alpha_ob4_seg0[2] <= 999975.1838
 R46: C1 + 1e+06 alpha_ob4_seg0[3] <= 1.000040282e+06
 R47: C3 + 1e+06 alpha_ob4_seg0[3] <= 1.000040282e+06
 R48: alpha_ob4_seg0[0] + alpha_ob4_seg0[1] + alpha_ob4_seg0[2]
   + alpha_ob4_seg0[3] >= 1
 R49: - C0 + 1e+06 alpha_ob5_seg0[0] <= 999992.49102
 R50: - C2 + 1e+06 alpha_ob5_seg0[0] <= 999992.49102
 R51: - C1 + 1e+06 alpha_ob5_seg0[1] <= 999957.1016000001
 R52: - C3 + 1e+06 alpha_ob5_seg0[1] <= 999957.1016000001
 R53: C0 + 1e+06 alpha_ob5_seg0[2] <= 999986.35898
 R54: C2 + 1e+06 alpha_ob5_seg0[2] <= 999986.35898
 R55: C1 + 1e+06 alpha_ob5_seg0[3] <= 1.0000403484e+06
 R56: C3 + 1e+06 alpha_ob5_seg0[3] <= 1.0000403484e+06
 R57: alpha_ob5_seg0[0] + alpha_ob5_seg0[1] + alpha_ob5_seg0[2]
   + alpha_ob5_seg0[3] >= 1
 R58: - C0 + 1e+06 alpha_ob6_seg0[0] <= 999981.3909999999
 R59: - C2 + 1e+06 alpha_ob6_seg0[0] <= 999981.3909999999
 R60: - C1 + 1e+06 alpha_ob6_seg0[1] <= 999957.1016000001
 R61: - C3 + 1e+06 alpha_ob6_seg0[1] <= 999957.1016000001
 R62: C0 + 1e+06 alpha_ob6_seg0[2] <= 1.000004959e+06
 R63: C2 + 1e+06 alpha_ob6_seg0[2] <= 1.000004959e+06
 R64: C1 + 1e+06 alpha_ob6_seg0[3] <= 1.0000403484e+06
 R65: C3 + 1e+06 alpha_ob6_seg0[3] <= 1.0000403484e+06
 R66: alpha_ob6_seg0[0] + alpha_ob6_seg0[1] + alpha_ob6_seg0[2]
   + alpha_ob6_seg0[3] >= 1
 R67: - C0 + 1e+06 alpha_ob7_seg0[0] <= 999981.3909825523
 R68: - C2 + 1e+06 alpha_ob7_seg0[0] <= 999981.3909825523
 R69: - C1 + 1e+06 alpha_ob7_seg0[1] <= 999957.1016002755
 R70: - C3 + 1e+06 alpha_ob7_seg0[1] <= 999957.1016002755
 R71: C0 + 1e+06 alpha_ob7_seg0[2] <= 1.0000160589825523e+06
 R72: C2 + 1e+06 alpha_ob7_seg0[2] <= 1.0000160589825523e+06
 R73: C1 + 1e+06 alpha_ob7_seg0[3] <= 1.0000309984002755e+06
 R74: C3 + 1e+06 alpha_ob7_seg0[3] <= 1.0000309984002755e+06
 R75: alpha_ob7_seg0[0] + alpha_ob7_seg0[1] + alpha_ob7_seg0[2]
   + alpha_ob7_seg0[3] >= 1
 R76: - C0 + 1e+06 alpha_ob8_seg0[0] <= 1.0000537118917353e+06
 R77: - C2 + 1e+06 alpha_ob8_seg0[0] <= 1.0000537118917353e+06
 R78: - C1 + 1e+06 alpha_ob8_seg0[1] <= 999947.6502002755
 R79: - C3 + 1e+06 alpha_ob8_seg0[1] <= 999947.6502002755
 R80: C0 + 1e+06 alpha_ob8_seg0[2] <= 999943.7380917353
 R81: C2 + 1e+06 alpha_ob8_seg0[2] <= 999943.7380917353
 R82: C1 + 1e+06 alpha_ob8_seg0[3] <= 1.0000454498002755e+06
 R83: C3 + 1e+06 alpha_ob8_seg0[3] <= 1.0000454498002755e+06
 R84: alpha_ob8_seg0[0] + alpha_ob8_seg0[1] + alpha_ob8_seg0[2]
   + alpha_ob8_seg0[3] >= 1
 R85: - C0 + 1e+06 alpha_ob9_seg0[0] <= 1.0000040868238758e+06
 R86: - C2 + 1e+06 alpha_ob9_seg0[0] <= 1.0000040868238758e+06
 R87: - C1 + 1e+06 alpha_ob9_seg0[1] <= 999957.2594997246
 R88: - C3 + 1e+06 alpha_ob9_seg0[1] <= 999957.2594997246
 R89: C0 + 1e+06 alpha_ob9_seg0[2] <= 999993.3632238759
 R90: C2 + 1e+06 alpha_ob9_seg0[2] <= 999993.3632238759
 R91: C1 + 1e+06 alpha_ob9_seg0[3] <= 1.0000273404997246e+06
 R92: C3 + 1e+06 alpha_ob9_seg0[3] <= 1.0000273404997246e+06
 R93: alpha_ob9_seg0[0] + alpha_ob9_seg0[1] + alpha_ob9_seg0[2]
   + alpha_ob9_seg0[3] >= 1
 R94: - C0 + 1e+06 alpha_ob10_seg0[0] <= 1.0000531119e+06
 R95: - C2 + 1e+06 alpha_ob10_seg0[0] <= 1.0000531119e+06
 R96: - C1 + 1e+06 alpha_ob10_seg0[1] <= 999952.0000999999
 R97: - C3 + 1e+06 alpha_ob10_seg0[1] <= 999952.0000999999
 R98: C0 + 1e+06 alpha_ob10_seg0[2] <= 999943.7381
 R99: C2 + 1e+06 alpha_ob10_seg0[2] <= 999943.7381
 R100: C1 + 1e+06 alpha_ob10_seg0[3] <= 1.0000454499e+06
 R101: C3 + 1e+06 alpha_ob10_seg0[3] <= 1.0000454499e+06
 R102: alpha_ob10_seg0[0] + alpha_ob10_seg0[1] + alpha_ob10_seg0[2]
   + alpha_ob10_seg0[3] >= 1
 R103: - C0 + 1e+06 alpha_ob11_seg0[0] <= 999948.0333
 R104: - C2 + 1e+06 alpha_ob11_seg0[0] <= 999948.0333
 R105: - C1 + 1e+06 alpha_ob11_seg0[1] <= 1.0000686083e+06
 R106: - C3 + 1e+06 alpha_ob11_seg0[1] <= 1.0000686083e+06
 R107: C0 + 1e+06 alpha_ob11_seg0[2] <= 999920.0667
 R108: C2 + 1e+06 alpha_ob11_seg0[2] <= 999920.0667
 R109: C1 + 1e+06 alpha_ob11_seg0[3] <= 999928.8417
 R110: C3 + 1e+06 alpha_ob11_seg0[3] <= 999928.8417
 R111: alpha_ob11_seg0[0] + alpha_ob11_seg0[1] + alpha_ob11_seg0[2]
   + alpha_ob11_seg0[3] >= 1
 R112: - C0 + 1e+06 alpha_ob12_seg0[0] <= 1.0000490237e+06
 R113: - C2 + 1e+06 alpha_ob12_seg0[0] <= 1.0000490237e+06
 R114: - C1 + 1e+06 alpha_ob12_seg0[1] <= 999940.7624
 R115: - C3 + 1e+06 alpha_ob12_seg0[1] <= 999940.7624
 R116: C0 + 1e+06 alpha_ob12_seg0[2] <= 999947.3263
 R117: C2 + 1e+06 alpha_ob12_seg0[2] <= 999947.3263
 R118: C1 + 1e+06 alpha_ob12_seg0[3] <= 1.0000566875999999e+06
 R119: C3 + 1e+06 alpha_ob12_seg0[3] <= 1.0000566875999999e+06
 R120: alpha_ob12_seg0[0] + alpha_ob12_seg0[1] + alpha_ob12_seg0[2]
   + alpha_ob12_seg0[3] >= 1
 R121: - C0 + 1e+06 alpha_ob13_seg0[0] <= 999915.9461796245
 R122: - C2 + 1e+06 alpha_ob13_seg0[0] <= 999915.9461796245
 R123: - C1 + 1e+06 alpha_ob13_seg0[1] <= 1.0000683610286588e+06
 R124: - C3 + 1e+06 alpha_ob13_seg0[1] <= 1.0000683610286588e+06
 R125: C0 + 1e+06 alpha_ob13_seg0[2] <= 1.0000495659796246e+06
 R126: C2 + 1e+06 alpha_ob13_seg0[2] <= 1.0000495659796246e+06
 R127: C1 + 1e+06 alpha_ob13_seg0[3] <= 999928.8376286589
 R128: C3 + 1e+06 alpha_ob13_seg0[3] <= 999928.8376286589
 R129: alpha_ob13_seg0[0] + alpha_ob13_seg0[1] + alpha_ob13_seg0[2]
   + alpha_ob13_seg0[3] >= 1
 R130: - C0 + 1e+06 alpha_ob14_seg0[0] <= 1.000049023678879e+06
 R131: - C2 + 1e+06 alpha_ob14_seg0[0] <= 1.000049023678879e+06
 R132: - C1 + 1e+06 alpha_ob14_seg0[1] <= 999940.7624002756
 R133: - C3 + 1e+06 alpha_ob14_seg0[1] <= 999940.7624002756
 R134: C0 + 1e+06 alpha_ob14_seg0[2] <= 999948.4262788791
 R135: C2 + 1e+06 alpha_ob14_seg0[2] <= 999948.4262788791
 R136: C1 + 1e+06 alpha_ob14_seg0[3] <= 1.0000453376002755e+06
 R137: C3 + 1e+06 alpha_ob14_seg0[3] <= 1.0000453376002755e+06
 R138: alpha_ob14_seg0[0] + alpha_ob14_seg0[1] + alpha_ob14_seg0[2]
   + alpha_ob14_seg0[3] >= 1
 R139: - C0 + 1e+06 alpha_ob15_seg0[0] <= 1.0000066737e+06
 R140: - C2 + 1e+06 alpha_ob15_seg0[0] <= 1.0000066737e+06
 R141: - C1 + 1e+06 alpha_ob15_seg0[1] <= 999952.1958
 R142: - C3 + 1e+06 alpha_ob15_seg0[1] <= 999952.1958
 R143: C0 + 1e+06 alpha_ob15_seg0[2] <= 999948.4263000001
 R144: C2 + 1e+06 alpha_ob15_seg0[2] <= 999948.4263000001
 R145: C1 + 1e+06 alpha_ob15_seg0[3] <= 1.0000452542e+06
 R146: C3 + 1e+06 alpha_ob15_seg0[3] <= 1.0000452542e+06
 R147: alpha_ob15_seg0[0] + alpha_ob15_seg0[1] + alpha_ob15_seg0[2]
   + alpha_ob15_seg0[3] >= 1
 R148: - C0 + 1e+06 alpha_ob16_seg0[0] <= 999991.073732
 R149: - C2 + 1e+06 alpha_ob16_seg0[0] <= 999991.073732
 R150: - C1 + 1e+06 alpha_ob16_seg0[1] <= 999952.2125
 R151: - C3 + 1e+06 alpha_ob16_seg0[1] <= 999952.2125
 R152: C0 + 1e+06 alpha_ob16_seg0[2] <= 999990.776268
 R153: C2 + 1e+06 alpha_ob16_seg0[2] <= 999990.776268
 R154: C1 + 1e+06 alpha_ob16_seg0[3] <= 1.0000452375e+06
 R155: C3 + 1e+06 alpha_ob16_seg0[3] <= 1.0000452375e+06
 R156: alpha_ob16_seg0[0] + alpha_ob16_seg0[1] + alpha_ob16_seg0[2]
   + alpha_ob16_seg0[3] >= 1
 R157: - C0 + 1e+06 alpha_ob17_seg0[0] <= 1.0000837618494935e+06
 R158: - C2 + 1e+06 alpha_ob17_seg0[0] <= 1.0000837618494935e+06
 R159: - C1 + 1e+06 alpha_ob17_seg0[1] <= 999925.5501002755
 R160: - C3 + 1e+06 alpha_ob17_seg0[1] <= 999925.5501002755
 R161: C0 + 1e+06 alpha_ob17_seg0[2] <= 999913.6880494935
 R162: C2 + 1e+06 alpha_ob17_seg0[2] <= 999913.6880494935
 R163: C1 + 1e+06 alpha_ob17_seg0[3] <= 1.0000445499002755e+06
 R164: C3 + 1e+06 alpha_ob17_seg0[3] <= 1.0000445499002755e+06
 R165: alpha_ob17_seg0[0] + alpha_ob17_seg0[1] + alpha_ob17_seg0[2]
   + alpha_ob17_seg0[3] >= 1
 R166: - C0 + 1e+06 alpha_ob18_seg0[0] <= 999956.8394000001
 R167: - C2 + 1e+06 alpha_ob18_seg0[0] <= 999956.8394000001
 R168: - C1 + 1e+06 alpha_ob18_seg0[1] <= 999952.2463
 R169: - C3 + 1e+06 alpha_ob18_seg0[1] <= 999952.2463
 R170: C0 + 1e+06 alpha_ob18_seg0[2] <= 1.0000097606e+06
 R171: C2 + 1e+06 alpha_ob18_seg0[2] <= 1.0000097606e+06
 R172: C1 + 1e+06 alpha_ob18_seg0[3] <= 1.0000452037e+06
 R173: C3 + 1e+06 alpha_ob18_seg0[3] <= 1.0000452037e+06
 R174: alpha_ob18_seg0[0] + alpha_ob18_seg0[1] + alpha_ob18_seg0[2]
   + alpha_ob18_seg0[3] >= 1
 R175: - C0 + 1e+06 alpha_ob19_seg0[0] <= 999964.9321250001
 R176: - C2 + 1e+06 alpha_ob19_seg0[0] <= 999964.9321250001
 R177: - C1 + 1e+06 alpha_ob19_seg0[1] <= 1.0000141893e+06
 R178: - C3 + 1e+06 alpha_ob19_seg0[1] <= 1.0000141893e+06
 R179: C0 + 1e+06 alpha_ob19_seg0[2] <= 1.000025197325e+06
 R180: C2 + 1e+06 alpha_ob19_seg0[2] <= 1.000025197325e+06
 R181: C1 + 1e+06 alpha_ob19_seg0[3] <= 999983.2607
 R182: C3 + 1e+06 alpha_ob19_seg0[3] <= 999983.2607
 R183: alpha_ob19_seg0[0] + alpha_ob19_seg0[1] + alpha_ob19_seg0[2]
   + alpha_ob19_seg0[3] >= 1
 R184: - C0 + 1e+06 alpha_ob20_seg0[0] <= 999964.922208629
 R185: - C2 + 1e+06 alpha_ob20_seg0[0] <= 999964.922208629
 R186: - C1 + 1e+06 alpha_ob20_seg0[1] <= 1.0000141893002756e+06
 R187: - C3 + 1e+06 alpha_ob20_seg0[1] <= 1.0000141893002756e+06
 R188: C0 + 1e+06 alpha_ob20_seg0[2] <= 1.000032527608629e+06
 R189: C2 + 1e+06 alpha_ob20_seg0[2] <= 1.000032527608629e+06
 R190: C1 + 1e+06 alpha_ob20_seg0[3] <= 999933.6607002757
 R191: C3 + 1e+06 alpha_ob20_seg0[3] <= 999933.6607002757
 R192: alpha_ob20_seg0[0] + alpha_ob20_seg0[1] + alpha_ob20_seg0[2]
   + alpha_ob20_seg0[3] >= 1
 R193: - C0 + 1e+06 alpha_ob21_seg0[0] <= 999964.922299801
 R194: - C2 + 1e+06 alpha_ob21_seg0[0] <= 999964.922299801
 R195: - C1 + 1e+06 alpha_ob21_seg0[1] <= 1.000063789309951e+06
 R196: - C3 + 1e+06 alpha_ob21_seg0[1] <= 1.000063789309951e+06
 R197: C0 + 1e+06 alpha_ob21_seg0[2] <= 1.000025177699801e+06
 R198: C2 + 1e+06 alpha_ob21_seg0[2] <= 1.000025177699801e+06
 R199: C1 + 1e+06 alpha_ob21_seg0[3] <= 999933.660709951
 R200: C3 + 1e+06 alpha_ob21_seg0[3] <= 999933.660709951
 R201: alpha_ob21_seg0[0] + alpha_ob21_seg0[1] + alpha_ob21_seg0[2]
   + alpha_ob21_seg0[3] >= 1
 R202: - C0 + 1e+06 alpha_ob22_seg0[0] <= 999965.1781
 R203: - C2 + 1e+06 alpha_ob22_seg0[0] <= 999965.1781
 R204: - C1 + 1e+06 alpha_ob22_seg0[1] <= 1.0000488976e+06
 R205: - C3 + 1e+06 alpha_ob22_seg0[1] <= 1.0000488976e+06
 R206: C0 + 1e+06 alpha_ob22_seg0[2] <= 1.0000254219e+06
 R207: C2 + 1e+06 alpha_ob22_seg0[2] <= 1.0000254219e+06
 R208: C1 + 1e+06 alpha_ob22_seg0[3] <= 999948.5524
 R209: C3 + 1e+06 alpha_ob22_seg0[3] <= 999948.5524
 R210: alpha_ob22_seg0[0] + alpha_ob22_seg0[1] + alpha_ob22_seg0[2]
   + alpha_ob22_seg0[3] >= 1
 R211: - C0 + 1e+06 alpha_ob23_seg0[0] <= 999965.0925
 R212: - C2 + 1e+06 alpha_ob23_seg0[0] <= 999965.0925
 R213: - C1 + 1e+06 alpha_ob23_seg0[1] <= 1.0000418517999999e+06
 R214: - C3 + 1e+06 alpha_ob23_seg0[1] <= 1.0000418517999999e+06
 R215: C0 + 1e+06 alpha_ob23_seg0[2] <= 1.0000250074999999e+06
 R216: C2 + 1e+06 alpha_ob23_seg0[2] <= 1.0000250074999999e+06
 R217: C1 + 1e+06 alpha_ob23_seg0[3] <= 999955.5982
 R218: C3 + 1e+06 alpha_ob23_seg0[3] <= 999955.5982
 R219: alpha_ob23_seg0[0] + alpha_ob23_seg0[1] + alpha_ob23_seg0[2]
   + alpha_ob23_seg0[3] >= 1
 R220: - C0 + 1e+06 alpha_ob24_seg0[0] <= 999964.9807
 R221: - C2 + 1e+06 alpha_ob24_seg0[0] <= 999964.9807
 R222: - C1 + 1e+06 alpha_ob24_seg0[1] <= 1.0000364639e+06
 R223: - C3 + 1e+06 alpha_ob24_seg0[1] <= 1.0000364639e+06
 R224: C0 + 1e+06 alpha_ob24_seg0[2] <= 1.0000248693e+06
 R225: C2 + 1e+06 alpha_ob24_seg0[2] <= 1.0000248693e+06
 R226: C1 + 1e+06 alpha_ob24_seg0[3] <= 999960.9861
 R227: C3 + 1e+06 alpha_ob24_seg0[3] <= 999960.9861
 R228: alpha_ob24_seg0[0] + alpha_ob24_seg0[1] + alpha_ob24_seg0[2]
   + alpha_ob24_seg0[3] >= 1
 R229: - C0 + 1e+06 alpha_ob25_seg0[0] <= 999965.0662
 R230: - C2 + 1e+06 alpha_ob25_seg0[0] <= 999965.0662
 R231: - C1 + 1e+06 alpha_ob25_seg0[1] <= 1.0000301090000001e+06
 R232: - C3 + 1e+06 alpha_ob25_seg0[1] <= 1.0000301090000001e+06
 R233: C0 + 1e+06 alpha_ob25_seg0[2] <= 1.0000252838e+06
 R234: C2 + 1e+06 alpha_ob25_seg0[2] <= 1.0000252838e+06
 R235: C1 + 1e+06 alpha_ob25_seg0[3] <= 999967.341
 R236: C3 + 1e+06 alpha_ob25_seg0[3] <= 999967.341
 R237: alpha_ob25_seg0[0] + alpha_ob25_seg0[1] + alpha_ob25_seg0[2]
   + alpha_ob25_seg0[3] >= 1
 R238: - C0 + 1e+06 alpha_ob26_seg0[0] <= 999964.9544
 R239: - C2 + 1e+06 alpha_ob26_seg0[0] <= 999964.9544
 R240: - C1 + 1e+06 alpha_ob26_seg0[1] <= 1.0000233395999999e+06
 R241: - C3 + 1e+06 alpha_ob26_seg0[1] <= 1.0000233395999999e+06
 R242: C0 + 1e+06 alpha_ob26_seg0[2] <= 1.0000251456e+06
 R243: C2 + 1e+06 alpha_ob26_seg0[2] <= 1.0000251456e+06
 R244: C1 + 1e+06 alpha_ob26_seg0[3] <= 999974.1104
 R245: C3 + 1e+06 alpha_ob26_seg0[3] <= 999974.1104
 R246: alpha_ob26_seg0[0] + alpha_ob26_seg0[1] + alpha_ob26_seg0[2]
   + alpha_ob26_seg0[3] >= 1
 R247: - C0 + 1e+06 alpha_ob27_seg0[0] <= 999931.4945
 R248: - C2 + 1e+06 alpha_ob27_seg0[0] <= 999931.4945
 R249: - C1 + 1e+06 alpha_ob27_seg0[1] <= 1.0000158794e+06
 R250: - C3 + 1e+06 alpha_ob27_seg0[1] <= 1.0000158794e+06
 R251: C0 + 1e+06 alpha_ob27_seg0[2] <= 1.0000378554999999e+06
 R252: C2 + 1e+06 alpha_ob27_seg0[2] <= 1.0000378554999999e+06
 R253: C1 + 1e+06 alpha_ob27_seg0[3] <= 999981.5706
 R254: C3 + 1e+06 alpha_ob27_seg0[3] <= 999981.5706
 R255: alpha_ob27_seg0[0] + alpha_ob27_seg0[1] + alpha_ob27_seg0[2]
   + alpha_ob27_seg0[3] >= 1
 R256: - C0 + 1e+06 alpha_ob28_seg0[0] <= 999953.8919709784
 R257: - C2 + 1e+06 alpha_ob28_seg0[0] <= 999953.8919709784
 R258: - C1 + 1e+06 alpha_ob28_seg0[1] <= 1.0000230754474958e+06
 R259: - C3 + 1e+06 alpha_ob28_seg0[1] <= 1.0000230754474958e+06
 R260: C0 + 1e+06 alpha_ob28_seg0[2] <= 1.0000383703709784e+06
 R261: C2 + 1e+06 alpha_ob28_seg0[2] <= 1.0000383703709784e+06
 R262: C1 + 1e+06 alpha_ob28_seg0[3] <= 999971.2696474958
 R263: C3 + 1e+06 alpha_ob28_seg0[3] <= 999971.2696474958
 R264: alpha_ob28_seg0[0] + alpha_ob28_seg0[1] + alpha_ob28_seg0[2]
   + alpha_ob28_seg0[3] >= 1
 R265: - C0 + 1e+06 alpha_ob29_seg0[0] <= 999947.4507336203
 R266: - C2 + 1e+06 alpha_ob29_seg0[0] <= 999947.4507336203
 R267: - C1 + 1e+06 alpha_ob29_seg0[1] <= 1.000021362399544e+06
 R268: - C3 + 1e+06 alpha_ob29_seg0[1] <= 1.000021362399544e+06
 R269: C0 + 1e+06 alpha_ob29_seg0[2] <= 1.0000443925336204e+06
 R270: C2 + 1e+06 alpha_ob29_seg0[2] <= 1.0000443925336204e+06
 R271: C1 + 1e+06 alpha_ob29_seg0[3] <= 999974.5397995439
 R272: C3 + 1e+06 alpha_ob29_seg0[3] <= 999974.5397995439
 R273: alpha_ob29_seg0[0] + alpha_ob29_seg0[1] + alpha_ob29_seg0[2]
   + alpha_ob29_seg0[3] >= 1
 R274: - C0 + 1e+06 alpha_ob30_seg0[0] <= 999956.8394495883
 R275: - C2 + 1e+06 alpha_ob30_seg0[0] <= 999956.8394495883
 R276: - C1 + 1e+06 alpha_ob30_seg0[1] <= 999925.3962997246
 R277: - C3 + 1e+06 alpha_ob30_seg0[1] <= 999925.3962997246
 R278: C0 + 1e+06 alpha_ob30_seg0[2] <= 1.0000406106495883e+06
 R279: C2 + 1e+06 alpha_ob30_seg0[2] <= 1.0000406106495883e+06
 R280: C1 + 1e+06 alpha_ob30_seg0[3] <= 1.0000452036997246e+06
 R281: C3 + 1e+06 alpha_ob30_seg0[3] <= 1.0000452036997246e+06
 R282: alpha_ob30_seg0[0] + alpha_ob30_seg0[1] + alpha_ob30_seg0[2]
   + alpha_ob30_seg0[3] >= 1
 R283: - C0 + 1e+06 alpha_ob31_seg0[0] <= 999941.1221928794
 R284: - C2 + 1e+06 alpha_ob31_seg0[0] <= 999941.1221928794
 R285: - C1 + 1e+06 alpha_ob31_seg0[1] <= 1.0000191770472102e+06
 R286: - C3 + 1e+06 alpha_ob31_seg0[1] <= 1.0000191770472102e+06
 R287: C0 + 1e+06 alpha_ob31_seg0[2] <= 1.0000504795928795e+06
 R288: C2 + 1e+06 alpha_ob31_seg0[2] <= 1.0000504795928795e+06
 R289: C1 + 1e+06 alpha_ob31_seg0[3] <= 999976.6604472102
 R290: C3 + 1e+06 alpha_ob31_seg0[3] <= 999976.6604472102
 R291: alpha_ob31_seg0[0] + alpha_ob31_seg0[1] + alpha_ob31_seg0[2]
   + alpha_ob31_seg0[3] >= 1
 R292: - C0 + 1e+06 alpha_ob32_seg0[0] <= 999934.8446336203
 R293: - C2 + 1e+06 alpha_ob32_seg0[0] <= 999934.8446336203
 R294: - C1 + 1e+06 alpha_ob32_seg0[1] <= 1.000021048199544e+06
 R295: - C3 + 1e+06 alpha_ob32_seg0[1] <= 1.000021048199544e+06
 R296: C0 + 1e+06 alpha_ob32_seg0[2] <= 1.0000569986336203e+06
 R297: C2 + 1e+06 alpha_ob32_seg0[2] <= 1.0000569986336203e+06
 R298: C1 + 1e+06 alpha_ob32_seg0[3] <= 999974.8539995439
 R299: C3 + 1e+06 alpha_ob32_seg0[3] <= 999974.8539995439
 R300: alpha_ob32_seg0[0] + alpha_ob32_seg0[1] + alpha_ob32_seg0[2]
   + alpha_ob32_seg0[3] >= 1
 R301: - C0 + 1e+06 alpha_ob33_seg0[0] <= 999956.839399801
 R302: - C2 + 1e+06 alpha_ob33_seg0[0] <= 999956.839399801
 R303: - C1 + 1e+06 alpha_ob33_seg0[1] <= 999925.3963434525
 R304: - C3 + 1e+06 alpha_ob33_seg0[1] <= 999925.3963434525
 R305: C0 + 1e+06 alpha_ob33_seg0[2] <= 1.0000080105998011e+06
 R306: C2 + 1e+06 alpha_ob33_seg0[2] <= 1.0000080105998011e+06
 R307: C1 + 1e+06 alpha_ob33_seg0[3] <= 1.0000720537434525e+06
 R308: C3 + 1e+06 alpha_ob33_seg0[3] <= 1.0000720537434525e+06
 R309: alpha_ob33_seg0[0] + alpha_ob33_seg0[1] + alpha_ob33_seg0[2]
   + alpha_ob33_seg0[3] >= 1
 R310: - C0 + 1e+06 alpha_ob34_seg0[0] <= 999958.8717898987
 R311: - C2 + 1e+06 alpha_ob34_seg0[0] <= 999958.8717898987
 R312: - C1 + 1e+06 alpha_ob34_seg0[1] <= 999987.7837602755
 R313: - C3 + 1e+06 alpha_ob34_seg0[1] <= 999987.7837602755
 R314: C0 + 1e+06 alpha_ob34_seg0[2] <= 1.0000385781898986e+06
 R315: C2 + 1e+06 alpha_ob34_seg0[2] <= 1.0000385781898986e+06
 R316: C1 + 1e+06 alpha_ob34_seg0[3] <= 1.0000043162402755e+06
 R317: C3 + 1e+06 alpha_ob34_seg0[3] <= 1.0000043162402755e+06
 R318: alpha_ob34_seg0[0] + alpha_ob34_seg0[1] + alpha_ob34_seg0[2]
   + alpha_ob34_seg0[3] >= 1
 R319: - C0 + 1e+06 alpha_ob35_seg0[0] <= 999939.2718
 R320: - C2 + 1e+06 alpha_ob35_seg0[0] <= 999939.2718
 R321: - C1 + 1e+06 alpha_ob35_seg0[1] <= 999993.13376
 R322: - C3 + 1e+06 alpha_ob35_seg0[1] <= 999993.13376
 R323: C0 + 1e+06 alpha_ob35_seg0[2] <= 1.0000385782e+06
 R324: C2 + 1e+06 alpha_ob35_seg0[2] <= 1.0000385782e+06
 R325: C1 + 1e+06 alpha_ob35_seg0[3] <= 1.00000431624e+06
 R326: C3 + 1e+06 alpha_ob35_seg0[3] <= 1.00000431624e+06
 R327: alpha_ob35_seg0[0] + alpha_ob35_seg0[1] + alpha_ob35_seg0[2]
   + alpha_ob35_seg0[3] >= 1
 R328: - C0 + 1e+06 alpha_ob36_seg0[0] <= 999939.2718101013
 R329: - C2 + 1e+06 alpha_ob36_seg0[0] <= 999939.2718101013
 R330: - C1 + 1e+06 alpha_ob36_seg0[1] <= 999987.7837597246
 R331: - C3 + 1e+06 alpha_ob36_seg0[1] <= 999987.7837597246
 R332: C0 + 1e+06 alpha_ob36_seg0[2] <= 1.0000581782101013e+06
 R333: C2 + 1e+06 alpha_ob36_seg0[2] <= 1.0000581782101013e+06
 R334: C1 + 1e+06 alpha_ob36_seg0[3] <= 1.0000043162397245e+06
 R335: C3 + 1e+06 alpha_ob36_seg0[3] <= 1.0000043162397245e+06
 R336: alpha_ob36_seg0[0] + alpha_ob36_seg0[1] + alpha_ob36_seg0[2]
   + alpha_ob36_seg0[3] >= 1
 R337: - C0 + 1e+06 alpha_ob37_seg0[0] <= 999939.271799801
 R338: - C2 + 1e+06 alpha_ob37_seg0[0] <= 999939.271799801
 R339: - C1 + 1e+06 alpha_ob37_seg0[1] <= 999987.7838262042
 R340: - C3 + 1e+06 alpha_ob37_seg0[1] <= 999987.7838262042
 R341: C0 + 1e+06 alpha_ob37_seg0[2] <= 1.000038578199801e+06
 R342: C2 + 1e+06 alpha_ob37_seg0[2] <= 1.000038578199801e+06
 R343: C1 + 1e+06 alpha_ob37_seg0[3] <= 1.0000096662262043e+06
 R344: C3 + 1e+06 alpha_ob37_seg0[3] <= 1.0000096662262043e+06
 R345: alpha_ob37_seg0[0] + alpha_ob37_seg0[1] + alpha_ob37_seg0[2]
   + alpha_ob37_seg0[3] >= 1
 R346: - C0 + 1e+06 alpha_ob38_seg0[0] <= 999972.6917508709
 R347: - C2 + 1e+06 alpha_ob38_seg0[0] <= 999972.6917508709
 R348: - C1 + 1e+06 alpha_ob38_seg0[1] <= 999957.1644002756
 R349: - C3 + 1e+06 alpha_ob38_seg0[1] <= 999957.1644002756
 R350: C0 + 1e+06 alpha_ob38_seg0[2] <= 1.0000247581508709e+06
 R351: C2 + 1e+06 alpha_ob38_seg0[2] <= 1.0000247581508709e+06
 R352: C1 + 1e+06 alpha_ob38_seg0[3] <= 1.0000136856002755e+06
 R353: C3 + 1e+06 alpha_ob38_seg0[3] <= 1.0000136856002755e+06
 R354: alpha_ob38_seg0[0] + alpha_ob38_seg0[1] + alpha_ob38_seg0[2]
   + alpha_ob38_seg0[3] >= 1
 R355: - C0 + 1e+06 alpha_ob39_seg0[0] <= 999925.8418000001
 R356: - C2 + 1e+06 alpha_ob39_seg0[0] <= 999925.8418000001
 R357: - C1 + 1e+06 alpha_ob39_seg0[1] <= 999957.1644
 R358: - C3 + 1e+06 alpha_ob39_seg0[1] <= 999957.1644
 R359: C0 + 1e+06 alpha_ob39_seg0[2] <= 1.0000247582e+06
 R360: C2 + 1e+06 alpha_ob39_seg0[2] <= 1.0000247582e+06
 R361: C1 + 1e+06 alpha_ob39_seg0[3] <= 1.0000402855999999e+06
 R362: C3 + 1e+06 alpha_ob39_seg0[3] <= 1.0000402855999999e+06
 R363: alpha_ob39_seg0[0] + alpha_ob39_seg0[1] + alpha_ob39_seg0[2]
   + alpha_ob39_seg0[3] >= 1
 R364: - C0 + 1e+06 alpha_ob40_seg0[0] <= 999925.8417898987
 R365: - C2 + 1e+06 alpha_ob40_seg0[0] <= 999925.8417898987
 R366: - C1 + 1e+06 alpha_ob40_seg0[1] <= 999957.1644002756
 R367: - C3 + 1e+06 alpha_ob40_seg0[1] <= 999957.1644002756
 R368: C0 + 1e+06 alpha_ob40_seg0[2] <= 1.0000716081898987e+06
 R369: C2 + 1e+06 alpha_ob40_seg0[2] <= 1.0000716081898987e+06
 R370: C1 + 1e+06 alpha_ob40_seg0[3] <= 1.0000349356002755e+06
 R371: C3 + 1e+06 alpha_ob40_seg0[3] <= 1.0000349356002755e+06
 R372: alpha_ob40_seg0[0] + alpha_ob40_seg0[1] + alpha_ob40_seg0[2]
   + alpha_ob40_seg0[3] >= 1
 R373: - C0 + 1e+06 alpha_ob41_seg0[0] <= 999939.0009674003
 R374: - C2 + 1e+06 alpha_ob41_seg0[0] <= 999939.0009674003
 R375: - C1 + 1e+06 alpha_ob41_seg0[1] <= 999961.9516002756
 R376: - C3 + 1e+06 alpha_ob41_seg0[1] <= 999961.9516002756
 R377: C0 + 1e+06 alpha_ob41_seg0[2] <= 1.0000584489674004e+06
 R378: C2 + 1e+06 alpha_ob41_seg0[2] <= 1.0000584489674004e+06
 R379: C1 + 1e+06 alpha_ob41_seg0[3] <= 1.0000178984002755e+06
 R380: C3 + 1e+06 alpha_ob41_seg0[3] <= 1.0000178984002755e+06
 R381: alpha_ob41_seg0[0] + alpha_ob41_seg0[1] + alpha_ob41_seg0[2]
   + alpha_ob41_seg0[3] >= 1
 R382: - C0 + 1e+06 alpha_ob42_seg0[0] <= 999925.901
 R383: - C2 + 1e+06 alpha_ob42_seg0[0] <= 999925.901
 R384: - C1 + 1e+06 alpha_ob42_seg0[1] <= 999961.9516
 R385: - C3 + 1e+06 alpha_ob42_seg0[1] <= 999961.9516
 R386: C0 + 1e+06 alpha_ob42_seg0[2] <= 1.000058449e+06
 R387: C2 + 1e+06 alpha_ob42_seg0[2] <= 1.000058449e+06
 R388: C1 + 1e+06 alpha_ob42_seg0[3] <= 1.0000354984e+06
 R389: C3 + 1e+06 alpha_ob42_seg0[3] <= 1.0000354984e+06
 R390: alpha_ob42_seg0[0] + alpha_ob42_seg0[1] + alpha_ob42_seg0[2]
   + alpha_ob42_seg0[3] >= 1
 R391: - C0 + 1e+06 alpha_ob43_seg0[0] <= 999919.2652454559
 R392: - C2 + 1e+06 alpha_ob43_seg0[0] <= 999919.2652454559
 R393: - C1 + 1e+06 alpha_ob43_seg0[1] <= 999956.8341997246
 R394: - C3 + 1e+06 alpha_ob43_seg0[1] <= 999956.8341997246
 R395: C0 + 1e+06 alpha_ob43_seg0[2] <= 1.0000781848454559e+06
 R396: C2 + 1e+06 alpha_ob43_seg0[2] <= 1.0000781848454559e+06
 R397: C1 + 1e+06 alpha_ob43_seg0[3] <= 1.0000160157997246e+06
 R398: C3 + 1e+06 alpha_ob43_seg0[3] <= 1.0000160157997246e+06
 R399: alpha_ob43_seg0[0] + alpha_ob43_seg0[1] + alpha_ob43_seg0[2]
   + alpha_ob43_seg0[3] >= 1
 R400: - C0 + 1e+06 alpha_ob44_seg0[0] <= 1.0000699118999999e+06
 R401: - C2 + 1e+06 alpha_ob44_seg0[0] <= 1.0000699118999999e+06
 R402: - C1 + 1e+06 alpha_ob44_seg0[1] <= 999952.9001
 R403: - C3 + 1e+06 alpha_ob44_seg0[1] <= 999952.9001
 R404: C0 + 1e+06 alpha_ob44_seg0[2] <= 999913.6881
 R405: C2 + 1e+06 alpha_ob44_seg0[2] <= 999913.6881
 R406: C1 + 1e+06 alpha_ob44_seg0[3] <= 1.0000445499e+06
 R407: C3 + 1e+06 alpha_ob44_seg0[3] <= 1.0000445499e+06
 R408: alpha_ob44_seg0[0] + alpha_ob44_seg0[1] + alpha_ob44_seg0[2]
   + alpha_ob44_seg0[3] >= 1
 R409: - C0 + 1e+06 alpha_ob45_seg0[0] <= 999919.3751998009
 R410: - C2 + 1e+06 alpha_ob45_seg0[0] <= 999919.3751998009
 R411: - C1 + 1e+06 alpha_ob45_seg0[1] <= 999957.1154039804
 R412: - C3 + 1e+06 alpha_ob45_seg0[1] <= 999957.1154039804
 R413: C0 + 1e+06 alpha_ob45_seg0[2] <= 1.000075224799801e+06
 R414: C2 + 1e+06 alpha_ob45_seg0[2] <= 1.000075224799801e+06
 R415: C1 + 1e+06 alpha_ob45_seg0[3] <= 1.0000403346039804e+06
 R416: C3 + 1e+06 alpha_ob45_seg0[3] <= 1.0000403346039804e+06
 R417: alpha_ob45_seg0[0] + alpha_ob45_seg0[1] + alpha_ob45_seg0[2]
   + alpha_ob45_seg0[3] >= 1
 R418: - C0 + 1e+06 alpha_ob46_seg0[0] <= 1.0000668119e+06
 R419: - C2 + 1e+06 alpha_ob46_seg0[0] <= 1.0000668119e+06
 R420: - C1 + 1e+06 alpha_ob46_seg0[1] <= 999952.9001
 R421: - C3 + 1e+06 alpha_ob46_seg0[1] <= 999952.9001
 R422: C0 + 1e+06 alpha_ob46_seg0[2] <= 999927.5381
 R423: C2 + 1e+06 alpha_ob46_seg0[2] <= 999927.5381
 R424: C1 + 1e+06 alpha_ob46_seg0[3] <= 1.0000445499e+06
 R425: C3 + 1e+06 alpha_ob46_seg0[3] <= 1.0000445499e+06
 R426: alpha_ob46_seg0[0] + alpha_ob46_seg0[1] + alpha_ob46_seg0[2]
   + alpha_ob46_seg0[3] >= 1
 R427: - C0 + 1e+06 alpha_ob47_seg0[0] <= 1.0000668119215801e+06
 R428: - C2 + 1e+06 alpha_ob47_seg0[0] <= 1.0000668119215801e+06
 R429: - C1 + 1e+06 alpha_ob47_seg0[1] <= 999941.3000997246
 R430: - C3 + 1e+06 alpha_ob47_seg0[1] <= 999941.3000997246
 R431: C0 + 1e+06 alpha_ob47_seg0[2] <= 999930.6381215801
 R432: C2 + 1e+06 alpha_ob47_seg0[2] <= 999930.6381215801
 R433: C1 + 1e+06 alpha_ob47_seg0[3] <= 1.0000445498997245e+06
 R434: C3 + 1e+06 alpha_ob47_seg0[3] <= 1.0000445498997245e+06
 R435: alpha_ob47_seg0[0] + alpha_ob47_seg0[1] + alpha_ob47_seg0[2]
   + alpha_ob47_seg0[3] >= 1
 R436: - C0 + 1e+06 alpha_ob48_seg0[0] <= 1.0000146737e+06
 R437: - C2 + 1e+06 alpha_ob48_seg0[0] <= 1.0000146737e+06
 R438: - C1 + 1e+06 alpha_ob48_seg0[1] <= 999940.7624
 R439: - C3 + 1e+06 alpha_ob48_seg0[1] <= 999940.7624
 R440: C0 + 1e+06 alpha_ob48_seg0[2] <= 999948.4263000001
 R441: C2 + 1e+06 alpha_ob48_seg0[2] <= 999948.4263000001
 R442: C1 + 1e+06 alpha_ob48_seg0[3] <= 1.0000566875999999e+06
 R443: C3 + 1e+06 alpha_ob48_seg0[3] <= 1.0000566875999999e+06
 R444: alpha_ob48_seg0[0] + alpha_ob48_seg0[1] + alpha_ob48_seg0[2]
   + alpha_ob48_seg0[3] >= 1
 R445: - C0 + 1e+06 alpha_ob49_seg0[0] <= 1.0000537119e+06
 R446: - C2 + 1e+06 alpha_ob49_seg0[0] <= 1.0000537119e+06
 R447: - C1 + 1e+06 alpha_ob49_seg0[1] <= 999941.3002000001
 R448: - C3 + 1e+06 alpha_ob49_seg0[1] <= 999941.3002000001
 R449: C0 + 1e+06 alpha_ob49_seg0[2] <= 999930.6381
 R450: C2 + 1e+06 alpha_ob49_seg0[2] <= 999930.6381
 R451: C1 + 1e+06 alpha_ob49_seg0[3] <= 1.0000561498e+06
 R452: C3 + 1e+06 alpha_ob49_seg0[3] <= 1.0000561498e+06
 R453: alpha_ob49_seg0[0] + alpha_ob49_seg0[1] + alpha_ob49_seg0[2]
   + alpha_ob49_seg0[3] >= 1
 R454: - C0 + 1e+06 alpha_ob50_seg0[0] <= 1.0000058189e+06
 R455: - C2 + 1e+06 alpha_ob50_seg0[0] <= 1.0000058189e+06
 R456: - C1 + 1e+06 alpha_ob50_seg0[1] <= 999940.5469
 R457: - C3 + 1e+06 alpha_ob50_seg0[1] <= 999940.5469
 R458: C0 + 1e+06 alpha_ob50_seg0[2] <= 999985.5311
 R459: C2 + 1e+06 alpha_ob50_seg0[2] <= 999985.5311
 R460: C1 + 1e+06 alpha_ob50_seg0[3] <= 1.0000569031e+06
 R461: C3 + 1e+06 alpha_ob50_seg0[3] <= 1.0000569031e+06
 R462: alpha_ob50_seg0[0] + alpha_ob50_seg0[1] + alpha_ob50_seg0[2]
   + alpha_ob50_seg0[3] >= 1
 R463: - C0 + 1e+06 alpha_ob51_seg0[0] <= 1.0000058188284199e+06
 R464: - C2 + 1e+06 alpha_ob51_seg0[0] <= 1.0000058188284199e+06
 R465: - C1 + 1e+06 alpha_ob51_seg0[1] <= 999940.5469002755
 R466: - C3 + 1e+06 alpha_ob51_seg0[1] <= 999940.5469002755
 R467: C0 + 1e+06 alpha_ob51_seg0[2] <= 999991.6311284199
 R468: C2 + 1e+06 alpha_ob51_seg0[2] <= 999991.6311284199
 R469: C1 + 1e+06 alpha_ob51_seg0[3] <= 1.0000453031002756e+06
 R470: C3 + 1e+06 alpha_ob51_seg0[3] <= 1.0000453031002756e+06
 R471: alpha_ob51_seg0[0] + alpha_ob51_seg0[1] + alpha_ob51_seg0[2]
   + alpha_ob51_seg0[3] >= 1
 R472: - C0 + 1e+06 alpha_ob52_seg0[0] <= 999996.9688499999
 R473: - C2 + 1e+06 alpha_ob52_seg0[0] <= 999996.9688499999
 R474: - C1 + 1e+06 alpha_ob52_seg0[1] <= 999940.5469
 R475: - C3 + 1e+06 alpha_ob52_seg0[1] <= 999940.5469
 R476: C0 + 1e+06 alpha_ob52_seg0[2] <= 999991.63115
 R477: C2 + 1e+06 alpha_ob52_seg0[2] <= 999991.63115
 R478: C1 + 1e+06 alpha_ob52_seg0[3] <= 1.0000569031e+06
 R479: C3 + 1e+06 alpha_ob52_seg0[3] <= 1.0000569031e+06
 R480: alpha_ob52_seg0[0] + alpha_ob52_seg0[1] + alpha_ob52_seg0[2]
   + alpha_ob52_seg0[3] >= 1
 R481: - C0 + 1e+06 alpha_ob53_seg0[0] <= 999991.83683
 R482: - C2 + 1e+06 alpha_ob53_seg0[0] <= 999991.83683
 R483: - C1 + 1e+06 alpha_ob53_seg0[1] <= 999940.589
 R484: - C3 + 1e+06 alpha_ob53_seg0[1] <= 999940.589
 R485: C0 + 1e+06 alpha_ob53_seg0[2] <= 1.00000401317e+06
 R486: C2 + 1e+06 alpha_ob53_seg0[2] <= 1.00000401317e+06
 R487: C1 + 1e+06 alpha_ob53_seg0[3] <= 1.000056861e+06
 R488: C3 + 1e+06 alpha_ob53_seg0[3] <= 1.000056861e+06
 R489: alpha_ob53_seg0[0] + alpha_ob53_seg0[1] + alpha_ob53_seg0[2]
   + alpha_ob53_seg0[3] >= 1
 R490: - C0 + 1e+06 alpha_ob54_seg0[0] <= 999991.8368084199
 R491: - C2 + 1e+06 alpha_ob54_seg0[0] <= 999991.8368084199
 R492: - C1 + 1e+06 alpha_ob54_seg0[1] <= 999940.5890002755
 R493: - C3 + 1e+06 alpha_ob54_seg0[1] <= 999940.5890002755
 R494: C0 + 1e+06 alpha_ob54_seg0[2] <= 1.0000056131484199e+06
 R495: C2 + 1e+06 alpha_ob54_seg0[2] <= 1.0000056131484199e+06
 R496: C1 + 1e+06 alpha_ob54_seg0[3] <= 1.0000452610002755e+06
 R497: C3 + 1e+06 alpha_ob54_seg0[3] <= 1.0000452610002755e+06
 R498: alpha_ob54_seg0[0] + alpha_ob54_seg0[1] + alpha_ob54_seg0[2]
   + alpha_ob54_seg0[3] >= 1
 R499: - C0 + 1e+06 alpha_ob55_seg0[0] <= 999910.5736
 R500: - C2 + 1e+06 alpha_ob55_seg0[0] <= 999910.5736
 R501: - C1 + 1e+06 alpha_ob55_seg0[1] <= 999926.115
 R502: - C3 + 1e+06 alpha_ob55_seg0[1] <= 999926.115
 R503: C0 + 1e+06 alpha_ob55_seg0[2] <= 1.0000467764e+06
 R504: C2 + 1e+06 alpha_ob55_seg0[2] <= 1.0000467764e+06
 R505: C1 + 1e+06 alpha_ob55_seg0[3] <= 1.000071335e+06
 R506: C3 + 1e+06 alpha_ob55_seg0[3] <= 1.000071335e+06
 R507: alpha_ob55_seg0[0] + alpha_ob55_seg0[1] + alpha_ob55_seg0[2]
   + alpha_ob55_seg0[3] >= 1
 R508: - C0 + 1e+06 alpha_ob56_seg0[0] <= 999910.5735508709
 R509: - C2 + 1e+06 alpha_ob56_seg0[0] <= 999910.5735508709
 R510: - C1 + 1e+06 alpha_ob56_seg0[1] <= 999926.1150002755
 R511: - C3 + 1e+06 alpha_ob56_seg0[1] <= 999926.1150002755
 R512: C0 + 1e+06 alpha_ob56_seg0[2] <= 1.0000868763508709e+06
 R513: C2 + 1e+06 alpha_ob56_seg0[2] <= 1.0000868763508709e+06
 R514: C1 + 1e+06 alpha_ob56_seg0[3] <= 1.0000447350002755e+06
 R515: C3 + 1e+06 alpha_ob56_seg0[3] <= 1.0000447350002755e+06
 R516: alpha_ob56_seg0[0] + alpha_ob56_seg0[1] + alpha_ob56_seg0[2]
   + alpha_ob56_seg0[3] >= 1
 R517: - C0 + 1e+06 alpha_ob57_seg0[0] <= 1.0000537118880621e+06
 R518: - C2 + 1e+06 alpha_ob57_seg0[0] <= 1.0000537118880621e+06
 R519: - C1 + 1e+06 alpha_ob57_seg0[1] <= 999941.3001002755
 R520: - C3 + 1e+06 alpha_ob57_seg0[1] <= 999941.3001002755
 R521: C0 + 1e+06 alpha_ob57_seg0[2] <= 999943.738088062
 R522: C2 + 1e+06 alpha_ob57_seg0[2] <= 999943.738088062
 R523: C1 + 1e+06 alpha_ob57_seg0[3] <= 1.0000497999002755e+06
 R524: C3 + 1e+06 alpha_ob57_seg0[3] <= 1.0000497999002755e+06
 R525: alpha_ob57_seg0[0] + alpha_ob57_seg0[1] + alpha_ob57_seg0[2]
   + alpha_ob57_seg0[3] >= 1
 R526: - C0 + 1e+06 alpha_ob58_seg0[0] <= 999910.573599801
 R527: - C2 + 1e+06 alpha_ob58_seg0[0] <= 999910.573599801
 R528: - C1 + 1e+06 alpha_ob58_seg0[1] <= 999952.7150152582
 R529: - C3 + 1e+06 alpha_ob58_seg0[1] <= 999952.7150152582
 R530: C0 + 1e+06 alpha_ob58_seg0[2] <= 1.000075526399801e+06
 R531: C2 + 1e+06 alpha_ob58_seg0[2] <= 1.000075526399801e+06
 R532: C1 + 1e+06 alpha_ob58_seg0[3] <= 1.0000447350152582e+06
 R533: C3 + 1e+06 alpha_ob58_seg0[3] <= 1.0000447350152582e+06
 R534: alpha_ob58_seg0[0] + alpha_ob58_seg0[1] + alpha_ob58_seg0[2]
   + alpha_ob58_seg0[3] >= 1
 R535: - C0 + 1e+06 alpha_ob59_seg0[0] <= 999921.6867474595
 R536: - C2 + 1e+06 alpha_ob59_seg0[0] <= 999921.6867474595
 R537: - C1 + 1e+06 alpha_ob59_seg0[1] <= 999951.9159478974
 R538: - C3 + 1e+06 alpha_ob59_seg0[1] <= 999951.9159478974
 R539: C0 + 1e+06 alpha_ob59_seg0[2] <= 1.0000755095474594e+06
 R540: C2 + 1e+06 alpha_ob59_seg0[2] <= 1.0000755095474594e+06
 R541: C1 + 1e+06 alpha_ob59_seg0[3] <= 1.0000447569478974e+06
 R542: C3 + 1e+06 alpha_ob59_seg0[3] <= 1.0000447569478974e+06
 R543: alpha_ob59_seg0[0] + alpha_ob59_seg0[1] + alpha_ob59_seg0[2]
   + alpha_ob59_seg0[3] >= 1
 R544: - C0 + 1e+06 alpha_ob60_seg0[0] <= 999921.7035998009
 R545: - C2 + 1e+06 alpha_ob60_seg0[0] <= 999921.7035998009
 R546: - C1 + 1e+06 alpha_ob60_seg0[1] <= 999951.8939069656
 R547: - C3 + 1e+06 alpha_ob60_seg0[1] <= 999951.8939069656
 R548: C0 + 1e+06 alpha_ob60_seg0[2] <= 1.000070646399801e+06
 R549: C2 + 1e+06 alpha_ob60_seg0[2] <= 1.000070646399801e+06
 R550: C1 + 1e+06 alpha_ob60_seg0[3] <= 1.0000455561069656e+06
 R551: C3 + 1e+06 alpha_ob60_seg0[3] <= 1.0000455561069656e+06
 R552: alpha_ob60_seg0[0] + alpha_ob60_seg0[1] + alpha_ob60_seg0[2]
   + alpha_ob60_seg0[3] >= 1
 R553: - C0 + 1e+06 alpha_ob61_seg0[0] <= 999950.6735522483
 R554: - C2 + 1e+06 alpha_ob61_seg0[0] <= 999950.6735522483
 R555: - C1 + 1e+06 alpha_ob61_seg0[1] <= 999926.1150002755
 R556: - C3 + 1e+06 alpha_ob61_seg0[1] <= 999926.1150002755
 R557: C0 + 1e+06 alpha_ob61_seg0[2] <= 1.0000467763522484e+06
 R558: C2 + 1e+06 alpha_ob61_seg0[2] <= 1.0000467763522484e+06
 R559: C1 + 1e+06 alpha_ob61_seg0[3] <= 1.0000454850002755e+06
 R560: C3 + 1e+06 alpha_ob61_seg0[3] <= 1.0000454850002755e+06
 R561: alpha_ob61_seg0[0] + alpha_ob61_seg0[1] + alpha_ob61_seg0[2]
   + alpha_ob61_seg0[3] >= 1
 R562: - C0 + 1e+06 alpha_ob62_seg0[0] <= 999941.3236
 R563: - C2 + 1e+06 alpha_ob62_seg0[0] <= 999941.3236
 R564: - C1 + 1e+06 alpha_ob62_seg0[1] <= 999951.965
 R565: - C3 + 1e+06 alpha_ob62_seg0[1] <= 999951.965
 R566: C0 + 1e+06 alpha_ob62_seg0[2] <= 1.0000467764e+06
 R567: C2 + 1e+06 alpha_ob62_seg0[2] <= 1.0000467764e+06
 R568: C1 + 1e+06 alpha_ob62_seg0[3] <= 1.000045485e+06
 R569: C3 + 1e+06 alpha_ob62_seg0[3] <= 1.000045485e+06
 R570: alpha_ob62_seg0[0] + alpha_ob62_seg0[1] + alpha_ob62_seg0[2]
   + alpha_ob62_seg0[3] >= 1
 R571: - C0 + 1e+06 alpha_ob63_seg0[0] <= 999941.3236073464
 R572: - C2 + 1e+06 alpha_ob63_seg0[0] <= 999941.3236073464
 R573: - C1 + 1e+06 alpha_ob63_seg0[1] <= 999948.1149997246
 R574: - C3 + 1e+06 alpha_ob63_seg0[1] <= 999948.1149997246
 R575: C0 + 1e+06 alpha_ob63_seg0[2] <= 1.0000561264073465e+06
 R576: C2 + 1e+06 alpha_ob63_seg0[2] <= 1.0000561264073465e+06
 R577: C1 + 1e+06 alpha_ob63_seg0[3] <= 1.0000454849997245e+06
 R578: C3 + 1e+06 alpha_ob63_seg0[3] <= 1.0000454849997245e+06
 R579: alpha_ob63_seg0[0] + alpha_ob63_seg0[1] + alpha_ob63_seg0[2]
   + alpha_ob63_seg0[3] >= 1
 R580: - C0 + 1e+06 alpha_ob64_seg0[0] <= 999931.7236
 R581: - C2 + 1e+06 alpha_ob64_seg0[0] <= 999931.7236
 R582: - C1 + 1e+06 alpha_ob64_seg0[1] <= 999948.115
 R583: - C3 + 1e+06 alpha_ob64_seg0[1] <= 999948.115
 R584: C0 + 1e+06 alpha_ob64_seg0[2] <= 1.0000561264e+06
 R585: C2 + 1e+06 alpha_ob64_seg0[2] <= 1.0000561264e+06
 R586: C1 + 1e+06 alpha_ob64_seg0[3] <= 1.000049335e+06
 R587: C3 + 1e+06 alpha_ob64_seg0[3] <= 1.000049335e+06
 R588: alpha_ob64_seg0[0] + alpha_ob64_seg0[1] + alpha_ob64_seg0[2]
   + alpha_ob64_seg0[3] >= 1
 R589: - C0 + 1e+06 alpha_ob65_seg0[0] <= 999931.7235921945
 R590: - C2 + 1e+06 alpha_ob65_seg0[0] <= 999931.7235921945
 R591: - C1 + 1e+06 alpha_ob65_seg0[1] <= 999948.1150002755
 R592: - C3 + 1e+06 alpha_ob65_seg0[1] <= 999948.1150002755
 R593: C0 + 1e+06 alpha_ob65_seg0[2] <= 1.0000657263921944e+06
 R594: C2 + 1e+06 alpha_ob65_seg0[2] <= 1.0000657263921944e+06
 R595: C1 + 1e+06 alpha_ob65_seg0[3] <= 1.0000452350002755e+06
 R596: C3 + 1e+06 alpha_ob65_seg0[3] <= 1.0000452350002755e+06
 R597: alpha_ob65_seg0[0] + alpha_ob65_seg0[1] + alpha_ob65_seg0[2]
   + alpha_ob65_seg0[3] >= 1
 R598: - C0 + 1e+06 alpha_ob66_seg0[0] <= 1.000053711899801e+06
 R599: - C2 + 1e+06 alpha_ob66_seg0[0] <= 1.000053711899801e+06
 R600: - C1 + 1e+06 alpha_ob66_seg0[1] <= 999947.6501129363
 R601: - C3 + 1e+06 alpha_ob66_seg0[1] <= 999947.6501129363
 R602: C0 + 1e+06 alpha_ob66_seg0[2] <= 999934.138099801
 R603: C2 + 1e+06 alpha_ob66_seg0[2] <= 999934.138099801
 R604: C1 + 1e+06 alpha_ob66_seg0[3] <= 1.0000497999129363e+06
 R605: C3 + 1e+06 alpha_ob66_seg0[3] <= 1.0000497999129363e+06
 R606: alpha_ob66_seg0[0] + alpha_ob66_seg0[1] + alpha_ob66_seg0[2]
   + alpha_ob66_seg0[3] >= 1
 R607: - C0 + 1e+06 alpha_ob67_seg0[0] <= 1.0000764021508709e+06
 R608: - C2 + 1e+06 alpha_ob67_seg0[0] <= 1.0000764021508709e+06
 R609: - C1 + 1e+06 alpha_ob67_seg0[1] <= 999957.1027002756
 R610: - C3 + 1e+06 alpha_ob67_seg0[1] <= 999957.1027002756
 R611: C0 + 1e+06 alpha_ob67_seg0[2] <= 999921.0477508709
 R612: C2 + 1e+06 alpha_ob67_seg0[2] <= 999921.0477508709
 R613: C1 + 1e+06 alpha_ob67_seg0[3] <= 1.0000137473002756e+06
 R614: C3 + 1e+06 alpha_ob67_seg0[3] <= 1.0000137473002756e+06
 R615: alpha_ob67_seg0[0] + alpha_ob67_seg0[1] + alpha_ob67_seg0[2]
   + alpha_ob67_seg0[3] >= 1
 R616: - C0 + 1e+06 alpha_ob68_seg0[0] <= 1.0000633118917353e+06
 R617: - C2 + 1e+06 alpha_ob68_seg0[0] <= 1.0000633118917353e+06
 R618: - C1 + 1e+06 alpha_ob68_seg0[1] <= 999947.6502002755
 R619: - C3 + 1e+06 alpha_ob68_seg0[1] <= 999947.6502002755
 R620: C0 + 1e+06 alpha_ob68_seg0[2] <= 999934.1380917353
 R621: C2 + 1e+06 alpha_ob68_seg0[2] <= 999934.1380917353
 R622: C1 + 1e+06 alpha_ob68_seg0[3] <= 1.0000454498002755e+06
 R623: C3 + 1e+06 alpha_ob68_seg0[3] <= 1.0000454498002755e+06
 R624: alpha_ob68_seg0[0] + alpha_ob68_seg0[1] + alpha_ob68_seg0[2]
   + alpha_ob68_seg0[3] >= 1
 R625: - C0 + 1e+06 alpha_ob69_seg0[0] <= 1.0000448022e+06
 R626: - C2 + 1e+06 alpha_ob69_seg0[0] <= 1.0000448022e+06
 R627: - C1 + 1e+06 alpha_ob69_seg0[1] <= 999957.1027
 R628: - C3 + 1e+06 alpha_ob69_seg0[1] <= 999957.1027
 R629: C0 + 1e+06 alpha_ob69_seg0[2] <= 999921.0477999999
 R630: C2 + 1e+06 alpha_ob69_seg0[2] <= 999921.0477999999
 R631: C1 + 1e+06 alpha_ob69_seg0[3] <= 1.0000403473e+06
 R632: C3 + 1e+06 alpha_ob69_seg0[3] <= 1.0000403473e+06
 R633: alpha_ob69_seg0[0] + alpha_ob69_seg0[1] + alpha_ob69_seg0[2]
   + alpha_ob69_seg0[3] >= 1
 R634: - C0 + 1e+06 alpha_ob70_seg0[0] <= 1.0000448021770425e+06
 R635: - C2 + 1e+06 alpha_ob70_seg0[0] <= 1.0000448021770425e+06
 R636: - C1 + 1e+06 alpha_ob70_seg0[1] <= 999957.1027002755
 R637: - C3 + 1e+06 alpha_ob70_seg0[1] <= 999957.1027002755
 R638: C0 + 1e+06 alpha_ob70_seg0[2] <= 999952.6477770425
 R639: C2 + 1e+06 alpha_ob70_seg0[2] <= 999952.6477770425
 R640: C1 + 1e+06 alpha_ob70_seg0[3] <= 1.0000279973002755e+06
 R641: C3 + 1e+06 alpha_ob70_seg0[3] <= 1.0000279973002755e+06
 R642: alpha_ob70_seg0[0] + alpha_ob70_seg0[1] + alpha_ob70_seg0[2]
   + alpha_ob70_seg0[3] >= 1
 R643: - C2 + 1e+06 alpha_ob0_seg1[0] <= 1.0000284171e+06
 R644: - C4 + 1e+06 alpha_ob0_seg1[0] <= 1.0000284171e+06
 R645: - C3 + 1e+06 alpha_ob0_seg1[1] <= 999957.0843
 R646: - C5 + 1e+06 alpha_ob0_seg1[1] <= 999957.0843
 R647: C2 + 1e+06 alpha_ob0_seg1[2] <= 999957.1829
 R648: C4 + 1e+06 alpha_ob0_seg1[2] <= 999957.1829
 R649: C3 + 1e+06 alpha_ob0_seg1[3] <= 1.0000403657e+06
 R650: C5 + 1e+06 alpha_ob0_seg1[3] <= 1.0000403657e+06
 R651: alpha_ob0_seg1[0] + alpha_ob0_seg1[1] + alpha_ob0_seg1[2]
   + alpha_ob0_seg1[3] >= 1
 R652: - C2 + 1e+06 alpha_ob1_seg1[0] <= 1.0000284170499525e+06
 R653: - C4 + 1e+06 alpha_ob1_seg1[0] <= 1.0000284170499525e+06
 R654: - C3 + 1e+06 alpha_ob1_seg1[1] <= 999957.0843002755
 R655: - C5 + 1e+06 alpha_ob1_seg1[1] <= 999957.0843002755
 R656: C2 + 1e+06 alpha_ob1_seg1[2] <= 999969.0328499526
 R657: C4 + 1e+06 alpha_ob1_seg1[2] <= 999969.0328499526
 R658: C3 + 1e+06 alpha_ob1_seg1[3] <= 1.0000132657002755e+06
 R659: C5 + 1e+06 alpha_ob1_seg1[3] <= 1.0000132657002755e+06
 R660: alpha_ob1_seg1[0] + alpha_ob1_seg1[1] + alpha_ob1_seg1[2]
   + alpha_ob1_seg1[3] >= 1
 R661: - C2 + 1e+06 alpha_ob2_seg1[0] <= 1.0000400171500475e+06
 R662: - C4 + 1e+06 alpha_ob2_seg1[0] <= 1.0000400171500475e+06
 R663: - C3 + 1e+06 alpha_ob2_seg1[1] <= 999957.0842997245
 R664: - C5 + 1e+06 alpha_ob2_seg1[1] <= 999957.0842997245
 R665: C2 + 1e+06 alpha_ob2_seg1[2] <= 999957.4329500474
 R666: C4 + 1e+06 alpha_ob2_seg1[2] <= 999957.4329500474
 R667: C3 + 1e+06 alpha_ob2_seg1[3] <= 1.0000132656997246e+06
 R668: C5 + 1e+06 alpha_ob2_seg1[3] <= 1.0000132656997246e+06
 R669: alpha_ob2_seg1[0] + alpha_ob2_seg1[1] + alpha_ob2_seg1[2]
   + alpha_ob2_seg1[3] >= 1
 R670: - C2 + 1e+06 alpha_ob3_seg1[0] <= 1.000022266175665e+06
 R671: - C4 + 1e+06 alpha_ob3_seg1[0] <= 1.000022266175665e+06
 R672: - C3 + 1e+06 alpha_ob3_seg1[1] <= 999957.1681002756
 R673: - C5 + 1e+06 alpha_ob3_seg1[1] <= 999957.1681002756
 R674: C2 + 1e+06 alpha_ob3_seg1[2] <= 999975.183775665
 R675: C4 + 1e+06 alpha_ob3_seg1[2] <= 999975.183775665
 R676: C3 + 1e+06 alpha_ob3_seg1[3] <= 1.0000271819002755e+06
 R677: C5 + 1e+06 alpha_ob3_seg1[3] <= 1.0000271819002755e+06
 R678: alpha_ob3_seg1[0] + alpha_ob3_seg1[1] + alpha_ob3_seg1[2]
   + alpha_ob3_seg1[3] >= 1
 R679: - C2 + 1e+06 alpha_ob4_seg1[0] <= 1.0000139162e+06
 R680: - C4 + 1e+06 alpha_ob4_seg1[0] <= 1.0000139162e+06
 R681: - C3 + 1e+06 alpha_ob4_seg1[1] <= 999957.1679999999
 R682: - C5 + 1e+06 alpha_ob4_seg1[1] <= 999957.1679999999
 R683: C2 + 1e+06 alpha_ob4_seg1[2] <= 999975.1838
 R684: C4 + 1e+06 alpha_ob4_seg1[2] <= 999975.1838
 R685: C3 + 1e+06 alpha_ob4_seg1[3] <= 1.000040282e+06
 R686: C5 + 1e+06 alpha_ob4_seg1[3] <= 1.000040282e+06
 R687: alpha_ob4_seg1[0] + alpha_ob4_seg1[1] + alpha_ob4_seg1[2]
   + alpha_ob4_seg1[3] >= 1
 R688: - C2 + 1e+06 alpha_ob5_seg1[0] <= 999992.49102
 R689: - C4 + 1e+06 alpha_ob5_seg1[0] <= 999992.49102
 R690: - C3 + 1e+06 alpha_ob5_seg1[1] <= 999957.1016000001
 R691: - C5 + 1e+06 alpha_ob5_seg1[1] <= 999957.1016000001
 R692: C2 + 1e+06 alpha_ob5_seg1[2] <= 999986.35898
 R693: C4 + 1e+06 alpha_ob5_seg1[2] <= 999986.35898
 R694: C3 + 1e+06 alpha_ob5_seg1[3] <= 1.0000403484e+06
 R695: C5 + 1e+06 alpha_ob5_seg1[3] <= 1.0000403484e+06
 R696: alpha_ob5_seg1[0] + alpha_ob5_seg1[1] + alpha_ob5_seg1[2]
   + alpha_ob5_seg1[3] >= 1
 R697: - C2 + 1e+06 alpha_ob6_seg1[0] <= 999981.3909999999
 R698: - C4 + 1e+06 alpha_ob6_seg1[0] <= 999981.3909999999
 R699: - C3 + 1e+06 alpha_ob6_seg1[1] <= 999957.1016000001
 R700: - C5 + 1e+06 alpha_ob6_seg1[1] <= 999957.1016000001
 R701: C2 + 1e+06 alpha_ob6_seg1[2] <= 1.000004959e+06
 R702: C4 + 1e+06 alpha_ob6_seg1[2] <= 1.000004959e+06
 R703: C3 + 1e+06 alpha_ob6_seg1[3] <= 1.0000403484e+06
 R704: C5 + 1e+06 alpha_ob6_seg1[3] <= 1.0000403484e+06
 R705: alpha_ob6_seg1[0] + alpha_ob6_seg1[1] + alpha_ob6_seg1[2]
   + alpha_ob6_seg1[3] >= 1
 R706: - C2 + 1e+06 alpha_ob7_seg1[0] <= 999981.3909825523
 R707: - C4 + 1e+06 alpha_ob7_seg1[0] <= 999981.3909825523
 R708: - C3 + 1e+06 alpha_ob7_seg1[1] <= 999957.1016002755
 R709: - C5 + 1e+06 alpha_ob7_seg1[1] <= 999957.1016002755
 R710: C2 + 1e+06 alpha_ob7_seg1[2] <= 1.0000160589825523e+06
 R711: C4 + 1e+06 alpha_ob7_seg1[2] <= 1.0000160589825523e+06
 R712: C3 + 1e+06 alpha_ob7_seg1[3] <= 1.0000309984002755e+06
 R713: C5 + 1e+06 alpha_ob7_seg1[3] <= 1.0000309984002755e+06
 R714: alpha_ob7_seg1[0] + alpha_ob7_seg1[1] + alpha_ob7_seg1[2]
   + alpha_ob7_seg1[3] >= 1
 R715: - C2 + 1e+06 alpha_ob8_seg1[0] <= 1.0000537118917353e+06
 R716: - C4 + 1e+06 alpha_ob8_seg1[0] <= 1.0000537118917353e+06
 R717: - C3 + 1e+06 alpha_ob8_seg1[1] <= 999947.6502002755
 R718: - C5 + 1e+06 alpha_ob8_seg1[1] <= 999947.6502002755
 R719: C2 + 1e+06 alpha_ob8_seg1[2] <= 999943.7380917353
 R720: C4 + 1e+06 alpha_ob8_seg1[2] <= 999943.7380917353
 R721: C3 + 1e+06 alpha_ob8_seg1[3] <= 1.0000454498002755e+06
 R722: C5 + 1e+06 alpha_ob8_seg1[3] <= 1.0000454498002755e+06
 R723: alpha_ob8_seg1[0] + alpha_ob8_seg1[1] + alpha_ob8_seg1[2]
   + alpha_ob8_seg1[3] >= 1
 R724: - C2 + 1e+06 alpha_ob9_seg1[0] <= 1.0000040868238758e+06
 R725: - C4 + 1e+06 alpha_ob9_seg1[0] <= 1.0000040868238758e+06
 R726: - C3 + 1e+06 alpha_ob9_seg1[1] <= 999957.2594997246
 R727: - C5 + 1e+06 alpha_ob9_seg1[1] <= 999957.2594997246
 R728: C2 + 1e+06 alpha_ob9_seg1[2] <= 999993.3632238759
 R729: C4 + 1e+06 alpha_ob9_seg1[2] <= 999993.3632238759
 R730: C3 + 1e+06 alpha_ob9_seg1[3] <= 1.0000273404997246e+06
 R731: C5 + 1e+06 alpha_ob9_seg1[3] <= 1.0000273404997246e+06
 R732: alpha_ob9_seg1[0] + alpha_ob9_seg1[1] + alpha_ob9_seg1[2]
   + alpha_ob9_seg1[3] >= 1
 R733: - C2 + 1e+06 alpha_ob10_seg1[0] <= 1.0000531119e+06
 R734: - C4 + 1e+06 alpha_ob10_seg1[0] <= 1.0000531119e+06
 R735: - C3 + 1e+06 alpha_ob10_seg1[1] <= 999952.0000999999
 R736: - C5 + 1e+06 alpha_ob10_seg1[1] <= 999952.0000999999
 R737: C2 + 1e+06 alpha_ob10_seg1[2] <= 999943.7381
 R738: C4 + 1e+06 alpha_ob10_seg1[2] <= 999943.7381
 R739: C3 + 1e+06 alpha_ob10_seg1[3] <= 1.0000454499e+06
 R740: C5 + 1e+06 alpha_ob10_seg1[3] <= 1.0000454499e+06
 R741: alpha_ob10_seg1[0] + alpha_ob10_seg1[1] + alpha_ob10_seg1[2]
   + alpha_ob10_seg1[3] >= 1
 R742: - C2 + 1e+06 alpha_ob11_seg1[0] <= 999948.0333
 R743: - C4 + 1e+06 alpha_ob11_seg1[0] <= 999948.0333
 R744: - C3 + 1e+06 alpha_ob11_seg1[1] <= 1.0000686083e+06
 R745: - C5 + 1e+06 alpha_ob11_seg1[1] <= 1.0000686083e+06
 R746: C2 + 1e+06 alpha_ob11_seg1[2] <= 999920.0667
 R747: C4 + 1e+06 alpha_ob11_seg1[2] <= 999920.0667
 R748: C3 + 1e+06 alpha_ob11_seg1[3] <= 999928.8417
 R749: C5 + 1e+06 alpha_ob11_seg1[3] <= 999928.8417
 R750: alpha_ob11_seg1[0] + alpha_ob11_seg1[1] + alpha_ob11_seg1[2]
   + alpha_ob11_seg1[3] >= 1
 R751: - C2 + 1e+06 alpha_ob12_seg1[0] <= 1.0000490237e+06
 R752: - C4 + 1e+06 alpha_ob12_seg1[0] <= 1.0000490237e+06
 R753: - C3 + 1e+06 alpha_ob12_seg1[1] <= 999940.7624
 R754: - C5 + 1e+06 alpha_ob12_seg1[1] <= 999940.7624
 R755: C2 + 1e+06 alpha_ob12_seg1[2] <= 999947.3263
 R756: C4 + 1e+06 alpha_ob12_seg1[2] <= 999947.3263
 R757: C3 + 1e+06 alpha_ob12_seg1[3] <= 1.0000566875999999e+06
 R758: C5 + 1e+06 alpha_ob12_seg1[3] <= 1.0000566875999999e+06
 R759: alpha_ob12_seg1[0] + alpha_ob12_seg1[1] + alpha_ob12_seg1[2]
   + alpha_ob12_seg1[3] >= 1
 R760: - C2 + 1e+06 alpha_ob13_seg1[0] <= 999915.9461796245
 R761: - C4 + 1e+06 alpha_ob13_seg1[0] <= 999915.9461796245
 R762: - C3 + 1e+06 alpha_ob13_seg1[1] <= 1.0000683610286588e+06
 R763: - C5 + 1e+06 alpha_ob13_seg1[1] <= 1.0000683610286588e+06
 R764: C2 + 1e+06 alpha_ob13_seg1[2] <= 1.0000495659796246e+06
 R765: C4 + 1e+06 alpha_ob13_seg1[2] <= 1.0000495659796246e+06
 R766: C3 + 1e+06 alpha_ob13_seg1[3] <= 999928.8376286589
 R767: C5 + 1e+06 alpha_ob13_seg1[3] <= 999928.8376286589
 R768: alpha_ob13_seg1[0] + alpha_ob13_seg1[1] + alpha_ob13_seg1[2]
   + alpha_ob13_seg1[3] >= 1
 R769: - C2 + 1e+06 alpha_ob14_seg1[0] <= 1.000049023678879e+06
 R770: - C4 + 1e+06 alpha_ob14_seg1[0] <= 1.000049023678879e+06
 R771: - C3 + 1e+06 alpha_ob14_seg1[1] <= 999940.7624002756
 R772: - C5 + 1e+06 alpha_ob14_seg1[1] <= 999940.7624002756
 R773: C2 + 1e+06 alpha_ob14_seg1[2] <= 999948.4262788791
 R774: C4 + 1e+06 alpha_ob14_seg1[2] <= 999948.4262788791
 R775: C3 + 1e+06 alpha_ob14_seg1[3] <= 1.0000453376002755e+06
 R776: C5 + 1e+06 alpha_ob14_seg1[3] <= 1.0000453376002755e+06
 R777: alpha_ob14_seg1[0] + alpha_ob14_seg1[1] + alpha_ob14_seg1[2]
   + alpha_ob14_seg1[3] >= 1
 R778: - C2 + 1e+06 alpha_ob15_seg1[0] <= 1.0000066737e+06
 R779: - C4 + 1e+06 alpha_ob15_seg1[0] <= 1.0000066737e+06
 R780: - C3 + 1e+06 alpha_ob15_seg1[1] <= 999952.1958
 R781: - C5 + 1e+06 alpha_ob15_seg1[1] <= 999952.1958
 R782: C2 + 1e+06 alpha_ob15_seg1[2] <= 999948.4263000001
 R783: C4 + 1e+06 alpha_ob15_seg1[2] <= 999948.4263000001
 R784: C3 + 1e+06 alpha_ob15_seg1[3] <= 1.0000452542e+06
 R785: C5 + 1e+06 alpha_ob15_seg1[3] <= 1.0000452542e+06
 R786: alpha_ob15_seg1[0] + alpha_ob15_seg1[1] + alpha_ob15_seg1[2]
   + alpha_ob15_seg1[3] >= 1
 R787: - C2 + 1e+06 alpha_ob16_seg1[0] <= 999991.073732
 R788: - C4 + 1e+06 alpha_ob16_seg1[0] <= 999991.073732
 R789: - C3 + 1e+06 alpha_ob16_seg1[1] <= 999952.2125
 R790: - C5 + 1e+06 alpha_ob16_seg1[1] <= 999952.2125
 R791: C2 + 1e+06 alpha_ob16_seg1[2] <= 999990.776268
 R792: C4 + 1e+06 alpha_ob16_seg1[2] <= 999990.776268
 R793: C3 + 1e+06 alpha_ob16_seg1[3] <= 1.0000452375e+06
 R794: C5 + 1e+06 alpha_ob16_seg1[3] <= 1.0000452375e+06
 R795: alpha_ob16_seg1[0] + alpha_ob16_seg1[1] + alpha_ob16_seg1[2]
   + alpha_ob16_seg1[3] >= 1
 R796: - C2 + 1e+06 alpha_ob17_seg1[0] <= 1.0000837618494935e+06
 R797: - C4 + 1e+06 alpha_ob17_seg1[0] <= 1.0000837618494935e+06
 R798: - C3 + 1e+06 alpha_ob17_seg1[1] <= 999925.5501002755
 R799: - C5 + 1e+06 alpha_ob17_seg1[1] <= 999925.5501002755
 R800: C2 + 1e+06 alpha_ob17_seg1[2] <= 999913.6880494935
 R801: C4 + 1e+06 alpha_ob17_seg1[2] <= 999913.6880494935
 R802: C3 + 1e+06 alpha_ob17_seg1[3] <= 1.0000445499002755e+06
 R803: C5 + 1e+06 alpha_ob17_seg1[3] <= 1.0000445499002755e+06
 R804: alpha_ob17_seg1[0] + alpha_ob17_seg1[1] + alpha_ob17_seg1[2]
   + alpha_ob17_seg1[3] >= 1
 R805: - C2 + 1e+06 alpha_ob18_seg1[0] <= 999956.8394000001
 R806: - C4 + 1e+06 alpha_ob18_seg1[0] <= 999956.8394000001
 R807: - C3 + 1e+06 alpha_ob18_seg1[1] <= 999952.2463
 R808: - C5 + 1e+06 alpha_ob18_seg1[1] <= 999952.2463
 R809: C2 + 1e+06 alpha_ob18_seg1[2] <= 1.0000097606e+06
 R810: C4 + 1e+06 alpha_ob18_seg1[2] <= 1.0000097606e+06
 R811: C3 + 1e+06 alpha_ob18_seg1[3] <= 1.0000452037e+06
 R812: C5 + 1e+06 alpha_ob18_seg1[3] <= 1.0000452037e+06
 R813: alpha_ob18_seg1[0] + alpha_ob18_seg1[1] + alpha_ob18_seg1[2]
   + alpha_ob18_seg1[3] >= 1
 R814: - C2 + 1e+06 alpha_ob19_seg1[0] <= 999964.9321250001
 R815: - C4 + 1e+06 alpha_ob19_seg1[0] <= 999964.9321250001
 R816: - C3 + 1e+06 alpha_ob19_seg1[1] <= 1.0000141893e+06
 R817: - C5 + 1e+06 alpha_ob19_seg1[1] <= 1.0000141893e+06
 R818: C2 + 1e+06 alpha_ob19_seg1[2] <= 1.000025197325e+06
 R819: C4 + 1e+06 alpha_ob19_seg1[2] <= 1.000025197325e+06
 R820: C3 + 1e+06 alpha_ob19_seg1[3] <= 999983.2607
 R821: C5 + 1e+06 alpha_ob19_seg1[3] <= 999983.2607
 R822: alpha_ob19_seg1[0] + alpha_ob19_seg1[1] + alpha_ob19_seg1[2]
   + alpha_ob19_seg1[3] >= 1
 R823: - C2 + 1e+06 alpha_ob20_seg1[0] <= 999964.922208629
 R824: - C4 + 1e+06 alpha_ob20_seg1[0] <= 999964.922208629
 R825: - C3 + 1e+06 alpha_ob20_seg1[1] <= 1.0000141893002756e+06
 R826: - C5 + 1e+06 alpha_ob20_seg1[1] <= 1.0000141893002756e+06
 R827: C2 + 1e+06 alpha_ob20_seg1[2] <= 1.000032527608629e+06
 R828: C4 + 1e+06 alpha_ob20_seg1[2] <= 1.000032527608629e+06
 R829: C3 + 1e+06 alpha_ob20_seg1[3] <= 999933.6607002757
 R830: C5 + 1e+06 alpha_ob20_seg1[3] <= 999933.6607002757
 R831: alpha_ob20_seg1[0] + alpha_ob20_seg1[1] + alpha_ob20_seg1[2]
   + alpha_ob20_seg1[3] >= 1
 R832: - C2 + 1e+06 alpha_ob21_seg1[0] <= 999964.922299801
 R833: - C4 + 1e+06 alpha_ob21_seg1[0] <= 999964.922299801
 R834: - C3 + 1e+06 alpha_ob21_seg1[1] <= 1.000063789309951e+06
 R835: - C5 + 1e+06 alpha_ob21_seg1[1] <= 1.000063789309951e+06
 R836: C2 + 1e+06 alpha_ob21_seg1[2] <= 1.000025177699801e+06
 R837: C4 + 1e+06 alpha_ob21_seg1[2] <= 1.000025177699801e+06
 R838: C3 + 1e+06 alpha_ob21_seg1[3] <= 999933.660709951
 R839: C5 + 1e+06 alpha_ob21_seg1[3] <= 999933.660709951
 R840: alpha_ob21_seg1[0] + alpha_ob21_seg1[1] + alpha_ob21_seg1[2]
   + alpha_ob21_seg1[3] >= 1
 R841: - C2 + 1e+06 alpha_ob22_seg1[0] <= 999965.1781
 R842: - C4 + 1e+06 alpha_ob22_seg1[0] <= 999965.1781
 R843: - C3 + 1e+06 alpha_ob22_seg1[1] <= 1.0000488976e+06
 R844: - C5 + 1e+06 alpha_ob22_seg1[1] <= 1.0000488976e+06
 R845: C2 + 1e+06 alpha_ob22_seg1[2] <= 1.0000254219e+06
 R846: C4 + 1e+06 alpha_ob22_seg1[2] <= 1.0000254219e+06
 R847: C3 + 1e+06 alpha_ob22_seg1[3] <= 999948.5524
 R848: C5 + 1e+06 alpha_ob22_seg1[3] <= 999948.5524
 R849: alpha_ob22_seg1[0] + alpha_ob22_seg1[1] + alpha_ob22_seg1[2]
   + alpha_ob22_seg1[3] >= 1
 R850: - C2 + 1e+06 alpha_ob23_seg1[0] <= 999965.0925
 R851: - C4 + 1e+06 alpha_ob23_seg1[0] <= 999965.0925
 R852: - C3 + 1e+06 alpha_ob23_seg1[1] <= 1.0000418517999999e+06
 R853: - C5 + 1e+06 alpha_ob23_seg1[1] <= 1.0000418517999999e+06
 R854: C2 + 1e+06 alpha_ob23_seg1[2] <= 1.0000250074999999e+06
 R855: C4 + 1e+06 alpha_ob23_seg1[2] <= 1.0000250074999999e+06
 R856: C3 + 1e+06 alpha_ob23_seg1[3] <= 999955.5982
 R857: C5 + 1e+06 alpha_ob23_seg1[3] <= 999955.5982
 R858: alpha_ob23_seg1[0] + alpha_ob23_seg1[1] + alpha_ob23_seg1[2]
   + alpha_ob23_seg1[3] >= 1
 R859: - C2 + 1e+06 alpha_ob24_seg1[0] <= 999964.9807
 R860: - C4 + 1e+06 alpha_ob24_seg1[0] <= 999964.9807
 R861: - C3 + 1e+06 alpha_ob24_seg1[1] <= 1.0000364639e+06
 R862: - C5 + 1e+06 alpha_ob24_seg1[1] <= 1.0000364639e+06
 R863: C2 + 1e+06 alpha_ob24_seg1[2] <= 1.0000248693e+06
 R864: C4 + 1e+06 alpha_ob24_seg1[2] <= 1.0000248693e+06
 R865: C3 + 1e+06 alpha_ob24_seg1[3] <= 999960.9861
 R866: C5 + 1e+06 alpha_ob24_seg1[3] <= 999960.9861
 R867: alpha_ob24_seg1[0] + alpha_ob24_seg1[1] + alpha_ob24_seg1[2]
   + alpha_ob24_seg1[3] >= 1
 R868: - C2 + 1e+06 alpha_ob25_seg1[0] <= 999965.0662
 R869: - C4 + 1e+06 alpha_ob25_seg1[0] <= 999965.0662
 R870: - C3 + 1e+06 alpha_ob25_seg1[1] <= 1.0000301090000001e+06
 R871: - C5 + 1e+06 alpha_ob25_seg1[1] <= 1.0000301090000001e+06
 R872: C2 + 1e+06 alpha_ob25_seg1[2] <= 1.0000252838e+06
 R873: C4 + 1e+06 alpha_ob25_seg1[2] <= 1.0000252838e+06
 R874: C3 + 1e+06 alpha_ob25_seg1[3] <= 999967.341
 R875: C5 + 1e+06 alpha_ob25_seg1[3] <= 999967.341
 R876: alpha_ob25_seg1[0] + alpha_ob25_seg1[1] + alpha_ob25_seg1[2]
   + alpha_ob25_seg1[3] >= 1
 R877: - C2 + 1e+06 alpha_ob26_seg1[0] <= 999964.9544
 R878: - C4 + 1e+06 alpha_ob26_seg1[0] <= 999964.9544
 R879: - C3 + 1e+06 alpha_ob26_seg1[1] <= 1.0000233395999999e+06
 R880: - C5 + 1e+06 alpha_ob26_seg1[1] <= 1.0000233395999999e+06
 R881: C2 + 1e+06 alpha_ob26_seg1[2] <= 1.0000251456e+06
 R882: C4 + 1e+06 alpha_ob26_seg1[2] <= 1.0000251456e+06
 R883: C3 + 1e+06 alpha_ob26_seg1[3] <= 999974.1104
 R884: C5 + 1e+06 alpha_ob26_seg1[3] <= 999974.1104
 R885: alpha_ob26_seg1[0] + alpha_ob26_seg1[1] + alpha_ob26_seg1[2]
   + alpha_ob26_seg1[3] >= 1
 R886: - C2 + 1e+06 alpha_ob27_seg1[0] <= 999931.4945
 R887: - C4 + 1e+06 alpha_ob27_seg1[0] <= 999931.4945
 R888: - C3 + 1e+06 alpha_ob27_seg1[1] <= 1.0000158794e+06
 R889: - C5 + 1e+06 alpha_ob27_seg1[1] <= 1.0000158794e+06
 R890: C2 + 1e+06 alpha_ob27_seg1[2] <= 1.0000378554999999e+06
 R891: C4 + 1e+06 alpha_ob27_seg1[2] <= 1.0000378554999999e+06
 R892: C3 + 1e+06 alpha_ob27_seg1[3] <= 999981.5706
 R893: C5 + 1e+06 alpha_ob27_seg1[3] <= 999981.5706
 R894: alpha_ob27_seg1[0] + alpha_ob27_seg1[1] + alpha_ob27_seg1[2]
   + alpha_ob27_seg1[3] >= 1
 R895: - C2 + 1e+06 alpha_ob28_seg1[0] <= 999953.8919709784
 R896: - C4 + 1e+06 alpha_ob28_seg1[0] <= 999953.8919709784
 R897: - C3 + 1e+06 alpha_ob28_seg1[1] <= 1.0000230754474958e+06
 R898: - C5 + 1e+06 alpha_ob28_seg1[1] <= 1.0000230754474958e+06
 R899: C2 + 1e+06 alpha_ob28_seg1[2] <= 1.0000383703709784e+06
 R900: C4 + 1e+06 alpha_ob28_seg1[2] <= 1.0000383703709784e+06
 R901: C3 + 1e+06 alpha_ob28_seg1[3] <= 999971.2696474958
 R902: C5 + 1e+06 alpha_ob28_seg1[3] <= 999971.2696474958
 R903: alpha_ob28_seg1[0] + alpha_ob28_seg1[1] + alpha_ob28_seg1[2]
   + alpha_ob28_seg1[3] >= 1
 R904: - C2 + 1e+06 alpha_ob29_seg1[0] <= 999947.4507336203
 R905: - C4 + 1e+06 alpha_ob29_seg1[0] <= 999947.4507336203
 R906: - C3 + 1e+06 alpha_ob29_seg1[1] <= 1.000021362399544e+06
 R907: - C5 + 1e+06 alpha_ob29_seg1[1] <= 1.000021362399544e+06
 R908: C2 + 1e+06 alpha_ob29_seg1[2] <= 1.0000443925336204e+06
 R909: C4 + 1e+06 alpha_ob29_seg1[2] <= 1.0000443925336204e+06
 R910: C3 + 1e+06 alpha_ob29_seg1[3] <= 999974.5397995439
 R911: C5 + 1e+06 alpha_ob29_seg1[3] <= 999974.5397995439
 R912: alpha_ob29_seg1[0] + alpha_ob29_seg1[1] + alpha_ob29_seg1[2]
   + alpha_ob29_seg1[3] >= 1
 R913: - C2 + 1e+06 alpha_ob30_seg1[0] <= 999956.8394495883
 R914: - C4 + 1e+06 alpha_ob30_seg1[0] <= 999956.8394495883
 R915: - C3 + 1e+06 alpha_ob30_seg1[1] <= 999925.3962997246
 R916: - C5 + 1e+06 alpha_ob30_seg1[1] <= 999925.3962997246
 R917: C2 + 1e+06 alpha_ob30_seg1[2] <= 1.0000406106495883e+06
 R918: C4 + 1e+06 alpha_ob30_seg1[2] <= 1.0000406106495883e+06
 R919: C3 + 1e+06 alpha_ob30_seg1[3] <= 1.0000452036997246e+06
 R920: C5 + 1e+06 alpha_ob30_seg1[3] <= 1.0000452036997246e+06
 R921: alpha_ob30_seg1[0] + alpha_ob30_seg1[1] + alpha_ob30_seg1[2]
   + alpha_ob30_seg1[3] >= 1
 R922: - C2 + 1e+06 alpha_ob31_seg1[0] <= 999941.1221928794
 R923: - C4 + 1e+06 alpha_ob31_seg1[0] <= 999941.1221928794
 R924: - C3 + 1e+06 alpha_ob31_seg1[1] <= 1.0000191770472102e+06
 R925: - C5 + 1e+06 alpha_ob31_seg1[1] <= 1.0000191770472102e+06
 R926: C2 + 1e+06 alpha_ob31_seg1[2] <= 1.0000504795928795e+06
 R927: C4 + 1e+06 alpha_ob31_seg1[2] <= 1.0000504795928795e+06
 R928: C3 + 1e+06 alpha_ob31_seg1[3] <= 999976.6604472102
 R929: C5 + 1e+06 alpha_ob31_seg1[3] <= 999976.6604472102
 R930: alpha_ob31_seg1[0] + alpha_ob31_seg1[1] + alpha_ob31_seg1[2]
   + alpha_ob31_seg1[3] >= 1
 R931: - C2 + 1e+06 alpha_ob32_seg1[0] <= 999934.8446336203
 R932: - C4 + 1e+06 alpha_ob32_seg1[0] <= 999934.8446336203
 R933: - C3 + 1e+06 alpha_ob32_seg1[1] <= 1.000021048199544e+06
 R934: - C5 + 1e+06 alpha_ob32_seg1[1] <= 1.000021048199544e+06
 R935: C2 + 1e+06 alpha_ob32_seg1[2] <= 1.0000569986336203e+06
 R936: C4 + 1e+06 alpha_ob32_seg1[2] <= 1.0000569986336203e+06
 R937: C3 + 1e+06 alpha_ob32_seg1[3] <= 999974.8539995439
 R938: C5 + 1e+06 alpha_ob32_seg1[3] <= 999974.8539995439
 R939: alpha_ob32_seg1[0] + alpha_ob32_seg1[1] + alpha_ob32_seg1[2]
   + alpha_ob32_seg1[3] >= 1
 R940: - C2 + 1e+06 alpha_ob33_seg1[0] <= 999956.839399801
 R941: - C4 + 1e+06 alpha_ob33_seg1[0] <= 999956.839399801
 R942: - C3 + 1e+06 alpha_ob33_seg1[1] <= 999925.3963434525
 R943: - C5 + 1e+06 alpha_ob33_seg1[1] <= 999925.3963434525
 R944: C2 + 1e+06 alpha_ob33_seg1[2] <= 1.0000080105998011e+06
 R945: C4 + 1e+06 alpha_ob33_seg1[2] <= 1.0000080105998011e+06
 R946: C3 + 1e+06 alpha_ob33_seg1[3] <= 1.0000720537434525e+06
 R947: C5 + 1e+06 alpha_ob33_seg1[3] <= 1.0000720537434525e+06
 R948: alpha_ob33_seg1[0] + alpha_ob33_seg1[1] + alpha_ob33_seg1[2]
   + alpha_ob33_seg1[3] >= 1
 R949: - C2 + 1e+06 alpha_ob34_seg1[0] <= 999958.8717898987
 R950: - C4 + 1e+06 alpha_ob34_seg1[0] <= 999958.8717898987
 R951: - C3 + 1e+06 alpha_ob34_seg1[1] <= 999987.7837602755
 R952: - C5 + 1e+06 alpha_ob34_seg1[1] <= 999987.7837602755
 R953: C2 + 1e+06 alpha_ob34_seg1[2] <= 1.0000385781898986e+06
 R954: C4 + 1e+06 alpha_ob34_seg1[2] <= 1.0000385781898986e+06
 R955: C3 + 1e+06 alpha_ob34_seg1[3] <= 1.0000043162402755e+06
 R956: C5 + 1e+06 alpha_ob34_seg1[3] <= 1.0000043162402755e+06
 R957: alpha_ob34_seg1[0] + alpha_ob34_seg1[1] + alpha_ob34_seg1[2]
   + alpha_ob34_seg1[3] >= 1
 R958: - C2 + 1e+06 alpha_ob35_seg1[0] <= 999939.2718
 R959: - C4 + 1e+06 alpha_ob35_seg1[0] <= 999939.2718
 R960: - C3 + 1e+06 alpha_ob35_seg1[1] <= 999993.13376
 R961: - C5 + 1e+06 alpha_ob35_seg1[1] <= 999993.13376
 R962: C2 + 1e+06 alpha_ob35_seg1[2] <= 1.0000385782e+06
 R963: C4 + 1e+06 alpha_ob35_seg1[2] <= 1.0000385782e+06
 R964: C3 + 1e+06 alpha_ob35_seg1[3] <= 1.00000431624e+06
 R965: C5 + 1e+06 alpha_ob35_seg1[3] <= 1.00000431624e+06
 R966: alpha_ob35_seg1[0] + alpha_ob35_seg1[1] + alpha_ob35_seg1[2]
   + alpha_ob35_seg1[3] >= 1
 R967: - C2 + 1e+06 alpha_ob36_seg1[0] <= 999939.2718101013
 R968: - C4 + 1e+06 alpha_ob36_seg1[0] <= 999939.2718101013
 R969: - C3 + 1e+06 alpha_ob36_seg1[1] <= 999987.7837597246
 R970: - C5 + 1e+06 alpha_ob36_seg1[1] <= 999987.7837597246
 R971: C2 + 1e+06 alpha_ob36_seg1[2] <= 1.0000581782101013e+06
 R972: C4 + 1e+06 alpha_ob36_seg1[2] <= 1.0000581782101013e+06
 R973: C3 + 1e+06 alpha_ob36_seg1[3] <= 1.0000043162397245e+06
 R974: C5 + 1e+06 alpha_ob36_seg1[3] <= 1.0000043162397245e+06
 R975: alpha_ob36_seg1[0] + alpha_ob36_seg1[1] + alpha_ob36_seg1[2]
   + alpha_ob36_seg1[3] >= 1
 R976: - C2 + 1e+06 alpha_ob37_seg1[0] <= 999939.271799801
 R977: - C4 + 1e+06 alpha_ob37_seg1[0] <= 999939.271799801
 R978: - C3 + 1e+06 alpha_ob37_seg1[1] <= 999987.7838262042
 R979: - C5 + 1e+06 alpha_ob37_seg1[1] <= 999987.7838262042
 R980: C2 + 1e+06 alpha_ob37_seg1[2] <= 1.000038578199801e+06
 R981: C4 + 1e+06 alpha_ob37_seg1[2] <= 1.000038578199801e+06
 R982: C3 + 1e+06 alpha_ob37_seg1[3] <= 1.0000096662262043e+06
 R983: C5 + 1e+06 alpha_ob37_seg1[3] <= 1.0000096662262043e+06
 R984: alpha_ob37_seg1[0] + alpha_ob37_seg1[1] + alpha_ob37_seg1[2]
   + alpha_ob37_seg1[3] >= 1
 R985: - C2 + 1e+06 alpha_ob38_seg1[0] <= 999972.6917508709
 R986: - C4 + 1e+06 alpha_ob38_seg1[0] <= 999972.6917508709
 R987: - C3 + 1e+06 alpha_ob38_seg1[1] <= 999957.1644002756
 R988: - C5 + 1e+06 alpha_ob38_seg1[1] <= 999957.1644002756
 R989: C2 + 1e+06 alpha_ob38_seg1[2] <= 1.0000247581508709e+06
 R990: C4 + 1e+06 alpha_ob38_seg1[2] <= 1.0000247581508709e+06
 R991: C3 + 1e+06 alpha_ob38_seg1[3] <= 1.0000136856002755e+06
 R992: C5 + 1e+06 alpha_ob38_seg1[3] <= 1.0000136856002755e+06
 R993: alpha_ob38_seg1[0] + alpha_ob38_seg1[1] + alpha_ob38_seg1[2]
   + alpha_ob38_seg1[3] >= 1
 R994: - C2 + 1e+06 alpha_ob39_seg1[0] <= 999925.8418000001
 R995: - C4 + 1e+06 alpha_ob39_seg1[0] <= 999925.8418000001
 R996: - C3 + 1e+06 alpha_ob39_seg1[1] <= 999957.1644
 R997: - C5 + 1e+06 alpha_ob39_seg1[1] <= 999957.1644
 R998: C2 + 1e+06 alpha_ob39_seg1[2] <= 1.0000247582e+06
 R999: C4 + 1e+06 alpha_ob39_seg1[2] <= 1.0000247582e+06
 R1000: C3 + 1e+06 alpha_ob39_seg1[3] <= 1.0000402855999999e+06
 R1001: C5 + 1e+06 alpha_ob39_seg1[3] <= 1.0000402855999999e+06
 R1002: alpha_ob39_seg1[0] + alpha_ob39_seg1[1] + alpha_ob39_seg1[2]
   + alpha_ob39_seg1[3] >= 1
 R1003: - C2 + 1e+06 alpha_ob40_seg1[0] <= 999925.8417898987
 R1004: - C4 + 1e+06 alpha_ob40_seg1[0] <= 999925.8417898987
 R1005: - C3 + 1e+06 alpha_ob40_seg1[1] <= 999957.1644002756
 R1006: - C5 + 1e+06 alpha_ob40_seg1[1] <= 999957.1644002756
 R1007: C2 + 1e+06 alpha_ob40_seg1[2] <= 1.0000716081898987e+06
 R1008: C4 + 1e+06 alpha_ob40_seg1[2] <= 1.0000716081898987e+06
 R1009: C3 + 1e+06 alpha_ob40_seg1[3] <= 1.0000349356002755e+06
 R1010: C5 + 1e+06 alpha_ob40_seg1[3] <= 1.0000349356002755e+06
 R1011: alpha_ob40_seg1[0] + alpha_ob40_seg1[1] + alpha_ob40_seg1[2]
   + alpha_ob40_seg1[3] >= 1
 R1012: - C2 + 1e+06 alpha_ob41_seg1[0] <= 999939.0009674003
 R1013: - C4 + 1e+06 alpha_ob41_seg1[0] <= 999939.0009674003
 R1014: - C3 + 1e+06 alpha_ob41_seg1[1] <= 999961.9516002756
 R1015: - C5 + 1e+06 alpha_ob41_seg1[1] <= 999961.9516002756
 R1016: C2 + 1e+06 alpha_ob41_seg1[2] <= 1.0000584489674004e+06
 R1017: C4 + 1e+06 alpha_ob41_seg1[2] <= 1.0000584489674004e+06
 R1018: C3 + 1e+06 alpha_ob41_seg1[3] <= 1.0000178984002755e+06
 R1019: C5 + 1e+06 alpha_ob41_seg1[3] <= 1.0000178984002755e+06
 R1020: alpha_ob41_seg1[0] + alpha_ob41_seg1[1] + alpha_ob41_seg1[2]
   + alpha_ob41_seg1[3] >= 1
 R1021: - C2 + 1e+06 alpha_ob42_seg1[0] <= 999925.901
 R1022: - C4 + 1e+06 alpha_ob42_seg1[0] <= 999925.901
 R1023: - C3 + 1e+06 alpha_ob42_seg1[1] <= 999961.9516
 R1024: - C5 + 1e+06 alpha_ob42_seg1[1] <= 999961.9516
 R1025: C2 + 1e+06 alpha_ob42_seg1[2] <= 1.000058449e+06
 R1026: C4 + 1e+06 alpha_ob42_seg1[2] <= 1.000058449e+06
 R1027: C3 + 1e+06 alpha_ob42_seg1[3] <= 1.0000354984e+06
 R1028: C5 + 1e+06 alpha_ob42_seg1[3] <= 1.0000354984e+06
 R1029: alpha_ob42_seg1[0] + alpha_ob42_seg1[1] + alpha_ob42_seg1[2]
   + alpha_ob42_seg1[3] >= 1
 R1030: - C2 + 1e+06 alpha_ob43_seg1[0] <= 999919.2652454559
 R1031: - C4 + 1e+06 alpha_ob43_seg1[0] <= 999919.2652454559
 R1032: - C3 + 1e+06 alpha_ob43_seg1[1] <= 999956.8341997246
 R1033: - C5 + 1e+06 alpha_ob43_seg1[1] <= 999956.8341997246
 R1034: C2 + 1e+06 alpha_ob43_seg1[2] <= 1.0000781848454559e+06
 R1035: C4 + 1e+06 alpha_ob43_seg1[2] <= 1.0000781848454559e+06
 R1036: C3 + 1e+06 alpha_ob43_seg1[3] <= 1.0000160157997246e+06
 R1037: C5 + 1e+06 alpha_ob43_seg1[3] <= 1.0000160157997246e+06
 R1038: alpha_ob43_seg1[0] + alpha_ob43_seg1[1] + alpha_ob43_seg1[2]
   + alpha_ob43_seg1[3] >= 1
 R1039: - C2 + 1e+06 alpha_ob44_seg1[0] <= 1.0000699118999999e+06
 R1040: - C4 + 1e+06 alpha_ob44_seg1[0] <= 1.0000699118999999e+06
 R1041: - C3 + 1e+06 alpha_ob44_seg1[1] <= 999952.9001
 R1042: - C5 + 1e+06 alpha_ob44_seg1[1] <= 999952.9001
 R1043: C2 + 1e+06 alpha_ob44_seg1[2] <= 999913.6881
 R1044: C4 + 1e+06 alpha_ob44_seg1[2] <= 999913.6881
 R1045: C3 + 1e+06 alpha_ob44_seg1[3] <= 1.0000445499e+06
 R1046: C5 + 1e+06 alpha_ob44_seg1[3] <= 1.0000445499e+06
 R1047: alpha_ob44_seg1[0] + alpha_ob44_seg1[1] + alpha_ob44_seg1[2]
   + alpha_ob44_seg1[3] >= 1
 R1048: - C2 + 1e+06 alpha_ob45_seg1[0] <= 999919.3751998009
 R1049: - C4 + 1e+06 alpha_ob45_seg1[0] <= 999919.3751998009
 R1050: - C3 + 1e+06 alpha_ob45_seg1[1] <= 999957.1154039804
 R1051: - C5 + 1e+06 alpha_ob45_seg1[1] <= 999957.1154039804
 R1052: C2 + 1e+06 alpha_ob45_seg1[2] <= 1.000075224799801e+06
 R1053: C4 + 1e+06 alpha_ob45_seg1[2] <= 1.000075224799801e+06
 R1054: C3 + 1e+06 alpha_ob45_seg1[3] <= 1.0000403346039804e+06
 R1055: C5 + 1e+06 alpha_ob45_seg1[3] <= 1.0000403346039804e+06
 R1056: alpha_ob45_seg1[0] + alpha_ob45_seg1[1] + alpha_ob45_seg1[2]
   + alpha_ob45_seg1[3] >= 1
 R1057: - C2 + 1e+06 alpha_ob46_seg1[0] <= 1.0000668119e+06
 R1058: - C4 + 1e+06 alpha_ob46_seg1[0] <= 1.0000668119e+06
 R1059: - C3 + 1e+06 alpha_ob46_seg1[1] <= 999952.9001
 R1060: - C5 + 1e+06 alpha_ob46_seg1[1] <= 999952.9001
 R1061: C2 + 1e+06 alpha_ob46_seg1[2] <= 999927.5381
 R1062: C4 + 1e+06 alpha_ob46_seg1[2] <= 999927.5381
 R1063: C3 + 1e+06 alpha_ob46_seg1[3] <= 1.0000445499e+06
 R1064: C5 + 1e+06 alpha_ob46_seg1[3] <= 1.0000445499e+06
 R1065: alpha_ob46_seg1[0] + alpha_ob46_seg1[1] + alpha_ob46_seg1[2]
   + alpha_ob46_seg1[3] >= 1
 R1066: - C2 + 1e+06 alpha_ob47_seg1[0] <= 1.0000668119215801e+06
 R1067: - C4 + 1e+06 alpha_ob47_seg1[0] <= 1.0000668119215801e+06
 R1068: - C3 + 1e+06 alpha_ob47_seg1[1] <= 999941.3000997246
 R1069: - C5 + 1e+06 alpha_ob47_seg1[1] <= 999941.3000997246
 R1070: C2 + 1e+06 alpha_ob47_seg1[2] <= 999930.6381215801
 R1071: C4 + 1e+06 alpha_ob47_seg1[2] <= 999930.6381215801
 R1072: C3 + 1e+06 alpha_ob47_seg1[3] <= 1.0000445498997245e+06
 R1073: C5 + 1e+06 alpha_ob47_seg1[3] <= 1.0000445498997245e+06
 R1074: alpha_ob47_seg1[0] + alpha_ob47_seg1[1] + alpha_ob47_seg1[2]
   + alpha_ob47_seg1[3] >= 1
 R1075: - C2 + 1e+06 alpha_ob48_seg1[0] <= 1.0000146737e+06
 R1076: - C4 + 1e+06 alpha_ob48_seg1[0] <= 1.0000146737e+06
 R1077: - C3 + 1e+06 alpha_ob48_seg1[1] <= 999940.7624
 R1078: - C5 + 1e+06 alpha_ob48_seg1[1] <= 999940.7624
 R1079: C2 + 1e+06 alpha_ob48_seg1[2] <= 999948.4263000001
 R1080: C4 + 1e+06 alpha_ob48_seg1[2] <= 999948.4263000001
 R1081: C3 + 1e+06 alpha_ob48_seg1[3] <= 1.0000566875999999e+06
 R1082: C5 + 1e+06 alpha_ob48_seg1[3] <= 1.0000566875999999e+06
 R1083: alpha_ob48_seg1[0] + alpha_ob48_seg1[1] + alpha_ob48_seg1[2]
   + alpha_ob48_seg1[3] >= 1
 R1084: - C2 + 1e+06 alpha_ob49_seg1[0] <= 1.0000537119e+06
 R1085: - C4 + 1e+06 alpha_ob49_seg1[0] <= 1.0000537119e+06
 R1086: - C3 + 1e+06 alpha_ob49_seg1[1] <= 999941.3002000001
 R1087: - C5 + 1e+06 alpha_ob49_seg1[1] <= 999941.3002000001
 R1088: C2 + 1e+06 alpha_ob49_seg1[2] <= 999930.6381
 R1089: C4 + 1e+06 alpha_ob49_seg1[2] <= 999930.6381
 R1090: C3 + 1e+06 alpha_ob49_seg1[3] <= 1.0000561498e+06
 R1091: C5 + 1e+06 alpha_ob49_seg1[3] <= 1.0000561498e+06
 R1092: alpha_ob49_seg1[0] + alpha_ob49_seg1[1] + alpha_ob49_seg1[2]
   + alpha_ob49_seg1[3] >= 1
 R1093: - C2 + 1e+06 alpha_ob50_seg1[0] <= 1.0000058189e+06
 R1094: - C4 + 1e+06 alpha_ob50_seg1[0] <= 1.0000058189e+06
 R1095: - C3 + 1e+06 alpha_ob50_seg1[1] <= 999940.5469
 R1096: - C5 + 1e+06 alpha_ob50_seg1[1] <= 999940.5469
 R1097: C2 + 1e+06 alpha_ob50_seg1[2] <= 999985.5311
 R1098: C4 + 1e+06 alpha_ob50_seg1[2] <= 999985.5311
 R1099: C3 + 1e+06 alpha_ob50_seg1[3] <= 1.0000569031e+06
 R1100: C5 + 1e+06 alpha_ob50_seg1[3] <= 1.0000569031e+06
 R1101: alpha_ob50_seg1[0] + alpha_ob50_seg1[1] + alpha_ob50_seg1[2]
   + alpha_ob50_seg1[3] >= 1
 R1102: - C2 + 1e+06 alpha_ob51_seg1[0] <= 1.0000058188284199e+06
 R1103: - C4 + 1e+06 alpha_ob51_seg1[0] <= 1.0000058188284199e+06
 R1104: - C3 + 1e+06 alpha_ob51_seg1[1] <= 999940.5469002755
 R1105: - C5 + 1e+06 alpha_ob51_seg1[1] <= 999940.5469002755
 R1106: C2 + 1e+06 alpha_ob51_seg1[2] <= 999991.6311284199
 R1107: C4 + 1e+06 alpha_ob51_seg1[2] <= 999991.6311284199
 R1108: C3 + 1e+06 alpha_ob51_seg1[3] <= 1.0000453031002756e+06
 R1109: C5 + 1e+06 alpha_ob51_seg1[3] <= 1.0000453031002756e+06
 R1110: alpha_ob51_seg1[0] + alpha_ob51_seg1[1] + alpha_ob51_seg1[2]
   + alpha_ob51_seg1[3] >= 1
 R1111: - C2 + 1e+06 alpha_ob52_seg1[0] <= 999996.9688499999
 R1112: - C4 + 1e+06 alpha_ob52_seg1[0] <= 999996.9688499999
 R1113: - C3 + 1e+06 alpha_ob52_seg1[1] <= 999940.5469
 R1114: - C5 + 1e+06 alpha_ob52_seg1[1] <= 999940.5469
 R1115: C2 + 1e+06 alpha_ob52_seg1[2] <= 999991.63115
 R1116: C4 + 1e+06 alpha_ob52_seg1[2] <= 999991.63115
 R1117: C3 + 1e+06 alpha_ob52_seg1[3] <= 1.0000569031e+06
 R1118: C5 + 1e+06 alpha_ob52_seg1[3] <= 1.0000569031e+06
 R1119: alpha_ob52_seg1[0] + alpha_ob52_seg1[1] + alpha_ob52_seg1[2]
   + alpha_ob52_seg1[3] >= 1
 R1120: - C2 + 1e+06 alpha_ob53_seg1[0] <= 999991.83683
 R1121: - C4 + 1e+06 alpha_ob53_seg1[0] <= 999991.83683
 R1122: - C3 + 1e+06 alpha_ob53_seg1[1] <= 999940.589
 R1123: - C5 + 1e+06 alpha_ob53_seg1[1] <= 999940.589
 R1124: C2 + 1e+06 alpha_ob53_seg1[2] <= 1.00000401317e+06
 R1125: C4 + 1e+06 alpha_ob53_seg1[2] <= 1.00000401317e+06
 R1126: C3 + 1e+06 alpha_ob53_seg1[3] <= 1.000056861e+06
 R1127: C5 + 1e+06 alpha_ob53_seg1[3] <= 1.000056861e+06
 R1128: alpha_ob53_seg1[0] + alpha_ob53_seg1[1] + alpha_ob53_seg1[2]
   + alpha_ob53_seg1[3] >= 1
 R1129: - C2 + 1e+06 alpha_ob54_seg1[0] <= 999991.8368084199
 R1130: - C4 + 1e+06 alpha_ob54_seg1[0] <= 999991.8368084199
 R1131: - C3 + 1e+06 alpha_ob54_seg1[1] <= 999940.5890002755
 R1132: - C5 + 1e+06 alpha_ob54_seg1[1] <= 999940.5890002755
 R1133: C2 + 1e+06 alpha_ob54_seg1[2] <= 1.0000056131484199e+06
 R1134: C4 + 1e+06 alpha_ob54_seg1[2] <= 1.0000056131484199e+06
 R1135: C3 + 1e+06 alpha_ob54_seg1[3] <= 1.0000452610002755e+06
 R1136: C5 + 1e+06 alpha_ob54_seg1[3] <= 1.0000452610002755e+06
 R1137: alpha_ob54_seg1[0] + alpha_ob54_seg1[1] + alpha_ob54_seg1[2]
   + alpha_ob54_seg1[3] >= 1
 R1138: - C2 + 1e+06 alpha_ob55_seg1[0] <= 999910.5736
 R1139: - C4 + 1e+06 alpha_ob55_seg1[0] <= 999910.5736
 R1140: - C3 + 1e+06 alpha_ob55_seg1[1] <= 999926.115
 R1141: - C5 + 1e+06 alpha_ob55_seg1[1] <= 999926.115
 R1142: C2 + 1e+06 alpha_ob55_seg1[2] <= 1.0000467764e+06
 R1143: C4 + 1e+06 alpha_ob55_seg1[2] <= 1.0000467764e+06
 R1144: C3 + 1e+06 alpha_ob55_seg1[3] <= 1.000071335e+06
 R1145: C5 + 1e+06 alpha_ob55_seg1[3] <= 1.000071335e+06
 R1146: alpha_ob55_seg1[0] + alpha_ob55_seg1[1] + alpha_ob55_seg1[2]
   + alpha_ob55_seg1[3] >= 1
 R1147: - C2 + 1e+06 alpha_ob56_seg1[0] <= 999910.5735508709
 R1148: - C4 + 1e+06 alpha_ob56_seg1[0] <= 999910.5735508709
 R1149: - C3 + 1e+06 alpha_ob56_seg1[1] <= 999926.1150002755
 R1150: - C5 + 1e+06 alpha_ob56_seg1[1] <= 999926.1150002755
 R1151: C2 + 1e+06 alpha_ob56_seg1[2] <= 1.0000868763508709e+06
 R1152: C4 + 1e+06 alpha_ob56_seg1[2] <= 1.0000868763508709e+06
 R1153: C3 + 1e+06 alpha_ob56_seg1[3] <= 1.0000447350002755e+06
 R1154: C5 + 1e+06 alpha_ob56_seg1[3] <= 1.0000447350002755e+06
 R1155: alpha_ob56_seg1[0] + alpha_ob56_seg1[1] + alpha_ob56_seg1[2]
   + alpha_ob56_seg1[3] >= 1
 R1156: - C2 + 1e+06 alpha_ob57_seg1[0] <= 1.0000537118880621e+06
 R1157: - C4 + 1e+06 alpha_ob57_seg1[0] <= 1.0000537118880621e+06
 R1158: - C3 + 1e+06 alpha_ob57_seg1[1] <= 999941.3001002755
 R1159: - C5 + 1e+06 alpha_ob57_seg1[1] <= 999941.3001002755
 R1160: C2 + 1e+06 alpha_ob57_seg1[2] <= 999943.738088062
 R1161: C4 + 1e+06 alpha_ob57_seg1[2] <= 999943.738088062
 R1162: C3 + 1e+06 alpha_ob57_seg1[3] <= 1.0000497999002755e+06
 R1163: C5 + 1e+06 alpha_ob57_seg1[3] <= 1.0000497999002755e+06
 R1164: alpha_ob57_seg1[0] + alpha_ob57_seg1[1] + alpha_ob57_seg1[2]
   + alpha_ob57_seg1[3] >= 1
 R1165: - C2 + 1e+06 alpha_ob58_seg1[0] <= 999910.573599801
 R1166: - C4 + 1e+06 alpha_ob58_seg1[0] <= 999910.573599801
 R1167: - C3 + 1e+06 alpha_ob58_seg1[1] <= 999952.7150152582
 R1168: - C5 + 1e+06 alpha_ob58_seg1[1] <= 999952.7150152582
 R1169: C2 + 1e+06 alpha_ob58_seg1[2] <= 1.000075526399801e+06
 R1170: C4 + 1e+06 alpha_ob58_seg1[2] <= 1.000075526399801e+06
 R1171: C3 + 1e+06 alpha_ob58_seg1[3] <= 1.0000447350152582e+06
 R1172: C5 + 1e+06 alpha_ob58_seg1[3] <= 1.0000447350152582e+06
 R1173: alpha_ob58_seg1[0] + alpha_ob58_seg1[1] + alpha_ob58_seg1[2]
   + alpha_ob58_seg1[3] >= 1
 R1174: - C2 + 1e+06 alpha_ob59_seg1[0] <= 999921.6867474595
 R1175: - C4 + 1e+06 alpha_ob59_seg1[0] <= 999921.6867474595
 R1176: - C3 + 1e+06 alpha_ob59_seg1[1] <= 999951.9159478974
 R1177: - C5 + 1e+06 alpha_ob59_seg1[1] <= 999951.9159478974
 R1178: C2 + 1e+06 alpha_ob59_seg1[2] <= 1.0000755095474594e+06
 R1179: C4 + 1e+06 alpha_ob59_seg1[2] <= 1.0000755095474594e+06
 R1180: C3 + 1e+06 alpha_ob59_seg1[3] <= 1.0000447569478974e+06
 R1181: C5 + 1e+06 alpha_ob59_seg1[3] <= 1.0000447569478974e+06
 R1182: alpha_ob59_seg1[0] + alpha_ob59_seg1[1] + alpha_ob59_seg1[2]
   + alpha_ob59_seg1[3] >= 1
 R1183: - C2 + 1e+06 alpha_ob60_seg1[0] <= 999921.7035998009
 R1184: - C4 + 1e+06 alpha_ob60_seg1[0] <= 999921.7035998009
 R1185: - C3 + 1e+06 alpha_ob60_seg1[1] <= 999951.8939069656
 R1186: - C5 + 1e+06 alpha_ob60_seg1[1] <= 999951.8939069656
 R1187: C2 + 1e+06 alpha_ob60_seg1[2] <= 1.000070646399801e+06
 R1188: C4 + 1e+06 alpha_ob60_seg1[2] <= 1.000070646399801e+06
 R1189: C3 + 1e+06 alpha_ob60_seg1[3] <= 1.0000455561069656e+06
 R1190: C5 + 1e+06 alpha_ob60_seg1[3] <= 1.0000455561069656e+06
 R1191: alpha_ob60_seg1[0] + alpha_ob60_seg1[1] + alpha_ob60_seg1[2]
   + alpha_ob60_seg1[3] >= 1
 R1192: - C2 + 1e+06 alpha_ob61_seg1[0] <= 999950.6735522483
 R1193: - C4 + 1e+06 alpha_ob61_seg1[0] <= 999950.6735522483
 R1194: - C3 + 1e+06 alpha_ob61_seg1[1] <= 999926.1150002755
 R1195: - C5 + 1e+06 alpha_ob61_seg1[1] <= 999926.1150002755
 R1196: C2 + 1e+06 alpha_ob61_seg1[2] <= 1.0000467763522484e+06
 R1197: C4 + 1e+06 alpha_ob61_seg1[2] <= 1.0000467763522484e+06
 R1198: C3 + 1e+06 alpha_ob61_seg1[3] <= 1.0000454850002755e+06
 R1199: C5 + 1e+06 alpha_ob61_seg1[3] <= 1.0000454850002755e+06
 R1200: alpha_ob61_seg1[0] + alpha_ob61_seg1[1] + alpha_ob61_seg1[2]
   + alpha_ob61_seg1[3] >= 1
 R1201: - C2 + 1e+06 alpha_ob62_seg1[0] <= 999941.3236
 R1202: - C4 + 1e+06 alpha_ob62_seg1[0] <= 999941.3236
 R1203: - C3 + 1e+06 alpha_ob62_seg1[1] <= 999951.965
 R1204: - C5 + 1e+06 alpha_ob62_seg1[1] <= 999951.965
 R1205: C2 + 1e+06 alpha_ob62_seg1[2] <= 1.0000467764e+06
 R1206: C4 + 1e+06 alpha_ob62_seg1[2] <= 1.0000467764e+06
 R1207: C3 + 1e+06 alpha_ob62_seg1[3] <= 1.000045485e+06
 R1208: C5 + 1e+06 alpha_ob62_seg1[3] <= 1.000045485e+06
 R1209: alpha_ob62_seg1[0] + alpha_ob62_seg1[1] + alpha_ob62_seg1[2]
   + alpha_ob62_seg1[3] >= 1
 R1210: - C2 + 1e+06 alpha_ob63_seg1[0] <= 999941.3236073464
 R1211: - C4 + 1e+06 alpha_ob63_seg1[0] <= 999941.3236073464
 R1212: - C3 + 1e+06 alpha_ob63_seg1[1] <= 999948.1149997246
 R1213: - C5 + 1e+06 alpha_ob63_seg1[1] <= 999948.1149997246
 R1214: C2 + 1e+06 alpha_ob63_seg1[2] <= 1.0000561264073465e+06
 R1215: C4 + 1e+06 alpha_ob63_seg1[2] <= 1.0000561264073465e+06
 R1216: C3 + 1e+06 alpha_ob63_seg1[3] <= 1.0000454849997245e+06
 R1217: C5 + 1e+06 alpha_ob63_seg1[3] <= 1.0000454849997245e+06
 R1218: alpha_ob63_seg1[0] + alpha_ob63_seg1[1] + alpha_ob63_seg1[2]
   + alpha_ob63_seg1[3] >= 1
 R1219: - C2 + 1e+06 alpha_ob64_seg1[0] <= 999931.7236
 R1220: - C4 + 1e+06 alpha_ob64_seg1[0] <= 999931.7236
 R1221: - C3 + 1e+06 alpha_ob64_seg1[1] <= 999948.115
 R1222: - C5 + 1e+06 alpha_ob64_seg1[1] <= 999948.115
 R1223: C2 + 1e+06 alpha_ob64_seg1[2] <= 1.0000561264e+06
 R1224: C4 + 1e+06 alpha_ob64_seg1[2] <= 1.0000561264e+06
 R1225: C3 + 1e+06 alpha_ob64_seg1[3] <= 1.000049335e+06
 R1226: C5 + 1e+06 alpha_ob64_seg1[3] <= 1.000049335e+06
 R1227: alpha_ob64_seg1[0] + alpha_ob64_seg1[1] + alpha_ob64_seg1[2]
   + alpha_ob64_seg1[3] >= 1
 R1228: - C2 + 1e+06 alpha_ob65_seg1[0] <= 999931.7235921945
 R1229: - C4 + 1e+06 alpha_ob65_seg1[0] <= 999931.7235921945
 R1230: - C3 + 1e+06 alpha_ob65_seg1[1] <= 999948.1150002755
 R1231: - C5 + 1e+06 alpha_ob65_seg1[1] <= 999948.1150002755
 R1232: C2 + 1e+06 alpha_ob65_seg1[2] <= 1.0000657263921944e+06
 R1233: C4 + 1e+06 alpha_ob65_seg1[2] <= 1.0000657263921944e+06
 R1234: C3 + 1e+06 alpha_ob65_seg1[3] <= 1.0000452350002755e+06
 R1235: C5 + 1e+06 alpha_ob65_seg1[3] <= 1.0000452350002755e+06
 R1236: alpha_ob65_seg1[0] + alpha_ob65_seg1[1] + alpha_ob65_seg1[2]
   + alpha_ob65_seg1[3] >= 1
 R1237: - C2 + 1e+06 alpha_ob66_seg1[0] <= 1.000053711899801e+06
 R1238: - C4 + 1e+06 alpha_ob66_seg1[0] <= 1.000053711899801e+06
 R1239: - C3 + 1e+06 alpha_ob66_seg1[1] <= 999947.6501129363
 R1240: - C5 + 1e+06 alpha_ob66_seg1[1] <= 999947.6501129363
 R1241: C2 + 1e+06 alpha_ob66_seg1[2] <= 999934.138099801
 R1242: C4 + 1e+06 alpha_ob66_seg1[2] <= 999934.138099801
 R1243: C3 + 1e+06 alpha_ob66_seg1[3] <= 1.0000497999129363e+06
 R1244: C5 + 1e+06 alpha_ob66_seg1[3] <= 1.0000497999129363e+06
 R1245: alpha_ob66_seg1[0] + alpha_ob66_seg1[1] + alpha_ob66_seg1[2]
   + alpha_ob66_seg1[3] >= 1
 R1246: - C2 + 1e+06 alpha_ob67_seg1[0] <= 1.0000764021508709e+06
 R1247: - C4 + 1e+06 alpha_ob67_seg1[0] <= 1.0000764021508709e+06
 R1248: - C3 + 1e+06 alpha_ob67_seg1[1] <= 999957.1027002756
 R1249: - C5 + 1e+06 alpha_ob67_seg1[1] <= 999957.1027002756
 R1250: C2 + 1e+06 alpha_ob67_seg1[2] <= 999921.0477508709
 R1251: C4 + 1e+06 alpha_ob67_seg1[2] <= 999921.0477508709
 R1252: C3 + 1e+06 alpha_ob67_seg1[3] <= 1.0000137473002756e+06
 R1253: C5 + 1e+06 alpha_ob67_seg1[3] <= 1.0000137473002756e+06
 R1254: alpha_ob67_seg1[0] + alpha_ob67_seg1[1] + alpha_ob67_seg1[2]
   + alpha_ob67_seg1[3] >= 1
 R1255: - C2 + 1e+06 alpha_ob68_seg1[0] <= 1.0000633118917353e+06
 R1256: - C4 + 1e+06 alpha_ob68_seg1[0] <= 1.0000633118917353e+06
 R1257: - C3 + 1e+06 alpha_ob68_seg1[1] <= 999947.6502002755
 R1258: - C5 + 1e+06 alpha_ob68_seg1[1] <= 999947.6502002755
 R1259: C2 + 1e+06 alpha_ob68_seg1[2] <= 999934.1380917353
 R1260: C4 + 1e+06 alpha_ob68_seg1[2] <= 999934.1380917353
 R1261: C3 + 1e+06 alpha_ob68_seg1[3] <= 1.0000454498002755e+06
 R1262: C5 + 1e+06 alpha_ob68_seg1[3] <= 1.0000454498002755e+06
 R1263: alpha_ob68_seg1[0] + alpha_ob68_seg1[1] + alpha_ob68_seg1[2]
   + alpha_ob68_seg1[3] >= 1
 R1264: - C2 + 1e+06 alpha_ob69_seg1[0] <= 1.0000448022e+06
 R1265: - C4 + 1e+06 alpha_ob69_seg1[0] <= 1.0000448022e+06
 R1266: - C3 + 1e+06 alpha_ob69_seg1[1] <= 999957.1027
 R1267: - C5 + 1e+06 alpha_ob69_seg1[1] <= 999957.1027
 R1268: C2 + 1e+06 alpha_ob69_seg1[2] <= 999921.0477999999
 R1269: C4 + 1e+06 alpha_ob69_seg1[2] <= 999921.0477999999
 R1270: C3 + 1e+06 alpha_ob69_seg1[3] <= 1.0000403473e+06
 R1271: C5 + 1e+06 alpha_ob69_seg1[3] <= 1.0000403473e+06
 R1272: alpha_ob69_seg1[0] + alpha_ob69_seg1[1] + alpha_ob69_seg1[2]
   + alpha_ob69_seg1[3] >= 1
 R1273: - C2 + 1e+06 alpha_ob70_seg1[0] <= 1.0000448021770425e+06
 R1274: - C4 + 1e+06 alpha_ob70_seg1[0] <= 1.0000448021770425e+06
 R1275: - C3 + 1e+06 alpha_ob70_seg1[1] <= 999957.1027002755
 R1276: - C5 + 1e+06 alpha_ob70_seg1[1] <= 999957.1027002755
 R1277: C2 + 1e+06 alpha_ob70_seg1[2] <= 999952.6477770425
 R1278: C4 + 1e+06 alpha_ob70_seg1[2] <= 999952.6477770425
 R1279: C3 + 1e+06 alpha_ob70_seg1[3] <= 1.0000279973002755e+06
 R1280: C5 + 1e+06 alpha_ob70_seg1[3] <= 1.0000279973002755e+06
 R1281: alpha_ob70_seg1[0] + alpha_ob70_seg1[1] + alpha_ob70_seg1[2]
   + alpha_ob70_seg1[3] >= 1
 R1282: C0 >= 0
 R1283: C1 >= 0
 R1284: C0 <= 85
 R1285: C1 <= 85
 R1286: C2 >= 0
 R1287: C3 >= 0
 R1288: C2 <= 85
 R1289: C3 <= 85
 R1290: C4 >= 0
 R1291: C5 >= 0
 R1292: C4 <= 85
 R1293: C5 <= 85
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
 alpha_ob34_seg0[3] alpha_ob35_seg0[0] alpha_ob35_seg0[1]
 alpha_ob35_seg0[2] alpha_ob35_seg0[3] alpha_ob36_seg0[0]
 alpha_ob36_seg0[1] alpha_ob36_seg0[2] alpha_ob36_seg0[3]
 alpha_ob37_seg0[0] alpha_ob37_seg0[1] alpha_ob37_seg0[2]
 alpha_ob37_seg0[3] alpha_ob38_seg0[0] alpha_ob38_seg0[1]
 alpha_ob38_seg0[2] alpha_ob38_seg0[3] alpha_ob39_seg0[0]
 alpha_ob39_seg0[1] alpha_ob39_seg0[2] alpha_ob39_seg0[3]
 alpha_ob40_seg0[0] alpha_ob40_seg0[1] alpha_ob40_seg0[2]
 alpha_ob40_seg0[3] alpha_ob41_seg0[0] alpha_ob41_seg0[1]
 alpha_ob41_seg0[2] alpha_ob41_seg0[3] alpha_ob42_seg0[0]
 alpha_ob42_seg0[1] alpha_ob42_seg0[2] alpha_ob42_seg0[3]
 alpha_ob43_seg0[0] alpha_ob43_seg0[1] alpha_ob43_seg0[2]
 alpha_ob43_seg0[3] alpha_ob44_seg0[0] alpha_ob44_seg0[1]
 alpha_ob44_seg0[2] alpha_ob44_seg0[3] alpha_ob45_seg0[0]
 alpha_ob45_seg0[1] alpha_ob45_seg0[2] alpha_ob45_seg0[3]
 alpha_ob46_seg0[0] alpha_ob46_seg0[1] alpha_ob46_seg0[2]
 alpha_ob46_seg0[3] alpha_ob47_seg0[0] alpha_ob47_seg0[1]
 alpha_ob47_seg0[2] alpha_ob47_seg0[3] alpha_ob48_seg0[0]
 alpha_ob48_seg0[1] alpha_ob48_seg0[2] alpha_ob48_seg0[3]
 alpha_ob49_seg0[0] alpha_ob49_seg0[1] alpha_ob49_seg0[2]
 alpha_ob49_seg0[3] alpha_ob50_seg0[0] alpha_ob50_seg0[1]
 alpha_ob50_seg0[2] alpha_ob50_seg0[3] alpha_ob51_seg0[0]
 alpha_ob51_seg0[1] alpha_ob51_seg0[2] alpha_ob51_seg0[3]
 alpha_ob52_seg0[0] alpha_ob52_seg0[1] alpha_ob52_seg0[2]
 alpha_ob52_seg0[3] alpha_ob53_seg0[0] alpha_ob53_seg0[1]
 alpha_ob53_seg0[2] alpha_ob53_seg0[3] alpha_ob54_seg0[0]
 alpha_ob54_seg0[1] alpha_ob54_seg0[2] alpha_ob54_seg0[3]
 alpha_ob55_seg0[0] alpha_ob55_seg0[1] alpha_ob55_seg0[2]
 alpha_ob55_seg0[3] alpha_ob56_seg0[0] alpha_ob56_seg0[1]
 alpha_ob56_seg0[2] alpha_ob56_seg0[3] alpha_ob57_seg0[0]
 alpha_ob57_seg0[1] alpha_ob57_seg0[2] alpha_ob57_seg0[3]
 alpha_ob58_seg0[0] alpha_ob58_seg0[1] alpha_ob58_seg0[2]
 alpha_ob58_seg0[3] alpha_ob59_seg0[0] alpha_ob59_seg0[1]
 alpha_ob59_seg0[2] alpha_ob59_seg0[3] alpha_ob60_seg0[0]
 alpha_ob60_seg0[1] alpha_ob60_seg0[2] alpha_ob60_seg0[3]
 alpha_ob61_seg0[0] alpha_ob61_seg0[1] alpha_ob61_seg0[2]
 alpha_ob61_seg0[3] alpha_ob62_seg0[0] alpha_ob62_seg0[1]
 alpha_ob62_seg0[2] alpha_ob62_seg0[3] alpha_ob63_seg0[0]
 alpha_ob63_seg0[1] alpha_ob63_seg0[2] alpha_ob63_seg0[3]
 alpha_ob64_seg0[0] alpha_ob64_seg0[1] alpha_ob64_seg0[2]
 alpha_ob64_seg0[3] alpha_ob65_seg0[0] alpha_ob65_seg0[1]
 alpha_ob65_seg0[2] alpha_ob65_seg0[3] alpha_ob66_seg0[0]
 alpha_ob66_seg0[1] alpha_ob66_seg0[2] alpha_ob66_seg0[3]
 alpha_ob67_seg0[0] alpha_ob67_seg0[1] alpha_ob67_seg0[2]
 alpha_ob67_seg0[3] alpha_ob68_seg0[0] alpha_ob68_seg0[1]
 alpha_ob68_seg0[2] alpha_ob68_seg0[3] alpha_ob69_seg0[0]
 alpha_ob69_seg0[1] alpha_ob69_seg0[2] alpha_ob69_seg0[3]
 alpha_ob70_seg0[0] alpha_ob70_seg0[1] alpha_ob70_seg0[2]
 alpha_ob70_seg0[3] alpha_ob0_seg1[0] alpha_ob0_seg1[1] alpha_ob0_seg1[2]
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
 alpha_ob34_seg1[3] alpha_ob35_seg1[0] alpha_ob35_seg1[1]
 alpha_ob35_seg1[2] alpha_ob35_seg1[3] alpha_ob36_seg1[0]
 alpha_ob36_seg1[1] alpha_ob36_seg1[2] alpha_ob36_seg1[3]
 alpha_ob37_seg1[0] alpha_ob37_seg1[1] alpha_ob37_seg1[2]
 alpha_ob37_seg1[3] alpha_ob38_seg1[0] alpha_ob38_seg1[1]
 alpha_ob38_seg1[2] alpha_ob38_seg1[3] alpha_ob39_seg1[0]
 alpha_ob39_seg1[1] alpha_ob39_seg1[2] alpha_ob39_seg1[3]
 alpha_ob40_seg1[0] alpha_ob40_seg1[1] alpha_ob40_seg1[2]
 alpha_ob40_seg1[3] alpha_ob41_seg1[0] alpha_ob41_seg1[1]
 alpha_ob41_seg1[2] alpha_ob41_seg1[3] alpha_ob42_seg1[0]
 alpha_ob42_seg1[1] alpha_ob42_seg1[2] alpha_ob42_seg1[3]
 alpha_ob43_seg1[0] alpha_ob43_seg1[1] alpha_ob43_seg1[2]
 alpha_ob43_seg1[3] alpha_ob44_seg1[0] alpha_ob44_seg1[1]
 alpha_ob44_seg1[2] alpha_ob44_seg1[3] alpha_ob45_seg1[0]
 alpha_ob45_seg1[1] alpha_ob45_seg1[2] alpha_ob45_seg1[3]
 alpha_ob46_seg1[0] alpha_ob46_seg1[1] alpha_ob46_seg1[2]
 alpha_ob46_seg1[3] alpha_ob47_seg1[0] alpha_ob47_seg1[1]
 alpha_ob47_seg1[2] alpha_ob47_seg1[3] alpha_ob48_seg1[0]
 alpha_ob48_seg1[1] alpha_ob48_seg1[2] alpha_ob48_seg1[3]
 alpha_ob49_seg1[0] alpha_ob49_seg1[1] alpha_ob49_seg1[2]
 alpha_ob49_seg1[3] alpha_ob50_seg1[0] alpha_ob50_seg1[1]
 alpha_ob50_seg1[2] alpha_ob50_seg1[3] alpha_ob51_seg1[0]
 alpha_ob51_seg1[1] alpha_ob51_seg1[2] alpha_ob51_seg1[3]
 alpha_ob52_seg1[0] alpha_ob52_seg1[1] alpha_ob52_seg1[2]
 alpha_ob52_seg1[3] alpha_ob53_seg1[0] alpha_ob53_seg1[1]
 alpha_ob53_seg1[2] alpha_ob53_seg1[3] alpha_ob54_seg1[0]
 alpha_ob54_seg1[1] alpha_ob54_seg1[2] alpha_ob54_seg1[3]
 alpha_ob55_seg1[0] alpha_ob55_seg1[1] alpha_ob55_seg1[2]
 alpha_ob55_seg1[3] alpha_ob56_seg1[0] alpha_ob56_seg1[1]
 alpha_ob56_seg1[2] alpha_ob56_seg1[3] alpha_ob57_seg1[0]
 alpha_ob57_seg1[1] alpha_ob57_seg1[2] alpha_ob57_seg1[3]
 alpha_ob58_seg1[0] alpha_ob58_seg1[1] alpha_ob58_seg1[2]
 alpha_ob58_seg1[3] alpha_ob59_seg1[0] alpha_ob59_seg1[1]
 alpha_ob59_seg1[2] alpha_ob59_seg1[3] alpha_ob60_seg1[0]
 alpha_ob60_seg1[1] alpha_ob60_seg1[2] alpha_ob60_seg1[3]
 alpha_ob61_seg1[0] alpha_ob61_seg1[1] alpha_ob61_seg1[2]
 alpha_ob61_seg1[3] alpha_ob62_seg1[0] alpha_ob62_seg1[1]
 alpha_ob62_seg1[2] alpha_ob62_seg1[3] alpha_ob63_seg1[0]
 alpha_ob63_seg1[1] alpha_ob63_seg1[2] alpha_ob63_seg1[3]
 alpha_ob64_seg1[0] alpha_ob64_seg1[1] alpha_ob64_seg1[2]
 alpha_ob64_seg1[3] alpha_ob65_seg1[0] alpha_ob65_seg1[1]
 alpha_ob65_seg1[2] alpha_ob65_seg1[3] alpha_ob66_seg1[0]
 alpha_ob66_seg1[1] alpha_ob66_seg1[2] alpha_ob66_seg1[3]
 alpha_ob67_seg1[0] alpha_ob67_seg1[1] alpha_ob67_seg1[2]
 alpha_ob67_seg1[3] alpha_ob68_seg1[0] alpha_ob68_seg1[1]
 alpha_ob68_seg1[2] alpha_ob68_seg1[3] alpha_ob69_seg1[0]
 alpha_ob69_seg1[1] alpha_ob69_seg1[2] alpha_ob69_seg1[3]
 alpha_ob70_seg1[0] alpha_ob70_seg1[1] alpha_ob70_seg1[2]
 alpha_ob70_seg1[3]
End
