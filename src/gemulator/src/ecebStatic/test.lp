\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 60 C0 - 20 C1 - 60 C2 - 20 C3 - 60 C4 - 20 C5 - 60 C6 - 20 C7
   + 4000 Constant + [ 2 C0 ^2 + 2 C1 ^2 + 2 C2 ^2 + 2 C3 ^2 + 2 C4 ^2
   + 2 C5 ^2 + 2 C6 ^2 + 2 C7 ^2 ] / 2 
Subject To
 R0: C0 = 0
 R1: C1 = 0
 R2: C6 = 30
 R3: C7 = 10
 R4: - C0 + 1e+06 alpha_ob0_seg0[0] <= 1.0000284170499525e+06
 R5: - C2 + 1e+06 alpha_ob0_seg0[0] <= 1.0000284170499525e+06
 R6: - C1 + 1e+06 alpha_ob0_seg0[1] <= 999957.0843002755
 R7: - C3 + 1e+06 alpha_ob0_seg0[1] <= 999957.0843002755
 R8: C0 + 1e+06 alpha_ob0_seg0[2] <= 999969.0328499526
 R9: C2 + 1e+06 alpha_ob0_seg0[2] <= 999969.0328499526
 R10: C1 + 1e+06 alpha_ob0_seg0[3] <= 1.0000132657002755e+06
 R11: C3 + 1e+06 alpha_ob0_seg0[3] <= 1.0000132657002755e+06
 R12: alpha_ob0_seg0[0] + alpha_ob0_seg0[1] + alpha_ob0_seg0[2]
   + alpha_ob0_seg0[3] >= 1
 R13: - C0 + 1e+06 alpha_ob1_seg0[0] <= 1.000028417099801e+06
 R14: - C2 + 1e+06 alpha_ob1_seg0[0] <= 1.000028417099801e+06
 R15: - C1 + 1e+06 alpha_ob1_seg0[1] <= 999984.1843155898
 R16: - C3 + 1e+06 alpha_ob1_seg0[1] <= 999984.1843155898
 R17: C0 + 1e+06 alpha_ob1_seg0[2] <= 999957.4328998009
 R18: C2 + 1e+06 alpha_ob1_seg0[2] <= 999957.4328998009
 R19: C1 + 1e+06 alpha_ob1_seg0[3] <= 1.0000132657155899e+06
 R20: C3 + 1e+06 alpha_ob1_seg0[3] <= 1.0000132657155899e+06
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
 R31: - C0 + 1e+06 alpha_ob3_seg0[0] <= 1.0000109162e+06
 R32: - C2 + 1e+06 alpha_ob3_seg0[0] <= 1.0000109162e+06
 R33: - C1 + 1e+06 alpha_ob3_seg0[1] <= 999970.2681
 R34: - C3 + 1e+06 alpha_ob3_seg0[1] <= 999970.2681
 R35: C0 + 1e+06 alpha_ob3_seg0[2] <= 999975.1838
 R36: C2 + 1e+06 alpha_ob3_seg0[2] <= 999975.1838
 R37: C1 + 1e+06 alpha_ob3_seg0[3] <= 1.0000271819e+06
 R38: C3 + 1e+06 alpha_ob3_seg0[3] <= 1.0000271819e+06
 R39: alpha_ob3_seg0[0] + alpha_ob3_seg0[1] + alpha_ob3_seg0[2]
   + alpha_ob3_seg0[3] >= 1
 R40: - C0 + 1e+06 alpha_ob4_seg0[0] <= 1.0000109161752058e+06
 R41: - C2 + 1e+06 alpha_ob4_seg0[0] <= 1.0000109161752058e+06
 R42: - C1 + 1e+06 alpha_ob4_seg0[1] <= 999970.2681002755
 R43: - C3 + 1e+06 alpha_ob4_seg0[1] <= 999970.2681002755
 R44: C0 + 1e+06 alpha_ob4_seg0[2] <= 999986.5337752058
 R45: C2 + 1e+06 alpha_ob4_seg0[2] <= 999986.5337752058
 R46: C1 + 1e+06 alpha_ob4_seg0[3] <= 1.0000138319002755e+06
 R47: C3 + 1e+06 alpha_ob4_seg0[3] <= 1.0000138319002755e+06
 R48: alpha_ob4_seg0[0] + alpha_ob4_seg0[1] + alpha_ob4_seg0[2]
   + alpha_ob4_seg0[3] >= 1
 R49: - C0 + 1e+06 alpha_ob5_seg0[0] <= 1.000010916199801e+06
 R50: - C2 + 1e+06 alpha_ob5_seg0[0] <= 1.000010916199801e+06
 R51: - C1 + 1e+06 alpha_ob5_seg0[1] <= 999983.6181179117
 R52: - C3 + 1e+06 alpha_ob5_seg0[1] <= 999983.6181179117
 R53: C0 + 1e+06 alpha_ob5_seg0[2] <= 999973.183799801
 R54: C2 + 1e+06 alpha_ob5_seg0[2] <= 999973.183799801
 R55: C1 + 1e+06 alpha_ob5_seg0[3] <= 1.0000138319179118e+06
 R56: C3 + 1e+06 alpha_ob5_seg0[3] <= 1.0000138319179118e+06
 R57: alpha_ob5_seg0[0] + alpha_ob5_seg0[1] + alpha_ob5_seg0[2]
   + alpha_ob5_seg0[3] >= 1
 R58: - C0 + 1e+06 alpha_ob6_seg0[0] <= 1.0000139162e+06
 R59: - C2 + 1e+06 alpha_ob6_seg0[0] <= 1.0000139162e+06
 R60: - C1 + 1e+06 alpha_ob6_seg0[1] <= 999957.1679999999
 R61: - C3 + 1e+06 alpha_ob6_seg0[1] <= 999957.1679999999
 R62: C0 + 1e+06 alpha_ob6_seg0[2] <= 999975.1838
 R63: C2 + 1e+06 alpha_ob6_seg0[2] <= 999975.1838
 R64: C1 + 1e+06 alpha_ob6_seg0[3] <= 1.000040282e+06
 R65: C3 + 1e+06 alpha_ob6_seg0[3] <= 1.000040282e+06
 R66: alpha_ob6_seg0[0] + alpha_ob6_seg0[1] + alpha_ob6_seg0[2]
   + alpha_ob6_seg0[3] >= 1
 R67: - C0 + 1e+06 alpha_ob7_seg0[0] <= 999992.49102
 R68: - C2 + 1e+06 alpha_ob7_seg0[0] <= 999992.49102
 R69: - C1 + 1e+06 alpha_ob7_seg0[1] <= 999957.1016000001
 R70: - C3 + 1e+06 alpha_ob7_seg0[1] <= 999957.1016000001
 R71: C0 + 1e+06 alpha_ob7_seg0[2] <= 999986.35898
 R72: C2 + 1e+06 alpha_ob7_seg0[2] <= 999986.35898
 R73: C1 + 1e+06 alpha_ob7_seg0[3] <= 1.0000403484e+06
 R74: C3 + 1e+06 alpha_ob7_seg0[3] <= 1.0000403484e+06
 R75: alpha_ob7_seg0[0] + alpha_ob7_seg0[1] + alpha_ob7_seg0[2]
   + alpha_ob7_seg0[3] >= 1
 R76: - C0 + 1e+06 alpha_ob8_seg0[0] <= 999981.3909999999
 R77: - C2 + 1e+06 alpha_ob8_seg0[0] <= 999981.3909999999
 R78: - C1 + 1e+06 alpha_ob8_seg0[1] <= 999957.1016000001
 R79: - C3 + 1e+06 alpha_ob8_seg0[1] <= 999957.1016000001
 R80: C0 + 1e+06 alpha_ob8_seg0[2] <= 1.000004959e+06
 R81: C2 + 1e+06 alpha_ob8_seg0[2] <= 1.000004959e+06
 R82: C1 + 1e+06 alpha_ob8_seg0[3] <= 1.0000403484e+06
 R83: C3 + 1e+06 alpha_ob8_seg0[3] <= 1.0000403484e+06
 R84: alpha_ob8_seg0[0] + alpha_ob8_seg0[1] + alpha_ob8_seg0[2]
   + alpha_ob8_seg0[3] >= 1
 R85: - C0 + 1e+06 alpha_ob9_seg0[0] <= 999981.3909825523
 R86: - C2 + 1e+06 alpha_ob9_seg0[0] <= 999981.3909825523
 R87: - C1 + 1e+06 alpha_ob9_seg0[1] <= 999957.1016002755
 R88: - C3 + 1e+06 alpha_ob9_seg0[1] <= 999957.1016002755
 R89: C0 + 1e+06 alpha_ob9_seg0[2] <= 1.0000160589825523e+06
 R90: C2 + 1e+06 alpha_ob9_seg0[2] <= 1.0000160589825523e+06
 R91: C1 + 1e+06 alpha_ob9_seg0[3] <= 1.0000309984002755e+06
 R92: C3 + 1e+06 alpha_ob9_seg0[3] <= 1.0000309984002755e+06
 R93: alpha_ob9_seg0[0] + alpha_ob9_seg0[1] + alpha_ob9_seg0[2]
   + alpha_ob9_seg0[3] >= 1
 R94: - C0 + 1e+06 alpha_ob10_seg0[0] <= 999981.3867954084
 R95: - C2 + 1e+06 alpha_ob10_seg0[0] <= 999981.3867954084
 R96: - C1 + 1e+06 alpha_ob10_seg0[1] <= 999968.8595002755
 R97: - C3 + 1e+06 alpha_ob10_seg0[1] <= 999968.8595002755
 R98: C0 + 1e+06 alpha_ob10_seg0[2] <= 1.0000160631954084e+06
 R99: C2 + 1e+06 alpha_ob10_seg0[2] <= 1.0000160631954084e+06
 R100: C1 + 1e+06 alpha_ob10_seg0[3] <= 1.0000262405002755e+06
 R101: C3 + 1e+06 alpha_ob10_seg0[3] <= 1.0000262405002755e+06
 R102: alpha_ob10_seg0[0] + alpha_ob10_seg0[1] + alpha_ob10_seg0[2]
   + alpha_ob10_seg0[3] >= 1
 R103: - C0 + 1e+06 alpha_ob11_seg0[0] <= 999981.386799801
 R104: - C2 + 1e+06 alpha_ob11_seg0[0] <= 999981.386799801
 R105: - C1 + 1e+06 alpha_ob11_seg0[1] <= 999971.2095013268
 R106: - C3 + 1e+06 alpha_ob11_seg0[1] <= 999971.2095013268
 R107: C0 + 1e+06 alpha_ob11_seg0[2] <= 1.0000152131998009e+06
 R108: C2 + 1e+06 alpha_ob11_seg0[2] <= 1.0000152131998009e+06
 R109: C1 + 1e+06 alpha_ob11_seg0[3] <= 1.0000262405013267e+06
 R110: C3 + 1e+06 alpha_ob11_seg0[3] <= 1.0000262405013267e+06
 R111: alpha_ob11_seg0[0] + alpha_ob11_seg0[1] + alpha_ob11_seg0[2]
   + alpha_ob11_seg0[3] >= 1
 R112: - C0 + 1e+06 alpha_ob12_seg0[0] <= 999982.2368022958
 R113: - C2 + 1e+06 alpha_ob12_seg0[0] <= 999982.2368022958
 R114: - C1 + 1e+06 alpha_ob12_seg0[1] <= 999970.1094997245
 R115: - C3 + 1e+06 alpha_ob12_seg0[1] <= 999970.1094997245
 R116: C0 + 1e+06 alpha_ob12_seg0[2] <= 1.0000152132022957e+06
 R117: C2 + 1e+06 alpha_ob12_seg0[2] <= 1.0000152132022957e+06
 R118: C1 + 1e+06 alpha_ob12_seg0[3] <= 1.0000262404997245e+06
 R119: C3 + 1e+06 alpha_ob12_seg0[3] <= 1.0000262404997245e+06
 R120: alpha_ob12_seg0[0] + alpha_ob12_seg0[1] + alpha_ob12_seg0[2]
   + alpha_ob12_seg0[3] >= 1
 R121: - C0 + 1e+06 alpha_ob13_seg0[0] <= 999982.236799801
 R122: - C2 + 1e+06 alpha_ob13_seg0[0] <= 999982.236799801
 R123: - C1 + 1e+06 alpha_ob13_seg0[1] <= 999970.1095291895
 R124: - C3 + 1e+06 alpha_ob13_seg0[1] <= 999970.1095291895
 R125: C0 + 1e+06 alpha_ob13_seg0[2] <= 999993.3631998011
 R126: C2 + 1e+06 alpha_ob13_seg0[2] <= 999993.3631998011
 R127: C1 + 1e+06 alpha_ob13_seg0[3] <= 1.0000273405291894e+06
 R128: C3 + 1e+06 alpha_ob13_seg0[3] <= 1.0000273405291894e+06
 R129: alpha_ob13_seg0[0] + alpha_ob13_seg0[1] + alpha_ob13_seg0[2]
   + alpha_ob13_seg0[3] >= 1
 R130: - C0 + 1e+06 alpha_ob14_seg0[0] <= 1.0000040868238758e+06
 R131: - C2 + 1e+06 alpha_ob14_seg0[0] <= 1.0000040868238758e+06
 R132: - C1 + 1e+06 alpha_ob14_seg0[1] <= 999957.2594997246
 R133: - C3 + 1e+06 alpha_ob14_seg0[1] <= 999957.2594997246
 R134: C0 + 1e+06 alpha_ob14_seg0[2] <= 999993.3632238759
 R135: C2 + 1e+06 alpha_ob14_seg0[2] <= 999993.3632238759
 R136: C1 + 1e+06 alpha_ob14_seg0[3] <= 1.0000273404997246e+06
 R137: C3 + 1e+06 alpha_ob14_seg0[3] <= 1.0000273404997246e+06
 R138: alpha_ob14_seg0[0] + alpha_ob14_seg0[1] + alpha_ob14_seg0[2]
   + alpha_ob14_seg0[3] >= 1
 R139: - C0 + 1e+06 alpha_ob15_seg0[0] <= 999981.6050802565
 R140: - C2 + 1e+06 alpha_ob15_seg0[0] <= 999981.6050802565
 R141: - C1 + 1e+06 alpha_ob15_seg0[1] <= 999972.8942002755
 R142: - C3 + 1e+06 alpha_ob15_seg0[1] <= 999972.8942002755
 R143: C0 + 1e+06 alpha_ob15_seg0[2] <= 1.0000158448802566e+06
 R144: C2 + 1e+06 alpha_ob15_seg0[2] <= 1.0000158448802566e+06
 R145: C1 + 1e+06 alpha_ob15_seg0[3] <= 1.0000139558002756e+06
 R146: C3 + 1e+06 alpha_ob15_seg0[3] <= 1.0000139558002756e+06
 R147: alpha_ob15_seg0[0] + alpha_ob15_seg0[1] + alpha_ob15_seg0[2]
   + alpha_ob15_seg0[3] >= 1
 R148: - C0 + 1e+06 alpha_ob16_seg0[0] <= 999981.605149801
 R149: - C2 + 1e+06 alpha_ob16_seg0[0] <= 999981.605149801
 R150: - C1 + 1e+06 alpha_ob16_seg0[1] <= 999983.4942391404
 R151: - C3 + 1e+06 alpha_ob16_seg0[1] <= 999983.4942391404
 R152: C0 + 1e+06 alpha_ob16_seg0[2] <= 999986.4948498011
 R153: C2 + 1e+06 alpha_ob16_seg0[2] <= 999986.4948498011
 R154: C1 + 1e+06 alpha_ob16_seg0[3] <= 1.0000139558391405e+06
 R155: C3 + 1e+06 alpha_ob16_seg0[3] <= 1.0000139558391405e+06
 R156: alpha_ob16_seg0[0] + alpha_ob16_seg0[1] + alpha_ob16_seg0[2]
   + alpha_ob16_seg0[3] >= 1
 R157: - C0 + 1e+06 alpha_ob17_seg0[0] <= 999997.6803502565
 R158: - C2 + 1e+06 alpha_ob17_seg0[0] <= 999997.6803502565
 R159: - C1 + 1e+06 alpha_ob17_seg0[1] <= 999972.7268002755
 R160: - C3 + 1e+06 alpha_ob17_seg0[1] <= 999972.7268002755
 R161: C0 + 1e+06 alpha_ob17_seg0[2] <= 999999.7696102565
 R162: C2 + 1e+06 alpha_ob17_seg0[2] <= 999999.7696102565
 R163: C1 + 1e+06 alpha_ob17_seg0[3] <= 1.0000141232002755e+06
 R164: C3 + 1e+06 alpha_ob17_seg0[3] <= 1.0000141232002755e+06
 R165: alpha_ob17_seg0[0] + alpha_ob17_seg0[1] + alpha_ob17_seg0[2]
   + alpha_ob17_seg0[3] >= 1
 R166: - C0 + 1e+06 alpha_ob18_seg0[0] <= 1.0000038341502565e+06
 R167: - C2 + 1e+06 alpha_ob18_seg0[0] <= 1.0000038341502565e+06
 R168: - C1 + 1e+06 alpha_ob18_seg0[1] <= 999972.6012002755
 R169: - C3 + 1e+06 alpha_ob18_seg0[1] <= 999972.6012002755
 R170: C0 + 1e+06 alpha_ob18_seg0[2] <= 999993.6158102566
 R171: C2 + 1e+06 alpha_ob18_seg0[2] <= 999993.6158102566
 R172: C1 + 1e+06 alpha_ob18_seg0[3] <= 1.0000142488002755e+06
 R173: C3 + 1e+06 alpha_ob18_seg0[3] <= 1.0000142488002755e+06
 R174: alpha_ob18_seg0[0] + alpha_ob18_seg0[1] + alpha_ob18_seg0[2]
   + alpha_ob18_seg0[3] >= 1
 R175: - C0 + 1e+06 alpha_ob19_seg0[0] <= 1.0000162874063333e+06
 R176: - C2 + 1e+06 alpha_ob19_seg0[0] <= 1.0000162874063333e+06
 R177: - C1 + 1e+06 alpha_ob19_seg0[1] <= 999982.9439602756
 R178: - C3 + 1e+06 alpha_ob19_seg0[1] <= 999982.9439602756
 R179: C0 + 1e+06 alpha_ob19_seg0[2] <= 999981.1624063333
 R180: C2 + 1e+06 alpha_ob19_seg0[2] <= 999981.1624063333
 R181: C1 + 1e+06 alpha_ob19_seg0[3] <= 999963.6560402757
 R182: C3 + 1e+06 alpha_ob19_seg0[3] <= 999963.6560402757
 R183: alpha_ob19_seg0[0] + alpha_ob19_seg0[1] + alpha_ob19_seg0[2]
   + alpha_ob19_seg0[3] >= 1
 R184: - C0 + 1e+06 alpha_ob20_seg0[0] <= 1.0000162874998011e+06
 R185: - C2 + 1e+06 alpha_ob20_seg0[0] <= 1.0000162874998011e+06
 R186: - C1 + 1e+06 alpha_ob20_seg0[1] <= 1.000033794079276e+06
 R187: - C3 + 1e+06 alpha_ob20_seg0[1] <= 1.000033794079276e+06
 R188: C0 + 1e+06 alpha_ob20_seg0[2] <= 999921.562499801
 R189: C2 + 1e+06 alpha_ob20_seg0[2] <= 999921.562499801
 R190: C1 + 1e+06 alpha_ob20_seg0[3] <= 999963.656079276
 R191: C3 + 1e+06 alpha_ob20_seg0[3] <= 999963.656079276
 R192: alpha_ob20_seg0[0] + alpha_ob20_seg0[1] + alpha_ob20_seg0[2]
   + alpha_ob20_seg0[3] >= 1
 R193: - C0 + 1e+06 alpha_ob21_seg0[0] <= 1.0000066737e+06
 R194: - C2 + 1e+06 alpha_ob21_seg0[0] <= 1.0000066737e+06
 R195: - C1 + 1e+06 alpha_ob21_seg0[1] <= 999952.1958
 R196: - C3 + 1e+06 alpha_ob21_seg0[1] <= 999952.1958
 R197: C0 + 1e+06 alpha_ob21_seg0[2] <= 999948.4263000001
 R198: C2 + 1e+06 alpha_ob21_seg0[2] <= 999948.4263000001
 R199: C1 + 1e+06 alpha_ob21_seg0[3] <= 1.0000452542e+06
 R200: C3 + 1e+06 alpha_ob21_seg0[3] <= 1.0000452542e+06
 R201: alpha_ob21_seg0[0] + alpha_ob21_seg0[1] + alpha_ob21_seg0[2]
   + alpha_ob21_seg0[3] >= 1
 R202: - C0 + 1e+06 alpha_ob22_seg0[0] <= 1.0000162874977043e+06
 R203: - C2 + 1e+06 alpha_ob22_seg0[0] <= 1.0000162874977043e+06
 R204: - C1 + 1e+06 alpha_ob22_seg0[1] <= 1.0000337940002755e+06
 R205: - C3 + 1e+06 alpha_ob22_seg0[1] <= 1.0000337940002755e+06
 R206: C0 + 1e+06 alpha_ob22_seg0[2] <= 999981.1624977043
 R207: C2 + 1e+06 alpha_ob22_seg0[2] <= 999981.1624977043
 R208: C1 + 1e+06 alpha_ob22_seg0[3] <= 999962.5560002755
 R209: C3 + 1e+06 alpha_ob22_seg0[3] <= 999962.5560002755
 R210: alpha_ob22_seg0[0] + alpha_ob22_seg0[1] + alpha_ob22_seg0[2]
   + alpha_ob22_seg0[3] >= 1
 R211: - C0 + 1e+06 alpha_ob23_seg0[0] <= 999980.937509
 R212: - C2 + 1e+06 alpha_ob23_seg0[0] <= 999980.937509
 R213: - C1 + 1e+06 alpha_ob23_seg0[1] <= 1.000034894e+06
 R214: - C3 + 1e+06 alpha_ob23_seg0[1] <= 1.000034894e+06
 R215: C0 + 1e+06 alpha_ob23_seg0[2] <= 999981.162491
 R216: C2 + 1e+06 alpha_ob23_seg0[2] <= 999981.162491
 R217: C1 + 1e+06 alpha_ob23_seg0[3] <= 999962.556
 R218: C3 + 1e+06 alpha_ob23_seg0[3] <= 999962.556
 R219: alpha_ob23_seg0[0] + alpha_ob23_seg0[1] + alpha_ob23_seg0[2]
   + alpha_ob23_seg0[3] >= 1
 R220: - C0 + 1e+06 alpha_ob24_seg0[0] <= 1.0000070091e+06
 R221: - C2 + 1e+06 alpha_ob24_seg0[0] <= 1.0000070091e+06
 R222: - C1 + 1e+06 alpha_ob24_seg0[1] <= 1.0000598843e+06
 R223: - C3 + 1e+06 alpha_ob24_seg0[1] <= 1.0000598843e+06
 R224: C0 + 1e+06 alpha_ob24_seg0[2] <= 999965.0909
 R225: C2 + 1e+06 alpha_ob24_seg0[2] <= 999965.0909
 R226: C1 + 1e+06 alpha_ob24_seg0[3] <= 999937.5657
 R227: C3 + 1e+06 alpha_ob24_seg0[3] <= 999937.5657
 R228: alpha_ob24_seg0[0] + alpha_ob24_seg0[1] + alpha_ob24_seg0[2]
   + alpha_ob24_seg0[3] >= 1
 R229: - C0 + 1e+06 alpha_ob25_seg0[0] <= 1.000006992114031e+06
 R230: - C2 + 1e+06 alpha_ob25_seg0[0] <= 1.000006992114031e+06
 R231: - C1 + 1e+06 alpha_ob25_seg0[1] <= 1.0000588068002755e+06
 R232: - C3 + 1e+06 alpha_ob25_seg0[1] <= 1.0000588068002755e+06
 R233: C0 + 1e+06 alpha_ob25_seg0[2] <= 999990.457874031
 R234: C2 + 1e+06 alpha_ob25_seg0[2] <= 999990.457874031
 R235: C1 + 1e+06 alpha_ob25_seg0[3] <= 999935.5432002755
 R236: C3 + 1e+06 alpha_ob25_seg0[3] <= 999935.5432002755
 R237: alpha_ob25_seg0[0] + alpha_ob25_seg0[1] + alpha_ob25_seg0[2]
   + alpha_ob25_seg0[3] >= 1
 R238: - C0 + 1e+06 alpha_ob26_seg0[0] <= 1.0000033775063743e+06
 R239: - C2 + 1e+06 alpha_ob26_seg0[0] <= 1.0000033775063743e+06
 R240: - C1 + 1e+06 alpha_ob26_seg0[1] <= 1.0000348666997246e+06
 R241: - C3 + 1e+06 alpha_ob26_seg0[1] <= 1.0000348666997246e+06
 R242: C0 + 1e+06 alpha_ob26_seg0[2] <= 999994.0725863742
 R243: C2 + 1e+06 alpha_ob26_seg0[2] <= 999994.0725863742
 R244: C1 + 1e+06 alpha_ob26_seg0[3] <= 999937.4832997246
 R245: C3 + 1e+06 alpha_ob26_seg0[3] <= 999937.4832997246
 R246: alpha_ob26_seg0[0] + alpha_ob26_seg0[1] + alpha_ob26_seg0[2]
   + alpha_ob26_seg0[3] >= 1
 R247: - C0 + 1e+06 alpha_ob27_seg0[0] <= 999990.2670277582
 R248: - C2 + 1e+06 alpha_ob27_seg0[0] <= 999990.2670277582
 R249: - C1 + 1e+06 alpha_ob27_seg0[1] <= 1.0000347702002756e+06
 R250: - C3 + 1e+06 alpha_ob27_seg0[1] <= 1.0000347702002756e+06
 R251: C0 + 1e+06 alpha_ob27_seg0[2] <= 1.0000071828877581e+06
 R252: C2 + 1e+06 alpha_ob27_seg0[2] <= 1.0000071828877581e+06
 R253: C1 + 1e+06 alpha_ob27_seg0[3] <= 999939.8298002756
 R254: C3 + 1e+06 alpha_ob27_seg0[3] <= 999939.8298002756
 R255: alpha_ob27_seg0[0] + alpha_ob27_seg0[1] + alpha_ob27_seg0[2]
   + alpha_ob27_seg0[3] >= 1
 R256: - C0 + 1e+06 alpha_ob28_seg0[0] <= 999990.267069801
 R257: - C2 + 1e+06 alpha_ob28_seg0[0] <= 999990.267069801
 R258: - C1 + 1e+06 alpha_ob28_seg0[1] <= 1.0000576202069657e+06
 R259: - C3 + 1e+06 alpha_ob28_seg0[1] <= 1.0000576202069657e+06
 R260: C0 + 1e+06 alpha_ob28_seg0[2] <= 1.000002082929801e+06
 R261: C2 + 1e+06 alpha_ob28_seg0[2] <= 1.000002082929801e+06
 R262: C1 + 1e+06 alpha_ob28_seg0[3] <= 999939.8298069657
 R263: C3 + 1e+06 alpha_ob28_seg0[3] <= 999939.8298069657
 R264: alpha_ob28_seg0[0] + alpha_ob28_seg0[1] + alpha_ob28_seg0[2]
   + alpha_ob28_seg0[3] >= 1
 R265: - C0 + 1e+06 alpha_ob29_seg0[0] <= 999991.073732
 R266: - C2 + 1e+06 alpha_ob29_seg0[0] <= 999991.073732
 R267: - C1 + 1e+06 alpha_ob29_seg0[1] <= 999952.2125
 R268: - C3 + 1e+06 alpha_ob29_seg0[1] <= 999952.2125
 R269: C0 + 1e+06 alpha_ob29_seg0[2] <= 999990.776268
 R270: C2 + 1e+06 alpha_ob29_seg0[2] <= 999990.776268
 R271: C1 + 1e+06 alpha_ob29_seg0[3] <= 1.0000452375e+06
 R272: C3 + 1e+06 alpha_ob29_seg0[3] <= 1.0000452375e+06
 R273: alpha_ob29_seg0[0] + alpha_ob29_seg0[1] + alpha_ob29_seg0[2]
   + alpha_ob29_seg0[3] >= 1
 R274: - C0 + 1e+06 alpha_ob30_seg0[0] <= 999985.657099801
 R275: - C2 + 1e+06 alpha_ob30_seg0[0] <= 999985.657099801
 R276: - C1 + 1e+06 alpha_ob30_seg0[1] <= 1.0000576202063023e+06
 R277: - C3 + 1e+06 alpha_ob30_seg0[1] <= 1.0000576202063023e+06
 R278: C0 + 1e+06 alpha_ob30_seg0[2] <= 1.000007192899801e+06
 R279: C2 + 1e+06 alpha_ob30_seg0[2] <= 1.000007192899801e+06
 R280: C1 + 1e+06 alpha_ob30_seg0[3] <= 999939.8298063023
 R281: C3 + 1e+06 alpha_ob30_seg0[3] <= 999939.8298063023
 R282: alpha_ob30_seg0[0] + alpha_ob30_seg0[1] + alpha_ob30_seg0[2]
   + alpha_ob30_seg0[3] >= 1
 R283: - C0 + 1e+06 alpha_ob31_seg0[0] <= 999994.771829801
 R284: - C2 + 1e+06 alpha_ob31_seg0[0] <= 999994.771829801
 R285: - C1 + 1e+06 alpha_ob31_seg0[1] <= 1.0000615983063023e+06
 R286: - C3 + 1e+06 alpha_ob31_seg0[1] <= 1.0000615983063023e+06
 R287: C0 + 1e+06 alpha_ob31_seg0[2] <= 999998.078169801
 R288: C2 + 1e+06 alpha_ob31_seg0[2] <= 999998.078169801
 R289: C1 + 1e+06 alpha_ob31_seg0[3] <= 999935.8517063023
 R290: C3 + 1e+06 alpha_ob31_seg0[3] <= 999935.8517063023
 R291: alpha_ob31_seg0[0] + alpha_ob31_seg0[1] + alpha_ob31_seg0[2]
   + alpha_ob31_seg0[3] >= 1
 R292: - C0 + 1e+06 alpha_ob32_seg0[0] <= 999999.3718445294
 R293: - C2 + 1e+06 alpha_ob32_seg0[0] <= 999999.3718445294
 R294: - C1 + 1e+06 alpha_ob32_seg0[1] <= 1.0000527482997245e+06
 R295: - C3 + 1e+06 alpha_ob32_seg0[1] <= 1.0000527482997245e+06
 R296: C0 + 1e+06 alpha_ob32_seg0[2] <= 999998.0781885295
 R297: C2 + 1e+06 alpha_ob32_seg0[2] <= 999998.0781885295
 R298: C1 + 1e+06 alpha_ob32_seg0[3] <= 999935.8516997246
 R299: C3 + 1e+06 alpha_ob32_seg0[3] <= 999935.8516997246
 R300: alpha_ob32_seg0[0] + alpha_ob32_seg0[1] + alpha_ob32_seg0[2]
   + alpha_ob32_seg0[3] >= 1
 R301: - C0 + 1e+06 alpha_ob33_seg0[0] <= 999999.371829801
 R302: - C2 + 1e+06 alpha_ob33_seg0[0] <= 999999.371829801
 R303: - C1 + 1e+06 alpha_ob33_seg0[1] <= 1.0000527483059706e+06
 R304: - C3 + 1e+06 alpha_ob33_seg0[1] <= 1.0000527483059706e+06
 R305: C0 + 1e+06 alpha_ob33_seg0[2] <= 999993.7281698009
 R306: C2 + 1e+06 alpha_ob33_seg0[2] <= 999993.7281698009
 R307: C1 + 1e+06 alpha_ob33_seg0[3] <= 999944.7017059706
 R308: C3 + 1e+06 alpha_ob33_seg0[3] <= 999944.7017059706
 R309: alpha_ob33_seg0[0] + alpha_ob33_seg0[1] + alpha_ob33_seg0[2]
   + alpha_ob33_seg0[3] >= 1
 R310: - C0 + 1e+06 alpha_ob34_seg0[0] <= 1.0000016470956831e+06
 R311: - C2 + 1e+06 alpha_ob34_seg0[0] <= 1.0000016470956831e+06
 R312: - C1 + 1e+06 alpha_ob34_seg0[1] <= 1.0000598666742218e+06
 R313: - C3 + 1e+06 alpha_ob34_seg0[1] <= 1.0000598666742218e+06
 R314: C0 + 1e+06 alpha_ob34_seg0[2] <= 999994.0767956832
 R315: C2 + 1e+06 alpha_ob34_seg0[2] <= 999994.0767956832
 R316: C1 + 1e+06 alpha_ob34_seg0[3] <= 999937.4792742218
 R317: C3 + 1e+06 alpha_ob34_seg0[3] <= 999937.4792742218
 R318: alpha_ob34_seg0[0] + alpha_ob34_seg0[1] + alpha_ob34_seg0[2]
   + alpha_ob34_seg0[3] >= 1
 R319: - C0 + 1e+06 alpha_ob35_seg0[0] <= 999999.3718601406
 R320: - C2 + 1e+06 alpha_ob35_seg0[0] <= 999999.3718601406
 R321: - C1 + 1e+06 alpha_ob35_seg0[1] <= 1.0000353982997246e+06
 R322: - C3 + 1e+06 alpha_ob35_seg0[1] <= 1.0000353982997246e+06
 R323: C0 + 1e+06 alpha_ob35_seg0[2] <= 999998.0782041405
 R324: C2 + 1e+06 alpha_ob35_seg0[2] <= 999998.0782041405
 R325: C1 + 1e+06 alpha_ob35_seg0[3] <= 999944.7016997245
 R326: C3 + 1e+06 alpha_ob35_seg0[3] <= 999944.7016997245
 R327: alpha_ob35_seg0[0] + alpha_ob35_seg0[1] + alpha_ob35_seg0[2]
   + alpha_ob35_seg0[3] >= 1
 R328: - C0 + 1e+06 alpha_ob36_seg0[0] <= 999980.9374508709
 R329: - C2 + 1e+06 alpha_ob36_seg0[0] <= 999980.9374508709
 R330: - C1 + 1e+06 alpha_ob36_seg0[1] <= 1.0000348940002755e+06
 R331: - C3 + 1e+06 alpha_ob36_seg0[1] <= 1.0000348940002755e+06
 R332: C0 + 1e+06 alpha_ob36_seg0[2] <= 1.0000165124508708e+06
 R333: C2 + 1e+06 alpha_ob36_seg0[2] <= 1.0000165124508708e+06
 R334: C1 + 1e+06 alpha_ob36_seg0[3] <= 999935.9560002756
 R335: C3 + 1e+06 alpha_ob36_seg0[3] <= 999935.9560002756
 R336: alpha_ob36_seg0[0] + alpha_ob36_seg0[1] + alpha_ob36_seg0[2]
   + alpha_ob36_seg0[3] >= 1
 R337: - C0 + 1e+06 alpha_ob37_seg0[0] <= 999980.937499801
 R338: - C2 + 1e+06 alpha_ob37_seg0[0] <= 999980.937499801
 R339: - C1 + 1e+06 alpha_ob37_seg0[1] <= 1.0000614940072973e+06
 R340: - C3 + 1e+06 alpha_ob37_seg0[1] <= 1.0000614940072973e+06
 R341: C0 + 1e+06 alpha_ob37_seg0[2] <= 1.000011162499801e+06
 R342: C2 + 1e+06 alpha_ob37_seg0[2] <= 1.000011162499801e+06
 R343: C1 + 1e+06 alpha_ob37_seg0[3] <= 999935.9560072974
 R344: C3 + 1e+06 alpha_ob37_seg0[3] <= 999935.9560072974
 R345: alpha_ob37_seg0[0] + alpha_ob37_seg0[1] + alpha_ob37_seg0[2]
   + alpha_ob37_seg0[3] >= 1
 R346: - C0 + 1e+06 alpha_ob38_seg0[0] <= 999956.8394000001
 R347: - C2 + 1e+06 alpha_ob38_seg0[0] <= 999956.8394000001
 R348: - C1 + 1e+06 alpha_ob38_seg0[1] <= 999952.2463
 R349: - C3 + 1e+06 alpha_ob38_seg0[1] <= 999952.2463
 R350: C0 + 1e+06 alpha_ob38_seg0[2] <= 1.0000097606e+06
 R351: C2 + 1e+06 alpha_ob38_seg0[2] <= 1.0000097606e+06
 R352: C1 + 1e+06 alpha_ob38_seg0[3] <= 1.0000452037e+06
 R353: C3 + 1e+06 alpha_ob38_seg0[3] <= 1.0000452037e+06
 R354: alpha_ob38_seg0[0] + alpha_ob38_seg0[1] + alpha_ob38_seg0[2]
   + alpha_ob38_seg0[3] >= 1
 R355: - C0 + 1e+06 alpha_ob39_seg0[0] <= 999964.9321250001
 R356: - C2 + 1e+06 alpha_ob39_seg0[0] <= 999964.9321250001
 R357: - C1 + 1e+06 alpha_ob39_seg0[1] <= 1.0000141893e+06
 R358: - C3 + 1e+06 alpha_ob39_seg0[1] <= 1.0000141893e+06
 R359: C0 + 1e+06 alpha_ob39_seg0[2] <= 1.000025197325e+06
 R360: C2 + 1e+06 alpha_ob39_seg0[2] <= 1.000025197325e+06
 R361: C1 + 1e+06 alpha_ob39_seg0[3] <= 999983.2607
 R362: C3 + 1e+06 alpha_ob39_seg0[3] <= 999983.2607
 R363: alpha_ob39_seg0[0] + alpha_ob39_seg0[1] + alpha_ob39_seg0[2]
   + alpha_ob39_seg0[3] >= 1
 R364: - C0 + 1e+06 alpha_ob40_seg0[0] <= 999964.922208629
 R365: - C2 + 1e+06 alpha_ob40_seg0[0] <= 999964.922208629
 R366: - C1 + 1e+06 alpha_ob40_seg0[1] <= 1.0000141893002756e+06
 R367: - C3 + 1e+06 alpha_ob40_seg0[1] <= 1.0000141893002756e+06
 R368: C0 + 1e+06 alpha_ob40_seg0[2] <= 1.000032527608629e+06
 R369: C2 + 1e+06 alpha_ob40_seg0[2] <= 1.000032527608629e+06
 R370: C1 + 1e+06 alpha_ob40_seg0[3] <= 999933.6607002757
 R371: C3 + 1e+06 alpha_ob40_seg0[3] <= 999933.6607002757
 R372: alpha_ob40_seg0[0] + alpha_ob40_seg0[1] + alpha_ob40_seg0[2]
   + alpha_ob40_seg0[3] >= 1
 R373: - C0 + 1e+06 alpha_ob41_seg0[0] <= 999972.166285402
 R374: - C2 + 1e+06 alpha_ob41_seg0[0] <= 999972.166285402
 R375: - C1 + 1e+06 alpha_ob41_seg0[1] <= 1.0000141604997247e+06
 R376: - C3 + 1e+06 alpha_ob41_seg0[1] <= 1.0000141604997247e+06
 R377: C0 + 1e+06 alpha_ob41_seg0[2] <= 1.000025283885402e+06
 R378: C2 + 1e+06 alpha_ob41_seg0[2] <= 1.000025283885402e+06
 R379: C1 + 1e+06 alpha_ob41_seg0[3] <= 999936.9394997247
 R380: C3 + 1e+06 alpha_ob41_seg0[3] <= 999936.9394997247
 R381: alpha_ob41_seg0[0] + alpha_ob41_seg0[1] + alpha_ob41_seg0[2]
   + alpha_ob41_seg0[3] >= 1
 R382: - C0 + 1e+06 alpha_ob42_seg0[0] <= 999931.4945
 R383: - C2 + 1e+06 alpha_ob42_seg0[0] <= 999931.4945
 R384: - C1 + 1e+06 alpha_ob42_seg0[1] <= 1.0000158794e+06
 R385: - C3 + 1e+06 alpha_ob42_seg0[1] <= 1.0000158794e+06
 R386: C0 + 1e+06 alpha_ob42_seg0[2] <= 1.0000378554999999e+06
 R387: C2 + 1e+06 alpha_ob42_seg0[2] <= 1.0000378554999999e+06
 R388: C1 + 1e+06 alpha_ob42_seg0[3] <= 999981.5706
 R389: C3 + 1e+06 alpha_ob42_seg0[3] <= 999981.5706
 R390: alpha_ob42_seg0[0] + alpha_ob42_seg0[1] + alpha_ob42_seg0[2]
   + alpha_ob42_seg0[3] >= 1
 R391: - C0 + 1e+06 alpha_ob43_seg0[0] <= 999916.6145
 R392: - C2 + 1e+06 alpha_ob43_seg0[0] <= 999916.6145
 R393: - C1 + 1e+06 alpha_ob43_seg0[1] <= 1.0000160176e+06
 R394: - C3 + 1e+06 alpha_ob43_seg0[1] <= 1.0000160176e+06
 R395: C0 + 1e+06 alpha_ob43_seg0[2] <= 1.0000707355e+06
 R396: C2 + 1e+06 alpha_ob43_seg0[2] <= 1.0000707355e+06
 R397: C1 + 1e+06 alpha_ob43_seg0[3] <= 999981.4324
 R398: C3 + 1e+06 alpha_ob43_seg0[3] <= 999981.4324
 R399: alpha_ob43_seg0[0] + alpha_ob43_seg0[1] + alpha_ob43_seg0[2]
   + alpha_ob43_seg0[3] >= 1
 R400: - C0 + 1e+06 alpha_ob44_seg0[0] <= 999916.6143035784
 R401: - C2 + 1e+06 alpha_ob44_seg0[0] <= 999916.6143035784
 R402: - C1 + 1e+06 alpha_ob44_seg0[1] <= 1.0000160176002757e+06
 R403: - C3 + 1e+06 alpha_ob44_seg0[1] <= 1.0000160176002757e+06
 R404: C0 + 1e+06 alpha_ob44_seg0[2] <= 1.0000808355035784e+06
 R405: C2 + 1e+06 alpha_ob44_seg0[2] <= 1.0000808355035784e+06
 R406: C1 + 1e+06 alpha_ob44_seg0[3] <= 999929.0824002756
 R407: C3 + 1e+06 alpha_ob44_seg0[3] <= 999929.0824002756
 R408: alpha_ob44_seg0[0] + alpha_ob44_seg0[1] + alpha_ob44_seg0[2]
   + alpha_ob44_seg0[3] >= 1
 R409: - C0 + 1e+06 alpha_ob45_seg0[0] <= 999980.9375348955
 R410: - C2 + 1e+06 alpha_ob45_seg0[0] <= 999980.9375348955
 R411: - C1 + 1e+06 alpha_ob45_seg0[1] <= 1.0000160439997246e+06
 R412: - C3 + 1e+06 alpha_ob45_seg0[1] <= 1.0000160439997246e+06
 R413: C0 + 1e+06 alpha_ob45_seg0[2] <= 1.0000165125348954e+06
 R414: C2 + 1e+06 alpha_ob45_seg0[2] <= 1.0000165125348954e+06
 R415: C1 + 1e+06 alpha_ob45_seg0[3] <= 999962.5559997245
 R416: C3 + 1e+06 alpha_ob45_seg0[3] <= 999962.5559997245
 R417: alpha_ob45_seg0[0] + alpha_ob45_seg0[1] + alpha_ob45_seg0[2]
   + alpha_ob45_seg0[3] >= 1
 R418: - C0 + 1e+06 alpha_ob46_seg0[0] <= 999956.839399801
 R419: - C2 + 1e+06 alpha_ob46_seg0[0] <= 999956.839399801
 R420: - C1 + 1e+06 alpha_ob46_seg0[1] <= 999925.3963434525
 R421: - C3 + 1e+06 alpha_ob46_seg0[1] <= 999925.3963434525
 R422: C0 + 1e+06 alpha_ob46_seg0[2] <= 1.0000080105998011e+06
 R423: C2 + 1e+06 alpha_ob46_seg0[2] <= 1.0000080105998011e+06
 R424: C1 + 1e+06 alpha_ob46_seg0[3] <= 1.0000720537434525e+06
 R425: C3 + 1e+06 alpha_ob46_seg0[3] <= 1.0000720537434525e+06
 R426: alpha_ob46_seg0[0] + alpha_ob46_seg0[1] + alpha_ob46_seg0[2]
   + alpha_ob46_seg0[3] >= 1
 R427: - C0 + 1e+06 alpha_ob47_seg0[0] <= 999980.9374998009
 R428: - C2 + 1e+06 alpha_ob47_seg0[0] <= 999980.9374998009
 R429: - C1 + 1e+06 alpha_ob47_seg0[1] <= 1.0000160440053071e+06
 R430: - C3 + 1e+06 alpha_ob47_seg0[1] <= 1.0000160440053071e+06
 R431: C0 + 1e+06 alpha_ob47_seg0[2] <= 1.000012662499801e+06
 R432: C2 + 1e+06 alpha_ob47_seg0[2] <= 1.000012662499801e+06
 R433: C1 + 1e+06 alpha_ob47_seg0[3] <= 999981.4060053072
 R434: C3 + 1e+06 alpha_ob47_seg0[3] <= 999981.4060053072
 R435: alpha_ob47_seg0[0] + alpha_ob47_seg0[1] + alpha_ob47_seg0[2]
   + alpha_ob47_seg0[3] >= 1
 R436: - C0 + 1e+06 alpha_ob48_seg0[0] <= 999983.6053459151
 R437: - C2 + 1e+06 alpha_ob48_seg0[0] <= 999983.6053459151
 R438: - C1 + 1e+06 alpha_ob48_seg0[1] <= 999987.7722497246
 R439: - C3 + 1e+06 alpha_ob48_seg0[1] <= 999987.7722497246
 R440: C0 + 1e+06 alpha_ob48_seg0[2] <= 1.000013844745915e+06
 R441: C2 + 1e+06 alpha_ob48_seg0[2] <= 1.000013844745915e+06
 R442: C1 + 1e+06 alpha_ob48_seg0[3] <= 999984.8277497246
 R443: C3 + 1e+06 alpha_ob48_seg0[3] <= 999984.8277497246
 R444: alpha_ob48_seg0[0] + alpha_ob48_seg0[1] + alpha_ob48_seg0[2]
   + alpha_ob48_seg0[3] >= 1
 R445: - C0 + 1e+06 alpha_ob49_seg0[0] <= 999958.8717898987
 R446: - C2 + 1e+06 alpha_ob49_seg0[0] <= 999958.8717898987
 R447: - C1 + 1e+06 alpha_ob49_seg0[1] <= 999987.7837602755
 R448: - C3 + 1e+06 alpha_ob49_seg0[1] <= 999987.7837602755
 R449: C0 + 1e+06 alpha_ob49_seg0[2] <= 1.0000385781898986e+06
 R450: C2 + 1e+06 alpha_ob49_seg0[2] <= 1.0000385781898986e+06
 R451: C1 + 1e+06 alpha_ob49_seg0[3] <= 1.0000043162402755e+06
 R452: C3 + 1e+06 alpha_ob49_seg0[3] <= 1.0000043162402755e+06
 R453: alpha_ob49_seg0[0] + alpha_ob49_seg0[1] + alpha_ob49_seg0[2]
   + alpha_ob49_seg0[3] >= 1
 R454: - C0 + 1e+06 alpha_ob50_seg0[0] <= 999939.2718
 R455: - C2 + 1e+06 alpha_ob50_seg0[0] <= 999939.2718
 R456: - C1 + 1e+06 alpha_ob50_seg0[1] <= 999993.13376
 R457: - C3 + 1e+06 alpha_ob50_seg0[1] <= 999993.13376
 R458: C0 + 1e+06 alpha_ob50_seg0[2] <= 1.0000385782e+06
 R459: C2 + 1e+06 alpha_ob50_seg0[2] <= 1.0000385782e+06
 R460: C1 + 1e+06 alpha_ob50_seg0[3] <= 1.00000431624e+06
 R461: C3 + 1e+06 alpha_ob50_seg0[3] <= 1.00000431624e+06
 R462: alpha_ob50_seg0[0] + alpha_ob50_seg0[1] + alpha_ob50_seg0[2]
   + alpha_ob50_seg0[3] >= 1
 R463: - C0 + 1e+06 alpha_ob51_seg0[0] <= 999939.2718101013
 R464: - C2 + 1e+06 alpha_ob51_seg0[0] <= 999939.2718101013
 R465: - C1 + 1e+06 alpha_ob51_seg0[1] <= 999987.7837597246
 R466: - C3 + 1e+06 alpha_ob51_seg0[1] <= 999987.7837597246
 R467: C0 + 1e+06 alpha_ob51_seg0[2] <= 1.0000581782101013e+06
 R468: C2 + 1e+06 alpha_ob51_seg0[2] <= 1.0000581782101013e+06
 R469: C1 + 1e+06 alpha_ob51_seg0[3] <= 1.0000043162397245e+06
 R470: C3 + 1e+06 alpha_ob51_seg0[3] <= 1.0000043162397245e+06
 R471: alpha_ob51_seg0[0] + alpha_ob51_seg0[1] + alpha_ob51_seg0[2]
   + alpha_ob51_seg0[3] >= 1
 R472: - C0 + 1e+06 alpha_ob52_seg0[0] <= 999939.271799801
 R473: - C2 + 1e+06 alpha_ob52_seg0[0] <= 999939.271799801
 R474: - C1 + 1e+06 alpha_ob52_seg0[1] <= 999987.7838262042
 R475: - C3 + 1e+06 alpha_ob52_seg0[1] <= 999987.7838262042
 R476: C0 + 1e+06 alpha_ob52_seg0[2] <= 1.000038578199801e+06
 R477: C2 + 1e+06 alpha_ob52_seg0[2] <= 1.000038578199801e+06
 R478: C1 + 1e+06 alpha_ob52_seg0[3] <= 1.0000096662262043e+06
 R479: C3 + 1e+06 alpha_ob52_seg0[3] <= 1.0000096662262043e+06
 R480: alpha_ob52_seg0[0] + alpha_ob52_seg0[1] + alpha_ob52_seg0[2]
   + alpha_ob52_seg0[3] >= 1
 R481: - C0 + 1e+06 alpha_ob53_seg0[0] <= 999926.0708
 R482: - C2 + 1e+06 alpha_ob53_seg0[0] <= 999926.0708
 R483: - C1 + 1e+06 alpha_ob53_seg0[1] <= 999983.7196
 R484: - C3 + 1e+06 alpha_ob53_seg0[1] <= 999983.7196
 R485: C0 + 1e+06 alpha_ob53_seg0[2] <= 1.0000250292e+06
 R486: C2 + 1e+06 alpha_ob53_seg0[2] <= 1.0000250292e+06
 R487: C1 + 1e+06 alpha_ob53_seg0[3] <= 1.0000137304e+06
 R488: C3 + 1e+06 alpha_ob53_seg0[3] <= 1.0000137304e+06
 R489: alpha_ob53_seg0[0] + alpha_ob53_seg0[1] + alpha_ob53_seg0[2]
   + alpha_ob53_seg0[3] >= 1
 R490: - C0 + 1e+06 alpha_ob54_seg0[0] <= 999926.0708293857
 R491: - C2 + 1e+06 alpha_ob54_seg0[0] <= 999926.0708293857
 R492: - C1 + 1e+06 alpha_ob54_seg0[1] <= 999967.8695997245
 R493: - C3 + 1e+06 alpha_ob54_seg0[1] <= 999967.8695997245
 R494: C0 + 1e+06 alpha_ob54_seg0[2] <= 1.0000713792293856e+06
 R495: C2 + 1e+06 alpha_ob54_seg0[2] <= 1.0000713792293856e+06
 R496: C1 + 1e+06 alpha_ob54_seg0[3] <= 1.0000137303997246e+06
 R497: C3 + 1e+06 alpha_ob54_seg0[3] <= 1.0000137303997246e+06
 R498: alpha_ob54_seg0[0] + alpha_ob54_seg0[1] + alpha_ob54_seg0[2]
   + alpha_ob54_seg0[3] >= 1
 R499: - C0 + 1e+06 alpha_ob55_seg0[0] <= 999972.6917508709
 R500: - C2 + 1e+06 alpha_ob55_seg0[0] <= 999972.6917508709
 R501: - C1 + 1e+06 alpha_ob55_seg0[1] <= 999957.1644002756
 R502: - C3 + 1e+06 alpha_ob55_seg0[1] <= 999957.1644002756
 R503: C0 + 1e+06 alpha_ob55_seg0[2] <= 1.0000247581508709e+06
 R504: C2 + 1e+06 alpha_ob55_seg0[2] <= 1.0000247581508709e+06
 R505: C1 + 1e+06 alpha_ob55_seg0[3] <= 1.0000136856002755e+06
 R506: C3 + 1e+06 alpha_ob55_seg0[3] <= 1.0000136856002755e+06
 R507: alpha_ob55_seg0[0] + alpha_ob55_seg0[1] + alpha_ob55_seg0[2]
   + alpha_ob55_seg0[3] >= 1
 R508: - C0 + 1e+06 alpha_ob56_seg0[0] <= 999939.0009674003
 R509: - C2 + 1e+06 alpha_ob56_seg0[0] <= 999939.0009674003
 R510: - C1 + 1e+06 alpha_ob56_seg0[1] <= 999961.9516002756
 R511: - C3 + 1e+06 alpha_ob56_seg0[1] <= 999961.9516002756
 R512: C0 + 1e+06 alpha_ob56_seg0[2] <= 1.0000584489674004e+06
 R513: C2 + 1e+06 alpha_ob56_seg0[2] <= 1.0000584489674004e+06
 R514: C1 + 1e+06 alpha_ob56_seg0[3] <= 1.0000178984002755e+06
 R515: C3 + 1e+06 alpha_ob56_seg0[3] <= 1.0000178984002755e+06
 R516: alpha_ob56_seg0[0] + alpha_ob56_seg0[1] + alpha_ob56_seg0[2]
   + alpha_ob56_seg0[3] >= 1
 R517: - C0 + 1e+06 alpha_ob57_seg0[0] <= 999994.9164998011
 R518: - C2 + 1e+06 alpha_ob57_seg0[0] <= 999994.9164998011
 R519: - C1 + 1e+06 alpha_ob57_seg0[1] <= 999925.4284782809
 R520: - C3 + 1e+06 alpha_ob57_seg0[1] <= 999925.4284782809
 R521: C0 + 1e+06 alpha_ob57_seg0[2] <= 999943.6834998011
 R522: C2 + 1e+06 alpha_ob57_seg0[2] <= 999943.6834998011
 R523: C1 + 1e+06 alpha_ob57_seg0[3] <= 1.0000720216782809e+06
 R524: C3 + 1e+06 alpha_ob57_seg0[3] <= 1.0000720216782809e+06
 R525: alpha_ob57_seg0[0] + alpha_ob57_seg0[1] + alpha_ob57_seg0[2]
   + alpha_ob57_seg0[3] >= 1
 R526: - C0 + 1e+06 alpha_ob58_seg0[0] <= 999920.7556087238
 R527: - C2 + 1e+06 alpha_ob58_seg0[0] <= 999920.7556087238
 R528: - C1 + 1e+06 alpha_ob58_seg0[1] <= 1.0000111845997245e+06
 R529: - C3 + 1e+06 alpha_ob58_seg0[1] <= 1.0000111845997245e+06
 R530: C0 + 1e+06 alpha_ob58_seg0[2] <= 1.0000766944087239e+06
 R531: C2 + 1e+06 alpha_ob58_seg0[2] <= 1.0000766944087239e+06
 R532: C1 + 1e+06 alpha_ob58_seg0[3] <= 999981.6653997245
 R533: C3 + 1e+06 alpha_ob58_seg0[3] <= 999981.6653997245
 R534: alpha_ob58_seg0[0] + alpha_ob58_seg0[1] + alpha_ob58_seg0[2]
   + alpha_ob58_seg0[3] >= 1
 R535: - C0 + 1e+06 alpha_ob59_seg0[0] <= 999911.9056000001
 R536: - C2 + 1e+06 alpha_ob59_seg0[0] <= 999911.9056000001
 R537: - C1 + 1e+06 alpha_ob59_seg0[1] <= 1.0000111846e+06
 R538: - C3 + 1e+06 alpha_ob59_seg0[1] <= 1.0000111846e+06
 R539: C0 + 1e+06 alpha_ob59_seg0[2] <= 1.0000766944e+06
 R540: C2 + 1e+06 alpha_ob59_seg0[2] <= 1.0000766944e+06
 R541: C1 + 1e+06 alpha_ob59_seg0[3] <= 999986.2654
 R542: C3 + 1e+06 alpha_ob59_seg0[3] <= 999986.2654
 R543: alpha_ob59_seg0[0] + alpha_ob59_seg0[1] + alpha_ob59_seg0[2]
   + alpha_ob59_seg0[3] >= 1
 R544: - C0 + 1e+06 alpha_ob60_seg0[0] <= 999914.52327842
 R545: - C2 + 1e+06 alpha_ob60_seg0[0] <= 999914.52327842
 R546: - C1 + 1e+06 alpha_ob60_seg0[1] <= 999993.6552362755
 R547: - C3 + 1e+06 alpha_ob60_seg0[1] <= 999993.6552362755
 R548: C0 + 1e+06 alpha_ob60_seg0[2] <= 1.0000829266784199e+06
 R549: C2 + 1e+06 alpha_ob60_seg0[2] <= 1.0000829266784199e+06
 R550: C1 + 1e+06 alpha_ob60_seg0[3] <= 999992.1947642756
 R551: C3 + 1e+06 alpha_ob60_seg0[3] <= 999992.1947642756
 R552: alpha_ob60_seg0[0] + alpha_ob60_seg0[1] + alpha_ob60_seg0[2]
   + alpha_ob60_seg0[3] >= 1
 R553: - C0 + 1e+06 alpha_ob61_seg0[0] <= 999912.265199801
 R554: - C2 + 1e+06 alpha_ob61_seg0[0] <= 999912.265199801
 R555: - C1 + 1e+06 alpha_ob61_seg0[1] <= 999987.7842112777
 R556: - C3 + 1e+06 alpha_ob61_seg0[1] <= 999987.7842112777
 R557: C0 + 1e+06 alpha_ob61_seg0[2] <= 1.000076834799801e+06
 R558: C2 + 1e+06 alpha_ob61_seg0[2] <= 1.000076834799801e+06
 R559: C1 + 1e+06 alpha_ob61_seg0[3] <= 1.0000096658112778e+06
 R560: C3 + 1e+06 alpha_ob61_seg0[3] <= 1.0000096658112778e+06
 R561: alpha_ob61_seg0[0] + alpha_ob61_seg0[1] + alpha_ob61_seg0[2]
   + alpha_ob61_seg0[3] >= 1
 R562: - C0 + 1e+06 alpha_ob62_seg0[0] <= 999920.6152119379
 R563: - C2 + 1e+06 alpha_ob62_seg0[0] <= 999920.6152119379
 R564: - C1 + 1e+06 alpha_ob62_seg0[1] <= 999981.4341997246
 R565: - C3 + 1e+06 alpha_ob62_seg0[1] <= 999981.4341997246
 R566: C0 + 1e+06 alpha_ob62_seg0[2] <= 1.0000768348119379e+06
 R567: C2 + 1e+06 alpha_ob62_seg0[2] <= 1.0000768348119379e+06
 R568: C1 + 1e+06 alpha_ob62_seg0[3] <= 1.0000096657997245e+06
 R569: C3 + 1e+06 alpha_ob62_seg0[3] <= 1.0000096657997245e+06
 R570: alpha_ob62_seg0[0] + alpha_ob62_seg0[1] + alpha_ob62_seg0[2]
   + alpha_ob62_seg0[3] >= 1
 R571: - C0 + 1e+06 alpha_ob63_seg0[0] <= 999919.2652
 R572: - C2 + 1e+06 alpha_ob63_seg0[0] <= 999919.2652
 R573: - C1 + 1e+06 alpha_ob63_seg0[1] <= 999981.4342
 R574: - C3 + 1e+06 alpha_ob63_seg0[1] <= 999981.4342
 R575: C0 + 1e+06 alpha_ob63_seg0[2] <= 1.0000768348e+06
 R576: C2 + 1e+06 alpha_ob63_seg0[2] <= 1.0000768348e+06
 R577: C1 + 1e+06 alpha_ob63_seg0[3] <= 1.0000160158000001e+06
 R578: C3 + 1e+06 alpha_ob63_seg0[3] <= 1.0000160158000001e+06
 R579: alpha_ob63_seg0[0] + alpha_ob63_seg0[1] + alpha_ob63_seg0[2]
   + alpha_ob63_seg0[3] >= 1
 R580: - C0 + 1e+06 alpha_ob64_seg0[0] <= 999919.2652454559
 R581: - C2 + 1e+06 alpha_ob64_seg0[0] <= 999919.2652454559
 R582: - C1 + 1e+06 alpha_ob64_seg0[1] <= 999956.8341997246
 R583: - C3 + 1e+06 alpha_ob64_seg0[1] <= 999956.8341997246
 R584: C0 + 1e+06 alpha_ob64_seg0[2] <= 1.0000781848454559e+06
 R585: C2 + 1e+06 alpha_ob64_seg0[2] <= 1.0000781848454559e+06
 R586: C1 + 1e+06 alpha_ob64_seg0[3] <= 1.0000160157997246e+06
 R587: C3 + 1e+06 alpha_ob64_seg0[3] <= 1.0000160157997246e+06
 R588: alpha_ob64_seg0[0] + alpha_ob64_seg0[1] + alpha_ob64_seg0[2]
   + alpha_ob64_seg0[3] >= 1
 R589: - C0 + 1e+06 alpha_ob65_seg0[0] <= 1.0000081732920931e+06
 R590: - C2 + 1e+06 alpha_ob65_seg0[0] <= 1.0000081732920931e+06
 R591: - C1 + 1e+06 alpha_ob65_seg0[1] <= 999925.4331002756
 R592: - C3 + 1e+06 alpha_ob65_seg0[1] <= 999925.4331002756
 R593: C0 + 1e+06 alpha_ob65_seg0[2] <= 999989.2766720931
 R594: C2 + 1e+06 alpha_ob65_seg0[2] <= 999989.2766720931
 R595: C1 + 1e+06 alpha_ob65_seg0[3] <= 1.0000624169002755e+06
 R596: C3 + 1e+06 alpha_ob65_seg0[3] <= 1.0000624169002755e+06
 R597: alpha_ob65_seg0[0] + alpha_ob65_seg0[1] + alpha_ob65_seg0[2]
   + alpha_ob65_seg0[3] >= 1
 R598: - C0 + 1e+06 alpha_ob66_seg0[0] <= 1.0000146737e+06
 R599: - C2 + 1e+06 alpha_ob66_seg0[0] <= 1.0000146737e+06
 R600: - C1 + 1e+06 alpha_ob66_seg0[1] <= 999940.7624
 R601: - C3 + 1e+06 alpha_ob66_seg0[1] <= 999940.7624
 R602: C0 + 1e+06 alpha_ob66_seg0[2] <= 999948.4263000001
 R603: C2 + 1e+06 alpha_ob66_seg0[2] <= 999948.4263000001
 R604: C1 + 1e+06 alpha_ob66_seg0[3] <= 1.0000566875999999e+06
 R605: C3 + 1e+06 alpha_ob66_seg0[3] <= 1.0000566875999999e+06
 R606: alpha_ob66_seg0[0] + alpha_ob66_seg0[1] + alpha_ob66_seg0[2]
   + alpha_ob66_seg0[3] >= 1
 R607: - C0 + 1e+06 alpha_ob67_seg0[0] <= 1.0000058189e+06
 R608: - C2 + 1e+06 alpha_ob67_seg0[0] <= 1.0000058189e+06
 R609: - C1 + 1e+06 alpha_ob67_seg0[1] <= 999940.5469
 R610: - C3 + 1e+06 alpha_ob67_seg0[1] <= 999940.5469
 R611: C0 + 1e+06 alpha_ob67_seg0[2] <= 999985.5311
 R612: C2 + 1e+06 alpha_ob67_seg0[2] <= 999985.5311
 R613: C1 + 1e+06 alpha_ob67_seg0[3] <= 1.0000569031e+06
 R614: C3 + 1e+06 alpha_ob67_seg0[3] <= 1.0000569031e+06
 R615: alpha_ob67_seg0[0] + alpha_ob67_seg0[1] + alpha_ob67_seg0[2]
   + alpha_ob67_seg0[3] >= 1
 R616: - C0 + 1e+06 alpha_ob68_seg0[0] <= 1.0000058188284199e+06
 R617: - C2 + 1e+06 alpha_ob68_seg0[0] <= 1.0000058188284199e+06
 R618: - C1 + 1e+06 alpha_ob68_seg0[1] <= 999940.5469002755
 R619: - C3 + 1e+06 alpha_ob68_seg0[1] <= 999940.5469002755
 R620: C0 + 1e+06 alpha_ob68_seg0[2] <= 999991.6311284199
 R621: C2 + 1e+06 alpha_ob68_seg0[2] <= 999991.6311284199
 R622: C1 + 1e+06 alpha_ob68_seg0[3] <= 1.0000453031002756e+06
 R623: C3 + 1e+06 alpha_ob68_seg0[3] <= 1.0000453031002756e+06
 R624: alpha_ob68_seg0[0] + alpha_ob68_seg0[1] + alpha_ob68_seg0[2]
   + alpha_ob68_seg0[3] >= 1
 R625: - C0 + 1e+06 alpha_ob69_seg0[0] <= 999996.9688499999
 R626: - C2 + 1e+06 alpha_ob69_seg0[0] <= 999996.9688499999
 R627: - C1 + 1e+06 alpha_ob69_seg0[1] <= 999940.5469
 R628: - C3 + 1e+06 alpha_ob69_seg0[1] <= 999940.5469
 R629: C0 + 1e+06 alpha_ob69_seg0[2] <= 999991.63115
 R630: C2 + 1e+06 alpha_ob69_seg0[2] <= 999991.63115
 R631: C1 + 1e+06 alpha_ob69_seg0[3] <= 1.0000569031e+06
 R632: C3 + 1e+06 alpha_ob69_seg0[3] <= 1.0000569031e+06
 R633: alpha_ob69_seg0[0] + alpha_ob69_seg0[1] + alpha_ob69_seg0[2]
   + alpha_ob69_seg0[3] >= 1
 R634: - C0 + 1e+06 alpha_ob70_seg0[0] <= 999991.83683
 R635: - C2 + 1e+06 alpha_ob70_seg0[0] <= 999991.83683
 R636: - C1 + 1e+06 alpha_ob70_seg0[1] <= 999940.589
 R637: - C3 + 1e+06 alpha_ob70_seg0[1] <= 999940.589
 R638: C0 + 1e+06 alpha_ob70_seg0[2] <= 1.00000401317e+06
 R639: C2 + 1e+06 alpha_ob70_seg0[2] <= 1.00000401317e+06
 R640: C1 + 1e+06 alpha_ob70_seg0[3] <= 1.000056861e+06
 R641: C3 + 1e+06 alpha_ob70_seg0[3] <= 1.000056861e+06
 R642: alpha_ob70_seg0[0] + alpha_ob70_seg0[1] + alpha_ob70_seg0[2]
   + alpha_ob70_seg0[3] >= 1
 R643: - C0 + 1e+06 alpha_ob71_seg0[0] <= 999991.8368084199
 R644: - C2 + 1e+06 alpha_ob71_seg0[0] <= 999991.8368084199
 R645: - C1 + 1e+06 alpha_ob71_seg0[1] <= 999940.5890002755
 R646: - C3 + 1e+06 alpha_ob71_seg0[1] <= 999940.5890002755
 R647: C0 + 1e+06 alpha_ob71_seg0[2] <= 1.0000056131484199e+06
 R648: C2 + 1e+06 alpha_ob71_seg0[2] <= 1.0000056131484199e+06
 R649: C1 + 1e+06 alpha_ob71_seg0[3] <= 1.0000452610002755e+06
 R650: C3 + 1e+06 alpha_ob71_seg0[3] <= 1.0000452610002755e+06
 R651: alpha_ob71_seg0[0] + alpha_ob71_seg0[1] + alpha_ob71_seg0[2]
   + alpha_ob71_seg0[3] >= 1
 R652: - C0 + 1e+06 alpha_ob72_seg0[0] <= 1.0000764021508709e+06
 R653: - C2 + 1e+06 alpha_ob72_seg0[0] <= 1.0000764021508709e+06
 R654: - C1 + 1e+06 alpha_ob72_seg0[1] <= 999957.1027002756
 R655: - C3 + 1e+06 alpha_ob72_seg0[1] <= 999957.1027002756
 R656: C0 + 1e+06 alpha_ob72_seg0[2] <= 999921.0477508709
 R657: C2 + 1e+06 alpha_ob72_seg0[2] <= 999921.0477508709
 R658: C1 + 1e+06 alpha_ob72_seg0[3] <= 1.0000137473002756e+06
 R659: C3 + 1e+06 alpha_ob72_seg0[3] <= 1.0000137473002756e+06
 R660: alpha_ob72_seg0[0] + alpha_ob72_seg0[1] + alpha_ob72_seg0[2]
   + alpha_ob72_seg0[3] >= 1
 R661: - C0 + 1e+06 alpha_ob73_seg0[0] <= 1.0000445522e+06
 R662: - C2 + 1e+06 alpha_ob73_seg0[0] <= 1.0000445522e+06
 R663: - C1 + 1e+06 alpha_ob73_seg0[1] <= 999983.7027
 R664: - C3 + 1e+06 alpha_ob73_seg0[1] <= 999983.7027
 R665: C0 + 1e+06 alpha_ob73_seg0[2] <= 999921.0477999999
 R666: C2 + 1e+06 alpha_ob73_seg0[2] <= 999921.0477999999
 R667: C1 + 1e+06 alpha_ob73_seg0[3] <= 1.0000137473e+06
 R668: C3 + 1e+06 alpha_ob73_seg0[3] <= 1.0000137473e+06
 R669: alpha_ob73_seg0[0] + alpha_ob73_seg0[1] + alpha_ob73_seg0[2]
   + alpha_ob73_seg0[3] >= 1
 R670: - C0 + 1e+06 alpha_ob74_seg0[0] <= 1.0000445522183661e+06
 R671: - C2 + 1e+06 alpha_ob74_seg0[0] <= 1.0000445522183661e+06
 R672: - C1 + 1e+06 alpha_ob74_seg0[1] <= 999973.8526997245
 R673: - C3 + 1e+06 alpha_ob74_seg0[1] <= 999973.8526997245
 R674: C0 + 1e+06 alpha_ob74_seg0[2] <= 999952.8978183661
 R675: C2 + 1e+06 alpha_ob74_seg0[2] <= 999952.8978183661
 R676: C1 + 1e+06 alpha_ob74_seg0[3] <= 1.0000137472997245e+06
 R677: C3 + 1e+06 alpha_ob74_seg0[3] <= 1.0000137472997245e+06
 R678: alpha_ob74_seg0[0] + alpha_ob74_seg0[1] + alpha_ob74_seg0[2]
   + alpha_ob74_seg0[3] >= 1
 R679: - C0 + 1e+06 alpha_ob75_seg0[0] <= 1.000064332081634e+06
 R680: - C2 + 1e+06 alpha_ob75_seg0[0] <= 1.000064332081634e+06
 R681: - C1 + 1e+06 alpha_ob75_seg0[1] <= 999973.9401002756
 R682: - C3 + 1e+06 alpha_ob75_seg0[1] <= 999973.9401002756
 R683: C0 + 1e+06 alpha_ob75_seg0[2] <= 999933.1178816339
 R684: C2 + 1e+06 alpha_ob75_seg0[2] <= 999933.1178816339
 R685: C1 + 1e+06 alpha_ob75_seg0[3] <= 1.0000136599002755e+06
 R686: C3 + 1e+06 alpha_ob75_seg0[3] <= 1.0000136599002755e+06
 R687: alpha_ob75_seg0[0] + alpha_ob75_seg0[1] + alpha_ob75_seg0[2]
   + alpha_ob75_seg0[3] >= 1
 R688: - C0 + 1e+06 alpha_ob76_seg0[0] <= 1.0000643321013774e+06
 R689: - C2 + 1e+06 alpha_ob76_seg0[0] <= 1.0000643321013774e+06
 R690: - C1 + 1e+06 alpha_ob76_seg0[1] <= 999983.1900997246
 R691: - C3 + 1e+06 alpha_ob76_seg0[1] <= 999983.1900997246
 R692: C0 + 1e+06 alpha_ob76_seg0[2] <= 999933.1179013775
 R693: C2 + 1e+06 alpha_ob76_seg0[2] <= 999933.1179013775
 R694: C1 + 1e+06 alpha_ob76_seg0[3] <= 1.0000136598997245e+06
 R695: C3 + 1e+06 alpha_ob76_seg0[3] <= 1.0000136598997245e+06
 R696: alpha_ob76_seg0[0] + alpha_ob76_seg0[1] + alpha_ob76_seg0[2]
   + alpha_ob76_seg0[3] >= 1
 R697: - C2 + 1e+06 alpha_ob0_seg1[0] <= 1.0000284170499525e+06
 R698: - C4 + 1e+06 alpha_ob0_seg1[0] <= 1.0000284170499525e+06
 R699: - C3 + 1e+06 alpha_ob0_seg1[1] <= 999957.0843002755
 R700: - C5 + 1e+06 alpha_ob0_seg1[1] <= 999957.0843002755
 R701: C2 + 1e+06 alpha_ob0_seg1[2] <= 999969.0328499526
 R702: C4 + 1e+06 alpha_ob0_seg1[2] <= 999969.0328499526
 R703: C3 + 1e+06 alpha_ob0_seg1[3] <= 1.0000132657002755e+06
 R704: C5 + 1e+06 alpha_ob0_seg1[3] <= 1.0000132657002755e+06
 R705: alpha_ob0_seg1[0] + alpha_ob0_seg1[1] + alpha_ob0_seg1[2]
   + alpha_ob0_seg1[3] >= 1
 R706: - C2 + 1e+06 alpha_ob1_seg1[0] <= 1.000028417099801e+06
 R707: - C4 + 1e+06 alpha_ob1_seg1[0] <= 1.000028417099801e+06
 R708: - C3 + 1e+06 alpha_ob1_seg1[1] <= 999984.1843155898
 R709: - C5 + 1e+06 alpha_ob1_seg1[1] <= 999984.1843155898
 R710: C2 + 1e+06 alpha_ob1_seg1[2] <= 999957.4328998009
 R711: C4 + 1e+06 alpha_ob1_seg1[2] <= 999957.4328998009
 R712: C3 + 1e+06 alpha_ob1_seg1[3] <= 1.0000132657155899e+06
 R713: C5 + 1e+06 alpha_ob1_seg1[3] <= 1.0000132657155899e+06
 R714: alpha_ob1_seg1[0] + alpha_ob1_seg1[1] + alpha_ob1_seg1[2]
   + alpha_ob1_seg1[3] >= 1
 R715: - C2 + 1e+06 alpha_ob2_seg1[0] <= 1.0000400171500475e+06
 R716: - C4 + 1e+06 alpha_ob2_seg1[0] <= 1.0000400171500475e+06
 R717: - C3 + 1e+06 alpha_ob2_seg1[1] <= 999957.0842997245
 R718: - C5 + 1e+06 alpha_ob2_seg1[1] <= 999957.0842997245
 R719: C2 + 1e+06 alpha_ob2_seg1[2] <= 999957.4329500474
 R720: C4 + 1e+06 alpha_ob2_seg1[2] <= 999957.4329500474
 R721: C3 + 1e+06 alpha_ob2_seg1[3] <= 1.0000132656997246e+06
 R722: C5 + 1e+06 alpha_ob2_seg1[3] <= 1.0000132656997246e+06
 R723: alpha_ob2_seg1[0] + alpha_ob2_seg1[1] + alpha_ob2_seg1[2]
   + alpha_ob2_seg1[3] >= 1
 R724: - C2 + 1e+06 alpha_ob3_seg1[0] <= 1.0000109162e+06
 R725: - C4 + 1e+06 alpha_ob3_seg1[0] <= 1.0000109162e+06
 R726: - C3 + 1e+06 alpha_ob3_seg1[1] <= 999970.2681
 R727: - C5 + 1e+06 alpha_ob3_seg1[1] <= 999970.2681
 R728: C2 + 1e+06 alpha_ob3_seg1[2] <= 999975.1838
 R729: C4 + 1e+06 alpha_ob3_seg1[2] <= 999975.1838
 R730: C3 + 1e+06 alpha_ob3_seg1[3] <= 1.0000271819e+06
 R731: C5 + 1e+06 alpha_ob3_seg1[3] <= 1.0000271819e+06
 R732: alpha_ob3_seg1[0] + alpha_ob3_seg1[1] + alpha_ob3_seg1[2]
   + alpha_ob3_seg1[3] >= 1
 R733: - C2 + 1e+06 alpha_ob4_seg1[0] <= 1.0000109161752058e+06
 R734: - C4 + 1e+06 alpha_ob4_seg1[0] <= 1.0000109161752058e+06
 R735: - C3 + 1e+06 alpha_ob4_seg1[1] <= 999970.2681002755
 R736: - C5 + 1e+06 alpha_ob4_seg1[1] <= 999970.2681002755
 R737: C2 + 1e+06 alpha_ob4_seg1[2] <= 999986.5337752058
 R738: C4 + 1e+06 alpha_ob4_seg1[2] <= 999986.5337752058
 R739: C3 + 1e+06 alpha_ob4_seg1[3] <= 1.0000138319002755e+06
 R740: C5 + 1e+06 alpha_ob4_seg1[3] <= 1.0000138319002755e+06
 R741: alpha_ob4_seg1[0] + alpha_ob4_seg1[1] + alpha_ob4_seg1[2]
   + alpha_ob4_seg1[3] >= 1
 R742: - C2 + 1e+06 alpha_ob5_seg1[0] <= 1.000010916199801e+06
 R743: - C4 + 1e+06 alpha_ob5_seg1[0] <= 1.000010916199801e+06
 R744: - C3 + 1e+06 alpha_ob5_seg1[1] <= 999983.6181179117
 R745: - C5 + 1e+06 alpha_ob5_seg1[1] <= 999983.6181179117
 R746: C2 + 1e+06 alpha_ob5_seg1[2] <= 999973.183799801
 R747: C4 + 1e+06 alpha_ob5_seg1[2] <= 999973.183799801
 R748: C3 + 1e+06 alpha_ob5_seg1[3] <= 1.0000138319179118e+06
 R749: C5 + 1e+06 alpha_ob5_seg1[3] <= 1.0000138319179118e+06
 R750: alpha_ob5_seg1[0] + alpha_ob5_seg1[1] + alpha_ob5_seg1[2]
   + alpha_ob5_seg1[3] >= 1
 R751: - C2 + 1e+06 alpha_ob6_seg1[0] <= 1.0000139162e+06
 R752: - C4 + 1e+06 alpha_ob6_seg1[0] <= 1.0000139162e+06
 R753: - C3 + 1e+06 alpha_ob6_seg1[1] <= 999957.1679999999
 R754: - C5 + 1e+06 alpha_ob6_seg1[1] <= 999957.1679999999
 R755: C2 + 1e+06 alpha_ob6_seg1[2] <= 999975.1838
 R756: C4 + 1e+06 alpha_ob6_seg1[2] <= 999975.1838
 R757: C3 + 1e+06 alpha_ob6_seg1[3] <= 1.000040282e+06
 R758: C5 + 1e+06 alpha_ob6_seg1[3] <= 1.000040282e+06
 R759: alpha_ob6_seg1[0] + alpha_ob6_seg1[1] + alpha_ob6_seg1[2]
   + alpha_ob6_seg1[3] >= 1
 R760: - C2 + 1e+06 alpha_ob7_seg1[0] <= 999992.49102
 R761: - C4 + 1e+06 alpha_ob7_seg1[0] <= 999992.49102
 R762: - C3 + 1e+06 alpha_ob7_seg1[1] <= 999957.1016000001
 R763: - C5 + 1e+06 alpha_ob7_seg1[1] <= 999957.1016000001
 R764: C2 + 1e+06 alpha_ob7_seg1[2] <= 999986.35898
 R765: C4 + 1e+06 alpha_ob7_seg1[2] <= 999986.35898
 R766: C3 + 1e+06 alpha_ob7_seg1[3] <= 1.0000403484e+06
 R767: C5 + 1e+06 alpha_ob7_seg1[3] <= 1.0000403484e+06
 R768: alpha_ob7_seg1[0] + alpha_ob7_seg1[1] + alpha_ob7_seg1[2]
   + alpha_ob7_seg1[3] >= 1
 R769: - C2 + 1e+06 alpha_ob8_seg1[0] <= 999981.3909999999
 R770: - C4 + 1e+06 alpha_ob8_seg1[0] <= 999981.3909999999
 R771: - C3 + 1e+06 alpha_ob8_seg1[1] <= 999957.1016000001
 R772: - C5 + 1e+06 alpha_ob8_seg1[1] <= 999957.1016000001
 R773: C2 + 1e+06 alpha_ob8_seg1[2] <= 1.000004959e+06
 R774: C4 + 1e+06 alpha_ob8_seg1[2] <= 1.000004959e+06
 R775: C3 + 1e+06 alpha_ob8_seg1[3] <= 1.0000403484e+06
 R776: C5 + 1e+06 alpha_ob8_seg1[3] <= 1.0000403484e+06
 R777: alpha_ob8_seg1[0] + alpha_ob8_seg1[1] + alpha_ob8_seg1[2]
   + alpha_ob8_seg1[3] >= 1
 R778: - C2 + 1e+06 alpha_ob9_seg1[0] <= 999981.3909825523
 R779: - C4 + 1e+06 alpha_ob9_seg1[0] <= 999981.3909825523
 R780: - C3 + 1e+06 alpha_ob9_seg1[1] <= 999957.1016002755
 R781: - C5 + 1e+06 alpha_ob9_seg1[1] <= 999957.1016002755
 R782: C2 + 1e+06 alpha_ob9_seg1[2] <= 1.0000160589825523e+06
 R783: C4 + 1e+06 alpha_ob9_seg1[2] <= 1.0000160589825523e+06
 R784: C3 + 1e+06 alpha_ob9_seg1[3] <= 1.0000309984002755e+06
 R785: C5 + 1e+06 alpha_ob9_seg1[3] <= 1.0000309984002755e+06
 R786: alpha_ob9_seg1[0] + alpha_ob9_seg1[1] + alpha_ob9_seg1[2]
   + alpha_ob9_seg1[3] >= 1
 R787: - C2 + 1e+06 alpha_ob10_seg1[0] <= 999981.3867954084
 R788: - C4 + 1e+06 alpha_ob10_seg1[0] <= 999981.3867954084
 R789: - C3 + 1e+06 alpha_ob10_seg1[1] <= 999968.8595002755
 R790: - C5 + 1e+06 alpha_ob10_seg1[1] <= 999968.8595002755
 R791: C2 + 1e+06 alpha_ob10_seg1[2] <= 1.0000160631954084e+06
 R792: C4 + 1e+06 alpha_ob10_seg1[2] <= 1.0000160631954084e+06
 R793: C3 + 1e+06 alpha_ob10_seg1[3] <= 1.0000262405002755e+06
 R794: C5 + 1e+06 alpha_ob10_seg1[3] <= 1.0000262405002755e+06
 R795: alpha_ob10_seg1[0] + alpha_ob10_seg1[1] + alpha_ob10_seg1[2]
   + alpha_ob10_seg1[3] >= 1
 R796: - C2 + 1e+06 alpha_ob11_seg1[0] <= 999981.386799801
 R797: - C4 + 1e+06 alpha_ob11_seg1[0] <= 999981.386799801
 R798: - C3 + 1e+06 alpha_ob11_seg1[1] <= 999971.2095013268
 R799: - C5 + 1e+06 alpha_ob11_seg1[1] <= 999971.2095013268
 R800: C2 + 1e+06 alpha_ob11_seg1[2] <= 1.0000152131998009e+06
 R801: C4 + 1e+06 alpha_ob11_seg1[2] <= 1.0000152131998009e+06
 R802: C3 + 1e+06 alpha_ob11_seg1[3] <= 1.0000262405013267e+06
 R803: C5 + 1e+06 alpha_ob11_seg1[3] <= 1.0000262405013267e+06
 R804: alpha_ob11_seg1[0] + alpha_ob11_seg1[1] + alpha_ob11_seg1[2]
   + alpha_ob11_seg1[3] >= 1
 R805: - C2 + 1e+06 alpha_ob12_seg1[0] <= 999982.2368022958
 R806: - C4 + 1e+06 alpha_ob12_seg1[0] <= 999982.2368022958
 R807: - C3 + 1e+06 alpha_ob12_seg1[1] <= 999970.1094997245
 R808: - C5 + 1e+06 alpha_ob12_seg1[1] <= 999970.1094997245
 R809: C2 + 1e+06 alpha_ob12_seg1[2] <= 1.0000152132022957e+06
 R810: C4 + 1e+06 alpha_ob12_seg1[2] <= 1.0000152132022957e+06
 R811: C3 + 1e+06 alpha_ob12_seg1[3] <= 1.0000262404997245e+06
 R812: C5 + 1e+06 alpha_ob12_seg1[3] <= 1.0000262404997245e+06
 R813: alpha_ob12_seg1[0] + alpha_ob12_seg1[1] + alpha_ob12_seg1[2]
   + alpha_ob12_seg1[3] >= 1
 R814: - C2 + 1e+06 alpha_ob13_seg1[0] <= 999982.236799801
 R815: - C4 + 1e+06 alpha_ob13_seg1[0] <= 999982.236799801
 R816: - C3 + 1e+06 alpha_ob13_seg1[1] <= 999970.1095291895
 R817: - C5 + 1e+06 alpha_ob13_seg1[1] <= 999970.1095291895
 R818: C2 + 1e+06 alpha_ob13_seg1[2] <= 999993.3631998011
 R819: C4 + 1e+06 alpha_ob13_seg1[2] <= 999993.3631998011
 R820: C3 + 1e+06 alpha_ob13_seg1[3] <= 1.0000273405291894e+06
 R821: C5 + 1e+06 alpha_ob13_seg1[3] <= 1.0000273405291894e+06
 R822: alpha_ob13_seg1[0] + alpha_ob13_seg1[1] + alpha_ob13_seg1[2]
   + alpha_ob13_seg1[3] >= 1
 R823: - C2 + 1e+06 alpha_ob14_seg1[0] <= 1.0000040868238758e+06
 R824: - C4 + 1e+06 alpha_ob14_seg1[0] <= 1.0000040868238758e+06
 R825: - C3 + 1e+06 alpha_ob14_seg1[1] <= 999957.2594997246
 R826: - C5 + 1e+06 alpha_ob14_seg1[1] <= 999957.2594997246
 R827: C2 + 1e+06 alpha_ob14_seg1[2] <= 999993.3632238759
 R828: C4 + 1e+06 alpha_ob14_seg1[2] <= 999993.3632238759
 R829: C3 + 1e+06 alpha_ob14_seg1[3] <= 1.0000273404997246e+06
 R830: C5 + 1e+06 alpha_ob14_seg1[3] <= 1.0000273404997246e+06
 R831: alpha_ob14_seg1[0] + alpha_ob14_seg1[1] + alpha_ob14_seg1[2]
   + alpha_ob14_seg1[3] >= 1
 R832: - C2 + 1e+06 alpha_ob15_seg1[0] <= 999981.6050802565
 R833: - C4 + 1e+06 alpha_ob15_seg1[0] <= 999981.6050802565
 R834: - C3 + 1e+06 alpha_ob15_seg1[1] <= 999972.8942002755
 R835: - C5 + 1e+06 alpha_ob15_seg1[1] <= 999972.8942002755
 R836: C2 + 1e+06 alpha_ob15_seg1[2] <= 1.0000158448802566e+06
 R837: C4 + 1e+06 alpha_ob15_seg1[2] <= 1.0000158448802566e+06
 R838: C3 + 1e+06 alpha_ob15_seg1[3] <= 1.0000139558002756e+06
 R839: C5 + 1e+06 alpha_ob15_seg1[3] <= 1.0000139558002756e+06
 R840: alpha_ob15_seg1[0] + alpha_ob15_seg1[1] + alpha_ob15_seg1[2]
   + alpha_ob15_seg1[3] >= 1
 R841: - C2 + 1e+06 alpha_ob16_seg1[0] <= 999981.605149801
 R842: - C4 + 1e+06 alpha_ob16_seg1[0] <= 999981.605149801
 R843: - C3 + 1e+06 alpha_ob16_seg1[1] <= 999983.4942391404
 R844: - C5 + 1e+06 alpha_ob16_seg1[1] <= 999983.4942391404
 R845: C2 + 1e+06 alpha_ob16_seg1[2] <= 999986.4948498011
 R846: C4 + 1e+06 alpha_ob16_seg1[2] <= 999986.4948498011
 R847: C3 + 1e+06 alpha_ob16_seg1[3] <= 1.0000139558391405e+06
 R848: C5 + 1e+06 alpha_ob16_seg1[3] <= 1.0000139558391405e+06
 R849: alpha_ob16_seg1[0] + alpha_ob16_seg1[1] + alpha_ob16_seg1[2]
   + alpha_ob16_seg1[3] >= 1
 R850: - C2 + 1e+06 alpha_ob17_seg1[0] <= 999997.6803502565
 R851: - C4 + 1e+06 alpha_ob17_seg1[0] <= 999997.6803502565
 R852: - C3 + 1e+06 alpha_ob17_seg1[1] <= 999972.7268002755
 R853: - C5 + 1e+06 alpha_ob17_seg1[1] <= 999972.7268002755
 R854: C2 + 1e+06 alpha_ob17_seg1[2] <= 999999.7696102565
 R855: C4 + 1e+06 alpha_ob17_seg1[2] <= 999999.7696102565
 R856: C3 + 1e+06 alpha_ob17_seg1[3] <= 1.0000141232002755e+06
 R857: C5 + 1e+06 alpha_ob17_seg1[3] <= 1.0000141232002755e+06
 R858: alpha_ob17_seg1[0] + alpha_ob17_seg1[1] + alpha_ob17_seg1[2]
   + alpha_ob17_seg1[3] >= 1
 R859: - C2 + 1e+06 alpha_ob18_seg1[0] <= 1.0000038341502565e+06
 R860: - C4 + 1e+06 alpha_ob18_seg1[0] <= 1.0000038341502565e+06
 R861: - C3 + 1e+06 alpha_ob18_seg1[1] <= 999972.6012002755
 R862: - C5 + 1e+06 alpha_ob18_seg1[1] <= 999972.6012002755
 R863: C2 + 1e+06 alpha_ob18_seg1[2] <= 999993.6158102566
 R864: C4 + 1e+06 alpha_ob18_seg1[2] <= 999993.6158102566
 R865: C3 + 1e+06 alpha_ob18_seg1[3] <= 1.0000142488002755e+06
 R866: C5 + 1e+06 alpha_ob18_seg1[3] <= 1.0000142488002755e+06
 R867: alpha_ob18_seg1[0] + alpha_ob18_seg1[1] + alpha_ob18_seg1[2]
   + alpha_ob18_seg1[3] >= 1
 R868: - C2 + 1e+06 alpha_ob19_seg1[0] <= 1.0000162874063333e+06
 R869: - C4 + 1e+06 alpha_ob19_seg1[0] <= 1.0000162874063333e+06
 R870: - C3 + 1e+06 alpha_ob19_seg1[1] <= 999982.9439602756
 R871: - C5 + 1e+06 alpha_ob19_seg1[1] <= 999982.9439602756
 R872: C2 + 1e+06 alpha_ob19_seg1[2] <= 999981.1624063333
 R873: C4 + 1e+06 alpha_ob19_seg1[2] <= 999981.1624063333
 R874: C3 + 1e+06 alpha_ob19_seg1[3] <= 999963.6560402757
 R875: C5 + 1e+06 alpha_ob19_seg1[3] <= 999963.6560402757
 R876: alpha_ob19_seg1[0] + alpha_ob19_seg1[1] + alpha_ob19_seg1[2]
   + alpha_ob19_seg1[3] >= 1
 R877: - C2 + 1e+06 alpha_ob20_seg1[0] <= 1.0000162874998011e+06
 R878: - C4 + 1e+06 alpha_ob20_seg1[0] <= 1.0000162874998011e+06
 R879: - C3 + 1e+06 alpha_ob20_seg1[1] <= 1.000033794079276e+06
 R880: - C5 + 1e+06 alpha_ob20_seg1[1] <= 1.000033794079276e+06
 R881: C2 + 1e+06 alpha_ob20_seg1[2] <= 999921.562499801
 R882: C4 + 1e+06 alpha_ob20_seg1[2] <= 999921.562499801
 R883: C3 + 1e+06 alpha_ob20_seg1[3] <= 999963.656079276
 R884: C5 + 1e+06 alpha_ob20_seg1[3] <= 999963.656079276
 R885: alpha_ob20_seg1[0] + alpha_ob20_seg1[1] + alpha_ob20_seg1[2]
   + alpha_ob20_seg1[3] >= 1
 R886: - C2 + 1e+06 alpha_ob21_seg1[0] <= 1.0000066737e+06
 R887: - C4 + 1e+06 alpha_ob21_seg1[0] <= 1.0000066737e+06
 R888: - C3 + 1e+06 alpha_ob21_seg1[1] <= 999952.1958
 R889: - C5 + 1e+06 alpha_ob21_seg1[1] <= 999952.1958
 R890: C2 + 1e+06 alpha_ob21_seg1[2] <= 999948.4263000001
 R891: C4 + 1e+06 alpha_ob21_seg1[2] <= 999948.4263000001
 R892: C3 + 1e+06 alpha_ob21_seg1[3] <= 1.0000452542e+06
 R893: C5 + 1e+06 alpha_ob21_seg1[3] <= 1.0000452542e+06
 R894: alpha_ob21_seg1[0] + alpha_ob21_seg1[1] + alpha_ob21_seg1[2]
   + alpha_ob21_seg1[3] >= 1
 R895: - C2 + 1e+06 alpha_ob22_seg1[0] <= 1.0000162874977043e+06
 R896: - C4 + 1e+06 alpha_ob22_seg1[0] <= 1.0000162874977043e+06
 R897: - C3 + 1e+06 alpha_ob22_seg1[1] <= 1.0000337940002755e+06
 R898: - C5 + 1e+06 alpha_ob22_seg1[1] <= 1.0000337940002755e+06
 R899: C2 + 1e+06 alpha_ob22_seg1[2] <= 999981.1624977043
 R900: C4 + 1e+06 alpha_ob22_seg1[2] <= 999981.1624977043
 R901: C3 + 1e+06 alpha_ob22_seg1[3] <= 999962.5560002755
 R902: C5 + 1e+06 alpha_ob22_seg1[3] <= 999962.5560002755
 R903: alpha_ob22_seg1[0] + alpha_ob22_seg1[1] + alpha_ob22_seg1[2]
   + alpha_ob22_seg1[3] >= 1
 R904: - C2 + 1e+06 alpha_ob23_seg1[0] <= 999980.937509
 R905: - C4 + 1e+06 alpha_ob23_seg1[0] <= 999980.937509
 R906: - C3 + 1e+06 alpha_ob23_seg1[1] <= 1.000034894e+06
 R907: - C5 + 1e+06 alpha_ob23_seg1[1] <= 1.000034894e+06
 R908: C2 + 1e+06 alpha_ob23_seg1[2] <= 999981.162491
 R909: C4 + 1e+06 alpha_ob23_seg1[2] <= 999981.162491
 R910: C3 + 1e+06 alpha_ob23_seg1[3] <= 999962.556
 R911: C5 + 1e+06 alpha_ob23_seg1[3] <= 999962.556
 R912: alpha_ob23_seg1[0] + alpha_ob23_seg1[1] + alpha_ob23_seg1[2]
   + alpha_ob23_seg1[3] >= 1
 R913: - C2 + 1e+06 alpha_ob24_seg1[0] <= 1.0000070091e+06
 R914: - C4 + 1e+06 alpha_ob24_seg1[0] <= 1.0000070091e+06
 R915: - C3 + 1e+06 alpha_ob24_seg1[1] <= 1.0000598843e+06
 R916: - C5 + 1e+06 alpha_ob24_seg1[1] <= 1.0000598843e+06
 R917: C2 + 1e+06 alpha_ob24_seg1[2] <= 999965.0909
 R918: C4 + 1e+06 alpha_ob24_seg1[2] <= 999965.0909
 R919: C3 + 1e+06 alpha_ob24_seg1[3] <= 999937.5657
 R920: C5 + 1e+06 alpha_ob24_seg1[3] <= 999937.5657
 R921: alpha_ob24_seg1[0] + alpha_ob24_seg1[1] + alpha_ob24_seg1[2]
   + alpha_ob24_seg1[3] >= 1
 R922: - C2 + 1e+06 alpha_ob25_seg1[0] <= 1.000006992114031e+06
 R923: - C4 + 1e+06 alpha_ob25_seg1[0] <= 1.000006992114031e+06
 R924: - C3 + 1e+06 alpha_ob25_seg1[1] <= 1.0000588068002755e+06
 R925: - C5 + 1e+06 alpha_ob25_seg1[1] <= 1.0000588068002755e+06
 R926: C2 + 1e+06 alpha_ob25_seg1[2] <= 999990.457874031
 R927: C4 + 1e+06 alpha_ob25_seg1[2] <= 999990.457874031
 R928: C3 + 1e+06 alpha_ob25_seg1[3] <= 999935.5432002755
 R929: C5 + 1e+06 alpha_ob25_seg1[3] <= 999935.5432002755
 R930: alpha_ob25_seg1[0] + alpha_ob25_seg1[1] + alpha_ob25_seg1[2]
   + alpha_ob25_seg1[3] >= 1
 R931: - C2 + 1e+06 alpha_ob26_seg1[0] <= 1.0000033775063743e+06
 R932: - C4 + 1e+06 alpha_ob26_seg1[0] <= 1.0000033775063743e+06
 R933: - C3 + 1e+06 alpha_ob26_seg1[1] <= 1.0000348666997246e+06
 R934: - C5 + 1e+06 alpha_ob26_seg1[1] <= 1.0000348666997246e+06
 R935: C2 + 1e+06 alpha_ob26_seg1[2] <= 999994.0725863742
 R936: C4 + 1e+06 alpha_ob26_seg1[2] <= 999994.0725863742
 R937: C3 + 1e+06 alpha_ob26_seg1[3] <= 999937.4832997246
 R938: C5 + 1e+06 alpha_ob26_seg1[3] <= 999937.4832997246
 R939: alpha_ob26_seg1[0] + alpha_ob26_seg1[1] + alpha_ob26_seg1[2]
   + alpha_ob26_seg1[3] >= 1
 R940: - C2 + 1e+06 alpha_ob27_seg1[0] <= 999990.2670277582
 R941: - C4 + 1e+06 alpha_ob27_seg1[0] <= 999990.2670277582
 R942: - C3 + 1e+06 alpha_ob27_seg1[1] <= 1.0000347702002756e+06
 R943: - C5 + 1e+06 alpha_ob27_seg1[1] <= 1.0000347702002756e+06
 R944: C2 + 1e+06 alpha_ob27_seg1[2] <= 1.0000071828877581e+06
 R945: C4 + 1e+06 alpha_ob27_seg1[2] <= 1.0000071828877581e+06
 R946: C3 + 1e+06 alpha_ob27_seg1[3] <= 999939.8298002756
 R947: C5 + 1e+06 alpha_ob27_seg1[3] <= 999939.8298002756
 R948: alpha_ob27_seg1[0] + alpha_ob27_seg1[1] + alpha_ob27_seg1[2]
   + alpha_ob27_seg1[3] >= 1
 R949: - C2 + 1e+06 alpha_ob28_seg1[0] <= 999990.267069801
 R950: - C4 + 1e+06 alpha_ob28_seg1[0] <= 999990.267069801
 R951: - C3 + 1e+06 alpha_ob28_seg1[1] <= 1.0000576202069657e+06
 R952: - C5 + 1e+06 alpha_ob28_seg1[1] <= 1.0000576202069657e+06
 R953: C2 + 1e+06 alpha_ob28_seg1[2] <= 1.000002082929801e+06
 R954: C4 + 1e+06 alpha_ob28_seg1[2] <= 1.000002082929801e+06
 R955: C3 + 1e+06 alpha_ob28_seg1[3] <= 999939.8298069657
 R956: C5 + 1e+06 alpha_ob28_seg1[3] <= 999939.8298069657
 R957: alpha_ob28_seg1[0] + alpha_ob28_seg1[1] + alpha_ob28_seg1[2]
   + alpha_ob28_seg1[3] >= 1
 R958: - C2 + 1e+06 alpha_ob29_seg1[0] <= 999991.073732
 R959: - C4 + 1e+06 alpha_ob29_seg1[0] <= 999991.073732
 R960: - C3 + 1e+06 alpha_ob29_seg1[1] <= 999952.2125
 R961: - C5 + 1e+06 alpha_ob29_seg1[1] <= 999952.2125
 R962: C2 + 1e+06 alpha_ob29_seg1[2] <= 999990.776268
 R963: C4 + 1e+06 alpha_ob29_seg1[2] <= 999990.776268
 R964: C3 + 1e+06 alpha_ob29_seg1[3] <= 1.0000452375e+06
 R965: C5 + 1e+06 alpha_ob29_seg1[3] <= 1.0000452375e+06
 R966: alpha_ob29_seg1[0] + alpha_ob29_seg1[1] + alpha_ob29_seg1[2]
   + alpha_ob29_seg1[3] >= 1
 R967: - C2 + 1e+06 alpha_ob30_seg1[0] <= 999985.657099801
 R968: - C4 + 1e+06 alpha_ob30_seg1[0] <= 999985.657099801
 R969: - C3 + 1e+06 alpha_ob30_seg1[1] <= 1.0000576202063023e+06
 R970: - C5 + 1e+06 alpha_ob30_seg1[1] <= 1.0000576202063023e+06
 R971: C2 + 1e+06 alpha_ob30_seg1[2] <= 1.000007192899801e+06
 R972: C4 + 1e+06 alpha_ob30_seg1[2] <= 1.000007192899801e+06
 R973: C3 + 1e+06 alpha_ob30_seg1[3] <= 999939.8298063023
 R974: C5 + 1e+06 alpha_ob30_seg1[3] <= 999939.8298063023
 R975: alpha_ob30_seg1[0] + alpha_ob30_seg1[1] + alpha_ob30_seg1[2]
   + alpha_ob30_seg1[3] >= 1
 R976: - C2 + 1e+06 alpha_ob31_seg1[0] <= 999994.771829801
 R977: - C4 + 1e+06 alpha_ob31_seg1[0] <= 999994.771829801
 R978: - C3 + 1e+06 alpha_ob31_seg1[1] <= 1.0000615983063023e+06
 R979: - C5 + 1e+06 alpha_ob31_seg1[1] <= 1.0000615983063023e+06
 R980: C2 + 1e+06 alpha_ob31_seg1[2] <= 999998.078169801
 R981: C4 + 1e+06 alpha_ob31_seg1[2] <= 999998.078169801
 R982: C3 + 1e+06 alpha_ob31_seg1[3] <= 999935.8517063023
 R983: C5 + 1e+06 alpha_ob31_seg1[3] <= 999935.8517063023
 R984: alpha_ob31_seg1[0] + alpha_ob31_seg1[1] + alpha_ob31_seg1[2]
   + alpha_ob31_seg1[3] >= 1
 R985: - C2 + 1e+06 alpha_ob32_seg1[0] <= 999999.3718445294
 R986: - C4 + 1e+06 alpha_ob32_seg1[0] <= 999999.3718445294
 R987: - C3 + 1e+06 alpha_ob32_seg1[1] <= 1.0000527482997245e+06
 R988: - C5 + 1e+06 alpha_ob32_seg1[1] <= 1.0000527482997245e+06
 R989: C2 + 1e+06 alpha_ob32_seg1[2] <= 999998.0781885295
 R990: C4 + 1e+06 alpha_ob32_seg1[2] <= 999998.0781885295
 R991: C3 + 1e+06 alpha_ob32_seg1[3] <= 999935.8516997246
 R992: C5 + 1e+06 alpha_ob32_seg1[3] <= 999935.8516997246
 R993: alpha_ob32_seg1[0] + alpha_ob32_seg1[1] + alpha_ob32_seg1[2]
   + alpha_ob32_seg1[3] >= 1
 R994: - C2 + 1e+06 alpha_ob33_seg1[0] <= 999999.371829801
 R995: - C4 + 1e+06 alpha_ob33_seg1[0] <= 999999.371829801
 R996: - C3 + 1e+06 alpha_ob33_seg1[1] <= 1.0000527483059706e+06
 R997: - C5 + 1e+06 alpha_ob33_seg1[1] <= 1.0000527483059706e+06
 R998: C2 + 1e+06 alpha_ob33_seg1[2] <= 999993.7281698009
 R999: C4 + 1e+06 alpha_ob33_seg1[2] <= 999993.7281698009
 R1000: C3 + 1e+06 alpha_ob33_seg1[3] <= 999944.7017059706
 R1001: C5 + 1e+06 alpha_ob33_seg1[3] <= 999944.7017059706
 R1002: alpha_ob33_seg1[0] + alpha_ob33_seg1[1] + alpha_ob33_seg1[2]
   + alpha_ob33_seg1[3] >= 1
 R1003: - C2 + 1e+06 alpha_ob34_seg1[0] <= 1.0000016470956831e+06
 R1004: - C4 + 1e+06 alpha_ob34_seg1[0] <= 1.0000016470956831e+06
 R1005: - C3 + 1e+06 alpha_ob34_seg1[1] <= 1.0000598666742218e+06
 R1006: - C5 + 1e+06 alpha_ob34_seg1[1] <= 1.0000598666742218e+06
 R1007: C2 + 1e+06 alpha_ob34_seg1[2] <= 999994.0767956832
 R1008: C4 + 1e+06 alpha_ob34_seg1[2] <= 999994.0767956832
 R1009: C3 + 1e+06 alpha_ob34_seg1[3] <= 999937.4792742218
 R1010: C5 + 1e+06 alpha_ob34_seg1[3] <= 999937.4792742218
 R1011: alpha_ob34_seg1[0] + alpha_ob34_seg1[1] + alpha_ob34_seg1[2]
   + alpha_ob34_seg1[3] >= 1
 R1012: - C2 + 1e+06 alpha_ob35_seg1[0] <= 999999.3718601406
 R1013: - C4 + 1e+06 alpha_ob35_seg1[0] <= 999999.3718601406
 R1014: - C3 + 1e+06 alpha_ob35_seg1[1] <= 1.0000353982997246e+06
 R1015: - C5 + 1e+06 alpha_ob35_seg1[1] <= 1.0000353982997246e+06
 R1016: C2 + 1e+06 alpha_ob35_seg1[2] <= 999998.0782041405
 R1017: C4 + 1e+06 alpha_ob35_seg1[2] <= 999998.0782041405
 R1018: C3 + 1e+06 alpha_ob35_seg1[3] <= 999944.7016997245
 R1019: C5 + 1e+06 alpha_ob35_seg1[3] <= 999944.7016997245
 R1020: alpha_ob35_seg1[0] + alpha_ob35_seg1[1] + alpha_ob35_seg1[2]
   + alpha_ob35_seg1[3] >= 1
 R1021: - C2 + 1e+06 alpha_ob36_seg1[0] <= 999980.9374508709
 R1022: - C4 + 1e+06 alpha_ob36_seg1[0] <= 999980.9374508709
 R1023: - C3 + 1e+06 alpha_ob36_seg1[1] <= 1.0000348940002755e+06
 R1024: - C5 + 1e+06 alpha_ob36_seg1[1] <= 1.0000348940002755e+06
 R1025: C2 + 1e+06 alpha_ob36_seg1[2] <= 1.0000165124508708e+06
 R1026: C4 + 1e+06 alpha_ob36_seg1[2] <= 1.0000165124508708e+06
 R1027: C3 + 1e+06 alpha_ob36_seg1[3] <= 999935.9560002756
 R1028: C5 + 1e+06 alpha_ob36_seg1[3] <= 999935.9560002756
 R1029: alpha_ob36_seg1[0] + alpha_ob36_seg1[1] + alpha_ob36_seg1[2]
   + alpha_ob36_seg1[3] >= 1
 R1030: - C2 + 1e+06 alpha_ob37_seg1[0] <= 999980.937499801
 R1031: - C4 + 1e+06 alpha_ob37_seg1[0] <= 999980.937499801
 R1032: - C3 + 1e+06 alpha_ob37_seg1[1] <= 1.0000614940072973e+06
 R1033: - C5 + 1e+06 alpha_ob37_seg1[1] <= 1.0000614940072973e+06
 R1034: C2 + 1e+06 alpha_ob37_seg1[2] <= 1.000011162499801e+06
 R1035: C4 + 1e+06 alpha_ob37_seg1[2] <= 1.000011162499801e+06
 R1036: C3 + 1e+06 alpha_ob37_seg1[3] <= 999935.9560072974
 R1037: C5 + 1e+06 alpha_ob37_seg1[3] <= 999935.9560072974
 R1038: alpha_ob37_seg1[0] + alpha_ob37_seg1[1] + alpha_ob37_seg1[2]
   + alpha_ob37_seg1[3] >= 1
 R1039: - C2 + 1e+06 alpha_ob38_seg1[0] <= 999956.8394000001
 R1040: - C4 + 1e+06 alpha_ob38_seg1[0] <= 999956.8394000001
 R1041: - C3 + 1e+06 alpha_ob38_seg1[1] <= 999952.2463
 R1042: - C5 + 1e+06 alpha_ob38_seg1[1] <= 999952.2463
 R1043: C2 + 1e+06 alpha_ob38_seg1[2] <= 1.0000097606e+06
 R1044: C4 + 1e+06 alpha_ob38_seg1[2] <= 1.0000097606e+06
 R1045: C3 + 1e+06 alpha_ob38_seg1[3] <= 1.0000452037e+06
 R1046: C5 + 1e+06 alpha_ob38_seg1[3] <= 1.0000452037e+06
 R1047: alpha_ob38_seg1[0] + alpha_ob38_seg1[1] + alpha_ob38_seg1[2]
   + alpha_ob38_seg1[3] >= 1
 R1048: - C2 + 1e+06 alpha_ob39_seg1[0] <= 999964.9321250001
 R1049: - C4 + 1e+06 alpha_ob39_seg1[0] <= 999964.9321250001
 R1050: - C3 + 1e+06 alpha_ob39_seg1[1] <= 1.0000141893e+06
 R1051: - C5 + 1e+06 alpha_ob39_seg1[1] <= 1.0000141893e+06
 R1052: C2 + 1e+06 alpha_ob39_seg1[2] <= 1.000025197325e+06
 R1053: C4 + 1e+06 alpha_ob39_seg1[2] <= 1.000025197325e+06
 R1054: C3 + 1e+06 alpha_ob39_seg1[3] <= 999983.2607
 R1055: C5 + 1e+06 alpha_ob39_seg1[3] <= 999983.2607
 R1056: alpha_ob39_seg1[0] + alpha_ob39_seg1[1] + alpha_ob39_seg1[2]
   + alpha_ob39_seg1[3] >= 1
 R1057: - C2 + 1e+06 alpha_ob40_seg1[0] <= 999964.922208629
 R1058: - C4 + 1e+06 alpha_ob40_seg1[0] <= 999964.922208629
 R1059: - C3 + 1e+06 alpha_ob40_seg1[1] <= 1.0000141893002756e+06
 R1060: - C5 + 1e+06 alpha_ob40_seg1[1] <= 1.0000141893002756e+06
 R1061: C2 + 1e+06 alpha_ob40_seg1[2] <= 1.000032527608629e+06
 R1062: C4 + 1e+06 alpha_ob40_seg1[2] <= 1.000032527608629e+06
 R1063: C3 + 1e+06 alpha_ob40_seg1[3] <= 999933.6607002757
 R1064: C5 + 1e+06 alpha_ob40_seg1[3] <= 999933.6607002757
 R1065: alpha_ob40_seg1[0] + alpha_ob40_seg1[1] + alpha_ob40_seg1[2]
   + alpha_ob40_seg1[3] >= 1
 R1066: - C2 + 1e+06 alpha_ob41_seg1[0] <= 999972.166285402
 R1067: - C4 + 1e+06 alpha_ob41_seg1[0] <= 999972.166285402
 R1068: - C3 + 1e+06 alpha_ob41_seg1[1] <= 1.0000141604997247e+06
 R1069: - C5 + 1e+06 alpha_ob41_seg1[1] <= 1.0000141604997247e+06
 R1070: C2 + 1e+06 alpha_ob41_seg1[2] <= 1.000025283885402e+06
 R1071: C4 + 1e+06 alpha_ob41_seg1[2] <= 1.000025283885402e+06
 R1072: C3 + 1e+06 alpha_ob41_seg1[3] <= 999936.9394997247
 R1073: C5 + 1e+06 alpha_ob41_seg1[3] <= 999936.9394997247
 R1074: alpha_ob41_seg1[0] + alpha_ob41_seg1[1] + alpha_ob41_seg1[2]
   + alpha_ob41_seg1[3] >= 1
 R1075: - C2 + 1e+06 alpha_ob42_seg1[0] <= 999931.4945
 R1076: - C4 + 1e+06 alpha_ob42_seg1[0] <= 999931.4945
 R1077: - C3 + 1e+06 alpha_ob42_seg1[1] <= 1.0000158794e+06
 R1078: - C5 + 1e+06 alpha_ob42_seg1[1] <= 1.0000158794e+06
 R1079: C2 + 1e+06 alpha_ob42_seg1[2] <= 1.0000378554999999e+06
 R1080: C4 + 1e+06 alpha_ob42_seg1[2] <= 1.0000378554999999e+06
 R1081: C3 + 1e+06 alpha_ob42_seg1[3] <= 999981.5706
 R1082: C5 + 1e+06 alpha_ob42_seg1[3] <= 999981.5706
 R1083: alpha_ob42_seg1[0] + alpha_ob42_seg1[1] + alpha_ob42_seg1[2]
   + alpha_ob42_seg1[3] >= 1
 R1084: - C2 + 1e+06 alpha_ob43_seg1[0] <= 999916.6145
 R1085: - C4 + 1e+06 alpha_ob43_seg1[0] <= 999916.6145
 R1086: - C3 + 1e+06 alpha_ob43_seg1[1] <= 1.0000160176e+06
 R1087: - C5 + 1e+06 alpha_ob43_seg1[1] <= 1.0000160176e+06
 R1088: C2 + 1e+06 alpha_ob43_seg1[2] <= 1.0000707355e+06
 R1089: C4 + 1e+06 alpha_ob43_seg1[2] <= 1.0000707355e+06
 R1090: C3 + 1e+06 alpha_ob43_seg1[3] <= 999981.4324
 R1091: C5 + 1e+06 alpha_ob43_seg1[3] <= 999981.4324
 R1092: alpha_ob43_seg1[0] + alpha_ob43_seg1[1] + alpha_ob43_seg1[2]
   + alpha_ob43_seg1[3] >= 1
 R1093: - C2 + 1e+06 alpha_ob44_seg1[0] <= 999916.6143035784
 R1094: - C4 + 1e+06 alpha_ob44_seg1[0] <= 999916.6143035784
 R1095: - C3 + 1e+06 alpha_ob44_seg1[1] <= 1.0000160176002757e+06
 R1096: - C5 + 1e+06 alpha_ob44_seg1[1] <= 1.0000160176002757e+06
 R1097: C2 + 1e+06 alpha_ob44_seg1[2] <= 1.0000808355035784e+06
 R1098: C4 + 1e+06 alpha_ob44_seg1[2] <= 1.0000808355035784e+06
 R1099: C3 + 1e+06 alpha_ob44_seg1[3] <= 999929.0824002756
 R1100: C5 + 1e+06 alpha_ob44_seg1[3] <= 999929.0824002756
 R1101: alpha_ob44_seg1[0] + alpha_ob44_seg1[1] + alpha_ob44_seg1[2]
   + alpha_ob44_seg1[3] >= 1
 R1102: - C2 + 1e+06 alpha_ob45_seg1[0] <= 999980.9375348955
 R1103: - C4 + 1e+06 alpha_ob45_seg1[0] <= 999980.9375348955
 R1104: - C3 + 1e+06 alpha_ob45_seg1[1] <= 1.0000160439997246e+06
 R1105: - C5 + 1e+06 alpha_ob45_seg1[1] <= 1.0000160439997246e+06
 R1106: C2 + 1e+06 alpha_ob45_seg1[2] <= 1.0000165125348954e+06
 R1107: C4 + 1e+06 alpha_ob45_seg1[2] <= 1.0000165125348954e+06
 R1108: C3 + 1e+06 alpha_ob45_seg1[3] <= 999962.5559997245
 R1109: C5 + 1e+06 alpha_ob45_seg1[3] <= 999962.5559997245
 R1110: alpha_ob45_seg1[0] + alpha_ob45_seg1[1] + alpha_ob45_seg1[2]
   + alpha_ob45_seg1[3] >= 1
 R1111: - C2 + 1e+06 alpha_ob46_seg1[0] <= 999956.839399801
 R1112: - C4 + 1e+06 alpha_ob46_seg1[0] <= 999956.839399801
 R1113: - C3 + 1e+06 alpha_ob46_seg1[1] <= 999925.3963434525
 R1114: - C5 + 1e+06 alpha_ob46_seg1[1] <= 999925.3963434525
 R1115: C2 + 1e+06 alpha_ob46_seg1[2] <= 1.0000080105998011e+06
 R1116: C4 + 1e+06 alpha_ob46_seg1[2] <= 1.0000080105998011e+06
 R1117: C3 + 1e+06 alpha_ob46_seg1[3] <= 1.0000720537434525e+06
 R1118: C5 + 1e+06 alpha_ob46_seg1[3] <= 1.0000720537434525e+06
 R1119: alpha_ob46_seg1[0] + alpha_ob46_seg1[1] + alpha_ob46_seg1[2]
   + alpha_ob46_seg1[3] >= 1
 R1120: - C2 + 1e+06 alpha_ob47_seg1[0] <= 999980.9374998009
 R1121: - C4 + 1e+06 alpha_ob47_seg1[0] <= 999980.9374998009
 R1122: - C3 + 1e+06 alpha_ob47_seg1[1] <= 1.0000160440053071e+06
 R1123: - C5 + 1e+06 alpha_ob47_seg1[1] <= 1.0000160440053071e+06
 R1124: C2 + 1e+06 alpha_ob47_seg1[2] <= 1.000012662499801e+06
 R1125: C4 + 1e+06 alpha_ob47_seg1[2] <= 1.000012662499801e+06
 R1126: C3 + 1e+06 alpha_ob47_seg1[3] <= 999981.4060053072
 R1127: C5 + 1e+06 alpha_ob47_seg1[3] <= 999981.4060053072
 R1128: alpha_ob47_seg1[0] + alpha_ob47_seg1[1] + alpha_ob47_seg1[2]
   + alpha_ob47_seg1[3] >= 1
 R1129: - C2 + 1e+06 alpha_ob48_seg1[0] <= 999983.6053459151
 R1130: - C4 + 1e+06 alpha_ob48_seg1[0] <= 999983.6053459151
 R1131: - C3 + 1e+06 alpha_ob48_seg1[1] <= 999987.7722497246
 R1132: - C5 + 1e+06 alpha_ob48_seg1[1] <= 999987.7722497246
 R1133: C2 + 1e+06 alpha_ob48_seg1[2] <= 1.000013844745915e+06
 R1134: C4 + 1e+06 alpha_ob48_seg1[2] <= 1.000013844745915e+06
 R1135: C3 + 1e+06 alpha_ob48_seg1[3] <= 999984.8277497246
 R1136: C5 + 1e+06 alpha_ob48_seg1[3] <= 999984.8277497246
 R1137: alpha_ob48_seg1[0] + alpha_ob48_seg1[1] + alpha_ob48_seg1[2]
   + alpha_ob48_seg1[3] >= 1
 R1138: - C2 + 1e+06 alpha_ob49_seg1[0] <= 999958.8717898987
 R1139: - C4 + 1e+06 alpha_ob49_seg1[0] <= 999958.8717898987
 R1140: - C3 + 1e+06 alpha_ob49_seg1[1] <= 999987.7837602755
 R1141: - C5 + 1e+06 alpha_ob49_seg1[1] <= 999987.7837602755
 R1142: C2 + 1e+06 alpha_ob49_seg1[2] <= 1.0000385781898986e+06
 R1143: C4 + 1e+06 alpha_ob49_seg1[2] <= 1.0000385781898986e+06
 R1144: C3 + 1e+06 alpha_ob49_seg1[3] <= 1.0000043162402755e+06
 R1145: C5 + 1e+06 alpha_ob49_seg1[3] <= 1.0000043162402755e+06
 R1146: alpha_ob49_seg1[0] + alpha_ob49_seg1[1] + alpha_ob49_seg1[2]
   + alpha_ob49_seg1[3] >= 1
 R1147: - C2 + 1e+06 alpha_ob50_seg1[0] <= 999939.2718
 R1148: - C4 + 1e+06 alpha_ob50_seg1[0] <= 999939.2718
 R1149: - C3 + 1e+06 alpha_ob50_seg1[1] <= 999993.13376
 R1150: - C5 + 1e+06 alpha_ob50_seg1[1] <= 999993.13376
 R1151: C2 + 1e+06 alpha_ob50_seg1[2] <= 1.0000385782e+06
 R1152: C4 + 1e+06 alpha_ob50_seg1[2] <= 1.0000385782e+06
 R1153: C3 + 1e+06 alpha_ob50_seg1[3] <= 1.00000431624e+06
 R1154: C5 + 1e+06 alpha_ob50_seg1[3] <= 1.00000431624e+06
 R1155: alpha_ob50_seg1[0] + alpha_ob50_seg1[1] + alpha_ob50_seg1[2]
   + alpha_ob50_seg1[3] >= 1
 R1156: - C2 + 1e+06 alpha_ob51_seg1[0] <= 999939.2718101013
 R1157: - C4 + 1e+06 alpha_ob51_seg1[0] <= 999939.2718101013
 R1158: - C3 + 1e+06 alpha_ob51_seg1[1] <= 999987.7837597246
 R1159: - C5 + 1e+06 alpha_ob51_seg1[1] <= 999987.7837597246
 R1160: C2 + 1e+06 alpha_ob51_seg1[2] <= 1.0000581782101013e+06
 R1161: C4 + 1e+06 alpha_ob51_seg1[2] <= 1.0000581782101013e+06
 R1162: C3 + 1e+06 alpha_ob51_seg1[3] <= 1.0000043162397245e+06
 R1163: C5 + 1e+06 alpha_ob51_seg1[3] <= 1.0000043162397245e+06
 R1164: alpha_ob51_seg1[0] + alpha_ob51_seg1[1] + alpha_ob51_seg1[2]
   + alpha_ob51_seg1[3] >= 1
 R1165: - C2 + 1e+06 alpha_ob52_seg1[0] <= 999939.271799801
 R1166: - C4 + 1e+06 alpha_ob52_seg1[0] <= 999939.271799801
 R1167: - C3 + 1e+06 alpha_ob52_seg1[1] <= 999987.7838262042
 R1168: - C5 + 1e+06 alpha_ob52_seg1[1] <= 999987.7838262042
 R1169: C2 + 1e+06 alpha_ob52_seg1[2] <= 1.000038578199801e+06
 R1170: C4 + 1e+06 alpha_ob52_seg1[2] <= 1.000038578199801e+06
 R1171: C3 + 1e+06 alpha_ob52_seg1[3] <= 1.0000096662262043e+06
 R1172: C5 + 1e+06 alpha_ob52_seg1[3] <= 1.0000096662262043e+06
 R1173: alpha_ob52_seg1[0] + alpha_ob52_seg1[1] + alpha_ob52_seg1[2]
   + alpha_ob52_seg1[3] >= 1
 R1174: - C2 + 1e+06 alpha_ob53_seg1[0] <= 999926.0708
 R1175: - C4 + 1e+06 alpha_ob53_seg1[0] <= 999926.0708
 R1176: - C3 + 1e+06 alpha_ob53_seg1[1] <= 999983.7196
 R1177: - C5 + 1e+06 alpha_ob53_seg1[1] <= 999983.7196
 R1178: C2 + 1e+06 alpha_ob53_seg1[2] <= 1.0000250292e+06
 R1179: C4 + 1e+06 alpha_ob53_seg1[2] <= 1.0000250292e+06
 R1180: C3 + 1e+06 alpha_ob53_seg1[3] <= 1.0000137304e+06
 R1181: C5 + 1e+06 alpha_ob53_seg1[3] <= 1.0000137304e+06
 R1182: alpha_ob53_seg1[0] + alpha_ob53_seg1[1] + alpha_ob53_seg1[2]
   + alpha_ob53_seg1[3] >= 1
 R1183: - C2 + 1e+06 alpha_ob54_seg1[0] <= 999926.0708293857
 R1184: - C4 + 1e+06 alpha_ob54_seg1[0] <= 999926.0708293857
 R1185: - C3 + 1e+06 alpha_ob54_seg1[1] <= 999967.8695997245
 R1186: - C5 + 1e+06 alpha_ob54_seg1[1] <= 999967.8695997245
 R1187: C2 + 1e+06 alpha_ob54_seg1[2] <= 1.0000713792293856e+06
 R1188: C4 + 1e+06 alpha_ob54_seg1[2] <= 1.0000713792293856e+06
 R1189: C3 + 1e+06 alpha_ob54_seg1[3] <= 1.0000137303997246e+06
 R1190: C5 + 1e+06 alpha_ob54_seg1[3] <= 1.0000137303997246e+06
 R1191: alpha_ob54_seg1[0] + alpha_ob54_seg1[1] + alpha_ob54_seg1[2]
   + alpha_ob54_seg1[3] >= 1
 R1192: - C2 + 1e+06 alpha_ob55_seg1[0] <= 999972.6917508709
 R1193: - C4 + 1e+06 alpha_ob55_seg1[0] <= 999972.6917508709
 R1194: - C3 + 1e+06 alpha_ob55_seg1[1] <= 999957.1644002756
 R1195: - C5 + 1e+06 alpha_ob55_seg1[1] <= 999957.1644002756
 R1196: C2 + 1e+06 alpha_ob55_seg1[2] <= 1.0000247581508709e+06
 R1197: C4 + 1e+06 alpha_ob55_seg1[2] <= 1.0000247581508709e+06
 R1198: C3 + 1e+06 alpha_ob55_seg1[3] <= 1.0000136856002755e+06
 R1199: C5 + 1e+06 alpha_ob55_seg1[3] <= 1.0000136856002755e+06
 R1200: alpha_ob55_seg1[0] + alpha_ob55_seg1[1] + alpha_ob55_seg1[2]
   + alpha_ob55_seg1[3] >= 1
 R1201: - C2 + 1e+06 alpha_ob56_seg1[0] <= 999939.0009674003
 R1202: - C4 + 1e+06 alpha_ob56_seg1[0] <= 999939.0009674003
 R1203: - C3 + 1e+06 alpha_ob56_seg1[1] <= 999961.9516002756
 R1204: - C5 + 1e+06 alpha_ob56_seg1[1] <= 999961.9516002756
 R1205: C2 + 1e+06 alpha_ob56_seg1[2] <= 1.0000584489674004e+06
 R1206: C4 + 1e+06 alpha_ob56_seg1[2] <= 1.0000584489674004e+06
 R1207: C3 + 1e+06 alpha_ob56_seg1[3] <= 1.0000178984002755e+06
 R1208: C5 + 1e+06 alpha_ob56_seg1[3] <= 1.0000178984002755e+06
 R1209: alpha_ob56_seg1[0] + alpha_ob56_seg1[1] + alpha_ob56_seg1[2]
   + alpha_ob56_seg1[3] >= 1
 R1210: - C2 + 1e+06 alpha_ob57_seg1[0] <= 999994.9164998011
 R1211: - C4 + 1e+06 alpha_ob57_seg1[0] <= 999994.9164998011
 R1212: - C3 + 1e+06 alpha_ob57_seg1[1] <= 999925.4284782809
 R1213: - C5 + 1e+06 alpha_ob57_seg1[1] <= 999925.4284782809
 R1214: C2 + 1e+06 alpha_ob57_seg1[2] <= 999943.6834998011
 R1215: C4 + 1e+06 alpha_ob57_seg1[2] <= 999943.6834998011
 R1216: C3 + 1e+06 alpha_ob57_seg1[3] <= 1.0000720216782809e+06
 R1217: C5 + 1e+06 alpha_ob57_seg1[3] <= 1.0000720216782809e+06
 R1218: alpha_ob57_seg1[0] + alpha_ob57_seg1[1] + alpha_ob57_seg1[2]
   + alpha_ob57_seg1[3] >= 1
 R1219: - C2 + 1e+06 alpha_ob58_seg1[0] <= 999920.7556087238
 R1220: - C4 + 1e+06 alpha_ob58_seg1[0] <= 999920.7556087238
 R1221: - C3 + 1e+06 alpha_ob58_seg1[1] <= 1.0000111845997245e+06
 R1222: - C5 + 1e+06 alpha_ob58_seg1[1] <= 1.0000111845997245e+06
 R1223: C2 + 1e+06 alpha_ob58_seg1[2] <= 1.0000766944087239e+06
 R1224: C4 + 1e+06 alpha_ob58_seg1[2] <= 1.0000766944087239e+06
 R1225: C3 + 1e+06 alpha_ob58_seg1[3] <= 999981.6653997245
 R1226: C5 + 1e+06 alpha_ob58_seg1[3] <= 999981.6653997245
 R1227: alpha_ob58_seg1[0] + alpha_ob58_seg1[1] + alpha_ob58_seg1[2]
   + alpha_ob58_seg1[3] >= 1
 R1228: - C2 + 1e+06 alpha_ob59_seg1[0] <= 999911.9056000001
 R1229: - C4 + 1e+06 alpha_ob59_seg1[0] <= 999911.9056000001
 R1230: - C3 + 1e+06 alpha_ob59_seg1[1] <= 1.0000111846e+06
 R1231: - C5 + 1e+06 alpha_ob59_seg1[1] <= 1.0000111846e+06
 R1232: C2 + 1e+06 alpha_ob59_seg1[2] <= 1.0000766944e+06
 R1233: C4 + 1e+06 alpha_ob59_seg1[2] <= 1.0000766944e+06
 R1234: C3 + 1e+06 alpha_ob59_seg1[3] <= 999986.2654
 R1235: C5 + 1e+06 alpha_ob59_seg1[3] <= 999986.2654
 R1236: alpha_ob59_seg1[0] + alpha_ob59_seg1[1] + alpha_ob59_seg1[2]
   + alpha_ob59_seg1[3] >= 1
 R1237: - C2 + 1e+06 alpha_ob60_seg1[0] <= 999914.52327842
 R1238: - C4 + 1e+06 alpha_ob60_seg1[0] <= 999914.52327842
 R1239: - C3 + 1e+06 alpha_ob60_seg1[1] <= 999993.6552362755
 R1240: - C5 + 1e+06 alpha_ob60_seg1[1] <= 999993.6552362755
 R1241: C2 + 1e+06 alpha_ob60_seg1[2] <= 1.0000829266784199e+06
 R1242: C4 + 1e+06 alpha_ob60_seg1[2] <= 1.0000829266784199e+06
 R1243: C3 + 1e+06 alpha_ob60_seg1[3] <= 999992.1947642756
 R1244: C5 + 1e+06 alpha_ob60_seg1[3] <= 999992.1947642756
 R1245: alpha_ob60_seg1[0] + alpha_ob60_seg1[1] + alpha_ob60_seg1[2]
   + alpha_ob60_seg1[3] >= 1
 R1246: - C2 + 1e+06 alpha_ob61_seg1[0] <= 999912.265199801
 R1247: - C4 + 1e+06 alpha_ob61_seg1[0] <= 999912.265199801
 R1248: - C3 + 1e+06 alpha_ob61_seg1[1] <= 999987.7842112777
 R1249: - C5 + 1e+06 alpha_ob61_seg1[1] <= 999987.7842112777
 R1250: C2 + 1e+06 alpha_ob61_seg1[2] <= 1.000076834799801e+06
 R1251: C4 + 1e+06 alpha_ob61_seg1[2] <= 1.000076834799801e+06
 R1252: C3 + 1e+06 alpha_ob61_seg1[3] <= 1.0000096658112778e+06
 R1253: C5 + 1e+06 alpha_ob61_seg1[3] <= 1.0000096658112778e+06
 R1254: alpha_ob61_seg1[0] + alpha_ob61_seg1[1] + alpha_ob61_seg1[2]
   + alpha_ob61_seg1[3] >= 1
 R1255: - C2 + 1e+06 alpha_ob62_seg1[0] <= 999920.6152119379
 R1256: - C4 + 1e+06 alpha_ob62_seg1[0] <= 999920.6152119379
 R1257: - C3 + 1e+06 alpha_ob62_seg1[1] <= 999981.4341997246
 R1258: - C5 + 1e+06 alpha_ob62_seg1[1] <= 999981.4341997246
 R1259: C2 + 1e+06 alpha_ob62_seg1[2] <= 1.0000768348119379e+06
 R1260: C4 + 1e+06 alpha_ob62_seg1[2] <= 1.0000768348119379e+06
 R1261: C3 + 1e+06 alpha_ob62_seg1[3] <= 1.0000096657997245e+06
 R1262: C5 + 1e+06 alpha_ob62_seg1[3] <= 1.0000096657997245e+06
 R1263: alpha_ob62_seg1[0] + alpha_ob62_seg1[1] + alpha_ob62_seg1[2]
   + alpha_ob62_seg1[3] >= 1
 R1264: - C2 + 1e+06 alpha_ob63_seg1[0] <= 999919.2652
 R1265: - C4 + 1e+06 alpha_ob63_seg1[0] <= 999919.2652
 R1266: - C3 + 1e+06 alpha_ob63_seg1[1] <= 999981.4342
 R1267: - C5 + 1e+06 alpha_ob63_seg1[1] <= 999981.4342
 R1268: C2 + 1e+06 alpha_ob63_seg1[2] <= 1.0000768348e+06
 R1269: C4 + 1e+06 alpha_ob63_seg1[2] <= 1.0000768348e+06
 R1270: C3 + 1e+06 alpha_ob63_seg1[3] <= 1.0000160158000001e+06
 R1271: C5 + 1e+06 alpha_ob63_seg1[3] <= 1.0000160158000001e+06
 R1272: alpha_ob63_seg1[0] + alpha_ob63_seg1[1] + alpha_ob63_seg1[2]
   + alpha_ob63_seg1[3] >= 1
 R1273: - C2 + 1e+06 alpha_ob64_seg1[0] <= 999919.2652454559
 R1274: - C4 + 1e+06 alpha_ob64_seg1[0] <= 999919.2652454559
 R1275: - C3 + 1e+06 alpha_ob64_seg1[1] <= 999956.8341997246
 R1276: - C5 + 1e+06 alpha_ob64_seg1[1] <= 999956.8341997246
 R1277: C2 + 1e+06 alpha_ob64_seg1[2] <= 1.0000781848454559e+06
 R1278: C4 + 1e+06 alpha_ob64_seg1[2] <= 1.0000781848454559e+06
 R1279: C3 + 1e+06 alpha_ob64_seg1[3] <= 1.0000160157997246e+06
 R1280: C5 + 1e+06 alpha_ob64_seg1[3] <= 1.0000160157997246e+06
 R1281: alpha_ob64_seg1[0] + alpha_ob64_seg1[1] + alpha_ob64_seg1[2]
   + alpha_ob64_seg1[3] >= 1
 R1282: - C2 + 1e+06 alpha_ob65_seg1[0] <= 1.0000081732920931e+06
 R1283: - C4 + 1e+06 alpha_ob65_seg1[0] <= 1.0000081732920931e+06
 R1284: - C3 + 1e+06 alpha_ob65_seg1[1] <= 999925.4331002756
 R1285: - C5 + 1e+06 alpha_ob65_seg1[1] <= 999925.4331002756
 R1286: C2 + 1e+06 alpha_ob65_seg1[2] <= 999989.2766720931
 R1287: C4 + 1e+06 alpha_ob65_seg1[2] <= 999989.2766720931
 R1288: C3 + 1e+06 alpha_ob65_seg1[3] <= 1.0000624169002755e+06
 R1289: C5 + 1e+06 alpha_ob65_seg1[3] <= 1.0000624169002755e+06
 R1290: alpha_ob65_seg1[0] + alpha_ob65_seg1[1] + alpha_ob65_seg1[2]
   + alpha_ob65_seg1[3] >= 1
 R1291: - C2 + 1e+06 alpha_ob66_seg1[0] <= 1.0000146737e+06
 R1292: - C4 + 1e+06 alpha_ob66_seg1[0] <= 1.0000146737e+06
 R1293: - C3 + 1e+06 alpha_ob66_seg1[1] <= 999940.7624
 R1294: - C5 + 1e+06 alpha_ob66_seg1[1] <= 999940.7624
 R1295: C2 + 1e+06 alpha_ob66_seg1[2] <= 999948.4263000001
 R1296: C4 + 1e+06 alpha_ob66_seg1[2] <= 999948.4263000001
 R1297: C3 + 1e+06 alpha_ob66_seg1[3] <= 1.0000566875999999e+06
 R1298: C5 + 1e+06 alpha_ob66_seg1[3] <= 1.0000566875999999e+06
 R1299: alpha_ob66_seg1[0] + alpha_ob66_seg1[1] + alpha_ob66_seg1[2]
   + alpha_ob66_seg1[3] >= 1
 R1300: - C2 + 1e+06 alpha_ob67_seg1[0] <= 1.0000058189e+06
 R1301: - C4 + 1e+06 alpha_ob67_seg1[0] <= 1.0000058189e+06
 R1302: - C3 + 1e+06 alpha_ob67_seg1[1] <= 999940.5469
 R1303: - C5 + 1e+06 alpha_ob67_seg1[1] <= 999940.5469
 R1304: C2 + 1e+06 alpha_ob67_seg1[2] <= 999985.5311
 R1305: C4 + 1e+06 alpha_ob67_seg1[2] <= 999985.5311
 R1306: C3 + 1e+06 alpha_ob67_seg1[3] <= 1.0000569031e+06
 R1307: C5 + 1e+06 alpha_ob67_seg1[3] <= 1.0000569031e+06
 R1308: alpha_ob67_seg1[0] + alpha_ob67_seg1[1] + alpha_ob67_seg1[2]
   + alpha_ob67_seg1[3] >= 1
 R1309: - C2 + 1e+06 alpha_ob68_seg1[0] <= 1.0000058188284199e+06
 R1310: - C4 + 1e+06 alpha_ob68_seg1[0] <= 1.0000058188284199e+06
 R1311: - C3 + 1e+06 alpha_ob68_seg1[1] <= 999940.5469002755
 R1312: - C5 + 1e+06 alpha_ob68_seg1[1] <= 999940.5469002755
 R1313: C2 + 1e+06 alpha_ob68_seg1[2] <= 999991.6311284199
 R1314: C4 + 1e+06 alpha_ob68_seg1[2] <= 999991.6311284199
 R1315: C3 + 1e+06 alpha_ob68_seg1[3] <= 1.0000453031002756e+06
 R1316: C5 + 1e+06 alpha_ob68_seg1[3] <= 1.0000453031002756e+06
 R1317: alpha_ob68_seg1[0] + alpha_ob68_seg1[1] + alpha_ob68_seg1[2]
   + alpha_ob68_seg1[3] >= 1
 R1318: - C2 + 1e+06 alpha_ob69_seg1[0] <= 999996.9688499999
 R1319: - C4 + 1e+06 alpha_ob69_seg1[0] <= 999996.9688499999
 R1320: - C3 + 1e+06 alpha_ob69_seg1[1] <= 999940.5469
 R1321: - C5 + 1e+06 alpha_ob69_seg1[1] <= 999940.5469
 R1322: C2 + 1e+06 alpha_ob69_seg1[2] <= 999991.63115
 R1323: C4 + 1e+06 alpha_ob69_seg1[2] <= 999991.63115
 R1324: C3 + 1e+06 alpha_ob69_seg1[3] <= 1.0000569031e+06
 R1325: C5 + 1e+06 alpha_ob69_seg1[3] <= 1.0000569031e+06
 R1326: alpha_ob69_seg1[0] + alpha_ob69_seg1[1] + alpha_ob69_seg1[2]
   + alpha_ob69_seg1[3] >= 1
 R1327: - C2 + 1e+06 alpha_ob70_seg1[0] <= 999991.83683
 R1328: - C4 + 1e+06 alpha_ob70_seg1[0] <= 999991.83683
 R1329: - C3 + 1e+06 alpha_ob70_seg1[1] <= 999940.589
 R1330: - C5 + 1e+06 alpha_ob70_seg1[1] <= 999940.589
 R1331: C2 + 1e+06 alpha_ob70_seg1[2] <= 1.00000401317e+06
 R1332: C4 + 1e+06 alpha_ob70_seg1[2] <= 1.00000401317e+06
 R1333: C3 + 1e+06 alpha_ob70_seg1[3] <= 1.000056861e+06
 R1334: C5 + 1e+06 alpha_ob70_seg1[3] <= 1.000056861e+06
 R1335: alpha_ob70_seg1[0] + alpha_ob70_seg1[1] + alpha_ob70_seg1[2]
   + alpha_ob70_seg1[3] >= 1
 R1336: - C2 + 1e+06 alpha_ob71_seg1[0] <= 999991.8368084199
 R1337: - C4 + 1e+06 alpha_ob71_seg1[0] <= 999991.8368084199
 R1338: - C3 + 1e+06 alpha_ob71_seg1[1] <= 999940.5890002755
 R1339: - C5 + 1e+06 alpha_ob71_seg1[1] <= 999940.5890002755
 R1340: C2 + 1e+06 alpha_ob71_seg1[2] <= 1.0000056131484199e+06
 R1341: C4 + 1e+06 alpha_ob71_seg1[2] <= 1.0000056131484199e+06
 R1342: C3 + 1e+06 alpha_ob71_seg1[3] <= 1.0000452610002755e+06
 R1343: C5 + 1e+06 alpha_ob71_seg1[3] <= 1.0000452610002755e+06
 R1344: alpha_ob71_seg1[0] + alpha_ob71_seg1[1] + alpha_ob71_seg1[2]
   + alpha_ob71_seg1[3] >= 1
 R1345: - C2 + 1e+06 alpha_ob72_seg1[0] <= 1.0000764021508709e+06
 R1346: - C4 + 1e+06 alpha_ob72_seg1[0] <= 1.0000764021508709e+06
 R1347: - C3 + 1e+06 alpha_ob72_seg1[1] <= 999957.1027002756
 R1348: - C5 + 1e+06 alpha_ob72_seg1[1] <= 999957.1027002756
 R1349: C2 + 1e+06 alpha_ob72_seg1[2] <= 999921.0477508709
 R1350: C4 + 1e+06 alpha_ob72_seg1[2] <= 999921.0477508709
 R1351: C3 + 1e+06 alpha_ob72_seg1[3] <= 1.0000137473002756e+06
 R1352: C5 + 1e+06 alpha_ob72_seg1[3] <= 1.0000137473002756e+06
 R1353: alpha_ob72_seg1[0] + alpha_ob72_seg1[1] + alpha_ob72_seg1[2]
   + alpha_ob72_seg1[3] >= 1
 R1354: - C2 + 1e+06 alpha_ob73_seg1[0] <= 1.0000445522e+06
 R1355: - C4 + 1e+06 alpha_ob73_seg1[0] <= 1.0000445522e+06
 R1356: - C3 + 1e+06 alpha_ob73_seg1[1] <= 999983.7027
 R1357: - C5 + 1e+06 alpha_ob73_seg1[1] <= 999983.7027
 R1358: C2 + 1e+06 alpha_ob73_seg1[2] <= 999921.0477999999
 R1359: C4 + 1e+06 alpha_ob73_seg1[2] <= 999921.0477999999
 R1360: C3 + 1e+06 alpha_ob73_seg1[3] <= 1.0000137473e+06
 R1361: C5 + 1e+06 alpha_ob73_seg1[3] <= 1.0000137473e+06
 R1362: alpha_ob73_seg1[0] + alpha_ob73_seg1[1] + alpha_ob73_seg1[2]
   + alpha_ob73_seg1[3] >= 1
 R1363: - C2 + 1e+06 alpha_ob74_seg1[0] <= 1.0000445522183661e+06
 R1364: - C4 + 1e+06 alpha_ob74_seg1[0] <= 1.0000445522183661e+06
 R1365: - C3 + 1e+06 alpha_ob74_seg1[1] <= 999973.8526997245
 R1366: - C5 + 1e+06 alpha_ob74_seg1[1] <= 999973.8526997245
 R1367: C2 + 1e+06 alpha_ob74_seg1[2] <= 999952.8978183661
 R1368: C4 + 1e+06 alpha_ob74_seg1[2] <= 999952.8978183661
 R1369: C3 + 1e+06 alpha_ob74_seg1[3] <= 1.0000137472997245e+06
 R1370: C5 + 1e+06 alpha_ob74_seg1[3] <= 1.0000137472997245e+06
 R1371: alpha_ob74_seg1[0] + alpha_ob74_seg1[1] + alpha_ob74_seg1[2]
   + alpha_ob74_seg1[3] >= 1
 R1372: - C2 + 1e+06 alpha_ob75_seg1[0] <= 1.000064332081634e+06
 R1373: - C4 + 1e+06 alpha_ob75_seg1[0] <= 1.000064332081634e+06
 R1374: - C3 + 1e+06 alpha_ob75_seg1[1] <= 999973.9401002756
 R1375: - C5 + 1e+06 alpha_ob75_seg1[1] <= 999973.9401002756
 R1376: C2 + 1e+06 alpha_ob75_seg1[2] <= 999933.1178816339
 R1377: C4 + 1e+06 alpha_ob75_seg1[2] <= 999933.1178816339
 R1378: C3 + 1e+06 alpha_ob75_seg1[3] <= 1.0000136599002755e+06
 R1379: C5 + 1e+06 alpha_ob75_seg1[3] <= 1.0000136599002755e+06
 R1380: alpha_ob75_seg1[0] + alpha_ob75_seg1[1] + alpha_ob75_seg1[2]
   + alpha_ob75_seg1[3] >= 1
 R1381: - C2 + 1e+06 alpha_ob76_seg1[0] <= 1.0000643321013774e+06
 R1382: - C4 + 1e+06 alpha_ob76_seg1[0] <= 1.0000643321013774e+06
 R1383: - C3 + 1e+06 alpha_ob76_seg1[1] <= 999983.1900997246
 R1384: - C5 + 1e+06 alpha_ob76_seg1[1] <= 999983.1900997246
 R1385: C2 + 1e+06 alpha_ob76_seg1[2] <= 999933.1179013775
 R1386: C4 + 1e+06 alpha_ob76_seg1[2] <= 999933.1179013775
 R1387: C3 + 1e+06 alpha_ob76_seg1[3] <= 1.0000136598997245e+06
 R1388: C5 + 1e+06 alpha_ob76_seg1[3] <= 1.0000136598997245e+06
 R1389: alpha_ob76_seg1[0] + alpha_ob76_seg1[1] + alpha_ob76_seg1[2]
   + alpha_ob76_seg1[3] >= 1
 R1390: - C4 + 1e+06 alpha_ob0_seg2[0] <= 1.0000284170499525e+06
 R1391: - C6 + 1e+06 alpha_ob0_seg2[0] <= 1.0000284170499525e+06
 R1392: - C5 + 1e+06 alpha_ob0_seg2[1] <= 999957.0843002755
 R1393: - C7 + 1e+06 alpha_ob0_seg2[1] <= 999957.0843002755
 R1394: C4 + 1e+06 alpha_ob0_seg2[2] <= 999969.0328499526
 R1395: C6 + 1e+06 alpha_ob0_seg2[2] <= 999969.0328499526
 R1396: C5 + 1e+06 alpha_ob0_seg2[3] <= 1.0000132657002755e+06
 R1397: C7 + 1e+06 alpha_ob0_seg2[3] <= 1.0000132657002755e+06
 R1398: alpha_ob0_seg2[0] + alpha_ob0_seg2[1] + alpha_ob0_seg2[2]
   + alpha_ob0_seg2[3] >= 1
 R1399: - C4 + 1e+06 alpha_ob1_seg2[0] <= 1.000028417099801e+06
 R1400: - C6 + 1e+06 alpha_ob1_seg2[0] <= 1.000028417099801e+06
 R1401: - C5 + 1e+06 alpha_ob1_seg2[1] <= 999984.1843155898
 R1402: - C7 + 1e+06 alpha_ob1_seg2[1] <= 999984.1843155898
 R1403: C4 + 1e+06 alpha_ob1_seg2[2] <= 999957.4328998009
 R1404: C6 + 1e+06 alpha_ob1_seg2[2] <= 999957.4328998009
 R1405: C5 + 1e+06 alpha_ob1_seg2[3] <= 1.0000132657155899e+06
 R1406: C7 + 1e+06 alpha_ob1_seg2[3] <= 1.0000132657155899e+06
 R1407: alpha_ob1_seg2[0] + alpha_ob1_seg2[1] + alpha_ob1_seg2[2]
   + alpha_ob1_seg2[3] >= 1
 R1408: - C4 + 1e+06 alpha_ob2_seg2[0] <= 1.0000400171500475e+06
 R1409: - C6 + 1e+06 alpha_ob2_seg2[0] <= 1.0000400171500475e+06
 R1410: - C5 + 1e+06 alpha_ob2_seg2[1] <= 999957.0842997245
 R1411: - C7 + 1e+06 alpha_ob2_seg2[1] <= 999957.0842997245
 R1412: C4 + 1e+06 alpha_ob2_seg2[2] <= 999957.4329500474
 R1413: C6 + 1e+06 alpha_ob2_seg2[2] <= 999957.4329500474
 R1414: C5 + 1e+06 alpha_ob2_seg2[3] <= 1.0000132656997246e+06
 R1415: C7 + 1e+06 alpha_ob2_seg2[3] <= 1.0000132656997246e+06
 R1416: alpha_ob2_seg2[0] + alpha_ob2_seg2[1] + alpha_ob2_seg2[2]
   + alpha_ob2_seg2[3] >= 1
 R1417: - C4 + 1e+06 alpha_ob3_seg2[0] <= 1.0000109162e+06
 R1418: - C6 + 1e+06 alpha_ob3_seg2[0] <= 1.0000109162e+06
 R1419: - C5 + 1e+06 alpha_ob3_seg2[1] <= 999970.2681
 R1420: - C7 + 1e+06 alpha_ob3_seg2[1] <= 999970.2681
 R1421: C4 + 1e+06 alpha_ob3_seg2[2] <= 999975.1838
 R1422: C6 + 1e+06 alpha_ob3_seg2[2] <= 999975.1838
 R1423: C5 + 1e+06 alpha_ob3_seg2[3] <= 1.0000271819e+06
 R1424: C7 + 1e+06 alpha_ob3_seg2[3] <= 1.0000271819e+06
 R1425: alpha_ob3_seg2[0] + alpha_ob3_seg2[1] + alpha_ob3_seg2[2]
   + alpha_ob3_seg2[3] >= 1
 R1426: - C4 + 1e+06 alpha_ob4_seg2[0] <= 1.0000109161752058e+06
 R1427: - C6 + 1e+06 alpha_ob4_seg2[0] <= 1.0000109161752058e+06
 R1428: - C5 + 1e+06 alpha_ob4_seg2[1] <= 999970.2681002755
 R1429: - C7 + 1e+06 alpha_ob4_seg2[1] <= 999970.2681002755
 R1430: C4 + 1e+06 alpha_ob4_seg2[2] <= 999986.5337752058
 R1431: C6 + 1e+06 alpha_ob4_seg2[2] <= 999986.5337752058
 R1432: C5 + 1e+06 alpha_ob4_seg2[3] <= 1.0000138319002755e+06
 R1433: C7 + 1e+06 alpha_ob4_seg2[3] <= 1.0000138319002755e+06
 R1434: alpha_ob4_seg2[0] + alpha_ob4_seg2[1] + alpha_ob4_seg2[2]
   + alpha_ob4_seg2[3] >= 1
 R1435: - C4 + 1e+06 alpha_ob5_seg2[0] <= 1.000010916199801e+06
 R1436: - C6 + 1e+06 alpha_ob5_seg2[0] <= 1.000010916199801e+06
 R1437: - C5 + 1e+06 alpha_ob5_seg2[1] <= 999983.6181179117
 R1438: - C7 + 1e+06 alpha_ob5_seg2[1] <= 999983.6181179117
 R1439: C4 + 1e+06 alpha_ob5_seg2[2] <= 999973.183799801
 R1440: C6 + 1e+06 alpha_ob5_seg2[2] <= 999973.183799801
 R1441: C5 + 1e+06 alpha_ob5_seg2[3] <= 1.0000138319179118e+06
 R1442: C7 + 1e+06 alpha_ob5_seg2[3] <= 1.0000138319179118e+06
 R1443: alpha_ob5_seg2[0] + alpha_ob5_seg2[1] + alpha_ob5_seg2[2]
   + alpha_ob5_seg2[3] >= 1
 R1444: - C4 + 1e+06 alpha_ob6_seg2[0] <= 1.0000139162e+06
 R1445: - C6 + 1e+06 alpha_ob6_seg2[0] <= 1.0000139162e+06
 R1446: - C5 + 1e+06 alpha_ob6_seg2[1] <= 999957.1679999999
 R1447: - C7 + 1e+06 alpha_ob6_seg2[1] <= 999957.1679999999
 R1448: C4 + 1e+06 alpha_ob6_seg2[2] <= 999975.1838
 R1449: C6 + 1e+06 alpha_ob6_seg2[2] <= 999975.1838
 R1450: C5 + 1e+06 alpha_ob6_seg2[3] <= 1.000040282e+06
 R1451: C7 + 1e+06 alpha_ob6_seg2[3] <= 1.000040282e+06
 R1452: alpha_ob6_seg2[0] + alpha_ob6_seg2[1] + alpha_ob6_seg2[2]
   + alpha_ob6_seg2[3] >= 1
 R1453: - C4 + 1e+06 alpha_ob7_seg2[0] <= 999992.49102
 R1454: - C6 + 1e+06 alpha_ob7_seg2[0] <= 999992.49102
 R1455: - C5 + 1e+06 alpha_ob7_seg2[1] <= 999957.1016000001
 R1456: - C7 + 1e+06 alpha_ob7_seg2[1] <= 999957.1016000001
 R1457: C4 + 1e+06 alpha_ob7_seg2[2] <= 999986.35898
 R1458: C6 + 1e+06 alpha_ob7_seg2[2] <= 999986.35898
 R1459: C5 + 1e+06 alpha_ob7_seg2[3] <= 1.0000403484e+06
 R1460: C7 + 1e+06 alpha_ob7_seg2[3] <= 1.0000403484e+06
 R1461: alpha_ob7_seg2[0] + alpha_ob7_seg2[1] + alpha_ob7_seg2[2]
   + alpha_ob7_seg2[3] >= 1
 R1462: - C4 + 1e+06 alpha_ob8_seg2[0] <= 999981.3909999999
 R1463: - C6 + 1e+06 alpha_ob8_seg2[0] <= 999981.3909999999
 R1464: - C5 + 1e+06 alpha_ob8_seg2[1] <= 999957.1016000001
 R1465: - C7 + 1e+06 alpha_ob8_seg2[1] <= 999957.1016000001
 R1466: C4 + 1e+06 alpha_ob8_seg2[2] <= 1.000004959e+06
 R1467: C6 + 1e+06 alpha_ob8_seg2[2] <= 1.000004959e+06
 R1468: C5 + 1e+06 alpha_ob8_seg2[3] <= 1.0000403484e+06
 R1469: C7 + 1e+06 alpha_ob8_seg2[3] <= 1.0000403484e+06
 R1470: alpha_ob8_seg2[0] + alpha_ob8_seg2[1] + alpha_ob8_seg2[2]
   + alpha_ob8_seg2[3] >= 1
 R1471: - C4 + 1e+06 alpha_ob9_seg2[0] <= 999981.3909825523
 R1472: - C6 + 1e+06 alpha_ob9_seg2[0] <= 999981.3909825523
 R1473: - C5 + 1e+06 alpha_ob9_seg2[1] <= 999957.1016002755
 R1474: - C7 + 1e+06 alpha_ob9_seg2[1] <= 999957.1016002755
 R1475: C4 + 1e+06 alpha_ob9_seg2[2] <= 1.0000160589825523e+06
 R1476: C6 + 1e+06 alpha_ob9_seg2[2] <= 1.0000160589825523e+06
 R1477: C5 + 1e+06 alpha_ob9_seg2[3] <= 1.0000309984002755e+06
 R1478: C7 + 1e+06 alpha_ob9_seg2[3] <= 1.0000309984002755e+06
 R1479: alpha_ob9_seg2[0] + alpha_ob9_seg2[1] + alpha_ob9_seg2[2]
   + alpha_ob9_seg2[3] >= 1
 R1480: - C4 + 1e+06 alpha_ob10_seg2[0] <= 999981.3867954084
 R1481: - C6 + 1e+06 alpha_ob10_seg2[0] <= 999981.3867954084
 R1482: - C5 + 1e+06 alpha_ob10_seg2[1] <= 999968.8595002755
 R1483: - C7 + 1e+06 alpha_ob10_seg2[1] <= 999968.8595002755
 R1484: C4 + 1e+06 alpha_ob10_seg2[2] <= 1.0000160631954084e+06
 R1485: C6 + 1e+06 alpha_ob10_seg2[2] <= 1.0000160631954084e+06
 R1486: C5 + 1e+06 alpha_ob10_seg2[3] <= 1.0000262405002755e+06
 R1487: C7 + 1e+06 alpha_ob10_seg2[3] <= 1.0000262405002755e+06
 R1488: alpha_ob10_seg2[0] + alpha_ob10_seg2[1] + alpha_ob10_seg2[2]
   + alpha_ob10_seg2[3] >= 1
 R1489: - C4 + 1e+06 alpha_ob11_seg2[0] <= 999981.386799801
 R1490: - C6 + 1e+06 alpha_ob11_seg2[0] <= 999981.386799801
 R1491: - C5 + 1e+06 alpha_ob11_seg2[1] <= 999971.2095013268
 R1492: - C7 + 1e+06 alpha_ob11_seg2[1] <= 999971.2095013268
 R1493: C4 + 1e+06 alpha_ob11_seg2[2] <= 1.0000152131998009e+06
 R1494: C6 + 1e+06 alpha_ob11_seg2[2] <= 1.0000152131998009e+06
 R1495: C5 + 1e+06 alpha_ob11_seg2[3] <= 1.0000262405013267e+06
 R1496: C7 + 1e+06 alpha_ob11_seg2[3] <= 1.0000262405013267e+06
 R1497: alpha_ob11_seg2[0] + alpha_ob11_seg2[1] + alpha_ob11_seg2[2]
   + alpha_ob11_seg2[3] >= 1
 R1498: - C4 + 1e+06 alpha_ob12_seg2[0] <= 999982.2368022958
 R1499: - C6 + 1e+06 alpha_ob12_seg2[0] <= 999982.2368022958
 R1500: - C5 + 1e+06 alpha_ob12_seg2[1] <= 999970.1094997245
 R1501: - C7 + 1e+06 alpha_ob12_seg2[1] <= 999970.1094997245
 R1502: C4 + 1e+06 alpha_ob12_seg2[2] <= 1.0000152132022957e+06
 R1503: C6 + 1e+06 alpha_ob12_seg2[2] <= 1.0000152132022957e+06
 R1504: C5 + 1e+06 alpha_ob12_seg2[3] <= 1.0000262404997245e+06
 R1505: C7 + 1e+06 alpha_ob12_seg2[3] <= 1.0000262404997245e+06
 R1506: alpha_ob12_seg2[0] + alpha_ob12_seg2[1] + alpha_ob12_seg2[2]
   + alpha_ob12_seg2[3] >= 1
 R1507: - C4 + 1e+06 alpha_ob13_seg2[0] <= 999982.236799801
 R1508: - C6 + 1e+06 alpha_ob13_seg2[0] <= 999982.236799801
 R1509: - C5 + 1e+06 alpha_ob13_seg2[1] <= 999970.1095291895
 R1510: - C7 + 1e+06 alpha_ob13_seg2[1] <= 999970.1095291895
 R1511: C4 + 1e+06 alpha_ob13_seg2[2] <= 999993.3631998011
 R1512: C6 + 1e+06 alpha_ob13_seg2[2] <= 999993.3631998011
 R1513: C5 + 1e+06 alpha_ob13_seg2[3] <= 1.0000273405291894e+06
 R1514: C7 + 1e+06 alpha_ob13_seg2[3] <= 1.0000273405291894e+06
 R1515: alpha_ob13_seg2[0] + alpha_ob13_seg2[1] + alpha_ob13_seg2[2]
   + alpha_ob13_seg2[3] >= 1
 R1516: - C4 + 1e+06 alpha_ob14_seg2[0] <= 1.0000040868238758e+06
 R1517: - C6 + 1e+06 alpha_ob14_seg2[0] <= 1.0000040868238758e+06
 R1518: - C5 + 1e+06 alpha_ob14_seg2[1] <= 999957.2594997246
 R1519: - C7 + 1e+06 alpha_ob14_seg2[1] <= 999957.2594997246
 R1520: C4 + 1e+06 alpha_ob14_seg2[2] <= 999993.3632238759
 R1521: C6 + 1e+06 alpha_ob14_seg2[2] <= 999993.3632238759
 R1522: C5 + 1e+06 alpha_ob14_seg2[3] <= 1.0000273404997246e+06
 R1523: C7 + 1e+06 alpha_ob14_seg2[3] <= 1.0000273404997246e+06
 R1524: alpha_ob14_seg2[0] + alpha_ob14_seg2[1] + alpha_ob14_seg2[2]
   + alpha_ob14_seg2[3] >= 1
 R1525: - C4 + 1e+06 alpha_ob15_seg2[0] <= 999981.6050802565
 R1526: - C6 + 1e+06 alpha_ob15_seg2[0] <= 999981.6050802565
 R1527: - C5 + 1e+06 alpha_ob15_seg2[1] <= 999972.8942002755
 R1528: - C7 + 1e+06 alpha_ob15_seg2[1] <= 999972.8942002755
 R1529: C4 + 1e+06 alpha_ob15_seg2[2] <= 1.0000158448802566e+06
 R1530: C6 + 1e+06 alpha_ob15_seg2[2] <= 1.0000158448802566e+06
 R1531: C5 + 1e+06 alpha_ob15_seg2[3] <= 1.0000139558002756e+06
 R1532: C7 + 1e+06 alpha_ob15_seg2[3] <= 1.0000139558002756e+06
 R1533: alpha_ob15_seg2[0] + alpha_ob15_seg2[1] + alpha_ob15_seg2[2]
   + alpha_ob15_seg2[3] >= 1
 R1534: - C4 + 1e+06 alpha_ob16_seg2[0] <= 999981.605149801
 R1535: - C6 + 1e+06 alpha_ob16_seg2[0] <= 999981.605149801
 R1536: - C5 + 1e+06 alpha_ob16_seg2[1] <= 999983.4942391404
 R1537: - C7 + 1e+06 alpha_ob16_seg2[1] <= 999983.4942391404
 R1538: C4 + 1e+06 alpha_ob16_seg2[2] <= 999986.4948498011
 R1539: C6 + 1e+06 alpha_ob16_seg2[2] <= 999986.4948498011
 R1540: C5 + 1e+06 alpha_ob16_seg2[3] <= 1.0000139558391405e+06
 R1541: C7 + 1e+06 alpha_ob16_seg2[3] <= 1.0000139558391405e+06
 R1542: alpha_ob16_seg2[0] + alpha_ob16_seg2[1] + alpha_ob16_seg2[2]
   + alpha_ob16_seg2[3] >= 1
 R1543: - C4 + 1e+06 alpha_ob17_seg2[0] <= 999997.6803502565
 R1544: - C6 + 1e+06 alpha_ob17_seg2[0] <= 999997.6803502565
 R1545: - C5 + 1e+06 alpha_ob17_seg2[1] <= 999972.7268002755
 R1546: - C7 + 1e+06 alpha_ob17_seg2[1] <= 999972.7268002755
 R1547: C4 + 1e+06 alpha_ob17_seg2[2] <= 999999.7696102565
 R1548: C6 + 1e+06 alpha_ob17_seg2[2] <= 999999.7696102565
 R1549: C5 + 1e+06 alpha_ob17_seg2[3] <= 1.0000141232002755e+06
 R1550: C7 + 1e+06 alpha_ob17_seg2[3] <= 1.0000141232002755e+06
 R1551: alpha_ob17_seg2[0] + alpha_ob17_seg2[1] + alpha_ob17_seg2[2]
   + alpha_ob17_seg2[3] >= 1
 R1552: - C4 + 1e+06 alpha_ob18_seg2[0] <= 1.0000038341502565e+06
 R1553: - C6 + 1e+06 alpha_ob18_seg2[0] <= 1.0000038341502565e+06
 R1554: - C5 + 1e+06 alpha_ob18_seg2[1] <= 999972.6012002755
 R1555: - C7 + 1e+06 alpha_ob18_seg2[1] <= 999972.6012002755
 R1556: C4 + 1e+06 alpha_ob18_seg2[2] <= 999993.6158102566
 R1557: C6 + 1e+06 alpha_ob18_seg2[2] <= 999993.6158102566
 R1558: C5 + 1e+06 alpha_ob18_seg2[3] <= 1.0000142488002755e+06
 R1559: C7 + 1e+06 alpha_ob18_seg2[3] <= 1.0000142488002755e+06
 R1560: alpha_ob18_seg2[0] + alpha_ob18_seg2[1] + alpha_ob18_seg2[2]
   + alpha_ob18_seg2[3] >= 1
 R1561: - C4 + 1e+06 alpha_ob19_seg2[0] <= 1.0000162874063333e+06
 R1562: - C6 + 1e+06 alpha_ob19_seg2[0] <= 1.0000162874063333e+06
 R1563: - C5 + 1e+06 alpha_ob19_seg2[1] <= 999982.9439602756
 R1564: - C7 + 1e+06 alpha_ob19_seg2[1] <= 999982.9439602756
 R1565: C4 + 1e+06 alpha_ob19_seg2[2] <= 999981.1624063333
 R1566: C6 + 1e+06 alpha_ob19_seg2[2] <= 999981.1624063333
 R1567: C5 + 1e+06 alpha_ob19_seg2[3] <= 999963.6560402757
 R1568: C7 + 1e+06 alpha_ob19_seg2[3] <= 999963.6560402757
 R1569: alpha_ob19_seg2[0] + alpha_ob19_seg2[1] + alpha_ob19_seg2[2]
   + alpha_ob19_seg2[3] >= 1
 R1570: - C4 + 1e+06 alpha_ob20_seg2[0] <= 1.0000162874998011e+06
 R1571: - C6 + 1e+06 alpha_ob20_seg2[0] <= 1.0000162874998011e+06
 R1572: - C5 + 1e+06 alpha_ob20_seg2[1] <= 1.000033794079276e+06
 R1573: - C7 + 1e+06 alpha_ob20_seg2[1] <= 1.000033794079276e+06
 R1574: C4 + 1e+06 alpha_ob20_seg2[2] <= 999921.562499801
 R1575: C6 + 1e+06 alpha_ob20_seg2[2] <= 999921.562499801
 R1576: C5 + 1e+06 alpha_ob20_seg2[3] <= 999963.656079276
 R1577: C7 + 1e+06 alpha_ob20_seg2[3] <= 999963.656079276
 R1578: alpha_ob20_seg2[0] + alpha_ob20_seg2[1] + alpha_ob20_seg2[2]
   + alpha_ob20_seg2[3] >= 1
 R1579: - C4 + 1e+06 alpha_ob21_seg2[0] <= 1.0000066737e+06
 R1580: - C6 + 1e+06 alpha_ob21_seg2[0] <= 1.0000066737e+06
 R1581: - C5 + 1e+06 alpha_ob21_seg2[1] <= 999952.1958
 R1582: - C7 + 1e+06 alpha_ob21_seg2[1] <= 999952.1958
 R1583: C4 + 1e+06 alpha_ob21_seg2[2] <= 999948.4263000001
 R1584: C6 + 1e+06 alpha_ob21_seg2[2] <= 999948.4263000001
 R1585: C5 + 1e+06 alpha_ob21_seg2[3] <= 1.0000452542e+06
 R1586: C7 + 1e+06 alpha_ob21_seg2[3] <= 1.0000452542e+06
 R1587: alpha_ob21_seg2[0] + alpha_ob21_seg2[1] + alpha_ob21_seg2[2]
   + alpha_ob21_seg2[3] >= 1
 R1588: - C4 + 1e+06 alpha_ob22_seg2[0] <= 1.0000162874977043e+06
 R1589: - C6 + 1e+06 alpha_ob22_seg2[0] <= 1.0000162874977043e+06
 R1590: - C5 + 1e+06 alpha_ob22_seg2[1] <= 1.0000337940002755e+06
 R1591: - C7 + 1e+06 alpha_ob22_seg2[1] <= 1.0000337940002755e+06
 R1592: C4 + 1e+06 alpha_ob22_seg2[2] <= 999981.1624977043
 R1593: C6 + 1e+06 alpha_ob22_seg2[2] <= 999981.1624977043
 R1594: C5 + 1e+06 alpha_ob22_seg2[3] <= 999962.5560002755
 R1595: C7 + 1e+06 alpha_ob22_seg2[3] <= 999962.5560002755
 R1596: alpha_ob22_seg2[0] + alpha_ob22_seg2[1] + alpha_ob22_seg2[2]
   + alpha_ob22_seg2[3] >= 1
 R1597: - C4 + 1e+06 alpha_ob23_seg2[0] <= 999980.937509
 R1598: - C6 + 1e+06 alpha_ob23_seg2[0] <= 999980.937509
 R1599: - C5 + 1e+06 alpha_ob23_seg2[1] <= 1.000034894e+06
 R1600: - C7 + 1e+06 alpha_ob23_seg2[1] <= 1.000034894e+06
 R1601: C4 + 1e+06 alpha_ob23_seg2[2] <= 999981.162491
 R1602: C6 + 1e+06 alpha_ob23_seg2[2] <= 999981.162491
 R1603: C5 + 1e+06 alpha_ob23_seg2[3] <= 999962.556
 R1604: C7 + 1e+06 alpha_ob23_seg2[3] <= 999962.556
 R1605: alpha_ob23_seg2[0] + alpha_ob23_seg2[1] + alpha_ob23_seg2[2]
   + alpha_ob23_seg2[3] >= 1
 R1606: - C4 + 1e+06 alpha_ob24_seg2[0] <= 1.0000070091e+06
 R1607: - C6 + 1e+06 alpha_ob24_seg2[0] <= 1.0000070091e+06
 R1608: - C5 + 1e+06 alpha_ob24_seg2[1] <= 1.0000598843e+06
 R1609: - C7 + 1e+06 alpha_ob24_seg2[1] <= 1.0000598843e+06
 R1610: C4 + 1e+06 alpha_ob24_seg2[2] <= 999965.0909
 R1611: C6 + 1e+06 alpha_ob24_seg2[2] <= 999965.0909
 R1612: C5 + 1e+06 alpha_ob24_seg2[3] <= 999937.5657
 R1613: C7 + 1e+06 alpha_ob24_seg2[3] <= 999937.5657
 R1614: alpha_ob24_seg2[0] + alpha_ob24_seg2[1] + alpha_ob24_seg2[2]
   + alpha_ob24_seg2[3] >= 1
 R1615: - C4 + 1e+06 alpha_ob25_seg2[0] <= 1.000006992114031e+06
 R1616: - C6 + 1e+06 alpha_ob25_seg2[0] <= 1.000006992114031e+06
 R1617: - C5 + 1e+06 alpha_ob25_seg2[1] <= 1.0000588068002755e+06
 R1618: - C7 + 1e+06 alpha_ob25_seg2[1] <= 1.0000588068002755e+06
 R1619: C4 + 1e+06 alpha_ob25_seg2[2] <= 999990.457874031
 R1620: C6 + 1e+06 alpha_ob25_seg2[2] <= 999990.457874031
 R1621: C5 + 1e+06 alpha_ob25_seg2[3] <= 999935.5432002755
 R1622: C7 + 1e+06 alpha_ob25_seg2[3] <= 999935.5432002755
 R1623: alpha_ob25_seg2[0] + alpha_ob25_seg2[1] + alpha_ob25_seg2[2]
   + alpha_ob25_seg2[3] >= 1
 R1624: - C4 + 1e+06 alpha_ob26_seg2[0] <= 1.0000033775063743e+06
 R1625: - C6 + 1e+06 alpha_ob26_seg2[0] <= 1.0000033775063743e+06
 R1626: - C5 + 1e+06 alpha_ob26_seg2[1] <= 1.0000348666997246e+06
 R1627: - C7 + 1e+06 alpha_ob26_seg2[1] <= 1.0000348666997246e+06
 R1628: C4 + 1e+06 alpha_ob26_seg2[2] <= 999994.0725863742
 R1629: C6 + 1e+06 alpha_ob26_seg2[2] <= 999994.0725863742
 R1630: C5 + 1e+06 alpha_ob26_seg2[3] <= 999937.4832997246
 R1631: C7 + 1e+06 alpha_ob26_seg2[3] <= 999937.4832997246
 R1632: alpha_ob26_seg2[0] + alpha_ob26_seg2[1] + alpha_ob26_seg2[2]
   + alpha_ob26_seg2[3] >= 1
 R1633: - C4 + 1e+06 alpha_ob27_seg2[0] <= 999990.2670277582
 R1634: - C6 + 1e+06 alpha_ob27_seg2[0] <= 999990.2670277582
 R1635: - C5 + 1e+06 alpha_ob27_seg2[1] <= 1.0000347702002756e+06
 R1636: - C7 + 1e+06 alpha_ob27_seg2[1] <= 1.0000347702002756e+06
 R1637: C4 + 1e+06 alpha_ob27_seg2[2] <= 1.0000071828877581e+06
 R1638: C6 + 1e+06 alpha_ob27_seg2[2] <= 1.0000071828877581e+06
 R1639: C5 + 1e+06 alpha_ob27_seg2[3] <= 999939.8298002756
 R1640: C7 + 1e+06 alpha_ob27_seg2[3] <= 999939.8298002756
 R1641: alpha_ob27_seg2[0] + alpha_ob27_seg2[1] + alpha_ob27_seg2[2]
   + alpha_ob27_seg2[3] >= 1
 R1642: - C4 + 1e+06 alpha_ob28_seg2[0] <= 999990.267069801
 R1643: - C6 + 1e+06 alpha_ob28_seg2[0] <= 999990.267069801
 R1644: - C5 + 1e+06 alpha_ob28_seg2[1] <= 1.0000576202069657e+06
 R1645: - C7 + 1e+06 alpha_ob28_seg2[1] <= 1.0000576202069657e+06
 R1646: C4 + 1e+06 alpha_ob28_seg2[2] <= 1.000002082929801e+06
 R1647: C6 + 1e+06 alpha_ob28_seg2[2] <= 1.000002082929801e+06
 R1648: C5 + 1e+06 alpha_ob28_seg2[3] <= 999939.8298069657
 R1649: C7 + 1e+06 alpha_ob28_seg2[3] <= 999939.8298069657
 R1650: alpha_ob28_seg2[0] + alpha_ob28_seg2[1] + alpha_ob28_seg2[2]
   + alpha_ob28_seg2[3] >= 1
 R1651: - C4 + 1e+06 alpha_ob29_seg2[0] <= 999991.073732
 R1652: - C6 + 1e+06 alpha_ob29_seg2[0] <= 999991.073732
 R1653: - C5 + 1e+06 alpha_ob29_seg2[1] <= 999952.2125
 R1654: - C7 + 1e+06 alpha_ob29_seg2[1] <= 999952.2125
 R1655: C4 + 1e+06 alpha_ob29_seg2[2] <= 999990.776268
 R1656: C6 + 1e+06 alpha_ob29_seg2[2] <= 999990.776268
 R1657: C5 + 1e+06 alpha_ob29_seg2[3] <= 1.0000452375e+06
 R1658: C7 + 1e+06 alpha_ob29_seg2[3] <= 1.0000452375e+06
 R1659: alpha_ob29_seg2[0] + alpha_ob29_seg2[1] + alpha_ob29_seg2[2]
   + alpha_ob29_seg2[3] >= 1
 R1660: - C4 + 1e+06 alpha_ob30_seg2[0] <= 999985.657099801
 R1661: - C6 + 1e+06 alpha_ob30_seg2[0] <= 999985.657099801
 R1662: - C5 + 1e+06 alpha_ob30_seg2[1] <= 1.0000576202063023e+06
 R1663: - C7 + 1e+06 alpha_ob30_seg2[1] <= 1.0000576202063023e+06
 R1664: C4 + 1e+06 alpha_ob30_seg2[2] <= 1.000007192899801e+06
 R1665: C6 + 1e+06 alpha_ob30_seg2[2] <= 1.000007192899801e+06
 R1666: C5 + 1e+06 alpha_ob30_seg2[3] <= 999939.8298063023
 R1667: C7 + 1e+06 alpha_ob30_seg2[3] <= 999939.8298063023
 R1668: alpha_ob30_seg2[0] + alpha_ob30_seg2[1] + alpha_ob30_seg2[2]
   + alpha_ob30_seg2[3] >= 1
 R1669: - C4 + 1e+06 alpha_ob31_seg2[0] <= 999994.771829801
 R1670: - C6 + 1e+06 alpha_ob31_seg2[0] <= 999994.771829801
 R1671: - C5 + 1e+06 alpha_ob31_seg2[1] <= 1.0000615983063023e+06
 R1672: - C7 + 1e+06 alpha_ob31_seg2[1] <= 1.0000615983063023e+06
 R1673: C4 + 1e+06 alpha_ob31_seg2[2] <= 999998.078169801
 R1674: C6 + 1e+06 alpha_ob31_seg2[2] <= 999998.078169801
 R1675: C5 + 1e+06 alpha_ob31_seg2[3] <= 999935.8517063023
 R1676: C7 + 1e+06 alpha_ob31_seg2[3] <= 999935.8517063023
 R1677: alpha_ob31_seg2[0] + alpha_ob31_seg2[1] + alpha_ob31_seg2[2]
   + alpha_ob31_seg2[3] >= 1
 R1678: - C4 + 1e+06 alpha_ob32_seg2[0] <= 999999.3718445294
 R1679: - C6 + 1e+06 alpha_ob32_seg2[0] <= 999999.3718445294
 R1680: - C5 + 1e+06 alpha_ob32_seg2[1] <= 1.0000527482997245e+06
 R1681: - C7 + 1e+06 alpha_ob32_seg2[1] <= 1.0000527482997245e+06
 R1682: C4 + 1e+06 alpha_ob32_seg2[2] <= 999998.0781885295
 R1683: C6 + 1e+06 alpha_ob32_seg2[2] <= 999998.0781885295
 R1684: C5 + 1e+06 alpha_ob32_seg2[3] <= 999935.8516997246
 R1685: C7 + 1e+06 alpha_ob32_seg2[3] <= 999935.8516997246
 R1686: alpha_ob32_seg2[0] + alpha_ob32_seg2[1] + alpha_ob32_seg2[2]
   + alpha_ob32_seg2[3] >= 1
 R1687: - C4 + 1e+06 alpha_ob33_seg2[0] <= 999999.371829801
 R1688: - C6 + 1e+06 alpha_ob33_seg2[0] <= 999999.371829801
 R1689: - C5 + 1e+06 alpha_ob33_seg2[1] <= 1.0000527483059706e+06
 R1690: - C7 + 1e+06 alpha_ob33_seg2[1] <= 1.0000527483059706e+06
 R1691: C4 + 1e+06 alpha_ob33_seg2[2] <= 999993.7281698009
 R1692: C6 + 1e+06 alpha_ob33_seg2[2] <= 999993.7281698009
 R1693: C5 + 1e+06 alpha_ob33_seg2[3] <= 999944.7017059706
 R1694: C7 + 1e+06 alpha_ob33_seg2[3] <= 999944.7017059706
 R1695: alpha_ob33_seg2[0] + alpha_ob33_seg2[1] + alpha_ob33_seg2[2]
   + alpha_ob33_seg2[3] >= 1
 R1696: - C4 + 1e+06 alpha_ob34_seg2[0] <= 1.0000016470956831e+06
 R1697: - C6 + 1e+06 alpha_ob34_seg2[0] <= 1.0000016470956831e+06
 R1698: - C5 + 1e+06 alpha_ob34_seg2[1] <= 1.0000598666742218e+06
 R1699: - C7 + 1e+06 alpha_ob34_seg2[1] <= 1.0000598666742218e+06
 R1700: C4 + 1e+06 alpha_ob34_seg2[2] <= 999994.0767956832
 R1701: C6 + 1e+06 alpha_ob34_seg2[2] <= 999994.0767956832
 R1702: C5 + 1e+06 alpha_ob34_seg2[3] <= 999937.4792742218
 R1703: C7 + 1e+06 alpha_ob34_seg2[3] <= 999937.4792742218
 R1704: alpha_ob34_seg2[0] + alpha_ob34_seg2[1] + alpha_ob34_seg2[2]
   + alpha_ob34_seg2[3] >= 1
 R1705: - C4 + 1e+06 alpha_ob35_seg2[0] <= 999999.3718601406
 R1706: - C6 + 1e+06 alpha_ob35_seg2[0] <= 999999.3718601406
 R1707: - C5 + 1e+06 alpha_ob35_seg2[1] <= 1.0000353982997246e+06
 R1708: - C7 + 1e+06 alpha_ob35_seg2[1] <= 1.0000353982997246e+06
 R1709: C4 + 1e+06 alpha_ob35_seg2[2] <= 999998.0782041405
 R1710: C6 + 1e+06 alpha_ob35_seg2[2] <= 999998.0782041405
 R1711: C5 + 1e+06 alpha_ob35_seg2[3] <= 999944.7016997245
 R1712: C7 + 1e+06 alpha_ob35_seg2[3] <= 999944.7016997245
 R1713: alpha_ob35_seg2[0] + alpha_ob35_seg2[1] + alpha_ob35_seg2[2]
   + alpha_ob35_seg2[3] >= 1
 R1714: - C4 + 1e+06 alpha_ob36_seg2[0] <= 999980.9374508709
 R1715: - C6 + 1e+06 alpha_ob36_seg2[0] <= 999980.9374508709
 R1716: - C5 + 1e+06 alpha_ob36_seg2[1] <= 1.0000348940002755e+06
 R1717: - C7 + 1e+06 alpha_ob36_seg2[1] <= 1.0000348940002755e+06
 R1718: C4 + 1e+06 alpha_ob36_seg2[2] <= 1.0000165124508708e+06
 R1719: C6 + 1e+06 alpha_ob36_seg2[2] <= 1.0000165124508708e+06
 R1720: C5 + 1e+06 alpha_ob36_seg2[3] <= 999935.9560002756
 R1721: C7 + 1e+06 alpha_ob36_seg2[3] <= 999935.9560002756
 R1722: alpha_ob36_seg2[0] + alpha_ob36_seg2[1] + alpha_ob36_seg2[2]
   + alpha_ob36_seg2[3] >= 1
 R1723: - C4 + 1e+06 alpha_ob37_seg2[0] <= 999980.937499801
 R1724: - C6 + 1e+06 alpha_ob37_seg2[0] <= 999980.937499801
 R1725: - C5 + 1e+06 alpha_ob37_seg2[1] <= 1.0000614940072973e+06
 R1726: - C7 + 1e+06 alpha_ob37_seg2[1] <= 1.0000614940072973e+06
 R1727: C4 + 1e+06 alpha_ob37_seg2[2] <= 1.000011162499801e+06
 R1728: C6 + 1e+06 alpha_ob37_seg2[2] <= 1.000011162499801e+06
 R1729: C5 + 1e+06 alpha_ob37_seg2[3] <= 999935.9560072974
 R1730: C7 + 1e+06 alpha_ob37_seg2[3] <= 999935.9560072974
 R1731: alpha_ob37_seg2[0] + alpha_ob37_seg2[1] + alpha_ob37_seg2[2]
   + alpha_ob37_seg2[3] >= 1
 R1732: - C4 + 1e+06 alpha_ob38_seg2[0] <= 999956.8394000001
 R1733: - C6 + 1e+06 alpha_ob38_seg2[0] <= 999956.8394000001
 R1734: - C5 + 1e+06 alpha_ob38_seg2[1] <= 999952.2463
 R1735: - C7 + 1e+06 alpha_ob38_seg2[1] <= 999952.2463
 R1736: C4 + 1e+06 alpha_ob38_seg2[2] <= 1.0000097606e+06
 R1737: C6 + 1e+06 alpha_ob38_seg2[2] <= 1.0000097606e+06
 R1738: C5 + 1e+06 alpha_ob38_seg2[3] <= 1.0000452037e+06
 R1739: C7 + 1e+06 alpha_ob38_seg2[3] <= 1.0000452037e+06
 R1740: alpha_ob38_seg2[0] + alpha_ob38_seg2[1] + alpha_ob38_seg2[2]
   + alpha_ob38_seg2[3] >= 1
 R1741: - C4 + 1e+06 alpha_ob39_seg2[0] <= 999964.9321250001
 R1742: - C6 + 1e+06 alpha_ob39_seg2[0] <= 999964.9321250001
 R1743: - C5 + 1e+06 alpha_ob39_seg2[1] <= 1.0000141893e+06
 R1744: - C7 + 1e+06 alpha_ob39_seg2[1] <= 1.0000141893e+06
 R1745: C4 + 1e+06 alpha_ob39_seg2[2] <= 1.000025197325e+06
 R1746: C6 + 1e+06 alpha_ob39_seg2[2] <= 1.000025197325e+06
 R1747: C5 + 1e+06 alpha_ob39_seg2[3] <= 999983.2607
 R1748: C7 + 1e+06 alpha_ob39_seg2[3] <= 999983.2607
 R1749: alpha_ob39_seg2[0] + alpha_ob39_seg2[1] + alpha_ob39_seg2[2]
   + alpha_ob39_seg2[3] >= 1
 R1750: - C4 + 1e+06 alpha_ob40_seg2[0] <= 999964.922208629
 R1751: - C6 + 1e+06 alpha_ob40_seg2[0] <= 999964.922208629
 R1752: - C5 + 1e+06 alpha_ob40_seg2[1] <= 1.0000141893002756e+06
 R1753: - C7 + 1e+06 alpha_ob40_seg2[1] <= 1.0000141893002756e+06
 R1754: C4 + 1e+06 alpha_ob40_seg2[2] <= 1.000032527608629e+06
 R1755: C6 + 1e+06 alpha_ob40_seg2[2] <= 1.000032527608629e+06
 R1756: C5 + 1e+06 alpha_ob40_seg2[3] <= 999933.6607002757
 R1757: C7 + 1e+06 alpha_ob40_seg2[3] <= 999933.6607002757
 R1758: alpha_ob40_seg2[0] + alpha_ob40_seg2[1] + alpha_ob40_seg2[2]
   + alpha_ob40_seg2[3] >= 1
 R1759: - C4 + 1e+06 alpha_ob41_seg2[0] <= 999972.166285402
 R1760: - C6 + 1e+06 alpha_ob41_seg2[0] <= 999972.166285402
 R1761: - C5 + 1e+06 alpha_ob41_seg2[1] <= 1.0000141604997247e+06
 R1762: - C7 + 1e+06 alpha_ob41_seg2[1] <= 1.0000141604997247e+06
 R1763: C4 + 1e+06 alpha_ob41_seg2[2] <= 1.000025283885402e+06
 R1764: C6 + 1e+06 alpha_ob41_seg2[2] <= 1.000025283885402e+06
 R1765: C5 + 1e+06 alpha_ob41_seg2[3] <= 999936.9394997247
 R1766: C7 + 1e+06 alpha_ob41_seg2[3] <= 999936.9394997247
 R1767: alpha_ob41_seg2[0] + alpha_ob41_seg2[1] + alpha_ob41_seg2[2]
   + alpha_ob41_seg2[3] >= 1
 R1768: - C4 + 1e+06 alpha_ob42_seg2[0] <= 999931.4945
 R1769: - C6 + 1e+06 alpha_ob42_seg2[0] <= 999931.4945
 R1770: - C5 + 1e+06 alpha_ob42_seg2[1] <= 1.0000158794e+06
 R1771: - C7 + 1e+06 alpha_ob42_seg2[1] <= 1.0000158794e+06
 R1772: C4 + 1e+06 alpha_ob42_seg2[2] <= 1.0000378554999999e+06
 R1773: C6 + 1e+06 alpha_ob42_seg2[2] <= 1.0000378554999999e+06
 R1774: C5 + 1e+06 alpha_ob42_seg2[3] <= 999981.5706
 R1775: C7 + 1e+06 alpha_ob42_seg2[3] <= 999981.5706
 R1776: alpha_ob42_seg2[0] + alpha_ob42_seg2[1] + alpha_ob42_seg2[2]
   + alpha_ob42_seg2[3] >= 1
 R1777: - C4 + 1e+06 alpha_ob43_seg2[0] <= 999916.6145
 R1778: - C6 + 1e+06 alpha_ob43_seg2[0] <= 999916.6145
 R1779: - C5 + 1e+06 alpha_ob43_seg2[1] <= 1.0000160176e+06
 R1780: - C7 + 1e+06 alpha_ob43_seg2[1] <= 1.0000160176e+06
 R1781: C4 + 1e+06 alpha_ob43_seg2[2] <= 1.0000707355e+06
 R1782: C6 + 1e+06 alpha_ob43_seg2[2] <= 1.0000707355e+06
 R1783: C5 + 1e+06 alpha_ob43_seg2[3] <= 999981.4324
 R1784: C7 + 1e+06 alpha_ob43_seg2[3] <= 999981.4324
 R1785: alpha_ob43_seg2[0] + alpha_ob43_seg2[1] + alpha_ob43_seg2[2]
   + alpha_ob43_seg2[3] >= 1
 R1786: - C4 + 1e+06 alpha_ob44_seg2[0] <= 999916.6143035784
 R1787: - C6 + 1e+06 alpha_ob44_seg2[0] <= 999916.6143035784
 R1788: - C5 + 1e+06 alpha_ob44_seg2[1] <= 1.0000160176002757e+06
 R1789: - C7 + 1e+06 alpha_ob44_seg2[1] <= 1.0000160176002757e+06
 R1790: C4 + 1e+06 alpha_ob44_seg2[2] <= 1.0000808355035784e+06
 R1791: C6 + 1e+06 alpha_ob44_seg2[2] <= 1.0000808355035784e+06
 R1792: C5 + 1e+06 alpha_ob44_seg2[3] <= 999929.0824002756
 R1793: C7 + 1e+06 alpha_ob44_seg2[3] <= 999929.0824002756
 R1794: alpha_ob44_seg2[0] + alpha_ob44_seg2[1] + alpha_ob44_seg2[2]
   + alpha_ob44_seg2[3] >= 1
 R1795: - C4 + 1e+06 alpha_ob45_seg2[0] <= 999980.9375348955
 R1796: - C6 + 1e+06 alpha_ob45_seg2[0] <= 999980.9375348955
 R1797: - C5 + 1e+06 alpha_ob45_seg2[1] <= 1.0000160439997246e+06
 R1798: - C7 + 1e+06 alpha_ob45_seg2[1] <= 1.0000160439997246e+06
 R1799: C4 + 1e+06 alpha_ob45_seg2[2] <= 1.0000165125348954e+06
 R1800: C6 + 1e+06 alpha_ob45_seg2[2] <= 1.0000165125348954e+06
 R1801: C5 + 1e+06 alpha_ob45_seg2[3] <= 999962.5559997245
 R1802: C7 + 1e+06 alpha_ob45_seg2[3] <= 999962.5559997245
 R1803: alpha_ob45_seg2[0] + alpha_ob45_seg2[1] + alpha_ob45_seg2[2]
   + alpha_ob45_seg2[3] >= 1
 R1804: - C4 + 1e+06 alpha_ob46_seg2[0] <= 999956.839399801
 R1805: - C6 + 1e+06 alpha_ob46_seg2[0] <= 999956.839399801
 R1806: - C5 + 1e+06 alpha_ob46_seg2[1] <= 999925.3963434525
 R1807: - C7 + 1e+06 alpha_ob46_seg2[1] <= 999925.3963434525
 R1808: C4 + 1e+06 alpha_ob46_seg2[2] <= 1.0000080105998011e+06
 R1809: C6 + 1e+06 alpha_ob46_seg2[2] <= 1.0000080105998011e+06
 R1810: C5 + 1e+06 alpha_ob46_seg2[3] <= 1.0000720537434525e+06
 R1811: C7 + 1e+06 alpha_ob46_seg2[3] <= 1.0000720537434525e+06
 R1812: alpha_ob46_seg2[0] + alpha_ob46_seg2[1] + alpha_ob46_seg2[2]
   + alpha_ob46_seg2[3] >= 1
 R1813: - C4 + 1e+06 alpha_ob47_seg2[0] <= 999980.9374998009
 R1814: - C6 + 1e+06 alpha_ob47_seg2[0] <= 999980.9374998009
 R1815: - C5 + 1e+06 alpha_ob47_seg2[1] <= 1.0000160440053071e+06
 R1816: - C7 + 1e+06 alpha_ob47_seg2[1] <= 1.0000160440053071e+06
 R1817: C4 + 1e+06 alpha_ob47_seg2[2] <= 1.000012662499801e+06
 R1818: C6 + 1e+06 alpha_ob47_seg2[2] <= 1.000012662499801e+06
 R1819: C5 + 1e+06 alpha_ob47_seg2[3] <= 999981.4060053072
 R1820: C7 + 1e+06 alpha_ob47_seg2[3] <= 999981.4060053072
 R1821: alpha_ob47_seg2[0] + alpha_ob47_seg2[1] + alpha_ob47_seg2[2]
   + alpha_ob47_seg2[3] >= 1
 R1822: - C4 + 1e+06 alpha_ob48_seg2[0] <= 999983.6053459151
 R1823: - C6 + 1e+06 alpha_ob48_seg2[0] <= 999983.6053459151
 R1824: - C5 + 1e+06 alpha_ob48_seg2[1] <= 999987.7722497246
 R1825: - C7 + 1e+06 alpha_ob48_seg2[1] <= 999987.7722497246
 R1826: C4 + 1e+06 alpha_ob48_seg2[2] <= 1.000013844745915e+06
 R1827: C6 + 1e+06 alpha_ob48_seg2[2] <= 1.000013844745915e+06
 R1828: C5 + 1e+06 alpha_ob48_seg2[3] <= 999984.8277497246
 R1829: C7 + 1e+06 alpha_ob48_seg2[3] <= 999984.8277497246
 R1830: alpha_ob48_seg2[0] + alpha_ob48_seg2[1] + alpha_ob48_seg2[2]
   + alpha_ob48_seg2[3] >= 1
 R1831: - C4 + 1e+06 alpha_ob49_seg2[0] <= 999958.8717898987
 R1832: - C6 + 1e+06 alpha_ob49_seg2[0] <= 999958.8717898987
 R1833: - C5 + 1e+06 alpha_ob49_seg2[1] <= 999987.7837602755
 R1834: - C7 + 1e+06 alpha_ob49_seg2[1] <= 999987.7837602755
 R1835: C4 + 1e+06 alpha_ob49_seg2[2] <= 1.0000385781898986e+06
 R1836: C6 + 1e+06 alpha_ob49_seg2[2] <= 1.0000385781898986e+06
 R1837: C5 + 1e+06 alpha_ob49_seg2[3] <= 1.0000043162402755e+06
 R1838: C7 + 1e+06 alpha_ob49_seg2[3] <= 1.0000043162402755e+06
 R1839: alpha_ob49_seg2[0] + alpha_ob49_seg2[1] + alpha_ob49_seg2[2]
   + alpha_ob49_seg2[3] >= 1
 R1840: - C4 + 1e+06 alpha_ob50_seg2[0] <= 999939.2718
 R1841: - C6 + 1e+06 alpha_ob50_seg2[0] <= 999939.2718
 R1842: - C5 + 1e+06 alpha_ob50_seg2[1] <= 999993.13376
 R1843: - C7 + 1e+06 alpha_ob50_seg2[1] <= 999993.13376
 R1844: C4 + 1e+06 alpha_ob50_seg2[2] <= 1.0000385782e+06
 R1845: C6 + 1e+06 alpha_ob50_seg2[2] <= 1.0000385782e+06
 R1846: C5 + 1e+06 alpha_ob50_seg2[3] <= 1.00000431624e+06
 R1847: C7 + 1e+06 alpha_ob50_seg2[3] <= 1.00000431624e+06
 R1848: alpha_ob50_seg2[0] + alpha_ob50_seg2[1] + alpha_ob50_seg2[2]
   + alpha_ob50_seg2[3] >= 1
 R1849: - C4 + 1e+06 alpha_ob51_seg2[0] <= 999939.2718101013
 R1850: - C6 + 1e+06 alpha_ob51_seg2[0] <= 999939.2718101013
 R1851: - C5 + 1e+06 alpha_ob51_seg2[1] <= 999987.7837597246
 R1852: - C7 + 1e+06 alpha_ob51_seg2[1] <= 999987.7837597246
 R1853: C4 + 1e+06 alpha_ob51_seg2[2] <= 1.0000581782101013e+06
 R1854: C6 + 1e+06 alpha_ob51_seg2[2] <= 1.0000581782101013e+06
 R1855: C5 + 1e+06 alpha_ob51_seg2[3] <= 1.0000043162397245e+06
 R1856: C7 + 1e+06 alpha_ob51_seg2[3] <= 1.0000043162397245e+06
 R1857: alpha_ob51_seg2[0] + alpha_ob51_seg2[1] + alpha_ob51_seg2[2]
   + alpha_ob51_seg2[3] >= 1
 R1858: - C4 + 1e+06 alpha_ob52_seg2[0] <= 999939.271799801
 R1859: - C6 + 1e+06 alpha_ob52_seg2[0] <= 999939.271799801
 R1860: - C5 + 1e+06 alpha_ob52_seg2[1] <= 999987.7838262042
 R1861: - C7 + 1e+06 alpha_ob52_seg2[1] <= 999987.7838262042
 R1862: C4 + 1e+06 alpha_ob52_seg2[2] <= 1.000038578199801e+06
 R1863: C6 + 1e+06 alpha_ob52_seg2[2] <= 1.000038578199801e+06
 R1864: C5 + 1e+06 alpha_ob52_seg2[3] <= 1.0000096662262043e+06
 R1865: C7 + 1e+06 alpha_ob52_seg2[3] <= 1.0000096662262043e+06
 R1866: alpha_ob52_seg2[0] + alpha_ob52_seg2[1] + alpha_ob52_seg2[2]
   + alpha_ob52_seg2[3] >= 1
 R1867: - C4 + 1e+06 alpha_ob53_seg2[0] <= 999926.0708
 R1868: - C6 + 1e+06 alpha_ob53_seg2[0] <= 999926.0708
 R1869: - C5 + 1e+06 alpha_ob53_seg2[1] <= 999983.7196
 R1870: - C7 + 1e+06 alpha_ob53_seg2[1] <= 999983.7196
 R1871: C4 + 1e+06 alpha_ob53_seg2[2] <= 1.0000250292e+06
 R1872: C6 + 1e+06 alpha_ob53_seg2[2] <= 1.0000250292e+06
 R1873: C5 + 1e+06 alpha_ob53_seg2[3] <= 1.0000137304e+06
 R1874: C7 + 1e+06 alpha_ob53_seg2[3] <= 1.0000137304e+06
 R1875: alpha_ob53_seg2[0] + alpha_ob53_seg2[1] + alpha_ob53_seg2[2]
   + alpha_ob53_seg2[3] >= 1
 R1876: - C4 + 1e+06 alpha_ob54_seg2[0] <= 999926.0708293857
 R1877: - C6 + 1e+06 alpha_ob54_seg2[0] <= 999926.0708293857
 R1878: - C5 + 1e+06 alpha_ob54_seg2[1] <= 999967.8695997245
 R1879: - C7 + 1e+06 alpha_ob54_seg2[1] <= 999967.8695997245
 R1880: C4 + 1e+06 alpha_ob54_seg2[2] <= 1.0000713792293856e+06
 R1881: C6 + 1e+06 alpha_ob54_seg2[2] <= 1.0000713792293856e+06
 R1882: C5 + 1e+06 alpha_ob54_seg2[3] <= 1.0000137303997246e+06
 R1883: C7 + 1e+06 alpha_ob54_seg2[3] <= 1.0000137303997246e+06
 R1884: alpha_ob54_seg2[0] + alpha_ob54_seg2[1] + alpha_ob54_seg2[2]
   + alpha_ob54_seg2[3] >= 1
 R1885: - C4 + 1e+06 alpha_ob55_seg2[0] <= 999972.6917508709
 R1886: - C6 + 1e+06 alpha_ob55_seg2[0] <= 999972.6917508709
 R1887: - C5 + 1e+06 alpha_ob55_seg2[1] <= 999957.1644002756
 R1888: - C7 + 1e+06 alpha_ob55_seg2[1] <= 999957.1644002756
 R1889: C4 + 1e+06 alpha_ob55_seg2[2] <= 1.0000247581508709e+06
 R1890: C6 + 1e+06 alpha_ob55_seg2[2] <= 1.0000247581508709e+06
 R1891: C5 + 1e+06 alpha_ob55_seg2[3] <= 1.0000136856002755e+06
 R1892: C7 + 1e+06 alpha_ob55_seg2[3] <= 1.0000136856002755e+06
 R1893: alpha_ob55_seg2[0] + alpha_ob55_seg2[1] + alpha_ob55_seg2[2]
   + alpha_ob55_seg2[3] >= 1
 R1894: - C4 + 1e+06 alpha_ob56_seg2[0] <= 999939.0009674003
 R1895: - C6 + 1e+06 alpha_ob56_seg2[0] <= 999939.0009674003
 R1896: - C5 + 1e+06 alpha_ob56_seg2[1] <= 999961.9516002756
 R1897: - C7 + 1e+06 alpha_ob56_seg2[1] <= 999961.9516002756
 R1898: C4 + 1e+06 alpha_ob56_seg2[2] <= 1.0000584489674004e+06
 R1899: C6 + 1e+06 alpha_ob56_seg2[2] <= 1.0000584489674004e+06
 R1900: C5 + 1e+06 alpha_ob56_seg2[3] <= 1.0000178984002755e+06
 R1901: C7 + 1e+06 alpha_ob56_seg2[3] <= 1.0000178984002755e+06
 R1902: alpha_ob56_seg2[0] + alpha_ob56_seg2[1] + alpha_ob56_seg2[2]
   + alpha_ob56_seg2[3] >= 1
 R1903: - C4 + 1e+06 alpha_ob57_seg2[0] <= 999994.9164998011
 R1904: - C6 + 1e+06 alpha_ob57_seg2[0] <= 999994.9164998011
 R1905: - C5 + 1e+06 alpha_ob57_seg2[1] <= 999925.4284782809
 R1906: - C7 + 1e+06 alpha_ob57_seg2[1] <= 999925.4284782809
 R1907: C4 + 1e+06 alpha_ob57_seg2[2] <= 999943.6834998011
 R1908: C6 + 1e+06 alpha_ob57_seg2[2] <= 999943.6834998011
 R1909: C5 + 1e+06 alpha_ob57_seg2[3] <= 1.0000720216782809e+06
 R1910: C7 + 1e+06 alpha_ob57_seg2[3] <= 1.0000720216782809e+06
 R1911: alpha_ob57_seg2[0] + alpha_ob57_seg2[1] + alpha_ob57_seg2[2]
   + alpha_ob57_seg2[3] >= 1
 R1912: - C4 + 1e+06 alpha_ob58_seg2[0] <= 999920.7556087238
 R1913: - C6 + 1e+06 alpha_ob58_seg2[0] <= 999920.7556087238
 R1914: - C5 + 1e+06 alpha_ob58_seg2[1] <= 1.0000111845997245e+06
 R1915: - C7 + 1e+06 alpha_ob58_seg2[1] <= 1.0000111845997245e+06
 R1916: C4 + 1e+06 alpha_ob58_seg2[2] <= 1.0000766944087239e+06
 R1917: C6 + 1e+06 alpha_ob58_seg2[2] <= 1.0000766944087239e+06
 R1918: C5 + 1e+06 alpha_ob58_seg2[3] <= 999981.6653997245
 R1919: C7 + 1e+06 alpha_ob58_seg2[3] <= 999981.6653997245
 R1920: alpha_ob58_seg2[0] + alpha_ob58_seg2[1] + alpha_ob58_seg2[2]
   + alpha_ob58_seg2[3] >= 1
 R1921: - C4 + 1e+06 alpha_ob59_seg2[0] <= 999911.9056000001
 R1922: - C6 + 1e+06 alpha_ob59_seg2[0] <= 999911.9056000001
 R1923: - C5 + 1e+06 alpha_ob59_seg2[1] <= 1.0000111846e+06
 R1924: - C7 + 1e+06 alpha_ob59_seg2[1] <= 1.0000111846e+06
 R1925: C4 + 1e+06 alpha_ob59_seg2[2] <= 1.0000766944e+06
 R1926: C6 + 1e+06 alpha_ob59_seg2[2] <= 1.0000766944e+06
 R1927: C5 + 1e+06 alpha_ob59_seg2[3] <= 999986.2654
 R1928: C7 + 1e+06 alpha_ob59_seg2[3] <= 999986.2654
 R1929: alpha_ob59_seg2[0] + alpha_ob59_seg2[1] + alpha_ob59_seg2[2]
   + alpha_ob59_seg2[3] >= 1
 R1930: - C4 + 1e+06 alpha_ob60_seg2[0] <= 999914.52327842
 R1931: - C6 + 1e+06 alpha_ob60_seg2[0] <= 999914.52327842
 R1932: - C5 + 1e+06 alpha_ob60_seg2[1] <= 999993.6552362755
 R1933: - C7 + 1e+06 alpha_ob60_seg2[1] <= 999993.6552362755
 R1934: C4 + 1e+06 alpha_ob60_seg2[2] <= 1.0000829266784199e+06
 R1935: C6 + 1e+06 alpha_ob60_seg2[2] <= 1.0000829266784199e+06
 R1936: C5 + 1e+06 alpha_ob60_seg2[3] <= 999992.1947642756
 R1937: C7 + 1e+06 alpha_ob60_seg2[3] <= 999992.1947642756
 R1938: alpha_ob60_seg2[0] + alpha_ob60_seg2[1] + alpha_ob60_seg2[2]
   + alpha_ob60_seg2[3] >= 1
 R1939: - C4 + 1e+06 alpha_ob61_seg2[0] <= 999912.265199801
 R1940: - C6 + 1e+06 alpha_ob61_seg2[0] <= 999912.265199801
 R1941: - C5 + 1e+06 alpha_ob61_seg2[1] <= 999987.7842112777
 R1942: - C7 + 1e+06 alpha_ob61_seg2[1] <= 999987.7842112777
 R1943: C4 + 1e+06 alpha_ob61_seg2[2] <= 1.000076834799801e+06
 R1944: C6 + 1e+06 alpha_ob61_seg2[2] <= 1.000076834799801e+06
 R1945: C5 + 1e+06 alpha_ob61_seg2[3] <= 1.0000096658112778e+06
 R1946: C7 + 1e+06 alpha_ob61_seg2[3] <= 1.0000096658112778e+06
 R1947: alpha_ob61_seg2[0] + alpha_ob61_seg2[1] + alpha_ob61_seg2[2]
   + alpha_ob61_seg2[3] >= 1
 R1948: - C4 + 1e+06 alpha_ob62_seg2[0] <= 999920.6152119379
 R1949: - C6 + 1e+06 alpha_ob62_seg2[0] <= 999920.6152119379
 R1950: - C5 + 1e+06 alpha_ob62_seg2[1] <= 999981.4341997246
 R1951: - C7 + 1e+06 alpha_ob62_seg2[1] <= 999981.4341997246
 R1952: C4 + 1e+06 alpha_ob62_seg2[2] <= 1.0000768348119379e+06
 R1953: C6 + 1e+06 alpha_ob62_seg2[2] <= 1.0000768348119379e+06
 R1954: C5 + 1e+06 alpha_ob62_seg2[3] <= 1.0000096657997245e+06
 R1955: C7 + 1e+06 alpha_ob62_seg2[3] <= 1.0000096657997245e+06
 R1956: alpha_ob62_seg2[0] + alpha_ob62_seg2[1] + alpha_ob62_seg2[2]
   + alpha_ob62_seg2[3] >= 1
 R1957: - C4 + 1e+06 alpha_ob63_seg2[0] <= 999919.2652
 R1958: - C6 + 1e+06 alpha_ob63_seg2[0] <= 999919.2652
 R1959: - C5 + 1e+06 alpha_ob63_seg2[1] <= 999981.4342
 R1960: - C7 + 1e+06 alpha_ob63_seg2[1] <= 999981.4342
 R1961: C4 + 1e+06 alpha_ob63_seg2[2] <= 1.0000768348e+06
 R1962: C6 + 1e+06 alpha_ob63_seg2[2] <= 1.0000768348e+06
 R1963: C5 + 1e+06 alpha_ob63_seg2[3] <= 1.0000160158000001e+06
 R1964: C7 + 1e+06 alpha_ob63_seg2[3] <= 1.0000160158000001e+06
 R1965: alpha_ob63_seg2[0] + alpha_ob63_seg2[1] + alpha_ob63_seg2[2]
   + alpha_ob63_seg2[3] >= 1
 R1966: - C4 + 1e+06 alpha_ob64_seg2[0] <= 999919.2652454559
 R1967: - C6 + 1e+06 alpha_ob64_seg2[0] <= 999919.2652454559
 R1968: - C5 + 1e+06 alpha_ob64_seg2[1] <= 999956.8341997246
 R1969: - C7 + 1e+06 alpha_ob64_seg2[1] <= 999956.8341997246
 R1970: C4 + 1e+06 alpha_ob64_seg2[2] <= 1.0000781848454559e+06
 R1971: C6 + 1e+06 alpha_ob64_seg2[2] <= 1.0000781848454559e+06
 R1972: C5 + 1e+06 alpha_ob64_seg2[3] <= 1.0000160157997246e+06
 R1973: C7 + 1e+06 alpha_ob64_seg2[3] <= 1.0000160157997246e+06
 R1974: alpha_ob64_seg2[0] + alpha_ob64_seg2[1] + alpha_ob64_seg2[2]
   + alpha_ob64_seg2[3] >= 1
 R1975: - C4 + 1e+06 alpha_ob65_seg2[0] <= 1.0000081732920931e+06
 R1976: - C6 + 1e+06 alpha_ob65_seg2[0] <= 1.0000081732920931e+06
 R1977: - C5 + 1e+06 alpha_ob65_seg2[1] <= 999925.4331002756
 R1978: - C7 + 1e+06 alpha_ob65_seg2[1] <= 999925.4331002756
 R1979: C4 + 1e+06 alpha_ob65_seg2[2] <= 999989.2766720931
 R1980: C6 + 1e+06 alpha_ob65_seg2[2] <= 999989.2766720931
 R1981: C5 + 1e+06 alpha_ob65_seg2[3] <= 1.0000624169002755e+06
 R1982: C7 + 1e+06 alpha_ob65_seg2[3] <= 1.0000624169002755e+06
 R1983: alpha_ob65_seg2[0] + alpha_ob65_seg2[1] + alpha_ob65_seg2[2]
   + alpha_ob65_seg2[3] >= 1
 R1984: - C4 + 1e+06 alpha_ob66_seg2[0] <= 1.0000146737e+06
 R1985: - C6 + 1e+06 alpha_ob66_seg2[0] <= 1.0000146737e+06
 R1986: - C5 + 1e+06 alpha_ob66_seg2[1] <= 999940.7624
 R1987: - C7 + 1e+06 alpha_ob66_seg2[1] <= 999940.7624
 R1988: C4 + 1e+06 alpha_ob66_seg2[2] <= 999948.4263000001
 R1989: C6 + 1e+06 alpha_ob66_seg2[2] <= 999948.4263000001
 R1990: C5 + 1e+06 alpha_ob66_seg2[3] <= 1.0000566875999999e+06
 R1991: C7 + 1e+06 alpha_ob66_seg2[3] <= 1.0000566875999999e+06
 R1992: alpha_ob66_seg2[0] + alpha_ob66_seg2[1] + alpha_ob66_seg2[2]
   + alpha_ob66_seg2[3] >= 1
 R1993: - C4 + 1e+06 alpha_ob67_seg2[0] <= 1.0000058189e+06
 R1994: - C6 + 1e+06 alpha_ob67_seg2[0] <= 1.0000058189e+06
 R1995: - C5 + 1e+06 alpha_ob67_seg2[1] <= 999940.5469
 R1996: - C7 + 1e+06 alpha_ob67_seg2[1] <= 999940.5469
 R1997: C4 + 1e+06 alpha_ob67_seg2[2] <= 999985.5311
 R1998: C6 + 1e+06 alpha_ob67_seg2[2] <= 999985.5311
 R1999: C5 + 1e+06 alpha_ob67_seg2[3] <= 1.0000569031e+06
 R2000: C7 + 1e+06 alpha_ob67_seg2[3] <= 1.0000569031e+06
 R2001: alpha_ob67_seg2[0] + alpha_ob67_seg2[1] + alpha_ob67_seg2[2]
   + alpha_ob67_seg2[3] >= 1
 R2002: - C4 + 1e+06 alpha_ob68_seg2[0] <= 1.0000058188284199e+06
 R2003: - C6 + 1e+06 alpha_ob68_seg2[0] <= 1.0000058188284199e+06
 R2004: - C5 + 1e+06 alpha_ob68_seg2[1] <= 999940.5469002755
 R2005: - C7 + 1e+06 alpha_ob68_seg2[1] <= 999940.5469002755
 R2006: C4 + 1e+06 alpha_ob68_seg2[2] <= 999991.6311284199
 R2007: C6 + 1e+06 alpha_ob68_seg2[2] <= 999991.6311284199
 R2008: C5 + 1e+06 alpha_ob68_seg2[3] <= 1.0000453031002756e+06
 R2009: C7 + 1e+06 alpha_ob68_seg2[3] <= 1.0000453031002756e+06
 R2010: alpha_ob68_seg2[0] + alpha_ob68_seg2[1] + alpha_ob68_seg2[2]
   + alpha_ob68_seg2[3] >= 1
 R2011: - C4 + 1e+06 alpha_ob69_seg2[0] <= 999996.9688499999
 R2012: - C6 + 1e+06 alpha_ob69_seg2[0] <= 999996.9688499999
 R2013: - C5 + 1e+06 alpha_ob69_seg2[1] <= 999940.5469
 R2014: - C7 + 1e+06 alpha_ob69_seg2[1] <= 999940.5469
 R2015: C4 + 1e+06 alpha_ob69_seg2[2] <= 999991.63115
 R2016: C6 + 1e+06 alpha_ob69_seg2[2] <= 999991.63115
 R2017: C5 + 1e+06 alpha_ob69_seg2[3] <= 1.0000569031e+06
 R2018: C7 + 1e+06 alpha_ob69_seg2[3] <= 1.0000569031e+06
 R2019: alpha_ob69_seg2[0] + alpha_ob69_seg2[1] + alpha_ob69_seg2[2]
   + alpha_ob69_seg2[3] >= 1
 R2020: - C4 + 1e+06 alpha_ob70_seg2[0] <= 999991.83683
 R2021: - C6 + 1e+06 alpha_ob70_seg2[0] <= 999991.83683
 R2022: - C5 + 1e+06 alpha_ob70_seg2[1] <= 999940.589
 R2023: - C7 + 1e+06 alpha_ob70_seg2[1] <= 999940.589
 R2024: C4 + 1e+06 alpha_ob70_seg2[2] <= 1.00000401317e+06
 R2025: C6 + 1e+06 alpha_ob70_seg2[2] <= 1.00000401317e+06
 R2026: C5 + 1e+06 alpha_ob70_seg2[3] <= 1.000056861e+06
 R2027: C7 + 1e+06 alpha_ob70_seg2[3] <= 1.000056861e+06
 R2028: alpha_ob70_seg2[0] + alpha_ob70_seg2[1] + alpha_ob70_seg2[2]
   + alpha_ob70_seg2[3] >= 1
 R2029: - C4 + 1e+06 alpha_ob71_seg2[0] <= 999991.8368084199
 R2030: - C6 + 1e+06 alpha_ob71_seg2[0] <= 999991.8368084199
 R2031: - C5 + 1e+06 alpha_ob71_seg2[1] <= 999940.5890002755
 R2032: - C7 + 1e+06 alpha_ob71_seg2[1] <= 999940.5890002755
 R2033: C4 + 1e+06 alpha_ob71_seg2[2] <= 1.0000056131484199e+06
 R2034: C6 + 1e+06 alpha_ob71_seg2[2] <= 1.0000056131484199e+06
 R2035: C5 + 1e+06 alpha_ob71_seg2[3] <= 1.0000452610002755e+06
 R2036: C7 + 1e+06 alpha_ob71_seg2[3] <= 1.0000452610002755e+06
 R2037: alpha_ob71_seg2[0] + alpha_ob71_seg2[1] + alpha_ob71_seg2[2]
   + alpha_ob71_seg2[3] >= 1
 R2038: - C4 + 1e+06 alpha_ob72_seg2[0] <= 1.0000764021508709e+06
 R2039: - C6 + 1e+06 alpha_ob72_seg2[0] <= 1.0000764021508709e+06
 R2040: - C5 + 1e+06 alpha_ob72_seg2[1] <= 999957.1027002756
 R2041: - C7 + 1e+06 alpha_ob72_seg2[1] <= 999957.1027002756
 R2042: C4 + 1e+06 alpha_ob72_seg2[2] <= 999921.0477508709
 R2043: C6 + 1e+06 alpha_ob72_seg2[2] <= 999921.0477508709
 R2044: C5 + 1e+06 alpha_ob72_seg2[3] <= 1.0000137473002756e+06
 R2045: C7 + 1e+06 alpha_ob72_seg2[3] <= 1.0000137473002756e+06
 R2046: alpha_ob72_seg2[0] + alpha_ob72_seg2[1] + alpha_ob72_seg2[2]
   + alpha_ob72_seg2[3] >= 1
 R2047: - C4 + 1e+06 alpha_ob73_seg2[0] <= 1.0000445522e+06
 R2048: - C6 + 1e+06 alpha_ob73_seg2[0] <= 1.0000445522e+06
 R2049: - C5 + 1e+06 alpha_ob73_seg2[1] <= 999983.7027
 R2050: - C7 + 1e+06 alpha_ob73_seg2[1] <= 999983.7027
 R2051: C4 + 1e+06 alpha_ob73_seg2[2] <= 999921.0477999999
 R2052: C6 + 1e+06 alpha_ob73_seg2[2] <= 999921.0477999999
 R2053: C5 + 1e+06 alpha_ob73_seg2[3] <= 1.0000137473e+06
 R2054: C7 + 1e+06 alpha_ob73_seg2[3] <= 1.0000137473e+06
 R2055: alpha_ob73_seg2[0] + alpha_ob73_seg2[1] + alpha_ob73_seg2[2]
   + alpha_ob73_seg2[3] >= 1
 R2056: - C4 + 1e+06 alpha_ob74_seg2[0] <= 1.0000445522183661e+06
 R2057: - C6 + 1e+06 alpha_ob74_seg2[0] <= 1.0000445522183661e+06
 R2058: - C5 + 1e+06 alpha_ob74_seg2[1] <= 999973.8526997245
 R2059: - C7 + 1e+06 alpha_ob74_seg2[1] <= 999973.8526997245
 R2060: C4 + 1e+06 alpha_ob74_seg2[2] <= 999952.8978183661
 R2061: C6 + 1e+06 alpha_ob74_seg2[2] <= 999952.8978183661
 R2062: C5 + 1e+06 alpha_ob74_seg2[3] <= 1.0000137472997245e+06
 R2063: C7 + 1e+06 alpha_ob74_seg2[3] <= 1.0000137472997245e+06
 R2064: alpha_ob74_seg2[0] + alpha_ob74_seg2[1] + alpha_ob74_seg2[2]
   + alpha_ob74_seg2[3] >= 1
 R2065: - C4 + 1e+06 alpha_ob75_seg2[0] <= 1.000064332081634e+06
 R2066: - C6 + 1e+06 alpha_ob75_seg2[0] <= 1.000064332081634e+06
 R2067: - C5 + 1e+06 alpha_ob75_seg2[1] <= 999973.9401002756
 R2068: - C7 + 1e+06 alpha_ob75_seg2[1] <= 999973.9401002756
 R2069: C4 + 1e+06 alpha_ob75_seg2[2] <= 999933.1178816339
 R2070: C6 + 1e+06 alpha_ob75_seg2[2] <= 999933.1178816339
 R2071: C5 + 1e+06 alpha_ob75_seg2[3] <= 1.0000136599002755e+06
 R2072: C7 + 1e+06 alpha_ob75_seg2[3] <= 1.0000136599002755e+06
 R2073: alpha_ob75_seg2[0] + alpha_ob75_seg2[1] + alpha_ob75_seg2[2]
   + alpha_ob75_seg2[3] >= 1
 R2074: - C4 + 1e+06 alpha_ob76_seg2[0] <= 1.0000643321013774e+06
 R2075: - C6 + 1e+06 alpha_ob76_seg2[0] <= 1.0000643321013774e+06
 R2076: - C5 + 1e+06 alpha_ob76_seg2[1] <= 999983.1900997246
 R2077: - C7 + 1e+06 alpha_ob76_seg2[1] <= 999983.1900997246
 R2078: C4 + 1e+06 alpha_ob76_seg2[2] <= 999933.1179013775
 R2079: C6 + 1e+06 alpha_ob76_seg2[2] <= 999933.1179013775
 R2080: C5 + 1e+06 alpha_ob76_seg2[3] <= 1.0000136598997245e+06
 R2081: C7 + 1e+06 alpha_ob76_seg2[3] <= 1.0000136598997245e+06
 R2082: alpha_ob76_seg2[0] + alpha_ob76_seg2[1] + alpha_ob76_seg2[2]
   + alpha_ob76_seg2[3] >= 1
 R2083: C0 >= 0
 R2084: C1 >= 0
 R2085: C0 <= 85
 R2086: C1 <= 85
 R2087: C2 >= 0
 R2088: C3 >= 0
 R2089: C2 <= 85
 R2090: C3 <= 85
 R2091: C4 >= 0
 R2092: C5 >= 0
 R2093: C4 <= 85
 R2094: C5 <= 85
 R2095: C6 >= 0
 R2096: C7 >= 0
 R2097: C6 <= 85
 R2098: C7 <= 85
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
 alpha_ob70_seg0[3] alpha_ob71_seg0[0] alpha_ob71_seg0[1]
 alpha_ob71_seg0[2] alpha_ob71_seg0[3] alpha_ob72_seg0[0]
 alpha_ob72_seg0[1] alpha_ob72_seg0[2] alpha_ob72_seg0[3]
 alpha_ob73_seg0[0] alpha_ob73_seg0[1] alpha_ob73_seg0[2]
 alpha_ob73_seg0[3] alpha_ob74_seg0[0] alpha_ob74_seg0[1]
 alpha_ob74_seg0[2] alpha_ob74_seg0[3] alpha_ob75_seg0[0]
 alpha_ob75_seg0[1] alpha_ob75_seg0[2] alpha_ob75_seg0[3]
 alpha_ob76_seg0[0] alpha_ob76_seg0[1] alpha_ob76_seg0[2]
 alpha_ob76_seg0[3] alpha_ob0_seg1[0] alpha_ob0_seg1[1] alpha_ob0_seg1[2]
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
 alpha_ob70_seg1[3] alpha_ob71_seg1[0] alpha_ob71_seg1[1]
 alpha_ob71_seg1[2] alpha_ob71_seg1[3] alpha_ob72_seg1[0]
 alpha_ob72_seg1[1] alpha_ob72_seg1[2] alpha_ob72_seg1[3]
 alpha_ob73_seg1[0] alpha_ob73_seg1[1] alpha_ob73_seg1[2]
 alpha_ob73_seg1[3] alpha_ob74_seg1[0] alpha_ob74_seg1[1]
 alpha_ob74_seg1[2] alpha_ob74_seg1[3] alpha_ob75_seg1[0]
 alpha_ob75_seg1[1] alpha_ob75_seg1[2] alpha_ob75_seg1[3]
 alpha_ob76_seg1[0] alpha_ob76_seg1[1] alpha_ob76_seg1[2]
 alpha_ob76_seg1[3] alpha_ob0_seg2[0] alpha_ob0_seg2[1] alpha_ob0_seg2[2]
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
 alpha_ob34_seg2[3] alpha_ob35_seg2[0] alpha_ob35_seg2[1]
 alpha_ob35_seg2[2] alpha_ob35_seg2[3] alpha_ob36_seg2[0]
 alpha_ob36_seg2[1] alpha_ob36_seg2[2] alpha_ob36_seg2[3]
 alpha_ob37_seg2[0] alpha_ob37_seg2[1] alpha_ob37_seg2[2]
 alpha_ob37_seg2[3] alpha_ob38_seg2[0] alpha_ob38_seg2[1]
 alpha_ob38_seg2[2] alpha_ob38_seg2[3] alpha_ob39_seg2[0]
 alpha_ob39_seg2[1] alpha_ob39_seg2[2] alpha_ob39_seg2[3]
 alpha_ob40_seg2[0] alpha_ob40_seg2[1] alpha_ob40_seg2[2]
 alpha_ob40_seg2[3] alpha_ob41_seg2[0] alpha_ob41_seg2[1]
 alpha_ob41_seg2[2] alpha_ob41_seg2[3] alpha_ob42_seg2[0]
 alpha_ob42_seg2[1] alpha_ob42_seg2[2] alpha_ob42_seg2[3]
 alpha_ob43_seg2[0] alpha_ob43_seg2[1] alpha_ob43_seg2[2]
 alpha_ob43_seg2[3] alpha_ob44_seg2[0] alpha_ob44_seg2[1]
 alpha_ob44_seg2[2] alpha_ob44_seg2[3] alpha_ob45_seg2[0]
 alpha_ob45_seg2[1] alpha_ob45_seg2[2] alpha_ob45_seg2[3]
 alpha_ob46_seg2[0] alpha_ob46_seg2[1] alpha_ob46_seg2[2]
 alpha_ob46_seg2[3] alpha_ob47_seg2[0] alpha_ob47_seg2[1]
 alpha_ob47_seg2[2] alpha_ob47_seg2[3] alpha_ob48_seg2[0]
 alpha_ob48_seg2[1] alpha_ob48_seg2[2] alpha_ob48_seg2[3]
 alpha_ob49_seg2[0] alpha_ob49_seg2[1] alpha_ob49_seg2[2]
 alpha_ob49_seg2[3] alpha_ob50_seg2[0] alpha_ob50_seg2[1]
 alpha_ob50_seg2[2] alpha_ob50_seg2[3] alpha_ob51_seg2[0]
 alpha_ob51_seg2[1] alpha_ob51_seg2[2] alpha_ob51_seg2[3]
 alpha_ob52_seg2[0] alpha_ob52_seg2[1] alpha_ob52_seg2[2]
 alpha_ob52_seg2[3] alpha_ob53_seg2[0] alpha_ob53_seg2[1]
 alpha_ob53_seg2[2] alpha_ob53_seg2[3] alpha_ob54_seg2[0]
 alpha_ob54_seg2[1] alpha_ob54_seg2[2] alpha_ob54_seg2[3]
 alpha_ob55_seg2[0] alpha_ob55_seg2[1] alpha_ob55_seg2[2]
 alpha_ob55_seg2[3] alpha_ob56_seg2[0] alpha_ob56_seg2[1]
 alpha_ob56_seg2[2] alpha_ob56_seg2[3] alpha_ob57_seg2[0]
 alpha_ob57_seg2[1] alpha_ob57_seg2[2] alpha_ob57_seg2[3]
 alpha_ob58_seg2[0] alpha_ob58_seg2[1] alpha_ob58_seg2[2]
 alpha_ob58_seg2[3] alpha_ob59_seg2[0] alpha_ob59_seg2[1]
 alpha_ob59_seg2[2] alpha_ob59_seg2[3] alpha_ob60_seg2[0]
 alpha_ob60_seg2[1] alpha_ob60_seg2[2] alpha_ob60_seg2[3]
 alpha_ob61_seg2[0] alpha_ob61_seg2[1] alpha_ob61_seg2[2]
 alpha_ob61_seg2[3] alpha_ob62_seg2[0] alpha_ob62_seg2[1]
 alpha_ob62_seg2[2] alpha_ob62_seg2[3] alpha_ob63_seg2[0]
 alpha_ob63_seg2[1] alpha_ob63_seg2[2] alpha_ob63_seg2[3]
 alpha_ob64_seg2[0] alpha_ob64_seg2[1] alpha_ob64_seg2[2]
 alpha_ob64_seg2[3] alpha_ob65_seg2[0] alpha_ob65_seg2[1]
 alpha_ob65_seg2[2] alpha_ob65_seg2[3] alpha_ob66_seg2[0]
 alpha_ob66_seg2[1] alpha_ob66_seg2[2] alpha_ob66_seg2[3]
 alpha_ob67_seg2[0] alpha_ob67_seg2[1] alpha_ob67_seg2[2]
 alpha_ob67_seg2[3] alpha_ob68_seg2[0] alpha_ob68_seg2[1]
 alpha_ob68_seg2[2] alpha_ob68_seg2[3] alpha_ob69_seg2[0]
 alpha_ob69_seg2[1] alpha_ob69_seg2[2] alpha_ob69_seg2[3]
 alpha_ob70_seg2[0] alpha_ob70_seg2[1] alpha_ob70_seg2[2]
 alpha_ob70_seg2[3] alpha_ob71_seg2[0] alpha_ob71_seg2[1]
 alpha_ob71_seg2[2] alpha_ob71_seg2[3] alpha_ob72_seg2[0]
 alpha_ob72_seg2[1] alpha_ob72_seg2[2] alpha_ob72_seg2[3]
 alpha_ob73_seg2[0] alpha_ob73_seg2[1] alpha_ob73_seg2[2]
 alpha_ob73_seg2[3] alpha_ob74_seg2[0] alpha_ob74_seg2[1]
 alpha_ob74_seg2[2] alpha_ob74_seg2[3] alpha_ob75_seg2[0]
 alpha_ob75_seg2[1] alpha_ob75_seg2[2] alpha_ob75_seg2[3]
 alpha_ob76_seg2[0] alpha_ob76_seg2[1] alpha_ob76_seg2[2]
 alpha_ob76_seg2[3]
End
