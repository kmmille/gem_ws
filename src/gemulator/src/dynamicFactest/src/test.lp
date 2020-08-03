\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 60 C0 - 20 C1 - 60 C2 - 20 C3 - 60 C4 - 20 C5 - 60 C6 - 20 C7
   + 4000 Constant + [ 2 C0 ^2 + 2 C1 ^2 + 2 C2 ^2 + 2 C3 ^2 + 2 C4 ^2
   + 2 C5 ^2 + 2 C6 ^2 + 2 C7 ^2 ] / 2 
Subject To
 R0: C0 = 12.53957752021324
 R1: C1 = 10.6135377244783
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
 R31: - C0 + 1e+06 alpha_ob3_seg0[0] <= 1.0000109161752058e+06
 R32: - C2 + 1e+06 alpha_ob3_seg0[0] <= 1.0000109161752058e+06
 R33: - C1 + 1e+06 alpha_ob3_seg0[1] <= 999970.2681002755
 R34: - C3 + 1e+06 alpha_ob3_seg0[1] <= 999970.2681002755
 R35: C0 + 1e+06 alpha_ob3_seg0[2] <= 999986.5337752058
 R36: C2 + 1e+06 alpha_ob3_seg0[2] <= 999986.5337752058
 R37: C1 + 1e+06 alpha_ob3_seg0[3] <= 1.0000138319002755e+06
 R38: C3 + 1e+06 alpha_ob3_seg0[3] <= 1.0000138319002755e+06
 R39: alpha_ob3_seg0[0] + alpha_ob3_seg0[1] + alpha_ob3_seg0[2]
   + alpha_ob3_seg0[3] >= 1
 R40: - C0 + 1e+06 alpha_ob4_seg0[0] <= 1.000010916199801e+06
 R41: - C2 + 1e+06 alpha_ob4_seg0[0] <= 1.000010916199801e+06
 R42: - C1 + 1e+06 alpha_ob4_seg0[1] <= 999983.6181179117
 R43: - C3 + 1e+06 alpha_ob4_seg0[1] <= 999983.6181179117
 R44: C0 + 1e+06 alpha_ob4_seg0[2] <= 999973.183799801
 R45: C2 + 1e+06 alpha_ob4_seg0[2] <= 999973.183799801
 R46: C1 + 1e+06 alpha_ob4_seg0[3] <= 1.0000138319179118e+06
 R47: C3 + 1e+06 alpha_ob4_seg0[3] <= 1.0000138319179118e+06
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
 R76: - C0 + 1e+06 alpha_ob8_seg0[0] <= 999981.3867954084
 R77: - C2 + 1e+06 alpha_ob8_seg0[0] <= 999981.3867954084
 R78: - C1 + 1e+06 alpha_ob8_seg0[1] <= 999968.8595002755
 R79: - C3 + 1e+06 alpha_ob8_seg0[1] <= 999968.8595002755
 R80: C0 + 1e+06 alpha_ob8_seg0[2] <= 1.0000160631954084e+06
 R81: C2 + 1e+06 alpha_ob8_seg0[2] <= 1.0000160631954084e+06
 R82: C1 + 1e+06 alpha_ob8_seg0[3] <= 1.0000262405002755e+06
 R83: C3 + 1e+06 alpha_ob8_seg0[3] <= 1.0000262405002755e+06
 R84: alpha_ob8_seg0[0] + alpha_ob8_seg0[1] + alpha_ob8_seg0[2]
   + alpha_ob8_seg0[3] >= 1
 R85: - C0 + 1e+06 alpha_ob9_seg0[0] <= 999981.386799801
 R86: - C2 + 1e+06 alpha_ob9_seg0[0] <= 999981.386799801
 R87: - C1 + 1e+06 alpha_ob9_seg0[1] <= 999971.2095013268
 R88: - C3 + 1e+06 alpha_ob9_seg0[1] <= 999971.2095013268
 R89: C0 + 1e+06 alpha_ob9_seg0[2] <= 1.0000152131998009e+06
 R90: C2 + 1e+06 alpha_ob9_seg0[2] <= 1.0000152131998009e+06
 R91: C1 + 1e+06 alpha_ob9_seg0[3] <= 1.0000262405013267e+06
 R92: C3 + 1e+06 alpha_ob9_seg0[3] <= 1.0000262405013267e+06
 R93: alpha_ob9_seg0[0] + alpha_ob9_seg0[1] + alpha_ob9_seg0[2]
   + alpha_ob9_seg0[3] >= 1
 R94: - C0 + 1e+06 alpha_ob10_seg0[0] <= 999982.2368022958
 R95: - C2 + 1e+06 alpha_ob10_seg0[0] <= 999982.2368022958
 R96: - C1 + 1e+06 alpha_ob10_seg0[1] <= 999970.1094997245
 R97: - C3 + 1e+06 alpha_ob10_seg0[1] <= 999970.1094997245
 R98: C0 + 1e+06 alpha_ob10_seg0[2] <= 1.0000152132022957e+06
 R99: C2 + 1e+06 alpha_ob10_seg0[2] <= 1.0000152132022957e+06
 R100: C1 + 1e+06 alpha_ob10_seg0[3] <= 1.0000262404997245e+06
 R101: C3 + 1e+06 alpha_ob10_seg0[3] <= 1.0000262404997245e+06
 R102: alpha_ob10_seg0[0] + alpha_ob10_seg0[1] + alpha_ob10_seg0[2]
   + alpha_ob10_seg0[3] >= 1
 R103: - C0 + 1e+06 alpha_ob11_seg0[0] <= 999982.236799801
 R104: - C2 + 1e+06 alpha_ob11_seg0[0] <= 999982.236799801
 R105: - C1 + 1e+06 alpha_ob11_seg0[1] <= 999970.1095291895
 R106: - C3 + 1e+06 alpha_ob11_seg0[1] <= 999970.1095291895
 R107: C0 + 1e+06 alpha_ob11_seg0[2] <= 999993.3631998011
 R108: C2 + 1e+06 alpha_ob11_seg0[2] <= 999993.3631998011
 R109: C1 + 1e+06 alpha_ob11_seg0[3] <= 1.0000273405291894e+06
 R110: C3 + 1e+06 alpha_ob11_seg0[3] <= 1.0000273405291894e+06
 R111: alpha_ob11_seg0[0] + alpha_ob11_seg0[1] + alpha_ob11_seg0[2]
   + alpha_ob11_seg0[3] >= 1
 R112: - C0 + 1e+06 alpha_ob12_seg0[0] <= 999981.6050802565
 R113: - C2 + 1e+06 alpha_ob12_seg0[0] <= 999981.6050802565
 R114: - C1 + 1e+06 alpha_ob12_seg0[1] <= 999972.8942002755
 R115: - C3 + 1e+06 alpha_ob12_seg0[1] <= 999972.8942002755
 R116: C0 + 1e+06 alpha_ob12_seg0[2] <= 1.0000158448802566e+06
 R117: C2 + 1e+06 alpha_ob12_seg0[2] <= 1.0000158448802566e+06
 R118: C1 + 1e+06 alpha_ob12_seg0[3] <= 1.0000139558002756e+06
 R119: C3 + 1e+06 alpha_ob12_seg0[3] <= 1.0000139558002756e+06
 R120: alpha_ob12_seg0[0] + alpha_ob12_seg0[1] + alpha_ob12_seg0[2]
   + alpha_ob12_seg0[3] >= 1
 R121: - C0 + 1e+06 alpha_ob13_seg0[0] <= 999981.605149801
 R122: - C2 + 1e+06 alpha_ob13_seg0[0] <= 999981.605149801
 R123: - C1 + 1e+06 alpha_ob13_seg0[1] <= 999983.4942391404
 R124: - C3 + 1e+06 alpha_ob13_seg0[1] <= 999983.4942391404
 R125: C0 + 1e+06 alpha_ob13_seg0[2] <= 999986.4948498011
 R126: C2 + 1e+06 alpha_ob13_seg0[2] <= 999986.4948498011
 R127: C1 + 1e+06 alpha_ob13_seg0[3] <= 1.0000139558391405e+06
 R128: C3 + 1e+06 alpha_ob13_seg0[3] <= 1.0000139558391405e+06
 R129: alpha_ob13_seg0[0] + alpha_ob13_seg0[1] + alpha_ob13_seg0[2]
   + alpha_ob13_seg0[3] >= 1
 R130: - C0 + 1e+06 alpha_ob14_seg0[0] <= 999997.6803502565
 R131: - C2 + 1e+06 alpha_ob14_seg0[0] <= 999997.6803502565
 R132: - C1 + 1e+06 alpha_ob14_seg0[1] <= 999972.7268002755
 R133: - C3 + 1e+06 alpha_ob14_seg0[1] <= 999972.7268002755
 R134: C0 + 1e+06 alpha_ob14_seg0[2] <= 999999.7696102565
 R135: C2 + 1e+06 alpha_ob14_seg0[2] <= 999999.7696102565
 R136: C1 + 1e+06 alpha_ob14_seg0[3] <= 1.0000141232002755e+06
 R137: C3 + 1e+06 alpha_ob14_seg0[3] <= 1.0000141232002755e+06
 R138: alpha_ob14_seg0[0] + alpha_ob14_seg0[1] + alpha_ob14_seg0[2]
   + alpha_ob14_seg0[3] >= 1
 R139: - C0 + 1e+06 alpha_ob15_seg0[0] <= 1.0000038341502565e+06
 R140: - C2 + 1e+06 alpha_ob15_seg0[0] <= 1.0000038341502565e+06
 R141: - C1 + 1e+06 alpha_ob15_seg0[1] <= 999972.6012002755
 R142: - C3 + 1e+06 alpha_ob15_seg0[1] <= 999972.6012002755
 R143: C0 + 1e+06 alpha_ob15_seg0[2] <= 999993.6158102566
 R144: C2 + 1e+06 alpha_ob15_seg0[2] <= 999993.6158102566
 R145: C1 + 1e+06 alpha_ob15_seg0[3] <= 1.0000142488002755e+06
 R146: C3 + 1e+06 alpha_ob15_seg0[3] <= 1.0000142488002755e+06
 R147: alpha_ob15_seg0[0] + alpha_ob15_seg0[1] + alpha_ob15_seg0[2]
   + alpha_ob15_seg0[3] >= 1
 R148: - C0 + 1e+06 alpha_ob16_seg0[0] <= 1.0000162874063333e+06
 R149: - C2 + 1e+06 alpha_ob16_seg0[0] <= 1.0000162874063333e+06
 R150: - C1 + 1e+06 alpha_ob16_seg0[1] <= 999982.9439602756
 R151: - C3 + 1e+06 alpha_ob16_seg0[1] <= 999982.9439602756
 R152: C0 + 1e+06 alpha_ob16_seg0[2] <= 999981.1624063333
 R153: C2 + 1e+06 alpha_ob16_seg0[2] <= 999981.1624063333
 R154: C1 + 1e+06 alpha_ob16_seg0[3] <= 999963.6560402757
 R155: C3 + 1e+06 alpha_ob16_seg0[3] <= 999963.6560402757
 R156: alpha_ob16_seg0[0] + alpha_ob16_seg0[1] + alpha_ob16_seg0[2]
   + alpha_ob16_seg0[3] >= 1
 R157: - C0 + 1e+06 alpha_ob17_seg0[0] <= 999980.937509
 R158: - C2 + 1e+06 alpha_ob17_seg0[0] <= 999980.937509
 R159: - C1 + 1e+06 alpha_ob17_seg0[1] <= 1.000034894e+06
 R160: - C3 + 1e+06 alpha_ob17_seg0[1] <= 1.000034894e+06
 R161: C0 + 1e+06 alpha_ob17_seg0[2] <= 999981.162491
 R162: C2 + 1e+06 alpha_ob17_seg0[2] <= 999981.162491
 R163: C1 + 1e+06 alpha_ob17_seg0[3] <= 999962.556
 R164: C3 + 1e+06 alpha_ob17_seg0[3] <= 999962.556
 R165: alpha_ob17_seg0[0] + alpha_ob17_seg0[1] + alpha_ob17_seg0[2]
   + alpha_ob17_seg0[3] >= 1
 R166: - C0 + 1e+06 alpha_ob18_seg0[0] <= 999990.2670277582
 R167: - C2 + 1e+06 alpha_ob18_seg0[0] <= 999990.2670277582
 R168: - C1 + 1e+06 alpha_ob18_seg0[1] <= 1.0000347702002756e+06
 R169: - C3 + 1e+06 alpha_ob18_seg0[1] <= 1.0000347702002756e+06
 R170: C0 + 1e+06 alpha_ob18_seg0[2] <= 1.0000071828877581e+06
 R171: C2 + 1e+06 alpha_ob18_seg0[2] <= 1.0000071828877581e+06
 R172: C1 + 1e+06 alpha_ob18_seg0[3] <= 999939.8298002756
 R173: C3 + 1e+06 alpha_ob18_seg0[3] <= 999939.8298002756
 R174: alpha_ob18_seg0[0] + alpha_ob18_seg0[1] + alpha_ob18_seg0[2]
   + alpha_ob18_seg0[3] >= 1
 R175: - C0 + 1e+06 alpha_ob19_seg0[0] <= 999990.267069801
 R176: - C2 + 1e+06 alpha_ob19_seg0[0] <= 999990.267069801
 R177: - C1 + 1e+06 alpha_ob19_seg0[1] <= 1.0000576202069657e+06
 R178: - C3 + 1e+06 alpha_ob19_seg0[1] <= 1.0000576202069657e+06
 R179: C0 + 1e+06 alpha_ob19_seg0[2] <= 1.000002082929801e+06
 R180: C2 + 1e+06 alpha_ob19_seg0[2] <= 1.000002082929801e+06
 R181: C1 + 1e+06 alpha_ob19_seg0[3] <= 999939.8298069657
 R182: C3 + 1e+06 alpha_ob19_seg0[3] <= 999939.8298069657
 R183: alpha_ob19_seg0[0] + alpha_ob19_seg0[1] + alpha_ob19_seg0[2]
   + alpha_ob19_seg0[3] >= 1
 R184: - C0 + 1e+06 alpha_ob20_seg0[0] <= 999991.073732
 R185: - C2 + 1e+06 alpha_ob20_seg0[0] <= 999991.073732
 R186: - C1 + 1e+06 alpha_ob20_seg0[1] <= 999952.2125
 R187: - C3 + 1e+06 alpha_ob20_seg0[1] <= 999952.2125
 R188: C0 + 1e+06 alpha_ob20_seg0[2] <= 999990.776268
 R189: C2 + 1e+06 alpha_ob20_seg0[2] <= 999990.776268
 R190: C1 + 1e+06 alpha_ob20_seg0[3] <= 1.0000452375e+06
 R191: C3 + 1e+06 alpha_ob20_seg0[3] <= 1.0000452375e+06
 R192: alpha_ob20_seg0[0] + alpha_ob20_seg0[1] + alpha_ob20_seg0[2]
   + alpha_ob20_seg0[3] >= 1
 R193: - C0 + 1e+06 alpha_ob21_seg0[0] <= 999985.657099801
 R194: - C2 + 1e+06 alpha_ob21_seg0[0] <= 999985.657099801
 R195: - C1 + 1e+06 alpha_ob21_seg0[1] <= 1.0000576202063023e+06
 R196: - C3 + 1e+06 alpha_ob21_seg0[1] <= 1.0000576202063023e+06
 R197: C0 + 1e+06 alpha_ob21_seg0[2] <= 1.000007192899801e+06
 R198: C2 + 1e+06 alpha_ob21_seg0[2] <= 1.000007192899801e+06
 R199: C1 + 1e+06 alpha_ob21_seg0[3] <= 999939.8298063023
 R200: C3 + 1e+06 alpha_ob21_seg0[3] <= 999939.8298063023
 R201: alpha_ob21_seg0[0] + alpha_ob21_seg0[1] + alpha_ob21_seg0[2]
   + alpha_ob21_seg0[3] >= 1
 R202: - C0 + 1e+06 alpha_ob22_seg0[0] <= 999994.771829801
 R203: - C2 + 1e+06 alpha_ob22_seg0[0] <= 999994.771829801
 R204: - C1 + 1e+06 alpha_ob22_seg0[1] <= 1.0000615983063023e+06
 R205: - C3 + 1e+06 alpha_ob22_seg0[1] <= 1.0000615983063023e+06
 R206: C0 + 1e+06 alpha_ob22_seg0[2] <= 999998.078169801
 R207: C2 + 1e+06 alpha_ob22_seg0[2] <= 999998.078169801
 R208: C1 + 1e+06 alpha_ob22_seg0[3] <= 999935.8517063023
 R209: C3 + 1e+06 alpha_ob22_seg0[3] <= 999935.8517063023
 R210: alpha_ob22_seg0[0] + alpha_ob22_seg0[1] + alpha_ob22_seg0[2]
   + alpha_ob22_seg0[3] >= 1
 R211: - C0 + 1e+06 alpha_ob23_seg0[0] <= 999980.9374508709
 R212: - C2 + 1e+06 alpha_ob23_seg0[0] <= 999980.9374508709
 R213: - C1 + 1e+06 alpha_ob23_seg0[1] <= 1.0000348940002755e+06
 R214: - C3 + 1e+06 alpha_ob23_seg0[1] <= 1.0000348940002755e+06
 R215: C0 + 1e+06 alpha_ob23_seg0[2] <= 1.0000165124508708e+06
 R216: C2 + 1e+06 alpha_ob23_seg0[2] <= 1.0000165124508708e+06
 R217: C1 + 1e+06 alpha_ob23_seg0[3] <= 999935.9560002756
 R218: C3 + 1e+06 alpha_ob23_seg0[3] <= 999935.9560002756
 R219: alpha_ob23_seg0[0] + alpha_ob23_seg0[1] + alpha_ob23_seg0[2]
   + alpha_ob23_seg0[3] >= 1
 R220: - C0 + 1e+06 alpha_ob24_seg0[0] <= 999980.937499801
 R221: - C2 + 1e+06 alpha_ob24_seg0[0] <= 999980.937499801
 R222: - C1 + 1e+06 alpha_ob24_seg0[1] <= 1.0000614940072973e+06
 R223: - C3 + 1e+06 alpha_ob24_seg0[1] <= 1.0000614940072973e+06
 R224: C0 + 1e+06 alpha_ob24_seg0[2] <= 1.000011162499801e+06
 R225: C2 + 1e+06 alpha_ob24_seg0[2] <= 1.000011162499801e+06
 R226: C1 + 1e+06 alpha_ob24_seg0[3] <= 999935.9560072974
 R227: C3 + 1e+06 alpha_ob24_seg0[3] <= 999935.9560072974
 R228: alpha_ob24_seg0[0] + alpha_ob24_seg0[1] + alpha_ob24_seg0[2]
   + alpha_ob24_seg0[3] >= 1
 R229: - C0 + 1e+06 alpha_ob25_seg0[0] <= 999956.8394000001
 R230: - C2 + 1e+06 alpha_ob25_seg0[0] <= 999956.8394000001
 R231: - C1 + 1e+06 alpha_ob25_seg0[1] <= 999952.2463
 R232: - C3 + 1e+06 alpha_ob25_seg0[1] <= 999952.2463
 R233: C0 + 1e+06 alpha_ob25_seg0[2] <= 1.0000097606e+06
 R234: C2 + 1e+06 alpha_ob25_seg0[2] <= 1.0000097606e+06
 R235: C1 + 1e+06 alpha_ob25_seg0[3] <= 1.0000452037e+06
 R236: C3 + 1e+06 alpha_ob25_seg0[3] <= 1.0000452037e+06
 R237: alpha_ob25_seg0[0] + alpha_ob25_seg0[1] + alpha_ob25_seg0[2]
   + alpha_ob25_seg0[3] >= 1
 R238: - C0 + 1e+06 alpha_ob26_seg0[0] <= 999980.9375348955
 R239: - C2 + 1e+06 alpha_ob26_seg0[0] <= 999980.9375348955
 R240: - C1 + 1e+06 alpha_ob26_seg0[1] <= 1.0000160439997246e+06
 R241: - C3 + 1e+06 alpha_ob26_seg0[1] <= 1.0000160439997246e+06
 R242: C0 + 1e+06 alpha_ob26_seg0[2] <= 1.0000165125348954e+06
 R243: C2 + 1e+06 alpha_ob26_seg0[2] <= 1.0000165125348954e+06
 R244: C1 + 1e+06 alpha_ob26_seg0[3] <= 999962.5559997245
 R245: C3 + 1e+06 alpha_ob26_seg0[3] <= 999962.5559997245
 R246: alpha_ob26_seg0[0] + alpha_ob26_seg0[1] + alpha_ob26_seg0[2]
   + alpha_ob26_seg0[3] >= 1
 R247: - C0 + 1e+06 alpha_ob27_seg0[0] <= 999956.839399801
 R248: - C2 + 1e+06 alpha_ob27_seg0[0] <= 999956.839399801
 R249: - C1 + 1e+06 alpha_ob27_seg0[1] <= 999925.3963434525
 R250: - C3 + 1e+06 alpha_ob27_seg0[1] <= 999925.3963434525
 R251: C0 + 1e+06 alpha_ob27_seg0[2] <= 1.0000080105998011e+06
 R252: C2 + 1e+06 alpha_ob27_seg0[2] <= 1.0000080105998011e+06
 R253: C1 + 1e+06 alpha_ob27_seg0[3] <= 1.0000720537434525e+06
 R254: C3 + 1e+06 alpha_ob27_seg0[3] <= 1.0000720537434525e+06
 R255: alpha_ob27_seg0[0] + alpha_ob27_seg0[1] + alpha_ob27_seg0[2]
   + alpha_ob27_seg0[3] >= 1
 R256: - C0 + 1e+06 alpha_ob28_seg0[0] <= 999980.9374998009
 R257: - C2 + 1e+06 alpha_ob28_seg0[0] <= 999980.9374998009
 R258: - C1 + 1e+06 alpha_ob28_seg0[1] <= 1.0000160440053071e+06
 R259: - C3 + 1e+06 alpha_ob28_seg0[1] <= 1.0000160440053071e+06
 R260: C0 + 1e+06 alpha_ob28_seg0[2] <= 1.000012662499801e+06
 R261: C2 + 1e+06 alpha_ob28_seg0[2] <= 1.000012662499801e+06
 R262: C1 + 1e+06 alpha_ob28_seg0[3] <= 999981.4060053072
 R263: C3 + 1e+06 alpha_ob28_seg0[3] <= 999981.4060053072
 R264: alpha_ob28_seg0[0] + alpha_ob28_seg0[1] + alpha_ob28_seg0[2]
   + alpha_ob28_seg0[3] >= 1
 R265: - C0 + 1e+06 alpha_ob29_seg0[0] <= 999983.6053459151
 R266: - C2 + 1e+06 alpha_ob29_seg0[0] <= 999983.6053459151
 R267: - C1 + 1e+06 alpha_ob29_seg0[1] <= 999987.7722497246
 R268: - C3 + 1e+06 alpha_ob29_seg0[1] <= 999987.7722497246
 R269: C0 + 1e+06 alpha_ob29_seg0[2] <= 1.000013844745915e+06
 R270: C2 + 1e+06 alpha_ob29_seg0[2] <= 1.000013844745915e+06
 R271: C1 + 1e+06 alpha_ob29_seg0[3] <= 999984.8277497246
 R272: C3 + 1e+06 alpha_ob29_seg0[3] <= 999984.8277497246
 R273: alpha_ob29_seg0[0] + alpha_ob29_seg0[1] + alpha_ob29_seg0[2]
   + alpha_ob29_seg0[3] >= 1
 R274: - C0 + 1e+06 alpha_ob30_seg0[0] <= 999958.8717898987
 R275: - C2 + 1e+06 alpha_ob30_seg0[0] <= 999958.8717898987
 R276: - C1 + 1e+06 alpha_ob30_seg0[1] <= 999987.7837602755
 R277: - C3 + 1e+06 alpha_ob30_seg0[1] <= 999987.7837602755
 R278: C0 + 1e+06 alpha_ob30_seg0[2] <= 1.0000385781898986e+06
 R279: C2 + 1e+06 alpha_ob30_seg0[2] <= 1.0000385781898986e+06
 R280: C1 + 1e+06 alpha_ob30_seg0[3] <= 1.0000043162402755e+06
 R281: C3 + 1e+06 alpha_ob30_seg0[3] <= 1.0000043162402755e+06
 R282: alpha_ob30_seg0[0] + alpha_ob30_seg0[1] + alpha_ob30_seg0[2]
   + alpha_ob30_seg0[3] >= 1
 R283: - C0 + 1e+06 alpha_ob31_seg0[0] <= 999939.2718
 R284: - C2 + 1e+06 alpha_ob31_seg0[0] <= 999939.2718
 R285: - C1 + 1e+06 alpha_ob31_seg0[1] <= 999993.13376
 R286: - C3 + 1e+06 alpha_ob31_seg0[1] <= 999993.13376
 R287: C0 + 1e+06 alpha_ob31_seg0[2] <= 1.0000385782e+06
 R288: C2 + 1e+06 alpha_ob31_seg0[2] <= 1.0000385782e+06
 R289: C1 + 1e+06 alpha_ob31_seg0[3] <= 1.00000431624e+06
 R290: C3 + 1e+06 alpha_ob31_seg0[3] <= 1.00000431624e+06
 R291: alpha_ob31_seg0[0] + alpha_ob31_seg0[1] + alpha_ob31_seg0[2]
   + alpha_ob31_seg0[3] >= 1
 R292: - C0 + 1e+06 alpha_ob32_seg0[0] <= 999939.2718101013
 R293: - C2 + 1e+06 alpha_ob32_seg0[0] <= 999939.2718101013
 R294: - C1 + 1e+06 alpha_ob32_seg0[1] <= 999987.7837597246
 R295: - C3 + 1e+06 alpha_ob32_seg0[1] <= 999987.7837597246
 R296: C0 + 1e+06 alpha_ob32_seg0[2] <= 1.0000581782101013e+06
 R297: C2 + 1e+06 alpha_ob32_seg0[2] <= 1.0000581782101013e+06
 R298: C1 + 1e+06 alpha_ob32_seg0[3] <= 1.0000043162397245e+06
 R299: C3 + 1e+06 alpha_ob32_seg0[3] <= 1.0000043162397245e+06
 R300: alpha_ob32_seg0[0] + alpha_ob32_seg0[1] + alpha_ob32_seg0[2]
   + alpha_ob32_seg0[3] >= 1
 R301: - C0 + 1e+06 alpha_ob33_seg0[0] <= 999939.271799801
 R302: - C2 + 1e+06 alpha_ob33_seg0[0] <= 999939.271799801
 R303: - C1 + 1e+06 alpha_ob33_seg0[1] <= 999987.7838262042
 R304: - C3 + 1e+06 alpha_ob33_seg0[1] <= 999987.7838262042
 R305: C0 + 1e+06 alpha_ob33_seg0[2] <= 1.000038578199801e+06
 R306: C2 + 1e+06 alpha_ob33_seg0[2] <= 1.000038578199801e+06
 R307: C1 + 1e+06 alpha_ob33_seg0[3] <= 1.0000096662262043e+06
 R308: C3 + 1e+06 alpha_ob33_seg0[3] <= 1.0000096662262043e+06
 R309: alpha_ob33_seg0[0] + alpha_ob33_seg0[1] + alpha_ob33_seg0[2]
   + alpha_ob33_seg0[3] >= 1
 R310: - C0 + 1e+06 alpha_ob34_seg0[0] <= 999926.0708
 R311: - C2 + 1e+06 alpha_ob34_seg0[0] <= 999926.0708
 R312: - C1 + 1e+06 alpha_ob34_seg0[1] <= 999983.7196
 R313: - C3 + 1e+06 alpha_ob34_seg0[1] <= 999983.7196
 R314: C0 + 1e+06 alpha_ob34_seg0[2] <= 1.0000250292e+06
 R315: C2 + 1e+06 alpha_ob34_seg0[2] <= 1.0000250292e+06
 R316: C1 + 1e+06 alpha_ob34_seg0[3] <= 1.0000137304e+06
 R317: C3 + 1e+06 alpha_ob34_seg0[3] <= 1.0000137304e+06
 R318: alpha_ob34_seg0[0] + alpha_ob34_seg0[1] + alpha_ob34_seg0[2]
   + alpha_ob34_seg0[3] >= 1
 R319: - C0 + 1e+06 alpha_ob35_seg0[0] <= 999926.0708293857
 R320: - C2 + 1e+06 alpha_ob35_seg0[0] <= 999926.0708293857
 R321: - C1 + 1e+06 alpha_ob35_seg0[1] <= 999967.8695997245
 R322: - C3 + 1e+06 alpha_ob35_seg0[1] <= 999967.8695997245
 R323: C0 + 1e+06 alpha_ob35_seg0[2] <= 1.0000713792293856e+06
 R324: C2 + 1e+06 alpha_ob35_seg0[2] <= 1.0000713792293856e+06
 R325: C1 + 1e+06 alpha_ob35_seg0[3] <= 1.0000137303997246e+06
 R326: C3 + 1e+06 alpha_ob35_seg0[3] <= 1.0000137303997246e+06
 R327: alpha_ob35_seg0[0] + alpha_ob35_seg0[1] + alpha_ob35_seg0[2]
   + alpha_ob35_seg0[3] >= 1
 R328: - C0 + 1e+06 alpha_ob36_seg0[0] <= 999972.6917508709
 R329: - C2 + 1e+06 alpha_ob36_seg0[0] <= 999972.6917508709
 R330: - C1 + 1e+06 alpha_ob36_seg0[1] <= 999957.1644002756
 R331: - C3 + 1e+06 alpha_ob36_seg0[1] <= 999957.1644002756
 R332: C0 + 1e+06 alpha_ob36_seg0[2] <= 1.0000247581508709e+06
 R333: C2 + 1e+06 alpha_ob36_seg0[2] <= 1.0000247581508709e+06
 R334: C1 + 1e+06 alpha_ob36_seg0[3] <= 1.0000136856002755e+06
 R335: C3 + 1e+06 alpha_ob36_seg0[3] <= 1.0000136856002755e+06
 R336: alpha_ob36_seg0[0] + alpha_ob36_seg0[1] + alpha_ob36_seg0[2]
   + alpha_ob36_seg0[3] >= 1
 R337: - C0 + 1e+06 alpha_ob37_seg0[0] <= 999939.0009674003
 R338: - C2 + 1e+06 alpha_ob37_seg0[0] <= 999939.0009674003
 R339: - C1 + 1e+06 alpha_ob37_seg0[1] <= 999961.9516002756
 R340: - C3 + 1e+06 alpha_ob37_seg0[1] <= 999961.9516002756
 R341: C0 + 1e+06 alpha_ob37_seg0[2] <= 1.0000584489674004e+06
 R342: C2 + 1e+06 alpha_ob37_seg0[2] <= 1.0000584489674004e+06
 R343: C1 + 1e+06 alpha_ob37_seg0[3] <= 1.0000178984002755e+06
 R344: C3 + 1e+06 alpha_ob37_seg0[3] <= 1.0000178984002755e+06
 R345: alpha_ob37_seg0[0] + alpha_ob37_seg0[1] + alpha_ob37_seg0[2]
   + alpha_ob37_seg0[3] >= 1
 R346: - C0 + 1e+06 alpha_ob38_seg0[0] <= 999994.9164998011
 R347: - C2 + 1e+06 alpha_ob38_seg0[0] <= 999994.9164998011
 R348: - C1 + 1e+06 alpha_ob38_seg0[1] <= 999925.4284782809
 R349: - C3 + 1e+06 alpha_ob38_seg0[1] <= 999925.4284782809
 R350: C0 + 1e+06 alpha_ob38_seg0[2] <= 999943.6834998011
 R351: C2 + 1e+06 alpha_ob38_seg0[2] <= 999943.6834998011
 R352: C1 + 1e+06 alpha_ob38_seg0[3] <= 1.0000720216782809e+06
 R353: C3 + 1e+06 alpha_ob38_seg0[3] <= 1.0000720216782809e+06
 R354: alpha_ob38_seg0[0] + alpha_ob38_seg0[1] + alpha_ob38_seg0[2]
   + alpha_ob38_seg0[3] >= 1
 R355: - C0 + 1e+06 alpha_ob39_seg0[0] <= 999914.52327842
 R356: - C2 + 1e+06 alpha_ob39_seg0[0] <= 999914.52327842
 R357: - C1 + 1e+06 alpha_ob39_seg0[1] <= 999993.6552362755
 R358: - C3 + 1e+06 alpha_ob39_seg0[1] <= 999993.6552362755
 R359: C0 + 1e+06 alpha_ob39_seg0[2] <= 1.0000829266784199e+06
 R360: C2 + 1e+06 alpha_ob39_seg0[2] <= 1.0000829266784199e+06
 R361: C1 + 1e+06 alpha_ob39_seg0[3] <= 999992.1947642756
 R362: C3 + 1e+06 alpha_ob39_seg0[3] <= 999992.1947642756
 R363: alpha_ob39_seg0[0] + alpha_ob39_seg0[1] + alpha_ob39_seg0[2]
   + alpha_ob39_seg0[3] >= 1
 R364: - C0 + 1e+06 alpha_ob40_seg0[0] <= 999912.265199801
 R365: - C2 + 1e+06 alpha_ob40_seg0[0] <= 999912.265199801
 R366: - C1 + 1e+06 alpha_ob40_seg0[1] <= 999987.7842112777
 R367: - C3 + 1e+06 alpha_ob40_seg0[1] <= 999987.7842112777
 R368: C0 + 1e+06 alpha_ob40_seg0[2] <= 1.000076834799801e+06
 R369: C2 + 1e+06 alpha_ob40_seg0[2] <= 1.000076834799801e+06
 R370: C1 + 1e+06 alpha_ob40_seg0[3] <= 1.0000096658112778e+06
 R371: C3 + 1e+06 alpha_ob40_seg0[3] <= 1.0000096658112778e+06
 R372: alpha_ob40_seg0[0] + alpha_ob40_seg0[1] + alpha_ob40_seg0[2]
   + alpha_ob40_seg0[3] >= 1
 R373: - C0 + 1e+06 alpha_ob41_seg0[0] <= 999920.6152119379
 R374: - C2 + 1e+06 alpha_ob41_seg0[0] <= 999920.6152119379
 R375: - C1 + 1e+06 alpha_ob41_seg0[1] <= 999981.4341997246
 R376: - C3 + 1e+06 alpha_ob41_seg0[1] <= 999981.4341997246
 R377: C0 + 1e+06 alpha_ob41_seg0[2] <= 1.0000768348119379e+06
 R378: C2 + 1e+06 alpha_ob41_seg0[2] <= 1.0000768348119379e+06
 R379: C1 + 1e+06 alpha_ob41_seg0[3] <= 1.0000096657997245e+06
 R380: C3 + 1e+06 alpha_ob41_seg0[3] <= 1.0000096657997245e+06
 R381: alpha_ob41_seg0[0] + alpha_ob41_seg0[1] + alpha_ob41_seg0[2]
   + alpha_ob41_seg0[3] >= 1
 R382: - C0 + 1e+06 alpha_ob42_seg0[0] <= 999919.2652
 R383: - C2 + 1e+06 alpha_ob42_seg0[0] <= 999919.2652
 R384: - C1 + 1e+06 alpha_ob42_seg0[1] <= 999981.4342
 R385: - C3 + 1e+06 alpha_ob42_seg0[1] <= 999981.4342
 R386: C0 + 1e+06 alpha_ob42_seg0[2] <= 1.0000768348e+06
 R387: C2 + 1e+06 alpha_ob42_seg0[2] <= 1.0000768348e+06
 R388: C1 + 1e+06 alpha_ob42_seg0[3] <= 1.0000160158000001e+06
 R389: C3 + 1e+06 alpha_ob42_seg0[3] <= 1.0000160158000001e+06
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
 R400: - C0 + 1e+06 alpha_ob44_seg0[0] <= 999991.83683
 R401: - C2 + 1e+06 alpha_ob44_seg0[0] <= 999991.83683
 R402: - C1 + 1e+06 alpha_ob44_seg0[1] <= 999940.589
 R403: - C3 + 1e+06 alpha_ob44_seg0[1] <= 999940.589
 R404: C0 + 1e+06 alpha_ob44_seg0[2] <= 1.00000401317e+06
 R405: C2 + 1e+06 alpha_ob44_seg0[2] <= 1.00000401317e+06
 R406: C1 + 1e+06 alpha_ob44_seg0[3] <= 1.000056861e+06
 R407: C3 + 1e+06 alpha_ob44_seg0[3] <= 1.000056861e+06
 R408: alpha_ob44_seg0[0] + alpha_ob44_seg0[1] + alpha_ob44_seg0[2]
   + alpha_ob44_seg0[3] >= 1
 R409: - C0 + 1e+06 alpha_ob45_seg0[0] <= 999991.8368084199
 R410: - C2 + 1e+06 alpha_ob45_seg0[0] <= 999991.8368084199
 R411: - C1 + 1e+06 alpha_ob45_seg0[1] <= 999940.5890002755
 R412: - C3 + 1e+06 alpha_ob45_seg0[1] <= 999940.5890002755
 R413: C0 + 1e+06 alpha_ob45_seg0[2] <= 1.0000056131484199e+06
 R414: C2 + 1e+06 alpha_ob45_seg0[2] <= 1.0000056131484199e+06
 R415: C1 + 1e+06 alpha_ob45_seg0[3] <= 1.0000452610002755e+06
 R416: C3 + 1e+06 alpha_ob45_seg0[3] <= 1.0000452610002755e+06
 R417: alpha_ob45_seg0[0] + alpha_ob45_seg0[1] + alpha_ob45_seg0[2]
   + alpha_ob45_seg0[3] >= 1
 R418: - C0 + 1e+06 alpha_ob46_seg0[0] <= 1.0000764021508709e+06
 R419: - C2 + 1e+06 alpha_ob46_seg0[0] <= 1.0000764021508709e+06
 R420: - C1 + 1e+06 alpha_ob46_seg0[1] <= 999957.1027002756
 R421: - C3 + 1e+06 alpha_ob46_seg0[1] <= 999957.1027002756
 R422: C0 + 1e+06 alpha_ob46_seg0[2] <= 999921.0477508709
 R423: C2 + 1e+06 alpha_ob46_seg0[2] <= 999921.0477508709
 R424: C1 + 1e+06 alpha_ob46_seg0[3] <= 1.0000137473002756e+06
 R425: C3 + 1e+06 alpha_ob46_seg0[3] <= 1.0000137473002756e+06
 R426: alpha_ob46_seg0[0] + alpha_ob46_seg0[1] + alpha_ob46_seg0[2]
   + alpha_ob46_seg0[3] >= 1
 R427: - C0 + 1e+06 alpha_ob47_seg0[0] <= 1.0000445522e+06
 R428: - C2 + 1e+06 alpha_ob47_seg0[0] <= 1.0000445522e+06
 R429: - C1 + 1e+06 alpha_ob47_seg0[1] <= 999983.7027
 R430: - C3 + 1e+06 alpha_ob47_seg0[1] <= 999983.7027
 R431: C0 + 1e+06 alpha_ob47_seg0[2] <= 999921.0477999999
 R432: C2 + 1e+06 alpha_ob47_seg0[2] <= 999921.0477999999
 R433: C1 + 1e+06 alpha_ob47_seg0[3] <= 1.0000137473e+06
 R434: C3 + 1e+06 alpha_ob47_seg0[3] <= 1.0000137473e+06
 R435: alpha_ob47_seg0[0] + alpha_ob47_seg0[1] + alpha_ob47_seg0[2]
   + alpha_ob47_seg0[3] >= 1
 R436: - C0 + 1e+06 alpha_ob48_seg0[0] <= 1.0000445522183661e+06
 R437: - C2 + 1e+06 alpha_ob48_seg0[0] <= 1.0000445522183661e+06
 R438: - C1 + 1e+06 alpha_ob48_seg0[1] <= 999973.8526997245
 R439: - C3 + 1e+06 alpha_ob48_seg0[1] <= 999973.8526997245
 R440: C0 + 1e+06 alpha_ob48_seg0[2] <= 999952.8978183661
 R441: C2 + 1e+06 alpha_ob48_seg0[2] <= 999952.8978183661
 R442: C1 + 1e+06 alpha_ob48_seg0[3] <= 1.0000137472997245e+06
 R443: C3 + 1e+06 alpha_ob48_seg0[3] <= 1.0000137472997245e+06
 R444: alpha_ob48_seg0[0] + alpha_ob48_seg0[1] + alpha_ob48_seg0[2]
   + alpha_ob48_seg0[3] >= 1
 R445: - C0 + 1e+06 alpha_ob49_seg0[0] <= 1.000064332081634e+06
 R446: - C2 + 1e+06 alpha_ob49_seg0[0] <= 1.000064332081634e+06
 R447: - C1 + 1e+06 alpha_ob49_seg0[1] <= 999973.9401002756
 R448: - C3 + 1e+06 alpha_ob49_seg0[1] <= 999973.9401002756
 R449: C0 + 1e+06 alpha_ob49_seg0[2] <= 999933.1178816339
 R450: C2 + 1e+06 alpha_ob49_seg0[2] <= 999933.1178816339
 R451: C1 + 1e+06 alpha_ob49_seg0[3] <= 1.0000136599002755e+06
 R452: C3 + 1e+06 alpha_ob49_seg0[3] <= 1.0000136599002755e+06
 R453: alpha_ob49_seg0[0] + alpha_ob49_seg0[1] + alpha_ob49_seg0[2]
   + alpha_ob49_seg0[3] >= 1
 R454: - C0 + 1e+06 alpha_ob50_seg0[0] <= 1.0000643321013774e+06
 R455: - C2 + 1e+06 alpha_ob50_seg0[0] <= 1.0000643321013774e+06
 R456: - C1 + 1e+06 alpha_ob50_seg0[1] <= 999983.1900997246
 R457: - C3 + 1e+06 alpha_ob50_seg0[1] <= 999983.1900997246
 R458: C0 + 1e+06 alpha_ob50_seg0[2] <= 999933.1179013775
 R459: C2 + 1e+06 alpha_ob50_seg0[2] <= 999933.1179013775
 R460: C1 + 1e+06 alpha_ob50_seg0[3] <= 1.0000136598997245e+06
 R461: C3 + 1e+06 alpha_ob50_seg0[3] <= 1.0000136598997245e+06
 R462: alpha_ob50_seg0[0] + alpha_ob50_seg0[1] + alpha_ob50_seg0[2]
   + alpha_ob50_seg0[3] >= 1
 R463: - C2 + 1e+06 alpha_ob0_seg1[0] <= 1.0000284170499525e+06
 R464: - C4 + 1e+06 alpha_ob0_seg1[0] <= 1.0000284170499525e+06
 R465: - C3 + 1e+06 alpha_ob0_seg1[1] <= 999957.0843002755
 R466: - C5 + 1e+06 alpha_ob0_seg1[1] <= 999957.0843002755
 R467: C2 + 1e+06 alpha_ob0_seg1[2] <= 999969.0328499526
 R468: C4 + 1e+06 alpha_ob0_seg1[2] <= 999969.0328499526
 R469: C3 + 1e+06 alpha_ob0_seg1[3] <= 1.0000132657002755e+06
 R470: C5 + 1e+06 alpha_ob0_seg1[3] <= 1.0000132657002755e+06
 R471: alpha_ob0_seg1[0] + alpha_ob0_seg1[1] + alpha_ob0_seg1[2]
   + alpha_ob0_seg1[3] >= 1
 R472: - C2 + 1e+06 alpha_ob1_seg1[0] <= 1.000028417099801e+06
 R473: - C4 + 1e+06 alpha_ob1_seg1[0] <= 1.000028417099801e+06
 R474: - C3 + 1e+06 alpha_ob1_seg1[1] <= 999984.1843155898
 R475: - C5 + 1e+06 alpha_ob1_seg1[1] <= 999984.1843155898
 R476: C2 + 1e+06 alpha_ob1_seg1[2] <= 999957.4328998009
 R477: C4 + 1e+06 alpha_ob1_seg1[2] <= 999957.4328998009
 R478: C3 + 1e+06 alpha_ob1_seg1[3] <= 1.0000132657155899e+06
 R479: C5 + 1e+06 alpha_ob1_seg1[3] <= 1.0000132657155899e+06
 R480: alpha_ob1_seg1[0] + alpha_ob1_seg1[1] + alpha_ob1_seg1[2]
   + alpha_ob1_seg1[3] >= 1
 R481: - C2 + 1e+06 alpha_ob2_seg1[0] <= 1.0000400171500475e+06
 R482: - C4 + 1e+06 alpha_ob2_seg1[0] <= 1.0000400171500475e+06
 R483: - C3 + 1e+06 alpha_ob2_seg1[1] <= 999957.0842997245
 R484: - C5 + 1e+06 alpha_ob2_seg1[1] <= 999957.0842997245
 R485: C2 + 1e+06 alpha_ob2_seg1[2] <= 999957.4329500474
 R486: C4 + 1e+06 alpha_ob2_seg1[2] <= 999957.4329500474
 R487: C3 + 1e+06 alpha_ob2_seg1[3] <= 1.0000132656997246e+06
 R488: C5 + 1e+06 alpha_ob2_seg1[3] <= 1.0000132656997246e+06
 R489: alpha_ob2_seg1[0] + alpha_ob2_seg1[1] + alpha_ob2_seg1[2]
   + alpha_ob2_seg1[3] >= 1
 R490: - C2 + 1e+06 alpha_ob3_seg1[0] <= 1.0000109161752058e+06
 R491: - C4 + 1e+06 alpha_ob3_seg1[0] <= 1.0000109161752058e+06
 R492: - C3 + 1e+06 alpha_ob3_seg1[1] <= 999970.2681002755
 R493: - C5 + 1e+06 alpha_ob3_seg1[1] <= 999970.2681002755
 R494: C2 + 1e+06 alpha_ob3_seg1[2] <= 999986.5337752058
 R495: C4 + 1e+06 alpha_ob3_seg1[2] <= 999986.5337752058
 R496: C3 + 1e+06 alpha_ob3_seg1[3] <= 1.0000138319002755e+06
 R497: C5 + 1e+06 alpha_ob3_seg1[3] <= 1.0000138319002755e+06
 R498: alpha_ob3_seg1[0] + alpha_ob3_seg1[1] + alpha_ob3_seg1[2]
   + alpha_ob3_seg1[3] >= 1
 R499: - C2 + 1e+06 alpha_ob4_seg1[0] <= 1.000010916199801e+06
 R500: - C4 + 1e+06 alpha_ob4_seg1[0] <= 1.000010916199801e+06
 R501: - C3 + 1e+06 alpha_ob4_seg1[1] <= 999983.6181179117
 R502: - C5 + 1e+06 alpha_ob4_seg1[1] <= 999983.6181179117
 R503: C2 + 1e+06 alpha_ob4_seg1[2] <= 999973.183799801
 R504: C4 + 1e+06 alpha_ob4_seg1[2] <= 999973.183799801
 R505: C3 + 1e+06 alpha_ob4_seg1[3] <= 1.0000138319179118e+06
 R506: C5 + 1e+06 alpha_ob4_seg1[3] <= 1.0000138319179118e+06
 R507: alpha_ob4_seg1[0] + alpha_ob4_seg1[1] + alpha_ob4_seg1[2]
   + alpha_ob4_seg1[3] >= 1
 R508: - C2 + 1e+06 alpha_ob5_seg1[0] <= 999992.49102
 R509: - C4 + 1e+06 alpha_ob5_seg1[0] <= 999992.49102
 R510: - C3 + 1e+06 alpha_ob5_seg1[1] <= 999957.1016000001
 R511: - C5 + 1e+06 alpha_ob5_seg1[1] <= 999957.1016000001
 R512: C2 + 1e+06 alpha_ob5_seg1[2] <= 999986.35898
 R513: C4 + 1e+06 alpha_ob5_seg1[2] <= 999986.35898
 R514: C3 + 1e+06 alpha_ob5_seg1[3] <= 1.0000403484e+06
 R515: C5 + 1e+06 alpha_ob5_seg1[3] <= 1.0000403484e+06
 R516: alpha_ob5_seg1[0] + alpha_ob5_seg1[1] + alpha_ob5_seg1[2]
   + alpha_ob5_seg1[3] >= 1
 R517: - C2 + 1e+06 alpha_ob6_seg1[0] <= 999981.3909999999
 R518: - C4 + 1e+06 alpha_ob6_seg1[0] <= 999981.3909999999
 R519: - C3 + 1e+06 alpha_ob6_seg1[1] <= 999957.1016000001
 R520: - C5 + 1e+06 alpha_ob6_seg1[1] <= 999957.1016000001
 R521: C2 + 1e+06 alpha_ob6_seg1[2] <= 1.000004959e+06
 R522: C4 + 1e+06 alpha_ob6_seg1[2] <= 1.000004959e+06
 R523: C3 + 1e+06 alpha_ob6_seg1[3] <= 1.0000403484e+06
 R524: C5 + 1e+06 alpha_ob6_seg1[3] <= 1.0000403484e+06
 R525: alpha_ob6_seg1[0] + alpha_ob6_seg1[1] + alpha_ob6_seg1[2]
   + alpha_ob6_seg1[3] >= 1
 R526: - C2 + 1e+06 alpha_ob7_seg1[0] <= 999981.3909825523
 R527: - C4 + 1e+06 alpha_ob7_seg1[0] <= 999981.3909825523
 R528: - C3 + 1e+06 alpha_ob7_seg1[1] <= 999957.1016002755
 R529: - C5 + 1e+06 alpha_ob7_seg1[1] <= 999957.1016002755
 R530: C2 + 1e+06 alpha_ob7_seg1[2] <= 1.0000160589825523e+06
 R531: C4 + 1e+06 alpha_ob7_seg1[2] <= 1.0000160589825523e+06
 R532: C3 + 1e+06 alpha_ob7_seg1[3] <= 1.0000309984002755e+06
 R533: C5 + 1e+06 alpha_ob7_seg1[3] <= 1.0000309984002755e+06
 R534: alpha_ob7_seg1[0] + alpha_ob7_seg1[1] + alpha_ob7_seg1[2]
   + alpha_ob7_seg1[3] >= 1
 R535: - C2 + 1e+06 alpha_ob8_seg1[0] <= 999981.3867954084
 R536: - C4 + 1e+06 alpha_ob8_seg1[0] <= 999981.3867954084
 R537: - C3 + 1e+06 alpha_ob8_seg1[1] <= 999968.8595002755
 R538: - C5 + 1e+06 alpha_ob8_seg1[1] <= 999968.8595002755
 R539: C2 + 1e+06 alpha_ob8_seg1[2] <= 1.0000160631954084e+06
 R540: C4 + 1e+06 alpha_ob8_seg1[2] <= 1.0000160631954084e+06
 R541: C3 + 1e+06 alpha_ob8_seg1[3] <= 1.0000262405002755e+06
 R542: C5 + 1e+06 alpha_ob8_seg1[3] <= 1.0000262405002755e+06
 R543: alpha_ob8_seg1[0] + alpha_ob8_seg1[1] + alpha_ob8_seg1[2]
   + alpha_ob8_seg1[3] >= 1
 R544: - C2 + 1e+06 alpha_ob9_seg1[0] <= 999981.386799801
 R545: - C4 + 1e+06 alpha_ob9_seg1[0] <= 999981.386799801
 R546: - C3 + 1e+06 alpha_ob9_seg1[1] <= 999971.2095013268
 R547: - C5 + 1e+06 alpha_ob9_seg1[1] <= 999971.2095013268
 R548: C2 + 1e+06 alpha_ob9_seg1[2] <= 1.0000152131998009e+06
 R549: C4 + 1e+06 alpha_ob9_seg1[2] <= 1.0000152131998009e+06
 R550: C3 + 1e+06 alpha_ob9_seg1[3] <= 1.0000262405013267e+06
 R551: C5 + 1e+06 alpha_ob9_seg1[3] <= 1.0000262405013267e+06
 R552: alpha_ob9_seg1[0] + alpha_ob9_seg1[1] + alpha_ob9_seg1[2]
   + alpha_ob9_seg1[3] >= 1
 R553: - C2 + 1e+06 alpha_ob10_seg1[0] <= 999982.2368022958
 R554: - C4 + 1e+06 alpha_ob10_seg1[0] <= 999982.2368022958
 R555: - C3 + 1e+06 alpha_ob10_seg1[1] <= 999970.1094997245
 R556: - C5 + 1e+06 alpha_ob10_seg1[1] <= 999970.1094997245
 R557: C2 + 1e+06 alpha_ob10_seg1[2] <= 1.0000152132022957e+06
 R558: C4 + 1e+06 alpha_ob10_seg1[2] <= 1.0000152132022957e+06
 R559: C3 + 1e+06 alpha_ob10_seg1[3] <= 1.0000262404997245e+06
 R560: C5 + 1e+06 alpha_ob10_seg1[3] <= 1.0000262404997245e+06
 R561: alpha_ob10_seg1[0] + alpha_ob10_seg1[1] + alpha_ob10_seg1[2]
   + alpha_ob10_seg1[3] >= 1
 R562: - C2 + 1e+06 alpha_ob11_seg1[0] <= 999982.236799801
 R563: - C4 + 1e+06 alpha_ob11_seg1[0] <= 999982.236799801
 R564: - C3 + 1e+06 alpha_ob11_seg1[1] <= 999970.1095291895
 R565: - C5 + 1e+06 alpha_ob11_seg1[1] <= 999970.1095291895
 R566: C2 + 1e+06 alpha_ob11_seg1[2] <= 999993.3631998011
 R567: C4 + 1e+06 alpha_ob11_seg1[2] <= 999993.3631998011
 R568: C3 + 1e+06 alpha_ob11_seg1[3] <= 1.0000273405291894e+06
 R569: C5 + 1e+06 alpha_ob11_seg1[3] <= 1.0000273405291894e+06
 R570: alpha_ob11_seg1[0] + alpha_ob11_seg1[1] + alpha_ob11_seg1[2]
   + alpha_ob11_seg1[3] >= 1
 R571: - C2 + 1e+06 alpha_ob12_seg1[0] <= 999981.6050802565
 R572: - C4 + 1e+06 alpha_ob12_seg1[0] <= 999981.6050802565
 R573: - C3 + 1e+06 alpha_ob12_seg1[1] <= 999972.8942002755
 R574: - C5 + 1e+06 alpha_ob12_seg1[1] <= 999972.8942002755
 R575: C2 + 1e+06 alpha_ob12_seg1[2] <= 1.0000158448802566e+06
 R576: C4 + 1e+06 alpha_ob12_seg1[2] <= 1.0000158448802566e+06
 R577: C3 + 1e+06 alpha_ob12_seg1[3] <= 1.0000139558002756e+06
 R578: C5 + 1e+06 alpha_ob12_seg1[3] <= 1.0000139558002756e+06
 R579: alpha_ob12_seg1[0] + alpha_ob12_seg1[1] + alpha_ob12_seg1[2]
   + alpha_ob12_seg1[3] >= 1
 R580: - C2 + 1e+06 alpha_ob13_seg1[0] <= 999981.605149801
 R581: - C4 + 1e+06 alpha_ob13_seg1[0] <= 999981.605149801
 R582: - C3 + 1e+06 alpha_ob13_seg1[1] <= 999983.4942391404
 R583: - C5 + 1e+06 alpha_ob13_seg1[1] <= 999983.4942391404
 R584: C2 + 1e+06 alpha_ob13_seg1[2] <= 999986.4948498011
 R585: C4 + 1e+06 alpha_ob13_seg1[2] <= 999986.4948498011
 R586: C3 + 1e+06 alpha_ob13_seg1[3] <= 1.0000139558391405e+06
 R587: C5 + 1e+06 alpha_ob13_seg1[3] <= 1.0000139558391405e+06
 R588: alpha_ob13_seg1[0] + alpha_ob13_seg1[1] + alpha_ob13_seg1[2]
   + alpha_ob13_seg1[3] >= 1
 R589: - C2 + 1e+06 alpha_ob14_seg1[0] <= 999997.6803502565
 R590: - C4 + 1e+06 alpha_ob14_seg1[0] <= 999997.6803502565
 R591: - C3 + 1e+06 alpha_ob14_seg1[1] <= 999972.7268002755
 R592: - C5 + 1e+06 alpha_ob14_seg1[1] <= 999972.7268002755
 R593: C2 + 1e+06 alpha_ob14_seg1[2] <= 999999.7696102565
 R594: C4 + 1e+06 alpha_ob14_seg1[2] <= 999999.7696102565
 R595: C3 + 1e+06 alpha_ob14_seg1[3] <= 1.0000141232002755e+06
 R596: C5 + 1e+06 alpha_ob14_seg1[3] <= 1.0000141232002755e+06
 R597: alpha_ob14_seg1[0] + alpha_ob14_seg1[1] + alpha_ob14_seg1[2]
   + alpha_ob14_seg1[3] >= 1
 R598: - C2 + 1e+06 alpha_ob15_seg1[0] <= 1.0000038341502565e+06
 R599: - C4 + 1e+06 alpha_ob15_seg1[0] <= 1.0000038341502565e+06
 R600: - C3 + 1e+06 alpha_ob15_seg1[1] <= 999972.6012002755
 R601: - C5 + 1e+06 alpha_ob15_seg1[1] <= 999972.6012002755
 R602: C2 + 1e+06 alpha_ob15_seg1[2] <= 999993.6158102566
 R603: C4 + 1e+06 alpha_ob15_seg1[2] <= 999993.6158102566
 R604: C3 + 1e+06 alpha_ob15_seg1[3] <= 1.0000142488002755e+06
 R605: C5 + 1e+06 alpha_ob15_seg1[3] <= 1.0000142488002755e+06
 R606: alpha_ob15_seg1[0] + alpha_ob15_seg1[1] + alpha_ob15_seg1[2]
   + alpha_ob15_seg1[3] >= 1
 R607: - C2 + 1e+06 alpha_ob16_seg1[0] <= 1.0000162874063333e+06
 R608: - C4 + 1e+06 alpha_ob16_seg1[0] <= 1.0000162874063333e+06
 R609: - C3 + 1e+06 alpha_ob16_seg1[1] <= 999982.9439602756
 R610: - C5 + 1e+06 alpha_ob16_seg1[1] <= 999982.9439602756
 R611: C2 + 1e+06 alpha_ob16_seg1[2] <= 999981.1624063333
 R612: C4 + 1e+06 alpha_ob16_seg1[2] <= 999981.1624063333
 R613: C3 + 1e+06 alpha_ob16_seg1[3] <= 999963.6560402757
 R614: C5 + 1e+06 alpha_ob16_seg1[3] <= 999963.6560402757
 R615: alpha_ob16_seg1[0] + alpha_ob16_seg1[1] + alpha_ob16_seg1[2]
   + alpha_ob16_seg1[3] >= 1
 R616: - C2 + 1e+06 alpha_ob17_seg1[0] <= 999980.937509
 R617: - C4 + 1e+06 alpha_ob17_seg1[0] <= 999980.937509
 R618: - C3 + 1e+06 alpha_ob17_seg1[1] <= 1.000034894e+06
 R619: - C5 + 1e+06 alpha_ob17_seg1[1] <= 1.000034894e+06
 R620: C2 + 1e+06 alpha_ob17_seg1[2] <= 999981.162491
 R621: C4 + 1e+06 alpha_ob17_seg1[2] <= 999981.162491
 R622: C3 + 1e+06 alpha_ob17_seg1[3] <= 999962.556
 R623: C5 + 1e+06 alpha_ob17_seg1[3] <= 999962.556
 R624: alpha_ob17_seg1[0] + alpha_ob17_seg1[1] + alpha_ob17_seg1[2]
   + alpha_ob17_seg1[3] >= 1
 R625: - C2 + 1e+06 alpha_ob18_seg1[0] <= 999990.2670277582
 R626: - C4 + 1e+06 alpha_ob18_seg1[0] <= 999990.2670277582
 R627: - C3 + 1e+06 alpha_ob18_seg1[1] <= 1.0000347702002756e+06
 R628: - C5 + 1e+06 alpha_ob18_seg1[1] <= 1.0000347702002756e+06
 R629: C2 + 1e+06 alpha_ob18_seg1[2] <= 1.0000071828877581e+06
 R630: C4 + 1e+06 alpha_ob18_seg1[2] <= 1.0000071828877581e+06
 R631: C3 + 1e+06 alpha_ob18_seg1[3] <= 999939.8298002756
 R632: C5 + 1e+06 alpha_ob18_seg1[3] <= 999939.8298002756
 R633: alpha_ob18_seg1[0] + alpha_ob18_seg1[1] + alpha_ob18_seg1[2]
   + alpha_ob18_seg1[3] >= 1
 R634: - C2 + 1e+06 alpha_ob19_seg1[0] <= 999990.267069801
 R635: - C4 + 1e+06 alpha_ob19_seg1[0] <= 999990.267069801
 R636: - C3 + 1e+06 alpha_ob19_seg1[1] <= 1.0000576202069657e+06
 R637: - C5 + 1e+06 alpha_ob19_seg1[1] <= 1.0000576202069657e+06
 R638: C2 + 1e+06 alpha_ob19_seg1[2] <= 1.000002082929801e+06
 R639: C4 + 1e+06 alpha_ob19_seg1[2] <= 1.000002082929801e+06
 R640: C3 + 1e+06 alpha_ob19_seg1[3] <= 999939.8298069657
 R641: C5 + 1e+06 alpha_ob19_seg1[3] <= 999939.8298069657
 R642: alpha_ob19_seg1[0] + alpha_ob19_seg1[1] + alpha_ob19_seg1[2]
   + alpha_ob19_seg1[3] >= 1
 R643: - C2 + 1e+06 alpha_ob20_seg1[0] <= 999991.073732
 R644: - C4 + 1e+06 alpha_ob20_seg1[0] <= 999991.073732
 R645: - C3 + 1e+06 alpha_ob20_seg1[1] <= 999952.2125
 R646: - C5 + 1e+06 alpha_ob20_seg1[1] <= 999952.2125
 R647: C2 + 1e+06 alpha_ob20_seg1[2] <= 999990.776268
 R648: C4 + 1e+06 alpha_ob20_seg1[2] <= 999990.776268
 R649: C3 + 1e+06 alpha_ob20_seg1[3] <= 1.0000452375e+06
 R650: C5 + 1e+06 alpha_ob20_seg1[3] <= 1.0000452375e+06
 R651: alpha_ob20_seg1[0] + alpha_ob20_seg1[1] + alpha_ob20_seg1[2]
   + alpha_ob20_seg1[3] >= 1
 R652: - C2 + 1e+06 alpha_ob21_seg1[0] <= 999985.657099801
 R653: - C4 + 1e+06 alpha_ob21_seg1[0] <= 999985.657099801
 R654: - C3 + 1e+06 alpha_ob21_seg1[1] <= 1.0000576202063023e+06
 R655: - C5 + 1e+06 alpha_ob21_seg1[1] <= 1.0000576202063023e+06
 R656: C2 + 1e+06 alpha_ob21_seg1[2] <= 1.000007192899801e+06
 R657: C4 + 1e+06 alpha_ob21_seg1[2] <= 1.000007192899801e+06
 R658: C3 + 1e+06 alpha_ob21_seg1[3] <= 999939.8298063023
 R659: C5 + 1e+06 alpha_ob21_seg1[3] <= 999939.8298063023
 R660: alpha_ob21_seg1[0] + alpha_ob21_seg1[1] + alpha_ob21_seg1[2]
   + alpha_ob21_seg1[3] >= 1
 R661: - C2 + 1e+06 alpha_ob22_seg1[0] <= 999994.771829801
 R662: - C4 + 1e+06 alpha_ob22_seg1[0] <= 999994.771829801
 R663: - C3 + 1e+06 alpha_ob22_seg1[1] <= 1.0000615983063023e+06
 R664: - C5 + 1e+06 alpha_ob22_seg1[1] <= 1.0000615983063023e+06
 R665: C2 + 1e+06 alpha_ob22_seg1[2] <= 999998.078169801
 R666: C4 + 1e+06 alpha_ob22_seg1[2] <= 999998.078169801
 R667: C3 + 1e+06 alpha_ob22_seg1[3] <= 999935.8517063023
 R668: C5 + 1e+06 alpha_ob22_seg1[3] <= 999935.8517063023
 R669: alpha_ob22_seg1[0] + alpha_ob22_seg1[1] + alpha_ob22_seg1[2]
   + alpha_ob22_seg1[3] >= 1
 R670: - C2 + 1e+06 alpha_ob23_seg1[0] <= 999980.9374508709
 R671: - C4 + 1e+06 alpha_ob23_seg1[0] <= 999980.9374508709
 R672: - C3 + 1e+06 alpha_ob23_seg1[1] <= 1.0000348940002755e+06
 R673: - C5 + 1e+06 alpha_ob23_seg1[1] <= 1.0000348940002755e+06
 R674: C2 + 1e+06 alpha_ob23_seg1[2] <= 1.0000165124508708e+06
 R675: C4 + 1e+06 alpha_ob23_seg1[2] <= 1.0000165124508708e+06
 R676: C3 + 1e+06 alpha_ob23_seg1[3] <= 999935.9560002756
 R677: C5 + 1e+06 alpha_ob23_seg1[3] <= 999935.9560002756
 R678: alpha_ob23_seg1[0] + alpha_ob23_seg1[1] + alpha_ob23_seg1[2]
   + alpha_ob23_seg1[3] >= 1
 R679: - C2 + 1e+06 alpha_ob24_seg1[0] <= 999980.937499801
 R680: - C4 + 1e+06 alpha_ob24_seg1[0] <= 999980.937499801
 R681: - C3 + 1e+06 alpha_ob24_seg1[1] <= 1.0000614940072973e+06
 R682: - C5 + 1e+06 alpha_ob24_seg1[1] <= 1.0000614940072973e+06
 R683: C2 + 1e+06 alpha_ob24_seg1[2] <= 1.000011162499801e+06
 R684: C4 + 1e+06 alpha_ob24_seg1[2] <= 1.000011162499801e+06
 R685: C3 + 1e+06 alpha_ob24_seg1[3] <= 999935.9560072974
 R686: C5 + 1e+06 alpha_ob24_seg1[3] <= 999935.9560072974
 R687: alpha_ob24_seg1[0] + alpha_ob24_seg1[1] + alpha_ob24_seg1[2]
   + alpha_ob24_seg1[3] >= 1
 R688: - C2 + 1e+06 alpha_ob25_seg1[0] <= 999956.8394000001
 R689: - C4 + 1e+06 alpha_ob25_seg1[0] <= 999956.8394000001
 R690: - C3 + 1e+06 alpha_ob25_seg1[1] <= 999952.2463
 R691: - C5 + 1e+06 alpha_ob25_seg1[1] <= 999952.2463
 R692: C2 + 1e+06 alpha_ob25_seg1[2] <= 1.0000097606e+06
 R693: C4 + 1e+06 alpha_ob25_seg1[2] <= 1.0000097606e+06
 R694: C3 + 1e+06 alpha_ob25_seg1[3] <= 1.0000452037e+06
 R695: C5 + 1e+06 alpha_ob25_seg1[3] <= 1.0000452037e+06
 R696: alpha_ob25_seg1[0] + alpha_ob25_seg1[1] + alpha_ob25_seg1[2]
   + alpha_ob25_seg1[3] >= 1
 R697: - C2 + 1e+06 alpha_ob26_seg1[0] <= 999980.9375348955
 R698: - C4 + 1e+06 alpha_ob26_seg1[0] <= 999980.9375348955
 R699: - C3 + 1e+06 alpha_ob26_seg1[1] <= 1.0000160439997246e+06
 R700: - C5 + 1e+06 alpha_ob26_seg1[1] <= 1.0000160439997246e+06
 R701: C2 + 1e+06 alpha_ob26_seg1[2] <= 1.0000165125348954e+06
 R702: C4 + 1e+06 alpha_ob26_seg1[2] <= 1.0000165125348954e+06
 R703: C3 + 1e+06 alpha_ob26_seg1[3] <= 999962.5559997245
 R704: C5 + 1e+06 alpha_ob26_seg1[3] <= 999962.5559997245
 R705: alpha_ob26_seg1[0] + alpha_ob26_seg1[1] + alpha_ob26_seg1[2]
   + alpha_ob26_seg1[3] >= 1
 R706: - C2 + 1e+06 alpha_ob27_seg1[0] <= 999956.839399801
 R707: - C4 + 1e+06 alpha_ob27_seg1[0] <= 999956.839399801
 R708: - C3 + 1e+06 alpha_ob27_seg1[1] <= 999925.3963434525
 R709: - C5 + 1e+06 alpha_ob27_seg1[1] <= 999925.3963434525
 R710: C2 + 1e+06 alpha_ob27_seg1[2] <= 1.0000080105998011e+06
 R711: C4 + 1e+06 alpha_ob27_seg1[2] <= 1.0000080105998011e+06
 R712: C3 + 1e+06 alpha_ob27_seg1[3] <= 1.0000720537434525e+06
 R713: C5 + 1e+06 alpha_ob27_seg1[3] <= 1.0000720537434525e+06
 R714: alpha_ob27_seg1[0] + alpha_ob27_seg1[1] + alpha_ob27_seg1[2]
   + alpha_ob27_seg1[3] >= 1
 R715: - C2 + 1e+06 alpha_ob28_seg1[0] <= 999980.9374998009
 R716: - C4 + 1e+06 alpha_ob28_seg1[0] <= 999980.9374998009
 R717: - C3 + 1e+06 alpha_ob28_seg1[1] <= 1.0000160440053071e+06
 R718: - C5 + 1e+06 alpha_ob28_seg1[1] <= 1.0000160440053071e+06
 R719: C2 + 1e+06 alpha_ob28_seg1[2] <= 1.000012662499801e+06
 R720: C4 + 1e+06 alpha_ob28_seg1[2] <= 1.000012662499801e+06
 R721: C3 + 1e+06 alpha_ob28_seg1[3] <= 999981.4060053072
 R722: C5 + 1e+06 alpha_ob28_seg1[3] <= 999981.4060053072
 R723: alpha_ob28_seg1[0] + alpha_ob28_seg1[1] + alpha_ob28_seg1[2]
   + alpha_ob28_seg1[3] >= 1
 R724: - C2 + 1e+06 alpha_ob29_seg1[0] <= 999983.6053459151
 R725: - C4 + 1e+06 alpha_ob29_seg1[0] <= 999983.6053459151
 R726: - C3 + 1e+06 alpha_ob29_seg1[1] <= 999987.7722497246
 R727: - C5 + 1e+06 alpha_ob29_seg1[1] <= 999987.7722497246
 R728: C2 + 1e+06 alpha_ob29_seg1[2] <= 1.000013844745915e+06
 R729: C4 + 1e+06 alpha_ob29_seg1[2] <= 1.000013844745915e+06
 R730: C3 + 1e+06 alpha_ob29_seg1[3] <= 999984.8277497246
 R731: C5 + 1e+06 alpha_ob29_seg1[3] <= 999984.8277497246
 R732: alpha_ob29_seg1[0] + alpha_ob29_seg1[1] + alpha_ob29_seg1[2]
   + alpha_ob29_seg1[3] >= 1
 R733: - C2 + 1e+06 alpha_ob30_seg1[0] <= 999958.8717898987
 R734: - C4 + 1e+06 alpha_ob30_seg1[0] <= 999958.8717898987
 R735: - C3 + 1e+06 alpha_ob30_seg1[1] <= 999987.7837602755
 R736: - C5 + 1e+06 alpha_ob30_seg1[1] <= 999987.7837602755
 R737: C2 + 1e+06 alpha_ob30_seg1[2] <= 1.0000385781898986e+06
 R738: C4 + 1e+06 alpha_ob30_seg1[2] <= 1.0000385781898986e+06
 R739: C3 + 1e+06 alpha_ob30_seg1[3] <= 1.0000043162402755e+06
 R740: C5 + 1e+06 alpha_ob30_seg1[3] <= 1.0000043162402755e+06
 R741: alpha_ob30_seg1[0] + alpha_ob30_seg1[1] + alpha_ob30_seg1[2]
   + alpha_ob30_seg1[3] >= 1
 R742: - C2 + 1e+06 alpha_ob31_seg1[0] <= 999939.2718
 R743: - C4 + 1e+06 alpha_ob31_seg1[0] <= 999939.2718
 R744: - C3 + 1e+06 alpha_ob31_seg1[1] <= 999993.13376
 R745: - C5 + 1e+06 alpha_ob31_seg1[1] <= 999993.13376
 R746: C2 + 1e+06 alpha_ob31_seg1[2] <= 1.0000385782e+06
 R747: C4 + 1e+06 alpha_ob31_seg1[2] <= 1.0000385782e+06
 R748: C3 + 1e+06 alpha_ob31_seg1[3] <= 1.00000431624e+06
 R749: C5 + 1e+06 alpha_ob31_seg1[3] <= 1.00000431624e+06
 R750: alpha_ob31_seg1[0] + alpha_ob31_seg1[1] + alpha_ob31_seg1[2]
   + alpha_ob31_seg1[3] >= 1
 R751: - C2 + 1e+06 alpha_ob32_seg1[0] <= 999939.2718101013
 R752: - C4 + 1e+06 alpha_ob32_seg1[0] <= 999939.2718101013
 R753: - C3 + 1e+06 alpha_ob32_seg1[1] <= 999987.7837597246
 R754: - C5 + 1e+06 alpha_ob32_seg1[1] <= 999987.7837597246
 R755: C2 + 1e+06 alpha_ob32_seg1[2] <= 1.0000581782101013e+06
 R756: C4 + 1e+06 alpha_ob32_seg1[2] <= 1.0000581782101013e+06
 R757: C3 + 1e+06 alpha_ob32_seg1[3] <= 1.0000043162397245e+06
 R758: C5 + 1e+06 alpha_ob32_seg1[3] <= 1.0000043162397245e+06
 R759: alpha_ob32_seg1[0] + alpha_ob32_seg1[1] + alpha_ob32_seg1[2]
   + alpha_ob32_seg1[3] >= 1
 R760: - C2 + 1e+06 alpha_ob33_seg1[0] <= 999939.271799801
 R761: - C4 + 1e+06 alpha_ob33_seg1[0] <= 999939.271799801
 R762: - C3 + 1e+06 alpha_ob33_seg1[1] <= 999987.7838262042
 R763: - C5 + 1e+06 alpha_ob33_seg1[1] <= 999987.7838262042
 R764: C2 + 1e+06 alpha_ob33_seg1[2] <= 1.000038578199801e+06
 R765: C4 + 1e+06 alpha_ob33_seg1[2] <= 1.000038578199801e+06
 R766: C3 + 1e+06 alpha_ob33_seg1[3] <= 1.0000096662262043e+06
 R767: C5 + 1e+06 alpha_ob33_seg1[3] <= 1.0000096662262043e+06
 R768: alpha_ob33_seg1[0] + alpha_ob33_seg1[1] + alpha_ob33_seg1[2]
   + alpha_ob33_seg1[3] >= 1
 R769: - C2 + 1e+06 alpha_ob34_seg1[0] <= 999926.0708
 R770: - C4 + 1e+06 alpha_ob34_seg1[0] <= 999926.0708
 R771: - C3 + 1e+06 alpha_ob34_seg1[1] <= 999983.7196
 R772: - C5 + 1e+06 alpha_ob34_seg1[1] <= 999983.7196
 R773: C2 + 1e+06 alpha_ob34_seg1[2] <= 1.0000250292e+06
 R774: C4 + 1e+06 alpha_ob34_seg1[2] <= 1.0000250292e+06
 R775: C3 + 1e+06 alpha_ob34_seg1[3] <= 1.0000137304e+06
 R776: C5 + 1e+06 alpha_ob34_seg1[3] <= 1.0000137304e+06
 R777: alpha_ob34_seg1[0] + alpha_ob34_seg1[1] + alpha_ob34_seg1[2]
   + alpha_ob34_seg1[3] >= 1
 R778: - C2 + 1e+06 alpha_ob35_seg1[0] <= 999926.0708293857
 R779: - C4 + 1e+06 alpha_ob35_seg1[0] <= 999926.0708293857
 R780: - C3 + 1e+06 alpha_ob35_seg1[1] <= 999967.8695997245
 R781: - C5 + 1e+06 alpha_ob35_seg1[1] <= 999967.8695997245
 R782: C2 + 1e+06 alpha_ob35_seg1[2] <= 1.0000713792293856e+06
 R783: C4 + 1e+06 alpha_ob35_seg1[2] <= 1.0000713792293856e+06
 R784: C3 + 1e+06 alpha_ob35_seg1[3] <= 1.0000137303997246e+06
 R785: C5 + 1e+06 alpha_ob35_seg1[3] <= 1.0000137303997246e+06
 R786: alpha_ob35_seg1[0] + alpha_ob35_seg1[1] + alpha_ob35_seg1[2]
   + alpha_ob35_seg1[3] >= 1
 R787: - C2 + 1e+06 alpha_ob36_seg1[0] <= 999972.6917508709
 R788: - C4 + 1e+06 alpha_ob36_seg1[0] <= 999972.6917508709
 R789: - C3 + 1e+06 alpha_ob36_seg1[1] <= 999957.1644002756
 R790: - C5 + 1e+06 alpha_ob36_seg1[1] <= 999957.1644002756
 R791: C2 + 1e+06 alpha_ob36_seg1[2] <= 1.0000247581508709e+06
 R792: C4 + 1e+06 alpha_ob36_seg1[2] <= 1.0000247581508709e+06
 R793: C3 + 1e+06 alpha_ob36_seg1[3] <= 1.0000136856002755e+06
 R794: C5 + 1e+06 alpha_ob36_seg1[3] <= 1.0000136856002755e+06
 R795: alpha_ob36_seg1[0] + alpha_ob36_seg1[1] + alpha_ob36_seg1[2]
   + alpha_ob36_seg1[3] >= 1
 R796: - C2 + 1e+06 alpha_ob37_seg1[0] <= 999939.0009674003
 R797: - C4 + 1e+06 alpha_ob37_seg1[0] <= 999939.0009674003
 R798: - C3 + 1e+06 alpha_ob37_seg1[1] <= 999961.9516002756
 R799: - C5 + 1e+06 alpha_ob37_seg1[1] <= 999961.9516002756
 R800: C2 + 1e+06 alpha_ob37_seg1[2] <= 1.0000584489674004e+06
 R801: C4 + 1e+06 alpha_ob37_seg1[2] <= 1.0000584489674004e+06
 R802: C3 + 1e+06 alpha_ob37_seg1[3] <= 1.0000178984002755e+06
 R803: C5 + 1e+06 alpha_ob37_seg1[3] <= 1.0000178984002755e+06
 R804: alpha_ob37_seg1[0] + alpha_ob37_seg1[1] + alpha_ob37_seg1[2]
   + alpha_ob37_seg1[3] >= 1
 R805: - C2 + 1e+06 alpha_ob38_seg1[0] <= 999994.9164998011
 R806: - C4 + 1e+06 alpha_ob38_seg1[0] <= 999994.9164998011
 R807: - C3 + 1e+06 alpha_ob38_seg1[1] <= 999925.4284782809
 R808: - C5 + 1e+06 alpha_ob38_seg1[1] <= 999925.4284782809
 R809: C2 + 1e+06 alpha_ob38_seg1[2] <= 999943.6834998011
 R810: C4 + 1e+06 alpha_ob38_seg1[2] <= 999943.6834998011
 R811: C3 + 1e+06 alpha_ob38_seg1[3] <= 1.0000720216782809e+06
 R812: C5 + 1e+06 alpha_ob38_seg1[3] <= 1.0000720216782809e+06
 R813: alpha_ob38_seg1[0] + alpha_ob38_seg1[1] + alpha_ob38_seg1[2]
   + alpha_ob38_seg1[3] >= 1
 R814: - C2 + 1e+06 alpha_ob39_seg1[0] <= 999914.52327842
 R815: - C4 + 1e+06 alpha_ob39_seg1[0] <= 999914.52327842
 R816: - C3 + 1e+06 alpha_ob39_seg1[1] <= 999993.6552362755
 R817: - C5 + 1e+06 alpha_ob39_seg1[1] <= 999993.6552362755
 R818: C2 + 1e+06 alpha_ob39_seg1[2] <= 1.0000829266784199e+06
 R819: C4 + 1e+06 alpha_ob39_seg1[2] <= 1.0000829266784199e+06
 R820: C3 + 1e+06 alpha_ob39_seg1[3] <= 999992.1947642756
 R821: C5 + 1e+06 alpha_ob39_seg1[3] <= 999992.1947642756
 R822: alpha_ob39_seg1[0] + alpha_ob39_seg1[1] + alpha_ob39_seg1[2]
   + alpha_ob39_seg1[3] >= 1
 R823: - C2 + 1e+06 alpha_ob40_seg1[0] <= 999912.265199801
 R824: - C4 + 1e+06 alpha_ob40_seg1[0] <= 999912.265199801
 R825: - C3 + 1e+06 alpha_ob40_seg1[1] <= 999987.7842112777
 R826: - C5 + 1e+06 alpha_ob40_seg1[1] <= 999987.7842112777
 R827: C2 + 1e+06 alpha_ob40_seg1[2] <= 1.000076834799801e+06
 R828: C4 + 1e+06 alpha_ob40_seg1[2] <= 1.000076834799801e+06
 R829: C3 + 1e+06 alpha_ob40_seg1[3] <= 1.0000096658112778e+06
 R830: C5 + 1e+06 alpha_ob40_seg1[3] <= 1.0000096658112778e+06
 R831: alpha_ob40_seg1[0] + alpha_ob40_seg1[1] + alpha_ob40_seg1[2]
   + alpha_ob40_seg1[3] >= 1
 R832: - C2 + 1e+06 alpha_ob41_seg1[0] <= 999920.6152119379
 R833: - C4 + 1e+06 alpha_ob41_seg1[0] <= 999920.6152119379
 R834: - C3 + 1e+06 alpha_ob41_seg1[1] <= 999981.4341997246
 R835: - C5 + 1e+06 alpha_ob41_seg1[1] <= 999981.4341997246
 R836: C2 + 1e+06 alpha_ob41_seg1[2] <= 1.0000768348119379e+06
 R837: C4 + 1e+06 alpha_ob41_seg1[2] <= 1.0000768348119379e+06
 R838: C3 + 1e+06 alpha_ob41_seg1[3] <= 1.0000096657997245e+06
 R839: C5 + 1e+06 alpha_ob41_seg1[3] <= 1.0000096657997245e+06
 R840: alpha_ob41_seg1[0] + alpha_ob41_seg1[1] + alpha_ob41_seg1[2]
   + alpha_ob41_seg1[3] >= 1
 R841: - C2 + 1e+06 alpha_ob42_seg1[0] <= 999919.2652
 R842: - C4 + 1e+06 alpha_ob42_seg1[0] <= 999919.2652
 R843: - C3 + 1e+06 alpha_ob42_seg1[1] <= 999981.4342
 R844: - C5 + 1e+06 alpha_ob42_seg1[1] <= 999981.4342
 R845: C2 + 1e+06 alpha_ob42_seg1[2] <= 1.0000768348e+06
 R846: C4 + 1e+06 alpha_ob42_seg1[2] <= 1.0000768348e+06
 R847: C3 + 1e+06 alpha_ob42_seg1[3] <= 1.0000160158000001e+06
 R848: C5 + 1e+06 alpha_ob42_seg1[3] <= 1.0000160158000001e+06
 R849: alpha_ob42_seg1[0] + alpha_ob42_seg1[1] + alpha_ob42_seg1[2]
   + alpha_ob42_seg1[3] >= 1
 R850: - C2 + 1e+06 alpha_ob43_seg1[0] <= 999919.2652454559
 R851: - C4 + 1e+06 alpha_ob43_seg1[0] <= 999919.2652454559
 R852: - C3 + 1e+06 alpha_ob43_seg1[1] <= 999956.8341997246
 R853: - C5 + 1e+06 alpha_ob43_seg1[1] <= 999956.8341997246
 R854: C2 + 1e+06 alpha_ob43_seg1[2] <= 1.0000781848454559e+06
 R855: C4 + 1e+06 alpha_ob43_seg1[2] <= 1.0000781848454559e+06
 R856: C3 + 1e+06 alpha_ob43_seg1[3] <= 1.0000160157997246e+06
 R857: C5 + 1e+06 alpha_ob43_seg1[3] <= 1.0000160157997246e+06
 R858: alpha_ob43_seg1[0] + alpha_ob43_seg1[1] + alpha_ob43_seg1[2]
   + alpha_ob43_seg1[3] >= 1
 R859: - C2 + 1e+06 alpha_ob44_seg1[0] <= 999991.83683
 R860: - C4 + 1e+06 alpha_ob44_seg1[0] <= 999991.83683
 R861: - C3 + 1e+06 alpha_ob44_seg1[1] <= 999940.589
 R862: - C5 + 1e+06 alpha_ob44_seg1[1] <= 999940.589
 R863: C2 + 1e+06 alpha_ob44_seg1[2] <= 1.00000401317e+06
 R864: C4 + 1e+06 alpha_ob44_seg1[2] <= 1.00000401317e+06
 R865: C3 + 1e+06 alpha_ob44_seg1[3] <= 1.000056861e+06
 R866: C5 + 1e+06 alpha_ob44_seg1[3] <= 1.000056861e+06
 R867: alpha_ob44_seg1[0] + alpha_ob44_seg1[1] + alpha_ob44_seg1[2]
   + alpha_ob44_seg1[3] >= 1
 R868: - C2 + 1e+06 alpha_ob45_seg1[0] <= 999991.8368084199
 R869: - C4 + 1e+06 alpha_ob45_seg1[0] <= 999991.8368084199
 R870: - C3 + 1e+06 alpha_ob45_seg1[1] <= 999940.5890002755
 R871: - C5 + 1e+06 alpha_ob45_seg1[1] <= 999940.5890002755
 R872: C2 + 1e+06 alpha_ob45_seg1[2] <= 1.0000056131484199e+06
 R873: C4 + 1e+06 alpha_ob45_seg1[2] <= 1.0000056131484199e+06
 R874: C3 + 1e+06 alpha_ob45_seg1[3] <= 1.0000452610002755e+06
 R875: C5 + 1e+06 alpha_ob45_seg1[3] <= 1.0000452610002755e+06
 R876: alpha_ob45_seg1[0] + alpha_ob45_seg1[1] + alpha_ob45_seg1[2]
   + alpha_ob45_seg1[3] >= 1
 R877: - C2 + 1e+06 alpha_ob46_seg1[0] <= 1.0000764021508709e+06
 R878: - C4 + 1e+06 alpha_ob46_seg1[0] <= 1.0000764021508709e+06
 R879: - C3 + 1e+06 alpha_ob46_seg1[1] <= 999957.1027002756
 R880: - C5 + 1e+06 alpha_ob46_seg1[1] <= 999957.1027002756
 R881: C2 + 1e+06 alpha_ob46_seg1[2] <= 999921.0477508709
 R882: C4 + 1e+06 alpha_ob46_seg1[2] <= 999921.0477508709
 R883: C3 + 1e+06 alpha_ob46_seg1[3] <= 1.0000137473002756e+06
 R884: C5 + 1e+06 alpha_ob46_seg1[3] <= 1.0000137473002756e+06
 R885: alpha_ob46_seg1[0] + alpha_ob46_seg1[1] + alpha_ob46_seg1[2]
   + alpha_ob46_seg1[3] >= 1
 R886: - C2 + 1e+06 alpha_ob47_seg1[0] <= 1.0000445522e+06
 R887: - C4 + 1e+06 alpha_ob47_seg1[0] <= 1.0000445522e+06
 R888: - C3 + 1e+06 alpha_ob47_seg1[1] <= 999983.7027
 R889: - C5 + 1e+06 alpha_ob47_seg1[1] <= 999983.7027
 R890: C2 + 1e+06 alpha_ob47_seg1[2] <= 999921.0477999999
 R891: C4 + 1e+06 alpha_ob47_seg1[2] <= 999921.0477999999
 R892: C3 + 1e+06 alpha_ob47_seg1[3] <= 1.0000137473e+06
 R893: C5 + 1e+06 alpha_ob47_seg1[3] <= 1.0000137473e+06
 R894: alpha_ob47_seg1[0] + alpha_ob47_seg1[1] + alpha_ob47_seg1[2]
   + alpha_ob47_seg1[3] >= 1
 R895: - C2 + 1e+06 alpha_ob48_seg1[0] <= 1.0000445522183661e+06
 R896: - C4 + 1e+06 alpha_ob48_seg1[0] <= 1.0000445522183661e+06
 R897: - C3 + 1e+06 alpha_ob48_seg1[1] <= 999973.8526997245
 R898: - C5 + 1e+06 alpha_ob48_seg1[1] <= 999973.8526997245
 R899: C2 + 1e+06 alpha_ob48_seg1[2] <= 999952.8978183661
 R900: C4 + 1e+06 alpha_ob48_seg1[2] <= 999952.8978183661
 R901: C3 + 1e+06 alpha_ob48_seg1[3] <= 1.0000137472997245e+06
 R902: C5 + 1e+06 alpha_ob48_seg1[3] <= 1.0000137472997245e+06
 R903: alpha_ob48_seg1[0] + alpha_ob48_seg1[1] + alpha_ob48_seg1[2]
   + alpha_ob48_seg1[3] >= 1
 R904: - C2 + 1e+06 alpha_ob49_seg1[0] <= 1.000064332081634e+06
 R905: - C4 + 1e+06 alpha_ob49_seg1[0] <= 1.000064332081634e+06
 R906: - C3 + 1e+06 alpha_ob49_seg1[1] <= 999973.9401002756
 R907: - C5 + 1e+06 alpha_ob49_seg1[1] <= 999973.9401002756
 R908: C2 + 1e+06 alpha_ob49_seg1[2] <= 999933.1178816339
 R909: C4 + 1e+06 alpha_ob49_seg1[2] <= 999933.1178816339
 R910: C3 + 1e+06 alpha_ob49_seg1[3] <= 1.0000136599002755e+06
 R911: C5 + 1e+06 alpha_ob49_seg1[3] <= 1.0000136599002755e+06
 R912: alpha_ob49_seg1[0] + alpha_ob49_seg1[1] + alpha_ob49_seg1[2]
   + alpha_ob49_seg1[3] >= 1
 R913: - C2 + 1e+06 alpha_ob50_seg1[0] <= 1.0000643321013774e+06
 R914: - C4 + 1e+06 alpha_ob50_seg1[0] <= 1.0000643321013774e+06
 R915: - C3 + 1e+06 alpha_ob50_seg1[1] <= 999983.1900997246
 R916: - C5 + 1e+06 alpha_ob50_seg1[1] <= 999983.1900997246
 R917: C2 + 1e+06 alpha_ob50_seg1[2] <= 999933.1179013775
 R918: C4 + 1e+06 alpha_ob50_seg1[2] <= 999933.1179013775
 R919: C3 + 1e+06 alpha_ob50_seg1[3] <= 1.0000136598997245e+06
 R920: C5 + 1e+06 alpha_ob50_seg1[3] <= 1.0000136598997245e+06
 R921: alpha_ob50_seg1[0] + alpha_ob50_seg1[1] + alpha_ob50_seg1[2]
   + alpha_ob50_seg1[3] >= 1
 R922: - C4 + 1e+06 alpha_ob0_seg2[0] <= 1.0000284170499525e+06
 R923: - C6 + 1e+06 alpha_ob0_seg2[0] <= 1.0000284170499525e+06
 R924: - C5 + 1e+06 alpha_ob0_seg2[1] <= 999957.0843002755
 R925: - C7 + 1e+06 alpha_ob0_seg2[1] <= 999957.0843002755
 R926: C4 + 1e+06 alpha_ob0_seg2[2] <= 999969.0328499526
 R927: C6 + 1e+06 alpha_ob0_seg2[2] <= 999969.0328499526
 R928: C5 + 1e+06 alpha_ob0_seg2[3] <= 1.0000132657002755e+06
 R929: C7 + 1e+06 alpha_ob0_seg2[3] <= 1.0000132657002755e+06
 R930: alpha_ob0_seg2[0] + alpha_ob0_seg2[1] + alpha_ob0_seg2[2]
   + alpha_ob0_seg2[3] >= 1
 R931: - C4 + 1e+06 alpha_ob1_seg2[0] <= 1.000028417099801e+06
 R932: - C6 + 1e+06 alpha_ob1_seg2[0] <= 1.000028417099801e+06
 R933: - C5 + 1e+06 alpha_ob1_seg2[1] <= 999984.1843155898
 R934: - C7 + 1e+06 alpha_ob1_seg2[1] <= 999984.1843155898
 R935: C4 + 1e+06 alpha_ob1_seg2[2] <= 999957.4328998009
 R936: C6 + 1e+06 alpha_ob1_seg2[2] <= 999957.4328998009
 R937: C5 + 1e+06 alpha_ob1_seg2[3] <= 1.0000132657155899e+06
 R938: C7 + 1e+06 alpha_ob1_seg2[3] <= 1.0000132657155899e+06
 R939: alpha_ob1_seg2[0] + alpha_ob1_seg2[1] + alpha_ob1_seg2[2]
   + alpha_ob1_seg2[3] >= 1
 R940: - C4 + 1e+06 alpha_ob2_seg2[0] <= 1.0000400171500475e+06
 R941: - C6 + 1e+06 alpha_ob2_seg2[0] <= 1.0000400171500475e+06
 R942: - C5 + 1e+06 alpha_ob2_seg2[1] <= 999957.0842997245
 R943: - C7 + 1e+06 alpha_ob2_seg2[1] <= 999957.0842997245
 R944: C4 + 1e+06 alpha_ob2_seg2[2] <= 999957.4329500474
 R945: C6 + 1e+06 alpha_ob2_seg2[2] <= 999957.4329500474
 R946: C5 + 1e+06 alpha_ob2_seg2[3] <= 1.0000132656997246e+06
 R947: C7 + 1e+06 alpha_ob2_seg2[3] <= 1.0000132656997246e+06
 R948: alpha_ob2_seg2[0] + alpha_ob2_seg2[1] + alpha_ob2_seg2[2]
   + alpha_ob2_seg2[3] >= 1
 R949: - C4 + 1e+06 alpha_ob3_seg2[0] <= 1.0000109161752058e+06
 R950: - C6 + 1e+06 alpha_ob3_seg2[0] <= 1.0000109161752058e+06
 R951: - C5 + 1e+06 alpha_ob3_seg2[1] <= 999970.2681002755
 R952: - C7 + 1e+06 alpha_ob3_seg2[1] <= 999970.2681002755
 R953: C4 + 1e+06 alpha_ob3_seg2[2] <= 999986.5337752058
 R954: C6 + 1e+06 alpha_ob3_seg2[2] <= 999986.5337752058
 R955: C5 + 1e+06 alpha_ob3_seg2[3] <= 1.0000138319002755e+06
 R956: C7 + 1e+06 alpha_ob3_seg2[3] <= 1.0000138319002755e+06
 R957: alpha_ob3_seg2[0] + alpha_ob3_seg2[1] + alpha_ob3_seg2[2]
   + alpha_ob3_seg2[3] >= 1
 R958: - C4 + 1e+06 alpha_ob4_seg2[0] <= 1.000010916199801e+06
 R959: - C6 + 1e+06 alpha_ob4_seg2[0] <= 1.000010916199801e+06
 R960: - C5 + 1e+06 alpha_ob4_seg2[1] <= 999983.6181179117
 R961: - C7 + 1e+06 alpha_ob4_seg2[1] <= 999983.6181179117
 R962: C4 + 1e+06 alpha_ob4_seg2[2] <= 999973.183799801
 R963: C6 + 1e+06 alpha_ob4_seg2[2] <= 999973.183799801
 R964: C5 + 1e+06 alpha_ob4_seg2[3] <= 1.0000138319179118e+06
 R965: C7 + 1e+06 alpha_ob4_seg2[3] <= 1.0000138319179118e+06
 R966: alpha_ob4_seg2[0] + alpha_ob4_seg2[1] + alpha_ob4_seg2[2]
   + alpha_ob4_seg2[3] >= 1
 R967: - C4 + 1e+06 alpha_ob5_seg2[0] <= 999992.49102
 R968: - C6 + 1e+06 alpha_ob5_seg2[0] <= 999992.49102
 R969: - C5 + 1e+06 alpha_ob5_seg2[1] <= 999957.1016000001
 R970: - C7 + 1e+06 alpha_ob5_seg2[1] <= 999957.1016000001
 R971: C4 + 1e+06 alpha_ob5_seg2[2] <= 999986.35898
 R972: C6 + 1e+06 alpha_ob5_seg2[2] <= 999986.35898
 R973: C5 + 1e+06 alpha_ob5_seg2[3] <= 1.0000403484e+06
 R974: C7 + 1e+06 alpha_ob5_seg2[3] <= 1.0000403484e+06
 R975: alpha_ob5_seg2[0] + alpha_ob5_seg2[1] + alpha_ob5_seg2[2]
   + alpha_ob5_seg2[3] >= 1
 R976: - C4 + 1e+06 alpha_ob6_seg2[0] <= 999981.3909999999
 R977: - C6 + 1e+06 alpha_ob6_seg2[0] <= 999981.3909999999
 R978: - C5 + 1e+06 alpha_ob6_seg2[1] <= 999957.1016000001
 R979: - C7 + 1e+06 alpha_ob6_seg2[1] <= 999957.1016000001
 R980: C4 + 1e+06 alpha_ob6_seg2[2] <= 1.000004959e+06
 R981: C6 + 1e+06 alpha_ob6_seg2[2] <= 1.000004959e+06
 R982: C5 + 1e+06 alpha_ob6_seg2[3] <= 1.0000403484e+06
 R983: C7 + 1e+06 alpha_ob6_seg2[3] <= 1.0000403484e+06
 R984: alpha_ob6_seg2[0] + alpha_ob6_seg2[1] + alpha_ob6_seg2[2]
   + alpha_ob6_seg2[3] >= 1
 R985: - C4 + 1e+06 alpha_ob7_seg2[0] <= 999981.3909825523
 R986: - C6 + 1e+06 alpha_ob7_seg2[0] <= 999981.3909825523
 R987: - C5 + 1e+06 alpha_ob7_seg2[1] <= 999957.1016002755
 R988: - C7 + 1e+06 alpha_ob7_seg2[1] <= 999957.1016002755
 R989: C4 + 1e+06 alpha_ob7_seg2[2] <= 1.0000160589825523e+06
 R990: C6 + 1e+06 alpha_ob7_seg2[2] <= 1.0000160589825523e+06
 R991: C5 + 1e+06 alpha_ob7_seg2[3] <= 1.0000309984002755e+06
 R992: C7 + 1e+06 alpha_ob7_seg2[3] <= 1.0000309984002755e+06
 R993: alpha_ob7_seg2[0] + alpha_ob7_seg2[1] + alpha_ob7_seg2[2]
   + alpha_ob7_seg2[3] >= 1
 R994: - C4 + 1e+06 alpha_ob8_seg2[0] <= 999981.3867954084
 R995: - C6 + 1e+06 alpha_ob8_seg2[0] <= 999981.3867954084
 R996: - C5 + 1e+06 alpha_ob8_seg2[1] <= 999968.8595002755
 R997: - C7 + 1e+06 alpha_ob8_seg2[1] <= 999968.8595002755
 R998: C4 + 1e+06 alpha_ob8_seg2[2] <= 1.0000160631954084e+06
 R999: C6 + 1e+06 alpha_ob8_seg2[2] <= 1.0000160631954084e+06
 R1000: C5 + 1e+06 alpha_ob8_seg2[3] <= 1.0000262405002755e+06
 R1001: C7 + 1e+06 alpha_ob8_seg2[3] <= 1.0000262405002755e+06
 R1002: alpha_ob8_seg2[0] + alpha_ob8_seg2[1] + alpha_ob8_seg2[2]
   + alpha_ob8_seg2[3] >= 1
 R1003: - C4 + 1e+06 alpha_ob9_seg2[0] <= 999981.386799801
 R1004: - C6 + 1e+06 alpha_ob9_seg2[0] <= 999981.386799801
 R1005: - C5 + 1e+06 alpha_ob9_seg2[1] <= 999971.2095013268
 R1006: - C7 + 1e+06 alpha_ob9_seg2[1] <= 999971.2095013268
 R1007: C4 + 1e+06 alpha_ob9_seg2[2] <= 1.0000152131998009e+06
 R1008: C6 + 1e+06 alpha_ob9_seg2[2] <= 1.0000152131998009e+06
 R1009: C5 + 1e+06 alpha_ob9_seg2[3] <= 1.0000262405013267e+06
 R1010: C7 + 1e+06 alpha_ob9_seg2[3] <= 1.0000262405013267e+06
 R1011: alpha_ob9_seg2[0] + alpha_ob9_seg2[1] + alpha_ob9_seg2[2]
   + alpha_ob9_seg2[3] >= 1
 R1012: - C4 + 1e+06 alpha_ob10_seg2[0] <= 999982.2368022958
 R1013: - C6 + 1e+06 alpha_ob10_seg2[0] <= 999982.2368022958
 R1014: - C5 + 1e+06 alpha_ob10_seg2[1] <= 999970.1094997245
 R1015: - C7 + 1e+06 alpha_ob10_seg2[1] <= 999970.1094997245
 R1016: C4 + 1e+06 alpha_ob10_seg2[2] <= 1.0000152132022957e+06
 R1017: C6 + 1e+06 alpha_ob10_seg2[2] <= 1.0000152132022957e+06
 R1018: C5 + 1e+06 alpha_ob10_seg2[3] <= 1.0000262404997245e+06
 R1019: C7 + 1e+06 alpha_ob10_seg2[3] <= 1.0000262404997245e+06
 R1020: alpha_ob10_seg2[0] + alpha_ob10_seg2[1] + alpha_ob10_seg2[2]
   + alpha_ob10_seg2[3] >= 1
 R1021: - C4 + 1e+06 alpha_ob11_seg2[0] <= 999982.236799801
 R1022: - C6 + 1e+06 alpha_ob11_seg2[0] <= 999982.236799801
 R1023: - C5 + 1e+06 alpha_ob11_seg2[1] <= 999970.1095291895
 R1024: - C7 + 1e+06 alpha_ob11_seg2[1] <= 999970.1095291895
 R1025: C4 + 1e+06 alpha_ob11_seg2[2] <= 999993.3631998011
 R1026: C6 + 1e+06 alpha_ob11_seg2[2] <= 999993.3631998011
 R1027: C5 + 1e+06 alpha_ob11_seg2[3] <= 1.0000273405291894e+06
 R1028: C7 + 1e+06 alpha_ob11_seg2[3] <= 1.0000273405291894e+06
 R1029: alpha_ob11_seg2[0] + alpha_ob11_seg2[1] + alpha_ob11_seg2[2]
   + alpha_ob11_seg2[3] >= 1
 R1030: - C4 + 1e+06 alpha_ob12_seg2[0] <= 999981.6050802565
 R1031: - C6 + 1e+06 alpha_ob12_seg2[0] <= 999981.6050802565
 R1032: - C5 + 1e+06 alpha_ob12_seg2[1] <= 999972.8942002755
 R1033: - C7 + 1e+06 alpha_ob12_seg2[1] <= 999972.8942002755
 R1034: C4 + 1e+06 alpha_ob12_seg2[2] <= 1.0000158448802566e+06
 R1035: C6 + 1e+06 alpha_ob12_seg2[2] <= 1.0000158448802566e+06
 R1036: C5 + 1e+06 alpha_ob12_seg2[3] <= 1.0000139558002756e+06
 R1037: C7 + 1e+06 alpha_ob12_seg2[3] <= 1.0000139558002756e+06
 R1038: alpha_ob12_seg2[0] + alpha_ob12_seg2[1] + alpha_ob12_seg2[2]
   + alpha_ob12_seg2[3] >= 1
 R1039: - C4 + 1e+06 alpha_ob13_seg2[0] <= 999981.605149801
 R1040: - C6 + 1e+06 alpha_ob13_seg2[0] <= 999981.605149801
 R1041: - C5 + 1e+06 alpha_ob13_seg2[1] <= 999983.4942391404
 R1042: - C7 + 1e+06 alpha_ob13_seg2[1] <= 999983.4942391404
 R1043: C4 + 1e+06 alpha_ob13_seg2[2] <= 999986.4948498011
 R1044: C6 + 1e+06 alpha_ob13_seg2[2] <= 999986.4948498011
 R1045: C5 + 1e+06 alpha_ob13_seg2[3] <= 1.0000139558391405e+06
 R1046: C7 + 1e+06 alpha_ob13_seg2[3] <= 1.0000139558391405e+06
 R1047: alpha_ob13_seg2[0] + alpha_ob13_seg2[1] + alpha_ob13_seg2[2]
   + alpha_ob13_seg2[3] >= 1
 R1048: - C4 + 1e+06 alpha_ob14_seg2[0] <= 999997.6803502565
 R1049: - C6 + 1e+06 alpha_ob14_seg2[0] <= 999997.6803502565
 R1050: - C5 + 1e+06 alpha_ob14_seg2[1] <= 999972.7268002755
 R1051: - C7 + 1e+06 alpha_ob14_seg2[1] <= 999972.7268002755
 R1052: C4 + 1e+06 alpha_ob14_seg2[2] <= 999999.7696102565
 R1053: C6 + 1e+06 alpha_ob14_seg2[2] <= 999999.7696102565
 R1054: C5 + 1e+06 alpha_ob14_seg2[3] <= 1.0000141232002755e+06
 R1055: C7 + 1e+06 alpha_ob14_seg2[3] <= 1.0000141232002755e+06
 R1056: alpha_ob14_seg2[0] + alpha_ob14_seg2[1] + alpha_ob14_seg2[2]
   + alpha_ob14_seg2[3] >= 1
 R1057: - C4 + 1e+06 alpha_ob15_seg2[0] <= 1.0000038341502565e+06
 R1058: - C6 + 1e+06 alpha_ob15_seg2[0] <= 1.0000038341502565e+06
 R1059: - C5 + 1e+06 alpha_ob15_seg2[1] <= 999972.6012002755
 R1060: - C7 + 1e+06 alpha_ob15_seg2[1] <= 999972.6012002755
 R1061: C4 + 1e+06 alpha_ob15_seg2[2] <= 999993.6158102566
 R1062: C6 + 1e+06 alpha_ob15_seg2[2] <= 999993.6158102566
 R1063: C5 + 1e+06 alpha_ob15_seg2[3] <= 1.0000142488002755e+06
 R1064: C7 + 1e+06 alpha_ob15_seg2[3] <= 1.0000142488002755e+06
 R1065: alpha_ob15_seg2[0] + alpha_ob15_seg2[1] + alpha_ob15_seg2[2]
   + alpha_ob15_seg2[3] >= 1
 R1066: - C4 + 1e+06 alpha_ob16_seg2[0] <= 1.0000162874063333e+06
 R1067: - C6 + 1e+06 alpha_ob16_seg2[0] <= 1.0000162874063333e+06
 R1068: - C5 + 1e+06 alpha_ob16_seg2[1] <= 999982.9439602756
 R1069: - C7 + 1e+06 alpha_ob16_seg2[1] <= 999982.9439602756
 R1070: C4 + 1e+06 alpha_ob16_seg2[2] <= 999981.1624063333
 R1071: C6 + 1e+06 alpha_ob16_seg2[2] <= 999981.1624063333
 R1072: C5 + 1e+06 alpha_ob16_seg2[3] <= 999963.6560402757
 R1073: C7 + 1e+06 alpha_ob16_seg2[3] <= 999963.6560402757
 R1074: alpha_ob16_seg2[0] + alpha_ob16_seg2[1] + alpha_ob16_seg2[2]
   + alpha_ob16_seg2[3] >= 1
 R1075: - C4 + 1e+06 alpha_ob17_seg2[0] <= 999980.937509
 R1076: - C6 + 1e+06 alpha_ob17_seg2[0] <= 999980.937509
 R1077: - C5 + 1e+06 alpha_ob17_seg2[1] <= 1.000034894e+06
 R1078: - C7 + 1e+06 alpha_ob17_seg2[1] <= 1.000034894e+06
 R1079: C4 + 1e+06 alpha_ob17_seg2[2] <= 999981.162491
 R1080: C6 + 1e+06 alpha_ob17_seg2[2] <= 999981.162491
 R1081: C5 + 1e+06 alpha_ob17_seg2[3] <= 999962.556
 R1082: C7 + 1e+06 alpha_ob17_seg2[3] <= 999962.556
 R1083: alpha_ob17_seg2[0] + alpha_ob17_seg2[1] + alpha_ob17_seg2[2]
   + alpha_ob17_seg2[3] >= 1
 R1084: - C4 + 1e+06 alpha_ob18_seg2[0] <= 999990.2670277582
 R1085: - C6 + 1e+06 alpha_ob18_seg2[0] <= 999990.2670277582
 R1086: - C5 + 1e+06 alpha_ob18_seg2[1] <= 1.0000347702002756e+06
 R1087: - C7 + 1e+06 alpha_ob18_seg2[1] <= 1.0000347702002756e+06
 R1088: C4 + 1e+06 alpha_ob18_seg2[2] <= 1.0000071828877581e+06
 R1089: C6 + 1e+06 alpha_ob18_seg2[2] <= 1.0000071828877581e+06
 R1090: C5 + 1e+06 alpha_ob18_seg2[3] <= 999939.8298002756
 R1091: C7 + 1e+06 alpha_ob18_seg2[3] <= 999939.8298002756
 R1092: alpha_ob18_seg2[0] + alpha_ob18_seg2[1] + alpha_ob18_seg2[2]
   + alpha_ob18_seg2[3] >= 1
 R1093: - C4 + 1e+06 alpha_ob19_seg2[0] <= 999990.267069801
 R1094: - C6 + 1e+06 alpha_ob19_seg2[0] <= 999990.267069801
 R1095: - C5 + 1e+06 alpha_ob19_seg2[1] <= 1.0000576202069657e+06
 R1096: - C7 + 1e+06 alpha_ob19_seg2[1] <= 1.0000576202069657e+06
 R1097: C4 + 1e+06 alpha_ob19_seg2[2] <= 1.000002082929801e+06
 R1098: C6 + 1e+06 alpha_ob19_seg2[2] <= 1.000002082929801e+06
 R1099: C5 + 1e+06 alpha_ob19_seg2[3] <= 999939.8298069657
 R1100: C7 + 1e+06 alpha_ob19_seg2[3] <= 999939.8298069657
 R1101: alpha_ob19_seg2[0] + alpha_ob19_seg2[1] + alpha_ob19_seg2[2]
   + alpha_ob19_seg2[3] >= 1
 R1102: - C4 + 1e+06 alpha_ob20_seg2[0] <= 999991.073732
 R1103: - C6 + 1e+06 alpha_ob20_seg2[0] <= 999991.073732
 R1104: - C5 + 1e+06 alpha_ob20_seg2[1] <= 999952.2125
 R1105: - C7 + 1e+06 alpha_ob20_seg2[1] <= 999952.2125
 R1106: C4 + 1e+06 alpha_ob20_seg2[2] <= 999990.776268
 R1107: C6 + 1e+06 alpha_ob20_seg2[2] <= 999990.776268
 R1108: C5 + 1e+06 alpha_ob20_seg2[3] <= 1.0000452375e+06
 R1109: C7 + 1e+06 alpha_ob20_seg2[3] <= 1.0000452375e+06
 R1110: alpha_ob20_seg2[0] + alpha_ob20_seg2[1] + alpha_ob20_seg2[2]
   + alpha_ob20_seg2[3] >= 1
 R1111: - C4 + 1e+06 alpha_ob21_seg2[0] <= 999985.657099801
 R1112: - C6 + 1e+06 alpha_ob21_seg2[0] <= 999985.657099801
 R1113: - C5 + 1e+06 alpha_ob21_seg2[1] <= 1.0000576202063023e+06
 R1114: - C7 + 1e+06 alpha_ob21_seg2[1] <= 1.0000576202063023e+06
 R1115: C4 + 1e+06 alpha_ob21_seg2[2] <= 1.000007192899801e+06
 R1116: C6 + 1e+06 alpha_ob21_seg2[2] <= 1.000007192899801e+06
 R1117: C5 + 1e+06 alpha_ob21_seg2[3] <= 999939.8298063023
 R1118: C7 + 1e+06 alpha_ob21_seg2[3] <= 999939.8298063023
 R1119: alpha_ob21_seg2[0] + alpha_ob21_seg2[1] + alpha_ob21_seg2[2]
   + alpha_ob21_seg2[3] >= 1
 R1120: - C4 + 1e+06 alpha_ob22_seg2[0] <= 999994.771829801
 R1121: - C6 + 1e+06 alpha_ob22_seg2[0] <= 999994.771829801
 R1122: - C5 + 1e+06 alpha_ob22_seg2[1] <= 1.0000615983063023e+06
 R1123: - C7 + 1e+06 alpha_ob22_seg2[1] <= 1.0000615983063023e+06
 R1124: C4 + 1e+06 alpha_ob22_seg2[2] <= 999998.078169801
 R1125: C6 + 1e+06 alpha_ob22_seg2[2] <= 999998.078169801
 R1126: C5 + 1e+06 alpha_ob22_seg2[3] <= 999935.8517063023
 R1127: C7 + 1e+06 alpha_ob22_seg2[3] <= 999935.8517063023
 R1128: alpha_ob22_seg2[0] + alpha_ob22_seg2[1] + alpha_ob22_seg2[2]
   + alpha_ob22_seg2[3] >= 1
 R1129: - C4 + 1e+06 alpha_ob23_seg2[0] <= 999980.9374508709
 R1130: - C6 + 1e+06 alpha_ob23_seg2[0] <= 999980.9374508709
 R1131: - C5 + 1e+06 alpha_ob23_seg2[1] <= 1.0000348940002755e+06
 R1132: - C7 + 1e+06 alpha_ob23_seg2[1] <= 1.0000348940002755e+06
 R1133: C4 + 1e+06 alpha_ob23_seg2[2] <= 1.0000165124508708e+06
 R1134: C6 + 1e+06 alpha_ob23_seg2[2] <= 1.0000165124508708e+06
 R1135: C5 + 1e+06 alpha_ob23_seg2[3] <= 999935.9560002756
 R1136: C7 + 1e+06 alpha_ob23_seg2[3] <= 999935.9560002756
 R1137: alpha_ob23_seg2[0] + alpha_ob23_seg2[1] + alpha_ob23_seg2[2]
   + alpha_ob23_seg2[3] >= 1
 R1138: - C4 + 1e+06 alpha_ob24_seg2[0] <= 999980.937499801
 R1139: - C6 + 1e+06 alpha_ob24_seg2[0] <= 999980.937499801
 R1140: - C5 + 1e+06 alpha_ob24_seg2[1] <= 1.0000614940072973e+06
 R1141: - C7 + 1e+06 alpha_ob24_seg2[1] <= 1.0000614940072973e+06
 R1142: C4 + 1e+06 alpha_ob24_seg2[2] <= 1.000011162499801e+06
 R1143: C6 + 1e+06 alpha_ob24_seg2[2] <= 1.000011162499801e+06
 R1144: C5 + 1e+06 alpha_ob24_seg2[3] <= 999935.9560072974
 R1145: C7 + 1e+06 alpha_ob24_seg2[3] <= 999935.9560072974
 R1146: alpha_ob24_seg2[0] + alpha_ob24_seg2[1] + alpha_ob24_seg2[2]
   + alpha_ob24_seg2[3] >= 1
 R1147: - C4 + 1e+06 alpha_ob25_seg2[0] <= 999956.8394000001
 R1148: - C6 + 1e+06 alpha_ob25_seg2[0] <= 999956.8394000001
 R1149: - C5 + 1e+06 alpha_ob25_seg2[1] <= 999952.2463
 R1150: - C7 + 1e+06 alpha_ob25_seg2[1] <= 999952.2463
 R1151: C4 + 1e+06 alpha_ob25_seg2[2] <= 1.0000097606e+06
 R1152: C6 + 1e+06 alpha_ob25_seg2[2] <= 1.0000097606e+06
 R1153: C5 + 1e+06 alpha_ob25_seg2[3] <= 1.0000452037e+06
 R1154: C7 + 1e+06 alpha_ob25_seg2[3] <= 1.0000452037e+06
 R1155: alpha_ob25_seg2[0] + alpha_ob25_seg2[1] + alpha_ob25_seg2[2]
   + alpha_ob25_seg2[3] >= 1
 R1156: - C4 + 1e+06 alpha_ob26_seg2[0] <= 999980.9375348955
 R1157: - C6 + 1e+06 alpha_ob26_seg2[0] <= 999980.9375348955
 R1158: - C5 + 1e+06 alpha_ob26_seg2[1] <= 1.0000160439997246e+06
 R1159: - C7 + 1e+06 alpha_ob26_seg2[1] <= 1.0000160439997246e+06
 R1160: C4 + 1e+06 alpha_ob26_seg2[2] <= 1.0000165125348954e+06
 R1161: C6 + 1e+06 alpha_ob26_seg2[2] <= 1.0000165125348954e+06
 R1162: C5 + 1e+06 alpha_ob26_seg2[3] <= 999962.5559997245
 R1163: C7 + 1e+06 alpha_ob26_seg2[3] <= 999962.5559997245
 R1164: alpha_ob26_seg2[0] + alpha_ob26_seg2[1] + alpha_ob26_seg2[2]
   + alpha_ob26_seg2[3] >= 1
 R1165: - C4 + 1e+06 alpha_ob27_seg2[0] <= 999956.839399801
 R1166: - C6 + 1e+06 alpha_ob27_seg2[0] <= 999956.839399801
 R1167: - C5 + 1e+06 alpha_ob27_seg2[1] <= 999925.3963434525
 R1168: - C7 + 1e+06 alpha_ob27_seg2[1] <= 999925.3963434525
 R1169: C4 + 1e+06 alpha_ob27_seg2[2] <= 1.0000080105998011e+06
 R1170: C6 + 1e+06 alpha_ob27_seg2[2] <= 1.0000080105998011e+06
 R1171: C5 + 1e+06 alpha_ob27_seg2[3] <= 1.0000720537434525e+06
 R1172: C7 + 1e+06 alpha_ob27_seg2[3] <= 1.0000720537434525e+06
 R1173: alpha_ob27_seg2[0] + alpha_ob27_seg2[1] + alpha_ob27_seg2[2]
   + alpha_ob27_seg2[3] >= 1
 R1174: - C4 + 1e+06 alpha_ob28_seg2[0] <= 999980.9374998009
 R1175: - C6 + 1e+06 alpha_ob28_seg2[0] <= 999980.9374998009
 R1176: - C5 + 1e+06 alpha_ob28_seg2[1] <= 1.0000160440053071e+06
 R1177: - C7 + 1e+06 alpha_ob28_seg2[1] <= 1.0000160440053071e+06
 R1178: C4 + 1e+06 alpha_ob28_seg2[2] <= 1.000012662499801e+06
 R1179: C6 + 1e+06 alpha_ob28_seg2[2] <= 1.000012662499801e+06
 R1180: C5 + 1e+06 alpha_ob28_seg2[3] <= 999981.4060053072
 R1181: C7 + 1e+06 alpha_ob28_seg2[3] <= 999981.4060053072
 R1182: alpha_ob28_seg2[0] + alpha_ob28_seg2[1] + alpha_ob28_seg2[2]
   + alpha_ob28_seg2[3] >= 1
 R1183: - C4 + 1e+06 alpha_ob29_seg2[0] <= 999983.6053459151
 R1184: - C6 + 1e+06 alpha_ob29_seg2[0] <= 999983.6053459151
 R1185: - C5 + 1e+06 alpha_ob29_seg2[1] <= 999987.7722497246
 R1186: - C7 + 1e+06 alpha_ob29_seg2[1] <= 999987.7722497246
 R1187: C4 + 1e+06 alpha_ob29_seg2[2] <= 1.000013844745915e+06
 R1188: C6 + 1e+06 alpha_ob29_seg2[2] <= 1.000013844745915e+06
 R1189: C5 + 1e+06 alpha_ob29_seg2[3] <= 999984.8277497246
 R1190: C7 + 1e+06 alpha_ob29_seg2[3] <= 999984.8277497246
 R1191: alpha_ob29_seg2[0] + alpha_ob29_seg2[1] + alpha_ob29_seg2[2]
   + alpha_ob29_seg2[3] >= 1
 R1192: - C4 + 1e+06 alpha_ob30_seg2[0] <= 999958.8717898987
 R1193: - C6 + 1e+06 alpha_ob30_seg2[0] <= 999958.8717898987
 R1194: - C5 + 1e+06 alpha_ob30_seg2[1] <= 999987.7837602755
 R1195: - C7 + 1e+06 alpha_ob30_seg2[1] <= 999987.7837602755
 R1196: C4 + 1e+06 alpha_ob30_seg2[2] <= 1.0000385781898986e+06
 R1197: C6 + 1e+06 alpha_ob30_seg2[2] <= 1.0000385781898986e+06
 R1198: C5 + 1e+06 alpha_ob30_seg2[3] <= 1.0000043162402755e+06
 R1199: C7 + 1e+06 alpha_ob30_seg2[3] <= 1.0000043162402755e+06
 R1200: alpha_ob30_seg2[0] + alpha_ob30_seg2[1] + alpha_ob30_seg2[2]
   + alpha_ob30_seg2[3] >= 1
 R1201: - C4 + 1e+06 alpha_ob31_seg2[0] <= 999939.2718
 R1202: - C6 + 1e+06 alpha_ob31_seg2[0] <= 999939.2718
 R1203: - C5 + 1e+06 alpha_ob31_seg2[1] <= 999993.13376
 R1204: - C7 + 1e+06 alpha_ob31_seg2[1] <= 999993.13376
 R1205: C4 + 1e+06 alpha_ob31_seg2[2] <= 1.0000385782e+06
 R1206: C6 + 1e+06 alpha_ob31_seg2[2] <= 1.0000385782e+06
 R1207: C5 + 1e+06 alpha_ob31_seg2[3] <= 1.00000431624e+06
 R1208: C7 + 1e+06 alpha_ob31_seg2[3] <= 1.00000431624e+06
 R1209: alpha_ob31_seg2[0] + alpha_ob31_seg2[1] + alpha_ob31_seg2[2]
   + alpha_ob31_seg2[3] >= 1
 R1210: - C4 + 1e+06 alpha_ob32_seg2[0] <= 999939.2718101013
 R1211: - C6 + 1e+06 alpha_ob32_seg2[0] <= 999939.2718101013
 R1212: - C5 + 1e+06 alpha_ob32_seg2[1] <= 999987.7837597246
 R1213: - C7 + 1e+06 alpha_ob32_seg2[1] <= 999987.7837597246
 R1214: C4 + 1e+06 alpha_ob32_seg2[2] <= 1.0000581782101013e+06
 R1215: C6 + 1e+06 alpha_ob32_seg2[2] <= 1.0000581782101013e+06
 R1216: C5 + 1e+06 alpha_ob32_seg2[3] <= 1.0000043162397245e+06
 R1217: C7 + 1e+06 alpha_ob32_seg2[3] <= 1.0000043162397245e+06
 R1218: alpha_ob32_seg2[0] + alpha_ob32_seg2[1] + alpha_ob32_seg2[2]
   + alpha_ob32_seg2[3] >= 1
 R1219: - C4 + 1e+06 alpha_ob33_seg2[0] <= 999939.271799801
 R1220: - C6 + 1e+06 alpha_ob33_seg2[0] <= 999939.271799801
 R1221: - C5 + 1e+06 alpha_ob33_seg2[1] <= 999987.7838262042
 R1222: - C7 + 1e+06 alpha_ob33_seg2[1] <= 999987.7838262042
 R1223: C4 + 1e+06 alpha_ob33_seg2[2] <= 1.000038578199801e+06
 R1224: C6 + 1e+06 alpha_ob33_seg2[2] <= 1.000038578199801e+06
 R1225: C5 + 1e+06 alpha_ob33_seg2[3] <= 1.0000096662262043e+06
 R1226: C7 + 1e+06 alpha_ob33_seg2[3] <= 1.0000096662262043e+06
 R1227: alpha_ob33_seg2[0] + alpha_ob33_seg2[1] + alpha_ob33_seg2[2]
   + alpha_ob33_seg2[3] >= 1
 R1228: - C4 + 1e+06 alpha_ob34_seg2[0] <= 999926.0708
 R1229: - C6 + 1e+06 alpha_ob34_seg2[0] <= 999926.0708
 R1230: - C5 + 1e+06 alpha_ob34_seg2[1] <= 999983.7196
 R1231: - C7 + 1e+06 alpha_ob34_seg2[1] <= 999983.7196
 R1232: C4 + 1e+06 alpha_ob34_seg2[2] <= 1.0000250292e+06
 R1233: C6 + 1e+06 alpha_ob34_seg2[2] <= 1.0000250292e+06
 R1234: C5 + 1e+06 alpha_ob34_seg2[3] <= 1.0000137304e+06
 R1235: C7 + 1e+06 alpha_ob34_seg2[3] <= 1.0000137304e+06
 R1236: alpha_ob34_seg2[0] + alpha_ob34_seg2[1] + alpha_ob34_seg2[2]
   + alpha_ob34_seg2[3] >= 1
 R1237: - C4 + 1e+06 alpha_ob35_seg2[0] <= 999926.0708293857
 R1238: - C6 + 1e+06 alpha_ob35_seg2[0] <= 999926.0708293857
 R1239: - C5 + 1e+06 alpha_ob35_seg2[1] <= 999967.8695997245
 R1240: - C7 + 1e+06 alpha_ob35_seg2[1] <= 999967.8695997245
 R1241: C4 + 1e+06 alpha_ob35_seg2[2] <= 1.0000713792293856e+06
 R1242: C6 + 1e+06 alpha_ob35_seg2[2] <= 1.0000713792293856e+06
 R1243: C5 + 1e+06 alpha_ob35_seg2[3] <= 1.0000137303997246e+06
 R1244: C7 + 1e+06 alpha_ob35_seg2[3] <= 1.0000137303997246e+06
 R1245: alpha_ob35_seg2[0] + alpha_ob35_seg2[1] + alpha_ob35_seg2[2]
   + alpha_ob35_seg2[3] >= 1
 R1246: - C4 + 1e+06 alpha_ob36_seg2[0] <= 999972.6917508709
 R1247: - C6 + 1e+06 alpha_ob36_seg2[0] <= 999972.6917508709
 R1248: - C5 + 1e+06 alpha_ob36_seg2[1] <= 999957.1644002756
 R1249: - C7 + 1e+06 alpha_ob36_seg2[1] <= 999957.1644002756
 R1250: C4 + 1e+06 alpha_ob36_seg2[2] <= 1.0000247581508709e+06
 R1251: C6 + 1e+06 alpha_ob36_seg2[2] <= 1.0000247581508709e+06
 R1252: C5 + 1e+06 alpha_ob36_seg2[3] <= 1.0000136856002755e+06
 R1253: C7 + 1e+06 alpha_ob36_seg2[3] <= 1.0000136856002755e+06
 R1254: alpha_ob36_seg2[0] + alpha_ob36_seg2[1] + alpha_ob36_seg2[2]
   + alpha_ob36_seg2[3] >= 1
 R1255: - C4 + 1e+06 alpha_ob37_seg2[0] <= 999939.0009674003
 R1256: - C6 + 1e+06 alpha_ob37_seg2[0] <= 999939.0009674003
 R1257: - C5 + 1e+06 alpha_ob37_seg2[1] <= 999961.9516002756
 R1258: - C7 + 1e+06 alpha_ob37_seg2[1] <= 999961.9516002756
 R1259: C4 + 1e+06 alpha_ob37_seg2[2] <= 1.0000584489674004e+06
 R1260: C6 + 1e+06 alpha_ob37_seg2[2] <= 1.0000584489674004e+06
 R1261: C5 + 1e+06 alpha_ob37_seg2[3] <= 1.0000178984002755e+06
 R1262: C7 + 1e+06 alpha_ob37_seg2[3] <= 1.0000178984002755e+06
 R1263: alpha_ob37_seg2[0] + alpha_ob37_seg2[1] + alpha_ob37_seg2[2]
   + alpha_ob37_seg2[3] >= 1
 R1264: - C4 + 1e+06 alpha_ob38_seg2[0] <= 999994.9164998011
 R1265: - C6 + 1e+06 alpha_ob38_seg2[0] <= 999994.9164998011
 R1266: - C5 + 1e+06 alpha_ob38_seg2[1] <= 999925.4284782809
 R1267: - C7 + 1e+06 alpha_ob38_seg2[1] <= 999925.4284782809
 R1268: C4 + 1e+06 alpha_ob38_seg2[2] <= 999943.6834998011
 R1269: C6 + 1e+06 alpha_ob38_seg2[2] <= 999943.6834998011
 R1270: C5 + 1e+06 alpha_ob38_seg2[3] <= 1.0000720216782809e+06
 R1271: C7 + 1e+06 alpha_ob38_seg2[3] <= 1.0000720216782809e+06
 R1272: alpha_ob38_seg2[0] + alpha_ob38_seg2[1] + alpha_ob38_seg2[2]
   + alpha_ob38_seg2[3] >= 1
 R1273: - C4 + 1e+06 alpha_ob39_seg2[0] <= 999914.52327842
 R1274: - C6 + 1e+06 alpha_ob39_seg2[0] <= 999914.52327842
 R1275: - C5 + 1e+06 alpha_ob39_seg2[1] <= 999993.6552362755
 R1276: - C7 + 1e+06 alpha_ob39_seg2[1] <= 999993.6552362755
 R1277: C4 + 1e+06 alpha_ob39_seg2[2] <= 1.0000829266784199e+06
 R1278: C6 + 1e+06 alpha_ob39_seg2[2] <= 1.0000829266784199e+06
 R1279: C5 + 1e+06 alpha_ob39_seg2[3] <= 999992.1947642756
 R1280: C7 + 1e+06 alpha_ob39_seg2[3] <= 999992.1947642756
 R1281: alpha_ob39_seg2[0] + alpha_ob39_seg2[1] + alpha_ob39_seg2[2]
   + alpha_ob39_seg2[3] >= 1
 R1282: - C4 + 1e+06 alpha_ob40_seg2[0] <= 999912.265199801
 R1283: - C6 + 1e+06 alpha_ob40_seg2[0] <= 999912.265199801
 R1284: - C5 + 1e+06 alpha_ob40_seg2[1] <= 999987.7842112777
 R1285: - C7 + 1e+06 alpha_ob40_seg2[1] <= 999987.7842112777
 R1286: C4 + 1e+06 alpha_ob40_seg2[2] <= 1.000076834799801e+06
 R1287: C6 + 1e+06 alpha_ob40_seg2[2] <= 1.000076834799801e+06
 R1288: C5 + 1e+06 alpha_ob40_seg2[3] <= 1.0000096658112778e+06
 R1289: C7 + 1e+06 alpha_ob40_seg2[3] <= 1.0000096658112778e+06
 R1290: alpha_ob40_seg2[0] + alpha_ob40_seg2[1] + alpha_ob40_seg2[2]
   + alpha_ob40_seg2[3] >= 1
 R1291: - C4 + 1e+06 alpha_ob41_seg2[0] <= 999920.6152119379
 R1292: - C6 + 1e+06 alpha_ob41_seg2[0] <= 999920.6152119379
 R1293: - C5 + 1e+06 alpha_ob41_seg2[1] <= 999981.4341997246
 R1294: - C7 + 1e+06 alpha_ob41_seg2[1] <= 999981.4341997246
 R1295: C4 + 1e+06 alpha_ob41_seg2[2] <= 1.0000768348119379e+06
 R1296: C6 + 1e+06 alpha_ob41_seg2[2] <= 1.0000768348119379e+06
 R1297: C5 + 1e+06 alpha_ob41_seg2[3] <= 1.0000096657997245e+06
 R1298: C7 + 1e+06 alpha_ob41_seg2[3] <= 1.0000096657997245e+06
 R1299: alpha_ob41_seg2[0] + alpha_ob41_seg2[1] + alpha_ob41_seg2[2]
   + alpha_ob41_seg2[3] >= 1
 R1300: - C4 + 1e+06 alpha_ob42_seg2[0] <= 999919.2652
 R1301: - C6 + 1e+06 alpha_ob42_seg2[0] <= 999919.2652
 R1302: - C5 + 1e+06 alpha_ob42_seg2[1] <= 999981.4342
 R1303: - C7 + 1e+06 alpha_ob42_seg2[1] <= 999981.4342
 R1304: C4 + 1e+06 alpha_ob42_seg2[2] <= 1.0000768348e+06
 R1305: C6 + 1e+06 alpha_ob42_seg2[2] <= 1.0000768348e+06
 R1306: C5 + 1e+06 alpha_ob42_seg2[3] <= 1.0000160158000001e+06
 R1307: C7 + 1e+06 alpha_ob42_seg2[3] <= 1.0000160158000001e+06
 R1308: alpha_ob42_seg2[0] + alpha_ob42_seg2[1] + alpha_ob42_seg2[2]
   + alpha_ob42_seg2[3] >= 1
 R1309: - C4 + 1e+06 alpha_ob43_seg2[0] <= 999919.2652454559
 R1310: - C6 + 1e+06 alpha_ob43_seg2[0] <= 999919.2652454559
 R1311: - C5 + 1e+06 alpha_ob43_seg2[1] <= 999956.8341997246
 R1312: - C7 + 1e+06 alpha_ob43_seg2[1] <= 999956.8341997246
 R1313: C4 + 1e+06 alpha_ob43_seg2[2] <= 1.0000781848454559e+06
 R1314: C6 + 1e+06 alpha_ob43_seg2[2] <= 1.0000781848454559e+06
 R1315: C5 + 1e+06 alpha_ob43_seg2[3] <= 1.0000160157997246e+06
 R1316: C7 + 1e+06 alpha_ob43_seg2[3] <= 1.0000160157997246e+06
 R1317: alpha_ob43_seg2[0] + alpha_ob43_seg2[1] + alpha_ob43_seg2[2]
   + alpha_ob43_seg2[3] >= 1
 R1318: - C4 + 1e+06 alpha_ob44_seg2[0] <= 999991.83683
 R1319: - C6 + 1e+06 alpha_ob44_seg2[0] <= 999991.83683
 R1320: - C5 + 1e+06 alpha_ob44_seg2[1] <= 999940.589
 R1321: - C7 + 1e+06 alpha_ob44_seg2[1] <= 999940.589
 R1322: C4 + 1e+06 alpha_ob44_seg2[2] <= 1.00000401317e+06
 R1323: C6 + 1e+06 alpha_ob44_seg2[2] <= 1.00000401317e+06
 R1324: C5 + 1e+06 alpha_ob44_seg2[3] <= 1.000056861e+06
 R1325: C7 + 1e+06 alpha_ob44_seg2[3] <= 1.000056861e+06
 R1326: alpha_ob44_seg2[0] + alpha_ob44_seg2[1] + alpha_ob44_seg2[2]
   + alpha_ob44_seg2[3] >= 1
 R1327: - C4 + 1e+06 alpha_ob45_seg2[0] <= 999991.8368084199
 R1328: - C6 + 1e+06 alpha_ob45_seg2[0] <= 999991.8368084199
 R1329: - C5 + 1e+06 alpha_ob45_seg2[1] <= 999940.5890002755
 R1330: - C7 + 1e+06 alpha_ob45_seg2[1] <= 999940.5890002755
 R1331: C4 + 1e+06 alpha_ob45_seg2[2] <= 1.0000056131484199e+06
 R1332: C6 + 1e+06 alpha_ob45_seg2[2] <= 1.0000056131484199e+06
 R1333: C5 + 1e+06 alpha_ob45_seg2[3] <= 1.0000452610002755e+06
 R1334: C7 + 1e+06 alpha_ob45_seg2[3] <= 1.0000452610002755e+06
 R1335: alpha_ob45_seg2[0] + alpha_ob45_seg2[1] + alpha_ob45_seg2[2]
   + alpha_ob45_seg2[3] >= 1
 R1336: - C4 + 1e+06 alpha_ob46_seg2[0] <= 1.0000764021508709e+06
 R1337: - C6 + 1e+06 alpha_ob46_seg2[0] <= 1.0000764021508709e+06
 R1338: - C5 + 1e+06 alpha_ob46_seg2[1] <= 999957.1027002756
 R1339: - C7 + 1e+06 alpha_ob46_seg2[1] <= 999957.1027002756
 R1340: C4 + 1e+06 alpha_ob46_seg2[2] <= 999921.0477508709
 R1341: C6 + 1e+06 alpha_ob46_seg2[2] <= 999921.0477508709
 R1342: C5 + 1e+06 alpha_ob46_seg2[3] <= 1.0000137473002756e+06
 R1343: C7 + 1e+06 alpha_ob46_seg2[3] <= 1.0000137473002756e+06
 R1344: alpha_ob46_seg2[0] + alpha_ob46_seg2[1] + alpha_ob46_seg2[2]
   + alpha_ob46_seg2[3] >= 1
 R1345: - C4 + 1e+06 alpha_ob47_seg2[0] <= 1.0000445522e+06
 R1346: - C6 + 1e+06 alpha_ob47_seg2[0] <= 1.0000445522e+06
 R1347: - C5 + 1e+06 alpha_ob47_seg2[1] <= 999983.7027
 R1348: - C7 + 1e+06 alpha_ob47_seg2[1] <= 999983.7027
 R1349: C4 + 1e+06 alpha_ob47_seg2[2] <= 999921.0477999999
 R1350: C6 + 1e+06 alpha_ob47_seg2[2] <= 999921.0477999999
 R1351: C5 + 1e+06 alpha_ob47_seg2[3] <= 1.0000137473e+06
 R1352: C7 + 1e+06 alpha_ob47_seg2[3] <= 1.0000137473e+06
 R1353: alpha_ob47_seg2[0] + alpha_ob47_seg2[1] + alpha_ob47_seg2[2]
   + alpha_ob47_seg2[3] >= 1
 R1354: - C4 + 1e+06 alpha_ob48_seg2[0] <= 1.0000445522183661e+06
 R1355: - C6 + 1e+06 alpha_ob48_seg2[0] <= 1.0000445522183661e+06
 R1356: - C5 + 1e+06 alpha_ob48_seg2[1] <= 999973.8526997245
 R1357: - C7 + 1e+06 alpha_ob48_seg2[1] <= 999973.8526997245
 R1358: C4 + 1e+06 alpha_ob48_seg2[2] <= 999952.8978183661
 R1359: C6 + 1e+06 alpha_ob48_seg2[2] <= 999952.8978183661
 R1360: C5 + 1e+06 alpha_ob48_seg2[3] <= 1.0000137472997245e+06
 R1361: C7 + 1e+06 alpha_ob48_seg2[3] <= 1.0000137472997245e+06
 R1362: alpha_ob48_seg2[0] + alpha_ob48_seg2[1] + alpha_ob48_seg2[2]
   + alpha_ob48_seg2[3] >= 1
 R1363: - C4 + 1e+06 alpha_ob49_seg2[0] <= 1.000064332081634e+06
 R1364: - C6 + 1e+06 alpha_ob49_seg2[0] <= 1.000064332081634e+06
 R1365: - C5 + 1e+06 alpha_ob49_seg2[1] <= 999973.9401002756
 R1366: - C7 + 1e+06 alpha_ob49_seg2[1] <= 999973.9401002756
 R1367: C4 + 1e+06 alpha_ob49_seg2[2] <= 999933.1178816339
 R1368: C6 + 1e+06 alpha_ob49_seg2[2] <= 999933.1178816339
 R1369: C5 + 1e+06 alpha_ob49_seg2[3] <= 1.0000136599002755e+06
 R1370: C7 + 1e+06 alpha_ob49_seg2[3] <= 1.0000136599002755e+06
 R1371: alpha_ob49_seg2[0] + alpha_ob49_seg2[1] + alpha_ob49_seg2[2]
   + alpha_ob49_seg2[3] >= 1
 R1372: - C4 + 1e+06 alpha_ob50_seg2[0] <= 1.0000643321013774e+06
 R1373: - C6 + 1e+06 alpha_ob50_seg2[0] <= 1.0000643321013774e+06
 R1374: - C5 + 1e+06 alpha_ob50_seg2[1] <= 999983.1900997246
 R1375: - C7 + 1e+06 alpha_ob50_seg2[1] <= 999983.1900997246
 R1376: C4 + 1e+06 alpha_ob50_seg2[2] <= 999933.1179013775
 R1377: C6 + 1e+06 alpha_ob50_seg2[2] <= 999933.1179013775
 R1378: C5 + 1e+06 alpha_ob50_seg2[3] <= 1.0000136598997245e+06
 R1379: C7 + 1e+06 alpha_ob50_seg2[3] <= 1.0000136598997245e+06
 R1380: alpha_ob50_seg2[0] + alpha_ob50_seg2[1] + alpha_ob50_seg2[2]
   + alpha_ob50_seg2[3] >= 1
 R1381: C0 >= 0
 R1382: C1 >= 0
 R1383: C0 <= 85
 R1384: C1 <= 85
 R1385: C2 >= 0
 R1386: C3 >= 0
 R1387: C2 <= 85
 R1388: C3 <= 85
 R1389: C4 >= 0
 R1390: C5 >= 0
 R1391: C4 <= 85
 R1392: C5 <= 85
 R1393: C6 >= 0
 R1394: C7 >= 0
 R1395: C6 <= 85
 R1396: C7 <= 85
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
 alpha_ob50_seg0[2] alpha_ob50_seg0[3] alpha_ob0_seg1[0] alpha_ob0_seg1[1]
 alpha_ob0_seg1[2] alpha_ob0_seg1[3] alpha_ob1_seg1[0] alpha_ob1_seg1[1]
 alpha_ob1_seg1[2] alpha_ob1_seg1[3] alpha_ob2_seg1[0] alpha_ob2_seg1[1]
 alpha_ob2_seg1[2] alpha_ob2_seg1[3] alpha_ob3_seg1[0] alpha_ob3_seg1[1]
 alpha_ob3_seg1[2] alpha_ob3_seg1[3] alpha_ob4_seg1[0] alpha_ob4_seg1[1]
 alpha_ob4_seg1[2] alpha_ob4_seg1[3] alpha_ob5_seg1[0] alpha_ob5_seg1[1]
 alpha_ob5_seg1[2] alpha_ob5_seg1[3] alpha_ob6_seg1[0] alpha_ob6_seg1[1]
 alpha_ob6_seg1[2] alpha_ob6_seg1[3] alpha_ob7_seg1[0] alpha_ob7_seg1[1]
 alpha_ob7_seg1[2] alpha_ob7_seg1[3] alpha_ob8_seg1[0] alpha_ob8_seg1[1]
 alpha_ob8_seg1[2] alpha_ob8_seg1[3] alpha_ob9_seg1[0] alpha_ob9_seg1[1]
 alpha_ob9_seg1[2] alpha_ob9_seg1[3] alpha_ob10_seg1[0] alpha_ob10_seg1[1]
 alpha_ob10_seg1[2] alpha_ob10_seg1[3] alpha_ob11_seg1[0]
 alpha_ob11_seg1[1] alpha_ob11_seg1[2] alpha_ob11_seg1[3]
 alpha_ob12_seg1[0] alpha_ob12_seg1[1] alpha_ob12_seg1[2]
 alpha_ob12_seg1[3] alpha_ob13_seg1[0] alpha_ob13_seg1[1]
 alpha_ob13_seg1[2] alpha_ob13_seg1[3] alpha_ob14_seg1[0]
 alpha_ob14_seg1[1] alpha_ob14_seg1[2] alpha_ob14_seg1[3]
 alpha_ob15_seg1[0] alpha_ob15_seg1[1] alpha_ob15_seg1[2]
 alpha_ob15_seg1[3] alpha_ob16_seg1[0] alpha_ob16_seg1[1]
 alpha_ob16_seg1[2] alpha_ob16_seg1[3] alpha_ob17_seg1[0]
 alpha_ob17_seg1[1] alpha_ob17_seg1[2] alpha_ob17_seg1[3]
 alpha_ob18_seg1[0] alpha_ob18_seg1[1] alpha_ob18_seg1[2]
 alpha_ob18_seg1[3] alpha_ob19_seg1[0] alpha_ob19_seg1[1]
 alpha_ob19_seg1[2] alpha_ob19_seg1[3] alpha_ob20_seg1[0]
 alpha_ob20_seg1[1] alpha_ob20_seg1[2] alpha_ob20_seg1[3]
 alpha_ob21_seg1[0] alpha_ob21_seg1[1] alpha_ob21_seg1[2]
 alpha_ob21_seg1[3] alpha_ob22_seg1[0] alpha_ob22_seg1[1]
 alpha_ob22_seg1[2] alpha_ob22_seg1[3] alpha_ob23_seg1[0]
 alpha_ob23_seg1[1] alpha_ob23_seg1[2] alpha_ob23_seg1[3]
 alpha_ob24_seg1[0] alpha_ob24_seg1[1] alpha_ob24_seg1[2]
 alpha_ob24_seg1[3] alpha_ob25_seg1[0] alpha_ob25_seg1[1]
 alpha_ob25_seg1[2] alpha_ob25_seg1[3] alpha_ob26_seg1[0]
 alpha_ob26_seg1[1] alpha_ob26_seg1[2] alpha_ob26_seg1[3]
 alpha_ob27_seg1[0] alpha_ob27_seg1[1] alpha_ob27_seg1[2]
 alpha_ob27_seg1[3] alpha_ob28_seg1[0] alpha_ob28_seg1[1]
 alpha_ob28_seg1[2] alpha_ob28_seg1[3] alpha_ob29_seg1[0]
 alpha_ob29_seg1[1] alpha_ob29_seg1[2] alpha_ob29_seg1[3]
 alpha_ob30_seg1[0] alpha_ob30_seg1[1] alpha_ob30_seg1[2]
 alpha_ob30_seg1[3] alpha_ob31_seg1[0] alpha_ob31_seg1[1]
 alpha_ob31_seg1[2] alpha_ob31_seg1[3] alpha_ob32_seg1[0]
 alpha_ob32_seg1[1] alpha_ob32_seg1[2] alpha_ob32_seg1[3]
 alpha_ob33_seg1[0] alpha_ob33_seg1[1] alpha_ob33_seg1[2]
 alpha_ob33_seg1[3] alpha_ob34_seg1[0] alpha_ob34_seg1[1]
 alpha_ob34_seg1[2] alpha_ob34_seg1[3] alpha_ob35_seg1[0]
 alpha_ob35_seg1[1] alpha_ob35_seg1[2] alpha_ob35_seg1[3]
 alpha_ob36_seg1[0] alpha_ob36_seg1[1] alpha_ob36_seg1[2]
 alpha_ob36_seg1[3] alpha_ob37_seg1[0] alpha_ob37_seg1[1]
 alpha_ob37_seg1[2] alpha_ob37_seg1[3] alpha_ob38_seg1[0]
 alpha_ob38_seg1[1] alpha_ob38_seg1[2] alpha_ob38_seg1[3]
 alpha_ob39_seg1[0] alpha_ob39_seg1[1] alpha_ob39_seg1[2]
 alpha_ob39_seg1[3] alpha_ob40_seg1[0] alpha_ob40_seg1[1]
 alpha_ob40_seg1[2] alpha_ob40_seg1[3] alpha_ob41_seg1[0]
 alpha_ob41_seg1[1] alpha_ob41_seg1[2] alpha_ob41_seg1[3]
 alpha_ob42_seg1[0] alpha_ob42_seg1[1] alpha_ob42_seg1[2]
 alpha_ob42_seg1[3] alpha_ob43_seg1[0] alpha_ob43_seg1[1]
 alpha_ob43_seg1[2] alpha_ob43_seg1[3] alpha_ob44_seg1[0]
 alpha_ob44_seg1[1] alpha_ob44_seg1[2] alpha_ob44_seg1[3]
 alpha_ob45_seg1[0] alpha_ob45_seg1[1] alpha_ob45_seg1[2]
 alpha_ob45_seg1[3] alpha_ob46_seg1[0] alpha_ob46_seg1[1]
 alpha_ob46_seg1[2] alpha_ob46_seg1[3] alpha_ob47_seg1[0]
 alpha_ob47_seg1[1] alpha_ob47_seg1[2] alpha_ob47_seg1[3]
 alpha_ob48_seg1[0] alpha_ob48_seg1[1] alpha_ob48_seg1[2]
 alpha_ob48_seg1[3] alpha_ob49_seg1[0] alpha_ob49_seg1[1]
 alpha_ob49_seg1[2] alpha_ob49_seg1[3] alpha_ob50_seg1[0]
 alpha_ob50_seg1[1] alpha_ob50_seg1[2] alpha_ob50_seg1[3] alpha_ob0_seg2[0]
 alpha_ob0_seg2[1] alpha_ob0_seg2[2] alpha_ob0_seg2[3] alpha_ob1_seg2[0]
 alpha_ob1_seg2[1] alpha_ob1_seg2[2] alpha_ob1_seg2[3] alpha_ob2_seg2[0]
 alpha_ob2_seg2[1] alpha_ob2_seg2[2] alpha_ob2_seg2[3] alpha_ob3_seg2[0]
 alpha_ob3_seg2[1] alpha_ob3_seg2[2] alpha_ob3_seg2[3] alpha_ob4_seg2[0]
 alpha_ob4_seg2[1] alpha_ob4_seg2[2] alpha_ob4_seg2[3] alpha_ob5_seg2[0]
 alpha_ob5_seg2[1] alpha_ob5_seg2[2] alpha_ob5_seg2[3] alpha_ob6_seg2[0]
 alpha_ob6_seg2[1] alpha_ob6_seg2[2] alpha_ob6_seg2[3] alpha_ob7_seg2[0]
 alpha_ob7_seg2[1] alpha_ob7_seg2[2] alpha_ob7_seg2[3] alpha_ob8_seg2[0]
 alpha_ob8_seg2[1] alpha_ob8_seg2[2] alpha_ob8_seg2[3] alpha_ob9_seg2[0]
 alpha_ob9_seg2[1] alpha_ob9_seg2[2] alpha_ob9_seg2[3] alpha_ob10_seg2[0]
 alpha_ob10_seg2[1] alpha_ob10_seg2[2] alpha_ob10_seg2[3]
 alpha_ob11_seg2[0] alpha_ob11_seg2[1] alpha_ob11_seg2[2]
 alpha_ob11_seg2[3] alpha_ob12_seg2[0] alpha_ob12_seg2[1]
 alpha_ob12_seg2[2] alpha_ob12_seg2[3] alpha_ob13_seg2[0]
 alpha_ob13_seg2[1] alpha_ob13_seg2[2] alpha_ob13_seg2[3]
 alpha_ob14_seg2[0] alpha_ob14_seg2[1] alpha_ob14_seg2[2]
 alpha_ob14_seg2[3] alpha_ob15_seg2[0] alpha_ob15_seg2[1]
 alpha_ob15_seg2[2] alpha_ob15_seg2[3] alpha_ob16_seg2[0]
 alpha_ob16_seg2[1] alpha_ob16_seg2[2] alpha_ob16_seg2[3]
 alpha_ob17_seg2[0] alpha_ob17_seg2[1] alpha_ob17_seg2[2]
 alpha_ob17_seg2[3] alpha_ob18_seg2[0] alpha_ob18_seg2[1]
 alpha_ob18_seg2[2] alpha_ob18_seg2[3] alpha_ob19_seg2[0]
 alpha_ob19_seg2[1] alpha_ob19_seg2[2] alpha_ob19_seg2[3]
 alpha_ob20_seg2[0] alpha_ob20_seg2[1] alpha_ob20_seg2[2]
 alpha_ob20_seg2[3] alpha_ob21_seg2[0] alpha_ob21_seg2[1]
 alpha_ob21_seg2[2] alpha_ob21_seg2[3] alpha_ob22_seg2[0]
 alpha_ob22_seg2[1] alpha_ob22_seg2[2] alpha_ob22_seg2[3]
 alpha_ob23_seg2[0] alpha_ob23_seg2[1] alpha_ob23_seg2[2]
 alpha_ob23_seg2[3] alpha_ob24_seg2[0] alpha_ob24_seg2[1]
 alpha_ob24_seg2[2] alpha_ob24_seg2[3] alpha_ob25_seg2[0]
 alpha_ob25_seg2[1] alpha_ob25_seg2[2] alpha_ob25_seg2[3]
 alpha_ob26_seg2[0] alpha_ob26_seg2[1] alpha_ob26_seg2[2]
 alpha_ob26_seg2[3] alpha_ob27_seg2[0] alpha_ob27_seg2[1]
 alpha_ob27_seg2[2] alpha_ob27_seg2[3] alpha_ob28_seg2[0]
 alpha_ob28_seg2[1] alpha_ob28_seg2[2] alpha_ob28_seg2[3]
 alpha_ob29_seg2[0] alpha_ob29_seg2[1] alpha_ob29_seg2[2]
 alpha_ob29_seg2[3] alpha_ob30_seg2[0] alpha_ob30_seg2[1]
 alpha_ob30_seg2[2] alpha_ob30_seg2[3] alpha_ob31_seg2[0]
 alpha_ob31_seg2[1] alpha_ob31_seg2[2] alpha_ob31_seg2[3]
 alpha_ob32_seg2[0] alpha_ob32_seg2[1] alpha_ob32_seg2[2]
 alpha_ob32_seg2[3] alpha_ob33_seg2[0] alpha_ob33_seg2[1]
 alpha_ob33_seg2[2] alpha_ob33_seg2[3] alpha_ob34_seg2[0]
 alpha_ob34_seg2[1] alpha_ob34_seg2[2] alpha_ob34_seg2[3]
 alpha_ob35_seg2[0] alpha_ob35_seg2[1] alpha_ob35_seg2[2]
 alpha_ob35_seg2[3] alpha_ob36_seg2[0] alpha_ob36_seg2[1]
 alpha_ob36_seg2[2] alpha_ob36_seg2[3] alpha_ob37_seg2[0]
 alpha_ob37_seg2[1] alpha_ob37_seg2[2] alpha_ob37_seg2[3]
 alpha_ob38_seg2[0] alpha_ob38_seg2[1] alpha_ob38_seg2[2]
 alpha_ob38_seg2[3] alpha_ob39_seg2[0] alpha_ob39_seg2[1]
 alpha_ob39_seg2[2] alpha_ob39_seg2[3] alpha_ob40_seg2[0]
 alpha_ob40_seg2[1] alpha_ob40_seg2[2] alpha_ob40_seg2[3]
 alpha_ob41_seg2[0] alpha_ob41_seg2[1] alpha_ob41_seg2[2]
 alpha_ob41_seg2[3] alpha_ob42_seg2[0] alpha_ob42_seg2[1]
 alpha_ob42_seg2[2] alpha_ob42_seg2[3] alpha_ob43_seg2[0]
 alpha_ob43_seg2[1] alpha_ob43_seg2[2] alpha_ob43_seg2[3]
 alpha_ob44_seg2[0] alpha_ob44_seg2[1] alpha_ob44_seg2[2]
 alpha_ob44_seg2[3] alpha_ob45_seg2[0] alpha_ob45_seg2[1]
 alpha_ob45_seg2[2] alpha_ob45_seg2[3] alpha_ob46_seg2[0]
 alpha_ob46_seg2[1] alpha_ob46_seg2[2] alpha_ob46_seg2[3]
 alpha_ob47_seg2[0] alpha_ob47_seg2[1] alpha_ob47_seg2[2]
 alpha_ob47_seg2[3] alpha_ob48_seg2[0] alpha_ob48_seg2[1]
 alpha_ob48_seg2[2] alpha_ob48_seg2[3] alpha_ob49_seg2[0]
 alpha_ob49_seg2[1] alpha_ob49_seg2[2] alpha_ob49_seg2[3]
 alpha_ob50_seg2[0] alpha_ob50_seg2[1] alpha_ob50_seg2[2]
 alpha_ob50_seg2[3]
End
