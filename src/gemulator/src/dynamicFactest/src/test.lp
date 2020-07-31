\ Model xref
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  - 60 C0 - 20 C1 - 60 C2 - 20 C3 - 60 C4 - 20 C5 - 60 C6 - 20 C7 - 60 C8
   - 20 C9 - 60 C10 - 20 C11 - 60 C12 - 20 C13 - 60 C14 - 20 C15 - 60 C16
   - 20 C17 - 60 C18 - 20 C19 - 60 C20 - 20 C21 - 60 C22 - 20 C23 - 60 C24
   - 20 C25 - 60 C26 - 20 C27 - 60 C28 - 20 C29 - 60 C30 - 20 C31 - 60 C32
   - 20 C33 - 60 C34 - 20 C35 - 60 C36 - 20 C37 - 60 C38 - 20 C39 - 60 C40
   - 20 C41 + 21000 Constant + [ 2 C0 ^2 + 2 C1 ^2 + 2 C2 ^2 + 2 C3 ^2
   + 2 C4 ^2 + 2 C5 ^2 + 2 C6 ^2 + 2 C7 ^2 + 2 C8 ^2 + 2 C9 ^2 + 2 C10 ^2
   + 2 C11 ^2 + 2 C12 ^2 + 2 C13 ^2 + 2 C14 ^2 + 2 C15 ^2 + 2 C16 ^2
   + 2 C17 ^2 + 2 C18 ^2 + 2 C19 ^2 + 2 C20 ^2 + 2 C21 ^2 + 2 C22 ^2
   + 2 C23 ^2 + 2 C24 ^2 + 2 C25 ^2 + 2 C26 ^2 + 2 C27 ^2 + 2 C28 ^2
   + 2 C29 ^2 + 2 C30 ^2 + 2 C31 ^2 + 2 C32 ^2 + 2 C33 ^2 + 2 C34 ^2
   + 2 C35 ^2 + 2 C36 ^2 + 2 C37 ^2 + 2 C38 ^2 + 2 C39 ^2 + 2 C40 ^2
   + 2 C41 ^2 ] / 2 
Subject To
 R0: C0 = 14
 R1: C1 = 11
 R2: C40 = 30
 R3: C41 = 10
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
 R1381: - C6 + 1e+06 alpha_ob0_seg3[0] <= 1.0000284170499525e+06
 R1382: - C8 + 1e+06 alpha_ob0_seg3[0] <= 1.0000284170499525e+06
 R1383: - C7 + 1e+06 alpha_ob0_seg3[1] <= 999957.0843002755
 R1384: - C9 + 1e+06 alpha_ob0_seg3[1] <= 999957.0843002755
 R1385: C6 + 1e+06 alpha_ob0_seg3[2] <= 999969.0328499526
 R1386: C8 + 1e+06 alpha_ob0_seg3[2] <= 999969.0328499526
 R1387: C7 + 1e+06 alpha_ob0_seg3[3] <= 1.0000132657002755e+06
 R1388: C9 + 1e+06 alpha_ob0_seg3[3] <= 1.0000132657002755e+06
 R1389: alpha_ob0_seg3[0] + alpha_ob0_seg3[1] + alpha_ob0_seg3[2]
   + alpha_ob0_seg3[3] >= 1
 R1390: - C6 + 1e+06 alpha_ob1_seg3[0] <= 1.000028417099801e+06
 R1391: - C8 + 1e+06 alpha_ob1_seg3[0] <= 1.000028417099801e+06
 R1392: - C7 + 1e+06 alpha_ob1_seg3[1] <= 999984.1843155898
 R1393: - C9 + 1e+06 alpha_ob1_seg3[1] <= 999984.1843155898
 R1394: C6 + 1e+06 alpha_ob1_seg3[2] <= 999957.4328998009
 R1395: C8 + 1e+06 alpha_ob1_seg3[2] <= 999957.4328998009
 R1396: C7 + 1e+06 alpha_ob1_seg3[3] <= 1.0000132657155899e+06
 R1397: C9 + 1e+06 alpha_ob1_seg3[3] <= 1.0000132657155899e+06
 R1398: alpha_ob1_seg3[0] + alpha_ob1_seg3[1] + alpha_ob1_seg3[2]
   + alpha_ob1_seg3[3] >= 1
 R1399: - C6 + 1e+06 alpha_ob2_seg3[0] <= 1.0000400171500475e+06
 R1400: - C8 + 1e+06 alpha_ob2_seg3[0] <= 1.0000400171500475e+06
 R1401: - C7 + 1e+06 alpha_ob2_seg3[1] <= 999957.0842997245
 R1402: - C9 + 1e+06 alpha_ob2_seg3[1] <= 999957.0842997245
 R1403: C6 + 1e+06 alpha_ob2_seg3[2] <= 999957.4329500474
 R1404: C8 + 1e+06 alpha_ob2_seg3[2] <= 999957.4329500474
 R1405: C7 + 1e+06 alpha_ob2_seg3[3] <= 1.0000132656997246e+06
 R1406: C9 + 1e+06 alpha_ob2_seg3[3] <= 1.0000132656997246e+06
 R1407: alpha_ob2_seg3[0] + alpha_ob2_seg3[1] + alpha_ob2_seg3[2]
   + alpha_ob2_seg3[3] >= 1
 R1408: - C6 + 1e+06 alpha_ob3_seg3[0] <= 1.0000109161752058e+06
 R1409: - C8 + 1e+06 alpha_ob3_seg3[0] <= 1.0000109161752058e+06
 R1410: - C7 + 1e+06 alpha_ob3_seg3[1] <= 999970.2681002755
 R1411: - C9 + 1e+06 alpha_ob3_seg3[1] <= 999970.2681002755
 R1412: C6 + 1e+06 alpha_ob3_seg3[2] <= 999986.5337752058
 R1413: C8 + 1e+06 alpha_ob3_seg3[2] <= 999986.5337752058
 R1414: C7 + 1e+06 alpha_ob3_seg3[3] <= 1.0000138319002755e+06
 R1415: C9 + 1e+06 alpha_ob3_seg3[3] <= 1.0000138319002755e+06
 R1416: alpha_ob3_seg3[0] + alpha_ob3_seg3[1] + alpha_ob3_seg3[2]
   + alpha_ob3_seg3[3] >= 1
 R1417: - C6 + 1e+06 alpha_ob4_seg3[0] <= 1.000010916199801e+06
 R1418: - C8 + 1e+06 alpha_ob4_seg3[0] <= 1.000010916199801e+06
 R1419: - C7 + 1e+06 alpha_ob4_seg3[1] <= 999983.6181179117
 R1420: - C9 + 1e+06 alpha_ob4_seg3[1] <= 999983.6181179117
 R1421: C6 + 1e+06 alpha_ob4_seg3[2] <= 999973.183799801
 R1422: C8 + 1e+06 alpha_ob4_seg3[2] <= 999973.183799801
 R1423: C7 + 1e+06 alpha_ob4_seg3[3] <= 1.0000138319179118e+06
 R1424: C9 + 1e+06 alpha_ob4_seg3[3] <= 1.0000138319179118e+06
 R1425: alpha_ob4_seg3[0] + alpha_ob4_seg3[1] + alpha_ob4_seg3[2]
   + alpha_ob4_seg3[3] >= 1
 R1426: - C6 + 1e+06 alpha_ob5_seg3[0] <= 999992.49102
 R1427: - C8 + 1e+06 alpha_ob5_seg3[0] <= 999992.49102
 R1428: - C7 + 1e+06 alpha_ob5_seg3[1] <= 999957.1016000001
 R1429: - C9 + 1e+06 alpha_ob5_seg3[1] <= 999957.1016000001
 R1430: C6 + 1e+06 alpha_ob5_seg3[2] <= 999986.35898
 R1431: C8 + 1e+06 alpha_ob5_seg3[2] <= 999986.35898
 R1432: C7 + 1e+06 alpha_ob5_seg3[3] <= 1.0000403484e+06
 R1433: C9 + 1e+06 alpha_ob5_seg3[3] <= 1.0000403484e+06
 R1434: alpha_ob5_seg3[0] + alpha_ob5_seg3[1] + alpha_ob5_seg3[2]
   + alpha_ob5_seg3[3] >= 1
 R1435: - C6 + 1e+06 alpha_ob6_seg3[0] <= 999981.3909999999
 R1436: - C8 + 1e+06 alpha_ob6_seg3[0] <= 999981.3909999999
 R1437: - C7 + 1e+06 alpha_ob6_seg3[1] <= 999957.1016000001
 R1438: - C9 + 1e+06 alpha_ob6_seg3[1] <= 999957.1016000001
 R1439: C6 + 1e+06 alpha_ob6_seg3[2] <= 1.000004959e+06
 R1440: C8 + 1e+06 alpha_ob6_seg3[2] <= 1.000004959e+06
 R1441: C7 + 1e+06 alpha_ob6_seg3[3] <= 1.0000403484e+06
 R1442: C9 + 1e+06 alpha_ob6_seg3[3] <= 1.0000403484e+06
 R1443: alpha_ob6_seg3[0] + alpha_ob6_seg3[1] + alpha_ob6_seg3[2]
   + alpha_ob6_seg3[3] >= 1
 R1444: - C6 + 1e+06 alpha_ob7_seg3[0] <= 999981.3909825523
 R1445: - C8 + 1e+06 alpha_ob7_seg3[0] <= 999981.3909825523
 R1446: - C7 + 1e+06 alpha_ob7_seg3[1] <= 999957.1016002755
 R1447: - C9 + 1e+06 alpha_ob7_seg3[1] <= 999957.1016002755
 R1448: C6 + 1e+06 alpha_ob7_seg3[2] <= 1.0000160589825523e+06
 R1449: C8 + 1e+06 alpha_ob7_seg3[2] <= 1.0000160589825523e+06
 R1450: C7 + 1e+06 alpha_ob7_seg3[3] <= 1.0000309984002755e+06
 R1451: C9 + 1e+06 alpha_ob7_seg3[3] <= 1.0000309984002755e+06
 R1452: alpha_ob7_seg3[0] + alpha_ob7_seg3[1] + alpha_ob7_seg3[2]
   + alpha_ob7_seg3[3] >= 1
 R1453: - C6 + 1e+06 alpha_ob8_seg3[0] <= 999981.3867954084
 R1454: - C8 + 1e+06 alpha_ob8_seg3[0] <= 999981.3867954084
 R1455: - C7 + 1e+06 alpha_ob8_seg3[1] <= 999968.8595002755
 R1456: - C9 + 1e+06 alpha_ob8_seg3[1] <= 999968.8595002755
 R1457: C6 + 1e+06 alpha_ob8_seg3[2] <= 1.0000160631954084e+06
 R1458: C8 + 1e+06 alpha_ob8_seg3[2] <= 1.0000160631954084e+06
 R1459: C7 + 1e+06 alpha_ob8_seg3[3] <= 1.0000262405002755e+06
 R1460: C9 + 1e+06 alpha_ob8_seg3[3] <= 1.0000262405002755e+06
 R1461: alpha_ob8_seg3[0] + alpha_ob8_seg3[1] + alpha_ob8_seg3[2]
   + alpha_ob8_seg3[3] >= 1
 R1462: - C6 + 1e+06 alpha_ob9_seg3[0] <= 999981.386799801
 R1463: - C8 + 1e+06 alpha_ob9_seg3[0] <= 999981.386799801
 R1464: - C7 + 1e+06 alpha_ob9_seg3[1] <= 999971.2095013268
 R1465: - C9 + 1e+06 alpha_ob9_seg3[1] <= 999971.2095013268
 R1466: C6 + 1e+06 alpha_ob9_seg3[2] <= 1.0000152131998009e+06
 R1467: C8 + 1e+06 alpha_ob9_seg3[2] <= 1.0000152131998009e+06
 R1468: C7 + 1e+06 alpha_ob9_seg3[3] <= 1.0000262405013267e+06
 R1469: C9 + 1e+06 alpha_ob9_seg3[3] <= 1.0000262405013267e+06
 R1470: alpha_ob9_seg3[0] + alpha_ob9_seg3[1] + alpha_ob9_seg3[2]
   + alpha_ob9_seg3[3] >= 1
 R1471: - C6 + 1e+06 alpha_ob10_seg3[0] <= 999982.2368022958
 R1472: - C8 + 1e+06 alpha_ob10_seg3[0] <= 999982.2368022958
 R1473: - C7 + 1e+06 alpha_ob10_seg3[1] <= 999970.1094997245
 R1474: - C9 + 1e+06 alpha_ob10_seg3[1] <= 999970.1094997245
 R1475: C6 + 1e+06 alpha_ob10_seg3[2] <= 1.0000152132022957e+06
 R1476: C8 + 1e+06 alpha_ob10_seg3[2] <= 1.0000152132022957e+06
 R1477: C7 + 1e+06 alpha_ob10_seg3[3] <= 1.0000262404997245e+06
 R1478: C9 + 1e+06 alpha_ob10_seg3[3] <= 1.0000262404997245e+06
 R1479: alpha_ob10_seg3[0] + alpha_ob10_seg3[1] + alpha_ob10_seg3[2]
   + alpha_ob10_seg3[3] >= 1
 R1480: - C6 + 1e+06 alpha_ob11_seg3[0] <= 999982.236799801
 R1481: - C8 + 1e+06 alpha_ob11_seg3[0] <= 999982.236799801
 R1482: - C7 + 1e+06 alpha_ob11_seg3[1] <= 999970.1095291895
 R1483: - C9 + 1e+06 alpha_ob11_seg3[1] <= 999970.1095291895
 R1484: C6 + 1e+06 alpha_ob11_seg3[2] <= 999993.3631998011
 R1485: C8 + 1e+06 alpha_ob11_seg3[2] <= 999993.3631998011
 R1486: C7 + 1e+06 alpha_ob11_seg3[3] <= 1.0000273405291894e+06
 R1487: C9 + 1e+06 alpha_ob11_seg3[3] <= 1.0000273405291894e+06
 R1488: alpha_ob11_seg3[0] + alpha_ob11_seg3[1] + alpha_ob11_seg3[2]
   + alpha_ob11_seg3[3] >= 1
 R1489: - C6 + 1e+06 alpha_ob12_seg3[0] <= 999981.6050802565
 R1490: - C8 + 1e+06 alpha_ob12_seg3[0] <= 999981.6050802565
 R1491: - C7 + 1e+06 alpha_ob12_seg3[1] <= 999972.8942002755
 R1492: - C9 + 1e+06 alpha_ob12_seg3[1] <= 999972.8942002755
 R1493: C6 + 1e+06 alpha_ob12_seg3[2] <= 1.0000158448802566e+06
 R1494: C8 + 1e+06 alpha_ob12_seg3[2] <= 1.0000158448802566e+06
 R1495: C7 + 1e+06 alpha_ob12_seg3[3] <= 1.0000139558002756e+06
 R1496: C9 + 1e+06 alpha_ob12_seg3[3] <= 1.0000139558002756e+06
 R1497: alpha_ob12_seg3[0] + alpha_ob12_seg3[1] + alpha_ob12_seg3[2]
   + alpha_ob12_seg3[3] >= 1
 R1498: - C6 + 1e+06 alpha_ob13_seg3[0] <= 999981.605149801
 R1499: - C8 + 1e+06 alpha_ob13_seg3[0] <= 999981.605149801
 R1500: - C7 + 1e+06 alpha_ob13_seg3[1] <= 999983.4942391404
 R1501: - C9 + 1e+06 alpha_ob13_seg3[1] <= 999983.4942391404
 R1502: C6 + 1e+06 alpha_ob13_seg3[2] <= 999986.4948498011
 R1503: C8 + 1e+06 alpha_ob13_seg3[2] <= 999986.4948498011
 R1504: C7 + 1e+06 alpha_ob13_seg3[3] <= 1.0000139558391405e+06
 R1505: C9 + 1e+06 alpha_ob13_seg3[3] <= 1.0000139558391405e+06
 R1506: alpha_ob13_seg3[0] + alpha_ob13_seg3[1] + alpha_ob13_seg3[2]
   + alpha_ob13_seg3[3] >= 1
 R1507: - C6 + 1e+06 alpha_ob14_seg3[0] <= 999997.6803502565
 R1508: - C8 + 1e+06 alpha_ob14_seg3[0] <= 999997.6803502565
 R1509: - C7 + 1e+06 alpha_ob14_seg3[1] <= 999972.7268002755
 R1510: - C9 + 1e+06 alpha_ob14_seg3[1] <= 999972.7268002755
 R1511: C6 + 1e+06 alpha_ob14_seg3[2] <= 999999.7696102565
 R1512: C8 + 1e+06 alpha_ob14_seg3[2] <= 999999.7696102565
 R1513: C7 + 1e+06 alpha_ob14_seg3[3] <= 1.0000141232002755e+06
 R1514: C9 + 1e+06 alpha_ob14_seg3[3] <= 1.0000141232002755e+06
 R1515: alpha_ob14_seg3[0] + alpha_ob14_seg3[1] + alpha_ob14_seg3[2]
   + alpha_ob14_seg3[3] >= 1
 R1516: - C6 + 1e+06 alpha_ob15_seg3[0] <= 1.0000038341502565e+06
 R1517: - C8 + 1e+06 alpha_ob15_seg3[0] <= 1.0000038341502565e+06
 R1518: - C7 + 1e+06 alpha_ob15_seg3[1] <= 999972.6012002755
 R1519: - C9 + 1e+06 alpha_ob15_seg3[1] <= 999972.6012002755
 R1520: C6 + 1e+06 alpha_ob15_seg3[2] <= 999993.6158102566
 R1521: C8 + 1e+06 alpha_ob15_seg3[2] <= 999993.6158102566
 R1522: C7 + 1e+06 alpha_ob15_seg3[3] <= 1.0000142488002755e+06
 R1523: C9 + 1e+06 alpha_ob15_seg3[3] <= 1.0000142488002755e+06
 R1524: alpha_ob15_seg3[0] + alpha_ob15_seg3[1] + alpha_ob15_seg3[2]
   + alpha_ob15_seg3[3] >= 1
 R1525: - C6 + 1e+06 alpha_ob16_seg3[0] <= 1.0000162874063333e+06
 R1526: - C8 + 1e+06 alpha_ob16_seg3[0] <= 1.0000162874063333e+06
 R1527: - C7 + 1e+06 alpha_ob16_seg3[1] <= 999982.9439602756
 R1528: - C9 + 1e+06 alpha_ob16_seg3[1] <= 999982.9439602756
 R1529: C6 + 1e+06 alpha_ob16_seg3[2] <= 999981.1624063333
 R1530: C8 + 1e+06 alpha_ob16_seg3[2] <= 999981.1624063333
 R1531: C7 + 1e+06 alpha_ob16_seg3[3] <= 999963.6560402757
 R1532: C9 + 1e+06 alpha_ob16_seg3[3] <= 999963.6560402757
 R1533: alpha_ob16_seg3[0] + alpha_ob16_seg3[1] + alpha_ob16_seg3[2]
   + alpha_ob16_seg3[3] >= 1
 R1534: - C6 + 1e+06 alpha_ob17_seg3[0] <= 999980.937509
 R1535: - C8 + 1e+06 alpha_ob17_seg3[0] <= 999980.937509
 R1536: - C7 + 1e+06 alpha_ob17_seg3[1] <= 1.000034894e+06
 R1537: - C9 + 1e+06 alpha_ob17_seg3[1] <= 1.000034894e+06
 R1538: C6 + 1e+06 alpha_ob17_seg3[2] <= 999981.162491
 R1539: C8 + 1e+06 alpha_ob17_seg3[2] <= 999981.162491
 R1540: C7 + 1e+06 alpha_ob17_seg3[3] <= 999962.556
 R1541: C9 + 1e+06 alpha_ob17_seg3[3] <= 999962.556
 R1542: alpha_ob17_seg3[0] + alpha_ob17_seg3[1] + alpha_ob17_seg3[2]
   + alpha_ob17_seg3[3] >= 1
 R1543: - C6 + 1e+06 alpha_ob18_seg3[0] <= 999990.2670277582
 R1544: - C8 + 1e+06 alpha_ob18_seg3[0] <= 999990.2670277582
 R1545: - C7 + 1e+06 alpha_ob18_seg3[1] <= 1.0000347702002756e+06
 R1546: - C9 + 1e+06 alpha_ob18_seg3[1] <= 1.0000347702002756e+06
 R1547: C6 + 1e+06 alpha_ob18_seg3[2] <= 1.0000071828877581e+06
 R1548: C8 + 1e+06 alpha_ob18_seg3[2] <= 1.0000071828877581e+06
 R1549: C7 + 1e+06 alpha_ob18_seg3[3] <= 999939.8298002756
 R1550: C9 + 1e+06 alpha_ob18_seg3[3] <= 999939.8298002756
 R1551: alpha_ob18_seg3[0] + alpha_ob18_seg3[1] + alpha_ob18_seg3[2]
   + alpha_ob18_seg3[3] >= 1
 R1552: - C6 + 1e+06 alpha_ob19_seg3[0] <= 999990.267069801
 R1553: - C8 + 1e+06 alpha_ob19_seg3[0] <= 999990.267069801
 R1554: - C7 + 1e+06 alpha_ob19_seg3[1] <= 1.0000576202069657e+06
 R1555: - C9 + 1e+06 alpha_ob19_seg3[1] <= 1.0000576202069657e+06
 R1556: C6 + 1e+06 alpha_ob19_seg3[2] <= 1.000002082929801e+06
 R1557: C8 + 1e+06 alpha_ob19_seg3[2] <= 1.000002082929801e+06
 R1558: C7 + 1e+06 alpha_ob19_seg3[3] <= 999939.8298069657
 R1559: C9 + 1e+06 alpha_ob19_seg3[3] <= 999939.8298069657
 R1560: alpha_ob19_seg3[0] + alpha_ob19_seg3[1] + alpha_ob19_seg3[2]
   + alpha_ob19_seg3[3] >= 1
 R1561: - C6 + 1e+06 alpha_ob20_seg3[0] <= 999991.073732
 R1562: - C8 + 1e+06 alpha_ob20_seg3[0] <= 999991.073732
 R1563: - C7 + 1e+06 alpha_ob20_seg3[1] <= 999952.2125
 R1564: - C9 + 1e+06 alpha_ob20_seg3[1] <= 999952.2125
 R1565: C6 + 1e+06 alpha_ob20_seg3[2] <= 999990.776268
 R1566: C8 + 1e+06 alpha_ob20_seg3[2] <= 999990.776268
 R1567: C7 + 1e+06 alpha_ob20_seg3[3] <= 1.0000452375e+06
 R1568: C9 + 1e+06 alpha_ob20_seg3[3] <= 1.0000452375e+06
 R1569: alpha_ob20_seg3[0] + alpha_ob20_seg3[1] + alpha_ob20_seg3[2]
   + alpha_ob20_seg3[3] >= 1
 R1570: - C6 + 1e+06 alpha_ob21_seg3[0] <= 999985.657099801
 R1571: - C8 + 1e+06 alpha_ob21_seg3[0] <= 999985.657099801
 R1572: - C7 + 1e+06 alpha_ob21_seg3[1] <= 1.0000576202063023e+06
 R1573: - C9 + 1e+06 alpha_ob21_seg3[1] <= 1.0000576202063023e+06
 R1574: C6 + 1e+06 alpha_ob21_seg3[2] <= 1.000007192899801e+06
 R1575: C8 + 1e+06 alpha_ob21_seg3[2] <= 1.000007192899801e+06
 R1576: C7 + 1e+06 alpha_ob21_seg3[3] <= 999939.8298063023
 R1577: C9 + 1e+06 alpha_ob21_seg3[3] <= 999939.8298063023
 R1578: alpha_ob21_seg3[0] + alpha_ob21_seg3[1] + alpha_ob21_seg3[2]
   + alpha_ob21_seg3[3] >= 1
 R1579: - C6 + 1e+06 alpha_ob22_seg3[0] <= 999994.771829801
 R1580: - C8 + 1e+06 alpha_ob22_seg3[0] <= 999994.771829801
 R1581: - C7 + 1e+06 alpha_ob22_seg3[1] <= 1.0000615983063023e+06
 R1582: - C9 + 1e+06 alpha_ob22_seg3[1] <= 1.0000615983063023e+06
 R1583: C6 + 1e+06 alpha_ob22_seg3[2] <= 999998.078169801
 R1584: C8 + 1e+06 alpha_ob22_seg3[2] <= 999998.078169801
 R1585: C7 + 1e+06 alpha_ob22_seg3[3] <= 999935.8517063023
 R1586: C9 + 1e+06 alpha_ob22_seg3[3] <= 999935.8517063023
 R1587: alpha_ob22_seg3[0] + alpha_ob22_seg3[1] + alpha_ob22_seg3[2]
   + alpha_ob22_seg3[3] >= 1
 R1588: - C6 + 1e+06 alpha_ob23_seg3[0] <= 999980.9374508709
 R1589: - C8 + 1e+06 alpha_ob23_seg3[0] <= 999980.9374508709
 R1590: - C7 + 1e+06 alpha_ob23_seg3[1] <= 1.0000348940002755e+06
 R1591: - C9 + 1e+06 alpha_ob23_seg3[1] <= 1.0000348940002755e+06
 R1592: C6 + 1e+06 alpha_ob23_seg3[2] <= 1.0000165124508708e+06
 R1593: C8 + 1e+06 alpha_ob23_seg3[2] <= 1.0000165124508708e+06
 R1594: C7 + 1e+06 alpha_ob23_seg3[3] <= 999935.9560002756
 R1595: C9 + 1e+06 alpha_ob23_seg3[3] <= 999935.9560002756
 R1596: alpha_ob23_seg3[0] + alpha_ob23_seg3[1] + alpha_ob23_seg3[2]
   + alpha_ob23_seg3[3] >= 1
 R1597: - C6 + 1e+06 alpha_ob24_seg3[0] <= 999980.937499801
 R1598: - C8 + 1e+06 alpha_ob24_seg3[0] <= 999980.937499801
 R1599: - C7 + 1e+06 alpha_ob24_seg3[1] <= 1.0000614940072973e+06
 R1600: - C9 + 1e+06 alpha_ob24_seg3[1] <= 1.0000614940072973e+06
 R1601: C6 + 1e+06 alpha_ob24_seg3[2] <= 1.000011162499801e+06
 R1602: C8 + 1e+06 alpha_ob24_seg3[2] <= 1.000011162499801e+06
 R1603: C7 + 1e+06 alpha_ob24_seg3[3] <= 999935.9560072974
 R1604: C9 + 1e+06 alpha_ob24_seg3[3] <= 999935.9560072974
 R1605: alpha_ob24_seg3[0] + alpha_ob24_seg3[1] + alpha_ob24_seg3[2]
   + alpha_ob24_seg3[3] >= 1
 R1606: - C6 + 1e+06 alpha_ob25_seg3[0] <= 999956.8394000001
 R1607: - C8 + 1e+06 alpha_ob25_seg3[0] <= 999956.8394000001
 R1608: - C7 + 1e+06 alpha_ob25_seg3[1] <= 999952.2463
 R1609: - C9 + 1e+06 alpha_ob25_seg3[1] <= 999952.2463
 R1610: C6 + 1e+06 alpha_ob25_seg3[2] <= 1.0000097606e+06
 R1611: C8 + 1e+06 alpha_ob25_seg3[2] <= 1.0000097606e+06
 R1612: C7 + 1e+06 alpha_ob25_seg3[3] <= 1.0000452037e+06
 R1613: C9 + 1e+06 alpha_ob25_seg3[3] <= 1.0000452037e+06
 R1614: alpha_ob25_seg3[0] + alpha_ob25_seg3[1] + alpha_ob25_seg3[2]
   + alpha_ob25_seg3[3] >= 1
 R1615: - C6 + 1e+06 alpha_ob26_seg3[0] <= 999980.9375348955
 R1616: - C8 + 1e+06 alpha_ob26_seg3[0] <= 999980.9375348955
 R1617: - C7 + 1e+06 alpha_ob26_seg3[1] <= 1.0000160439997246e+06
 R1618: - C9 + 1e+06 alpha_ob26_seg3[1] <= 1.0000160439997246e+06
 R1619: C6 + 1e+06 alpha_ob26_seg3[2] <= 1.0000165125348954e+06
 R1620: C8 + 1e+06 alpha_ob26_seg3[2] <= 1.0000165125348954e+06
 R1621: C7 + 1e+06 alpha_ob26_seg3[3] <= 999962.5559997245
 R1622: C9 + 1e+06 alpha_ob26_seg3[3] <= 999962.5559997245
 R1623: alpha_ob26_seg3[0] + alpha_ob26_seg3[1] + alpha_ob26_seg3[2]
   + alpha_ob26_seg3[3] >= 1
 R1624: - C6 + 1e+06 alpha_ob27_seg3[0] <= 999956.839399801
 R1625: - C8 + 1e+06 alpha_ob27_seg3[0] <= 999956.839399801
 R1626: - C7 + 1e+06 alpha_ob27_seg3[1] <= 999925.3963434525
 R1627: - C9 + 1e+06 alpha_ob27_seg3[1] <= 999925.3963434525
 R1628: C6 + 1e+06 alpha_ob27_seg3[2] <= 1.0000080105998011e+06
 R1629: C8 + 1e+06 alpha_ob27_seg3[2] <= 1.0000080105998011e+06
 R1630: C7 + 1e+06 alpha_ob27_seg3[3] <= 1.0000720537434525e+06
 R1631: C9 + 1e+06 alpha_ob27_seg3[3] <= 1.0000720537434525e+06
 R1632: alpha_ob27_seg3[0] + alpha_ob27_seg3[1] + alpha_ob27_seg3[2]
   + alpha_ob27_seg3[3] >= 1
 R1633: - C6 + 1e+06 alpha_ob28_seg3[0] <= 999980.9374998009
 R1634: - C8 + 1e+06 alpha_ob28_seg3[0] <= 999980.9374998009
 R1635: - C7 + 1e+06 alpha_ob28_seg3[1] <= 1.0000160440053071e+06
 R1636: - C9 + 1e+06 alpha_ob28_seg3[1] <= 1.0000160440053071e+06
 R1637: C6 + 1e+06 alpha_ob28_seg3[2] <= 1.000012662499801e+06
 R1638: C8 + 1e+06 alpha_ob28_seg3[2] <= 1.000012662499801e+06
 R1639: C7 + 1e+06 alpha_ob28_seg3[3] <= 999981.4060053072
 R1640: C9 + 1e+06 alpha_ob28_seg3[3] <= 999981.4060053072
 R1641: alpha_ob28_seg3[0] + alpha_ob28_seg3[1] + alpha_ob28_seg3[2]
   + alpha_ob28_seg3[3] >= 1
 R1642: - C6 + 1e+06 alpha_ob29_seg3[0] <= 999983.6053459151
 R1643: - C8 + 1e+06 alpha_ob29_seg3[0] <= 999983.6053459151
 R1644: - C7 + 1e+06 alpha_ob29_seg3[1] <= 999987.7722497246
 R1645: - C9 + 1e+06 alpha_ob29_seg3[1] <= 999987.7722497246
 R1646: C6 + 1e+06 alpha_ob29_seg3[2] <= 1.000013844745915e+06
 R1647: C8 + 1e+06 alpha_ob29_seg3[2] <= 1.000013844745915e+06
 R1648: C7 + 1e+06 alpha_ob29_seg3[3] <= 999984.8277497246
 R1649: C9 + 1e+06 alpha_ob29_seg3[3] <= 999984.8277497246
 R1650: alpha_ob29_seg3[0] + alpha_ob29_seg3[1] + alpha_ob29_seg3[2]
   + alpha_ob29_seg3[3] >= 1
 R1651: - C6 + 1e+06 alpha_ob30_seg3[0] <= 999958.8717898987
 R1652: - C8 + 1e+06 alpha_ob30_seg3[0] <= 999958.8717898987
 R1653: - C7 + 1e+06 alpha_ob30_seg3[1] <= 999987.7837602755
 R1654: - C9 + 1e+06 alpha_ob30_seg3[1] <= 999987.7837602755
 R1655: C6 + 1e+06 alpha_ob30_seg3[2] <= 1.0000385781898986e+06
 R1656: C8 + 1e+06 alpha_ob30_seg3[2] <= 1.0000385781898986e+06
 R1657: C7 + 1e+06 alpha_ob30_seg3[3] <= 1.0000043162402755e+06
 R1658: C9 + 1e+06 alpha_ob30_seg3[3] <= 1.0000043162402755e+06
 R1659: alpha_ob30_seg3[0] + alpha_ob30_seg3[1] + alpha_ob30_seg3[2]
   + alpha_ob30_seg3[3] >= 1
 R1660: - C6 + 1e+06 alpha_ob31_seg3[0] <= 999939.2718
 R1661: - C8 + 1e+06 alpha_ob31_seg3[0] <= 999939.2718
 R1662: - C7 + 1e+06 alpha_ob31_seg3[1] <= 999993.13376
 R1663: - C9 + 1e+06 alpha_ob31_seg3[1] <= 999993.13376
 R1664: C6 + 1e+06 alpha_ob31_seg3[2] <= 1.0000385782e+06
 R1665: C8 + 1e+06 alpha_ob31_seg3[2] <= 1.0000385782e+06
 R1666: C7 + 1e+06 alpha_ob31_seg3[3] <= 1.00000431624e+06
 R1667: C9 + 1e+06 alpha_ob31_seg3[3] <= 1.00000431624e+06
 R1668: alpha_ob31_seg3[0] + alpha_ob31_seg3[1] + alpha_ob31_seg3[2]
   + alpha_ob31_seg3[3] >= 1
 R1669: - C6 + 1e+06 alpha_ob32_seg3[0] <= 999939.2718101013
 R1670: - C8 + 1e+06 alpha_ob32_seg3[0] <= 999939.2718101013
 R1671: - C7 + 1e+06 alpha_ob32_seg3[1] <= 999987.7837597246
 R1672: - C9 + 1e+06 alpha_ob32_seg3[1] <= 999987.7837597246
 R1673: C6 + 1e+06 alpha_ob32_seg3[2] <= 1.0000581782101013e+06
 R1674: C8 + 1e+06 alpha_ob32_seg3[2] <= 1.0000581782101013e+06
 R1675: C7 + 1e+06 alpha_ob32_seg3[3] <= 1.0000043162397245e+06
 R1676: C9 + 1e+06 alpha_ob32_seg3[3] <= 1.0000043162397245e+06
 R1677: alpha_ob32_seg3[0] + alpha_ob32_seg3[1] + alpha_ob32_seg3[2]
   + alpha_ob32_seg3[3] >= 1
 R1678: - C6 + 1e+06 alpha_ob33_seg3[0] <= 999939.271799801
 R1679: - C8 + 1e+06 alpha_ob33_seg3[0] <= 999939.271799801
 R1680: - C7 + 1e+06 alpha_ob33_seg3[1] <= 999987.7838262042
 R1681: - C9 + 1e+06 alpha_ob33_seg3[1] <= 999987.7838262042
 R1682: C6 + 1e+06 alpha_ob33_seg3[2] <= 1.000038578199801e+06
 R1683: C8 + 1e+06 alpha_ob33_seg3[2] <= 1.000038578199801e+06
 R1684: C7 + 1e+06 alpha_ob33_seg3[3] <= 1.0000096662262043e+06
 R1685: C9 + 1e+06 alpha_ob33_seg3[3] <= 1.0000096662262043e+06
 R1686: alpha_ob33_seg3[0] + alpha_ob33_seg3[1] + alpha_ob33_seg3[2]
   + alpha_ob33_seg3[3] >= 1
 R1687: - C6 + 1e+06 alpha_ob34_seg3[0] <= 999926.0708
 R1688: - C8 + 1e+06 alpha_ob34_seg3[0] <= 999926.0708
 R1689: - C7 + 1e+06 alpha_ob34_seg3[1] <= 999983.7196
 R1690: - C9 + 1e+06 alpha_ob34_seg3[1] <= 999983.7196
 R1691: C6 + 1e+06 alpha_ob34_seg3[2] <= 1.0000250292e+06
 R1692: C8 + 1e+06 alpha_ob34_seg3[2] <= 1.0000250292e+06
 R1693: C7 + 1e+06 alpha_ob34_seg3[3] <= 1.0000137304e+06
 R1694: C9 + 1e+06 alpha_ob34_seg3[3] <= 1.0000137304e+06
 R1695: alpha_ob34_seg3[0] + alpha_ob34_seg3[1] + alpha_ob34_seg3[2]
   + alpha_ob34_seg3[3] >= 1
 R1696: - C6 + 1e+06 alpha_ob35_seg3[0] <= 999926.0708293857
 R1697: - C8 + 1e+06 alpha_ob35_seg3[0] <= 999926.0708293857
 R1698: - C7 + 1e+06 alpha_ob35_seg3[1] <= 999967.8695997245
 R1699: - C9 + 1e+06 alpha_ob35_seg3[1] <= 999967.8695997245
 R1700: C6 + 1e+06 alpha_ob35_seg3[2] <= 1.0000713792293856e+06
 R1701: C8 + 1e+06 alpha_ob35_seg3[2] <= 1.0000713792293856e+06
 R1702: C7 + 1e+06 alpha_ob35_seg3[3] <= 1.0000137303997246e+06
 R1703: C9 + 1e+06 alpha_ob35_seg3[3] <= 1.0000137303997246e+06
 R1704: alpha_ob35_seg3[0] + alpha_ob35_seg3[1] + alpha_ob35_seg3[2]
   + alpha_ob35_seg3[3] >= 1
 R1705: - C6 + 1e+06 alpha_ob36_seg3[0] <= 999972.6917508709
 R1706: - C8 + 1e+06 alpha_ob36_seg3[0] <= 999972.6917508709
 R1707: - C7 + 1e+06 alpha_ob36_seg3[1] <= 999957.1644002756
 R1708: - C9 + 1e+06 alpha_ob36_seg3[1] <= 999957.1644002756
 R1709: C6 + 1e+06 alpha_ob36_seg3[2] <= 1.0000247581508709e+06
 R1710: C8 + 1e+06 alpha_ob36_seg3[2] <= 1.0000247581508709e+06
 R1711: C7 + 1e+06 alpha_ob36_seg3[3] <= 1.0000136856002755e+06
 R1712: C9 + 1e+06 alpha_ob36_seg3[3] <= 1.0000136856002755e+06
 R1713: alpha_ob36_seg3[0] + alpha_ob36_seg3[1] + alpha_ob36_seg3[2]
   + alpha_ob36_seg3[3] >= 1
 R1714: - C6 + 1e+06 alpha_ob37_seg3[0] <= 999939.0009674003
 R1715: - C8 + 1e+06 alpha_ob37_seg3[0] <= 999939.0009674003
 R1716: - C7 + 1e+06 alpha_ob37_seg3[1] <= 999961.9516002756
 R1717: - C9 + 1e+06 alpha_ob37_seg3[1] <= 999961.9516002756
 R1718: C6 + 1e+06 alpha_ob37_seg3[2] <= 1.0000584489674004e+06
 R1719: C8 + 1e+06 alpha_ob37_seg3[2] <= 1.0000584489674004e+06
 R1720: C7 + 1e+06 alpha_ob37_seg3[3] <= 1.0000178984002755e+06
 R1721: C9 + 1e+06 alpha_ob37_seg3[3] <= 1.0000178984002755e+06
 R1722: alpha_ob37_seg3[0] + alpha_ob37_seg3[1] + alpha_ob37_seg3[2]
   + alpha_ob37_seg3[3] >= 1
 R1723: - C6 + 1e+06 alpha_ob38_seg3[0] <= 999994.9164998011
 R1724: - C8 + 1e+06 alpha_ob38_seg3[0] <= 999994.9164998011
 R1725: - C7 + 1e+06 alpha_ob38_seg3[1] <= 999925.4284782809
 R1726: - C9 + 1e+06 alpha_ob38_seg3[1] <= 999925.4284782809
 R1727: C6 + 1e+06 alpha_ob38_seg3[2] <= 999943.6834998011
 R1728: C8 + 1e+06 alpha_ob38_seg3[2] <= 999943.6834998011
 R1729: C7 + 1e+06 alpha_ob38_seg3[3] <= 1.0000720216782809e+06
 R1730: C9 + 1e+06 alpha_ob38_seg3[3] <= 1.0000720216782809e+06
 R1731: alpha_ob38_seg3[0] + alpha_ob38_seg3[1] + alpha_ob38_seg3[2]
   + alpha_ob38_seg3[3] >= 1
 R1732: - C6 + 1e+06 alpha_ob39_seg3[0] <= 999914.52327842
 R1733: - C8 + 1e+06 alpha_ob39_seg3[0] <= 999914.52327842
 R1734: - C7 + 1e+06 alpha_ob39_seg3[1] <= 999993.6552362755
 R1735: - C9 + 1e+06 alpha_ob39_seg3[1] <= 999993.6552362755
 R1736: C6 + 1e+06 alpha_ob39_seg3[2] <= 1.0000829266784199e+06
 R1737: C8 + 1e+06 alpha_ob39_seg3[2] <= 1.0000829266784199e+06
 R1738: C7 + 1e+06 alpha_ob39_seg3[3] <= 999992.1947642756
 R1739: C9 + 1e+06 alpha_ob39_seg3[3] <= 999992.1947642756
 R1740: alpha_ob39_seg3[0] + alpha_ob39_seg3[1] + alpha_ob39_seg3[2]
   + alpha_ob39_seg3[3] >= 1
 R1741: - C6 + 1e+06 alpha_ob40_seg3[0] <= 999912.265199801
 R1742: - C8 + 1e+06 alpha_ob40_seg3[0] <= 999912.265199801
 R1743: - C7 + 1e+06 alpha_ob40_seg3[1] <= 999987.7842112777
 R1744: - C9 + 1e+06 alpha_ob40_seg3[1] <= 999987.7842112777
 R1745: C6 + 1e+06 alpha_ob40_seg3[2] <= 1.000076834799801e+06
 R1746: C8 + 1e+06 alpha_ob40_seg3[2] <= 1.000076834799801e+06
 R1747: C7 + 1e+06 alpha_ob40_seg3[3] <= 1.0000096658112778e+06
 R1748: C9 + 1e+06 alpha_ob40_seg3[3] <= 1.0000096658112778e+06
 R1749: alpha_ob40_seg3[0] + alpha_ob40_seg3[1] + alpha_ob40_seg3[2]
   + alpha_ob40_seg3[3] >= 1
 R1750: - C6 + 1e+06 alpha_ob41_seg3[0] <= 999920.6152119379
 R1751: - C8 + 1e+06 alpha_ob41_seg3[0] <= 999920.6152119379
 R1752: - C7 + 1e+06 alpha_ob41_seg3[1] <= 999981.4341997246
 R1753: - C9 + 1e+06 alpha_ob41_seg3[1] <= 999981.4341997246
 R1754: C6 + 1e+06 alpha_ob41_seg3[2] <= 1.0000768348119379e+06
 R1755: C8 + 1e+06 alpha_ob41_seg3[2] <= 1.0000768348119379e+06
 R1756: C7 + 1e+06 alpha_ob41_seg3[3] <= 1.0000096657997245e+06
 R1757: C9 + 1e+06 alpha_ob41_seg3[3] <= 1.0000096657997245e+06
 R1758: alpha_ob41_seg3[0] + alpha_ob41_seg3[1] + alpha_ob41_seg3[2]
   + alpha_ob41_seg3[3] >= 1
 R1759: - C6 + 1e+06 alpha_ob42_seg3[0] <= 999919.2652
 R1760: - C8 + 1e+06 alpha_ob42_seg3[0] <= 999919.2652
 R1761: - C7 + 1e+06 alpha_ob42_seg3[1] <= 999981.4342
 R1762: - C9 + 1e+06 alpha_ob42_seg3[1] <= 999981.4342
 R1763: C6 + 1e+06 alpha_ob42_seg3[2] <= 1.0000768348e+06
 R1764: C8 + 1e+06 alpha_ob42_seg3[2] <= 1.0000768348e+06
 R1765: C7 + 1e+06 alpha_ob42_seg3[3] <= 1.0000160158000001e+06
 R1766: C9 + 1e+06 alpha_ob42_seg3[3] <= 1.0000160158000001e+06
 R1767: alpha_ob42_seg3[0] + alpha_ob42_seg3[1] + alpha_ob42_seg3[2]
   + alpha_ob42_seg3[3] >= 1
 R1768: - C6 + 1e+06 alpha_ob43_seg3[0] <= 999919.2652454559
 R1769: - C8 + 1e+06 alpha_ob43_seg3[0] <= 999919.2652454559
 R1770: - C7 + 1e+06 alpha_ob43_seg3[1] <= 999956.8341997246
 R1771: - C9 + 1e+06 alpha_ob43_seg3[1] <= 999956.8341997246
 R1772: C6 + 1e+06 alpha_ob43_seg3[2] <= 1.0000781848454559e+06
 R1773: C8 + 1e+06 alpha_ob43_seg3[2] <= 1.0000781848454559e+06
 R1774: C7 + 1e+06 alpha_ob43_seg3[3] <= 1.0000160157997246e+06
 R1775: C9 + 1e+06 alpha_ob43_seg3[3] <= 1.0000160157997246e+06
 R1776: alpha_ob43_seg3[0] + alpha_ob43_seg3[1] + alpha_ob43_seg3[2]
   + alpha_ob43_seg3[3] >= 1
 R1777: - C6 + 1e+06 alpha_ob44_seg3[0] <= 999991.83683
 R1778: - C8 + 1e+06 alpha_ob44_seg3[0] <= 999991.83683
 R1779: - C7 + 1e+06 alpha_ob44_seg3[1] <= 999940.589
 R1780: - C9 + 1e+06 alpha_ob44_seg3[1] <= 999940.589
 R1781: C6 + 1e+06 alpha_ob44_seg3[2] <= 1.00000401317e+06
 R1782: C8 + 1e+06 alpha_ob44_seg3[2] <= 1.00000401317e+06
 R1783: C7 + 1e+06 alpha_ob44_seg3[3] <= 1.000056861e+06
 R1784: C9 + 1e+06 alpha_ob44_seg3[3] <= 1.000056861e+06
 R1785: alpha_ob44_seg3[0] + alpha_ob44_seg3[1] + alpha_ob44_seg3[2]
   + alpha_ob44_seg3[3] >= 1
 R1786: - C6 + 1e+06 alpha_ob45_seg3[0] <= 999991.8368084199
 R1787: - C8 + 1e+06 alpha_ob45_seg3[0] <= 999991.8368084199
 R1788: - C7 + 1e+06 alpha_ob45_seg3[1] <= 999940.5890002755
 R1789: - C9 + 1e+06 alpha_ob45_seg3[1] <= 999940.5890002755
 R1790: C6 + 1e+06 alpha_ob45_seg3[2] <= 1.0000056131484199e+06
 R1791: C8 + 1e+06 alpha_ob45_seg3[2] <= 1.0000056131484199e+06
 R1792: C7 + 1e+06 alpha_ob45_seg3[3] <= 1.0000452610002755e+06
 R1793: C9 + 1e+06 alpha_ob45_seg3[3] <= 1.0000452610002755e+06
 R1794: alpha_ob45_seg3[0] + alpha_ob45_seg3[1] + alpha_ob45_seg3[2]
   + alpha_ob45_seg3[3] >= 1
 R1795: - C6 + 1e+06 alpha_ob46_seg3[0] <= 1.0000764021508709e+06
 R1796: - C8 + 1e+06 alpha_ob46_seg3[0] <= 1.0000764021508709e+06
 R1797: - C7 + 1e+06 alpha_ob46_seg3[1] <= 999957.1027002756
 R1798: - C9 + 1e+06 alpha_ob46_seg3[1] <= 999957.1027002756
 R1799: C6 + 1e+06 alpha_ob46_seg3[2] <= 999921.0477508709
 R1800: C8 + 1e+06 alpha_ob46_seg3[2] <= 999921.0477508709
 R1801: C7 + 1e+06 alpha_ob46_seg3[3] <= 1.0000137473002756e+06
 R1802: C9 + 1e+06 alpha_ob46_seg3[3] <= 1.0000137473002756e+06
 R1803: alpha_ob46_seg3[0] + alpha_ob46_seg3[1] + alpha_ob46_seg3[2]
   + alpha_ob46_seg3[3] >= 1
 R1804: - C6 + 1e+06 alpha_ob47_seg3[0] <= 1.0000445522e+06
 R1805: - C8 + 1e+06 alpha_ob47_seg3[0] <= 1.0000445522e+06
 R1806: - C7 + 1e+06 alpha_ob47_seg3[1] <= 999983.7027
 R1807: - C9 + 1e+06 alpha_ob47_seg3[1] <= 999983.7027
 R1808: C6 + 1e+06 alpha_ob47_seg3[2] <= 999921.0477999999
 R1809: C8 + 1e+06 alpha_ob47_seg3[2] <= 999921.0477999999
 R1810: C7 + 1e+06 alpha_ob47_seg3[3] <= 1.0000137473e+06
 R1811: C9 + 1e+06 alpha_ob47_seg3[3] <= 1.0000137473e+06
 R1812: alpha_ob47_seg3[0] + alpha_ob47_seg3[1] + alpha_ob47_seg3[2]
   + alpha_ob47_seg3[3] >= 1
 R1813: - C6 + 1e+06 alpha_ob48_seg3[0] <= 1.0000445522183661e+06
 R1814: - C8 + 1e+06 alpha_ob48_seg3[0] <= 1.0000445522183661e+06
 R1815: - C7 + 1e+06 alpha_ob48_seg3[1] <= 999973.8526997245
 R1816: - C9 + 1e+06 alpha_ob48_seg3[1] <= 999973.8526997245
 R1817: C6 + 1e+06 alpha_ob48_seg3[2] <= 999952.8978183661
 R1818: C8 + 1e+06 alpha_ob48_seg3[2] <= 999952.8978183661
 R1819: C7 + 1e+06 alpha_ob48_seg3[3] <= 1.0000137472997245e+06
 R1820: C9 + 1e+06 alpha_ob48_seg3[3] <= 1.0000137472997245e+06
 R1821: alpha_ob48_seg3[0] + alpha_ob48_seg3[1] + alpha_ob48_seg3[2]
   + alpha_ob48_seg3[3] >= 1
 R1822: - C6 + 1e+06 alpha_ob49_seg3[0] <= 1.000064332081634e+06
 R1823: - C8 + 1e+06 alpha_ob49_seg3[0] <= 1.000064332081634e+06
 R1824: - C7 + 1e+06 alpha_ob49_seg3[1] <= 999973.9401002756
 R1825: - C9 + 1e+06 alpha_ob49_seg3[1] <= 999973.9401002756
 R1826: C6 + 1e+06 alpha_ob49_seg3[2] <= 999933.1178816339
 R1827: C8 + 1e+06 alpha_ob49_seg3[2] <= 999933.1178816339
 R1828: C7 + 1e+06 alpha_ob49_seg3[3] <= 1.0000136599002755e+06
 R1829: C9 + 1e+06 alpha_ob49_seg3[3] <= 1.0000136599002755e+06
 R1830: alpha_ob49_seg3[0] + alpha_ob49_seg3[1] + alpha_ob49_seg3[2]
   + alpha_ob49_seg3[3] >= 1
 R1831: - C6 + 1e+06 alpha_ob50_seg3[0] <= 1.0000643321013774e+06
 R1832: - C8 + 1e+06 alpha_ob50_seg3[0] <= 1.0000643321013774e+06
 R1833: - C7 + 1e+06 alpha_ob50_seg3[1] <= 999983.1900997246
 R1834: - C9 + 1e+06 alpha_ob50_seg3[1] <= 999983.1900997246
 R1835: C6 + 1e+06 alpha_ob50_seg3[2] <= 999933.1179013775
 R1836: C8 + 1e+06 alpha_ob50_seg3[2] <= 999933.1179013775
 R1837: C7 + 1e+06 alpha_ob50_seg3[3] <= 1.0000136598997245e+06
 R1838: C9 + 1e+06 alpha_ob50_seg3[3] <= 1.0000136598997245e+06
 R1839: alpha_ob50_seg3[0] + alpha_ob50_seg3[1] + alpha_ob50_seg3[2]
   + alpha_ob50_seg3[3] >= 1
 R1840: - C8 + 1e+06 alpha_ob0_seg4[0] <= 1.0000284170499525e+06
 R1841: - C10 + 1e+06 alpha_ob0_seg4[0] <= 1.0000284170499525e+06
 R1842: - C9 + 1e+06 alpha_ob0_seg4[1] <= 999957.0843002755
 R1843: - C11 + 1e+06 alpha_ob0_seg4[1] <= 999957.0843002755
 R1844: C8 + 1e+06 alpha_ob0_seg4[2] <= 999969.0328499526
 R1845: C10 + 1e+06 alpha_ob0_seg4[2] <= 999969.0328499526
 R1846: C9 + 1e+06 alpha_ob0_seg4[3] <= 1.0000132657002755e+06
 R1847: C11 + 1e+06 alpha_ob0_seg4[3] <= 1.0000132657002755e+06
 R1848: alpha_ob0_seg4[0] + alpha_ob0_seg4[1] + alpha_ob0_seg4[2]
   + alpha_ob0_seg4[3] >= 1
 R1849: - C8 + 1e+06 alpha_ob1_seg4[0] <= 1.000028417099801e+06
 R1850: - C10 + 1e+06 alpha_ob1_seg4[0] <= 1.000028417099801e+06
 R1851: - C9 + 1e+06 alpha_ob1_seg4[1] <= 999984.1843155898
 R1852: - C11 + 1e+06 alpha_ob1_seg4[1] <= 999984.1843155898
 R1853: C8 + 1e+06 alpha_ob1_seg4[2] <= 999957.4328998009
 R1854: C10 + 1e+06 alpha_ob1_seg4[2] <= 999957.4328998009
 R1855: C9 + 1e+06 alpha_ob1_seg4[3] <= 1.0000132657155899e+06
 R1856: C11 + 1e+06 alpha_ob1_seg4[3] <= 1.0000132657155899e+06
 R1857: alpha_ob1_seg4[0] + alpha_ob1_seg4[1] + alpha_ob1_seg4[2]
   + alpha_ob1_seg4[3] >= 1
 R1858: - C8 + 1e+06 alpha_ob2_seg4[0] <= 1.0000400171500475e+06
 R1859: - C10 + 1e+06 alpha_ob2_seg4[0] <= 1.0000400171500475e+06
 R1860: - C9 + 1e+06 alpha_ob2_seg4[1] <= 999957.0842997245
 R1861: - C11 + 1e+06 alpha_ob2_seg4[1] <= 999957.0842997245
 R1862: C8 + 1e+06 alpha_ob2_seg4[2] <= 999957.4329500474
 R1863: C10 + 1e+06 alpha_ob2_seg4[2] <= 999957.4329500474
 R1864: C9 + 1e+06 alpha_ob2_seg4[3] <= 1.0000132656997246e+06
 R1865: C11 + 1e+06 alpha_ob2_seg4[3] <= 1.0000132656997246e+06
 R1866: alpha_ob2_seg4[0] + alpha_ob2_seg4[1] + alpha_ob2_seg4[2]
   + alpha_ob2_seg4[3] >= 1
 R1867: - C8 + 1e+06 alpha_ob3_seg4[0] <= 1.0000109161752058e+06
 R1868: - C10 + 1e+06 alpha_ob3_seg4[0] <= 1.0000109161752058e+06
 R1869: - C9 + 1e+06 alpha_ob3_seg4[1] <= 999970.2681002755
 R1870: - C11 + 1e+06 alpha_ob3_seg4[1] <= 999970.2681002755
 R1871: C8 + 1e+06 alpha_ob3_seg4[2] <= 999986.5337752058
 R1872: C10 + 1e+06 alpha_ob3_seg4[2] <= 999986.5337752058
 R1873: C9 + 1e+06 alpha_ob3_seg4[3] <= 1.0000138319002755e+06
 R1874: C11 + 1e+06 alpha_ob3_seg4[3] <= 1.0000138319002755e+06
 R1875: alpha_ob3_seg4[0] + alpha_ob3_seg4[1] + alpha_ob3_seg4[2]
   + alpha_ob3_seg4[3] >= 1
 R1876: - C8 + 1e+06 alpha_ob4_seg4[0] <= 1.000010916199801e+06
 R1877: - C10 + 1e+06 alpha_ob4_seg4[0] <= 1.000010916199801e+06
 R1878: - C9 + 1e+06 alpha_ob4_seg4[1] <= 999983.6181179117
 R1879: - C11 + 1e+06 alpha_ob4_seg4[1] <= 999983.6181179117
 R1880: C8 + 1e+06 alpha_ob4_seg4[2] <= 999973.183799801
 R1881: C10 + 1e+06 alpha_ob4_seg4[2] <= 999973.183799801
 R1882: C9 + 1e+06 alpha_ob4_seg4[3] <= 1.0000138319179118e+06
 R1883: C11 + 1e+06 alpha_ob4_seg4[3] <= 1.0000138319179118e+06
 R1884: alpha_ob4_seg4[0] + alpha_ob4_seg4[1] + alpha_ob4_seg4[2]
   + alpha_ob4_seg4[3] >= 1
 R1885: - C8 + 1e+06 alpha_ob5_seg4[0] <= 999992.49102
 R1886: - C10 + 1e+06 alpha_ob5_seg4[0] <= 999992.49102
 R1887: - C9 + 1e+06 alpha_ob5_seg4[1] <= 999957.1016000001
 R1888: - C11 + 1e+06 alpha_ob5_seg4[1] <= 999957.1016000001
 R1889: C8 + 1e+06 alpha_ob5_seg4[2] <= 999986.35898
 R1890: C10 + 1e+06 alpha_ob5_seg4[2] <= 999986.35898
 R1891: C9 + 1e+06 alpha_ob5_seg4[3] <= 1.0000403484e+06
 R1892: C11 + 1e+06 alpha_ob5_seg4[3] <= 1.0000403484e+06
 R1893: alpha_ob5_seg4[0] + alpha_ob5_seg4[1] + alpha_ob5_seg4[2]
   + alpha_ob5_seg4[3] >= 1
 R1894: - C8 + 1e+06 alpha_ob6_seg4[0] <= 999981.3909999999
 R1895: - C10 + 1e+06 alpha_ob6_seg4[0] <= 999981.3909999999
 R1896: - C9 + 1e+06 alpha_ob6_seg4[1] <= 999957.1016000001
 R1897: - C11 + 1e+06 alpha_ob6_seg4[1] <= 999957.1016000001
 R1898: C8 + 1e+06 alpha_ob6_seg4[2] <= 1.000004959e+06
 R1899: C10 + 1e+06 alpha_ob6_seg4[2] <= 1.000004959e+06
 R1900: C9 + 1e+06 alpha_ob6_seg4[3] <= 1.0000403484e+06
 R1901: C11 + 1e+06 alpha_ob6_seg4[3] <= 1.0000403484e+06
 R1902: alpha_ob6_seg4[0] + alpha_ob6_seg4[1] + alpha_ob6_seg4[2]
   + alpha_ob6_seg4[3] >= 1
 R1903: - C8 + 1e+06 alpha_ob7_seg4[0] <= 999981.3909825523
 R1904: - C10 + 1e+06 alpha_ob7_seg4[0] <= 999981.3909825523
 R1905: - C9 + 1e+06 alpha_ob7_seg4[1] <= 999957.1016002755
 R1906: - C11 + 1e+06 alpha_ob7_seg4[1] <= 999957.1016002755
 R1907: C8 + 1e+06 alpha_ob7_seg4[2] <= 1.0000160589825523e+06
 R1908: C10 + 1e+06 alpha_ob7_seg4[2] <= 1.0000160589825523e+06
 R1909: C9 + 1e+06 alpha_ob7_seg4[3] <= 1.0000309984002755e+06
 R1910: C11 + 1e+06 alpha_ob7_seg4[3] <= 1.0000309984002755e+06
 R1911: alpha_ob7_seg4[0] + alpha_ob7_seg4[1] + alpha_ob7_seg4[2]
   + alpha_ob7_seg4[3] >= 1
 R1912: - C8 + 1e+06 alpha_ob8_seg4[0] <= 999981.3867954084
 R1913: - C10 + 1e+06 alpha_ob8_seg4[0] <= 999981.3867954084
 R1914: - C9 + 1e+06 alpha_ob8_seg4[1] <= 999968.8595002755
 R1915: - C11 + 1e+06 alpha_ob8_seg4[1] <= 999968.8595002755
 R1916: C8 + 1e+06 alpha_ob8_seg4[2] <= 1.0000160631954084e+06
 R1917: C10 + 1e+06 alpha_ob8_seg4[2] <= 1.0000160631954084e+06
 R1918: C9 + 1e+06 alpha_ob8_seg4[3] <= 1.0000262405002755e+06
 R1919: C11 + 1e+06 alpha_ob8_seg4[3] <= 1.0000262405002755e+06
 R1920: alpha_ob8_seg4[0] + alpha_ob8_seg4[1] + alpha_ob8_seg4[2]
   + alpha_ob8_seg4[3] >= 1
 R1921: - C8 + 1e+06 alpha_ob9_seg4[0] <= 999981.386799801
 R1922: - C10 + 1e+06 alpha_ob9_seg4[0] <= 999981.386799801
 R1923: - C9 + 1e+06 alpha_ob9_seg4[1] <= 999971.2095013268
 R1924: - C11 + 1e+06 alpha_ob9_seg4[1] <= 999971.2095013268
 R1925: C8 + 1e+06 alpha_ob9_seg4[2] <= 1.0000152131998009e+06
 R1926: C10 + 1e+06 alpha_ob9_seg4[2] <= 1.0000152131998009e+06
 R1927: C9 + 1e+06 alpha_ob9_seg4[3] <= 1.0000262405013267e+06
 R1928: C11 + 1e+06 alpha_ob9_seg4[3] <= 1.0000262405013267e+06
 R1929: alpha_ob9_seg4[0] + alpha_ob9_seg4[1] + alpha_ob9_seg4[2]
   + alpha_ob9_seg4[3] >= 1
 R1930: - C8 + 1e+06 alpha_ob10_seg4[0] <= 999982.2368022958
 R1931: - C10 + 1e+06 alpha_ob10_seg4[0] <= 999982.2368022958
 R1932: - C9 + 1e+06 alpha_ob10_seg4[1] <= 999970.1094997245
 R1933: - C11 + 1e+06 alpha_ob10_seg4[1] <= 999970.1094997245
 R1934: C8 + 1e+06 alpha_ob10_seg4[2] <= 1.0000152132022957e+06
 R1935: C10 + 1e+06 alpha_ob10_seg4[2] <= 1.0000152132022957e+06
 R1936: C9 + 1e+06 alpha_ob10_seg4[3] <= 1.0000262404997245e+06
 R1937: C11 + 1e+06 alpha_ob10_seg4[3] <= 1.0000262404997245e+06
 R1938: alpha_ob10_seg4[0] + alpha_ob10_seg4[1] + alpha_ob10_seg4[2]
   + alpha_ob10_seg4[3] >= 1
 R1939: - C8 + 1e+06 alpha_ob11_seg4[0] <= 999982.236799801
 R1940: - C10 + 1e+06 alpha_ob11_seg4[0] <= 999982.236799801
 R1941: - C9 + 1e+06 alpha_ob11_seg4[1] <= 999970.1095291895
 R1942: - C11 + 1e+06 alpha_ob11_seg4[1] <= 999970.1095291895
 R1943: C8 + 1e+06 alpha_ob11_seg4[2] <= 999993.3631998011
 R1944: C10 + 1e+06 alpha_ob11_seg4[2] <= 999993.3631998011
 R1945: C9 + 1e+06 alpha_ob11_seg4[3] <= 1.0000273405291894e+06
 R1946: C11 + 1e+06 alpha_ob11_seg4[3] <= 1.0000273405291894e+06
 R1947: alpha_ob11_seg4[0] + alpha_ob11_seg4[1] + alpha_ob11_seg4[2]
   + alpha_ob11_seg4[3] >= 1
 R1948: - C8 + 1e+06 alpha_ob12_seg4[0] <= 999981.6050802565
 R1949: - C10 + 1e+06 alpha_ob12_seg4[0] <= 999981.6050802565
 R1950: - C9 + 1e+06 alpha_ob12_seg4[1] <= 999972.8942002755
 R1951: - C11 + 1e+06 alpha_ob12_seg4[1] <= 999972.8942002755
 R1952: C8 + 1e+06 alpha_ob12_seg4[2] <= 1.0000158448802566e+06
 R1953: C10 + 1e+06 alpha_ob12_seg4[2] <= 1.0000158448802566e+06
 R1954: C9 + 1e+06 alpha_ob12_seg4[3] <= 1.0000139558002756e+06
 R1955: C11 + 1e+06 alpha_ob12_seg4[3] <= 1.0000139558002756e+06
 R1956: alpha_ob12_seg4[0] + alpha_ob12_seg4[1] + alpha_ob12_seg4[2]
   + alpha_ob12_seg4[3] >= 1
 R1957: - C8 + 1e+06 alpha_ob13_seg4[0] <= 999981.605149801
 R1958: - C10 + 1e+06 alpha_ob13_seg4[0] <= 999981.605149801
 R1959: - C9 + 1e+06 alpha_ob13_seg4[1] <= 999983.4942391404
 R1960: - C11 + 1e+06 alpha_ob13_seg4[1] <= 999983.4942391404
 R1961: C8 + 1e+06 alpha_ob13_seg4[2] <= 999986.4948498011
 R1962: C10 + 1e+06 alpha_ob13_seg4[2] <= 999986.4948498011
 R1963: C9 + 1e+06 alpha_ob13_seg4[3] <= 1.0000139558391405e+06
 R1964: C11 + 1e+06 alpha_ob13_seg4[3] <= 1.0000139558391405e+06
 R1965: alpha_ob13_seg4[0] + alpha_ob13_seg4[1] + alpha_ob13_seg4[2]
   + alpha_ob13_seg4[3] >= 1
 R1966: - C8 + 1e+06 alpha_ob14_seg4[0] <= 999997.6803502565
 R1967: - C10 + 1e+06 alpha_ob14_seg4[0] <= 999997.6803502565
 R1968: - C9 + 1e+06 alpha_ob14_seg4[1] <= 999972.7268002755
 R1969: - C11 + 1e+06 alpha_ob14_seg4[1] <= 999972.7268002755
 R1970: C8 + 1e+06 alpha_ob14_seg4[2] <= 999999.7696102565
 R1971: C10 + 1e+06 alpha_ob14_seg4[2] <= 999999.7696102565
 R1972: C9 + 1e+06 alpha_ob14_seg4[3] <= 1.0000141232002755e+06
 R1973: C11 + 1e+06 alpha_ob14_seg4[3] <= 1.0000141232002755e+06
 R1974: alpha_ob14_seg4[0] + alpha_ob14_seg4[1] + alpha_ob14_seg4[2]
   + alpha_ob14_seg4[3] >= 1
 R1975: - C8 + 1e+06 alpha_ob15_seg4[0] <= 1.0000038341502565e+06
 R1976: - C10 + 1e+06 alpha_ob15_seg4[0] <= 1.0000038341502565e+06
 R1977: - C9 + 1e+06 alpha_ob15_seg4[1] <= 999972.6012002755
 R1978: - C11 + 1e+06 alpha_ob15_seg4[1] <= 999972.6012002755
 R1979: C8 + 1e+06 alpha_ob15_seg4[2] <= 999993.6158102566
 R1980: C10 + 1e+06 alpha_ob15_seg4[2] <= 999993.6158102566
 R1981: C9 + 1e+06 alpha_ob15_seg4[3] <= 1.0000142488002755e+06
 R1982: C11 + 1e+06 alpha_ob15_seg4[3] <= 1.0000142488002755e+06
 R1983: alpha_ob15_seg4[0] + alpha_ob15_seg4[1] + alpha_ob15_seg4[2]
   + alpha_ob15_seg4[3] >= 1
 R1984: - C8 + 1e+06 alpha_ob16_seg4[0] <= 1.0000162874063333e+06
 R1985: - C10 + 1e+06 alpha_ob16_seg4[0] <= 1.0000162874063333e+06
 R1986: - C9 + 1e+06 alpha_ob16_seg4[1] <= 999982.9439602756
 R1987: - C11 + 1e+06 alpha_ob16_seg4[1] <= 999982.9439602756
 R1988: C8 + 1e+06 alpha_ob16_seg4[2] <= 999981.1624063333
 R1989: C10 + 1e+06 alpha_ob16_seg4[2] <= 999981.1624063333
 R1990: C9 + 1e+06 alpha_ob16_seg4[3] <= 999963.6560402757
 R1991: C11 + 1e+06 alpha_ob16_seg4[3] <= 999963.6560402757
 R1992: alpha_ob16_seg4[0] + alpha_ob16_seg4[1] + alpha_ob16_seg4[2]
   + alpha_ob16_seg4[3] >= 1
 R1993: - C8 + 1e+06 alpha_ob17_seg4[0] <= 999980.937509
 R1994: - C10 + 1e+06 alpha_ob17_seg4[0] <= 999980.937509
 R1995: - C9 + 1e+06 alpha_ob17_seg4[1] <= 1.000034894e+06
 R1996: - C11 + 1e+06 alpha_ob17_seg4[1] <= 1.000034894e+06
 R1997: C8 + 1e+06 alpha_ob17_seg4[2] <= 999981.162491
 R1998: C10 + 1e+06 alpha_ob17_seg4[2] <= 999981.162491
 R1999: C9 + 1e+06 alpha_ob17_seg4[3] <= 999962.556
 R2000: C11 + 1e+06 alpha_ob17_seg4[3] <= 999962.556
 R2001: alpha_ob17_seg4[0] + alpha_ob17_seg4[1] + alpha_ob17_seg4[2]
   + alpha_ob17_seg4[3] >= 1
 R2002: - C8 + 1e+06 alpha_ob18_seg4[0] <= 999990.2670277582
 R2003: - C10 + 1e+06 alpha_ob18_seg4[0] <= 999990.2670277582
 R2004: - C9 + 1e+06 alpha_ob18_seg4[1] <= 1.0000347702002756e+06
 R2005: - C11 + 1e+06 alpha_ob18_seg4[1] <= 1.0000347702002756e+06
 R2006: C8 + 1e+06 alpha_ob18_seg4[2] <= 1.0000071828877581e+06
 R2007: C10 + 1e+06 alpha_ob18_seg4[2] <= 1.0000071828877581e+06
 R2008: C9 + 1e+06 alpha_ob18_seg4[3] <= 999939.8298002756
 R2009: C11 + 1e+06 alpha_ob18_seg4[3] <= 999939.8298002756
 R2010: alpha_ob18_seg4[0] + alpha_ob18_seg4[1] + alpha_ob18_seg4[2]
   + alpha_ob18_seg4[3] >= 1
 R2011: - C8 + 1e+06 alpha_ob19_seg4[0] <= 999990.267069801
 R2012: - C10 + 1e+06 alpha_ob19_seg4[0] <= 999990.267069801
 R2013: - C9 + 1e+06 alpha_ob19_seg4[1] <= 1.0000576202069657e+06
 R2014: - C11 + 1e+06 alpha_ob19_seg4[1] <= 1.0000576202069657e+06
 R2015: C8 + 1e+06 alpha_ob19_seg4[2] <= 1.000002082929801e+06
 R2016: C10 + 1e+06 alpha_ob19_seg4[2] <= 1.000002082929801e+06
 R2017: C9 + 1e+06 alpha_ob19_seg4[3] <= 999939.8298069657
 R2018: C11 + 1e+06 alpha_ob19_seg4[3] <= 999939.8298069657
 R2019: alpha_ob19_seg4[0] + alpha_ob19_seg4[1] + alpha_ob19_seg4[2]
   + alpha_ob19_seg4[3] >= 1
 R2020: - C8 + 1e+06 alpha_ob20_seg4[0] <= 999991.073732
 R2021: - C10 + 1e+06 alpha_ob20_seg4[0] <= 999991.073732
 R2022: - C9 + 1e+06 alpha_ob20_seg4[1] <= 999952.2125
 R2023: - C11 + 1e+06 alpha_ob20_seg4[1] <= 999952.2125
 R2024: C8 + 1e+06 alpha_ob20_seg4[2] <= 999990.776268
 R2025: C10 + 1e+06 alpha_ob20_seg4[2] <= 999990.776268
 R2026: C9 + 1e+06 alpha_ob20_seg4[3] <= 1.0000452375e+06
 R2027: C11 + 1e+06 alpha_ob20_seg4[3] <= 1.0000452375e+06
 R2028: alpha_ob20_seg4[0] + alpha_ob20_seg4[1] + alpha_ob20_seg4[2]
   + alpha_ob20_seg4[3] >= 1
 R2029: - C8 + 1e+06 alpha_ob21_seg4[0] <= 999985.657099801
 R2030: - C10 + 1e+06 alpha_ob21_seg4[0] <= 999985.657099801
 R2031: - C9 + 1e+06 alpha_ob21_seg4[1] <= 1.0000576202063023e+06
 R2032: - C11 + 1e+06 alpha_ob21_seg4[1] <= 1.0000576202063023e+06
 R2033: C8 + 1e+06 alpha_ob21_seg4[2] <= 1.000007192899801e+06
 R2034: C10 + 1e+06 alpha_ob21_seg4[2] <= 1.000007192899801e+06
 R2035: C9 + 1e+06 alpha_ob21_seg4[3] <= 999939.8298063023
 R2036: C11 + 1e+06 alpha_ob21_seg4[3] <= 999939.8298063023
 R2037: alpha_ob21_seg4[0] + alpha_ob21_seg4[1] + alpha_ob21_seg4[2]
   + alpha_ob21_seg4[3] >= 1
 R2038: - C8 + 1e+06 alpha_ob22_seg4[0] <= 999994.771829801
 R2039: - C10 + 1e+06 alpha_ob22_seg4[0] <= 999994.771829801
 R2040: - C9 + 1e+06 alpha_ob22_seg4[1] <= 1.0000615983063023e+06
 R2041: - C11 + 1e+06 alpha_ob22_seg4[1] <= 1.0000615983063023e+06
 R2042: C8 + 1e+06 alpha_ob22_seg4[2] <= 999998.078169801
 R2043: C10 + 1e+06 alpha_ob22_seg4[2] <= 999998.078169801
 R2044: C9 + 1e+06 alpha_ob22_seg4[3] <= 999935.8517063023
 R2045: C11 + 1e+06 alpha_ob22_seg4[3] <= 999935.8517063023
 R2046: alpha_ob22_seg4[0] + alpha_ob22_seg4[1] + alpha_ob22_seg4[2]
   + alpha_ob22_seg4[3] >= 1
 R2047: - C8 + 1e+06 alpha_ob23_seg4[0] <= 999980.9374508709
 R2048: - C10 + 1e+06 alpha_ob23_seg4[0] <= 999980.9374508709
 R2049: - C9 + 1e+06 alpha_ob23_seg4[1] <= 1.0000348940002755e+06
 R2050: - C11 + 1e+06 alpha_ob23_seg4[1] <= 1.0000348940002755e+06
 R2051: C8 + 1e+06 alpha_ob23_seg4[2] <= 1.0000165124508708e+06
 R2052: C10 + 1e+06 alpha_ob23_seg4[2] <= 1.0000165124508708e+06
 R2053: C9 + 1e+06 alpha_ob23_seg4[3] <= 999935.9560002756
 R2054: C11 + 1e+06 alpha_ob23_seg4[3] <= 999935.9560002756
 R2055: alpha_ob23_seg4[0] + alpha_ob23_seg4[1] + alpha_ob23_seg4[2]
   + alpha_ob23_seg4[3] >= 1
 R2056: - C8 + 1e+06 alpha_ob24_seg4[0] <= 999980.937499801
 R2057: - C10 + 1e+06 alpha_ob24_seg4[0] <= 999980.937499801
 R2058: - C9 + 1e+06 alpha_ob24_seg4[1] <= 1.0000614940072973e+06
 R2059: - C11 + 1e+06 alpha_ob24_seg4[1] <= 1.0000614940072973e+06
 R2060: C8 + 1e+06 alpha_ob24_seg4[2] <= 1.000011162499801e+06
 R2061: C10 + 1e+06 alpha_ob24_seg4[2] <= 1.000011162499801e+06
 R2062: C9 + 1e+06 alpha_ob24_seg4[3] <= 999935.9560072974
 R2063: C11 + 1e+06 alpha_ob24_seg4[3] <= 999935.9560072974
 R2064: alpha_ob24_seg4[0] + alpha_ob24_seg4[1] + alpha_ob24_seg4[2]
   + alpha_ob24_seg4[3] >= 1
 R2065: - C8 + 1e+06 alpha_ob25_seg4[0] <= 999956.8394000001
 R2066: - C10 + 1e+06 alpha_ob25_seg4[0] <= 999956.8394000001
 R2067: - C9 + 1e+06 alpha_ob25_seg4[1] <= 999952.2463
 R2068: - C11 + 1e+06 alpha_ob25_seg4[1] <= 999952.2463
 R2069: C8 + 1e+06 alpha_ob25_seg4[2] <= 1.0000097606e+06
 R2070: C10 + 1e+06 alpha_ob25_seg4[2] <= 1.0000097606e+06
 R2071: C9 + 1e+06 alpha_ob25_seg4[3] <= 1.0000452037e+06
 R2072: C11 + 1e+06 alpha_ob25_seg4[3] <= 1.0000452037e+06
 R2073: alpha_ob25_seg4[0] + alpha_ob25_seg4[1] + alpha_ob25_seg4[2]
   + alpha_ob25_seg4[3] >= 1
 R2074: - C8 + 1e+06 alpha_ob26_seg4[0] <= 999980.9375348955
 R2075: - C10 + 1e+06 alpha_ob26_seg4[0] <= 999980.9375348955
 R2076: - C9 + 1e+06 alpha_ob26_seg4[1] <= 1.0000160439997246e+06
 R2077: - C11 + 1e+06 alpha_ob26_seg4[1] <= 1.0000160439997246e+06
 R2078: C8 + 1e+06 alpha_ob26_seg4[2] <= 1.0000165125348954e+06
 R2079: C10 + 1e+06 alpha_ob26_seg4[2] <= 1.0000165125348954e+06
 R2080: C9 + 1e+06 alpha_ob26_seg4[3] <= 999962.5559997245
 R2081: C11 + 1e+06 alpha_ob26_seg4[3] <= 999962.5559997245
 R2082: alpha_ob26_seg4[0] + alpha_ob26_seg4[1] + alpha_ob26_seg4[2]
   + alpha_ob26_seg4[3] >= 1
 R2083: - C8 + 1e+06 alpha_ob27_seg4[0] <= 999956.839399801
 R2084: - C10 + 1e+06 alpha_ob27_seg4[0] <= 999956.839399801
 R2085: - C9 + 1e+06 alpha_ob27_seg4[1] <= 999925.3963434525
 R2086: - C11 + 1e+06 alpha_ob27_seg4[1] <= 999925.3963434525
 R2087: C8 + 1e+06 alpha_ob27_seg4[2] <= 1.0000080105998011e+06
 R2088: C10 + 1e+06 alpha_ob27_seg4[2] <= 1.0000080105998011e+06
 R2089: C9 + 1e+06 alpha_ob27_seg4[3] <= 1.0000720537434525e+06
 R2090: C11 + 1e+06 alpha_ob27_seg4[3] <= 1.0000720537434525e+06
 R2091: alpha_ob27_seg4[0] + alpha_ob27_seg4[1] + alpha_ob27_seg4[2]
   + alpha_ob27_seg4[3] >= 1
 R2092: - C8 + 1e+06 alpha_ob28_seg4[0] <= 999980.9374998009
 R2093: - C10 + 1e+06 alpha_ob28_seg4[0] <= 999980.9374998009
 R2094: - C9 + 1e+06 alpha_ob28_seg4[1] <= 1.0000160440053071e+06
 R2095: - C11 + 1e+06 alpha_ob28_seg4[1] <= 1.0000160440053071e+06
 R2096: C8 + 1e+06 alpha_ob28_seg4[2] <= 1.000012662499801e+06
 R2097: C10 + 1e+06 alpha_ob28_seg4[2] <= 1.000012662499801e+06
 R2098: C9 + 1e+06 alpha_ob28_seg4[3] <= 999981.4060053072
 R2099: C11 + 1e+06 alpha_ob28_seg4[3] <= 999981.4060053072
 R2100: alpha_ob28_seg4[0] + alpha_ob28_seg4[1] + alpha_ob28_seg4[2]
   + alpha_ob28_seg4[3] >= 1
 R2101: - C8 + 1e+06 alpha_ob29_seg4[0] <= 999983.6053459151
 R2102: - C10 + 1e+06 alpha_ob29_seg4[0] <= 999983.6053459151
 R2103: - C9 + 1e+06 alpha_ob29_seg4[1] <= 999987.7722497246
 R2104: - C11 + 1e+06 alpha_ob29_seg4[1] <= 999987.7722497246
 R2105: C8 + 1e+06 alpha_ob29_seg4[2] <= 1.000013844745915e+06
 R2106: C10 + 1e+06 alpha_ob29_seg4[2] <= 1.000013844745915e+06
 R2107: C9 + 1e+06 alpha_ob29_seg4[3] <= 999984.8277497246
 R2108: C11 + 1e+06 alpha_ob29_seg4[3] <= 999984.8277497246
 R2109: alpha_ob29_seg4[0] + alpha_ob29_seg4[1] + alpha_ob29_seg4[2]
   + alpha_ob29_seg4[3] >= 1
 R2110: - C8 + 1e+06 alpha_ob30_seg4[0] <= 999958.8717898987
 R2111: - C10 + 1e+06 alpha_ob30_seg4[0] <= 999958.8717898987
 R2112: - C9 + 1e+06 alpha_ob30_seg4[1] <= 999987.7837602755
 R2113: - C11 + 1e+06 alpha_ob30_seg4[1] <= 999987.7837602755
 R2114: C8 + 1e+06 alpha_ob30_seg4[2] <= 1.0000385781898986e+06
 R2115: C10 + 1e+06 alpha_ob30_seg4[2] <= 1.0000385781898986e+06
 R2116: C9 + 1e+06 alpha_ob30_seg4[3] <= 1.0000043162402755e+06
 R2117: C11 + 1e+06 alpha_ob30_seg4[3] <= 1.0000043162402755e+06
 R2118: alpha_ob30_seg4[0] + alpha_ob30_seg4[1] + alpha_ob30_seg4[2]
   + alpha_ob30_seg4[3] >= 1
 R2119: - C8 + 1e+06 alpha_ob31_seg4[0] <= 999939.2718
 R2120: - C10 + 1e+06 alpha_ob31_seg4[0] <= 999939.2718
 R2121: - C9 + 1e+06 alpha_ob31_seg4[1] <= 999993.13376
 R2122: - C11 + 1e+06 alpha_ob31_seg4[1] <= 999993.13376
 R2123: C8 + 1e+06 alpha_ob31_seg4[2] <= 1.0000385782e+06
 R2124: C10 + 1e+06 alpha_ob31_seg4[2] <= 1.0000385782e+06
 R2125: C9 + 1e+06 alpha_ob31_seg4[3] <= 1.00000431624e+06
 R2126: C11 + 1e+06 alpha_ob31_seg4[3] <= 1.00000431624e+06
 R2127: alpha_ob31_seg4[0] + alpha_ob31_seg4[1] + alpha_ob31_seg4[2]
   + alpha_ob31_seg4[3] >= 1
 R2128: - C8 + 1e+06 alpha_ob32_seg4[0] <= 999939.2718101013
 R2129: - C10 + 1e+06 alpha_ob32_seg4[0] <= 999939.2718101013
 R2130: - C9 + 1e+06 alpha_ob32_seg4[1] <= 999987.7837597246
 R2131: - C11 + 1e+06 alpha_ob32_seg4[1] <= 999987.7837597246
 R2132: C8 + 1e+06 alpha_ob32_seg4[2] <= 1.0000581782101013e+06
 R2133: C10 + 1e+06 alpha_ob32_seg4[2] <= 1.0000581782101013e+06
 R2134: C9 + 1e+06 alpha_ob32_seg4[3] <= 1.0000043162397245e+06
 R2135: C11 + 1e+06 alpha_ob32_seg4[3] <= 1.0000043162397245e+06
 R2136: alpha_ob32_seg4[0] + alpha_ob32_seg4[1] + alpha_ob32_seg4[2]
   + alpha_ob32_seg4[3] >= 1
 R2137: - C8 + 1e+06 alpha_ob33_seg4[0] <= 999939.271799801
 R2138: - C10 + 1e+06 alpha_ob33_seg4[0] <= 999939.271799801
 R2139: - C9 + 1e+06 alpha_ob33_seg4[1] <= 999987.7838262042
 R2140: - C11 + 1e+06 alpha_ob33_seg4[1] <= 999987.7838262042
 R2141: C8 + 1e+06 alpha_ob33_seg4[2] <= 1.000038578199801e+06
 R2142: C10 + 1e+06 alpha_ob33_seg4[2] <= 1.000038578199801e+06
 R2143: C9 + 1e+06 alpha_ob33_seg4[3] <= 1.0000096662262043e+06
 R2144: C11 + 1e+06 alpha_ob33_seg4[3] <= 1.0000096662262043e+06
 R2145: alpha_ob33_seg4[0] + alpha_ob33_seg4[1] + alpha_ob33_seg4[2]
   + alpha_ob33_seg4[3] >= 1
 R2146: - C8 + 1e+06 alpha_ob34_seg4[0] <= 999926.0708
 R2147: - C10 + 1e+06 alpha_ob34_seg4[0] <= 999926.0708
 R2148: - C9 + 1e+06 alpha_ob34_seg4[1] <= 999983.7196
 R2149: - C11 + 1e+06 alpha_ob34_seg4[1] <= 999983.7196
 R2150: C8 + 1e+06 alpha_ob34_seg4[2] <= 1.0000250292e+06
 R2151: C10 + 1e+06 alpha_ob34_seg4[2] <= 1.0000250292e+06
 R2152: C9 + 1e+06 alpha_ob34_seg4[3] <= 1.0000137304e+06
 R2153: C11 + 1e+06 alpha_ob34_seg4[3] <= 1.0000137304e+06
 R2154: alpha_ob34_seg4[0] + alpha_ob34_seg4[1] + alpha_ob34_seg4[2]
   + alpha_ob34_seg4[3] >= 1
 R2155: - C8 + 1e+06 alpha_ob35_seg4[0] <= 999926.0708293857
 R2156: - C10 + 1e+06 alpha_ob35_seg4[0] <= 999926.0708293857
 R2157: - C9 + 1e+06 alpha_ob35_seg4[1] <= 999967.8695997245
 R2158: - C11 + 1e+06 alpha_ob35_seg4[1] <= 999967.8695997245
 R2159: C8 + 1e+06 alpha_ob35_seg4[2] <= 1.0000713792293856e+06
 R2160: C10 + 1e+06 alpha_ob35_seg4[2] <= 1.0000713792293856e+06
 R2161: C9 + 1e+06 alpha_ob35_seg4[3] <= 1.0000137303997246e+06
 R2162: C11 + 1e+06 alpha_ob35_seg4[3] <= 1.0000137303997246e+06
 R2163: alpha_ob35_seg4[0] + alpha_ob35_seg4[1] + alpha_ob35_seg4[2]
   + alpha_ob35_seg4[3] >= 1
 R2164: - C8 + 1e+06 alpha_ob36_seg4[0] <= 999972.6917508709
 R2165: - C10 + 1e+06 alpha_ob36_seg4[0] <= 999972.6917508709
 R2166: - C9 + 1e+06 alpha_ob36_seg4[1] <= 999957.1644002756
 R2167: - C11 + 1e+06 alpha_ob36_seg4[1] <= 999957.1644002756
 R2168: C8 + 1e+06 alpha_ob36_seg4[2] <= 1.0000247581508709e+06
 R2169: C10 + 1e+06 alpha_ob36_seg4[2] <= 1.0000247581508709e+06
 R2170: C9 + 1e+06 alpha_ob36_seg4[3] <= 1.0000136856002755e+06
 R2171: C11 + 1e+06 alpha_ob36_seg4[3] <= 1.0000136856002755e+06
 R2172: alpha_ob36_seg4[0] + alpha_ob36_seg4[1] + alpha_ob36_seg4[2]
   + alpha_ob36_seg4[3] >= 1
 R2173: - C8 + 1e+06 alpha_ob37_seg4[0] <= 999939.0009674003
 R2174: - C10 + 1e+06 alpha_ob37_seg4[0] <= 999939.0009674003
 R2175: - C9 + 1e+06 alpha_ob37_seg4[1] <= 999961.9516002756
 R2176: - C11 + 1e+06 alpha_ob37_seg4[1] <= 999961.9516002756
 R2177: C8 + 1e+06 alpha_ob37_seg4[2] <= 1.0000584489674004e+06
 R2178: C10 + 1e+06 alpha_ob37_seg4[2] <= 1.0000584489674004e+06
 R2179: C9 + 1e+06 alpha_ob37_seg4[3] <= 1.0000178984002755e+06
 R2180: C11 + 1e+06 alpha_ob37_seg4[3] <= 1.0000178984002755e+06
 R2181: alpha_ob37_seg4[0] + alpha_ob37_seg4[1] + alpha_ob37_seg4[2]
   + alpha_ob37_seg4[3] >= 1
 R2182: - C8 + 1e+06 alpha_ob38_seg4[0] <= 999994.9164998011
 R2183: - C10 + 1e+06 alpha_ob38_seg4[0] <= 999994.9164998011
 R2184: - C9 + 1e+06 alpha_ob38_seg4[1] <= 999925.4284782809
 R2185: - C11 + 1e+06 alpha_ob38_seg4[1] <= 999925.4284782809
 R2186: C8 + 1e+06 alpha_ob38_seg4[2] <= 999943.6834998011
 R2187: C10 + 1e+06 alpha_ob38_seg4[2] <= 999943.6834998011
 R2188: C9 + 1e+06 alpha_ob38_seg4[3] <= 1.0000720216782809e+06
 R2189: C11 + 1e+06 alpha_ob38_seg4[3] <= 1.0000720216782809e+06
 R2190: alpha_ob38_seg4[0] + alpha_ob38_seg4[1] + alpha_ob38_seg4[2]
   + alpha_ob38_seg4[3] >= 1
 R2191: - C8 + 1e+06 alpha_ob39_seg4[0] <= 999914.52327842
 R2192: - C10 + 1e+06 alpha_ob39_seg4[0] <= 999914.52327842
 R2193: - C9 + 1e+06 alpha_ob39_seg4[1] <= 999993.6552362755
 R2194: - C11 + 1e+06 alpha_ob39_seg4[1] <= 999993.6552362755
 R2195: C8 + 1e+06 alpha_ob39_seg4[2] <= 1.0000829266784199e+06
 R2196: C10 + 1e+06 alpha_ob39_seg4[2] <= 1.0000829266784199e+06
 R2197: C9 + 1e+06 alpha_ob39_seg4[3] <= 999992.1947642756
 R2198: C11 + 1e+06 alpha_ob39_seg4[3] <= 999992.1947642756
 R2199: alpha_ob39_seg4[0] + alpha_ob39_seg4[1] + alpha_ob39_seg4[2]
   + alpha_ob39_seg4[3] >= 1
 R2200: - C8 + 1e+06 alpha_ob40_seg4[0] <= 999912.265199801
 R2201: - C10 + 1e+06 alpha_ob40_seg4[0] <= 999912.265199801
 R2202: - C9 + 1e+06 alpha_ob40_seg4[1] <= 999987.7842112777
 R2203: - C11 + 1e+06 alpha_ob40_seg4[1] <= 999987.7842112777
 R2204: C8 + 1e+06 alpha_ob40_seg4[2] <= 1.000076834799801e+06
 R2205: C10 + 1e+06 alpha_ob40_seg4[2] <= 1.000076834799801e+06
 R2206: C9 + 1e+06 alpha_ob40_seg4[3] <= 1.0000096658112778e+06
 R2207: C11 + 1e+06 alpha_ob40_seg4[3] <= 1.0000096658112778e+06
 R2208: alpha_ob40_seg4[0] + alpha_ob40_seg4[1] + alpha_ob40_seg4[2]
   + alpha_ob40_seg4[3] >= 1
 R2209: - C8 + 1e+06 alpha_ob41_seg4[0] <= 999920.6152119379
 R2210: - C10 + 1e+06 alpha_ob41_seg4[0] <= 999920.6152119379
 R2211: - C9 + 1e+06 alpha_ob41_seg4[1] <= 999981.4341997246
 R2212: - C11 + 1e+06 alpha_ob41_seg4[1] <= 999981.4341997246
 R2213: C8 + 1e+06 alpha_ob41_seg4[2] <= 1.0000768348119379e+06
 R2214: C10 + 1e+06 alpha_ob41_seg4[2] <= 1.0000768348119379e+06
 R2215: C9 + 1e+06 alpha_ob41_seg4[3] <= 1.0000096657997245e+06
 R2216: C11 + 1e+06 alpha_ob41_seg4[3] <= 1.0000096657997245e+06
 R2217: alpha_ob41_seg4[0] + alpha_ob41_seg4[1] + alpha_ob41_seg4[2]
   + alpha_ob41_seg4[3] >= 1
 R2218: - C8 + 1e+06 alpha_ob42_seg4[0] <= 999919.2652
 R2219: - C10 + 1e+06 alpha_ob42_seg4[0] <= 999919.2652
 R2220: - C9 + 1e+06 alpha_ob42_seg4[1] <= 999981.4342
 R2221: - C11 + 1e+06 alpha_ob42_seg4[1] <= 999981.4342
 R2222: C8 + 1e+06 alpha_ob42_seg4[2] <= 1.0000768348e+06
 R2223: C10 + 1e+06 alpha_ob42_seg4[2] <= 1.0000768348e+06
 R2224: C9 + 1e+06 alpha_ob42_seg4[3] <= 1.0000160158000001e+06
 R2225: C11 + 1e+06 alpha_ob42_seg4[3] <= 1.0000160158000001e+06
 R2226: alpha_ob42_seg4[0] + alpha_ob42_seg4[1] + alpha_ob42_seg4[2]
   + alpha_ob42_seg4[3] >= 1
 R2227: - C8 + 1e+06 alpha_ob43_seg4[0] <= 999919.2652454559
 R2228: - C10 + 1e+06 alpha_ob43_seg4[0] <= 999919.2652454559
 R2229: - C9 + 1e+06 alpha_ob43_seg4[1] <= 999956.8341997246
 R2230: - C11 + 1e+06 alpha_ob43_seg4[1] <= 999956.8341997246
 R2231: C8 + 1e+06 alpha_ob43_seg4[2] <= 1.0000781848454559e+06
 R2232: C10 + 1e+06 alpha_ob43_seg4[2] <= 1.0000781848454559e+06
 R2233: C9 + 1e+06 alpha_ob43_seg4[3] <= 1.0000160157997246e+06
 R2234: C11 + 1e+06 alpha_ob43_seg4[3] <= 1.0000160157997246e+06
 R2235: alpha_ob43_seg4[0] + alpha_ob43_seg4[1] + alpha_ob43_seg4[2]
   + alpha_ob43_seg4[3] >= 1
 R2236: - C8 + 1e+06 alpha_ob44_seg4[0] <= 999991.83683
 R2237: - C10 + 1e+06 alpha_ob44_seg4[0] <= 999991.83683
 R2238: - C9 + 1e+06 alpha_ob44_seg4[1] <= 999940.589
 R2239: - C11 + 1e+06 alpha_ob44_seg4[1] <= 999940.589
 R2240: C8 + 1e+06 alpha_ob44_seg4[2] <= 1.00000401317e+06
 R2241: C10 + 1e+06 alpha_ob44_seg4[2] <= 1.00000401317e+06
 R2242: C9 + 1e+06 alpha_ob44_seg4[3] <= 1.000056861e+06
 R2243: C11 + 1e+06 alpha_ob44_seg4[3] <= 1.000056861e+06
 R2244: alpha_ob44_seg4[0] + alpha_ob44_seg4[1] + alpha_ob44_seg4[2]
   + alpha_ob44_seg4[3] >= 1
 R2245: - C8 + 1e+06 alpha_ob45_seg4[0] <= 999991.8368084199
 R2246: - C10 + 1e+06 alpha_ob45_seg4[0] <= 999991.8368084199
 R2247: - C9 + 1e+06 alpha_ob45_seg4[1] <= 999940.5890002755
 R2248: - C11 + 1e+06 alpha_ob45_seg4[1] <= 999940.5890002755
 R2249: C8 + 1e+06 alpha_ob45_seg4[2] <= 1.0000056131484199e+06
 R2250: C10 + 1e+06 alpha_ob45_seg4[2] <= 1.0000056131484199e+06
 R2251: C9 + 1e+06 alpha_ob45_seg4[3] <= 1.0000452610002755e+06
 R2252: C11 + 1e+06 alpha_ob45_seg4[3] <= 1.0000452610002755e+06
 R2253: alpha_ob45_seg4[0] + alpha_ob45_seg4[1] + alpha_ob45_seg4[2]
   + alpha_ob45_seg4[3] >= 1
 R2254: - C8 + 1e+06 alpha_ob46_seg4[0] <= 1.0000764021508709e+06
 R2255: - C10 + 1e+06 alpha_ob46_seg4[0] <= 1.0000764021508709e+06
 R2256: - C9 + 1e+06 alpha_ob46_seg4[1] <= 999957.1027002756
 R2257: - C11 + 1e+06 alpha_ob46_seg4[1] <= 999957.1027002756
 R2258: C8 + 1e+06 alpha_ob46_seg4[2] <= 999921.0477508709
 R2259: C10 + 1e+06 alpha_ob46_seg4[2] <= 999921.0477508709
 R2260: C9 + 1e+06 alpha_ob46_seg4[3] <= 1.0000137473002756e+06
 R2261: C11 + 1e+06 alpha_ob46_seg4[3] <= 1.0000137473002756e+06
 R2262: alpha_ob46_seg4[0] + alpha_ob46_seg4[1] + alpha_ob46_seg4[2]
   + alpha_ob46_seg4[3] >= 1
 R2263: - C8 + 1e+06 alpha_ob47_seg4[0] <= 1.0000445522e+06
 R2264: - C10 + 1e+06 alpha_ob47_seg4[0] <= 1.0000445522e+06
 R2265: - C9 + 1e+06 alpha_ob47_seg4[1] <= 999983.7027
 R2266: - C11 + 1e+06 alpha_ob47_seg4[1] <= 999983.7027
 R2267: C8 + 1e+06 alpha_ob47_seg4[2] <= 999921.0477999999
 R2268: C10 + 1e+06 alpha_ob47_seg4[2] <= 999921.0477999999
 R2269: C9 + 1e+06 alpha_ob47_seg4[3] <= 1.0000137473e+06
 R2270: C11 + 1e+06 alpha_ob47_seg4[3] <= 1.0000137473e+06
 R2271: alpha_ob47_seg4[0] + alpha_ob47_seg4[1] + alpha_ob47_seg4[2]
   + alpha_ob47_seg4[3] >= 1
 R2272: - C8 + 1e+06 alpha_ob48_seg4[0] <= 1.0000445522183661e+06
 R2273: - C10 + 1e+06 alpha_ob48_seg4[0] <= 1.0000445522183661e+06
 R2274: - C9 + 1e+06 alpha_ob48_seg4[1] <= 999973.8526997245
 R2275: - C11 + 1e+06 alpha_ob48_seg4[1] <= 999973.8526997245
 R2276: C8 + 1e+06 alpha_ob48_seg4[2] <= 999952.8978183661
 R2277: C10 + 1e+06 alpha_ob48_seg4[2] <= 999952.8978183661
 R2278: C9 + 1e+06 alpha_ob48_seg4[3] <= 1.0000137472997245e+06
 R2279: C11 + 1e+06 alpha_ob48_seg4[3] <= 1.0000137472997245e+06
 R2280: alpha_ob48_seg4[0] + alpha_ob48_seg4[1] + alpha_ob48_seg4[2]
   + alpha_ob48_seg4[3] >= 1
 R2281: - C8 + 1e+06 alpha_ob49_seg4[0] <= 1.000064332081634e+06
 R2282: - C10 + 1e+06 alpha_ob49_seg4[0] <= 1.000064332081634e+06
 R2283: - C9 + 1e+06 alpha_ob49_seg4[1] <= 999973.9401002756
 R2284: - C11 + 1e+06 alpha_ob49_seg4[1] <= 999973.9401002756
 R2285: C8 + 1e+06 alpha_ob49_seg4[2] <= 999933.1178816339
 R2286: C10 + 1e+06 alpha_ob49_seg4[2] <= 999933.1178816339
 R2287: C9 + 1e+06 alpha_ob49_seg4[3] <= 1.0000136599002755e+06
 R2288: C11 + 1e+06 alpha_ob49_seg4[3] <= 1.0000136599002755e+06
 R2289: alpha_ob49_seg4[0] + alpha_ob49_seg4[1] + alpha_ob49_seg4[2]
   + alpha_ob49_seg4[3] >= 1
 R2290: - C8 + 1e+06 alpha_ob50_seg4[0] <= 1.0000643321013774e+06
 R2291: - C10 + 1e+06 alpha_ob50_seg4[0] <= 1.0000643321013774e+06
 R2292: - C9 + 1e+06 alpha_ob50_seg4[1] <= 999983.1900997246
 R2293: - C11 + 1e+06 alpha_ob50_seg4[1] <= 999983.1900997246
 R2294: C8 + 1e+06 alpha_ob50_seg4[2] <= 999933.1179013775
 R2295: C10 + 1e+06 alpha_ob50_seg4[2] <= 999933.1179013775
 R2296: C9 + 1e+06 alpha_ob50_seg4[3] <= 1.0000136598997245e+06
 R2297: C11 + 1e+06 alpha_ob50_seg4[3] <= 1.0000136598997245e+06
 R2298: alpha_ob50_seg4[0] + alpha_ob50_seg4[1] + alpha_ob50_seg4[2]
   + alpha_ob50_seg4[3] >= 1
 R2299: - C10 + 1e+06 alpha_ob0_seg5[0] <= 1.0000284170499525e+06
 R2300: - C12 + 1e+06 alpha_ob0_seg5[0] <= 1.0000284170499525e+06
 R2301: - C11 + 1e+06 alpha_ob0_seg5[1] <= 999957.0843002755
 R2302: - C13 + 1e+06 alpha_ob0_seg5[1] <= 999957.0843002755
 R2303: C10 + 1e+06 alpha_ob0_seg5[2] <= 999969.0328499526
 R2304: C12 + 1e+06 alpha_ob0_seg5[2] <= 999969.0328499526
 R2305: C11 + 1e+06 alpha_ob0_seg5[3] <= 1.0000132657002755e+06
 R2306: C13 + 1e+06 alpha_ob0_seg5[3] <= 1.0000132657002755e+06
 R2307: alpha_ob0_seg5[0] + alpha_ob0_seg5[1] + alpha_ob0_seg5[2]
   + alpha_ob0_seg5[3] >= 1
 R2308: - C10 + 1e+06 alpha_ob1_seg5[0] <= 1.000028417099801e+06
 R2309: - C12 + 1e+06 alpha_ob1_seg5[0] <= 1.000028417099801e+06
 R2310: - C11 + 1e+06 alpha_ob1_seg5[1] <= 999984.1843155898
 R2311: - C13 + 1e+06 alpha_ob1_seg5[1] <= 999984.1843155898
 R2312: C10 + 1e+06 alpha_ob1_seg5[2] <= 999957.4328998009
 R2313: C12 + 1e+06 alpha_ob1_seg5[2] <= 999957.4328998009
 R2314: C11 + 1e+06 alpha_ob1_seg5[3] <= 1.0000132657155899e+06
 R2315: C13 + 1e+06 alpha_ob1_seg5[3] <= 1.0000132657155899e+06
 R2316: alpha_ob1_seg5[0] + alpha_ob1_seg5[1] + alpha_ob1_seg5[2]
   + alpha_ob1_seg5[3] >= 1
 R2317: - C10 + 1e+06 alpha_ob2_seg5[0] <= 1.0000400171500475e+06
 R2318: - C12 + 1e+06 alpha_ob2_seg5[0] <= 1.0000400171500475e+06
 R2319: - C11 + 1e+06 alpha_ob2_seg5[1] <= 999957.0842997245
 R2320: - C13 + 1e+06 alpha_ob2_seg5[1] <= 999957.0842997245
 R2321: C10 + 1e+06 alpha_ob2_seg5[2] <= 999957.4329500474
 R2322: C12 + 1e+06 alpha_ob2_seg5[2] <= 999957.4329500474
 R2323: C11 + 1e+06 alpha_ob2_seg5[3] <= 1.0000132656997246e+06
 R2324: C13 + 1e+06 alpha_ob2_seg5[3] <= 1.0000132656997246e+06
 R2325: alpha_ob2_seg5[0] + alpha_ob2_seg5[1] + alpha_ob2_seg5[2]
   + alpha_ob2_seg5[3] >= 1
 R2326: - C10 + 1e+06 alpha_ob3_seg5[0] <= 1.0000109161752058e+06
 R2327: - C12 + 1e+06 alpha_ob3_seg5[0] <= 1.0000109161752058e+06
 R2328: - C11 + 1e+06 alpha_ob3_seg5[1] <= 999970.2681002755
 R2329: - C13 + 1e+06 alpha_ob3_seg5[1] <= 999970.2681002755
 R2330: C10 + 1e+06 alpha_ob3_seg5[2] <= 999986.5337752058
 R2331: C12 + 1e+06 alpha_ob3_seg5[2] <= 999986.5337752058
 R2332: C11 + 1e+06 alpha_ob3_seg5[3] <= 1.0000138319002755e+06
 R2333: C13 + 1e+06 alpha_ob3_seg5[3] <= 1.0000138319002755e+06
 R2334: alpha_ob3_seg5[0] + alpha_ob3_seg5[1] + alpha_ob3_seg5[2]
   + alpha_ob3_seg5[3] >= 1
 R2335: - C10 + 1e+06 alpha_ob4_seg5[0] <= 1.000010916199801e+06
 R2336: - C12 + 1e+06 alpha_ob4_seg5[0] <= 1.000010916199801e+06
 R2337: - C11 + 1e+06 alpha_ob4_seg5[1] <= 999983.6181179117
 R2338: - C13 + 1e+06 alpha_ob4_seg5[1] <= 999983.6181179117
 R2339: C10 + 1e+06 alpha_ob4_seg5[2] <= 999973.183799801
 R2340: C12 + 1e+06 alpha_ob4_seg5[2] <= 999973.183799801
 R2341: C11 + 1e+06 alpha_ob4_seg5[3] <= 1.0000138319179118e+06
 R2342: C13 + 1e+06 alpha_ob4_seg5[3] <= 1.0000138319179118e+06
 R2343: alpha_ob4_seg5[0] + alpha_ob4_seg5[1] + alpha_ob4_seg5[2]
   + alpha_ob4_seg5[3] >= 1
 R2344: - C10 + 1e+06 alpha_ob5_seg5[0] <= 999992.49102
 R2345: - C12 + 1e+06 alpha_ob5_seg5[0] <= 999992.49102
 R2346: - C11 + 1e+06 alpha_ob5_seg5[1] <= 999957.1016000001
 R2347: - C13 + 1e+06 alpha_ob5_seg5[1] <= 999957.1016000001
 R2348: C10 + 1e+06 alpha_ob5_seg5[2] <= 999986.35898
 R2349: C12 + 1e+06 alpha_ob5_seg5[2] <= 999986.35898
 R2350: C11 + 1e+06 alpha_ob5_seg5[3] <= 1.0000403484e+06
 R2351: C13 + 1e+06 alpha_ob5_seg5[3] <= 1.0000403484e+06
 R2352: alpha_ob5_seg5[0] + alpha_ob5_seg5[1] + alpha_ob5_seg5[2]
   + alpha_ob5_seg5[3] >= 1
 R2353: - C10 + 1e+06 alpha_ob6_seg5[0] <= 999981.3909999999
 R2354: - C12 + 1e+06 alpha_ob6_seg5[0] <= 999981.3909999999
 R2355: - C11 + 1e+06 alpha_ob6_seg5[1] <= 999957.1016000001
 R2356: - C13 + 1e+06 alpha_ob6_seg5[1] <= 999957.1016000001
 R2357: C10 + 1e+06 alpha_ob6_seg5[2] <= 1.000004959e+06
 R2358: C12 + 1e+06 alpha_ob6_seg5[2] <= 1.000004959e+06
 R2359: C11 + 1e+06 alpha_ob6_seg5[3] <= 1.0000403484e+06
 R2360: C13 + 1e+06 alpha_ob6_seg5[3] <= 1.0000403484e+06
 R2361: alpha_ob6_seg5[0] + alpha_ob6_seg5[1] + alpha_ob6_seg5[2]
   + alpha_ob6_seg5[3] >= 1
 R2362: - C10 + 1e+06 alpha_ob7_seg5[0] <= 999981.3909825523
 R2363: - C12 + 1e+06 alpha_ob7_seg5[0] <= 999981.3909825523
 R2364: - C11 + 1e+06 alpha_ob7_seg5[1] <= 999957.1016002755
 R2365: - C13 + 1e+06 alpha_ob7_seg5[1] <= 999957.1016002755
 R2366: C10 + 1e+06 alpha_ob7_seg5[2] <= 1.0000160589825523e+06
 R2367: C12 + 1e+06 alpha_ob7_seg5[2] <= 1.0000160589825523e+06
 R2368: C11 + 1e+06 alpha_ob7_seg5[3] <= 1.0000309984002755e+06
 R2369: C13 + 1e+06 alpha_ob7_seg5[3] <= 1.0000309984002755e+06
 R2370: alpha_ob7_seg5[0] + alpha_ob7_seg5[1] + alpha_ob7_seg5[2]
   + alpha_ob7_seg5[3] >= 1
 R2371: - C10 + 1e+06 alpha_ob8_seg5[0] <= 999981.3867954084
 R2372: - C12 + 1e+06 alpha_ob8_seg5[0] <= 999981.3867954084
 R2373: - C11 + 1e+06 alpha_ob8_seg5[1] <= 999968.8595002755
 R2374: - C13 + 1e+06 alpha_ob8_seg5[1] <= 999968.8595002755
 R2375: C10 + 1e+06 alpha_ob8_seg5[2] <= 1.0000160631954084e+06
 R2376: C12 + 1e+06 alpha_ob8_seg5[2] <= 1.0000160631954084e+06
 R2377: C11 + 1e+06 alpha_ob8_seg5[3] <= 1.0000262405002755e+06
 R2378: C13 + 1e+06 alpha_ob8_seg5[3] <= 1.0000262405002755e+06
 R2379: alpha_ob8_seg5[0] + alpha_ob8_seg5[1] + alpha_ob8_seg5[2]
   + alpha_ob8_seg5[3] >= 1
 R2380: - C10 + 1e+06 alpha_ob9_seg5[0] <= 999981.386799801
 R2381: - C12 + 1e+06 alpha_ob9_seg5[0] <= 999981.386799801
 R2382: - C11 + 1e+06 alpha_ob9_seg5[1] <= 999971.2095013268
 R2383: - C13 + 1e+06 alpha_ob9_seg5[1] <= 999971.2095013268
 R2384: C10 + 1e+06 alpha_ob9_seg5[2] <= 1.0000152131998009e+06
 R2385: C12 + 1e+06 alpha_ob9_seg5[2] <= 1.0000152131998009e+06
 R2386: C11 + 1e+06 alpha_ob9_seg5[3] <= 1.0000262405013267e+06
 R2387: C13 + 1e+06 alpha_ob9_seg5[3] <= 1.0000262405013267e+06
 R2388: alpha_ob9_seg5[0] + alpha_ob9_seg5[1] + alpha_ob9_seg5[2]
   + alpha_ob9_seg5[3] >= 1
 R2389: - C10 + 1e+06 alpha_ob10_seg5[0] <= 999982.2368022958
 R2390: - C12 + 1e+06 alpha_ob10_seg5[0] <= 999982.2368022958
 R2391: - C11 + 1e+06 alpha_ob10_seg5[1] <= 999970.1094997245
 R2392: - C13 + 1e+06 alpha_ob10_seg5[1] <= 999970.1094997245
 R2393: C10 + 1e+06 alpha_ob10_seg5[2] <= 1.0000152132022957e+06
 R2394: C12 + 1e+06 alpha_ob10_seg5[2] <= 1.0000152132022957e+06
 R2395: C11 + 1e+06 alpha_ob10_seg5[3] <= 1.0000262404997245e+06
 R2396: C13 + 1e+06 alpha_ob10_seg5[3] <= 1.0000262404997245e+06
 R2397: alpha_ob10_seg5[0] + alpha_ob10_seg5[1] + alpha_ob10_seg5[2]
   + alpha_ob10_seg5[3] >= 1
 R2398: - C10 + 1e+06 alpha_ob11_seg5[0] <= 999982.236799801
 R2399: - C12 + 1e+06 alpha_ob11_seg5[0] <= 999982.236799801
 R2400: - C11 + 1e+06 alpha_ob11_seg5[1] <= 999970.1095291895
 R2401: - C13 + 1e+06 alpha_ob11_seg5[1] <= 999970.1095291895
 R2402: C10 + 1e+06 alpha_ob11_seg5[2] <= 999993.3631998011
 R2403: C12 + 1e+06 alpha_ob11_seg5[2] <= 999993.3631998011
 R2404: C11 + 1e+06 alpha_ob11_seg5[3] <= 1.0000273405291894e+06
 R2405: C13 + 1e+06 alpha_ob11_seg5[3] <= 1.0000273405291894e+06
 R2406: alpha_ob11_seg5[0] + alpha_ob11_seg5[1] + alpha_ob11_seg5[2]
   + alpha_ob11_seg5[3] >= 1
 R2407: - C10 + 1e+06 alpha_ob12_seg5[0] <= 999981.6050802565
 R2408: - C12 + 1e+06 alpha_ob12_seg5[0] <= 999981.6050802565
 R2409: - C11 + 1e+06 alpha_ob12_seg5[1] <= 999972.8942002755
 R2410: - C13 + 1e+06 alpha_ob12_seg5[1] <= 999972.8942002755
 R2411: C10 + 1e+06 alpha_ob12_seg5[2] <= 1.0000158448802566e+06
 R2412: C12 + 1e+06 alpha_ob12_seg5[2] <= 1.0000158448802566e+06
 R2413: C11 + 1e+06 alpha_ob12_seg5[3] <= 1.0000139558002756e+06
 R2414: C13 + 1e+06 alpha_ob12_seg5[3] <= 1.0000139558002756e+06
 R2415: alpha_ob12_seg5[0] + alpha_ob12_seg5[1] + alpha_ob12_seg5[2]
   + alpha_ob12_seg5[3] >= 1
 R2416: - C10 + 1e+06 alpha_ob13_seg5[0] <= 999981.605149801
 R2417: - C12 + 1e+06 alpha_ob13_seg5[0] <= 999981.605149801
 R2418: - C11 + 1e+06 alpha_ob13_seg5[1] <= 999983.4942391404
 R2419: - C13 + 1e+06 alpha_ob13_seg5[1] <= 999983.4942391404
 R2420: C10 + 1e+06 alpha_ob13_seg5[2] <= 999986.4948498011
 R2421: C12 + 1e+06 alpha_ob13_seg5[2] <= 999986.4948498011
 R2422: C11 + 1e+06 alpha_ob13_seg5[3] <= 1.0000139558391405e+06
 R2423: C13 + 1e+06 alpha_ob13_seg5[3] <= 1.0000139558391405e+06
 R2424: alpha_ob13_seg5[0] + alpha_ob13_seg5[1] + alpha_ob13_seg5[2]
   + alpha_ob13_seg5[3] >= 1
 R2425: - C10 + 1e+06 alpha_ob14_seg5[0] <= 999997.6803502565
 R2426: - C12 + 1e+06 alpha_ob14_seg5[0] <= 999997.6803502565
 R2427: - C11 + 1e+06 alpha_ob14_seg5[1] <= 999972.7268002755
 R2428: - C13 + 1e+06 alpha_ob14_seg5[1] <= 999972.7268002755
 R2429: C10 + 1e+06 alpha_ob14_seg5[2] <= 999999.7696102565
 R2430: C12 + 1e+06 alpha_ob14_seg5[2] <= 999999.7696102565
 R2431: C11 + 1e+06 alpha_ob14_seg5[3] <= 1.0000141232002755e+06
 R2432: C13 + 1e+06 alpha_ob14_seg5[3] <= 1.0000141232002755e+06
 R2433: alpha_ob14_seg5[0] + alpha_ob14_seg5[1] + alpha_ob14_seg5[2]
   + alpha_ob14_seg5[3] >= 1
 R2434: - C10 + 1e+06 alpha_ob15_seg5[0] <= 1.0000038341502565e+06
 R2435: - C12 + 1e+06 alpha_ob15_seg5[0] <= 1.0000038341502565e+06
 R2436: - C11 + 1e+06 alpha_ob15_seg5[1] <= 999972.6012002755
 R2437: - C13 + 1e+06 alpha_ob15_seg5[1] <= 999972.6012002755
 R2438: C10 + 1e+06 alpha_ob15_seg5[2] <= 999993.6158102566
 R2439: C12 + 1e+06 alpha_ob15_seg5[2] <= 999993.6158102566
 R2440: C11 + 1e+06 alpha_ob15_seg5[3] <= 1.0000142488002755e+06
 R2441: C13 + 1e+06 alpha_ob15_seg5[3] <= 1.0000142488002755e+06
 R2442: alpha_ob15_seg5[0] + alpha_ob15_seg5[1] + alpha_ob15_seg5[2]
   + alpha_ob15_seg5[3] >= 1
 R2443: - C10 + 1e+06 alpha_ob16_seg5[0] <= 1.0000162874063333e+06
 R2444: - C12 + 1e+06 alpha_ob16_seg5[0] <= 1.0000162874063333e+06
 R2445: - C11 + 1e+06 alpha_ob16_seg5[1] <= 999982.9439602756
 R2446: - C13 + 1e+06 alpha_ob16_seg5[1] <= 999982.9439602756
 R2447: C10 + 1e+06 alpha_ob16_seg5[2] <= 999981.1624063333
 R2448: C12 + 1e+06 alpha_ob16_seg5[2] <= 999981.1624063333
 R2449: C11 + 1e+06 alpha_ob16_seg5[3] <= 999963.6560402757
 R2450: C13 + 1e+06 alpha_ob16_seg5[3] <= 999963.6560402757
 R2451: alpha_ob16_seg5[0] + alpha_ob16_seg5[1] + alpha_ob16_seg5[2]
   + alpha_ob16_seg5[3] >= 1
 R2452: - C10 + 1e+06 alpha_ob17_seg5[0] <= 999980.937509
 R2453: - C12 + 1e+06 alpha_ob17_seg5[0] <= 999980.937509
 R2454: - C11 + 1e+06 alpha_ob17_seg5[1] <= 1.000034894e+06
 R2455: - C13 + 1e+06 alpha_ob17_seg5[1] <= 1.000034894e+06
 R2456: C10 + 1e+06 alpha_ob17_seg5[2] <= 999981.162491
 R2457: C12 + 1e+06 alpha_ob17_seg5[2] <= 999981.162491
 R2458: C11 + 1e+06 alpha_ob17_seg5[3] <= 999962.556
 R2459: C13 + 1e+06 alpha_ob17_seg5[3] <= 999962.556
 R2460: alpha_ob17_seg5[0] + alpha_ob17_seg5[1] + alpha_ob17_seg5[2]
   + alpha_ob17_seg5[3] >= 1
 R2461: - C10 + 1e+06 alpha_ob18_seg5[0] <= 999990.2670277582
 R2462: - C12 + 1e+06 alpha_ob18_seg5[0] <= 999990.2670277582
 R2463: - C11 + 1e+06 alpha_ob18_seg5[1] <= 1.0000347702002756e+06
 R2464: - C13 + 1e+06 alpha_ob18_seg5[1] <= 1.0000347702002756e+06
 R2465: C10 + 1e+06 alpha_ob18_seg5[2] <= 1.0000071828877581e+06
 R2466: C12 + 1e+06 alpha_ob18_seg5[2] <= 1.0000071828877581e+06
 R2467: C11 + 1e+06 alpha_ob18_seg5[3] <= 999939.8298002756
 R2468: C13 + 1e+06 alpha_ob18_seg5[3] <= 999939.8298002756
 R2469: alpha_ob18_seg5[0] + alpha_ob18_seg5[1] + alpha_ob18_seg5[2]
   + alpha_ob18_seg5[3] >= 1
 R2470: - C10 + 1e+06 alpha_ob19_seg5[0] <= 999990.267069801
 R2471: - C12 + 1e+06 alpha_ob19_seg5[0] <= 999990.267069801
 R2472: - C11 + 1e+06 alpha_ob19_seg5[1] <= 1.0000576202069657e+06
 R2473: - C13 + 1e+06 alpha_ob19_seg5[1] <= 1.0000576202069657e+06
 R2474: C10 + 1e+06 alpha_ob19_seg5[2] <= 1.000002082929801e+06
 R2475: C12 + 1e+06 alpha_ob19_seg5[2] <= 1.000002082929801e+06
 R2476: C11 + 1e+06 alpha_ob19_seg5[3] <= 999939.8298069657
 R2477: C13 + 1e+06 alpha_ob19_seg5[3] <= 999939.8298069657
 R2478: alpha_ob19_seg5[0] + alpha_ob19_seg5[1] + alpha_ob19_seg5[2]
   + alpha_ob19_seg5[3] >= 1
 R2479: - C10 + 1e+06 alpha_ob20_seg5[0] <= 999991.073732
 R2480: - C12 + 1e+06 alpha_ob20_seg5[0] <= 999991.073732
 R2481: - C11 + 1e+06 alpha_ob20_seg5[1] <= 999952.2125
 R2482: - C13 + 1e+06 alpha_ob20_seg5[1] <= 999952.2125
 R2483: C10 + 1e+06 alpha_ob20_seg5[2] <= 999990.776268
 R2484: C12 + 1e+06 alpha_ob20_seg5[2] <= 999990.776268
 R2485: C11 + 1e+06 alpha_ob20_seg5[3] <= 1.0000452375e+06
 R2486: C13 + 1e+06 alpha_ob20_seg5[3] <= 1.0000452375e+06
 R2487: alpha_ob20_seg5[0] + alpha_ob20_seg5[1] + alpha_ob20_seg5[2]
   + alpha_ob20_seg5[3] >= 1
 R2488: - C10 + 1e+06 alpha_ob21_seg5[0] <= 999985.657099801
 R2489: - C12 + 1e+06 alpha_ob21_seg5[0] <= 999985.657099801
 R2490: - C11 + 1e+06 alpha_ob21_seg5[1] <= 1.0000576202063023e+06
 R2491: - C13 + 1e+06 alpha_ob21_seg5[1] <= 1.0000576202063023e+06
 R2492: C10 + 1e+06 alpha_ob21_seg5[2] <= 1.000007192899801e+06
 R2493: C12 + 1e+06 alpha_ob21_seg5[2] <= 1.000007192899801e+06
 R2494: C11 + 1e+06 alpha_ob21_seg5[3] <= 999939.8298063023
 R2495: C13 + 1e+06 alpha_ob21_seg5[3] <= 999939.8298063023
 R2496: alpha_ob21_seg5[0] + alpha_ob21_seg5[1] + alpha_ob21_seg5[2]
   + alpha_ob21_seg5[3] >= 1
 R2497: - C10 + 1e+06 alpha_ob22_seg5[0] <= 999994.771829801
 R2498: - C12 + 1e+06 alpha_ob22_seg5[0] <= 999994.771829801
 R2499: - C11 + 1e+06 alpha_ob22_seg5[1] <= 1.0000615983063023e+06
 R2500: - C13 + 1e+06 alpha_ob22_seg5[1] <= 1.0000615983063023e+06
 R2501: C10 + 1e+06 alpha_ob22_seg5[2] <= 999998.078169801
 R2502: C12 + 1e+06 alpha_ob22_seg5[2] <= 999998.078169801
 R2503: C11 + 1e+06 alpha_ob22_seg5[3] <= 999935.8517063023
 R2504: C13 + 1e+06 alpha_ob22_seg5[3] <= 999935.8517063023
 R2505: alpha_ob22_seg5[0] + alpha_ob22_seg5[1] + alpha_ob22_seg5[2]
   + alpha_ob22_seg5[3] >= 1
 R2506: - C10 + 1e+06 alpha_ob23_seg5[0] <= 999980.9374508709
 R2507: - C12 + 1e+06 alpha_ob23_seg5[0] <= 999980.9374508709
 R2508: - C11 + 1e+06 alpha_ob23_seg5[1] <= 1.0000348940002755e+06
 R2509: - C13 + 1e+06 alpha_ob23_seg5[1] <= 1.0000348940002755e+06
 R2510: C10 + 1e+06 alpha_ob23_seg5[2] <= 1.0000165124508708e+06
 R2511: C12 + 1e+06 alpha_ob23_seg5[2] <= 1.0000165124508708e+06
 R2512: C11 + 1e+06 alpha_ob23_seg5[3] <= 999935.9560002756
 R2513: C13 + 1e+06 alpha_ob23_seg5[3] <= 999935.9560002756
 R2514: alpha_ob23_seg5[0] + alpha_ob23_seg5[1] + alpha_ob23_seg5[2]
   + alpha_ob23_seg5[3] >= 1
 R2515: - C10 + 1e+06 alpha_ob24_seg5[0] <= 999980.937499801
 R2516: - C12 + 1e+06 alpha_ob24_seg5[0] <= 999980.937499801
 R2517: - C11 + 1e+06 alpha_ob24_seg5[1] <= 1.0000614940072973e+06
 R2518: - C13 + 1e+06 alpha_ob24_seg5[1] <= 1.0000614940072973e+06
 R2519: C10 + 1e+06 alpha_ob24_seg5[2] <= 1.000011162499801e+06
 R2520: C12 + 1e+06 alpha_ob24_seg5[2] <= 1.000011162499801e+06
 R2521: C11 + 1e+06 alpha_ob24_seg5[3] <= 999935.9560072974
 R2522: C13 + 1e+06 alpha_ob24_seg5[3] <= 999935.9560072974
 R2523: alpha_ob24_seg5[0] + alpha_ob24_seg5[1] + alpha_ob24_seg5[2]
   + alpha_ob24_seg5[3] >= 1
 R2524: - C10 + 1e+06 alpha_ob25_seg5[0] <= 999956.8394000001
 R2525: - C12 + 1e+06 alpha_ob25_seg5[0] <= 999956.8394000001
 R2526: - C11 + 1e+06 alpha_ob25_seg5[1] <= 999952.2463
 R2527: - C13 + 1e+06 alpha_ob25_seg5[1] <= 999952.2463
 R2528: C10 + 1e+06 alpha_ob25_seg5[2] <= 1.0000097606e+06
 R2529: C12 + 1e+06 alpha_ob25_seg5[2] <= 1.0000097606e+06
 R2530: C11 + 1e+06 alpha_ob25_seg5[3] <= 1.0000452037e+06
 R2531: C13 + 1e+06 alpha_ob25_seg5[3] <= 1.0000452037e+06
 R2532: alpha_ob25_seg5[0] + alpha_ob25_seg5[1] + alpha_ob25_seg5[2]
   + alpha_ob25_seg5[3] >= 1
 R2533: - C10 + 1e+06 alpha_ob26_seg5[0] <= 999980.9375348955
 R2534: - C12 + 1e+06 alpha_ob26_seg5[0] <= 999980.9375348955
 R2535: - C11 + 1e+06 alpha_ob26_seg5[1] <= 1.0000160439997246e+06
 R2536: - C13 + 1e+06 alpha_ob26_seg5[1] <= 1.0000160439997246e+06
 R2537: C10 + 1e+06 alpha_ob26_seg5[2] <= 1.0000165125348954e+06
 R2538: C12 + 1e+06 alpha_ob26_seg5[2] <= 1.0000165125348954e+06
 R2539: C11 + 1e+06 alpha_ob26_seg5[3] <= 999962.5559997245
 R2540: C13 + 1e+06 alpha_ob26_seg5[3] <= 999962.5559997245
 R2541: alpha_ob26_seg5[0] + alpha_ob26_seg5[1] + alpha_ob26_seg5[2]
   + alpha_ob26_seg5[3] >= 1
 R2542: - C10 + 1e+06 alpha_ob27_seg5[0] <= 999956.839399801
 R2543: - C12 + 1e+06 alpha_ob27_seg5[0] <= 999956.839399801
 R2544: - C11 + 1e+06 alpha_ob27_seg5[1] <= 999925.3963434525
 R2545: - C13 + 1e+06 alpha_ob27_seg5[1] <= 999925.3963434525
 R2546: C10 + 1e+06 alpha_ob27_seg5[2] <= 1.0000080105998011e+06
 R2547: C12 + 1e+06 alpha_ob27_seg5[2] <= 1.0000080105998011e+06
 R2548: C11 + 1e+06 alpha_ob27_seg5[3] <= 1.0000720537434525e+06
 R2549: C13 + 1e+06 alpha_ob27_seg5[3] <= 1.0000720537434525e+06
 R2550: alpha_ob27_seg5[0] + alpha_ob27_seg5[1] + alpha_ob27_seg5[2]
   + alpha_ob27_seg5[3] >= 1
 R2551: - C10 + 1e+06 alpha_ob28_seg5[0] <= 999980.9374998009
 R2552: - C12 + 1e+06 alpha_ob28_seg5[0] <= 999980.9374998009
 R2553: - C11 + 1e+06 alpha_ob28_seg5[1] <= 1.0000160440053071e+06
 R2554: - C13 + 1e+06 alpha_ob28_seg5[1] <= 1.0000160440053071e+06
 R2555: C10 + 1e+06 alpha_ob28_seg5[2] <= 1.000012662499801e+06
 R2556: C12 + 1e+06 alpha_ob28_seg5[2] <= 1.000012662499801e+06
 R2557: C11 + 1e+06 alpha_ob28_seg5[3] <= 999981.4060053072
 R2558: C13 + 1e+06 alpha_ob28_seg5[3] <= 999981.4060053072
 R2559: alpha_ob28_seg5[0] + alpha_ob28_seg5[1] + alpha_ob28_seg5[2]
   + alpha_ob28_seg5[3] >= 1
 R2560: - C10 + 1e+06 alpha_ob29_seg5[0] <= 999983.6053459151
 R2561: - C12 + 1e+06 alpha_ob29_seg5[0] <= 999983.6053459151
 R2562: - C11 + 1e+06 alpha_ob29_seg5[1] <= 999987.7722497246
 R2563: - C13 + 1e+06 alpha_ob29_seg5[1] <= 999987.7722497246
 R2564: C10 + 1e+06 alpha_ob29_seg5[2] <= 1.000013844745915e+06
 R2565: C12 + 1e+06 alpha_ob29_seg5[2] <= 1.000013844745915e+06
 R2566: C11 + 1e+06 alpha_ob29_seg5[3] <= 999984.8277497246
 R2567: C13 + 1e+06 alpha_ob29_seg5[3] <= 999984.8277497246
 R2568: alpha_ob29_seg5[0] + alpha_ob29_seg5[1] + alpha_ob29_seg5[2]
   + alpha_ob29_seg5[3] >= 1
 R2569: - C10 + 1e+06 alpha_ob30_seg5[0] <= 999958.8717898987
 R2570: - C12 + 1e+06 alpha_ob30_seg5[0] <= 999958.8717898987
 R2571: - C11 + 1e+06 alpha_ob30_seg5[1] <= 999987.7837602755
 R2572: - C13 + 1e+06 alpha_ob30_seg5[1] <= 999987.7837602755
 R2573: C10 + 1e+06 alpha_ob30_seg5[2] <= 1.0000385781898986e+06
 R2574: C12 + 1e+06 alpha_ob30_seg5[2] <= 1.0000385781898986e+06
 R2575: C11 + 1e+06 alpha_ob30_seg5[3] <= 1.0000043162402755e+06
 R2576: C13 + 1e+06 alpha_ob30_seg5[3] <= 1.0000043162402755e+06
 R2577: alpha_ob30_seg5[0] + alpha_ob30_seg5[1] + alpha_ob30_seg5[2]
   + alpha_ob30_seg5[3] >= 1
 R2578: - C10 + 1e+06 alpha_ob31_seg5[0] <= 999939.2718
 R2579: - C12 + 1e+06 alpha_ob31_seg5[0] <= 999939.2718
 R2580: - C11 + 1e+06 alpha_ob31_seg5[1] <= 999993.13376
 R2581: - C13 + 1e+06 alpha_ob31_seg5[1] <= 999993.13376
 R2582: C10 + 1e+06 alpha_ob31_seg5[2] <= 1.0000385782e+06
 R2583: C12 + 1e+06 alpha_ob31_seg5[2] <= 1.0000385782e+06
 R2584: C11 + 1e+06 alpha_ob31_seg5[3] <= 1.00000431624e+06
 R2585: C13 + 1e+06 alpha_ob31_seg5[3] <= 1.00000431624e+06
 R2586: alpha_ob31_seg5[0] + alpha_ob31_seg5[1] + alpha_ob31_seg5[2]
   + alpha_ob31_seg5[3] >= 1
 R2587: - C10 + 1e+06 alpha_ob32_seg5[0] <= 999939.2718101013
 R2588: - C12 + 1e+06 alpha_ob32_seg5[0] <= 999939.2718101013
 R2589: - C11 + 1e+06 alpha_ob32_seg5[1] <= 999987.7837597246
 R2590: - C13 + 1e+06 alpha_ob32_seg5[1] <= 999987.7837597246
 R2591: C10 + 1e+06 alpha_ob32_seg5[2] <= 1.0000581782101013e+06
 R2592: C12 + 1e+06 alpha_ob32_seg5[2] <= 1.0000581782101013e+06
 R2593: C11 + 1e+06 alpha_ob32_seg5[3] <= 1.0000043162397245e+06
 R2594: C13 + 1e+06 alpha_ob32_seg5[3] <= 1.0000043162397245e+06
 R2595: alpha_ob32_seg5[0] + alpha_ob32_seg5[1] + alpha_ob32_seg5[2]
   + alpha_ob32_seg5[3] >= 1
 R2596: - C10 + 1e+06 alpha_ob33_seg5[0] <= 999939.271799801
 R2597: - C12 + 1e+06 alpha_ob33_seg5[0] <= 999939.271799801
 R2598: - C11 + 1e+06 alpha_ob33_seg5[1] <= 999987.7838262042
 R2599: - C13 + 1e+06 alpha_ob33_seg5[1] <= 999987.7838262042
 R2600: C10 + 1e+06 alpha_ob33_seg5[2] <= 1.000038578199801e+06
 R2601: C12 + 1e+06 alpha_ob33_seg5[2] <= 1.000038578199801e+06
 R2602: C11 + 1e+06 alpha_ob33_seg5[3] <= 1.0000096662262043e+06
 R2603: C13 + 1e+06 alpha_ob33_seg5[3] <= 1.0000096662262043e+06
 R2604: alpha_ob33_seg5[0] + alpha_ob33_seg5[1] + alpha_ob33_seg5[2]
   + alpha_ob33_seg5[3] >= 1
 R2605: - C10 + 1e+06 alpha_ob34_seg5[0] <= 999926.0708
 R2606: - C12 + 1e+06 alpha_ob34_seg5[0] <= 999926.0708
 R2607: - C11 + 1e+06 alpha_ob34_seg5[1] <= 999983.7196
 R2608: - C13 + 1e+06 alpha_ob34_seg5[1] <= 999983.7196
 R2609: C10 + 1e+06 alpha_ob34_seg5[2] <= 1.0000250292e+06
 R2610: C12 + 1e+06 alpha_ob34_seg5[2] <= 1.0000250292e+06
 R2611: C11 + 1e+06 alpha_ob34_seg5[3] <= 1.0000137304e+06
 R2612: C13 + 1e+06 alpha_ob34_seg5[3] <= 1.0000137304e+06
 R2613: alpha_ob34_seg5[0] + alpha_ob34_seg5[1] + alpha_ob34_seg5[2]
   + alpha_ob34_seg5[3] >= 1
 R2614: - C10 + 1e+06 alpha_ob35_seg5[0] <= 999926.0708293857
 R2615: - C12 + 1e+06 alpha_ob35_seg5[0] <= 999926.0708293857
 R2616: - C11 + 1e+06 alpha_ob35_seg5[1] <= 999967.8695997245
 R2617: - C13 + 1e+06 alpha_ob35_seg5[1] <= 999967.8695997245
 R2618: C10 + 1e+06 alpha_ob35_seg5[2] <= 1.0000713792293856e+06
 R2619: C12 + 1e+06 alpha_ob35_seg5[2] <= 1.0000713792293856e+06
 R2620: C11 + 1e+06 alpha_ob35_seg5[3] <= 1.0000137303997246e+06
 R2621: C13 + 1e+06 alpha_ob35_seg5[3] <= 1.0000137303997246e+06
 R2622: alpha_ob35_seg5[0] + alpha_ob35_seg5[1] + alpha_ob35_seg5[2]
   + alpha_ob35_seg5[3] >= 1
 R2623: - C10 + 1e+06 alpha_ob36_seg5[0] <= 999972.6917508709
 R2624: - C12 + 1e+06 alpha_ob36_seg5[0] <= 999972.6917508709
 R2625: - C11 + 1e+06 alpha_ob36_seg5[1] <= 999957.1644002756
 R2626: - C13 + 1e+06 alpha_ob36_seg5[1] <= 999957.1644002756
 R2627: C10 + 1e+06 alpha_ob36_seg5[2] <= 1.0000247581508709e+06
 R2628: C12 + 1e+06 alpha_ob36_seg5[2] <= 1.0000247581508709e+06
 R2629: C11 + 1e+06 alpha_ob36_seg5[3] <= 1.0000136856002755e+06
 R2630: C13 + 1e+06 alpha_ob36_seg5[3] <= 1.0000136856002755e+06
 R2631: alpha_ob36_seg5[0] + alpha_ob36_seg5[1] + alpha_ob36_seg5[2]
   + alpha_ob36_seg5[3] >= 1
 R2632: - C10 + 1e+06 alpha_ob37_seg5[0] <= 999939.0009674003
 R2633: - C12 + 1e+06 alpha_ob37_seg5[0] <= 999939.0009674003
 R2634: - C11 + 1e+06 alpha_ob37_seg5[1] <= 999961.9516002756
 R2635: - C13 + 1e+06 alpha_ob37_seg5[1] <= 999961.9516002756
 R2636: C10 + 1e+06 alpha_ob37_seg5[2] <= 1.0000584489674004e+06
 R2637: C12 + 1e+06 alpha_ob37_seg5[2] <= 1.0000584489674004e+06
 R2638: C11 + 1e+06 alpha_ob37_seg5[3] <= 1.0000178984002755e+06
 R2639: C13 + 1e+06 alpha_ob37_seg5[3] <= 1.0000178984002755e+06
 R2640: alpha_ob37_seg5[0] + alpha_ob37_seg5[1] + alpha_ob37_seg5[2]
   + alpha_ob37_seg5[3] >= 1
 R2641: - C10 + 1e+06 alpha_ob38_seg5[0] <= 999994.9164998011
 R2642: - C12 + 1e+06 alpha_ob38_seg5[0] <= 999994.9164998011
 R2643: - C11 + 1e+06 alpha_ob38_seg5[1] <= 999925.4284782809
 R2644: - C13 + 1e+06 alpha_ob38_seg5[1] <= 999925.4284782809
 R2645: C10 + 1e+06 alpha_ob38_seg5[2] <= 999943.6834998011
 R2646: C12 + 1e+06 alpha_ob38_seg5[2] <= 999943.6834998011
 R2647: C11 + 1e+06 alpha_ob38_seg5[3] <= 1.0000720216782809e+06
 R2648: C13 + 1e+06 alpha_ob38_seg5[3] <= 1.0000720216782809e+06
 R2649: alpha_ob38_seg5[0] + alpha_ob38_seg5[1] + alpha_ob38_seg5[2]
   + alpha_ob38_seg5[3] >= 1
 R2650: - C10 + 1e+06 alpha_ob39_seg5[0] <= 999914.52327842
 R2651: - C12 + 1e+06 alpha_ob39_seg5[0] <= 999914.52327842
 R2652: - C11 + 1e+06 alpha_ob39_seg5[1] <= 999993.6552362755
 R2653: - C13 + 1e+06 alpha_ob39_seg5[1] <= 999993.6552362755
 R2654: C10 + 1e+06 alpha_ob39_seg5[2] <= 1.0000829266784199e+06
 R2655: C12 + 1e+06 alpha_ob39_seg5[2] <= 1.0000829266784199e+06
 R2656: C11 + 1e+06 alpha_ob39_seg5[3] <= 999992.1947642756
 R2657: C13 + 1e+06 alpha_ob39_seg5[3] <= 999992.1947642756
 R2658: alpha_ob39_seg5[0] + alpha_ob39_seg5[1] + alpha_ob39_seg5[2]
   + alpha_ob39_seg5[3] >= 1
 R2659: - C10 + 1e+06 alpha_ob40_seg5[0] <= 999912.265199801
 R2660: - C12 + 1e+06 alpha_ob40_seg5[0] <= 999912.265199801
 R2661: - C11 + 1e+06 alpha_ob40_seg5[1] <= 999987.7842112777
 R2662: - C13 + 1e+06 alpha_ob40_seg5[1] <= 999987.7842112777
 R2663: C10 + 1e+06 alpha_ob40_seg5[2] <= 1.000076834799801e+06
 R2664: C12 + 1e+06 alpha_ob40_seg5[2] <= 1.000076834799801e+06
 R2665: C11 + 1e+06 alpha_ob40_seg5[3] <= 1.0000096658112778e+06
 R2666: C13 + 1e+06 alpha_ob40_seg5[3] <= 1.0000096658112778e+06
 R2667: alpha_ob40_seg5[0] + alpha_ob40_seg5[1] + alpha_ob40_seg5[2]
   + alpha_ob40_seg5[3] >= 1
 R2668: - C10 + 1e+06 alpha_ob41_seg5[0] <= 999920.6152119379
 R2669: - C12 + 1e+06 alpha_ob41_seg5[0] <= 999920.6152119379
 R2670: - C11 + 1e+06 alpha_ob41_seg5[1] <= 999981.4341997246
 R2671: - C13 + 1e+06 alpha_ob41_seg5[1] <= 999981.4341997246
 R2672: C10 + 1e+06 alpha_ob41_seg5[2] <= 1.0000768348119379e+06
 R2673: C12 + 1e+06 alpha_ob41_seg5[2] <= 1.0000768348119379e+06
 R2674: C11 + 1e+06 alpha_ob41_seg5[3] <= 1.0000096657997245e+06
 R2675: C13 + 1e+06 alpha_ob41_seg5[3] <= 1.0000096657997245e+06
 R2676: alpha_ob41_seg5[0] + alpha_ob41_seg5[1] + alpha_ob41_seg5[2]
   + alpha_ob41_seg5[3] >= 1
 R2677: - C10 + 1e+06 alpha_ob42_seg5[0] <= 999919.2652
 R2678: - C12 + 1e+06 alpha_ob42_seg5[0] <= 999919.2652
 R2679: - C11 + 1e+06 alpha_ob42_seg5[1] <= 999981.4342
 R2680: - C13 + 1e+06 alpha_ob42_seg5[1] <= 999981.4342
 R2681: C10 + 1e+06 alpha_ob42_seg5[2] <= 1.0000768348e+06
 R2682: C12 + 1e+06 alpha_ob42_seg5[2] <= 1.0000768348e+06
 R2683: C11 + 1e+06 alpha_ob42_seg5[3] <= 1.0000160158000001e+06
 R2684: C13 + 1e+06 alpha_ob42_seg5[3] <= 1.0000160158000001e+06
 R2685: alpha_ob42_seg5[0] + alpha_ob42_seg5[1] + alpha_ob42_seg5[2]
   + alpha_ob42_seg5[3] >= 1
 R2686: - C10 + 1e+06 alpha_ob43_seg5[0] <= 999919.2652454559
 R2687: - C12 + 1e+06 alpha_ob43_seg5[0] <= 999919.2652454559
 R2688: - C11 + 1e+06 alpha_ob43_seg5[1] <= 999956.8341997246
 R2689: - C13 + 1e+06 alpha_ob43_seg5[1] <= 999956.8341997246
 R2690: C10 + 1e+06 alpha_ob43_seg5[2] <= 1.0000781848454559e+06
 R2691: C12 + 1e+06 alpha_ob43_seg5[2] <= 1.0000781848454559e+06
 R2692: C11 + 1e+06 alpha_ob43_seg5[3] <= 1.0000160157997246e+06
 R2693: C13 + 1e+06 alpha_ob43_seg5[3] <= 1.0000160157997246e+06
 R2694: alpha_ob43_seg5[0] + alpha_ob43_seg5[1] + alpha_ob43_seg5[2]
   + alpha_ob43_seg5[3] >= 1
 R2695: - C10 + 1e+06 alpha_ob44_seg5[0] <= 999991.83683
 R2696: - C12 + 1e+06 alpha_ob44_seg5[0] <= 999991.83683
 R2697: - C11 + 1e+06 alpha_ob44_seg5[1] <= 999940.589
 R2698: - C13 + 1e+06 alpha_ob44_seg5[1] <= 999940.589
 R2699: C10 + 1e+06 alpha_ob44_seg5[2] <= 1.00000401317e+06
 R2700: C12 + 1e+06 alpha_ob44_seg5[2] <= 1.00000401317e+06
 R2701: C11 + 1e+06 alpha_ob44_seg5[3] <= 1.000056861e+06
 R2702: C13 + 1e+06 alpha_ob44_seg5[3] <= 1.000056861e+06
 R2703: alpha_ob44_seg5[0] + alpha_ob44_seg5[1] + alpha_ob44_seg5[2]
   + alpha_ob44_seg5[3] >= 1
 R2704: - C10 + 1e+06 alpha_ob45_seg5[0] <= 999991.8368084199
 R2705: - C12 + 1e+06 alpha_ob45_seg5[0] <= 999991.8368084199
 R2706: - C11 + 1e+06 alpha_ob45_seg5[1] <= 999940.5890002755
 R2707: - C13 + 1e+06 alpha_ob45_seg5[1] <= 999940.5890002755
 R2708: C10 + 1e+06 alpha_ob45_seg5[2] <= 1.0000056131484199e+06
 R2709: C12 + 1e+06 alpha_ob45_seg5[2] <= 1.0000056131484199e+06
 R2710: C11 + 1e+06 alpha_ob45_seg5[3] <= 1.0000452610002755e+06
 R2711: C13 + 1e+06 alpha_ob45_seg5[3] <= 1.0000452610002755e+06
 R2712: alpha_ob45_seg5[0] + alpha_ob45_seg5[1] + alpha_ob45_seg5[2]
   + alpha_ob45_seg5[3] >= 1
 R2713: - C10 + 1e+06 alpha_ob46_seg5[0] <= 1.0000764021508709e+06
 R2714: - C12 + 1e+06 alpha_ob46_seg5[0] <= 1.0000764021508709e+06
 R2715: - C11 + 1e+06 alpha_ob46_seg5[1] <= 999957.1027002756
 R2716: - C13 + 1e+06 alpha_ob46_seg5[1] <= 999957.1027002756
 R2717: C10 + 1e+06 alpha_ob46_seg5[2] <= 999921.0477508709
 R2718: C12 + 1e+06 alpha_ob46_seg5[2] <= 999921.0477508709
 R2719: C11 + 1e+06 alpha_ob46_seg5[3] <= 1.0000137473002756e+06
 R2720: C13 + 1e+06 alpha_ob46_seg5[3] <= 1.0000137473002756e+06
 R2721: alpha_ob46_seg5[0] + alpha_ob46_seg5[1] + alpha_ob46_seg5[2]
   + alpha_ob46_seg5[3] >= 1
 R2722: - C10 + 1e+06 alpha_ob47_seg5[0] <= 1.0000445522e+06
 R2723: - C12 + 1e+06 alpha_ob47_seg5[0] <= 1.0000445522e+06
 R2724: - C11 + 1e+06 alpha_ob47_seg5[1] <= 999983.7027
 R2725: - C13 + 1e+06 alpha_ob47_seg5[1] <= 999983.7027
 R2726: C10 + 1e+06 alpha_ob47_seg5[2] <= 999921.0477999999
 R2727: C12 + 1e+06 alpha_ob47_seg5[2] <= 999921.0477999999
 R2728: C11 + 1e+06 alpha_ob47_seg5[3] <= 1.0000137473e+06
 R2729: C13 + 1e+06 alpha_ob47_seg5[3] <= 1.0000137473e+06
 R2730: alpha_ob47_seg5[0] + alpha_ob47_seg5[1] + alpha_ob47_seg5[2]
   + alpha_ob47_seg5[3] >= 1
 R2731: - C10 + 1e+06 alpha_ob48_seg5[0] <= 1.0000445522183661e+06
 R2732: - C12 + 1e+06 alpha_ob48_seg5[0] <= 1.0000445522183661e+06
 R2733: - C11 + 1e+06 alpha_ob48_seg5[1] <= 999973.8526997245
 R2734: - C13 + 1e+06 alpha_ob48_seg5[1] <= 999973.8526997245
 R2735: C10 + 1e+06 alpha_ob48_seg5[2] <= 999952.8978183661
 R2736: C12 + 1e+06 alpha_ob48_seg5[2] <= 999952.8978183661
 R2737: C11 + 1e+06 alpha_ob48_seg5[3] <= 1.0000137472997245e+06
 R2738: C13 + 1e+06 alpha_ob48_seg5[3] <= 1.0000137472997245e+06
 R2739: alpha_ob48_seg5[0] + alpha_ob48_seg5[1] + alpha_ob48_seg5[2]
   + alpha_ob48_seg5[3] >= 1
 R2740: - C10 + 1e+06 alpha_ob49_seg5[0] <= 1.000064332081634e+06
 R2741: - C12 + 1e+06 alpha_ob49_seg5[0] <= 1.000064332081634e+06
 R2742: - C11 + 1e+06 alpha_ob49_seg5[1] <= 999973.9401002756
 R2743: - C13 + 1e+06 alpha_ob49_seg5[1] <= 999973.9401002756
 R2744: C10 + 1e+06 alpha_ob49_seg5[2] <= 999933.1178816339
 R2745: C12 + 1e+06 alpha_ob49_seg5[2] <= 999933.1178816339
 R2746: C11 + 1e+06 alpha_ob49_seg5[3] <= 1.0000136599002755e+06
 R2747: C13 + 1e+06 alpha_ob49_seg5[3] <= 1.0000136599002755e+06
 R2748: alpha_ob49_seg5[0] + alpha_ob49_seg5[1] + alpha_ob49_seg5[2]
   + alpha_ob49_seg5[3] >= 1
 R2749: - C10 + 1e+06 alpha_ob50_seg5[0] <= 1.0000643321013774e+06
 R2750: - C12 + 1e+06 alpha_ob50_seg5[0] <= 1.0000643321013774e+06
 R2751: - C11 + 1e+06 alpha_ob50_seg5[1] <= 999983.1900997246
 R2752: - C13 + 1e+06 alpha_ob50_seg5[1] <= 999983.1900997246
 R2753: C10 + 1e+06 alpha_ob50_seg5[2] <= 999933.1179013775
 R2754: C12 + 1e+06 alpha_ob50_seg5[2] <= 999933.1179013775
 R2755: C11 + 1e+06 alpha_ob50_seg5[3] <= 1.0000136598997245e+06
 R2756: C13 + 1e+06 alpha_ob50_seg5[3] <= 1.0000136598997245e+06
 R2757: alpha_ob50_seg5[0] + alpha_ob50_seg5[1] + alpha_ob50_seg5[2]
   + alpha_ob50_seg5[3] >= 1
 R2758: - C12 + 1e+06 alpha_ob0_seg6[0] <= 1.0000284170499525e+06
 R2759: - C14 + 1e+06 alpha_ob0_seg6[0] <= 1.0000284170499525e+06
 R2760: - C13 + 1e+06 alpha_ob0_seg6[1] <= 999957.0843002755
 R2761: - C15 + 1e+06 alpha_ob0_seg6[1] <= 999957.0843002755
 R2762: C12 + 1e+06 alpha_ob0_seg6[2] <= 999969.0328499526
 R2763: C14 + 1e+06 alpha_ob0_seg6[2] <= 999969.0328499526
 R2764: C13 + 1e+06 alpha_ob0_seg6[3] <= 1.0000132657002755e+06
 R2765: C15 + 1e+06 alpha_ob0_seg6[3] <= 1.0000132657002755e+06
 R2766: alpha_ob0_seg6[0] + alpha_ob0_seg6[1] + alpha_ob0_seg6[2]
   + alpha_ob0_seg6[3] >= 1
 R2767: - C12 + 1e+06 alpha_ob1_seg6[0] <= 1.000028417099801e+06
 R2768: - C14 + 1e+06 alpha_ob1_seg6[0] <= 1.000028417099801e+06
 R2769: - C13 + 1e+06 alpha_ob1_seg6[1] <= 999984.1843155898
 R2770: - C15 + 1e+06 alpha_ob1_seg6[1] <= 999984.1843155898
 R2771: C12 + 1e+06 alpha_ob1_seg6[2] <= 999957.4328998009
 R2772: C14 + 1e+06 alpha_ob1_seg6[2] <= 999957.4328998009
 R2773: C13 + 1e+06 alpha_ob1_seg6[3] <= 1.0000132657155899e+06
 R2774: C15 + 1e+06 alpha_ob1_seg6[3] <= 1.0000132657155899e+06
 R2775: alpha_ob1_seg6[0] + alpha_ob1_seg6[1] + alpha_ob1_seg6[2]
   + alpha_ob1_seg6[3] >= 1
 R2776: - C12 + 1e+06 alpha_ob2_seg6[0] <= 1.0000400171500475e+06
 R2777: - C14 + 1e+06 alpha_ob2_seg6[0] <= 1.0000400171500475e+06
 R2778: - C13 + 1e+06 alpha_ob2_seg6[1] <= 999957.0842997245
 R2779: - C15 + 1e+06 alpha_ob2_seg6[1] <= 999957.0842997245
 R2780: C12 + 1e+06 alpha_ob2_seg6[2] <= 999957.4329500474
 R2781: C14 + 1e+06 alpha_ob2_seg6[2] <= 999957.4329500474
 R2782: C13 + 1e+06 alpha_ob2_seg6[3] <= 1.0000132656997246e+06
 R2783: C15 + 1e+06 alpha_ob2_seg6[3] <= 1.0000132656997246e+06
 R2784: alpha_ob2_seg6[0] + alpha_ob2_seg6[1] + alpha_ob2_seg6[2]
   + alpha_ob2_seg6[3] >= 1
 R2785: - C12 + 1e+06 alpha_ob3_seg6[0] <= 1.0000109161752058e+06
 R2786: - C14 + 1e+06 alpha_ob3_seg6[0] <= 1.0000109161752058e+06
 R2787: - C13 + 1e+06 alpha_ob3_seg6[1] <= 999970.2681002755
 R2788: - C15 + 1e+06 alpha_ob3_seg6[1] <= 999970.2681002755
 R2789: C12 + 1e+06 alpha_ob3_seg6[2] <= 999986.5337752058
 R2790: C14 + 1e+06 alpha_ob3_seg6[2] <= 999986.5337752058
 R2791: C13 + 1e+06 alpha_ob3_seg6[3] <= 1.0000138319002755e+06
 R2792: C15 + 1e+06 alpha_ob3_seg6[3] <= 1.0000138319002755e+06
 R2793: alpha_ob3_seg6[0] + alpha_ob3_seg6[1] + alpha_ob3_seg6[2]
   + alpha_ob3_seg6[3] >= 1
 R2794: - C12 + 1e+06 alpha_ob4_seg6[0] <= 1.000010916199801e+06
 R2795: - C14 + 1e+06 alpha_ob4_seg6[0] <= 1.000010916199801e+06
 R2796: - C13 + 1e+06 alpha_ob4_seg6[1] <= 999983.6181179117
 R2797: - C15 + 1e+06 alpha_ob4_seg6[1] <= 999983.6181179117
 R2798: C12 + 1e+06 alpha_ob4_seg6[2] <= 999973.183799801
 R2799: C14 + 1e+06 alpha_ob4_seg6[2] <= 999973.183799801
 R2800: C13 + 1e+06 alpha_ob4_seg6[3] <= 1.0000138319179118e+06
 R2801: C15 + 1e+06 alpha_ob4_seg6[3] <= 1.0000138319179118e+06
 R2802: alpha_ob4_seg6[0] + alpha_ob4_seg6[1] + alpha_ob4_seg6[2]
   + alpha_ob4_seg6[3] >= 1
 R2803: - C12 + 1e+06 alpha_ob5_seg6[0] <= 999992.49102
 R2804: - C14 + 1e+06 alpha_ob5_seg6[0] <= 999992.49102
 R2805: - C13 + 1e+06 alpha_ob5_seg6[1] <= 999957.1016000001
 R2806: - C15 + 1e+06 alpha_ob5_seg6[1] <= 999957.1016000001
 R2807: C12 + 1e+06 alpha_ob5_seg6[2] <= 999986.35898
 R2808: C14 + 1e+06 alpha_ob5_seg6[2] <= 999986.35898
 R2809: C13 + 1e+06 alpha_ob5_seg6[3] <= 1.0000403484e+06
 R2810: C15 + 1e+06 alpha_ob5_seg6[3] <= 1.0000403484e+06
 R2811: alpha_ob5_seg6[0] + alpha_ob5_seg6[1] + alpha_ob5_seg6[2]
   + alpha_ob5_seg6[3] >= 1
 R2812: - C12 + 1e+06 alpha_ob6_seg6[0] <= 999981.3909999999
 R2813: - C14 + 1e+06 alpha_ob6_seg6[0] <= 999981.3909999999
 R2814: - C13 + 1e+06 alpha_ob6_seg6[1] <= 999957.1016000001
 R2815: - C15 + 1e+06 alpha_ob6_seg6[1] <= 999957.1016000001
 R2816: C12 + 1e+06 alpha_ob6_seg6[2] <= 1.000004959e+06
 R2817: C14 + 1e+06 alpha_ob6_seg6[2] <= 1.000004959e+06
 R2818: C13 + 1e+06 alpha_ob6_seg6[3] <= 1.0000403484e+06
 R2819: C15 + 1e+06 alpha_ob6_seg6[3] <= 1.0000403484e+06
 R2820: alpha_ob6_seg6[0] + alpha_ob6_seg6[1] + alpha_ob6_seg6[2]
   + alpha_ob6_seg6[3] >= 1
 R2821: - C12 + 1e+06 alpha_ob7_seg6[0] <= 999981.3909825523
 R2822: - C14 + 1e+06 alpha_ob7_seg6[0] <= 999981.3909825523
 R2823: - C13 + 1e+06 alpha_ob7_seg6[1] <= 999957.1016002755
 R2824: - C15 + 1e+06 alpha_ob7_seg6[1] <= 999957.1016002755
 R2825: C12 + 1e+06 alpha_ob7_seg6[2] <= 1.0000160589825523e+06
 R2826: C14 + 1e+06 alpha_ob7_seg6[2] <= 1.0000160589825523e+06
 R2827: C13 + 1e+06 alpha_ob7_seg6[3] <= 1.0000309984002755e+06
 R2828: C15 + 1e+06 alpha_ob7_seg6[3] <= 1.0000309984002755e+06
 R2829: alpha_ob7_seg6[0] + alpha_ob7_seg6[1] + alpha_ob7_seg6[2]
   + alpha_ob7_seg6[3] >= 1
 R2830: - C12 + 1e+06 alpha_ob8_seg6[0] <= 999981.3867954084
 R2831: - C14 + 1e+06 alpha_ob8_seg6[0] <= 999981.3867954084
 R2832: - C13 + 1e+06 alpha_ob8_seg6[1] <= 999968.8595002755
 R2833: - C15 + 1e+06 alpha_ob8_seg6[1] <= 999968.8595002755
 R2834: C12 + 1e+06 alpha_ob8_seg6[2] <= 1.0000160631954084e+06
 R2835: C14 + 1e+06 alpha_ob8_seg6[2] <= 1.0000160631954084e+06
 R2836: C13 + 1e+06 alpha_ob8_seg6[3] <= 1.0000262405002755e+06
 R2837: C15 + 1e+06 alpha_ob8_seg6[3] <= 1.0000262405002755e+06
 R2838: alpha_ob8_seg6[0] + alpha_ob8_seg6[1] + alpha_ob8_seg6[2]
   + alpha_ob8_seg6[3] >= 1
 R2839: - C12 + 1e+06 alpha_ob9_seg6[0] <= 999981.386799801
 R2840: - C14 + 1e+06 alpha_ob9_seg6[0] <= 999981.386799801
 R2841: - C13 + 1e+06 alpha_ob9_seg6[1] <= 999971.2095013268
 R2842: - C15 + 1e+06 alpha_ob9_seg6[1] <= 999971.2095013268
 R2843: C12 + 1e+06 alpha_ob9_seg6[2] <= 1.0000152131998009e+06
 R2844: C14 + 1e+06 alpha_ob9_seg6[2] <= 1.0000152131998009e+06
 R2845: C13 + 1e+06 alpha_ob9_seg6[3] <= 1.0000262405013267e+06
 R2846: C15 + 1e+06 alpha_ob9_seg6[3] <= 1.0000262405013267e+06
 R2847: alpha_ob9_seg6[0] + alpha_ob9_seg6[1] + alpha_ob9_seg6[2]
   + alpha_ob9_seg6[3] >= 1
 R2848: - C12 + 1e+06 alpha_ob10_seg6[0] <= 999982.2368022958
 R2849: - C14 + 1e+06 alpha_ob10_seg6[0] <= 999982.2368022958
 R2850: - C13 + 1e+06 alpha_ob10_seg6[1] <= 999970.1094997245
 R2851: - C15 + 1e+06 alpha_ob10_seg6[1] <= 999970.1094997245
 R2852: C12 + 1e+06 alpha_ob10_seg6[2] <= 1.0000152132022957e+06
 R2853: C14 + 1e+06 alpha_ob10_seg6[2] <= 1.0000152132022957e+06
 R2854: C13 + 1e+06 alpha_ob10_seg6[3] <= 1.0000262404997245e+06
 R2855: C15 + 1e+06 alpha_ob10_seg6[3] <= 1.0000262404997245e+06
 R2856: alpha_ob10_seg6[0] + alpha_ob10_seg6[1] + alpha_ob10_seg6[2]
   + alpha_ob10_seg6[3] >= 1
 R2857: - C12 + 1e+06 alpha_ob11_seg6[0] <= 999982.236799801
 R2858: - C14 + 1e+06 alpha_ob11_seg6[0] <= 999982.236799801
 R2859: - C13 + 1e+06 alpha_ob11_seg6[1] <= 999970.1095291895
 R2860: - C15 + 1e+06 alpha_ob11_seg6[1] <= 999970.1095291895
 R2861: C12 + 1e+06 alpha_ob11_seg6[2] <= 999993.3631998011
 R2862: C14 + 1e+06 alpha_ob11_seg6[2] <= 999993.3631998011
 R2863: C13 + 1e+06 alpha_ob11_seg6[3] <= 1.0000273405291894e+06
 R2864: C15 + 1e+06 alpha_ob11_seg6[3] <= 1.0000273405291894e+06
 R2865: alpha_ob11_seg6[0] + alpha_ob11_seg6[1] + alpha_ob11_seg6[2]
   + alpha_ob11_seg6[3] >= 1
 R2866: - C12 + 1e+06 alpha_ob12_seg6[0] <= 999981.6050802565
 R2867: - C14 + 1e+06 alpha_ob12_seg6[0] <= 999981.6050802565
 R2868: - C13 + 1e+06 alpha_ob12_seg6[1] <= 999972.8942002755
 R2869: - C15 + 1e+06 alpha_ob12_seg6[1] <= 999972.8942002755
 R2870: C12 + 1e+06 alpha_ob12_seg6[2] <= 1.0000158448802566e+06
 R2871: C14 + 1e+06 alpha_ob12_seg6[2] <= 1.0000158448802566e+06
 R2872: C13 + 1e+06 alpha_ob12_seg6[3] <= 1.0000139558002756e+06
 R2873: C15 + 1e+06 alpha_ob12_seg6[3] <= 1.0000139558002756e+06
 R2874: alpha_ob12_seg6[0] + alpha_ob12_seg6[1] + alpha_ob12_seg6[2]
   + alpha_ob12_seg6[3] >= 1
 R2875: - C12 + 1e+06 alpha_ob13_seg6[0] <= 999981.605149801
 R2876: - C14 + 1e+06 alpha_ob13_seg6[0] <= 999981.605149801
 R2877: - C13 + 1e+06 alpha_ob13_seg6[1] <= 999983.4942391404
 R2878: - C15 + 1e+06 alpha_ob13_seg6[1] <= 999983.4942391404
 R2879: C12 + 1e+06 alpha_ob13_seg6[2] <= 999986.4948498011
 R2880: C14 + 1e+06 alpha_ob13_seg6[2] <= 999986.4948498011
 R2881: C13 + 1e+06 alpha_ob13_seg6[3] <= 1.0000139558391405e+06
 R2882: C15 + 1e+06 alpha_ob13_seg6[3] <= 1.0000139558391405e+06
 R2883: alpha_ob13_seg6[0] + alpha_ob13_seg6[1] + alpha_ob13_seg6[2]
   + alpha_ob13_seg6[3] >= 1
 R2884: - C12 + 1e+06 alpha_ob14_seg6[0] <= 999997.6803502565
 R2885: - C14 + 1e+06 alpha_ob14_seg6[0] <= 999997.6803502565
 R2886: - C13 + 1e+06 alpha_ob14_seg6[1] <= 999972.7268002755
 R2887: - C15 + 1e+06 alpha_ob14_seg6[1] <= 999972.7268002755
 R2888: C12 + 1e+06 alpha_ob14_seg6[2] <= 999999.7696102565
 R2889: C14 + 1e+06 alpha_ob14_seg6[2] <= 999999.7696102565
 R2890: C13 + 1e+06 alpha_ob14_seg6[3] <= 1.0000141232002755e+06
 R2891: C15 + 1e+06 alpha_ob14_seg6[3] <= 1.0000141232002755e+06
 R2892: alpha_ob14_seg6[0] + alpha_ob14_seg6[1] + alpha_ob14_seg6[2]
   + alpha_ob14_seg6[3] >= 1
 R2893: - C12 + 1e+06 alpha_ob15_seg6[0] <= 1.0000038341502565e+06
 R2894: - C14 + 1e+06 alpha_ob15_seg6[0] <= 1.0000038341502565e+06
 R2895: - C13 + 1e+06 alpha_ob15_seg6[1] <= 999972.6012002755
 R2896: - C15 + 1e+06 alpha_ob15_seg6[1] <= 999972.6012002755
 R2897: C12 + 1e+06 alpha_ob15_seg6[2] <= 999993.6158102566
 R2898: C14 + 1e+06 alpha_ob15_seg6[2] <= 999993.6158102566
 R2899: C13 + 1e+06 alpha_ob15_seg6[3] <= 1.0000142488002755e+06
 R2900: C15 + 1e+06 alpha_ob15_seg6[3] <= 1.0000142488002755e+06
 R2901: alpha_ob15_seg6[0] + alpha_ob15_seg6[1] + alpha_ob15_seg6[2]
   + alpha_ob15_seg6[3] >= 1
 R2902: - C12 + 1e+06 alpha_ob16_seg6[0] <= 1.0000162874063333e+06
 R2903: - C14 + 1e+06 alpha_ob16_seg6[0] <= 1.0000162874063333e+06
 R2904: - C13 + 1e+06 alpha_ob16_seg6[1] <= 999982.9439602756
 R2905: - C15 + 1e+06 alpha_ob16_seg6[1] <= 999982.9439602756
 R2906: C12 + 1e+06 alpha_ob16_seg6[2] <= 999981.1624063333
 R2907: C14 + 1e+06 alpha_ob16_seg6[2] <= 999981.1624063333
 R2908: C13 + 1e+06 alpha_ob16_seg6[3] <= 999963.6560402757
 R2909: C15 + 1e+06 alpha_ob16_seg6[3] <= 999963.6560402757
 R2910: alpha_ob16_seg6[0] + alpha_ob16_seg6[1] + alpha_ob16_seg6[2]
   + alpha_ob16_seg6[3] >= 1
 R2911: - C12 + 1e+06 alpha_ob17_seg6[0] <= 999980.937509
 R2912: - C14 + 1e+06 alpha_ob17_seg6[0] <= 999980.937509
 R2913: - C13 + 1e+06 alpha_ob17_seg6[1] <= 1.000034894e+06
 R2914: - C15 + 1e+06 alpha_ob17_seg6[1] <= 1.000034894e+06
 R2915: C12 + 1e+06 alpha_ob17_seg6[2] <= 999981.162491
 R2916: C14 + 1e+06 alpha_ob17_seg6[2] <= 999981.162491
 R2917: C13 + 1e+06 alpha_ob17_seg6[3] <= 999962.556
 R2918: C15 + 1e+06 alpha_ob17_seg6[3] <= 999962.556
 R2919: alpha_ob17_seg6[0] + alpha_ob17_seg6[1] + alpha_ob17_seg6[2]
   + alpha_ob17_seg6[3] >= 1
 R2920: - C12 + 1e+06 alpha_ob18_seg6[0] <= 999990.2670277582
 R2921: - C14 + 1e+06 alpha_ob18_seg6[0] <= 999990.2670277582
 R2922: - C13 + 1e+06 alpha_ob18_seg6[1] <= 1.0000347702002756e+06
 R2923: - C15 + 1e+06 alpha_ob18_seg6[1] <= 1.0000347702002756e+06
 R2924: C12 + 1e+06 alpha_ob18_seg6[2] <= 1.0000071828877581e+06
 R2925: C14 + 1e+06 alpha_ob18_seg6[2] <= 1.0000071828877581e+06
 R2926: C13 + 1e+06 alpha_ob18_seg6[3] <= 999939.8298002756
 R2927: C15 + 1e+06 alpha_ob18_seg6[3] <= 999939.8298002756
 R2928: alpha_ob18_seg6[0] + alpha_ob18_seg6[1] + alpha_ob18_seg6[2]
   + alpha_ob18_seg6[3] >= 1
 R2929: - C12 + 1e+06 alpha_ob19_seg6[0] <= 999990.267069801
 R2930: - C14 + 1e+06 alpha_ob19_seg6[0] <= 999990.267069801
 R2931: - C13 + 1e+06 alpha_ob19_seg6[1] <= 1.0000576202069657e+06
 R2932: - C15 + 1e+06 alpha_ob19_seg6[1] <= 1.0000576202069657e+06
 R2933: C12 + 1e+06 alpha_ob19_seg6[2] <= 1.000002082929801e+06
 R2934: C14 + 1e+06 alpha_ob19_seg6[2] <= 1.000002082929801e+06
 R2935: C13 + 1e+06 alpha_ob19_seg6[3] <= 999939.8298069657
 R2936: C15 + 1e+06 alpha_ob19_seg6[3] <= 999939.8298069657
 R2937: alpha_ob19_seg6[0] + alpha_ob19_seg6[1] + alpha_ob19_seg6[2]
   + alpha_ob19_seg6[3] >= 1
 R2938: - C12 + 1e+06 alpha_ob20_seg6[0] <= 999991.073732
 R2939: - C14 + 1e+06 alpha_ob20_seg6[0] <= 999991.073732
 R2940: - C13 + 1e+06 alpha_ob20_seg6[1] <= 999952.2125
 R2941: - C15 + 1e+06 alpha_ob20_seg6[1] <= 999952.2125
 R2942: C12 + 1e+06 alpha_ob20_seg6[2] <= 999990.776268
 R2943: C14 + 1e+06 alpha_ob20_seg6[2] <= 999990.776268
 R2944: C13 + 1e+06 alpha_ob20_seg6[3] <= 1.0000452375e+06
 R2945: C15 + 1e+06 alpha_ob20_seg6[3] <= 1.0000452375e+06
 R2946: alpha_ob20_seg6[0] + alpha_ob20_seg6[1] + alpha_ob20_seg6[2]
   + alpha_ob20_seg6[3] >= 1
 R2947: - C12 + 1e+06 alpha_ob21_seg6[0] <= 999985.657099801
 R2948: - C14 + 1e+06 alpha_ob21_seg6[0] <= 999985.657099801
 R2949: - C13 + 1e+06 alpha_ob21_seg6[1] <= 1.0000576202063023e+06
 R2950: - C15 + 1e+06 alpha_ob21_seg6[1] <= 1.0000576202063023e+06
 R2951: C12 + 1e+06 alpha_ob21_seg6[2] <= 1.000007192899801e+06
 R2952: C14 + 1e+06 alpha_ob21_seg6[2] <= 1.000007192899801e+06
 R2953: C13 + 1e+06 alpha_ob21_seg6[3] <= 999939.8298063023
 R2954: C15 + 1e+06 alpha_ob21_seg6[3] <= 999939.8298063023
 R2955: alpha_ob21_seg6[0] + alpha_ob21_seg6[1] + alpha_ob21_seg6[2]
   + alpha_ob21_seg6[3] >= 1
 R2956: - C12 + 1e+06 alpha_ob22_seg6[0] <= 999994.771829801
 R2957: - C14 + 1e+06 alpha_ob22_seg6[0] <= 999994.771829801
 R2958: - C13 + 1e+06 alpha_ob22_seg6[1] <= 1.0000615983063023e+06
 R2959: - C15 + 1e+06 alpha_ob22_seg6[1] <= 1.0000615983063023e+06
 R2960: C12 + 1e+06 alpha_ob22_seg6[2] <= 999998.078169801
 R2961: C14 + 1e+06 alpha_ob22_seg6[2] <= 999998.078169801
 R2962: C13 + 1e+06 alpha_ob22_seg6[3] <= 999935.8517063023
 R2963: C15 + 1e+06 alpha_ob22_seg6[3] <= 999935.8517063023
 R2964: alpha_ob22_seg6[0] + alpha_ob22_seg6[1] + alpha_ob22_seg6[2]
   + alpha_ob22_seg6[3] >= 1
 R2965: - C12 + 1e+06 alpha_ob23_seg6[0] <= 999980.9374508709
 R2966: - C14 + 1e+06 alpha_ob23_seg6[0] <= 999980.9374508709
 R2967: - C13 + 1e+06 alpha_ob23_seg6[1] <= 1.0000348940002755e+06
 R2968: - C15 + 1e+06 alpha_ob23_seg6[1] <= 1.0000348940002755e+06
 R2969: C12 + 1e+06 alpha_ob23_seg6[2] <= 1.0000165124508708e+06
 R2970: C14 + 1e+06 alpha_ob23_seg6[2] <= 1.0000165124508708e+06
 R2971: C13 + 1e+06 alpha_ob23_seg6[3] <= 999935.9560002756
 R2972: C15 + 1e+06 alpha_ob23_seg6[3] <= 999935.9560002756
 R2973: alpha_ob23_seg6[0] + alpha_ob23_seg6[1] + alpha_ob23_seg6[2]
   + alpha_ob23_seg6[3] >= 1
 R2974: - C12 + 1e+06 alpha_ob24_seg6[0] <= 999980.937499801
 R2975: - C14 + 1e+06 alpha_ob24_seg6[0] <= 999980.937499801
 R2976: - C13 + 1e+06 alpha_ob24_seg6[1] <= 1.0000614940072973e+06
 R2977: - C15 + 1e+06 alpha_ob24_seg6[1] <= 1.0000614940072973e+06
 R2978: C12 + 1e+06 alpha_ob24_seg6[2] <= 1.000011162499801e+06
 R2979: C14 + 1e+06 alpha_ob24_seg6[2] <= 1.000011162499801e+06
 R2980: C13 + 1e+06 alpha_ob24_seg6[3] <= 999935.9560072974
 R2981: C15 + 1e+06 alpha_ob24_seg6[3] <= 999935.9560072974
 R2982: alpha_ob24_seg6[0] + alpha_ob24_seg6[1] + alpha_ob24_seg6[2]
   + alpha_ob24_seg6[3] >= 1
 R2983: - C12 + 1e+06 alpha_ob25_seg6[0] <= 999956.8394000001
 R2984: - C14 + 1e+06 alpha_ob25_seg6[0] <= 999956.8394000001
 R2985: - C13 + 1e+06 alpha_ob25_seg6[1] <= 999952.2463
 R2986: - C15 + 1e+06 alpha_ob25_seg6[1] <= 999952.2463
 R2987: C12 + 1e+06 alpha_ob25_seg6[2] <= 1.0000097606e+06
 R2988: C14 + 1e+06 alpha_ob25_seg6[2] <= 1.0000097606e+06
 R2989: C13 + 1e+06 alpha_ob25_seg6[3] <= 1.0000452037e+06
 R2990: C15 + 1e+06 alpha_ob25_seg6[3] <= 1.0000452037e+06
 R2991: alpha_ob25_seg6[0] + alpha_ob25_seg6[1] + alpha_ob25_seg6[2]
   + alpha_ob25_seg6[3] >= 1
 R2992: - C12 + 1e+06 alpha_ob26_seg6[0] <= 999980.9375348955
 R2993: - C14 + 1e+06 alpha_ob26_seg6[0] <= 999980.9375348955
 R2994: - C13 + 1e+06 alpha_ob26_seg6[1] <= 1.0000160439997246e+06
 R2995: - C15 + 1e+06 alpha_ob26_seg6[1] <= 1.0000160439997246e+06
 R2996: C12 + 1e+06 alpha_ob26_seg6[2] <= 1.0000165125348954e+06
 R2997: C14 + 1e+06 alpha_ob26_seg6[2] <= 1.0000165125348954e+06
 R2998: C13 + 1e+06 alpha_ob26_seg6[3] <= 999962.5559997245
 R2999: C15 + 1e+06 alpha_ob26_seg6[3] <= 999962.5559997245
 R3000: alpha_ob26_seg6[0] + alpha_ob26_seg6[1] + alpha_ob26_seg6[2]
   + alpha_ob26_seg6[3] >= 1
 R3001: - C12 + 1e+06 alpha_ob27_seg6[0] <= 999956.839399801
 R3002: - C14 + 1e+06 alpha_ob27_seg6[0] <= 999956.839399801
 R3003: - C13 + 1e+06 alpha_ob27_seg6[1] <= 999925.3963434525
 R3004: - C15 + 1e+06 alpha_ob27_seg6[1] <= 999925.3963434525
 R3005: C12 + 1e+06 alpha_ob27_seg6[2] <= 1.0000080105998011e+06
 R3006: C14 + 1e+06 alpha_ob27_seg6[2] <= 1.0000080105998011e+06
 R3007: C13 + 1e+06 alpha_ob27_seg6[3] <= 1.0000720537434525e+06
 R3008: C15 + 1e+06 alpha_ob27_seg6[3] <= 1.0000720537434525e+06
 R3009: alpha_ob27_seg6[0] + alpha_ob27_seg6[1] + alpha_ob27_seg6[2]
   + alpha_ob27_seg6[3] >= 1
 R3010: - C12 + 1e+06 alpha_ob28_seg6[0] <= 999980.9374998009
 R3011: - C14 + 1e+06 alpha_ob28_seg6[0] <= 999980.9374998009
 R3012: - C13 + 1e+06 alpha_ob28_seg6[1] <= 1.0000160440053071e+06
 R3013: - C15 + 1e+06 alpha_ob28_seg6[1] <= 1.0000160440053071e+06
 R3014: C12 + 1e+06 alpha_ob28_seg6[2] <= 1.000012662499801e+06
 R3015: C14 + 1e+06 alpha_ob28_seg6[2] <= 1.000012662499801e+06
 R3016: C13 + 1e+06 alpha_ob28_seg6[3] <= 999981.4060053072
 R3017: C15 + 1e+06 alpha_ob28_seg6[3] <= 999981.4060053072
 R3018: alpha_ob28_seg6[0] + alpha_ob28_seg6[1] + alpha_ob28_seg6[2]
   + alpha_ob28_seg6[3] >= 1
 R3019: - C12 + 1e+06 alpha_ob29_seg6[0] <= 999983.6053459151
 R3020: - C14 + 1e+06 alpha_ob29_seg6[0] <= 999983.6053459151
 R3021: - C13 + 1e+06 alpha_ob29_seg6[1] <= 999987.7722497246
 R3022: - C15 + 1e+06 alpha_ob29_seg6[1] <= 999987.7722497246
 R3023: C12 + 1e+06 alpha_ob29_seg6[2] <= 1.000013844745915e+06
 R3024: C14 + 1e+06 alpha_ob29_seg6[2] <= 1.000013844745915e+06
 R3025: C13 + 1e+06 alpha_ob29_seg6[3] <= 999984.8277497246
 R3026: C15 + 1e+06 alpha_ob29_seg6[3] <= 999984.8277497246
 R3027: alpha_ob29_seg6[0] + alpha_ob29_seg6[1] + alpha_ob29_seg6[2]
   + alpha_ob29_seg6[3] >= 1
 R3028: - C12 + 1e+06 alpha_ob30_seg6[0] <= 999958.8717898987
 R3029: - C14 + 1e+06 alpha_ob30_seg6[0] <= 999958.8717898987
 R3030: - C13 + 1e+06 alpha_ob30_seg6[1] <= 999987.7837602755
 R3031: - C15 + 1e+06 alpha_ob30_seg6[1] <= 999987.7837602755
 R3032: C12 + 1e+06 alpha_ob30_seg6[2] <= 1.0000385781898986e+06
 R3033: C14 + 1e+06 alpha_ob30_seg6[2] <= 1.0000385781898986e+06
 R3034: C13 + 1e+06 alpha_ob30_seg6[3] <= 1.0000043162402755e+06
 R3035: C15 + 1e+06 alpha_ob30_seg6[3] <= 1.0000043162402755e+06
 R3036: alpha_ob30_seg6[0] + alpha_ob30_seg6[1] + alpha_ob30_seg6[2]
   + alpha_ob30_seg6[3] >= 1
 R3037: - C12 + 1e+06 alpha_ob31_seg6[0] <= 999939.2718
 R3038: - C14 + 1e+06 alpha_ob31_seg6[0] <= 999939.2718
 R3039: - C13 + 1e+06 alpha_ob31_seg6[1] <= 999993.13376
 R3040: - C15 + 1e+06 alpha_ob31_seg6[1] <= 999993.13376
 R3041: C12 + 1e+06 alpha_ob31_seg6[2] <= 1.0000385782e+06
 R3042: C14 + 1e+06 alpha_ob31_seg6[2] <= 1.0000385782e+06
 R3043: C13 + 1e+06 alpha_ob31_seg6[3] <= 1.00000431624e+06
 R3044: C15 + 1e+06 alpha_ob31_seg6[3] <= 1.00000431624e+06
 R3045: alpha_ob31_seg6[0] + alpha_ob31_seg6[1] + alpha_ob31_seg6[2]
   + alpha_ob31_seg6[3] >= 1
 R3046: - C12 + 1e+06 alpha_ob32_seg6[0] <= 999939.2718101013
 R3047: - C14 + 1e+06 alpha_ob32_seg6[0] <= 999939.2718101013
 R3048: - C13 + 1e+06 alpha_ob32_seg6[1] <= 999987.7837597246
 R3049: - C15 + 1e+06 alpha_ob32_seg6[1] <= 999987.7837597246
 R3050: C12 + 1e+06 alpha_ob32_seg6[2] <= 1.0000581782101013e+06
 R3051: C14 + 1e+06 alpha_ob32_seg6[2] <= 1.0000581782101013e+06
 R3052: C13 + 1e+06 alpha_ob32_seg6[3] <= 1.0000043162397245e+06
 R3053: C15 + 1e+06 alpha_ob32_seg6[3] <= 1.0000043162397245e+06
 R3054: alpha_ob32_seg6[0] + alpha_ob32_seg6[1] + alpha_ob32_seg6[2]
   + alpha_ob32_seg6[3] >= 1
 R3055: - C12 + 1e+06 alpha_ob33_seg6[0] <= 999939.271799801
 R3056: - C14 + 1e+06 alpha_ob33_seg6[0] <= 999939.271799801
 R3057: - C13 + 1e+06 alpha_ob33_seg6[1] <= 999987.7838262042
 R3058: - C15 + 1e+06 alpha_ob33_seg6[1] <= 999987.7838262042
 R3059: C12 + 1e+06 alpha_ob33_seg6[2] <= 1.000038578199801e+06
 R3060: C14 + 1e+06 alpha_ob33_seg6[2] <= 1.000038578199801e+06
 R3061: C13 + 1e+06 alpha_ob33_seg6[3] <= 1.0000096662262043e+06
 R3062: C15 + 1e+06 alpha_ob33_seg6[3] <= 1.0000096662262043e+06
 R3063: alpha_ob33_seg6[0] + alpha_ob33_seg6[1] + alpha_ob33_seg6[2]
   + alpha_ob33_seg6[3] >= 1
 R3064: - C12 + 1e+06 alpha_ob34_seg6[0] <= 999926.0708
 R3065: - C14 + 1e+06 alpha_ob34_seg6[0] <= 999926.0708
 R3066: - C13 + 1e+06 alpha_ob34_seg6[1] <= 999983.7196
 R3067: - C15 + 1e+06 alpha_ob34_seg6[1] <= 999983.7196
 R3068: C12 + 1e+06 alpha_ob34_seg6[2] <= 1.0000250292e+06
 R3069: C14 + 1e+06 alpha_ob34_seg6[2] <= 1.0000250292e+06
 R3070: C13 + 1e+06 alpha_ob34_seg6[3] <= 1.0000137304e+06
 R3071: C15 + 1e+06 alpha_ob34_seg6[3] <= 1.0000137304e+06
 R3072: alpha_ob34_seg6[0] + alpha_ob34_seg6[1] + alpha_ob34_seg6[2]
   + alpha_ob34_seg6[3] >= 1
 R3073: - C12 + 1e+06 alpha_ob35_seg6[0] <= 999926.0708293857
 R3074: - C14 + 1e+06 alpha_ob35_seg6[0] <= 999926.0708293857
 R3075: - C13 + 1e+06 alpha_ob35_seg6[1] <= 999967.8695997245
 R3076: - C15 + 1e+06 alpha_ob35_seg6[1] <= 999967.8695997245
 R3077: C12 + 1e+06 alpha_ob35_seg6[2] <= 1.0000713792293856e+06
 R3078: C14 + 1e+06 alpha_ob35_seg6[2] <= 1.0000713792293856e+06
 R3079: C13 + 1e+06 alpha_ob35_seg6[3] <= 1.0000137303997246e+06
 R3080: C15 + 1e+06 alpha_ob35_seg6[3] <= 1.0000137303997246e+06
 R3081: alpha_ob35_seg6[0] + alpha_ob35_seg6[1] + alpha_ob35_seg6[2]
   + alpha_ob35_seg6[3] >= 1
 R3082: - C12 + 1e+06 alpha_ob36_seg6[0] <= 999972.6917508709
 R3083: - C14 + 1e+06 alpha_ob36_seg6[0] <= 999972.6917508709
 R3084: - C13 + 1e+06 alpha_ob36_seg6[1] <= 999957.1644002756
 R3085: - C15 + 1e+06 alpha_ob36_seg6[1] <= 999957.1644002756
 R3086: C12 + 1e+06 alpha_ob36_seg6[2] <= 1.0000247581508709e+06
 R3087: C14 + 1e+06 alpha_ob36_seg6[2] <= 1.0000247581508709e+06
 R3088: C13 + 1e+06 alpha_ob36_seg6[3] <= 1.0000136856002755e+06
 R3089: C15 + 1e+06 alpha_ob36_seg6[3] <= 1.0000136856002755e+06
 R3090: alpha_ob36_seg6[0] + alpha_ob36_seg6[1] + alpha_ob36_seg6[2]
   + alpha_ob36_seg6[3] >= 1
 R3091: - C12 + 1e+06 alpha_ob37_seg6[0] <= 999939.0009674003
 R3092: - C14 + 1e+06 alpha_ob37_seg6[0] <= 999939.0009674003
 R3093: - C13 + 1e+06 alpha_ob37_seg6[1] <= 999961.9516002756
 R3094: - C15 + 1e+06 alpha_ob37_seg6[1] <= 999961.9516002756
 R3095: C12 + 1e+06 alpha_ob37_seg6[2] <= 1.0000584489674004e+06
 R3096: C14 + 1e+06 alpha_ob37_seg6[2] <= 1.0000584489674004e+06
 R3097: C13 + 1e+06 alpha_ob37_seg6[3] <= 1.0000178984002755e+06
 R3098: C15 + 1e+06 alpha_ob37_seg6[3] <= 1.0000178984002755e+06
 R3099: alpha_ob37_seg6[0] + alpha_ob37_seg6[1] + alpha_ob37_seg6[2]
   + alpha_ob37_seg6[3] >= 1
 R3100: - C12 + 1e+06 alpha_ob38_seg6[0] <= 999994.9164998011
 R3101: - C14 + 1e+06 alpha_ob38_seg6[0] <= 999994.9164998011
 R3102: - C13 + 1e+06 alpha_ob38_seg6[1] <= 999925.4284782809
 R3103: - C15 + 1e+06 alpha_ob38_seg6[1] <= 999925.4284782809
 R3104: C12 + 1e+06 alpha_ob38_seg6[2] <= 999943.6834998011
 R3105: C14 + 1e+06 alpha_ob38_seg6[2] <= 999943.6834998011
 R3106: C13 + 1e+06 alpha_ob38_seg6[3] <= 1.0000720216782809e+06
 R3107: C15 + 1e+06 alpha_ob38_seg6[3] <= 1.0000720216782809e+06
 R3108: alpha_ob38_seg6[0] + alpha_ob38_seg6[1] + alpha_ob38_seg6[2]
   + alpha_ob38_seg6[3] >= 1
 R3109: - C12 + 1e+06 alpha_ob39_seg6[0] <= 999914.52327842
 R3110: - C14 + 1e+06 alpha_ob39_seg6[0] <= 999914.52327842
 R3111: - C13 + 1e+06 alpha_ob39_seg6[1] <= 999993.6552362755
 R3112: - C15 + 1e+06 alpha_ob39_seg6[1] <= 999993.6552362755
 R3113: C12 + 1e+06 alpha_ob39_seg6[2] <= 1.0000829266784199e+06
 R3114: C14 + 1e+06 alpha_ob39_seg6[2] <= 1.0000829266784199e+06
 R3115: C13 + 1e+06 alpha_ob39_seg6[3] <= 999992.1947642756
 R3116: C15 + 1e+06 alpha_ob39_seg6[3] <= 999992.1947642756
 R3117: alpha_ob39_seg6[0] + alpha_ob39_seg6[1] + alpha_ob39_seg6[2]
   + alpha_ob39_seg6[3] >= 1
 R3118: - C12 + 1e+06 alpha_ob40_seg6[0] <= 999912.265199801
 R3119: - C14 + 1e+06 alpha_ob40_seg6[0] <= 999912.265199801
 R3120: - C13 + 1e+06 alpha_ob40_seg6[1] <= 999987.7842112777
 R3121: - C15 + 1e+06 alpha_ob40_seg6[1] <= 999987.7842112777
 R3122: C12 + 1e+06 alpha_ob40_seg6[2] <= 1.000076834799801e+06
 R3123: C14 + 1e+06 alpha_ob40_seg6[2] <= 1.000076834799801e+06
 R3124: C13 + 1e+06 alpha_ob40_seg6[3] <= 1.0000096658112778e+06
 R3125: C15 + 1e+06 alpha_ob40_seg6[3] <= 1.0000096658112778e+06
 R3126: alpha_ob40_seg6[0] + alpha_ob40_seg6[1] + alpha_ob40_seg6[2]
   + alpha_ob40_seg6[3] >= 1
 R3127: - C12 + 1e+06 alpha_ob41_seg6[0] <= 999920.6152119379
 R3128: - C14 + 1e+06 alpha_ob41_seg6[0] <= 999920.6152119379
 R3129: - C13 + 1e+06 alpha_ob41_seg6[1] <= 999981.4341997246
 R3130: - C15 + 1e+06 alpha_ob41_seg6[1] <= 999981.4341997246
 R3131: C12 + 1e+06 alpha_ob41_seg6[2] <= 1.0000768348119379e+06
 R3132: C14 + 1e+06 alpha_ob41_seg6[2] <= 1.0000768348119379e+06
 R3133: C13 + 1e+06 alpha_ob41_seg6[3] <= 1.0000096657997245e+06
 R3134: C15 + 1e+06 alpha_ob41_seg6[3] <= 1.0000096657997245e+06
 R3135: alpha_ob41_seg6[0] + alpha_ob41_seg6[1] + alpha_ob41_seg6[2]
   + alpha_ob41_seg6[3] >= 1
 R3136: - C12 + 1e+06 alpha_ob42_seg6[0] <= 999919.2652
 R3137: - C14 + 1e+06 alpha_ob42_seg6[0] <= 999919.2652
 R3138: - C13 + 1e+06 alpha_ob42_seg6[1] <= 999981.4342
 R3139: - C15 + 1e+06 alpha_ob42_seg6[1] <= 999981.4342
 R3140: C12 + 1e+06 alpha_ob42_seg6[2] <= 1.0000768348e+06
 R3141: C14 + 1e+06 alpha_ob42_seg6[2] <= 1.0000768348e+06
 R3142: C13 + 1e+06 alpha_ob42_seg6[3] <= 1.0000160158000001e+06
 R3143: C15 + 1e+06 alpha_ob42_seg6[3] <= 1.0000160158000001e+06
 R3144: alpha_ob42_seg6[0] + alpha_ob42_seg6[1] + alpha_ob42_seg6[2]
   + alpha_ob42_seg6[3] >= 1
 R3145: - C12 + 1e+06 alpha_ob43_seg6[0] <= 999919.2652454559
 R3146: - C14 + 1e+06 alpha_ob43_seg6[0] <= 999919.2652454559
 R3147: - C13 + 1e+06 alpha_ob43_seg6[1] <= 999956.8341997246
 R3148: - C15 + 1e+06 alpha_ob43_seg6[1] <= 999956.8341997246
 R3149: C12 + 1e+06 alpha_ob43_seg6[2] <= 1.0000781848454559e+06
 R3150: C14 + 1e+06 alpha_ob43_seg6[2] <= 1.0000781848454559e+06
 R3151: C13 + 1e+06 alpha_ob43_seg6[3] <= 1.0000160157997246e+06
 R3152: C15 + 1e+06 alpha_ob43_seg6[3] <= 1.0000160157997246e+06
 R3153: alpha_ob43_seg6[0] + alpha_ob43_seg6[1] + alpha_ob43_seg6[2]
   + alpha_ob43_seg6[3] >= 1
 R3154: - C12 + 1e+06 alpha_ob44_seg6[0] <= 999991.83683
 R3155: - C14 + 1e+06 alpha_ob44_seg6[0] <= 999991.83683
 R3156: - C13 + 1e+06 alpha_ob44_seg6[1] <= 999940.589
 R3157: - C15 + 1e+06 alpha_ob44_seg6[1] <= 999940.589
 R3158: C12 + 1e+06 alpha_ob44_seg6[2] <= 1.00000401317e+06
 R3159: C14 + 1e+06 alpha_ob44_seg6[2] <= 1.00000401317e+06
 R3160: C13 + 1e+06 alpha_ob44_seg6[3] <= 1.000056861e+06
 R3161: C15 + 1e+06 alpha_ob44_seg6[3] <= 1.000056861e+06
 R3162: alpha_ob44_seg6[0] + alpha_ob44_seg6[1] + alpha_ob44_seg6[2]
   + alpha_ob44_seg6[3] >= 1
 R3163: - C12 + 1e+06 alpha_ob45_seg6[0] <= 999991.8368084199
 R3164: - C14 + 1e+06 alpha_ob45_seg6[0] <= 999991.8368084199
 R3165: - C13 + 1e+06 alpha_ob45_seg6[1] <= 999940.5890002755
 R3166: - C15 + 1e+06 alpha_ob45_seg6[1] <= 999940.5890002755
 R3167: C12 + 1e+06 alpha_ob45_seg6[2] <= 1.0000056131484199e+06
 R3168: C14 + 1e+06 alpha_ob45_seg6[2] <= 1.0000056131484199e+06
 R3169: C13 + 1e+06 alpha_ob45_seg6[3] <= 1.0000452610002755e+06
 R3170: C15 + 1e+06 alpha_ob45_seg6[3] <= 1.0000452610002755e+06
 R3171: alpha_ob45_seg6[0] + alpha_ob45_seg6[1] + alpha_ob45_seg6[2]
   + alpha_ob45_seg6[3] >= 1
 R3172: - C12 + 1e+06 alpha_ob46_seg6[0] <= 1.0000764021508709e+06
 R3173: - C14 + 1e+06 alpha_ob46_seg6[0] <= 1.0000764021508709e+06
 R3174: - C13 + 1e+06 alpha_ob46_seg6[1] <= 999957.1027002756
 R3175: - C15 + 1e+06 alpha_ob46_seg6[1] <= 999957.1027002756
 R3176: C12 + 1e+06 alpha_ob46_seg6[2] <= 999921.0477508709
 R3177: C14 + 1e+06 alpha_ob46_seg6[2] <= 999921.0477508709
 R3178: C13 + 1e+06 alpha_ob46_seg6[3] <= 1.0000137473002756e+06
 R3179: C15 + 1e+06 alpha_ob46_seg6[3] <= 1.0000137473002756e+06
 R3180: alpha_ob46_seg6[0] + alpha_ob46_seg6[1] + alpha_ob46_seg6[2]
   + alpha_ob46_seg6[3] >= 1
 R3181: - C12 + 1e+06 alpha_ob47_seg6[0] <= 1.0000445522e+06
 R3182: - C14 + 1e+06 alpha_ob47_seg6[0] <= 1.0000445522e+06
 R3183: - C13 + 1e+06 alpha_ob47_seg6[1] <= 999983.7027
 R3184: - C15 + 1e+06 alpha_ob47_seg6[1] <= 999983.7027
 R3185: C12 + 1e+06 alpha_ob47_seg6[2] <= 999921.0477999999
 R3186: C14 + 1e+06 alpha_ob47_seg6[2] <= 999921.0477999999
 R3187: C13 + 1e+06 alpha_ob47_seg6[3] <= 1.0000137473e+06
 R3188: C15 + 1e+06 alpha_ob47_seg6[3] <= 1.0000137473e+06
 R3189: alpha_ob47_seg6[0] + alpha_ob47_seg6[1] + alpha_ob47_seg6[2]
   + alpha_ob47_seg6[3] >= 1
 R3190: - C12 + 1e+06 alpha_ob48_seg6[0] <= 1.0000445522183661e+06
 R3191: - C14 + 1e+06 alpha_ob48_seg6[0] <= 1.0000445522183661e+06
 R3192: - C13 + 1e+06 alpha_ob48_seg6[1] <= 999973.8526997245
 R3193: - C15 + 1e+06 alpha_ob48_seg6[1] <= 999973.8526997245
 R3194: C12 + 1e+06 alpha_ob48_seg6[2] <= 999952.8978183661
 R3195: C14 + 1e+06 alpha_ob48_seg6[2] <= 999952.8978183661
 R3196: C13 + 1e+06 alpha_ob48_seg6[3] <= 1.0000137472997245e+06
 R3197: C15 + 1e+06 alpha_ob48_seg6[3] <= 1.0000137472997245e+06
 R3198: alpha_ob48_seg6[0] + alpha_ob48_seg6[1] + alpha_ob48_seg6[2]
   + alpha_ob48_seg6[3] >= 1
 R3199: - C12 + 1e+06 alpha_ob49_seg6[0] <= 1.000064332081634e+06
 R3200: - C14 + 1e+06 alpha_ob49_seg6[0] <= 1.000064332081634e+06
 R3201: - C13 + 1e+06 alpha_ob49_seg6[1] <= 999973.9401002756
 R3202: - C15 + 1e+06 alpha_ob49_seg6[1] <= 999973.9401002756
 R3203: C12 + 1e+06 alpha_ob49_seg6[2] <= 999933.1178816339
 R3204: C14 + 1e+06 alpha_ob49_seg6[2] <= 999933.1178816339
 R3205: C13 + 1e+06 alpha_ob49_seg6[3] <= 1.0000136599002755e+06
 R3206: C15 + 1e+06 alpha_ob49_seg6[3] <= 1.0000136599002755e+06
 R3207: alpha_ob49_seg6[0] + alpha_ob49_seg6[1] + alpha_ob49_seg6[2]
   + alpha_ob49_seg6[3] >= 1
 R3208: - C12 + 1e+06 alpha_ob50_seg6[0] <= 1.0000643321013774e+06
 R3209: - C14 + 1e+06 alpha_ob50_seg6[0] <= 1.0000643321013774e+06
 R3210: - C13 + 1e+06 alpha_ob50_seg6[1] <= 999983.1900997246
 R3211: - C15 + 1e+06 alpha_ob50_seg6[1] <= 999983.1900997246
 R3212: C12 + 1e+06 alpha_ob50_seg6[2] <= 999933.1179013775
 R3213: C14 + 1e+06 alpha_ob50_seg6[2] <= 999933.1179013775
 R3214: C13 + 1e+06 alpha_ob50_seg6[3] <= 1.0000136598997245e+06
 R3215: C15 + 1e+06 alpha_ob50_seg6[3] <= 1.0000136598997245e+06
 R3216: alpha_ob50_seg6[0] + alpha_ob50_seg6[1] + alpha_ob50_seg6[2]
   + alpha_ob50_seg6[3] >= 1
 R3217: - C14 + 1e+06 alpha_ob0_seg7[0] <= 1.0000284170499525e+06
 R3218: - C16 + 1e+06 alpha_ob0_seg7[0] <= 1.0000284170499525e+06
 R3219: - C15 + 1e+06 alpha_ob0_seg7[1] <= 999957.0843002755
 R3220: - C17 + 1e+06 alpha_ob0_seg7[1] <= 999957.0843002755
 R3221: C14 + 1e+06 alpha_ob0_seg7[2] <= 999969.0328499526
 R3222: C16 + 1e+06 alpha_ob0_seg7[2] <= 999969.0328499526
 R3223: C15 + 1e+06 alpha_ob0_seg7[3] <= 1.0000132657002755e+06
 R3224: C17 + 1e+06 alpha_ob0_seg7[3] <= 1.0000132657002755e+06
 R3225: alpha_ob0_seg7[0] + alpha_ob0_seg7[1] + alpha_ob0_seg7[2]
   + alpha_ob0_seg7[3] >= 1
 R3226: - C14 + 1e+06 alpha_ob1_seg7[0] <= 1.000028417099801e+06
 R3227: - C16 + 1e+06 alpha_ob1_seg7[0] <= 1.000028417099801e+06
 R3228: - C15 + 1e+06 alpha_ob1_seg7[1] <= 999984.1843155898
 R3229: - C17 + 1e+06 alpha_ob1_seg7[1] <= 999984.1843155898
 R3230: C14 + 1e+06 alpha_ob1_seg7[2] <= 999957.4328998009
 R3231: C16 + 1e+06 alpha_ob1_seg7[2] <= 999957.4328998009
 R3232: C15 + 1e+06 alpha_ob1_seg7[3] <= 1.0000132657155899e+06
 R3233: C17 + 1e+06 alpha_ob1_seg7[3] <= 1.0000132657155899e+06
 R3234: alpha_ob1_seg7[0] + alpha_ob1_seg7[1] + alpha_ob1_seg7[2]
   + alpha_ob1_seg7[3] >= 1
 R3235: - C14 + 1e+06 alpha_ob2_seg7[0] <= 1.0000400171500475e+06
 R3236: - C16 + 1e+06 alpha_ob2_seg7[0] <= 1.0000400171500475e+06
 R3237: - C15 + 1e+06 alpha_ob2_seg7[1] <= 999957.0842997245
 R3238: - C17 + 1e+06 alpha_ob2_seg7[1] <= 999957.0842997245
 R3239: C14 + 1e+06 alpha_ob2_seg7[2] <= 999957.4329500474
 R3240: C16 + 1e+06 alpha_ob2_seg7[2] <= 999957.4329500474
 R3241: C15 + 1e+06 alpha_ob2_seg7[3] <= 1.0000132656997246e+06
 R3242: C17 + 1e+06 alpha_ob2_seg7[3] <= 1.0000132656997246e+06
 R3243: alpha_ob2_seg7[0] + alpha_ob2_seg7[1] + alpha_ob2_seg7[2]
   + alpha_ob2_seg7[3] >= 1
 R3244: - C14 + 1e+06 alpha_ob3_seg7[0] <= 1.0000109161752058e+06
 R3245: - C16 + 1e+06 alpha_ob3_seg7[0] <= 1.0000109161752058e+06
 R3246: - C15 + 1e+06 alpha_ob3_seg7[1] <= 999970.2681002755
 R3247: - C17 + 1e+06 alpha_ob3_seg7[1] <= 999970.2681002755
 R3248: C14 + 1e+06 alpha_ob3_seg7[2] <= 999986.5337752058
 R3249: C16 + 1e+06 alpha_ob3_seg7[2] <= 999986.5337752058
 R3250: C15 + 1e+06 alpha_ob3_seg7[3] <= 1.0000138319002755e+06
 R3251: C17 + 1e+06 alpha_ob3_seg7[3] <= 1.0000138319002755e+06
 R3252: alpha_ob3_seg7[0] + alpha_ob3_seg7[1] + alpha_ob3_seg7[2]
   + alpha_ob3_seg7[3] >= 1
 R3253: - C14 + 1e+06 alpha_ob4_seg7[0] <= 1.000010916199801e+06
 R3254: - C16 + 1e+06 alpha_ob4_seg7[0] <= 1.000010916199801e+06
 R3255: - C15 + 1e+06 alpha_ob4_seg7[1] <= 999983.6181179117
 R3256: - C17 + 1e+06 alpha_ob4_seg7[1] <= 999983.6181179117
 R3257: C14 + 1e+06 alpha_ob4_seg7[2] <= 999973.183799801
 R3258: C16 + 1e+06 alpha_ob4_seg7[2] <= 999973.183799801
 R3259: C15 + 1e+06 alpha_ob4_seg7[3] <= 1.0000138319179118e+06
 R3260: C17 + 1e+06 alpha_ob4_seg7[3] <= 1.0000138319179118e+06
 R3261: alpha_ob4_seg7[0] + alpha_ob4_seg7[1] + alpha_ob4_seg7[2]
   + alpha_ob4_seg7[3] >= 1
 R3262: - C14 + 1e+06 alpha_ob5_seg7[0] <= 999992.49102
 R3263: - C16 + 1e+06 alpha_ob5_seg7[0] <= 999992.49102
 R3264: - C15 + 1e+06 alpha_ob5_seg7[1] <= 999957.1016000001
 R3265: - C17 + 1e+06 alpha_ob5_seg7[1] <= 999957.1016000001
 R3266: C14 + 1e+06 alpha_ob5_seg7[2] <= 999986.35898
 R3267: C16 + 1e+06 alpha_ob5_seg7[2] <= 999986.35898
 R3268: C15 + 1e+06 alpha_ob5_seg7[3] <= 1.0000403484e+06
 R3269: C17 + 1e+06 alpha_ob5_seg7[3] <= 1.0000403484e+06
 R3270: alpha_ob5_seg7[0] + alpha_ob5_seg7[1] + alpha_ob5_seg7[2]
   + alpha_ob5_seg7[3] >= 1
 R3271: - C14 + 1e+06 alpha_ob6_seg7[0] <= 999981.3909999999
 R3272: - C16 + 1e+06 alpha_ob6_seg7[0] <= 999981.3909999999
 R3273: - C15 + 1e+06 alpha_ob6_seg7[1] <= 999957.1016000001
 R3274: - C17 + 1e+06 alpha_ob6_seg7[1] <= 999957.1016000001
 R3275: C14 + 1e+06 alpha_ob6_seg7[2] <= 1.000004959e+06
 R3276: C16 + 1e+06 alpha_ob6_seg7[2] <= 1.000004959e+06
 R3277: C15 + 1e+06 alpha_ob6_seg7[3] <= 1.0000403484e+06
 R3278: C17 + 1e+06 alpha_ob6_seg7[3] <= 1.0000403484e+06
 R3279: alpha_ob6_seg7[0] + alpha_ob6_seg7[1] + alpha_ob6_seg7[2]
   + alpha_ob6_seg7[3] >= 1
 R3280: - C14 + 1e+06 alpha_ob7_seg7[0] <= 999981.3909825523
 R3281: - C16 + 1e+06 alpha_ob7_seg7[0] <= 999981.3909825523
 R3282: - C15 + 1e+06 alpha_ob7_seg7[1] <= 999957.1016002755
 R3283: - C17 + 1e+06 alpha_ob7_seg7[1] <= 999957.1016002755
 R3284: C14 + 1e+06 alpha_ob7_seg7[2] <= 1.0000160589825523e+06
 R3285: C16 + 1e+06 alpha_ob7_seg7[2] <= 1.0000160589825523e+06
 R3286: C15 + 1e+06 alpha_ob7_seg7[3] <= 1.0000309984002755e+06
 R3287: C17 + 1e+06 alpha_ob7_seg7[3] <= 1.0000309984002755e+06
 R3288: alpha_ob7_seg7[0] + alpha_ob7_seg7[1] + alpha_ob7_seg7[2]
   + alpha_ob7_seg7[3] >= 1
 R3289: - C14 + 1e+06 alpha_ob8_seg7[0] <= 999981.3867954084
 R3290: - C16 + 1e+06 alpha_ob8_seg7[0] <= 999981.3867954084
 R3291: - C15 + 1e+06 alpha_ob8_seg7[1] <= 999968.8595002755
 R3292: - C17 + 1e+06 alpha_ob8_seg7[1] <= 999968.8595002755
 R3293: C14 + 1e+06 alpha_ob8_seg7[2] <= 1.0000160631954084e+06
 R3294: C16 + 1e+06 alpha_ob8_seg7[2] <= 1.0000160631954084e+06
 R3295: C15 + 1e+06 alpha_ob8_seg7[3] <= 1.0000262405002755e+06
 R3296: C17 + 1e+06 alpha_ob8_seg7[3] <= 1.0000262405002755e+06
 R3297: alpha_ob8_seg7[0] + alpha_ob8_seg7[1] + alpha_ob8_seg7[2]
   + alpha_ob8_seg7[3] >= 1
 R3298: - C14 + 1e+06 alpha_ob9_seg7[0] <= 999981.386799801
 R3299: - C16 + 1e+06 alpha_ob9_seg7[0] <= 999981.386799801
 R3300: - C15 + 1e+06 alpha_ob9_seg7[1] <= 999971.2095013268
 R3301: - C17 + 1e+06 alpha_ob9_seg7[1] <= 999971.2095013268
 R3302: C14 + 1e+06 alpha_ob9_seg7[2] <= 1.0000152131998009e+06
 R3303: C16 + 1e+06 alpha_ob9_seg7[2] <= 1.0000152131998009e+06
 R3304: C15 + 1e+06 alpha_ob9_seg7[3] <= 1.0000262405013267e+06
 R3305: C17 + 1e+06 alpha_ob9_seg7[3] <= 1.0000262405013267e+06
 R3306: alpha_ob9_seg7[0] + alpha_ob9_seg7[1] + alpha_ob9_seg7[2]
   + alpha_ob9_seg7[3] >= 1
 R3307: - C14 + 1e+06 alpha_ob10_seg7[0] <= 999982.2368022958
 R3308: - C16 + 1e+06 alpha_ob10_seg7[0] <= 999982.2368022958
 R3309: - C15 + 1e+06 alpha_ob10_seg7[1] <= 999970.1094997245
 R3310: - C17 + 1e+06 alpha_ob10_seg7[1] <= 999970.1094997245
 R3311: C14 + 1e+06 alpha_ob10_seg7[2] <= 1.0000152132022957e+06
 R3312: C16 + 1e+06 alpha_ob10_seg7[2] <= 1.0000152132022957e+06
 R3313: C15 + 1e+06 alpha_ob10_seg7[3] <= 1.0000262404997245e+06
 R3314: C17 + 1e+06 alpha_ob10_seg7[3] <= 1.0000262404997245e+06
 R3315: alpha_ob10_seg7[0] + alpha_ob10_seg7[1] + alpha_ob10_seg7[2]
   + alpha_ob10_seg7[3] >= 1
 R3316: - C14 + 1e+06 alpha_ob11_seg7[0] <= 999982.236799801
 R3317: - C16 + 1e+06 alpha_ob11_seg7[0] <= 999982.236799801
 R3318: - C15 + 1e+06 alpha_ob11_seg7[1] <= 999970.1095291895
 R3319: - C17 + 1e+06 alpha_ob11_seg7[1] <= 999970.1095291895
 R3320: C14 + 1e+06 alpha_ob11_seg7[2] <= 999993.3631998011
 R3321: C16 + 1e+06 alpha_ob11_seg7[2] <= 999993.3631998011
 R3322: C15 + 1e+06 alpha_ob11_seg7[3] <= 1.0000273405291894e+06
 R3323: C17 + 1e+06 alpha_ob11_seg7[3] <= 1.0000273405291894e+06
 R3324: alpha_ob11_seg7[0] + alpha_ob11_seg7[1] + alpha_ob11_seg7[2]
   + alpha_ob11_seg7[3] >= 1
 R3325: - C14 + 1e+06 alpha_ob12_seg7[0] <= 999981.6050802565
 R3326: - C16 + 1e+06 alpha_ob12_seg7[0] <= 999981.6050802565
 R3327: - C15 + 1e+06 alpha_ob12_seg7[1] <= 999972.8942002755
 R3328: - C17 + 1e+06 alpha_ob12_seg7[1] <= 999972.8942002755
 R3329: C14 + 1e+06 alpha_ob12_seg7[2] <= 1.0000158448802566e+06
 R3330: C16 + 1e+06 alpha_ob12_seg7[2] <= 1.0000158448802566e+06
 R3331: C15 + 1e+06 alpha_ob12_seg7[3] <= 1.0000139558002756e+06
 R3332: C17 + 1e+06 alpha_ob12_seg7[3] <= 1.0000139558002756e+06
 R3333: alpha_ob12_seg7[0] + alpha_ob12_seg7[1] + alpha_ob12_seg7[2]
   + alpha_ob12_seg7[3] >= 1
 R3334: - C14 + 1e+06 alpha_ob13_seg7[0] <= 999981.605149801
 R3335: - C16 + 1e+06 alpha_ob13_seg7[0] <= 999981.605149801
 R3336: - C15 + 1e+06 alpha_ob13_seg7[1] <= 999983.4942391404
 R3337: - C17 + 1e+06 alpha_ob13_seg7[1] <= 999983.4942391404
 R3338: C14 + 1e+06 alpha_ob13_seg7[2] <= 999986.4948498011
 R3339: C16 + 1e+06 alpha_ob13_seg7[2] <= 999986.4948498011
 R3340: C15 + 1e+06 alpha_ob13_seg7[3] <= 1.0000139558391405e+06
 R3341: C17 + 1e+06 alpha_ob13_seg7[3] <= 1.0000139558391405e+06
 R3342: alpha_ob13_seg7[0] + alpha_ob13_seg7[1] + alpha_ob13_seg7[2]
   + alpha_ob13_seg7[3] >= 1
 R3343: - C14 + 1e+06 alpha_ob14_seg7[0] <= 999997.6803502565
 R3344: - C16 + 1e+06 alpha_ob14_seg7[0] <= 999997.6803502565
 R3345: - C15 + 1e+06 alpha_ob14_seg7[1] <= 999972.7268002755
 R3346: - C17 + 1e+06 alpha_ob14_seg7[1] <= 999972.7268002755
 R3347: C14 + 1e+06 alpha_ob14_seg7[2] <= 999999.7696102565
 R3348: C16 + 1e+06 alpha_ob14_seg7[2] <= 999999.7696102565
 R3349: C15 + 1e+06 alpha_ob14_seg7[3] <= 1.0000141232002755e+06
 R3350: C17 + 1e+06 alpha_ob14_seg7[3] <= 1.0000141232002755e+06
 R3351: alpha_ob14_seg7[0] + alpha_ob14_seg7[1] + alpha_ob14_seg7[2]
   + alpha_ob14_seg7[3] >= 1
 R3352: - C14 + 1e+06 alpha_ob15_seg7[0] <= 1.0000038341502565e+06
 R3353: - C16 + 1e+06 alpha_ob15_seg7[0] <= 1.0000038341502565e+06
 R3354: - C15 + 1e+06 alpha_ob15_seg7[1] <= 999972.6012002755
 R3355: - C17 + 1e+06 alpha_ob15_seg7[1] <= 999972.6012002755
 R3356: C14 + 1e+06 alpha_ob15_seg7[2] <= 999993.6158102566
 R3357: C16 + 1e+06 alpha_ob15_seg7[2] <= 999993.6158102566
 R3358: C15 + 1e+06 alpha_ob15_seg7[3] <= 1.0000142488002755e+06
 R3359: C17 + 1e+06 alpha_ob15_seg7[3] <= 1.0000142488002755e+06
 R3360: alpha_ob15_seg7[0] + alpha_ob15_seg7[1] + alpha_ob15_seg7[2]
   + alpha_ob15_seg7[3] >= 1
 R3361: - C14 + 1e+06 alpha_ob16_seg7[0] <= 1.0000162874063333e+06
 R3362: - C16 + 1e+06 alpha_ob16_seg7[0] <= 1.0000162874063333e+06
 R3363: - C15 + 1e+06 alpha_ob16_seg7[1] <= 999982.9439602756
 R3364: - C17 + 1e+06 alpha_ob16_seg7[1] <= 999982.9439602756
 R3365: C14 + 1e+06 alpha_ob16_seg7[2] <= 999981.1624063333
 R3366: C16 + 1e+06 alpha_ob16_seg7[2] <= 999981.1624063333
 R3367: C15 + 1e+06 alpha_ob16_seg7[3] <= 999963.6560402757
 R3368: C17 + 1e+06 alpha_ob16_seg7[3] <= 999963.6560402757
 R3369: alpha_ob16_seg7[0] + alpha_ob16_seg7[1] + alpha_ob16_seg7[2]
   + alpha_ob16_seg7[3] >= 1
 R3370: - C14 + 1e+06 alpha_ob17_seg7[0] <= 999980.937509
 R3371: - C16 + 1e+06 alpha_ob17_seg7[0] <= 999980.937509
 R3372: - C15 + 1e+06 alpha_ob17_seg7[1] <= 1.000034894e+06
 R3373: - C17 + 1e+06 alpha_ob17_seg7[1] <= 1.000034894e+06
 R3374: C14 + 1e+06 alpha_ob17_seg7[2] <= 999981.162491
 R3375: C16 + 1e+06 alpha_ob17_seg7[2] <= 999981.162491
 R3376: C15 + 1e+06 alpha_ob17_seg7[3] <= 999962.556
 R3377: C17 + 1e+06 alpha_ob17_seg7[3] <= 999962.556
 R3378: alpha_ob17_seg7[0] + alpha_ob17_seg7[1] + alpha_ob17_seg7[2]
   + alpha_ob17_seg7[3] >= 1
 R3379: - C14 + 1e+06 alpha_ob18_seg7[0] <= 999990.2670277582
 R3380: - C16 + 1e+06 alpha_ob18_seg7[0] <= 999990.2670277582
 R3381: - C15 + 1e+06 alpha_ob18_seg7[1] <= 1.0000347702002756e+06
 R3382: - C17 + 1e+06 alpha_ob18_seg7[1] <= 1.0000347702002756e+06
 R3383: C14 + 1e+06 alpha_ob18_seg7[2] <= 1.0000071828877581e+06
 R3384: C16 + 1e+06 alpha_ob18_seg7[2] <= 1.0000071828877581e+06
 R3385: C15 + 1e+06 alpha_ob18_seg7[3] <= 999939.8298002756
 R3386: C17 + 1e+06 alpha_ob18_seg7[3] <= 999939.8298002756
 R3387: alpha_ob18_seg7[0] + alpha_ob18_seg7[1] + alpha_ob18_seg7[2]
   + alpha_ob18_seg7[3] >= 1
 R3388: - C14 + 1e+06 alpha_ob19_seg7[0] <= 999990.267069801
 R3389: - C16 + 1e+06 alpha_ob19_seg7[0] <= 999990.267069801
 R3390: - C15 + 1e+06 alpha_ob19_seg7[1] <= 1.0000576202069657e+06
 R3391: - C17 + 1e+06 alpha_ob19_seg7[1] <= 1.0000576202069657e+06
 R3392: C14 + 1e+06 alpha_ob19_seg7[2] <= 1.000002082929801e+06
 R3393: C16 + 1e+06 alpha_ob19_seg7[2] <= 1.000002082929801e+06
 R3394: C15 + 1e+06 alpha_ob19_seg7[3] <= 999939.8298069657
 R3395: C17 + 1e+06 alpha_ob19_seg7[3] <= 999939.8298069657
 R3396: alpha_ob19_seg7[0] + alpha_ob19_seg7[1] + alpha_ob19_seg7[2]
   + alpha_ob19_seg7[3] >= 1
 R3397: - C14 + 1e+06 alpha_ob20_seg7[0] <= 999991.073732
 R3398: - C16 + 1e+06 alpha_ob20_seg7[0] <= 999991.073732
 R3399: - C15 + 1e+06 alpha_ob20_seg7[1] <= 999952.2125
 R3400: - C17 + 1e+06 alpha_ob20_seg7[1] <= 999952.2125
 R3401: C14 + 1e+06 alpha_ob20_seg7[2] <= 999990.776268
 R3402: C16 + 1e+06 alpha_ob20_seg7[2] <= 999990.776268
 R3403: C15 + 1e+06 alpha_ob20_seg7[3] <= 1.0000452375e+06
 R3404: C17 + 1e+06 alpha_ob20_seg7[3] <= 1.0000452375e+06
 R3405: alpha_ob20_seg7[0] + alpha_ob20_seg7[1] + alpha_ob20_seg7[2]
   + alpha_ob20_seg7[3] >= 1
 R3406: - C14 + 1e+06 alpha_ob21_seg7[0] <= 999985.657099801
 R3407: - C16 + 1e+06 alpha_ob21_seg7[0] <= 999985.657099801
 R3408: - C15 + 1e+06 alpha_ob21_seg7[1] <= 1.0000576202063023e+06
 R3409: - C17 + 1e+06 alpha_ob21_seg7[1] <= 1.0000576202063023e+06
 R3410: C14 + 1e+06 alpha_ob21_seg7[2] <= 1.000007192899801e+06
 R3411: C16 + 1e+06 alpha_ob21_seg7[2] <= 1.000007192899801e+06
 R3412: C15 + 1e+06 alpha_ob21_seg7[3] <= 999939.8298063023
 R3413: C17 + 1e+06 alpha_ob21_seg7[3] <= 999939.8298063023
 R3414: alpha_ob21_seg7[0] + alpha_ob21_seg7[1] + alpha_ob21_seg7[2]
   + alpha_ob21_seg7[3] >= 1
 R3415: - C14 + 1e+06 alpha_ob22_seg7[0] <= 999994.771829801
 R3416: - C16 + 1e+06 alpha_ob22_seg7[0] <= 999994.771829801
 R3417: - C15 + 1e+06 alpha_ob22_seg7[1] <= 1.0000615983063023e+06
 R3418: - C17 + 1e+06 alpha_ob22_seg7[1] <= 1.0000615983063023e+06
 R3419: C14 + 1e+06 alpha_ob22_seg7[2] <= 999998.078169801
 R3420: C16 + 1e+06 alpha_ob22_seg7[2] <= 999998.078169801
 R3421: C15 + 1e+06 alpha_ob22_seg7[3] <= 999935.8517063023
 R3422: C17 + 1e+06 alpha_ob22_seg7[3] <= 999935.8517063023
 R3423: alpha_ob22_seg7[0] + alpha_ob22_seg7[1] + alpha_ob22_seg7[2]
   + alpha_ob22_seg7[3] >= 1
 R3424: - C14 + 1e+06 alpha_ob23_seg7[0] <= 999980.9374508709
 R3425: - C16 + 1e+06 alpha_ob23_seg7[0] <= 999980.9374508709
 R3426: - C15 + 1e+06 alpha_ob23_seg7[1] <= 1.0000348940002755e+06
 R3427: - C17 + 1e+06 alpha_ob23_seg7[1] <= 1.0000348940002755e+06
 R3428: C14 + 1e+06 alpha_ob23_seg7[2] <= 1.0000165124508708e+06
 R3429: C16 + 1e+06 alpha_ob23_seg7[2] <= 1.0000165124508708e+06
 R3430: C15 + 1e+06 alpha_ob23_seg7[3] <= 999935.9560002756
 R3431: C17 + 1e+06 alpha_ob23_seg7[3] <= 999935.9560002756
 R3432: alpha_ob23_seg7[0] + alpha_ob23_seg7[1] + alpha_ob23_seg7[2]
   + alpha_ob23_seg7[3] >= 1
 R3433: - C14 + 1e+06 alpha_ob24_seg7[0] <= 999980.937499801
 R3434: - C16 + 1e+06 alpha_ob24_seg7[0] <= 999980.937499801
 R3435: - C15 + 1e+06 alpha_ob24_seg7[1] <= 1.0000614940072973e+06
 R3436: - C17 + 1e+06 alpha_ob24_seg7[1] <= 1.0000614940072973e+06
 R3437: C14 + 1e+06 alpha_ob24_seg7[2] <= 1.000011162499801e+06
 R3438: C16 + 1e+06 alpha_ob24_seg7[2] <= 1.000011162499801e+06
 R3439: C15 + 1e+06 alpha_ob24_seg7[3] <= 999935.9560072974
 R3440: C17 + 1e+06 alpha_ob24_seg7[3] <= 999935.9560072974
 R3441: alpha_ob24_seg7[0] + alpha_ob24_seg7[1] + alpha_ob24_seg7[2]
   + alpha_ob24_seg7[3] >= 1
 R3442: - C14 + 1e+06 alpha_ob25_seg7[0] <= 999956.8394000001
 R3443: - C16 + 1e+06 alpha_ob25_seg7[0] <= 999956.8394000001
 R3444: - C15 + 1e+06 alpha_ob25_seg7[1] <= 999952.2463
 R3445: - C17 + 1e+06 alpha_ob25_seg7[1] <= 999952.2463
 R3446: C14 + 1e+06 alpha_ob25_seg7[2] <= 1.0000097606e+06
 R3447: C16 + 1e+06 alpha_ob25_seg7[2] <= 1.0000097606e+06
 R3448: C15 + 1e+06 alpha_ob25_seg7[3] <= 1.0000452037e+06
 R3449: C17 + 1e+06 alpha_ob25_seg7[3] <= 1.0000452037e+06
 R3450: alpha_ob25_seg7[0] + alpha_ob25_seg7[1] + alpha_ob25_seg7[2]
   + alpha_ob25_seg7[3] >= 1
 R3451: - C14 + 1e+06 alpha_ob26_seg7[0] <= 999980.9375348955
 R3452: - C16 + 1e+06 alpha_ob26_seg7[0] <= 999980.9375348955
 R3453: - C15 + 1e+06 alpha_ob26_seg7[1] <= 1.0000160439997246e+06
 R3454: - C17 + 1e+06 alpha_ob26_seg7[1] <= 1.0000160439997246e+06
 R3455: C14 + 1e+06 alpha_ob26_seg7[2] <= 1.0000165125348954e+06
 R3456: C16 + 1e+06 alpha_ob26_seg7[2] <= 1.0000165125348954e+06
 R3457: C15 + 1e+06 alpha_ob26_seg7[3] <= 999962.5559997245
 R3458: C17 + 1e+06 alpha_ob26_seg7[3] <= 999962.5559997245
 R3459: alpha_ob26_seg7[0] + alpha_ob26_seg7[1] + alpha_ob26_seg7[2]
   + alpha_ob26_seg7[3] >= 1
 R3460: - C14 + 1e+06 alpha_ob27_seg7[0] <= 999956.839399801
 R3461: - C16 + 1e+06 alpha_ob27_seg7[0] <= 999956.839399801
 R3462: - C15 + 1e+06 alpha_ob27_seg7[1] <= 999925.3963434525
 R3463: - C17 + 1e+06 alpha_ob27_seg7[1] <= 999925.3963434525
 R3464: C14 + 1e+06 alpha_ob27_seg7[2] <= 1.0000080105998011e+06
 R3465: C16 + 1e+06 alpha_ob27_seg7[2] <= 1.0000080105998011e+06
 R3466: C15 + 1e+06 alpha_ob27_seg7[3] <= 1.0000720537434525e+06
 R3467: C17 + 1e+06 alpha_ob27_seg7[3] <= 1.0000720537434525e+06
 R3468: alpha_ob27_seg7[0] + alpha_ob27_seg7[1] + alpha_ob27_seg7[2]
   + alpha_ob27_seg7[3] >= 1
 R3469: - C14 + 1e+06 alpha_ob28_seg7[0] <= 999980.9374998009
 R3470: - C16 + 1e+06 alpha_ob28_seg7[0] <= 999980.9374998009
 R3471: - C15 + 1e+06 alpha_ob28_seg7[1] <= 1.0000160440053071e+06
 R3472: - C17 + 1e+06 alpha_ob28_seg7[1] <= 1.0000160440053071e+06
 R3473: C14 + 1e+06 alpha_ob28_seg7[2] <= 1.000012662499801e+06
 R3474: C16 + 1e+06 alpha_ob28_seg7[2] <= 1.000012662499801e+06
 R3475: C15 + 1e+06 alpha_ob28_seg7[3] <= 999981.4060053072
 R3476: C17 + 1e+06 alpha_ob28_seg7[3] <= 999981.4060053072
 R3477: alpha_ob28_seg7[0] + alpha_ob28_seg7[1] + alpha_ob28_seg7[2]
   + alpha_ob28_seg7[3] >= 1
 R3478: - C14 + 1e+06 alpha_ob29_seg7[0] <= 999983.6053459151
 R3479: - C16 + 1e+06 alpha_ob29_seg7[0] <= 999983.6053459151
 R3480: - C15 + 1e+06 alpha_ob29_seg7[1] <= 999987.7722497246
 R3481: - C17 + 1e+06 alpha_ob29_seg7[1] <= 999987.7722497246
 R3482: C14 + 1e+06 alpha_ob29_seg7[2] <= 1.000013844745915e+06
 R3483: C16 + 1e+06 alpha_ob29_seg7[2] <= 1.000013844745915e+06
 R3484: C15 + 1e+06 alpha_ob29_seg7[3] <= 999984.8277497246
 R3485: C17 + 1e+06 alpha_ob29_seg7[3] <= 999984.8277497246
 R3486: alpha_ob29_seg7[0] + alpha_ob29_seg7[1] + alpha_ob29_seg7[2]
   + alpha_ob29_seg7[3] >= 1
 R3487: - C14 + 1e+06 alpha_ob30_seg7[0] <= 999958.8717898987
 R3488: - C16 + 1e+06 alpha_ob30_seg7[0] <= 999958.8717898987
 R3489: - C15 + 1e+06 alpha_ob30_seg7[1] <= 999987.7837602755
 R3490: - C17 + 1e+06 alpha_ob30_seg7[1] <= 999987.7837602755
 R3491: C14 + 1e+06 alpha_ob30_seg7[2] <= 1.0000385781898986e+06
 R3492: C16 + 1e+06 alpha_ob30_seg7[2] <= 1.0000385781898986e+06
 R3493: C15 + 1e+06 alpha_ob30_seg7[3] <= 1.0000043162402755e+06
 R3494: C17 + 1e+06 alpha_ob30_seg7[3] <= 1.0000043162402755e+06
 R3495: alpha_ob30_seg7[0] + alpha_ob30_seg7[1] + alpha_ob30_seg7[2]
   + alpha_ob30_seg7[3] >= 1
 R3496: - C14 + 1e+06 alpha_ob31_seg7[0] <= 999939.2718
 R3497: - C16 + 1e+06 alpha_ob31_seg7[0] <= 999939.2718
 R3498: - C15 + 1e+06 alpha_ob31_seg7[1] <= 999993.13376
 R3499: - C17 + 1e+06 alpha_ob31_seg7[1] <= 999993.13376
 R3500: C14 + 1e+06 alpha_ob31_seg7[2] <= 1.0000385782e+06
 R3501: C16 + 1e+06 alpha_ob31_seg7[2] <= 1.0000385782e+06
 R3502: C15 + 1e+06 alpha_ob31_seg7[3] <= 1.00000431624e+06
 R3503: C17 + 1e+06 alpha_ob31_seg7[3] <= 1.00000431624e+06
 R3504: alpha_ob31_seg7[0] + alpha_ob31_seg7[1] + alpha_ob31_seg7[2]
   + alpha_ob31_seg7[3] >= 1
 R3505: - C14 + 1e+06 alpha_ob32_seg7[0] <= 999939.2718101013
 R3506: - C16 + 1e+06 alpha_ob32_seg7[0] <= 999939.2718101013
 R3507: - C15 + 1e+06 alpha_ob32_seg7[1] <= 999987.7837597246
 R3508: - C17 + 1e+06 alpha_ob32_seg7[1] <= 999987.7837597246
 R3509: C14 + 1e+06 alpha_ob32_seg7[2] <= 1.0000581782101013e+06
 R3510: C16 + 1e+06 alpha_ob32_seg7[2] <= 1.0000581782101013e+06
 R3511: C15 + 1e+06 alpha_ob32_seg7[3] <= 1.0000043162397245e+06
 R3512: C17 + 1e+06 alpha_ob32_seg7[3] <= 1.0000043162397245e+06
 R3513: alpha_ob32_seg7[0] + alpha_ob32_seg7[1] + alpha_ob32_seg7[2]
   + alpha_ob32_seg7[3] >= 1
 R3514: - C14 + 1e+06 alpha_ob33_seg7[0] <= 999939.271799801
 R3515: - C16 + 1e+06 alpha_ob33_seg7[0] <= 999939.271799801
 R3516: - C15 + 1e+06 alpha_ob33_seg7[1] <= 999987.7838262042
 R3517: - C17 + 1e+06 alpha_ob33_seg7[1] <= 999987.7838262042
 R3518: C14 + 1e+06 alpha_ob33_seg7[2] <= 1.000038578199801e+06
 R3519: C16 + 1e+06 alpha_ob33_seg7[2] <= 1.000038578199801e+06
 R3520: C15 + 1e+06 alpha_ob33_seg7[3] <= 1.0000096662262043e+06
 R3521: C17 + 1e+06 alpha_ob33_seg7[3] <= 1.0000096662262043e+06
 R3522: alpha_ob33_seg7[0] + alpha_ob33_seg7[1] + alpha_ob33_seg7[2]
   + alpha_ob33_seg7[3] >= 1
 R3523: - C14 + 1e+06 alpha_ob34_seg7[0] <= 999926.0708
 R3524: - C16 + 1e+06 alpha_ob34_seg7[0] <= 999926.0708
 R3525: - C15 + 1e+06 alpha_ob34_seg7[1] <= 999983.7196
 R3526: - C17 + 1e+06 alpha_ob34_seg7[1] <= 999983.7196
 R3527: C14 + 1e+06 alpha_ob34_seg7[2] <= 1.0000250292e+06
 R3528: C16 + 1e+06 alpha_ob34_seg7[2] <= 1.0000250292e+06
 R3529: C15 + 1e+06 alpha_ob34_seg7[3] <= 1.0000137304e+06
 R3530: C17 + 1e+06 alpha_ob34_seg7[3] <= 1.0000137304e+06
 R3531: alpha_ob34_seg7[0] + alpha_ob34_seg7[1] + alpha_ob34_seg7[2]
   + alpha_ob34_seg7[3] >= 1
 R3532: - C14 + 1e+06 alpha_ob35_seg7[0] <= 999926.0708293857
 R3533: - C16 + 1e+06 alpha_ob35_seg7[0] <= 999926.0708293857
 R3534: - C15 + 1e+06 alpha_ob35_seg7[1] <= 999967.8695997245
 R3535: - C17 + 1e+06 alpha_ob35_seg7[1] <= 999967.8695997245
 R3536: C14 + 1e+06 alpha_ob35_seg7[2] <= 1.0000713792293856e+06
 R3537: C16 + 1e+06 alpha_ob35_seg7[2] <= 1.0000713792293856e+06
 R3538: C15 + 1e+06 alpha_ob35_seg7[3] <= 1.0000137303997246e+06
 R3539: C17 + 1e+06 alpha_ob35_seg7[3] <= 1.0000137303997246e+06
 R3540: alpha_ob35_seg7[0] + alpha_ob35_seg7[1] + alpha_ob35_seg7[2]
   + alpha_ob35_seg7[3] >= 1
 R3541: - C14 + 1e+06 alpha_ob36_seg7[0] <= 999972.6917508709
 R3542: - C16 + 1e+06 alpha_ob36_seg7[0] <= 999972.6917508709
 R3543: - C15 + 1e+06 alpha_ob36_seg7[1] <= 999957.1644002756
 R3544: - C17 + 1e+06 alpha_ob36_seg7[1] <= 999957.1644002756
 R3545: C14 + 1e+06 alpha_ob36_seg7[2] <= 1.0000247581508709e+06
 R3546: C16 + 1e+06 alpha_ob36_seg7[2] <= 1.0000247581508709e+06
 R3547: C15 + 1e+06 alpha_ob36_seg7[3] <= 1.0000136856002755e+06
 R3548: C17 + 1e+06 alpha_ob36_seg7[3] <= 1.0000136856002755e+06
 R3549: alpha_ob36_seg7[0] + alpha_ob36_seg7[1] + alpha_ob36_seg7[2]
   + alpha_ob36_seg7[3] >= 1
 R3550: - C14 + 1e+06 alpha_ob37_seg7[0] <= 999939.0009674003
 R3551: - C16 + 1e+06 alpha_ob37_seg7[0] <= 999939.0009674003
 R3552: - C15 + 1e+06 alpha_ob37_seg7[1] <= 999961.9516002756
 R3553: - C17 + 1e+06 alpha_ob37_seg7[1] <= 999961.9516002756
 R3554: C14 + 1e+06 alpha_ob37_seg7[2] <= 1.0000584489674004e+06
 R3555: C16 + 1e+06 alpha_ob37_seg7[2] <= 1.0000584489674004e+06
 R3556: C15 + 1e+06 alpha_ob37_seg7[3] <= 1.0000178984002755e+06
 R3557: C17 + 1e+06 alpha_ob37_seg7[3] <= 1.0000178984002755e+06
 R3558: alpha_ob37_seg7[0] + alpha_ob37_seg7[1] + alpha_ob37_seg7[2]
   + alpha_ob37_seg7[3] >= 1
 R3559: - C14 + 1e+06 alpha_ob38_seg7[0] <= 999994.9164998011
 R3560: - C16 + 1e+06 alpha_ob38_seg7[0] <= 999994.9164998011
 R3561: - C15 + 1e+06 alpha_ob38_seg7[1] <= 999925.4284782809
 R3562: - C17 + 1e+06 alpha_ob38_seg7[1] <= 999925.4284782809
 R3563: C14 + 1e+06 alpha_ob38_seg7[2] <= 999943.6834998011
 R3564: C16 + 1e+06 alpha_ob38_seg7[2] <= 999943.6834998011
 R3565: C15 + 1e+06 alpha_ob38_seg7[3] <= 1.0000720216782809e+06
 R3566: C17 + 1e+06 alpha_ob38_seg7[3] <= 1.0000720216782809e+06
 R3567: alpha_ob38_seg7[0] + alpha_ob38_seg7[1] + alpha_ob38_seg7[2]
   + alpha_ob38_seg7[3] >= 1
 R3568: - C14 + 1e+06 alpha_ob39_seg7[0] <= 999914.52327842
 R3569: - C16 + 1e+06 alpha_ob39_seg7[0] <= 999914.52327842
 R3570: - C15 + 1e+06 alpha_ob39_seg7[1] <= 999993.6552362755
 R3571: - C17 + 1e+06 alpha_ob39_seg7[1] <= 999993.6552362755
 R3572: C14 + 1e+06 alpha_ob39_seg7[2] <= 1.0000829266784199e+06
 R3573: C16 + 1e+06 alpha_ob39_seg7[2] <= 1.0000829266784199e+06
 R3574: C15 + 1e+06 alpha_ob39_seg7[3] <= 999992.1947642756
 R3575: C17 + 1e+06 alpha_ob39_seg7[3] <= 999992.1947642756
 R3576: alpha_ob39_seg7[0] + alpha_ob39_seg7[1] + alpha_ob39_seg7[2]
   + alpha_ob39_seg7[3] >= 1
 R3577: - C14 + 1e+06 alpha_ob40_seg7[0] <= 999912.265199801
 R3578: - C16 + 1e+06 alpha_ob40_seg7[0] <= 999912.265199801
 R3579: - C15 + 1e+06 alpha_ob40_seg7[1] <= 999987.7842112777
 R3580: - C17 + 1e+06 alpha_ob40_seg7[1] <= 999987.7842112777
 R3581: C14 + 1e+06 alpha_ob40_seg7[2] <= 1.000076834799801e+06
 R3582: C16 + 1e+06 alpha_ob40_seg7[2] <= 1.000076834799801e+06
 R3583: C15 + 1e+06 alpha_ob40_seg7[3] <= 1.0000096658112778e+06
 R3584: C17 + 1e+06 alpha_ob40_seg7[3] <= 1.0000096658112778e+06
 R3585: alpha_ob40_seg7[0] + alpha_ob40_seg7[1] + alpha_ob40_seg7[2]
   + alpha_ob40_seg7[3] >= 1
 R3586: - C14 + 1e+06 alpha_ob41_seg7[0] <= 999920.6152119379
 R3587: - C16 + 1e+06 alpha_ob41_seg7[0] <= 999920.6152119379
 R3588: - C15 + 1e+06 alpha_ob41_seg7[1] <= 999981.4341997246
 R3589: - C17 + 1e+06 alpha_ob41_seg7[1] <= 999981.4341997246
 R3590: C14 + 1e+06 alpha_ob41_seg7[2] <= 1.0000768348119379e+06
 R3591: C16 + 1e+06 alpha_ob41_seg7[2] <= 1.0000768348119379e+06
 R3592: C15 + 1e+06 alpha_ob41_seg7[3] <= 1.0000096657997245e+06
 R3593: C17 + 1e+06 alpha_ob41_seg7[3] <= 1.0000096657997245e+06
 R3594: alpha_ob41_seg7[0] + alpha_ob41_seg7[1] + alpha_ob41_seg7[2]
   + alpha_ob41_seg7[3] >= 1
 R3595: - C14 + 1e+06 alpha_ob42_seg7[0] <= 999919.2652
 R3596: - C16 + 1e+06 alpha_ob42_seg7[0] <= 999919.2652
 R3597: - C15 + 1e+06 alpha_ob42_seg7[1] <= 999981.4342
 R3598: - C17 + 1e+06 alpha_ob42_seg7[1] <= 999981.4342
 R3599: C14 + 1e+06 alpha_ob42_seg7[2] <= 1.0000768348e+06
 R3600: C16 + 1e+06 alpha_ob42_seg7[2] <= 1.0000768348e+06
 R3601: C15 + 1e+06 alpha_ob42_seg7[3] <= 1.0000160158000001e+06
 R3602: C17 + 1e+06 alpha_ob42_seg7[3] <= 1.0000160158000001e+06
 R3603: alpha_ob42_seg7[0] + alpha_ob42_seg7[1] + alpha_ob42_seg7[2]
   + alpha_ob42_seg7[3] >= 1
 R3604: - C14 + 1e+06 alpha_ob43_seg7[0] <= 999919.2652454559
 R3605: - C16 + 1e+06 alpha_ob43_seg7[0] <= 999919.2652454559
 R3606: - C15 + 1e+06 alpha_ob43_seg7[1] <= 999956.8341997246
 R3607: - C17 + 1e+06 alpha_ob43_seg7[1] <= 999956.8341997246
 R3608: C14 + 1e+06 alpha_ob43_seg7[2] <= 1.0000781848454559e+06
 R3609: C16 + 1e+06 alpha_ob43_seg7[2] <= 1.0000781848454559e+06
 R3610: C15 + 1e+06 alpha_ob43_seg7[3] <= 1.0000160157997246e+06
 R3611: C17 + 1e+06 alpha_ob43_seg7[3] <= 1.0000160157997246e+06
 R3612: alpha_ob43_seg7[0] + alpha_ob43_seg7[1] + alpha_ob43_seg7[2]
   + alpha_ob43_seg7[3] >= 1
 R3613: - C14 + 1e+06 alpha_ob44_seg7[0] <= 999991.83683
 R3614: - C16 + 1e+06 alpha_ob44_seg7[0] <= 999991.83683
 R3615: - C15 + 1e+06 alpha_ob44_seg7[1] <= 999940.589
 R3616: - C17 + 1e+06 alpha_ob44_seg7[1] <= 999940.589
 R3617: C14 + 1e+06 alpha_ob44_seg7[2] <= 1.00000401317e+06
 R3618: C16 + 1e+06 alpha_ob44_seg7[2] <= 1.00000401317e+06
 R3619: C15 + 1e+06 alpha_ob44_seg7[3] <= 1.000056861e+06
 R3620: C17 + 1e+06 alpha_ob44_seg7[3] <= 1.000056861e+06
 R3621: alpha_ob44_seg7[0] + alpha_ob44_seg7[1] + alpha_ob44_seg7[2]
   + alpha_ob44_seg7[3] >= 1
 R3622: - C14 + 1e+06 alpha_ob45_seg7[0] <= 999991.8368084199
 R3623: - C16 + 1e+06 alpha_ob45_seg7[0] <= 999991.8368084199
 R3624: - C15 + 1e+06 alpha_ob45_seg7[1] <= 999940.5890002755
 R3625: - C17 + 1e+06 alpha_ob45_seg7[1] <= 999940.5890002755
 R3626: C14 + 1e+06 alpha_ob45_seg7[2] <= 1.0000056131484199e+06
 R3627: C16 + 1e+06 alpha_ob45_seg7[2] <= 1.0000056131484199e+06
 R3628: C15 + 1e+06 alpha_ob45_seg7[3] <= 1.0000452610002755e+06
 R3629: C17 + 1e+06 alpha_ob45_seg7[3] <= 1.0000452610002755e+06
 R3630: alpha_ob45_seg7[0] + alpha_ob45_seg7[1] + alpha_ob45_seg7[2]
   + alpha_ob45_seg7[3] >= 1
 R3631: - C14 + 1e+06 alpha_ob46_seg7[0] <= 1.0000764021508709e+06
 R3632: - C16 + 1e+06 alpha_ob46_seg7[0] <= 1.0000764021508709e+06
 R3633: - C15 + 1e+06 alpha_ob46_seg7[1] <= 999957.1027002756
 R3634: - C17 + 1e+06 alpha_ob46_seg7[1] <= 999957.1027002756
 R3635: C14 + 1e+06 alpha_ob46_seg7[2] <= 999921.0477508709
 R3636: C16 + 1e+06 alpha_ob46_seg7[2] <= 999921.0477508709
 R3637: C15 + 1e+06 alpha_ob46_seg7[3] <= 1.0000137473002756e+06
 R3638: C17 + 1e+06 alpha_ob46_seg7[3] <= 1.0000137473002756e+06
 R3639: alpha_ob46_seg7[0] + alpha_ob46_seg7[1] + alpha_ob46_seg7[2]
   + alpha_ob46_seg7[3] >= 1
 R3640: - C14 + 1e+06 alpha_ob47_seg7[0] <= 1.0000445522e+06
 R3641: - C16 + 1e+06 alpha_ob47_seg7[0] <= 1.0000445522e+06
 R3642: - C15 + 1e+06 alpha_ob47_seg7[1] <= 999983.7027
 R3643: - C17 + 1e+06 alpha_ob47_seg7[1] <= 999983.7027
 R3644: C14 + 1e+06 alpha_ob47_seg7[2] <= 999921.0477999999
 R3645: C16 + 1e+06 alpha_ob47_seg7[2] <= 999921.0477999999
 R3646: C15 + 1e+06 alpha_ob47_seg7[3] <= 1.0000137473e+06
 R3647: C17 + 1e+06 alpha_ob47_seg7[3] <= 1.0000137473e+06
 R3648: alpha_ob47_seg7[0] + alpha_ob47_seg7[1] + alpha_ob47_seg7[2]
   + alpha_ob47_seg7[3] >= 1
 R3649: - C14 + 1e+06 alpha_ob48_seg7[0] <= 1.0000445522183661e+06
 R3650: - C16 + 1e+06 alpha_ob48_seg7[0] <= 1.0000445522183661e+06
 R3651: - C15 + 1e+06 alpha_ob48_seg7[1] <= 999973.8526997245
 R3652: - C17 + 1e+06 alpha_ob48_seg7[1] <= 999973.8526997245
 R3653: C14 + 1e+06 alpha_ob48_seg7[2] <= 999952.8978183661
 R3654: C16 + 1e+06 alpha_ob48_seg7[2] <= 999952.8978183661
 R3655: C15 + 1e+06 alpha_ob48_seg7[3] <= 1.0000137472997245e+06
 R3656: C17 + 1e+06 alpha_ob48_seg7[3] <= 1.0000137472997245e+06
 R3657: alpha_ob48_seg7[0] + alpha_ob48_seg7[1] + alpha_ob48_seg7[2]
   + alpha_ob48_seg7[3] >= 1
 R3658: - C14 + 1e+06 alpha_ob49_seg7[0] <= 1.000064332081634e+06
 R3659: - C16 + 1e+06 alpha_ob49_seg7[0] <= 1.000064332081634e+06
 R3660: - C15 + 1e+06 alpha_ob49_seg7[1] <= 999973.9401002756
 R3661: - C17 + 1e+06 alpha_ob49_seg7[1] <= 999973.9401002756
 R3662: C14 + 1e+06 alpha_ob49_seg7[2] <= 999933.1178816339
 R3663: C16 + 1e+06 alpha_ob49_seg7[2] <= 999933.1178816339
 R3664: C15 + 1e+06 alpha_ob49_seg7[3] <= 1.0000136599002755e+06
 R3665: C17 + 1e+06 alpha_ob49_seg7[3] <= 1.0000136599002755e+06
 R3666: alpha_ob49_seg7[0] + alpha_ob49_seg7[1] + alpha_ob49_seg7[2]
   + alpha_ob49_seg7[3] >= 1
 R3667: - C14 + 1e+06 alpha_ob50_seg7[0] <= 1.0000643321013774e+06
 R3668: - C16 + 1e+06 alpha_ob50_seg7[0] <= 1.0000643321013774e+06
 R3669: - C15 + 1e+06 alpha_ob50_seg7[1] <= 999983.1900997246
 R3670: - C17 + 1e+06 alpha_ob50_seg7[1] <= 999983.1900997246
 R3671: C14 + 1e+06 alpha_ob50_seg7[2] <= 999933.1179013775
 R3672: C16 + 1e+06 alpha_ob50_seg7[2] <= 999933.1179013775
 R3673: C15 + 1e+06 alpha_ob50_seg7[3] <= 1.0000136598997245e+06
 R3674: C17 + 1e+06 alpha_ob50_seg7[3] <= 1.0000136598997245e+06
 R3675: alpha_ob50_seg7[0] + alpha_ob50_seg7[1] + alpha_ob50_seg7[2]
   + alpha_ob50_seg7[3] >= 1
 R3676: - C16 + 1e+06 alpha_ob0_seg8[0] <= 1.0000284170499525e+06
 R3677: - C18 + 1e+06 alpha_ob0_seg8[0] <= 1.0000284170499525e+06
 R3678: - C17 + 1e+06 alpha_ob0_seg8[1] <= 999957.0843002755
 R3679: - C19 + 1e+06 alpha_ob0_seg8[1] <= 999957.0843002755
 R3680: C16 + 1e+06 alpha_ob0_seg8[2] <= 999969.0328499526
 R3681: C18 + 1e+06 alpha_ob0_seg8[2] <= 999969.0328499526
 R3682: C17 + 1e+06 alpha_ob0_seg8[3] <= 1.0000132657002755e+06
 R3683: C19 + 1e+06 alpha_ob0_seg8[3] <= 1.0000132657002755e+06
 R3684: alpha_ob0_seg8[0] + alpha_ob0_seg8[1] + alpha_ob0_seg8[2]
   + alpha_ob0_seg8[3] >= 1
 R3685: - C16 + 1e+06 alpha_ob1_seg8[0] <= 1.000028417099801e+06
 R3686: - C18 + 1e+06 alpha_ob1_seg8[0] <= 1.000028417099801e+06
 R3687: - C17 + 1e+06 alpha_ob1_seg8[1] <= 999984.1843155898
 R3688: - C19 + 1e+06 alpha_ob1_seg8[1] <= 999984.1843155898
 R3689: C16 + 1e+06 alpha_ob1_seg8[2] <= 999957.4328998009
 R3690: C18 + 1e+06 alpha_ob1_seg8[2] <= 999957.4328998009
 R3691: C17 + 1e+06 alpha_ob1_seg8[3] <= 1.0000132657155899e+06
 R3692: C19 + 1e+06 alpha_ob1_seg8[3] <= 1.0000132657155899e+06
 R3693: alpha_ob1_seg8[0] + alpha_ob1_seg8[1] + alpha_ob1_seg8[2]
   + alpha_ob1_seg8[3] >= 1
 R3694: - C16 + 1e+06 alpha_ob2_seg8[0] <= 1.0000400171500475e+06
 R3695: - C18 + 1e+06 alpha_ob2_seg8[0] <= 1.0000400171500475e+06
 R3696: - C17 + 1e+06 alpha_ob2_seg8[1] <= 999957.0842997245
 R3697: - C19 + 1e+06 alpha_ob2_seg8[1] <= 999957.0842997245
 R3698: C16 + 1e+06 alpha_ob2_seg8[2] <= 999957.4329500474
 R3699: C18 + 1e+06 alpha_ob2_seg8[2] <= 999957.4329500474
 R3700: C17 + 1e+06 alpha_ob2_seg8[3] <= 1.0000132656997246e+06
 R3701: C19 + 1e+06 alpha_ob2_seg8[3] <= 1.0000132656997246e+06
 R3702: alpha_ob2_seg8[0] + alpha_ob2_seg8[1] + alpha_ob2_seg8[2]
   + alpha_ob2_seg8[3] >= 1
 R3703: - C16 + 1e+06 alpha_ob3_seg8[0] <= 1.0000109161752058e+06
 R3704: - C18 + 1e+06 alpha_ob3_seg8[0] <= 1.0000109161752058e+06
 R3705: - C17 + 1e+06 alpha_ob3_seg8[1] <= 999970.2681002755
 R3706: - C19 + 1e+06 alpha_ob3_seg8[1] <= 999970.2681002755
 R3707: C16 + 1e+06 alpha_ob3_seg8[2] <= 999986.5337752058
 R3708: C18 + 1e+06 alpha_ob3_seg8[2] <= 999986.5337752058
 R3709: C17 + 1e+06 alpha_ob3_seg8[3] <= 1.0000138319002755e+06
 R3710: C19 + 1e+06 alpha_ob3_seg8[3] <= 1.0000138319002755e+06
 R3711: alpha_ob3_seg8[0] + alpha_ob3_seg8[1] + alpha_ob3_seg8[2]
   + alpha_ob3_seg8[3] >= 1
 R3712: - C16 + 1e+06 alpha_ob4_seg8[0] <= 1.000010916199801e+06
 R3713: - C18 + 1e+06 alpha_ob4_seg8[0] <= 1.000010916199801e+06
 R3714: - C17 + 1e+06 alpha_ob4_seg8[1] <= 999983.6181179117
 R3715: - C19 + 1e+06 alpha_ob4_seg8[1] <= 999983.6181179117
 R3716: C16 + 1e+06 alpha_ob4_seg8[2] <= 999973.183799801
 R3717: C18 + 1e+06 alpha_ob4_seg8[2] <= 999973.183799801
 R3718: C17 + 1e+06 alpha_ob4_seg8[3] <= 1.0000138319179118e+06
 R3719: C19 + 1e+06 alpha_ob4_seg8[3] <= 1.0000138319179118e+06
 R3720: alpha_ob4_seg8[0] + alpha_ob4_seg8[1] + alpha_ob4_seg8[2]
   + alpha_ob4_seg8[3] >= 1
 R3721: - C16 + 1e+06 alpha_ob5_seg8[0] <= 999992.49102
 R3722: - C18 + 1e+06 alpha_ob5_seg8[0] <= 999992.49102
 R3723: - C17 + 1e+06 alpha_ob5_seg8[1] <= 999957.1016000001
 R3724: - C19 + 1e+06 alpha_ob5_seg8[1] <= 999957.1016000001
 R3725: C16 + 1e+06 alpha_ob5_seg8[2] <= 999986.35898
 R3726: C18 + 1e+06 alpha_ob5_seg8[2] <= 999986.35898
 R3727: C17 + 1e+06 alpha_ob5_seg8[3] <= 1.0000403484e+06
 R3728: C19 + 1e+06 alpha_ob5_seg8[3] <= 1.0000403484e+06
 R3729: alpha_ob5_seg8[0] + alpha_ob5_seg8[1] + alpha_ob5_seg8[2]
   + alpha_ob5_seg8[3] >= 1
 R3730: - C16 + 1e+06 alpha_ob6_seg8[0] <= 999981.3909999999
 R3731: - C18 + 1e+06 alpha_ob6_seg8[0] <= 999981.3909999999
 R3732: - C17 + 1e+06 alpha_ob6_seg8[1] <= 999957.1016000001
 R3733: - C19 + 1e+06 alpha_ob6_seg8[1] <= 999957.1016000001
 R3734: C16 + 1e+06 alpha_ob6_seg8[2] <= 1.000004959e+06
 R3735: C18 + 1e+06 alpha_ob6_seg8[2] <= 1.000004959e+06
 R3736: C17 + 1e+06 alpha_ob6_seg8[3] <= 1.0000403484e+06
 R3737: C19 + 1e+06 alpha_ob6_seg8[3] <= 1.0000403484e+06
 R3738: alpha_ob6_seg8[0] + alpha_ob6_seg8[1] + alpha_ob6_seg8[2]
   + alpha_ob6_seg8[3] >= 1
 R3739: - C16 + 1e+06 alpha_ob7_seg8[0] <= 999981.3909825523
 R3740: - C18 + 1e+06 alpha_ob7_seg8[0] <= 999981.3909825523
 R3741: - C17 + 1e+06 alpha_ob7_seg8[1] <= 999957.1016002755
 R3742: - C19 + 1e+06 alpha_ob7_seg8[1] <= 999957.1016002755
 R3743: C16 + 1e+06 alpha_ob7_seg8[2] <= 1.0000160589825523e+06
 R3744: C18 + 1e+06 alpha_ob7_seg8[2] <= 1.0000160589825523e+06
 R3745: C17 + 1e+06 alpha_ob7_seg8[3] <= 1.0000309984002755e+06
 R3746: C19 + 1e+06 alpha_ob7_seg8[3] <= 1.0000309984002755e+06
 R3747: alpha_ob7_seg8[0] + alpha_ob7_seg8[1] + alpha_ob7_seg8[2]
   + alpha_ob7_seg8[3] >= 1
 R3748: - C16 + 1e+06 alpha_ob8_seg8[0] <= 999981.3867954084
 R3749: - C18 + 1e+06 alpha_ob8_seg8[0] <= 999981.3867954084
 R3750: - C17 + 1e+06 alpha_ob8_seg8[1] <= 999968.8595002755
 R3751: - C19 + 1e+06 alpha_ob8_seg8[1] <= 999968.8595002755
 R3752: C16 + 1e+06 alpha_ob8_seg8[2] <= 1.0000160631954084e+06
 R3753: C18 + 1e+06 alpha_ob8_seg8[2] <= 1.0000160631954084e+06
 R3754: C17 + 1e+06 alpha_ob8_seg8[3] <= 1.0000262405002755e+06
 R3755: C19 + 1e+06 alpha_ob8_seg8[3] <= 1.0000262405002755e+06
 R3756: alpha_ob8_seg8[0] + alpha_ob8_seg8[1] + alpha_ob8_seg8[2]
   + alpha_ob8_seg8[3] >= 1
 R3757: - C16 + 1e+06 alpha_ob9_seg8[0] <= 999981.386799801
 R3758: - C18 + 1e+06 alpha_ob9_seg8[0] <= 999981.386799801
 R3759: - C17 + 1e+06 alpha_ob9_seg8[1] <= 999971.2095013268
 R3760: - C19 + 1e+06 alpha_ob9_seg8[1] <= 999971.2095013268
 R3761: C16 + 1e+06 alpha_ob9_seg8[2] <= 1.0000152131998009e+06
 R3762: C18 + 1e+06 alpha_ob9_seg8[2] <= 1.0000152131998009e+06
 R3763: C17 + 1e+06 alpha_ob9_seg8[3] <= 1.0000262405013267e+06
 R3764: C19 + 1e+06 alpha_ob9_seg8[3] <= 1.0000262405013267e+06
 R3765: alpha_ob9_seg8[0] + alpha_ob9_seg8[1] + alpha_ob9_seg8[2]
   + alpha_ob9_seg8[3] >= 1
 R3766: - C16 + 1e+06 alpha_ob10_seg8[0] <= 999982.2368022958
 R3767: - C18 + 1e+06 alpha_ob10_seg8[0] <= 999982.2368022958
 R3768: - C17 + 1e+06 alpha_ob10_seg8[1] <= 999970.1094997245
 R3769: - C19 + 1e+06 alpha_ob10_seg8[1] <= 999970.1094997245
 R3770: C16 + 1e+06 alpha_ob10_seg8[2] <= 1.0000152132022957e+06
 R3771: C18 + 1e+06 alpha_ob10_seg8[2] <= 1.0000152132022957e+06
 R3772: C17 + 1e+06 alpha_ob10_seg8[3] <= 1.0000262404997245e+06
 R3773: C19 + 1e+06 alpha_ob10_seg8[3] <= 1.0000262404997245e+06
 R3774: alpha_ob10_seg8[0] + alpha_ob10_seg8[1] + alpha_ob10_seg8[2]
   + alpha_ob10_seg8[3] >= 1
 R3775: - C16 + 1e+06 alpha_ob11_seg8[0] <= 999982.236799801
 R3776: - C18 + 1e+06 alpha_ob11_seg8[0] <= 999982.236799801
 R3777: - C17 + 1e+06 alpha_ob11_seg8[1] <= 999970.1095291895
 R3778: - C19 + 1e+06 alpha_ob11_seg8[1] <= 999970.1095291895
 R3779: C16 + 1e+06 alpha_ob11_seg8[2] <= 999993.3631998011
 R3780: C18 + 1e+06 alpha_ob11_seg8[2] <= 999993.3631998011
 R3781: C17 + 1e+06 alpha_ob11_seg8[3] <= 1.0000273405291894e+06
 R3782: C19 + 1e+06 alpha_ob11_seg8[3] <= 1.0000273405291894e+06
 R3783: alpha_ob11_seg8[0] + alpha_ob11_seg8[1] + alpha_ob11_seg8[2]
   + alpha_ob11_seg8[3] >= 1
 R3784: - C16 + 1e+06 alpha_ob12_seg8[0] <= 999981.6050802565
 R3785: - C18 + 1e+06 alpha_ob12_seg8[0] <= 999981.6050802565
 R3786: - C17 + 1e+06 alpha_ob12_seg8[1] <= 999972.8942002755
 R3787: - C19 + 1e+06 alpha_ob12_seg8[1] <= 999972.8942002755
 R3788: C16 + 1e+06 alpha_ob12_seg8[2] <= 1.0000158448802566e+06
 R3789: C18 + 1e+06 alpha_ob12_seg8[2] <= 1.0000158448802566e+06
 R3790: C17 + 1e+06 alpha_ob12_seg8[3] <= 1.0000139558002756e+06
 R3791: C19 + 1e+06 alpha_ob12_seg8[3] <= 1.0000139558002756e+06
 R3792: alpha_ob12_seg8[0] + alpha_ob12_seg8[1] + alpha_ob12_seg8[2]
   + alpha_ob12_seg8[3] >= 1
 R3793: - C16 + 1e+06 alpha_ob13_seg8[0] <= 999981.605149801
 R3794: - C18 + 1e+06 alpha_ob13_seg8[0] <= 999981.605149801
 R3795: - C17 + 1e+06 alpha_ob13_seg8[1] <= 999983.4942391404
 R3796: - C19 + 1e+06 alpha_ob13_seg8[1] <= 999983.4942391404
 R3797: C16 + 1e+06 alpha_ob13_seg8[2] <= 999986.4948498011
 R3798: C18 + 1e+06 alpha_ob13_seg8[2] <= 999986.4948498011
 R3799: C17 + 1e+06 alpha_ob13_seg8[3] <= 1.0000139558391405e+06
 R3800: C19 + 1e+06 alpha_ob13_seg8[3] <= 1.0000139558391405e+06
 R3801: alpha_ob13_seg8[0] + alpha_ob13_seg8[1] + alpha_ob13_seg8[2]
   + alpha_ob13_seg8[3] >= 1
 R3802: - C16 + 1e+06 alpha_ob14_seg8[0] <= 999997.6803502565
 R3803: - C18 + 1e+06 alpha_ob14_seg8[0] <= 999997.6803502565
 R3804: - C17 + 1e+06 alpha_ob14_seg8[1] <= 999972.7268002755
 R3805: - C19 + 1e+06 alpha_ob14_seg8[1] <= 999972.7268002755
 R3806: C16 + 1e+06 alpha_ob14_seg8[2] <= 999999.7696102565
 R3807: C18 + 1e+06 alpha_ob14_seg8[2] <= 999999.7696102565
 R3808: C17 + 1e+06 alpha_ob14_seg8[3] <= 1.0000141232002755e+06
 R3809: C19 + 1e+06 alpha_ob14_seg8[3] <= 1.0000141232002755e+06
 R3810: alpha_ob14_seg8[0] + alpha_ob14_seg8[1] + alpha_ob14_seg8[2]
   + alpha_ob14_seg8[3] >= 1
 R3811: - C16 + 1e+06 alpha_ob15_seg8[0] <= 1.0000038341502565e+06
 R3812: - C18 + 1e+06 alpha_ob15_seg8[0] <= 1.0000038341502565e+06
 R3813: - C17 + 1e+06 alpha_ob15_seg8[1] <= 999972.6012002755
 R3814: - C19 + 1e+06 alpha_ob15_seg8[1] <= 999972.6012002755
 R3815: C16 + 1e+06 alpha_ob15_seg8[2] <= 999993.6158102566
 R3816: C18 + 1e+06 alpha_ob15_seg8[2] <= 999993.6158102566
 R3817: C17 + 1e+06 alpha_ob15_seg8[3] <= 1.0000142488002755e+06
 R3818: C19 + 1e+06 alpha_ob15_seg8[3] <= 1.0000142488002755e+06
 R3819: alpha_ob15_seg8[0] + alpha_ob15_seg8[1] + alpha_ob15_seg8[2]
   + alpha_ob15_seg8[3] >= 1
 R3820: - C16 + 1e+06 alpha_ob16_seg8[0] <= 1.0000162874063333e+06
 R3821: - C18 + 1e+06 alpha_ob16_seg8[0] <= 1.0000162874063333e+06
 R3822: - C17 + 1e+06 alpha_ob16_seg8[1] <= 999982.9439602756
 R3823: - C19 + 1e+06 alpha_ob16_seg8[1] <= 999982.9439602756
 R3824: C16 + 1e+06 alpha_ob16_seg8[2] <= 999981.1624063333
 R3825: C18 + 1e+06 alpha_ob16_seg8[2] <= 999981.1624063333
 R3826: C17 + 1e+06 alpha_ob16_seg8[3] <= 999963.6560402757
 R3827: C19 + 1e+06 alpha_ob16_seg8[3] <= 999963.6560402757
 R3828: alpha_ob16_seg8[0] + alpha_ob16_seg8[1] + alpha_ob16_seg8[2]
   + alpha_ob16_seg8[3] >= 1
 R3829: - C16 + 1e+06 alpha_ob17_seg8[0] <= 999980.937509
 R3830: - C18 + 1e+06 alpha_ob17_seg8[0] <= 999980.937509
 R3831: - C17 + 1e+06 alpha_ob17_seg8[1] <= 1.000034894e+06
 R3832: - C19 + 1e+06 alpha_ob17_seg8[1] <= 1.000034894e+06
 R3833: C16 + 1e+06 alpha_ob17_seg8[2] <= 999981.162491
 R3834: C18 + 1e+06 alpha_ob17_seg8[2] <= 999981.162491
 R3835: C17 + 1e+06 alpha_ob17_seg8[3] <= 999962.556
 R3836: C19 + 1e+06 alpha_ob17_seg8[3] <= 999962.556
 R3837: alpha_ob17_seg8[0] + alpha_ob17_seg8[1] + alpha_ob17_seg8[2]
   + alpha_ob17_seg8[3] >= 1
 R3838: - C16 + 1e+06 alpha_ob18_seg8[0] <= 999990.2670277582
 R3839: - C18 + 1e+06 alpha_ob18_seg8[0] <= 999990.2670277582
 R3840: - C17 + 1e+06 alpha_ob18_seg8[1] <= 1.0000347702002756e+06
 R3841: - C19 + 1e+06 alpha_ob18_seg8[1] <= 1.0000347702002756e+06
 R3842: C16 + 1e+06 alpha_ob18_seg8[2] <= 1.0000071828877581e+06
 R3843: C18 + 1e+06 alpha_ob18_seg8[2] <= 1.0000071828877581e+06
 R3844: C17 + 1e+06 alpha_ob18_seg8[3] <= 999939.8298002756
 R3845: C19 + 1e+06 alpha_ob18_seg8[3] <= 999939.8298002756
 R3846: alpha_ob18_seg8[0] + alpha_ob18_seg8[1] + alpha_ob18_seg8[2]
   + alpha_ob18_seg8[3] >= 1
 R3847: - C16 + 1e+06 alpha_ob19_seg8[0] <= 999990.267069801
 R3848: - C18 + 1e+06 alpha_ob19_seg8[0] <= 999990.267069801
 R3849: - C17 + 1e+06 alpha_ob19_seg8[1] <= 1.0000576202069657e+06
 R3850: - C19 + 1e+06 alpha_ob19_seg8[1] <= 1.0000576202069657e+06
 R3851: C16 + 1e+06 alpha_ob19_seg8[2] <= 1.000002082929801e+06
 R3852: C18 + 1e+06 alpha_ob19_seg8[2] <= 1.000002082929801e+06
 R3853: C17 + 1e+06 alpha_ob19_seg8[3] <= 999939.8298069657
 R3854: C19 + 1e+06 alpha_ob19_seg8[3] <= 999939.8298069657
 R3855: alpha_ob19_seg8[0] + alpha_ob19_seg8[1] + alpha_ob19_seg8[2]
   + alpha_ob19_seg8[3] >= 1
 R3856: - C16 + 1e+06 alpha_ob20_seg8[0] <= 999991.073732
 R3857: - C18 + 1e+06 alpha_ob20_seg8[0] <= 999991.073732
 R3858: - C17 + 1e+06 alpha_ob20_seg8[1] <= 999952.2125
 R3859: - C19 + 1e+06 alpha_ob20_seg8[1] <= 999952.2125
 R3860: C16 + 1e+06 alpha_ob20_seg8[2] <= 999990.776268
 R3861: C18 + 1e+06 alpha_ob20_seg8[2] <= 999990.776268
 R3862: C17 + 1e+06 alpha_ob20_seg8[3] <= 1.0000452375e+06
 R3863: C19 + 1e+06 alpha_ob20_seg8[3] <= 1.0000452375e+06
 R3864: alpha_ob20_seg8[0] + alpha_ob20_seg8[1] + alpha_ob20_seg8[2]
   + alpha_ob20_seg8[3] >= 1
 R3865: - C16 + 1e+06 alpha_ob21_seg8[0] <= 999985.657099801
 R3866: - C18 + 1e+06 alpha_ob21_seg8[0] <= 999985.657099801
 R3867: - C17 + 1e+06 alpha_ob21_seg8[1] <= 1.0000576202063023e+06
 R3868: - C19 + 1e+06 alpha_ob21_seg8[1] <= 1.0000576202063023e+06
 R3869: C16 + 1e+06 alpha_ob21_seg8[2] <= 1.000007192899801e+06
 R3870: C18 + 1e+06 alpha_ob21_seg8[2] <= 1.000007192899801e+06
 R3871: C17 + 1e+06 alpha_ob21_seg8[3] <= 999939.8298063023
 R3872: C19 + 1e+06 alpha_ob21_seg8[3] <= 999939.8298063023
 R3873: alpha_ob21_seg8[0] + alpha_ob21_seg8[1] + alpha_ob21_seg8[2]
   + alpha_ob21_seg8[3] >= 1
 R3874: - C16 + 1e+06 alpha_ob22_seg8[0] <= 999994.771829801
 R3875: - C18 + 1e+06 alpha_ob22_seg8[0] <= 999994.771829801
 R3876: - C17 + 1e+06 alpha_ob22_seg8[1] <= 1.0000615983063023e+06
 R3877: - C19 + 1e+06 alpha_ob22_seg8[1] <= 1.0000615983063023e+06
 R3878: C16 + 1e+06 alpha_ob22_seg8[2] <= 999998.078169801
 R3879: C18 + 1e+06 alpha_ob22_seg8[2] <= 999998.078169801
 R3880: C17 + 1e+06 alpha_ob22_seg8[3] <= 999935.8517063023
 R3881: C19 + 1e+06 alpha_ob22_seg8[3] <= 999935.8517063023
 R3882: alpha_ob22_seg8[0] + alpha_ob22_seg8[1] + alpha_ob22_seg8[2]
   + alpha_ob22_seg8[3] >= 1
 R3883: - C16 + 1e+06 alpha_ob23_seg8[0] <= 999980.9374508709
 R3884: - C18 + 1e+06 alpha_ob23_seg8[0] <= 999980.9374508709
 R3885: - C17 + 1e+06 alpha_ob23_seg8[1] <= 1.0000348940002755e+06
 R3886: - C19 + 1e+06 alpha_ob23_seg8[1] <= 1.0000348940002755e+06
 R3887: C16 + 1e+06 alpha_ob23_seg8[2] <= 1.0000165124508708e+06
 R3888: C18 + 1e+06 alpha_ob23_seg8[2] <= 1.0000165124508708e+06
 R3889: C17 + 1e+06 alpha_ob23_seg8[3] <= 999935.9560002756
 R3890: C19 + 1e+06 alpha_ob23_seg8[3] <= 999935.9560002756
 R3891: alpha_ob23_seg8[0] + alpha_ob23_seg8[1] + alpha_ob23_seg8[2]
   + alpha_ob23_seg8[3] >= 1
 R3892: - C16 + 1e+06 alpha_ob24_seg8[0] <= 999980.937499801
 R3893: - C18 + 1e+06 alpha_ob24_seg8[0] <= 999980.937499801
 R3894: - C17 + 1e+06 alpha_ob24_seg8[1] <= 1.0000614940072973e+06
 R3895: - C19 + 1e+06 alpha_ob24_seg8[1] <= 1.0000614940072973e+06
 R3896: C16 + 1e+06 alpha_ob24_seg8[2] <= 1.000011162499801e+06
 R3897: C18 + 1e+06 alpha_ob24_seg8[2] <= 1.000011162499801e+06
 R3898: C17 + 1e+06 alpha_ob24_seg8[3] <= 999935.9560072974
 R3899: C19 + 1e+06 alpha_ob24_seg8[3] <= 999935.9560072974
 R3900: alpha_ob24_seg8[0] + alpha_ob24_seg8[1] + alpha_ob24_seg8[2]
   + alpha_ob24_seg8[3] >= 1
 R3901: - C16 + 1e+06 alpha_ob25_seg8[0] <= 999956.8394000001
 R3902: - C18 + 1e+06 alpha_ob25_seg8[0] <= 999956.8394000001
 R3903: - C17 + 1e+06 alpha_ob25_seg8[1] <= 999952.2463
 R3904: - C19 + 1e+06 alpha_ob25_seg8[1] <= 999952.2463
 R3905: C16 + 1e+06 alpha_ob25_seg8[2] <= 1.0000097606e+06
 R3906: C18 + 1e+06 alpha_ob25_seg8[2] <= 1.0000097606e+06
 R3907: C17 + 1e+06 alpha_ob25_seg8[3] <= 1.0000452037e+06
 R3908: C19 + 1e+06 alpha_ob25_seg8[3] <= 1.0000452037e+06
 R3909: alpha_ob25_seg8[0] + alpha_ob25_seg8[1] + alpha_ob25_seg8[2]
   + alpha_ob25_seg8[3] >= 1
 R3910: - C16 + 1e+06 alpha_ob26_seg8[0] <= 999980.9375348955
 R3911: - C18 + 1e+06 alpha_ob26_seg8[0] <= 999980.9375348955
 R3912: - C17 + 1e+06 alpha_ob26_seg8[1] <= 1.0000160439997246e+06
 R3913: - C19 + 1e+06 alpha_ob26_seg8[1] <= 1.0000160439997246e+06
 R3914: C16 + 1e+06 alpha_ob26_seg8[2] <= 1.0000165125348954e+06
 R3915: C18 + 1e+06 alpha_ob26_seg8[2] <= 1.0000165125348954e+06
 R3916: C17 + 1e+06 alpha_ob26_seg8[3] <= 999962.5559997245
 R3917: C19 + 1e+06 alpha_ob26_seg8[3] <= 999962.5559997245
 R3918: alpha_ob26_seg8[0] + alpha_ob26_seg8[1] + alpha_ob26_seg8[2]
   + alpha_ob26_seg8[3] >= 1
 R3919: - C16 + 1e+06 alpha_ob27_seg8[0] <= 999956.839399801
 R3920: - C18 + 1e+06 alpha_ob27_seg8[0] <= 999956.839399801
 R3921: - C17 + 1e+06 alpha_ob27_seg8[1] <= 999925.3963434525
 R3922: - C19 + 1e+06 alpha_ob27_seg8[1] <= 999925.3963434525
 R3923: C16 + 1e+06 alpha_ob27_seg8[2] <= 1.0000080105998011e+06
 R3924: C18 + 1e+06 alpha_ob27_seg8[2] <= 1.0000080105998011e+06
 R3925: C17 + 1e+06 alpha_ob27_seg8[3] <= 1.0000720537434525e+06
 R3926: C19 + 1e+06 alpha_ob27_seg8[3] <= 1.0000720537434525e+06
 R3927: alpha_ob27_seg8[0] + alpha_ob27_seg8[1] + alpha_ob27_seg8[2]
   + alpha_ob27_seg8[3] >= 1
 R3928: - C16 + 1e+06 alpha_ob28_seg8[0] <= 999980.9374998009
 R3929: - C18 + 1e+06 alpha_ob28_seg8[0] <= 999980.9374998009
 R3930: - C17 + 1e+06 alpha_ob28_seg8[1] <= 1.0000160440053071e+06
 R3931: - C19 + 1e+06 alpha_ob28_seg8[1] <= 1.0000160440053071e+06
 R3932: C16 + 1e+06 alpha_ob28_seg8[2] <= 1.000012662499801e+06
 R3933: C18 + 1e+06 alpha_ob28_seg8[2] <= 1.000012662499801e+06
 R3934: C17 + 1e+06 alpha_ob28_seg8[3] <= 999981.4060053072
 R3935: C19 + 1e+06 alpha_ob28_seg8[3] <= 999981.4060053072
 R3936: alpha_ob28_seg8[0] + alpha_ob28_seg8[1] + alpha_ob28_seg8[2]
   + alpha_ob28_seg8[3] >= 1
 R3937: - C16 + 1e+06 alpha_ob29_seg8[0] <= 999983.6053459151
 R3938: - C18 + 1e+06 alpha_ob29_seg8[0] <= 999983.6053459151
 R3939: - C17 + 1e+06 alpha_ob29_seg8[1] <= 999987.7722497246
 R3940: - C19 + 1e+06 alpha_ob29_seg8[1] <= 999987.7722497246
 R3941: C16 + 1e+06 alpha_ob29_seg8[2] <= 1.000013844745915e+06
 R3942: C18 + 1e+06 alpha_ob29_seg8[2] <= 1.000013844745915e+06
 R3943: C17 + 1e+06 alpha_ob29_seg8[3] <= 999984.8277497246
 R3944: C19 + 1e+06 alpha_ob29_seg8[3] <= 999984.8277497246
 R3945: alpha_ob29_seg8[0] + alpha_ob29_seg8[1] + alpha_ob29_seg8[2]
   + alpha_ob29_seg8[3] >= 1
 R3946: - C16 + 1e+06 alpha_ob30_seg8[0] <= 999958.8717898987
 R3947: - C18 + 1e+06 alpha_ob30_seg8[0] <= 999958.8717898987
 R3948: - C17 + 1e+06 alpha_ob30_seg8[1] <= 999987.7837602755
 R3949: - C19 + 1e+06 alpha_ob30_seg8[1] <= 999987.7837602755
 R3950: C16 + 1e+06 alpha_ob30_seg8[2] <= 1.0000385781898986e+06
 R3951: C18 + 1e+06 alpha_ob30_seg8[2] <= 1.0000385781898986e+06
 R3952: C17 + 1e+06 alpha_ob30_seg8[3] <= 1.0000043162402755e+06
 R3953: C19 + 1e+06 alpha_ob30_seg8[3] <= 1.0000043162402755e+06
 R3954: alpha_ob30_seg8[0] + alpha_ob30_seg8[1] + alpha_ob30_seg8[2]
   + alpha_ob30_seg8[3] >= 1
 R3955: - C16 + 1e+06 alpha_ob31_seg8[0] <= 999939.2718
 R3956: - C18 + 1e+06 alpha_ob31_seg8[0] <= 999939.2718
 R3957: - C17 + 1e+06 alpha_ob31_seg8[1] <= 999993.13376
 R3958: - C19 + 1e+06 alpha_ob31_seg8[1] <= 999993.13376
 R3959: C16 + 1e+06 alpha_ob31_seg8[2] <= 1.0000385782e+06
 R3960: C18 + 1e+06 alpha_ob31_seg8[2] <= 1.0000385782e+06
 R3961: C17 + 1e+06 alpha_ob31_seg8[3] <= 1.00000431624e+06
 R3962: C19 + 1e+06 alpha_ob31_seg8[3] <= 1.00000431624e+06
 R3963: alpha_ob31_seg8[0] + alpha_ob31_seg8[1] + alpha_ob31_seg8[2]
   + alpha_ob31_seg8[3] >= 1
 R3964: - C16 + 1e+06 alpha_ob32_seg8[0] <= 999939.2718101013
 R3965: - C18 + 1e+06 alpha_ob32_seg8[0] <= 999939.2718101013
 R3966: - C17 + 1e+06 alpha_ob32_seg8[1] <= 999987.7837597246
 R3967: - C19 + 1e+06 alpha_ob32_seg8[1] <= 999987.7837597246
 R3968: C16 + 1e+06 alpha_ob32_seg8[2] <= 1.0000581782101013e+06
 R3969: C18 + 1e+06 alpha_ob32_seg8[2] <= 1.0000581782101013e+06
 R3970: C17 + 1e+06 alpha_ob32_seg8[3] <= 1.0000043162397245e+06
 R3971: C19 + 1e+06 alpha_ob32_seg8[3] <= 1.0000043162397245e+06
 R3972: alpha_ob32_seg8[0] + alpha_ob32_seg8[1] + alpha_ob32_seg8[2]
   + alpha_ob32_seg8[3] >= 1
 R3973: - C16 + 1e+06 alpha_ob33_seg8[0] <= 999939.271799801
 R3974: - C18 + 1e+06 alpha_ob33_seg8[0] <= 999939.271799801
 R3975: - C17 + 1e+06 alpha_ob33_seg8[1] <= 999987.7838262042
 R3976: - C19 + 1e+06 alpha_ob33_seg8[1] <= 999987.7838262042
 R3977: C16 + 1e+06 alpha_ob33_seg8[2] <= 1.000038578199801e+06
 R3978: C18 + 1e+06 alpha_ob33_seg8[2] <= 1.000038578199801e+06
 R3979: C17 + 1e+06 alpha_ob33_seg8[3] <= 1.0000096662262043e+06
 R3980: C19 + 1e+06 alpha_ob33_seg8[3] <= 1.0000096662262043e+06
 R3981: alpha_ob33_seg8[0] + alpha_ob33_seg8[1] + alpha_ob33_seg8[2]
   + alpha_ob33_seg8[3] >= 1
 R3982: - C16 + 1e+06 alpha_ob34_seg8[0] <= 999926.0708
 R3983: - C18 + 1e+06 alpha_ob34_seg8[0] <= 999926.0708
 R3984: - C17 + 1e+06 alpha_ob34_seg8[1] <= 999983.7196
 R3985: - C19 + 1e+06 alpha_ob34_seg8[1] <= 999983.7196
 R3986: C16 + 1e+06 alpha_ob34_seg8[2] <= 1.0000250292e+06
 R3987: C18 + 1e+06 alpha_ob34_seg8[2] <= 1.0000250292e+06
 R3988: C17 + 1e+06 alpha_ob34_seg8[3] <= 1.0000137304e+06
 R3989: C19 + 1e+06 alpha_ob34_seg8[3] <= 1.0000137304e+06
 R3990: alpha_ob34_seg8[0] + alpha_ob34_seg8[1] + alpha_ob34_seg8[2]
   + alpha_ob34_seg8[3] >= 1
 R3991: - C16 + 1e+06 alpha_ob35_seg8[0] <= 999926.0708293857
 R3992: - C18 + 1e+06 alpha_ob35_seg8[0] <= 999926.0708293857
 R3993: - C17 + 1e+06 alpha_ob35_seg8[1] <= 999967.8695997245
 R3994: - C19 + 1e+06 alpha_ob35_seg8[1] <= 999967.8695997245
 R3995: C16 + 1e+06 alpha_ob35_seg8[2] <= 1.0000713792293856e+06
 R3996: C18 + 1e+06 alpha_ob35_seg8[2] <= 1.0000713792293856e+06
 R3997: C17 + 1e+06 alpha_ob35_seg8[3] <= 1.0000137303997246e+06
 R3998: C19 + 1e+06 alpha_ob35_seg8[3] <= 1.0000137303997246e+06
 R3999: alpha_ob35_seg8[0] + alpha_ob35_seg8[1] + alpha_ob35_seg8[2]
   + alpha_ob35_seg8[3] >= 1
 R4000: - C16 + 1e+06 alpha_ob36_seg8[0] <= 999972.6917508709
 R4001: - C18 + 1e+06 alpha_ob36_seg8[0] <= 999972.6917508709
 R4002: - C17 + 1e+06 alpha_ob36_seg8[1] <= 999957.1644002756
 R4003: - C19 + 1e+06 alpha_ob36_seg8[1] <= 999957.1644002756
 R4004: C16 + 1e+06 alpha_ob36_seg8[2] <= 1.0000247581508709e+06
 R4005: C18 + 1e+06 alpha_ob36_seg8[2] <= 1.0000247581508709e+06
 R4006: C17 + 1e+06 alpha_ob36_seg8[3] <= 1.0000136856002755e+06
 R4007: C19 + 1e+06 alpha_ob36_seg8[3] <= 1.0000136856002755e+06
 R4008: alpha_ob36_seg8[0] + alpha_ob36_seg8[1] + alpha_ob36_seg8[2]
   + alpha_ob36_seg8[3] >= 1
 R4009: - C16 + 1e+06 alpha_ob37_seg8[0] <= 999939.0009674003
 R4010: - C18 + 1e+06 alpha_ob37_seg8[0] <= 999939.0009674003
 R4011: - C17 + 1e+06 alpha_ob37_seg8[1] <= 999961.9516002756
 R4012: - C19 + 1e+06 alpha_ob37_seg8[1] <= 999961.9516002756
 R4013: C16 + 1e+06 alpha_ob37_seg8[2] <= 1.0000584489674004e+06
 R4014: C18 + 1e+06 alpha_ob37_seg8[2] <= 1.0000584489674004e+06
 R4015: C17 + 1e+06 alpha_ob37_seg8[3] <= 1.0000178984002755e+06
 R4016: C19 + 1e+06 alpha_ob37_seg8[3] <= 1.0000178984002755e+06
 R4017: alpha_ob37_seg8[0] + alpha_ob37_seg8[1] + alpha_ob37_seg8[2]
   + alpha_ob37_seg8[3] >= 1
 R4018: - C16 + 1e+06 alpha_ob38_seg8[0] <= 999994.9164998011
 R4019: - C18 + 1e+06 alpha_ob38_seg8[0] <= 999994.9164998011
 R4020: - C17 + 1e+06 alpha_ob38_seg8[1] <= 999925.4284782809
 R4021: - C19 + 1e+06 alpha_ob38_seg8[1] <= 999925.4284782809
 R4022: C16 + 1e+06 alpha_ob38_seg8[2] <= 999943.6834998011
 R4023: C18 + 1e+06 alpha_ob38_seg8[2] <= 999943.6834998011
 R4024: C17 + 1e+06 alpha_ob38_seg8[3] <= 1.0000720216782809e+06
 R4025: C19 + 1e+06 alpha_ob38_seg8[3] <= 1.0000720216782809e+06
 R4026: alpha_ob38_seg8[0] + alpha_ob38_seg8[1] + alpha_ob38_seg8[2]
   + alpha_ob38_seg8[3] >= 1
 R4027: - C16 + 1e+06 alpha_ob39_seg8[0] <= 999914.52327842
 R4028: - C18 + 1e+06 alpha_ob39_seg8[0] <= 999914.52327842
 R4029: - C17 + 1e+06 alpha_ob39_seg8[1] <= 999993.6552362755
 R4030: - C19 + 1e+06 alpha_ob39_seg8[1] <= 999993.6552362755
 R4031: C16 + 1e+06 alpha_ob39_seg8[2] <= 1.0000829266784199e+06
 R4032: C18 + 1e+06 alpha_ob39_seg8[2] <= 1.0000829266784199e+06
 R4033: C17 + 1e+06 alpha_ob39_seg8[3] <= 999992.1947642756
 R4034: C19 + 1e+06 alpha_ob39_seg8[3] <= 999992.1947642756
 R4035: alpha_ob39_seg8[0] + alpha_ob39_seg8[1] + alpha_ob39_seg8[2]
   + alpha_ob39_seg8[3] >= 1
 R4036: - C16 + 1e+06 alpha_ob40_seg8[0] <= 999912.265199801
 R4037: - C18 + 1e+06 alpha_ob40_seg8[0] <= 999912.265199801
 R4038: - C17 + 1e+06 alpha_ob40_seg8[1] <= 999987.7842112777
 R4039: - C19 + 1e+06 alpha_ob40_seg8[1] <= 999987.7842112777
 R4040: C16 + 1e+06 alpha_ob40_seg8[2] <= 1.000076834799801e+06
 R4041: C18 + 1e+06 alpha_ob40_seg8[2] <= 1.000076834799801e+06
 R4042: C17 + 1e+06 alpha_ob40_seg8[3] <= 1.0000096658112778e+06
 R4043: C19 + 1e+06 alpha_ob40_seg8[3] <= 1.0000096658112778e+06
 R4044: alpha_ob40_seg8[0] + alpha_ob40_seg8[1] + alpha_ob40_seg8[2]
   + alpha_ob40_seg8[3] >= 1
 R4045: - C16 + 1e+06 alpha_ob41_seg8[0] <= 999920.6152119379
 R4046: - C18 + 1e+06 alpha_ob41_seg8[0] <= 999920.6152119379
 R4047: - C17 + 1e+06 alpha_ob41_seg8[1] <= 999981.4341997246
 R4048: - C19 + 1e+06 alpha_ob41_seg8[1] <= 999981.4341997246
 R4049: C16 + 1e+06 alpha_ob41_seg8[2] <= 1.0000768348119379e+06
 R4050: C18 + 1e+06 alpha_ob41_seg8[2] <= 1.0000768348119379e+06
 R4051: C17 + 1e+06 alpha_ob41_seg8[3] <= 1.0000096657997245e+06
 R4052: C19 + 1e+06 alpha_ob41_seg8[3] <= 1.0000096657997245e+06
 R4053: alpha_ob41_seg8[0] + alpha_ob41_seg8[1] + alpha_ob41_seg8[2]
   + alpha_ob41_seg8[3] >= 1
 R4054: - C16 + 1e+06 alpha_ob42_seg8[0] <= 999919.2652
 R4055: - C18 + 1e+06 alpha_ob42_seg8[0] <= 999919.2652
 R4056: - C17 + 1e+06 alpha_ob42_seg8[1] <= 999981.4342
 R4057: - C19 + 1e+06 alpha_ob42_seg8[1] <= 999981.4342
 R4058: C16 + 1e+06 alpha_ob42_seg8[2] <= 1.0000768348e+06
 R4059: C18 + 1e+06 alpha_ob42_seg8[2] <= 1.0000768348e+06
 R4060: C17 + 1e+06 alpha_ob42_seg8[3] <= 1.0000160158000001e+06
 R4061: C19 + 1e+06 alpha_ob42_seg8[3] <= 1.0000160158000001e+06
 R4062: alpha_ob42_seg8[0] + alpha_ob42_seg8[1] + alpha_ob42_seg8[2]
   + alpha_ob42_seg8[3] >= 1
 R4063: - C16 + 1e+06 alpha_ob43_seg8[0] <= 999919.2652454559
 R4064: - C18 + 1e+06 alpha_ob43_seg8[0] <= 999919.2652454559
 R4065: - C17 + 1e+06 alpha_ob43_seg8[1] <= 999956.8341997246
 R4066: - C19 + 1e+06 alpha_ob43_seg8[1] <= 999956.8341997246
 R4067: C16 + 1e+06 alpha_ob43_seg8[2] <= 1.0000781848454559e+06
 R4068: C18 + 1e+06 alpha_ob43_seg8[2] <= 1.0000781848454559e+06
 R4069: C17 + 1e+06 alpha_ob43_seg8[3] <= 1.0000160157997246e+06
 R4070: C19 + 1e+06 alpha_ob43_seg8[3] <= 1.0000160157997246e+06
 R4071: alpha_ob43_seg8[0] + alpha_ob43_seg8[1] + alpha_ob43_seg8[2]
   + alpha_ob43_seg8[3] >= 1
 R4072: - C16 + 1e+06 alpha_ob44_seg8[0] <= 999991.83683
 R4073: - C18 + 1e+06 alpha_ob44_seg8[0] <= 999991.83683
 R4074: - C17 + 1e+06 alpha_ob44_seg8[1] <= 999940.589
 R4075: - C19 + 1e+06 alpha_ob44_seg8[1] <= 999940.589
 R4076: C16 + 1e+06 alpha_ob44_seg8[2] <= 1.00000401317e+06
 R4077: C18 + 1e+06 alpha_ob44_seg8[2] <= 1.00000401317e+06
 R4078: C17 + 1e+06 alpha_ob44_seg8[3] <= 1.000056861e+06
 R4079: C19 + 1e+06 alpha_ob44_seg8[3] <= 1.000056861e+06
 R4080: alpha_ob44_seg8[0] + alpha_ob44_seg8[1] + alpha_ob44_seg8[2]
   + alpha_ob44_seg8[3] >= 1
 R4081: - C16 + 1e+06 alpha_ob45_seg8[0] <= 999991.8368084199
 R4082: - C18 + 1e+06 alpha_ob45_seg8[0] <= 999991.8368084199
 R4083: - C17 + 1e+06 alpha_ob45_seg8[1] <= 999940.5890002755
 R4084: - C19 + 1e+06 alpha_ob45_seg8[1] <= 999940.5890002755
 R4085: C16 + 1e+06 alpha_ob45_seg8[2] <= 1.0000056131484199e+06
 R4086: C18 + 1e+06 alpha_ob45_seg8[2] <= 1.0000056131484199e+06
 R4087: C17 + 1e+06 alpha_ob45_seg8[3] <= 1.0000452610002755e+06
 R4088: C19 + 1e+06 alpha_ob45_seg8[3] <= 1.0000452610002755e+06
 R4089: alpha_ob45_seg8[0] + alpha_ob45_seg8[1] + alpha_ob45_seg8[2]
   + alpha_ob45_seg8[3] >= 1
 R4090: - C16 + 1e+06 alpha_ob46_seg8[0] <= 1.0000764021508709e+06
 R4091: - C18 + 1e+06 alpha_ob46_seg8[0] <= 1.0000764021508709e+06
 R4092: - C17 + 1e+06 alpha_ob46_seg8[1] <= 999957.1027002756
 R4093: - C19 + 1e+06 alpha_ob46_seg8[1] <= 999957.1027002756
 R4094: C16 + 1e+06 alpha_ob46_seg8[2] <= 999921.0477508709
 R4095: C18 + 1e+06 alpha_ob46_seg8[2] <= 999921.0477508709
 R4096: C17 + 1e+06 alpha_ob46_seg8[3] <= 1.0000137473002756e+06
 R4097: C19 + 1e+06 alpha_ob46_seg8[3] <= 1.0000137473002756e+06
 R4098: alpha_ob46_seg8[0] + alpha_ob46_seg8[1] + alpha_ob46_seg8[2]
   + alpha_ob46_seg8[3] >= 1
 R4099: - C16 + 1e+06 alpha_ob47_seg8[0] <= 1.0000445522e+06
 R4100: - C18 + 1e+06 alpha_ob47_seg8[0] <= 1.0000445522e+06
 R4101: - C17 + 1e+06 alpha_ob47_seg8[1] <= 999983.7027
 R4102: - C19 + 1e+06 alpha_ob47_seg8[1] <= 999983.7027
 R4103: C16 + 1e+06 alpha_ob47_seg8[2] <= 999921.0477999999
 R4104: C18 + 1e+06 alpha_ob47_seg8[2] <= 999921.0477999999
 R4105: C17 + 1e+06 alpha_ob47_seg8[3] <= 1.0000137473e+06
 R4106: C19 + 1e+06 alpha_ob47_seg8[3] <= 1.0000137473e+06
 R4107: alpha_ob47_seg8[0] + alpha_ob47_seg8[1] + alpha_ob47_seg8[2]
   + alpha_ob47_seg8[3] >= 1
 R4108: - C16 + 1e+06 alpha_ob48_seg8[0] <= 1.0000445522183661e+06
 R4109: - C18 + 1e+06 alpha_ob48_seg8[0] <= 1.0000445522183661e+06
 R4110: - C17 + 1e+06 alpha_ob48_seg8[1] <= 999973.8526997245
 R4111: - C19 + 1e+06 alpha_ob48_seg8[1] <= 999973.8526997245
 R4112: C16 + 1e+06 alpha_ob48_seg8[2] <= 999952.8978183661
 R4113: C18 + 1e+06 alpha_ob48_seg8[2] <= 999952.8978183661
 R4114: C17 + 1e+06 alpha_ob48_seg8[3] <= 1.0000137472997245e+06
 R4115: C19 + 1e+06 alpha_ob48_seg8[3] <= 1.0000137472997245e+06
 R4116: alpha_ob48_seg8[0] + alpha_ob48_seg8[1] + alpha_ob48_seg8[2]
   + alpha_ob48_seg8[3] >= 1
 R4117: - C16 + 1e+06 alpha_ob49_seg8[0] <= 1.000064332081634e+06
 R4118: - C18 + 1e+06 alpha_ob49_seg8[0] <= 1.000064332081634e+06
 R4119: - C17 + 1e+06 alpha_ob49_seg8[1] <= 999973.9401002756
 R4120: - C19 + 1e+06 alpha_ob49_seg8[1] <= 999973.9401002756
 R4121: C16 + 1e+06 alpha_ob49_seg8[2] <= 999933.1178816339
 R4122: C18 + 1e+06 alpha_ob49_seg8[2] <= 999933.1178816339
 R4123: C17 + 1e+06 alpha_ob49_seg8[3] <= 1.0000136599002755e+06
 R4124: C19 + 1e+06 alpha_ob49_seg8[3] <= 1.0000136599002755e+06
 R4125: alpha_ob49_seg8[0] + alpha_ob49_seg8[1] + alpha_ob49_seg8[2]
   + alpha_ob49_seg8[3] >= 1
 R4126: - C16 + 1e+06 alpha_ob50_seg8[0] <= 1.0000643321013774e+06
 R4127: - C18 + 1e+06 alpha_ob50_seg8[0] <= 1.0000643321013774e+06
 R4128: - C17 + 1e+06 alpha_ob50_seg8[1] <= 999983.1900997246
 R4129: - C19 + 1e+06 alpha_ob50_seg8[1] <= 999983.1900997246
 R4130: C16 + 1e+06 alpha_ob50_seg8[2] <= 999933.1179013775
 R4131: C18 + 1e+06 alpha_ob50_seg8[2] <= 999933.1179013775
 R4132: C17 + 1e+06 alpha_ob50_seg8[3] <= 1.0000136598997245e+06
 R4133: C19 + 1e+06 alpha_ob50_seg8[3] <= 1.0000136598997245e+06
 R4134: alpha_ob50_seg8[0] + alpha_ob50_seg8[1] + alpha_ob50_seg8[2]
   + alpha_ob50_seg8[3] >= 1
 R4135: - C18 + 1e+06 alpha_ob0_seg9[0] <= 1.0000284170499525e+06
 R4136: - C20 + 1e+06 alpha_ob0_seg9[0] <= 1.0000284170499525e+06
 R4137: - C19 + 1e+06 alpha_ob0_seg9[1] <= 999957.0843002755
 R4138: - C21 + 1e+06 alpha_ob0_seg9[1] <= 999957.0843002755
 R4139: C18 + 1e+06 alpha_ob0_seg9[2] <= 999969.0328499526
 R4140: C20 + 1e+06 alpha_ob0_seg9[2] <= 999969.0328499526
 R4141: C19 + 1e+06 alpha_ob0_seg9[3] <= 1.0000132657002755e+06
 R4142: C21 + 1e+06 alpha_ob0_seg9[3] <= 1.0000132657002755e+06
 R4143: alpha_ob0_seg9[0] + alpha_ob0_seg9[1] + alpha_ob0_seg9[2]
   + alpha_ob0_seg9[3] >= 1
 R4144: - C18 + 1e+06 alpha_ob1_seg9[0] <= 1.000028417099801e+06
 R4145: - C20 + 1e+06 alpha_ob1_seg9[0] <= 1.000028417099801e+06
 R4146: - C19 + 1e+06 alpha_ob1_seg9[1] <= 999984.1843155898
 R4147: - C21 + 1e+06 alpha_ob1_seg9[1] <= 999984.1843155898
 R4148: C18 + 1e+06 alpha_ob1_seg9[2] <= 999957.4328998009
 R4149: C20 + 1e+06 alpha_ob1_seg9[2] <= 999957.4328998009
 R4150: C19 + 1e+06 alpha_ob1_seg9[3] <= 1.0000132657155899e+06
 R4151: C21 + 1e+06 alpha_ob1_seg9[3] <= 1.0000132657155899e+06
 R4152: alpha_ob1_seg9[0] + alpha_ob1_seg9[1] + alpha_ob1_seg9[2]
   + alpha_ob1_seg9[3] >= 1
 R4153: - C18 + 1e+06 alpha_ob2_seg9[0] <= 1.0000400171500475e+06
 R4154: - C20 + 1e+06 alpha_ob2_seg9[0] <= 1.0000400171500475e+06
 R4155: - C19 + 1e+06 alpha_ob2_seg9[1] <= 999957.0842997245
 R4156: - C21 + 1e+06 alpha_ob2_seg9[1] <= 999957.0842997245
 R4157: C18 + 1e+06 alpha_ob2_seg9[2] <= 999957.4329500474
 R4158: C20 + 1e+06 alpha_ob2_seg9[2] <= 999957.4329500474
 R4159: C19 + 1e+06 alpha_ob2_seg9[3] <= 1.0000132656997246e+06
 R4160: C21 + 1e+06 alpha_ob2_seg9[3] <= 1.0000132656997246e+06
 R4161: alpha_ob2_seg9[0] + alpha_ob2_seg9[1] + alpha_ob2_seg9[2]
   + alpha_ob2_seg9[3] >= 1
 R4162: - C18 + 1e+06 alpha_ob3_seg9[0] <= 1.0000109161752058e+06
 R4163: - C20 + 1e+06 alpha_ob3_seg9[0] <= 1.0000109161752058e+06
 R4164: - C19 + 1e+06 alpha_ob3_seg9[1] <= 999970.2681002755
 R4165: - C21 + 1e+06 alpha_ob3_seg9[1] <= 999970.2681002755
 R4166: C18 + 1e+06 alpha_ob3_seg9[2] <= 999986.5337752058
 R4167: C20 + 1e+06 alpha_ob3_seg9[2] <= 999986.5337752058
 R4168: C19 + 1e+06 alpha_ob3_seg9[3] <= 1.0000138319002755e+06
 R4169: C21 + 1e+06 alpha_ob3_seg9[3] <= 1.0000138319002755e+06
 R4170: alpha_ob3_seg9[0] + alpha_ob3_seg9[1] + alpha_ob3_seg9[2]
   + alpha_ob3_seg9[3] >= 1
 R4171: - C18 + 1e+06 alpha_ob4_seg9[0] <= 1.000010916199801e+06
 R4172: - C20 + 1e+06 alpha_ob4_seg9[0] <= 1.000010916199801e+06
 R4173: - C19 + 1e+06 alpha_ob4_seg9[1] <= 999983.6181179117
 R4174: - C21 + 1e+06 alpha_ob4_seg9[1] <= 999983.6181179117
 R4175: C18 + 1e+06 alpha_ob4_seg9[2] <= 999973.183799801
 R4176: C20 + 1e+06 alpha_ob4_seg9[2] <= 999973.183799801
 R4177: C19 + 1e+06 alpha_ob4_seg9[3] <= 1.0000138319179118e+06
 R4178: C21 + 1e+06 alpha_ob4_seg9[3] <= 1.0000138319179118e+06
 R4179: alpha_ob4_seg9[0] + alpha_ob4_seg9[1] + alpha_ob4_seg9[2]
   + alpha_ob4_seg9[3] >= 1
 R4180: - C18 + 1e+06 alpha_ob5_seg9[0] <= 999992.49102
 R4181: - C20 + 1e+06 alpha_ob5_seg9[0] <= 999992.49102
 R4182: - C19 + 1e+06 alpha_ob5_seg9[1] <= 999957.1016000001
 R4183: - C21 + 1e+06 alpha_ob5_seg9[1] <= 999957.1016000001
 R4184: C18 + 1e+06 alpha_ob5_seg9[2] <= 999986.35898
 R4185: C20 + 1e+06 alpha_ob5_seg9[2] <= 999986.35898
 R4186: C19 + 1e+06 alpha_ob5_seg9[3] <= 1.0000403484e+06
 R4187: C21 + 1e+06 alpha_ob5_seg9[3] <= 1.0000403484e+06
 R4188: alpha_ob5_seg9[0] + alpha_ob5_seg9[1] + alpha_ob5_seg9[2]
   + alpha_ob5_seg9[3] >= 1
 R4189: - C18 + 1e+06 alpha_ob6_seg9[0] <= 999981.3909999999
 R4190: - C20 + 1e+06 alpha_ob6_seg9[0] <= 999981.3909999999
 R4191: - C19 + 1e+06 alpha_ob6_seg9[1] <= 999957.1016000001
 R4192: - C21 + 1e+06 alpha_ob6_seg9[1] <= 999957.1016000001
 R4193: C18 + 1e+06 alpha_ob6_seg9[2] <= 1.000004959e+06
 R4194: C20 + 1e+06 alpha_ob6_seg9[2] <= 1.000004959e+06
 R4195: C19 + 1e+06 alpha_ob6_seg9[3] <= 1.0000403484e+06
 R4196: C21 + 1e+06 alpha_ob6_seg9[3] <= 1.0000403484e+06
 R4197: alpha_ob6_seg9[0] + alpha_ob6_seg9[1] + alpha_ob6_seg9[2]
   + alpha_ob6_seg9[3] >= 1
 R4198: - C18 + 1e+06 alpha_ob7_seg9[0] <= 999981.3909825523
 R4199: - C20 + 1e+06 alpha_ob7_seg9[0] <= 999981.3909825523
 R4200: - C19 + 1e+06 alpha_ob7_seg9[1] <= 999957.1016002755
 R4201: - C21 + 1e+06 alpha_ob7_seg9[1] <= 999957.1016002755
 R4202: C18 + 1e+06 alpha_ob7_seg9[2] <= 1.0000160589825523e+06
 R4203: C20 + 1e+06 alpha_ob7_seg9[2] <= 1.0000160589825523e+06
 R4204: C19 + 1e+06 alpha_ob7_seg9[3] <= 1.0000309984002755e+06
 R4205: C21 + 1e+06 alpha_ob7_seg9[3] <= 1.0000309984002755e+06
 R4206: alpha_ob7_seg9[0] + alpha_ob7_seg9[1] + alpha_ob7_seg9[2]
   + alpha_ob7_seg9[3] >= 1
 R4207: - C18 + 1e+06 alpha_ob8_seg9[0] <= 999981.3867954084
 R4208: - C20 + 1e+06 alpha_ob8_seg9[0] <= 999981.3867954084
 R4209: - C19 + 1e+06 alpha_ob8_seg9[1] <= 999968.8595002755
 R4210: - C21 + 1e+06 alpha_ob8_seg9[1] <= 999968.8595002755
 R4211: C18 + 1e+06 alpha_ob8_seg9[2] <= 1.0000160631954084e+06
 R4212: C20 + 1e+06 alpha_ob8_seg9[2] <= 1.0000160631954084e+06
 R4213: C19 + 1e+06 alpha_ob8_seg9[3] <= 1.0000262405002755e+06
 R4214: C21 + 1e+06 alpha_ob8_seg9[3] <= 1.0000262405002755e+06
 R4215: alpha_ob8_seg9[0] + alpha_ob8_seg9[1] + alpha_ob8_seg9[2]
   + alpha_ob8_seg9[3] >= 1
 R4216: - C18 + 1e+06 alpha_ob9_seg9[0] <= 999981.386799801
 R4217: - C20 + 1e+06 alpha_ob9_seg9[0] <= 999981.386799801
 R4218: - C19 + 1e+06 alpha_ob9_seg9[1] <= 999971.2095013268
 R4219: - C21 + 1e+06 alpha_ob9_seg9[1] <= 999971.2095013268
 R4220: C18 + 1e+06 alpha_ob9_seg9[2] <= 1.0000152131998009e+06
 R4221: C20 + 1e+06 alpha_ob9_seg9[2] <= 1.0000152131998009e+06
 R4222: C19 + 1e+06 alpha_ob9_seg9[3] <= 1.0000262405013267e+06
 R4223: C21 + 1e+06 alpha_ob9_seg9[3] <= 1.0000262405013267e+06
 R4224: alpha_ob9_seg9[0] + alpha_ob9_seg9[1] + alpha_ob9_seg9[2]
   + alpha_ob9_seg9[3] >= 1
 R4225: - C18 + 1e+06 alpha_ob10_seg9[0] <= 999982.2368022958
 R4226: - C20 + 1e+06 alpha_ob10_seg9[0] <= 999982.2368022958
 R4227: - C19 + 1e+06 alpha_ob10_seg9[1] <= 999970.1094997245
 R4228: - C21 + 1e+06 alpha_ob10_seg9[1] <= 999970.1094997245
 R4229: C18 + 1e+06 alpha_ob10_seg9[2] <= 1.0000152132022957e+06
 R4230: C20 + 1e+06 alpha_ob10_seg9[2] <= 1.0000152132022957e+06
 R4231: C19 + 1e+06 alpha_ob10_seg9[3] <= 1.0000262404997245e+06
 R4232: C21 + 1e+06 alpha_ob10_seg9[3] <= 1.0000262404997245e+06
 R4233: alpha_ob10_seg9[0] + alpha_ob10_seg9[1] + alpha_ob10_seg9[2]
   + alpha_ob10_seg9[3] >= 1
 R4234: - C18 + 1e+06 alpha_ob11_seg9[0] <= 999982.236799801
 R4235: - C20 + 1e+06 alpha_ob11_seg9[0] <= 999982.236799801
 R4236: - C19 + 1e+06 alpha_ob11_seg9[1] <= 999970.1095291895
 R4237: - C21 + 1e+06 alpha_ob11_seg9[1] <= 999970.1095291895
 R4238: C18 + 1e+06 alpha_ob11_seg9[2] <= 999993.3631998011
 R4239: C20 + 1e+06 alpha_ob11_seg9[2] <= 999993.3631998011
 R4240: C19 + 1e+06 alpha_ob11_seg9[3] <= 1.0000273405291894e+06
 R4241: C21 + 1e+06 alpha_ob11_seg9[3] <= 1.0000273405291894e+06
 R4242: alpha_ob11_seg9[0] + alpha_ob11_seg9[1] + alpha_ob11_seg9[2]
   + alpha_ob11_seg9[3] >= 1
 R4243: - C18 + 1e+06 alpha_ob12_seg9[0] <= 999981.6050802565
 R4244: - C20 + 1e+06 alpha_ob12_seg9[0] <= 999981.6050802565
 R4245: - C19 + 1e+06 alpha_ob12_seg9[1] <= 999972.8942002755
 R4246: - C21 + 1e+06 alpha_ob12_seg9[1] <= 999972.8942002755
 R4247: C18 + 1e+06 alpha_ob12_seg9[2] <= 1.0000158448802566e+06
 R4248: C20 + 1e+06 alpha_ob12_seg9[2] <= 1.0000158448802566e+06
 R4249: C19 + 1e+06 alpha_ob12_seg9[3] <= 1.0000139558002756e+06
 R4250: C21 + 1e+06 alpha_ob12_seg9[3] <= 1.0000139558002756e+06
 R4251: alpha_ob12_seg9[0] + alpha_ob12_seg9[1] + alpha_ob12_seg9[2]
   + alpha_ob12_seg9[3] >= 1
 R4252: - C18 + 1e+06 alpha_ob13_seg9[0] <= 999981.605149801
 R4253: - C20 + 1e+06 alpha_ob13_seg9[0] <= 999981.605149801
 R4254: - C19 + 1e+06 alpha_ob13_seg9[1] <= 999983.4942391404
 R4255: - C21 + 1e+06 alpha_ob13_seg9[1] <= 999983.4942391404
 R4256: C18 + 1e+06 alpha_ob13_seg9[2] <= 999986.4948498011
 R4257: C20 + 1e+06 alpha_ob13_seg9[2] <= 999986.4948498011
 R4258: C19 + 1e+06 alpha_ob13_seg9[3] <= 1.0000139558391405e+06
 R4259: C21 + 1e+06 alpha_ob13_seg9[3] <= 1.0000139558391405e+06
 R4260: alpha_ob13_seg9[0] + alpha_ob13_seg9[1] + alpha_ob13_seg9[2]
   + alpha_ob13_seg9[3] >= 1
 R4261: - C18 + 1e+06 alpha_ob14_seg9[0] <= 999997.6803502565
 R4262: - C20 + 1e+06 alpha_ob14_seg9[0] <= 999997.6803502565
 R4263: - C19 + 1e+06 alpha_ob14_seg9[1] <= 999972.7268002755
 R4264: - C21 + 1e+06 alpha_ob14_seg9[1] <= 999972.7268002755
 R4265: C18 + 1e+06 alpha_ob14_seg9[2] <= 999999.7696102565
 R4266: C20 + 1e+06 alpha_ob14_seg9[2] <= 999999.7696102565
 R4267: C19 + 1e+06 alpha_ob14_seg9[3] <= 1.0000141232002755e+06
 R4268: C21 + 1e+06 alpha_ob14_seg9[3] <= 1.0000141232002755e+06
 R4269: alpha_ob14_seg9[0] + alpha_ob14_seg9[1] + alpha_ob14_seg9[2]
   + alpha_ob14_seg9[3] >= 1
 R4270: - C18 + 1e+06 alpha_ob15_seg9[0] <= 1.0000038341502565e+06
 R4271: - C20 + 1e+06 alpha_ob15_seg9[0] <= 1.0000038341502565e+06
 R4272: - C19 + 1e+06 alpha_ob15_seg9[1] <= 999972.6012002755
 R4273: - C21 + 1e+06 alpha_ob15_seg9[1] <= 999972.6012002755
 R4274: C18 + 1e+06 alpha_ob15_seg9[2] <= 999993.6158102566
 R4275: C20 + 1e+06 alpha_ob15_seg9[2] <= 999993.6158102566
 R4276: C19 + 1e+06 alpha_ob15_seg9[3] <= 1.0000142488002755e+06
 R4277: C21 + 1e+06 alpha_ob15_seg9[3] <= 1.0000142488002755e+06
 R4278: alpha_ob15_seg9[0] + alpha_ob15_seg9[1] + alpha_ob15_seg9[2]
   + alpha_ob15_seg9[3] >= 1
 R4279: - C18 + 1e+06 alpha_ob16_seg9[0] <= 1.0000162874063333e+06
 R4280: - C20 + 1e+06 alpha_ob16_seg9[0] <= 1.0000162874063333e+06
 R4281: - C19 + 1e+06 alpha_ob16_seg9[1] <= 999982.9439602756
 R4282: - C21 + 1e+06 alpha_ob16_seg9[1] <= 999982.9439602756
 R4283: C18 + 1e+06 alpha_ob16_seg9[2] <= 999981.1624063333
 R4284: C20 + 1e+06 alpha_ob16_seg9[2] <= 999981.1624063333
 R4285: C19 + 1e+06 alpha_ob16_seg9[3] <= 999963.6560402757
 R4286: C21 + 1e+06 alpha_ob16_seg9[3] <= 999963.6560402757
 R4287: alpha_ob16_seg9[0] + alpha_ob16_seg9[1] + alpha_ob16_seg9[2]
   + alpha_ob16_seg9[3] >= 1
 R4288: - C18 + 1e+06 alpha_ob17_seg9[0] <= 999980.937509
 R4289: - C20 + 1e+06 alpha_ob17_seg9[0] <= 999980.937509
 R4290: - C19 + 1e+06 alpha_ob17_seg9[1] <= 1.000034894e+06
 R4291: - C21 + 1e+06 alpha_ob17_seg9[1] <= 1.000034894e+06
 R4292: C18 + 1e+06 alpha_ob17_seg9[2] <= 999981.162491
 R4293: C20 + 1e+06 alpha_ob17_seg9[2] <= 999981.162491
 R4294: C19 + 1e+06 alpha_ob17_seg9[3] <= 999962.556
 R4295: C21 + 1e+06 alpha_ob17_seg9[3] <= 999962.556
 R4296: alpha_ob17_seg9[0] + alpha_ob17_seg9[1] + alpha_ob17_seg9[2]
   + alpha_ob17_seg9[3] >= 1
 R4297: - C18 + 1e+06 alpha_ob18_seg9[0] <= 999990.2670277582
 R4298: - C20 + 1e+06 alpha_ob18_seg9[0] <= 999990.2670277582
 R4299: - C19 + 1e+06 alpha_ob18_seg9[1] <= 1.0000347702002756e+06
 R4300: - C21 + 1e+06 alpha_ob18_seg9[1] <= 1.0000347702002756e+06
 R4301: C18 + 1e+06 alpha_ob18_seg9[2] <= 1.0000071828877581e+06
 R4302: C20 + 1e+06 alpha_ob18_seg9[2] <= 1.0000071828877581e+06
 R4303: C19 + 1e+06 alpha_ob18_seg9[3] <= 999939.8298002756
 R4304: C21 + 1e+06 alpha_ob18_seg9[3] <= 999939.8298002756
 R4305: alpha_ob18_seg9[0] + alpha_ob18_seg9[1] + alpha_ob18_seg9[2]
   + alpha_ob18_seg9[3] >= 1
 R4306: - C18 + 1e+06 alpha_ob19_seg9[0] <= 999990.267069801
 R4307: - C20 + 1e+06 alpha_ob19_seg9[0] <= 999990.267069801
 R4308: - C19 + 1e+06 alpha_ob19_seg9[1] <= 1.0000576202069657e+06
 R4309: - C21 + 1e+06 alpha_ob19_seg9[1] <= 1.0000576202069657e+06
 R4310: C18 + 1e+06 alpha_ob19_seg9[2] <= 1.000002082929801e+06
 R4311: C20 + 1e+06 alpha_ob19_seg9[2] <= 1.000002082929801e+06
 R4312: C19 + 1e+06 alpha_ob19_seg9[3] <= 999939.8298069657
 R4313: C21 + 1e+06 alpha_ob19_seg9[3] <= 999939.8298069657
 R4314: alpha_ob19_seg9[0] + alpha_ob19_seg9[1] + alpha_ob19_seg9[2]
   + alpha_ob19_seg9[3] >= 1
 R4315: - C18 + 1e+06 alpha_ob20_seg9[0] <= 999991.073732
 R4316: - C20 + 1e+06 alpha_ob20_seg9[0] <= 999991.073732
 R4317: - C19 + 1e+06 alpha_ob20_seg9[1] <= 999952.2125
 R4318: - C21 + 1e+06 alpha_ob20_seg9[1] <= 999952.2125
 R4319: C18 + 1e+06 alpha_ob20_seg9[2] <= 999990.776268
 R4320: C20 + 1e+06 alpha_ob20_seg9[2] <= 999990.776268
 R4321: C19 + 1e+06 alpha_ob20_seg9[3] <= 1.0000452375e+06
 R4322: C21 + 1e+06 alpha_ob20_seg9[3] <= 1.0000452375e+06
 R4323: alpha_ob20_seg9[0] + alpha_ob20_seg9[1] + alpha_ob20_seg9[2]
   + alpha_ob20_seg9[3] >= 1
 R4324: - C18 + 1e+06 alpha_ob21_seg9[0] <= 999985.657099801
 R4325: - C20 + 1e+06 alpha_ob21_seg9[0] <= 999985.657099801
 R4326: - C19 + 1e+06 alpha_ob21_seg9[1] <= 1.0000576202063023e+06
 R4327: - C21 + 1e+06 alpha_ob21_seg9[1] <= 1.0000576202063023e+06
 R4328: C18 + 1e+06 alpha_ob21_seg9[2] <= 1.000007192899801e+06
 R4329: C20 + 1e+06 alpha_ob21_seg9[2] <= 1.000007192899801e+06
 R4330: C19 + 1e+06 alpha_ob21_seg9[3] <= 999939.8298063023
 R4331: C21 + 1e+06 alpha_ob21_seg9[3] <= 999939.8298063023
 R4332: alpha_ob21_seg9[0] + alpha_ob21_seg9[1] + alpha_ob21_seg9[2]
   + alpha_ob21_seg9[3] >= 1
 R4333: - C18 + 1e+06 alpha_ob22_seg9[0] <= 999994.771829801
 R4334: - C20 + 1e+06 alpha_ob22_seg9[0] <= 999994.771829801
 R4335: - C19 + 1e+06 alpha_ob22_seg9[1] <= 1.0000615983063023e+06
 R4336: - C21 + 1e+06 alpha_ob22_seg9[1] <= 1.0000615983063023e+06
 R4337: C18 + 1e+06 alpha_ob22_seg9[2] <= 999998.078169801
 R4338: C20 + 1e+06 alpha_ob22_seg9[2] <= 999998.078169801
 R4339: C19 + 1e+06 alpha_ob22_seg9[3] <= 999935.8517063023
 R4340: C21 + 1e+06 alpha_ob22_seg9[3] <= 999935.8517063023
 R4341: alpha_ob22_seg9[0] + alpha_ob22_seg9[1] + alpha_ob22_seg9[2]
   + alpha_ob22_seg9[3] >= 1
 R4342: - C18 + 1e+06 alpha_ob23_seg9[0] <= 999980.9374508709
 R4343: - C20 + 1e+06 alpha_ob23_seg9[0] <= 999980.9374508709
 R4344: - C19 + 1e+06 alpha_ob23_seg9[1] <= 1.0000348940002755e+06
 R4345: - C21 + 1e+06 alpha_ob23_seg9[1] <= 1.0000348940002755e+06
 R4346: C18 + 1e+06 alpha_ob23_seg9[2] <= 1.0000165124508708e+06
 R4347: C20 + 1e+06 alpha_ob23_seg9[2] <= 1.0000165124508708e+06
 R4348: C19 + 1e+06 alpha_ob23_seg9[3] <= 999935.9560002756
 R4349: C21 + 1e+06 alpha_ob23_seg9[3] <= 999935.9560002756
 R4350: alpha_ob23_seg9[0] + alpha_ob23_seg9[1] + alpha_ob23_seg9[2]
   + alpha_ob23_seg9[3] >= 1
 R4351: - C18 + 1e+06 alpha_ob24_seg9[0] <= 999980.937499801
 R4352: - C20 + 1e+06 alpha_ob24_seg9[0] <= 999980.937499801
 R4353: - C19 + 1e+06 alpha_ob24_seg9[1] <= 1.0000614940072973e+06
 R4354: - C21 + 1e+06 alpha_ob24_seg9[1] <= 1.0000614940072973e+06
 R4355: C18 + 1e+06 alpha_ob24_seg9[2] <= 1.000011162499801e+06
 R4356: C20 + 1e+06 alpha_ob24_seg9[2] <= 1.000011162499801e+06
 R4357: C19 + 1e+06 alpha_ob24_seg9[3] <= 999935.9560072974
 R4358: C21 + 1e+06 alpha_ob24_seg9[3] <= 999935.9560072974
 R4359: alpha_ob24_seg9[0] + alpha_ob24_seg9[1] + alpha_ob24_seg9[2]
   + alpha_ob24_seg9[3] >= 1
 R4360: - C18 + 1e+06 alpha_ob25_seg9[0] <= 999956.8394000001
 R4361: - C20 + 1e+06 alpha_ob25_seg9[0] <= 999956.8394000001
 R4362: - C19 + 1e+06 alpha_ob25_seg9[1] <= 999952.2463
 R4363: - C21 + 1e+06 alpha_ob25_seg9[1] <= 999952.2463
 R4364: C18 + 1e+06 alpha_ob25_seg9[2] <= 1.0000097606e+06
 R4365: C20 + 1e+06 alpha_ob25_seg9[2] <= 1.0000097606e+06
 R4366: C19 + 1e+06 alpha_ob25_seg9[3] <= 1.0000452037e+06
 R4367: C21 + 1e+06 alpha_ob25_seg9[3] <= 1.0000452037e+06
 R4368: alpha_ob25_seg9[0] + alpha_ob25_seg9[1] + alpha_ob25_seg9[2]
   + alpha_ob25_seg9[3] >= 1
 R4369: - C18 + 1e+06 alpha_ob26_seg9[0] <= 999980.9375348955
 R4370: - C20 + 1e+06 alpha_ob26_seg9[0] <= 999980.9375348955
 R4371: - C19 + 1e+06 alpha_ob26_seg9[1] <= 1.0000160439997246e+06
 R4372: - C21 + 1e+06 alpha_ob26_seg9[1] <= 1.0000160439997246e+06
 R4373: C18 + 1e+06 alpha_ob26_seg9[2] <= 1.0000165125348954e+06
 R4374: C20 + 1e+06 alpha_ob26_seg9[2] <= 1.0000165125348954e+06
 R4375: C19 + 1e+06 alpha_ob26_seg9[3] <= 999962.5559997245
 R4376: C21 + 1e+06 alpha_ob26_seg9[3] <= 999962.5559997245
 R4377: alpha_ob26_seg9[0] + alpha_ob26_seg9[1] + alpha_ob26_seg9[2]
   + alpha_ob26_seg9[3] >= 1
 R4378: - C18 + 1e+06 alpha_ob27_seg9[0] <= 999956.839399801
 R4379: - C20 + 1e+06 alpha_ob27_seg9[0] <= 999956.839399801
 R4380: - C19 + 1e+06 alpha_ob27_seg9[1] <= 999925.3963434525
 R4381: - C21 + 1e+06 alpha_ob27_seg9[1] <= 999925.3963434525
 R4382: C18 + 1e+06 alpha_ob27_seg9[2] <= 1.0000080105998011e+06
 R4383: C20 + 1e+06 alpha_ob27_seg9[2] <= 1.0000080105998011e+06
 R4384: C19 + 1e+06 alpha_ob27_seg9[3] <= 1.0000720537434525e+06
 R4385: C21 + 1e+06 alpha_ob27_seg9[3] <= 1.0000720537434525e+06
 R4386: alpha_ob27_seg9[0] + alpha_ob27_seg9[1] + alpha_ob27_seg9[2]
   + alpha_ob27_seg9[3] >= 1
 R4387: - C18 + 1e+06 alpha_ob28_seg9[0] <= 999980.9374998009
 R4388: - C20 + 1e+06 alpha_ob28_seg9[0] <= 999980.9374998009
 R4389: - C19 + 1e+06 alpha_ob28_seg9[1] <= 1.0000160440053071e+06
 R4390: - C21 + 1e+06 alpha_ob28_seg9[1] <= 1.0000160440053071e+06
 R4391: C18 + 1e+06 alpha_ob28_seg9[2] <= 1.000012662499801e+06
 R4392: C20 + 1e+06 alpha_ob28_seg9[2] <= 1.000012662499801e+06
 R4393: C19 + 1e+06 alpha_ob28_seg9[3] <= 999981.4060053072
 R4394: C21 + 1e+06 alpha_ob28_seg9[3] <= 999981.4060053072
 R4395: alpha_ob28_seg9[0] + alpha_ob28_seg9[1] + alpha_ob28_seg9[2]
   + alpha_ob28_seg9[3] >= 1
 R4396: - C18 + 1e+06 alpha_ob29_seg9[0] <= 999983.6053459151
 R4397: - C20 + 1e+06 alpha_ob29_seg9[0] <= 999983.6053459151
 R4398: - C19 + 1e+06 alpha_ob29_seg9[1] <= 999987.7722497246
 R4399: - C21 + 1e+06 alpha_ob29_seg9[1] <= 999987.7722497246
 R4400: C18 + 1e+06 alpha_ob29_seg9[2] <= 1.000013844745915e+06
 R4401: C20 + 1e+06 alpha_ob29_seg9[2] <= 1.000013844745915e+06
 R4402: C19 + 1e+06 alpha_ob29_seg9[3] <= 999984.8277497246
 R4403: C21 + 1e+06 alpha_ob29_seg9[3] <= 999984.8277497246
 R4404: alpha_ob29_seg9[0] + alpha_ob29_seg9[1] + alpha_ob29_seg9[2]
   + alpha_ob29_seg9[3] >= 1
 R4405: - C18 + 1e+06 alpha_ob30_seg9[0] <= 999958.8717898987
 R4406: - C20 + 1e+06 alpha_ob30_seg9[0] <= 999958.8717898987
 R4407: - C19 + 1e+06 alpha_ob30_seg9[1] <= 999987.7837602755
 R4408: - C21 + 1e+06 alpha_ob30_seg9[1] <= 999987.7837602755
 R4409: C18 + 1e+06 alpha_ob30_seg9[2] <= 1.0000385781898986e+06
 R4410: C20 + 1e+06 alpha_ob30_seg9[2] <= 1.0000385781898986e+06
 R4411: C19 + 1e+06 alpha_ob30_seg9[3] <= 1.0000043162402755e+06
 R4412: C21 + 1e+06 alpha_ob30_seg9[3] <= 1.0000043162402755e+06
 R4413: alpha_ob30_seg9[0] + alpha_ob30_seg9[1] + alpha_ob30_seg9[2]
   + alpha_ob30_seg9[3] >= 1
 R4414: - C18 + 1e+06 alpha_ob31_seg9[0] <= 999939.2718
 R4415: - C20 + 1e+06 alpha_ob31_seg9[0] <= 999939.2718
 R4416: - C19 + 1e+06 alpha_ob31_seg9[1] <= 999993.13376
 R4417: - C21 + 1e+06 alpha_ob31_seg9[1] <= 999993.13376
 R4418: C18 + 1e+06 alpha_ob31_seg9[2] <= 1.0000385782e+06
 R4419: C20 + 1e+06 alpha_ob31_seg9[2] <= 1.0000385782e+06
 R4420: C19 + 1e+06 alpha_ob31_seg9[3] <= 1.00000431624e+06
 R4421: C21 + 1e+06 alpha_ob31_seg9[3] <= 1.00000431624e+06
 R4422: alpha_ob31_seg9[0] + alpha_ob31_seg9[1] + alpha_ob31_seg9[2]
   + alpha_ob31_seg9[3] >= 1
 R4423: - C18 + 1e+06 alpha_ob32_seg9[0] <= 999939.2718101013
 R4424: - C20 + 1e+06 alpha_ob32_seg9[0] <= 999939.2718101013
 R4425: - C19 + 1e+06 alpha_ob32_seg9[1] <= 999987.7837597246
 R4426: - C21 + 1e+06 alpha_ob32_seg9[1] <= 999987.7837597246
 R4427: C18 + 1e+06 alpha_ob32_seg9[2] <= 1.0000581782101013e+06
 R4428: C20 + 1e+06 alpha_ob32_seg9[2] <= 1.0000581782101013e+06
 R4429: C19 + 1e+06 alpha_ob32_seg9[3] <= 1.0000043162397245e+06
 R4430: C21 + 1e+06 alpha_ob32_seg9[3] <= 1.0000043162397245e+06
 R4431: alpha_ob32_seg9[0] + alpha_ob32_seg9[1] + alpha_ob32_seg9[2]
   + alpha_ob32_seg9[3] >= 1
 R4432: - C18 + 1e+06 alpha_ob33_seg9[0] <= 999939.271799801
 R4433: - C20 + 1e+06 alpha_ob33_seg9[0] <= 999939.271799801
 R4434: - C19 + 1e+06 alpha_ob33_seg9[1] <= 999987.7838262042
 R4435: - C21 + 1e+06 alpha_ob33_seg9[1] <= 999987.7838262042
 R4436: C18 + 1e+06 alpha_ob33_seg9[2] <= 1.000038578199801e+06
 R4437: C20 + 1e+06 alpha_ob33_seg9[2] <= 1.000038578199801e+06
 R4438: C19 + 1e+06 alpha_ob33_seg9[3] <= 1.0000096662262043e+06
 R4439: C21 + 1e+06 alpha_ob33_seg9[3] <= 1.0000096662262043e+06
 R4440: alpha_ob33_seg9[0] + alpha_ob33_seg9[1] + alpha_ob33_seg9[2]
   + alpha_ob33_seg9[3] >= 1
 R4441: - C18 + 1e+06 alpha_ob34_seg9[0] <= 999926.0708
 R4442: - C20 + 1e+06 alpha_ob34_seg9[0] <= 999926.0708
 R4443: - C19 + 1e+06 alpha_ob34_seg9[1] <= 999983.7196
 R4444: - C21 + 1e+06 alpha_ob34_seg9[1] <= 999983.7196
 R4445: C18 + 1e+06 alpha_ob34_seg9[2] <= 1.0000250292e+06
 R4446: C20 + 1e+06 alpha_ob34_seg9[2] <= 1.0000250292e+06
 R4447: C19 + 1e+06 alpha_ob34_seg9[3] <= 1.0000137304e+06
 R4448: C21 + 1e+06 alpha_ob34_seg9[3] <= 1.0000137304e+06
 R4449: alpha_ob34_seg9[0] + alpha_ob34_seg9[1] + alpha_ob34_seg9[2]
   + alpha_ob34_seg9[3] >= 1
 R4450: - C18 + 1e+06 alpha_ob35_seg9[0] <= 999926.0708293857
 R4451: - C20 + 1e+06 alpha_ob35_seg9[0] <= 999926.0708293857
 R4452: - C19 + 1e+06 alpha_ob35_seg9[1] <= 999967.8695997245
 R4453: - C21 + 1e+06 alpha_ob35_seg9[1] <= 999967.8695997245
 R4454: C18 + 1e+06 alpha_ob35_seg9[2] <= 1.0000713792293856e+06
 R4455: C20 + 1e+06 alpha_ob35_seg9[2] <= 1.0000713792293856e+06
 R4456: C19 + 1e+06 alpha_ob35_seg9[3] <= 1.0000137303997246e+06
 R4457: C21 + 1e+06 alpha_ob35_seg9[3] <= 1.0000137303997246e+06
 R4458: alpha_ob35_seg9[0] + alpha_ob35_seg9[1] + alpha_ob35_seg9[2]
   + alpha_ob35_seg9[3] >= 1
 R4459: - C18 + 1e+06 alpha_ob36_seg9[0] <= 999972.6917508709
 R4460: - C20 + 1e+06 alpha_ob36_seg9[0] <= 999972.6917508709
 R4461: - C19 + 1e+06 alpha_ob36_seg9[1] <= 999957.1644002756
 R4462: - C21 + 1e+06 alpha_ob36_seg9[1] <= 999957.1644002756
 R4463: C18 + 1e+06 alpha_ob36_seg9[2] <= 1.0000247581508709e+06
 R4464: C20 + 1e+06 alpha_ob36_seg9[2] <= 1.0000247581508709e+06
 R4465: C19 + 1e+06 alpha_ob36_seg9[3] <= 1.0000136856002755e+06
 R4466: C21 + 1e+06 alpha_ob36_seg9[3] <= 1.0000136856002755e+06
 R4467: alpha_ob36_seg9[0] + alpha_ob36_seg9[1] + alpha_ob36_seg9[2]
   + alpha_ob36_seg9[3] >= 1
 R4468: - C18 + 1e+06 alpha_ob37_seg9[0] <= 999939.0009674003
 R4469: - C20 + 1e+06 alpha_ob37_seg9[0] <= 999939.0009674003
 R4470: - C19 + 1e+06 alpha_ob37_seg9[1] <= 999961.9516002756
 R4471: - C21 + 1e+06 alpha_ob37_seg9[1] <= 999961.9516002756
 R4472: C18 + 1e+06 alpha_ob37_seg9[2] <= 1.0000584489674004e+06
 R4473: C20 + 1e+06 alpha_ob37_seg9[2] <= 1.0000584489674004e+06
 R4474: C19 + 1e+06 alpha_ob37_seg9[3] <= 1.0000178984002755e+06
 R4475: C21 + 1e+06 alpha_ob37_seg9[3] <= 1.0000178984002755e+06
 R4476: alpha_ob37_seg9[0] + alpha_ob37_seg9[1] + alpha_ob37_seg9[2]
   + alpha_ob37_seg9[3] >= 1
 R4477: - C18 + 1e+06 alpha_ob38_seg9[0] <= 999994.9164998011
 R4478: - C20 + 1e+06 alpha_ob38_seg9[0] <= 999994.9164998011
 R4479: - C19 + 1e+06 alpha_ob38_seg9[1] <= 999925.4284782809
 R4480: - C21 + 1e+06 alpha_ob38_seg9[1] <= 999925.4284782809
 R4481: C18 + 1e+06 alpha_ob38_seg9[2] <= 999943.6834998011
 R4482: C20 + 1e+06 alpha_ob38_seg9[2] <= 999943.6834998011
 R4483: C19 + 1e+06 alpha_ob38_seg9[3] <= 1.0000720216782809e+06
 R4484: C21 + 1e+06 alpha_ob38_seg9[3] <= 1.0000720216782809e+06
 R4485: alpha_ob38_seg9[0] + alpha_ob38_seg9[1] + alpha_ob38_seg9[2]
   + alpha_ob38_seg9[3] >= 1
 R4486: - C18 + 1e+06 alpha_ob39_seg9[0] <= 999914.52327842
 R4487: - C20 + 1e+06 alpha_ob39_seg9[0] <= 999914.52327842
 R4488: - C19 + 1e+06 alpha_ob39_seg9[1] <= 999993.6552362755
 R4489: - C21 + 1e+06 alpha_ob39_seg9[1] <= 999993.6552362755
 R4490: C18 + 1e+06 alpha_ob39_seg9[2] <= 1.0000829266784199e+06
 R4491: C20 + 1e+06 alpha_ob39_seg9[2] <= 1.0000829266784199e+06
 R4492: C19 + 1e+06 alpha_ob39_seg9[3] <= 999992.1947642756
 R4493: C21 + 1e+06 alpha_ob39_seg9[3] <= 999992.1947642756
 R4494: alpha_ob39_seg9[0] + alpha_ob39_seg9[1] + alpha_ob39_seg9[2]
   + alpha_ob39_seg9[3] >= 1
 R4495: - C18 + 1e+06 alpha_ob40_seg9[0] <= 999912.265199801
 R4496: - C20 + 1e+06 alpha_ob40_seg9[0] <= 999912.265199801
 R4497: - C19 + 1e+06 alpha_ob40_seg9[1] <= 999987.7842112777
 R4498: - C21 + 1e+06 alpha_ob40_seg9[1] <= 999987.7842112777
 R4499: C18 + 1e+06 alpha_ob40_seg9[2] <= 1.000076834799801e+06
 R4500: C20 + 1e+06 alpha_ob40_seg9[2] <= 1.000076834799801e+06
 R4501: C19 + 1e+06 alpha_ob40_seg9[3] <= 1.0000096658112778e+06
 R4502: C21 + 1e+06 alpha_ob40_seg9[3] <= 1.0000096658112778e+06
 R4503: alpha_ob40_seg9[0] + alpha_ob40_seg9[1] + alpha_ob40_seg9[2]
   + alpha_ob40_seg9[3] >= 1
 R4504: - C18 + 1e+06 alpha_ob41_seg9[0] <= 999920.6152119379
 R4505: - C20 + 1e+06 alpha_ob41_seg9[0] <= 999920.6152119379
 R4506: - C19 + 1e+06 alpha_ob41_seg9[1] <= 999981.4341997246
 R4507: - C21 + 1e+06 alpha_ob41_seg9[1] <= 999981.4341997246
 R4508: C18 + 1e+06 alpha_ob41_seg9[2] <= 1.0000768348119379e+06
 R4509: C20 + 1e+06 alpha_ob41_seg9[2] <= 1.0000768348119379e+06
 R4510: C19 + 1e+06 alpha_ob41_seg9[3] <= 1.0000096657997245e+06
 R4511: C21 + 1e+06 alpha_ob41_seg9[3] <= 1.0000096657997245e+06
 R4512: alpha_ob41_seg9[0] + alpha_ob41_seg9[1] + alpha_ob41_seg9[2]
   + alpha_ob41_seg9[3] >= 1
 R4513: - C18 + 1e+06 alpha_ob42_seg9[0] <= 999919.2652
 R4514: - C20 + 1e+06 alpha_ob42_seg9[0] <= 999919.2652
 R4515: - C19 + 1e+06 alpha_ob42_seg9[1] <= 999981.4342
 R4516: - C21 + 1e+06 alpha_ob42_seg9[1] <= 999981.4342
 R4517: C18 + 1e+06 alpha_ob42_seg9[2] <= 1.0000768348e+06
 R4518: C20 + 1e+06 alpha_ob42_seg9[2] <= 1.0000768348e+06
 R4519: C19 + 1e+06 alpha_ob42_seg9[3] <= 1.0000160158000001e+06
 R4520: C21 + 1e+06 alpha_ob42_seg9[3] <= 1.0000160158000001e+06
 R4521: alpha_ob42_seg9[0] + alpha_ob42_seg9[1] + alpha_ob42_seg9[2]
   + alpha_ob42_seg9[3] >= 1
 R4522: - C18 + 1e+06 alpha_ob43_seg9[0] <= 999919.2652454559
 R4523: - C20 + 1e+06 alpha_ob43_seg9[0] <= 999919.2652454559
 R4524: - C19 + 1e+06 alpha_ob43_seg9[1] <= 999956.8341997246
 R4525: - C21 + 1e+06 alpha_ob43_seg9[1] <= 999956.8341997246
 R4526: C18 + 1e+06 alpha_ob43_seg9[2] <= 1.0000781848454559e+06
 R4527: C20 + 1e+06 alpha_ob43_seg9[2] <= 1.0000781848454559e+06
 R4528: C19 + 1e+06 alpha_ob43_seg9[3] <= 1.0000160157997246e+06
 R4529: C21 + 1e+06 alpha_ob43_seg9[3] <= 1.0000160157997246e+06
 R4530: alpha_ob43_seg9[0] + alpha_ob43_seg9[1] + alpha_ob43_seg9[2]
   + alpha_ob43_seg9[3] >= 1
 R4531: - C18 + 1e+06 alpha_ob44_seg9[0] <= 999991.83683
 R4532: - C20 + 1e+06 alpha_ob44_seg9[0] <= 999991.83683
 R4533: - C19 + 1e+06 alpha_ob44_seg9[1] <= 999940.589
 R4534: - C21 + 1e+06 alpha_ob44_seg9[1] <= 999940.589
 R4535: C18 + 1e+06 alpha_ob44_seg9[2] <= 1.00000401317e+06
 R4536: C20 + 1e+06 alpha_ob44_seg9[2] <= 1.00000401317e+06
 R4537: C19 + 1e+06 alpha_ob44_seg9[3] <= 1.000056861e+06
 R4538: C21 + 1e+06 alpha_ob44_seg9[3] <= 1.000056861e+06
 R4539: alpha_ob44_seg9[0] + alpha_ob44_seg9[1] + alpha_ob44_seg9[2]
   + alpha_ob44_seg9[3] >= 1
 R4540: - C18 + 1e+06 alpha_ob45_seg9[0] <= 999991.8368084199
 R4541: - C20 + 1e+06 alpha_ob45_seg9[0] <= 999991.8368084199
 R4542: - C19 + 1e+06 alpha_ob45_seg9[1] <= 999940.5890002755
 R4543: - C21 + 1e+06 alpha_ob45_seg9[1] <= 999940.5890002755
 R4544: C18 + 1e+06 alpha_ob45_seg9[2] <= 1.0000056131484199e+06
 R4545: C20 + 1e+06 alpha_ob45_seg9[2] <= 1.0000056131484199e+06
 R4546: C19 + 1e+06 alpha_ob45_seg9[3] <= 1.0000452610002755e+06
 R4547: C21 + 1e+06 alpha_ob45_seg9[3] <= 1.0000452610002755e+06
 R4548: alpha_ob45_seg9[0] + alpha_ob45_seg9[1] + alpha_ob45_seg9[2]
   + alpha_ob45_seg9[3] >= 1
 R4549: - C18 + 1e+06 alpha_ob46_seg9[0] <= 1.0000764021508709e+06
 R4550: - C20 + 1e+06 alpha_ob46_seg9[0] <= 1.0000764021508709e+06
 R4551: - C19 + 1e+06 alpha_ob46_seg9[1] <= 999957.1027002756
 R4552: - C21 + 1e+06 alpha_ob46_seg9[1] <= 999957.1027002756
 R4553: C18 + 1e+06 alpha_ob46_seg9[2] <= 999921.0477508709
 R4554: C20 + 1e+06 alpha_ob46_seg9[2] <= 999921.0477508709
 R4555: C19 + 1e+06 alpha_ob46_seg9[3] <= 1.0000137473002756e+06
 R4556: C21 + 1e+06 alpha_ob46_seg9[3] <= 1.0000137473002756e+06
 R4557: alpha_ob46_seg9[0] + alpha_ob46_seg9[1] + alpha_ob46_seg9[2]
   + alpha_ob46_seg9[3] >= 1
 R4558: - C18 + 1e+06 alpha_ob47_seg9[0] <= 1.0000445522e+06
 R4559: - C20 + 1e+06 alpha_ob47_seg9[0] <= 1.0000445522e+06
 R4560: - C19 + 1e+06 alpha_ob47_seg9[1] <= 999983.7027
 R4561: - C21 + 1e+06 alpha_ob47_seg9[1] <= 999983.7027
 R4562: C18 + 1e+06 alpha_ob47_seg9[2] <= 999921.0477999999
 R4563: C20 + 1e+06 alpha_ob47_seg9[2] <= 999921.0477999999
 R4564: C19 + 1e+06 alpha_ob47_seg9[3] <= 1.0000137473e+06
 R4565: C21 + 1e+06 alpha_ob47_seg9[3] <= 1.0000137473e+06
 R4566: alpha_ob47_seg9[0] + alpha_ob47_seg9[1] + alpha_ob47_seg9[2]
   + alpha_ob47_seg9[3] >= 1
 R4567: - C18 + 1e+06 alpha_ob48_seg9[0] <= 1.0000445522183661e+06
 R4568: - C20 + 1e+06 alpha_ob48_seg9[0] <= 1.0000445522183661e+06
 R4569: - C19 + 1e+06 alpha_ob48_seg9[1] <= 999973.8526997245
 R4570: - C21 + 1e+06 alpha_ob48_seg9[1] <= 999973.8526997245
 R4571: C18 + 1e+06 alpha_ob48_seg9[2] <= 999952.8978183661
 R4572: C20 + 1e+06 alpha_ob48_seg9[2] <= 999952.8978183661
 R4573: C19 + 1e+06 alpha_ob48_seg9[3] <= 1.0000137472997245e+06
 R4574: C21 + 1e+06 alpha_ob48_seg9[3] <= 1.0000137472997245e+06
 R4575: alpha_ob48_seg9[0] + alpha_ob48_seg9[1] + alpha_ob48_seg9[2]
   + alpha_ob48_seg9[3] >= 1
 R4576: - C18 + 1e+06 alpha_ob49_seg9[0] <= 1.000064332081634e+06
 R4577: - C20 + 1e+06 alpha_ob49_seg9[0] <= 1.000064332081634e+06
 R4578: - C19 + 1e+06 alpha_ob49_seg9[1] <= 999973.9401002756
 R4579: - C21 + 1e+06 alpha_ob49_seg9[1] <= 999973.9401002756
 R4580: C18 + 1e+06 alpha_ob49_seg9[2] <= 999933.1178816339
 R4581: C20 + 1e+06 alpha_ob49_seg9[2] <= 999933.1178816339
 R4582: C19 + 1e+06 alpha_ob49_seg9[3] <= 1.0000136599002755e+06
 R4583: C21 + 1e+06 alpha_ob49_seg9[3] <= 1.0000136599002755e+06
 R4584: alpha_ob49_seg9[0] + alpha_ob49_seg9[1] + alpha_ob49_seg9[2]
   + alpha_ob49_seg9[3] >= 1
 R4585: - C18 + 1e+06 alpha_ob50_seg9[0] <= 1.0000643321013774e+06
 R4586: - C20 + 1e+06 alpha_ob50_seg9[0] <= 1.0000643321013774e+06
 R4587: - C19 + 1e+06 alpha_ob50_seg9[1] <= 999983.1900997246
 R4588: - C21 + 1e+06 alpha_ob50_seg9[1] <= 999983.1900997246
 R4589: C18 + 1e+06 alpha_ob50_seg9[2] <= 999933.1179013775
 R4590: C20 + 1e+06 alpha_ob50_seg9[2] <= 999933.1179013775
 R4591: C19 + 1e+06 alpha_ob50_seg9[3] <= 1.0000136598997245e+06
 R4592: C21 + 1e+06 alpha_ob50_seg9[3] <= 1.0000136598997245e+06
 R4593: alpha_ob50_seg9[0] + alpha_ob50_seg9[1] + alpha_ob50_seg9[2]
   + alpha_ob50_seg9[3] >= 1
 R4594: - C20 + 1e+06 alpha_ob0_seg10[0] <= 1.0000284170499525e+06
 R4595: - C22 + 1e+06 alpha_ob0_seg10[0] <= 1.0000284170499525e+06
 R4596: - C21 + 1e+06 alpha_ob0_seg10[1] <= 999957.0843002755
 R4597: - C23 + 1e+06 alpha_ob0_seg10[1] <= 999957.0843002755
 R4598: C20 + 1e+06 alpha_ob0_seg10[2] <= 999969.0328499526
 R4599: C22 + 1e+06 alpha_ob0_seg10[2] <= 999969.0328499526
 R4600: C21 + 1e+06 alpha_ob0_seg10[3] <= 1.0000132657002755e+06
 R4601: C23 + 1e+06 alpha_ob0_seg10[3] <= 1.0000132657002755e+06
 R4602: alpha_ob0_seg10[0] + alpha_ob0_seg10[1] + alpha_ob0_seg10[2]
   + alpha_ob0_seg10[3] >= 1
 R4603: - C20 + 1e+06 alpha_ob1_seg10[0] <= 1.000028417099801e+06
 R4604: - C22 + 1e+06 alpha_ob1_seg10[0] <= 1.000028417099801e+06
 R4605: - C21 + 1e+06 alpha_ob1_seg10[1] <= 999984.1843155898
 R4606: - C23 + 1e+06 alpha_ob1_seg10[1] <= 999984.1843155898
 R4607: C20 + 1e+06 alpha_ob1_seg10[2] <= 999957.4328998009
 R4608: C22 + 1e+06 alpha_ob1_seg10[2] <= 999957.4328998009
 R4609: C21 + 1e+06 alpha_ob1_seg10[3] <= 1.0000132657155899e+06
 R4610: C23 + 1e+06 alpha_ob1_seg10[3] <= 1.0000132657155899e+06
 R4611: alpha_ob1_seg10[0] + alpha_ob1_seg10[1] + alpha_ob1_seg10[2]
   + alpha_ob1_seg10[3] >= 1
 R4612: - C20 + 1e+06 alpha_ob2_seg10[0] <= 1.0000400171500475e+06
 R4613: - C22 + 1e+06 alpha_ob2_seg10[0] <= 1.0000400171500475e+06
 R4614: - C21 + 1e+06 alpha_ob2_seg10[1] <= 999957.0842997245
 R4615: - C23 + 1e+06 alpha_ob2_seg10[1] <= 999957.0842997245
 R4616: C20 + 1e+06 alpha_ob2_seg10[2] <= 999957.4329500474
 R4617: C22 + 1e+06 alpha_ob2_seg10[2] <= 999957.4329500474
 R4618: C21 + 1e+06 alpha_ob2_seg10[3] <= 1.0000132656997246e+06
 R4619: C23 + 1e+06 alpha_ob2_seg10[3] <= 1.0000132656997246e+06
 R4620: alpha_ob2_seg10[0] + alpha_ob2_seg10[1] + alpha_ob2_seg10[2]
   + alpha_ob2_seg10[3] >= 1
 R4621: - C20 + 1e+06 alpha_ob3_seg10[0] <= 1.0000109161752058e+06
 R4622: - C22 + 1e+06 alpha_ob3_seg10[0] <= 1.0000109161752058e+06
 R4623: - C21 + 1e+06 alpha_ob3_seg10[1] <= 999970.2681002755
 R4624: - C23 + 1e+06 alpha_ob3_seg10[1] <= 999970.2681002755
 R4625: C20 + 1e+06 alpha_ob3_seg10[2] <= 999986.5337752058
 R4626: C22 + 1e+06 alpha_ob3_seg10[2] <= 999986.5337752058
 R4627: C21 + 1e+06 alpha_ob3_seg10[3] <= 1.0000138319002755e+06
 R4628: C23 + 1e+06 alpha_ob3_seg10[3] <= 1.0000138319002755e+06
 R4629: alpha_ob3_seg10[0] + alpha_ob3_seg10[1] + alpha_ob3_seg10[2]
   + alpha_ob3_seg10[3] >= 1
 R4630: - C20 + 1e+06 alpha_ob4_seg10[0] <= 1.000010916199801e+06
 R4631: - C22 + 1e+06 alpha_ob4_seg10[0] <= 1.000010916199801e+06
 R4632: - C21 + 1e+06 alpha_ob4_seg10[1] <= 999983.6181179117
 R4633: - C23 + 1e+06 alpha_ob4_seg10[1] <= 999983.6181179117
 R4634: C20 + 1e+06 alpha_ob4_seg10[2] <= 999973.183799801
 R4635: C22 + 1e+06 alpha_ob4_seg10[2] <= 999973.183799801
 R4636: C21 + 1e+06 alpha_ob4_seg10[3] <= 1.0000138319179118e+06
 R4637: C23 + 1e+06 alpha_ob4_seg10[3] <= 1.0000138319179118e+06
 R4638: alpha_ob4_seg10[0] + alpha_ob4_seg10[1] + alpha_ob4_seg10[2]
   + alpha_ob4_seg10[3] >= 1
 R4639: - C20 + 1e+06 alpha_ob5_seg10[0] <= 999992.49102
 R4640: - C22 + 1e+06 alpha_ob5_seg10[0] <= 999992.49102
 R4641: - C21 + 1e+06 alpha_ob5_seg10[1] <= 999957.1016000001
 R4642: - C23 + 1e+06 alpha_ob5_seg10[1] <= 999957.1016000001
 R4643: C20 + 1e+06 alpha_ob5_seg10[2] <= 999986.35898
 R4644: C22 + 1e+06 alpha_ob5_seg10[2] <= 999986.35898
 R4645: C21 + 1e+06 alpha_ob5_seg10[3] <= 1.0000403484e+06
 R4646: C23 + 1e+06 alpha_ob5_seg10[3] <= 1.0000403484e+06
 R4647: alpha_ob5_seg10[0] + alpha_ob5_seg10[1] + alpha_ob5_seg10[2]
   + alpha_ob5_seg10[3] >= 1
 R4648: - C20 + 1e+06 alpha_ob6_seg10[0] <= 999981.3909999999
 R4649: - C22 + 1e+06 alpha_ob6_seg10[0] <= 999981.3909999999
 R4650: - C21 + 1e+06 alpha_ob6_seg10[1] <= 999957.1016000001
 R4651: - C23 + 1e+06 alpha_ob6_seg10[1] <= 999957.1016000001
 R4652: C20 + 1e+06 alpha_ob6_seg10[2] <= 1.000004959e+06
 R4653: C22 + 1e+06 alpha_ob6_seg10[2] <= 1.000004959e+06
 R4654: C21 + 1e+06 alpha_ob6_seg10[3] <= 1.0000403484e+06
 R4655: C23 + 1e+06 alpha_ob6_seg10[3] <= 1.0000403484e+06
 R4656: alpha_ob6_seg10[0] + alpha_ob6_seg10[1] + alpha_ob6_seg10[2]
   + alpha_ob6_seg10[3] >= 1
 R4657: - C20 + 1e+06 alpha_ob7_seg10[0] <= 999981.3909825523
 R4658: - C22 + 1e+06 alpha_ob7_seg10[0] <= 999981.3909825523
 R4659: - C21 + 1e+06 alpha_ob7_seg10[1] <= 999957.1016002755
 R4660: - C23 + 1e+06 alpha_ob7_seg10[1] <= 999957.1016002755
 R4661: C20 + 1e+06 alpha_ob7_seg10[2] <= 1.0000160589825523e+06
 R4662: C22 + 1e+06 alpha_ob7_seg10[2] <= 1.0000160589825523e+06
 R4663: C21 + 1e+06 alpha_ob7_seg10[3] <= 1.0000309984002755e+06
 R4664: C23 + 1e+06 alpha_ob7_seg10[3] <= 1.0000309984002755e+06
 R4665: alpha_ob7_seg10[0] + alpha_ob7_seg10[1] + alpha_ob7_seg10[2]
   + alpha_ob7_seg10[3] >= 1
 R4666: - C20 + 1e+06 alpha_ob8_seg10[0] <= 999981.3867954084
 R4667: - C22 + 1e+06 alpha_ob8_seg10[0] <= 999981.3867954084
 R4668: - C21 + 1e+06 alpha_ob8_seg10[1] <= 999968.8595002755
 R4669: - C23 + 1e+06 alpha_ob8_seg10[1] <= 999968.8595002755
 R4670: C20 + 1e+06 alpha_ob8_seg10[2] <= 1.0000160631954084e+06
 R4671: C22 + 1e+06 alpha_ob8_seg10[2] <= 1.0000160631954084e+06
 R4672: C21 + 1e+06 alpha_ob8_seg10[3] <= 1.0000262405002755e+06
 R4673: C23 + 1e+06 alpha_ob8_seg10[3] <= 1.0000262405002755e+06
 R4674: alpha_ob8_seg10[0] + alpha_ob8_seg10[1] + alpha_ob8_seg10[2]
   + alpha_ob8_seg10[3] >= 1
 R4675: - C20 + 1e+06 alpha_ob9_seg10[0] <= 999981.386799801
 R4676: - C22 + 1e+06 alpha_ob9_seg10[0] <= 999981.386799801
 R4677: - C21 + 1e+06 alpha_ob9_seg10[1] <= 999971.2095013268
 R4678: - C23 + 1e+06 alpha_ob9_seg10[1] <= 999971.2095013268
 R4679: C20 + 1e+06 alpha_ob9_seg10[2] <= 1.0000152131998009e+06
 R4680: C22 + 1e+06 alpha_ob9_seg10[2] <= 1.0000152131998009e+06
 R4681: C21 + 1e+06 alpha_ob9_seg10[3] <= 1.0000262405013267e+06
 R4682: C23 + 1e+06 alpha_ob9_seg10[3] <= 1.0000262405013267e+06
 R4683: alpha_ob9_seg10[0] + alpha_ob9_seg10[1] + alpha_ob9_seg10[2]
   + alpha_ob9_seg10[3] >= 1
 R4684: - C20 + 1e+06 alpha_ob10_seg10[0] <= 999982.2368022958
 R4685: - C22 + 1e+06 alpha_ob10_seg10[0] <= 999982.2368022958
 R4686: - C21 + 1e+06 alpha_ob10_seg10[1] <= 999970.1094997245
 R4687: - C23 + 1e+06 alpha_ob10_seg10[1] <= 999970.1094997245
 R4688: C20 + 1e+06 alpha_ob10_seg10[2] <= 1.0000152132022957e+06
 R4689: C22 + 1e+06 alpha_ob10_seg10[2] <= 1.0000152132022957e+06
 R4690: C21 + 1e+06 alpha_ob10_seg10[3] <= 1.0000262404997245e+06
 R4691: C23 + 1e+06 alpha_ob10_seg10[3] <= 1.0000262404997245e+06
 R4692: alpha_ob10_seg10[0] + alpha_ob10_seg10[1] + alpha_ob10_seg10[2]
   + alpha_ob10_seg10[3] >= 1
 R4693: - C20 + 1e+06 alpha_ob11_seg10[0] <= 999982.236799801
 R4694: - C22 + 1e+06 alpha_ob11_seg10[0] <= 999982.236799801
 R4695: - C21 + 1e+06 alpha_ob11_seg10[1] <= 999970.1095291895
 R4696: - C23 + 1e+06 alpha_ob11_seg10[1] <= 999970.1095291895
 R4697: C20 + 1e+06 alpha_ob11_seg10[2] <= 999993.3631998011
 R4698: C22 + 1e+06 alpha_ob11_seg10[2] <= 999993.3631998011
 R4699: C21 + 1e+06 alpha_ob11_seg10[3] <= 1.0000273405291894e+06
 R4700: C23 + 1e+06 alpha_ob11_seg10[3] <= 1.0000273405291894e+06
 R4701: alpha_ob11_seg10[0] + alpha_ob11_seg10[1] + alpha_ob11_seg10[2]
   + alpha_ob11_seg10[3] >= 1
 R4702: - C20 + 1e+06 alpha_ob12_seg10[0] <= 999981.6050802565
 R4703: - C22 + 1e+06 alpha_ob12_seg10[0] <= 999981.6050802565
 R4704: - C21 + 1e+06 alpha_ob12_seg10[1] <= 999972.8942002755
 R4705: - C23 + 1e+06 alpha_ob12_seg10[1] <= 999972.8942002755
 R4706: C20 + 1e+06 alpha_ob12_seg10[2] <= 1.0000158448802566e+06
 R4707: C22 + 1e+06 alpha_ob12_seg10[2] <= 1.0000158448802566e+06
 R4708: C21 + 1e+06 alpha_ob12_seg10[3] <= 1.0000139558002756e+06
 R4709: C23 + 1e+06 alpha_ob12_seg10[3] <= 1.0000139558002756e+06
 R4710: alpha_ob12_seg10[0] + alpha_ob12_seg10[1] + alpha_ob12_seg10[2]
   + alpha_ob12_seg10[3] >= 1
 R4711: - C20 + 1e+06 alpha_ob13_seg10[0] <= 999981.605149801
 R4712: - C22 + 1e+06 alpha_ob13_seg10[0] <= 999981.605149801
 R4713: - C21 + 1e+06 alpha_ob13_seg10[1] <= 999983.4942391404
 R4714: - C23 + 1e+06 alpha_ob13_seg10[1] <= 999983.4942391404
 R4715: C20 + 1e+06 alpha_ob13_seg10[2] <= 999986.4948498011
 R4716: C22 + 1e+06 alpha_ob13_seg10[2] <= 999986.4948498011
 R4717: C21 + 1e+06 alpha_ob13_seg10[3] <= 1.0000139558391405e+06
 R4718: C23 + 1e+06 alpha_ob13_seg10[3] <= 1.0000139558391405e+06
 R4719: alpha_ob13_seg10[0] + alpha_ob13_seg10[1] + alpha_ob13_seg10[2]
   + alpha_ob13_seg10[3] >= 1
 R4720: - C20 + 1e+06 alpha_ob14_seg10[0] <= 999997.6803502565
 R4721: - C22 + 1e+06 alpha_ob14_seg10[0] <= 999997.6803502565
 R4722: - C21 + 1e+06 alpha_ob14_seg10[1] <= 999972.7268002755
 R4723: - C23 + 1e+06 alpha_ob14_seg10[1] <= 999972.7268002755
 R4724: C20 + 1e+06 alpha_ob14_seg10[2] <= 999999.7696102565
 R4725: C22 + 1e+06 alpha_ob14_seg10[2] <= 999999.7696102565
 R4726: C21 + 1e+06 alpha_ob14_seg10[3] <= 1.0000141232002755e+06
 R4727: C23 + 1e+06 alpha_ob14_seg10[3] <= 1.0000141232002755e+06
 R4728: alpha_ob14_seg10[0] + alpha_ob14_seg10[1] + alpha_ob14_seg10[2]
   + alpha_ob14_seg10[3] >= 1
 R4729: - C20 + 1e+06 alpha_ob15_seg10[0] <= 1.0000038341502565e+06
 R4730: - C22 + 1e+06 alpha_ob15_seg10[0] <= 1.0000038341502565e+06
 R4731: - C21 + 1e+06 alpha_ob15_seg10[1] <= 999972.6012002755
 R4732: - C23 + 1e+06 alpha_ob15_seg10[1] <= 999972.6012002755
 R4733: C20 + 1e+06 alpha_ob15_seg10[2] <= 999993.6158102566
 R4734: C22 + 1e+06 alpha_ob15_seg10[2] <= 999993.6158102566
 R4735: C21 + 1e+06 alpha_ob15_seg10[3] <= 1.0000142488002755e+06
 R4736: C23 + 1e+06 alpha_ob15_seg10[3] <= 1.0000142488002755e+06
 R4737: alpha_ob15_seg10[0] + alpha_ob15_seg10[1] + alpha_ob15_seg10[2]
   + alpha_ob15_seg10[3] >= 1
 R4738: - C20 + 1e+06 alpha_ob16_seg10[0] <= 1.0000162874063333e+06
 R4739: - C22 + 1e+06 alpha_ob16_seg10[0] <= 1.0000162874063333e+06
 R4740: - C21 + 1e+06 alpha_ob16_seg10[1] <= 999982.9439602756
 R4741: - C23 + 1e+06 alpha_ob16_seg10[1] <= 999982.9439602756
 R4742: C20 + 1e+06 alpha_ob16_seg10[2] <= 999981.1624063333
 R4743: C22 + 1e+06 alpha_ob16_seg10[2] <= 999981.1624063333
 R4744: C21 + 1e+06 alpha_ob16_seg10[3] <= 999963.6560402757
 R4745: C23 + 1e+06 alpha_ob16_seg10[3] <= 999963.6560402757
 R4746: alpha_ob16_seg10[0] + alpha_ob16_seg10[1] + alpha_ob16_seg10[2]
   + alpha_ob16_seg10[3] >= 1
 R4747: - C20 + 1e+06 alpha_ob17_seg10[0] <= 999980.937509
 R4748: - C22 + 1e+06 alpha_ob17_seg10[0] <= 999980.937509
 R4749: - C21 + 1e+06 alpha_ob17_seg10[1] <= 1.000034894e+06
 R4750: - C23 + 1e+06 alpha_ob17_seg10[1] <= 1.000034894e+06
 R4751: C20 + 1e+06 alpha_ob17_seg10[2] <= 999981.162491
 R4752: C22 + 1e+06 alpha_ob17_seg10[2] <= 999981.162491
 R4753: C21 + 1e+06 alpha_ob17_seg10[3] <= 999962.556
 R4754: C23 + 1e+06 alpha_ob17_seg10[3] <= 999962.556
 R4755: alpha_ob17_seg10[0] + alpha_ob17_seg10[1] + alpha_ob17_seg10[2]
   + alpha_ob17_seg10[3] >= 1
 R4756: - C20 + 1e+06 alpha_ob18_seg10[0] <= 999990.2670277582
 R4757: - C22 + 1e+06 alpha_ob18_seg10[0] <= 999990.2670277582
 R4758: - C21 + 1e+06 alpha_ob18_seg10[1] <= 1.0000347702002756e+06
 R4759: - C23 + 1e+06 alpha_ob18_seg10[1] <= 1.0000347702002756e+06
 R4760: C20 + 1e+06 alpha_ob18_seg10[2] <= 1.0000071828877581e+06
 R4761: C22 + 1e+06 alpha_ob18_seg10[2] <= 1.0000071828877581e+06
 R4762: C21 + 1e+06 alpha_ob18_seg10[3] <= 999939.8298002756
 R4763: C23 + 1e+06 alpha_ob18_seg10[3] <= 999939.8298002756
 R4764: alpha_ob18_seg10[0] + alpha_ob18_seg10[1] + alpha_ob18_seg10[2]
   + alpha_ob18_seg10[3] >= 1
 R4765: - C20 + 1e+06 alpha_ob19_seg10[0] <= 999990.267069801
 R4766: - C22 + 1e+06 alpha_ob19_seg10[0] <= 999990.267069801
 R4767: - C21 + 1e+06 alpha_ob19_seg10[1] <= 1.0000576202069657e+06
 R4768: - C23 + 1e+06 alpha_ob19_seg10[1] <= 1.0000576202069657e+06
 R4769: C20 + 1e+06 alpha_ob19_seg10[2] <= 1.000002082929801e+06
 R4770: C22 + 1e+06 alpha_ob19_seg10[2] <= 1.000002082929801e+06
 R4771: C21 + 1e+06 alpha_ob19_seg10[3] <= 999939.8298069657
 R4772: C23 + 1e+06 alpha_ob19_seg10[3] <= 999939.8298069657
 R4773: alpha_ob19_seg10[0] + alpha_ob19_seg10[1] + alpha_ob19_seg10[2]
   + alpha_ob19_seg10[3] >= 1
 R4774: - C20 + 1e+06 alpha_ob20_seg10[0] <= 999991.073732
 R4775: - C22 + 1e+06 alpha_ob20_seg10[0] <= 999991.073732
 R4776: - C21 + 1e+06 alpha_ob20_seg10[1] <= 999952.2125
 R4777: - C23 + 1e+06 alpha_ob20_seg10[1] <= 999952.2125
 R4778: C20 + 1e+06 alpha_ob20_seg10[2] <= 999990.776268
 R4779: C22 + 1e+06 alpha_ob20_seg10[2] <= 999990.776268
 R4780: C21 + 1e+06 alpha_ob20_seg10[3] <= 1.0000452375e+06
 R4781: C23 + 1e+06 alpha_ob20_seg10[3] <= 1.0000452375e+06
 R4782: alpha_ob20_seg10[0] + alpha_ob20_seg10[1] + alpha_ob20_seg10[2]
   + alpha_ob20_seg10[3] >= 1
 R4783: - C20 + 1e+06 alpha_ob21_seg10[0] <= 999985.657099801
 R4784: - C22 + 1e+06 alpha_ob21_seg10[0] <= 999985.657099801
 R4785: - C21 + 1e+06 alpha_ob21_seg10[1] <= 1.0000576202063023e+06
 R4786: - C23 + 1e+06 alpha_ob21_seg10[1] <= 1.0000576202063023e+06
 R4787: C20 + 1e+06 alpha_ob21_seg10[2] <= 1.000007192899801e+06
 R4788: C22 + 1e+06 alpha_ob21_seg10[2] <= 1.000007192899801e+06
 R4789: C21 + 1e+06 alpha_ob21_seg10[3] <= 999939.8298063023
 R4790: C23 + 1e+06 alpha_ob21_seg10[3] <= 999939.8298063023
 R4791: alpha_ob21_seg10[0] + alpha_ob21_seg10[1] + alpha_ob21_seg10[2]
   + alpha_ob21_seg10[3] >= 1
 R4792: - C20 + 1e+06 alpha_ob22_seg10[0] <= 999994.771829801
 R4793: - C22 + 1e+06 alpha_ob22_seg10[0] <= 999994.771829801
 R4794: - C21 + 1e+06 alpha_ob22_seg10[1] <= 1.0000615983063023e+06
 R4795: - C23 + 1e+06 alpha_ob22_seg10[1] <= 1.0000615983063023e+06
 R4796: C20 + 1e+06 alpha_ob22_seg10[2] <= 999998.078169801
 R4797: C22 + 1e+06 alpha_ob22_seg10[2] <= 999998.078169801
 R4798: C21 + 1e+06 alpha_ob22_seg10[3] <= 999935.8517063023
 R4799: C23 + 1e+06 alpha_ob22_seg10[3] <= 999935.8517063023
 R4800: alpha_ob22_seg10[0] + alpha_ob22_seg10[1] + alpha_ob22_seg10[2]
   + alpha_ob22_seg10[3] >= 1
 R4801: - C20 + 1e+06 alpha_ob23_seg10[0] <= 999980.9374508709
 R4802: - C22 + 1e+06 alpha_ob23_seg10[0] <= 999980.9374508709
 R4803: - C21 + 1e+06 alpha_ob23_seg10[1] <= 1.0000348940002755e+06
 R4804: - C23 + 1e+06 alpha_ob23_seg10[1] <= 1.0000348940002755e+06
 R4805: C20 + 1e+06 alpha_ob23_seg10[2] <= 1.0000165124508708e+06
 R4806: C22 + 1e+06 alpha_ob23_seg10[2] <= 1.0000165124508708e+06
 R4807: C21 + 1e+06 alpha_ob23_seg10[3] <= 999935.9560002756
 R4808: C23 + 1e+06 alpha_ob23_seg10[3] <= 999935.9560002756
 R4809: alpha_ob23_seg10[0] + alpha_ob23_seg10[1] + alpha_ob23_seg10[2]
   + alpha_ob23_seg10[3] >= 1
 R4810: - C20 + 1e+06 alpha_ob24_seg10[0] <= 999980.937499801
 R4811: - C22 + 1e+06 alpha_ob24_seg10[0] <= 999980.937499801
 R4812: - C21 + 1e+06 alpha_ob24_seg10[1] <= 1.0000614940072973e+06
 R4813: - C23 + 1e+06 alpha_ob24_seg10[1] <= 1.0000614940072973e+06
 R4814: C20 + 1e+06 alpha_ob24_seg10[2] <= 1.000011162499801e+06
 R4815: C22 + 1e+06 alpha_ob24_seg10[2] <= 1.000011162499801e+06
 R4816: C21 + 1e+06 alpha_ob24_seg10[3] <= 999935.9560072974
 R4817: C23 + 1e+06 alpha_ob24_seg10[3] <= 999935.9560072974
 R4818: alpha_ob24_seg10[0] + alpha_ob24_seg10[1] + alpha_ob24_seg10[2]
   + alpha_ob24_seg10[3] >= 1
 R4819: - C20 + 1e+06 alpha_ob25_seg10[0] <= 999956.8394000001
 R4820: - C22 + 1e+06 alpha_ob25_seg10[0] <= 999956.8394000001
 R4821: - C21 + 1e+06 alpha_ob25_seg10[1] <= 999952.2463
 R4822: - C23 + 1e+06 alpha_ob25_seg10[1] <= 999952.2463
 R4823: C20 + 1e+06 alpha_ob25_seg10[2] <= 1.0000097606e+06
 R4824: C22 + 1e+06 alpha_ob25_seg10[2] <= 1.0000097606e+06
 R4825: C21 + 1e+06 alpha_ob25_seg10[3] <= 1.0000452037e+06
 R4826: C23 + 1e+06 alpha_ob25_seg10[3] <= 1.0000452037e+06
 R4827: alpha_ob25_seg10[0] + alpha_ob25_seg10[1] + alpha_ob25_seg10[2]
   + alpha_ob25_seg10[3] >= 1
 R4828: - C20 + 1e+06 alpha_ob26_seg10[0] <= 999980.9375348955
 R4829: - C22 + 1e+06 alpha_ob26_seg10[0] <= 999980.9375348955
 R4830: - C21 + 1e+06 alpha_ob26_seg10[1] <= 1.0000160439997246e+06
 R4831: - C23 + 1e+06 alpha_ob26_seg10[1] <= 1.0000160439997246e+06
 R4832: C20 + 1e+06 alpha_ob26_seg10[2] <= 1.0000165125348954e+06
 R4833: C22 + 1e+06 alpha_ob26_seg10[2] <= 1.0000165125348954e+06
 R4834: C21 + 1e+06 alpha_ob26_seg10[3] <= 999962.5559997245
 R4835: C23 + 1e+06 alpha_ob26_seg10[3] <= 999962.5559997245
 R4836: alpha_ob26_seg10[0] + alpha_ob26_seg10[1] + alpha_ob26_seg10[2]
   + alpha_ob26_seg10[3] >= 1
 R4837: - C20 + 1e+06 alpha_ob27_seg10[0] <= 999956.839399801
 R4838: - C22 + 1e+06 alpha_ob27_seg10[0] <= 999956.839399801
 R4839: - C21 + 1e+06 alpha_ob27_seg10[1] <= 999925.3963434525
 R4840: - C23 + 1e+06 alpha_ob27_seg10[1] <= 999925.3963434525
 R4841: C20 + 1e+06 alpha_ob27_seg10[2] <= 1.0000080105998011e+06
 R4842: C22 + 1e+06 alpha_ob27_seg10[2] <= 1.0000080105998011e+06
 R4843: C21 + 1e+06 alpha_ob27_seg10[3] <= 1.0000720537434525e+06
 R4844: C23 + 1e+06 alpha_ob27_seg10[3] <= 1.0000720537434525e+06
 R4845: alpha_ob27_seg10[0] + alpha_ob27_seg10[1] + alpha_ob27_seg10[2]
   + alpha_ob27_seg10[3] >= 1
 R4846: - C20 + 1e+06 alpha_ob28_seg10[0] <= 999980.9374998009
 R4847: - C22 + 1e+06 alpha_ob28_seg10[0] <= 999980.9374998009
 R4848: - C21 + 1e+06 alpha_ob28_seg10[1] <= 1.0000160440053071e+06
 R4849: - C23 + 1e+06 alpha_ob28_seg10[1] <= 1.0000160440053071e+06
 R4850: C20 + 1e+06 alpha_ob28_seg10[2] <= 1.000012662499801e+06
 R4851: C22 + 1e+06 alpha_ob28_seg10[2] <= 1.000012662499801e+06
 R4852: C21 + 1e+06 alpha_ob28_seg10[3] <= 999981.4060053072
 R4853: C23 + 1e+06 alpha_ob28_seg10[3] <= 999981.4060053072
 R4854: alpha_ob28_seg10[0] + alpha_ob28_seg10[1] + alpha_ob28_seg10[2]
   + alpha_ob28_seg10[3] >= 1
 R4855: - C20 + 1e+06 alpha_ob29_seg10[0] <= 999983.6053459151
 R4856: - C22 + 1e+06 alpha_ob29_seg10[0] <= 999983.6053459151
 R4857: - C21 + 1e+06 alpha_ob29_seg10[1] <= 999987.7722497246
 R4858: - C23 + 1e+06 alpha_ob29_seg10[1] <= 999987.7722497246
 R4859: C20 + 1e+06 alpha_ob29_seg10[2] <= 1.000013844745915e+06
 R4860: C22 + 1e+06 alpha_ob29_seg10[2] <= 1.000013844745915e+06
 R4861: C21 + 1e+06 alpha_ob29_seg10[3] <= 999984.8277497246
 R4862: C23 + 1e+06 alpha_ob29_seg10[3] <= 999984.8277497246
 R4863: alpha_ob29_seg10[0] + alpha_ob29_seg10[1] + alpha_ob29_seg10[2]
   + alpha_ob29_seg10[3] >= 1
 R4864: - C20 + 1e+06 alpha_ob30_seg10[0] <= 999958.8717898987
 R4865: - C22 + 1e+06 alpha_ob30_seg10[0] <= 999958.8717898987
 R4866: - C21 + 1e+06 alpha_ob30_seg10[1] <= 999987.7837602755
 R4867: - C23 + 1e+06 alpha_ob30_seg10[1] <= 999987.7837602755
 R4868: C20 + 1e+06 alpha_ob30_seg10[2] <= 1.0000385781898986e+06
 R4869: C22 + 1e+06 alpha_ob30_seg10[2] <= 1.0000385781898986e+06
 R4870: C21 + 1e+06 alpha_ob30_seg10[3] <= 1.0000043162402755e+06
 R4871: C23 + 1e+06 alpha_ob30_seg10[3] <= 1.0000043162402755e+06
 R4872: alpha_ob30_seg10[0] + alpha_ob30_seg10[1] + alpha_ob30_seg10[2]
   + alpha_ob30_seg10[3] >= 1
 R4873: - C20 + 1e+06 alpha_ob31_seg10[0] <= 999939.2718
 R4874: - C22 + 1e+06 alpha_ob31_seg10[0] <= 999939.2718
 R4875: - C21 + 1e+06 alpha_ob31_seg10[1] <= 999993.13376
 R4876: - C23 + 1e+06 alpha_ob31_seg10[1] <= 999993.13376
 R4877: C20 + 1e+06 alpha_ob31_seg10[2] <= 1.0000385782e+06
 R4878: C22 + 1e+06 alpha_ob31_seg10[2] <= 1.0000385782e+06
 R4879: C21 + 1e+06 alpha_ob31_seg10[3] <= 1.00000431624e+06
 R4880: C23 + 1e+06 alpha_ob31_seg10[3] <= 1.00000431624e+06
 R4881: alpha_ob31_seg10[0] + alpha_ob31_seg10[1] + alpha_ob31_seg10[2]
   + alpha_ob31_seg10[3] >= 1
 R4882: - C20 + 1e+06 alpha_ob32_seg10[0] <= 999939.2718101013
 R4883: - C22 + 1e+06 alpha_ob32_seg10[0] <= 999939.2718101013
 R4884: - C21 + 1e+06 alpha_ob32_seg10[1] <= 999987.7837597246
 R4885: - C23 + 1e+06 alpha_ob32_seg10[1] <= 999987.7837597246
 R4886: C20 + 1e+06 alpha_ob32_seg10[2] <= 1.0000581782101013e+06
 R4887: C22 + 1e+06 alpha_ob32_seg10[2] <= 1.0000581782101013e+06
 R4888: C21 + 1e+06 alpha_ob32_seg10[3] <= 1.0000043162397245e+06
 R4889: C23 + 1e+06 alpha_ob32_seg10[3] <= 1.0000043162397245e+06
 R4890: alpha_ob32_seg10[0] + alpha_ob32_seg10[1] + alpha_ob32_seg10[2]
   + alpha_ob32_seg10[3] >= 1
 R4891: - C20 + 1e+06 alpha_ob33_seg10[0] <= 999939.271799801
 R4892: - C22 + 1e+06 alpha_ob33_seg10[0] <= 999939.271799801
 R4893: - C21 + 1e+06 alpha_ob33_seg10[1] <= 999987.7838262042
 R4894: - C23 + 1e+06 alpha_ob33_seg10[1] <= 999987.7838262042
 R4895: C20 + 1e+06 alpha_ob33_seg10[2] <= 1.000038578199801e+06
 R4896: C22 + 1e+06 alpha_ob33_seg10[2] <= 1.000038578199801e+06
 R4897: C21 + 1e+06 alpha_ob33_seg10[3] <= 1.0000096662262043e+06
 R4898: C23 + 1e+06 alpha_ob33_seg10[3] <= 1.0000096662262043e+06
 R4899: alpha_ob33_seg10[0] + alpha_ob33_seg10[1] + alpha_ob33_seg10[2]
   + alpha_ob33_seg10[3] >= 1
 R4900: - C20 + 1e+06 alpha_ob34_seg10[0] <= 999926.0708
 R4901: - C22 + 1e+06 alpha_ob34_seg10[0] <= 999926.0708
 R4902: - C21 + 1e+06 alpha_ob34_seg10[1] <= 999983.7196
 R4903: - C23 + 1e+06 alpha_ob34_seg10[1] <= 999983.7196
 R4904: C20 + 1e+06 alpha_ob34_seg10[2] <= 1.0000250292e+06
 R4905: C22 + 1e+06 alpha_ob34_seg10[2] <= 1.0000250292e+06
 R4906: C21 + 1e+06 alpha_ob34_seg10[3] <= 1.0000137304e+06
 R4907: C23 + 1e+06 alpha_ob34_seg10[3] <= 1.0000137304e+06
 R4908: alpha_ob34_seg10[0] + alpha_ob34_seg10[1] + alpha_ob34_seg10[2]
   + alpha_ob34_seg10[3] >= 1
 R4909: - C20 + 1e+06 alpha_ob35_seg10[0] <= 999926.0708293857
 R4910: - C22 + 1e+06 alpha_ob35_seg10[0] <= 999926.0708293857
 R4911: - C21 + 1e+06 alpha_ob35_seg10[1] <= 999967.8695997245
 R4912: - C23 + 1e+06 alpha_ob35_seg10[1] <= 999967.8695997245
 R4913: C20 + 1e+06 alpha_ob35_seg10[2] <= 1.0000713792293856e+06
 R4914: C22 + 1e+06 alpha_ob35_seg10[2] <= 1.0000713792293856e+06
 R4915: C21 + 1e+06 alpha_ob35_seg10[3] <= 1.0000137303997246e+06
 R4916: C23 + 1e+06 alpha_ob35_seg10[3] <= 1.0000137303997246e+06
 R4917: alpha_ob35_seg10[0] + alpha_ob35_seg10[1] + alpha_ob35_seg10[2]
   + alpha_ob35_seg10[3] >= 1
 R4918: - C20 + 1e+06 alpha_ob36_seg10[0] <= 999972.6917508709
 R4919: - C22 + 1e+06 alpha_ob36_seg10[0] <= 999972.6917508709
 R4920: - C21 + 1e+06 alpha_ob36_seg10[1] <= 999957.1644002756
 R4921: - C23 + 1e+06 alpha_ob36_seg10[1] <= 999957.1644002756
 R4922: C20 + 1e+06 alpha_ob36_seg10[2] <= 1.0000247581508709e+06
 R4923: C22 + 1e+06 alpha_ob36_seg10[2] <= 1.0000247581508709e+06
 R4924: C21 + 1e+06 alpha_ob36_seg10[3] <= 1.0000136856002755e+06
 R4925: C23 + 1e+06 alpha_ob36_seg10[3] <= 1.0000136856002755e+06
 R4926: alpha_ob36_seg10[0] + alpha_ob36_seg10[1] + alpha_ob36_seg10[2]
   + alpha_ob36_seg10[3] >= 1
 R4927: - C20 + 1e+06 alpha_ob37_seg10[0] <= 999939.0009674003
 R4928: - C22 + 1e+06 alpha_ob37_seg10[0] <= 999939.0009674003
 R4929: - C21 + 1e+06 alpha_ob37_seg10[1] <= 999961.9516002756
 R4930: - C23 + 1e+06 alpha_ob37_seg10[1] <= 999961.9516002756
 R4931: C20 + 1e+06 alpha_ob37_seg10[2] <= 1.0000584489674004e+06
 R4932: C22 + 1e+06 alpha_ob37_seg10[2] <= 1.0000584489674004e+06
 R4933: C21 + 1e+06 alpha_ob37_seg10[3] <= 1.0000178984002755e+06
 R4934: C23 + 1e+06 alpha_ob37_seg10[3] <= 1.0000178984002755e+06
 R4935: alpha_ob37_seg10[0] + alpha_ob37_seg10[1] + alpha_ob37_seg10[2]
   + alpha_ob37_seg10[3] >= 1
 R4936: - C20 + 1e+06 alpha_ob38_seg10[0] <= 999994.9164998011
 R4937: - C22 + 1e+06 alpha_ob38_seg10[0] <= 999994.9164998011
 R4938: - C21 + 1e+06 alpha_ob38_seg10[1] <= 999925.4284782809
 R4939: - C23 + 1e+06 alpha_ob38_seg10[1] <= 999925.4284782809
 R4940: C20 + 1e+06 alpha_ob38_seg10[2] <= 999943.6834998011
 R4941: C22 + 1e+06 alpha_ob38_seg10[2] <= 999943.6834998011
 R4942: C21 + 1e+06 alpha_ob38_seg10[3] <= 1.0000720216782809e+06
 R4943: C23 + 1e+06 alpha_ob38_seg10[3] <= 1.0000720216782809e+06
 R4944: alpha_ob38_seg10[0] + alpha_ob38_seg10[1] + alpha_ob38_seg10[2]
   + alpha_ob38_seg10[3] >= 1
 R4945: - C20 + 1e+06 alpha_ob39_seg10[0] <= 999914.52327842
 R4946: - C22 + 1e+06 alpha_ob39_seg10[0] <= 999914.52327842
 R4947: - C21 + 1e+06 alpha_ob39_seg10[1] <= 999993.6552362755
 R4948: - C23 + 1e+06 alpha_ob39_seg10[1] <= 999993.6552362755
 R4949: C20 + 1e+06 alpha_ob39_seg10[2] <= 1.0000829266784199e+06
 R4950: C22 + 1e+06 alpha_ob39_seg10[2] <= 1.0000829266784199e+06
 R4951: C21 + 1e+06 alpha_ob39_seg10[3] <= 999992.1947642756
 R4952: C23 + 1e+06 alpha_ob39_seg10[3] <= 999992.1947642756
 R4953: alpha_ob39_seg10[0] + alpha_ob39_seg10[1] + alpha_ob39_seg10[2]
   + alpha_ob39_seg10[3] >= 1
 R4954: - C20 + 1e+06 alpha_ob40_seg10[0] <= 999912.265199801
 R4955: - C22 + 1e+06 alpha_ob40_seg10[0] <= 999912.265199801
 R4956: - C21 + 1e+06 alpha_ob40_seg10[1] <= 999987.7842112777
 R4957: - C23 + 1e+06 alpha_ob40_seg10[1] <= 999987.7842112777
 R4958: C20 + 1e+06 alpha_ob40_seg10[2] <= 1.000076834799801e+06
 R4959: C22 + 1e+06 alpha_ob40_seg10[2] <= 1.000076834799801e+06
 R4960: C21 + 1e+06 alpha_ob40_seg10[3] <= 1.0000096658112778e+06
 R4961: C23 + 1e+06 alpha_ob40_seg10[3] <= 1.0000096658112778e+06
 R4962: alpha_ob40_seg10[0] + alpha_ob40_seg10[1] + alpha_ob40_seg10[2]
   + alpha_ob40_seg10[3] >= 1
 R4963: - C20 + 1e+06 alpha_ob41_seg10[0] <= 999920.6152119379
 R4964: - C22 + 1e+06 alpha_ob41_seg10[0] <= 999920.6152119379
 R4965: - C21 + 1e+06 alpha_ob41_seg10[1] <= 999981.4341997246
 R4966: - C23 + 1e+06 alpha_ob41_seg10[1] <= 999981.4341997246
 R4967: C20 + 1e+06 alpha_ob41_seg10[2] <= 1.0000768348119379e+06
 R4968: C22 + 1e+06 alpha_ob41_seg10[2] <= 1.0000768348119379e+06
 R4969: C21 + 1e+06 alpha_ob41_seg10[3] <= 1.0000096657997245e+06
 R4970: C23 + 1e+06 alpha_ob41_seg10[3] <= 1.0000096657997245e+06
 R4971: alpha_ob41_seg10[0] + alpha_ob41_seg10[1] + alpha_ob41_seg10[2]
   + alpha_ob41_seg10[3] >= 1
 R4972: - C20 + 1e+06 alpha_ob42_seg10[0] <= 999919.2652
 R4973: - C22 + 1e+06 alpha_ob42_seg10[0] <= 999919.2652
 R4974: - C21 + 1e+06 alpha_ob42_seg10[1] <= 999981.4342
 R4975: - C23 + 1e+06 alpha_ob42_seg10[1] <= 999981.4342
 R4976: C20 + 1e+06 alpha_ob42_seg10[2] <= 1.0000768348e+06
 R4977: C22 + 1e+06 alpha_ob42_seg10[2] <= 1.0000768348e+06
 R4978: C21 + 1e+06 alpha_ob42_seg10[3] <= 1.0000160158000001e+06
 R4979: C23 + 1e+06 alpha_ob42_seg10[3] <= 1.0000160158000001e+06
 R4980: alpha_ob42_seg10[0] + alpha_ob42_seg10[1] + alpha_ob42_seg10[2]
   + alpha_ob42_seg10[3] >= 1
 R4981: - C20 + 1e+06 alpha_ob43_seg10[0] <= 999919.2652454559
 R4982: - C22 + 1e+06 alpha_ob43_seg10[0] <= 999919.2652454559
 R4983: - C21 + 1e+06 alpha_ob43_seg10[1] <= 999956.8341997246
 R4984: - C23 + 1e+06 alpha_ob43_seg10[1] <= 999956.8341997246
 R4985: C20 + 1e+06 alpha_ob43_seg10[2] <= 1.0000781848454559e+06
 R4986: C22 + 1e+06 alpha_ob43_seg10[2] <= 1.0000781848454559e+06
 R4987: C21 + 1e+06 alpha_ob43_seg10[3] <= 1.0000160157997246e+06
 R4988: C23 + 1e+06 alpha_ob43_seg10[3] <= 1.0000160157997246e+06
 R4989: alpha_ob43_seg10[0] + alpha_ob43_seg10[1] + alpha_ob43_seg10[2]
   + alpha_ob43_seg10[3] >= 1
 R4990: - C20 + 1e+06 alpha_ob44_seg10[0] <= 999991.83683
 R4991: - C22 + 1e+06 alpha_ob44_seg10[0] <= 999991.83683
 R4992: - C21 + 1e+06 alpha_ob44_seg10[1] <= 999940.589
 R4993: - C23 + 1e+06 alpha_ob44_seg10[1] <= 999940.589
 R4994: C20 + 1e+06 alpha_ob44_seg10[2] <= 1.00000401317e+06
 R4995: C22 + 1e+06 alpha_ob44_seg10[2] <= 1.00000401317e+06
 R4996: C21 + 1e+06 alpha_ob44_seg10[3] <= 1.000056861e+06
 R4997: C23 + 1e+06 alpha_ob44_seg10[3] <= 1.000056861e+06
 R4998: alpha_ob44_seg10[0] + alpha_ob44_seg10[1] + alpha_ob44_seg10[2]
   + alpha_ob44_seg10[3] >= 1
 R4999: - C20 + 1e+06 alpha_ob45_seg10[0] <= 999991.8368084199
 R5000: - C22 + 1e+06 alpha_ob45_seg10[0] <= 999991.8368084199
 R5001: - C21 + 1e+06 alpha_ob45_seg10[1] <= 999940.5890002755
 R5002: - C23 + 1e+06 alpha_ob45_seg10[1] <= 999940.5890002755
 R5003: C20 + 1e+06 alpha_ob45_seg10[2] <= 1.0000056131484199e+06
 R5004: C22 + 1e+06 alpha_ob45_seg10[2] <= 1.0000056131484199e+06
 R5005: C21 + 1e+06 alpha_ob45_seg10[3] <= 1.0000452610002755e+06
 R5006: C23 + 1e+06 alpha_ob45_seg10[3] <= 1.0000452610002755e+06
 R5007: alpha_ob45_seg10[0] + alpha_ob45_seg10[1] + alpha_ob45_seg10[2]
   + alpha_ob45_seg10[3] >= 1
 R5008: - C20 + 1e+06 alpha_ob46_seg10[0] <= 1.0000764021508709e+06
 R5009: - C22 + 1e+06 alpha_ob46_seg10[0] <= 1.0000764021508709e+06
 R5010: - C21 + 1e+06 alpha_ob46_seg10[1] <= 999957.1027002756
 R5011: - C23 + 1e+06 alpha_ob46_seg10[1] <= 999957.1027002756
 R5012: C20 + 1e+06 alpha_ob46_seg10[2] <= 999921.0477508709
 R5013: C22 + 1e+06 alpha_ob46_seg10[2] <= 999921.0477508709
 R5014: C21 + 1e+06 alpha_ob46_seg10[3] <= 1.0000137473002756e+06
 R5015: C23 + 1e+06 alpha_ob46_seg10[3] <= 1.0000137473002756e+06
 R5016: alpha_ob46_seg10[0] + alpha_ob46_seg10[1] + alpha_ob46_seg10[2]
   + alpha_ob46_seg10[3] >= 1
 R5017: - C20 + 1e+06 alpha_ob47_seg10[0] <= 1.0000445522e+06
 R5018: - C22 + 1e+06 alpha_ob47_seg10[0] <= 1.0000445522e+06
 R5019: - C21 + 1e+06 alpha_ob47_seg10[1] <= 999983.7027
 R5020: - C23 + 1e+06 alpha_ob47_seg10[1] <= 999983.7027
 R5021: C20 + 1e+06 alpha_ob47_seg10[2] <= 999921.0477999999
 R5022: C22 + 1e+06 alpha_ob47_seg10[2] <= 999921.0477999999
 R5023: C21 + 1e+06 alpha_ob47_seg10[3] <= 1.0000137473e+06
 R5024: C23 + 1e+06 alpha_ob47_seg10[3] <= 1.0000137473e+06
 R5025: alpha_ob47_seg10[0] + alpha_ob47_seg10[1] + alpha_ob47_seg10[2]
   + alpha_ob47_seg10[3] >= 1
 R5026: - C20 + 1e+06 alpha_ob48_seg10[0] <= 1.0000445522183661e+06
 R5027: - C22 + 1e+06 alpha_ob48_seg10[0] <= 1.0000445522183661e+06
 R5028: - C21 + 1e+06 alpha_ob48_seg10[1] <= 999973.8526997245
 R5029: - C23 + 1e+06 alpha_ob48_seg10[1] <= 999973.8526997245
 R5030: C20 + 1e+06 alpha_ob48_seg10[2] <= 999952.8978183661
 R5031: C22 + 1e+06 alpha_ob48_seg10[2] <= 999952.8978183661
 R5032: C21 + 1e+06 alpha_ob48_seg10[3] <= 1.0000137472997245e+06
 R5033: C23 + 1e+06 alpha_ob48_seg10[3] <= 1.0000137472997245e+06
 R5034: alpha_ob48_seg10[0] + alpha_ob48_seg10[1] + alpha_ob48_seg10[2]
   + alpha_ob48_seg10[3] >= 1
 R5035: - C20 + 1e+06 alpha_ob49_seg10[0] <= 1.000064332081634e+06
 R5036: - C22 + 1e+06 alpha_ob49_seg10[0] <= 1.000064332081634e+06
 R5037: - C21 + 1e+06 alpha_ob49_seg10[1] <= 999973.9401002756
 R5038: - C23 + 1e+06 alpha_ob49_seg10[1] <= 999973.9401002756
 R5039: C20 + 1e+06 alpha_ob49_seg10[2] <= 999933.1178816339
 R5040: C22 + 1e+06 alpha_ob49_seg10[2] <= 999933.1178816339
 R5041: C21 + 1e+06 alpha_ob49_seg10[3] <= 1.0000136599002755e+06
 R5042: C23 + 1e+06 alpha_ob49_seg10[3] <= 1.0000136599002755e+06
 R5043: alpha_ob49_seg10[0] + alpha_ob49_seg10[1] + alpha_ob49_seg10[2]
   + alpha_ob49_seg10[3] >= 1
 R5044: - C20 + 1e+06 alpha_ob50_seg10[0] <= 1.0000643321013774e+06
 R5045: - C22 + 1e+06 alpha_ob50_seg10[0] <= 1.0000643321013774e+06
 R5046: - C21 + 1e+06 alpha_ob50_seg10[1] <= 999983.1900997246
 R5047: - C23 + 1e+06 alpha_ob50_seg10[1] <= 999983.1900997246
 R5048: C20 + 1e+06 alpha_ob50_seg10[2] <= 999933.1179013775
 R5049: C22 + 1e+06 alpha_ob50_seg10[2] <= 999933.1179013775
 R5050: C21 + 1e+06 alpha_ob50_seg10[3] <= 1.0000136598997245e+06
 R5051: C23 + 1e+06 alpha_ob50_seg10[3] <= 1.0000136598997245e+06
 R5052: alpha_ob50_seg10[0] + alpha_ob50_seg10[1] + alpha_ob50_seg10[2]
   + alpha_ob50_seg10[3] >= 1
 R5053: - C22 + 1e+06 alpha_ob0_seg11[0] <= 1.0000284170499525e+06
 R5054: - C24 + 1e+06 alpha_ob0_seg11[0] <= 1.0000284170499525e+06
 R5055: - C23 + 1e+06 alpha_ob0_seg11[1] <= 999957.0843002755
 R5056: - C25 + 1e+06 alpha_ob0_seg11[1] <= 999957.0843002755
 R5057: C22 + 1e+06 alpha_ob0_seg11[2] <= 999969.0328499526
 R5058: C24 + 1e+06 alpha_ob0_seg11[2] <= 999969.0328499526
 R5059: C23 + 1e+06 alpha_ob0_seg11[3] <= 1.0000132657002755e+06
 R5060: C25 + 1e+06 alpha_ob0_seg11[3] <= 1.0000132657002755e+06
 R5061: alpha_ob0_seg11[0] + alpha_ob0_seg11[1] + alpha_ob0_seg11[2]
   + alpha_ob0_seg11[3] >= 1
 R5062: - C22 + 1e+06 alpha_ob1_seg11[0] <= 1.000028417099801e+06
 R5063: - C24 + 1e+06 alpha_ob1_seg11[0] <= 1.000028417099801e+06
 R5064: - C23 + 1e+06 alpha_ob1_seg11[1] <= 999984.1843155898
 R5065: - C25 + 1e+06 alpha_ob1_seg11[1] <= 999984.1843155898
 R5066: C22 + 1e+06 alpha_ob1_seg11[2] <= 999957.4328998009
 R5067: C24 + 1e+06 alpha_ob1_seg11[2] <= 999957.4328998009
 R5068: C23 + 1e+06 alpha_ob1_seg11[3] <= 1.0000132657155899e+06
 R5069: C25 + 1e+06 alpha_ob1_seg11[3] <= 1.0000132657155899e+06
 R5070: alpha_ob1_seg11[0] + alpha_ob1_seg11[1] + alpha_ob1_seg11[2]
   + alpha_ob1_seg11[3] >= 1
 R5071: - C22 + 1e+06 alpha_ob2_seg11[0] <= 1.0000400171500475e+06
 R5072: - C24 + 1e+06 alpha_ob2_seg11[0] <= 1.0000400171500475e+06
 R5073: - C23 + 1e+06 alpha_ob2_seg11[1] <= 999957.0842997245
 R5074: - C25 + 1e+06 alpha_ob2_seg11[1] <= 999957.0842997245
 R5075: C22 + 1e+06 alpha_ob2_seg11[2] <= 999957.4329500474
 R5076: C24 + 1e+06 alpha_ob2_seg11[2] <= 999957.4329500474
 R5077: C23 + 1e+06 alpha_ob2_seg11[3] <= 1.0000132656997246e+06
 R5078: C25 + 1e+06 alpha_ob2_seg11[3] <= 1.0000132656997246e+06
 R5079: alpha_ob2_seg11[0] + alpha_ob2_seg11[1] + alpha_ob2_seg11[2]
   + alpha_ob2_seg11[3] >= 1
 R5080: - C22 + 1e+06 alpha_ob3_seg11[0] <= 1.0000109161752058e+06
 R5081: - C24 + 1e+06 alpha_ob3_seg11[0] <= 1.0000109161752058e+06
 R5082: - C23 + 1e+06 alpha_ob3_seg11[1] <= 999970.2681002755
 R5083: - C25 + 1e+06 alpha_ob3_seg11[1] <= 999970.2681002755
 R5084: C22 + 1e+06 alpha_ob3_seg11[2] <= 999986.5337752058
 R5085: C24 + 1e+06 alpha_ob3_seg11[2] <= 999986.5337752058
 R5086: C23 + 1e+06 alpha_ob3_seg11[3] <= 1.0000138319002755e+06
 R5087: C25 + 1e+06 alpha_ob3_seg11[3] <= 1.0000138319002755e+06
 R5088: alpha_ob3_seg11[0] + alpha_ob3_seg11[1] + alpha_ob3_seg11[2]
   + alpha_ob3_seg11[3] >= 1
 R5089: - C22 + 1e+06 alpha_ob4_seg11[0] <= 1.000010916199801e+06
 R5090: - C24 + 1e+06 alpha_ob4_seg11[0] <= 1.000010916199801e+06
 R5091: - C23 + 1e+06 alpha_ob4_seg11[1] <= 999983.6181179117
 R5092: - C25 + 1e+06 alpha_ob4_seg11[1] <= 999983.6181179117
 R5093: C22 + 1e+06 alpha_ob4_seg11[2] <= 999973.183799801
 R5094: C24 + 1e+06 alpha_ob4_seg11[2] <= 999973.183799801
 R5095: C23 + 1e+06 alpha_ob4_seg11[3] <= 1.0000138319179118e+06
 R5096: C25 + 1e+06 alpha_ob4_seg11[3] <= 1.0000138319179118e+06
 R5097: alpha_ob4_seg11[0] + alpha_ob4_seg11[1] + alpha_ob4_seg11[2]
   + alpha_ob4_seg11[3] >= 1
 R5098: - C22 + 1e+06 alpha_ob5_seg11[0] <= 999992.49102
 R5099: - C24 + 1e+06 alpha_ob5_seg11[0] <= 999992.49102
 R5100: - C23 + 1e+06 alpha_ob5_seg11[1] <= 999957.1016000001
 R5101: - C25 + 1e+06 alpha_ob5_seg11[1] <= 999957.1016000001
 R5102: C22 + 1e+06 alpha_ob5_seg11[2] <= 999986.35898
 R5103: C24 + 1e+06 alpha_ob5_seg11[2] <= 999986.35898
 R5104: C23 + 1e+06 alpha_ob5_seg11[3] <= 1.0000403484e+06
 R5105: C25 + 1e+06 alpha_ob5_seg11[3] <= 1.0000403484e+06
 R5106: alpha_ob5_seg11[0] + alpha_ob5_seg11[1] + alpha_ob5_seg11[2]
   + alpha_ob5_seg11[3] >= 1
 R5107: - C22 + 1e+06 alpha_ob6_seg11[0] <= 999981.3909999999
 R5108: - C24 + 1e+06 alpha_ob6_seg11[0] <= 999981.3909999999
 R5109: - C23 + 1e+06 alpha_ob6_seg11[1] <= 999957.1016000001
 R5110: - C25 + 1e+06 alpha_ob6_seg11[1] <= 999957.1016000001
 R5111: C22 + 1e+06 alpha_ob6_seg11[2] <= 1.000004959e+06
 R5112: C24 + 1e+06 alpha_ob6_seg11[2] <= 1.000004959e+06
 R5113: C23 + 1e+06 alpha_ob6_seg11[3] <= 1.0000403484e+06
 R5114: C25 + 1e+06 alpha_ob6_seg11[3] <= 1.0000403484e+06
 R5115: alpha_ob6_seg11[0] + alpha_ob6_seg11[1] + alpha_ob6_seg11[2]
   + alpha_ob6_seg11[3] >= 1
 R5116: - C22 + 1e+06 alpha_ob7_seg11[0] <= 999981.3909825523
 R5117: - C24 + 1e+06 alpha_ob7_seg11[0] <= 999981.3909825523
 R5118: - C23 + 1e+06 alpha_ob7_seg11[1] <= 999957.1016002755
 R5119: - C25 + 1e+06 alpha_ob7_seg11[1] <= 999957.1016002755
 R5120: C22 + 1e+06 alpha_ob7_seg11[2] <= 1.0000160589825523e+06
 R5121: C24 + 1e+06 alpha_ob7_seg11[2] <= 1.0000160589825523e+06
 R5122: C23 + 1e+06 alpha_ob7_seg11[3] <= 1.0000309984002755e+06
 R5123: C25 + 1e+06 alpha_ob7_seg11[3] <= 1.0000309984002755e+06
 R5124: alpha_ob7_seg11[0] + alpha_ob7_seg11[1] + alpha_ob7_seg11[2]
   + alpha_ob7_seg11[3] >= 1
 R5125: - C22 + 1e+06 alpha_ob8_seg11[0] <= 999981.3867954084
 R5126: - C24 + 1e+06 alpha_ob8_seg11[0] <= 999981.3867954084
 R5127: - C23 + 1e+06 alpha_ob8_seg11[1] <= 999968.8595002755
 R5128: - C25 + 1e+06 alpha_ob8_seg11[1] <= 999968.8595002755
 R5129: C22 + 1e+06 alpha_ob8_seg11[2] <= 1.0000160631954084e+06
 R5130: C24 + 1e+06 alpha_ob8_seg11[2] <= 1.0000160631954084e+06
 R5131: C23 + 1e+06 alpha_ob8_seg11[3] <= 1.0000262405002755e+06
 R5132: C25 + 1e+06 alpha_ob8_seg11[3] <= 1.0000262405002755e+06
 R5133: alpha_ob8_seg11[0] + alpha_ob8_seg11[1] + alpha_ob8_seg11[2]
   + alpha_ob8_seg11[3] >= 1
 R5134: - C22 + 1e+06 alpha_ob9_seg11[0] <= 999981.386799801
 R5135: - C24 + 1e+06 alpha_ob9_seg11[0] <= 999981.386799801
 R5136: - C23 + 1e+06 alpha_ob9_seg11[1] <= 999971.2095013268
 R5137: - C25 + 1e+06 alpha_ob9_seg11[1] <= 999971.2095013268
 R5138: C22 + 1e+06 alpha_ob9_seg11[2] <= 1.0000152131998009e+06
 R5139: C24 + 1e+06 alpha_ob9_seg11[2] <= 1.0000152131998009e+06
 R5140: C23 + 1e+06 alpha_ob9_seg11[3] <= 1.0000262405013267e+06
 R5141: C25 + 1e+06 alpha_ob9_seg11[3] <= 1.0000262405013267e+06
 R5142: alpha_ob9_seg11[0] + alpha_ob9_seg11[1] + alpha_ob9_seg11[2]
   + alpha_ob9_seg11[3] >= 1
 R5143: - C22 + 1e+06 alpha_ob10_seg11[0] <= 999982.2368022958
 R5144: - C24 + 1e+06 alpha_ob10_seg11[0] <= 999982.2368022958
 R5145: - C23 + 1e+06 alpha_ob10_seg11[1] <= 999970.1094997245
 R5146: - C25 + 1e+06 alpha_ob10_seg11[1] <= 999970.1094997245
 R5147: C22 + 1e+06 alpha_ob10_seg11[2] <= 1.0000152132022957e+06
 R5148: C24 + 1e+06 alpha_ob10_seg11[2] <= 1.0000152132022957e+06
 R5149: C23 + 1e+06 alpha_ob10_seg11[3] <= 1.0000262404997245e+06
 R5150: C25 + 1e+06 alpha_ob10_seg11[3] <= 1.0000262404997245e+06
 R5151: alpha_ob10_seg11[0] + alpha_ob10_seg11[1] + alpha_ob10_seg11[2]
   + alpha_ob10_seg11[3] >= 1
 R5152: - C22 + 1e+06 alpha_ob11_seg11[0] <= 999982.236799801
 R5153: - C24 + 1e+06 alpha_ob11_seg11[0] <= 999982.236799801
 R5154: - C23 + 1e+06 alpha_ob11_seg11[1] <= 999970.1095291895
 R5155: - C25 + 1e+06 alpha_ob11_seg11[1] <= 999970.1095291895
 R5156: C22 + 1e+06 alpha_ob11_seg11[2] <= 999993.3631998011
 R5157: C24 + 1e+06 alpha_ob11_seg11[2] <= 999993.3631998011
 R5158: C23 + 1e+06 alpha_ob11_seg11[3] <= 1.0000273405291894e+06
 R5159: C25 + 1e+06 alpha_ob11_seg11[3] <= 1.0000273405291894e+06
 R5160: alpha_ob11_seg11[0] + alpha_ob11_seg11[1] + alpha_ob11_seg11[2]
   + alpha_ob11_seg11[3] >= 1
 R5161: - C22 + 1e+06 alpha_ob12_seg11[0] <= 999981.6050802565
 R5162: - C24 + 1e+06 alpha_ob12_seg11[0] <= 999981.6050802565
 R5163: - C23 + 1e+06 alpha_ob12_seg11[1] <= 999972.8942002755
 R5164: - C25 + 1e+06 alpha_ob12_seg11[1] <= 999972.8942002755
 R5165: C22 + 1e+06 alpha_ob12_seg11[2] <= 1.0000158448802566e+06
 R5166: C24 + 1e+06 alpha_ob12_seg11[2] <= 1.0000158448802566e+06
 R5167: C23 + 1e+06 alpha_ob12_seg11[3] <= 1.0000139558002756e+06
 R5168: C25 + 1e+06 alpha_ob12_seg11[3] <= 1.0000139558002756e+06
 R5169: alpha_ob12_seg11[0] + alpha_ob12_seg11[1] + alpha_ob12_seg11[2]
   + alpha_ob12_seg11[3] >= 1
 R5170: - C22 + 1e+06 alpha_ob13_seg11[0] <= 999981.605149801
 R5171: - C24 + 1e+06 alpha_ob13_seg11[0] <= 999981.605149801
 R5172: - C23 + 1e+06 alpha_ob13_seg11[1] <= 999983.4942391404
 R5173: - C25 + 1e+06 alpha_ob13_seg11[1] <= 999983.4942391404
 R5174: C22 + 1e+06 alpha_ob13_seg11[2] <= 999986.4948498011
 R5175: C24 + 1e+06 alpha_ob13_seg11[2] <= 999986.4948498011
 R5176: C23 + 1e+06 alpha_ob13_seg11[3] <= 1.0000139558391405e+06
 R5177: C25 + 1e+06 alpha_ob13_seg11[3] <= 1.0000139558391405e+06
 R5178: alpha_ob13_seg11[0] + alpha_ob13_seg11[1] + alpha_ob13_seg11[2]
   + alpha_ob13_seg11[3] >= 1
 R5179: - C22 + 1e+06 alpha_ob14_seg11[0] <= 999997.6803502565
 R5180: - C24 + 1e+06 alpha_ob14_seg11[0] <= 999997.6803502565
 R5181: - C23 + 1e+06 alpha_ob14_seg11[1] <= 999972.7268002755
 R5182: - C25 + 1e+06 alpha_ob14_seg11[1] <= 999972.7268002755
 R5183: C22 + 1e+06 alpha_ob14_seg11[2] <= 999999.7696102565
 R5184: C24 + 1e+06 alpha_ob14_seg11[2] <= 999999.7696102565
 R5185: C23 + 1e+06 alpha_ob14_seg11[3] <= 1.0000141232002755e+06
 R5186: C25 + 1e+06 alpha_ob14_seg11[3] <= 1.0000141232002755e+06
 R5187: alpha_ob14_seg11[0] + alpha_ob14_seg11[1] + alpha_ob14_seg11[2]
   + alpha_ob14_seg11[3] >= 1
 R5188: - C22 + 1e+06 alpha_ob15_seg11[0] <= 1.0000038341502565e+06
 R5189: - C24 + 1e+06 alpha_ob15_seg11[0] <= 1.0000038341502565e+06
 R5190: - C23 + 1e+06 alpha_ob15_seg11[1] <= 999972.6012002755
 R5191: - C25 + 1e+06 alpha_ob15_seg11[1] <= 999972.6012002755
 R5192: C22 + 1e+06 alpha_ob15_seg11[2] <= 999993.6158102566
 R5193: C24 + 1e+06 alpha_ob15_seg11[2] <= 999993.6158102566
 R5194: C23 + 1e+06 alpha_ob15_seg11[3] <= 1.0000142488002755e+06
 R5195: C25 + 1e+06 alpha_ob15_seg11[3] <= 1.0000142488002755e+06
 R5196: alpha_ob15_seg11[0] + alpha_ob15_seg11[1] + alpha_ob15_seg11[2]
   + alpha_ob15_seg11[3] >= 1
 R5197: - C22 + 1e+06 alpha_ob16_seg11[0] <= 1.0000162874063333e+06
 R5198: - C24 + 1e+06 alpha_ob16_seg11[0] <= 1.0000162874063333e+06
 R5199: - C23 + 1e+06 alpha_ob16_seg11[1] <= 999982.9439602756
 R5200: - C25 + 1e+06 alpha_ob16_seg11[1] <= 999982.9439602756
 R5201: C22 + 1e+06 alpha_ob16_seg11[2] <= 999981.1624063333
 R5202: C24 + 1e+06 alpha_ob16_seg11[2] <= 999981.1624063333
 R5203: C23 + 1e+06 alpha_ob16_seg11[3] <= 999963.6560402757
 R5204: C25 + 1e+06 alpha_ob16_seg11[3] <= 999963.6560402757
 R5205: alpha_ob16_seg11[0] + alpha_ob16_seg11[1] + alpha_ob16_seg11[2]
   + alpha_ob16_seg11[3] >= 1
 R5206: - C22 + 1e+06 alpha_ob17_seg11[0] <= 999980.937509
 R5207: - C24 + 1e+06 alpha_ob17_seg11[0] <= 999980.937509
 R5208: - C23 + 1e+06 alpha_ob17_seg11[1] <= 1.000034894e+06
 R5209: - C25 + 1e+06 alpha_ob17_seg11[1] <= 1.000034894e+06
 R5210: C22 + 1e+06 alpha_ob17_seg11[2] <= 999981.162491
 R5211: C24 + 1e+06 alpha_ob17_seg11[2] <= 999981.162491
 R5212: C23 + 1e+06 alpha_ob17_seg11[3] <= 999962.556
 R5213: C25 + 1e+06 alpha_ob17_seg11[3] <= 999962.556
 R5214: alpha_ob17_seg11[0] + alpha_ob17_seg11[1] + alpha_ob17_seg11[2]
   + alpha_ob17_seg11[3] >= 1
 R5215: - C22 + 1e+06 alpha_ob18_seg11[0] <= 999990.2670277582
 R5216: - C24 + 1e+06 alpha_ob18_seg11[0] <= 999990.2670277582
 R5217: - C23 + 1e+06 alpha_ob18_seg11[1] <= 1.0000347702002756e+06
 R5218: - C25 + 1e+06 alpha_ob18_seg11[1] <= 1.0000347702002756e+06
 R5219: C22 + 1e+06 alpha_ob18_seg11[2] <= 1.0000071828877581e+06
 R5220: C24 + 1e+06 alpha_ob18_seg11[2] <= 1.0000071828877581e+06
 R5221: C23 + 1e+06 alpha_ob18_seg11[3] <= 999939.8298002756
 R5222: C25 + 1e+06 alpha_ob18_seg11[3] <= 999939.8298002756
 R5223: alpha_ob18_seg11[0] + alpha_ob18_seg11[1] + alpha_ob18_seg11[2]
   + alpha_ob18_seg11[3] >= 1
 R5224: - C22 + 1e+06 alpha_ob19_seg11[0] <= 999990.267069801
 R5225: - C24 + 1e+06 alpha_ob19_seg11[0] <= 999990.267069801
 R5226: - C23 + 1e+06 alpha_ob19_seg11[1] <= 1.0000576202069657e+06
 R5227: - C25 + 1e+06 alpha_ob19_seg11[1] <= 1.0000576202069657e+06
 R5228: C22 + 1e+06 alpha_ob19_seg11[2] <= 1.000002082929801e+06
 R5229: C24 + 1e+06 alpha_ob19_seg11[2] <= 1.000002082929801e+06
 R5230: C23 + 1e+06 alpha_ob19_seg11[3] <= 999939.8298069657
 R5231: C25 + 1e+06 alpha_ob19_seg11[3] <= 999939.8298069657
 R5232: alpha_ob19_seg11[0] + alpha_ob19_seg11[1] + alpha_ob19_seg11[2]
   + alpha_ob19_seg11[3] >= 1
 R5233: - C22 + 1e+06 alpha_ob20_seg11[0] <= 999991.073732
 R5234: - C24 + 1e+06 alpha_ob20_seg11[0] <= 999991.073732
 R5235: - C23 + 1e+06 alpha_ob20_seg11[1] <= 999952.2125
 R5236: - C25 + 1e+06 alpha_ob20_seg11[1] <= 999952.2125
 R5237: C22 + 1e+06 alpha_ob20_seg11[2] <= 999990.776268
 R5238: C24 + 1e+06 alpha_ob20_seg11[2] <= 999990.776268
 R5239: C23 + 1e+06 alpha_ob20_seg11[3] <= 1.0000452375e+06
 R5240: C25 + 1e+06 alpha_ob20_seg11[3] <= 1.0000452375e+06
 R5241: alpha_ob20_seg11[0] + alpha_ob20_seg11[1] + alpha_ob20_seg11[2]
   + alpha_ob20_seg11[3] >= 1
 R5242: - C22 + 1e+06 alpha_ob21_seg11[0] <= 999985.657099801
 R5243: - C24 + 1e+06 alpha_ob21_seg11[0] <= 999985.657099801
 R5244: - C23 + 1e+06 alpha_ob21_seg11[1] <= 1.0000576202063023e+06
 R5245: - C25 + 1e+06 alpha_ob21_seg11[1] <= 1.0000576202063023e+06
 R5246: C22 + 1e+06 alpha_ob21_seg11[2] <= 1.000007192899801e+06
 R5247: C24 + 1e+06 alpha_ob21_seg11[2] <= 1.000007192899801e+06
 R5248: C23 + 1e+06 alpha_ob21_seg11[3] <= 999939.8298063023
 R5249: C25 + 1e+06 alpha_ob21_seg11[3] <= 999939.8298063023
 R5250: alpha_ob21_seg11[0] + alpha_ob21_seg11[1] + alpha_ob21_seg11[2]
   + alpha_ob21_seg11[3] >= 1
 R5251: - C22 + 1e+06 alpha_ob22_seg11[0] <= 999994.771829801
 R5252: - C24 + 1e+06 alpha_ob22_seg11[0] <= 999994.771829801
 R5253: - C23 + 1e+06 alpha_ob22_seg11[1] <= 1.0000615983063023e+06
 R5254: - C25 + 1e+06 alpha_ob22_seg11[1] <= 1.0000615983063023e+06
 R5255: C22 + 1e+06 alpha_ob22_seg11[2] <= 999998.078169801
 R5256: C24 + 1e+06 alpha_ob22_seg11[2] <= 999998.078169801
 R5257: C23 + 1e+06 alpha_ob22_seg11[3] <= 999935.8517063023
 R5258: C25 + 1e+06 alpha_ob22_seg11[3] <= 999935.8517063023
 R5259: alpha_ob22_seg11[0] + alpha_ob22_seg11[1] + alpha_ob22_seg11[2]
   + alpha_ob22_seg11[3] >= 1
 R5260: - C22 + 1e+06 alpha_ob23_seg11[0] <= 999980.9374508709
 R5261: - C24 + 1e+06 alpha_ob23_seg11[0] <= 999980.9374508709
 R5262: - C23 + 1e+06 alpha_ob23_seg11[1] <= 1.0000348940002755e+06
 R5263: - C25 + 1e+06 alpha_ob23_seg11[1] <= 1.0000348940002755e+06
 R5264: C22 + 1e+06 alpha_ob23_seg11[2] <= 1.0000165124508708e+06
 R5265: C24 + 1e+06 alpha_ob23_seg11[2] <= 1.0000165124508708e+06
 R5266: C23 + 1e+06 alpha_ob23_seg11[3] <= 999935.9560002756
 R5267: C25 + 1e+06 alpha_ob23_seg11[3] <= 999935.9560002756
 R5268: alpha_ob23_seg11[0] + alpha_ob23_seg11[1] + alpha_ob23_seg11[2]
   + alpha_ob23_seg11[3] >= 1
 R5269: - C22 + 1e+06 alpha_ob24_seg11[0] <= 999980.937499801
 R5270: - C24 + 1e+06 alpha_ob24_seg11[0] <= 999980.937499801
 R5271: - C23 + 1e+06 alpha_ob24_seg11[1] <= 1.0000614940072973e+06
 R5272: - C25 + 1e+06 alpha_ob24_seg11[1] <= 1.0000614940072973e+06
 R5273: C22 + 1e+06 alpha_ob24_seg11[2] <= 1.000011162499801e+06
 R5274: C24 + 1e+06 alpha_ob24_seg11[2] <= 1.000011162499801e+06
 R5275: C23 + 1e+06 alpha_ob24_seg11[3] <= 999935.9560072974
 R5276: C25 + 1e+06 alpha_ob24_seg11[3] <= 999935.9560072974
 R5277: alpha_ob24_seg11[0] + alpha_ob24_seg11[1] + alpha_ob24_seg11[2]
   + alpha_ob24_seg11[3] >= 1
 R5278: - C22 + 1e+06 alpha_ob25_seg11[0] <= 999956.8394000001
 R5279: - C24 + 1e+06 alpha_ob25_seg11[0] <= 999956.8394000001
 R5280: - C23 + 1e+06 alpha_ob25_seg11[1] <= 999952.2463
 R5281: - C25 + 1e+06 alpha_ob25_seg11[1] <= 999952.2463
 R5282: C22 + 1e+06 alpha_ob25_seg11[2] <= 1.0000097606e+06
 R5283: C24 + 1e+06 alpha_ob25_seg11[2] <= 1.0000097606e+06
 R5284: C23 + 1e+06 alpha_ob25_seg11[3] <= 1.0000452037e+06
 R5285: C25 + 1e+06 alpha_ob25_seg11[3] <= 1.0000452037e+06
 R5286: alpha_ob25_seg11[0] + alpha_ob25_seg11[1] + alpha_ob25_seg11[2]
   + alpha_ob25_seg11[3] >= 1
 R5287: - C22 + 1e+06 alpha_ob26_seg11[0] <= 999980.9375348955
 R5288: - C24 + 1e+06 alpha_ob26_seg11[0] <= 999980.9375348955
 R5289: - C23 + 1e+06 alpha_ob26_seg11[1] <= 1.0000160439997246e+06
 R5290: - C25 + 1e+06 alpha_ob26_seg11[1] <= 1.0000160439997246e+06
 R5291: C22 + 1e+06 alpha_ob26_seg11[2] <= 1.0000165125348954e+06
 R5292: C24 + 1e+06 alpha_ob26_seg11[2] <= 1.0000165125348954e+06
 R5293: C23 + 1e+06 alpha_ob26_seg11[3] <= 999962.5559997245
 R5294: C25 + 1e+06 alpha_ob26_seg11[3] <= 999962.5559997245
 R5295: alpha_ob26_seg11[0] + alpha_ob26_seg11[1] + alpha_ob26_seg11[2]
   + alpha_ob26_seg11[3] >= 1
 R5296: - C22 + 1e+06 alpha_ob27_seg11[0] <= 999956.839399801
 R5297: - C24 + 1e+06 alpha_ob27_seg11[0] <= 999956.839399801
 R5298: - C23 + 1e+06 alpha_ob27_seg11[1] <= 999925.3963434525
 R5299: - C25 + 1e+06 alpha_ob27_seg11[1] <= 999925.3963434525
 R5300: C22 + 1e+06 alpha_ob27_seg11[2] <= 1.0000080105998011e+06
 R5301: C24 + 1e+06 alpha_ob27_seg11[2] <= 1.0000080105998011e+06
 R5302: C23 + 1e+06 alpha_ob27_seg11[3] <= 1.0000720537434525e+06
 R5303: C25 + 1e+06 alpha_ob27_seg11[3] <= 1.0000720537434525e+06
 R5304: alpha_ob27_seg11[0] + alpha_ob27_seg11[1] + alpha_ob27_seg11[2]
   + alpha_ob27_seg11[3] >= 1
 R5305: - C22 + 1e+06 alpha_ob28_seg11[0] <= 999980.9374998009
 R5306: - C24 + 1e+06 alpha_ob28_seg11[0] <= 999980.9374998009
 R5307: - C23 + 1e+06 alpha_ob28_seg11[1] <= 1.0000160440053071e+06
 R5308: - C25 + 1e+06 alpha_ob28_seg11[1] <= 1.0000160440053071e+06
 R5309: C22 + 1e+06 alpha_ob28_seg11[2] <= 1.000012662499801e+06
 R5310: C24 + 1e+06 alpha_ob28_seg11[2] <= 1.000012662499801e+06
 R5311: C23 + 1e+06 alpha_ob28_seg11[3] <= 999981.4060053072
 R5312: C25 + 1e+06 alpha_ob28_seg11[3] <= 999981.4060053072
 R5313: alpha_ob28_seg11[0] + alpha_ob28_seg11[1] + alpha_ob28_seg11[2]
   + alpha_ob28_seg11[3] >= 1
 R5314: - C22 + 1e+06 alpha_ob29_seg11[0] <= 999983.6053459151
 R5315: - C24 + 1e+06 alpha_ob29_seg11[0] <= 999983.6053459151
 R5316: - C23 + 1e+06 alpha_ob29_seg11[1] <= 999987.7722497246
 R5317: - C25 + 1e+06 alpha_ob29_seg11[1] <= 999987.7722497246
 R5318: C22 + 1e+06 alpha_ob29_seg11[2] <= 1.000013844745915e+06
 R5319: C24 + 1e+06 alpha_ob29_seg11[2] <= 1.000013844745915e+06
 R5320: C23 + 1e+06 alpha_ob29_seg11[3] <= 999984.8277497246
 R5321: C25 + 1e+06 alpha_ob29_seg11[3] <= 999984.8277497246
 R5322: alpha_ob29_seg11[0] + alpha_ob29_seg11[1] + alpha_ob29_seg11[2]
   + alpha_ob29_seg11[3] >= 1
 R5323: - C22 + 1e+06 alpha_ob30_seg11[0] <= 999958.8717898987
 R5324: - C24 + 1e+06 alpha_ob30_seg11[0] <= 999958.8717898987
 R5325: - C23 + 1e+06 alpha_ob30_seg11[1] <= 999987.7837602755
 R5326: - C25 + 1e+06 alpha_ob30_seg11[1] <= 999987.7837602755
 R5327: C22 + 1e+06 alpha_ob30_seg11[2] <= 1.0000385781898986e+06
 R5328: C24 + 1e+06 alpha_ob30_seg11[2] <= 1.0000385781898986e+06
 R5329: C23 + 1e+06 alpha_ob30_seg11[3] <= 1.0000043162402755e+06
 R5330: C25 + 1e+06 alpha_ob30_seg11[3] <= 1.0000043162402755e+06
 R5331: alpha_ob30_seg11[0] + alpha_ob30_seg11[1] + alpha_ob30_seg11[2]
   + alpha_ob30_seg11[3] >= 1
 R5332: - C22 + 1e+06 alpha_ob31_seg11[0] <= 999939.2718
 R5333: - C24 + 1e+06 alpha_ob31_seg11[0] <= 999939.2718
 R5334: - C23 + 1e+06 alpha_ob31_seg11[1] <= 999993.13376
 R5335: - C25 + 1e+06 alpha_ob31_seg11[1] <= 999993.13376
 R5336: C22 + 1e+06 alpha_ob31_seg11[2] <= 1.0000385782e+06
 R5337: C24 + 1e+06 alpha_ob31_seg11[2] <= 1.0000385782e+06
 R5338: C23 + 1e+06 alpha_ob31_seg11[3] <= 1.00000431624e+06
 R5339: C25 + 1e+06 alpha_ob31_seg11[3] <= 1.00000431624e+06
 R5340: alpha_ob31_seg11[0] + alpha_ob31_seg11[1] + alpha_ob31_seg11[2]
   + alpha_ob31_seg11[3] >= 1
 R5341: - C22 + 1e+06 alpha_ob32_seg11[0] <= 999939.2718101013
 R5342: - C24 + 1e+06 alpha_ob32_seg11[0] <= 999939.2718101013
 R5343: - C23 + 1e+06 alpha_ob32_seg11[1] <= 999987.7837597246
 R5344: - C25 + 1e+06 alpha_ob32_seg11[1] <= 999987.7837597246
 R5345: C22 + 1e+06 alpha_ob32_seg11[2] <= 1.0000581782101013e+06
 R5346: C24 + 1e+06 alpha_ob32_seg11[2] <= 1.0000581782101013e+06
 R5347: C23 + 1e+06 alpha_ob32_seg11[3] <= 1.0000043162397245e+06
 R5348: C25 + 1e+06 alpha_ob32_seg11[3] <= 1.0000043162397245e+06
 R5349: alpha_ob32_seg11[0] + alpha_ob32_seg11[1] + alpha_ob32_seg11[2]
   + alpha_ob32_seg11[3] >= 1
 R5350: - C22 + 1e+06 alpha_ob33_seg11[0] <= 999939.271799801
 R5351: - C24 + 1e+06 alpha_ob33_seg11[0] <= 999939.271799801
 R5352: - C23 + 1e+06 alpha_ob33_seg11[1] <= 999987.7838262042
 R5353: - C25 + 1e+06 alpha_ob33_seg11[1] <= 999987.7838262042
 R5354: C22 + 1e+06 alpha_ob33_seg11[2] <= 1.000038578199801e+06
 R5355: C24 + 1e+06 alpha_ob33_seg11[2] <= 1.000038578199801e+06
 R5356: C23 + 1e+06 alpha_ob33_seg11[3] <= 1.0000096662262043e+06
 R5357: C25 + 1e+06 alpha_ob33_seg11[3] <= 1.0000096662262043e+06
 R5358: alpha_ob33_seg11[0] + alpha_ob33_seg11[1] + alpha_ob33_seg11[2]
   + alpha_ob33_seg11[3] >= 1
 R5359: - C22 + 1e+06 alpha_ob34_seg11[0] <= 999926.0708
 R5360: - C24 + 1e+06 alpha_ob34_seg11[0] <= 999926.0708
 R5361: - C23 + 1e+06 alpha_ob34_seg11[1] <= 999983.7196
 R5362: - C25 + 1e+06 alpha_ob34_seg11[1] <= 999983.7196
 R5363: C22 + 1e+06 alpha_ob34_seg11[2] <= 1.0000250292e+06
 R5364: C24 + 1e+06 alpha_ob34_seg11[2] <= 1.0000250292e+06
 R5365: C23 + 1e+06 alpha_ob34_seg11[3] <= 1.0000137304e+06
 R5366: C25 + 1e+06 alpha_ob34_seg11[3] <= 1.0000137304e+06
 R5367: alpha_ob34_seg11[0] + alpha_ob34_seg11[1] + alpha_ob34_seg11[2]
   + alpha_ob34_seg11[3] >= 1
 R5368: - C22 + 1e+06 alpha_ob35_seg11[0] <= 999926.0708293857
 R5369: - C24 + 1e+06 alpha_ob35_seg11[0] <= 999926.0708293857
 R5370: - C23 + 1e+06 alpha_ob35_seg11[1] <= 999967.8695997245
 R5371: - C25 + 1e+06 alpha_ob35_seg11[1] <= 999967.8695997245
 R5372: C22 + 1e+06 alpha_ob35_seg11[2] <= 1.0000713792293856e+06
 R5373: C24 + 1e+06 alpha_ob35_seg11[2] <= 1.0000713792293856e+06
 R5374: C23 + 1e+06 alpha_ob35_seg11[3] <= 1.0000137303997246e+06
 R5375: C25 + 1e+06 alpha_ob35_seg11[3] <= 1.0000137303997246e+06
 R5376: alpha_ob35_seg11[0] + alpha_ob35_seg11[1] + alpha_ob35_seg11[2]
   + alpha_ob35_seg11[3] >= 1
 R5377: - C22 + 1e+06 alpha_ob36_seg11[0] <= 999972.6917508709
 R5378: - C24 + 1e+06 alpha_ob36_seg11[0] <= 999972.6917508709
 R5379: - C23 + 1e+06 alpha_ob36_seg11[1] <= 999957.1644002756
 R5380: - C25 + 1e+06 alpha_ob36_seg11[1] <= 999957.1644002756
 R5381: C22 + 1e+06 alpha_ob36_seg11[2] <= 1.0000247581508709e+06
 R5382: C24 + 1e+06 alpha_ob36_seg11[2] <= 1.0000247581508709e+06
 R5383: C23 + 1e+06 alpha_ob36_seg11[3] <= 1.0000136856002755e+06
 R5384: C25 + 1e+06 alpha_ob36_seg11[3] <= 1.0000136856002755e+06
 R5385: alpha_ob36_seg11[0] + alpha_ob36_seg11[1] + alpha_ob36_seg11[2]
   + alpha_ob36_seg11[3] >= 1
 R5386: - C22 + 1e+06 alpha_ob37_seg11[0] <= 999939.0009674003
 R5387: - C24 + 1e+06 alpha_ob37_seg11[0] <= 999939.0009674003
 R5388: - C23 + 1e+06 alpha_ob37_seg11[1] <= 999961.9516002756
 R5389: - C25 + 1e+06 alpha_ob37_seg11[1] <= 999961.9516002756
 R5390: C22 + 1e+06 alpha_ob37_seg11[2] <= 1.0000584489674004e+06
 R5391: C24 + 1e+06 alpha_ob37_seg11[2] <= 1.0000584489674004e+06
 R5392: C23 + 1e+06 alpha_ob37_seg11[3] <= 1.0000178984002755e+06
 R5393: C25 + 1e+06 alpha_ob37_seg11[3] <= 1.0000178984002755e+06
 R5394: alpha_ob37_seg11[0] + alpha_ob37_seg11[1] + alpha_ob37_seg11[2]
   + alpha_ob37_seg11[3] >= 1
 R5395: - C22 + 1e+06 alpha_ob38_seg11[0] <= 999994.9164998011
 R5396: - C24 + 1e+06 alpha_ob38_seg11[0] <= 999994.9164998011
 R5397: - C23 + 1e+06 alpha_ob38_seg11[1] <= 999925.4284782809
 R5398: - C25 + 1e+06 alpha_ob38_seg11[1] <= 999925.4284782809
 R5399: C22 + 1e+06 alpha_ob38_seg11[2] <= 999943.6834998011
 R5400: C24 + 1e+06 alpha_ob38_seg11[2] <= 999943.6834998011
 R5401: C23 + 1e+06 alpha_ob38_seg11[3] <= 1.0000720216782809e+06
 R5402: C25 + 1e+06 alpha_ob38_seg11[3] <= 1.0000720216782809e+06
 R5403: alpha_ob38_seg11[0] + alpha_ob38_seg11[1] + alpha_ob38_seg11[2]
   + alpha_ob38_seg11[3] >= 1
 R5404: - C22 + 1e+06 alpha_ob39_seg11[0] <= 999914.52327842
 R5405: - C24 + 1e+06 alpha_ob39_seg11[0] <= 999914.52327842
 R5406: - C23 + 1e+06 alpha_ob39_seg11[1] <= 999993.6552362755
 R5407: - C25 + 1e+06 alpha_ob39_seg11[1] <= 999993.6552362755
 R5408: C22 + 1e+06 alpha_ob39_seg11[2] <= 1.0000829266784199e+06
 R5409: C24 + 1e+06 alpha_ob39_seg11[2] <= 1.0000829266784199e+06
 R5410: C23 + 1e+06 alpha_ob39_seg11[3] <= 999992.1947642756
 R5411: C25 + 1e+06 alpha_ob39_seg11[3] <= 999992.1947642756
 R5412: alpha_ob39_seg11[0] + alpha_ob39_seg11[1] + alpha_ob39_seg11[2]
   + alpha_ob39_seg11[3] >= 1
 R5413: - C22 + 1e+06 alpha_ob40_seg11[0] <= 999912.265199801
 R5414: - C24 + 1e+06 alpha_ob40_seg11[0] <= 999912.265199801
 R5415: - C23 + 1e+06 alpha_ob40_seg11[1] <= 999987.7842112777
 R5416: - C25 + 1e+06 alpha_ob40_seg11[1] <= 999987.7842112777
 R5417: C22 + 1e+06 alpha_ob40_seg11[2] <= 1.000076834799801e+06
 R5418: C24 + 1e+06 alpha_ob40_seg11[2] <= 1.000076834799801e+06
 R5419: C23 + 1e+06 alpha_ob40_seg11[3] <= 1.0000096658112778e+06
 R5420: C25 + 1e+06 alpha_ob40_seg11[3] <= 1.0000096658112778e+06
 R5421: alpha_ob40_seg11[0] + alpha_ob40_seg11[1] + alpha_ob40_seg11[2]
   + alpha_ob40_seg11[3] >= 1
 R5422: - C22 + 1e+06 alpha_ob41_seg11[0] <= 999920.6152119379
 R5423: - C24 + 1e+06 alpha_ob41_seg11[0] <= 999920.6152119379
 R5424: - C23 + 1e+06 alpha_ob41_seg11[1] <= 999981.4341997246
 R5425: - C25 + 1e+06 alpha_ob41_seg11[1] <= 999981.4341997246
 R5426: C22 + 1e+06 alpha_ob41_seg11[2] <= 1.0000768348119379e+06
 R5427: C24 + 1e+06 alpha_ob41_seg11[2] <= 1.0000768348119379e+06
 R5428: C23 + 1e+06 alpha_ob41_seg11[3] <= 1.0000096657997245e+06
 R5429: C25 + 1e+06 alpha_ob41_seg11[3] <= 1.0000096657997245e+06
 R5430: alpha_ob41_seg11[0] + alpha_ob41_seg11[1] + alpha_ob41_seg11[2]
   + alpha_ob41_seg11[3] >= 1
 R5431: - C22 + 1e+06 alpha_ob42_seg11[0] <= 999919.2652
 R5432: - C24 + 1e+06 alpha_ob42_seg11[0] <= 999919.2652
 R5433: - C23 + 1e+06 alpha_ob42_seg11[1] <= 999981.4342
 R5434: - C25 + 1e+06 alpha_ob42_seg11[1] <= 999981.4342
 R5435: C22 + 1e+06 alpha_ob42_seg11[2] <= 1.0000768348e+06
 R5436: C24 + 1e+06 alpha_ob42_seg11[2] <= 1.0000768348e+06
 R5437: C23 + 1e+06 alpha_ob42_seg11[3] <= 1.0000160158000001e+06
 R5438: C25 + 1e+06 alpha_ob42_seg11[3] <= 1.0000160158000001e+06
 R5439: alpha_ob42_seg11[0] + alpha_ob42_seg11[1] + alpha_ob42_seg11[2]
   + alpha_ob42_seg11[3] >= 1
 R5440: - C22 + 1e+06 alpha_ob43_seg11[0] <= 999919.2652454559
 R5441: - C24 + 1e+06 alpha_ob43_seg11[0] <= 999919.2652454559
 R5442: - C23 + 1e+06 alpha_ob43_seg11[1] <= 999956.8341997246
 R5443: - C25 + 1e+06 alpha_ob43_seg11[1] <= 999956.8341997246
 R5444: C22 + 1e+06 alpha_ob43_seg11[2] <= 1.0000781848454559e+06
 R5445: C24 + 1e+06 alpha_ob43_seg11[2] <= 1.0000781848454559e+06
 R5446: C23 + 1e+06 alpha_ob43_seg11[3] <= 1.0000160157997246e+06
 R5447: C25 + 1e+06 alpha_ob43_seg11[3] <= 1.0000160157997246e+06
 R5448: alpha_ob43_seg11[0] + alpha_ob43_seg11[1] + alpha_ob43_seg11[2]
   + alpha_ob43_seg11[3] >= 1
 R5449: - C22 + 1e+06 alpha_ob44_seg11[0] <= 999991.83683
 R5450: - C24 + 1e+06 alpha_ob44_seg11[0] <= 999991.83683
 R5451: - C23 + 1e+06 alpha_ob44_seg11[1] <= 999940.589
 R5452: - C25 + 1e+06 alpha_ob44_seg11[1] <= 999940.589
 R5453: C22 + 1e+06 alpha_ob44_seg11[2] <= 1.00000401317e+06
 R5454: C24 + 1e+06 alpha_ob44_seg11[2] <= 1.00000401317e+06
 R5455: C23 + 1e+06 alpha_ob44_seg11[3] <= 1.000056861e+06
 R5456: C25 + 1e+06 alpha_ob44_seg11[3] <= 1.000056861e+06
 R5457: alpha_ob44_seg11[0] + alpha_ob44_seg11[1] + alpha_ob44_seg11[2]
   + alpha_ob44_seg11[3] >= 1
 R5458: - C22 + 1e+06 alpha_ob45_seg11[0] <= 999991.8368084199
 R5459: - C24 + 1e+06 alpha_ob45_seg11[0] <= 999991.8368084199
 R5460: - C23 + 1e+06 alpha_ob45_seg11[1] <= 999940.5890002755
 R5461: - C25 + 1e+06 alpha_ob45_seg11[1] <= 999940.5890002755
 R5462: C22 + 1e+06 alpha_ob45_seg11[2] <= 1.0000056131484199e+06
 R5463: C24 + 1e+06 alpha_ob45_seg11[2] <= 1.0000056131484199e+06
 R5464: C23 + 1e+06 alpha_ob45_seg11[3] <= 1.0000452610002755e+06
 R5465: C25 + 1e+06 alpha_ob45_seg11[3] <= 1.0000452610002755e+06
 R5466: alpha_ob45_seg11[0] + alpha_ob45_seg11[1] + alpha_ob45_seg11[2]
   + alpha_ob45_seg11[3] >= 1
 R5467: - C22 + 1e+06 alpha_ob46_seg11[0] <= 1.0000764021508709e+06
 R5468: - C24 + 1e+06 alpha_ob46_seg11[0] <= 1.0000764021508709e+06
 R5469: - C23 + 1e+06 alpha_ob46_seg11[1] <= 999957.1027002756
 R5470: - C25 + 1e+06 alpha_ob46_seg11[1] <= 999957.1027002756
 R5471: C22 + 1e+06 alpha_ob46_seg11[2] <= 999921.0477508709
 R5472: C24 + 1e+06 alpha_ob46_seg11[2] <= 999921.0477508709
 R5473: C23 + 1e+06 alpha_ob46_seg11[3] <= 1.0000137473002756e+06
 R5474: C25 + 1e+06 alpha_ob46_seg11[3] <= 1.0000137473002756e+06
 R5475: alpha_ob46_seg11[0] + alpha_ob46_seg11[1] + alpha_ob46_seg11[2]
   + alpha_ob46_seg11[3] >= 1
 R5476: - C22 + 1e+06 alpha_ob47_seg11[0] <= 1.0000445522e+06
 R5477: - C24 + 1e+06 alpha_ob47_seg11[0] <= 1.0000445522e+06
 R5478: - C23 + 1e+06 alpha_ob47_seg11[1] <= 999983.7027
 R5479: - C25 + 1e+06 alpha_ob47_seg11[1] <= 999983.7027
 R5480: C22 + 1e+06 alpha_ob47_seg11[2] <= 999921.0477999999
 R5481: C24 + 1e+06 alpha_ob47_seg11[2] <= 999921.0477999999
 R5482: C23 + 1e+06 alpha_ob47_seg11[3] <= 1.0000137473e+06
 R5483: C25 + 1e+06 alpha_ob47_seg11[3] <= 1.0000137473e+06
 R5484: alpha_ob47_seg11[0] + alpha_ob47_seg11[1] + alpha_ob47_seg11[2]
   + alpha_ob47_seg11[3] >= 1
 R5485: - C22 + 1e+06 alpha_ob48_seg11[0] <= 1.0000445522183661e+06
 R5486: - C24 + 1e+06 alpha_ob48_seg11[0] <= 1.0000445522183661e+06
 R5487: - C23 + 1e+06 alpha_ob48_seg11[1] <= 999973.8526997245
 R5488: - C25 + 1e+06 alpha_ob48_seg11[1] <= 999973.8526997245
 R5489: C22 + 1e+06 alpha_ob48_seg11[2] <= 999952.8978183661
 R5490: C24 + 1e+06 alpha_ob48_seg11[2] <= 999952.8978183661
 R5491: C23 + 1e+06 alpha_ob48_seg11[3] <= 1.0000137472997245e+06
 R5492: C25 + 1e+06 alpha_ob48_seg11[3] <= 1.0000137472997245e+06
 R5493: alpha_ob48_seg11[0] + alpha_ob48_seg11[1] + alpha_ob48_seg11[2]
   + alpha_ob48_seg11[3] >= 1
 R5494: - C22 + 1e+06 alpha_ob49_seg11[0] <= 1.000064332081634e+06
 R5495: - C24 + 1e+06 alpha_ob49_seg11[0] <= 1.000064332081634e+06
 R5496: - C23 + 1e+06 alpha_ob49_seg11[1] <= 999973.9401002756
 R5497: - C25 + 1e+06 alpha_ob49_seg11[1] <= 999973.9401002756
 R5498: C22 + 1e+06 alpha_ob49_seg11[2] <= 999933.1178816339
 R5499: C24 + 1e+06 alpha_ob49_seg11[2] <= 999933.1178816339
 R5500: C23 + 1e+06 alpha_ob49_seg11[3] <= 1.0000136599002755e+06
 R5501: C25 + 1e+06 alpha_ob49_seg11[3] <= 1.0000136599002755e+06
 R5502: alpha_ob49_seg11[0] + alpha_ob49_seg11[1] + alpha_ob49_seg11[2]
   + alpha_ob49_seg11[3] >= 1
 R5503: - C22 + 1e+06 alpha_ob50_seg11[0] <= 1.0000643321013774e+06
 R5504: - C24 + 1e+06 alpha_ob50_seg11[0] <= 1.0000643321013774e+06
 R5505: - C23 + 1e+06 alpha_ob50_seg11[1] <= 999983.1900997246
 R5506: - C25 + 1e+06 alpha_ob50_seg11[1] <= 999983.1900997246
 R5507: C22 + 1e+06 alpha_ob50_seg11[2] <= 999933.1179013775
 R5508: C24 + 1e+06 alpha_ob50_seg11[2] <= 999933.1179013775
 R5509: C23 + 1e+06 alpha_ob50_seg11[3] <= 1.0000136598997245e+06
 R5510: C25 + 1e+06 alpha_ob50_seg11[3] <= 1.0000136598997245e+06
 R5511: alpha_ob50_seg11[0] + alpha_ob50_seg11[1] + alpha_ob50_seg11[2]
   + alpha_ob50_seg11[3] >= 1
 R5512: - C24 + 1e+06 alpha_ob0_seg12[0] <= 1.0000284170499525e+06
 R5513: - C26 + 1e+06 alpha_ob0_seg12[0] <= 1.0000284170499525e+06
 R5514: - C25 + 1e+06 alpha_ob0_seg12[1] <= 999957.0843002755
 R5515: - C27 + 1e+06 alpha_ob0_seg12[1] <= 999957.0843002755
 R5516: C24 + 1e+06 alpha_ob0_seg12[2] <= 999969.0328499526
 R5517: C26 + 1e+06 alpha_ob0_seg12[2] <= 999969.0328499526
 R5518: C25 + 1e+06 alpha_ob0_seg12[3] <= 1.0000132657002755e+06
 R5519: C27 + 1e+06 alpha_ob0_seg12[3] <= 1.0000132657002755e+06
 R5520: alpha_ob0_seg12[0] + alpha_ob0_seg12[1] + alpha_ob0_seg12[2]
   + alpha_ob0_seg12[3] >= 1
 R5521: - C24 + 1e+06 alpha_ob1_seg12[0] <= 1.000028417099801e+06
 R5522: - C26 + 1e+06 alpha_ob1_seg12[0] <= 1.000028417099801e+06
 R5523: - C25 + 1e+06 alpha_ob1_seg12[1] <= 999984.1843155898
 R5524: - C27 + 1e+06 alpha_ob1_seg12[1] <= 999984.1843155898
 R5525: C24 + 1e+06 alpha_ob1_seg12[2] <= 999957.4328998009
 R5526: C26 + 1e+06 alpha_ob1_seg12[2] <= 999957.4328998009
 R5527: C25 + 1e+06 alpha_ob1_seg12[3] <= 1.0000132657155899e+06
 R5528: C27 + 1e+06 alpha_ob1_seg12[3] <= 1.0000132657155899e+06
 R5529: alpha_ob1_seg12[0] + alpha_ob1_seg12[1] + alpha_ob1_seg12[2]
   + alpha_ob1_seg12[3] >= 1
 R5530: - C24 + 1e+06 alpha_ob2_seg12[0] <= 1.0000400171500475e+06
 R5531: - C26 + 1e+06 alpha_ob2_seg12[0] <= 1.0000400171500475e+06
 R5532: - C25 + 1e+06 alpha_ob2_seg12[1] <= 999957.0842997245
 R5533: - C27 + 1e+06 alpha_ob2_seg12[1] <= 999957.0842997245
 R5534: C24 + 1e+06 alpha_ob2_seg12[2] <= 999957.4329500474
 R5535: C26 + 1e+06 alpha_ob2_seg12[2] <= 999957.4329500474
 R5536: C25 + 1e+06 alpha_ob2_seg12[3] <= 1.0000132656997246e+06
 R5537: C27 + 1e+06 alpha_ob2_seg12[3] <= 1.0000132656997246e+06
 R5538: alpha_ob2_seg12[0] + alpha_ob2_seg12[1] + alpha_ob2_seg12[2]
   + alpha_ob2_seg12[3] >= 1
 R5539: - C24 + 1e+06 alpha_ob3_seg12[0] <= 1.0000109161752058e+06
 R5540: - C26 + 1e+06 alpha_ob3_seg12[0] <= 1.0000109161752058e+06
 R5541: - C25 + 1e+06 alpha_ob3_seg12[1] <= 999970.2681002755
 R5542: - C27 + 1e+06 alpha_ob3_seg12[1] <= 999970.2681002755
 R5543: C24 + 1e+06 alpha_ob3_seg12[2] <= 999986.5337752058
 R5544: C26 + 1e+06 alpha_ob3_seg12[2] <= 999986.5337752058
 R5545: C25 + 1e+06 alpha_ob3_seg12[3] <= 1.0000138319002755e+06
 R5546: C27 + 1e+06 alpha_ob3_seg12[3] <= 1.0000138319002755e+06
 R5547: alpha_ob3_seg12[0] + alpha_ob3_seg12[1] + alpha_ob3_seg12[2]
   + alpha_ob3_seg12[3] >= 1
 R5548: - C24 + 1e+06 alpha_ob4_seg12[0] <= 1.000010916199801e+06
 R5549: - C26 + 1e+06 alpha_ob4_seg12[0] <= 1.000010916199801e+06
 R5550: - C25 + 1e+06 alpha_ob4_seg12[1] <= 999983.6181179117
 R5551: - C27 + 1e+06 alpha_ob4_seg12[1] <= 999983.6181179117
 R5552: C24 + 1e+06 alpha_ob4_seg12[2] <= 999973.183799801
 R5553: C26 + 1e+06 alpha_ob4_seg12[2] <= 999973.183799801
 R5554: C25 + 1e+06 alpha_ob4_seg12[3] <= 1.0000138319179118e+06
 R5555: C27 + 1e+06 alpha_ob4_seg12[3] <= 1.0000138319179118e+06
 R5556: alpha_ob4_seg12[0] + alpha_ob4_seg12[1] + alpha_ob4_seg12[2]
   + alpha_ob4_seg12[3] >= 1
 R5557: - C24 + 1e+06 alpha_ob5_seg12[0] <= 999992.49102
 R5558: - C26 + 1e+06 alpha_ob5_seg12[0] <= 999992.49102
 R5559: - C25 + 1e+06 alpha_ob5_seg12[1] <= 999957.1016000001
 R5560: - C27 + 1e+06 alpha_ob5_seg12[1] <= 999957.1016000001
 R5561: C24 + 1e+06 alpha_ob5_seg12[2] <= 999986.35898
 R5562: C26 + 1e+06 alpha_ob5_seg12[2] <= 999986.35898
 R5563: C25 + 1e+06 alpha_ob5_seg12[3] <= 1.0000403484e+06
 R5564: C27 + 1e+06 alpha_ob5_seg12[3] <= 1.0000403484e+06
 R5565: alpha_ob5_seg12[0] + alpha_ob5_seg12[1] + alpha_ob5_seg12[2]
   + alpha_ob5_seg12[3] >= 1
 R5566: - C24 + 1e+06 alpha_ob6_seg12[0] <= 999981.3909999999
 R5567: - C26 + 1e+06 alpha_ob6_seg12[0] <= 999981.3909999999
 R5568: - C25 + 1e+06 alpha_ob6_seg12[1] <= 999957.1016000001
 R5569: - C27 + 1e+06 alpha_ob6_seg12[1] <= 999957.1016000001
 R5570: C24 + 1e+06 alpha_ob6_seg12[2] <= 1.000004959e+06
 R5571: C26 + 1e+06 alpha_ob6_seg12[2] <= 1.000004959e+06
 R5572: C25 + 1e+06 alpha_ob6_seg12[3] <= 1.0000403484e+06
 R5573: C27 + 1e+06 alpha_ob6_seg12[3] <= 1.0000403484e+06
 R5574: alpha_ob6_seg12[0] + alpha_ob6_seg12[1] + alpha_ob6_seg12[2]
   + alpha_ob6_seg12[3] >= 1
 R5575: - C24 + 1e+06 alpha_ob7_seg12[0] <= 999981.3909825523
 R5576: - C26 + 1e+06 alpha_ob7_seg12[0] <= 999981.3909825523
 R5577: - C25 + 1e+06 alpha_ob7_seg12[1] <= 999957.1016002755
 R5578: - C27 + 1e+06 alpha_ob7_seg12[1] <= 999957.1016002755
 R5579: C24 + 1e+06 alpha_ob7_seg12[2] <= 1.0000160589825523e+06
 R5580: C26 + 1e+06 alpha_ob7_seg12[2] <= 1.0000160589825523e+06
 R5581: C25 + 1e+06 alpha_ob7_seg12[3] <= 1.0000309984002755e+06
 R5582: C27 + 1e+06 alpha_ob7_seg12[3] <= 1.0000309984002755e+06
 R5583: alpha_ob7_seg12[0] + alpha_ob7_seg12[1] + alpha_ob7_seg12[2]
   + alpha_ob7_seg12[3] >= 1
 R5584: - C24 + 1e+06 alpha_ob8_seg12[0] <= 999981.3867954084
 R5585: - C26 + 1e+06 alpha_ob8_seg12[0] <= 999981.3867954084
 R5586: - C25 + 1e+06 alpha_ob8_seg12[1] <= 999968.8595002755
 R5587: - C27 + 1e+06 alpha_ob8_seg12[1] <= 999968.8595002755
 R5588: C24 + 1e+06 alpha_ob8_seg12[2] <= 1.0000160631954084e+06
 R5589: C26 + 1e+06 alpha_ob8_seg12[2] <= 1.0000160631954084e+06
 R5590: C25 + 1e+06 alpha_ob8_seg12[3] <= 1.0000262405002755e+06
 R5591: C27 + 1e+06 alpha_ob8_seg12[3] <= 1.0000262405002755e+06
 R5592: alpha_ob8_seg12[0] + alpha_ob8_seg12[1] + alpha_ob8_seg12[2]
   + alpha_ob8_seg12[3] >= 1
 R5593: - C24 + 1e+06 alpha_ob9_seg12[0] <= 999981.386799801
 R5594: - C26 + 1e+06 alpha_ob9_seg12[0] <= 999981.386799801
 R5595: - C25 + 1e+06 alpha_ob9_seg12[1] <= 999971.2095013268
 R5596: - C27 + 1e+06 alpha_ob9_seg12[1] <= 999971.2095013268
 R5597: C24 + 1e+06 alpha_ob9_seg12[2] <= 1.0000152131998009e+06
 R5598: C26 + 1e+06 alpha_ob9_seg12[2] <= 1.0000152131998009e+06
 R5599: C25 + 1e+06 alpha_ob9_seg12[3] <= 1.0000262405013267e+06
 R5600: C27 + 1e+06 alpha_ob9_seg12[3] <= 1.0000262405013267e+06
 R5601: alpha_ob9_seg12[0] + alpha_ob9_seg12[1] + alpha_ob9_seg12[2]
   + alpha_ob9_seg12[3] >= 1
 R5602: - C24 + 1e+06 alpha_ob10_seg12[0] <= 999982.2368022958
 R5603: - C26 + 1e+06 alpha_ob10_seg12[0] <= 999982.2368022958
 R5604: - C25 + 1e+06 alpha_ob10_seg12[1] <= 999970.1094997245
 R5605: - C27 + 1e+06 alpha_ob10_seg12[1] <= 999970.1094997245
 R5606: C24 + 1e+06 alpha_ob10_seg12[2] <= 1.0000152132022957e+06
 R5607: C26 + 1e+06 alpha_ob10_seg12[2] <= 1.0000152132022957e+06
 R5608: C25 + 1e+06 alpha_ob10_seg12[3] <= 1.0000262404997245e+06
 R5609: C27 + 1e+06 alpha_ob10_seg12[3] <= 1.0000262404997245e+06
 R5610: alpha_ob10_seg12[0] + alpha_ob10_seg12[1] + alpha_ob10_seg12[2]
   + alpha_ob10_seg12[3] >= 1
 R5611: - C24 + 1e+06 alpha_ob11_seg12[0] <= 999982.236799801
 R5612: - C26 + 1e+06 alpha_ob11_seg12[0] <= 999982.236799801
 R5613: - C25 + 1e+06 alpha_ob11_seg12[1] <= 999970.1095291895
 R5614: - C27 + 1e+06 alpha_ob11_seg12[1] <= 999970.1095291895
 R5615: C24 + 1e+06 alpha_ob11_seg12[2] <= 999993.3631998011
 R5616: C26 + 1e+06 alpha_ob11_seg12[2] <= 999993.3631998011
 R5617: C25 + 1e+06 alpha_ob11_seg12[3] <= 1.0000273405291894e+06
 R5618: C27 + 1e+06 alpha_ob11_seg12[3] <= 1.0000273405291894e+06
 R5619: alpha_ob11_seg12[0] + alpha_ob11_seg12[1] + alpha_ob11_seg12[2]
   + alpha_ob11_seg12[3] >= 1
 R5620: - C24 + 1e+06 alpha_ob12_seg12[0] <= 999981.6050802565
 R5621: - C26 + 1e+06 alpha_ob12_seg12[0] <= 999981.6050802565
 R5622: - C25 + 1e+06 alpha_ob12_seg12[1] <= 999972.8942002755
 R5623: - C27 + 1e+06 alpha_ob12_seg12[1] <= 999972.8942002755
 R5624: C24 + 1e+06 alpha_ob12_seg12[2] <= 1.0000158448802566e+06
 R5625: C26 + 1e+06 alpha_ob12_seg12[2] <= 1.0000158448802566e+06
 R5626: C25 + 1e+06 alpha_ob12_seg12[3] <= 1.0000139558002756e+06
 R5627: C27 + 1e+06 alpha_ob12_seg12[3] <= 1.0000139558002756e+06
 R5628: alpha_ob12_seg12[0] + alpha_ob12_seg12[1] + alpha_ob12_seg12[2]
   + alpha_ob12_seg12[3] >= 1
 R5629: - C24 + 1e+06 alpha_ob13_seg12[0] <= 999981.605149801
 R5630: - C26 + 1e+06 alpha_ob13_seg12[0] <= 999981.605149801
 R5631: - C25 + 1e+06 alpha_ob13_seg12[1] <= 999983.4942391404
 R5632: - C27 + 1e+06 alpha_ob13_seg12[1] <= 999983.4942391404
 R5633: C24 + 1e+06 alpha_ob13_seg12[2] <= 999986.4948498011
 R5634: C26 + 1e+06 alpha_ob13_seg12[2] <= 999986.4948498011
 R5635: C25 + 1e+06 alpha_ob13_seg12[3] <= 1.0000139558391405e+06
 R5636: C27 + 1e+06 alpha_ob13_seg12[3] <= 1.0000139558391405e+06
 R5637: alpha_ob13_seg12[0] + alpha_ob13_seg12[1] + alpha_ob13_seg12[2]
   + alpha_ob13_seg12[3] >= 1
 R5638: - C24 + 1e+06 alpha_ob14_seg12[0] <= 999997.6803502565
 R5639: - C26 + 1e+06 alpha_ob14_seg12[0] <= 999997.6803502565
 R5640: - C25 + 1e+06 alpha_ob14_seg12[1] <= 999972.7268002755
 R5641: - C27 + 1e+06 alpha_ob14_seg12[1] <= 999972.7268002755
 R5642: C24 + 1e+06 alpha_ob14_seg12[2] <= 999999.7696102565
 R5643: C26 + 1e+06 alpha_ob14_seg12[2] <= 999999.7696102565
 R5644: C25 + 1e+06 alpha_ob14_seg12[3] <= 1.0000141232002755e+06
 R5645: C27 + 1e+06 alpha_ob14_seg12[3] <= 1.0000141232002755e+06
 R5646: alpha_ob14_seg12[0] + alpha_ob14_seg12[1] + alpha_ob14_seg12[2]
   + alpha_ob14_seg12[3] >= 1
 R5647: - C24 + 1e+06 alpha_ob15_seg12[0] <= 1.0000038341502565e+06
 R5648: - C26 + 1e+06 alpha_ob15_seg12[0] <= 1.0000038341502565e+06
 R5649: - C25 + 1e+06 alpha_ob15_seg12[1] <= 999972.6012002755
 R5650: - C27 + 1e+06 alpha_ob15_seg12[1] <= 999972.6012002755
 R5651: C24 + 1e+06 alpha_ob15_seg12[2] <= 999993.6158102566
 R5652: C26 + 1e+06 alpha_ob15_seg12[2] <= 999993.6158102566
 R5653: C25 + 1e+06 alpha_ob15_seg12[3] <= 1.0000142488002755e+06
 R5654: C27 + 1e+06 alpha_ob15_seg12[3] <= 1.0000142488002755e+06
 R5655: alpha_ob15_seg12[0] + alpha_ob15_seg12[1] + alpha_ob15_seg12[2]
   + alpha_ob15_seg12[3] >= 1
 R5656: - C24 + 1e+06 alpha_ob16_seg12[0] <= 1.0000162874063333e+06
 R5657: - C26 + 1e+06 alpha_ob16_seg12[0] <= 1.0000162874063333e+06
 R5658: - C25 + 1e+06 alpha_ob16_seg12[1] <= 999982.9439602756
 R5659: - C27 + 1e+06 alpha_ob16_seg12[1] <= 999982.9439602756
 R5660: C24 + 1e+06 alpha_ob16_seg12[2] <= 999981.1624063333
 R5661: C26 + 1e+06 alpha_ob16_seg12[2] <= 999981.1624063333
 R5662: C25 + 1e+06 alpha_ob16_seg12[3] <= 999963.6560402757
 R5663: C27 + 1e+06 alpha_ob16_seg12[3] <= 999963.6560402757
 R5664: alpha_ob16_seg12[0] + alpha_ob16_seg12[1] + alpha_ob16_seg12[2]
   + alpha_ob16_seg12[3] >= 1
 R5665: - C24 + 1e+06 alpha_ob17_seg12[0] <= 999980.937509
 R5666: - C26 + 1e+06 alpha_ob17_seg12[0] <= 999980.937509
 R5667: - C25 + 1e+06 alpha_ob17_seg12[1] <= 1.000034894e+06
 R5668: - C27 + 1e+06 alpha_ob17_seg12[1] <= 1.000034894e+06
 R5669: C24 + 1e+06 alpha_ob17_seg12[2] <= 999981.162491
 R5670: C26 + 1e+06 alpha_ob17_seg12[2] <= 999981.162491
 R5671: C25 + 1e+06 alpha_ob17_seg12[3] <= 999962.556
 R5672: C27 + 1e+06 alpha_ob17_seg12[3] <= 999962.556
 R5673: alpha_ob17_seg12[0] + alpha_ob17_seg12[1] + alpha_ob17_seg12[2]
   + alpha_ob17_seg12[3] >= 1
 R5674: - C24 + 1e+06 alpha_ob18_seg12[0] <= 999990.2670277582
 R5675: - C26 + 1e+06 alpha_ob18_seg12[0] <= 999990.2670277582
 R5676: - C25 + 1e+06 alpha_ob18_seg12[1] <= 1.0000347702002756e+06
 R5677: - C27 + 1e+06 alpha_ob18_seg12[1] <= 1.0000347702002756e+06
 R5678: C24 + 1e+06 alpha_ob18_seg12[2] <= 1.0000071828877581e+06
 R5679: C26 + 1e+06 alpha_ob18_seg12[2] <= 1.0000071828877581e+06
 R5680: C25 + 1e+06 alpha_ob18_seg12[3] <= 999939.8298002756
 R5681: C27 + 1e+06 alpha_ob18_seg12[3] <= 999939.8298002756
 R5682: alpha_ob18_seg12[0] + alpha_ob18_seg12[1] + alpha_ob18_seg12[2]
   + alpha_ob18_seg12[3] >= 1
 R5683: - C24 + 1e+06 alpha_ob19_seg12[0] <= 999990.267069801
 R5684: - C26 + 1e+06 alpha_ob19_seg12[0] <= 999990.267069801
 R5685: - C25 + 1e+06 alpha_ob19_seg12[1] <= 1.0000576202069657e+06
 R5686: - C27 + 1e+06 alpha_ob19_seg12[1] <= 1.0000576202069657e+06
 R5687: C24 + 1e+06 alpha_ob19_seg12[2] <= 1.000002082929801e+06
 R5688: C26 + 1e+06 alpha_ob19_seg12[2] <= 1.000002082929801e+06
 R5689: C25 + 1e+06 alpha_ob19_seg12[3] <= 999939.8298069657
 R5690: C27 + 1e+06 alpha_ob19_seg12[3] <= 999939.8298069657
 R5691: alpha_ob19_seg12[0] + alpha_ob19_seg12[1] + alpha_ob19_seg12[2]
   + alpha_ob19_seg12[3] >= 1
 R5692: - C24 + 1e+06 alpha_ob20_seg12[0] <= 999991.073732
 R5693: - C26 + 1e+06 alpha_ob20_seg12[0] <= 999991.073732
 R5694: - C25 + 1e+06 alpha_ob20_seg12[1] <= 999952.2125
 R5695: - C27 + 1e+06 alpha_ob20_seg12[1] <= 999952.2125
 R5696: C24 + 1e+06 alpha_ob20_seg12[2] <= 999990.776268
 R5697: C26 + 1e+06 alpha_ob20_seg12[2] <= 999990.776268
 R5698: C25 + 1e+06 alpha_ob20_seg12[3] <= 1.0000452375e+06
 R5699: C27 + 1e+06 alpha_ob20_seg12[3] <= 1.0000452375e+06
 R5700: alpha_ob20_seg12[0] + alpha_ob20_seg12[1] + alpha_ob20_seg12[2]
   + alpha_ob20_seg12[3] >= 1
 R5701: - C24 + 1e+06 alpha_ob21_seg12[0] <= 999985.657099801
 R5702: - C26 + 1e+06 alpha_ob21_seg12[0] <= 999985.657099801
 R5703: - C25 + 1e+06 alpha_ob21_seg12[1] <= 1.0000576202063023e+06
 R5704: - C27 + 1e+06 alpha_ob21_seg12[1] <= 1.0000576202063023e+06
 R5705: C24 + 1e+06 alpha_ob21_seg12[2] <= 1.000007192899801e+06
 R5706: C26 + 1e+06 alpha_ob21_seg12[2] <= 1.000007192899801e+06
 R5707: C25 + 1e+06 alpha_ob21_seg12[3] <= 999939.8298063023
 R5708: C27 + 1e+06 alpha_ob21_seg12[3] <= 999939.8298063023
 R5709: alpha_ob21_seg12[0] + alpha_ob21_seg12[1] + alpha_ob21_seg12[2]
   + alpha_ob21_seg12[3] >= 1
 R5710: - C24 + 1e+06 alpha_ob22_seg12[0] <= 999994.771829801
 R5711: - C26 + 1e+06 alpha_ob22_seg12[0] <= 999994.771829801
 R5712: - C25 + 1e+06 alpha_ob22_seg12[1] <= 1.0000615983063023e+06
 R5713: - C27 + 1e+06 alpha_ob22_seg12[1] <= 1.0000615983063023e+06
 R5714: C24 + 1e+06 alpha_ob22_seg12[2] <= 999998.078169801
 R5715: C26 + 1e+06 alpha_ob22_seg12[2] <= 999998.078169801
 R5716: C25 + 1e+06 alpha_ob22_seg12[3] <= 999935.8517063023
 R5717: C27 + 1e+06 alpha_ob22_seg12[3] <= 999935.8517063023
 R5718: alpha_ob22_seg12[0] + alpha_ob22_seg12[1] + alpha_ob22_seg12[2]
   + alpha_ob22_seg12[3] >= 1
 R5719: - C24 + 1e+06 alpha_ob23_seg12[0] <= 999980.9374508709
 R5720: - C26 + 1e+06 alpha_ob23_seg12[0] <= 999980.9374508709
 R5721: - C25 + 1e+06 alpha_ob23_seg12[1] <= 1.0000348940002755e+06
 R5722: - C27 + 1e+06 alpha_ob23_seg12[1] <= 1.0000348940002755e+06
 R5723: C24 + 1e+06 alpha_ob23_seg12[2] <= 1.0000165124508708e+06
 R5724: C26 + 1e+06 alpha_ob23_seg12[2] <= 1.0000165124508708e+06
 R5725: C25 + 1e+06 alpha_ob23_seg12[3] <= 999935.9560002756
 R5726: C27 + 1e+06 alpha_ob23_seg12[3] <= 999935.9560002756
 R5727: alpha_ob23_seg12[0] + alpha_ob23_seg12[1] + alpha_ob23_seg12[2]
   + alpha_ob23_seg12[3] >= 1
 R5728: - C24 + 1e+06 alpha_ob24_seg12[0] <= 999980.937499801
 R5729: - C26 + 1e+06 alpha_ob24_seg12[0] <= 999980.937499801
 R5730: - C25 + 1e+06 alpha_ob24_seg12[1] <= 1.0000614940072973e+06
 R5731: - C27 + 1e+06 alpha_ob24_seg12[1] <= 1.0000614940072973e+06
 R5732: C24 + 1e+06 alpha_ob24_seg12[2] <= 1.000011162499801e+06
 R5733: C26 + 1e+06 alpha_ob24_seg12[2] <= 1.000011162499801e+06
 R5734: C25 + 1e+06 alpha_ob24_seg12[3] <= 999935.9560072974
 R5735: C27 + 1e+06 alpha_ob24_seg12[3] <= 999935.9560072974
 R5736: alpha_ob24_seg12[0] + alpha_ob24_seg12[1] + alpha_ob24_seg12[2]
   + alpha_ob24_seg12[3] >= 1
 R5737: - C24 + 1e+06 alpha_ob25_seg12[0] <= 999956.8394000001
 R5738: - C26 + 1e+06 alpha_ob25_seg12[0] <= 999956.8394000001
 R5739: - C25 + 1e+06 alpha_ob25_seg12[1] <= 999952.2463
 R5740: - C27 + 1e+06 alpha_ob25_seg12[1] <= 999952.2463
 R5741: C24 + 1e+06 alpha_ob25_seg12[2] <= 1.0000097606e+06
 R5742: C26 + 1e+06 alpha_ob25_seg12[2] <= 1.0000097606e+06
 R5743: C25 + 1e+06 alpha_ob25_seg12[3] <= 1.0000452037e+06
 R5744: C27 + 1e+06 alpha_ob25_seg12[3] <= 1.0000452037e+06
 R5745: alpha_ob25_seg12[0] + alpha_ob25_seg12[1] + alpha_ob25_seg12[2]
   + alpha_ob25_seg12[3] >= 1
 R5746: - C24 + 1e+06 alpha_ob26_seg12[0] <= 999980.9375348955
 R5747: - C26 + 1e+06 alpha_ob26_seg12[0] <= 999980.9375348955
 R5748: - C25 + 1e+06 alpha_ob26_seg12[1] <= 1.0000160439997246e+06
 R5749: - C27 + 1e+06 alpha_ob26_seg12[1] <= 1.0000160439997246e+06
 R5750: C24 + 1e+06 alpha_ob26_seg12[2] <= 1.0000165125348954e+06
 R5751: C26 + 1e+06 alpha_ob26_seg12[2] <= 1.0000165125348954e+06
 R5752: C25 + 1e+06 alpha_ob26_seg12[3] <= 999962.5559997245
 R5753: C27 + 1e+06 alpha_ob26_seg12[3] <= 999962.5559997245
 R5754: alpha_ob26_seg12[0] + alpha_ob26_seg12[1] + alpha_ob26_seg12[2]
   + alpha_ob26_seg12[3] >= 1
 R5755: - C24 + 1e+06 alpha_ob27_seg12[0] <= 999956.839399801
 R5756: - C26 + 1e+06 alpha_ob27_seg12[0] <= 999956.839399801
 R5757: - C25 + 1e+06 alpha_ob27_seg12[1] <= 999925.3963434525
 R5758: - C27 + 1e+06 alpha_ob27_seg12[1] <= 999925.3963434525
 R5759: C24 + 1e+06 alpha_ob27_seg12[2] <= 1.0000080105998011e+06
 R5760: C26 + 1e+06 alpha_ob27_seg12[2] <= 1.0000080105998011e+06
 R5761: C25 + 1e+06 alpha_ob27_seg12[3] <= 1.0000720537434525e+06
 R5762: C27 + 1e+06 alpha_ob27_seg12[3] <= 1.0000720537434525e+06
 R5763: alpha_ob27_seg12[0] + alpha_ob27_seg12[1] + alpha_ob27_seg12[2]
   + alpha_ob27_seg12[3] >= 1
 R5764: - C24 + 1e+06 alpha_ob28_seg12[0] <= 999980.9374998009
 R5765: - C26 + 1e+06 alpha_ob28_seg12[0] <= 999980.9374998009
 R5766: - C25 + 1e+06 alpha_ob28_seg12[1] <= 1.0000160440053071e+06
 R5767: - C27 + 1e+06 alpha_ob28_seg12[1] <= 1.0000160440053071e+06
 R5768: C24 + 1e+06 alpha_ob28_seg12[2] <= 1.000012662499801e+06
 R5769: C26 + 1e+06 alpha_ob28_seg12[2] <= 1.000012662499801e+06
 R5770: C25 + 1e+06 alpha_ob28_seg12[3] <= 999981.4060053072
 R5771: C27 + 1e+06 alpha_ob28_seg12[3] <= 999981.4060053072
 R5772: alpha_ob28_seg12[0] + alpha_ob28_seg12[1] + alpha_ob28_seg12[2]
   + alpha_ob28_seg12[3] >= 1
 R5773: - C24 + 1e+06 alpha_ob29_seg12[0] <= 999983.6053459151
 R5774: - C26 + 1e+06 alpha_ob29_seg12[0] <= 999983.6053459151
 R5775: - C25 + 1e+06 alpha_ob29_seg12[1] <= 999987.7722497246
 R5776: - C27 + 1e+06 alpha_ob29_seg12[1] <= 999987.7722497246
 R5777: C24 + 1e+06 alpha_ob29_seg12[2] <= 1.000013844745915e+06
 R5778: C26 + 1e+06 alpha_ob29_seg12[2] <= 1.000013844745915e+06
 R5779: C25 + 1e+06 alpha_ob29_seg12[3] <= 999984.8277497246
 R5780: C27 + 1e+06 alpha_ob29_seg12[3] <= 999984.8277497246
 R5781: alpha_ob29_seg12[0] + alpha_ob29_seg12[1] + alpha_ob29_seg12[2]
   + alpha_ob29_seg12[3] >= 1
 R5782: - C24 + 1e+06 alpha_ob30_seg12[0] <= 999958.8717898987
 R5783: - C26 + 1e+06 alpha_ob30_seg12[0] <= 999958.8717898987
 R5784: - C25 + 1e+06 alpha_ob30_seg12[1] <= 999987.7837602755
 R5785: - C27 + 1e+06 alpha_ob30_seg12[1] <= 999987.7837602755
 R5786: C24 + 1e+06 alpha_ob30_seg12[2] <= 1.0000385781898986e+06
 R5787: C26 + 1e+06 alpha_ob30_seg12[2] <= 1.0000385781898986e+06
 R5788: C25 + 1e+06 alpha_ob30_seg12[3] <= 1.0000043162402755e+06
 R5789: C27 + 1e+06 alpha_ob30_seg12[3] <= 1.0000043162402755e+06
 R5790: alpha_ob30_seg12[0] + alpha_ob30_seg12[1] + alpha_ob30_seg12[2]
   + alpha_ob30_seg12[3] >= 1
 R5791: - C24 + 1e+06 alpha_ob31_seg12[0] <= 999939.2718
 R5792: - C26 + 1e+06 alpha_ob31_seg12[0] <= 999939.2718
 R5793: - C25 + 1e+06 alpha_ob31_seg12[1] <= 999993.13376
 R5794: - C27 + 1e+06 alpha_ob31_seg12[1] <= 999993.13376
 R5795: C24 + 1e+06 alpha_ob31_seg12[2] <= 1.0000385782e+06
 R5796: C26 + 1e+06 alpha_ob31_seg12[2] <= 1.0000385782e+06
 R5797: C25 + 1e+06 alpha_ob31_seg12[3] <= 1.00000431624e+06
 R5798: C27 + 1e+06 alpha_ob31_seg12[3] <= 1.00000431624e+06
 R5799: alpha_ob31_seg12[0] + alpha_ob31_seg12[1] + alpha_ob31_seg12[2]
   + alpha_ob31_seg12[3] >= 1
 R5800: - C24 + 1e+06 alpha_ob32_seg12[0] <= 999939.2718101013
 R5801: - C26 + 1e+06 alpha_ob32_seg12[0] <= 999939.2718101013
 R5802: - C25 + 1e+06 alpha_ob32_seg12[1] <= 999987.7837597246
 R5803: - C27 + 1e+06 alpha_ob32_seg12[1] <= 999987.7837597246
 R5804: C24 + 1e+06 alpha_ob32_seg12[2] <= 1.0000581782101013e+06
 R5805: C26 + 1e+06 alpha_ob32_seg12[2] <= 1.0000581782101013e+06
 R5806: C25 + 1e+06 alpha_ob32_seg12[3] <= 1.0000043162397245e+06
 R5807: C27 + 1e+06 alpha_ob32_seg12[3] <= 1.0000043162397245e+06
 R5808: alpha_ob32_seg12[0] + alpha_ob32_seg12[1] + alpha_ob32_seg12[2]
   + alpha_ob32_seg12[3] >= 1
 R5809: - C24 + 1e+06 alpha_ob33_seg12[0] <= 999939.271799801
 R5810: - C26 + 1e+06 alpha_ob33_seg12[0] <= 999939.271799801
 R5811: - C25 + 1e+06 alpha_ob33_seg12[1] <= 999987.7838262042
 R5812: - C27 + 1e+06 alpha_ob33_seg12[1] <= 999987.7838262042
 R5813: C24 + 1e+06 alpha_ob33_seg12[2] <= 1.000038578199801e+06
 R5814: C26 + 1e+06 alpha_ob33_seg12[2] <= 1.000038578199801e+06
 R5815: C25 + 1e+06 alpha_ob33_seg12[3] <= 1.0000096662262043e+06
 R5816: C27 + 1e+06 alpha_ob33_seg12[3] <= 1.0000096662262043e+06
 R5817: alpha_ob33_seg12[0] + alpha_ob33_seg12[1] + alpha_ob33_seg12[2]
   + alpha_ob33_seg12[3] >= 1
 R5818: - C24 + 1e+06 alpha_ob34_seg12[0] <= 999926.0708
 R5819: - C26 + 1e+06 alpha_ob34_seg12[0] <= 999926.0708
 R5820: - C25 + 1e+06 alpha_ob34_seg12[1] <= 999983.7196
 R5821: - C27 + 1e+06 alpha_ob34_seg12[1] <= 999983.7196
 R5822: C24 + 1e+06 alpha_ob34_seg12[2] <= 1.0000250292e+06
 R5823: C26 + 1e+06 alpha_ob34_seg12[2] <= 1.0000250292e+06
 R5824: C25 + 1e+06 alpha_ob34_seg12[3] <= 1.0000137304e+06
 R5825: C27 + 1e+06 alpha_ob34_seg12[3] <= 1.0000137304e+06
 R5826: alpha_ob34_seg12[0] + alpha_ob34_seg12[1] + alpha_ob34_seg12[2]
   + alpha_ob34_seg12[3] >= 1
 R5827: - C24 + 1e+06 alpha_ob35_seg12[0] <= 999926.0708293857
 R5828: - C26 + 1e+06 alpha_ob35_seg12[0] <= 999926.0708293857
 R5829: - C25 + 1e+06 alpha_ob35_seg12[1] <= 999967.8695997245
 R5830: - C27 + 1e+06 alpha_ob35_seg12[1] <= 999967.8695997245
 R5831: C24 + 1e+06 alpha_ob35_seg12[2] <= 1.0000713792293856e+06
 R5832: C26 + 1e+06 alpha_ob35_seg12[2] <= 1.0000713792293856e+06
 R5833: C25 + 1e+06 alpha_ob35_seg12[3] <= 1.0000137303997246e+06
 R5834: C27 + 1e+06 alpha_ob35_seg12[3] <= 1.0000137303997246e+06
 R5835: alpha_ob35_seg12[0] + alpha_ob35_seg12[1] + alpha_ob35_seg12[2]
   + alpha_ob35_seg12[3] >= 1
 R5836: - C24 + 1e+06 alpha_ob36_seg12[0] <= 999972.6917508709
 R5837: - C26 + 1e+06 alpha_ob36_seg12[0] <= 999972.6917508709
 R5838: - C25 + 1e+06 alpha_ob36_seg12[1] <= 999957.1644002756
 R5839: - C27 + 1e+06 alpha_ob36_seg12[1] <= 999957.1644002756
 R5840: C24 + 1e+06 alpha_ob36_seg12[2] <= 1.0000247581508709e+06
 R5841: C26 + 1e+06 alpha_ob36_seg12[2] <= 1.0000247581508709e+06
 R5842: C25 + 1e+06 alpha_ob36_seg12[3] <= 1.0000136856002755e+06
 R5843: C27 + 1e+06 alpha_ob36_seg12[3] <= 1.0000136856002755e+06
 R5844: alpha_ob36_seg12[0] + alpha_ob36_seg12[1] + alpha_ob36_seg12[2]
   + alpha_ob36_seg12[3] >= 1
 R5845: - C24 + 1e+06 alpha_ob37_seg12[0] <= 999939.0009674003
 R5846: - C26 + 1e+06 alpha_ob37_seg12[0] <= 999939.0009674003
 R5847: - C25 + 1e+06 alpha_ob37_seg12[1] <= 999961.9516002756
 R5848: - C27 + 1e+06 alpha_ob37_seg12[1] <= 999961.9516002756
 R5849: C24 + 1e+06 alpha_ob37_seg12[2] <= 1.0000584489674004e+06
 R5850: C26 + 1e+06 alpha_ob37_seg12[2] <= 1.0000584489674004e+06
 R5851: C25 + 1e+06 alpha_ob37_seg12[3] <= 1.0000178984002755e+06
 R5852: C27 + 1e+06 alpha_ob37_seg12[3] <= 1.0000178984002755e+06
 R5853: alpha_ob37_seg12[0] + alpha_ob37_seg12[1] + alpha_ob37_seg12[2]
   + alpha_ob37_seg12[3] >= 1
 R5854: - C24 + 1e+06 alpha_ob38_seg12[0] <= 999994.9164998011
 R5855: - C26 + 1e+06 alpha_ob38_seg12[0] <= 999994.9164998011
 R5856: - C25 + 1e+06 alpha_ob38_seg12[1] <= 999925.4284782809
 R5857: - C27 + 1e+06 alpha_ob38_seg12[1] <= 999925.4284782809
 R5858: C24 + 1e+06 alpha_ob38_seg12[2] <= 999943.6834998011
 R5859: C26 + 1e+06 alpha_ob38_seg12[2] <= 999943.6834998011
 R5860: C25 + 1e+06 alpha_ob38_seg12[3] <= 1.0000720216782809e+06
 R5861: C27 + 1e+06 alpha_ob38_seg12[3] <= 1.0000720216782809e+06
 R5862: alpha_ob38_seg12[0] + alpha_ob38_seg12[1] + alpha_ob38_seg12[2]
   + alpha_ob38_seg12[3] >= 1
 R5863: - C24 + 1e+06 alpha_ob39_seg12[0] <= 999914.52327842
 R5864: - C26 + 1e+06 alpha_ob39_seg12[0] <= 999914.52327842
 R5865: - C25 + 1e+06 alpha_ob39_seg12[1] <= 999993.6552362755
 R5866: - C27 + 1e+06 alpha_ob39_seg12[1] <= 999993.6552362755
 R5867: C24 + 1e+06 alpha_ob39_seg12[2] <= 1.0000829266784199e+06
 R5868: C26 + 1e+06 alpha_ob39_seg12[2] <= 1.0000829266784199e+06
 R5869: C25 + 1e+06 alpha_ob39_seg12[3] <= 999992.1947642756
 R5870: C27 + 1e+06 alpha_ob39_seg12[3] <= 999992.1947642756
 R5871: alpha_ob39_seg12[0] + alpha_ob39_seg12[1] + alpha_ob39_seg12[2]
   + alpha_ob39_seg12[3] >= 1
 R5872: - C24 + 1e+06 alpha_ob40_seg12[0] <= 999912.265199801
 R5873: - C26 + 1e+06 alpha_ob40_seg12[0] <= 999912.265199801
 R5874: - C25 + 1e+06 alpha_ob40_seg12[1] <= 999987.7842112777
 R5875: - C27 + 1e+06 alpha_ob40_seg12[1] <= 999987.7842112777
 R5876: C24 + 1e+06 alpha_ob40_seg12[2] <= 1.000076834799801e+06
 R5877: C26 + 1e+06 alpha_ob40_seg12[2] <= 1.000076834799801e+06
 R5878: C25 + 1e+06 alpha_ob40_seg12[3] <= 1.0000096658112778e+06
 R5879: C27 + 1e+06 alpha_ob40_seg12[3] <= 1.0000096658112778e+06
 R5880: alpha_ob40_seg12[0] + alpha_ob40_seg12[1] + alpha_ob40_seg12[2]
   + alpha_ob40_seg12[3] >= 1
 R5881: - C24 + 1e+06 alpha_ob41_seg12[0] <= 999920.6152119379
 R5882: - C26 + 1e+06 alpha_ob41_seg12[0] <= 999920.6152119379
 R5883: - C25 + 1e+06 alpha_ob41_seg12[1] <= 999981.4341997246
 R5884: - C27 + 1e+06 alpha_ob41_seg12[1] <= 999981.4341997246
 R5885: C24 + 1e+06 alpha_ob41_seg12[2] <= 1.0000768348119379e+06
 R5886: C26 + 1e+06 alpha_ob41_seg12[2] <= 1.0000768348119379e+06
 R5887: C25 + 1e+06 alpha_ob41_seg12[3] <= 1.0000096657997245e+06
 R5888: C27 + 1e+06 alpha_ob41_seg12[3] <= 1.0000096657997245e+06
 R5889: alpha_ob41_seg12[0] + alpha_ob41_seg12[1] + alpha_ob41_seg12[2]
   + alpha_ob41_seg12[3] >= 1
 R5890: - C24 + 1e+06 alpha_ob42_seg12[0] <= 999919.2652
 R5891: - C26 + 1e+06 alpha_ob42_seg12[0] <= 999919.2652
 R5892: - C25 + 1e+06 alpha_ob42_seg12[1] <= 999981.4342
 R5893: - C27 + 1e+06 alpha_ob42_seg12[1] <= 999981.4342
 R5894: C24 + 1e+06 alpha_ob42_seg12[2] <= 1.0000768348e+06
 R5895: C26 + 1e+06 alpha_ob42_seg12[2] <= 1.0000768348e+06
 R5896: C25 + 1e+06 alpha_ob42_seg12[3] <= 1.0000160158000001e+06
 R5897: C27 + 1e+06 alpha_ob42_seg12[3] <= 1.0000160158000001e+06
 R5898: alpha_ob42_seg12[0] + alpha_ob42_seg12[1] + alpha_ob42_seg12[2]
   + alpha_ob42_seg12[3] >= 1
 R5899: - C24 + 1e+06 alpha_ob43_seg12[0] <= 999919.2652454559
 R5900: - C26 + 1e+06 alpha_ob43_seg12[0] <= 999919.2652454559
 R5901: - C25 + 1e+06 alpha_ob43_seg12[1] <= 999956.8341997246
 R5902: - C27 + 1e+06 alpha_ob43_seg12[1] <= 999956.8341997246
 R5903: C24 + 1e+06 alpha_ob43_seg12[2] <= 1.0000781848454559e+06
 R5904: C26 + 1e+06 alpha_ob43_seg12[2] <= 1.0000781848454559e+06
 R5905: C25 + 1e+06 alpha_ob43_seg12[3] <= 1.0000160157997246e+06
 R5906: C27 + 1e+06 alpha_ob43_seg12[3] <= 1.0000160157997246e+06
 R5907: alpha_ob43_seg12[0] + alpha_ob43_seg12[1] + alpha_ob43_seg12[2]
   + alpha_ob43_seg12[3] >= 1
 R5908: - C24 + 1e+06 alpha_ob44_seg12[0] <= 999991.83683
 R5909: - C26 + 1e+06 alpha_ob44_seg12[0] <= 999991.83683
 R5910: - C25 + 1e+06 alpha_ob44_seg12[1] <= 999940.589
 R5911: - C27 + 1e+06 alpha_ob44_seg12[1] <= 999940.589
 R5912: C24 + 1e+06 alpha_ob44_seg12[2] <= 1.00000401317e+06
 R5913: C26 + 1e+06 alpha_ob44_seg12[2] <= 1.00000401317e+06
 R5914: C25 + 1e+06 alpha_ob44_seg12[3] <= 1.000056861e+06
 R5915: C27 + 1e+06 alpha_ob44_seg12[3] <= 1.000056861e+06
 R5916: alpha_ob44_seg12[0] + alpha_ob44_seg12[1] + alpha_ob44_seg12[2]
   + alpha_ob44_seg12[3] >= 1
 R5917: - C24 + 1e+06 alpha_ob45_seg12[0] <= 999991.8368084199
 R5918: - C26 + 1e+06 alpha_ob45_seg12[0] <= 999991.8368084199
 R5919: - C25 + 1e+06 alpha_ob45_seg12[1] <= 999940.5890002755
 R5920: - C27 + 1e+06 alpha_ob45_seg12[1] <= 999940.5890002755
 R5921: C24 + 1e+06 alpha_ob45_seg12[2] <= 1.0000056131484199e+06
 R5922: C26 + 1e+06 alpha_ob45_seg12[2] <= 1.0000056131484199e+06
 R5923: C25 + 1e+06 alpha_ob45_seg12[3] <= 1.0000452610002755e+06
 R5924: C27 + 1e+06 alpha_ob45_seg12[3] <= 1.0000452610002755e+06
 R5925: alpha_ob45_seg12[0] + alpha_ob45_seg12[1] + alpha_ob45_seg12[2]
   + alpha_ob45_seg12[3] >= 1
 R5926: - C24 + 1e+06 alpha_ob46_seg12[0] <= 1.0000764021508709e+06
 R5927: - C26 + 1e+06 alpha_ob46_seg12[0] <= 1.0000764021508709e+06
 R5928: - C25 + 1e+06 alpha_ob46_seg12[1] <= 999957.1027002756
 R5929: - C27 + 1e+06 alpha_ob46_seg12[1] <= 999957.1027002756
 R5930: C24 + 1e+06 alpha_ob46_seg12[2] <= 999921.0477508709
 R5931: C26 + 1e+06 alpha_ob46_seg12[2] <= 999921.0477508709
 R5932: C25 + 1e+06 alpha_ob46_seg12[3] <= 1.0000137473002756e+06
 R5933: C27 + 1e+06 alpha_ob46_seg12[3] <= 1.0000137473002756e+06
 R5934: alpha_ob46_seg12[0] + alpha_ob46_seg12[1] + alpha_ob46_seg12[2]
   + alpha_ob46_seg12[3] >= 1
 R5935: - C24 + 1e+06 alpha_ob47_seg12[0] <= 1.0000445522e+06
 R5936: - C26 + 1e+06 alpha_ob47_seg12[0] <= 1.0000445522e+06
 R5937: - C25 + 1e+06 alpha_ob47_seg12[1] <= 999983.7027
 R5938: - C27 + 1e+06 alpha_ob47_seg12[1] <= 999983.7027
 R5939: C24 + 1e+06 alpha_ob47_seg12[2] <= 999921.0477999999
 R5940: C26 + 1e+06 alpha_ob47_seg12[2] <= 999921.0477999999
 R5941: C25 + 1e+06 alpha_ob47_seg12[3] <= 1.0000137473e+06
 R5942: C27 + 1e+06 alpha_ob47_seg12[3] <= 1.0000137473e+06
 R5943: alpha_ob47_seg12[0] + alpha_ob47_seg12[1] + alpha_ob47_seg12[2]
   + alpha_ob47_seg12[3] >= 1
 R5944: - C24 + 1e+06 alpha_ob48_seg12[0] <= 1.0000445522183661e+06
 R5945: - C26 + 1e+06 alpha_ob48_seg12[0] <= 1.0000445522183661e+06
 R5946: - C25 + 1e+06 alpha_ob48_seg12[1] <= 999973.8526997245
 R5947: - C27 + 1e+06 alpha_ob48_seg12[1] <= 999973.8526997245
 R5948: C24 + 1e+06 alpha_ob48_seg12[2] <= 999952.8978183661
 R5949: C26 + 1e+06 alpha_ob48_seg12[2] <= 999952.8978183661
 R5950: C25 + 1e+06 alpha_ob48_seg12[3] <= 1.0000137472997245e+06
 R5951: C27 + 1e+06 alpha_ob48_seg12[3] <= 1.0000137472997245e+06
 R5952: alpha_ob48_seg12[0] + alpha_ob48_seg12[1] + alpha_ob48_seg12[2]
   + alpha_ob48_seg12[3] >= 1
 R5953: - C24 + 1e+06 alpha_ob49_seg12[0] <= 1.000064332081634e+06
 R5954: - C26 + 1e+06 alpha_ob49_seg12[0] <= 1.000064332081634e+06
 R5955: - C25 + 1e+06 alpha_ob49_seg12[1] <= 999973.9401002756
 R5956: - C27 + 1e+06 alpha_ob49_seg12[1] <= 999973.9401002756
 R5957: C24 + 1e+06 alpha_ob49_seg12[2] <= 999933.1178816339
 R5958: C26 + 1e+06 alpha_ob49_seg12[2] <= 999933.1178816339
 R5959: C25 + 1e+06 alpha_ob49_seg12[3] <= 1.0000136599002755e+06
 R5960: C27 + 1e+06 alpha_ob49_seg12[3] <= 1.0000136599002755e+06
 R5961: alpha_ob49_seg12[0] + alpha_ob49_seg12[1] + alpha_ob49_seg12[2]
   + alpha_ob49_seg12[3] >= 1
 R5962: - C24 + 1e+06 alpha_ob50_seg12[0] <= 1.0000643321013774e+06
 R5963: - C26 + 1e+06 alpha_ob50_seg12[0] <= 1.0000643321013774e+06
 R5964: - C25 + 1e+06 alpha_ob50_seg12[1] <= 999983.1900997246
 R5965: - C27 + 1e+06 alpha_ob50_seg12[1] <= 999983.1900997246
 R5966: C24 + 1e+06 alpha_ob50_seg12[2] <= 999933.1179013775
 R5967: C26 + 1e+06 alpha_ob50_seg12[2] <= 999933.1179013775
 R5968: C25 + 1e+06 alpha_ob50_seg12[3] <= 1.0000136598997245e+06
 R5969: C27 + 1e+06 alpha_ob50_seg12[3] <= 1.0000136598997245e+06
 R5970: alpha_ob50_seg12[0] + alpha_ob50_seg12[1] + alpha_ob50_seg12[2]
   + alpha_ob50_seg12[3] >= 1
 R5971: - C26 + 1e+06 alpha_ob0_seg13[0] <= 1.0000284170499525e+06
 R5972: - C28 + 1e+06 alpha_ob0_seg13[0] <= 1.0000284170499525e+06
 R5973: - C27 + 1e+06 alpha_ob0_seg13[1] <= 999957.0843002755
 R5974: - C29 + 1e+06 alpha_ob0_seg13[1] <= 999957.0843002755
 R5975: C26 + 1e+06 alpha_ob0_seg13[2] <= 999969.0328499526
 R5976: C28 + 1e+06 alpha_ob0_seg13[2] <= 999969.0328499526
 R5977: C27 + 1e+06 alpha_ob0_seg13[3] <= 1.0000132657002755e+06
 R5978: C29 + 1e+06 alpha_ob0_seg13[3] <= 1.0000132657002755e+06
 R5979: alpha_ob0_seg13[0] + alpha_ob0_seg13[1] + alpha_ob0_seg13[2]
   + alpha_ob0_seg13[3] >= 1
 R5980: - C26 + 1e+06 alpha_ob1_seg13[0] <= 1.000028417099801e+06
 R5981: - C28 + 1e+06 alpha_ob1_seg13[0] <= 1.000028417099801e+06
 R5982: - C27 + 1e+06 alpha_ob1_seg13[1] <= 999984.1843155898
 R5983: - C29 + 1e+06 alpha_ob1_seg13[1] <= 999984.1843155898
 R5984: C26 + 1e+06 alpha_ob1_seg13[2] <= 999957.4328998009
 R5985: C28 + 1e+06 alpha_ob1_seg13[2] <= 999957.4328998009
 R5986: C27 + 1e+06 alpha_ob1_seg13[3] <= 1.0000132657155899e+06
 R5987: C29 + 1e+06 alpha_ob1_seg13[3] <= 1.0000132657155899e+06
 R5988: alpha_ob1_seg13[0] + alpha_ob1_seg13[1] + alpha_ob1_seg13[2]
   + alpha_ob1_seg13[3] >= 1
 R5989: - C26 + 1e+06 alpha_ob2_seg13[0] <= 1.0000400171500475e+06
 R5990: - C28 + 1e+06 alpha_ob2_seg13[0] <= 1.0000400171500475e+06
 R5991: - C27 + 1e+06 alpha_ob2_seg13[1] <= 999957.0842997245
 R5992: - C29 + 1e+06 alpha_ob2_seg13[1] <= 999957.0842997245
 R5993: C26 + 1e+06 alpha_ob2_seg13[2] <= 999957.4329500474
 R5994: C28 + 1e+06 alpha_ob2_seg13[2] <= 999957.4329500474
 R5995: C27 + 1e+06 alpha_ob2_seg13[3] <= 1.0000132656997246e+06
 R5996: C29 + 1e+06 alpha_ob2_seg13[3] <= 1.0000132656997246e+06
 R5997: alpha_ob2_seg13[0] + alpha_ob2_seg13[1] + alpha_ob2_seg13[2]
   + alpha_ob2_seg13[3] >= 1
 R5998: - C26 + 1e+06 alpha_ob3_seg13[0] <= 1.0000109161752058e+06
 R5999: - C28 + 1e+06 alpha_ob3_seg13[0] <= 1.0000109161752058e+06
 R6000: - C27 + 1e+06 alpha_ob3_seg13[1] <= 999970.2681002755
 R6001: - C29 + 1e+06 alpha_ob3_seg13[1] <= 999970.2681002755
 R6002: C26 + 1e+06 alpha_ob3_seg13[2] <= 999986.5337752058
 R6003: C28 + 1e+06 alpha_ob3_seg13[2] <= 999986.5337752058
 R6004: C27 + 1e+06 alpha_ob3_seg13[3] <= 1.0000138319002755e+06
 R6005: C29 + 1e+06 alpha_ob3_seg13[3] <= 1.0000138319002755e+06
 R6006: alpha_ob3_seg13[0] + alpha_ob3_seg13[1] + alpha_ob3_seg13[2]
   + alpha_ob3_seg13[3] >= 1
 R6007: - C26 + 1e+06 alpha_ob4_seg13[0] <= 1.000010916199801e+06
 R6008: - C28 + 1e+06 alpha_ob4_seg13[0] <= 1.000010916199801e+06
 R6009: - C27 + 1e+06 alpha_ob4_seg13[1] <= 999983.6181179117
 R6010: - C29 + 1e+06 alpha_ob4_seg13[1] <= 999983.6181179117
 R6011: C26 + 1e+06 alpha_ob4_seg13[2] <= 999973.183799801
 R6012: C28 + 1e+06 alpha_ob4_seg13[2] <= 999973.183799801
 R6013: C27 + 1e+06 alpha_ob4_seg13[3] <= 1.0000138319179118e+06
 R6014: C29 + 1e+06 alpha_ob4_seg13[3] <= 1.0000138319179118e+06
 R6015: alpha_ob4_seg13[0] + alpha_ob4_seg13[1] + alpha_ob4_seg13[2]
   + alpha_ob4_seg13[3] >= 1
 R6016: - C26 + 1e+06 alpha_ob5_seg13[0] <= 999992.49102
 R6017: - C28 + 1e+06 alpha_ob5_seg13[0] <= 999992.49102
 R6018: - C27 + 1e+06 alpha_ob5_seg13[1] <= 999957.1016000001
 R6019: - C29 + 1e+06 alpha_ob5_seg13[1] <= 999957.1016000001
 R6020: C26 + 1e+06 alpha_ob5_seg13[2] <= 999986.35898
 R6021: C28 + 1e+06 alpha_ob5_seg13[2] <= 999986.35898
 R6022: C27 + 1e+06 alpha_ob5_seg13[3] <= 1.0000403484e+06
 R6023: C29 + 1e+06 alpha_ob5_seg13[3] <= 1.0000403484e+06
 R6024: alpha_ob5_seg13[0] + alpha_ob5_seg13[1] + alpha_ob5_seg13[2]
   + alpha_ob5_seg13[3] >= 1
 R6025: - C26 + 1e+06 alpha_ob6_seg13[0] <= 999981.3909999999
 R6026: - C28 + 1e+06 alpha_ob6_seg13[0] <= 999981.3909999999
 R6027: - C27 + 1e+06 alpha_ob6_seg13[1] <= 999957.1016000001
 R6028: - C29 + 1e+06 alpha_ob6_seg13[1] <= 999957.1016000001
 R6029: C26 + 1e+06 alpha_ob6_seg13[2] <= 1.000004959e+06
 R6030: C28 + 1e+06 alpha_ob6_seg13[2] <= 1.000004959e+06
 R6031: C27 + 1e+06 alpha_ob6_seg13[3] <= 1.0000403484e+06
 R6032: C29 + 1e+06 alpha_ob6_seg13[3] <= 1.0000403484e+06
 R6033: alpha_ob6_seg13[0] + alpha_ob6_seg13[1] + alpha_ob6_seg13[2]
   + alpha_ob6_seg13[3] >= 1
 R6034: - C26 + 1e+06 alpha_ob7_seg13[0] <= 999981.3909825523
 R6035: - C28 + 1e+06 alpha_ob7_seg13[0] <= 999981.3909825523
 R6036: - C27 + 1e+06 alpha_ob7_seg13[1] <= 999957.1016002755
 R6037: - C29 + 1e+06 alpha_ob7_seg13[1] <= 999957.1016002755
 R6038: C26 + 1e+06 alpha_ob7_seg13[2] <= 1.0000160589825523e+06
 R6039: C28 + 1e+06 alpha_ob7_seg13[2] <= 1.0000160589825523e+06
 R6040: C27 + 1e+06 alpha_ob7_seg13[3] <= 1.0000309984002755e+06
 R6041: C29 + 1e+06 alpha_ob7_seg13[3] <= 1.0000309984002755e+06
 R6042: alpha_ob7_seg13[0] + alpha_ob7_seg13[1] + alpha_ob7_seg13[2]
   + alpha_ob7_seg13[3] >= 1
 R6043: - C26 + 1e+06 alpha_ob8_seg13[0] <= 999981.3867954084
 R6044: - C28 + 1e+06 alpha_ob8_seg13[0] <= 999981.3867954084
 R6045: - C27 + 1e+06 alpha_ob8_seg13[1] <= 999968.8595002755
 R6046: - C29 + 1e+06 alpha_ob8_seg13[1] <= 999968.8595002755
 R6047: C26 + 1e+06 alpha_ob8_seg13[2] <= 1.0000160631954084e+06
 R6048: C28 + 1e+06 alpha_ob8_seg13[2] <= 1.0000160631954084e+06
 R6049: C27 + 1e+06 alpha_ob8_seg13[3] <= 1.0000262405002755e+06
 R6050: C29 + 1e+06 alpha_ob8_seg13[3] <= 1.0000262405002755e+06
 R6051: alpha_ob8_seg13[0] + alpha_ob8_seg13[1] + alpha_ob8_seg13[2]
   + alpha_ob8_seg13[3] >= 1
 R6052: - C26 + 1e+06 alpha_ob9_seg13[0] <= 999981.386799801
 R6053: - C28 + 1e+06 alpha_ob9_seg13[0] <= 999981.386799801
 R6054: - C27 + 1e+06 alpha_ob9_seg13[1] <= 999971.2095013268
 R6055: - C29 + 1e+06 alpha_ob9_seg13[1] <= 999971.2095013268
 R6056: C26 + 1e+06 alpha_ob9_seg13[2] <= 1.0000152131998009e+06
 R6057: C28 + 1e+06 alpha_ob9_seg13[2] <= 1.0000152131998009e+06
 R6058: C27 + 1e+06 alpha_ob9_seg13[3] <= 1.0000262405013267e+06
 R6059: C29 + 1e+06 alpha_ob9_seg13[3] <= 1.0000262405013267e+06
 R6060: alpha_ob9_seg13[0] + alpha_ob9_seg13[1] + alpha_ob9_seg13[2]
   + alpha_ob9_seg13[3] >= 1
 R6061: - C26 + 1e+06 alpha_ob10_seg13[0] <= 999982.2368022958
 R6062: - C28 + 1e+06 alpha_ob10_seg13[0] <= 999982.2368022958
 R6063: - C27 + 1e+06 alpha_ob10_seg13[1] <= 999970.1094997245
 R6064: - C29 + 1e+06 alpha_ob10_seg13[1] <= 999970.1094997245
 R6065: C26 + 1e+06 alpha_ob10_seg13[2] <= 1.0000152132022957e+06
 R6066: C28 + 1e+06 alpha_ob10_seg13[2] <= 1.0000152132022957e+06
 R6067: C27 + 1e+06 alpha_ob10_seg13[3] <= 1.0000262404997245e+06
 R6068: C29 + 1e+06 alpha_ob10_seg13[3] <= 1.0000262404997245e+06
 R6069: alpha_ob10_seg13[0] + alpha_ob10_seg13[1] + alpha_ob10_seg13[2]
   + alpha_ob10_seg13[3] >= 1
 R6070: - C26 + 1e+06 alpha_ob11_seg13[0] <= 999982.236799801
 R6071: - C28 + 1e+06 alpha_ob11_seg13[0] <= 999982.236799801
 R6072: - C27 + 1e+06 alpha_ob11_seg13[1] <= 999970.1095291895
 R6073: - C29 + 1e+06 alpha_ob11_seg13[1] <= 999970.1095291895
 R6074: C26 + 1e+06 alpha_ob11_seg13[2] <= 999993.3631998011
 R6075: C28 + 1e+06 alpha_ob11_seg13[2] <= 999993.3631998011
 R6076: C27 + 1e+06 alpha_ob11_seg13[3] <= 1.0000273405291894e+06
 R6077: C29 + 1e+06 alpha_ob11_seg13[3] <= 1.0000273405291894e+06
 R6078: alpha_ob11_seg13[0] + alpha_ob11_seg13[1] + alpha_ob11_seg13[2]
   + alpha_ob11_seg13[3] >= 1
 R6079: - C26 + 1e+06 alpha_ob12_seg13[0] <= 999981.6050802565
 R6080: - C28 + 1e+06 alpha_ob12_seg13[0] <= 999981.6050802565
 R6081: - C27 + 1e+06 alpha_ob12_seg13[1] <= 999972.8942002755
 R6082: - C29 + 1e+06 alpha_ob12_seg13[1] <= 999972.8942002755
 R6083: C26 + 1e+06 alpha_ob12_seg13[2] <= 1.0000158448802566e+06
 R6084: C28 + 1e+06 alpha_ob12_seg13[2] <= 1.0000158448802566e+06
 R6085: C27 + 1e+06 alpha_ob12_seg13[3] <= 1.0000139558002756e+06
 R6086: C29 + 1e+06 alpha_ob12_seg13[3] <= 1.0000139558002756e+06
 R6087: alpha_ob12_seg13[0] + alpha_ob12_seg13[1] + alpha_ob12_seg13[2]
   + alpha_ob12_seg13[3] >= 1
 R6088: - C26 + 1e+06 alpha_ob13_seg13[0] <= 999981.605149801
 R6089: - C28 + 1e+06 alpha_ob13_seg13[0] <= 999981.605149801
 R6090: - C27 + 1e+06 alpha_ob13_seg13[1] <= 999983.4942391404
 R6091: - C29 + 1e+06 alpha_ob13_seg13[1] <= 999983.4942391404
 R6092: C26 + 1e+06 alpha_ob13_seg13[2] <= 999986.4948498011
 R6093: C28 + 1e+06 alpha_ob13_seg13[2] <= 999986.4948498011
 R6094: C27 + 1e+06 alpha_ob13_seg13[3] <= 1.0000139558391405e+06
 R6095: C29 + 1e+06 alpha_ob13_seg13[3] <= 1.0000139558391405e+06
 R6096: alpha_ob13_seg13[0] + alpha_ob13_seg13[1] + alpha_ob13_seg13[2]
   + alpha_ob13_seg13[3] >= 1
 R6097: - C26 + 1e+06 alpha_ob14_seg13[0] <= 999997.6803502565
 R6098: - C28 + 1e+06 alpha_ob14_seg13[0] <= 999997.6803502565
 R6099: - C27 + 1e+06 alpha_ob14_seg13[1] <= 999972.7268002755
 R6100: - C29 + 1e+06 alpha_ob14_seg13[1] <= 999972.7268002755
 R6101: C26 + 1e+06 alpha_ob14_seg13[2] <= 999999.7696102565
 R6102: C28 + 1e+06 alpha_ob14_seg13[2] <= 999999.7696102565
 R6103: C27 + 1e+06 alpha_ob14_seg13[3] <= 1.0000141232002755e+06
 R6104: C29 + 1e+06 alpha_ob14_seg13[3] <= 1.0000141232002755e+06
 R6105: alpha_ob14_seg13[0] + alpha_ob14_seg13[1] + alpha_ob14_seg13[2]
   + alpha_ob14_seg13[3] >= 1
 R6106: - C26 + 1e+06 alpha_ob15_seg13[0] <= 1.0000038341502565e+06
 R6107: - C28 + 1e+06 alpha_ob15_seg13[0] <= 1.0000038341502565e+06
 R6108: - C27 + 1e+06 alpha_ob15_seg13[1] <= 999972.6012002755
 R6109: - C29 + 1e+06 alpha_ob15_seg13[1] <= 999972.6012002755
 R6110: C26 + 1e+06 alpha_ob15_seg13[2] <= 999993.6158102566
 R6111: C28 + 1e+06 alpha_ob15_seg13[2] <= 999993.6158102566
 R6112: C27 + 1e+06 alpha_ob15_seg13[3] <= 1.0000142488002755e+06
 R6113: C29 + 1e+06 alpha_ob15_seg13[3] <= 1.0000142488002755e+06
 R6114: alpha_ob15_seg13[0] + alpha_ob15_seg13[1] + alpha_ob15_seg13[2]
   + alpha_ob15_seg13[3] >= 1
 R6115: - C26 + 1e+06 alpha_ob16_seg13[0] <= 1.0000162874063333e+06
 R6116: - C28 + 1e+06 alpha_ob16_seg13[0] <= 1.0000162874063333e+06
 R6117: - C27 + 1e+06 alpha_ob16_seg13[1] <= 999982.9439602756
 R6118: - C29 + 1e+06 alpha_ob16_seg13[1] <= 999982.9439602756
 R6119: C26 + 1e+06 alpha_ob16_seg13[2] <= 999981.1624063333
 R6120: C28 + 1e+06 alpha_ob16_seg13[2] <= 999981.1624063333
 R6121: C27 + 1e+06 alpha_ob16_seg13[3] <= 999963.6560402757
 R6122: C29 + 1e+06 alpha_ob16_seg13[3] <= 999963.6560402757
 R6123: alpha_ob16_seg13[0] + alpha_ob16_seg13[1] + alpha_ob16_seg13[2]
   + alpha_ob16_seg13[3] >= 1
 R6124: - C26 + 1e+06 alpha_ob17_seg13[0] <= 999980.937509
 R6125: - C28 + 1e+06 alpha_ob17_seg13[0] <= 999980.937509
 R6126: - C27 + 1e+06 alpha_ob17_seg13[1] <= 1.000034894e+06
 R6127: - C29 + 1e+06 alpha_ob17_seg13[1] <= 1.000034894e+06
 R6128: C26 + 1e+06 alpha_ob17_seg13[2] <= 999981.162491
 R6129: C28 + 1e+06 alpha_ob17_seg13[2] <= 999981.162491
 R6130: C27 + 1e+06 alpha_ob17_seg13[3] <= 999962.556
 R6131: C29 + 1e+06 alpha_ob17_seg13[3] <= 999962.556
 R6132: alpha_ob17_seg13[0] + alpha_ob17_seg13[1] + alpha_ob17_seg13[2]
   + alpha_ob17_seg13[3] >= 1
 R6133: - C26 + 1e+06 alpha_ob18_seg13[0] <= 999990.2670277582
 R6134: - C28 + 1e+06 alpha_ob18_seg13[0] <= 999990.2670277582
 R6135: - C27 + 1e+06 alpha_ob18_seg13[1] <= 1.0000347702002756e+06
 R6136: - C29 + 1e+06 alpha_ob18_seg13[1] <= 1.0000347702002756e+06
 R6137: C26 + 1e+06 alpha_ob18_seg13[2] <= 1.0000071828877581e+06
 R6138: C28 + 1e+06 alpha_ob18_seg13[2] <= 1.0000071828877581e+06
 R6139: C27 + 1e+06 alpha_ob18_seg13[3] <= 999939.8298002756
 R6140: C29 + 1e+06 alpha_ob18_seg13[3] <= 999939.8298002756
 R6141: alpha_ob18_seg13[0] + alpha_ob18_seg13[1] + alpha_ob18_seg13[2]
   + alpha_ob18_seg13[3] >= 1
 R6142: - C26 + 1e+06 alpha_ob19_seg13[0] <= 999990.267069801
 R6143: - C28 + 1e+06 alpha_ob19_seg13[0] <= 999990.267069801
 R6144: - C27 + 1e+06 alpha_ob19_seg13[1] <= 1.0000576202069657e+06
 R6145: - C29 + 1e+06 alpha_ob19_seg13[1] <= 1.0000576202069657e+06
 R6146: C26 + 1e+06 alpha_ob19_seg13[2] <= 1.000002082929801e+06
 R6147: C28 + 1e+06 alpha_ob19_seg13[2] <= 1.000002082929801e+06
 R6148: C27 + 1e+06 alpha_ob19_seg13[3] <= 999939.8298069657
 R6149: C29 + 1e+06 alpha_ob19_seg13[3] <= 999939.8298069657
 R6150: alpha_ob19_seg13[0] + alpha_ob19_seg13[1] + alpha_ob19_seg13[2]
   + alpha_ob19_seg13[3] >= 1
 R6151: - C26 + 1e+06 alpha_ob20_seg13[0] <= 999991.073732
 R6152: - C28 + 1e+06 alpha_ob20_seg13[0] <= 999991.073732
 R6153: - C27 + 1e+06 alpha_ob20_seg13[1] <= 999952.2125
 R6154: - C29 + 1e+06 alpha_ob20_seg13[1] <= 999952.2125
 R6155: C26 + 1e+06 alpha_ob20_seg13[2] <= 999990.776268
 R6156: C28 + 1e+06 alpha_ob20_seg13[2] <= 999990.776268
 R6157: C27 + 1e+06 alpha_ob20_seg13[3] <= 1.0000452375e+06
 R6158: C29 + 1e+06 alpha_ob20_seg13[3] <= 1.0000452375e+06
 R6159: alpha_ob20_seg13[0] + alpha_ob20_seg13[1] + alpha_ob20_seg13[2]
   + alpha_ob20_seg13[3] >= 1
 R6160: - C26 + 1e+06 alpha_ob21_seg13[0] <= 999985.657099801
 R6161: - C28 + 1e+06 alpha_ob21_seg13[0] <= 999985.657099801
 R6162: - C27 + 1e+06 alpha_ob21_seg13[1] <= 1.0000576202063023e+06
 R6163: - C29 + 1e+06 alpha_ob21_seg13[1] <= 1.0000576202063023e+06
 R6164: C26 + 1e+06 alpha_ob21_seg13[2] <= 1.000007192899801e+06
 R6165: C28 + 1e+06 alpha_ob21_seg13[2] <= 1.000007192899801e+06
 R6166: C27 + 1e+06 alpha_ob21_seg13[3] <= 999939.8298063023
 R6167: C29 + 1e+06 alpha_ob21_seg13[3] <= 999939.8298063023
 R6168: alpha_ob21_seg13[0] + alpha_ob21_seg13[1] + alpha_ob21_seg13[2]
   + alpha_ob21_seg13[3] >= 1
 R6169: - C26 + 1e+06 alpha_ob22_seg13[0] <= 999994.771829801
 R6170: - C28 + 1e+06 alpha_ob22_seg13[0] <= 999994.771829801
 R6171: - C27 + 1e+06 alpha_ob22_seg13[1] <= 1.0000615983063023e+06
 R6172: - C29 + 1e+06 alpha_ob22_seg13[1] <= 1.0000615983063023e+06
 R6173: C26 + 1e+06 alpha_ob22_seg13[2] <= 999998.078169801
 R6174: C28 + 1e+06 alpha_ob22_seg13[2] <= 999998.078169801
 R6175: C27 + 1e+06 alpha_ob22_seg13[3] <= 999935.8517063023
 R6176: C29 + 1e+06 alpha_ob22_seg13[3] <= 999935.8517063023
 R6177: alpha_ob22_seg13[0] + alpha_ob22_seg13[1] + alpha_ob22_seg13[2]
   + alpha_ob22_seg13[3] >= 1
 R6178: - C26 + 1e+06 alpha_ob23_seg13[0] <= 999980.9374508709
 R6179: - C28 + 1e+06 alpha_ob23_seg13[0] <= 999980.9374508709
 R6180: - C27 + 1e+06 alpha_ob23_seg13[1] <= 1.0000348940002755e+06
 R6181: - C29 + 1e+06 alpha_ob23_seg13[1] <= 1.0000348940002755e+06
 R6182: C26 + 1e+06 alpha_ob23_seg13[2] <= 1.0000165124508708e+06
 R6183: C28 + 1e+06 alpha_ob23_seg13[2] <= 1.0000165124508708e+06
 R6184: C27 + 1e+06 alpha_ob23_seg13[3] <= 999935.9560002756
 R6185: C29 + 1e+06 alpha_ob23_seg13[3] <= 999935.9560002756
 R6186: alpha_ob23_seg13[0] + alpha_ob23_seg13[1] + alpha_ob23_seg13[2]
   + alpha_ob23_seg13[3] >= 1
 R6187: - C26 + 1e+06 alpha_ob24_seg13[0] <= 999980.937499801
 R6188: - C28 + 1e+06 alpha_ob24_seg13[0] <= 999980.937499801
 R6189: - C27 + 1e+06 alpha_ob24_seg13[1] <= 1.0000614940072973e+06
 R6190: - C29 + 1e+06 alpha_ob24_seg13[1] <= 1.0000614940072973e+06
 R6191: C26 + 1e+06 alpha_ob24_seg13[2] <= 1.000011162499801e+06
 R6192: C28 + 1e+06 alpha_ob24_seg13[2] <= 1.000011162499801e+06
 R6193: C27 + 1e+06 alpha_ob24_seg13[3] <= 999935.9560072974
 R6194: C29 + 1e+06 alpha_ob24_seg13[3] <= 999935.9560072974
 R6195: alpha_ob24_seg13[0] + alpha_ob24_seg13[1] + alpha_ob24_seg13[2]
   + alpha_ob24_seg13[3] >= 1
 R6196: - C26 + 1e+06 alpha_ob25_seg13[0] <= 999956.8394000001
 R6197: - C28 + 1e+06 alpha_ob25_seg13[0] <= 999956.8394000001
 R6198: - C27 + 1e+06 alpha_ob25_seg13[1] <= 999952.2463
 R6199: - C29 + 1e+06 alpha_ob25_seg13[1] <= 999952.2463
 R6200: C26 + 1e+06 alpha_ob25_seg13[2] <= 1.0000097606e+06
 R6201: C28 + 1e+06 alpha_ob25_seg13[2] <= 1.0000097606e+06
 R6202: C27 + 1e+06 alpha_ob25_seg13[3] <= 1.0000452037e+06
 R6203: C29 + 1e+06 alpha_ob25_seg13[3] <= 1.0000452037e+06
 R6204: alpha_ob25_seg13[0] + alpha_ob25_seg13[1] + alpha_ob25_seg13[2]
   + alpha_ob25_seg13[3] >= 1
 R6205: - C26 + 1e+06 alpha_ob26_seg13[0] <= 999980.9375348955
 R6206: - C28 + 1e+06 alpha_ob26_seg13[0] <= 999980.9375348955
 R6207: - C27 + 1e+06 alpha_ob26_seg13[1] <= 1.0000160439997246e+06
 R6208: - C29 + 1e+06 alpha_ob26_seg13[1] <= 1.0000160439997246e+06
 R6209: C26 + 1e+06 alpha_ob26_seg13[2] <= 1.0000165125348954e+06
 R6210: C28 + 1e+06 alpha_ob26_seg13[2] <= 1.0000165125348954e+06
 R6211: C27 + 1e+06 alpha_ob26_seg13[3] <= 999962.5559997245
 R6212: C29 + 1e+06 alpha_ob26_seg13[3] <= 999962.5559997245
 R6213: alpha_ob26_seg13[0] + alpha_ob26_seg13[1] + alpha_ob26_seg13[2]
   + alpha_ob26_seg13[3] >= 1
 R6214: - C26 + 1e+06 alpha_ob27_seg13[0] <= 999956.839399801
 R6215: - C28 + 1e+06 alpha_ob27_seg13[0] <= 999956.839399801
 R6216: - C27 + 1e+06 alpha_ob27_seg13[1] <= 999925.3963434525
 R6217: - C29 + 1e+06 alpha_ob27_seg13[1] <= 999925.3963434525
 R6218: C26 + 1e+06 alpha_ob27_seg13[2] <= 1.0000080105998011e+06
 R6219: C28 + 1e+06 alpha_ob27_seg13[2] <= 1.0000080105998011e+06
 R6220: C27 + 1e+06 alpha_ob27_seg13[3] <= 1.0000720537434525e+06
 R6221: C29 + 1e+06 alpha_ob27_seg13[3] <= 1.0000720537434525e+06
 R6222: alpha_ob27_seg13[0] + alpha_ob27_seg13[1] + alpha_ob27_seg13[2]
   + alpha_ob27_seg13[3] >= 1
 R6223: - C26 + 1e+06 alpha_ob28_seg13[0] <= 999980.9374998009
 R6224: - C28 + 1e+06 alpha_ob28_seg13[0] <= 999980.9374998009
 R6225: - C27 + 1e+06 alpha_ob28_seg13[1] <= 1.0000160440053071e+06
 R6226: - C29 + 1e+06 alpha_ob28_seg13[1] <= 1.0000160440053071e+06
 R6227: C26 + 1e+06 alpha_ob28_seg13[2] <= 1.000012662499801e+06
 R6228: C28 + 1e+06 alpha_ob28_seg13[2] <= 1.000012662499801e+06
 R6229: C27 + 1e+06 alpha_ob28_seg13[3] <= 999981.4060053072
 R6230: C29 + 1e+06 alpha_ob28_seg13[3] <= 999981.4060053072
 R6231: alpha_ob28_seg13[0] + alpha_ob28_seg13[1] + alpha_ob28_seg13[2]
   + alpha_ob28_seg13[3] >= 1
 R6232: - C26 + 1e+06 alpha_ob29_seg13[0] <= 999983.6053459151
 R6233: - C28 + 1e+06 alpha_ob29_seg13[0] <= 999983.6053459151
 R6234: - C27 + 1e+06 alpha_ob29_seg13[1] <= 999987.7722497246
 R6235: - C29 + 1e+06 alpha_ob29_seg13[1] <= 999987.7722497246
 R6236: C26 + 1e+06 alpha_ob29_seg13[2] <= 1.000013844745915e+06
 R6237: C28 + 1e+06 alpha_ob29_seg13[2] <= 1.000013844745915e+06
 R6238: C27 + 1e+06 alpha_ob29_seg13[3] <= 999984.8277497246
 R6239: C29 + 1e+06 alpha_ob29_seg13[3] <= 999984.8277497246
 R6240: alpha_ob29_seg13[0] + alpha_ob29_seg13[1] + alpha_ob29_seg13[2]
   + alpha_ob29_seg13[3] >= 1
 R6241: - C26 + 1e+06 alpha_ob30_seg13[0] <= 999958.8717898987
 R6242: - C28 + 1e+06 alpha_ob30_seg13[0] <= 999958.8717898987
 R6243: - C27 + 1e+06 alpha_ob30_seg13[1] <= 999987.7837602755
 R6244: - C29 + 1e+06 alpha_ob30_seg13[1] <= 999987.7837602755
 R6245: C26 + 1e+06 alpha_ob30_seg13[2] <= 1.0000385781898986e+06
 R6246: C28 + 1e+06 alpha_ob30_seg13[2] <= 1.0000385781898986e+06
 R6247: C27 + 1e+06 alpha_ob30_seg13[3] <= 1.0000043162402755e+06
 R6248: C29 + 1e+06 alpha_ob30_seg13[3] <= 1.0000043162402755e+06
 R6249: alpha_ob30_seg13[0] + alpha_ob30_seg13[1] + alpha_ob30_seg13[2]
   + alpha_ob30_seg13[3] >= 1
 R6250: - C26 + 1e+06 alpha_ob31_seg13[0] <= 999939.2718
 R6251: - C28 + 1e+06 alpha_ob31_seg13[0] <= 999939.2718
 R6252: - C27 + 1e+06 alpha_ob31_seg13[1] <= 999993.13376
 R6253: - C29 + 1e+06 alpha_ob31_seg13[1] <= 999993.13376
 R6254: C26 + 1e+06 alpha_ob31_seg13[2] <= 1.0000385782e+06
 R6255: C28 + 1e+06 alpha_ob31_seg13[2] <= 1.0000385782e+06
 R6256: C27 + 1e+06 alpha_ob31_seg13[3] <= 1.00000431624e+06
 R6257: C29 + 1e+06 alpha_ob31_seg13[3] <= 1.00000431624e+06
 R6258: alpha_ob31_seg13[0] + alpha_ob31_seg13[1] + alpha_ob31_seg13[2]
   + alpha_ob31_seg13[3] >= 1
 R6259: - C26 + 1e+06 alpha_ob32_seg13[0] <= 999939.2718101013
 R6260: - C28 + 1e+06 alpha_ob32_seg13[0] <= 999939.2718101013
 R6261: - C27 + 1e+06 alpha_ob32_seg13[1] <= 999987.7837597246
 R6262: - C29 + 1e+06 alpha_ob32_seg13[1] <= 999987.7837597246
 R6263: C26 + 1e+06 alpha_ob32_seg13[2] <= 1.0000581782101013e+06
 R6264: C28 + 1e+06 alpha_ob32_seg13[2] <= 1.0000581782101013e+06
 R6265: C27 + 1e+06 alpha_ob32_seg13[3] <= 1.0000043162397245e+06
 R6266: C29 + 1e+06 alpha_ob32_seg13[3] <= 1.0000043162397245e+06
 R6267: alpha_ob32_seg13[0] + alpha_ob32_seg13[1] + alpha_ob32_seg13[2]
   + alpha_ob32_seg13[3] >= 1
 R6268: - C26 + 1e+06 alpha_ob33_seg13[0] <= 999939.271799801
 R6269: - C28 + 1e+06 alpha_ob33_seg13[0] <= 999939.271799801
 R6270: - C27 + 1e+06 alpha_ob33_seg13[1] <= 999987.7838262042
 R6271: - C29 + 1e+06 alpha_ob33_seg13[1] <= 999987.7838262042
 R6272: C26 + 1e+06 alpha_ob33_seg13[2] <= 1.000038578199801e+06
 R6273: C28 + 1e+06 alpha_ob33_seg13[2] <= 1.000038578199801e+06
 R6274: C27 + 1e+06 alpha_ob33_seg13[3] <= 1.0000096662262043e+06
 R6275: C29 + 1e+06 alpha_ob33_seg13[3] <= 1.0000096662262043e+06
 R6276: alpha_ob33_seg13[0] + alpha_ob33_seg13[1] + alpha_ob33_seg13[2]
   + alpha_ob33_seg13[3] >= 1
 R6277: - C26 + 1e+06 alpha_ob34_seg13[0] <= 999926.0708
 R6278: - C28 + 1e+06 alpha_ob34_seg13[0] <= 999926.0708
 R6279: - C27 + 1e+06 alpha_ob34_seg13[1] <= 999983.7196
 R6280: - C29 + 1e+06 alpha_ob34_seg13[1] <= 999983.7196
 R6281: C26 + 1e+06 alpha_ob34_seg13[2] <= 1.0000250292e+06
 R6282: C28 + 1e+06 alpha_ob34_seg13[2] <= 1.0000250292e+06
 R6283: C27 + 1e+06 alpha_ob34_seg13[3] <= 1.0000137304e+06
 R6284: C29 + 1e+06 alpha_ob34_seg13[3] <= 1.0000137304e+06
 R6285: alpha_ob34_seg13[0] + alpha_ob34_seg13[1] + alpha_ob34_seg13[2]
   + alpha_ob34_seg13[3] >= 1
 R6286: - C26 + 1e+06 alpha_ob35_seg13[0] <= 999926.0708293857
 R6287: - C28 + 1e+06 alpha_ob35_seg13[0] <= 999926.0708293857
 R6288: - C27 + 1e+06 alpha_ob35_seg13[1] <= 999967.8695997245
 R6289: - C29 + 1e+06 alpha_ob35_seg13[1] <= 999967.8695997245
 R6290: C26 + 1e+06 alpha_ob35_seg13[2] <= 1.0000713792293856e+06
 R6291: C28 + 1e+06 alpha_ob35_seg13[2] <= 1.0000713792293856e+06
 R6292: C27 + 1e+06 alpha_ob35_seg13[3] <= 1.0000137303997246e+06
 R6293: C29 + 1e+06 alpha_ob35_seg13[3] <= 1.0000137303997246e+06
 R6294: alpha_ob35_seg13[0] + alpha_ob35_seg13[1] + alpha_ob35_seg13[2]
   + alpha_ob35_seg13[3] >= 1
 R6295: - C26 + 1e+06 alpha_ob36_seg13[0] <= 999972.6917508709
 R6296: - C28 + 1e+06 alpha_ob36_seg13[0] <= 999972.6917508709
 R6297: - C27 + 1e+06 alpha_ob36_seg13[1] <= 999957.1644002756
 R6298: - C29 + 1e+06 alpha_ob36_seg13[1] <= 999957.1644002756
 R6299: C26 + 1e+06 alpha_ob36_seg13[2] <= 1.0000247581508709e+06
 R6300: C28 + 1e+06 alpha_ob36_seg13[2] <= 1.0000247581508709e+06
 R6301: C27 + 1e+06 alpha_ob36_seg13[3] <= 1.0000136856002755e+06
 R6302: C29 + 1e+06 alpha_ob36_seg13[3] <= 1.0000136856002755e+06
 R6303: alpha_ob36_seg13[0] + alpha_ob36_seg13[1] + alpha_ob36_seg13[2]
   + alpha_ob36_seg13[3] >= 1
 R6304: - C26 + 1e+06 alpha_ob37_seg13[0] <= 999939.0009674003
 R6305: - C28 + 1e+06 alpha_ob37_seg13[0] <= 999939.0009674003
 R6306: - C27 + 1e+06 alpha_ob37_seg13[1] <= 999961.9516002756
 R6307: - C29 + 1e+06 alpha_ob37_seg13[1] <= 999961.9516002756
 R6308: C26 + 1e+06 alpha_ob37_seg13[2] <= 1.0000584489674004e+06
 R6309: C28 + 1e+06 alpha_ob37_seg13[2] <= 1.0000584489674004e+06
 R6310: C27 + 1e+06 alpha_ob37_seg13[3] <= 1.0000178984002755e+06
 R6311: C29 + 1e+06 alpha_ob37_seg13[3] <= 1.0000178984002755e+06
 R6312: alpha_ob37_seg13[0] + alpha_ob37_seg13[1] + alpha_ob37_seg13[2]
   + alpha_ob37_seg13[3] >= 1
 R6313: - C26 + 1e+06 alpha_ob38_seg13[0] <= 999994.9164998011
 R6314: - C28 + 1e+06 alpha_ob38_seg13[0] <= 999994.9164998011
 R6315: - C27 + 1e+06 alpha_ob38_seg13[1] <= 999925.4284782809
 R6316: - C29 + 1e+06 alpha_ob38_seg13[1] <= 999925.4284782809
 R6317: C26 + 1e+06 alpha_ob38_seg13[2] <= 999943.6834998011
 R6318: C28 + 1e+06 alpha_ob38_seg13[2] <= 999943.6834998011
 R6319: C27 + 1e+06 alpha_ob38_seg13[3] <= 1.0000720216782809e+06
 R6320: C29 + 1e+06 alpha_ob38_seg13[3] <= 1.0000720216782809e+06
 R6321: alpha_ob38_seg13[0] + alpha_ob38_seg13[1] + alpha_ob38_seg13[2]
   + alpha_ob38_seg13[3] >= 1
 R6322: - C26 + 1e+06 alpha_ob39_seg13[0] <= 999914.52327842
 R6323: - C28 + 1e+06 alpha_ob39_seg13[0] <= 999914.52327842
 R6324: - C27 + 1e+06 alpha_ob39_seg13[1] <= 999993.6552362755
 R6325: - C29 + 1e+06 alpha_ob39_seg13[1] <= 999993.6552362755
 R6326: C26 + 1e+06 alpha_ob39_seg13[2] <= 1.0000829266784199e+06
 R6327: C28 + 1e+06 alpha_ob39_seg13[2] <= 1.0000829266784199e+06
 R6328: C27 + 1e+06 alpha_ob39_seg13[3] <= 999992.1947642756
 R6329: C29 + 1e+06 alpha_ob39_seg13[3] <= 999992.1947642756
 R6330: alpha_ob39_seg13[0] + alpha_ob39_seg13[1] + alpha_ob39_seg13[2]
   + alpha_ob39_seg13[3] >= 1
 R6331: - C26 + 1e+06 alpha_ob40_seg13[0] <= 999912.265199801
 R6332: - C28 + 1e+06 alpha_ob40_seg13[0] <= 999912.265199801
 R6333: - C27 + 1e+06 alpha_ob40_seg13[1] <= 999987.7842112777
 R6334: - C29 + 1e+06 alpha_ob40_seg13[1] <= 999987.7842112777
 R6335: C26 + 1e+06 alpha_ob40_seg13[2] <= 1.000076834799801e+06
 R6336: C28 + 1e+06 alpha_ob40_seg13[2] <= 1.000076834799801e+06
 R6337: C27 + 1e+06 alpha_ob40_seg13[3] <= 1.0000096658112778e+06
 R6338: C29 + 1e+06 alpha_ob40_seg13[3] <= 1.0000096658112778e+06
 R6339: alpha_ob40_seg13[0] + alpha_ob40_seg13[1] + alpha_ob40_seg13[2]
   + alpha_ob40_seg13[3] >= 1
 R6340: - C26 + 1e+06 alpha_ob41_seg13[0] <= 999920.6152119379
 R6341: - C28 + 1e+06 alpha_ob41_seg13[0] <= 999920.6152119379
 R6342: - C27 + 1e+06 alpha_ob41_seg13[1] <= 999981.4341997246
 R6343: - C29 + 1e+06 alpha_ob41_seg13[1] <= 999981.4341997246
 R6344: C26 + 1e+06 alpha_ob41_seg13[2] <= 1.0000768348119379e+06
 R6345: C28 + 1e+06 alpha_ob41_seg13[2] <= 1.0000768348119379e+06
 R6346: C27 + 1e+06 alpha_ob41_seg13[3] <= 1.0000096657997245e+06
 R6347: C29 + 1e+06 alpha_ob41_seg13[3] <= 1.0000096657997245e+06
 R6348: alpha_ob41_seg13[0] + alpha_ob41_seg13[1] + alpha_ob41_seg13[2]
   + alpha_ob41_seg13[3] >= 1
 R6349: - C26 + 1e+06 alpha_ob42_seg13[0] <= 999919.2652
 R6350: - C28 + 1e+06 alpha_ob42_seg13[0] <= 999919.2652
 R6351: - C27 + 1e+06 alpha_ob42_seg13[1] <= 999981.4342
 R6352: - C29 + 1e+06 alpha_ob42_seg13[1] <= 999981.4342
 R6353: C26 + 1e+06 alpha_ob42_seg13[2] <= 1.0000768348e+06
 R6354: C28 + 1e+06 alpha_ob42_seg13[2] <= 1.0000768348e+06
 R6355: C27 + 1e+06 alpha_ob42_seg13[3] <= 1.0000160158000001e+06
 R6356: C29 + 1e+06 alpha_ob42_seg13[3] <= 1.0000160158000001e+06
 R6357: alpha_ob42_seg13[0] + alpha_ob42_seg13[1] + alpha_ob42_seg13[2]
   + alpha_ob42_seg13[3] >= 1
 R6358: - C26 + 1e+06 alpha_ob43_seg13[0] <= 999919.2652454559
 R6359: - C28 + 1e+06 alpha_ob43_seg13[0] <= 999919.2652454559
 R6360: - C27 + 1e+06 alpha_ob43_seg13[1] <= 999956.8341997246
 R6361: - C29 + 1e+06 alpha_ob43_seg13[1] <= 999956.8341997246
 R6362: C26 + 1e+06 alpha_ob43_seg13[2] <= 1.0000781848454559e+06
 R6363: C28 + 1e+06 alpha_ob43_seg13[2] <= 1.0000781848454559e+06
 R6364: C27 + 1e+06 alpha_ob43_seg13[3] <= 1.0000160157997246e+06
 R6365: C29 + 1e+06 alpha_ob43_seg13[3] <= 1.0000160157997246e+06
 R6366: alpha_ob43_seg13[0] + alpha_ob43_seg13[1] + alpha_ob43_seg13[2]
   + alpha_ob43_seg13[3] >= 1
 R6367: - C26 + 1e+06 alpha_ob44_seg13[0] <= 999991.83683
 R6368: - C28 + 1e+06 alpha_ob44_seg13[0] <= 999991.83683
 R6369: - C27 + 1e+06 alpha_ob44_seg13[1] <= 999940.589
 R6370: - C29 + 1e+06 alpha_ob44_seg13[1] <= 999940.589
 R6371: C26 + 1e+06 alpha_ob44_seg13[2] <= 1.00000401317e+06
 R6372: C28 + 1e+06 alpha_ob44_seg13[2] <= 1.00000401317e+06
 R6373: C27 + 1e+06 alpha_ob44_seg13[3] <= 1.000056861e+06
 R6374: C29 + 1e+06 alpha_ob44_seg13[3] <= 1.000056861e+06
 R6375: alpha_ob44_seg13[0] + alpha_ob44_seg13[1] + alpha_ob44_seg13[2]
   + alpha_ob44_seg13[3] >= 1
 R6376: - C26 + 1e+06 alpha_ob45_seg13[0] <= 999991.8368084199
 R6377: - C28 + 1e+06 alpha_ob45_seg13[0] <= 999991.8368084199
 R6378: - C27 + 1e+06 alpha_ob45_seg13[1] <= 999940.5890002755
 R6379: - C29 + 1e+06 alpha_ob45_seg13[1] <= 999940.5890002755
 R6380: C26 + 1e+06 alpha_ob45_seg13[2] <= 1.0000056131484199e+06
 R6381: C28 + 1e+06 alpha_ob45_seg13[2] <= 1.0000056131484199e+06
 R6382: C27 + 1e+06 alpha_ob45_seg13[3] <= 1.0000452610002755e+06
 R6383: C29 + 1e+06 alpha_ob45_seg13[3] <= 1.0000452610002755e+06
 R6384: alpha_ob45_seg13[0] + alpha_ob45_seg13[1] + alpha_ob45_seg13[2]
   + alpha_ob45_seg13[3] >= 1
 R6385: - C26 + 1e+06 alpha_ob46_seg13[0] <= 1.0000764021508709e+06
 R6386: - C28 + 1e+06 alpha_ob46_seg13[0] <= 1.0000764021508709e+06
 R6387: - C27 + 1e+06 alpha_ob46_seg13[1] <= 999957.1027002756
 R6388: - C29 + 1e+06 alpha_ob46_seg13[1] <= 999957.1027002756
 R6389: C26 + 1e+06 alpha_ob46_seg13[2] <= 999921.0477508709
 R6390: C28 + 1e+06 alpha_ob46_seg13[2] <= 999921.0477508709
 R6391: C27 + 1e+06 alpha_ob46_seg13[3] <= 1.0000137473002756e+06
 R6392: C29 + 1e+06 alpha_ob46_seg13[3] <= 1.0000137473002756e+06
 R6393: alpha_ob46_seg13[0] + alpha_ob46_seg13[1] + alpha_ob46_seg13[2]
   + alpha_ob46_seg13[3] >= 1
 R6394: - C26 + 1e+06 alpha_ob47_seg13[0] <= 1.0000445522e+06
 R6395: - C28 + 1e+06 alpha_ob47_seg13[0] <= 1.0000445522e+06
 R6396: - C27 + 1e+06 alpha_ob47_seg13[1] <= 999983.7027
 R6397: - C29 + 1e+06 alpha_ob47_seg13[1] <= 999983.7027
 R6398: C26 + 1e+06 alpha_ob47_seg13[2] <= 999921.0477999999
 R6399: C28 + 1e+06 alpha_ob47_seg13[2] <= 999921.0477999999
 R6400: C27 + 1e+06 alpha_ob47_seg13[3] <= 1.0000137473e+06
 R6401: C29 + 1e+06 alpha_ob47_seg13[3] <= 1.0000137473e+06
 R6402: alpha_ob47_seg13[0] + alpha_ob47_seg13[1] + alpha_ob47_seg13[2]
   + alpha_ob47_seg13[3] >= 1
 R6403: - C26 + 1e+06 alpha_ob48_seg13[0] <= 1.0000445522183661e+06
 R6404: - C28 + 1e+06 alpha_ob48_seg13[0] <= 1.0000445522183661e+06
 R6405: - C27 + 1e+06 alpha_ob48_seg13[1] <= 999973.8526997245
 R6406: - C29 + 1e+06 alpha_ob48_seg13[1] <= 999973.8526997245
 R6407: C26 + 1e+06 alpha_ob48_seg13[2] <= 999952.8978183661
 R6408: C28 + 1e+06 alpha_ob48_seg13[2] <= 999952.8978183661
 R6409: C27 + 1e+06 alpha_ob48_seg13[3] <= 1.0000137472997245e+06
 R6410: C29 + 1e+06 alpha_ob48_seg13[3] <= 1.0000137472997245e+06
 R6411: alpha_ob48_seg13[0] + alpha_ob48_seg13[1] + alpha_ob48_seg13[2]
   + alpha_ob48_seg13[3] >= 1
 R6412: - C26 + 1e+06 alpha_ob49_seg13[0] <= 1.000064332081634e+06
 R6413: - C28 + 1e+06 alpha_ob49_seg13[0] <= 1.000064332081634e+06
 R6414: - C27 + 1e+06 alpha_ob49_seg13[1] <= 999973.9401002756
 R6415: - C29 + 1e+06 alpha_ob49_seg13[1] <= 999973.9401002756
 R6416: C26 + 1e+06 alpha_ob49_seg13[2] <= 999933.1178816339
 R6417: C28 + 1e+06 alpha_ob49_seg13[2] <= 999933.1178816339
 R6418: C27 + 1e+06 alpha_ob49_seg13[3] <= 1.0000136599002755e+06
 R6419: C29 + 1e+06 alpha_ob49_seg13[3] <= 1.0000136599002755e+06
 R6420: alpha_ob49_seg13[0] + alpha_ob49_seg13[1] + alpha_ob49_seg13[2]
   + alpha_ob49_seg13[3] >= 1
 R6421: - C26 + 1e+06 alpha_ob50_seg13[0] <= 1.0000643321013774e+06
 R6422: - C28 + 1e+06 alpha_ob50_seg13[0] <= 1.0000643321013774e+06
 R6423: - C27 + 1e+06 alpha_ob50_seg13[1] <= 999983.1900997246
 R6424: - C29 + 1e+06 alpha_ob50_seg13[1] <= 999983.1900997246
 R6425: C26 + 1e+06 alpha_ob50_seg13[2] <= 999933.1179013775
 R6426: C28 + 1e+06 alpha_ob50_seg13[2] <= 999933.1179013775
 R6427: C27 + 1e+06 alpha_ob50_seg13[3] <= 1.0000136598997245e+06
 R6428: C29 + 1e+06 alpha_ob50_seg13[3] <= 1.0000136598997245e+06
 R6429: alpha_ob50_seg13[0] + alpha_ob50_seg13[1] + alpha_ob50_seg13[2]
   + alpha_ob50_seg13[3] >= 1
 R6430: - C28 + 1e+06 alpha_ob0_seg14[0] <= 1.0000284170499525e+06
 R6431: - C30 + 1e+06 alpha_ob0_seg14[0] <= 1.0000284170499525e+06
 R6432: - C29 + 1e+06 alpha_ob0_seg14[1] <= 999957.0843002755
 R6433: - C31 + 1e+06 alpha_ob0_seg14[1] <= 999957.0843002755
 R6434: C28 + 1e+06 alpha_ob0_seg14[2] <= 999969.0328499526
 R6435: C30 + 1e+06 alpha_ob0_seg14[2] <= 999969.0328499526
 R6436: C29 + 1e+06 alpha_ob0_seg14[3] <= 1.0000132657002755e+06
 R6437: C31 + 1e+06 alpha_ob0_seg14[3] <= 1.0000132657002755e+06
 R6438: alpha_ob0_seg14[0] + alpha_ob0_seg14[1] + alpha_ob0_seg14[2]
   + alpha_ob0_seg14[3] >= 1
 R6439: - C28 + 1e+06 alpha_ob1_seg14[0] <= 1.000028417099801e+06
 R6440: - C30 + 1e+06 alpha_ob1_seg14[0] <= 1.000028417099801e+06
 R6441: - C29 + 1e+06 alpha_ob1_seg14[1] <= 999984.1843155898
 R6442: - C31 + 1e+06 alpha_ob1_seg14[1] <= 999984.1843155898
 R6443: C28 + 1e+06 alpha_ob1_seg14[2] <= 999957.4328998009
 R6444: C30 + 1e+06 alpha_ob1_seg14[2] <= 999957.4328998009
 R6445: C29 + 1e+06 alpha_ob1_seg14[3] <= 1.0000132657155899e+06
 R6446: C31 + 1e+06 alpha_ob1_seg14[3] <= 1.0000132657155899e+06
 R6447: alpha_ob1_seg14[0] + alpha_ob1_seg14[1] + alpha_ob1_seg14[2]
   + alpha_ob1_seg14[3] >= 1
 R6448: - C28 + 1e+06 alpha_ob2_seg14[0] <= 1.0000400171500475e+06
 R6449: - C30 + 1e+06 alpha_ob2_seg14[0] <= 1.0000400171500475e+06
 R6450: - C29 + 1e+06 alpha_ob2_seg14[1] <= 999957.0842997245
 R6451: - C31 + 1e+06 alpha_ob2_seg14[1] <= 999957.0842997245
 R6452: C28 + 1e+06 alpha_ob2_seg14[2] <= 999957.4329500474
 R6453: C30 + 1e+06 alpha_ob2_seg14[2] <= 999957.4329500474
 R6454: C29 + 1e+06 alpha_ob2_seg14[3] <= 1.0000132656997246e+06
 R6455: C31 + 1e+06 alpha_ob2_seg14[3] <= 1.0000132656997246e+06
 R6456: alpha_ob2_seg14[0] + alpha_ob2_seg14[1] + alpha_ob2_seg14[2]
   + alpha_ob2_seg14[3] >= 1
 R6457: - C28 + 1e+06 alpha_ob3_seg14[0] <= 1.0000109161752058e+06
 R6458: - C30 + 1e+06 alpha_ob3_seg14[0] <= 1.0000109161752058e+06
 R6459: - C29 + 1e+06 alpha_ob3_seg14[1] <= 999970.2681002755
 R6460: - C31 + 1e+06 alpha_ob3_seg14[1] <= 999970.2681002755
 R6461: C28 + 1e+06 alpha_ob3_seg14[2] <= 999986.5337752058
 R6462: C30 + 1e+06 alpha_ob3_seg14[2] <= 999986.5337752058
 R6463: C29 + 1e+06 alpha_ob3_seg14[3] <= 1.0000138319002755e+06
 R6464: C31 + 1e+06 alpha_ob3_seg14[3] <= 1.0000138319002755e+06
 R6465: alpha_ob3_seg14[0] + alpha_ob3_seg14[1] + alpha_ob3_seg14[2]
   + alpha_ob3_seg14[3] >= 1
 R6466: - C28 + 1e+06 alpha_ob4_seg14[0] <= 1.000010916199801e+06
 R6467: - C30 + 1e+06 alpha_ob4_seg14[0] <= 1.000010916199801e+06
 R6468: - C29 + 1e+06 alpha_ob4_seg14[1] <= 999983.6181179117
 R6469: - C31 + 1e+06 alpha_ob4_seg14[1] <= 999983.6181179117
 R6470: C28 + 1e+06 alpha_ob4_seg14[2] <= 999973.183799801
 R6471: C30 + 1e+06 alpha_ob4_seg14[2] <= 999973.183799801
 R6472: C29 + 1e+06 alpha_ob4_seg14[3] <= 1.0000138319179118e+06
 R6473: C31 + 1e+06 alpha_ob4_seg14[3] <= 1.0000138319179118e+06
 R6474: alpha_ob4_seg14[0] + alpha_ob4_seg14[1] + alpha_ob4_seg14[2]
   + alpha_ob4_seg14[3] >= 1
 R6475: - C28 + 1e+06 alpha_ob5_seg14[0] <= 999992.49102
 R6476: - C30 + 1e+06 alpha_ob5_seg14[0] <= 999992.49102
 R6477: - C29 + 1e+06 alpha_ob5_seg14[1] <= 999957.1016000001
 R6478: - C31 + 1e+06 alpha_ob5_seg14[1] <= 999957.1016000001
 R6479: C28 + 1e+06 alpha_ob5_seg14[2] <= 999986.35898
 R6480: C30 + 1e+06 alpha_ob5_seg14[2] <= 999986.35898
 R6481: C29 + 1e+06 alpha_ob5_seg14[3] <= 1.0000403484e+06
 R6482: C31 + 1e+06 alpha_ob5_seg14[3] <= 1.0000403484e+06
 R6483: alpha_ob5_seg14[0] + alpha_ob5_seg14[1] + alpha_ob5_seg14[2]
   + alpha_ob5_seg14[3] >= 1
 R6484: - C28 + 1e+06 alpha_ob6_seg14[0] <= 999981.3909999999
 R6485: - C30 + 1e+06 alpha_ob6_seg14[0] <= 999981.3909999999
 R6486: - C29 + 1e+06 alpha_ob6_seg14[1] <= 999957.1016000001
 R6487: - C31 + 1e+06 alpha_ob6_seg14[1] <= 999957.1016000001
 R6488: C28 + 1e+06 alpha_ob6_seg14[2] <= 1.000004959e+06
 R6489: C30 + 1e+06 alpha_ob6_seg14[2] <= 1.000004959e+06
 R6490: C29 + 1e+06 alpha_ob6_seg14[3] <= 1.0000403484e+06
 R6491: C31 + 1e+06 alpha_ob6_seg14[3] <= 1.0000403484e+06
 R6492: alpha_ob6_seg14[0] + alpha_ob6_seg14[1] + alpha_ob6_seg14[2]
   + alpha_ob6_seg14[3] >= 1
 R6493: - C28 + 1e+06 alpha_ob7_seg14[0] <= 999981.3909825523
 R6494: - C30 + 1e+06 alpha_ob7_seg14[0] <= 999981.3909825523
 R6495: - C29 + 1e+06 alpha_ob7_seg14[1] <= 999957.1016002755
 R6496: - C31 + 1e+06 alpha_ob7_seg14[1] <= 999957.1016002755
 R6497: C28 + 1e+06 alpha_ob7_seg14[2] <= 1.0000160589825523e+06
 R6498: C30 + 1e+06 alpha_ob7_seg14[2] <= 1.0000160589825523e+06
 R6499: C29 + 1e+06 alpha_ob7_seg14[3] <= 1.0000309984002755e+06
 R6500: C31 + 1e+06 alpha_ob7_seg14[3] <= 1.0000309984002755e+06
 R6501: alpha_ob7_seg14[0] + alpha_ob7_seg14[1] + alpha_ob7_seg14[2]
   + alpha_ob7_seg14[3] >= 1
 R6502: - C28 + 1e+06 alpha_ob8_seg14[0] <= 999981.3867954084
 R6503: - C30 + 1e+06 alpha_ob8_seg14[0] <= 999981.3867954084
 R6504: - C29 + 1e+06 alpha_ob8_seg14[1] <= 999968.8595002755
 R6505: - C31 + 1e+06 alpha_ob8_seg14[1] <= 999968.8595002755
 R6506: C28 + 1e+06 alpha_ob8_seg14[2] <= 1.0000160631954084e+06
 R6507: C30 + 1e+06 alpha_ob8_seg14[2] <= 1.0000160631954084e+06
 R6508: C29 + 1e+06 alpha_ob8_seg14[3] <= 1.0000262405002755e+06
 R6509: C31 + 1e+06 alpha_ob8_seg14[3] <= 1.0000262405002755e+06
 R6510: alpha_ob8_seg14[0] + alpha_ob8_seg14[1] + alpha_ob8_seg14[2]
   + alpha_ob8_seg14[3] >= 1
 R6511: - C28 + 1e+06 alpha_ob9_seg14[0] <= 999981.386799801
 R6512: - C30 + 1e+06 alpha_ob9_seg14[0] <= 999981.386799801
 R6513: - C29 + 1e+06 alpha_ob9_seg14[1] <= 999971.2095013268
 R6514: - C31 + 1e+06 alpha_ob9_seg14[1] <= 999971.2095013268
 R6515: C28 + 1e+06 alpha_ob9_seg14[2] <= 1.0000152131998009e+06
 R6516: C30 + 1e+06 alpha_ob9_seg14[2] <= 1.0000152131998009e+06
 R6517: C29 + 1e+06 alpha_ob9_seg14[3] <= 1.0000262405013267e+06
 R6518: C31 + 1e+06 alpha_ob9_seg14[3] <= 1.0000262405013267e+06
 R6519: alpha_ob9_seg14[0] + alpha_ob9_seg14[1] + alpha_ob9_seg14[2]
   + alpha_ob9_seg14[3] >= 1
 R6520: - C28 + 1e+06 alpha_ob10_seg14[0] <= 999982.2368022958
 R6521: - C30 + 1e+06 alpha_ob10_seg14[0] <= 999982.2368022958
 R6522: - C29 + 1e+06 alpha_ob10_seg14[1] <= 999970.1094997245
 R6523: - C31 + 1e+06 alpha_ob10_seg14[1] <= 999970.1094997245
 R6524: C28 + 1e+06 alpha_ob10_seg14[2] <= 1.0000152132022957e+06
 R6525: C30 + 1e+06 alpha_ob10_seg14[2] <= 1.0000152132022957e+06
 R6526: C29 + 1e+06 alpha_ob10_seg14[3] <= 1.0000262404997245e+06
 R6527: C31 + 1e+06 alpha_ob10_seg14[3] <= 1.0000262404997245e+06
 R6528: alpha_ob10_seg14[0] + alpha_ob10_seg14[1] + alpha_ob10_seg14[2]
   + alpha_ob10_seg14[3] >= 1
 R6529: - C28 + 1e+06 alpha_ob11_seg14[0] <= 999982.236799801
 R6530: - C30 + 1e+06 alpha_ob11_seg14[0] <= 999982.236799801
 R6531: - C29 + 1e+06 alpha_ob11_seg14[1] <= 999970.1095291895
 R6532: - C31 + 1e+06 alpha_ob11_seg14[1] <= 999970.1095291895
 R6533: C28 + 1e+06 alpha_ob11_seg14[2] <= 999993.3631998011
 R6534: C30 + 1e+06 alpha_ob11_seg14[2] <= 999993.3631998011
 R6535: C29 + 1e+06 alpha_ob11_seg14[3] <= 1.0000273405291894e+06
 R6536: C31 + 1e+06 alpha_ob11_seg14[3] <= 1.0000273405291894e+06
 R6537: alpha_ob11_seg14[0] + alpha_ob11_seg14[1] + alpha_ob11_seg14[2]
   + alpha_ob11_seg14[3] >= 1
 R6538: - C28 + 1e+06 alpha_ob12_seg14[0] <= 999981.6050802565
 R6539: - C30 + 1e+06 alpha_ob12_seg14[0] <= 999981.6050802565
 R6540: - C29 + 1e+06 alpha_ob12_seg14[1] <= 999972.8942002755
 R6541: - C31 + 1e+06 alpha_ob12_seg14[1] <= 999972.8942002755
 R6542: C28 + 1e+06 alpha_ob12_seg14[2] <= 1.0000158448802566e+06
 R6543: C30 + 1e+06 alpha_ob12_seg14[2] <= 1.0000158448802566e+06
 R6544: C29 + 1e+06 alpha_ob12_seg14[3] <= 1.0000139558002756e+06
 R6545: C31 + 1e+06 alpha_ob12_seg14[3] <= 1.0000139558002756e+06
 R6546: alpha_ob12_seg14[0] + alpha_ob12_seg14[1] + alpha_ob12_seg14[2]
   + alpha_ob12_seg14[3] >= 1
 R6547: - C28 + 1e+06 alpha_ob13_seg14[0] <= 999981.605149801
 R6548: - C30 + 1e+06 alpha_ob13_seg14[0] <= 999981.605149801
 R6549: - C29 + 1e+06 alpha_ob13_seg14[1] <= 999983.4942391404
 R6550: - C31 + 1e+06 alpha_ob13_seg14[1] <= 999983.4942391404
 R6551: C28 + 1e+06 alpha_ob13_seg14[2] <= 999986.4948498011
 R6552: C30 + 1e+06 alpha_ob13_seg14[2] <= 999986.4948498011
 R6553: C29 + 1e+06 alpha_ob13_seg14[3] <= 1.0000139558391405e+06
 R6554: C31 + 1e+06 alpha_ob13_seg14[3] <= 1.0000139558391405e+06
 R6555: alpha_ob13_seg14[0] + alpha_ob13_seg14[1] + alpha_ob13_seg14[2]
   + alpha_ob13_seg14[3] >= 1
 R6556: - C28 + 1e+06 alpha_ob14_seg14[0] <= 999997.6803502565
 R6557: - C30 + 1e+06 alpha_ob14_seg14[0] <= 999997.6803502565
 R6558: - C29 + 1e+06 alpha_ob14_seg14[1] <= 999972.7268002755
 R6559: - C31 + 1e+06 alpha_ob14_seg14[1] <= 999972.7268002755
 R6560: C28 + 1e+06 alpha_ob14_seg14[2] <= 999999.7696102565
 R6561: C30 + 1e+06 alpha_ob14_seg14[2] <= 999999.7696102565
 R6562: C29 + 1e+06 alpha_ob14_seg14[3] <= 1.0000141232002755e+06
 R6563: C31 + 1e+06 alpha_ob14_seg14[3] <= 1.0000141232002755e+06
 R6564: alpha_ob14_seg14[0] + alpha_ob14_seg14[1] + alpha_ob14_seg14[2]
   + alpha_ob14_seg14[3] >= 1
 R6565: - C28 + 1e+06 alpha_ob15_seg14[0] <= 1.0000038341502565e+06
 R6566: - C30 + 1e+06 alpha_ob15_seg14[0] <= 1.0000038341502565e+06
 R6567: - C29 + 1e+06 alpha_ob15_seg14[1] <= 999972.6012002755
 R6568: - C31 + 1e+06 alpha_ob15_seg14[1] <= 999972.6012002755
 R6569: C28 + 1e+06 alpha_ob15_seg14[2] <= 999993.6158102566
 R6570: C30 + 1e+06 alpha_ob15_seg14[2] <= 999993.6158102566
 R6571: C29 + 1e+06 alpha_ob15_seg14[3] <= 1.0000142488002755e+06
 R6572: C31 + 1e+06 alpha_ob15_seg14[3] <= 1.0000142488002755e+06
 R6573: alpha_ob15_seg14[0] + alpha_ob15_seg14[1] + alpha_ob15_seg14[2]
   + alpha_ob15_seg14[3] >= 1
 R6574: - C28 + 1e+06 alpha_ob16_seg14[0] <= 1.0000162874063333e+06
 R6575: - C30 + 1e+06 alpha_ob16_seg14[0] <= 1.0000162874063333e+06
 R6576: - C29 + 1e+06 alpha_ob16_seg14[1] <= 999982.9439602756
 R6577: - C31 + 1e+06 alpha_ob16_seg14[1] <= 999982.9439602756
 R6578: C28 + 1e+06 alpha_ob16_seg14[2] <= 999981.1624063333
 R6579: C30 + 1e+06 alpha_ob16_seg14[2] <= 999981.1624063333
 R6580: C29 + 1e+06 alpha_ob16_seg14[3] <= 999963.6560402757
 R6581: C31 + 1e+06 alpha_ob16_seg14[3] <= 999963.6560402757
 R6582: alpha_ob16_seg14[0] + alpha_ob16_seg14[1] + alpha_ob16_seg14[2]
   + alpha_ob16_seg14[3] >= 1
 R6583: - C28 + 1e+06 alpha_ob17_seg14[0] <= 999980.937509
 R6584: - C30 + 1e+06 alpha_ob17_seg14[0] <= 999980.937509
 R6585: - C29 + 1e+06 alpha_ob17_seg14[1] <= 1.000034894e+06
 R6586: - C31 + 1e+06 alpha_ob17_seg14[1] <= 1.000034894e+06
 R6587: C28 + 1e+06 alpha_ob17_seg14[2] <= 999981.162491
 R6588: C30 + 1e+06 alpha_ob17_seg14[2] <= 999981.162491
 R6589: C29 + 1e+06 alpha_ob17_seg14[3] <= 999962.556
 R6590: C31 + 1e+06 alpha_ob17_seg14[3] <= 999962.556
 R6591: alpha_ob17_seg14[0] + alpha_ob17_seg14[1] + alpha_ob17_seg14[2]
   + alpha_ob17_seg14[3] >= 1
 R6592: - C28 + 1e+06 alpha_ob18_seg14[0] <= 999990.2670277582
 R6593: - C30 + 1e+06 alpha_ob18_seg14[0] <= 999990.2670277582
 R6594: - C29 + 1e+06 alpha_ob18_seg14[1] <= 1.0000347702002756e+06
 R6595: - C31 + 1e+06 alpha_ob18_seg14[1] <= 1.0000347702002756e+06
 R6596: C28 + 1e+06 alpha_ob18_seg14[2] <= 1.0000071828877581e+06
 R6597: C30 + 1e+06 alpha_ob18_seg14[2] <= 1.0000071828877581e+06
 R6598: C29 + 1e+06 alpha_ob18_seg14[3] <= 999939.8298002756
 R6599: C31 + 1e+06 alpha_ob18_seg14[3] <= 999939.8298002756
 R6600: alpha_ob18_seg14[0] + alpha_ob18_seg14[1] + alpha_ob18_seg14[2]
   + alpha_ob18_seg14[3] >= 1
 R6601: - C28 + 1e+06 alpha_ob19_seg14[0] <= 999990.267069801
 R6602: - C30 + 1e+06 alpha_ob19_seg14[0] <= 999990.267069801
 R6603: - C29 + 1e+06 alpha_ob19_seg14[1] <= 1.0000576202069657e+06
 R6604: - C31 + 1e+06 alpha_ob19_seg14[1] <= 1.0000576202069657e+06
 R6605: C28 + 1e+06 alpha_ob19_seg14[2] <= 1.000002082929801e+06
 R6606: C30 + 1e+06 alpha_ob19_seg14[2] <= 1.000002082929801e+06
 R6607: C29 + 1e+06 alpha_ob19_seg14[3] <= 999939.8298069657
 R6608: C31 + 1e+06 alpha_ob19_seg14[3] <= 999939.8298069657
 R6609: alpha_ob19_seg14[0] + alpha_ob19_seg14[1] + alpha_ob19_seg14[2]
   + alpha_ob19_seg14[3] >= 1
 R6610: - C28 + 1e+06 alpha_ob20_seg14[0] <= 999991.073732
 R6611: - C30 + 1e+06 alpha_ob20_seg14[0] <= 999991.073732
 R6612: - C29 + 1e+06 alpha_ob20_seg14[1] <= 999952.2125
 R6613: - C31 + 1e+06 alpha_ob20_seg14[1] <= 999952.2125
 R6614: C28 + 1e+06 alpha_ob20_seg14[2] <= 999990.776268
 R6615: C30 + 1e+06 alpha_ob20_seg14[2] <= 999990.776268
 R6616: C29 + 1e+06 alpha_ob20_seg14[3] <= 1.0000452375e+06
 R6617: C31 + 1e+06 alpha_ob20_seg14[3] <= 1.0000452375e+06
 R6618: alpha_ob20_seg14[0] + alpha_ob20_seg14[1] + alpha_ob20_seg14[2]
   + alpha_ob20_seg14[3] >= 1
 R6619: - C28 + 1e+06 alpha_ob21_seg14[0] <= 999985.657099801
 R6620: - C30 + 1e+06 alpha_ob21_seg14[0] <= 999985.657099801
 R6621: - C29 + 1e+06 alpha_ob21_seg14[1] <= 1.0000576202063023e+06
 R6622: - C31 + 1e+06 alpha_ob21_seg14[1] <= 1.0000576202063023e+06
 R6623: C28 + 1e+06 alpha_ob21_seg14[2] <= 1.000007192899801e+06
 R6624: C30 + 1e+06 alpha_ob21_seg14[2] <= 1.000007192899801e+06
 R6625: C29 + 1e+06 alpha_ob21_seg14[3] <= 999939.8298063023
 R6626: C31 + 1e+06 alpha_ob21_seg14[3] <= 999939.8298063023
 R6627: alpha_ob21_seg14[0] + alpha_ob21_seg14[1] + alpha_ob21_seg14[2]
   + alpha_ob21_seg14[3] >= 1
 R6628: - C28 + 1e+06 alpha_ob22_seg14[0] <= 999994.771829801
 R6629: - C30 + 1e+06 alpha_ob22_seg14[0] <= 999994.771829801
 R6630: - C29 + 1e+06 alpha_ob22_seg14[1] <= 1.0000615983063023e+06
 R6631: - C31 + 1e+06 alpha_ob22_seg14[1] <= 1.0000615983063023e+06
 R6632: C28 + 1e+06 alpha_ob22_seg14[2] <= 999998.078169801
 R6633: C30 + 1e+06 alpha_ob22_seg14[2] <= 999998.078169801
 R6634: C29 + 1e+06 alpha_ob22_seg14[3] <= 999935.8517063023
 R6635: C31 + 1e+06 alpha_ob22_seg14[3] <= 999935.8517063023
 R6636: alpha_ob22_seg14[0] + alpha_ob22_seg14[1] + alpha_ob22_seg14[2]
   + alpha_ob22_seg14[3] >= 1
 R6637: - C28 + 1e+06 alpha_ob23_seg14[0] <= 999980.9374508709
 R6638: - C30 + 1e+06 alpha_ob23_seg14[0] <= 999980.9374508709
 R6639: - C29 + 1e+06 alpha_ob23_seg14[1] <= 1.0000348940002755e+06
 R6640: - C31 + 1e+06 alpha_ob23_seg14[1] <= 1.0000348940002755e+06
 R6641: C28 + 1e+06 alpha_ob23_seg14[2] <= 1.0000165124508708e+06
 R6642: C30 + 1e+06 alpha_ob23_seg14[2] <= 1.0000165124508708e+06
 R6643: C29 + 1e+06 alpha_ob23_seg14[3] <= 999935.9560002756
 R6644: C31 + 1e+06 alpha_ob23_seg14[3] <= 999935.9560002756
 R6645: alpha_ob23_seg14[0] + alpha_ob23_seg14[1] + alpha_ob23_seg14[2]
   + alpha_ob23_seg14[3] >= 1
 R6646: - C28 + 1e+06 alpha_ob24_seg14[0] <= 999980.937499801
 R6647: - C30 + 1e+06 alpha_ob24_seg14[0] <= 999980.937499801
 R6648: - C29 + 1e+06 alpha_ob24_seg14[1] <= 1.0000614940072973e+06
 R6649: - C31 + 1e+06 alpha_ob24_seg14[1] <= 1.0000614940072973e+06
 R6650: C28 + 1e+06 alpha_ob24_seg14[2] <= 1.000011162499801e+06
 R6651: C30 + 1e+06 alpha_ob24_seg14[2] <= 1.000011162499801e+06
 R6652: C29 + 1e+06 alpha_ob24_seg14[3] <= 999935.9560072974
 R6653: C31 + 1e+06 alpha_ob24_seg14[3] <= 999935.9560072974
 R6654: alpha_ob24_seg14[0] + alpha_ob24_seg14[1] + alpha_ob24_seg14[2]
   + alpha_ob24_seg14[3] >= 1
 R6655: - C28 + 1e+06 alpha_ob25_seg14[0] <= 999956.8394000001
 R6656: - C30 + 1e+06 alpha_ob25_seg14[0] <= 999956.8394000001
 R6657: - C29 + 1e+06 alpha_ob25_seg14[1] <= 999952.2463
 R6658: - C31 + 1e+06 alpha_ob25_seg14[1] <= 999952.2463
 R6659: C28 + 1e+06 alpha_ob25_seg14[2] <= 1.0000097606e+06
 R6660: C30 + 1e+06 alpha_ob25_seg14[2] <= 1.0000097606e+06
 R6661: C29 + 1e+06 alpha_ob25_seg14[3] <= 1.0000452037e+06
 R6662: C31 + 1e+06 alpha_ob25_seg14[3] <= 1.0000452037e+06
 R6663: alpha_ob25_seg14[0] + alpha_ob25_seg14[1] + alpha_ob25_seg14[2]
   + alpha_ob25_seg14[3] >= 1
 R6664: - C28 + 1e+06 alpha_ob26_seg14[0] <= 999980.9375348955
 R6665: - C30 + 1e+06 alpha_ob26_seg14[0] <= 999980.9375348955
 R6666: - C29 + 1e+06 alpha_ob26_seg14[1] <= 1.0000160439997246e+06
 R6667: - C31 + 1e+06 alpha_ob26_seg14[1] <= 1.0000160439997246e+06
 R6668: C28 + 1e+06 alpha_ob26_seg14[2] <= 1.0000165125348954e+06
 R6669: C30 + 1e+06 alpha_ob26_seg14[2] <= 1.0000165125348954e+06
 R6670: C29 + 1e+06 alpha_ob26_seg14[3] <= 999962.5559997245
 R6671: C31 + 1e+06 alpha_ob26_seg14[3] <= 999962.5559997245
 R6672: alpha_ob26_seg14[0] + alpha_ob26_seg14[1] + alpha_ob26_seg14[2]
   + alpha_ob26_seg14[3] >= 1
 R6673: - C28 + 1e+06 alpha_ob27_seg14[0] <= 999956.839399801
 R6674: - C30 + 1e+06 alpha_ob27_seg14[0] <= 999956.839399801
 R6675: - C29 + 1e+06 alpha_ob27_seg14[1] <= 999925.3963434525
 R6676: - C31 + 1e+06 alpha_ob27_seg14[1] <= 999925.3963434525
 R6677: C28 + 1e+06 alpha_ob27_seg14[2] <= 1.0000080105998011e+06
 R6678: C30 + 1e+06 alpha_ob27_seg14[2] <= 1.0000080105998011e+06
 R6679: C29 + 1e+06 alpha_ob27_seg14[3] <= 1.0000720537434525e+06
 R6680: C31 + 1e+06 alpha_ob27_seg14[3] <= 1.0000720537434525e+06
 R6681: alpha_ob27_seg14[0] + alpha_ob27_seg14[1] + alpha_ob27_seg14[2]
   + alpha_ob27_seg14[3] >= 1
 R6682: - C28 + 1e+06 alpha_ob28_seg14[0] <= 999980.9374998009
 R6683: - C30 + 1e+06 alpha_ob28_seg14[0] <= 999980.9374998009
 R6684: - C29 + 1e+06 alpha_ob28_seg14[1] <= 1.0000160440053071e+06
 R6685: - C31 + 1e+06 alpha_ob28_seg14[1] <= 1.0000160440053071e+06
 R6686: C28 + 1e+06 alpha_ob28_seg14[2] <= 1.000012662499801e+06
 R6687: C30 + 1e+06 alpha_ob28_seg14[2] <= 1.000012662499801e+06
 R6688: C29 + 1e+06 alpha_ob28_seg14[3] <= 999981.4060053072
 R6689: C31 + 1e+06 alpha_ob28_seg14[3] <= 999981.4060053072
 R6690: alpha_ob28_seg14[0] + alpha_ob28_seg14[1] + alpha_ob28_seg14[2]
   + alpha_ob28_seg14[3] >= 1
 R6691: - C28 + 1e+06 alpha_ob29_seg14[0] <= 999983.6053459151
 R6692: - C30 + 1e+06 alpha_ob29_seg14[0] <= 999983.6053459151
 R6693: - C29 + 1e+06 alpha_ob29_seg14[1] <= 999987.7722497246
 R6694: - C31 + 1e+06 alpha_ob29_seg14[1] <= 999987.7722497246
 R6695: C28 + 1e+06 alpha_ob29_seg14[2] <= 1.000013844745915e+06
 R6696: C30 + 1e+06 alpha_ob29_seg14[2] <= 1.000013844745915e+06
 R6697: C29 + 1e+06 alpha_ob29_seg14[3] <= 999984.8277497246
 R6698: C31 + 1e+06 alpha_ob29_seg14[3] <= 999984.8277497246
 R6699: alpha_ob29_seg14[0] + alpha_ob29_seg14[1] + alpha_ob29_seg14[2]
   + alpha_ob29_seg14[3] >= 1
 R6700: - C28 + 1e+06 alpha_ob30_seg14[0] <= 999958.8717898987
 R6701: - C30 + 1e+06 alpha_ob30_seg14[0] <= 999958.8717898987
 R6702: - C29 + 1e+06 alpha_ob30_seg14[1] <= 999987.7837602755
 R6703: - C31 + 1e+06 alpha_ob30_seg14[1] <= 999987.7837602755
 R6704: C28 + 1e+06 alpha_ob30_seg14[2] <= 1.0000385781898986e+06
 R6705: C30 + 1e+06 alpha_ob30_seg14[2] <= 1.0000385781898986e+06
 R6706: C29 + 1e+06 alpha_ob30_seg14[3] <= 1.0000043162402755e+06
 R6707: C31 + 1e+06 alpha_ob30_seg14[3] <= 1.0000043162402755e+06
 R6708: alpha_ob30_seg14[0] + alpha_ob30_seg14[1] + alpha_ob30_seg14[2]
   + alpha_ob30_seg14[3] >= 1
 R6709: - C28 + 1e+06 alpha_ob31_seg14[0] <= 999939.2718
 R6710: - C30 + 1e+06 alpha_ob31_seg14[0] <= 999939.2718
 R6711: - C29 + 1e+06 alpha_ob31_seg14[1] <= 999993.13376
 R6712: - C31 + 1e+06 alpha_ob31_seg14[1] <= 999993.13376
 R6713: C28 + 1e+06 alpha_ob31_seg14[2] <= 1.0000385782e+06
 R6714: C30 + 1e+06 alpha_ob31_seg14[2] <= 1.0000385782e+06
 R6715: C29 + 1e+06 alpha_ob31_seg14[3] <= 1.00000431624e+06
 R6716: C31 + 1e+06 alpha_ob31_seg14[3] <= 1.00000431624e+06
 R6717: alpha_ob31_seg14[0] + alpha_ob31_seg14[1] + alpha_ob31_seg14[2]
   + alpha_ob31_seg14[3] >= 1
 R6718: - C28 + 1e+06 alpha_ob32_seg14[0] <= 999939.2718101013
 R6719: - C30 + 1e+06 alpha_ob32_seg14[0] <= 999939.2718101013
 R6720: - C29 + 1e+06 alpha_ob32_seg14[1] <= 999987.7837597246
 R6721: - C31 + 1e+06 alpha_ob32_seg14[1] <= 999987.7837597246
 R6722: C28 + 1e+06 alpha_ob32_seg14[2] <= 1.0000581782101013e+06
 R6723: C30 + 1e+06 alpha_ob32_seg14[2] <= 1.0000581782101013e+06
 R6724: C29 + 1e+06 alpha_ob32_seg14[3] <= 1.0000043162397245e+06
 R6725: C31 + 1e+06 alpha_ob32_seg14[3] <= 1.0000043162397245e+06
 R6726: alpha_ob32_seg14[0] + alpha_ob32_seg14[1] + alpha_ob32_seg14[2]
   + alpha_ob32_seg14[3] >= 1
 R6727: - C28 + 1e+06 alpha_ob33_seg14[0] <= 999939.271799801
 R6728: - C30 + 1e+06 alpha_ob33_seg14[0] <= 999939.271799801
 R6729: - C29 + 1e+06 alpha_ob33_seg14[1] <= 999987.7838262042
 R6730: - C31 + 1e+06 alpha_ob33_seg14[1] <= 999987.7838262042
 R6731: C28 + 1e+06 alpha_ob33_seg14[2] <= 1.000038578199801e+06
 R6732: C30 + 1e+06 alpha_ob33_seg14[2] <= 1.000038578199801e+06
 R6733: C29 + 1e+06 alpha_ob33_seg14[3] <= 1.0000096662262043e+06
 R6734: C31 + 1e+06 alpha_ob33_seg14[3] <= 1.0000096662262043e+06
 R6735: alpha_ob33_seg14[0] + alpha_ob33_seg14[1] + alpha_ob33_seg14[2]
   + alpha_ob33_seg14[3] >= 1
 R6736: - C28 + 1e+06 alpha_ob34_seg14[0] <= 999926.0708
 R6737: - C30 + 1e+06 alpha_ob34_seg14[0] <= 999926.0708
 R6738: - C29 + 1e+06 alpha_ob34_seg14[1] <= 999983.7196
 R6739: - C31 + 1e+06 alpha_ob34_seg14[1] <= 999983.7196
 R6740: C28 + 1e+06 alpha_ob34_seg14[2] <= 1.0000250292e+06
 R6741: C30 + 1e+06 alpha_ob34_seg14[2] <= 1.0000250292e+06
 R6742: C29 + 1e+06 alpha_ob34_seg14[3] <= 1.0000137304e+06
 R6743: C31 + 1e+06 alpha_ob34_seg14[3] <= 1.0000137304e+06
 R6744: alpha_ob34_seg14[0] + alpha_ob34_seg14[1] + alpha_ob34_seg14[2]
   + alpha_ob34_seg14[3] >= 1
 R6745: - C28 + 1e+06 alpha_ob35_seg14[0] <= 999926.0708293857
 R6746: - C30 + 1e+06 alpha_ob35_seg14[0] <= 999926.0708293857
 R6747: - C29 + 1e+06 alpha_ob35_seg14[1] <= 999967.8695997245
 R6748: - C31 + 1e+06 alpha_ob35_seg14[1] <= 999967.8695997245
 R6749: C28 + 1e+06 alpha_ob35_seg14[2] <= 1.0000713792293856e+06
 R6750: C30 + 1e+06 alpha_ob35_seg14[2] <= 1.0000713792293856e+06
 R6751: C29 + 1e+06 alpha_ob35_seg14[3] <= 1.0000137303997246e+06
 R6752: C31 + 1e+06 alpha_ob35_seg14[3] <= 1.0000137303997246e+06
 R6753: alpha_ob35_seg14[0] + alpha_ob35_seg14[1] + alpha_ob35_seg14[2]
   + alpha_ob35_seg14[3] >= 1
 R6754: - C28 + 1e+06 alpha_ob36_seg14[0] <= 999972.6917508709
 R6755: - C30 + 1e+06 alpha_ob36_seg14[0] <= 999972.6917508709
 R6756: - C29 + 1e+06 alpha_ob36_seg14[1] <= 999957.1644002756
 R6757: - C31 + 1e+06 alpha_ob36_seg14[1] <= 999957.1644002756
 R6758: C28 + 1e+06 alpha_ob36_seg14[2] <= 1.0000247581508709e+06
 R6759: C30 + 1e+06 alpha_ob36_seg14[2] <= 1.0000247581508709e+06
 R6760: C29 + 1e+06 alpha_ob36_seg14[3] <= 1.0000136856002755e+06
 R6761: C31 + 1e+06 alpha_ob36_seg14[3] <= 1.0000136856002755e+06
 R6762: alpha_ob36_seg14[0] + alpha_ob36_seg14[1] + alpha_ob36_seg14[2]
   + alpha_ob36_seg14[3] >= 1
 R6763: - C28 + 1e+06 alpha_ob37_seg14[0] <= 999939.0009674003
 R6764: - C30 + 1e+06 alpha_ob37_seg14[0] <= 999939.0009674003
 R6765: - C29 + 1e+06 alpha_ob37_seg14[1] <= 999961.9516002756
 R6766: - C31 + 1e+06 alpha_ob37_seg14[1] <= 999961.9516002756
 R6767: C28 + 1e+06 alpha_ob37_seg14[2] <= 1.0000584489674004e+06
 R6768: C30 + 1e+06 alpha_ob37_seg14[2] <= 1.0000584489674004e+06
 R6769: C29 + 1e+06 alpha_ob37_seg14[3] <= 1.0000178984002755e+06
 R6770: C31 + 1e+06 alpha_ob37_seg14[3] <= 1.0000178984002755e+06
 R6771: alpha_ob37_seg14[0] + alpha_ob37_seg14[1] + alpha_ob37_seg14[2]
   + alpha_ob37_seg14[3] >= 1
 R6772: - C28 + 1e+06 alpha_ob38_seg14[0] <= 999994.9164998011
 R6773: - C30 + 1e+06 alpha_ob38_seg14[0] <= 999994.9164998011
 R6774: - C29 + 1e+06 alpha_ob38_seg14[1] <= 999925.4284782809
 R6775: - C31 + 1e+06 alpha_ob38_seg14[1] <= 999925.4284782809
 R6776: C28 + 1e+06 alpha_ob38_seg14[2] <= 999943.6834998011
 R6777: C30 + 1e+06 alpha_ob38_seg14[2] <= 999943.6834998011
 R6778: C29 + 1e+06 alpha_ob38_seg14[3] <= 1.0000720216782809e+06
 R6779: C31 + 1e+06 alpha_ob38_seg14[3] <= 1.0000720216782809e+06
 R6780: alpha_ob38_seg14[0] + alpha_ob38_seg14[1] + alpha_ob38_seg14[2]
   + alpha_ob38_seg14[3] >= 1
 R6781: - C28 + 1e+06 alpha_ob39_seg14[0] <= 999914.52327842
 R6782: - C30 + 1e+06 alpha_ob39_seg14[0] <= 999914.52327842
 R6783: - C29 + 1e+06 alpha_ob39_seg14[1] <= 999993.6552362755
 R6784: - C31 + 1e+06 alpha_ob39_seg14[1] <= 999993.6552362755
 R6785: C28 + 1e+06 alpha_ob39_seg14[2] <= 1.0000829266784199e+06
 R6786: C30 + 1e+06 alpha_ob39_seg14[2] <= 1.0000829266784199e+06
 R6787: C29 + 1e+06 alpha_ob39_seg14[3] <= 999992.1947642756
 R6788: C31 + 1e+06 alpha_ob39_seg14[3] <= 999992.1947642756
 R6789: alpha_ob39_seg14[0] + alpha_ob39_seg14[1] + alpha_ob39_seg14[2]
   + alpha_ob39_seg14[3] >= 1
 R6790: - C28 + 1e+06 alpha_ob40_seg14[0] <= 999912.265199801
 R6791: - C30 + 1e+06 alpha_ob40_seg14[0] <= 999912.265199801
 R6792: - C29 + 1e+06 alpha_ob40_seg14[1] <= 999987.7842112777
 R6793: - C31 + 1e+06 alpha_ob40_seg14[1] <= 999987.7842112777
 R6794: C28 + 1e+06 alpha_ob40_seg14[2] <= 1.000076834799801e+06
 R6795: C30 + 1e+06 alpha_ob40_seg14[2] <= 1.000076834799801e+06
 R6796: C29 + 1e+06 alpha_ob40_seg14[3] <= 1.0000096658112778e+06
 R6797: C31 + 1e+06 alpha_ob40_seg14[3] <= 1.0000096658112778e+06
 R6798: alpha_ob40_seg14[0] + alpha_ob40_seg14[1] + alpha_ob40_seg14[2]
   + alpha_ob40_seg14[3] >= 1
 R6799: - C28 + 1e+06 alpha_ob41_seg14[0] <= 999920.6152119379
 R6800: - C30 + 1e+06 alpha_ob41_seg14[0] <= 999920.6152119379
 R6801: - C29 + 1e+06 alpha_ob41_seg14[1] <= 999981.4341997246
 R6802: - C31 + 1e+06 alpha_ob41_seg14[1] <= 999981.4341997246
 R6803: C28 + 1e+06 alpha_ob41_seg14[2] <= 1.0000768348119379e+06
 R6804: C30 + 1e+06 alpha_ob41_seg14[2] <= 1.0000768348119379e+06
 R6805: C29 + 1e+06 alpha_ob41_seg14[3] <= 1.0000096657997245e+06
 R6806: C31 + 1e+06 alpha_ob41_seg14[3] <= 1.0000096657997245e+06
 R6807: alpha_ob41_seg14[0] + alpha_ob41_seg14[1] + alpha_ob41_seg14[2]
   + alpha_ob41_seg14[3] >= 1
 R6808: - C28 + 1e+06 alpha_ob42_seg14[0] <= 999919.2652
 R6809: - C30 + 1e+06 alpha_ob42_seg14[0] <= 999919.2652
 R6810: - C29 + 1e+06 alpha_ob42_seg14[1] <= 999981.4342
 R6811: - C31 + 1e+06 alpha_ob42_seg14[1] <= 999981.4342
 R6812: C28 + 1e+06 alpha_ob42_seg14[2] <= 1.0000768348e+06
 R6813: C30 + 1e+06 alpha_ob42_seg14[2] <= 1.0000768348e+06
 R6814: C29 + 1e+06 alpha_ob42_seg14[3] <= 1.0000160158000001e+06
 R6815: C31 + 1e+06 alpha_ob42_seg14[3] <= 1.0000160158000001e+06
 R6816: alpha_ob42_seg14[0] + alpha_ob42_seg14[1] + alpha_ob42_seg14[2]
   + alpha_ob42_seg14[3] >= 1
 R6817: - C28 + 1e+06 alpha_ob43_seg14[0] <= 999919.2652454559
 R6818: - C30 + 1e+06 alpha_ob43_seg14[0] <= 999919.2652454559
 R6819: - C29 + 1e+06 alpha_ob43_seg14[1] <= 999956.8341997246
 R6820: - C31 + 1e+06 alpha_ob43_seg14[1] <= 999956.8341997246
 R6821: C28 + 1e+06 alpha_ob43_seg14[2] <= 1.0000781848454559e+06
 R6822: C30 + 1e+06 alpha_ob43_seg14[2] <= 1.0000781848454559e+06
 R6823: C29 + 1e+06 alpha_ob43_seg14[3] <= 1.0000160157997246e+06
 R6824: C31 + 1e+06 alpha_ob43_seg14[3] <= 1.0000160157997246e+06
 R6825: alpha_ob43_seg14[0] + alpha_ob43_seg14[1] + alpha_ob43_seg14[2]
   + alpha_ob43_seg14[3] >= 1
 R6826: - C28 + 1e+06 alpha_ob44_seg14[0] <= 999991.83683
 R6827: - C30 + 1e+06 alpha_ob44_seg14[0] <= 999991.83683
 R6828: - C29 + 1e+06 alpha_ob44_seg14[1] <= 999940.589
 R6829: - C31 + 1e+06 alpha_ob44_seg14[1] <= 999940.589
 R6830: C28 + 1e+06 alpha_ob44_seg14[2] <= 1.00000401317e+06
 R6831: C30 + 1e+06 alpha_ob44_seg14[2] <= 1.00000401317e+06
 R6832: C29 + 1e+06 alpha_ob44_seg14[3] <= 1.000056861e+06
 R6833: C31 + 1e+06 alpha_ob44_seg14[3] <= 1.000056861e+06
 R6834: alpha_ob44_seg14[0] + alpha_ob44_seg14[1] + alpha_ob44_seg14[2]
   + alpha_ob44_seg14[3] >= 1
 R6835: - C28 + 1e+06 alpha_ob45_seg14[0] <= 999991.8368084199
 R6836: - C30 + 1e+06 alpha_ob45_seg14[0] <= 999991.8368084199
 R6837: - C29 + 1e+06 alpha_ob45_seg14[1] <= 999940.5890002755
 R6838: - C31 + 1e+06 alpha_ob45_seg14[1] <= 999940.5890002755
 R6839: C28 + 1e+06 alpha_ob45_seg14[2] <= 1.0000056131484199e+06
 R6840: C30 + 1e+06 alpha_ob45_seg14[2] <= 1.0000056131484199e+06
 R6841: C29 + 1e+06 alpha_ob45_seg14[3] <= 1.0000452610002755e+06
 R6842: C31 + 1e+06 alpha_ob45_seg14[3] <= 1.0000452610002755e+06
 R6843: alpha_ob45_seg14[0] + alpha_ob45_seg14[1] + alpha_ob45_seg14[2]
   + alpha_ob45_seg14[3] >= 1
 R6844: - C28 + 1e+06 alpha_ob46_seg14[0] <= 1.0000764021508709e+06
 R6845: - C30 + 1e+06 alpha_ob46_seg14[0] <= 1.0000764021508709e+06
 R6846: - C29 + 1e+06 alpha_ob46_seg14[1] <= 999957.1027002756
 R6847: - C31 + 1e+06 alpha_ob46_seg14[1] <= 999957.1027002756
 R6848: C28 + 1e+06 alpha_ob46_seg14[2] <= 999921.0477508709
 R6849: C30 + 1e+06 alpha_ob46_seg14[2] <= 999921.0477508709
 R6850: C29 + 1e+06 alpha_ob46_seg14[3] <= 1.0000137473002756e+06
 R6851: C31 + 1e+06 alpha_ob46_seg14[3] <= 1.0000137473002756e+06
 R6852: alpha_ob46_seg14[0] + alpha_ob46_seg14[1] + alpha_ob46_seg14[2]
   + alpha_ob46_seg14[3] >= 1
 R6853: - C28 + 1e+06 alpha_ob47_seg14[0] <= 1.0000445522e+06
 R6854: - C30 + 1e+06 alpha_ob47_seg14[0] <= 1.0000445522e+06
 R6855: - C29 + 1e+06 alpha_ob47_seg14[1] <= 999983.7027
 R6856: - C31 + 1e+06 alpha_ob47_seg14[1] <= 999983.7027
 R6857: C28 + 1e+06 alpha_ob47_seg14[2] <= 999921.0477999999
 R6858: C30 + 1e+06 alpha_ob47_seg14[2] <= 999921.0477999999
 R6859: C29 + 1e+06 alpha_ob47_seg14[3] <= 1.0000137473e+06
 R6860: C31 + 1e+06 alpha_ob47_seg14[3] <= 1.0000137473e+06
 R6861: alpha_ob47_seg14[0] + alpha_ob47_seg14[1] + alpha_ob47_seg14[2]
   + alpha_ob47_seg14[3] >= 1
 R6862: - C28 + 1e+06 alpha_ob48_seg14[0] <= 1.0000445522183661e+06
 R6863: - C30 + 1e+06 alpha_ob48_seg14[0] <= 1.0000445522183661e+06
 R6864: - C29 + 1e+06 alpha_ob48_seg14[1] <= 999973.8526997245
 R6865: - C31 + 1e+06 alpha_ob48_seg14[1] <= 999973.8526997245
 R6866: C28 + 1e+06 alpha_ob48_seg14[2] <= 999952.8978183661
 R6867: C30 + 1e+06 alpha_ob48_seg14[2] <= 999952.8978183661
 R6868: C29 + 1e+06 alpha_ob48_seg14[3] <= 1.0000137472997245e+06
 R6869: C31 + 1e+06 alpha_ob48_seg14[3] <= 1.0000137472997245e+06
 R6870: alpha_ob48_seg14[0] + alpha_ob48_seg14[1] + alpha_ob48_seg14[2]
   + alpha_ob48_seg14[3] >= 1
 R6871: - C28 + 1e+06 alpha_ob49_seg14[0] <= 1.000064332081634e+06
 R6872: - C30 + 1e+06 alpha_ob49_seg14[0] <= 1.000064332081634e+06
 R6873: - C29 + 1e+06 alpha_ob49_seg14[1] <= 999973.9401002756
 R6874: - C31 + 1e+06 alpha_ob49_seg14[1] <= 999973.9401002756
 R6875: C28 + 1e+06 alpha_ob49_seg14[2] <= 999933.1178816339
 R6876: C30 + 1e+06 alpha_ob49_seg14[2] <= 999933.1178816339
 R6877: C29 + 1e+06 alpha_ob49_seg14[3] <= 1.0000136599002755e+06
 R6878: C31 + 1e+06 alpha_ob49_seg14[3] <= 1.0000136599002755e+06
 R6879: alpha_ob49_seg14[0] + alpha_ob49_seg14[1] + alpha_ob49_seg14[2]
   + alpha_ob49_seg14[3] >= 1
 R6880: - C28 + 1e+06 alpha_ob50_seg14[0] <= 1.0000643321013774e+06
 R6881: - C30 + 1e+06 alpha_ob50_seg14[0] <= 1.0000643321013774e+06
 R6882: - C29 + 1e+06 alpha_ob50_seg14[1] <= 999983.1900997246
 R6883: - C31 + 1e+06 alpha_ob50_seg14[1] <= 999983.1900997246
 R6884: C28 + 1e+06 alpha_ob50_seg14[2] <= 999933.1179013775
 R6885: C30 + 1e+06 alpha_ob50_seg14[2] <= 999933.1179013775
 R6886: C29 + 1e+06 alpha_ob50_seg14[3] <= 1.0000136598997245e+06
 R6887: C31 + 1e+06 alpha_ob50_seg14[3] <= 1.0000136598997245e+06
 R6888: alpha_ob50_seg14[0] + alpha_ob50_seg14[1] + alpha_ob50_seg14[2]
   + alpha_ob50_seg14[3] >= 1
 R6889: - C30 + 1e+06 alpha_ob0_seg15[0] <= 1.0000284170499525e+06
 R6890: - C32 + 1e+06 alpha_ob0_seg15[0] <= 1.0000284170499525e+06
 R6891: - C31 + 1e+06 alpha_ob0_seg15[1] <= 999957.0843002755
 R6892: - C33 + 1e+06 alpha_ob0_seg15[1] <= 999957.0843002755
 R6893: C30 + 1e+06 alpha_ob0_seg15[2] <= 999969.0328499526
 R6894: C32 + 1e+06 alpha_ob0_seg15[2] <= 999969.0328499526
 R6895: C31 + 1e+06 alpha_ob0_seg15[3] <= 1.0000132657002755e+06
 R6896: C33 + 1e+06 alpha_ob0_seg15[3] <= 1.0000132657002755e+06
 R6897: alpha_ob0_seg15[0] + alpha_ob0_seg15[1] + alpha_ob0_seg15[2]
   + alpha_ob0_seg15[3] >= 1
 R6898: - C30 + 1e+06 alpha_ob1_seg15[0] <= 1.000028417099801e+06
 R6899: - C32 + 1e+06 alpha_ob1_seg15[0] <= 1.000028417099801e+06
 R6900: - C31 + 1e+06 alpha_ob1_seg15[1] <= 999984.1843155898
 R6901: - C33 + 1e+06 alpha_ob1_seg15[1] <= 999984.1843155898
 R6902: C30 + 1e+06 alpha_ob1_seg15[2] <= 999957.4328998009
 R6903: C32 + 1e+06 alpha_ob1_seg15[2] <= 999957.4328998009
 R6904: C31 + 1e+06 alpha_ob1_seg15[3] <= 1.0000132657155899e+06
 R6905: C33 + 1e+06 alpha_ob1_seg15[3] <= 1.0000132657155899e+06
 R6906: alpha_ob1_seg15[0] + alpha_ob1_seg15[1] + alpha_ob1_seg15[2]
   + alpha_ob1_seg15[3] >= 1
 R6907: - C30 + 1e+06 alpha_ob2_seg15[0] <= 1.0000400171500475e+06
 R6908: - C32 + 1e+06 alpha_ob2_seg15[0] <= 1.0000400171500475e+06
 R6909: - C31 + 1e+06 alpha_ob2_seg15[1] <= 999957.0842997245
 R6910: - C33 + 1e+06 alpha_ob2_seg15[1] <= 999957.0842997245
 R6911: C30 + 1e+06 alpha_ob2_seg15[2] <= 999957.4329500474
 R6912: C32 + 1e+06 alpha_ob2_seg15[2] <= 999957.4329500474
 R6913: C31 + 1e+06 alpha_ob2_seg15[3] <= 1.0000132656997246e+06
 R6914: C33 + 1e+06 alpha_ob2_seg15[3] <= 1.0000132656997246e+06
 R6915: alpha_ob2_seg15[0] + alpha_ob2_seg15[1] + alpha_ob2_seg15[2]
   + alpha_ob2_seg15[3] >= 1
 R6916: - C30 + 1e+06 alpha_ob3_seg15[0] <= 1.0000109161752058e+06
 R6917: - C32 + 1e+06 alpha_ob3_seg15[0] <= 1.0000109161752058e+06
 R6918: - C31 + 1e+06 alpha_ob3_seg15[1] <= 999970.2681002755
 R6919: - C33 + 1e+06 alpha_ob3_seg15[1] <= 999970.2681002755
 R6920: C30 + 1e+06 alpha_ob3_seg15[2] <= 999986.5337752058
 R6921: C32 + 1e+06 alpha_ob3_seg15[2] <= 999986.5337752058
 R6922: C31 + 1e+06 alpha_ob3_seg15[3] <= 1.0000138319002755e+06
 R6923: C33 + 1e+06 alpha_ob3_seg15[3] <= 1.0000138319002755e+06
 R6924: alpha_ob3_seg15[0] + alpha_ob3_seg15[1] + alpha_ob3_seg15[2]
   + alpha_ob3_seg15[3] >= 1
 R6925: - C30 + 1e+06 alpha_ob4_seg15[0] <= 1.000010916199801e+06
 R6926: - C32 + 1e+06 alpha_ob4_seg15[0] <= 1.000010916199801e+06
 R6927: - C31 + 1e+06 alpha_ob4_seg15[1] <= 999983.6181179117
 R6928: - C33 + 1e+06 alpha_ob4_seg15[1] <= 999983.6181179117
 R6929: C30 + 1e+06 alpha_ob4_seg15[2] <= 999973.183799801
 R6930: C32 + 1e+06 alpha_ob4_seg15[2] <= 999973.183799801
 R6931: C31 + 1e+06 alpha_ob4_seg15[3] <= 1.0000138319179118e+06
 R6932: C33 + 1e+06 alpha_ob4_seg15[3] <= 1.0000138319179118e+06
 R6933: alpha_ob4_seg15[0] + alpha_ob4_seg15[1] + alpha_ob4_seg15[2]
   + alpha_ob4_seg15[3] >= 1
 R6934: - C30 + 1e+06 alpha_ob5_seg15[0] <= 999992.49102
 R6935: - C32 + 1e+06 alpha_ob5_seg15[0] <= 999992.49102
 R6936: - C31 + 1e+06 alpha_ob5_seg15[1] <= 999957.1016000001
 R6937: - C33 + 1e+06 alpha_ob5_seg15[1] <= 999957.1016000001
 R6938: C30 + 1e+06 alpha_ob5_seg15[2] <= 999986.35898
 R6939: C32 + 1e+06 alpha_ob5_seg15[2] <= 999986.35898
 R6940: C31 + 1e+06 alpha_ob5_seg15[3] <= 1.0000403484e+06
 R6941: C33 + 1e+06 alpha_ob5_seg15[3] <= 1.0000403484e+06
 R6942: alpha_ob5_seg15[0] + alpha_ob5_seg15[1] + alpha_ob5_seg15[2]
   + alpha_ob5_seg15[3] >= 1
 R6943: - C30 + 1e+06 alpha_ob6_seg15[0] <= 999981.3909999999
 R6944: - C32 + 1e+06 alpha_ob6_seg15[0] <= 999981.3909999999
 R6945: - C31 + 1e+06 alpha_ob6_seg15[1] <= 999957.1016000001
 R6946: - C33 + 1e+06 alpha_ob6_seg15[1] <= 999957.1016000001
 R6947: C30 + 1e+06 alpha_ob6_seg15[2] <= 1.000004959e+06
 R6948: C32 + 1e+06 alpha_ob6_seg15[2] <= 1.000004959e+06
 R6949: C31 + 1e+06 alpha_ob6_seg15[3] <= 1.0000403484e+06
 R6950: C33 + 1e+06 alpha_ob6_seg15[3] <= 1.0000403484e+06
 R6951: alpha_ob6_seg15[0] + alpha_ob6_seg15[1] + alpha_ob6_seg15[2]
   + alpha_ob6_seg15[3] >= 1
 R6952: - C30 + 1e+06 alpha_ob7_seg15[0] <= 999981.3909825523
 R6953: - C32 + 1e+06 alpha_ob7_seg15[0] <= 999981.3909825523
 R6954: - C31 + 1e+06 alpha_ob7_seg15[1] <= 999957.1016002755
 R6955: - C33 + 1e+06 alpha_ob7_seg15[1] <= 999957.1016002755
 R6956: C30 + 1e+06 alpha_ob7_seg15[2] <= 1.0000160589825523e+06
 R6957: C32 + 1e+06 alpha_ob7_seg15[2] <= 1.0000160589825523e+06
 R6958: C31 + 1e+06 alpha_ob7_seg15[3] <= 1.0000309984002755e+06
 R6959: C33 + 1e+06 alpha_ob7_seg15[3] <= 1.0000309984002755e+06
 R6960: alpha_ob7_seg15[0] + alpha_ob7_seg15[1] + alpha_ob7_seg15[2]
   + alpha_ob7_seg15[3] >= 1
 R6961: - C30 + 1e+06 alpha_ob8_seg15[0] <= 999981.3867954084
 R6962: - C32 + 1e+06 alpha_ob8_seg15[0] <= 999981.3867954084
 R6963: - C31 + 1e+06 alpha_ob8_seg15[1] <= 999968.8595002755
 R6964: - C33 + 1e+06 alpha_ob8_seg15[1] <= 999968.8595002755
 R6965: C30 + 1e+06 alpha_ob8_seg15[2] <= 1.0000160631954084e+06
 R6966: C32 + 1e+06 alpha_ob8_seg15[2] <= 1.0000160631954084e+06
 R6967: C31 + 1e+06 alpha_ob8_seg15[3] <= 1.0000262405002755e+06
 R6968: C33 + 1e+06 alpha_ob8_seg15[3] <= 1.0000262405002755e+06
 R6969: alpha_ob8_seg15[0] + alpha_ob8_seg15[1] + alpha_ob8_seg15[2]
   + alpha_ob8_seg15[3] >= 1
 R6970: - C30 + 1e+06 alpha_ob9_seg15[0] <= 999981.386799801
 R6971: - C32 + 1e+06 alpha_ob9_seg15[0] <= 999981.386799801
 R6972: - C31 + 1e+06 alpha_ob9_seg15[1] <= 999971.2095013268
 R6973: - C33 + 1e+06 alpha_ob9_seg15[1] <= 999971.2095013268
 R6974: C30 + 1e+06 alpha_ob9_seg15[2] <= 1.0000152131998009e+06
 R6975: C32 + 1e+06 alpha_ob9_seg15[2] <= 1.0000152131998009e+06
 R6976: C31 + 1e+06 alpha_ob9_seg15[3] <= 1.0000262405013267e+06
 R6977: C33 + 1e+06 alpha_ob9_seg15[3] <= 1.0000262405013267e+06
 R6978: alpha_ob9_seg15[0] + alpha_ob9_seg15[1] + alpha_ob9_seg15[2]
   + alpha_ob9_seg15[3] >= 1
 R6979: - C30 + 1e+06 alpha_ob10_seg15[0] <= 999982.2368022958
 R6980: - C32 + 1e+06 alpha_ob10_seg15[0] <= 999982.2368022958
 R6981: - C31 + 1e+06 alpha_ob10_seg15[1] <= 999970.1094997245
 R6982: - C33 + 1e+06 alpha_ob10_seg15[1] <= 999970.1094997245
 R6983: C30 + 1e+06 alpha_ob10_seg15[2] <= 1.0000152132022957e+06
 R6984: C32 + 1e+06 alpha_ob10_seg15[2] <= 1.0000152132022957e+06
 R6985: C31 + 1e+06 alpha_ob10_seg15[3] <= 1.0000262404997245e+06
 R6986: C33 + 1e+06 alpha_ob10_seg15[3] <= 1.0000262404997245e+06
 R6987: alpha_ob10_seg15[0] + alpha_ob10_seg15[1] + alpha_ob10_seg15[2]
   + alpha_ob10_seg15[3] >= 1
 R6988: - C30 + 1e+06 alpha_ob11_seg15[0] <= 999982.236799801
 R6989: - C32 + 1e+06 alpha_ob11_seg15[0] <= 999982.236799801
 R6990: - C31 + 1e+06 alpha_ob11_seg15[1] <= 999970.1095291895
 R6991: - C33 + 1e+06 alpha_ob11_seg15[1] <= 999970.1095291895
 R6992: C30 + 1e+06 alpha_ob11_seg15[2] <= 999993.3631998011
 R6993: C32 + 1e+06 alpha_ob11_seg15[2] <= 999993.3631998011
 R6994: C31 + 1e+06 alpha_ob11_seg15[3] <= 1.0000273405291894e+06
 R6995: C33 + 1e+06 alpha_ob11_seg15[3] <= 1.0000273405291894e+06
 R6996: alpha_ob11_seg15[0] + alpha_ob11_seg15[1] + alpha_ob11_seg15[2]
   + alpha_ob11_seg15[3] >= 1
 R6997: - C30 + 1e+06 alpha_ob12_seg15[0] <= 999981.6050802565
 R6998: - C32 + 1e+06 alpha_ob12_seg15[0] <= 999981.6050802565
 R6999: - C31 + 1e+06 alpha_ob12_seg15[1] <= 999972.8942002755
 R7000: - C33 + 1e+06 alpha_ob12_seg15[1] <= 999972.8942002755
 R7001: C30 + 1e+06 alpha_ob12_seg15[2] <= 1.0000158448802566e+06
 R7002: C32 + 1e+06 alpha_ob12_seg15[2] <= 1.0000158448802566e+06
 R7003: C31 + 1e+06 alpha_ob12_seg15[3] <= 1.0000139558002756e+06
 R7004: C33 + 1e+06 alpha_ob12_seg15[3] <= 1.0000139558002756e+06
 R7005: alpha_ob12_seg15[0] + alpha_ob12_seg15[1] + alpha_ob12_seg15[2]
   + alpha_ob12_seg15[3] >= 1
 R7006: - C30 + 1e+06 alpha_ob13_seg15[0] <= 999981.605149801
 R7007: - C32 + 1e+06 alpha_ob13_seg15[0] <= 999981.605149801
 R7008: - C31 + 1e+06 alpha_ob13_seg15[1] <= 999983.4942391404
 R7009: - C33 + 1e+06 alpha_ob13_seg15[1] <= 999983.4942391404
 R7010: C30 + 1e+06 alpha_ob13_seg15[2] <= 999986.4948498011
 R7011: C32 + 1e+06 alpha_ob13_seg15[2] <= 999986.4948498011
 R7012: C31 + 1e+06 alpha_ob13_seg15[3] <= 1.0000139558391405e+06
 R7013: C33 + 1e+06 alpha_ob13_seg15[3] <= 1.0000139558391405e+06
 R7014: alpha_ob13_seg15[0] + alpha_ob13_seg15[1] + alpha_ob13_seg15[2]
   + alpha_ob13_seg15[3] >= 1
 R7015: - C30 + 1e+06 alpha_ob14_seg15[0] <= 999997.6803502565
 R7016: - C32 + 1e+06 alpha_ob14_seg15[0] <= 999997.6803502565
 R7017: - C31 + 1e+06 alpha_ob14_seg15[1] <= 999972.7268002755
 R7018: - C33 + 1e+06 alpha_ob14_seg15[1] <= 999972.7268002755
 R7019: C30 + 1e+06 alpha_ob14_seg15[2] <= 999999.7696102565
 R7020: C32 + 1e+06 alpha_ob14_seg15[2] <= 999999.7696102565
 R7021: C31 + 1e+06 alpha_ob14_seg15[3] <= 1.0000141232002755e+06
 R7022: C33 + 1e+06 alpha_ob14_seg15[3] <= 1.0000141232002755e+06
 R7023: alpha_ob14_seg15[0] + alpha_ob14_seg15[1] + alpha_ob14_seg15[2]
   + alpha_ob14_seg15[3] >= 1
 R7024: - C30 + 1e+06 alpha_ob15_seg15[0] <= 1.0000038341502565e+06
 R7025: - C32 + 1e+06 alpha_ob15_seg15[0] <= 1.0000038341502565e+06
 R7026: - C31 + 1e+06 alpha_ob15_seg15[1] <= 999972.6012002755
 R7027: - C33 + 1e+06 alpha_ob15_seg15[1] <= 999972.6012002755
 R7028: C30 + 1e+06 alpha_ob15_seg15[2] <= 999993.6158102566
 R7029: C32 + 1e+06 alpha_ob15_seg15[2] <= 999993.6158102566
 R7030: C31 + 1e+06 alpha_ob15_seg15[3] <= 1.0000142488002755e+06
 R7031: C33 + 1e+06 alpha_ob15_seg15[3] <= 1.0000142488002755e+06
 R7032: alpha_ob15_seg15[0] + alpha_ob15_seg15[1] + alpha_ob15_seg15[2]
   + alpha_ob15_seg15[3] >= 1
 R7033: - C30 + 1e+06 alpha_ob16_seg15[0] <= 1.0000162874063333e+06
 R7034: - C32 + 1e+06 alpha_ob16_seg15[0] <= 1.0000162874063333e+06
 R7035: - C31 + 1e+06 alpha_ob16_seg15[1] <= 999982.9439602756
 R7036: - C33 + 1e+06 alpha_ob16_seg15[1] <= 999982.9439602756
 R7037: C30 + 1e+06 alpha_ob16_seg15[2] <= 999981.1624063333
 R7038: C32 + 1e+06 alpha_ob16_seg15[2] <= 999981.1624063333
 R7039: C31 + 1e+06 alpha_ob16_seg15[3] <= 999963.6560402757
 R7040: C33 + 1e+06 alpha_ob16_seg15[3] <= 999963.6560402757
 R7041: alpha_ob16_seg15[0] + alpha_ob16_seg15[1] + alpha_ob16_seg15[2]
   + alpha_ob16_seg15[3] >= 1
 R7042: - C30 + 1e+06 alpha_ob17_seg15[0] <= 999980.937509
 R7043: - C32 + 1e+06 alpha_ob17_seg15[0] <= 999980.937509
 R7044: - C31 + 1e+06 alpha_ob17_seg15[1] <= 1.000034894e+06
 R7045: - C33 + 1e+06 alpha_ob17_seg15[1] <= 1.000034894e+06
 R7046: C30 + 1e+06 alpha_ob17_seg15[2] <= 999981.162491
 R7047: C32 + 1e+06 alpha_ob17_seg15[2] <= 999981.162491
 R7048: C31 + 1e+06 alpha_ob17_seg15[3] <= 999962.556
 R7049: C33 + 1e+06 alpha_ob17_seg15[3] <= 999962.556
 R7050: alpha_ob17_seg15[0] + alpha_ob17_seg15[1] + alpha_ob17_seg15[2]
   + alpha_ob17_seg15[3] >= 1
 R7051: - C30 + 1e+06 alpha_ob18_seg15[0] <= 999990.2670277582
 R7052: - C32 + 1e+06 alpha_ob18_seg15[0] <= 999990.2670277582
 R7053: - C31 + 1e+06 alpha_ob18_seg15[1] <= 1.0000347702002756e+06
 R7054: - C33 + 1e+06 alpha_ob18_seg15[1] <= 1.0000347702002756e+06
 R7055: C30 + 1e+06 alpha_ob18_seg15[2] <= 1.0000071828877581e+06
 R7056: C32 + 1e+06 alpha_ob18_seg15[2] <= 1.0000071828877581e+06
 R7057: C31 + 1e+06 alpha_ob18_seg15[3] <= 999939.8298002756
 R7058: C33 + 1e+06 alpha_ob18_seg15[3] <= 999939.8298002756
 R7059: alpha_ob18_seg15[0] + alpha_ob18_seg15[1] + alpha_ob18_seg15[2]
   + alpha_ob18_seg15[3] >= 1
 R7060: - C30 + 1e+06 alpha_ob19_seg15[0] <= 999990.267069801
 R7061: - C32 + 1e+06 alpha_ob19_seg15[0] <= 999990.267069801
 R7062: - C31 + 1e+06 alpha_ob19_seg15[1] <= 1.0000576202069657e+06
 R7063: - C33 + 1e+06 alpha_ob19_seg15[1] <= 1.0000576202069657e+06
 R7064: C30 + 1e+06 alpha_ob19_seg15[2] <= 1.000002082929801e+06
 R7065: C32 + 1e+06 alpha_ob19_seg15[2] <= 1.000002082929801e+06
 R7066: C31 + 1e+06 alpha_ob19_seg15[3] <= 999939.8298069657
 R7067: C33 + 1e+06 alpha_ob19_seg15[3] <= 999939.8298069657
 R7068: alpha_ob19_seg15[0] + alpha_ob19_seg15[1] + alpha_ob19_seg15[2]
   + alpha_ob19_seg15[3] >= 1
 R7069: - C30 + 1e+06 alpha_ob20_seg15[0] <= 999991.073732
 R7070: - C32 + 1e+06 alpha_ob20_seg15[0] <= 999991.073732
 R7071: - C31 + 1e+06 alpha_ob20_seg15[1] <= 999952.2125
 R7072: - C33 + 1e+06 alpha_ob20_seg15[1] <= 999952.2125
 R7073: C30 + 1e+06 alpha_ob20_seg15[2] <= 999990.776268
 R7074: C32 + 1e+06 alpha_ob20_seg15[2] <= 999990.776268
 R7075: C31 + 1e+06 alpha_ob20_seg15[3] <= 1.0000452375e+06
 R7076: C33 + 1e+06 alpha_ob20_seg15[3] <= 1.0000452375e+06
 R7077: alpha_ob20_seg15[0] + alpha_ob20_seg15[1] + alpha_ob20_seg15[2]
   + alpha_ob20_seg15[3] >= 1
 R7078: - C30 + 1e+06 alpha_ob21_seg15[0] <= 999985.657099801
 R7079: - C32 + 1e+06 alpha_ob21_seg15[0] <= 999985.657099801
 R7080: - C31 + 1e+06 alpha_ob21_seg15[1] <= 1.0000576202063023e+06
 R7081: - C33 + 1e+06 alpha_ob21_seg15[1] <= 1.0000576202063023e+06
 R7082: C30 + 1e+06 alpha_ob21_seg15[2] <= 1.000007192899801e+06
 R7083: C32 + 1e+06 alpha_ob21_seg15[2] <= 1.000007192899801e+06
 R7084: C31 + 1e+06 alpha_ob21_seg15[3] <= 999939.8298063023
 R7085: C33 + 1e+06 alpha_ob21_seg15[3] <= 999939.8298063023
 R7086: alpha_ob21_seg15[0] + alpha_ob21_seg15[1] + alpha_ob21_seg15[2]
   + alpha_ob21_seg15[3] >= 1
 R7087: - C30 + 1e+06 alpha_ob22_seg15[0] <= 999994.771829801
 R7088: - C32 + 1e+06 alpha_ob22_seg15[0] <= 999994.771829801
 R7089: - C31 + 1e+06 alpha_ob22_seg15[1] <= 1.0000615983063023e+06
 R7090: - C33 + 1e+06 alpha_ob22_seg15[1] <= 1.0000615983063023e+06
 R7091: C30 + 1e+06 alpha_ob22_seg15[2] <= 999998.078169801
 R7092: C32 + 1e+06 alpha_ob22_seg15[2] <= 999998.078169801
 R7093: C31 + 1e+06 alpha_ob22_seg15[3] <= 999935.8517063023
 R7094: C33 + 1e+06 alpha_ob22_seg15[3] <= 999935.8517063023
 R7095: alpha_ob22_seg15[0] + alpha_ob22_seg15[1] + alpha_ob22_seg15[2]
   + alpha_ob22_seg15[3] >= 1
 R7096: - C30 + 1e+06 alpha_ob23_seg15[0] <= 999980.9374508709
 R7097: - C32 + 1e+06 alpha_ob23_seg15[0] <= 999980.9374508709
 R7098: - C31 + 1e+06 alpha_ob23_seg15[1] <= 1.0000348940002755e+06
 R7099: - C33 + 1e+06 alpha_ob23_seg15[1] <= 1.0000348940002755e+06
 R7100: C30 + 1e+06 alpha_ob23_seg15[2] <= 1.0000165124508708e+06
 R7101: C32 + 1e+06 alpha_ob23_seg15[2] <= 1.0000165124508708e+06
 R7102: C31 + 1e+06 alpha_ob23_seg15[3] <= 999935.9560002756
 R7103: C33 + 1e+06 alpha_ob23_seg15[3] <= 999935.9560002756
 R7104: alpha_ob23_seg15[0] + alpha_ob23_seg15[1] + alpha_ob23_seg15[2]
   + alpha_ob23_seg15[3] >= 1
 R7105: - C30 + 1e+06 alpha_ob24_seg15[0] <= 999980.937499801
 R7106: - C32 + 1e+06 alpha_ob24_seg15[0] <= 999980.937499801
 R7107: - C31 + 1e+06 alpha_ob24_seg15[1] <= 1.0000614940072973e+06
 R7108: - C33 + 1e+06 alpha_ob24_seg15[1] <= 1.0000614940072973e+06
 R7109: C30 + 1e+06 alpha_ob24_seg15[2] <= 1.000011162499801e+06
 R7110: C32 + 1e+06 alpha_ob24_seg15[2] <= 1.000011162499801e+06
 R7111: C31 + 1e+06 alpha_ob24_seg15[3] <= 999935.9560072974
 R7112: C33 + 1e+06 alpha_ob24_seg15[3] <= 999935.9560072974
 R7113: alpha_ob24_seg15[0] + alpha_ob24_seg15[1] + alpha_ob24_seg15[2]
   + alpha_ob24_seg15[3] >= 1
 R7114: - C30 + 1e+06 alpha_ob25_seg15[0] <= 999956.8394000001
 R7115: - C32 + 1e+06 alpha_ob25_seg15[0] <= 999956.8394000001
 R7116: - C31 + 1e+06 alpha_ob25_seg15[1] <= 999952.2463
 R7117: - C33 + 1e+06 alpha_ob25_seg15[1] <= 999952.2463
 R7118: C30 + 1e+06 alpha_ob25_seg15[2] <= 1.0000097606e+06
 R7119: C32 + 1e+06 alpha_ob25_seg15[2] <= 1.0000097606e+06
 R7120: C31 + 1e+06 alpha_ob25_seg15[3] <= 1.0000452037e+06
 R7121: C33 + 1e+06 alpha_ob25_seg15[3] <= 1.0000452037e+06
 R7122: alpha_ob25_seg15[0] + alpha_ob25_seg15[1] + alpha_ob25_seg15[2]
   + alpha_ob25_seg15[3] >= 1
 R7123: - C30 + 1e+06 alpha_ob26_seg15[0] <= 999980.9375348955
 R7124: - C32 + 1e+06 alpha_ob26_seg15[0] <= 999980.9375348955
 R7125: - C31 + 1e+06 alpha_ob26_seg15[1] <= 1.0000160439997246e+06
 R7126: - C33 + 1e+06 alpha_ob26_seg15[1] <= 1.0000160439997246e+06
 R7127: C30 + 1e+06 alpha_ob26_seg15[2] <= 1.0000165125348954e+06
 R7128: C32 + 1e+06 alpha_ob26_seg15[2] <= 1.0000165125348954e+06
 R7129: C31 + 1e+06 alpha_ob26_seg15[3] <= 999962.5559997245
 R7130: C33 + 1e+06 alpha_ob26_seg15[3] <= 999962.5559997245
 R7131: alpha_ob26_seg15[0] + alpha_ob26_seg15[1] + alpha_ob26_seg15[2]
   + alpha_ob26_seg15[3] >= 1
 R7132: - C30 + 1e+06 alpha_ob27_seg15[0] <= 999956.839399801
 R7133: - C32 + 1e+06 alpha_ob27_seg15[0] <= 999956.839399801
 R7134: - C31 + 1e+06 alpha_ob27_seg15[1] <= 999925.3963434525
 R7135: - C33 + 1e+06 alpha_ob27_seg15[1] <= 999925.3963434525
 R7136: C30 + 1e+06 alpha_ob27_seg15[2] <= 1.0000080105998011e+06
 R7137: C32 + 1e+06 alpha_ob27_seg15[2] <= 1.0000080105998011e+06
 R7138: C31 + 1e+06 alpha_ob27_seg15[3] <= 1.0000720537434525e+06
 R7139: C33 + 1e+06 alpha_ob27_seg15[3] <= 1.0000720537434525e+06
 R7140: alpha_ob27_seg15[0] + alpha_ob27_seg15[1] + alpha_ob27_seg15[2]
   + alpha_ob27_seg15[3] >= 1
 R7141: - C30 + 1e+06 alpha_ob28_seg15[0] <= 999980.9374998009
 R7142: - C32 + 1e+06 alpha_ob28_seg15[0] <= 999980.9374998009
 R7143: - C31 + 1e+06 alpha_ob28_seg15[1] <= 1.0000160440053071e+06
 R7144: - C33 + 1e+06 alpha_ob28_seg15[1] <= 1.0000160440053071e+06
 R7145: C30 + 1e+06 alpha_ob28_seg15[2] <= 1.000012662499801e+06
 R7146: C32 + 1e+06 alpha_ob28_seg15[2] <= 1.000012662499801e+06
 R7147: C31 + 1e+06 alpha_ob28_seg15[3] <= 999981.4060053072
 R7148: C33 + 1e+06 alpha_ob28_seg15[3] <= 999981.4060053072
 R7149: alpha_ob28_seg15[0] + alpha_ob28_seg15[1] + alpha_ob28_seg15[2]
   + alpha_ob28_seg15[3] >= 1
 R7150: - C30 + 1e+06 alpha_ob29_seg15[0] <= 999983.6053459151
 R7151: - C32 + 1e+06 alpha_ob29_seg15[0] <= 999983.6053459151
 R7152: - C31 + 1e+06 alpha_ob29_seg15[1] <= 999987.7722497246
 R7153: - C33 + 1e+06 alpha_ob29_seg15[1] <= 999987.7722497246
 R7154: C30 + 1e+06 alpha_ob29_seg15[2] <= 1.000013844745915e+06
 R7155: C32 + 1e+06 alpha_ob29_seg15[2] <= 1.000013844745915e+06
 R7156: C31 + 1e+06 alpha_ob29_seg15[3] <= 999984.8277497246
 R7157: C33 + 1e+06 alpha_ob29_seg15[3] <= 999984.8277497246
 R7158: alpha_ob29_seg15[0] + alpha_ob29_seg15[1] + alpha_ob29_seg15[2]
   + alpha_ob29_seg15[3] >= 1
 R7159: - C30 + 1e+06 alpha_ob30_seg15[0] <= 999958.8717898987
 R7160: - C32 + 1e+06 alpha_ob30_seg15[0] <= 999958.8717898987
 R7161: - C31 + 1e+06 alpha_ob30_seg15[1] <= 999987.7837602755
 R7162: - C33 + 1e+06 alpha_ob30_seg15[1] <= 999987.7837602755
 R7163: C30 + 1e+06 alpha_ob30_seg15[2] <= 1.0000385781898986e+06
 R7164: C32 + 1e+06 alpha_ob30_seg15[2] <= 1.0000385781898986e+06
 R7165: C31 + 1e+06 alpha_ob30_seg15[3] <= 1.0000043162402755e+06
 R7166: C33 + 1e+06 alpha_ob30_seg15[3] <= 1.0000043162402755e+06
 R7167: alpha_ob30_seg15[0] + alpha_ob30_seg15[1] + alpha_ob30_seg15[2]
   + alpha_ob30_seg15[3] >= 1
 R7168: - C30 + 1e+06 alpha_ob31_seg15[0] <= 999939.2718
 R7169: - C32 + 1e+06 alpha_ob31_seg15[0] <= 999939.2718
 R7170: - C31 + 1e+06 alpha_ob31_seg15[1] <= 999993.13376
 R7171: - C33 + 1e+06 alpha_ob31_seg15[1] <= 999993.13376
 R7172: C30 + 1e+06 alpha_ob31_seg15[2] <= 1.0000385782e+06
 R7173: C32 + 1e+06 alpha_ob31_seg15[2] <= 1.0000385782e+06
 R7174: C31 + 1e+06 alpha_ob31_seg15[3] <= 1.00000431624e+06
 R7175: C33 + 1e+06 alpha_ob31_seg15[3] <= 1.00000431624e+06
 R7176: alpha_ob31_seg15[0] + alpha_ob31_seg15[1] + alpha_ob31_seg15[2]
   + alpha_ob31_seg15[3] >= 1
 R7177: - C30 + 1e+06 alpha_ob32_seg15[0] <= 999939.2718101013
 R7178: - C32 + 1e+06 alpha_ob32_seg15[0] <= 999939.2718101013
 R7179: - C31 + 1e+06 alpha_ob32_seg15[1] <= 999987.7837597246
 R7180: - C33 + 1e+06 alpha_ob32_seg15[1] <= 999987.7837597246
 R7181: C30 + 1e+06 alpha_ob32_seg15[2] <= 1.0000581782101013e+06
 R7182: C32 + 1e+06 alpha_ob32_seg15[2] <= 1.0000581782101013e+06
 R7183: C31 + 1e+06 alpha_ob32_seg15[3] <= 1.0000043162397245e+06
 R7184: C33 + 1e+06 alpha_ob32_seg15[3] <= 1.0000043162397245e+06
 R7185: alpha_ob32_seg15[0] + alpha_ob32_seg15[1] + alpha_ob32_seg15[2]
   + alpha_ob32_seg15[3] >= 1
 R7186: - C30 + 1e+06 alpha_ob33_seg15[0] <= 999939.271799801
 R7187: - C32 + 1e+06 alpha_ob33_seg15[0] <= 999939.271799801
 R7188: - C31 + 1e+06 alpha_ob33_seg15[1] <= 999987.7838262042
 R7189: - C33 + 1e+06 alpha_ob33_seg15[1] <= 999987.7838262042
 R7190: C30 + 1e+06 alpha_ob33_seg15[2] <= 1.000038578199801e+06
 R7191: C32 + 1e+06 alpha_ob33_seg15[2] <= 1.000038578199801e+06
 R7192: C31 + 1e+06 alpha_ob33_seg15[3] <= 1.0000096662262043e+06
 R7193: C33 + 1e+06 alpha_ob33_seg15[3] <= 1.0000096662262043e+06
 R7194: alpha_ob33_seg15[0] + alpha_ob33_seg15[1] + alpha_ob33_seg15[2]
   + alpha_ob33_seg15[3] >= 1
 R7195: - C30 + 1e+06 alpha_ob34_seg15[0] <= 999926.0708
 R7196: - C32 + 1e+06 alpha_ob34_seg15[0] <= 999926.0708
 R7197: - C31 + 1e+06 alpha_ob34_seg15[1] <= 999983.7196
 R7198: - C33 + 1e+06 alpha_ob34_seg15[1] <= 999983.7196
 R7199: C30 + 1e+06 alpha_ob34_seg15[2] <= 1.0000250292e+06
 R7200: C32 + 1e+06 alpha_ob34_seg15[2] <= 1.0000250292e+06
 R7201: C31 + 1e+06 alpha_ob34_seg15[3] <= 1.0000137304e+06
 R7202: C33 + 1e+06 alpha_ob34_seg15[3] <= 1.0000137304e+06
 R7203: alpha_ob34_seg15[0] + alpha_ob34_seg15[1] + alpha_ob34_seg15[2]
   + alpha_ob34_seg15[3] >= 1
 R7204: - C30 + 1e+06 alpha_ob35_seg15[0] <= 999926.0708293857
 R7205: - C32 + 1e+06 alpha_ob35_seg15[0] <= 999926.0708293857
 R7206: - C31 + 1e+06 alpha_ob35_seg15[1] <= 999967.8695997245
 R7207: - C33 + 1e+06 alpha_ob35_seg15[1] <= 999967.8695997245
 R7208: C30 + 1e+06 alpha_ob35_seg15[2] <= 1.0000713792293856e+06
 R7209: C32 + 1e+06 alpha_ob35_seg15[2] <= 1.0000713792293856e+06
 R7210: C31 + 1e+06 alpha_ob35_seg15[3] <= 1.0000137303997246e+06
 R7211: C33 + 1e+06 alpha_ob35_seg15[3] <= 1.0000137303997246e+06
 R7212: alpha_ob35_seg15[0] + alpha_ob35_seg15[1] + alpha_ob35_seg15[2]
   + alpha_ob35_seg15[3] >= 1
 R7213: - C30 + 1e+06 alpha_ob36_seg15[0] <= 999972.6917508709
 R7214: - C32 + 1e+06 alpha_ob36_seg15[0] <= 999972.6917508709
 R7215: - C31 + 1e+06 alpha_ob36_seg15[1] <= 999957.1644002756
 R7216: - C33 + 1e+06 alpha_ob36_seg15[1] <= 999957.1644002756
 R7217: C30 + 1e+06 alpha_ob36_seg15[2] <= 1.0000247581508709e+06
 R7218: C32 + 1e+06 alpha_ob36_seg15[2] <= 1.0000247581508709e+06
 R7219: C31 + 1e+06 alpha_ob36_seg15[3] <= 1.0000136856002755e+06
 R7220: C33 + 1e+06 alpha_ob36_seg15[3] <= 1.0000136856002755e+06
 R7221: alpha_ob36_seg15[0] + alpha_ob36_seg15[1] + alpha_ob36_seg15[2]
   + alpha_ob36_seg15[3] >= 1
 R7222: - C30 + 1e+06 alpha_ob37_seg15[0] <= 999939.0009674003
 R7223: - C32 + 1e+06 alpha_ob37_seg15[0] <= 999939.0009674003
 R7224: - C31 + 1e+06 alpha_ob37_seg15[1] <= 999961.9516002756
 R7225: - C33 + 1e+06 alpha_ob37_seg15[1] <= 999961.9516002756
 R7226: C30 + 1e+06 alpha_ob37_seg15[2] <= 1.0000584489674004e+06
 R7227: C32 + 1e+06 alpha_ob37_seg15[2] <= 1.0000584489674004e+06
 R7228: C31 + 1e+06 alpha_ob37_seg15[3] <= 1.0000178984002755e+06
 R7229: C33 + 1e+06 alpha_ob37_seg15[3] <= 1.0000178984002755e+06
 R7230: alpha_ob37_seg15[0] + alpha_ob37_seg15[1] + alpha_ob37_seg15[2]
   + alpha_ob37_seg15[3] >= 1
 R7231: - C30 + 1e+06 alpha_ob38_seg15[0] <= 999994.9164998011
 R7232: - C32 + 1e+06 alpha_ob38_seg15[0] <= 999994.9164998011
 R7233: - C31 + 1e+06 alpha_ob38_seg15[1] <= 999925.4284782809
 R7234: - C33 + 1e+06 alpha_ob38_seg15[1] <= 999925.4284782809
 R7235: C30 + 1e+06 alpha_ob38_seg15[2] <= 999943.6834998011
 R7236: C32 + 1e+06 alpha_ob38_seg15[2] <= 999943.6834998011
 R7237: C31 + 1e+06 alpha_ob38_seg15[3] <= 1.0000720216782809e+06
 R7238: C33 + 1e+06 alpha_ob38_seg15[3] <= 1.0000720216782809e+06
 R7239: alpha_ob38_seg15[0] + alpha_ob38_seg15[1] + alpha_ob38_seg15[2]
   + alpha_ob38_seg15[3] >= 1
 R7240: - C30 + 1e+06 alpha_ob39_seg15[0] <= 999914.52327842
 R7241: - C32 + 1e+06 alpha_ob39_seg15[0] <= 999914.52327842
 R7242: - C31 + 1e+06 alpha_ob39_seg15[1] <= 999993.6552362755
 R7243: - C33 + 1e+06 alpha_ob39_seg15[1] <= 999993.6552362755
 R7244: C30 + 1e+06 alpha_ob39_seg15[2] <= 1.0000829266784199e+06
 R7245: C32 + 1e+06 alpha_ob39_seg15[2] <= 1.0000829266784199e+06
 R7246: C31 + 1e+06 alpha_ob39_seg15[3] <= 999992.1947642756
 R7247: C33 + 1e+06 alpha_ob39_seg15[3] <= 999992.1947642756
 R7248: alpha_ob39_seg15[0] + alpha_ob39_seg15[1] + alpha_ob39_seg15[2]
   + alpha_ob39_seg15[3] >= 1
 R7249: - C30 + 1e+06 alpha_ob40_seg15[0] <= 999912.265199801
 R7250: - C32 + 1e+06 alpha_ob40_seg15[0] <= 999912.265199801
 R7251: - C31 + 1e+06 alpha_ob40_seg15[1] <= 999987.7842112777
 R7252: - C33 + 1e+06 alpha_ob40_seg15[1] <= 999987.7842112777
 R7253: C30 + 1e+06 alpha_ob40_seg15[2] <= 1.000076834799801e+06
 R7254: C32 + 1e+06 alpha_ob40_seg15[2] <= 1.000076834799801e+06
 R7255: C31 + 1e+06 alpha_ob40_seg15[3] <= 1.0000096658112778e+06
 R7256: C33 + 1e+06 alpha_ob40_seg15[3] <= 1.0000096658112778e+06
 R7257: alpha_ob40_seg15[0] + alpha_ob40_seg15[1] + alpha_ob40_seg15[2]
   + alpha_ob40_seg15[3] >= 1
 R7258: - C30 + 1e+06 alpha_ob41_seg15[0] <= 999920.6152119379
 R7259: - C32 + 1e+06 alpha_ob41_seg15[0] <= 999920.6152119379
 R7260: - C31 + 1e+06 alpha_ob41_seg15[1] <= 999981.4341997246
 R7261: - C33 + 1e+06 alpha_ob41_seg15[1] <= 999981.4341997246
 R7262: C30 + 1e+06 alpha_ob41_seg15[2] <= 1.0000768348119379e+06
 R7263: C32 + 1e+06 alpha_ob41_seg15[2] <= 1.0000768348119379e+06
 R7264: C31 + 1e+06 alpha_ob41_seg15[3] <= 1.0000096657997245e+06
 R7265: C33 + 1e+06 alpha_ob41_seg15[3] <= 1.0000096657997245e+06
 R7266: alpha_ob41_seg15[0] + alpha_ob41_seg15[1] + alpha_ob41_seg15[2]
   + alpha_ob41_seg15[3] >= 1
 R7267: - C30 + 1e+06 alpha_ob42_seg15[0] <= 999919.2652
 R7268: - C32 + 1e+06 alpha_ob42_seg15[0] <= 999919.2652
 R7269: - C31 + 1e+06 alpha_ob42_seg15[1] <= 999981.4342
 R7270: - C33 + 1e+06 alpha_ob42_seg15[1] <= 999981.4342
 R7271: C30 + 1e+06 alpha_ob42_seg15[2] <= 1.0000768348e+06
 R7272: C32 + 1e+06 alpha_ob42_seg15[2] <= 1.0000768348e+06
 R7273: C31 + 1e+06 alpha_ob42_seg15[3] <= 1.0000160158000001e+06
 R7274: C33 + 1e+06 alpha_ob42_seg15[3] <= 1.0000160158000001e+06
 R7275: alpha_ob42_seg15[0] + alpha_ob42_seg15[1] + alpha_ob42_seg15[2]
   + alpha_ob42_seg15[3] >= 1
 R7276: - C30 + 1e+06 alpha_ob43_seg15[0] <= 999919.2652454559
 R7277: - C32 + 1e+06 alpha_ob43_seg15[0] <= 999919.2652454559
 R7278: - C31 + 1e+06 alpha_ob43_seg15[1] <= 999956.8341997246
 R7279: - C33 + 1e+06 alpha_ob43_seg15[1] <= 999956.8341997246
 R7280: C30 + 1e+06 alpha_ob43_seg15[2] <= 1.0000781848454559e+06
 R7281: C32 + 1e+06 alpha_ob43_seg15[2] <= 1.0000781848454559e+06
 R7282: C31 + 1e+06 alpha_ob43_seg15[3] <= 1.0000160157997246e+06
 R7283: C33 + 1e+06 alpha_ob43_seg15[3] <= 1.0000160157997246e+06
 R7284: alpha_ob43_seg15[0] + alpha_ob43_seg15[1] + alpha_ob43_seg15[2]
   + alpha_ob43_seg15[3] >= 1
 R7285: - C30 + 1e+06 alpha_ob44_seg15[0] <= 999991.83683
 R7286: - C32 + 1e+06 alpha_ob44_seg15[0] <= 999991.83683
 R7287: - C31 + 1e+06 alpha_ob44_seg15[1] <= 999940.589
 R7288: - C33 + 1e+06 alpha_ob44_seg15[1] <= 999940.589
 R7289: C30 + 1e+06 alpha_ob44_seg15[2] <= 1.00000401317e+06
 R7290: C32 + 1e+06 alpha_ob44_seg15[2] <= 1.00000401317e+06
 R7291: C31 + 1e+06 alpha_ob44_seg15[3] <= 1.000056861e+06
 R7292: C33 + 1e+06 alpha_ob44_seg15[3] <= 1.000056861e+06
 R7293: alpha_ob44_seg15[0] + alpha_ob44_seg15[1] + alpha_ob44_seg15[2]
   + alpha_ob44_seg15[3] >= 1
 R7294: - C30 + 1e+06 alpha_ob45_seg15[0] <= 999991.8368084199
 R7295: - C32 + 1e+06 alpha_ob45_seg15[0] <= 999991.8368084199
 R7296: - C31 + 1e+06 alpha_ob45_seg15[1] <= 999940.5890002755
 R7297: - C33 + 1e+06 alpha_ob45_seg15[1] <= 999940.5890002755
 R7298: C30 + 1e+06 alpha_ob45_seg15[2] <= 1.0000056131484199e+06
 R7299: C32 + 1e+06 alpha_ob45_seg15[2] <= 1.0000056131484199e+06
 R7300: C31 + 1e+06 alpha_ob45_seg15[3] <= 1.0000452610002755e+06
 R7301: C33 + 1e+06 alpha_ob45_seg15[3] <= 1.0000452610002755e+06
 R7302: alpha_ob45_seg15[0] + alpha_ob45_seg15[1] + alpha_ob45_seg15[2]
   + alpha_ob45_seg15[3] >= 1
 R7303: - C30 + 1e+06 alpha_ob46_seg15[0] <= 1.0000764021508709e+06
 R7304: - C32 + 1e+06 alpha_ob46_seg15[0] <= 1.0000764021508709e+06
 R7305: - C31 + 1e+06 alpha_ob46_seg15[1] <= 999957.1027002756
 R7306: - C33 + 1e+06 alpha_ob46_seg15[1] <= 999957.1027002756
 R7307: C30 + 1e+06 alpha_ob46_seg15[2] <= 999921.0477508709
 R7308: C32 + 1e+06 alpha_ob46_seg15[2] <= 999921.0477508709
 R7309: C31 + 1e+06 alpha_ob46_seg15[3] <= 1.0000137473002756e+06
 R7310: C33 + 1e+06 alpha_ob46_seg15[3] <= 1.0000137473002756e+06
 R7311: alpha_ob46_seg15[0] + alpha_ob46_seg15[1] + alpha_ob46_seg15[2]
   + alpha_ob46_seg15[3] >= 1
 R7312: - C30 + 1e+06 alpha_ob47_seg15[0] <= 1.0000445522e+06
 R7313: - C32 + 1e+06 alpha_ob47_seg15[0] <= 1.0000445522e+06
 R7314: - C31 + 1e+06 alpha_ob47_seg15[1] <= 999983.7027
 R7315: - C33 + 1e+06 alpha_ob47_seg15[1] <= 999983.7027
 R7316: C30 + 1e+06 alpha_ob47_seg15[2] <= 999921.0477999999
 R7317: C32 + 1e+06 alpha_ob47_seg15[2] <= 999921.0477999999
 R7318: C31 + 1e+06 alpha_ob47_seg15[3] <= 1.0000137473e+06
 R7319: C33 + 1e+06 alpha_ob47_seg15[3] <= 1.0000137473e+06
 R7320: alpha_ob47_seg15[0] + alpha_ob47_seg15[1] + alpha_ob47_seg15[2]
   + alpha_ob47_seg15[3] >= 1
 R7321: - C30 + 1e+06 alpha_ob48_seg15[0] <= 1.0000445522183661e+06
 R7322: - C32 + 1e+06 alpha_ob48_seg15[0] <= 1.0000445522183661e+06
 R7323: - C31 + 1e+06 alpha_ob48_seg15[1] <= 999973.8526997245
 R7324: - C33 + 1e+06 alpha_ob48_seg15[1] <= 999973.8526997245
 R7325: C30 + 1e+06 alpha_ob48_seg15[2] <= 999952.8978183661
 R7326: C32 + 1e+06 alpha_ob48_seg15[2] <= 999952.8978183661
 R7327: C31 + 1e+06 alpha_ob48_seg15[3] <= 1.0000137472997245e+06
 R7328: C33 + 1e+06 alpha_ob48_seg15[3] <= 1.0000137472997245e+06
 R7329: alpha_ob48_seg15[0] + alpha_ob48_seg15[1] + alpha_ob48_seg15[2]
   + alpha_ob48_seg15[3] >= 1
 R7330: - C30 + 1e+06 alpha_ob49_seg15[0] <= 1.000064332081634e+06
 R7331: - C32 + 1e+06 alpha_ob49_seg15[0] <= 1.000064332081634e+06
 R7332: - C31 + 1e+06 alpha_ob49_seg15[1] <= 999973.9401002756
 R7333: - C33 + 1e+06 alpha_ob49_seg15[1] <= 999973.9401002756
 R7334: C30 + 1e+06 alpha_ob49_seg15[2] <= 999933.1178816339
 R7335: C32 + 1e+06 alpha_ob49_seg15[2] <= 999933.1178816339
 R7336: C31 + 1e+06 alpha_ob49_seg15[3] <= 1.0000136599002755e+06
 R7337: C33 + 1e+06 alpha_ob49_seg15[3] <= 1.0000136599002755e+06
 R7338: alpha_ob49_seg15[0] + alpha_ob49_seg15[1] + alpha_ob49_seg15[2]
   + alpha_ob49_seg15[3] >= 1
 R7339: - C30 + 1e+06 alpha_ob50_seg15[0] <= 1.0000643321013774e+06
 R7340: - C32 + 1e+06 alpha_ob50_seg15[0] <= 1.0000643321013774e+06
 R7341: - C31 + 1e+06 alpha_ob50_seg15[1] <= 999983.1900997246
 R7342: - C33 + 1e+06 alpha_ob50_seg15[1] <= 999983.1900997246
 R7343: C30 + 1e+06 alpha_ob50_seg15[2] <= 999933.1179013775
 R7344: C32 + 1e+06 alpha_ob50_seg15[2] <= 999933.1179013775
 R7345: C31 + 1e+06 alpha_ob50_seg15[3] <= 1.0000136598997245e+06
 R7346: C33 + 1e+06 alpha_ob50_seg15[3] <= 1.0000136598997245e+06
 R7347: alpha_ob50_seg15[0] + alpha_ob50_seg15[1] + alpha_ob50_seg15[2]
   + alpha_ob50_seg15[3] >= 1
 R7348: - C32 + 1e+06 alpha_ob0_seg16[0] <= 1.0000284170499525e+06
 R7349: - C34 + 1e+06 alpha_ob0_seg16[0] <= 1.0000284170499525e+06
 R7350: - C33 + 1e+06 alpha_ob0_seg16[1] <= 999957.0843002755
 R7351: - C35 + 1e+06 alpha_ob0_seg16[1] <= 999957.0843002755
 R7352: C32 + 1e+06 alpha_ob0_seg16[2] <= 999969.0328499526
 R7353: C34 + 1e+06 alpha_ob0_seg16[2] <= 999969.0328499526
 R7354: C33 + 1e+06 alpha_ob0_seg16[3] <= 1.0000132657002755e+06
 R7355: C35 + 1e+06 alpha_ob0_seg16[3] <= 1.0000132657002755e+06
 R7356: alpha_ob0_seg16[0] + alpha_ob0_seg16[1] + alpha_ob0_seg16[2]
   + alpha_ob0_seg16[3] >= 1
 R7357: - C32 + 1e+06 alpha_ob1_seg16[0] <= 1.000028417099801e+06
 R7358: - C34 + 1e+06 alpha_ob1_seg16[0] <= 1.000028417099801e+06
 R7359: - C33 + 1e+06 alpha_ob1_seg16[1] <= 999984.1843155898
 R7360: - C35 + 1e+06 alpha_ob1_seg16[1] <= 999984.1843155898
 R7361: C32 + 1e+06 alpha_ob1_seg16[2] <= 999957.4328998009
 R7362: C34 + 1e+06 alpha_ob1_seg16[2] <= 999957.4328998009
 R7363: C33 + 1e+06 alpha_ob1_seg16[3] <= 1.0000132657155899e+06
 R7364: C35 + 1e+06 alpha_ob1_seg16[3] <= 1.0000132657155899e+06
 R7365: alpha_ob1_seg16[0] + alpha_ob1_seg16[1] + alpha_ob1_seg16[2]
   + alpha_ob1_seg16[3] >= 1
 R7366: - C32 + 1e+06 alpha_ob2_seg16[0] <= 1.0000400171500475e+06
 R7367: - C34 + 1e+06 alpha_ob2_seg16[0] <= 1.0000400171500475e+06
 R7368: - C33 + 1e+06 alpha_ob2_seg16[1] <= 999957.0842997245
 R7369: - C35 + 1e+06 alpha_ob2_seg16[1] <= 999957.0842997245
 R7370: C32 + 1e+06 alpha_ob2_seg16[2] <= 999957.4329500474
 R7371: C34 + 1e+06 alpha_ob2_seg16[2] <= 999957.4329500474
 R7372: C33 + 1e+06 alpha_ob2_seg16[3] <= 1.0000132656997246e+06
 R7373: C35 + 1e+06 alpha_ob2_seg16[3] <= 1.0000132656997246e+06
 R7374: alpha_ob2_seg16[0] + alpha_ob2_seg16[1] + alpha_ob2_seg16[2]
   + alpha_ob2_seg16[3] >= 1
 R7375: - C32 + 1e+06 alpha_ob3_seg16[0] <= 1.0000109161752058e+06
 R7376: - C34 + 1e+06 alpha_ob3_seg16[0] <= 1.0000109161752058e+06
 R7377: - C33 + 1e+06 alpha_ob3_seg16[1] <= 999970.2681002755
 R7378: - C35 + 1e+06 alpha_ob3_seg16[1] <= 999970.2681002755
 R7379: C32 + 1e+06 alpha_ob3_seg16[2] <= 999986.5337752058
 R7380: C34 + 1e+06 alpha_ob3_seg16[2] <= 999986.5337752058
 R7381: C33 + 1e+06 alpha_ob3_seg16[3] <= 1.0000138319002755e+06
 R7382: C35 + 1e+06 alpha_ob3_seg16[3] <= 1.0000138319002755e+06
 R7383: alpha_ob3_seg16[0] + alpha_ob3_seg16[1] + alpha_ob3_seg16[2]
   + alpha_ob3_seg16[3] >= 1
 R7384: - C32 + 1e+06 alpha_ob4_seg16[0] <= 1.000010916199801e+06
 R7385: - C34 + 1e+06 alpha_ob4_seg16[0] <= 1.000010916199801e+06
 R7386: - C33 + 1e+06 alpha_ob4_seg16[1] <= 999983.6181179117
 R7387: - C35 + 1e+06 alpha_ob4_seg16[1] <= 999983.6181179117
 R7388: C32 + 1e+06 alpha_ob4_seg16[2] <= 999973.183799801
 R7389: C34 + 1e+06 alpha_ob4_seg16[2] <= 999973.183799801
 R7390: C33 + 1e+06 alpha_ob4_seg16[3] <= 1.0000138319179118e+06
 R7391: C35 + 1e+06 alpha_ob4_seg16[3] <= 1.0000138319179118e+06
 R7392: alpha_ob4_seg16[0] + alpha_ob4_seg16[1] + alpha_ob4_seg16[2]
   + alpha_ob4_seg16[3] >= 1
 R7393: - C32 + 1e+06 alpha_ob5_seg16[0] <= 999992.49102
 R7394: - C34 + 1e+06 alpha_ob5_seg16[0] <= 999992.49102
 R7395: - C33 + 1e+06 alpha_ob5_seg16[1] <= 999957.1016000001
 R7396: - C35 + 1e+06 alpha_ob5_seg16[1] <= 999957.1016000001
 R7397: C32 + 1e+06 alpha_ob5_seg16[2] <= 999986.35898
 R7398: C34 + 1e+06 alpha_ob5_seg16[2] <= 999986.35898
 R7399: C33 + 1e+06 alpha_ob5_seg16[3] <= 1.0000403484e+06
 R7400: C35 + 1e+06 alpha_ob5_seg16[3] <= 1.0000403484e+06
 R7401: alpha_ob5_seg16[0] + alpha_ob5_seg16[1] + alpha_ob5_seg16[2]
   + alpha_ob5_seg16[3] >= 1
 R7402: - C32 + 1e+06 alpha_ob6_seg16[0] <= 999981.3909999999
 R7403: - C34 + 1e+06 alpha_ob6_seg16[0] <= 999981.3909999999
 R7404: - C33 + 1e+06 alpha_ob6_seg16[1] <= 999957.1016000001
 R7405: - C35 + 1e+06 alpha_ob6_seg16[1] <= 999957.1016000001
 R7406: C32 + 1e+06 alpha_ob6_seg16[2] <= 1.000004959e+06
 R7407: C34 + 1e+06 alpha_ob6_seg16[2] <= 1.000004959e+06
 R7408: C33 + 1e+06 alpha_ob6_seg16[3] <= 1.0000403484e+06
 R7409: C35 + 1e+06 alpha_ob6_seg16[3] <= 1.0000403484e+06
 R7410: alpha_ob6_seg16[0] + alpha_ob6_seg16[1] + alpha_ob6_seg16[2]
   + alpha_ob6_seg16[3] >= 1
 R7411: - C32 + 1e+06 alpha_ob7_seg16[0] <= 999981.3909825523
 R7412: - C34 + 1e+06 alpha_ob7_seg16[0] <= 999981.3909825523
 R7413: - C33 + 1e+06 alpha_ob7_seg16[1] <= 999957.1016002755
 R7414: - C35 + 1e+06 alpha_ob7_seg16[1] <= 999957.1016002755
 R7415: C32 + 1e+06 alpha_ob7_seg16[2] <= 1.0000160589825523e+06
 R7416: C34 + 1e+06 alpha_ob7_seg16[2] <= 1.0000160589825523e+06
 R7417: C33 + 1e+06 alpha_ob7_seg16[3] <= 1.0000309984002755e+06
 R7418: C35 + 1e+06 alpha_ob7_seg16[3] <= 1.0000309984002755e+06
 R7419: alpha_ob7_seg16[0] + alpha_ob7_seg16[1] + alpha_ob7_seg16[2]
   + alpha_ob7_seg16[3] >= 1
 R7420: - C32 + 1e+06 alpha_ob8_seg16[0] <= 999981.3867954084
 R7421: - C34 + 1e+06 alpha_ob8_seg16[0] <= 999981.3867954084
 R7422: - C33 + 1e+06 alpha_ob8_seg16[1] <= 999968.8595002755
 R7423: - C35 + 1e+06 alpha_ob8_seg16[1] <= 999968.8595002755
 R7424: C32 + 1e+06 alpha_ob8_seg16[2] <= 1.0000160631954084e+06
 R7425: C34 + 1e+06 alpha_ob8_seg16[2] <= 1.0000160631954084e+06
 R7426: C33 + 1e+06 alpha_ob8_seg16[3] <= 1.0000262405002755e+06
 R7427: C35 + 1e+06 alpha_ob8_seg16[3] <= 1.0000262405002755e+06
 R7428: alpha_ob8_seg16[0] + alpha_ob8_seg16[1] + alpha_ob8_seg16[2]
   + alpha_ob8_seg16[3] >= 1
 R7429: - C32 + 1e+06 alpha_ob9_seg16[0] <= 999981.386799801
 R7430: - C34 + 1e+06 alpha_ob9_seg16[0] <= 999981.386799801
 R7431: - C33 + 1e+06 alpha_ob9_seg16[1] <= 999971.2095013268
 R7432: - C35 + 1e+06 alpha_ob9_seg16[1] <= 999971.2095013268
 R7433: C32 + 1e+06 alpha_ob9_seg16[2] <= 1.0000152131998009e+06
 R7434: C34 + 1e+06 alpha_ob9_seg16[2] <= 1.0000152131998009e+06
 R7435: C33 + 1e+06 alpha_ob9_seg16[3] <= 1.0000262405013267e+06
 R7436: C35 + 1e+06 alpha_ob9_seg16[3] <= 1.0000262405013267e+06
 R7437: alpha_ob9_seg16[0] + alpha_ob9_seg16[1] + alpha_ob9_seg16[2]
   + alpha_ob9_seg16[3] >= 1
 R7438: - C32 + 1e+06 alpha_ob10_seg16[0] <= 999982.2368022958
 R7439: - C34 + 1e+06 alpha_ob10_seg16[0] <= 999982.2368022958
 R7440: - C33 + 1e+06 alpha_ob10_seg16[1] <= 999970.1094997245
 R7441: - C35 + 1e+06 alpha_ob10_seg16[1] <= 999970.1094997245
 R7442: C32 + 1e+06 alpha_ob10_seg16[2] <= 1.0000152132022957e+06
 R7443: C34 + 1e+06 alpha_ob10_seg16[2] <= 1.0000152132022957e+06
 R7444: C33 + 1e+06 alpha_ob10_seg16[3] <= 1.0000262404997245e+06
 R7445: C35 + 1e+06 alpha_ob10_seg16[3] <= 1.0000262404997245e+06
 R7446: alpha_ob10_seg16[0] + alpha_ob10_seg16[1] + alpha_ob10_seg16[2]
   + alpha_ob10_seg16[3] >= 1
 R7447: - C32 + 1e+06 alpha_ob11_seg16[0] <= 999982.236799801
 R7448: - C34 + 1e+06 alpha_ob11_seg16[0] <= 999982.236799801
 R7449: - C33 + 1e+06 alpha_ob11_seg16[1] <= 999970.1095291895
 R7450: - C35 + 1e+06 alpha_ob11_seg16[1] <= 999970.1095291895
 R7451: C32 + 1e+06 alpha_ob11_seg16[2] <= 999993.3631998011
 R7452: C34 + 1e+06 alpha_ob11_seg16[2] <= 999993.3631998011
 R7453: C33 + 1e+06 alpha_ob11_seg16[3] <= 1.0000273405291894e+06
 R7454: C35 + 1e+06 alpha_ob11_seg16[3] <= 1.0000273405291894e+06
 R7455: alpha_ob11_seg16[0] + alpha_ob11_seg16[1] + alpha_ob11_seg16[2]
   + alpha_ob11_seg16[3] >= 1
 R7456: - C32 + 1e+06 alpha_ob12_seg16[0] <= 999981.6050802565
 R7457: - C34 + 1e+06 alpha_ob12_seg16[0] <= 999981.6050802565
 R7458: - C33 + 1e+06 alpha_ob12_seg16[1] <= 999972.8942002755
 R7459: - C35 + 1e+06 alpha_ob12_seg16[1] <= 999972.8942002755
 R7460: C32 + 1e+06 alpha_ob12_seg16[2] <= 1.0000158448802566e+06
 R7461: C34 + 1e+06 alpha_ob12_seg16[2] <= 1.0000158448802566e+06
 R7462: C33 + 1e+06 alpha_ob12_seg16[3] <= 1.0000139558002756e+06
 R7463: C35 + 1e+06 alpha_ob12_seg16[3] <= 1.0000139558002756e+06
 R7464: alpha_ob12_seg16[0] + alpha_ob12_seg16[1] + alpha_ob12_seg16[2]
   + alpha_ob12_seg16[3] >= 1
 R7465: - C32 + 1e+06 alpha_ob13_seg16[0] <= 999981.605149801
 R7466: - C34 + 1e+06 alpha_ob13_seg16[0] <= 999981.605149801
 R7467: - C33 + 1e+06 alpha_ob13_seg16[1] <= 999983.4942391404
 R7468: - C35 + 1e+06 alpha_ob13_seg16[1] <= 999983.4942391404
 R7469: C32 + 1e+06 alpha_ob13_seg16[2] <= 999986.4948498011
 R7470: C34 + 1e+06 alpha_ob13_seg16[2] <= 999986.4948498011
 R7471: C33 + 1e+06 alpha_ob13_seg16[3] <= 1.0000139558391405e+06
 R7472: C35 + 1e+06 alpha_ob13_seg16[3] <= 1.0000139558391405e+06
 R7473: alpha_ob13_seg16[0] + alpha_ob13_seg16[1] + alpha_ob13_seg16[2]
   + alpha_ob13_seg16[3] >= 1
 R7474: - C32 + 1e+06 alpha_ob14_seg16[0] <= 999997.6803502565
 R7475: - C34 + 1e+06 alpha_ob14_seg16[0] <= 999997.6803502565
 R7476: - C33 + 1e+06 alpha_ob14_seg16[1] <= 999972.7268002755
 R7477: - C35 + 1e+06 alpha_ob14_seg16[1] <= 999972.7268002755
 R7478: C32 + 1e+06 alpha_ob14_seg16[2] <= 999999.7696102565
 R7479: C34 + 1e+06 alpha_ob14_seg16[2] <= 999999.7696102565
 R7480: C33 + 1e+06 alpha_ob14_seg16[3] <= 1.0000141232002755e+06
 R7481: C35 + 1e+06 alpha_ob14_seg16[3] <= 1.0000141232002755e+06
 R7482: alpha_ob14_seg16[0] + alpha_ob14_seg16[1] + alpha_ob14_seg16[2]
   + alpha_ob14_seg16[3] >= 1
 R7483: - C32 + 1e+06 alpha_ob15_seg16[0] <= 1.0000038341502565e+06
 R7484: - C34 + 1e+06 alpha_ob15_seg16[0] <= 1.0000038341502565e+06
 R7485: - C33 + 1e+06 alpha_ob15_seg16[1] <= 999972.6012002755
 R7486: - C35 + 1e+06 alpha_ob15_seg16[1] <= 999972.6012002755
 R7487: C32 + 1e+06 alpha_ob15_seg16[2] <= 999993.6158102566
 R7488: C34 + 1e+06 alpha_ob15_seg16[2] <= 999993.6158102566
 R7489: C33 + 1e+06 alpha_ob15_seg16[3] <= 1.0000142488002755e+06
 R7490: C35 + 1e+06 alpha_ob15_seg16[3] <= 1.0000142488002755e+06
 R7491: alpha_ob15_seg16[0] + alpha_ob15_seg16[1] + alpha_ob15_seg16[2]
   + alpha_ob15_seg16[3] >= 1
 R7492: - C32 + 1e+06 alpha_ob16_seg16[0] <= 1.0000162874063333e+06
 R7493: - C34 + 1e+06 alpha_ob16_seg16[0] <= 1.0000162874063333e+06
 R7494: - C33 + 1e+06 alpha_ob16_seg16[1] <= 999982.9439602756
 R7495: - C35 + 1e+06 alpha_ob16_seg16[1] <= 999982.9439602756
 R7496: C32 + 1e+06 alpha_ob16_seg16[2] <= 999981.1624063333
 R7497: C34 + 1e+06 alpha_ob16_seg16[2] <= 999981.1624063333
 R7498: C33 + 1e+06 alpha_ob16_seg16[3] <= 999963.6560402757
 R7499: C35 + 1e+06 alpha_ob16_seg16[3] <= 999963.6560402757
 R7500: alpha_ob16_seg16[0] + alpha_ob16_seg16[1] + alpha_ob16_seg16[2]
   + alpha_ob16_seg16[3] >= 1
 R7501: - C32 + 1e+06 alpha_ob17_seg16[0] <= 999980.937509
 R7502: - C34 + 1e+06 alpha_ob17_seg16[0] <= 999980.937509
 R7503: - C33 + 1e+06 alpha_ob17_seg16[1] <= 1.000034894e+06
 R7504: - C35 + 1e+06 alpha_ob17_seg16[1] <= 1.000034894e+06
 R7505: C32 + 1e+06 alpha_ob17_seg16[2] <= 999981.162491
 R7506: C34 + 1e+06 alpha_ob17_seg16[2] <= 999981.162491
 R7507: C33 + 1e+06 alpha_ob17_seg16[3] <= 999962.556
 R7508: C35 + 1e+06 alpha_ob17_seg16[3] <= 999962.556
 R7509: alpha_ob17_seg16[0] + alpha_ob17_seg16[1] + alpha_ob17_seg16[2]
   + alpha_ob17_seg16[3] >= 1
 R7510: - C32 + 1e+06 alpha_ob18_seg16[0] <= 999990.2670277582
 R7511: - C34 + 1e+06 alpha_ob18_seg16[0] <= 999990.2670277582
 R7512: - C33 + 1e+06 alpha_ob18_seg16[1] <= 1.0000347702002756e+06
 R7513: - C35 + 1e+06 alpha_ob18_seg16[1] <= 1.0000347702002756e+06
 R7514: C32 + 1e+06 alpha_ob18_seg16[2] <= 1.0000071828877581e+06
 R7515: C34 + 1e+06 alpha_ob18_seg16[2] <= 1.0000071828877581e+06
 R7516: C33 + 1e+06 alpha_ob18_seg16[3] <= 999939.8298002756
 R7517: C35 + 1e+06 alpha_ob18_seg16[3] <= 999939.8298002756
 R7518: alpha_ob18_seg16[0] + alpha_ob18_seg16[1] + alpha_ob18_seg16[2]
   + alpha_ob18_seg16[3] >= 1
 R7519: - C32 + 1e+06 alpha_ob19_seg16[0] <= 999990.267069801
 R7520: - C34 + 1e+06 alpha_ob19_seg16[0] <= 999990.267069801
 R7521: - C33 + 1e+06 alpha_ob19_seg16[1] <= 1.0000576202069657e+06
 R7522: - C35 + 1e+06 alpha_ob19_seg16[1] <= 1.0000576202069657e+06
 R7523: C32 + 1e+06 alpha_ob19_seg16[2] <= 1.000002082929801e+06
 R7524: C34 + 1e+06 alpha_ob19_seg16[2] <= 1.000002082929801e+06
 R7525: C33 + 1e+06 alpha_ob19_seg16[3] <= 999939.8298069657
 R7526: C35 + 1e+06 alpha_ob19_seg16[3] <= 999939.8298069657
 R7527: alpha_ob19_seg16[0] + alpha_ob19_seg16[1] + alpha_ob19_seg16[2]
   + alpha_ob19_seg16[3] >= 1
 R7528: - C32 + 1e+06 alpha_ob20_seg16[0] <= 999991.073732
 R7529: - C34 + 1e+06 alpha_ob20_seg16[0] <= 999991.073732
 R7530: - C33 + 1e+06 alpha_ob20_seg16[1] <= 999952.2125
 R7531: - C35 + 1e+06 alpha_ob20_seg16[1] <= 999952.2125
 R7532: C32 + 1e+06 alpha_ob20_seg16[2] <= 999990.776268
 R7533: C34 + 1e+06 alpha_ob20_seg16[2] <= 999990.776268
 R7534: C33 + 1e+06 alpha_ob20_seg16[3] <= 1.0000452375e+06
 R7535: C35 + 1e+06 alpha_ob20_seg16[3] <= 1.0000452375e+06
 R7536: alpha_ob20_seg16[0] + alpha_ob20_seg16[1] + alpha_ob20_seg16[2]
   + alpha_ob20_seg16[3] >= 1
 R7537: - C32 + 1e+06 alpha_ob21_seg16[0] <= 999985.657099801
 R7538: - C34 + 1e+06 alpha_ob21_seg16[0] <= 999985.657099801
 R7539: - C33 + 1e+06 alpha_ob21_seg16[1] <= 1.0000576202063023e+06
 R7540: - C35 + 1e+06 alpha_ob21_seg16[1] <= 1.0000576202063023e+06
 R7541: C32 + 1e+06 alpha_ob21_seg16[2] <= 1.000007192899801e+06
 R7542: C34 + 1e+06 alpha_ob21_seg16[2] <= 1.000007192899801e+06
 R7543: C33 + 1e+06 alpha_ob21_seg16[3] <= 999939.8298063023
 R7544: C35 + 1e+06 alpha_ob21_seg16[3] <= 999939.8298063023
 R7545: alpha_ob21_seg16[0] + alpha_ob21_seg16[1] + alpha_ob21_seg16[2]
   + alpha_ob21_seg16[3] >= 1
 R7546: - C32 + 1e+06 alpha_ob22_seg16[0] <= 999994.771829801
 R7547: - C34 + 1e+06 alpha_ob22_seg16[0] <= 999994.771829801
 R7548: - C33 + 1e+06 alpha_ob22_seg16[1] <= 1.0000615983063023e+06
 R7549: - C35 + 1e+06 alpha_ob22_seg16[1] <= 1.0000615983063023e+06
 R7550: C32 + 1e+06 alpha_ob22_seg16[2] <= 999998.078169801
 R7551: C34 + 1e+06 alpha_ob22_seg16[2] <= 999998.078169801
 R7552: C33 + 1e+06 alpha_ob22_seg16[3] <= 999935.8517063023
 R7553: C35 + 1e+06 alpha_ob22_seg16[3] <= 999935.8517063023
 R7554: alpha_ob22_seg16[0] + alpha_ob22_seg16[1] + alpha_ob22_seg16[2]
   + alpha_ob22_seg16[3] >= 1
 R7555: - C32 + 1e+06 alpha_ob23_seg16[0] <= 999980.9374508709
 R7556: - C34 + 1e+06 alpha_ob23_seg16[0] <= 999980.9374508709
 R7557: - C33 + 1e+06 alpha_ob23_seg16[1] <= 1.0000348940002755e+06
 R7558: - C35 + 1e+06 alpha_ob23_seg16[1] <= 1.0000348940002755e+06
 R7559: C32 + 1e+06 alpha_ob23_seg16[2] <= 1.0000165124508708e+06
 R7560: C34 + 1e+06 alpha_ob23_seg16[2] <= 1.0000165124508708e+06
 R7561: C33 + 1e+06 alpha_ob23_seg16[3] <= 999935.9560002756
 R7562: C35 + 1e+06 alpha_ob23_seg16[3] <= 999935.9560002756
 R7563: alpha_ob23_seg16[0] + alpha_ob23_seg16[1] + alpha_ob23_seg16[2]
   + alpha_ob23_seg16[3] >= 1
 R7564: - C32 + 1e+06 alpha_ob24_seg16[0] <= 999980.937499801
 R7565: - C34 + 1e+06 alpha_ob24_seg16[0] <= 999980.937499801
 R7566: - C33 + 1e+06 alpha_ob24_seg16[1] <= 1.0000614940072973e+06
 R7567: - C35 + 1e+06 alpha_ob24_seg16[1] <= 1.0000614940072973e+06
 R7568: C32 + 1e+06 alpha_ob24_seg16[2] <= 1.000011162499801e+06
 R7569: C34 + 1e+06 alpha_ob24_seg16[2] <= 1.000011162499801e+06
 R7570: C33 + 1e+06 alpha_ob24_seg16[3] <= 999935.9560072974
 R7571: C35 + 1e+06 alpha_ob24_seg16[3] <= 999935.9560072974
 R7572: alpha_ob24_seg16[0] + alpha_ob24_seg16[1] + alpha_ob24_seg16[2]
   + alpha_ob24_seg16[3] >= 1
 R7573: - C32 + 1e+06 alpha_ob25_seg16[0] <= 999956.8394000001
 R7574: - C34 + 1e+06 alpha_ob25_seg16[0] <= 999956.8394000001
 R7575: - C33 + 1e+06 alpha_ob25_seg16[1] <= 999952.2463
 R7576: - C35 + 1e+06 alpha_ob25_seg16[1] <= 999952.2463
 R7577: C32 + 1e+06 alpha_ob25_seg16[2] <= 1.0000097606e+06
 R7578: C34 + 1e+06 alpha_ob25_seg16[2] <= 1.0000097606e+06
 R7579: C33 + 1e+06 alpha_ob25_seg16[3] <= 1.0000452037e+06
 R7580: C35 + 1e+06 alpha_ob25_seg16[3] <= 1.0000452037e+06
 R7581: alpha_ob25_seg16[0] + alpha_ob25_seg16[1] + alpha_ob25_seg16[2]
   + alpha_ob25_seg16[3] >= 1
 R7582: - C32 + 1e+06 alpha_ob26_seg16[0] <= 999980.9375348955
 R7583: - C34 + 1e+06 alpha_ob26_seg16[0] <= 999980.9375348955
 R7584: - C33 + 1e+06 alpha_ob26_seg16[1] <= 1.0000160439997246e+06
 R7585: - C35 + 1e+06 alpha_ob26_seg16[1] <= 1.0000160439997246e+06
 R7586: C32 + 1e+06 alpha_ob26_seg16[2] <= 1.0000165125348954e+06
 R7587: C34 + 1e+06 alpha_ob26_seg16[2] <= 1.0000165125348954e+06
 R7588: C33 + 1e+06 alpha_ob26_seg16[3] <= 999962.5559997245
 R7589: C35 + 1e+06 alpha_ob26_seg16[3] <= 999962.5559997245
 R7590: alpha_ob26_seg16[0] + alpha_ob26_seg16[1] + alpha_ob26_seg16[2]
   + alpha_ob26_seg16[3] >= 1
 R7591: - C32 + 1e+06 alpha_ob27_seg16[0] <= 999956.839399801
 R7592: - C34 + 1e+06 alpha_ob27_seg16[0] <= 999956.839399801
 R7593: - C33 + 1e+06 alpha_ob27_seg16[1] <= 999925.3963434525
 R7594: - C35 + 1e+06 alpha_ob27_seg16[1] <= 999925.3963434525
 R7595: C32 + 1e+06 alpha_ob27_seg16[2] <= 1.0000080105998011e+06
 R7596: C34 + 1e+06 alpha_ob27_seg16[2] <= 1.0000080105998011e+06
 R7597: C33 + 1e+06 alpha_ob27_seg16[3] <= 1.0000720537434525e+06
 R7598: C35 + 1e+06 alpha_ob27_seg16[3] <= 1.0000720537434525e+06
 R7599: alpha_ob27_seg16[0] + alpha_ob27_seg16[1] + alpha_ob27_seg16[2]
   + alpha_ob27_seg16[3] >= 1
 R7600: - C32 + 1e+06 alpha_ob28_seg16[0] <= 999980.9374998009
 R7601: - C34 + 1e+06 alpha_ob28_seg16[0] <= 999980.9374998009
 R7602: - C33 + 1e+06 alpha_ob28_seg16[1] <= 1.0000160440053071e+06
 R7603: - C35 + 1e+06 alpha_ob28_seg16[1] <= 1.0000160440053071e+06
 R7604: C32 + 1e+06 alpha_ob28_seg16[2] <= 1.000012662499801e+06
 R7605: C34 + 1e+06 alpha_ob28_seg16[2] <= 1.000012662499801e+06
 R7606: C33 + 1e+06 alpha_ob28_seg16[3] <= 999981.4060053072
 R7607: C35 + 1e+06 alpha_ob28_seg16[3] <= 999981.4060053072
 R7608: alpha_ob28_seg16[0] + alpha_ob28_seg16[1] + alpha_ob28_seg16[2]
   + alpha_ob28_seg16[3] >= 1
 R7609: - C32 + 1e+06 alpha_ob29_seg16[0] <= 999983.6053459151
 R7610: - C34 + 1e+06 alpha_ob29_seg16[0] <= 999983.6053459151
 R7611: - C33 + 1e+06 alpha_ob29_seg16[1] <= 999987.7722497246
 R7612: - C35 + 1e+06 alpha_ob29_seg16[1] <= 999987.7722497246
 R7613: C32 + 1e+06 alpha_ob29_seg16[2] <= 1.000013844745915e+06
 R7614: C34 + 1e+06 alpha_ob29_seg16[2] <= 1.000013844745915e+06
 R7615: C33 + 1e+06 alpha_ob29_seg16[3] <= 999984.8277497246
 R7616: C35 + 1e+06 alpha_ob29_seg16[3] <= 999984.8277497246
 R7617: alpha_ob29_seg16[0] + alpha_ob29_seg16[1] + alpha_ob29_seg16[2]
   + alpha_ob29_seg16[3] >= 1
 R7618: - C32 + 1e+06 alpha_ob30_seg16[0] <= 999958.8717898987
 R7619: - C34 + 1e+06 alpha_ob30_seg16[0] <= 999958.8717898987
 R7620: - C33 + 1e+06 alpha_ob30_seg16[1] <= 999987.7837602755
 R7621: - C35 + 1e+06 alpha_ob30_seg16[1] <= 999987.7837602755
 R7622: C32 + 1e+06 alpha_ob30_seg16[2] <= 1.0000385781898986e+06
 R7623: C34 + 1e+06 alpha_ob30_seg16[2] <= 1.0000385781898986e+06
 R7624: C33 + 1e+06 alpha_ob30_seg16[3] <= 1.0000043162402755e+06
 R7625: C35 + 1e+06 alpha_ob30_seg16[3] <= 1.0000043162402755e+06
 R7626: alpha_ob30_seg16[0] + alpha_ob30_seg16[1] + alpha_ob30_seg16[2]
   + alpha_ob30_seg16[3] >= 1
 R7627: - C32 + 1e+06 alpha_ob31_seg16[0] <= 999939.2718
 R7628: - C34 + 1e+06 alpha_ob31_seg16[0] <= 999939.2718
 R7629: - C33 + 1e+06 alpha_ob31_seg16[1] <= 999993.13376
 R7630: - C35 + 1e+06 alpha_ob31_seg16[1] <= 999993.13376
 R7631: C32 + 1e+06 alpha_ob31_seg16[2] <= 1.0000385782e+06
 R7632: C34 + 1e+06 alpha_ob31_seg16[2] <= 1.0000385782e+06
 R7633: C33 + 1e+06 alpha_ob31_seg16[3] <= 1.00000431624e+06
 R7634: C35 + 1e+06 alpha_ob31_seg16[3] <= 1.00000431624e+06
 R7635: alpha_ob31_seg16[0] + alpha_ob31_seg16[1] + alpha_ob31_seg16[2]
   + alpha_ob31_seg16[3] >= 1
 R7636: - C32 + 1e+06 alpha_ob32_seg16[0] <= 999939.2718101013
 R7637: - C34 + 1e+06 alpha_ob32_seg16[0] <= 999939.2718101013
 R7638: - C33 + 1e+06 alpha_ob32_seg16[1] <= 999987.7837597246
 R7639: - C35 + 1e+06 alpha_ob32_seg16[1] <= 999987.7837597246
 R7640: C32 + 1e+06 alpha_ob32_seg16[2] <= 1.0000581782101013e+06
 R7641: C34 + 1e+06 alpha_ob32_seg16[2] <= 1.0000581782101013e+06
 R7642: C33 + 1e+06 alpha_ob32_seg16[3] <= 1.0000043162397245e+06
 R7643: C35 + 1e+06 alpha_ob32_seg16[3] <= 1.0000043162397245e+06
 R7644: alpha_ob32_seg16[0] + alpha_ob32_seg16[1] + alpha_ob32_seg16[2]
   + alpha_ob32_seg16[3] >= 1
 R7645: - C32 + 1e+06 alpha_ob33_seg16[0] <= 999939.271799801
 R7646: - C34 + 1e+06 alpha_ob33_seg16[0] <= 999939.271799801
 R7647: - C33 + 1e+06 alpha_ob33_seg16[1] <= 999987.7838262042
 R7648: - C35 + 1e+06 alpha_ob33_seg16[1] <= 999987.7838262042
 R7649: C32 + 1e+06 alpha_ob33_seg16[2] <= 1.000038578199801e+06
 R7650: C34 + 1e+06 alpha_ob33_seg16[2] <= 1.000038578199801e+06
 R7651: C33 + 1e+06 alpha_ob33_seg16[3] <= 1.0000096662262043e+06
 R7652: C35 + 1e+06 alpha_ob33_seg16[3] <= 1.0000096662262043e+06
 R7653: alpha_ob33_seg16[0] + alpha_ob33_seg16[1] + alpha_ob33_seg16[2]
   + alpha_ob33_seg16[3] >= 1
 R7654: - C32 + 1e+06 alpha_ob34_seg16[0] <= 999926.0708
 R7655: - C34 + 1e+06 alpha_ob34_seg16[0] <= 999926.0708
 R7656: - C33 + 1e+06 alpha_ob34_seg16[1] <= 999983.7196
 R7657: - C35 + 1e+06 alpha_ob34_seg16[1] <= 999983.7196
 R7658: C32 + 1e+06 alpha_ob34_seg16[2] <= 1.0000250292e+06
 R7659: C34 + 1e+06 alpha_ob34_seg16[2] <= 1.0000250292e+06
 R7660: C33 + 1e+06 alpha_ob34_seg16[3] <= 1.0000137304e+06
 R7661: C35 + 1e+06 alpha_ob34_seg16[3] <= 1.0000137304e+06
 R7662: alpha_ob34_seg16[0] + alpha_ob34_seg16[1] + alpha_ob34_seg16[2]
   + alpha_ob34_seg16[3] >= 1
 R7663: - C32 + 1e+06 alpha_ob35_seg16[0] <= 999926.0708293857
 R7664: - C34 + 1e+06 alpha_ob35_seg16[0] <= 999926.0708293857
 R7665: - C33 + 1e+06 alpha_ob35_seg16[1] <= 999967.8695997245
 R7666: - C35 + 1e+06 alpha_ob35_seg16[1] <= 999967.8695997245
 R7667: C32 + 1e+06 alpha_ob35_seg16[2] <= 1.0000713792293856e+06
 R7668: C34 + 1e+06 alpha_ob35_seg16[2] <= 1.0000713792293856e+06
 R7669: C33 + 1e+06 alpha_ob35_seg16[3] <= 1.0000137303997246e+06
 R7670: C35 + 1e+06 alpha_ob35_seg16[3] <= 1.0000137303997246e+06
 R7671: alpha_ob35_seg16[0] + alpha_ob35_seg16[1] + alpha_ob35_seg16[2]
   + alpha_ob35_seg16[3] >= 1
 R7672: - C32 + 1e+06 alpha_ob36_seg16[0] <= 999972.6917508709
 R7673: - C34 + 1e+06 alpha_ob36_seg16[0] <= 999972.6917508709
 R7674: - C33 + 1e+06 alpha_ob36_seg16[1] <= 999957.1644002756
 R7675: - C35 + 1e+06 alpha_ob36_seg16[1] <= 999957.1644002756
 R7676: C32 + 1e+06 alpha_ob36_seg16[2] <= 1.0000247581508709e+06
 R7677: C34 + 1e+06 alpha_ob36_seg16[2] <= 1.0000247581508709e+06
 R7678: C33 + 1e+06 alpha_ob36_seg16[3] <= 1.0000136856002755e+06
 R7679: C35 + 1e+06 alpha_ob36_seg16[3] <= 1.0000136856002755e+06
 R7680: alpha_ob36_seg16[0] + alpha_ob36_seg16[1] + alpha_ob36_seg16[2]
   + alpha_ob36_seg16[3] >= 1
 R7681: - C32 + 1e+06 alpha_ob37_seg16[0] <= 999939.0009674003
 R7682: - C34 + 1e+06 alpha_ob37_seg16[0] <= 999939.0009674003
 R7683: - C33 + 1e+06 alpha_ob37_seg16[1] <= 999961.9516002756
 R7684: - C35 + 1e+06 alpha_ob37_seg16[1] <= 999961.9516002756
 R7685: C32 + 1e+06 alpha_ob37_seg16[2] <= 1.0000584489674004e+06
 R7686: C34 + 1e+06 alpha_ob37_seg16[2] <= 1.0000584489674004e+06
 R7687: C33 + 1e+06 alpha_ob37_seg16[3] <= 1.0000178984002755e+06
 R7688: C35 + 1e+06 alpha_ob37_seg16[3] <= 1.0000178984002755e+06
 R7689: alpha_ob37_seg16[0] + alpha_ob37_seg16[1] + alpha_ob37_seg16[2]
   + alpha_ob37_seg16[3] >= 1
 R7690: - C32 + 1e+06 alpha_ob38_seg16[0] <= 999994.9164998011
 R7691: - C34 + 1e+06 alpha_ob38_seg16[0] <= 999994.9164998011
 R7692: - C33 + 1e+06 alpha_ob38_seg16[1] <= 999925.4284782809
 R7693: - C35 + 1e+06 alpha_ob38_seg16[1] <= 999925.4284782809
 R7694: C32 + 1e+06 alpha_ob38_seg16[2] <= 999943.6834998011
 R7695: C34 + 1e+06 alpha_ob38_seg16[2] <= 999943.6834998011
 R7696: C33 + 1e+06 alpha_ob38_seg16[3] <= 1.0000720216782809e+06
 R7697: C35 + 1e+06 alpha_ob38_seg16[3] <= 1.0000720216782809e+06
 R7698: alpha_ob38_seg16[0] + alpha_ob38_seg16[1] + alpha_ob38_seg16[2]
   + alpha_ob38_seg16[3] >= 1
 R7699: - C32 + 1e+06 alpha_ob39_seg16[0] <= 999914.52327842
 R7700: - C34 + 1e+06 alpha_ob39_seg16[0] <= 999914.52327842
 R7701: - C33 + 1e+06 alpha_ob39_seg16[1] <= 999993.6552362755
 R7702: - C35 + 1e+06 alpha_ob39_seg16[1] <= 999993.6552362755
 R7703: C32 + 1e+06 alpha_ob39_seg16[2] <= 1.0000829266784199e+06
 R7704: C34 + 1e+06 alpha_ob39_seg16[2] <= 1.0000829266784199e+06
 R7705: C33 + 1e+06 alpha_ob39_seg16[3] <= 999992.1947642756
 R7706: C35 + 1e+06 alpha_ob39_seg16[3] <= 999992.1947642756
 R7707: alpha_ob39_seg16[0] + alpha_ob39_seg16[1] + alpha_ob39_seg16[2]
   + alpha_ob39_seg16[3] >= 1
 R7708: - C32 + 1e+06 alpha_ob40_seg16[0] <= 999912.265199801
 R7709: - C34 + 1e+06 alpha_ob40_seg16[0] <= 999912.265199801
 R7710: - C33 + 1e+06 alpha_ob40_seg16[1] <= 999987.7842112777
 R7711: - C35 + 1e+06 alpha_ob40_seg16[1] <= 999987.7842112777
 R7712: C32 + 1e+06 alpha_ob40_seg16[2] <= 1.000076834799801e+06
 R7713: C34 + 1e+06 alpha_ob40_seg16[2] <= 1.000076834799801e+06
 R7714: C33 + 1e+06 alpha_ob40_seg16[3] <= 1.0000096658112778e+06
 R7715: C35 + 1e+06 alpha_ob40_seg16[3] <= 1.0000096658112778e+06
 R7716: alpha_ob40_seg16[0] + alpha_ob40_seg16[1] + alpha_ob40_seg16[2]
   + alpha_ob40_seg16[3] >= 1
 R7717: - C32 + 1e+06 alpha_ob41_seg16[0] <= 999920.6152119379
 R7718: - C34 + 1e+06 alpha_ob41_seg16[0] <= 999920.6152119379
 R7719: - C33 + 1e+06 alpha_ob41_seg16[1] <= 999981.4341997246
 R7720: - C35 + 1e+06 alpha_ob41_seg16[1] <= 999981.4341997246
 R7721: C32 + 1e+06 alpha_ob41_seg16[2] <= 1.0000768348119379e+06
 R7722: C34 + 1e+06 alpha_ob41_seg16[2] <= 1.0000768348119379e+06
 R7723: C33 + 1e+06 alpha_ob41_seg16[3] <= 1.0000096657997245e+06
 R7724: C35 + 1e+06 alpha_ob41_seg16[3] <= 1.0000096657997245e+06
 R7725: alpha_ob41_seg16[0] + alpha_ob41_seg16[1] + alpha_ob41_seg16[2]
   + alpha_ob41_seg16[3] >= 1
 R7726: - C32 + 1e+06 alpha_ob42_seg16[0] <= 999919.2652
 R7727: - C34 + 1e+06 alpha_ob42_seg16[0] <= 999919.2652
 R7728: - C33 + 1e+06 alpha_ob42_seg16[1] <= 999981.4342
 R7729: - C35 + 1e+06 alpha_ob42_seg16[1] <= 999981.4342
 R7730: C32 + 1e+06 alpha_ob42_seg16[2] <= 1.0000768348e+06
 R7731: C34 + 1e+06 alpha_ob42_seg16[2] <= 1.0000768348e+06
 R7732: C33 + 1e+06 alpha_ob42_seg16[3] <= 1.0000160158000001e+06
 R7733: C35 + 1e+06 alpha_ob42_seg16[3] <= 1.0000160158000001e+06
 R7734: alpha_ob42_seg16[0] + alpha_ob42_seg16[1] + alpha_ob42_seg16[2]
   + alpha_ob42_seg16[3] >= 1
 R7735: - C32 + 1e+06 alpha_ob43_seg16[0] <= 999919.2652454559
 R7736: - C34 + 1e+06 alpha_ob43_seg16[0] <= 999919.2652454559
 R7737: - C33 + 1e+06 alpha_ob43_seg16[1] <= 999956.8341997246
 R7738: - C35 + 1e+06 alpha_ob43_seg16[1] <= 999956.8341997246
 R7739: C32 + 1e+06 alpha_ob43_seg16[2] <= 1.0000781848454559e+06
 R7740: C34 + 1e+06 alpha_ob43_seg16[2] <= 1.0000781848454559e+06
 R7741: C33 + 1e+06 alpha_ob43_seg16[3] <= 1.0000160157997246e+06
 R7742: C35 + 1e+06 alpha_ob43_seg16[3] <= 1.0000160157997246e+06
 R7743: alpha_ob43_seg16[0] + alpha_ob43_seg16[1] + alpha_ob43_seg16[2]
   + alpha_ob43_seg16[3] >= 1
 R7744: - C32 + 1e+06 alpha_ob44_seg16[0] <= 999991.83683
 R7745: - C34 + 1e+06 alpha_ob44_seg16[0] <= 999991.83683
 R7746: - C33 + 1e+06 alpha_ob44_seg16[1] <= 999940.589
 R7747: - C35 + 1e+06 alpha_ob44_seg16[1] <= 999940.589
 R7748: C32 + 1e+06 alpha_ob44_seg16[2] <= 1.00000401317e+06
 R7749: C34 + 1e+06 alpha_ob44_seg16[2] <= 1.00000401317e+06
 R7750: C33 + 1e+06 alpha_ob44_seg16[3] <= 1.000056861e+06
 R7751: C35 + 1e+06 alpha_ob44_seg16[3] <= 1.000056861e+06
 R7752: alpha_ob44_seg16[0] + alpha_ob44_seg16[1] + alpha_ob44_seg16[2]
   + alpha_ob44_seg16[3] >= 1
 R7753: - C32 + 1e+06 alpha_ob45_seg16[0] <= 999991.8368084199
 R7754: - C34 + 1e+06 alpha_ob45_seg16[0] <= 999991.8368084199
 R7755: - C33 + 1e+06 alpha_ob45_seg16[1] <= 999940.5890002755
 R7756: - C35 + 1e+06 alpha_ob45_seg16[1] <= 999940.5890002755
 R7757: C32 + 1e+06 alpha_ob45_seg16[2] <= 1.0000056131484199e+06
 R7758: C34 + 1e+06 alpha_ob45_seg16[2] <= 1.0000056131484199e+06
 R7759: C33 + 1e+06 alpha_ob45_seg16[3] <= 1.0000452610002755e+06
 R7760: C35 + 1e+06 alpha_ob45_seg16[3] <= 1.0000452610002755e+06
 R7761: alpha_ob45_seg16[0] + alpha_ob45_seg16[1] + alpha_ob45_seg16[2]
   + alpha_ob45_seg16[3] >= 1
 R7762: - C32 + 1e+06 alpha_ob46_seg16[0] <= 1.0000764021508709e+06
 R7763: - C34 + 1e+06 alpha_ob46_seg16[0] <= 1.0000764021508709e+06
 R7764: - C33 + 1e+06 alpha_ob46_seg16[1] <= 999957.1027002756
 R7765: - C35 + 1e+06 alpha_ob46_seg16[1] <= 999957.1027002756
 R7766: C32 + 1e+06 alpha_ob46_seg16[2] <= 999921.0477508709
 R7767: C34 + 1e+06 alpha_ob46_seg16[2] <= 999921.0477508709
 R7768: C33 + 1e+06 alpha_ob46_seg16[3] <= 1.0000137473002756e+06
 R7769: C35 + 1e+06 alpha_ob46_seg16[3] <= 1.0000137473002756e+06
 R7770: alpha_ob46_seg16[0] + alpha_ob46_seg16[1] + alpha_ob46_seg16[2]
   + alpha_ob46_seg16[3] >= 1
 R7771: - C32 + 1e+06 alpha_ob47_seg16[0] <= 1.0000445522e+06
 R7772: - C34 + 1e+06 alpha_ob47_seg16[0] <= 1.0000445522e+06
 R7773: - C33 + 1e+06 alpha_ob47_seg16[1] <= 999983.7027
 R7774: - C35 + 1e+06 alpha_ob47_seg16[1] <= 999983.7027
 R7775: C32 + 1e+06 alpha_ob47_seg16[2] <= 999921.0477999999
 R7776: C34 + 1e+06 alpha_ob47_seg16[2] <= 999921.0477999999
 R7777: C33 + 1e+06 alpha_ob47_seg16[3] <= 1.0000137473e+06
 R7778: C35 + 1e+06 alpha_ob47_seg16[3] <= 1.0000137473e+06
 R7779: alpha_ob47_seg16[0] + alpha_ob47_seg16[1] + alpha_ob47_seg16[2]
   + alpha_ob47_seg16[3] >= 1
 R7780: - C32 + 1e+06 alpha_ob48_seg16[0] <= 1.0000445522183661e+06
 R7781: - C34 + 1e+06 alpha_ob48_seg16[0] <= 1.0000445522183661e+06
 R7782: - C33 + 1e+06 alpha_ob48_seg16[1] <= 999973.8526997245
 R7783: - C35 + 1e+06 alpha_ob48_seg16[1] <= 999973.8526997245
 R7784: C32 + 1e+06 alpha_ob48_seg16[2] <= 999952.8978183661
 R7785: C34 + 1e+06 alpha_ob48_seg16[2] <= 999952.8978183661
 R7786: C33 + 1e+06 alpha_ob48_seg16[3] <= 1.0000137472997245e+06
 R7787: C35 + 1e+06 alpha_ob48_seg16[3] <= 1.0000137472997245e+06
 R7788: alpha_ob48_seg16[0] + alpha_ob48_seg16[1] + alpha_ob48_seg16[2]
   + alpha_ob48_seg16[3] >= 1
 R7789: - C32 + 1e+06 alpha_ob49_seg16[0] <= 1.000064332081634e+06
 R7790: - C34 + 1e+06 alpha_ob49_seg16[0] <= 1.000064332081634e+06
 R7791: - C33 + 1e+06 alpha_ob49_seg16[1] <= 999973.9401002756
 R7792: - C35 + 1e+06 alpha_ob49_seg16[1] <= 999973.9401002756
 R7793: C32 + 1e+06 alpha_ob49_seg16[2] <= 999933.1178816339
 R7794: C34 + 1e+06 alpha_ob49_seg16[2] <= 999933.1178816339
 R7795: C33 + 1e+06 alpha_ob49_seg16[3] <= 1.0000136599002755e+06
 R7796: C35 + 1e+06 alpha_ob49_seg16[3] <= 1.0000136599002755e+06
 R7797: alpha_ob49_seg16[0] + alpha_ob49_seg16[1] + alpha_ob49_seg16[2]
   + alpha_ob49_seg16[3] >= 1
 R7798: - C32 + 1e+06 alpha_ob50_seg16[0] <= 1.0000643321013774e+06
 R7799: - C34 + 1e+06 alpha_ob50_seg16[0] <= 1.0000643321013774e+06
 R7800: - C33 + 1e+06 alpha_ob50_seg16[1] <= 999983.1900997246
 R7801: - C35 + 1e+06 alpha_ob50_seg16[1] <= 999983.1900997246
 R7802: C32 + 1e+06 alpha_ob50_seg16[2] <= 999933.1179013775
 R7803: C34 + 1e+06 alpha_ob50_seg16[2] <= 999933.1179013775
 R7804: C33 + 1e+06 alpha_ob50_seg16[3] <= 1.0000136598997245e+06
 R7805: C35 + 1e+06 alpha_ob50_seg16[3] <= 1.0000136598997245e+06
 R7806: alpha_ob50_seg16[0] + alpha_ob50_seg16[1] + alpha_ob50_seg16[2]
   + alpha_ob50_seg16[3] >= 1
 R7807: - C34 + 1e+06 alpha_ob0_seg17[0] <= 1.0000284170499525e+06
 R7808: - C36 + 1e+06 alpha_ob0_seg17[0] <= 1.0000284170499525e+06
 R7809: - C35 + 1e+06 alpha_ob0_seg17[1] <= 999957.0843002755
 R7810: - C37 + 1e+06 alpha_ob0_seg17[1] <= 999957.0843002755
 R7811: C34 + 1e+06 alpha_ob0_seg17[2] <= 999969.0328499526
 R7812: C36 + 1e+06 alpha_ob0_seg17[2] <= 999969.0328499526
 R7813: C35 + 1e+06 alpha_ob0_seg17[3] <= 1.0000132657002755e+06
 R7814: C37 + 1e+06 alpha_ob0_seg17[3] <= 1.0000132657002755e+06
 R7815: alpha_ob0_seg17[0] + alpha_ob0_seg17[1] + alpha_ob0_seg17[2]
   + alpha_ob0_seg17[3] >= 1
 R7816: - C34 + 1e+06 alpha_ob1_seg17[0] <= 1.000028417099801e+06
 R7817: - C36 + 1e+06 alpha_ob1_seg17[0] <= 1.000028417099801e+06
 R7818: - C35 + 1e+06 alpha_ob1_seg17[1] <= 999984.1843155898
 R7819: - C37 + 1e+06 alpha_ob1_seg17[1] <= 999984.1843155898
 R7820: C34 + 1e+06 alpha_ob1_seg17[2] <= 999957.4328998009
 R7821: C36 + 1e+06 alpha_ob1_seg17[2] <= 999957.4328998009
 R7822: C35 + 1e+06 alpha_ob1_seg17[3] <= 1.0000132657155899e+06
 R7823: C37 + 1e+06 alpha_ob1_seg17[3] <= 1.0000132657155899e+06
 R7824: alpha_ob1_seg17[0] + alpha_ob1_seg17[1] + alpha_ob1_seg17[2]
   + alpha_ob1_seg17[3] >= 1
 R7825: - C34 + 1e+06 alpha_ob2_seg17[0] <= 1.0000400171500475e+06
 R7826: - C36 + 1e+06 alpha_ob2_seg17[0] <= 1.0000400171500475e+06
 R7827: - C35 + 1e+06 alpha_ob2_seg17[1] <= 999957.0842997245
 R7828: - C37 + 1e+06 alpha_ob2_seg17[1] <= 999957.0842997245
 R7829: C34 + 1e+06 alpha_ob2_seg17[2] <= 999957.4329500474
 R7830: C36 + 1e+06 alpha_ob2_seg17[2] <= 999957.4329500474
 R7831: C35 + 1e+06 alpha_ob2_seg17[3] <= 1.0000132656997246e+06
 R7832: C37 + 1e+06 alpha_ob2_seg17[3] <= 1.0000132656997246e+06
 R7833: alpha_ob2_seg17[0] + alpha_ob2_seg17[1] + alpha_ob2_seg17[2]
   + alpha_ob2_seg17[3] >= 1
 R7834: - C34 + 1e+06 alpha_ob3_seg17[0] <= 1.0000109161752058e+06
 R7835: - C36 + 1e+06 alpha_ob3_seg17[0] <= 1.0000109161752058e+06
 R7836: - C35 + 1e+06 alpha_ob3_seg17[1] <= 999970.2681002755
 R7837: - C37 + 1e+06 alpha_ob3_seg17[1] <= 999970.2681002755
 R7838: C34 + 1e+06 alpha_ob3_seg17[2] <= 999986.5337752058
 R7839: C36 + 1e+06 alpha_ob3_seg17[2] <= 999986.5337752058
 R7840: C35 + 1e+06 alpha_ob3_seg17[3] <= 1.0000138319002755e+06
 R7841: C37 + 1e+06 alpha_ob3_seg17[3] <= 1.0000138319002755e+06
 R7842: alpha_ob3_seg17[0] + alpha_ob3_seg17[1] + alpha_ob3_seg17[2]
   + alpha_ob3_seg17[3] >= 1
 R7843: - C34 + 1e+06 alpha_ob4_seg17[0] <= 1.000010916199801e+06
 R7844: - C36 + 1e+06 alpha_ob4_seg17[0] <= 1.000010916199801e+06
 R7845: - C35 + 1e+06 alpha_ob4_seg17[1] <= 999983.6181179117
 R7846: - C37 + 1e+06 alpha_ob4_seg17[1] <= 999983.6181179117
 R7847: C34 + 1e+06 alpha_ob4_seg17[2] <= 999973.183799801
 R7848: C36 + 1e+06 alpha_ob4_seg17[2] <= 999973.183799801
 R7849: C35 + 1e+06 alpha_ob4_seg17[3] <= 1.0000138319179118e+06
 R7850: C37 + 1e+06 alpha_ob4_seg17[3] <= 1.0000138319179118e+06
 R7851: alpha_ob4_seg17[0] + alpha_ob4_seg17[1] + alpha_ob4_seg17[2]
   + alpha_ob4_seg17[3] >= 1
 R7852: - C34 + 1e+06 alpha_ob5_seg17[0] <= 999992.49102
 R7853: - C36 + 1e+06 alpha_ob5_seg17[0] <= 999992.49102
 R7854: - C35 + 1e+06 alpha_ob5_seg17[1] <= 999957.1016000001
 R7855: - C37 + 1e+06 alpha_ob5_seg17[1] <= 999957.1016000001
 R7856: C34 + 1e+06 alpha_ob5_seg17[2] <= 999986.35898
 R7857: C36 + 1e+06 alpha_ob5_seg17[2] <= 999986.35898
 R7858: C35 + 1e+06 alpha_ob5_seg17[3] <= 1.0000403484e+06
 R7859: C37 + 1e+06 alpha_ob5_seg17[3] <= 1.0000403484e+06
 R7860: alpha_ob5_seg17[0] + alpha_ob5_seg17[1] + alpha_ob5_seg17[2]
   + alpha_ob5_seg17[3] >= 1
 R7861: - C34 + 1e+06 alpha_ob6_seg17[0] <= 999981.3909999999
 R7862: - C36 + 1e+06 alpha_ob6_seg17[0] <= 999981.3909999999
 R7863: - C35 + 1e+06 alpha_ob6_seg17[1] <= 999957.1016000001
 R7864: - C37 + 1e+06 alpha_ob6_seg17[1] <= 999957.1016000001
 R7865: C34 + 1e+06 alpha_ob6_seg17[2] <= 1.000004959e+06
 R7866: C36 + 1e+06 alpha_ob6_seg17[2] <= 1.000004959e+06
 R7867: C35 + 1e+06 alpha_ob6_seg17[3] <= 1.0000403484e+06
 R7868: C37 + 1e+06 alpha_ob6_seg17[3] <= 1.0000403484e+06
 R7869: alpha_ob6_seg17[0] + alpha_ob6_seg17[1] + alpha_ob6_seg17[2]
   + alpha_ob6_seg17[3] >= 1
 R7870: - C34 + 1e+06 alpha_ob7_seg17[0] <= 999981.3909825523
 R7871: - C36 + 1e+06 alpha_ob7_seg17[0] <= 999981.3909825523
 R7872: - C35 + 1e+06 alpha_ob7_seg17[1] <= 999957.1016002755
 R7873: - C37 + 1e+06 alpha_ob7_seg17[1] <= 999957.1016002755
 R7874: C34 + 1e+06 alpha_ob7_seg17[2] <= 1.0000160589825523e+06
 R7875: C36 + 1e+06 alpha_ob7_seg17[2] <= 1.0000160589825523e+06
 R7876: C35 + 1e+06 alpha_ob7_seg17[3] <= 1.0000309984002755e+06
 R7877: C37 + 1e+06 alpha_ob7_seg17[3] <= 1.0000309984002755e+06
 R7878: alpha_ob7_seg17[0] + alpha_ob7_seg17[1] + alpha_ob7_seg17[2]
   + alpha_ob7_seg17[3] >= 1
 R7879: - C34 + 1e+06 alpha_ob8_seg17[0] <= 999981.3867954084
 R7880: - C36 + 1e+06 alpha_ob8_seg17[0] <= 999981.3867954084
 R7881: - C35 + 1e+06 alpha_ob8_seg17[1] <= 999968.8595002755
 R7882: - C37 + 1e+06 alpha_ob8_seg17[1] <= 999968.8595002755
 R7883: C34 + 1e+06 alpha_ob8_seg17[2] <= 1.0000160631954084e+06
 R7884: C36 + 1e+06 alpha_ob8_seg17[2] <= 1.0000160631954084e+06
 R7885: C35 + 1e+06 alpha_ob8_seg17[3] <= 1.0000262405002755e+06
 R7886: C37 + 1e+06 alpha_ob8_seg17[3] <= 1.0000262405002755e+06
 R7887: alpha_ob8_seg17[0] + alpha_ob8_seg17[1] + alpha_ob8_seg17[2]
   + alpha_ob8_seg17[3] >= 1
 R7888: - C34 + 1e+06 alpha_ob9_seg17[0] <= 999981.386799801
 R7889: - C36 + 1e+06 alpha_ob9_seg17[0] <= 999981.386799801
 R7890: - C35 + 1e+06 alpha_ob9_seg17[1] <= 999971.2095013268
 R7891: - C37 + 1e+06 alpha_ob9_seg17[1] <= 999971.2095013268
 R7892: C34 + 1e+06 alpha_ob9_seg17[2] <= 1.0000152131998009e+06
 R7893: C36 + 1e+06 alpha_ob9_seg17[2] <= 1.0000152131998009e+06
 R7894: C35 + 1e+06 alpha_ob9_seg17[3] <= 1.0000262405013267e+06
 R7895: C37 + 1e+06 alpha_ob9_seg17[3] <= 1.0000262405013267e+06
 R7896: alpha_ob9_seg17[0] + alpha_ob9_seg17[1] + alpha_ob9_seg17[2]
   + alpha_ob9_seg17[3] >= 1
 R7897: - C34 + 1e+06 alpha_ob10_seg17[0] <= 999982.2368022958
 R7898: - C36 + 1e+06 alpha_ob10_seg17[0] <= 999982.2368022958
 R7899: - C35 + 1e+06 alpha_ob10_seg17[1] <= 999970.1094997245
 R7900: - C37 + 1e+06 alpha_ob10_seg17[1] <= 999970.1094997245
 R7901: C34 + 1e+06 alpha_ob10_seg17[2] <= 1.0000152132022957e+06
 R7902: C36 + 1e+06 alpha_ob10_seg17[2] <= 1.0000152132022957e+06
 R7903: C35 + 1e+06 alpha_ob10_seg17[3] <= 1.0000262404997245e+06
 R7904: C37 + 1e+06 alpha_ob10_seg17[3] <= 1.0000262404997245e+06
 R7905: alpha_ob10_seg17[0] + alpha_ob10_seg17[1] + alpha_ob10_seg17[2]
   + alpha_ob10_seg17[3] >= 1
 R7906: - C34 + 1e+06 alpha_ob11_seg17[0] <= 999982.236799801
 R7907: - C36 + 1e+06 alpha_ob11_seg17[0] <= 999982.236799801
 R7908: - C35 + 1e+06 alpha_ob11_seg17[1] <= 999970.1095291895
 R7909: - C37 + 1e+06 alpha_ob11_seg17[1] <= 999970.1095291895
 R7910: C34 + 1e+06 alpha_ob11_seg17[2] <= 999993.3631998011
 R7911: C36 + 1e+06 alpha_ob11_seg17[2] <= 999993.3631998011
 R7912: C35 + 1e+06 alpha_ob11_seg17[3] <= 1.0000273405291894e+06
 R7913: C37 + 1e+06 alpha_ob11_seg17[3] <= 1.0000273405291894e+06
 R7914: alpha_ob11_seg17[0] + alpha_ob11_seg17[1] + alpha_ob11_seg17[2]
   + alpha_ob11_seg17[3] >= 1
 R7915: - C34 + 1e+06 alpha_ob12_seg17[0] <= 999981.6050802565
 R7916: - C36 + 1e+06 alpha_ob12_seg17[0] <= 999981.6050802565
 R7917: - C35 + 1e+06 alpha_ob12_seg17[1] <= 999972.8942002755
 R7918: - C37 + 1e+06 alpha_ob12_seg17[1] <= 999972.8942002755
 R7919: C34 + 1e+06 alpha_ob12_seg17[2] <= 1.0000158448802566e+06
 R7920: C36 + 1e+06 alpha_ob12_seg17[2] <= 1.0000158448802566e+06
 R7921: C35 + 1e+06 alpha_ob12_seg17[3] <= 1.0000139558002756e+06
 R7922: C37 + 1e+06 alpha_ob12_seg17[3] <= 1.0000139558002756e+06
 R7923: alpha_ob12_seg17[0] + alpha_ob12_seg17[1] + alpha_ob12_seg17[2]
   + alpha_ob12_seg17[3] >= 1
 R7924: - C34 + 1e+06 alpha_ob13_seg17[0] <= 999981.605149801
 R7925: - C36 + 1e+06 alpha_ob13_seg17[0] <= 999981.605149801
 R7926: - C35 + 1e+06 alpha_ob13_seg17[1] <= 999983.4942391404
 R7927: - C37 + 1e+06 alpha_ob13_seg17[1] <= 999983.4942391404
 R7928: C34 + 1e+06 alpha_ob13_seg17[2] <= 999986.4948498011
 R7929: C36 + 1e+06 alpha_ob13_seg17[2] <= 999986.4948498011
 R7930: C35 + 1e+06 alpha_ob13_seg17[3] <= 1.0000139558391405e+06
 R7931: C37 + 1e+06 alpha_ob13_seg17[3] <= 1.0000139558391405e+06
 R7932: alpha_ob13_seg17[0] + alpha_ob13_seg17[1] + alpha_ob13_seg17[2]
   + alpha_ob13_seg17[3] >= 1
 R7933: - C34 + 1e+06 alpha_ob14_seg17[0] <= 999997.6803502565
 R7934: - C36 + 1e+06 alpha_ob14_seg17[0] <= 999997.6803502565
 R7935: - C35 + 1e+06 alpha_ob14_seg17[1] <= 999972.7268002755
 R7936: - C37 + 1e+06 alpha_ob14_seg17[1] <= 999972.7268002755
 R7937: C34 + 1e+06 alpha_ob14_seg17[2] <= 999999.7696102565
 R7938: C36 + 1e+06 alpha_ob14_seg17[2] <= 999999.7696102565
 R7939: C35 + 1e+06 alpha_ob14_seg17[3] <= 1.0000141232002755e+06
 R7940: C37 + 1e+06 alpha_ob14_seg17[3] <= 1.0000141232002755e+06
 R7941: alpha_ob14_seg17[0] + alpha_ob14_seg17[1] + alpha_ob14_seg17[2]
   + alpha_ob14_seg17[3] >= 1
 R7942: - C34 + 1e+06 alpha_ob15_seg17[0] <= 1.0000038341502565e+06
 R7943: - C36 + 1e+06 alpha_ob15_seg17[0] <= 1.0000038341502565e+06
 R7944: - C35 + 1e+06 alpha_ob15_seg17[1] <= 999972.6012002755
 R7945: - C37 + 1e+06 alpha_ob15_seg17[1] <= 999972.6012002755
 R7946: C34 + 1e+06 alpha_ob15_seg17[2] <= 999993.6158102566
 R7947: C36 + 1e+06 alpha_ob15_seg17[2] <= 999993.6158102566
 R7948: C35 + 1e+06 alpha_ob15_seg17[3] <= 1.0000142488002755e+06
 R7949: C37 + 1e+06 alpha_ob15_seg17[3] <= 1.0000142488002755e+06
 R7950: alpha_ob15_seg17[0] + alpha_ob15_seg17[1] + alpha_ob15_seg17[2]
   + alpha_ob15_seg17[3] >= 1
 R7951: - C34 + 1e+06 alpha_ob16_seg17[0] <= 1.0000162874063333e+06
 R7952: - C36 + 1e+06 alpha_ob16_seg17[0] <= 1.0000162874063333e+06
 R7953: - C35 + 1e+06 alpha_ob16_seg17[1] <= 999982.9439602756
 R7954: - C37 + 1e+06 alpha_ob16_seg17[1] <= 999982.9439602756
 R7955: C34 + 1e+06 alpha_ob16_seg17[2] <= 999981.1624063333
 R7956: C36 + 1e+06 alpha_ob16_seg17[2] <= 999981.1624063333
 R7957: C35 + 1e+06 alpha_ob16_seg17[3] <= 999963.6560402757
 R7958: C37 + 1e+06 alpha_ob16_seg17[3] <= 999963.6560402757
 R7959: alpha_ob16_seg17[0] + alpha_ob16_seg17[1] + alpha_ob16_seg17[2]
   + alpha_ob16_seg17[3] >= 1
 R7960: - C34 + 1e+06 alpha_ob17_seg17[0] <= 999980.937509
 R7961: - C36 + 1e+06 alpha_ob17_seg17[0] <= 999980.937509
 R7962: - C35 + 1e+06 alpha_ob17_seg17[1] <= 1.000034894e+06
 R7963: - C37 + 1e+06 alpha_ob17_seg17[1] <= 1.000034894e+06
 R7964: C34 + 1e+06 alpha_ob17_seg17[2] <= 999981.162491
 R7965: C36 + 1e+06 alpha_ob17_seg17[2] <= 999981.162491
 R7966: C35 + 1e+06 alpha_ob17_seg17[3] <= 999962.556
 R7967: C37 + 1e+06 alpha_ob17_seg17[3] <= 999962.556
 R7968: alpha_ob17_seg17[0] + alpha_ob17_seg17[1] + alpha_ob17_seg17[2]
   + alpha_ob17_seg17[3] >= 1
 R7969: - C34 + 1e+06 alpha_ob18_seg17[0] <= 999990.2670277582
 R7970: - C36 + 1e+06 alpha_ob18_seg17[0] <= 999990.2670277582
 R7971: - C35 + 1e+06 alpha_ob18_seg17[1] <= 1.0000347702002756e+06
 R7972: - C37 + 1e+06 alpha_ob18_seg17[1] <= 1.0000347702002756e+06
 R7973: C34 + 1e+06 alpha_ob18_seg17[2] <= 1.0000071828877581e+06
 R7974: C36 + 1e+06 alpha_ob18_seg17[2] <= 1.0000071828877581e+06
 R7975: C35 + 1e+06 alpha_ob18_seg17[3] <= 999939.8298002756
 R7976: C37 + 1e+06 alpha_ob18_seg17[3] <= 999939.8298002756
 R7977: alpha_ob18_seg17[0] + alpha_ob18_seg17[1] + alpha_ob18_seg17[2]
   + alpha_ob18_seg17[3] >= 1
 R7978: - C34 + 1e+06 alpha_ob19_seg17[0] <= 999990.267069801
 R7979: - C36 + 1e+06 alpha_ob19_seg17[0] <= 999990.267069801
 R7980: - C35 + 1e+06 alpha_ob19_seg17[1] <= 1.0000576202069657e+06
 R7981: - C37 + 1e+06 alpha_ob19_seg17[1] <= 1.0000576202069657e+06
 R7982: C34 + 1e+06 alpha_ob19_seg17[2] <= 1.000002082929801e+06
 R7983: C36 + 1e+06 alpha_ob19_seg17[2] <= 1.000002082929801e+06
 R7984: C35 + 1e+06 alpha_ob19_seg17[3] <= 999939.8298069657
 R7985: C37 + 1e+06 alpha_ob19_seg17[3] <= 999939.8298069657
 R7986: alpha_ob19_seg17[0] + alpha_ob19_seg17[1] + alpha_ob19_seg17[2]
   + alpha_ob19_seg17[3] >= 1
 R7987: - C34 + 1e+06 alpha_ob20_seg17[0] <= 999991.073732
 R7988: - C36 + 1e+06 alpha_ob20_seg17[0] <= 999991.073732
 R7989: - C35 + 1e+06 alpha_ob20_seg17[1] <= 999952.2125
 R7990: - C37 + 1e+06 alpha_ob20_seg17[1] <= 999952.2125
 R7991: C34 + 1e+06 alpha_ob20_seg17[2] <= 999990.776268
 R7992: C36 + 1e+06 alpha_ob20_seg17[2] <= 999990.776268
 R7993: C35 + 1e+06 alpha_ob20_seg17[3] <= 1.0000452375e+06
 R7994: C37 + 1e+06 alpha_ob20_seg17[3] <= 1.0000452375e+06
 R7995: alpha_ob20_seg17[0] + alpha_ob20_seg17[1] + alpha_ob20_seg17[2]
   + alpha_ob20_seg17[3] >= 1
 R7996: - C34 + 1e+06 alpha_ob21_seg17[0] <= 999985.657099801
 R7997: - C36 + 1e+06 alpha_ob21_seg17[0] <= 999985.657099801
 R7998: - C35 + 1e+06 alpha_ob21_seg17[1] <= 1.0000576202063023e+06
 R7999: - C37 + 1e+06 alpha_ob21_seg17[1] <= 1.0000576202063023e+06
 R8000: C34 + 1e+06 alpha_ob21_seg17[2] <= 1.000007192899801e+06
 R8001: C36 + 1e+06 alpha_ob21_seg17[2] <= 1.000007192899801e+06
 R8002: C35 + 1e+06 alpha_ob21_seg17[3] <= 999939.8298063023
 R8003: C37 + 1e+06 alpha_ob21_seg17[3] <= 999939.8298063023
 R8004: alpha_ob21_seg17[0] + alpha_ob21_seg17[1] + alpha_ob21_seg17[2]
   + alpha_ob21_seg17[3] >= 1
 R8005: - C34 + 1e+06 alpha_ob22_seg17[0] <= 999994.771829801
 R8006: - C36 + 1e+06 alpha_ob22_seg17[0] <= 999994.771829801
 R8007: - C35 + 1e+06 alpha_ob22_seg17[1] <= 1.0000615983063023e+06
 R8008: - C37 + 1e+06 alpha_ob22_seg17[1] <= 1.0000615983063023e+06
 R8009: C34 + 1e+06 alpha_ob22_seg17[2] <= 999998.078169801
 R8010: C36 + 1e+06 alpha_ob22_seg17[2] <= 999998.078169801
 R8011: C35 + 1e+06 alpha_ob22_seg17[3] <= 999935.8517063023
 R8012: C37 + 1e+06 alpha_ob22_seg17[3] <= 999935.8517063023
 R8013: alpha_ob22_seg17[0] + alpha_ob22_seg17[1] + alpha_ob22_seg17[2]
   + alpha_ob22_seg17[3] >= 1
 R8014: - C34 + 1e+06 alpha_ob23_seg17[0] <= 999980.9374508709
 R8015: - C36 + 1e+06 alpha_ob23_seg17[0] <= 999980.9374508709
 R8016: - C35 + 1e+06 alpha_ob23_seg17[1] <= 1.0000348940002755e+06
 R8017: - C37 + 1e+06 alpha_ob23_seg17[1] <= 1.0000348940002755e+06
 R8018: C34 + 1e+06 alpha_ob23_seg17[2] <= 1.0000165124508708e+06
 R8019: C36 + 1e+06 alpha_ob23_seg17[2] <= 1.0000165124508708e+06
 R8020: C35 + 1e+06 alpha_ob23_seg17[3] <= 999935.9560002756
 R8021: C37 + 1e+06 alpha_ob23_seg17[3] <= 999935.9560002756
 R8022: alpha_ob23_seg17[0] + alpha_ob23_seg17[1] + alpha_ob23_seg17[2]
   + alpha_ob23_seg17[3] >= 1
 R8023: - C34 + 1e+06 alpha_ob24_seg17[0] <= 999980.937499801
 R8024: - C36 + 1e+06 alpha_ob24_seg17[0] <= 999980.937499801
 R8025: - C35 + 1e+06 alpha_ob24_seg17[1] <= 1.0000614940072973e+06
 R8026: - C37 + 1e+06 alpha_ob24_seg17[1] <= 1.0000614940072973e+06
 R8027: C34 + 1e+06 alpha_ob24_seg17[2] <= 1.000011162499801e+06
 R8028: C36 + 1e+06 alpha_ob24_seg17[2] <= 1.000011162499801e+06
 R8029: C35 + 1e+06 alpha_ob24_seg17[3] <= 999935.9560072974
 R8030: C37 + 1e+06 alpha_ob24_seg17[3] <= 999935.9560072974
 R8031: alpha_ob24_seg17[0] + alpha_ob24_seg17[1] + alpha_ob24_seg17[2]
   + alpha_ob24_seg17[3] >= 1
 R8032: - C34 + 1e+06 alpha_ob25_seg17[0] <= 999956.8394000001
 R8033: - C36 + 1e+06 alpha_ob25_seg17[0] <= 999956.8394000001
 R8034: - C35 + 1e+06 alpha_ob25_seg17[1] <= 999952.2463
 R8035: - C37 + 1e+06 alpha_ob25_seg17[1] <= 999952.2463
 R8036: C34 + 1e+06 alpha_ob25_seg17[2] <= 1.0000097606e+06
 R8037: C36 + 1e+06 alpha_ob25_seg17[2] <= 1.0000097606e+06
 R8038: C35 + 1e+06 alpha_ob25_seg17[3] <= 1.0000452037e+06
 R8039: C37 + 1e+06 alpha_ob25_seg17[3] <= 1.0000452037e+06
 R8040: alpha_ob25_seg17[0] + alpha_ob25_seg17[1] + alpha_ob25_seg17[2]
   + alpha_ob25_seg17[3] >= 1
 R8041: - C34 + 1e+06 alpha_ob26_seg17[0] <= 999980.9375348955
 R8042: - C36 + 1e+06 alpha_ob26_seg17[0] <= 999980.9375348955
 R8043: - C35 + 1e+06 alpha_ob26_seg17[1] <= 1.0000160439997246e+06
 R8044: - C37 + 1e+06 alpha_ob26_seg17[1] <= 1.0000160439997246e+06
 R8045: C34 + 1e+06 alpha_ob26_seg17[2] <= 1.0000165125348954e+06
 R8046: C36 + 1e+06 alpha_ob26_seg17[2] <= 1.0000165125348954e+06
 R8047: C35 + 1e+06 alpha_ob26_seg17[3] <= 999962.5559997245
 R8048: C37 + 1e+06 alpha_ob26_seg17[3] <= 999962.5559997245
 R8049: alpha_ob26_seg17[0] + alpha_ob26_seg17[1] + alpha_ob26_seg17[2]
   + alpha_ob26_seg17[3] >= 1
 R8050: - C34 + 1e+06 alpha_ob27_seg17[0] <= 999956.839399801
 R8051: - C36 + 1e+06 alpha_ob27_seg17[0] <= 999956.839399801
 R8052: - C35 + 1e+06 alpha_ob27_seg17[1] <= 999925.3963434525
 R8053: - C37 + 1e+06 alpha_ob27_seg17[1] <= 999925.3963434525
 R8054: C34 + 1e+06 alpha_ob27_seg17[2] <= 1.0000080105998011e+06
 R8055: C36 + 1e+06 alpha_ob27_seg17[2] <= 1.0000080105998011e+06
 R8056: C35 + 1e+06 alpha_ob27_seg17[3] <= 1.0000720537434525e+06
 R8057: C37 + 1e+06 alpha_ob27_seg17[3] <= 1.0000720537434525e+06
 R8058: alpha_ob27_seg17[0] + alpha_ob27_seg17[1] + alpha_ob27_seg17[2]
   + alpha_ob27_seg17[3] >= 1
 R8059: - C34 + 1e+06 alpha_ob28_seg17[0] <= 999980.9374998009
 R8060: - C36 + 1e+06 alpha_ob28_seg17[0] <= 999980.9374998009
 R8061: - C35 + 1e+06 alpha_ob28_seg17[1] <= 1.0000160440053071e+06
 R8062: - C37 + 1e+06 alpha_ob28_seg17[1] <= 1.0000160440053071e+06
 R8063: C34 + 1e+06 alpha_ob28_seg17[2] <= 1.000012662499801e+06
 R8064: C36 + 1e+06 alpha_ob28_seg17[2] <= 1.000012662499801e+06
 R8065: C35 + 1e+06 alpha_ob28_seg17[3] <= 999981.4060053072
 R8066: C37 + 1e+06 alpha_ob28_seg17[3] <= 999981.4060053072
 R8067: alpha_ob28_seg17[0] + alpha_ob28_seg17[1] + alpha_ob28_seg17[2]
   + alpha_ob28_seg17[3] >= 1
 R8068: - C34 + 1e+06 alpha_ob29_seg17[0] <= 999983.6053459151
 R8069: - C36 + 1e+06 alpha_ob29_seg17[0] <= 999983.6053459151
 R8070: - C35 + 1e+06 alpha_ob29_seg17[1] <= 999987.7722497246
 R8071: - C37 + 1e+06 alpha_ob29_seg17[1] <= 999987.7722497246
 R8072: C34 + 1e+06 alpha_ob29_seg17[2] <= 1.000013844745915e+06
 R8073: C36 + 1e+06 alpha_ob29_seg17[2] <= 1.000013844745915e+06
 R8074: C35 + 1e+06 alpha_ob29_seg17[3] <= 999984.8277497246
 R8075: C37 + 1e+06 alpha_ob29_seg17[3] <= 999984.8277497246
 R8076: alpha_ob29_seg17[0] + alpha_ob29_seg17[1] + alpha_ob29_seg17[2]
   + alpha_ob29_seg17[3] >= 1
 R8077: - C34 + 1e+06 alpha_ob30_seg17[0] <= 999958.8717898987
 R8078: - C36 + 1e+06 alpha_ob30_seg17[0] <= 999958.8717898987
 R8079: - C35 + 1e+06 alpha_ob30_seg17[1] <= 999987.7837602755
 R8080: - C37 + 1e+06 alpha_ob30_seg17[1] <= 999987.7837602755
 R8081: C34 + 1e+06 alpha_ob30_seg17[2] <= 1.0000385781898986e+06
 R8082: C36 + 1e+06 alpha_ob30_seg17[2] <= 1.0000385781898986e+06
 R8083: C35 + 1e+06 alpha_ob30_seg17[3] <= 1.0000043162402755e+06
 R8084: C37 + 1e+06 alpha_ob30_seg17[3] <= 1.0000043162402755e+06
 R8085: alpha_ob30_seg17[0] + alpha_ob30_seg17[1] + alpha_ob30_seg17[2]
   + alpha_ob30_seg17[3] >= 1
 R8086: - C34 + 1e+06 alpha_ob31_seg17[0] <= 999939.2718
 R8087: - C36 + 1e+06 alpha_ob31_seg17[0] <= 999939.2718
 R8088: - C35 + 1e+06 alpha_ob31_seg17[1] <= 999993.13376
 R8089: - C37 + 1e+06 alpha_ob31_seg17[1] <= 999993.13376
 R8090: C34 + 1e+06 alpha_ob31_seg17[2] <= 1.0000385782e+06
 R8091: C36 + 1e+06 alpha_ob31_seg17[2] <= 1.0000385782e+06
 R8092: C35 + 1e+06 alpha_ob31_seg17[3] <= 1.00000431624e+06
 R8093: C37 + 1e+06 alpha_ob31_seg17[3] <= 1.00000431624e+06
 R8094: alpha_ob31_seg17[0] + alpha_ob31_seg17[1] + alpha_ob31_seg17[2]
   + alpha_ob31_seg17[3] >= 1
 R8095: - C34 + 1e+06 alpha_ob32_seg17[0] <= 999939.2718101013
 R8096: - C36 + 1e+06 alpha_ob32_seg17[0] <= 999939.2718101013
 R8097: - C35 + 1e+06 alpha_ob32_seg17[1] <= 999987.7837597246
 R8098: - C37 + 1e+06 alpha_ob32_seg17[1] <= 999987.7837597246
 R8099: C34 + 1e+06 alpha_ob32_seg17[2] <= 1.0000581782101013e+06
 R8100: C36 + 1e+06 alpha_ob32_seg17[2] <= 1.0000581782101013e+06
 R8101: C35 + 1e+06 alpha_ob32_seg17[3] <= 1.0000043162397245e+06
 R8102: C37 + 1e+06 alpha_ob32_seg17[3] <= 1.0000043162397245e+06
 R8103: alpha_ob32_seg17[0] + alpha_ob32_seg17[1] + alpha_ob32_seg17[2]
   + alpha_ob32_seg17[3] >= 1
 R8104: - C34 + 1e+06 alpha_ob33_seg17[0] <= 999939.271799801
 R8105: - C36 + 1e+06 alpha_ob33_seg17[0] <= 999939.271799801
 R8106: - C35 + 1e+06 alpha_ob33_seg17[1] <= 999987.7838262042
 R8107: - C37 + 1e+06 alpha_ob33_seg17[1] <= 999987.7838262042
 R8108: C34 + 1e+06 alpha_ob33_seg17[2] <= 1.000038578199801e+06
 R8109: C36 + 1e+06 alpha_ob33_seg17[2] <= 1.000038578199801e+06
 R8110: C35 + 1e+06 alpha_ob33_seg17[3] <= 1.0000096662262043e+06
 R8111: C37 + 1e+06 alpha_ob33_seg17[3] <= 1.0000096662262043e+06
 R8112: alpha_ob33_seg17[0] + alpha_ob33_seg17[1] + alpha_ob33_seg17[2]
   + alpha_ob33_seg17[3] >= 1
 R8113: - C34 + 1e+06 alpha_ob34_seg17[0] <= 999926.0708
 R8114: - C36 + 1e+06 alpha_ob34_seg17[0] <= 999926.0708
 R8115: - C35 + 1e+06 alpha_ob34_seg17[1] <= 999983.7196
 R8116: - C37 + 1e+06 alpha_ob34_seg17[1] <= 999983.7196
 R8117: C34 + 1e+06 alpha_ob34_seg17[2] <= 1.0000250292e+06
 R8118: C36 + 1e+06 alpha_ob34_seg17[2] <= 1.0000250292e+06
 R8119: C35 + 1e+06 alpha_ob34_seg17[3] <= 1.0000137304e+06
 R8120: C37 + 1e+06 alpha_ob34_seg17[3] <= 1.0000137304e+06
 R8121: alpha_ob34_seg17[0] + alpha_ob34_seg17[1] + alpha_ob34_seg17[2]
   + alpha_ob34_seg17[3] >= 1
 R8122: - C34 + 1e+06 alpha_ob35_seg17[0] <= 999926.0708293857
 R8123: - C36 + 1e+06 alpha_ob35_seg17[0] <= 999926.0708293857
 R8124: - C35 + 1e+06 alpha_ob35_seg17[1] <= 999967.8695997245
 R8125: - C37 + 1e+06 alpha_ob35_seg17[1] <= 999967.8695997245
 R8126: C34 + 1e+06 alpha_ob35_seg17[2] <= 1.0000713792293856e+06
 R8127: C36 + 1e+06 alpha_ob35_seg17[2] <= 1.0000713792293856e+06
 R8128: C35 + 1e+06 alpha_ob35_seg17[3] <= 1.0000137303997246e+06
 R8129: C37 + 1e+06 alpha_ob35_seg17[3] <= 1.0000137303997246e+06
 R8130: alpha_ob35_seg17[0] + alpha_ob35_seg17[1] + alpha_ob35_seg17[2]
   + alpha_ob35_seg17[3] >= 1
 R8131: - C34 + 1e+06 alpha_ob36_seg17[0] <= 999972.6917508709
 R8132: - C36 + 1e+06 alpha_ob36_seg17[0] <= 999972.6917508709
 R8133: - C35 + 1e+06 alpha_ob36_seg17[1] <= 999957.1644002756
 R8134: - C37 + 1e+06 alpha_ob36_seg17[1] <= 999957.1644002756
 R8135: C34 + 1e+06 alpha_ob36_seg17[2] <= 1.0000247581508709e+06
 R8136: C36 + 1e+06 alpha_ob36_seg17[2] <= 1.0000247581508709e+06
 R8137: C35 + 1e+06 alpha_ob36_seg17[3] <= 1.0000136856002755e+06
 R8138: C37 + 1e+06 alpha_ob36_seg17[3] <= 1.0000136856002755e+06
 R8139: alpha_ob36_seg17[0] + alpha_ob36_seg17[1] + alpha_ob36_seg17[2]
   + alpha_ob36_seg17[3] >= 1
 R8140: - C34 + 1e+06 alpha_ob37_seg17[0] <= 999939.0009674003
 R8141: - C36 + 1e+06 alpha_ob37_seg17[0] <= 999939.0009674003
 R8142: - C35 + 1e+06 alpha_ob37_seg17[1] <= 999961.9516002756
 R8143: - C37 + 1e+06 alpha_ob37_seg17[1] <= 999961.9516002756
 R8144: C34 + 1e+06 alpha_ob37_seg17[2] <= 1.0000584489674004e+06
 R8145: C36 + 1e+06 alpha_ob37_seg17[2] <= 1.0000584489674004e+06
 R8146: C35 + 1e+06 alpha_ob37_seg17[3] <= 1.0000178984002755e+06
 R8147: C37 + 1e+06 alpha_ob37_seg17[3] <= 1.0000178984002755e+06
 R8148: alpha_ob37_seg17[0] + alpha_ob37_seg17[1] + alpha_ob37_seg17[2]
   + alpha_ob37_seg17[3] >= 1
 R8149: - C34 + 1e+06 alpha_ob38_seg17[0] <= 999994.9164998011
 R8150: - C36 + 1e+06 alpha_ob38_seg17[0] <= 999994.9164998011
 R8151: - C35 + 1e+06 alpha_ob38_seg17[1] <= 999925.4284782809
 R8152: - C37 + 1e+06 alpha_ob38_seg17[1] <= 999925.4284782809
 R8153: C34 + 1e+06 alpha_ob38_seg17[2] <= 999943.6834998011
 R8154: C36 + 1e+06 alpha_ob38_seg17[2] <= 999943.6834998011
 R8155: C35 + 1e+06 alpha_ob38_seg17[3] <= 1.0000720216782809e+06
 R8156: C37 + 1e+06 alpha_ob38_seg17[3] <= 1.0000720216782809e+06
 R8157: alpha_ob38_seg17[0] + alpha_ob38_seg17[1] + alpha_ob38_seg17[2]
   + alpha_ob38_seg17[3] >= 1
 R8158: - C34 + 1e+06 alpha_ob39_seg17[0] <= 999914.52327842
 R8159: - C36 + 1e+06 alpha_ob39_seg17[0] <= 999914.52327842
 R8160: - C35 + 1e+06 alpha_ob39_seg17[1] <= 999993.6552362755
 R8161: - C37 + 1e+06 alpha_ob39_seg17[1] <= 999993.6552362755
 R8162: C34 + 1e+06 alpha_ob39_seg17[2] <= 1.0000829266784199e+06
 R8163: C36 + 1e+06 alpha_ob39_seg17[2] <= 1.0000829266784199e+06
 R8164: C35 + 1e+06 alpha_ob39_seg17[3] <= 999992.1947642756
 R8165: C37 + 1e+06 alpha_ob39_seg17[3] <= 999992.1947642756
 R8166: alpha_ob39_seg17[0] + alpha_ob39_seg17[1] + alpha_ob39_seg17[2]
   + alpha_ob39_seg17[3] >= 1
 R8167: - C34 + 1e+06 alpha_ob40_seg17[0] <= 999912.265199801
 R8168: - C36 + 1e+06 alpha_ob40_seg17[0] <= 999912.265199801
 R8169: - C35 + 1e+06 alpha_ob40_seg17[1] <= 999987.7842112777
 R8170: - C37 + 1e+06 alpha_ob40_seg17[1] <= 999987.7842112777
 R8171: C34 + 1e+06 alpha_ob40_seg17[2] <= 1.000076834799801e+06
 R8172: C36 + 1e+06 alpha_ob40_seg17[2] <= 1.000076834799801e+06
 R8173: C35 + 1e+06 alpha_ob40_seg17[3] <= 1.0000096658112778e+06
 R8174: C37 + 1e+06 alpha_ob40_seg17[3] <= 1.0000096658112778e+06
 R8175: alpha_ob40_seg17[0] + alpha_ob40_seg17[1] + alpha_ob40_seg17[2]
   + alpha_ob40_seg17[3] >= 1
 R8176: - C34 + 1e+06 alpha_ob41_seg17[0] <= 999920.6152119379
 R8177: - C36 + 1e+06 alpha_ob41_seg17[0] <= 999920.6152119379
 R8178: - C35 + 1e+06 alpha_ob41_seg17[1] <= 999981.4341997246
 R8179: - C37 + 1e+06 alpha_ob41_seg17[1] <= 999981.4341997246
 R8180: C34 + 1e+06 alpha_ob41_seg17[2] <= 1.0000768348119379e+06
 R8181: C36 + 1e+06 alpha_ob41_seg17[2] <= 1.0000768348119379e+06
 R8182: C35 + 1e+06 alpha_ob41_seg17[3] <= 1.0000096657997245e+06
 R8183: C37 + 1e+06 alpha_ob41_seg17[3] <= 1.0000096657997245e+06
 R8184: alpha_ob41_seg17[0] + alpha_ob41_seg17[1] + alpha_ob41_seg17[2]
   + alpha_ob41_seg17[3] >= 1
 R8185: - C34 + 1e+06 alpha_ob42_seg17[0] <= 999919.2652
 R8186: - C36 + 1e+06 alpha_ob42_seg17[0] <= 999919.2652
 R8187: - C35 + 1e+06 alpha_ob42_seg17[1] <= 999981.4342
 R8188: - C37 + 1e+06 alpha_ob42_seg17[1] <= 999981.4342
 R8189: C34 + 1e+06 alpha_ob42_seg17[2] <= 1.0000768348e+06
 R8190: C36 + 1e+06 alpha_ob42_seg17[2] <= 1.0000768348e+06
 R8191: C35 + 1e+06 alpha_ob42_seg17[3] <= 1.0000160158000001e+06
 R8192: C37 + 1e+06 alpha_ob42_seg17[3] <= 1.0000160158000001e+06
 R8193: alpha_ob42_seg17[0] + alpha_ob42_seg17[1] + alpha_ob42_seg17[2]
   + alpha_ob42_seg17[3] >= 1
 R8194: - C34 + 1e+06 alpha_ob43_seg17[0] <= 999919.2652454559
 R8195: - C36 + 1e+06 alpha_ob43_seg17[0] <= 999919.2652454559
 R8196: - C35 + 1e+06 alpha_ob43_seg17[1] <= 999956.8341997246
 R8197: - C37 + 1e+06 alpha_ob43_seg17[1] <= 999956.8341997246
 R8198: C34 + 1e+06 alpha_ob43_seg17[2] <= 1.0000781848454559e+06
 R8199: C36 + 1e+06 alpha_ob43_seg17[2] <= 1.0000781848454559e+06
 R8200: C35 + 1e+06 alpha_ob43_seg17[3] <= 1.0000160157997246e+06
 R8201: C37 + 1e+06 alpha_ob43_seg17[3] <= 1.0000160157997246e+06
 R8202: alpha_ob43_seg17[0] + alpha_ob43_seg17[1] + alpha_ob43_seg17[2]
   + alpha_ob43_seg17[3] >= 1
 R8203: - C34 + 1e+06 alpha_ob44_seg17[0] <= 999991.83683
 R8204: - C36 + 1e+06 alpha_ob44_seg17[0] <= 999991.83683
 R8205: - C35 + 1e+06 alpha_ob44_seg17[1] <= 999940.589
 R8206: - C37 + 1e+06 alpha_ob44_seg17[1] <= 999940.589
 R8207: C34 + 1e+06 alpha_ob44_seg17[2] <= 1.00000401317e+06
 R8208: C36 + 1e+06 alpha_ob44_seg17[2] <= 1.00000401317e+06
 R8209: C35 + 1e+06 alpha_ob44_seg17[3] <= 1.000056861e+06
 R8210: C37 + 1e+06 alpha_ob44_seg17[3] <= 1.000056861e+06
 R8211: alpha_ob44_seg17[0] + alpha_ob44_seg17[1] + alpha_ob44_seg17[2]
   + alpha_ob44_seg17[3] >= 1
 R8212: - C34 + 1e+06 alpha_ob45_seg17[0] <= 999991.8368084199
 R8213: - C36 + 1e+06 alpha_ob45_seg17[0] <= 999991.8368084199
 R8214: - C35 + 1e+06 alpha_ob45_seg17[1] <= 999940.5890002755
 R8215: - C37 + 1e+06 alpha_ob45_seg17[1] <= 999940.5890002755
 R8216: C34 + 1e+06 alpha_ob45_seg17[2] <= 1.0000056131484199e+06
 R8217: C36 + 1e+06 alpha_ob45_seg17[2] <= 1.0000056131484199e+06
 R8218: C35 + 1e+06 alpha_ob45_seg17[3] <= 1.0000452610002755e+06
 R8219: C37 + 1e+06 alpha_ob45_seg17[3] <= 1.0000452610002755e+06
 R8220: alpha_ob45_seg17[0] + alpha_ob45_seg17[1] + alpha_ob45_seg17[2]
   + alpha_ob45_seg17[3] >= 1
 R8221: - C34 + 1e+06 alpha_ob46_seg17[0] <= 1.0000764021508709e+06
 R8222: - C36 + 1e+06 alpha_ob46_seg17[0] <= 1.0000764021508709e+06
 R8223: - C35 + 1e+06 alpha_ob46_seg17[1] <= 999957.1027002756
 R8224: - C37 + 1e+06 alpha_ob46_seg17[1] <= 999957.1027002756
 R8225: C34 + 1e+06 alpha_ob46_seg17[2] <= 999921.0477508709
 R8226: C36 + 1e+06 alpha_ob46_seg17[2] <= 999921.0477508709
 R8227: C35 + 1e+06 alpha_ob46_seg17[3] <= 1.0000137473002756e+06
 R8228: C37 + 1e+06 alpha_ob46_seg17[3] <= 1.0000137473002756e+06
 R8229: alpha_ob46_seg17[0] + alpha_ob46_seg17[1] + alpha_ob46_seg17[2]
   + alpha_ob46_seg17[3] >= 1
 R8230: - C34 + 1e+06 alpha_ob47_seg17[0] <= 1.0000445522e+06
 R8231: - C36 + 1e+06 alpha_ob47_seg17[0] <= 1.0000445522e+06
 R8232: - C35 + 1e+06 alpha_ob47_seg17[1] <= 999983.7027
 R8233: - C37 + 1e+06 alpha_ob47_seg17[1] <= 999983.7027
 R8234: C34 + 1e+06 alpha_ob47_seg17[2] <= 999921.0477999999
 R8235: C36 + 1e+06 alpha_ob47_seg17[2] <= 999921.0477999999
 R8236: C35 + 1e+06 alpha_ob47_seg17[3] <= 1.0000137473e+06
 R8237: C37 + 1e+06 alpha_ob47_seg17[3] <= 1.0000137473e+06
 R8238: alpha_ob47_seg17[0] + alpha_ob47_seg17[1] + alpha_ob47_seg17[2]
   + alpha_ob47_seg17[3] >= 1
 R8239: - C34 + 1e+06 alpha_ob48_seg17[0] <= 1.0000445522183661e+06
 R8240: - C36 + 1e+06 alpha_ob48_seg17[0] <= 1.0000445522183661e+06
 R8241: - C35 + 1e+06 alpha_ob48_seg17[1] <= 999973.8526997245
 R8242: - C37 + 1e+06 alpha_ob48_seg17[1] <= 999973.8526997245
 R8243: C34 + 1e+06 alpha_ob48_seg17[2] <= 999952.8978183661
 R8244: C36 + 1e+06 alpha_ob48_seg17[2] <= 999952.8978183661
 R8245: C35 + 1e+06 alpha_ob48_seg17[3] <= 1.0000137472997245e+06
 R8246: C37 + 1e+06 alpha_ob48_seg17[3] <= 1.0000137472997245e+06
 R8247: alpha_ob48_seg17[0] + alpha_ob48_seg17[1] + alpha_ob48_seg17[2]
   + alpha_ob48_seg17[3] >= 1
 R8248: - C34 + 1e+06 alpha_ob49_seg17[0] <= 1.000064332081634e+06
 R8249: - C36 + 1e+06 alpha_ob49_seg17[0] <= 1.000064332081634e+06
 R8250: - C35 + 1e+06 alpha_ob49_seg17[1] <= 999973.9401002756
 R8251: - C37 + 1e+06 alpha_ob49_seg17[1] <= 999973.9401002756
 R8252: C34 + 1e+06 alpha_ob49_seg17[2] <= 999933.1178816339
 R8253: C36 + 1e+06 alpha_ob49_seg17[2] <= 999933.1178816339
 R8254: C35 + 1e+06 alpha_ob49_seg17[3] <= 1.0000136599002755e+06
 R8255: C37 + 1e+06 alpha_ob49_seg17[3] <= 1.0000136599002755e+06
 R8256: alpha_ob49_seg17[0] + alpha_ob49_seg17[1] + alpha_ob49_seg17[2]
   + alpha_ob49_seg17[3] >= 1
 R8257: - C34 + 1e+06 alpha_ob50_seg17[0] <= 1.0000643321013774e+06
 R8258: - C36 + 1e+06 alpha_ob50_seg17[0] <= 1.0000643321013774e+06
 R8259: - C35 + 1e+06 alpha_ob50_seg17[1] <= 999983.1900997246
 R8260: - C37 + 1e+06 alpha_ob50_seg17[1] <= 999983.1900997246
 R8261: C34 + 1e+06 alpha_ob50_seg17[2] <= 999933.1179013775
 R8262: C36 + 1e+06 alpha_ob50_seg17[2] <= 999933.1179013775
 R8263: C35 + 1e+06 alpha_ob50_seg17[3] <= 1.0000136598997245e+06
 R8264: C37 + 1e+06 alpha_ob50_seg17[3] <= 1.0000136598997245e+06
 R8265: alpha_ob50_seg17[0] + alpha_ob50_seg17[1] + alpha_ob50_seg17[2]
   + alpha_ob50_seg17[3] >= 1
 R8266: - C36 + 1e+06 alpha_ob0_seg18[0] <= 1.0000284170499525e+06
 R8267: - C38 + 1e+06 alpha_ob0_seg18[0] <= 1.0000284170499525e+06
 R8268: - C37 + 1e+06 alpha_ob0_seg18[1] <= 999957.0843002755
 R8269: - C39 + 1e+06 alpha_ob0_seg18[1] <= 999957.0843002755
 R8270: C36 + 1e+06 alpha_ob0_seg18[2] <= 999969.0328499526
 R8271: C38 + 1e+06 alpha_ob0_seg18[2] <= 999969.0328499526
 R8272: C37 + 1e+06 alpha_ob0_seg18[3] <= 1.0000132657002755e+06
 R8273: C39 + 1e+06 alpha_ob0_seg18[3] <= 1.0000132657002755e+06
 R8274: alpha_ob0_seg18[0] + alpha_ob0_seg18[1] + alpha_ob0_seg18[2]
   + alpha_ob0_seg18[3] >= 1
 R8275: - C36 + 1e+06 alpha_ob1_seg18[0] <= 1.000028417099801e+06
 R8276: - C38 + 1e+06 alpha_ob1_seg18[0] <= 1.000028417099801e+06
 R8277: - C37 + 1e+06 alpha_ob1_seg18[1] <= 999984.1843155898
 R8278: - C39 + 1e+06 alpha_ob1_seg18[1] <= 999984.1843155898
 R8279: C36 + 1e+06 alpha_ob1_seg18[2] <= 999957.4328998009
 R8280: C38 + 1e+06 alpha_ob1_seg18[2] <= 999957.4328998009
 R8281: C37 + 1e+06 alpha_ob1_seg18[3] <= 1.0000132657155899e+06
 R8282: C39 + 1e+06 alpha_ob1_seg18[3] <= 1.0000132657155899e+06
 R8283: alpha_ob1_seg18[0] + alpha_ob1_seg18[1] + alpha_ob1_seg18[2]
   + alpha_ob1_seg18[3] >= 1
 R8284: - C36 + 1e+06 alpha_ob2_seg18[0] <= 1.0000400171500475e+06
 R8285: - C38 + 1e+06 alpha_ob2_seg18[0] <= 1.0000400171500475e+06
 R8286: - C37 + 1e+06 alpha_ob2_seg18[1] <= 999957.0842997245
 R8287: - C39 + 1e+06 alpha_ob2_seg18[1] <= 999957.0842997245
 R8288: C36 + 1e+06 alpha_ob2_seg18[2] <= 999957.4329500474
 R8289: C38 + 1e+06 alpha_ob2_seg18[2] <= 999957.4329500474
 R8290: C37 + 1e+06 alpha_ob2_seg18[3] <= 1.0000132656997246e+06
 R8291: C39 + 1e+06 alpha_ob2_seg18[3] <= 1.0000132656997246e+06
 R8292: alpha_ob2_seg18[0] + alpha_ob2_seg18[1] + alpha_ob2_seg18[2]
   + alpha_ob2_seg18[3] >= 1
 R8293: - C36 + 1e+06 alpha_ob3_seg18[0] <= 1.0000109161752058e+06
 R8294: - C38 + 1e+06 alpha_ob3_seg18[0] <= 1.0000109161752058e+06
 R8295: - C37 + 1e+06 alpha_ob3_seg18[1] <= 999970.2681002755
 R8296: - C39 + 1e+06 alpha_ob3_seg18[1] <= 999970.2681002755
 R8297: C36 + 1e+06 alpha_ob3_seg18[2] <= 999986.5337752058
 R8298: C38 + 1e+06 alpha_ob3_seg18[2] <= 999986.5337752058
 R8299: C37 + 1e+06 alpha_ob3_seg18[3] <= 1.0000138319002755e+06
 R8300: C39 + 1e+06 alpha_ob3_seg18[3] <= 1.0000138319002755e+06
 R8301: alpha_ob3_seg18[0] + alpha_ob3_seg18[1] + alpha_ob3_seg18[2]
   + alpha_ob3_seg18[3] >= 1
 R8302: - C36 + 1e+06 alpha_ob4_seg18[0] <= 1.000010916199801e+06
 R8303: - C38 + 1e+06 alpha_ob4_seg18[0] <= 1.000010916199801e+06
 R8304: - C37 + 1e+06 alpha_ob4_seg18[1] <= 999983.6181179117
 R8305: - C39 + 1e+06 alpha_ob4_seg18[1] <= 999983.6181179117
 R8306: C36 + 1e+06 alpha_ob4_seg18[2] <= 999973.183799801
 R8307: C38 + 1e+06 alpha_ob4_seg18[2] <= 999973.183799801
 R8308: C37 + 1e+06 alpha_ob4_seg18[3] <= 1.0000138319179118e+06
 R8309: C39 + 1e+06 alpha_ob4_seg18[3] <= 1.0000138319179118e+06
 R8310: alpha_ob4_seg18[0] + alpha_ob4_seg18[1] + alpha_ob4_seg18[2]
   + alpha_ob4_seg18[3] >= 1
 R8311: - C36 + 1e+06 alpha_ob5_seg18[0] <= 999992.49102
 R8312: - C38 + 1e+06 alpha_ob5_seg18[0] <= 999992.49102
 R8313: - C37 + 1e+06 alpha_ob5_seg18[1] <= 999957.1016000001
 R8314: - C39 + 1e+06 alpha_ob5_seg18[1] <= 999957.1016000001
 R8315: C36 + 1e+06 alpha_ob5_seg18[2] <= 999986.35898
 R8316: C38 + 1e+06 alpha_ob5_seg18[2] <= 999986.35898
 R8317: C37 + 1e+06 alpha_ob5_seg18[3] <= 1.0000403484e+06
 R8318: C39 + 1e+06 alpha_ob5_seg18[3] <= 1.0000403484e+06
 R8319: alpha_ob5_seg18[0] + alpha_ob5_seg18[1] + alpha_ob5_seg18[2]
   + alpha_ob5_seg18[3] >= 1
 R8320: - C36 + 1e+06 alpha_ob6_seg18[0] <= 999981.3909999999
 R8321: - C38 + 1e+06 alpha_ob6_seg18[0] <= 999981.3909999999
 R8322: - C37 + 1e+06 alpha_ob6_seg18[1] <= 999957.1016000001
 R8323: - C39 + 1e+06 alpha_ob6_seg18[1] <= 999957.1016000001
 R8324: C36 + 1e+06 alpha_ob6_seg18[2] <= 1.000004959e+06
 R8325: C38 + 1e+06 alpha_ob6_seg18[2] <= 1.000004959e+06
 R8326: C37 + 1e+06 alpha_ob6_seg18[3] <= 1.0000403484e+06
 R8327: C39 + 1e+06 alpha_ob6_seg18[3] <= 1.0000403484e+06
 R8328: alpha_ob6_seg18[0] + alpha_ob6_seg18[1] + alpha_ob6_seg18[2]
   + alpha_ob6_seg18[3] >= 1
 R8329: - C36 + 1e+06 alpha_ob7_seg18[0] <= 999981.3909825523
 R8330: - C38 + 1e+06 alpha_ob7_seg18[0] <= 999981.3909825523
 R8331: - C37 + 1e+06 alpha_ob7_seg18[1] <= 999957.1016002755
 R8332: - C39 + 1e+06 alpha_ob7_seg18[1] <= 999957.1016002755
 R8333: C36 + 1e+06 alpha_ob7_seg18[2] <= 1.0000160589825523e+06
 R8334: C38 + 1e+06 alpha_ob7_seg18[2] <= 1.0000160589825523e+06
 R8335: C37 + 1e+06 alpha_ob7_seg18[3] <= 1.0000309984002755e+06
 R8336: C39 + 1e+06 alpha_ob7_seg18[3] <= 1.0000309984002755e+06
 R8337: alpha_ob7_seg18[0] + alpha_ob7_seg18[1] + alpha_ob7_seg18[2]
   + alpha_ob7_seg18[3] >= 1
 R8338: - C36 + 1e+06 alpha_ob8_seg18[0] <= 999981.3867954084
 R8339: - C38 + 1e+06 alpha_ob8_seg18[0] <= 999981.3867954084
 R8340: - C37 + 1e+06 alpha_ob8_seg18[1] <= 999968.8595002755
 R8341: - C39 + 1e+06 alpha_ob8_seg18[1] <= 999968.8595002755
 R8342: C36 + 1e+06 alpha_ob8_seg18[2] <= 1.0000160631954084e+06
 R8343: C38 + 1e+06 alpha_ob8_seg18[2] <= 1.0000160631954084e+06
 R8344: C37 + 1e+06 alpha_ob8_seg18[3] <= 1.0000262405002755e+06
 R8345: C39 + 1e+06 alpha_ob8_seg18[3] <= 1.0000262405002755e+06
 R8346: alpha_ob8_seg18[0] + alpha_ob8_seg18[1] + alpha_ob8_seg18[2]
   + alpha_ob8_seg18[3] >= 1
 R8347: - C36 + 1e+06 alpha_ob9_seg18[0] <= 999981.386799801
 R8348: - C38 + 1e+06 alpha_ob9_seg18[0] <= 999981.386799801
 R8349: - C37 + 1e+06 alpha_ob9_seg18[1] <= 999971.2095013268
 R8350: - C39 + 1e+06 alpha_ob9_seg18[1] <= 999971.2095013268
 R8351: C36 + 1e+06 alpha_ob9_seg18[2] <= 1.0000152131998009e+06
 R8352: C38 + 1e+06 alpha_ob9_seg18[2] <= 1.0000152131998009e+06
 R8353: C37 + 1e+06 alpha_ob9_seg18[3] <= 1.0000262405013267e+06
 R8354: C39 + 1e+06 alpha_ob9_seg18[3] <= 1.0000262405013267e+06
 R8355: alpha_ob9_seg18[0] + alpha_ob9_seg18[1] + alpha_ob9_seg18[2]
   + alpha_ob9_seg18[3] >= 1
 R8356: - C36 + 1e+06 alpha_ob10_seg18[0] <= 999982.2368022958
 R8357: - C38 + 1e+06 alpha_ob10_seg18[0] <= 999982.2368022958
 R8358: - C37 + 1e+06 alpha_ob10_seg18[1] <= 999970.1094997245
 R8359: - C39 + 1e+06 alpha_ob10_seg18[1] <= 999970.1094997245
 R8360: C36 + 1e+06 alpha_ob10_seg18[2] <= 1.0000152132022957e+06
 R8361: C38 + 1e+06 alpha_ob10_seg18[2] <= 1.0000152132022957e+06
 R8362: C37 + 1e+06 alpha_ob10_seg18[3] <= 1.0000262404997245e+06
 R8363: C39 + 1e+06 alpha_ob10_seg18[3] <= 1.0000262404997245e+06
 R8364: alpha_ob10_seg18[0] + alpha_ob10_seg18[1] + alpha_ob10_seg18[2]
   + alpha_ob10_seg18[3] >= 1
 R8365: - C36 + 1e+06 alpha_ob11_seg18[0] <= 999982.236799801
 R8366: - C38 + 1e+06 alpha_ob11_seg18[0] <= 999982.236799801
 R8367: - C37 + 1e+06 alpha_ob11_seg18[1] <= 999970.1095291895
 R8368: - C39 + 1e+06 alpha_ob11_seg18[1] <= 999970.1095291895
 R8369: C36 + 1e+06 alpha_ob11_seg18[2] <= 999993.3631998011
 R8370: C38 + 1e+06 alpha_ob11_seg18[2] <= 999993.3631998011
 R8371: C37 + 1e+06 alpha_ob11_seg18[3] <= 1.0000273405291894e+06
 R8372: C39 + 1e+06 alpha_ob11_seg18[3] <= 1.0000273405291894e+06
 R8373: alpha_ob11_seg18[0] + alpha_ob11_seg18[1] + alpha_ob11_seg18[2]
   + alpha_ob11_seg18[3] >= 1
 R8374: - C36 + 1e+06 alpha_ob12_seg18[0] <= 999981.6050802565
 R8375: - C38 + 1e+06 alpha_ob12_seg18[0] <= 999981.6050802565
 R8376: - C37 + 1e+06 alpha_ob12_seg18[1] <= 999972.8942002755
 R8377: - C39 + 1e+06 alpha_ob12_seg18[1] <= 999972.8942002755
 R8378: C36 + 1e+06 alpha_ob12_seg18[2] <= 1.0000158448802566e+06
 R8379: C38 + 1e+06 alpha_ob12_seg18[2] <= 1.0000158448802566e+06
 R8380: C37 + 1e+06 alpha_ob12_seg18[3] <= 1.0000139558002756e+06
 R8381: C39 + 1e+06 alpha_ob12_seg18[3] <= 1.0000139558002756e+06
 R8382: alpha_ob12_seg18[0] + alpha_ob12_seg18[1] + alpha_ob12_seg18[2]
   + alpha_ob12_seg18[3] >= 1
 R8383: - C36 + 1e+06 alpha_ob13_seg18[0] <= 999981.605149801
 R8384: - C38 + 1e+06 alpha_ob13_seg18[0] <= 999981.605149801
 R8385: - C37 + 1e+06 alpha_ob13_seg18[1] <= 999983.4942391404
 R8386: - C39 + 1e+06 alpha_ob13_seg18[1] <= 999983.4942391404
 R8387: C36 + 1e+06 alpha_ob13_seg18[2] <= 999986.4948498011
 R8388: C38 + 1e+06 alpha_ob13_seg18[2] <= 999986.4948498011
 R8389: C37 + 1e+06 alpha_ob13_seg18[3] <= 1.0000139558391405e+06
 R8390: C39 + 1e+06 alpha_ob13_seg18[3] <= 1.0000139558391405e+06
 R8391: alpha_ob13_seg18[0] + alpha_ob13_seg18[1] + alpha_ob13_seg18[2]
   + alpha_ob13_seg18[3] >= 1
 R8392: - C36 + 1e+06 alpha_ob14_seg18[0] <= 999997.6803502565
 R8393: - C38 + 1e+06 alpha_ob14_seg18[0] <= 999997.6803502565
 R8394: - C37 + 1e+06 alpha_ob14_seg18[1] <= 999972.7268002755
 R8395: - C39 + 1e+06 alpha_ob14_seg18[1] <= 999972.7268002755
 R8396: C36 + 1e+06 alpha_ob14_seg18[2] <= 999999.7696102565
 R8397: C38 + 1e+06 alpha_ob14_seg18[2] <= 999999.7696102565
 R8398: C37 + 1e+06 alpha_ob14_seg18[3] <= 1.0000141232002755e+06
 R8399: C39 + 1e+06 alpha_ob14_seg18[3] <= 1.0000141232002755e+06
 R8400: alpha_ob14_seg18[0] + alpha_ob14_seg18[1] + alpha_ob14_seg18[2]
   + alpha_ob14_seg18[3] >= 1
 R8401: - C36 + 1e+06 alpha_ob15_seg18[0] <= 1.0000038341502565e+06
 R8402: - C38 + 1e+06 alpha_ob15_seg18[0] <= 1.0000038341502565e+06
 R8403: - C37 + 1e+06 alpha_ob15_seg18[1] <= 999972.6012002755
 R8404: - C39 + 1e+06 alpha_ob15_seg18[1] <= 999972.6012002755
 R8405: C36 + 1e+06 alpha_ob15_seg18[2] <= 999993.6158102566
 R8406: C38 + 1e+06 alpha_ob15_seg18[2] <= 999993.6158102566
 R8407: C37 + 1e+06 alpha_ob15_seg18[3] <= 1.0000142488002755e+06
 R8408: C39 + 1e+06 alpha_ob15_seg18[3] <= 1.0000142488002755e+06
 R8409: alpha_ob15_seg18[0] + alpha_ob15_seg18[1] + alpha_ob15_seg18[2]
   + alpha_ob15_seg18[3] >= 1
 R8410: - C36 + 1e+06 alpha_ob16_seg18[0] <= 1.0000162874063333e+06
 R8411: - C38 + 1e+06 alpha_ob16_seg18[0] <= 1.0000162874063333e+06
 R8412: - C37 + 1e+06 alpha_ob16_seg18[1] <= 999982.9439602756
 R8413: - C39 + 1e+06 alpha_ob16_seg18[1] <= 999982.9439602756
 R8414: C36 + 1e+06 alpha_ob16_seg18[2] <= 999981.1624063333
 R8415: C38 + 1e+06 alpha_ob16_seg18[2] <= 999981.1624063333
 R8416: C37 + 1e+06 alpha_ob16_seg18[3] <= 999963.6560402757
 R8417: C39 + 1e+06 alpha_ob16_seg18[3] <= 999963.6560402757
 R8418: alpha_ob16_seg18[0] + alpha_ob16_seg18[1] + alpha_ob16_seg18[2]
   + alpha_ob16_seg18[3] >= 1
 R8419: - C36 + 1e+06 alpha_ob17_seg18[0] <= 999980.937509
 R8420: - C38 + 1e+06 alpha_ob17_seg18[0] <= 999980.937509
 R8421: - C37 + 1e+06 alpha_ob17_seg18[1] <= 1.000034894e+06
 R8422: - C39 + 1e+06 alpha_ob17_seg18[1] <= 1.000034894e+06
 R8423: C36 + 1e+06 alpha_ob17_seg18[2] <= 999981.162491
 R8424: C38 + 1e+06 alpha_ob17_seg18[2] <= 999981.162491
 R8425: C37 + 1e+06 alpha_ob17_seg18[3] <= 999962.556
 R8426: C39 + 1e+06 alpha_ob17_seg18[3] <= 999962.556
 R8427: alpha_ob17_seg18[0] + alpha_ob17_seg18[1] + alpha_ob17_seg18[2]
   + alpha_ob17_seg18[3] >= 1
 R8428: - C36 + 1e+06 alpha_ob18_seg18[0] <= 999990.2670277582
 R8429: - C38 + 1e+06 alpha_ob18_seg18[0] <= 999990.2670277582
 R8430: - C37 + 1e+06 alpha_ob18_seg18[1] <= 1.0000347702002756e+06
 R8431: - C39 + 1e+06 alpha_ob18_seg18[1] <= 1.0000347702002756e+06
 R8432: C36 + 1e+06 alpha_ob18_seg18[2] <= 1.0000071828877581e+06
 R8433: C38 + 1e+06 alpha_ob18_seg18[2] <= 1.0000071828877581e+06
 R8434: C37 + 1e+06 alpha_ob18_seg18[3] <= 999939.8298002756
 R8435: C39 + 1e+06 alpha_ob18_seg18[3] <= 999939.8298002756
 R8436: alpha_ob18_seg18[0] + alpha_ob18_seg18[1] + alpha_ob18_seg18[2]
   + alpha_ob18_seg18[3] >= 1
 R8437: - C36 + 1e+06 alpha_ob19_seg18[0] <= 999990.267069801
 R8438: - C38 + 1e+06 alpha_ob19_seg18[0] <= 999990.267069801
 R8439: - C37 + 1e+06 alpha_ob19_seg18[1] <= 1.0000576202069657e+06
 R8440: - C39 + 1e+06 alpha_ob19_seg18[1] <= 1.0000576202069657e+06
 R8441: C36 + 1e+06 alpha_ob19_seg18[2] <= 1.000002082929801e+06
 R8442: C38 + 1e+06 alpha_ob19_seg18[2] <= 1.000002082929801e+06
 R8443: C37 + 1e+06 alpha_ob19_seg18[3] <= 999939.8298069657
 R8444: C39 + 1e+06 alpha_ob19_seg18[3] <= 999939.8298069657
 R8445: alpha_ob19_seg18[0] + alpha_ob19_seg18[1] + alpha_ob19_seg18[2]
   + alpha_ob19_seg18[3] >= 1
 R8446: - C36 + 1e+06 alpha_ob20_seg18[0] <= 999991.073732
 R8447: - C38 + 1e+06 alpha_ob20_seg18[0] <= 999991.073732
 R8448: - C37 + 1e+06 alpha_ob20_seg18[1] <= 999952.2125
 R8449: - C39 + 1e+06 alpha_ob20_seg18[1] <= 999952.2125
 R8450: C36 + 1e+06 alpha_ob20_seg18[2] <= 999990.776268
 R8451: C38 + 1e+06 alpha_ob20_seg18[2] <= 999990.776268
 R8452: C37 + 1e+06 alpha_ob20_seg18[3] <= 1.0000452375e+06
 R8453: C39 + 1e+06 alpha_ob20_seg18[3] <= 1.0000452375e+06
 R8454: alpha_ob20_seg18[0] + alpha_ob20_seg18[1] + alpha_ob20_seg18[2]
   + alpha_ob20_seg18[3] >= 1
 R8455: - C36 + 1e+06 alpha_ob21_seg18[0] <= 999985.657099801
 R8456: - C38 + 1e+06 alpha_ob21_seg18[0] <= 999985.657099801
 R8457: - C37 + 1e+06 alpha_ob21_seg18[1] <= 1.0000576202063023e+06
 R8458: - C39 + 1e+06 alpha_ob21_seg18[1] <= 1.0000576202063023e+06
 R8459: C36 + 1e+06 alpha_ob21_seg18[2] <= 1.000007192899801e+06
 R8460: C38 + 1e+06 alpha_ob21_seg18[2] <= 1.000007192899801e+06
 R8461: C37 + 1e+06 alpha_ob21_seg18[3] <= 999939.8298063023
 R8462: C39 + 1e+06 alpha_ob21_seg18[3] <= 999939.8298063023
 R8463: alpha_ob21_seg18[0] + alpha_ob21_seg18[1] + alpha_ob21_seg18[2]
   + alpha_ob21_seg18[3] >= 1
 R8464: - C36 + 1e+06 alpha_ob22_seg18[0] <= 999994.771829801
 R8465: - C38 + 1e+06 alpha_ob22_seg18[0] <= 999994.771829801
 R8466: - C37 + 1e+06 alpha_ob22_seg18[1] <= 1.0000615983063023e+06
 R8467: - C39 + 1e+06 alpha_ob22_seg18[1] <= 1.0000615983063023e+06
 R8468: C36 + 1e+06 alpha_ob22_seg18[2] <= 999998.078169801
 R8469: C38 + 1e+06 alpha_ob22_seg18[2] <= 999998.078169801
 R8470: C37 + 1e+06 alpha_ob22_seg18[3] <= 999935.8517063023
 R8471: C39 + 1e+06 alpha_ob22_seg18[3] <= 999935.8517063023
 R8472: alpha_ob22_seg18[0] + alpha_ob22_seg18[1] + alpha_ob22_seg18[2]
   + alpha_ob22_seg18[3] >= 1
 R8473: - C36 + 1e+06 alpha_ob23_seg18[0] <= 999980.9374508709
 R8474: - C38 + 1e+06 alpha_ob23_seg18[0] <= 999980.9374508709
 R8475: - C37 + 1e+06 alpha_ob23_seg18[1] <= 1.0000348940002755e+06
 R8476: - C39 + 1e+06 alpha_ob23_seg18[1] <= 1.0000348940002755e+06
 R8477: C36 + 1e+06 alpha_ob23_seg18[2] <= 1.0000165124508708e+06
 R8478: C38 + 1e+06 alpha_ob23_seg18[2] <= 1.0000165124508708e+06
 R8479: C37 + 1e+06 alpha_ob23_seg18[3] <= 999935.9560002756
 R8480: C39 + 1e+06 alpha_ob23_seg18[3] <= 999935.9560002756
 R8481: alpha_ob23_seg18[0] + alpha_ob23_seg18[1] + alpha_ob23_seg18[2]
   + alpha_ob23_seg18[3] >= 1
 R8482: - C36 + 1e+06 alpha_ob24_seg18[0] <= 999980.937499801
 R8483: - C38 + 1e+06 alpha_ob24_seg18[0] <= 999980.937499801
 R8484: - C37 + 1e+06 alpha_ob24_seg18[1] <= 1.0000614940072973e+06
 R8485: - C39 + 1e+06 alpha_ob24_seg18[1] <= 1.0000614940072973e+06
 R8486: C36 + 1e+06 alpha_ob24_seg18[2] <= 1.000011162499801e+06
 R8487: C38 + 1e+06 alpha_ob24_seg18[2] <= 1.000011162499801e+06
 R8488: C37 + 1e+06 alpha_ob24_seg18[3] <= 999935.9560072974
 R8489: C39 + 1e+06 alpha_ob24_seg18[3] <= 999935.9560072974
 R8490: alpha_ob24_seg18[0] + alpha_ob24_seg18[1] + alpha_ob24_seg18[2]
   + alpha_ob24_seg18[3] >= 1
 R8491: - C36 + 1e+06 alpha_ob25_seg18[0] <= 999956.8394000001
 R8492: - C38 + 1e+06 alpha_ob25_seg18[0] <= 999956.8394000001
 R8493: - C37 + 1e+06 alpha_ob25_seg18[1] <= 999952.2463
 R8494: - C39 + 1e+06 alpha_ob25_seg18[1] <= 999952.2463
 R8495: C36 + 1e+06 alpha_ob25_seg18[2] <= 1.0000097606e+06
 R8496: C38 + 1e+06 alpha_ob25_seg18[2] <= 1.0000097606e+06
 R8497: C37 + 1e+06 alpha_ob25_seg18[3] <= 1.0000452037e+06
 R8498: C39 + 1e+06 alpha_ob25_seg18[3] <= 1.0000452037e+06
 R8499: alpha_ob25_seg18[0] + alpha_ob25_seg18[1] + alpha_ob25_seg18[2]
   + alpha_ob25_seg18[3] >= 1
 R8500: - C36 + 1e+06 alpha_ob26_seg18[0] <= 999980.9375348955
 R8501: - C38 + 1e+06 alpha_ob26_seg18[0] <= 999980.9375348955
 R8502: - C37 + 1e+06 alpha_ob26_seg18[1] <= 1.0000160439997246e+06
 R8503: - C39 + 1e+06 alpha_ob26_seg18[1] <= 1.0000160439997246e+06
 R8504: C36 + 1e+06 alpha_ob26_seg18[2] <= 1.0000165125348954e+06
 R8505: C38 + 1e+06 alpha_ob26_seg18[2] <= 1.0000165125348954e+06
 R8506: C37 + 1e+06 alpha_ob26_seg18[3] <= 999962.5559997245
 R8507: C39 + 1e+06 alpha_ob26_seg18[3] <= 999962.5559997245
 R8508: alpha_ob26_seg18[0] + alpha_ob26_seg18[1] + alpha_ob26_seg18[2]
   + alpha_ob26_seg18[3] >= 1
 R8509: - C36 + 1e+06 alpha_ob27_seg18[0] <= 999956.839399801
 R8510: - C38 + 1e+06 alpha_ob27_seg18[0] <= 999956.839399801
 R8511: - C37 + 1e+06 alpha_ob27_seg18[1] <= 999925.3963434525
 R8512: - C39 + 1e+06 alpha_ob27_seg18[1] <= 999925.3963434525
 R8513: C36 + 1e+06 alpha_ob27_seg18[2] <= 1.0000080105998011e+06
 R8514: C38 + 1e+06 alpha_ob27_seg18[2] <= 1.0000080105998011e+06
 R8515: C37 + 1e+06 alpha_ob27_seg18[3] <= 1.0000720537434525e+06
 R8516: C39 + 1e+06 alpha_ob27_seg18[3] <= 1.0000720537434525e+06
 R8517: alpha_ob27_seg18[0] + alpha_ob27_seg18[1] + alpha_ob27_seg18[2]
   + alpha_ob27_seg18[3] >= 1
 R8518: - C36 + 1e+06 alpha_ob28_seg18[0] <= 999980.9374998009
 R8519: - C38 + 1e+06 alpha_ob28_seg18[0] <= 999980.9374998009
 R8520: - C37 + 1e+06 alpha_ob28_seg18[1] <= 1.0000160440053071e+06
 R8521: - C39 + 1e+06 alpha_ob28_seg18[1] <= 1.0000160440053071e+06
 R8522: C36 + 1e+06 alpha_ob28_seg18[2] <= 1.000012662499801e+06
 R8523: C38 + 1e+06 alpha_ob28_seg18[2] <= 1.000012662499801e+06
 R8524: C37 + 1e+06 alpha_ob28_seg18[3] <= 999981.4060053072
 R8525: C39 + 1e+06 alpha_ob28_seg18[3] <= 999981.4060053072
 R8526: alpha_ob28_seg18[0] + alpha_ob28_seg18[1] + alpha_ob28_seg18[2]
   + alpha_ob28_seg18[3] >= 1
 R8527: - C36 + 1e+06 alpha_ob29_seg18[0] <= 999983.6053459151
 R8528: - C38 + 1e+06 alpha_ob29_seg18[0] <= 999983.6053459151
 R8529: - C37 + 1e+06 alpha_ob29_seg18[1] <= 999987.7722497246
 R8530: - C39 + 1e+06 alpha_ob29_seg18[1] <= 999987.7722497246
 R8531: C36 + 1e+06 alpha_ob29_seg18[2] <= 1.000013844745915e+06
 R8532: C38 + 1e+06 alpha_ob29_seg18[2] <= 1.000013844745915e+06
 R8533: C37 + 1e+06 alpha_ob29_seg18[3] <= 999984.8277497246
 R8534: C39 + 1e+06 alpha_ob29_seg18[3] <= 999984.8277497246
 R8535: alpha_ob29_seg18[0] + alpha_ob29_seg18[1] + alpha_ob29_seg18[2]
   + alpha_ob29_seg18[3] >= 1
 R8536: - C36 + 1e+06 alpha_ob30_seg18[0] <= 999958.8717898987
 R8537: - C38 + 1e+06 alpha_ob30_seg18[0] <= 999958.8717898987
 R8538: - C37 + 1e+06 alpha_ob30_seg18[1] <= 999987.7837602755
 R8539: - C39 + 1e+06 alpha_ob30_seg18[1] <= 999987.7837602755
 R8540: C36 + 1e+06 alpha_ob30_seg18[2] <= 1.0000385781898986e+06
 R8541: C38 + 1e+06 alpha_ob30_seg18[2] <= 1.0000385781898986e+06
 R8542: C37 + 1e+06 alpha_ob30_seg18[3] <= 1.0000043162402755e+06
 R8543: C39 + 1e+06 alpha_ob30_seg18[3] <= 1.0000043162402755e+06
 R8544: alpha_ob30_seg18[0] + alpha_ob30_seg18[1] + alpha_ob30_seg18[2]
   + alpha_ob30_seg18[3] >= 1
 R8545: - C36 + 1e+06 alpha_ob31_seg18[0] <= 999939.2718
 R8546: - C38 + 1e+06 alpha_ob31_seg18[0] <= 999939.2718
 R8547: - C37 + 1e+06 alpha_ob31_seg18[1] <= 999993.13376
 R8548: - C39 + 1e+06 alpha_ob31_seg18[1] <= 999993.13376
 R8549: C36 + 1e+06 alpha_ob31_seg18[2] <= 1.0000385782e+06
 R8550: C38 + 1e+06 alpha_ob31_seg18[2] <= 1.0000385782e+06
 R8551: C37 + 1e+06 alpha_ob31_seg18[3] <= 1.00000431624e+06
 R8552: C39 + 1e+06 alpha_ob31_seg18[3] <= 1.00000431624e+06
 R8553: alpha_ob31_seg18[0] + alpha_ob31_seg18[1] + alpha_ob31_seg18[2]
   + alpha_ob31_seg18[3] >= 1
 R8554: - C36 + 1e+06 alpha_ob32_seg18[0] <= 999939.2718101013
 R8555: - C38 + 1e+06 alpha_ob32_seg18[0] <= 999939.2718101013
 R8556: - C37 + 1e+06 alpha_ob32_seg18[1] <= 999987.7837597246
 R8557: - C39 + 1e+06 alpha_ob32_seg18[1] <= 999987.7837597246
 R8558: C36 + 1e+06 alpha_ob32_seg18[2] <= 1.0000581782101013e+06
 R8559: C38 + 1e+06 alpha_ob32_seg18[2] <= 1.0000581782101013e+06
 R8560: C37 + 1e+06 alpha_ob32_seg18[3] <= 1.0000043162397245e+06
 R8561: C39 + 1e+06 alpha_ob32_seg18[3] <= 1.0000043162397245e+06
 R8562: alpha_ob32_seg18[0] + alpha_ob32_seg18[1] + alpha_ob32_seg18[2]
   + alpha_ob32_seg18[3] >= 1
 R8563: - C36 + 1e+06 alpha_ob33_seg18[0] <= 999939.271799801
 R8564: - C38 + 1e+06 alpha_ob33_seg18[0] <= 999939.271799801
 R8565: - C37 + 1e+06 alpha_ob33_seg18[1] <= 999987.7838262042
 R8566: - C39 + 1e+06 alpha_ob33_seg18[1] <= 999987.7838262042
 R8567: C36 + 1e+06 alpha_ob33_seg18[2] <= 1.000038578199801e+06
 R8568: C38 + 1e+06 alpha_ob33_seg18[2] <= 1.000038578199801e+06
 R8569: C37 + 1e+06 alpha_ob33_seg18[3] <= 1.0000096662262043e+06
 R8570: C39 + 1e+06 alpha_ob33_seg18[3] <= 1.0000096662262043e+06
 R8571: alpha_ob33_seg18[0] + alpha_ob33_seg18[1] + alpha_ob33_seg18[2]
   + alpha_ob33_seg18[3] >= 1
 R8572: - C36 + 1e+06 alpha_ob34_seg18[0] <= 999926.0708
 R8573: - C38 + 1e+06 alpha_ob34_seg18[0] <= 999926.0708
 R8574: - C37 + 1e+06 alpha_ob34_seg18[1] <= 999983.7196
 R8575: - C39 + 1e+06 alpha_ob34_seg18[1] <= 999983.7196
 R8576: C36 + 1e+06 alpha_ob34_seg18[2] <= 1.0000250292e+06
 R8577: C38 + 1e+06 alpha_ob34_seg18[2] <= 1.0000250292e+06
 R8578: C37 + 1e+06 alpha_ob34_seg18[3] <= 1.0000137304e+06
 R8579: C39 + 1e+06 alpha_ob34_seg18[3] <= 1.0000137304e+06
 R8580: alpha_ob34_seg18[0] + alpha_ob34_seg18[1] + alpha_ob34_seg18[2]
   + alpha_ob34_seg18[3] >= 1
 R8581: - C36 + 1e+06 alpha_ob35_seg18[0] <= 999926.0708293857
 R8582: - C38 + 1e+06 alpha_ob35_seg18[0] <= 999926.0708293857
 R8583: - C37 + 1e+06 alpha_ob35_seg18[1] <= 999967.8695997245
 R8584: - C39 + 1e+06 alpha_ob35_seg18[1] <= 999967.8695997245
 R8585: C36 + 1e+06 alpha_ob35_seg18[2] <= 1.0000713792293856e+06
 R8586: C38 + 1e+06 alpha_ob35_seg18[2] <= 1.0000713792293856e+06
 R8587: C37 + 1e+06 alpha_ob35_seg18[3] <= 1.0000137303997246e+06
 R8588: C39 + 1e+06 alpha_ob35_seg18[3] <= 1.0000137303997246e+06
 R8589: alpha_ob35_seg18[0] + alpha_ob35_seg18[1] + alpha_ob35_seg18[2]
   + alpha_ob35_seg18[3] >= 1
 R8590: - C36 + 1e+06 alpha_ob36_seg18[0] <= 999972.6917508709
 R8591: - C38 + 1e+06 alpha_ob36_seg18[0] <= 999972.6917508709
 R8592: - C37 + 1e+06 alpha_ob36_seg18[1] <= 999957.1644002756
 R8593: - C39 + 1e+06 alpha_ob36_seg18[1] <= 999957.1644002756
 R8594: C36 + 1e+06 alpha_ob36_seg18[2] <= 1.0000247581508709e+06
 R8595: C38 + 1e+06 alpha_ob36_seg18[2] <= 1.0000247581508709e+06
 R8596: C37 + 1e+06 alpha_ob36_seg18[3] <= 1.0000136856002755e+06
 R8597: C39 + 1e+06 alpha_ob36_seg18[3] <= 1.0000136856002755e+06
 R8598: alpha_ob36_seg18[0] + alpha_ob36_seg18[1] + alpha_ob36_seg18[2]
   + alpha_ob36_seg18[3] >= 1
 R8599: - C36 + 1e+06 alpha_ob37_seg18[0] <= 999939.0009674003
 R8600: - C38 + 1e+06 alpha_ob37_seg18[0] <= 999939.0009674003
 R8601: - C37 + 1e+06 alpha_ob37_seg18[1] <= 999961.9516002756
 R8602: - C39 + 1e+06 alpha_ob37_seg18[1] <= 999961.9516002756
 R8603: C36 + 1e+06 alpha_ob37_seg18[2] <= 1.0000584489674004e+06
 R8604: C38 + 1e+06 alpha_ob37_seg18[2] <= 1.0000584489674004e+06
 R8605: C37 + 1e+06 alpha_ob37_seg18[3] <= 1.0000178984002755e+06
 R8606: C39 + 1e+06 alpha_ob37_seg18[3] <= 1.0000178984002755e+06
 R8607: alpha_ob37_seg18[0] + alpha_ob37_seg18[1] + alpha_ob37_seg18[2]
   + alpha_ob37_seg18[3] >= 1
 R8608: - C36 + 1e+06 alpha_ob38_seg18[0] <= 999994.9164998011
 R8609: - C38 + 1e+06 alpha_ob38_seg18[0] <= 999994.9164998011
 R8610: - C37 + 1e+06 alpha_ob38_seg18[1] <= 999925.4284782809
 R8611: - C39 + 1e+06 alpha_ob38_seg18[1] <= 999925.4284782809
 R8612: C36 + 1e+06 alpha_ob38_seg18[2] <= 999943.6834998011
 R8613: C38 + 1e+06 alpha_ob38_seg18[2] <= 999943.6834998011
 R8614: C37 + 1e+06 alpha_ob38_seg18[3] <= 1.0000720216782809e+06
 R8615: C39 + 1e+06 alpha_ob38_seg18[3] <= 1.0000720216782809e+06
 R8616: alpha_ob38_seg18[0] + alpha_ob38_seg18[1] + alpha_ob38_seg18[2]
   + alpha_ob38_seg18[3] >= 1
 R8617: - C36 + 1e+06 alpha_ob39_seg18[0] <= 999914.52327842
 R8618: - C38 + 1e+06 alpha_ob39_seg18[0] <= 999914.52327842
 R8619: - C37 + 1e+06 alpha_ob39_seg18[1] <= 999993.6552362755
 R8620: - C39 + 1e+06 alpha_ob39_seg18[1] <= 999993.6552362755
 R8621: C36 + 1e+06 alpha_ob39_seg18[2] <= 1.0000829266784199e+06
 R8622: C38 + 1e+06 alpha_ob39_seg18[2] <= 1.0000829266784199e+06
 R8623: C37 + 1e+06 alpha_ob39_seg18[3] <= 999992.1947642756
 R8624: C39 + 1e+06 alpha_ob39_seg18[3] <= 999992.1947642756
 R8625: alpha_ob39_seg18[0] + alpha_ob39_seg18[1] + alpha_ob39_seg18[2]
   + alpha_ob39_seg18[3] >= 1
 R8626: - C36 + 1e+06 alpha_ob40_seg18[0] <= 999912.265199801
 R8627: - C38 + 1e+06 alpha_ob40_seg18[0] <= 999912.265199801
 R8628: - C37 + 1e+06 alpha_ob40_seg18[1] <= 999987.7842112777
 R8629: - C39 + 1e+06 alpha_ob40_seg18[1] <= 999987.7842112777
 R8630: C36 + 1e+06 alpha_ob40_seg18[2] <= 1.000076834799801e+06
 R8631: C38 + 1e+06 alpha_ob40_seg18[2] <= 1.000076834799801e+06
 R8632: C37 + 1e+06 alpha_ob40_seg18[3] <= 1.0000096658112778e+06
 R8633: C39 + 1e+06 alpha_ob40_seg18[3] <= 1.0000096658112778e+06
 R8634: alpha_ob40_seg18[0] + alpha_ob40_seg18[1] + alpha_ob40_seg18[2]
   + alpha_ob40_seg18[3] >= 1
 R8635: - C36 + 1e+06 alpha_ob41_seg18[0] <= 999920.6152119379
 R8636: - C38 + 1e+06 alpha_ob41_seg18[0] <= 999920.6152119379
 R8637: - C37 + 1e+06 alpha_ob41_seg18[1] <= 999981.4341997246
 R8638: - C39 + 1e+06 alpha_ob41_seg18[1] <= 999981.4341997246
 R8639: C36 + 1e+06 alpha_ob41_seg18[2] <= 1.0000768348119379e+06
 R8640: C38 + 1e+06 alpha_ob41_seg18[2] <= 1.0000768348119379e+06
 R8641: C37 + 1e+06 alpha_ob41_seg18[3] <= 1.0000096657997245e+06
 R8642: C39 + 1e+06 alpha_ob41_seg18[3] <= 1.0000096657997245e+06
 R8643: alpha_ob41_seg18[0] + alpha_ob41_seg18[1] + alpha_ob41_seg18[2]
   + alpha_ob41_seg18[3] >= 1
 R8644: - C36 + 1e+06 alpha_ob42_seg18[0] <= 999919.2652
 R8645: - C38 + 1e+06 alpha_ob42_seg18[0] <= 999919.2652
 R8646: - C37 + 1e+06 alpha_ob42_seg18[1] <= 999981.4342
 R8647: - C39 + 1e+06 alpha_ob42_seg18[1] <= 999981.4342
 R8648: C36 + 1e+06 alpha_ob42_seg18[2] <= 1.0000768348e+06
 R8649: C38 + 1e+06 alpha_ob42_seg18[2] <= 1.0000768348e+06
 R8650: C37 + 1e+06 alpha_ob42_seg18[3] <= 1.0000160158000001e+06
 R8651: C39 + 1e+06 alpha_ob42_seg18[3] <= 1.0000160158000001e+06
 R8652: alpha_ob42_seg18[0] + alpha_ob42_seg18[1] + alpha_ob42_seg18[2]
   + alpha_ob42_seg18[3] >= 1
 R8653: - C36 + 1e+06 alpha_ob43_seg18[0] <= 999919.2652454559
 R8654: - C38 + 1e+06 alpha_ob43_seg18[0] <= 999919.2652454559
 R8655: - C37 + 1e+06 alpha_ob43_seg18[1] <= 999956.8341997246
 R8656: - C39 + 1e+06 alpha_ob43_seg18[1] <= 999956.8341997246
 R8657: C36 + 1e+06 alpha_ob43_seg18[2] <= 1.0000781848454559e+06
 R8658: C38 + 1e+06 alpha_ob43_seg18[2] <= 1.0000781848454559e+06
 R8659: C37 + 1e+06 alpha_ob43_seg18[3] <= 1.0000160157997246e+06
 R8660: C39 + 1e+06 alpha_ob43_seg18[3] <= 1.0000160157997246e+06
 R8661: alpha_ob43_seg18[0] + alpha_ob43_seg18[1] + alpha_ob43_seg18[2]
   + alpha_ob43_seg18[3] >= 1
 R8662: - C36 + 1e+06 alpha_ob44_seg18[0] <= 999991.83683
 R8663: - C38 + 1e+06 alpha_ob44_seg18[0] <= 999991.83683
 R8664: - C37 + 1e+06 alpha_ob44_seg18[1] <= 999940.589
 R8665: - C39 + 1e+06 alpha_ob44_seg18[1] <= 999940.589
 R8666: C36 + 1e+06 alpha_ob44_seg18[2] <= 1.00000401317e+06
 R8667: C38 + 1e+06 alpha_ob44_seg18[2] <= 1.00000401317e+06
 R8668: C37 + 1e+06 alpha_ob44_seg18[3] <= 1.000056861e+06
 R8669: C39 + 1e+06 alpha_ob44_seg18[3] <= 1.000056861e+06
 R8670: alpha_ob44_seg18[0] + alpha_ob44_seg18[1] + alpha_ob44_seg18[2]
   + alpha_ob44_seg18[3] >= 1
 R8671: - C36 + 1e+06 alpha_ob45_seg18[0] <= 999991.8368084199
 R8672: - C38 + 1e+06 alpha_ob45_seg18[0] <= 999991.8368084199
 R8673: - C37 + 1e+06 alpha_ob45_seg18[1] <= 999940.5890002755
 R8674: - C39 + 1e+06 alpha_ob45_seg18[1] <= 999940.5890002755
 R8675: C36 + 1e+06 alpha_ob45_seg18[2] <= 1.0000056131484199e+06
 R8676: C38 + 1e+06 alpha_ob45_seg18[2] <= 1.0000056131484199e+06
 R8677: C37 + 1e+06 alpha_ob45_seg18[3] <= 1.0000452610002755e+06
 R8678: C39 + 1e+06 alpha_ob45_seg18[3] <= 1.0000452610002755e+06
 R8679: alpha_ob45_seg18[0] + alpha_ob45_seg18[1] + alpha_ob45_seg18[2]
   + alpha_ob45_seg18[3] >= 1
 R8680: - C36 + 1e+06 alpha_ob46_seg18[0] <= 1.0000764021508709e+06
 R8681: - C38 + 1e+06 alpha_ob46_seg18[0] <= 1.0000764021508709e+06
 R8682: - C37 + 1e+06 alpha_ob46_seg18[1] <= 999957.1027002756
 R8683: - C39 + 1e+06 alpha_ob46_seg18[1] <= 999957.1027002756
 R8684: C36 + 1e+06 alpha_ob46_seg18[2] <= 999921.0477508709
 R8685: C38 + 1e+06 alpha_ob46_seg18[2] <= 999921.0477508709
 R8686: C37 + 1e+06 alpha_ob46_seg18[3] <= 1.0000137473002756e+06
 R8687: C39 + 1e+06 alpha_ob46_seg18[3] <= 1.0000137473002756e+06
 R8688: alpha_ob46_seg18[0] + alpha_ob46_seg18[1] + alpha_ob46_seg18[2]
   + alpha_ob46_seg18[3] >= 1
 R8689: - C36 + 1e+06 alpha_ob47_seg18[0] <= 1.0000445522e+06
 R8690: - C38 + 1e+06 alpha_ob47_seg18[0] <= 1.0000445522e+06
 R8691: - C37 + 1e+06 alpha_ob47_seg18[1] <= 999983.7027
 R8692: - C39 + 1e+06 alpha_ob47_seg18[1] <= 999983.7027
 R8693: C36 + 1e+06 alpha_ob47_seg18[2] <= 999921.0477999999
 R8694: C38 + 1e+06 alpha_ob47_seg18[2] <= 999921.0477999999
 R8695: C37 + 1e+06 alpha_ob47_seg18[3] <= 1.0000137473e+06
 R8696: C39 + 1e+06 alpha_ob47_seg18[3] <= 1.0000137473e+06
 R8697: alpha_ob47_seg18[0] + alpha_ob47_seg18[1] + alpha_ob47_seg18[2]
   + alpha_ob47_seg18[3] >= 1
 R8698: - C36 + 1e+06 alpha_ob48_seg18[0] <= 1.0000445522183661e+06
 R8699: - C38 + 1e+06 alpha_ob48_seg18[0] <= 1.0000445522183661e+06
 R8700: - C37 + 1e+06 alpha_ob48_seg18[1] <= 999973.8526997245
 R8701: - C39 + 1e+06 alpha_ob48_seg18[1] <= 999973.8526997245
 R8702: C36 + 1e+06 alpha_ob48_seg18[2] <= 999952.8978183661
 R8703: C38 + 1e+06 alpha_ob48_seg18[2] <= 999952.8978183661
 R8704: C37 + 1e+06 alpha_ob48_seg18[3] <= 1.0000137472997245e+06
 R8705: C39 + 1e+06 alpha_ob48_seg18[3] <= 1.0000137472997245e+06
 R8706: alpha_ob48_seg18[0] + alpha_ob48_seg18[1] + alpha_ob48_seg18[2]
   + alpha_ob48_seg18[3] >= 1
 R8707: - C36 + 1e+06 alpha_ob49_seg18[0] <= 1.000064332081634e+06
 R8708: - C38 + 1e+06 alpha_ob49_seg18[0] <= 1.000064332081634e+06
 R8709: - C37 + 1e+06 alpha_ob49_seg18[1] <= 999973.9401002756
 R8710: - C39 + 1e+06 alpha_ob49_seg18[1] <= 999973.9401002756
 R8711: C36 + 1e+06 alpha_ob49_seg18[2] <= 999933.1178816339
 R8712: C38 + 1e+06 alpha_ob49_seg18[2] <= 999933.1178816339
 R8713: C37 + 1e+06 alpha_ob49_seg18[3] <= 1.0000136599002755e+06
 R8714: C39 + 1e+06 alpha_ob49_seg18[3] <= 1.0000136599002755e+06
 R8715: alpha_ob49_seg18[0] + alpha_ob49_seg18[1] + alpha_ob49_seg18[2]
   + alpha_ob49_seg18[3] >= 1
 R8716: - C36 + 1e+06 alpha_ob50_seg18[0] <= 1.0000643321013774e+06
 R8717: - C38 + 1e+06 alpha_ob50_seg18[0] <= 1.0000643321013774e+06
 R8718: - C37 + 1e+06 alpha_ob50_seg18[1] <= 999983.1900997246
 R8719: - C39 + 1e+06 alpha_ob50_seg18[1] <= 999983.1900997246
 R8720: C36 + 1e+06 alpha_ob50_seg18[2] <= 999933.1179013775
 R8721: C38 + 1e+06 alpha_ob50_seg18[2] <= 999933.1179013775
 R8722: C37 + 1e+06 alpha_ob50_seg18[3] <= 1.0000136598997245e+06
 R8723: C39 + 1e+06 alpha_ob50_seg18[3] <= 1.0000136598997245e+06
 R8724: alpha_ob50_seg18[0] + alpha_ob50_seg18[1] + alpha_ob50_seg18[2]
   + alpha_ob50_seg18[3] >= 1
 R8725: - C38 + 1e+06 alpha_ob0_seg19[0] <= 1.0000284170499525e+06
 R8726: - C40 + 1e+06 alpha_ob0_seg19[0] <= 1.0000284170499525e+06
 R8727: - C39 + 1e+06 alpha_ob0_seg19[1] <= 999957.0843002755
 R8728: - C41 + 1e+06 alpha_ob0_seg19[1] <= 999957.0843002755
 R8729: C38 + 1e+06 alpha_ob0_seg19[2] <= 999969.0328499526
 R8730: C40 + 1e+06 alpha_ob0_seg19[2] <= 999969.0328499526
 R8731: C39 + 1e+06 alpha_ob0_seg19[3] <= 1.0000132657002755e+06
 R8732: C41 + 1e+06 alpha_ob0_seg19[3] <= 1.0000132657002755e+06
 R8733: alpha_ob0_seg19[0] + alpha_ob0_seg19[1] + alpha_ob0_seg19[2]
   + alpha_ob0_seg19[3] >= 1
 R8734: - C38 + 1e+06 alpha_ob1_seg19[0] <= 1.000028417099801e+06
 R8735: - C40 + 1e+06 alpha_ob1_seg19[0] <= 1.000028417099801e+06
 R8736: - C39 + 1e+06 alpha_ob1_seg19[1] <= 999984.1843155898
 R8737: - C41 + 1e+06 alpha_ob1_seg19[1] <= 999984.1843155898
 R8738: C38 + 1e+06 alpha_ob1_seg19[2] <= 999957.4328998009
 R8739: C40 + 1e+06 alpha_ob1_seg19[2] <= 999957.4328998009
 R8740: C39 + 1e+06 alpha_ob1_seg19[3] <= 1.0000132657155899e+06
 R8741: C41 + 1e+06 alpha_ob1_seg19[3] <= 1.0000132657155899e+06
 R8742: alpha_ob1_seg19[0] + alpha_ob1_seg19[1] + alpha_ob1_seg19[2]
   + alpha_ob1_seg19[3] >= 1
 R8743: - C38 + 1e+06 alpha_ob2_seg19[0] <= 1.0000400171500475e+06
 R8744: - C40 + 1e+06 alpha_ob2_seg19[0] <= 1.0000400171500475e+06
 R8745: - C39 + 1e+06 alpha_ob2_seg19[1] <= 999957.0842997245
 R8746: - C41 + 1e+06 alpha_ob2_seg19[1] <= 999957.0842997245
 R8747: C38 + 1e+06 alpha_ob2_seg19[2] <= 999957.4329500474
 R8748: C40 + 1e+06 alpha_ob2_seg19[2] <= 999957.4329500474
 R8749: C39 + 1e+06 alpha_ob2_seg19[3] <= 1.0000132656997246e+06
 R8750: C41 + 1e+06 alpha_ob2_seg19[3] <= 1.0000132656997246e+06
 R8751: alpha_ob2_seg19[0] + alpha_ob2_seg19[1] + alpha_ob2_seg19[2]
   + alpha_ob2_seg19[3] >= 1
 R8752: - C38 + 1e+06 alpha_ob3_seg19[0] <= 1.0000109161752058e+06
 R8753: - C40 + 1e+06 alpha_ob3_seg19[0] <= 1.0000109161752058e+06
 R8754: - C39 + 1e+06 alpha_ob3_seg19[1] <= 999970.2681002755
 R8755: - C41 + 1e+06 alpha_ob3_seg19[1] <= 999970.2681002755
 R8756: C38 + 1e+06 alpha_ob3_seg19[2] <= 999986.5337752058
 R8757: C40 + 1e+06 alpha_ob3_seg19[2] <= 999986.5337752058
 R8758: C39 + 1e+06 alpha_ob3_seg19[3] <= 1.0000138319002755e+06
 R8759: C41 + 1e+06 alpha_ob3_seg19[3] <= 1.0000138319002755e+06
 R8760: alpha_ob3_seg19[0] + alpha_ob3_seg19[1] + alpha_ob3_seg19[2]
   + alpha_ob3_seg19[3] >= 1
 R8761: - C38 + 1e+06 alpha_ob4_seg19[0] <= 1.000010916199801e+06
 R8762: - C40 + 1e+06 alpha_ob4_seg19[0] <= 1.000010916199801e+06
 R8763: - C39 + 1e+06 alpha_ob4_seg19[1] <= 999983.6181179117
 R8764: - C41 + 1e+06 alpha_ob4_seg19[1] <= 999983.6181179117
 R8765: C38 + 1e+06 alpha_ob4_seg19[2] <= 999973.183799801
 R8766: C40 + 1e+06 alpha_ob4_seg19[2] <= 999973.183799801
 R8767: C39 + 1e+06 alpha_ob4_seg19[3] <= 1.0000138319179118e+06
 R8768: C41 + 1e+06 alpha_ob4_seg19[3] <= 1.0000138319179118e+06
 R8769: alpha_ob4_seg19[0] + alpha_ob4_seg19[1] + alpha_ob4_seg19[2]
   + alpha_ob4_seg19[3] >= 1
 R8770: - C38 + 1e+06 alpha_ob5_seg19[0] <= 999992.49102
 R8771: - C40 + 1e+06 alpha_ob5_seg19[0] <= 999992.49102
 R8772: - C39 + 1e+06 alpha_ob5_seg19[1] <= 999957.1016000001
 R8773: - C41 + 1e+06 alpha_ob5_seg19[1] <= 999957.1016000001
 R8774: C38 + 1e+06 alpha_ob5_seg19[2] <= 999986.35898
 R8775: C40 + 1e+06 alpha_ob5_seg19[2] <= 999986.35898
 R8776: C39 + 1e+06 alpha_ob5_seg19[3] <= 1.0000403484e+06
 R8777: C41 + 1e+06 alpha_ob5_seg19[3] <= 1.0000403484e+06
 R8778: alpha_ob5_seg19[0] + alpha_ob5_seg19[1] + alpha_ob5_seg19[2]
   + alpha_ob5_seg19[3] >= 1
 R8779: - C38 + 1e+06 alpha_ob6_seg19[0] <= 999981.3909999999
 R8780: - C40 + 1e+06 alpha_ob6_seg19[0] <= 999981.3909999999
 R8781: - C39 + 1e+06 alpha_ob6_seg19[1] <= 999957.1016000001
 R8782: - C41 + 1e+06 alpha_ob6_seg19[1] <= 999957.1016000001
 R8783: C38 + 1e+06 alpha_ob6_seg19[2] <= 1.000004959e+06
 R8784: C40 + 1e+06 alpha_ob6_seg19[2] <= 1.000004959e+06
 R8785: C39 + 1e+06 alpha_ob6_seg19[3] <= 1.0000403484e+06
 R8786: C41 + 1e+06 alpha_ob6_seg19[3] <= 1.0000403484e+06
 R8787: alpha_ob6_seg19[0] + alpha_ob6_seg19[1] + alpha_ob6_seg19[2]
   + alpha_ob6_seg19[3] >= 1
 R8788: - C38 + 1e+06 alpha_ob7_seg19[0] <= 999981.3909825523
 R8789: - C40 + 1e+06 alpha_ob7_seg19[0] <= 999981.3909825523
 R8790: - C39 + 1e+06 alpha_ob7_seg19[1] <= 999957.1016002755
 R8791: - C41 + 1e+06 alpha_ob7_seg19[1] <= 999957.1016002755
 R8792: C38 + 1e+06 alpha_ob7_seg19[2] <= 1.0000160589825523e+06
 R8793: C40 + 1e+06 alpha_ob7_seg19[2] <= 1.0000160589825523e+06
 R8794: C39 + 1e+06 alpha_ob7_seg19[3] <= 1.0000309984002755e+06
 R8795: C41 + 1e+06 alpha_ob7_seg19[3] <= 1.0000309984002755e+06
 R8796: alpha_ob7_seg19[0] + alpha_ob7_seg19[1] + alpha_ob7_seg19[2]
   + alpha_ob7_seg19[3] >= 1
 R8797: - C38 + 1e+06 alpha_ob8_seg19[0] <= 999981.3867954084
 R8798: - C40 + 1e+06 alpha_ob8_seg19[0] <= 999981.3867954084
 R8799: - C39 + 1e+06 alpha_ob8_seg19[1] <= 999968.8595002755
 R8800: - C41 + 1e+06 alpha_ob8_seg19[1] <= 999968.8595002755
 R8801: C38 + 1e+06 alpha_ob8_seg19[2] <= 1.0000160631954084e+06
 R8802: C40 + 1e+06 alpha_ob8_seg19[2] <= 1.0000160631954084e+06
 R8803: C39 + 1e+06 alpha_ob8_seg19[3] <= 1.0000262405002755e+06
 R8804: C41 + 1e+06 alpha_ob8_seg19[3] <= 1.0000262405002755e+06
 R8805: alpha_ob8_seg19[0] + alpha_ob8_seg19[1] + alpha_ob8_seg19[2]
   + alpha_ob8_seg19[3] >= 1
 R8806: - C38 + 1e+06 alpha_ob9_seg19[0] <= 999981.386799801
 R8807: - C40 + 1e+06 alpha_ob9_seg19[0] <= 999981.386799801
 R8808: - C39 + 1e+06 alpha_ob9_seg19[1] <= 999971.2095013268
 R8809: - C41 + 1e+06 alpha_ob9_seg19[1] <= 999971.2095013268
 R8810: C38 + 1e+06 alpha_ob9_seg19[2] <= 1.0000152131998009e+06
 R8811: C40 + 1e+06 alpha_ob9_seg19[2] <= 1.0000152131998009e+06
 R8812: C39 + 1e+06 alpha_ob9_seg19[3] <= 1.0000262405013267e+06
 R8813: C41 + 1e+06 alpha_ob9_seg19[3] <= 1.0000262405013267e+06
 R8814: alpha_ob9_seg19[0] + alpha_ob9_seg19[1] + alpha_ob9_seg19[2]
   + alpha_ob9_seg19[3] >= 1
 R8815: - C38 + 1e+06 alpha_ob10_seg19[0] <= 999982.2368022958
 R8816: - C40 + 1e+06 alpha_ob10_seg19[0] <= 999982.2368022958
 R8817: - C39 + 1e+06 alpha_ob10_seg19[1] <= 999970.1094997245
 R8818: - C41 + 1e+06 alpha_ob10_seg19[1] <= 999970.1094997245
 R8819: C38 + 1e+06 alpha_ob10_seg19[2] <= 1.0000152132022957e+06
 R8820: C40 + 1e+06 alpha_ob10_seg19[2] <= 1.0000152132022957e+06
 R8821: C39 + 1e+06 alpha_ob10_seg19[3] <= 1.0000262404997245e+06
 R8822: C41 + 1e+06 alpha_ob10_seg19[3] <= 1.0000262404997245e+06
 R8823: alpha_ob10_seg19[0] + alpha_ob10_seg19[1] + alpha_ob10_seg19[2]
   + alpha_ob10_seg19[3] >= 1
 R8824: - C38 + 1e+06 alpha_ob11_seg19[0] <= 999982.236799801
 R8825: - C40 + 1e+06 alpha_ob11_seg19[0] <= 999982.236799801
 R8826: - C39 + 1e+06 alpha_ob11_seg19[1] <= 999970.1095291895
 R8827: - C41 + 1e+06 alpha_ob11_seg19[1] <= 999970.1095291895
 R8828: C38 + 1e+06 alpha_ob11_seg19[2] <= 999993.3631998011
 R8829: C40 + 1e+06 alpha_ob11_seg19[2] <= 999993.3631998011
 R8830: C39 + 1e+06 alpha_ob11_seg19[3] <= 1.0000273405291894e+06
 R8831: C41 + 1e+06 alpha_ob11_seg19[3] <= 1.0000273405291894e+06
 R8832: alpha_ob11_seg19[0] + alpha_ob11_seg19[1] + alpha_ob11_seg19[2]
   + alpha_ob11_seg19[3] >= 1
 R8833: - C38 + 1e+06 alpha_ob12_seg19[0] <= 999981.6050802565
 R8834: - C40 + 1e+06 alpha_ob12_seg19[0] <= 999981.6050802565
 R8835: - C39 + 1e+06 alpha_ob12_seg19[1] <= 999972.8942002755
 R8836: - C41 + 1e+06 alpha_ob12_seg19[1] <= 999972.8942002755
 R8837: C38 + 1e+06 alpha_ob12_seg19[2] <= 1.0000158448802566e+06
 R8838: C40 + 1e+06 alpha_ob12_seg19[2] <= 1.0000158448802566e+06
 R8839: C39 + 1e+06 alpha_ob12_seg19[3] <= 1.0000139558002756e+06
 R8840: C41 + 1e+06 alpha_ob12_seg19[3] <= 1.0000139558002756e+06
 R8841: alpha_ob12_seg19[0] + alpha_ob12_seg19[1] + alpha_ob12_seg19[2]
   + alpha_ob12_seg19[3] >= 1
 R8842: - C38 + 1e+06 alpha_ob13_seg19[0] <= 999981.605149801
 R8843: - C40 + 1e+06 alpha_ob13_seg19[0] <= 999981.605149801
 R8844: - C39 + 1e+06 alpha_ob13_seg19[1] <= 999983.4942391404
 R8845: - C41 + 1e+06 alpha_ob13_seg19[1] <= 999983.4942391404
 R8846: C38 + 1e+06 alpha_ob13_seg19[2] <= 999986.4948498011
 R8847: C40 + 1e+06 alpha_ob13_seg19[2] <= 999986.4948498011
 R8848: C39 + 1e+06 alpha_ob13_seg19[3] <= 1.0000139558391405e+06
 R8849: C41 + 1e+06 alpha_ob13_seg19[3] <= 1.0000139558391405e+06
 R8850: alpha_ob13_seg19[0] + alpha_ob13_seg19[1] + alpha_ob13_seg19[2]
   + alpha_ob13_seg19[3] >= 1
 R8851: - C38 + 1e+06 alpha_ob14_seg19[0] <= 999997.6803502565
 R8852: - C40 + 1e+06 alpha_ob14_seg19[0] <= 999997.6803502565
 R8853: - C39 + 1e+06 alpha_ob14_seg19[1] <= 999972.7268002755
 R8854: - C41 + 1e+06 alpha_ob14_seg19[1] <= 999972.7268002755
 R8855: C38 + 1e+06 alpha_ob14_seg19[2] <= 999999.7696102565
 R8856: C40 + 1e+06 alpha_ob14_seg19[2] <= 999999.7696102565
 R8857: C39 + 1e+06 alpha_ob14_seg19[3] <= 1.0000141232002755e+06
 R8858: C41 + 1e+06 alpha_ob14_seg19[3] <= 1.0000141232002755e+06
 R8859: alpha_ob14_seg19[0] + alpha_ob14_seg19[1] + alpha_ob14_seg19[2]
   + alpha_ob14_seg19[3] >= 1
 R8860: - C38 + 1e+06 alpha_ob15_seg19[0] <= 1.0000038341502565e+06
 R8861: - C40 + 1e+06 alpha_ob15_seg19[0] <= 1.0000038341502565e+06
 R8862: - C39 + 1e+06 alpha_ob15_seg19[1] <= 999972.6012002755
 R8863: - C41 + 1e+06 alpha_ob15_seg19[1] <= 999972.6012002755
 R8864: C38 + 1e+06 alpha_ob15_seg19[2] <= 999993.6158102566
 R8865: C40 + 1e+06 alpha_ob15_seg19[2] <= 999993.6158102566
 R8866: C39 + 1e+06 alpha_ob15_seg19[3] <= 1.0000142488002755e+06
 R8867: C41 + 1e+06 alpha_ob15_seg19[3] <= 1.0000142488002755e+06
 R8868: alpha_ob15_seg19[0] + alpha_ob15_seg19[1] + alpha_ob15_seg19[2]
   + alpha_ob15_seg19[3] >= 1
 R8869: - C38 + 1e+06 alpha_ob16_seg19[0] <= 1.0000162874063333e+06
 R8870: - C40 + 1e+06 alpha_ob16_seg19[0] <= 1.0000162874063333e+06
 R8871: - C39 + 1e+06 alpha_ob16_seg19[1] <= 999982.9439602756
 R8872: - C41 + 1e+06 alpha_ob16_seg19[1] <= 999982.9439602756
 R8873: C38 + 1e+06 alpha_ob16_seg19[2] <= 999981.1624063333
 R8874: C40 + 1e+06 alpha_ob16_seg19[2] <= 999981.1624063333
 R8875: C39 + 1e+06 alpha_ob16_seg19[3] <= 999963.6560402757
 R8876: C41 + 1e+06 alpha_ob16_seg19[3] <= 999963.6560402757
 R8877: alpha_ob16_seg19[0] + alpha_ob16_seg19[1] + alpha_ob16_seg19[2]
   + alpha_ob16_seg19[3] >= 1
 R8878: - C38 + 1e+06 alpha_ob17_seg19[0] <= 999980.937509
 R8879: - C40 + 1e+06 alpha_ob17_seg19[0] <= 999980.937509
 R8880: - C39 + 1e+06 alpha_ob17_seg19[1] <= 1.000034894e+06
 R8881: - C41 + 1e+06 alpha_ob17_seg19[1] <= 1.000034894e+06
 R8882: C38 + 1e+06 alpha_ob17_seg19[2] <= 999981.162491
 R8883: C40 + 1e+06 alpha_ob17_seg19[2] <= 999981.162491
 R8884: C39 + 1e+06 alpha_ob17_seg19[3] <= 999962.556
 R8885: C41 + 1e+06 alpha_ob17_seg19[3] <= 999962.556
 R8886: alpha_ob17_seg19[0] + alpha_ob17_seg19[1] + alpha_ob17_seg19[2]
   + alpha_ob17_seg19[3] >= 1
 R8887: - C38 + 1e+06 alpha_ob18_seg19[0] <= 999990.2670277582
 R8888: - C40 + 1e+06 alpha_ob18_seg19[0] <= 999990.2670277582
 R8889: - C39 + 1e+06 alpha_ob18_seg19[1] <= 1.0000347702002756e+06
 R8890: - C41 + 1e+06 alpha_ob18_seg19[1] <= 1.0000347702002756e+06
 R8891: C38 + 1e+06 alpha_ob18_seg19[2] <= 1.0000071828877581e+06
 R8892: C40 + 1e+06 alpha_ob18_seg19[2] <= 1.0000071828877581e+06
 R8893: C39 + 1e+06 alpha_ob18_seg19[3] <= 999939.8298002756
 R8894: C41 + 1e+06 alpha_ob18_seg19[3] <= 999939.8298002756
 R8895: alpha_ob18_seg19[0] + alpha_ob18_seg19[1] + alpha_ob18_seg19[2]
   + alpha_ob18_seg19[3] >= 1
 R8896: - C38 + 1e+06 alpha_ob19_seg19[0] <= 999990.267069801
 R8897: - C40 + 1e+06 alpha_ob19_seg19[0] <= 999990.267069801
 R8898: - C39 + 1e+06 alpha_ob19_seg19[1] <= 1.0000576202069657e+06
 R8899: - C41 + 1e+06 alpha_ob19_seg19[1] <= 1.0000576202069657e+06
 R8900: C38 + 1e+06 alpha_ob19_seg19[2] <= 1.000002082929801e+06
 R8901: C40 + 1e+06 alpha_ob19_seg19[2] <= 1.000002082929801e+06
 R8902: C39 + 1e+06 alpha_ob19_seg19[3] <= 999939.8298069657
 R8903: C41 + 1e+06 alpha_ob19_seg19[3] <= 999939.8298069657
 R8904: alpha_ob19_seg19[0] + alpha_ob19_seg19[1] + alpha_ob19_seg19[2]
   + alpha_ob19_seg19[3] >= 1
 R8905: - C38 + 1e+06 alpha_ob20_seg19[0] <= 999991.073732
 R8906: - C40 + 1e+06 alpha_ob20_seg19[0] <= 999991.073732
 R8907: - C39 + 1e+06 alpha_ob20_seg19[1] <= 999952.2125
 R8908: - C41 + 1e+06 alpha_ob20_seg19[1] <= 999952.2125
 R8909: C38 + 1e+06 alpha_ob20_seg19[2] <= 999990.776268
 R8910: C40 + 1e+06 alpha_ob20_seg19[2] <= 999990.776268
 R8911: C39 + 1e+06 alpha_ob20_seg19[3] <= 1.0000452375e+06
 R8912: C41 + 1e+06 alpha_ob20_seg19[3] <= 1.0000452375e+06
 R8913: alpha_ob20_seg19[0] + alpha_ob20_seg19[1] + alpha_ob20_seg19[2]
   + alpha_ob20_seg19[3] >= 1
 R8914: - C38 + 1e+06 alpha_ob21_seg19[0] <= 999985.657099801
 R8915: - C40 + 1e+06 alpha_ob21_seg19[0] <= 999985.657099801
 R8916: - C39 + 1e+06 alpha_ob21_seg19[1] <= 1.0000576202063023e+06
 R8917: - C41 + 1e+06 alpha_ob21_seg19[1] <= 1.0000576202063023e+06
 R8918: C38 + 1e+06 alpha_ob21_seg19[2] <= 1.000007192899801e+06
 R8919: C40 + 1e+06 alpha_ob21_seg19[2] <= 1.000007192899801e+06
 R8920: C39 + 1e+06 alpha_ob21_seg19[3] <= 999939.8298063023
 R8921: C41 + 1e+06 alpha_ob21_seg19[3] <= 999939.8298063023
 R8922: alpha_ob21_seg19[0] + alpha_ob21_seg19[1] + alpha_ob21_seg19[2]
   + alpha_ob21_seg19[3] >= 1
 R8923: - C38 + 1e+06 alpha_ob22_seg19[0] <= 999994.771829801
 R8924: - C40 + 1e+06 alpha_ob22_seg19[0] <= 999994.771829801
 R8925: - C39 + 1e+06 alpha_ob22_seg19[1] <= 1.0000615983063023e+06
 R8926: - C41 + 1e+06 alpha_ob22_seg19[1] <= 1.0000615983063023e+06
 R8927: C38 + 1e+06 alpha_ob22_seg19[2] <= 999998.078169801
 R8928: C40 + 1e+06 alpha_ob22_seg19[2] <= 999998.078169801
 R8929: C39 + 1e+06 alpha_ob22_seg19[3] <= 999935.8517063023
 R8930: C41 + 1e+06 alpha_ob22_seg19[3] <= 999935.8517063023
 R8931: alpha_ob22_seg19[0] + alpha_ob22_seg19[1] + alpha_ob22_seg19[2]
   + alpha_ob22_seg19[3] >= 1
 R8932: - C38 + 1e+06 alpha_ob23_seg19[0] <= 999980.9374508709
 R8933: - C40 + 1e+06 alpha_ob23_seg19[0] <= 999980.9374508709
 R8934: - C39 + 1e+06 alpha_ob23_seg19[1] <= 1.0000348940002755e+06
 R8935: - C41 + 1e+06 alpha_ob23_seg19[1] <= 1.0000348940002755e+06
 R8936: C38 + 1e+06 alpha_ob23_seg19[2] <= 1.0000165124508708e+06
 R8937: C40 + 1e+06 alpha_ob23_seg19[2] <= 1.0000165124508708e+06
 R8938: C39 + 1e+06 alpha_ob23_seg19[3] <= 999935.9560002756
 R8939: C41 + 1e+06 alpha_ob23_seg19[3] <= 999935.9560002756
 R8940: alpha_ob23_seg19[0] + alpha_ob23_seg19[1] + alpha_ob23_seg19[2]
   + alpha_ob23_seg19[3] >= 1
 R8941: - C38 + 1e+06 alpha_ob24_seg19[0] <= 999980.937499801
 R8942: - C40 + 1e+06 alpha_ob24_seg19[0] <= 999980.937499801
 R8943: - C39 + 1e+06 alpha_ob24_seg19[1] <= 1.0000614940072973e+06
 R8944: - C41 + 1e+06 alpha_ob24_seg19[1] <= 1.0000614940072973e+06
 R8945: C38 + 1e+06 alpha_ob24_seg19[2] <= 1.000011162499801e+06
 R8946: C40 + 1e+06 alpha_ob24_seg19[2] <= 1.000011162499801e+06
 R8947: C39 + 1e+06 alpha_ob24_seg19[3] <= 999935.9560072974
 R8948: C41 + 1e+06 alpha_ob24_seg19[3] <= 999935.9560072974
 R8949: alpha_ob24_seg19[0] + alpha_ob24_seg19[1] + alpha_ob24_seg19[2]
   + alpha_ob24_seg19[3] >= 1
 R8950: - C38 + 1e+06 alpha_ob25_seg19[0] <= 999956.8394000001
 R8951: - C40 + 1e+06 alpha_ob25_seg19[0] <= 999956.8394000001
 R8952: - C39 + 1e+06 alpha_ob25_seg19[1] <= 999952.2463
 R8953: - C41 + 1e+06 alpha_ob25_seg19[1] <= 999952.2463
 R8954: C38 + 1e+06 alpha_ob25_seg19[2] <= 1.0000097606e+06
 R8955: C40 + 1e+06 alpha_ob25_seg19[2] <= 1.0000097606e+06
 R8956: C39 + 1e+06 alpha_ob25_seg19[3] <= 1.0000452037e+06
 R8957: C41 + 1e+06 alpha_ob25_seg19[3] <= 1.0000452037e+06
 R8958: alpha_ob25_seg19[0] + alpha_ob25_seg19[1] + alpha_ob25_seg19[2]
   + alpha_ob25_seg19[3] >= 1
 R8959: - C38 + 1e+06 alpha_ob26_seg19[0] <= 999980.9375348955
 R8960: - C40 + 1e+06 alpha_ob26_seg19[0] <= 999980.9375348955
 R8961: - C39 + 1e+06 alpha_ob26_seg19[1] <= 1.0000160439997246e+06
 R8962: - C41 + 1e+06 alpha_ob26_seg19[1] <= 1.0000160439997246e+06
 R8963: C38 + 1e+06 alpha_ob26_seg19[2] <= 1.0000165125348954e+06
 R8964: C40 + 1e+06 alpha_ob26_seg19[2] <= 1.0000165125348954e+06
 R8965: C39 + 1e+06 alpha_ob26_seg19[3] <= 999962.5559997245
 R8966: C41 + 1e+06 alpha_ob26_seg19[3] <= 999962.5559997245
 R8967: alpha_ob26_seg19[0] + alpha_ob26_seg19[1] + alpha_ob26_seg19[2]
   + alpha_ob26_seg19[3] >= 1
 R8968: - C38 + 1e+06 alpha_ob27_seg19[0] <= 999956.839399801
 R8969: - C40 + 1e+06 alpha_ob27_seg19[0] <= 999956.839399801
 R8970: - C39 + 1e+06 alpha_ob27_seg19[1] <= 999925.3963434525
 R8971: - C41 + 1e+06 alpha_ob27_seg19[1] <= 999925.3963434525
 R8972: C38 + 1e+06 alpha_ob27_seg19[2] <= 1.0000080105998011e+06
 R8973: C40 + 1e+06 alpha_ob27_seg19[2] <= 1.0000080105998011e+06
 R8974: C39 + 1e+06 alpha_ob27_seg19[3] <= 1.0000720537434525e+06
 R8975: C41 + 1e+06 alpha_ob27_seg19[3] <= 1.0000720537434525e+06
 R8976: alpha_ob27_seg19[0] + alpha_ob27_seg19[1] + alpha_ob27_seg19[2]
   + alpha_ob27_seg19[3] >= 1
 R8977: - C38 + 1e+06 alpha_ob28_seg19[0] <= 999980.9374998009
 R8978: - C40 + 1e+06 alpha_ob28_seg19[0] <= 999980.9374998009
 R8979: - C39 + 1e+06 alpha_ob28_seg19[1] <= 1.0000160440053071e+06
 R8980: - C41 + 1e+06 alpha_ob28_seg19[1] <= 1.0000160440053071e+06
 R8981: C38 + 1e+06 alpha_ob28_seg19[2] <= 1.000012662499801e+06
 R8982: C40 + 1e+06 alpha_ob28_seg19[2] <= 1.000012662499801e+06
 R8983: C39 + 1e+06 alpha_ob28_seg19[3] <= 999981.4060053072
 R8984: C41 + 1e+06 alpha_ob28_seg19[3] <= 999981.4060053072
 R8985: alpha_ob28_seg19[0] + alpha_ob28_seg19[1] + alpha_ob28_seg19[2]
   + alpha_ob28_seg19[3] >= 1
 R8986: - C38 + 1e+06 alpha_ob29_seg19[0] <= 999983.6053459151
 R8987: - C40 + 1e+06 alpha_ob29_seg19[0] <= 999983.6053459151
 R8988: - C39 + 1e+06 alpha_ob29_seg19[1] <= 999987.7722497246
 R8989: - C41 + 1e+06 alpha_ob29_seg19[1] <= 999987.7722497246
 R8990: C38 + 1e+06 alpha_ob29_seg19[2] <= 1.000013844745915e+06
 R8991: C40 + 1e+06 alpha_ob29_seg19[2] <= 1.000013844745915e+06
 R8992: C39 + 1e+06 alpha_ob29_seg19[3] <= 999984.8277497246
 R8993: C41 + 1e+06 alpha_ob29_seg19[3] <= 999984.8277497246
 R8994: alpha_ob29_seg19[0] + alpha_ob29_seg19[1] + alpha_ob29_seg19[2]
   + alpha_ob29_seg19[3] >= 1
 R8995: - C38 + 1e+06 alpha_ob30_seg19[0] <= 999958.8717898987
 R8996: - C40 + 1e+06 alpha_ob30_seg19[0] <= 999958.8717898987
 R8997: - C39 + 1e+06 alpha_ob30_seg19[1] <= 999987.7837602755
 R8998: - C41 + 1e+06 alpha_ob30_seg19[1] <= 999987.7837602755
 R8999: C38 + 1e+06 alpha_ob30_seg19[2] <= 1.0000385781898986e+06
 R9000: C40 + 1e+06 alpha_ob30_seg19[2] <= 1.0000385781898986e+06
 R9001: C39 + 1e+06 alpha_ob30_seg19[3] <= 1.0000043162402755e+06
 R9002: C41 + 1e+06 alpha_ob30_seg19[3] <= 1.0000043162402755e+06
 R9003: alpha_ob30_seg19[0] + alpha_ob30_seg19[1] + alpha_ob30_seg19[2]
   + alpha_ob30_seg19[3] >= 1
 R9004: - C38 + 1e+06 alpha_ob31_seg19[0] <= 999939.2718
 R9005: - C40 + 1e+06 alpha_ob31_seg19[0] <= 999939.2718
 R9006: - C39 + 1e+06 alpha_ob31_seg19[1] <= 999993.13376
 R9007: - C41 + 1e+06 alpha_ob31_seg19[1] <= 999993.13376
 R9008: C38 + 1e+06 alpha_ob31_seg19[2] <= 1.0000385782e+06
 R9009: C40 + 1e+06 alpha_ob31_seg19[2] <= 1.0000385782e+06
 R9010: C39 + 1e+06 alpha_ob31_seg19[3] <= 1.00000431624e+06
 R9011: C41 + 1e+06 alpha_ob31_seg19[3] <= 1.00000431624e+06
 R9012: alpha_ob31_seg19[0] + alpha_ob31_seg19[1] + alpha_ob31_seg19[2]
   + alpha_ob31_seg19[3] >= 1
 R9013: - C38 + 1e+06 alpha_ob32_seg19[0] <= 999939.2718101013
 R9014: - C40 + 1e+06 alpha_ob32_seg19[0] <= 999939.2718101013
 R9015: - C39 + 1e+06 alpha_ob32_seg19[1] <= 999987.7837597246
 R9016: - C41 + 1e+06 alpha_ob32_seg19[1] <= 999987.7837597246
 R9017: C38 + 1e+06 alpha_ob32_seg19[2] <= 1.0000581782101013e+06
 R9018: C40 + 1e+06 alpha_ob32_seg19[2] <= 1.0000581782101013e+06
 R9019: C39 + 1e+06 alpha_ob32_seg19[3] <= 1.0000043162397245e+06
 R9020: C41 + 1e+06 alpha_ob32_seg19[3] <= 1.0000043162397245e+06
 R9021: alpha_ob32_seg19[0] + alpha_ob32_seg19[1] + alpha_ob32_seg19[2]
   + alpha_ob32_seg19[3] >= 1
 R9022: - C38 + 1e+06 alpha_ob33_seg19[0] <= 999939.271799801
 R9023: - C40 + 1e+06 alpha_ob33_seg19[0] <= 999939.271799801
 R9024: - C39 + 1e+06 alpha_ob33_seg19[1] <= 999987.7838262042
 R9025: - C41 + 1e+06 alpha_ob33_seg19[1] <= 999987.7838262042
 R9026: C38 + 1e+06 alpha_ob33_seg19[2] <= 1.000038578199801e+06
 R9027: C40 + 1e+06 alpha_ob33_seg19[2] <= 1.000038578199801e+06
 R9028: C39 + 1e+06 alpha_ob33_seg19[3] <= 1.0000096662262043e+06
 R9029: C41 + 1e+06 alpha_ob33_seg19[3] <= 1.0000096662262043e+06
 R9030: alpha_ob33_seg19[0] + alpha_ob33_seg19[1] + alpha_ob33_seg19[2]
   + alpha_ob33_seg19[3] >= 1
 R9031: - C38 + 1e+06 alpha_ob34_seg19[0] <= 999926.0708
 R9032: - C40 + 1e+06 alpha_ob34_seg19[0] <= 999926.0708
 R9033: - C39 + 1e+06 alpha_ob34_seg19[1] <= 999983.7196
 R9034: - C41 + 1e+06 alpha_ob34_seg19[1] <= 999983.7196
 R9035: C38 + 1e+06 alpha_ob34_seg19[2] <= 1.0000250292e+06
 R9036: C40 + 1e+06 alpha_ob34_seg19[2] <= 1.0000250292e+06
 R9037: C39 + 1e+06 alpha_ob34_seg19[3] <= 1.0000137304e+06
 R9038: C41 + 1e+06 alpha_ob34_seg19[3] <= 1.0000137304e+06
 R9039: alpha_ob34_seg19[0] + alpha_ob34_seg19[1] + alpha_ob34_seg19[2]
   + alpha_ob34_seg19[3] >= 1
 R9040: - C38 + 1e+06 alpha_ob35_seg19[0] <= 999926.0708293857
 R9041: - C40 + 1e+06 alpha_ob35_seg19[0] <= 999926.0708293857
 R9042: - C39 + 1e+06 alpha_ob35_seg19[1] <= 999967.8695997245
 R9043: - C41 + 1e+06 alpha_ob35_seg19[1] <= 999967.8695997245
 R9044: C38 + 1e+06 alpha_ob35_seg19[2] <= 1.0000713792293856e+06
 R9045: C40 + 1e+06 alpha_ob35_seg19[2] <= 1.0000713792293856e+06
 R9046: C39 + 1e+06 alpha_ob35_seg19[3] <= 1.0000137303997246e+06
 R9047: C41 + 1e+06 alpha_ob35_seg19[3] <= 1.0000137303997246e+06
 R9048: alpha_ob35_seg19[0] + alpha_ob35_seg19[1] + alpha_ob35_seg19[2]
   + alpha_ob35_seg19[3] >= 1
 R9049: - C38 + 1e+06 alpha_ob36_seg19[0] <= 999972.6917508709
 R9050: - C40 + 1e+06 alpha_ob36_seg19[0] <= 999972.6917508709
 R9051: - C39 + 1e+06 alpha_ob36_seg19[1] <= 999957.1644002756
 R9052: - C41 + 1e+06 alpha_ob36_seg19[1] <= 999957.1644002756
 R9053: C38 + 1e+06 alpha_ob36_seg19[2] <= 1.0000247581508709e+06
 R9054: C40 + 1e+06 alpha_ob36_seg19[2] <= 1.0000247581508709e+06
 R9055: C39 + 1e+06 alpha_ob36_seg19[3] <= 1.0000136856002755e+06
 R9056: C41 + 1e+06 alpha_ob36_seg19[3] <= 1.0000136856002755e+06
 R9057: alpha_ob36_seg19[0] + alpha_ob36_seg19[1] + alpha_ob36_seg19[2]
   + alpha_ob36_seg19[3] >= 1
 R9058: - C38 + 1e+06 alpha_ob37_seg19[0] <= 999939.0009674003
 R9059: - C40 + 1e+06 alpha_ob37_seg19[0] <= 999939.0009674003
 R9060: - C39 + 1e+06 alpha_ob37_seg19[1] <= 999961.9516002756
 R9061: - C41 + 1e+06 alpha_ob37_seg19[1] <= 999961.9516002756
 R9062: C38 + 1e+06 alpha_ob37_seg19[2] <= 1.0000584489674004e+06
 R9063: C40 + 1e+06 alpha_ob37_seg19[2] <= 1.0000584489674004e+06
 R9064: C39 + 1e+06 alpha_ob37_seg19[3] <= 1.0000178984002755e+06
 R9065: C41 + 1e+06 alpha_ob37_seg19[3] <= 1.0000178984002755e+06
 R9066: alpha_ob37_seg19[0] + alpha_ob37_seg19[1] + alpha_ob37_seg19[2]
   + alpha_ob37_seg19[3] >= 1
 R9067: - C38 + 1e+06 alpha_ob38_seg19[0] <= 999994.9164998011
 R9068: - C40 + 1e+06 alpha_ob38_seg19[0] <= 999994.9164998011
 R9069: - C39 + 1e+06 alpha_ob38_seg19[1] <= 999925.4284782809
 R9070: - C41 + 1e+06 alpha_ob38_seg19[1] <= 999925.4284782809
 R9071: C38 + 1e+06 alpha_ob38_seg19[2] <= 999943.6834998011
 R9072: C40 + 1e+06 alpha_ob38_seg19[2] <= 999943.6834998011
 R9073: C39 + 1e+06 alpha_ob38_seg19[3] <= 1.0000720216782809e+06
 R9074: C41 + 1e+06 alpha_ob38_seg19[3] <= 1.0000720216782809e+06
 R9075: alpha_ob38_seg19[0] + alpha_ob38_seg19[1] + alpha_ob38_seg19[2]
   + alpha_ob38_seg19[3] >= 1
 R9076: - C38 + 1e+06 alpha_ob39_seg19[0] <= 999914.52327842
 R9077: - C40 + 1e+06 alpha_ob39_seg19[0] <= 999914.52327842
 R9078: - C39 + 1e+06 alpha_ob39_seg19[1] <= 999993.6552362755
 R9079: - C41 + 1e+06 alpha_ob39_seg19[1] <= 999993.6552362755
 R9080: C38 + 1e+06 alpha_ob39_seg19[2] <= 1.0000829266784199e+06
 R9081: C40 + 1e+06 alpha_ob39_seg19[2] <= 1.0000829266784199e+06
 R9082: C39 + 1e+06 alpha_ob39_seg19[3] <= 999992.1947642756
 R9083: C41 + 1e+06 alpha_ob39_seg19[3] <= 999992.1947642756
 R9084: alpha_ob39_seg19[0] + alpha_ob39_seg19[1] + alpha_ob39_seg19[2]
   + alpha_ob39_seg19[3] >= 1
 R9085: - C38 + 1e+06 alpha_ob40_seg19[0] <= 999912.265199801
 R9086: - C40 + 1e+06 alpha_ob40_seg19[0] <= 999912.265199801
 R9087: - C39 + 1e+06 alpha_ob40_seg19[1] <= 999987.7842112777
 R9088: - C41 + 1e+06 alpha_ob40_seg19[1] <= 999987.7842112777
 R9089: C38 + 1e+06 alpha_ob40_seg19[2] <= 1.000076834799801e+06
 R9090: C40 + 1e+06 alpha_ob40_seg19[2] <= 1.000076834799801e+06
 R9091: C39 + 1e+06 alpha_ob40_seg19[3] <= 1.0000096658112778e+06
 R9092: C41 + 1e+06 alpha_ob40_seg19[3] <= 1.0000096658112778e+06
 R9093: alpha_ob40_seg19[0] + alpha_ob40_seg19[1] + alpha_ob40_seg19[2]
   + alpha_ob40_seg19[3] >= 1
 R9094: - C38 + 1e+06 alpha_ob41_seg19[0] <= 999920.6152119379
 R9095: - C40 + 1e+06 alpha_ob41_seg19[0] <= 999920.6152119379
 R9096: - C39 + 1e+06 alpha_ob41_seg19[1] <= 999981.4341997246
 R9097: - C41 + 1e+06 alpha_ob41_seg19[1] <= 999981.4341997246
 R9098: C38 + 1e+06 alpha_ob41_seg19[2] <= 1.0000768348119379e+06
 R9099: C40 + 1e+06 alpha_ob41_seg19[2] <= 1.0000768348119379e+06
 R9100: C39 + 1e+06 alpha_ob41_seg19[3] <= 1.0000096657997245e+06
 R9101: C41 + 1e+06 alpha_ob41_seg19[3] <= 1.0000096657997245e+06
 R9102: alpha_ob41_seg19[0] + alpha_ob41_seg19[1] + alpha_ob41_seg19[2]
   + alpha_ob41_seg19[3] >= 1
 R9103: - C38 + 1e+06 alpha_ob42_seg19[0] <= 999919.2652
 R9104: - C40 + 1e+06 alpha_ob42_seg19[0] <= 999919.2652
 R9105: - C39 + 1e+06 alpha_ob42_seg19[1] <= 999981.4342
 R9106: - C41 + 1e+06 alpha_ob42_seg19[1] <= 999981.4342
 R9107: C38 + 1e+06 alpha_ob42_seg19[2] <= 1.0000768348e+06
 R9108: C40 + 1e+06 alpha_ob42_seg19[2] <= 1.0000768348e+06
 R9109: C39 + 1e+06 alpha_ob42_seg19[3] <= 1.0000160158000001e+06
 R9110: C41 + 1e+06 alpha_ob42_seg19[3] <= 1.0000160158000001e+06
 R9111: alpha_ob42_seg19[0] + alpha_ob42_seg19[1] + alpha_ob42_seg19[2]
   + alpha_ob42_seg19[3] >= 1
 R9112: - C38 + 1e+06 alpha_ob43_seg19[0] <= 999919.2652454559
 R9113: - C40 + 1e+06 alpha_ob43_seg19[0] <= 999919.2652454559
 R9114: - C39 + 1e+06 alpha_ob43_seg19[1] <= 999956.8341997246
 R9115: - C41 + 1e+06 alpha_ob43_seg19[1] <= 999956.8341997246
 R9116: C38 + 1e+06 alpha_ob43_seg19[2] <= 1.0000781848454559e+06
 R9117: C40 + 1e+06 alpha_ob43_seg19[2] <= 1.0000781848454559e+06
 R9118: C39 + 1e+06 alpha_ob43_seg19[3] <= 1.0000160157997246e+06
 R9119: C41 + 1e+06 alpha_ob43_seg19[3] <= 1.0000160157997246e+06
 R9120: alpha_ob43_seg19[0] + alpha_ob43_seg19[1] + alpha_ob43_seg19[2]
   + alpha_ob43_seg19[3] >= 1
 R9121: - C38 + 1e+06 alpha_ob44_seg19[0] <= 999991.83683
 R9122: - C40 + 1e+06 alpha_ob44_seg19[0] <= 999991.83683
 R9123: - C39 + 1e+06 alpha_ob44_seg19[1] <= 999940.589
 R9124: - C41 + 1e+06 alpha_ob44_seg19[1] <= 999940.589
 R9125: C38 + 1e+06 alpha_ob44_seg19[2] <= 1.00000401317e+06
 R9126: C40 + 1e+06 alpha_ob44_seg19[2] <= 1.00000401317e+06
 R9127: C39 + 1e+06 alpha_ob44_seg19[3] <= 1.000056861e+06
 R9128: C41 + 1e+06 alpha_ob44_seg19[3] <= 1.000056861e+06
 R9129: alpha_ob44_seg19[0] + alpha_ob44_seg19[1] + alpha_ob44_seg19[2]
   + alpha_ob44_seg19[3] >= 1
 R9130: - C38 + 1e+06 alpha_ob45_seg19[0] <= 999991.8368084199
 R9131: - C40 + 1e+06 alpha_ob45_seg19[0] <= 999991.8368084199
 R9132: - C39 + 1e+06 alpha_ob45_seg19[1] <= 999940.5890002755
 R9133: - C41 + 1e+06 alpha_ob45_seg19[1] <= 999940.5890002755
 R9134: C38 + 1e+06 alpha_ob45_seg19[2] <= 1.0000056131484199e+06
 R9135: C40 + 1e+06 alpha_ob45_seg19[2] <= 1.0000056131484199e+06
 R9136: C39 + 1e+06 alpha_ob45_seg19[3] <= 1.0000452610002755e+06
 R9137: C41 + 1e+06 alpha_ob45_seg19[3] <= 1.0000452610002755e+06
 R9138: alpha_ob45_seg19[0] + alpha_ob45_seg19[1] + alpha_ob45_seg19[2]
   + alpha_ob45_seg19[3] >= 1
 R9139: - C38 + 1e+06 alpha_ob46_seg19[0] <= 1.0000764021508709e+06
 R9140: - C40 + 1e+06 alpha_ob46_seg19[0] <= 1.0000764021508709e+06
 R9141: - C39 + 1e+06 alpha_ob46_seg19[1] <= 999957.1027002756
 R9142: - C41 + 1e+06 alpha_ob46_seg19[1] <= 999957.1027002756
 R9143: C38 + 1e+06 alpha_ob46_seg19[2] <= 999921.0477508709
 R9144: C40 + 1e+06 alpha_ob46_seg19[2] <= 999921.0477508709
 R9145: C39 + 1e+06 alpha_ob46_seg19[3] <= 1.0000137473002756e+06
 R9146: C41 + 1e+06 alpha_ob46_seg19[3] <= 1.0000137473002756e+06
 R9147: alpha_ob46_seg19[0] + alpha_ob46_seg19[1] + alpha_ob46_seg19[2]
   + alpha_ob46_seg19[3] >= 1
 R9148: - C38 + 1e+06 alpha_ob47_seg19[0] <= 1.0000445522e+06
 R9149: - C40 + 1e+06 alpha_ob47_seg19[0] <= 1.0000445522e+06
 R9150: - C39 + 1e+06 alpha_ob47_seg19[1] <= 999983.7027
 R9151: - C41 + 1e+06 alpha_ob47_seg19[1] <= 999983.7027
 R9152: C38 + 1e+06 alpha_ob47_seg19[2] <= 999921.0477999999
 R9153: C40 + 1e+06 alpha_ob47_seg19[2] <= 999921.0477999999
 R9154: C39 + 1e+06 alpha_ob47_seg19[3] <= 1.0000137473e+06
 R9155: C41 + 1e+06 alpha_ob47_seg19[3] <= 1.0000137473e+06
 R9156: alpha_ob47_seg19[0] + alpha_ob47_seg19[1] + alpha_ob47_seg19[2]
   + alpha_ob47_seg19[3] >= 1
 R9157: - C38 + 1e+06 alpha_ob48_seg19[0] <= 1.0000445522183661e+06
 R9158: - C40 + 1e+06 alpha_ob48_seg19[0] <= 1.0000445522183661e+06
 R9159: - C39 + 1e+06 alpha_ob48_seg19[1] <= 999973.8526997245
 R9160: - C41 + 1e+06 alpha_ob48_seg19[1] <= 999973.8526997245
 R9161: C38 + 1e+06 alpha_ob48_seg19[2] <= 999952.8978183661
 R9162: C40 + 1e+06 alpha_ob48_seg19[2] <= 999952.8978183661
 R9163: C39 + 1e+06 alpha_ob48_seg19[3] <= 1.0000137472997245e+06
 R9164: C41 + 1e+06 alpha_ob48_seg19[3] <= 1.0000137472997245e+06
 R9165: alpha_ob48_seg19[0] + alpha_ob48_seg19[1] + alpha_ob48_seg19[2]
   + alpha_ob48_seg19[3] >= 1
 R9166: - C38 + 1e+06 alpha_ob49_seg19[0] <= 1.000064332081634e+06
 R9167: - C40 + 1e+06 alpha_ob49_seg19[0] <= 1.000064332081634e+06
 R9168: - C39 + 1e+06 alpha_ob49_seg19[1] <= 999973.9401002756
 R9169: - C41 + 1e+06 alpha_ob49_seg19[1] <= 999973.9401002756
 R9170: C38 + 1e+06 alpha_ob49_seg19[2] <= 999933.1178816339
 R9171: C40 + 1e+06 alpha_ob49_seg19[2] <= 999933.1178816339
 R9172: C39 + 1e+06 alpha_ob49_seg19[3] <= 1.0000136599002755e+06
 R9173: C41 + 1e+06 alpha_ob49_seg19[3] <= 1.0000136599002755e+06
 R9174: alpha_ob49_seg19[0] + alpha_ob49_seg19[1] + alpha_ob49_seg19[2]
   + alpha_ob49_seg19[3] >= 1
 R9175: - C38 + 1e+06 alpha_ob50_seg19[0] <= 1.0000643321013774e+06
 R9176: - C40 + 1e+06 alpha_ob50_seg19[0] <= 1.0000643321013774e+06
 R9177: - C39 + 1e+06 alpha_ob50_seg19[1] <= 999983.1900997246
 R9178: - C41 + 1e+06 alpha_ob50_seg19[1] <= 999983.1900997246
 R9179: C38 + 1e+06 alpha_ob50_seg19[2] <= 999933.1179013775
 R9180: C40 + 1e+06 alpha_ob50_seg19[2] <= 999933.1179013775
 R9181: C39 + 1e+06 alpha_ob50_seg19[3] <= 1.0000136598997245e+06
 R9182: C41 + 1e+06 alpha_ob50_seg19[3] <= 1.0000136598997245e+06
 R9183: alpha_ob50_seg19[0] + alpha_ob50_seg19[1] + alpha_ob50_seg19[2]
   + alpha_ob50_seg19[3] >= 1
 R9184: C0 >= 0
 R9185: C1 >= 0
 R9186: C0 <= 85
 R9187: C1 <= 85
 R9188: C2 >= 0
 R9189: C3 >= 0
 R9190: C2 <= 85
 R9191: C3 <= 85
 R9192: C4 >= 0
 R9193: C5 >= 0
 R9194: C4 <= 85
 R9195: C5 <= 85
 R9196: C6 >= 0
 R9197: C7 >= 0
 R9198: C6 <= 85
 R9199: C7 <= 85
 R9200: C8 >= 0
 R9201: C9 >= 0
 R9202: C8 <= 85
 R9203: C9 <= 85
 R9204: C10 >= 0
 R9205: C11 >= 0
 R9206: C10 <= 85
 R9207: C11 <= 85
 R9208: C12 >= 0
 R9209: C13 >= 0
 R9210: C12 <= 85
 R9211: C13 <= 85
 R9212: C14 >= 0
 R9213: C15 >= 0
 R9214: C14 <= 85
 R9215: C15 <= 85
 R9216: C16 >= 0
 R9217: C17 >= 0
 R9218: C16 <= 85
 R9219: C17 <= 85
 R9220: C18 >= 0
 R9221: C19 >= 0
 R9222: C18 <= 85
 R9223: C19 <= 85
 R9224: C20 >= 0
 R9225: C21 >= 0
 R9226: C20 <= 85
 R9227: C21 <= 85
 R9228: C22 >= 0
 R9229: C23 >= 0
 R9230: C22 <= 85
 R9231: C23 <= 85
 R9232: C24 >= 0
 R9233: C25 >= 0
 R9234: C24 <= 85
 R9235: C25 <= 85
 R9236: C26 >= 0
 R9237: C27 >= 0
 R9238: C26 <= 85
 R9239: C27 <= 85
 R9240: C28 >= 0
 R9241: C29 >= 0
 R9242: C28 <= 85
 R9243: C29 <= 85
 R9244: C30 >= 0
 R9245: C31 >= 0
 R9246: C30 <= 85
 R9247: C31 <= 85
 R9248: C32 >= 0
 R9249: C33 >= 0
 R9250: C32 <= 85
 R9251: C33 <= 85
 R9252: C34 >= 0
 R9253: C35 >= 0
 R9254: C34 <= 85
 R9255: C35 <= 85
 R9256: C36 >= 0
 R9257: C37 >= 0
 R9258: C36 <= 85
 R9259: C37 <= 85
 R9260: C38 >= 0
 R9261: C39 >= 0
 R9262: C38 <= 85
 R9263: C39 <= 85
 R9264: C40 >= 0
 R9265: C41 >= 0
 R9266: C40 <= 85
 R9267: C41 <= 85
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
 alpha_ob50_seg2[3] alpha_ob0_seg3[0] alpha_ob0_seg3[1] alpha_ob0_seg3[2]
 alpha_ob0_seg3[3] alpha_ob1_seg3[0] alpha_ob1_seg3[1] alpha_ob1_seg3[2]
 alpha_ob1_seg3[3] alpha_ob2_seg3[0] alpha_ob2_seg3[1] alpha_ob2_seg3[2]
 alpha_ob2_seg3[3] alpha_ob3_seg3[0] alpha_ob3_seg3[1] alpha_ob3_seg3[2]
 alpha_ob3_seg3[3] alpha_ob4_seg3[0] alpha_ob4_seg3[1] alpha_ob4_seg3[2]
 alpha_ob4_seg3[3] alpha_ob5_seg3[0] alpha_ob5_seg3[1] alpha_ob5_seg3[2]
 alpha_ob5_seg3[3] alpha_ob6_seg3[0] alpha_ob6_seg3[1] alpha_ob6_seg3[2]
 alpha_ob6_seg3[3] alpha_ob7_seg3[0] alpha_ob7_seg3[1] alpha_ob7_seg3[2]
 alpha_ob7_seg3[3] alpha_ob8_seg3[0] alpha_ob8_seg3[1] alpha_ob8_seg3[2]
 alpha_ob8_seg3[3] alpha_ob9_seg3[0] alpha_ob9_seg3[1] alpha_ob9_seg3[2]
 alpha_ob9_seg3[3] alpha_ob10_seg3[0] alpha_ob10_seg3[1] alpha_ob10_seg3[2]
 alpha_ob10_seg3[3] alpha_ob11_seg3[0] alpha_ob11_seg3[1]
 alpha_ob11_seg3[2] alpha_ob11_seg3[3] alpha_ob12_seg3[0]
 alpha_ob12_seg3[1] alpha_ob12_seg3[2] alpha_ob12_seg3[3]
 alpha_ob13_seg3[0] alpha_ob13_seg3[1] alpha_ob13_seg3[2]
 alpha_ob13_seg3[3] alpha_ob14_seg3[0] alpha_ob14_seg3[1]
 alpha_ob14_seg3[2] alpha_ob14_seg3[3] alpha_ob15_seg3[0]
 alpha_ob15_seg3[1] alpha_ob15_seg3[2] alpha_ob15_seg3[3]
 alpha_ob16_seg3[0] alpha_ob16_seg3[1] alpha_ob16_seg3[2]
 alpha_ob16_seg3[3] alpha_ob17_seg3[0] alpha_ob17_seg3[1]
 alpha_ob17_seg3[2] alpha_ob17_seg3[3] alpha_ob18_seg3[0]
 alpha_ob18_seg3[1] alpha_ob18_seg3[2] alpha_ob18_seg3[3]
 alpha_ob19_seg3[0] alpha_ob19_seg3[1] alpha_ob19_seg3[2]
 alpha_ob19_seg3[3] alpha_ob20_seg3[0] alpha_ob20_seg3[1]
 alpha_ob20_seg3[2] alpha_ob20_seg3[3] alpha_ob21_seg3[0]
 alpha_ob21_seg3[1] alpha_ob21_seg3[2] alpha_ob21_seg3[3]
 alpha_ob22_seg3[0] alpha_ob22_seg3[1] alpha_ob22_seg3[2]
 alpha_ob22_seg3[3] alpha_ob23_seg3[0] alpha_ob23_seg3[1]
 alpha_ob23_seg3[2] alpha_ob23_seg3[3] alpha_ob24_seg3[0]
 alpha_ob24_seg3[1] alpha_ob24_seg3[2] alpha_ob24_seg3[3]
 alpha_ob25_seg3[0] alpha_ob25_seg3[1] alpha_ob25_seg3[2]
 alpha_ob25_seg3[3] alpha_ob26_seg3[0] alpha_ob26_seg3[1]
 alpha_ob26_seg3[2] alpha_ob26_seg3[3] alpha_ob27_seg3[0]
 alpha_ob27_seg3[1] alpha_ob27_seg3[2] alpha_ob27_seg3[3]
 alpha_ob28_seg3[0] alpha_ob28_seg3[1] alpha_ob28_seg3[2]
 alpha_ob28_seg3[3] alpha_ob29_seg3[0] alpha_ob29_seg3[1]
 alpha_ob29_seg3[2] alpha_ob29_seg3[3] alpha_ob30_seg3[0]
 alpha_ob30_seg3[1] alpha_ob30_seg3[2] alpha_ob30_seg3[3]
 alpha_ob31_seg3[0] alpha_ob31_seg3[1] alpha_ob31_seg3[2]
 alpha_ob31_seg3[3] alpha_ob32_seg3[0] alpha_ob32_seg3[1]
 alpha_ob32_seg3[2] alpha_ob32_seg3[3] alpha_ob33_seg3[0]
 alpha_ob33_seg3[1] alpha_ob33_seg3[2] alpha_ob33_seg3[3]
 alpha_ob34_seg3[0] alpha_ob34_seg3[1] alpha_ob34_seg3[2]
 alpha_ob34_seg3[3] alpha_ob35_seg3[0] alpha_ob35_seg3[1]
 alpha_ob35_seg3[2] alpha_ob35_seg3[3] alpha_ob36_seg3[0]
 alpha_ob36_seg3[1] alpha_ob36_seg3[2] alpha_ob36_seg3[3]
 alpha_ob37_seg3[0] alpha_ob37_seg3[1] alpha_ob37_seg3[2]
 alpha_ob37_seg3[3] alpha_ob38_seg3[0] alpha_ob38_seg3[1]
 alpha_ob38_seg3[2] alpha_ob38_seg3[3] alpha_ob39_seg3[0]
 alpha_ob39_seg3[1] alpha_ob39_seg3[2] alpha_ob39_seg3[3]
 alpha_ob40_seg3[0] alpha_ob40_seg3[1] alpha_ob40_seg3[2]
 alpha_ob40_seg3[3] alpha_ob41_seg3[0] alpha_ob41_seg3[1]
 alpha_ob41_seg3[2] alpha_ob41_seg3[3] alpha_ob42_seg3[0]
 alpha_ob42_seg3[1] alpha_ob42_seg3[2] alpha_ob42_seg3[3]
 alpha_ob43_seg3[0] alpha_ob43_seg3[1] alpha_ob43_seg3[2]
 alpha_ob43_seg3[3] alpha_ob44_seg3[0] alpha_ob44_seg3[1]
 alpha_ob44_seg3[2] alpha_ob44_seg3[3] alpha_ob45_seg3[0]
 alpha_ob45_seg3[1] alpha_ob45_seg3[2] alpha_ob45_seg3[3]
 alpha_ob46_seg3[0] alpha_ob46_seg3[1] alpha_ob46_seg3[2]
 alpha_ob46_seg3[3] alpha_ob47_seg3[0] alpha_ob47_seg3[1]
 alpha_ob47_seg3[2] alpha_ob47_seg3[3] alpha_ob48_seg3[0]
 alpha_ob48_seg3[1] alpha_ob48_seg3[2] alpha_ob48_seg3[3]
 alpha_ob49_seg3[0] alpha_ob49_seg3[1] alpha_ob49_seg3[2]
 alpha_ob49_seg3[3] alpha_ob50_seg3[0] alpha_ob50_seg3[1]
 alpha_ob50_seg3[2] alpha_ob50_seg3[3] alpha_ob0_seg4[0] alpha_ob0_seg4[1]
 alpha_ob0_seg4[2] alpha_ob0_seg4[3] alpha_ob1_seg4[0] alpha_ob1_seg4[1]
 alpha_ob1_seg4[2] alpha_ob1_seg4[3] alpha_ob2_seg4[0] alpha_ob2_seg4[1]
 alpha_ob2_seg4[2] alpha_ob2_seg4[3] alpha_ob3_seg4[0] alpha_ob3_seg4[1]
 alpha_ob3_seg4[2] alpha_ob3_seg4[3] alpha_ob4_seg4[0] alpha_ob4_seg4[1]
 alpha_ob4_seg4[2] alpha_ob4_seg4[3] alpha_ob5_seg4[0] alpha_ob5_seg4[1]
 alpha_ob5_seg4[2] alpha_ob5_seg4[3] alpha_ob6_seg4[0] alpha_ob6_seg4[1]
 alpha_ob6_seg4[2] alpha_ob6_seg4[3] alpha_ob7_seg4[0] alpha_ob7_seg4[1]
 alpha_ob7_seg4[2] alpha_ob7_seg4[3] alpha_ob8_seg4[0] alpha_ob8_seg4[1]
 alpha_ob8_seg4[2] alpha_ob8_seg4[3] alpha_ob9_seg4[0] alpha_ob9_seg4[1]
 alpha_ob9_seg4[2] alpha_ob9_seg4[3] alpha_ob10_seg4[0] alpha_ob10_seg4[1]
 alpha_ob10_seg4[2] alpha_ob10_seg4[3] alpha_ob11_seg4[0]
 alpha_ob11_seg4[1] alpha_ob11_seg4[2] alpha_ob11_seg4[3]
 alpha_ob12_seg4[0] alpha_ob12_seg4[1] alpha_ob12_seg4[2]
 alpha_ob12_seg4[3] alpha_ob13_seg4[0] alpha_ob13_seg4[1]
 alpha_ob13_seg4[2] alpha_ob13_seg4[3] alpha_ob14_seg4[0]
 alpha_ob14_seg4[1] alpha_ob14_seg4[2] alpha_ob14_seg4[3]
 alpha_ob15_seg4[0] alpha_ob15_seg4[1] alpha_ob15_seg4[2]
 alpha_ob15_seg4[3] alpha_ob16_seg4[0] alpha_ob16_seg4[1]
 alpha_ob16_seg4[2] alpha_ob16_seg4[3] alpha_ob17_seg4[0]
 alpha_ob17_seg4[1] alpha_ob17_seg4[2] alpha_ob17_seg4[3]
 alpha_ob18_seg4[0] alpha_ob18_seg4[1] alpha_ob18_seg4[2]
 alpha_ob18_seg4[3] alpha_ob19_seg4[0] alpha_ob19_seg4[1]
 alpha_ob19_seg4[2] alpha_ob19_seg4[3] alpha_ob20_seg4[0]
 alpha_ob20_seg4[1] alpha_ob20_seg4[2] alpha_ob20_seg4[3]
 alpha_ob21_seg4[0] alpha_ob21_seg4[1] alpha_ob21_seg4[2]
 alpha_ob21_seg4[3] alpha_ob22_seg4[0] alpha_ob22_seg4[1]
 alpha_ob22_seg4[2] alpha_ob22_seg4[3] alpha_ob23_seg4[0]
 alpha_ob23_seg4[1] alpha_ob23_seg4[2] alpha_ob23_seg4[3]
 alpha_ob24_seg4[0] alpha_ob24_seg4[1] alpha_ob24_seg4[2]
 alpha_ob24_seg4[3] alpha_ob25_seg4[0] alpha_ob25_seg4[1]
 alpha_ob25_seg4[2] alpha_ob25_seg4[3] alpha_ob26_seg4[0]
 alpha_ob26_seg4[1] alpha_ob26_seg4[2] alpha_ob26_seg4[3]
 alpha_ob27_seg4[0] alpha_ob27_seg4[1] alpha_ob27_seg4[2]
 alpha_ob27_seg4[3] alpha_ob28_seg4[0] alpha_ob28_seg4[1]
 alpha_ob28_seg4[2] alpha_ob28_seg4[3] alpha_ob29_seg4[0]
 alpha_ob29_seg4[1] alpha_ob29_seg4[2] alpha_ob29_seg4[3]
 alpha_ob30_seg4[0] alpha_ob30_seg4[1] alpha_ob30_seg4[2]
 alpha_ob30_seg4[3] alpha_ob31_seg4[0] alpha_ob31_seg4[1]
 alpha_ob31_seg4[2] alpha_ob31_seg4[3] alpha_ob32_seg4[0]
 alpha_ob32_seg4[1] alpha_ob32_seg4[2] alpha_ob32_seg4[3]
 alpha_ob33_seg4[0] alpha_ob33_seg4[1] alpha_ob33_seg4[2]
 alpha_ob33_seg4[3] alpha_ob34_seg4[0] alpha_ob34_seg4[1]
 alpha_ob34_seg4[2] alpha_ob34_seg4[3] alpha_ob35_seg4[0]
 alpha_ob35_seg4[1] alpha_ob35_seg4[2] alpha_ob35_seg4[3]
 alpha_ob36_seg4[0] alpha_ob36_seg4[1] alpha_ob36_seg4[2]
 alpha_ob36_seg4[3] alpha_ob37_seg4[0] alpha_ob37_seg4[1]
 alpha_ob37_seg4[2] alpha_ob37_seg4[3] alpha_ob38_seg4[0]
 alpha_ob38_seg4[1] alpha_ob38_seg4[2] alpha_ob38_seg4[3]
 alpha_ob39_seg4[0] alpha_ob39_seg4[1] alpha_ob39_seg4[2]
 alpha_ob39_seg4[3] alpha_ob40_seg4[0] alpha_ob40_seg4[1]
 alpha_ob40_seg4[2] alpha_ob40_seg4[3] alpha_ob41_seg4[0]
 alpha_ob41_seg4[1] alpha_ob41_seg4[2] alpha_ob41_seg4[3]
 alpha_ob42_seg4[0] alpha_ob42_seg4[1] alpha_ob42_seg4[2]
 alpha_ob42_seg4[3] alpha_ob43_seg4[0] alpha_ob43_seg4[1]
 alpha_ob43_seg4[2] alpha_ob43_seg4[3] alpha_ob44_seg4[0]
 alpha_ob44_seg4[1] alpha_ob44_seg4[2] alpha_ob44_seg4[3]
 alpha_ob45_seg4[0] alpha_ob45_seg4[1] alpha_ob45_seg4[2]
 alpha_ob45_seg4[3] alpha_ob46_seg4[0] alpha_ob46_seg4[1]
 alpha_ob46_seg4[2] alpha_ob46_seg4[3] alpha_ob47_seg4[0]
 alpha_ob47_seg4[1] alpha_ob47_seg4[2] alpha_ob47_seg4[3]
 alpha_ob48_seg4[0] alpha_ob48_seg4[1] alpha_ob48_seg4[2]
 alpha_ob48_seg4[3] alpha_ob49_seg4[0] alpha_ob49_seg4[1]
 alpha_ob49_seg4[2] alpha_ob49_seg4[3] alpha_ob50_seg4[0]
 alpha_ob50_seg4[1] alpha_ob50_seg4[2] alpha_ob50_seg4[3] alpha_ob0_seg5[0]
 alpha_ob0_seg5[1] alpha_ob0_seg5[2] alpha_ob0_seg5[3] alpha_ob1_seg5[0]
 alpha_ob1_seg5[1] alpha_ob1_seg5[2] alpha_ob1_seg5[3] alpha_ob2_seg5[0]
 alpha_ob2_seg5[1] alpha_ob2_seg5[2] alpha_ob2_seg5[3] alpha_ob3_seg5[0]
 alpha_ob3_seg5[1] alpha_ob3_seg5[2] alpha_ob3_seg5[3] alpha_ob4_seg5[0]
 alpha_ob4_seg5[1] alpha_ob4_seg5[2] alpha_ob4_seg5[3] alpha_ob5_seg5[0]
 alpha_ob5_seg5[1] alpha_ob5_seg5[2] alpha_ob5_seg5[3] alpha_ob6_seg5[0]
 alpha_ob6_seg5[1] alpha_ob6_seg5[2] alpha_ob6_seg5[3] alpha_ob7_seg5[0]
 alpha_ob7_seg5[1] alpha_ob7_seg5[2] alpha_ob7_seg5[3] alpha_ob8_seg5[0]
 alpha_ob8_seg5[1] alpha_ob8_seg5[2] alpha_ob8_seg5[3] alpha_ob9_seg5[0]
 alpha_ob9_seg5[1] alpha_ob9_seg5[2] alpha_ob9_seg5[3] alpha_ob10_seg5[0]
 alpha_ob10_seg5[1] alpha_ob10_seg5[2] alpha_ob10_seg5[3]
 alpha_ob11_seg5[0] alpha_ob11_seg5[1] alpha_ob11_seg5[2]
 alpha_ob11_seg5[3] alpha_ob12_seg5[0] alpha_ob12_seg5[1]
 alpha_ob12_seg5[2] alpha_ob12_seg5[3] alpha_ob13_seg5[0]
 alpha_ob13_seg5[1] alpha_ob13_seg5[2] alpha_ob13_seg5[3]
 alpha_ob14_seg5[0] alpha_ob14_seg5[1] alpha_ob14_seg5[2]
 alpha_ob14_seg5[3] alpha_ob15_seg5[0] alpha_ob15_seg5[1]
 alpha_ob15_seg5[2] alpha_ob15_seg5[3] alpha_ob16_seg5[0]
 alpha_ob16_seg5[1] alpha_ob16_seg5[2] alpha_ob16_seg5[3]
 alpha_ob17_seg5[0] alpha_ob17_seg5[1] alpha_ob17_seg5[2]
 alpha_ob17_seg5[3] alpha_ob18_seg5[0] alpha_ob18_seg5[1]
 alpha_ob18_seg5[2] alpha_ob18_seg5[3] alpha_ob19_seg5[0]
 alpha_ob19_seg5[1] alpha_ob19_seg5[2] alpha_ob19_seg5[3]
 alpha_ob20_seg5[0] alpha_ob20_seg5[1] alpha_ob20_seg5[2]
 alpha_ob20_seg5[3] alpha_ob21_seg5[0] alpha_ob21_seg5[1]
 alpha_ob21_seg5[2] alpha_ob21_seg5[3] alpha_ob22_seg5[0]
 alpha_ob22_seg5[1] alpha_ob22_seg5[2] alpha_ob22_seg5[3]
 alpha_ob23_seg5[0] alpha_ob23_seg5[1] alpha_ob23_seg5[2]
 alpha_ob23_seg5[3] alpha_ob24_seg5[0] alpha_ob24_seg5[1]
 alpha_ob24_seg5[2] alpha_ob24_seg5[3] alpha_ob25_seg5[0]
 alpha_ob25_seg5[1] alpha_ob25_seg5[2] alpha_ob25_seg5[3]
 alpha_ob26_seg5[0] alpha_ob26_seg5[1] alpha_ob26_seg5[2]
 alpha_ob26_seg5[3] alpha_ob27_seg5[0] alpha_ob27_seg5[1]
 alpha_ob27_seg5[2] alpha_ob27_seg5[3] alpha_ob28_seg5[0]
 alpha_ob28_seg5[1] alpha_ob28_seg5[2] alpha_ob28_seg5[3]
 alpha_ob29_seg5[0] alpha_ob29_seg5[1] alpha_ob29_seg5[2]
 alpha_ob29_seg5[3] alpha_ob30_seg5[0] alpha_ob30_seg5[1]
 alpha_ob30_seg5[2] alpha_ob30_seg5[3] alpha_ob31_seg5[0]
 alpha_ob31_seg5[1] alpha_ob31_seg5[2] alpha_ob31_seg5[3]
 alpha_ob32_seg5[0] alpha_ob32_seg5[1] alpha_ob32_seg5[2]
 alpha_ob32_seg5[3] alpha_ob33_seg5[0] alpha_ob33_seg5[1]
 alpha_ob33_seg5[2] alpha_ob33_seg5[3] alpha_ob34_seg5[0]
 alpha_ob34_seg5[1] alpha_ob34_seg5[2] alpha_ob34_seg5[3]
 alpha_ob35_seg5[0] alpha_ob35_seg5[1] alpha_ob35_seg5[2]
 alpha_ob35_seg5[3] alpha_ob36_seg5[0] alpha_ob36_seg5[1]
 alpha_ob36_seg5[2] alpha_ob36_seg5[3] alpha_ob37_seg5[0]
 alpha_ob37_seg5[1] alpha_ob37_seg5[2] alpha_ob37_seg5[3]
 alpha_ob38_seg5[0] alpha_ob38_seg5[1] alpha_ob38_seg5[2]
 alpha_ob38_seg5[3] alpha_ob39_seg5[0] alpha_ob39_seg5[1]
 alpha_ob39_seg5[2] alpha_ob39_seg5[3] alpha_ob40_seg5[0]
 alpha_ob40_seg5[1] alpha_ob40_seg5[2] alpha_ob40_seg5[3]
 alpha_ob41_seg5[0] alpha_ob41_seg5[1] alpha_ob41_seg5[2]
 alpha_ob41_seg5[3] alpha_ob42_seg5[0] alpha_ob42_seg5[1]
 alpha_ob42_seg5[2] alpha_ob42_seg5[3] alpha_ob43_seg5[0]
 alpha_ob43_seg5[1] alpha_ob43_seg5[2] alpha_ob43_seg5[3]
 alpha_ob44_seg5[0] alpha_ob44_seg5[1] alpha_ob44_seg5[2]
 alpha_ob44_seg5[3] alpha_ob45_seg5[0] alpha_ob45_seg5[1]
 alpha_ob45_seg5[2] alpha_ob45_seg5[3] alpha_ob46_seg5[0]
 alpha_ob46_seg5[1] alpha_ob46_seg5[2] alpha_ob46_seg5[3]
 alpha_ob47_seg5[0] alpha_ob47_seg5[1] alpha_ob47_seg5[2]
 alpha_ob47_seg5[3] alpha_ob48_seg5[0] alpha_ob48_seg5[1]
 alpha_ob48_seg5[2] alpha_ob48_seg5[3] alpha_ob49_seg5[0]
 alpha_ob49_seg5[1] alpha_ob49_seg5[2] alpha_ob49_seg5[3]
 alpha_ob50_seg5[0] alpha_ob50_seg5[1] alpha_ob50_seg5[2]
 alpha_ob50_seg5[3] alpha_ob0_seg6[0] alpha_ob0_seg6[1] alpha_ob0_seg6[2]
 alpha_ob0_seg6[3] alpha_ob1_seg6[0] alpha_ob1_seg6[1] alpha_ob1_seg6[2]
 alpha_ob1_seg6[3] alpha_ob2_seg6[0] alpha_ob2_seg6[1] alpha_ob2_seg6[2]
 alpha_ob2_seg6[3] alpha_ob3_seg6[0] alpha_ob3_seg6[1] alpha_ob3_seg6[2]
 alpha_ob3_seg6[3] alpha_ob4_seg6[0] alpha_ob4_seg6[1] alpha_ob4_seg6[2]
 alpha_ob4_seg6[3] alpha_ob5_seg6[0] alpha_ob5_seg6[1] alpha_ob5_seg6[2]
 alpha_ob5_seg6[3] alpha_ob6_seg6[0] alpha_ob6_seg6[1] alpha_ob6_seg6[2]
 alpha_ob6_seg6[3] alpha_ob7_seg6[0] alpha_ob7_seg6[1] alpha_ob7_seg6[2]
 alpha_ob7_seg6[3] alpha_ob8_seg6[0] alpha_ob8_seg6[1] alpha_ob8_seg6[2]
 alpha_ob8_seg6[3] alpha_ob9_seg6[0] alpha_ob9_seg6[1] alpha_ob9_seg6[2]
 alpha_ob9_seg6[3] alpha_ob10_seg6[0] alpha_ob10_seg6[1] alpha_ob10_seg6[2]
 alpha_ob10_seg6[3] alpha_ob11_seg6[0] alpha_ob11_seg6[1]
 alpha_ob11_seg6[2] alpha_ob11_seg6[3] alpha_ob12_seg6[0]
 alpha_ob12_seg6[1] alpha_ob12_seg6[2] alpha_ob12_seg6[3]
 alpha_ob13_seg6[0] alpha_ob13_seg6[1] alpha_ob13_seg6[2]
 alpha_ob13_seg6[3] alpha_ob14_seg6[0] alpha_ob14_seg6[1]
 alpha_ob14_seg6[2] alpha_ob14_seg6[3] alpha_ob15_seg6[0]
 alpha_ob15_seg6[1] alpha_ob15_seg6[2] alpha_ob15_seg6[3]
 alpha_ob16_seg6[0] alpha_ob16_seg6[1] alpha_ob16_seg6[2]
 alpha_ob16_seg6[3] alpha_ob17_seg6[0] alpha_ob17_seg6[1]
 alpha_ob17_seg6[2] alpha_ob17_seg6[3] alpha_ob18_seg6[0]
 alpha_ob18_seg6[1] alpha_ob18_seg6[2] alpha_ob18_seg6[3]
 alpha_ob19_seg6[0] alpha_ob19_seg6[1] alpha_ob19_seg6[2]
 alpha_ob19_seg6[3] alpha_ob20_seg6[0] alpha_ob20_seg6[1]
 alpha_ob20_seg6[2] alpha_ob20_seg6[3] alpha_ob21_seg6[0]
 alpha_ob21_seg6[1] alpha_ob21_seg6[2] alpha_ob21_seg6[3]
 alpha_ob22_seg6[0] alpha_ob22_seg6[1] alpha_ob22_seg6[2]
 alpha_ob22_seg6[3] alpha_ob23_seg6[0] alpha_ob23_seg6[1]
 alpha_ob23_seg6[2] alpha_ob23_seg6[3] alpha_ob24_seg6[0]
 alpha_ob24_seg6[1] alpha_ob24_seg6[2] alpha_ob24_seg6[3]
 alpha_ob25_seg6[0] alpha_ob25_seg6[1] alpha_ob25_seg6[2]
 alpha_ob25_seg6[3] alpha_ob26_seg6[0] alpha_ob26_seg6[1]
 alpha_ob26_seg6[2] alpha_ob26_seg6[3] alpha_ob27_seg6[0]
 alpha_ob27_seg6[1] alpha_ob27_seg6[2] alpha_ob27_seg6[3]
 alpha_ob28_seg6[0] alpha_ob28_seg6[1] alpha_ob28_seg6[2]
 alpha_ob28_seg6[3] alpha_ob29_seg6[0] alpha_ob29_seg6[1]
 alpha_ob29_seg6[2] alpha_ob29_seg6[3] alpha_ob30_seg6[0]
 alpha_ob30_seg6[1] alpha_ob30_seg6[2] alpha_ob30_seg6[3]
 alpha_ob31_seg6[0] alpha_ob31_seg6[1] alpha_ob31_seg6[2]
 alpha_ob31_seg6[3] alpha_ob32_seg6[0] alpha_ob32_seg6[1]
 alpha_ob32_seg6[2] alpha_ob32_seg6[3] alpha_ob33_seg6[0]
 alpha_ob33_seg6[1] alpha_ob33_seg6[2] alpha_ob33_seg6[3]
 alpha_ob34_seg6[0] alpha_ob34_seg6[1] alpha_ob34_seg6[2]
 alpha_ob34_seg6[3] alpha_ob35_seg6[0] alpha_ob35_seg6[1]
 alpha_ob35_seg6[2] alpha_ob35_seg6[3] alpha_ob36_seg6[0]
 alpha_ob36_seg6[1] alpha_ob36_seg6[2] alpha_ob36_seg6[3]
 alpha_ob37_seg6[0] alpha_ob37_seg6[1] alpha_ob37_seg6[2]
 alpha_ob37_seg6[3] alpha_ob38_seg6[0] alpha_ob38_seg6[1]
 alpha_ob38_seg6[2] alpha_ob38_seg6[3] alpha_ob39_seg6[0]
 alpha_ob39_seg6[1] alpha_ob39_seg6[2] alpha_ob39_seg6[3]
 alpha_ob40_seg6[0] alpha_ob40_seg6[1] alpha_ob40_seg6[2]
 alpha_ob40_seg6[3] alpha_ob41_seg6[0] alpha_ob41_seg6[1]
 alpha_ob41_seg6[2] alpha_ob41_seg6[3] alpha_ob42_seg6[0]
 alpha_ob42_seg6[1] alpha_ob42_seg6[2] alpha_ob42_seg6[3]
 alpha_ob43_seg6[0] alpha_ob43_seg6[1] alpha_ob43_seg6[2]
 alpha_ob43_seg6[3] alpha_ob44_seg6[0] alpha_ob44_seg6[1]
 alpha_ob44_seg6[2] alpha_ob44_seg6[3] alpha_ob45_seg6[0]
 alpha_ob45_seg6[1] alpha_ob45_seg6[2] alpha_ob45_seg6[3]
 alpha_ob46_seg6[0] alpha_ob46_seg6[1] alpha_ob46_seg6[2]
 alpha_ob46_seg6[3] alpha_ob47_seg6[0] alpha_ob47_seg6[1]
 alpha_ob47_seg6[2] alpha_ob47_seg6[3] alpha_ob48_seg6[0]
 alpha_ob48_seg6[1] alpha_ob48_seg6[2] alpha_ob48_seg6[3]
 alpha_ob49_seg6[0] alpha_ob49_seg6[1] alpha_ob49_seg6[2]
 alpha_ob49_seg6[3] alpha_ob50_seg6[0] alpha_ob50_seg6[1]
 alpha_ob50_seg6[2] alpha_ob50_seg6[3] alpha_ob0_seg7[0] alpha_ob0_seg7[1]
 alpha_ob0_seg7[2] alpha_ob0_seg7[3] alpha_ob1_seg7[0] alpha_ob1_seg7[1]
 alpha_ob1_seg7[2] alpha_ob1_seg7[3] alpha_ob2_seg7[0] alpha_ob2_seg7[1]
 alpha_ob2_seg7[2] alpha_ob2_seg7[3] alpha_ob3_seg7[0] alpha_ob3_seg7[1]
 alpha_ob3_seg7[2] alpha_ob3_seg7[3] alpha_ob4_seg7[0] alpha_ob4_seg7[1]
 alpha_ob4_seg7[2] alpha_ob4_seg7[3] alpha_ob5_seg7[0] alpha_ob5_seg7[1]
 alpha_ob5_seg7[2] alpha_ob5_seg7[3] alpha_ob6_seg7[0] alpha_ob6_seg7[1]
 alpha_ob6_seg7[2] alpha_ob6_seg7[3] alpha_ob7_seg7[0] alpha_ob7_seg7[1]
 alpha_ob7_seg7[2] alpha_ob7_seg7[3] alpha_ob8_seg7[0] alpha_ob8_seg7[1]
 alpha_ob8_seg7[2] alpha_ob8_seg7[3] alpha_ob9_seg7[0] alpha_ob9_seg7[1]
 alpha_ob9_seg7[2] alpha_ob9_seg7[3] alpha_ob10_seg7[0] alpha_ob10_seg7[1]
 alpha_ob10_seg7[2] alpha_ob10_seg7[3] alpha_ob11_seg7[0]
 alpha_ob11_seg7[1] alpha_ob11_seg7[2] alpha_ob11_seg7[3]
 alpha_ob12_seg7[0] alpha_ob12_seg7[1] alpha_ob12_seg7[2]
 alpha_ob12_seg7[3] alpha_ob13_seg7[0] alpha_ob13_seg7[1]
 alpha_ob13_seg7[2] alpha_ob13_seg7[3] alpha_ob14_seg7[0]
 alpha_ob14_seg7[1] alpha_ob14_seg7[2] alpha_ob14_seg7[3]
 alpha_ob15_seg7[0] alpha_ob15_seg7[1] alpha_ob15_seg7[2]
 alpha_ob15_seg7[3] alpha_ob16_seg7[0] alpha_ob16_seg7[1]
 alpha_ob16_seg7[2] alpha_ob16_seg7[3] alpha_ob17_seg7[0]
 alpha_ob17_seg7[1] alpha_ob17_seg7[2] alpha_ob17_seg7[3]
 alpha_ob18_seg7[0] alpha_ob18_seg7[1] alpha_ob18_seg7[2]
 alpha_ob18_seg7[3] alpha_ob19_seg7[0] alpha_ob19_seg7[1]
 alpha_ob19_seg7[2] alpha_ob19_seg7[3] alpha_ob20_seg7[0]
 alpha_ob20_seg7[1] alpha_ob20_seg7[2] alpha_ob20_seg7[3]
 alpha_ob21_seg7[0] alpha_ob21_seg7[1] alpha_ob21_seg7[2]
 alpha_ob21_seg7[3] alpha_ob22_seg7[0] alpha_ob22_seg7[1]
 alpha_ob22_seg7[2] alpha_ob22_seg7[3] alpha_ob23_seg7[0]
 alpha_ob23_seg7[1] alpha_ob23_seg7[2] alpha_ob23_seg7[3]
 alpha_ob24_seg7[0] alpha_ob24_seg7[1] alpha_ob24_seg7[2]
 alpha_ob24_seg7[3] alpha_ob25_seg7[0] alpha_ob25_seg7[1]
 alpha_ob25_seg7[2] alpha_ob25_seg7[3] alpha_ob26_seg7[0]
 alpha_ob26_seg7[1] alpha_ob26_seg7[2] alpha_ob26_seg7[3]
 alpha_ob27_seg7[0] alpha_ob27_seg7[1] alpha_ob27_seg7[2]
 alpha_ob27_seg7[3] alpha_ob28_seg7[0] alpha_ob28_seg7[1]
 alpha_ob28_seg7[2] alpha_ob28_seg7[3] alpha_ob29_seg7[0]
 alpha_ob29_seg7[1] alpha_ob29_seg7[2] alpha_ob29_seg7[3]
 alpha_ob30_seg7[0] alpha_ob30_seg7[1] alpha_ob30_seg7[2]
 alpha_ob30_seg7[3] alpha_ob31_seg7[0] alpha_ob31_seg7[1]
 alpha_ob31_seg7[2] alpha_ob31_seg7[3] alpha_ob32_seg7[0]
 alpha_ob32_seg7[1] alpha_ob32_seg7[2] alpha_ob32_seg7[3]
 alpha_ob33_seg7[0] alpha_ob33_seg7[1] alpha_ob33_seg7[2]
 alpha_ob33_seg7[3] alpha_ob34_seg7[0] alpha_ob34_seg7[1]
 alpha_ob34_seg7[2] alpha_ob34_seg7[3] alpha_ob35_seg7[0]
 alpha_ob35_seg7[1] alpha_ob35_seg7[2] alpha_ob35_seg7[3]
 alpha_ob36_seg7[0] alpha_ob36_seg7[1] alpha_ob36_seg7[2]
 alpha_ob36_seg7[3] alpha_ob37_seg7[0] alpha_ob37_seg7[1]
 alpha_ob37_seg7[2] alpha_ob37_seg7[3] alpha_ob38_seg7[0]
 alpha_ob38_seg7[1] alpha_ob38_seg7[2] alpha_ob38_seg7[3]
 alpha_ob39_seg7[0] alpha_ob39_seg7[1] alpha_ob39_seg7[2]
 alpha_ob39_seg7[3] alpha_ob40_seg7[0] alpha_ob40_seg7[1]
 alpha_ob40_seg7[2] alpha_ob40_seg7[3] alpha_ob41_seg7[0]
 alpha_ob41_seg7[1] alpha_ob41_seg7[2] alpha_ob41_seg7[3]
 alpha_ob42_seg7[0] alpha_ob42_seg7[1] alpha_ob42_seg7[2]
 alpha_ob42_seg7[3] alpha_ob43_seg7[0] alpha_ob43_seg7[1]
 alpha_ob43_seg7[2] alpha_ob43_seg7[3] alpha_ob44_seg7[0]
 alpha_ob44_seg7[1] alpha_ob44_seg7[2] alpha_ob44_seg7[3]
 alpha_ob45_seg7[0] alpha_ob45_seg7[1] alpha_ob45_seg7[2]
 alpha_ob45_seg7[3] alpha_ob46_seg7[0] alpha_ob46_seg7[1]
 alpha_ob46_seg7[2] alpha_ob46_seg7[3] alpha_ob47_seg7[0]
 alpha_ob47_seg7[1] alpha_ob47_seg7[2] alpha_ob47_seg7[3]
 alpha_ob48_seg7[0] alpha_ob48_seg7[1] alpha_ob48_seg7[2]
 alpha_ob48_seg7[3] alpha_ob49_seg7[0] alpha_ob49_seg7[1]
 alpha_ob49_seg7[2] alpha_ob49_seg7[3] alpha_ob50_seg7[0]
 alpha_ob50_seg7[1] alpha_ob50_seg7[2] alpha_ob50_seg7[3] alpha_ob0_seg8[0]
 alpha_ob0_seg8[1] alpha_ob0_seg8[2] alpha_ob0_seg8[3] alpha_ob1_seg8[0]
 alpha_ob1_seg8[1] alpha_ob1_seg8[2] alpha_ob1_seg8[3] alpha_ob2_seg8[0]
 alpha_ob2_seg8[1] alpha_ob2_seg8[2] alpha_ob2_seg8[3] alpha_ob3_seg8[0]
 alpha_ob3_seg8[1] alpha_ob3_seg8[2] alpha_ob3_seg8[3] alpha_ob4_seg8[0]
 alpha_ob4_seg8[1] alpha_ob4_seg8[2] alpha_ob4_seg8[3] alpha_ob5_seg8[0]
 alpha_ob5_seg8[1] alpha_ob5_seg8[2] alpha_ob5_seg8[3] alpha_ob6_seg8[0]
 alpha_ob6_seg8[1] alpha_ob6_seg8[2] alpha_ob6_seg8[3] alpha_ob7_seg8[0]
 alpha_ob7_seg8[1] alpha_ob7_seg8[2] alpha_ob7_seg8[3] alpha_ob8_seg8[0]
 alpha_ob8_seg8[1] alpha_ob8_seg8[2] alpha_ob8_seg8[3] alpha_ob9_seg8[0]
 alpha_ob9_seg8[1] alpha_ob9_seg8[2] alpha_ob9_seg8[3] alpha_ob10_seg8[0]
 alpha_ob10_seg8[1] alpha_ob10_seg8[2] alpha_ob10_seg8[3]
 alpha_ob11_seg8[0] alpha_ob11_seg8[1] alpha_ob11_seg8[2]
 alpha_ob11_seg8[3] alpha_ob12_seg8[0] alpha_ob12_seg8[1]
 alpha_ob12_seg8[2] alpha_ob12_seg8[3] alpha_ob13_seg8[0]
 alpha_ob13_seg8[1] alpha_ob13_seg8[2] alpha_ob13_seg8[3]
 alpha_ob14_seg8[0] alpha_ob14_seg8[1] alpha_ob14_seg8[2]
 alpha_ob14_seg8[3] alpha_ob15_seg8[0] alpha_ob15_seg8[1]
 alpha_ob15_seg8[2] alpha_ob15_seg8[3] alpha_ob16_seg8[0]
 alpha_ob16_seg8[1] alpha_ob16_seg8[2] alpha_ob16_seg8[3]
 alpha_ob17_seg8[0] alpha_ob17_seg8[1] alpha_ob17_seg8[2]
 alpha_ob17_seg8[3] alpha_ob18_seg8[0] alpha_ob18_seg8[1]
 alpha_ob18_seg8[2] alpha_ob18_seg8[3] alpha_ob19_seg8[0]
 alpha_ob19_seg8[1] alpha_ob19_seg8[2] alpha_ob19_seg8[3]
 alpha_ob20_seg8[0] alpha_ob20_seg8[1] alpha_ob20_seg8[2]
 alpha_ob20_seg8[3] alpha_ob21_seg8[0] alpha_ob21_seg8[1]
 alpha_ob21_seg8[2] alpha_ob21_seg8[3] alpha_ob22_seg8[0]
 alpha_ob22_seg8[1] alpha_ob22_seg8[2] alpha_ob22_seg8[3]
 alpha_ob23_seg8[0] alpha_ob23_seg8[1] alpha_ob23_seg8[2]
 alpha_ob23_seg8[3] alpha_ob24_seg8[0] alpha_ob24_seg8[1]
 alpha_ob24_seg8[2] alpha_ob24_seg8[3] alpha_ob25_seg8[0]
 alpha_ob25_seg8[1] alpha_ob25_seg8[2] alpha_ob25_seg8[3]
 alpha_ob26_seg8[0] alpha_ob26_seg8[1] alpha_ob26_seg8[2]
 alpha_ob26_seg8[3] alpha_ob27_seg8[0] alpha_ob27_seg8[1]
 alpha_ob27_seg8[2] alpha_ob27_seg8[3] alpha_ob28_seg8[0]
 alpha_ob28_seg8[1] alpha_ob28_seg8[2] alpha_ob28_seg8[3]
 alpha_ob29_seg8[0] alpha_ob29_seg8[1] alpha_ob29_seg8[2]
 alpha_ob29_seg8[3] alpha_ob30_seg8[0] alpha_ob30_seg8[1]
 alpha_ob30_seg8[2] alpha_ob30_seg8[3] alpha_ob31_seg8[0]
 alpha_ob31_seg8[1] alpha_ob31_seg8[2] alpha_ob31_seg8[3]
 alpha_ob32_seg8[0] alpha_ob32_seg8[1] alpha_ob32_seg8[2]
 alpha_ob32_seg8[3] alpha_ob33_seg8[0] alpha_ob33_seg8[1]
 alpha_ob33_seg8[2] alpha_ob33_seg8[3] alpha_ob34_seg8[0]
 alpha_ob34_seg8[1] alpha_ob34_seg8[2] alpha_ob34_seg8[3]
 alpha_ob35_seg8[0] alpha_ob35_seg8[1] alpha_ob35_seg8[2]
 alpha_ob35_seg8[3] alpha_ob36_seg8[0] alpha_ob36_seg8[1]
 alpha_ob36_seg8[2] alpha_ob36_seg8[3] alpha_ob37_seg8[0]
 alpha_ob37_seg8[1] alpha_ob37_seg8[2] alpha_ob37_seg8[3]
 alpha_ob38_seg8[0] alpha_ob38_seg8[1] alpha_ob38_seg8[2]
 alpha_ob38_seg8[3] alpha_ob39_seg8[0] alpha_ob39_seg8[1]
 alpha_ob39_seg8[2] alpha_ob39_seg8[3] alpha_ob40_seg8[0]
 alpha_ob40_seg8[1] alpha_ob40_seg8[2] alpha_ob40_seg8[3]
 alpha_ob41_seg8[0] alpha_ob41_seg8[1] alpha_ob41_seg8[2]
 alpha_ob41_seg8[3] alpha_ob42_seg8[0] alpha_ob42_seg8[1]
 alpha_ob42_seg8[2] alpha_ob42_seg8[3] alpha_ob43_seg8[0]
 alpha_ob43_seg8[1] alpha_ob43_seg8[2] alpha_ob43_seg8[3]
 alpha_ob44_seg8[0] alpha_ob44_seg8[1] alpha_ob44_seg8[2]
 alpha_ob44_seg8[3] alpha_ob45_seg8[0] alpha_ob45_seg8[1]
 alpha_ob45_seg8[2] alpha_ob45_seg8[3] alpha_ob46_seg8[0]
 alpha_ob46_seg8[1] alpha_ob46_seg8[2] alpha_ob46_seg8[3]
 alpha_ob47_seg8[0] alpha_ob47_seg8[1] alpha_ob47_seg8[2]
 alpha_ob47_seg8[3] alpha_ob48_seg8[0] alpha_ob48_seg8[1]
 alpha_ob48_seg8[2] alpha_ob48_seg8[3] alpha_ob49_seg8[0]
 alpha_ob49_seg8[1] alpha_ob49_seg8[2] alpha_ob49_seg8[3]
 alpha_ob50_seg8[0] alpha_ob50_seg8[1] alpha_ob50_seg8[2]
 alpha_ob50_seg8[3] alpha_ob0_seg9[0] alpha_ob0_seg9[1] alpha_ob0_seg9[2]
 alpha_ob0_seg9[3] alpha_ob1_seg9[0] alpha_ob1_seg9[1] alpha_ob1_seg9[2]
 alpha_ob1_seg9[3] alpha_ob2_seg9[0] alpha_ob2_seg9[1] alpha_ob2_seg9[2]
 alpha_ob2_seg9[3] alpha_ob3_seg9[0] alpha_ob3_seg9[1] alpha_ob3_seg9[2]
 alpha_ob3_seg9[3] alpha_ob4_seg9[0] alpha_ob4_seg9[1] alpha_ob4_seg9[2]
 alpha_ob4_seg9[3] alpha_ob5_seg9[0] alpha_ob5_seg9[1] alpha_ob5_seg9[2]
 alpha_ob5_seg9[3] alpha_ob6_seg9[0] alpha_ob6_seg9[1] alpha_ob6_seg9[2]
 alpha_ob6_seg9[3] alpha_ob7_seg9[0] alpha_ob7_seg9[1] alpha_ob7_seg9[2]
 alpha_ob7_seg9[3] alpha_ob8_seg9[0] alpha_ob8_seg9[1] alpha_ob8_seg9[2]
 alpha_ob8_seg9[3] alpha_ob9_seg9[0] alpha_ob9_seg9[1] alpha_ob9_seg9[2]
 alpha_ob9_seg9[3] alpha_ob10_seg9[0] alpha_ob10_seg9[1] alpha_ob10_seg9[2]
 alpha_ob10_seg9[3] alpha_ob11_seg9[0] alpha_ob11_seg9[1]
 alpha_ob11_seg9[2] alpha_ob11_seg9[3] alpha_ob12_seg9[0]
 alpha_ob12_seg9[1] alpha_ob12_seg9[2] alpha_ob12_seg9[3]
 alpha_ob13_seg9[0] alpha_ob13_seg9[1] alpha_ob13_seg9[2]
 alpha_ob13_seg9[3] alpha_ob14_seg9[0] alpha_ob14_seg9[1]
 alpha_ob14_seg9[2] alpha_ob14_seg9[3] alpha_ob15_seg9[0]
 alpha_ob15_seg9[1] alpha_ob15_seg9[2] alpha_ob15_seg9[3]
 alpha_ob16_seg9[0] alpha_ob16_seg9[1] alpha_ob16_seg9[2]
 alpha_ob16_seg9[3] alpha_ob17_seg9[0] alpha_ob17_seg9[1]
 alpha_ob17_seg9[2] alpha_ob17_seg9[3] alpha_ob18_seg9[0]
 alpha_ob18_seg9[1] alpha_ob18_seg9[2] alpha_ob18_seg9[3]
 alpha_ob19_seg9[0] alpha_ob19_seg9[1] alpha_ob19_seg9[2]
 alpha_ob19_seg9[3] alpha_ob20_seg9[0] alpha_ob20_seg9[1]
 alpha_ob20_seg9[2] alpha_ob20_seg9[3] alpha_ob21_seg9[0]
 alpha_ob21_seg9[1] alpha_ob21_seg9[2] alpha_ob21_seg9[3]
 alpha_ob22_seg9[0] alpha_ob22_seg9[1] alpha_ob22_seg9[2]
 alpha_ob22_seg9[3] alpha_ob23_seg9[0] alpha_ob23_seg9[1]
 alpha_ob23_seg9[2] alpha_ob23_seg9[3] alpha_ob24_seg9[0]
 alpha_ob24_seg9[1] alpha_ob24_seg9[2] alpha_ob24_seg9[3]
 alpha_ob25_seg9[0] alpha_ob25_seg9[1] alpha_ob25_seg9[2]
 alpha_ob25_seg9[3] alpha_ob26_seg9[0] alpha_ob26_seg9[1]
 alpha_ob26_seg9[2] alpha_ob26_seg9[3] alpha_ob27_seg9[0]
 alpha_ob27_seg9[1] alpha_ob27_seg9[2] alpha_ob27_seg9[3]
 alpha_ob28_seg9[0] alpha_ob28_seg9[1] alpha_ob28_seg9[2]
 alpha_ob28_seg9[3] alpha_ob29_seg9[0] alpha_ob29_seg9[1]
 alpha_ob29_seg9[2] alpha_ob29_seg9[3] alpha_ob30_seg9[0]
 alpha_ob30_seg9[1] alpha_ob30_seg9[2] alpha_ob30_seg9[3]
 alpha_ob31_seg9[0] alpha_ob31_seg9[1] alpha_ob31_seg9[2]
 alpha_ob31_seg9[3] alpha_ob32_seg9[0] alpha_ob32_seg9[1]
 alpha_ob32_seg9[2] alpha_ob32_seg9[3] alpha_ob33_seg9[0]
 alpha_ob33_seg9[1] alpha_ob33_seg9[2] alpha_ob33_seg9[3]
 alpha_ob34_seg9[0] alpha_ob34_seg9[1] alpha_ob34_seg9[2]
 alpha_ob34_seg9[3] alpha_ob35_seg9[0] alpha_ob35_seg9[1]
 alpha_ob35_seg9[2] alpha_ob35_seg9[3] alpha_ob36_seg9[0]
 alpha_ob36_seg9[1] alpha_ob36_seg9[2] alpha_ob36_seg9[3]
 alpha_ob37_seg9[0] alpha_ob37_seg9[1] alpha_ob37_seg9[2]
 alpha_ob37_seg9[3] alpha_ob38_seg9[0] alpha_ob38_seg9[1]
 alpha_ob38_seg9[2] alpha_ob38_seg9[3] alpha_ob39_seg9[0]
 alpha_ob39_seg9[1] alpha_ob39_seg9[2] alpha_ob39_seg9[3]
 alpha_ob40_seg9[0] alpha_ob40_seg9[1] alpha_ob40_seg9[2]
 alpha_ob40_seg9[3] alpha_ob41_seg9[0] alpha_ob41_seg9[1]
 alpha_ob41_seg9[2] alpha_ob41_seg9[3] alpha_ob42_seg9[0]
 alpha_ob42_seg9[1] alpha_ob42_seg9[2] alpha_ob42_seg9[3]
 alpha_ob43_seg9[0] alpha_ob43_seg9[1] alpha_ob43_seg9[2]
 alpha_ob43_seg9[3] alpha_ob44_seg9[0] alpha_ob44_seg9[1]
 alpha_ob44_seg9[2] alpha_ob44_seg9[3] alpha_ob45_seg9[0]
 alpha_ob45_seg9[1] alpha_ob45_seg9[2] alpha_ob45_seg9[3]
 alpha_ob46_seg9[0] alpha_ob46_seg9[1] alpha_ob46_seg9[2]
 alpha_ob46_seg9[3] alpha_ob47_seg9[0] alpha_ob47_seg9[1]
 alpha_ob47_seg9[2] alpha_ob47_seg9[3] alpha_ob48_seg9[0]
 alpha_ob48_seg9[1] alpha_ob48_seg9[2] alpha_ob48_seg9[3]
 alpha_ob49_seg9[0] alpha_ob49_seg9[1] alpha_ob49_seg9[2]
 alpha_ob49_seg9[3] alpha_ob50_seg9[0] alpha_ob50_seg9[1]
 alpha_ob50_seg9[2] alpha_ob50_seg9[3] alpha_ob0_seg10[0]
 alpha_ob0_seg10[1] alpha_ob0_seg10[2] alpha_ob0_seg10[3]
 alpha_ob1_seg10[0] alpha_ob1_seg10[1] alpha_ob1_seg10[2]
 alpha_ob1_seg10[3] alpha_ob2_seg10[0] alpha_ob2_seg10[1]
 alpha_ob2_seg10[2] alpha_ob2_seg10[3] alpha_ob3_seg10[0]
 alpha_ob3_seg10[1] alpha_ob3_seg10[2] alpha_ob3_seg10[3]
 alpha_ob4_seg10[0] alpha_ob4_seg10[1] alpha_ob4_seg10[2]
 alpha_ob4_seg10[3] alpha_ob5_seg10[0] alpha_ob5_seg10[1]
 alpha_ob5_seg10[2] alpha_ob5_seg10[3] alpha_ob6_seg10[0]
 alpha_ob6_seg10[1] alpha_ob6_seg10[2] alpha_ob6_seg10[3]
 alpha_ob7_seg10[0] alpha_ob7_seg10[1] alpha_ob7_seg10[2]
 alpha_ob7_seg10[3] alpha_ob8_seg10[0] alpha_ob8_seg10[1]
 alpha_ob8_seg10[2] alpha_ob8_seg10[3] alpha_ob9_seg10[0]
 alpha_ob9_seg10[1] alpha_ob9_seg10[2] alpha_ob9_seg10[3]
 alpha_ob10_seg10[0] alpha_ob10_seg10[1] alpha_ob10_seg10[2]
 alpha_ob10_seg10[3] alpha_ob11_seg10[0] alpha_ob11_seg10[1]
 alpha_ob11_seg10[2] alpha_ob11_seg10[3] alpha_ob12_seg10[0]
 alpha_ob12_seg10[1] alpha_ob12_seg10[2] alpha_ob12_seg10[3]
 alpha_ob13_seg10[0] alpha_ob13_seg10[1] alpha_ob13_seg10[2]
 alpha_ob13_seg10[3] alpha_ob14_seg10[0] alpha_ob14_seg10[1]
 alpha_ob14_seg10[2] alpha_ob14_seg10[3] alpha_ob15_seg10[0]
 alpha_ob15_seg10[1] alpha_ob15_seg10[2] alpha_ob15_seg10[3]
 alpha_ob16_seg10[0] alpha_ob16_seg10[1] alpha_ob16_seg10[2]
 alpha_ob16_seg10[3] alpha_ob17_seg10[0] alpha_ob17_seg10[1]
 alpha_ob17_seg10[2] alpha_ob17_seg10[3] alpha_ob18_seg10[0]
 alpha_ob18_seg10[1] alpha_ob18_seg10[2] alpha_ob18_seg10[3]
 alpha_ob19_seg10[0] alpha_ob19_seg10[1] alpha_ob19_seg10[2]
 alpha_ob19_seg10[3] alpha_ob20_seg10[0] alpha_ob20_seg10[1]
 alpha_ob20_seg10[2] alpha_ob20_seg10[3] alpha_ob21_seg10[0]
 alpha_ob21_seg10[1] alpha_ob21_seg10[2] alpha_ob21_seg10[3]
 alpha_ob22_seg10[0] alpha_ob22_seg10[1] alpha_ob22_seg10[2]
 alpha_ob22_seg10[3] alpha_ob23_seg10[0] alpha_ob23_seg10[1]
 alpha_ob23_seg10[2] alpha_ob23_seg10[3] alpha_ob24_seg10[0]
 alpha_ob24_seg10[1] alpha_ob24_seg10[2] alpha_ob24_seg10[3]
 alpha_ob25_seg10[0] alpha_ob25_seg10[1] alpha_ob25_seg10[2]
 alpha_ob25_seg10[3] alpha_ob26_seg10[0] alpha_ob26_seg10[1]
 alpha_ob26_seg10[2] alpha_ob26_seg10[3] alpha_ob27_seg10[0]
 alpha_ob27_seg10[1] alpha_ob27_seg10[2] alpha_ob27_seg10[3]
 alpha_ob28_seg10[0] alpha_ob28_seg10[1] alpha_ob28_seg10[2]
 alpha_ob28_seg10[3] alpha_ob29_seg10[0] alpha_ob29_seg10[1]
 alpha_ob29_seg10[2] alpha_ob29_seg10[3] alpha_ob30_seg10[0]
 alpha_ob30_seg10[1] alpha_ob30_seg10[2] alpha_ob30_seg10[3]
 alpha_ob31_seg10[0] alpha_ob31_seg10[1] alpha_ob31_seg10[2]
 alpha_ob31_seg10[3] alpha_ob32_seg10[0] alpha_ob32_seg10[1]
 alpha_ob32_seg10[2] alpha_ob32_seg10[3] alpha_ob33_seg10[0]
 alpha_ob33_seg10[1] alpha_ob33_seg10[2] alpha_ob33_seg10[3]
 alpha_ob34_seg10[0] alpha_ob34_seg10[1] alpha_ob34_seg10[2]
 alpha_ob34_seg10[3] alpha_ob35_seg10[0] alpha_ob35_seg10[1]
 alpha_ob35_seg10[2] alpha_ob35_seg10[3] alpha_ob36_seg10[0]
 alpha_ob36_seg10[1] alpha_ob36_seg10[2] alpha_ob36_seg10[3]
 alpha_ob37_seg10[0] alpha_ob37_seg10[1] alpha_ob37_seg10[2]
 alpha_ob37_seg10[3] alpha_ob38_seg10[0] alpha_ob38_seg10[1]
 alpha_ob38_seg10[2] alpha_ob38_seg10[3] alpha_ob39_seg10[0]
 alpha_ob39_seg10[1] alpha_ob39_seg10[2] alpha_ob39_seg10[3]
 alpha_ob40_seg10[0] alpha_ob40_seg10[1] alpha_ob40_seg10[2]
 alpha_ob40_seg10[3] alpha_ob41_seg10[0] alpha_ob41_seg10[1]
 alpha_ob41_seg10[2] alpha_ob41_seg10[3] alpha_ob42_seg10[0]
 alpha_ob42_seg10[1] alpha_ob42_seg10[2] alpha_ob42_seg10[3]
 alpha_ob43_seg10[0] alpha_ob43_seg10[1] alpha_ob43_seg10[2]
 alpha_ob43_seg10[3] alpha_ob44_seg10[0] alpha_ob44_seg10[1]
 alpha_ob44_seg10[2] alpha_ob44_seg10[3] alpha_ob45_seg10[0]
 alpha_ob45_seg10[1] alpha_ob45_seg10[2] alpha_ob45_seg10[3]
 alpha_ob46_seg10[0] alpha_ob46_seg10[1] alpha_ob46_seg10[2]
 alpha_ob46_seg10[3] alpha_ob47_seg10[0] alpha_ob47_seg10[1]
 alpha_ob47_seg10[2] alpha_ob47_seg10[3] alpha_ob48_seg10[0]
 alpha_ob48_seg10[1] alpha_ob48_seg10[2] alpha_ob48_seg10[3]
 alpha_ob49_seg10[0] alpha_ob49_seg10[1] alpha_ob49_seg10[2]
 alpha_ob49_seg10[3] alpha_ob50_seg10[0] alpha_ob50_seg10[1]
 alpha_ob50_seg10[2] alpha_ob50_seg10[3] alpha_ob0_seg11[0]
 alpha_ob0_seg11[1] alpha_ob0_seg11[2] alpha_ob0_seg11[3]
 alpha_ob1_seg11[0] alpha_ob1_seg11[1] alpha_ob1_seg11[2]
 alpha_ob1_seg11[3] alpha_ob2_seg11[0] alpha_ob2_seg11[1]
 alpha_ob2_seg11[2] alpha_ob2_seg11[3] alpha_ob3_seg11[0]
 alpha_ob3_seg11[1] alpha_ob3_seg11[2] alpha_ob3_seg11[3]
 alpha_ob4_seg11[0] alpha_ob4_seg11[1] alpha_ob4_seg11[2]
 alpha_ob4_seg11[3] alpha_ob5_seg11[0] alpha_ob5_seg11[1]
 alpha_ob5_seg11[2] alpha_ob5_seg11[3] alpha_ob6_seg11[0]
 alpha_ob6_seg11[1] alpha_ob6_seg11[2] alpha_ob6_seg11[3]
 alpha_ob7_seg11[0] alpha_ob7_seg11[1] alpha_ob7_seg11[2]
 alpha_ob7_seg11[3] alpha_ob8_seg11[0] alpha_ob8_seg11[1]
 alpha_ob8_seg11[2] alpha_ob8_seg11[3] alpha_ob9_seg11[0]
 alpha_ob9_seg11[1] alpha_ob9_seg11[2] alpha_ob9_seg11[3]
 alpha_ob10_seg11[0] alpha_ob10_seg11[1] alpha_ob10_seg11[2]
 alpha_ob10_seg11[3] alpha_ob11_seg11[0] alpha_ob11_seg11[1]
 alpha_ob11_seg11[2] alpha_ob11_seg11[3] alpha_ob12_seg11[0]
 alpha_ob12_seg11[1] alpha_ob12_seg11[2] alpha_ob12_seg11[3]
 alpha_ob13_seg11[0] alpha_ob13_seg11[1] alpha_ob13_seg11[2]
 alpha_ob13_seg11[3] alpha_ob14_seg11[0] alpha_ob14_seg11[1]
 alpha_ob14_seg11[2] alpha_ob14_seg11[3] alpha_ob15_seg11[0]
 alpha_ob15_seg11[1] alpha_ob15_seg11[2] alpha_ob15_seg11[3]
 alpha_ob16_seg11[0] alpha_ob16_seg11[1] alpha_ob16_seg11[2]
 alpha_ob16_seg11[3] alpha_ob17_seg11[0] alpha_ob17_seg11[1]
 alpha_ob17_seg11[2] alpha_ob17_seg11[3] alpha_ob18_seg11[0]
 alpha_ob18_seg11[1] alpha_ob18_seg11[2] alpha_ob18_seg11[3]
 alpha_ob19_seg11[0] alpha_ob19_seg11[1] alpha_ob19_seg11[2]
 alpha_ob19_seg11[3] alpha_ob20_seg11[0] alpha_ob20_seg11[1]
 alpha_ob20_seg11[2] alpha_ob20_seg11[3] alpha_ob21_seg11[0]
 alpha_ob21_seg11[1] alpha_ob21_seg11[2] alpha_ob21_seg11[3]
 alpha_ob22_seg11[0] alpha_ob22_seg11[1] alpha_ob22_seg11[2]
 alpha_ob22_seg11[3] alpha_ob23_seg11[0] alpha_ob23_seg11[1]
 alpha_ob23_seg11[2] alpha_ob23_seg11[3] alpha_ob24_seg11[0]
 alpha_ob24_seg11[1] alpha_ob24_seg11[2] alpha_ob24_seg11[3]
 alpha_ob25_seg11[0] alpha_ob25_seg11[1] alpha_ob25_seg11[2]
 alpha_ob25_seg11[3] alpha_ob26_seg11[0] alpha_ob26_seg11[1]
 alpha_ob26_seg11[2] alpha_ob26_seg11[3] alpha_ob27_seg11[0]
 alpha_ob27_seg11[1] alpha_ob27_seg11[2] alpha_ob27_seg11[3]
 alpha_ob28_seg11[0] alpha_ob28_seg11[1] alpha_ob28_seg11[2]
 alpha_ob28_seg11[3] alpha_ob29_seg11[0] alpha_ob29_seg11[1]
 alpha_ob29_seg11[2] alpha_ob29_seg11[3] alpha_ob30_seg11[0]
 alpha_ob30_seg11[1] alpha_ob30_seg11[2] alpha_ob30_seg11[3]
 alpha_ob31_seg11[0] alpha_ob31_seg11[1] alpha_ob31_seg11[2]
 alpha_ob31_seg11[3] alpha_ob32_seg11[0] alpha_ob32_seg11[1]
 alpha_ob32_seg11[2] alpha_ob32_seg11[3] alpha_ob33_seg11[0]
 alpha_ob33_seg11[1] alpha_ob33_seg11[2] alpha_ob33_seg11[3]
 alpha_ob34_seg11[0] alpha_ob34_seg11[1] alpha_ob34_seg11[2]
 alpha_ob34_seg11[3] alpha_ob35_seg11[0] alpha_ob35_seg11[1]
 alpha_ob35_seg11[2] alpha_ob35_seg11[3] alpha_ob36_seg11[0]
 alpha_ob36_seg11[1] alpha_ob36_seg11[2] alpha_ob36_seg11[3]
 alpha_ob37_seg11[0] alpha_ob37_seg11[1] alpha_ob37_seg11[2]
 alpha_ob37_seg11[3] alpha_ob38_seg11[0] alpha_ob38_seg11[1]
 alpha_ob38_seg11[2] alpha_ob38_seg11[3] alpha_ob39_seg11[0]
 alpha_ob39_seg11[1] alpha_ob39_seg11[2] alpha_ob39_seg11[3]
 alpha_ob40_seg11[0] alpha_ob40_seg11[1] alpha_ob40_seg11[2]
 alpha_ob40_seg11[3] alpha_ob41_seg11[0] alpha_ob41_seg11[1]
 alpha_ob41_seg11[2] alpha_ob41_seg11[3] alpha_ob42_seg11[0]
 alpha_ob42_seg11[1] alpha_ob42_seg11[2] alpha_ob42_seg11[3]
 alpha_ob43_seg11[0] alpha_ob43_seg11[1] alpha_ob43_seg11[2]
 alpha_ob43_seg11[3] alpha_ob44_seg11[0] alpha_ob44_seg11[1]
 alpha_ob44_seg11[2] alpha_ob44_seg11[3] alpha_ob45_seg11[0]
 alpha_ob45_seg11[1] alpha_ob45_seg11[2] alpha_ob45_seg11[3]
 alpha_ob46_seg11[0] alpha_ob46_seg11[1] alpha_ob46_seg11[2]
 alpha_ob46_seg11[3] alpha_ob47_seg11[0] alpha_ob47_seg11[1]
 alpha_ob47_seg11[2] alpha_ob47_seg11[3] alpha_ob48_seg11[0]
 alpha_ob48_seg11[1] alpha_ob48_seg11[2] alpha_ob48_seg11[3]
 alpha_ob49_seg11[0] alpha_ob49_seg11[1] alpha_ob49_seg11[2]
 alpha_ob49_seg11[3] alpha_ob50_seg11[0] alpha_ob50_seg11[1]
 alpha_ob50_seg11[2] alpha_ob50_seg11[3] alpha_ob0_seg12[0]
 alpha_ob0_seg12[1] alpha_ob0_seg12[2] alpha_ob0_seg12[3]
 alpha_ob1_seg12[0] alpha_ob1_seg12[1] alpha_ob1_seg12[2]
 alpha_ob1_seg12[3] alpha_ob2_seg12[0] alpha_ob2_seg12[1]
 alpha_ob2_seg12[2] alpha_ob2_seg12[3] alpha_ob3_seg12[0]
 alpha_ob3_seg12[1] alpha_ob3_seg12[2] alpha_ob3_seg12[3]
 alpha_ob4_seg12[0] alpha_ob4_seg12[1] alpha_ob4_seg12[2]
 alpha_ob4_seg12[3] alpha_ob5_seg12[0] alpha_ob5_seg12[1]
 alpha_ob5_seg12[2] alpha_ob5_seg12[3] alpha_ob6_seg12[0]
 alpha_ob6_seg12[1] alpha_ob6_seg12[2] alpha_ob6_seg12[3]
 alpha_ob7_seg12[0] alpha_ob7_seg12[1] alpha_ob7_seg12[2]
 alpha_ob7_seg12[3] alpha_ob8_seg12[0] alpha_ob8_seg12[1]
 alpha_ob8_seg12[2] alpha_ob8_seg12[3] alpha_ob9_seg12[0]
 alpha_ob9_seg12[1] alpha_ob9_seg12[2] alpha_ob9_seg12[3]
 alpha_ob10_seg12[0] alpha_ob10_seg12[1] alpha_ob10_seg12[2]
 alpha_ob10_seg12[3] alpha_ob11_seg12[0] alpha_ob11_seg12[1]
 alpha_ob11_seg12[2] alpha_ob11_seg12[3] alpha_ob12_seg12[0]
 alpha_ob12_seg12[1] alpha_ob12_seg12[2] alpha_ob12_seg12[3]
 alpha_ob13_seg12[0] alpha_ob13_seg12[1] alpha_ob13_seg12[2]
 alpha_ob13_seg12[3] alpha_ob14_seg12[0] alpha_ob14_seg12[1]
 alpha_ob14_seg12[2] alpha_ob14_seg12[3] alpha_ob15_seg12[0]
 alpha_ob15_seg12[1] alpha_ob15_seg12[2] alpha_ob15_seg12[3]
 alpha_ob16_seg12[0] alpha_ob16_seg12[1] alpha_ob16_seg12[2]
 alpha_ob16_seg12[3] alpha_ob17_seg12[0] alpha_ob17_seg12[1]
 alpha_ob17_seg12[2] alpha_ob17_seg12[3] alpha_ob18_seg12[0]
 alpha_ob18_seg12[1] alpha_ob18_seg12[2] alpha_ob18_seg12[3]
 alpha_ob19_seg12[0] alpha_ob19_seg12[1] alpha_ob19_seg12[2]
 alpha_ob19_seg12[3] alpha_ob20_seg12[0] alpha_ob20_seg12[1]
 alpha_ob20_seg12[2] alpha_ob20_seg12[3] alpha_ob21_seg12[0]
 alpha_ob21_seg12[1] alpha_ob21_seg12[2] alpha_ob21_seg12[3]
 alpha_ob22_seg12[0] alpha_ob22_seg12[1] alpha_ob22_seg12[2]
 alpha_ob22_seg12[3] alpha_ob23_seg12[0] alpha_ob23_seg12[1]
 alpha_ob23_seg12[2] alpha_ob23_seg12[3] alpha_ob24_seg12[0]
 alpha_ob24_seg12[1] alpha_ob24_seg12[2] alpha_ob24_seg12[3]
 alpha_ob25_seg12[0] alpha_ob25_seg12[1] alpha_ob25_seg12[2]
 alpha_ob25_seg12[3] alpha_ob26_seg12[0] alpha_ob26_seg12[1]
 alpha_ob26_seg12[2] alpha_ob26_seg12[3] alpha_ob27_seg12[0]
 alpha_ob27_seg12[1] alpha_ob27_seg12[2] alpha_ob27_seg12[3]
 alpha_ob28_seg12[0] alpha_ob28_seg12[1] alpha_ob28_seg12[2]
 alpha_ob28_seg12[3] alpha_ob29_seg12[0] alpha_ob29_seg12[1]
 alpha_ob29_seg12[2] alpha_ob29_seg12[3] alpha_ob30_seg12[0]
 alpha_ob30_seg12[1] alpha_ob30_seg12[2] alpha_ob30_seg12[3]
 alpha_ob31_seg12[0] alpha_ob31_seg12[1] alpha_ob31_seg12[2]
 alpha_ob31_seg12[3] alpha_ob32_seg12[0] alpha_ob32_seg12[1]
 alpha_ob32_seg12[2] alpha_ob32_seg12[3] alpha_ob33_seg12[0]
 alpha_ob33_seg12[1] alpha_ob33_seg12[2] alpha_ob33_seg12[3]
 alpha_ob34_seg12[0] alpha_ob34_seg12[1] alpha_ob34_seg12[2]
 alpha_ob34_seg12[3] alpha_ob35_seg12[0] alpha_ob35_seg12[1]
 alpha_ob35_seg12[2] alpha_ob35_seg12[3] alpha_ob36_seg12[0]
 alpha_ob36_seg12[1] alpha_ob36_seg12[2] alpha_ob36_seg12[3]
 alpha_ob37_seg12[0] alpha_ob37_seg12[1] alpha_ob37_seg12[2]
 alpha_ob37_seg12[3] alpha_ob38_seg12[0] alpha_ob38_seg12[1]
 alpha_ob38_seg12[2] alpha_ob38_seg12[3] alpha_ob39_seg12[0]
 alpha_ob39_seg12[1] alpha_ob39_seg12[2] alpha_ob39_seg12[3]
 alpha_ob40_seg12[0] alpha_ob40_seg12[1] alpha_ob40_seg12[2]
 alpha_ob40_seg12[3] alpha_ob41_seg12[0] alpha_ob41_seg12[1]
 alpha_ob41_seg12[2] alpha_ob41_seg12[3] alpha_ob42_seg12[0]
 alpha_ob42_seg12[1] alpha_ob42_seg12[2] alpha_ob42_seg12[3]
 alpha_ob43_seg12[0] alpha_ob43_seg12[1] alpha_ob43_seg12[2]
 alpha_ob43_seg12[3] alpha_ob44_seg12[0] alpha_ob44_seg12[1]
 alpha_ob44_seg12[2] alpha_ob44_seg12[3] alpha_ob45_seg12[0]
 alpha_ob45_seg12[1] alpha_ob45_seg12[2] alpha_ob45_seg12[3]
 alpha_ob46_seg12[0] alpha_ob46_seg12[1] alpha_ob46_seg12[2]
 alpha_ob46_seg12[3] alpha_ob47_seg12[0] alpha_ob47_seg12[1]
 alpha_ob47_seg12[2] alpha_ob47_seg12[3] alpha_ob48_seg12[0]
 alpha_ob48_seg12[1] alpha_ob48_seg12[2] alpha_ob48_seg12[3]
 alpha_ob49_seg12[0] alpha_ob49_seg12[1] alpha_ob49_seg12[2]
 alpha_ob49_seg12[3] alpha_ob50_seg12[0] alpha_ob50_seg12[1]
 alpha_ob50_seg12[2] alpha_ob50_seg12[3] alpha_ob0_seg13[0]
 alpha_ob0_seg13[1] alpha_ob0_seg13[2] alpha_ob0_seg13[3]
 alpha_ob1_seg13[0] alpha_ob1_seg13[1] alpha_ob1_seg13[2]
 alpha_ob1_seg13[3] alpha_ob2_seg13[0] alpha_ob2_seg13[1]
 alpha_ob2_seg13[2] alpha_ob2_seg13[3] alpha_ob3_seg13[0]
 alpha_ob3_seg13[1] alpha_ob3_seg13[2] alpha_ob3_seg13[3]
 alpha_ob4_seg13[0] alpha_ob4_seg13[1] alpha_ob4_seg13[2]
 alpha_ob4_seg13[3] alpha_ob5_seg13[0] alpha_ob5_seg13[1]
 alpha_ob5_seg13[2] alpha_ob5_seg13[3] alpha_ob6_seg13[0]
 alpha_ob6_seg13[1] alpha_ob6_seg13[2] alpha_ob6_seg13[3]
 alpha_ob7_seg13[0] alpha_ob7_seg13[1] alpha_ob7_seg13[2]
 alpha_ob7_seg13[3] alpha_ob8_seg13[0] alpha_ob8_seg13[1]
 alpha_ob8_seg13[2] alpha_ob8_seg13[3] alpha_ob9_seg13[0]
 alpha_ob9_seg13[1] alpha_ob9_seg13[2] alpha_ob9_seg13[3]
 alpha_ob10_seg13[0] alpha_ob10_seg13[1] alpha_ob10_seg13[2]
 alpha_ob10_seg13[3] alpha_ob11_seg13[0] alpha_ob11_seg13[1]
 alpha_ob11_seg13[2] alpha_ob11_seg13[3] alpha_ob12_seg13[0]
 alpha_ob12_seg13[1] alpha_ob12_seg13[2] alpha_ob12_seg13[3]
 alpha_ob13_seg13[0] alpha_ob13_seg13[1] alpha_ob13_seg13[2]
 alpha_ob13_seg13[3] alpha_ob14_seg13[0] alpha_ob14_seg13[1]
 alpha_ob14_seg13[2] alpha_ob14_seg13[3] alpha_ob15_seg13[0]
 alpha_ob15_seg13[1] alpha_ob15_seg13[2] alpha_ob15_seg13[3]
 alpha_ob16_seg13[0] alpha_ob16_seg13[1] alpha_ob16_seg13[2]
 alpha_ob16_seg13[3] alpha_ob17_seg13[0] alpha_ob17_seg13[1]
 alpha_ob17_seg13[2] alpha_ob17_seg13[3] alpha_ob18_seg13[0]
 alpha_ob18_seg13[1] alpha_ob18_seg13[2] alpha_ob18_seg13[3]
 alpha_ob19_seg13[0] alpha_ob19_seg13[1] alpha_ob19_seg13[2]
 alpha_ob19_seg13[3] alpha_ob20_seg13[0] alpha_ob20_seg13[1]
 alpha_ob20_seg13[2] alpha_ob20_seg13[3] alpha_ob21_seg13[0]
 alpha_ob21_seg13[1] alpha_ob21_seg13[2] alpha_ob21_seg13[3]
 alpha_ob22_seg13[0] alpha_ob22_seg13[1] alpha_ob22_seg13[2]
 alpha_ob22_seg13[3] alpha_ob23_seg13[0] alpha_ob23_seg13[1]
 alpha_ob23_seg13[2] alpha_ob23_seg13[3] alpha_ob24_seg13[0]
 alpha_ob24_seg13[1] alpha_ob24_seg13[2] alpha_ob24_seg13[3]
 alpha_ob25_seg13[0] alpha_ob25_seg13[1] alpha_ob25_seg13[2]
 alpha_ob25_seg13[3] alpha_ob26_seg13[0] alpha_ob26_seg13[1]
 alpha_ob26_seg13[2] alpha_ob26_seg13[3] alpha_ob27_seg13[0]
 alpha_ob27_seg13[1] alpha_ob27_seg13[2] alpha_ob27_seg13[3]
 alpha_ob28_seg13[0] alpha_ob28_seg13[1] alpha_ob28_seg13[2]
 alpha_ob28_seg13[3] alpha_ob29_seg13[0] alpha_ob29_seg13[1]
 alpha_ob29_seg13[2] alpha_ob29_seg13[3] alpha_ob30_seg13[0]
 alpha_ob30_seg13[1] alpha_ob30_seg13[2] alpha_ob30_seg13[3]
 alpha_ob31_seg13[0] alpha_ob31_seg13[1] alpha_ob31_seg13[2]
 alpha_ob31_seg13[3] alpha_ob32_seg13[0] alpha_ob32_seg13[1]
 alpha_ob32_seg13[2] alpha_ob32_seg13[3] alpha_ob33_seg13[0]
 alpha_ob33_seg13[1] alpha_ob33_seg13[2] alpha_ob33_seg13[3]
 alpha_ob34_seg13[0] alpha_ob34_seg13[1] alpha_ob34_seg13[2]
 alpha_ob34_seg13[3] alpha_ob35_seg13[0] alpha_ob35_seg13[1]
 alpha_ob35_seg13[2] alpha_ob35_seg13[3] alpha_ob36_seg13[0]
 alpha_ob36_seg13[1] alpha_ob36_seg13[2] alpha_ob36_seg13[3]
 alpha_ob37_seg13[0] alpha_ob37_seg13[1] alpha_ob37_seg13[2]
 alpha_ob37_seg13[3] alpha_ob38_seg13[0] alpha_ob38_seg13[1]
 alpha_ob38_seg13[2] alpha_ob38_seg13[3] alpha_ob39_seg13[0]
 alpha_ob39_seg13[1] alpha_ob39_seg13[2] alpha_ob39_seg13[3]
 alpha_ob40_seg13[0] alpha_ob40_seg13[1] alpha_ob40_seg13[2]
 alpha_ob40_seg13[3] alpha_ob41_seg13[0] alpha_ob41_seg13[1]
 alpha_ob41_seg13[2] alpha_ob41_seg13[3] alpha_ob42_seg13[0]
 alpha_ob42_seg13[1] alpha_ob42_seg13[2] alpha_ob42_seg13[3]
 alpha_ob43_seg13[0] alpha_ob43_seg13[1] alpha_ob43_seg13[2]
 alpha_ob43_seg13[3] alpha_ob44_seg13[0] alpha_ob44_seg13[1]
 alpha_ob44_seg13[2] alpha_ob44_seg13[3] alpha_ob45_seg13[0]
 alpha_ob45_seg13[1] alpha_ob45_seg13[2] alpha_ob45_seg13[3]
 alpha_ob46_seg13[0] alpha_ob46_seg13[1] alpha_ob46_seg13[2]
 alpha_ob46_seg13[3] alpha_ob47_seg13[0] alpha_ob47_seg13[1]
 alpha_ob47_seg13[2] alpha_ob47_seg13[3] alpha_ob48_seg13[0]
 alpha_ob48_seg13[1] alpha_ob48_seg13[2] alpha_ob48_seg13[3]
 alpha_ob49_seg13[0] alpha_ob49_seg13[1] alpha_ob49_seg13[2]
 alpha_ob49_seg13[3] alpha_ob50_seg13[0] alpha_ob50_seg13[1]
 alpha_ob50_seg13[2] alpha_ob50_seg13[3] alpha_ob0_seg14[0]
 alpha_ob0_seg14[1] alpha_ob0_seg14[2] alpha_ob0_seg14[3]
 alpha_ob1_seg14[0] alpha_ob1_seg14[1] alpha_ob1_seg14[2]
 alpha_ob1_seg14[3] alpha_ob2_seg14[0] alpha_ob2_seg14[1]
 alpha_ob2_seg14[2] alpha_ob2_seg14[3] alpha_ob3_seg14[0]
 alpha_ob3_seg14[1] alpha_ob3_seg14[2] alpha_ob3_seg14[3]
 alpha_ob4_seg14[0] alpha_ob4_seg14[1] alpha_ob4_seg14[2]
 alpha_ob4_seg14[3] alpha_ob5_seg14[0] alpha_ob5_seg14[1]
 alpha_ob5_seg14[2] alpha_ob5_seg14[3] alpha_ob6_seg14[0]
 alpha_ob6_seg14[1] alpha_ob6_seg14[2] alpha_ob6_seg14[3]
 alpha_ob7_seg14[0] alpha_ob7_seg14[1] alpha_ob7_seg14[2]
 alpha_ob7_seg14[3] alpha_ob8_seg14[0] alpha_ob8_seg14[1]
 alpha_ob8_seg14[2] alpha_ob8_seg14[3] alpha_ob9_seg14[0]
 alpha_ob9_seg14[1] alpha_ob9_seg14[2] alpha_ob9_seg14[3]
 alpha_ob10_seg14[0] alpha_ob10_seg14[1] alpha_ob10_seg14[2]
 alpha_ob10_seg14[3] alpha_ob11_seg14[0] alpha_ob11_seg14[1]
 alpha_ob11_seg14[2] alpha_ob11_seg14[3] alpha_ob12_seg14[0]
 alpha_ob12_seg14[1] alpha_ob12_seg14[2] alpha_ob12_seg14[3]
 alpha_ob13_seg14[0] alpha_ob13_seg14[1] alpha_ob13_seg14[2]
 alpha_ob13_seg14[3] alpha_ob14_seg14[0] alpha_ob14_seg14[1]
 alpha_ob14_seg14[2] alpha_ob14_seg14[3] alpha_ob15_seg14[0]
 alpha_ob15_seg14[1] alpha_ob15_seg14[2] alpha_ob15_seg14[3]
 alpha_ob16_seg14[0] alpha_ob16_seg14[1] alpha_ob16_seg14[2]
 alpha_ob16_seg14[3] alpha_ob17_seg14[0] alpha_ob17_seg14[1]
 alpha_ob17_seg14[2] alpha_ob17_seg14[3] alpha_ob18_seg14[0]
 alpha_ob18_seg14[1] alpha_ob18_seg14[2] alpha_ob18_seg14[3]
 alpha_ob19_seg14[0] alpha_ob19_seg14[1] alpha_ob19_seg14[2]
 alpha_ob19_seg14[3] alpha_ob20_seg14[0] alpha_ob20_seg14[1]
 alpha_ob20_seg14[2] alpha_ob20_seg14[3] alpha_ob21_seg14[0]
 alpha_ob21_seg14[1] alpha_ob21_seg14[2] alpha_ob21_seg14[3]
 alpha_ob22_seg14[0] alpha_ob22_seg14[1] alpha_ob22_seg14[2]
 alpha_ob22_seg14[3] alpha_ob23_seg14[0] alpha_ob23_seg14[1]
 alpha_ob23_seg14[2] alpha_ob23_seg14[3] alpha_ob24_seg14[0]
 alpha_ob24_seg14[1] alpha_ob24_seg14[2] alpha_ob24_seg14[3]
 alpha_ob25_seg14[0] alpha_ob25_seg14[1] alpha_ob25_seg14[2]
 alpha_ob25_seg14[3] alpha_ob26_seg14[0] alpha_ob26_seg14[1]
 alpha_ob26_seg14[2] alpha_ob26_seg14[3] alpha_ob27_seg14[0]
 alpha_ob27_seg14[1] alpha_ob27_seg14[2] alpha_ob27_seg14[3]
 alpha_ob28_seg14[0] alpha_ob28_seg14[1] alpha_ob28_seg14[2]
 alpha_ob28_seg14[3] alpha_ob29_seg14[0] alpha_ob29_seg14[1]
 alpha_ob29_seg14[2] alpha_ob29_seg14[3] alpha_ob30_seg14[0]
 alpha_ob30_seg14[1] alpha_ob30_seg14[2] alpha_ob30_seg14[3]
 alpha_ob31_seg14[0] alpha_ob31_seg14[1] alpha_ob31_seg14[2]
 alpha_ob31_seg14[3] alpha_ob32_seg14[0] alpha_ob32_seg14[1]
 alpha_ob32_seg14[2] alpha_ob32_seg14[3] alpha_ob33_seg14[0]
 alpha_ob33_seg14[1] alpha_ob33_seg14[2] alpha_ob33_seg14[3]
 alpha_ob34_seg14[0] alpha_ob34_seg14[1] alpha_ob34_seg14[2]
 alpha_ob34_seg14[3] alpha_ob35_seg14[0] alpha_ob35_seg14[1]
 alpha_ob35_seg14[2] alpha_ob35_seg14[3] alpha_ob36_seg14[0]
 alpha_ob36_seg14[1] alpha_ob36_seg14[2] alpha_ob36_seg14[3]
 alpha_ob37_seg14[0] alpha_ob37_seg14[1] alpha_ob37_seg14[2]
 alpha_ob37_seg14[3] alpha_ob38_seg14[0] alpha_ob38_seg14[1]
 alpha_ob38_seg14[2] alpha_ob38_seg14[3] alpha_ob39_seg14[0]
 alpha_ob39_seg14[1] alpha_ob39_seg14[2] alpha_ob39_seg14[3]
 alpha_ob40_seg14[0] alpha_ob40_seg14[1] alpha_ob40_seg14[2]
 alpha_ob40_seg14[3] alpha_ob41_seg14[0] alpha_ob41_seg14[1]
 alpha_ob41_seg14[2] alpha_ob41_seg14[3] alpha_ob42_seg14[0]
 alpha_ob42_seg14[1] alpha_ob42_seg14[2] alpha_ob42_seg14[3]
 alpha_ob43_seg14[0] alpha_ob43_seg14[1] alpha_ob43_seg14[2]
 alpha_ob43_seg14[3] alpha_ob44_seg14[0] alpha_ob44_seg14[1]
 alpha_ob44_seg14[2] alpha_ob44_seg14[3] alpha_ob45_seg14[0]
 alpha_ob45_seg14[1] alpha_ob45_seg14[2] alpha_ob45_seg14[3]
 alpha_ob46_seg14[0] alpha_ob46_seg14[1] alpha_ob46_seg14[2]
 alpha_ob46_seg14[3] alpha_ob47_seg14[0] alpha_ob47_seg14[1]
 alpha_ob47_seg14[2] alpha_ob47_seg14[3] alpha_ob48_seg14[0]
 alpha_ob48_seg14[1] alpha_ob48_seg14[2] alpha_ob48_seg14[3]
 alpha_ob49_seg14[0] alpha_ob49_seg14[1] alpha_ob49_seg14[2]
 alpha_ob49_seg14[3] alpha_ob50_seg14[0] alpha_ob50_seg14[1]
 alpha_ob50_seg14[2] alpha_ob50_seg14[3] alpha_ob0_seg15[0]
 alpha_ob0_seg15[1] alpha_ob0_seg15[2] alpha_ob0_seg15[3]
 alpha_ob1_seg15[0] alpha_ob1_seg15[1] alpha_ob1_seg15[2]
 alpha_ob1_seg15[3] alpha_ob2_seg15[0] alpha_ob2_seg15[1]
 alpha_ob2_seg15[2] alpha_ob2_seg15[3] alpha_ob3_seg15[0]
 alpha_ob3_seg15[1] alpha_ob3_seg15[2] alpha_ob3_seg15[3]
 alpha_ob4_seg15[0] alpha_ob4_seg15[1] alpha_ob4_seg15[2]
 alpha_ob4_seg15[3] alpha_ob5_seg15[0] alpha_ob5_seg15[1]
 alpha_ob5_seg15[2] alpha_ob5_seg15[3] alpha_ob6_seg15[0]
 alpha_ob6_seg15[1] alpha_ob6_seg15[2] alpha_ob6_seg15[3]
 alpha_ob7_seg15[0] alpha_ob7_seg15[1] alpha_ob7_seg15[2]
 alpha_ob7_seg15[3] alpha_ob8_seg15[0] alpha_ob8_seg15[1]
 alpha_ob8_seg15[2] alpha_ob8_seg15[3] alpha_ob9_seg15[0]
 alpha_ob9_seg15[1] alpha_ob9_seg15[2] alpha_ob9_seg15[3]
 alpha_ob10_seg15[0] alpha_ob10_seg15[1] alpha_ob10_seg15[2]
 alpha_ob10_seg15[3] alpha_ob11_seg15[0] alpha_ob11_seg15[1]
 alpha_ob11_seg15[2] alpha_ob11_seg15[3] alpha_ob12_seg15[0]
 alpha_ob12_seg15[1] alpha_ob12_seg15[2] alpha_ob12_seg15[3]
 alpha_ob13_seg15[0] alpha_ob13_seg15[1] alpha_ob13_seg15[2]
 alpha_ob13_seg15[3] alpha_ob14_seg15[0] alpha_ob14_seg15[1]
 alpha_ob14_seg15[2] alpha_ob14_seg15[3] alpha_ob15_seg15[0]
 alpha_ob15_seg15[1] alpha_ob15_seg15[2] alpha_ob15_seg15[3]
 alpha_ob16_seg15[0] alpha_ob16_seg15[1] alpha_ob16_seg15[2]
 alpha_ob16_seg15[3] alpha_ob17_seg15[0] alpha_ob17_seg15[1]
 alpha_ob17_seg15[2] alpha_ob17_seg15[3] alpha_ob18_seg15[0]
 alpha_ob18_seg15[1] alpha_ob18_seg15[2] alpha_ob18_seg15[3]
 alpha_ob19_seg15[0] alpha_ob19_seg15[1] alpha_ob19_seg15[2]
 alpha_ob19_seg15[3] alpha_ob20_seg15[0] alpha_ob20_seg15[1]
 alpha_ob20_seg15[2] alpha_ob20_seg15[3] alpha_ob21_seg15[0]
 alpha_ob21_seg15[1] alpha_ob21_seg15[2] alpha_ob21_seg15[3]
 alpha_ob22_seg15[0] alpha_ob22_seg15[1] alpha_ob22_seg15[2]
 alpha_ob22_seg15[3] alpha_ob23_seg15[0] alpha_ob23_seg15[1]
 alpha_ob23_seg15[2] alpha_ob23_seg15[3] alpha_ob24_seg15[0]
 alpha_ob24_seg15[1] alpha_ob24_seg15[2] alpha_ob24_seg15[3]
 alpha_ob25_seg15[0] alpha_ob25_seg15[1] alpha_ob25_seg15[2]
 alpha_ob25_seg15[3] alpha_ob26_seg15[0] alpha_ob26_seg15[1]
 alpha_ob26_seg15[2] alpha_ob26_seg15[3] alpha_ob27_seg15[0]
 alpha_ob27_seg15[1] alpha_ob27_seg15[2] alpha_ob27_seg15[3]
 alpha_ob28_seg15[0] alpha_ob28_seg15[1] alpha_ob28_seg15[2]
 alpha_ob28_seg15[3] alpha_ob29_seg15[0] alpha_ob29_seg15[1]
 alpha_ob29_seg15[2] alpha_ob29_seg15[3] alpha_ob30_seg15[0]
 alpha_ob30_seg15[1] alpha_ob30_seg15[2] alpha_ob30_seg15[3]
 alpha_ob31_seg15[0] alpha_ob31_seg15[1] alpha_ob31_seg15[2]
 alpha_ob31_seg15[3] alpha_ob32_seg15[0] alpha_ob32_seg15[1]
 alpha_ob32_seg15[2] alpha_ob32_seg15[3] alpha_ob33_seg15[0]
 alpha_ob33_seg15[1] alpha_ob33_seg15[2] alpha_ob33_seg15[3]
 alpha_ob34_seg15[0] alpha_ob34_seg15[1] alpha_ob34_seg15[2]
 alpha_ob34_seg15[3] alpha_ob35_seg15[0] alpha_ob35_seg15[1]
 alpha_ob35_seg15[2] alpha_ob35_seg15[3] alpha_ob36_seg15[0]
 alpha_ob36_seg15[1] alpha_ob36_seg15[2] alpha_ob36_seg15[3]
 alpha_ob37_seg15[0] alpha_ob37_seg15[1] alpha_ob37_seg15[2]
 alpha_ob37_seg15[3] alpha_ob38_seg15[0] alpha_ob38_seg15[1]
 alpha_ob38_seg15[2] alpha_ob38_seg15[3] alpha_ob39_seg15[0]
 alpha_ob39_seg15[1] alpha_ob39_seg15[2] alpha_ob39_seg15[3]
 alpha_ob40_seg15[0] alpha_ob40_seg15[1] alpha_ob40_seg15[2]
 alpha_ob40_seg15[3] alpha_ob41_seg15[0] alpha_ob41_seg15[1]
 alpha_ob41_seg15[2] alpha_ob41_seg15[3] alpha_ob42_seg15[0]
 alpha_ob42_seg15[1] alpha_ob42_seg15[2] alpha_ob42_seg15[3]
 alpha_ob43_seg15[0] alpha_ob43_seg15[1] alpha_ob43_seg15[2]
 alpha_ob43_seg15[3] alpha_ob44_seg15[0] alpha_ob44_seg15[1]
 alpha_ob44_seg15[2] alpha_ob44_seg15[3] alpha_ob45_seg15[0]
 alpha_ob45_seg15[1] alpha_ob45_seg15[2] alpha_ob45_seg15[3]
 alpha_ob46_seg15[0] alpha_ob46_seg15[1] alpha_ob46_seg15[2]
 alpha_ob46_seg15[3] alpha_ob47_seg15[0] alpha_ob47_seg15[1]
 alpha_ob47_seg15[2] alpha_ob47_seg15[3] alpha_ob48_seg15[0]
 alpha_ob48_seg15[1] alpha_ob48_seg15[2] alpha_ob48_seg15[3]
 alpha_ob49_seg15[0] alpha_ob49_seg15[1] alpha_ob49_seg15[2]
 alpha_ob49_seg15[3] alpha_ob50_seg15[0] alpha_ob50_seg15[1]
 alpha_ob50_seg15[2] alpha_ob50_seg15[3] alpha_ob0_seg16[0]
 alpha_ob0_seg16[1] alpha_ob0_seg16[2] alpha_ob0_seg16[3]
 alpha_ob1_seg16[0] alpha_ob1_seg16[1] alpha_ob1_seg16[2]
 alpha_ob1_seg16[3] alpha_ob2_seg16[0] alpha_ob2_seg16[1]
 alpha_ob2_seg16[2] alpha_ob2_seg16[3] alpha_ob3_seg16[0]
 alpha_ob3_seg16[1] alpha_ob3_seg16[2] alpha_ob3_seg16[3]
 alpha_ob4_seg16[0] alpha_ob4_seg16[1] alpha_ob4_seg16[2]
 alpha_ob4_seg16[3] alpha_ob5_seg16[0] alpha_ob5_seg16[1]
 alpha_ob5_seg16[2] alpha_ob5_seg16[3] alpha_ob6_seg16[0]
 alpha_ob6_seg16[1] alpha_ob6_seg16[2] alpha_ob6_seg16[3]
 alpha_ob7_seg16[0] alpha_ob7_seg16[1] alpha_ob7_seg16[2]
 alpha_ob7_seg16[3] alpha_ob8_seg16[0] alpha_ob8_seg16[1]
 alpha_ob8_seg16[2] alpha_ob8_seg16[3] alpha_ob9_seg16[0]
 alpha_ob9_seg16[1] alpha_ob9_seg16[2] alpha_ob9_seg16[3]
 alpha_ob10_seg16[0] alpha_ob10_seg16[1] alpha_ob10_seg16[2]
 alpha_ob10_seg16[3] alpha_ob11_seg16[0] alpha_ob11_seg16[1]
 alpha_ob11_seg16[2] alpha_ob11_seg16[3] alpha_ob12_seg16[0]
 alpha_ob12_seg16[1] alpha_ob12_seg16[2] alpha_ob12_seg16[3]
 alpha_ob13_seg16[0] alpha_ob13_seg16[1] alpha_ob13_seg16[2]
 alpha_ob13_seg16[3] alpha_ob14_seg16[0] alpha_ob14_seg16[1]
 alpha_ob14_seg16[2] alpha_ob14_seg16[3] alpha_ob15_seg16[0]
 alpha_ob15_seg16[1] alpha_ob15_seg16[2] alpha_ob15_seg16[3]
 alpha_ob16_seg16[0] alpha_ob16_seg16[1] alpha_ob16_seg16[2]
 alpha_ob16_seg16[3] alpha_ob17_seg16[0] alpha_ob17_seg16[1]
 alpha_ob17_seg16[2] alpha_ob17_seg16[3] alpha_ob18_seg16[0]
 alpha_ob18_seg16[1] alpha_ob18_seg16[2] alpha_ob18_seg16[3]
 alpha_ob19_seg16[0] alpha_ob19_seg16[1] alpha_ob19_seg16[2]
 alpha_ob19_seg16[3] alpha_ob20_seg16[0] alpha_ob20_seg16[1]
 alpha_ob20_seg16[2] alpha_ob20_seg16[3] alpha_ob21_seg16[0]
 alpha_ob21_seg16[1] alpha_ob21_seg16[2] alpha_ob21_seg16[3]
 alpha_ob22_seg16[0] alpha_ob22_seg16[1] alpha_ob22_seg16[2]
 alpha_ob22_seg16[3] alpha_ob23_seg16[0] alpha_ob23_seg16[1]
 alpha_ob23_seg16[2] alpha_ob23_seg16[3] alpha_ob24_seg16[0]
 alpha_ob24_seg16[1] alpha_ob24_seg16[2] alpha_ob24_seg16[3]
 alpha_ob25_seg16[0] alpha_ob25_seg16[1] alpha_ob25_seg16[2]
 alpha_ob25_seg16[3] alpha_ob26_seg16[0] alpha_ob26_seg16[1]
 alpha_ob26_seg16[2] alpha_ob26_seg16[3] alpha_ob27_seg16[0]
 alpha_ob27_seg16[1] alpha_ob27_seg16[2] alpha_ob27_seg16[3]
 alpha_ob28_seg16[0] alpha_ob28_seg16[1] alpha_ob28_seg16[2]
 alpha_ob28_seg16[3] alpha_ob29_seg16[0] alpha_ob29_seg16[1]
 alpha_ob29_seg16[2] alpha_ob29_seg16[3] alpha_ob30_seg16[0]
 alpha_ob30_seg16[1] alpha_ob30_seg16[2] alpha_ob30_seg16[3]
 alpha_ob31_seg16[0] alpha_ob31_seg16[1] alpha_ob31_seg16[2]
 alpha_ob31_seg16[3] alpha_ob32_seg16[0] alpha_ob32_seg16[1]
 alpha_ob32_seg16[2] alpha_ob32_seg16[3] alpha_ob33_seg16[0]
 alpha_ob33_seg16[1] alpha_ob33_seg16[2] alpha_ob33_seg16[3]
 alpha_ob34_seg16[0] alpha_ob34_seg16[1] alpha_ob34_seg16[2]
 alpha_ob34_seg16[3] alpha_ob35_seg16[0] alpha_ob35_seg16[1]
 alpha_ob35_seg16[2] alpha_ob35_seg16[3] alpha_ob36_seg16[0]
 alpha_ob36_seg16[1] alpha_ob36_seg16[2] alpha_ob36_seg16[3]
 alpha_ob37_seg16[0] alpha_ob37_seg16[1] alpha_ob37_seg16[2]
 alpha_ob37_seg16[3] alpha_ob38_seg16[0] alpha_ob38_seg16[1]
 alpha_ob38_seg16[2] alpha_ob38_seg16[3] alpha_ob39_seg16[0]
 alpha_ob39_seg16[1] alpha_ob39_seg16[2] alpha_ob39_seg16[3]
 alpha_ob40_seg16[0] alpha_ob40_seg16[1] alpha_ob40_seg16[2]
 alpha_ob40_seg16[3] alpha_ob41_seg16[0] alpha_ob41_seg16[1]
 alpha_ob41_seg16[2] alpha_ob41_seg16[3] alpha_ob42_seg16[0]
 alpha_ob42_seg16[1] alpha_ob42_seg16[2] alpha_ob42_seg16[3]
 alpha_ob43_seg16[0] alpha_ob43_seg16[1] alpha_ob43_seg16[2]
 alpha_ob43_seg16[3] alpha_ob44_seg16[0] alpha_ob44_seg16[1]
 alpha_ob44_seg16[2] alpha_ob44_seg16[3] alpha_ob45_seg16[0]
 alpha_ob45_seg16[1] alpha_ob45_seg16[2] alpha_ob45_seg16[3]
 alpha_ob46_seg16[0] alpha_ob46_seg16[1] alpha_ob46_seg16[2]
 alpha_ob46_seg16[3] alpha_ob47_seg16[0] alpha_ob47_seg16[1]
 alpha_ob47_seg16[2] alpha_ob47_seg16[3] alpha_ob48_seg16[0]
 alpha_ob48_seg16[1] alpha_ob48_seg16[2] alpha_ob48_seg16[3]
 alpha_ob49_seg16[0] alpha_ob49_seg16[1] alpha_ob49_seg16[2]
 alpha_ob49_seg16[3] alpha_ob50_seg16[0] alpha_ob50_seg16[1]
 alpha_ob50_seg16[2] alpha_ob50_seg16[3] alpha_ob0_seg17[0]
 alpha_ob0_seg17[1] alpha_ob0_seg17[2] alpha_ob0_seg17[3]
 alpha_ob1_seg17[0] alpha_ob1_seg17[1] alpha_ob1_seg17[2]
 alpha_ob1_seg17[3] alpha_ob2_seg17[0] alpha_ob2_seg17[1]
 alpha_ob2_seg17[2] alpha_ob2_seg17[3] alpha_ob3_seg17[0]
 alpha_ob3_seg17[1] alpha_ob3_seg17[2] alpha_ob3_seg17[3]
 alpha_ob4_seg17[0] alpha_ob4_seg17[1] alpha_ob4_seg17[2]
 alpha_ob4_seg17[3] alpha_ob5_seg17[0] alpha_ob5_seg17[1]
 alpha_ob5_seg17[2] alpha_ob5_seg17[3] alpha_ob6_seg17[0]
 alpha_ob6_seg17[1] alpha_ob6_seg17[2] alpha_ob6_seg17[3]
 alpha_ob7_seg17[0] alpha_ob7_seg17[1] alpha_ob7_seg17[2]
 alpha_ob7_seg17[3] alpha_ob8_seg17[0] alpha_ob8_seg17[1]
 alpha_ob8_seg17[2] alpha_ob8_seg17[3] alpha_ob9_seg17[0]
 alpha_ob9_seg17[1] alpha_ob9_seg17[2] alpha_ob9_seg17[3]
 alpha_ob10_seg17[0] alpha_ob10_seg17[1] alpha_ob10_seg17[2]
 alpha_ob10_seg17[3] alpha_ob11_seg17[0] alpha_ob11_seg17[1]
 alpha_ob11_seg17[2] alpha_ob11_seg17[3] alpha_ob12_seg17[0]
 alpha_ob12_seg17[1] alpha_ob12_seg17[2] alpha_ob12_seg17[3]
 alpha_ob13_seg17[0] alpha_ob13_seg17[1] alpha_ob13_seg17[2]
 alpha_ob13_seg17[3] alpha_ob14_seg17[0] alpha_ob14_seg17[1]
 alpha_ob14_seg17[2] alpha_ob14_seg17[3] alpha_ob15_seg17[0]
 alpha_ob15_seg17[1] alpha_ob15_seg17[2] alpha_ob15_seg17[3]
 alpha_ob16_seg17[0] alpha_ob16_seg17[1] alpha_ob16_seg17[2]
 alpha_ob16_seg17[3] alpha_ob17_seg17[0] alpha_ob17_seg17[1]
 alpha_ob17_seg17[2] alpha_ob17_seg17[3] alpha_ob18_seg17[0]
 alpha_ob18_seg17[1] alpha_ob18_seg17[2] alpha_ob18_seg17[3]
 alpha_ob19_seg17[0] alpha_ob19_seg17[1] alpha_ob19_seg17[2]
 alpha_ob19_seg17[3] alpha_ob20_seg17[0] alpha_ob20_seg17[1]
 alpha_ob20_seg17[2] alpha_ob20_seg17[3] alpha_ob21_seg17[0]
 alpha_ob21_seg17[1] alpha_ob21_seg17[2] alpha_ob21_seg17[3]
 alpha_ob22_seg17[0] alpha_ob22_seg17[1] alpha_ob22_seg17[2]
 alpha_ob22_seg17[3] alpha_ob23_seg17[0] alpha_ob23_seg17[1]
 alpha_ob23_seg17[2] alpha_ob23_seg17[3] alpha_ob24_seg17[0]
 alpha_ob24_seg17[1] alpha_ob24_seg17[2] alpha_ob24_seg17[3]
 alpha_ob25_seg17[0] alpha_ob25_seg17[1] alpha_ob25_seg17[2]
 alpha_ob25_seg17[3] alpha_ob26_seg17[0] alpha_ob26_seg17[1]
 alpha_ob26_seg17[2] alpha_ob26_seg17[3] alpha_ob27_seg17[0]
 alpha_ob27_seg17[1] alpha_ob27_seg17[2] alpha_ob27_seg17[3]
 alpha_ob28_seg17[0] alpha_ob28_seg17[1] alpha_ob28_seg17[2]
 alpha_ob28_seg17[3] alpha_ob29_seg17[0] alpha_ob29_seg17[1]
 alpha_ob29_seg17[2] alpha_ob29_seg17[3] alpha_ob30_seg17[0]
 alpha_ob30_seg17[1] alpha_ob30_seg17[2] alpha_ob30_seg17[3]
 alpha_ob31_seg17[0] alpha_ob31_seg17[1] alpha_ob31_seg17[2]
 alpha_ob31_seg17[3] alpha_ob32_seg17[0] alpha_ob32_seg17[1]
 alpha_ob32_seg17[2] alpha_ob32_seg17[3] alpha_ob33_seg17[0]
 alpha_ob33_seg17[1] alpha_ob33_seg17[2] alpha_ob33_seg17[3]
 alpha_ob34_seg17[0] alpha_ob34_seg17[1] alpha_ob34_seg17[2]
 alpha_ob34_seg17[3] alpha_ob35_seg17[0] alpha_ob35_seg17[1]
 alpha_ob35_seg17[2] alpha_ob35_seg17[3] alpha_ob36_seg17[0]
 alpha_ob36_seg17[1] alpha_ob36_seg17[2] alpha_ob36_seg17[3]
 alpha_ob37_seg17[0] alpha_ob37_seg17[1] alpha_ob37_seg17[2]
 alpha_ob37_seg17[3] alpha_ob38_seg17[0] alpha_ob38_seg17[1]
 alpha_ob38_seg17[2] alpha_ob38_seg17[3] alpha_ob39_seg17[0]
 alpha_ob39_seg17[1] alpha_ob39_seg17[2] alpha_ob39_seg17[3]
 alpha_ob40_seg17[0] alpha_ob40_seg17[1] alpha_ob40_seg17[2]
 alpha_ob40_seg17[3] alpha_ob41_seg17[0] alpha_ob41_seg17[1]
 alpha_ob41_seg17[2] alpha_ob41_seg17[3] alpha_ob42_seg17[0]
 alpha_ob42_seg17[1] alpha_ob42_seg17[2] alpha_ob42_seg17[3]
 alpha_ob43_seg17[0] alpha_ob43_seg17[1] alpha_ob43_seg17[2]
 alpha_ob43_seg17[3] alpha_ob44_seg17[0] alpha_ob44_seg17[1]
 alpha_ob44_seg17[2] alpha_ob44_seg17[3] alpha_ob45_seg17[0]
 alpha_ob45_seg17[1] alpha_ob45_seg17[2] alpha_ob45_seg17[3]
 alpha_ob46_seg17[0] alpha_ob46_seg17[1] alpha_ob46_seg17[2]
 alpha_ob46_seg17[3] alpha_ob47_seg17[0] alpha_ob47_seg17[1]
 alpha_ob47_seg17[2] alpha_ob47_seg17[3] alpha_ob48_seg17[0]
 alpha_ob48_seg17[1] alpha_ob48_seg17[2] alpha_ob48_seg17[3]
 alpha_ob49_seg17[0] alpha_ob49_seg17[1] alpha_ob49_seg17[2]
 alpha_ob49_seg17[3] alpha_ob50_seg17[0] alpha_ob50_seg17[1]
 alpha_ob50_seg17[2] alpha_ob50_seg17[3] alpha_ob0_seg18[0]
 alpha_ob0_seg18[1] alpha_ob0_seg18[2] alpha_ob0_seg18[3]
 alpha_ob1_seg18[0] alpha_ob1_seg18[1] alpha_ob1_seg18[2]
 alpha_ob1_seg18[3] alpha_ob2_seg18[0] alpha_ob2_seg18[1]
 alpha_ob2_seg18[2] alpha_ob2_seg18[3] alpha_ob3_seg18[0]
 alpha_ob3_seg18[1] alpha_ob3_seg18[2] alpha_ob3_seg18[3]
 alpha_ob4_seg18[0] alpha_ob4_seg18[1] alpha_ob4_seg18[2]
 alpha_ob4_seg18[3] alpha_ob5_seg18[0] alpha_ob5_seg18[1]
 alpha_ob5_seg18[2] alpha_ob5_seg18[3] alpha_ob6_seg18[0]
 alpha_ob6_seg18[1] alpha_ob6_seg18[2] alpha_ob6_seg18[3]
 alpha_ob7_seg18[0] alpha_ob7_seg18[1] alpha_ob7_seg18[2]
 alpha_ob7_seg18[3] alpha_ob8_seg18[0] alpha_ob8_seg18[1]
 alpha_ob8_seg18[2] alpha_ob8_seg18[3] alpha_ob9_seg18[0]
 alpha_ob9_seg18[1] alpha_ob9_seg18[2] alpha_ob9_seg18[3]
 alpha_ob10_seg18[0] alpha_ob10_seg18[1] alpha_ob10_seg18[2]
 alpha_ob10_seg18[3] alpha_ob11_seg18[0] alpha_ob11_seg18[1]
 alpha_ob11_seg18[2] alpha_ob11_seg18[3] alpha_ob12_seg18[0]
 alpha_ob12_seg18[1] alpha_ob12_seg18[2] alpha_ob12_seg18[3]
 alpha_ob13_seg18[0] alpha_ob13_seg18[1] alpha_ob13_seg18[2]
 alpha_ob13_seg18[3] alpha_ob14_seg18[0] alpha_ob14_seg18[1]
 alpha_ob14_seg18[2] alpha_ob14_seg18[3] alpha_ob15_seg18[0]
 alpha_ob15_seg18[1] alpha_ob15_seg18[2] alpha_ob15_seg18[3]
 alpha_ob16_seg18[0] alpha_ob16_seg18[1] alpha_ob16_seg18[2]
 alpha_ob16_seg18[3] alpha_ob17_seg18[0] alpha_ob17_seg18[1]
 alpha_ob17_seg18[2] alpha_ob17_seg18[3] alpha_ob18_seg18[0]
 alpha_ob18_seg18[1] alpha_ob18_seg18[2] alpha_ob18_seg18[3]
 alpha_ob19_seg18[0] alpha_ob19_seg18[1] alpha_ob19_seg18[2]
 alpha_ob19_seg18[3] alpha_ob20_seg18[0] alpha_ob20_seg18[1]
 alpha_ob20_seg18[2] alpha_ob20_seg18[3] alpha_ob21_seg18[0]
 alpha_ob21_seg18[1] alpha_ob21_seg18[2] alpha_ob21_seg18[3]
 alpha_ob22_seg18[0] alpha_ob22_seg18[1] alpha_ob22_seg18[2]
 alpha_ob22_seg18[3] alpha_ob23_seg18[0] alpha_ob23_seg18[1]
 alpha_ob23_seg18[2] alpha_ob23_seg18[3] alpha_ob24_seg18[0]
 alpha_ob24_seg18[1] alpha_ob24_seg18[2] alpha_ob24_seg18[3]
 alpha_ob25_seg18[0] alpha_ob25_seg18[1] alpha_ob25_seg18[2]
 alpha_ob25_seg18[3] alpha_ob26_seg18[0] alpha_ob26_seg18[1]
 alpha_ob26_seg18[2] alpha_ob26_seg18[3] alpha_ob27_seg18[0]
 alpha_ob27_seg18[1] alpha_ob27_seg18[2] alpha_ob27_seg18[3]
 alpha_ob28_seg18[0] alpha_ob28_seg18[1] alpha_ob28_seg18[2]
 alpha_ob28_seg18[3] alpha_ob29_seg18[0] alpha_ob29_seg18[1]
 alpha_ob29_seg18[2] alpha_ob29_seg18[3] alpha_ob30_seg18[0]
 alpha_ob30_seg18[1] alpha_ob30_seg18[2] alpha_ob30_seg18[3]
 alpha_ob31_seg18[0] alpha_ob31_seg18[1] alpha_ob31_seg18[2]
 alpha_ob31_seg18[3] alpha_ob32_seg18[0] alpha_ob32_seg18[1]
 alpha_ob32_seg18[2] alpha_ob32_seg18[3] alpha_ob33_seg18[0]
 alpha_ob33_seg18[1] alpha_ob33_seg18[2] alpha_ob33_seg18[3]
 alpha_ob34_seg18[0] alpha_ob34_seg18[1] alpha_ob34_seg18[2]
 alpha_ob34_seg18[3] alpha_ob35_seg18[0] alpha_ob35_seg18[1]
 alpha_ob35_seg18[2] alpha_ob35_seg18[3] alpha_ob36_seg18[0]
 alpha_ob36_seg18[1] alpha_ob36_seg18[2] alpha_ob36_seg18[3]
 alpha_ob37_seg18[0] alpha_ob37_seg18[1] alpha_ob37_seg18[2]
 alpha_ob37_seg18[3] alpha_ob38_seg18[0] alpha_ob38_seg18[1]
 alpha_ob38_seg18[2] alpha_ob38_seg18[3] alpha_ob39_seg18[0]
 alpha_ob39_seg18[1] alpha_ob39_seg18[2] alpha_ob39_seg18[3]
 alpha_ob40_seg18[0] alpha_ob40_seg18[1] alpha_ob40_seg18[2]
 alpha_ob40_seg18[3] alpha_ob41_seg18[0] alpha_ob41_seg18[1]
 alpha_ob41_seg18[2] alpha_ob41_seg18[3] alpha_ob42_seg18[0]
 alpha_ob42_seg18[1] alpha_ob42_seg18[2] alpha_ob42_seg18[3]
 alpha_ob43_seg18[0] alpha_ob43_seg18[1] alpha_ob43_seg18[2]
 alpha_ob43_seg18[3] alpha_ob44_seg18[0] alpha_ob44_seg18[1]
 alpha_ob44_seg18[2] alpha_ob44_seg18[3] alpha_ob45_seg18[0]
 alpha_ob45_seg18[1] alpha_ob45_seg18[2] alpha_ob45_seg18[3]
 alpha_ob46_seg18[0] alpha_ob46_seg18[1] alpha_ob46_seg18[2]
 alpha_ob46_seg18[3] alpha_ob47_seg18[0] alpha_ob47_seg18[1]
 alpha_ob47_seg18[2] alpha_ob47_seg18[3] alpha_ob48_seg18[0]
 alpha_ob48_seg18[1] alpha_ob48_seg18[2] alpha_ob48_seg18[3]
 alpha_ob49_seg18[0] alpha_ob49_seg18[1] alpha_ob49_seg18[2]
 alpha_ob49_seg18[3] alpha_ob50_seg18[0] alpha_ob50_seg18[1]
 alpha_ob50_seg18[2] alpha_ob50_seg18[3] alpha_ob0_seg19[0]
 alpha_ob0_seg19[1] alpha_ob0_seg19[2] alpha_ob0_seg19[3]
 alpha_ob1_seg19[0] alpha_ob1_seg19[1] alpha_ob1_seg19[2]
 alpha_ob1_seg19[3] alpha_ob2_seg19[0] alpha_ob2_seg19[1]
 alpha_ob2_seg19[2] alpha_ob2_seg19[3] alpha_ob3_seg19[0]
 alpha_ob3_seg19[1] alpha_ob3_seg19[2] alpha_ob3_seg19[3]
 alpha_ob4_seg19[0] alpha_ob4_seg19[1] alpha_ob4_seg19[2]
 alpha_ob4_seg19[3] alpha_ob5_seg19[0] alpha_ob5_seg19[1]
 alpha_ob5_seg19[2] alpha_ob5_seg19[3] alpha_ob6_seg19[0]
 alpha_ob6_seg19[1] alpha_ob6_seg19[2] alpha_ob6_seg19[3]
 alpha_ob7_seg19[0] alpha_ob7_seg19[1] alpha_ob7_seg19[2]
 alpha_ob7_seg19[3] alpha_ob8_seg19[0] alpha_ob8_seg19[1]
 alpha_ob8_seg19[2] alpha_ob8_seg19[3] alpha_ob9_seg19[0]
 alpha_ob9_seg19[1] alpha_ob9_seg19[2] alpha_ob9_seg19[3]
 alpha_ob10_seg19[0] alpha_ob10_seg19[1] alpha_ob10_seg19[2]
 alpha_ob10_seg19[3] alpha_ob11_seg19[0] alpha_ob11_seg19[1]
 alpha_ob11_seg19[2] alpha_ob11_seg19[3] alpha_ob12_seg19[0]
 alpha_ob12_seg19[1] alpha_ob12_seg19[2] alpha_ob12_seg19[3]
 alpha_ob13_seg19[0] alpha_ob13_seg19[1] alpha_ob13_seg19[2]
 alpha_ob13_seg19[3] alpha_ob14_seg19[0] alpha_ob14_seg19[1]
 alpha_ob14_seg19[2] alpha_ob14_seg19[3] alpha_ob15_seg19[0]
 alpha_ob15_seg19[1] alpha_ob15_seg19[2] alpha_ob15_seg19[3]
 alpha_ob16_seg19[0] alpha_ob16_seg19[1] alpha_ob16_seg19[2]
 alpha_ob16_seg19[3] alpha_ob17_seg19[0] alpha_ob17_seg19[1]
 alpha_ob17_seg19[2] alpha_ob17_seg19[3] alpha_ob18_seg19[0]
 alpha_ob18_seg19[1] alpha_ob18_seg19[2] alpha_ob18_seg19[3]
 alpha_ob19_seg19[0] alpha_ob19_seg19[1] alpha_ob19_seg19[2]
 alpha_ob19_seg19[3] alpha_ob20_seg19[0] alpha_ob20_seg19[1]
 alpha_ob20_seg19[2] alpha_ob20_seg19[3] alpha_ob21_seg19[0]
 alpha_ob21_seg19[1] alpha_ob21_seg19[2] alpha_ob21_seg19[3]
 alpha_ob22_seg19[0] alpha_ob22_seg19[1] alpha_ob22_seg19[2]
 alpha_ob22_seg19[3] alpha_ob23_seg19[0] alpha_ob23_seg19[1]
 alpha_ob23_seg19[2] alpha_ob23_seg19[3] alpha_ob24_seg19[0]
 alpha_ob24_seg19[1] alpha_ob24_seg19[2] alpha_ob24_seg19[3]
 alpha_ob25_seg19[0] alpha_ob25_seg19[1] alpha_ob25_seg19[2]
 alpha_ob25_seg19[3] alpha_ob26_seg19[0] alpha_ob26_seg19[1]
 alpha_ob26_seg19[2] alpha_ob26_seg19[3] alpha_ob27_seg19[0]
 alpha_ob27_seg19[1] alpha_ob27_seg19[2] alpha_ob27_seg19[3]
 alpha_ob28_seg19[0] alpha_ob28_seg19[1] alpha_ob28_seg19[2]
 alpha_ob28_seg19[3] alpha_ob29_seg19[0] alpha_ob29_seg19[1]
 alpha_ob29_seg19[2] alpha_ob29_seg19[3] alpha_ob30_seg19[0]
 alpha_ob30_seg19[1] alpha_ob30_seg19[2] alpha_ob30_seg19[3]
 alpha_ob31_seg19[0] alpha_ob31_seg19[1] alpha_ob31_seg19[2]
 alpha_ob31_seg19[3] alpha_ob32_seg19[0] alpha_ob32_seg19[1]
 alpha_ob32_seg19[2] alpha_ob32_seg19[3] alpha_ob33_seg19[0]
 alpha_ob33_seg19[1] alpha_ob33_seg19[2] alpha_ob33_seg19[3]
 alpha_ob34_seg19[0] alpha_ob34_seg19[1] alpha_ob34_seg19[2]
 alpha_ob34_seg19[3] alpha_ob35_seg19[0] alpha_ob35_seg19[1]
 alpha_ob35_seg19[2] alpha_ob35_seg19[3] alpha_ob36_seg19[0]
 alpha_ob36_seg19[1] alpha_ob36_seg19[2] alpha_ob36_seg19[3]
 alpha_ob37_seg19[0] alpha_ob37_seg19[1] alpha_ob37_seg19[2]
 alpha_ob37_seg19[3] alpha_ob38_seg19[0] alpha_ob38_seg19[1]
 alpha_ob38_seg19[2] alpha_ob38_seg19[3] alpha_ob39_seg19[0]
 alpha_ob39_seg19[1] alpha_ob39_seg19[2] alpha_ob39_seg19[3]
 alpha_ob40_seg19[0] alpha_ob40_seg19[1] alpha_ob40_seg19[2]
 alpha_ob40_seg19[3] alpha_ob41_seg19[0] alpha_ob41_seg19[1]
 alpha_ob41_seg19[2] alpha_ob41_seg19[3] alpha_ob42_seg19[0]
 alpha_ob42_seg19[1] alpha_ob42_seg19[2] alpha_ob42_seg19[3]
 alpha_ob43_seg19[0] alpha_ob43_seg19[1] alpha_ob43_seg19[2]
 alpha_ob43_seg19[3] alpha_ob44_seg19[0] alpha_ob44_seg19[1]
 alpha_ob44_seg19[2] alpha_ob44_seg19[3] alpha_ob45_seg19[0]
 alpha_ob45_seg19[1] alpha_ob45_seg19[2] alpha_ob45_seg19[3]
 alpha_ob46_seg19[0] alpha_ob46_seg19[1] alpha_ob46_seg19[2]
 alpha_ob46_seg19[3] alpha_ob47_seg19[0] alpha_ob47_seg19[1]
 alpha_ob47_seg19[2] alpha_ob47_seg19[3] alpha_ob48_seg19[0]
 alpha_ob48_seg19[1] alpha_ob48_seg19[2] alpha_ob48_seg19[3]
 alpha_ob49_seg19[0] alpha_ob49_seg19[1] alpha_ob49_seg19[2]
 alpha_ob49_seg19[3] alpha_ob50_seg19[0] alpha_ob50_seg19[1]
 alpha_ob50_seg19[2] alpha_ob50_seg19[3]
End
