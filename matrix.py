import math

ROBOT_nDOFs = 6

class Kinematics:
    def __init__(self):
        # declare in out vars
        self.xyzuvw_Out = [0] * ROBOT_nDOFs
        self.xyzuvw_In = [0] * ROBOT_nDOFs
        self.xyzuvw_Temp = [0] * ROBOT_nDOFs

        self.JangleOut = [0] * ROBOT_nDOFs
        self.JangleIn = [0] * ROBOT_nDOFs

        # external axis
        self.J7_pos = 0
        self.J8_pos = 0
        self.J9_pos = 0

        self.J7_In = 0
        self.J8_In = 0
        self.J9_In = 0

        self.Table_Size = 6
        self.Matrix4x4 = [0] * 16
        self.tRobot = [0] * 66

        self.pose = [0] * 16

        self.moveSequence = ""

        # define rounding vars
        self.rndArcStart = [0] * 6
        self.rndArcMid = [0] * 6
        self.rndArcEnd = [0] * 6
        self.rndCalcCen = [0] * 6
        self.rndData = ""
        self.rndTrue = False
        self.rndSpeed = 0
        self.splineTrue = False
        self.splineEndReceived = False

        self.Xtool = 0
        self.Ytool = 0
        self.Ztool = 0
        self.RZtool = 0
        self.RYtool = 0
        self.RXtool = 0

        # DENAVIT HARTENBERG PARAMETERS
        self.DHparams = [
            [0, -90, 169.77, 64.2],
            [-90, 0, 0, 305],
            [180, 90, 0, -0.0001],
            [0, -90, 222.63, 0],
            [0, 90, 0, 0],
            [0, 0, 36.25, 0]
        ]

        # DECLARE TOOL FRAME
        self.toolFrame = [[0] * 4 for _ in range(4)]
        self.toolFrameRev = [[0] * 4 for _ in range(4)]

        # DECLARE R06 NEG FRAME
        self.R06_neg_matrix = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -self.DHparams[5][2]],
            [0, 0, 0, 1]
        ]

        # DECLARE JOINT MATRICES
        self.J1matrix = [[0] * 4 for _ in range(4)]
        self.J2matrix = [[0] * 4 for _ in range(4)]
        self.J3matrix = [[0] * 4 for _ in range(4)]
        self.J4matrix = [[0] * 4 for _ in range(4)]
        self.J5matrix = [[0] * 4 for _ in range(4)]
        self.J6matrix = [[0] * 4 for _ in range(4)]

        self.J1matrix_rev = [[0] * 4 for _ in range(4)]
        self.J2matrix_rev = [[0] * 4 for _ in range(4)]
        self.J3matrix_rev = [[0] * 4 for _ in range(4)]

        self.R02matrix = [[0] * 4 for _ in range(4)]
        self.R03matrix = [[0] * 4 for _ in range(4)]
        self.R04matrix = [[0] * 4 for _ in range(4)]
        self.R05matrix = [[0] * 4 for _ in range(4)]
        self.R06matrix = [[0] * 4 for _ in range(4)]
        self.R0Tmatrix = [[0] * 4 for _ in range(4)]

        self.R02matrix_rev = [[0] * 4 for _ in range(4)]
        self.R03matrix_rev = [[0] * 4 for _ in range(4)]

        self.R0T_rev_matrix = [[0] * 4 for _ in range(4)]
        self.InvtoolFrame = [[0] * 4 for _ in range(4)]
        self.R06_rev_matrix = [[0] * 4 for _ in range(4)]
        self.R05_rev_matrix = [[0] * 4 for _ in range(4)]
        self.InvR03matrix_rev = [[0] * 4 for _ in range(4)]
        self.R03_6matrix = [[0] * 4 for _ in range(4)]

        self.blank = [[0] * 4 for _ in range(4)]

        self.toolFrame = [[0] * 4 for _ in range(4)]
        self.toolFrameRev = [[0] * 4 for _ in range(4)]


    # /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    # //CONSTRUCT TOOL MATRIX
    # /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    def Tool_Matrix(self, Xval, Yval, Zval, RZval, RYval, RXval):
        self.toolFrame = [[0 for _ in range(4)] for _ in range(4)]
        self.toolFrame[0][0] = math.cos(math.radians(RZval)) * math.cos(math.radians(RYval))
        self.toolFrame[0][1] = math.cos(math.radians(RZval)) * math.sin(math.radians(RYval)) * math.sin(math.radians(RXval)) - math.sin(math.radians(RZval)) * math.cos(math.radians(RXval))
        self.toolFrame[0][2] = math.cos(math.radians(RZval)) * math.sin(math.radians(RYval)) * math.cos(math.radians(RXval)) + math.sin(math.radians(RZval)) * math.sin(math.radians(RXval))
        self.toolFrame[0][3] = Xval
        self.toolFrame[1][0] = math.sin(math.radians(RZval)) * math.cos(math.radians(RYval))
        self.toolFrame[1][1] = math.sin(math.radians(RZval)) * math.sin(math.radians(RYval)) * math.sin(math.radians(RXval)) + math.cos(math.radians(RZval)) * math.cos(math.radians(RXval))
        self.toolFrame[1][2] = math.sin(math.radians(RZval)) * math.sin(math.radians(RYval)) * math.cos(math.radians(RXval)) - math.cos(math.radians(RZval)) * math.sin(math.radians(RXval))
        self.toolFrame[1][3] = Yval
        self.toolFrame[2][0] = -math.sin(math.radians(RYval))
        self.toolFrame[2][1] = math.cos(math.radians(RYval)) * math.sin(math.radians(RXval))
        self.toolFrame[2][2] = math.cos(math.radians(RYval)) * math.cos(math.radians(RXval))
        self.toolFrame[2][3] = Zval
        self.toolFrame[3][0] = 0
        self.toolFrame[3][1] = 0
        self.toolFrame[3][2] = 0
        self.toolFrame[3][3] = 1
        return self.toolFrame

    def Tool_MatrixRev(self, Xval, Yval, Zval, RZval, RYval, RXval):
        self.toolFrameRev = [[0 for _ in range(4)] for _ in range(4)]
        self.toolFrameRev[0][0] = math.cos(math.radians(RZval)) * math.cos(math.radians(RYval))
        self.toolFrameRev[0][1] = math.cos(math.radians(RZval)) * math.sin(math.radians(RYval)) * math.sin(math.radians(RXval)) - math.sin(math.radians(RZval)) * math.cos(math.radians(RXval))
        self.toolFrameRev[0][2] = math.cos(math.radians(RZval)) * math.sin(math.radians(RYval)) * math.cos(math.radians(RXval)) + math.sin(math.radians(RZval)) * math.sin(math.radians(RXval))
        self.toolFrameRev[0][3] = -Xval
        self.toolFrameRev[1][0] = math.sin(math.radians(RZval)) * math.cos(math.radians(RYval))
        self.toolFrameRev[1][1] = math.sin(math.radians(RZval)) * math.sin(math.radians(RYval)) * math.sin(math.radians(RXval)) + math.cos(math.radians(RZval)) * math.cos(math.radians(RXval))
        self.toolFrameRev[1][2] = math.sin(math.radians(RZval)) * math.sin(math.radians(RYval)) * math.cos(math.radians(RXval)) - math.cos(math.radians(RZval)) * math.sin(math.radians(RXval))
        self.toolFrameRev[1][3] = -Yval
        self.toolFrameRev[2][0] = -math.sin(math.radians(RYval))
        self.toolFrameRev[2][1] = math.cos(math.radians(RYval)) * math.sin(math.radians(RXval))
        self.toolFrameRev[2][2] = math.cos(math.radians(RYval)) * math.cos(math.radians(RXval))
        self.toolFrameRev[2][3] = -Zval
        self.toolFrameRev[3][0] = 0
        self.toolFrameRev[3][1] = 0
        self.toolFrameRev[3][2] = 0
        self.toolFrameRev[3][3] = 1
        return self.toolFrameRev

    # /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    # //CONSTRUCT DH MATRICES FORWARD KINEMATICS
    # /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    def DH_Matrices(self, JangleIn, DHparams):

        if JangleIn[0] == 0:
            JangleIn[0] = 0.0001
        if JangleIn[1] == 0:
            JangleIn[1] = 0.0001
        if JangleIn[2] == 0:
            JangleIn[2] = 0.0001
        if JangleIn[3] == 0:
            JangleIn[3] = 0.0001
        if JangleIn[4] == 0:
            JangleIn[4] = 0.0001
        if JangleIn[5] == 0:
            JangleIn[5] = 0.0001

        self.J1matrix[0][0] = math.cos(math.radians(JangleIn[0] + DHparams[0][0]))
        self.J1matrix[0][1] = -math.s in(math.radians(JangleIn[0] + DHparams[0][0])) * math.cos(math.radians(DHparams[0][1]))
        self.J1matrix[0][2] = math.sin(math.radians(JangleIn[0] + DHparams[0][0])) * math.sin(math.radians(DHparams[0][1]))
        self.J1matrix[0][3] = DHparams[0][3] * math.cos(math.radians(JangleIn[0] + DHparams[0][0]))
        self.J1matrix[1][0] = math.sin(math.radians(JangleIn[0] + DHparams[0][0]))
        self.J1matrix[1][1] = math.cos(math.radians(JangleIn[0] + DHparams[0][0])) * math.cos(math.radians(DHparams[0][1]))
        self.J1matrix[1][2] = -math.cos(math.radians(JangleIn[0] + DHparams[0][0])) * math.sin(math.radians(DHparams[0][1]))
        self.J1matrix[1][3] = DHparams[0][3] * math.sin(math.radians(JangleIn[0] + DHparams[0][0]))
        self.J1matrix[2][0] = 0
        self.J1matrix[2][1] = math.sin(math.radians(DHparams[0][1]))
        self.J1matrix[2][2] = math.cos(math.radians(DHparams[0][1]))
        self.J1matrix[2][3] = DHparams[0][2]
        self.J1matrix[3][0] = 0
        self.J1matrix[3][1] = 0
        self.J1matrix[3][2] = 0
        self.J1matrix[3][3] = 1

        self.J2matrix[0][0] = math.cos(math.radians(JangleIn[1] + DHparams[1][0]))
        self.J2matrix[0][1] = -math.sin(math.radians(JangleIn[1] + DHparams[1][0])) * math.cos(math.radians(DHparams[1][1]))
        self.J2matrix[0][2] = math.sin(math.radians(JangleIn[1] + DHparams[1][0])) * math.sin(math.radians(DHparams[1][1]))
        self.J2matrix[0][3] = DHparams[1][3] * math.cos(math.radians(JangleIn[1] + DHparams[1][0]))
        self.J2matrix[1][0] = math.sin(math.radians(JangleIn[1] + DHparams[1][0]))
        self.J2matrix[1][1] = math.cos(math.radians(JangleIn[1] + DHparams[1][0])) * math.cos(math.radians(DHparams[1][1]))
        self.J2matrix[1][2] = -math.cos(math.radians(JangleIn[1] + DHparams[1][0])) * math.sin(math.radians(DHparams[1][1]))
        self.J2matrix[1][3] = DHparams[1][3] * math.sin(math.radians(JangleIn[1] + DHparams[1][0]))
        self.J2matrix[2][0] = 0
        self.J2matrix[2][1] = math.sin(math.radians(DHparams[1][1]))
        self.J2matrix[2][2] = math.cos(math.radians(DHparams[1][1]))
        self.J2matrix[2][3] = DHparams[1][2]
        self.J2matrix[3][0] = 0
        self.J2matrix[3][1] = 0
        self.J2matrix[3][2] = 0
        self.J2matrix[3][3] = 1

        self.J3matrix[0][0] = math.cos(math.radians(JangleIn[2] + DHparams[2][0]))
        self.J3matrix[0][1] = -math.sin(math.radians(JangleIn[2] + DHparams[2][0])) * math.cos(math.radians(DHparams[2][1]))
        self.J3matrix[0][2] = math.sin(math.radians(JangleIn[2] + DHparams[2][0])) * math.sin(math.radians(DHparams[2][1]))
        self.J3matrix[0][3] = DHparams[2][3] * math.cos(math.radians(JangleIn[2] + DHparams[2][0]))
        self.J3matrix[1][0] = math.sin(math.radians(JangleIn[2] + DHparams[2][0]))
        self.J3matrix[1][1] = math.cos(math.radians(JangleIn[2] + DHparams[2][0])) * math.cos(math.radians(DHparams[2][1]))
        self.J3matrix[1][2] = -math.cos(math.radians(JangleIn[2] + DHparams[2][0])) * math.sin(math.radians(DHparams[2][1]))
        self.J3matrix[1][3] = DHparams[2][3] * math.sin(math.radians(JangleIn[2] + DHparams[2][0]))
        self.J3matrix[2][0] = 0
        self.J3matrix[2][1] = math.sin(math.radians(DHparams[2][1]))
        self.J3matrix[2][2] = math.cos(math.radians(DHparams[2][1]))
        self.J3matrix[2][3] = DHparams[2][2]
        self.J3matrix[3][0] = 0
        self.J3matrix[3][1] = 0
        self.J3matrix[3][2] = 0
        self.J3matrix[3][3] = 1

        self.J4matrix[0][0] = math.cos(math.radians(JangleIn[3] + DHparams[3][0]))
        self.J4matrix[0][1] = -math.sin(math.radians(JangleIn[3] + DHparams[3][0])) * math.cos(math.radians(DHparams[3][1]))
        self.J4matrix[0][2] = math.sin(math.radians(JangleIn[3] + DHparams[3][0])) * math.sin(math.radians(DHparams[3][1]))
        self.J4matrix[0][3] = DHparams[3][3] * math.cos(math.radians(JangleIn[3] + DHparams[3][0]))
        self.J4matrix[1][0] = math.sin(math.radians(JangleIn[3] + DHparams[3][0]))
        self.J4matrix[1][1] = math.cos(math.radians(JangleIn[3] + DHparams[3][0])) * math.cos(math.radians(DHparams[3][1]))
        self.J4matrix[1][2] = -math.cos(math.radians(JangleIn[3] + DHparams[3][0])) * math.sin(math.radians(DHparams[3][1]))
        self.J4matrix[1][3] = DHparams[3][3] * math.sin(math.radians(JangleIn[3] + DHparams[3][0]))
        self.J4matrix[2][0] = 0
        self.J4matrix[2][1] = math.sin(math.radians(DHparams[3][1]))
        self.J4matrix[2][2] = math.cos(math.radians(DHparams[3][1]))
        self.J4matrix[2][3] = DHparams[3][2]
        self.J4matrix[3][0] = 0
        self.J4matrix[3][1] = 0
        self.J4matrix[3][2] = 0
        self.J4matrix[3][3] = 1

        self.J5matrix[0][0] = math.cos(math.radians(JangleIn[4] + DHparams[4][0]))
        self.J5matrix[0][1] = -math.sin(math.radians(JangleIn[4] + DHparams[4][0])) * math.cos(math.radians(DHparams[4][1]))
        self.J5matrix[0][2] = math.sin(math.radians(JangleIn[4] + DHparams[4][0])) * math.sin(math.radians(DHparams[4][1]))
        self.J5matrix[0][3] = DHparams[4][3] * math.cos(math.radians(JangleIn[4] + DHparams[4][0]))
        self.J5matrix[1][0] = math.sin(math.radians(JangleIn[4] + DHparams[4][0]))
        self.J5matrix[1][1] = math.cos(math.radians(JangleIn[4] + DHparams[4][0])) * math.cos(math.radians(DHparams[4][1]))
        self.J5matrix[1][2] = -math.cos(math.radians(JangleIn[4] + DHparams[4][0])) * math.sin(math.radians(DHparams[4][1]))
        self.J5matrix[1][3] = DHparams[4][3] * math.sin(math.radians(JangleIn[4] + DHparams[4][0]))
        self.J5matrix[2][0] = 0
        self.J5matrix[2][1] = math.sin(math.radians(DHparams[4][1]))
        self.J5matrix[2][2] = math.cos(math.radians(DHparams[4][1]))
        self.J5matrix[2][3] = DHparams[4][2]
        self.J5matrix[3][0] = 0
        self.J5matrix[3][1] = 0
        self.J5matrix[3][2] = 0
        self.J5matrix[3][3] = 1

        self.J6matrix[0][0] = math.cos(math.radians(JangleIn[5] + DHparams[5][0]))
        self.J6matrix[0][1] = -math.sin(math.radians(JangleIn[5] + DHparams[5][0])) * math.cos(math.radians(DHparams[5][1]))
        self.J6matrix[0][2] = math.sin(math.radians(JangleIn[5] + DHparams[5][0])) * math.sin(math.radians(DHparams[5][1]))
        self.J6matrix[0][3] = DHparams[5][3] * math.cos(math.radians(JangleIn[5] + DHparams[5][0]))
        self.J6matrix[1][0] = math.sin(math.radians(JangleIn[5] + DHparams[5][0]))
        self.J6matrix[1][1] = math.cos(math.radians(JangleIn[5] + DHparams[5][0])) * math.cos(math.radians(DHparams[5][1]))
        self.J6matrix[1][2] = -math.cos(math.radians(JangleIn[5] + DHparams[5][0])) * math.sin(math.radians(DHparams[5][1]))
        self.J6matrix[1][3] = DHparams[5][3] * math.sin(math.radians(JangleIn[5] + DHparams[5][0]))
        self.J6matrix[2][0] = 0
        self.J6matrix[2][1] = math.sin(math.radians(DHparams[5][1]))
        self.J6matrix[2][2] = math.cos(math.radians(DHparams[5][1]))
        self.J6matrix[2][3] = DHparams[5][2]
        self.J6matrix[3][0] = 0
        self.J6matrix[3][1] = 0
        self.J6matrix[3][2] = 0
        self.J6matrix[3][3] = 1

        return [self.J1matrix, self.J2matrix, self.J3matrix, self.J4matrix, self.J5matrix, self.J6matrix]
    
# /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
# //CONSTRUCT ROTATION MATRICES FORWARD KINEMATICS
# /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

# void FwdMatrices() {

#   int i, j, k = 0;

#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R02matrix[j][i] = 0;
#     }
#   }
#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R03matrix[j][i] = 0;
#     }
#   }
#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R04matrix[j][i] = 0;
#     }
#   }
#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R05matrix[j][i] = 0;
#     }
#   }
#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R06matrix[j][i] = 0;
#     }
#   }
#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R0Tmatrix[j][i] = 0;
#     }
#   }

#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R02matrix[k][i] = R02matrix[k][i] + (J1matrix [k][j] * J2matrix [j][i]);
#       }
#     }
#   }
#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R03matrix[k][i] = R03matrix[k][i] + (R02matrix [k][j] * J3matrix [j][i]);
#       }
#     }
#   }
#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R04matrix[k][i] = R04matrix[k][i] + (R03matrix [k][j] * J4matrix [j][i]);
#       }
#     }
#   }
#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R05matrix[k][i] = R05matrix[k][i] + (R04matrix [k][j] * J5matrix [j][i]);
#       }
#     }
#   }
#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R06matrix[k][i] = R06matrix[k][i] + (R05matrix [k][j] * J6matrix [j][i]);
#       }
#     }
#   }
#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R0Tmatrix[k][i] = R0Tmatrix[k][i] + (R06matrix [k][j] * toolFrame [j][i]);
#       }
#     }
#   }
# }



# void SolveFowardKinematic() {
#   DH_Matrices();
#   FwdMatrices();
#   xyzuvw_Out[0] = R0Tmatrix[0][3];
#   xyzuvw_Out[1] = R0Tmatrix[1][3];
#   xyzuvw_Out[2] = R0Tmatrix[2][3];
#   xyzuvw_Out[4] = atan2(-R0Tmatrix[2][0], pow((pow(R0Tmatrix[2][1], 2) + pow(R0Tmatrix[2][2], 2)), 0.5));
#   xyzuvw_Out[3] = degrees(atan2(R0Tmatrix[1][0] / cos(xyzuvw_Out[4]), R0Tmatrix[0][0] / cos(xyzuvw_Out[4])));
#   xyzuvw_Out[5] = degrees(atan2(R0Tmatrix[2][1] / cos(xyzuvw_Out[4]), R0Tmatrix[2][2] / cos(xyzuvw_Out[4])));
#   xyzuvw_Out[4] = degrees(xyzuvw_Out[4]);
# }





# /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
# //REVERSE KINEMATICS
# /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

# void SolveInverseKinematic() {

#   float pX;
#   float pY;
#   float pX_a1_fwd;
#   float pX_a1_mid;
#   float pa2H_fwd;
#   float pa2H_mid;
#   float pa3H;
#   float thetaA_fwd;
#   float thetaA_mid;
#   float thetaB_fwd;
#   float thetaB_mid;
#   float thetaC_fwd;
#   float thetaC_mid;
#   float thetaD;
#   float thetaE;

#   float XatJ1zero;
#   float YatJ1zero;
#   float Length_1;
#   float Length_2;
#   float Length_3;
#   float Length_4;
#   float Theta_A;
#   float Theta_B;
#   float Theta_C;
#   float Theta_D;
#   float Theta_E;


#   int i, j, k = 0;
#   KinematicError = 0;

#   //generate matrices for center of spherical wrist location

#   R0T_rev_matrix [0][0] = cos(radians(xyzuvw_In[3])) * cos(radians(xyzuvw_In[4]));
#   R0T_rev_matrix [0][1] = cos(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[4])) * sin(radians(xyzuvw_In[5])) - sin(radians(xyzuvw_In[3])) * cos(radians(xyzuvw_In[5]));
#   R0T_rev_matrix [0][2] = cos(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[4])) * cos(radians(xyzuvw_In[5])) + sin(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[5]));
#   R0T_rev_matrix [0][3] = xyzuvw_In[0];
#   R0T_rev_matrix [1][0] = sin(radians(xyzuvw_In[3])) * cos(radians(xyzuvw_In[4]));
#   R0T_rev_matrix [1][1] = sin(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[4])) * sin(radians(xyzuvw_In[5])) + cos(radians(xyzuvw_In[3])) * cos(radians(xyzuvw_In[5]));
#   R0T_rev_matrix [1][2] = sin(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[4])) * cos(radians(xyzuvw_In[5])) - cos(radians(xyzuvw_In[3])) * sin(radians(xyzuvw_In[5]));
#   R0T_rev_matrix [1][3] = xyzuvw_In[1];
#   R0T_rev_matrix [2][0] = -sin(radians(xyzuvw_In[4]));
#   R0T_rev_matrix [2][1] = cos(radians(xyzuvw_In[4])) * sin(radians(xyzuvw_In[5]));
#   R0T_rev_matrix [2][2] = cos(radians(xyzuvw_In[4])) * cos(radians(xyzuvw_In[5]));
#   R0T_rev_matrix [2][3] = xyzuvw_In[2];
#   R0T_rev_matrix [3][0] = 0;
#   R0T_rev_matrix [3][1] = 0;
#   R0T_rev_matrix [3][2] = 0;
#   R0T_rev_matrix [3][3] = 1;

#   InvtoolFrame [0][0] = toolFrameRev [0][0];
#   InvtoolFrame [0][1] = toolFrameRev [1][0];
#   InvtoolFrame [0][2] = toolFrameRev [2][0];
#   InvtoolFrame [0][3] = (InvtoolFrame [0][0] * toolFrameRev [0][3]) + (InvtoolFrame [0][1] * toolFrameRev [1][3]) + (InvtoolFrame [0][2] * toolFrameRev [2][3]);
#   InvtoolFrame [1][0] = toolFrameRev [0][1];
#   InvtoolFrame [1][1] = toolFrameRev [1][1];
#   InvtoolFrame [1][2] = toolFrameRev [2][1];
#   InvtoolFrame [1][3] = (InvtoolFrame [1][0] * toolFrameRev [0][3]) + (InvtoolFrame [1][1] * toolFrameRev [1][3]) + (InvtoolFrame [1][2] * toolFrameRev [2][3]);
#   InvtoolFrame [2][0] = toolFrameRev [0][2];
#   InvtoolFrame [2][1] = toolFrameRev [1][2];
#   InvtoolFrame [2][2] = toolFrameRev [2][2];
#   InvtoolFrame [2][3] = (InvtoolFrame [2][0] * toolFrameRev [0][3]) + (InvtoolFrame [2][1] * toolFrameRev [1][3]) + (InvtoolFrame [2][2] * toolFrameRev [2][3]);
#   InvtoolFrame [3][0] = 0;
#   InvtoolFrame [3][1] = 0;
#   InvtoolFrame [3][2] = 0;
#   InvtoolFrame [3][3] = 1;

#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R06_rev_matrix[j][i] = 0;
#     }
#   }

#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R05_rev_matrix[j][i] = 0;
#     }
#   }

#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R02matrix_rev[j][i] = 0;
#     }
#   }

#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R03matrix_rev[j][i] = 0;
#     }
#   }

#   for (int j = 0; j < 4; j++) {
#     for (int i = 0; i < 4; i++) {
#       R03_6matrix[j][i] = 0;
#     }
#   }

#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R06_rev_matrix[k][i] = R06_rev_matrix[k][i] + (R0T_rev_matrix [k][j] * InvtoolFrame [j][i]);
#       }
#     }
#   }

#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R05_rev_matrix[k][i] = R05_rev_matrix[k][i] + (R06_rev_matrix [k][j] * R06_neg_matrix [j][i]);
#       }
#     }
#   }

#   //calc J1 angle

#   if (R05_rev_matrix[0][3] >= 0 and R05_rev_matrix[1][3] > 0) {
#     JangleOut[0] = degrees(atan(R05_rev_matrix[1][3] / R05_rev_matrix[0][3]));
#   }
#   else if (R05_rev_matrix[0][3] >= 0 and R05_rev_matrix[1][3] < 0) {
#     JangleOut[0] = degrees(atan(R05_rev_matrix[1][3] / R05_rev_matrix[0][3]));
#   }
#   else if (R05_rev_matrix[0][3] < 0 and R05_rev_matrix[1][3] <= 0) {
#     JangleOut[0] = -180 + degrees(atan(R05_rev_matrix[1][3] / R05_rev_matrix[0][3]));
#   }
#   else if (R05_rev_matrix[0][3] <= 0  and R05_rev_matrix[1][3] > 0) {
#     JangleOut[0] = 180 + degrees(atan(R05_rev_matrix[1][3] / R05_rev_matrix[0][3]));
#   }

#   //calculate J2 & J3 geometry

#   XatJ1zero = (R05_rev_matrix[0][3] * cos(radians(-JangleOut[0]))) - (R05_rev_matrix[1][3] * sin(radians(-JangleOut[0])));
#   YatJ1zero = 0;

#   Length_1 = abs(XatJ1zero - DHparams[0][3]);
#   Length_2 = sqrt(pow((XatJ1zero - DHparams[0][3]), 2) + pow((YatJ1zero - YatJ1zero), 2) + pow((R05_rev_matrix[2][3] - DHparams[0][2]), 2));
#   Length_3 = sqrt(pow(DHparams[3][2], 2) + pow(DHparams[2][3], 2));
#   Length_4 = R05_rev_matrix[2][3] - DHparams[0][2];
#   Theta_B = degrees(atan(Length_1 / Length_4));
#   Theta_C = degrees(acos((pow(DHparams[1][3], 2) + pow(Length_2, 2) - pow(Length_3, 2)) / (2 * DHparams[1][3] * Length_2)));
#   Theta_D = degrees(acos((pow(Length_3, 2) + pow(DHparams[1][3], 2) - pow(Length_2, 2)) / (2 * Length_3 * DHparams[1][3])));
#   Theta_E = degrees(atan(DHparams[2][3] / DHparams[3][2]));

#   // calc J2 angle

#   if (XatJ1zero > DHparams[0][3]) {
#     if (Length_4 > 0) {
#       JangleOut[1] = Theta_B - Theta_C;
#     }
#     else {
#       JangleOut[1] = Theta_B - Theta_C + 180;
#     }
#   }
#   else {
#     JangleOut[1] = -(Theta_B + Theta_C);
#   }

#   // calc J3 angle

#   JangleOut[2] = -(Theta_D + Theta_E) + 90;


#   // generate reverse matrices for wrist orientaion

#   J1matrix_rev [0][0] = cos(radians(JangleOut[0] + DHparams[0][0]));
#   J1matrix_rev [0][1] = -sin(radians(JangleOut[0] + DHparams[0][0])) * cos(radians(DHparams[0][1]));
#   J1matrix_rev [0][2] = sin(radians(JangleOut[0] + DHparams[0][0])) * sin(radians(DHparams[0][1]));
#   J1matrix_rev [0][3] = DHparams[0][3] * cos(radians(JangleOut[0] + DHparams[0][0]));
#   J1matrix_rev [1][0] = sin(radians(JangleOut[0] + DHparams[0][0]));
#   J1matrix_rev [1][1] = cos(radians(JangleOut[0] + DHparams[0][0])) * cos(radians(DHparams[0][1]));
#   J1matrix_rev [1][2] = -cos(radians(JangleOut[0] + DHparams[0][0])) * sin(radians(DHparams[0][1]));
#   J1matrix_rev [1][3] = DHparams[0][3] * sin(radians(JangleOut[0] + DHparams[0][0]));
#   J1matrix_rev [2][0] = 0;
#   J1matrix_rev [2][1] = sin(radians(DHparams[0][1]));
#   J1matrix_rev [2][2] = cos(radians(DHparams[0][1]));
#   J1matrix_rev [2][3] = DHparams[0][2];
#   J1matrix_rev [3][0] = 0;
#   J1matrix_rev [3][1] = 0;
#   J1matrix_rev [3][2] = 0;
#   J1matrix_rev [3][3] = 1;

#   J2matrix_rev [0][0] = cos(radians(JangleOut[1] + DHparams[1][0]));
#   J2matrix_rev [0][1] = -sin(radians(JangleOut[1] + DHparams[1][0])) * cos(radians(DHparams[1][1]));
#   J2matrix_rev [0][2] = sin(radians(JangleOut[1] + DHparams[1][0])) * sin(radians(DHparams[1][1]));
#   J2matrix_rev [0][3] = DHparams[1][3] * cos(radians(JangleOut[1] + DHparams[1][0]));
#   J2matrix_rev [1][0] = sin(radians(JangleOut[1] + DHparams[1][0]));
#   J2matrix_rev [1][1] = cos(radians(JangleOut[1] + DHparams[1][0])) * cos(radians(DHparams[1][1]));
#   J2matrix_rev [1][2] = -cos(radians(JangleOut[1] + DHparams[1][0])) * sin(radians(DHparams[1][1]));
#   J2matrix_rev [1][3] = DHparams[1][3] * sin(radians(JangleOut[1] + DHparams[1][0]));
#   J2matrix_rev [2][0] = 0;
#   J2matrix_rev [2][1] = sin(radians(DHparams[1][1]));
#   J2matrix_rev [2][2] = cos(radians(DHparams[1][1]));
#   J2matrix_rev [2][3] = DHparams[1][2];
#   J2matrix_rev [3][0] = 0;
#   J2matrix_rev [3][1] = 0;
#   J2matrix_rev [3][2] = 0;
#   J2matrix_rev [3][3] = 1;

#   J3matrix_rev [0][0] = cos(radians(JangleOut[2] + DHparams[2][0]));
#   J3matrix_rev [0][1] = -sin(radians(JangleOut[2] + DHparams[2][0])) * cos(radians(DHparams[2][1]));
#   J3matrix_rev [0][2] = sin(radians(JangleOut[2] + DHparams[2][0])) * sin(radians(DHparams[2][1]));
#   J3matrix_rev [0][3] = DHparams[2][3] * cos(radians(JangleOut[2] + DHparams[2][0]));
#   J3matrix_rev [1][0] = sin(radians(JangleOut[2] + DHparams[2][0]));
#   J3matrix_rev [1][1] = cos(radians(JangleOut[2] + DHparams[2][0])) * cos(radians(DHparams[2][1]));
#   J3matrix_rev [1][2] = -cos(radians(JangleOut[2] + DHparams[2][0])) * sin(radians(DHparams[2][1]));
#   J3matrix_rev [1][3] = DHparams[2][3] * sin(radians(JangleOut[2] + DHparams[2][0]));
#   J3matrix_rev [2][0] = 0;
#   J3matrix_rev [2][1] = sin(radians(DHparams[2][1]));
#   J3matrix_rev [2][2] = cos(radians(DHparams[2][1]));
#   J3matrix_rev [2][3] = DHparams[2][2];
#   J3matrix_rev [3][0] = 0;
#   J3matrix_rev [3][1] = 0;
#   J3matrix_rev [3][2] = 0;
#   J3matrix_rev [3][3] = 1;

#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R02matrix_rev[k][i] = R02matrix_rev[k][i] + (J1matrix_rev [k][j] * J2matrix_rev [j][i]);
#       }
#     }
#   }
#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R03matrix_rev[k][i] = R03matrix_rev[k][i] + (R02matrix_rev [k][j] * J3matrix_rev [j][i]);
#       }
#     }
#   }

#   InvR03matrix_rev [0][0] = R03matrix_rev [0][0];
#   InvR03matrix_rev [0][1] = R03matrix_rev [1][0];
#   InvR03matrix_rev [0][2] = R03matrix_rev [2][0];
#   InvR03matrix_rev [1][0] = R03matrix_rev [0][1];
#   InvR03matrix_rev [1][1] = R03matrix_rev [1][1];
#   InvR03matrix_rev [1][2] = R03matrix_rev [2][1];
#   InvR03matrix_rev [2][0] = R03matrix_rev [0][2];
#   InvR03matrix_rev [2][1] = R03matrix_rev [1][2];
#   InvR03matrix_rev [2][2] = R03matrix_rev [2][2];

#   for (int k = 0; k < 4; k++) {
#     for (int i = 0; i < 4; i++) {
#       for (int j = 0; j < 4; j++) {
#         R03_6matrix[k][i] 