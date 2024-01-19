import math
import numpy as np
import math

ROBOT_nDOFs = 6

class Kinematics:
    def __init__(self):
        
        self.WristConstraint = "F"
        
        """ cartesian coordinates """
        self.xyzuvw_Out = np.zeros(ROBOT_nDOFs)
        self.xyzuvw_In = np.zeros(ROBOT_nDOFs)
        self.xyzuvw_Temp = np.zeros(ROBOT_nDOFs)

        """ joint angles """
        self.JangleOut = np.zeros(ROBOT_nDOFs)
        self.JangleIn = np.zeros(ROBOT_nDOFs)

        self.DHparams = np.array([
            [0, -90, 169.77, 64.2],
            [-90, 0, 0, 305],
            [180, 90, 0, -0.0001],
            [0, -90, 222.63, 0],
            [0, 90, 0, 0],
            [0, 0, 36.25, 0]
        ])

        self.toolFrame = np.zeros((4, 4))
        self.toolFrameRev = np.zeros((4, 4))

        self.R06_neg_matrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -self.DHparams[5][2]],
            [0, 0, 0, 1]
        ])

        self.J1matrix = np.zeros((4, 4))
        self.J2matrix = np.zeros((4, 4))
        self.J3matrix = np.zeros((4, 4))
        self.J4matrix = np.zeros((4, 4))
        self.J5matrix = np.zeros((4, 4))
        self.J6matrix = np.zeros((4, 4))
        
        self.toolFrame = np.zeros((4, 4))
        self.toolFrameRev = np.zeros((4, 4))
        
        
    def Set_Tool_Frame(self, Xval, Yval, Zval, RZval, RYval, RXval):
        self.toolFrame = np.zeros((4, 4))
        self.toolFrameRev = np.zeros((4, 4))
        
        cos_RZ = math.cos(math.radians(RZval))
        sin_RZ = math.sin(math.radians(RZval))
        cos_RY = math.cos(math.radians(RYval))
        sin_RY = math.sin(math.radians(RYval))
        cos_RX = math.cos(math.radians(RXval))
        sin_RX = math.sin(math.radians(RXval))
        
        # calculate forward tool dh-matrix
        self.toolFrame[0][0] = cos_RZ * cos_RY
        self.toolFrame[0][1] = cos_RZ * sin_RY * sin_RX - sin_RZ * cos_RX
        self.toolFrame[0][2] = cos_RZ * sin_RY * cos_RX + sin_RZ * sin_RX
        self.toolFrame[0][3] = Xval
        self.toolFrame[1][0] = sin_RZ * cos_RY
        self.toolFrame[1][1] = sin_RZ * sin_RY * sin_RX + cos_RZ * cos_RX
        self.toolFrame[1][2] = sin_RZ * sin_RY * cos_RX - cos_RZ * sin_RX
        self.toolFrame[1][3] = Yval
        self.toolFrame[2][0] = -sin_RY
        self.toolFrame[2][1] = cos_RY * sin_RX
        self.toolFrame[2][2] = cos_RY * cos_RX
        self.toolFrame[2][3] = Zval
        self.toolFrame[3][0] = 0
        self.toolFrame[3][1] = 0
        self.toolFrame[3][2] = 0
        self.toolFrame[3][3] = 1
        
        # reverse tool dh-matrix
        self.toolFrameRev[0][0] = self.toolFrame[0][0]
        self.toolFrameRev[0][1] = self.toolFrame[0][1]
        self.toolFrameRev[0][2] = self.toolFrame[0][2]
        self.toolFrameRev[0][3] = -Xval
        self.toolFrameRev[1][0] = self.toolFrame[1][0]
        self.toolFrameRev[1][1] = self.toolFrame[1][1]
        self.toolFrameRev[1][2] = self.toolFrame[1][2]
        self.toolFrameRev[1][3] = -Yval
        self.toolFrameRev[2][0] = self.toolFrame[2][0]
        self.toolFrameRev[2][1] = self.toolFrame[2][1]
        self.toolFrameRev[2][2] = self.toolFrame[2][2]
        self.toolFrameRev[2][3] = -Zval
        self.toolFrameRev[3][0] = 0
        self.toolFrameRev[3][1] = 0
        self.toolFrameRev[3][2] = 0
        self.toolFrameRev[3][3] = 1
        
    def calc_dh_matrices(self, JangleIn):
        DHparams = self.DHparams

        for i in range(len(JangleIn)):
            if JangleIn[i] == 0:
                JangleIn[i] = 0.0001
            
        matrices = [self.J1matrix, self.J2matrix, self.J3matrix, self.J4matrix, self.J5matrix, self.J6matrix]

        for i in range(len(matrices)):
            matrices[i][0][0] = math.cos(math.radians(JangleIn[i] + DHparams[i][0]))
            matrices[i][0][1] = -math.sin(math.radians(JangleIn[i] + DHparams[i][0])) * math.cos(math.radians(DHparams[i][1]))
            matrices[i][0][2] = math.sin(math.radians(JangleIn[i] + DHparams[i][0])) * math.sin(math.radians(DHparams[i][1]))
            matrices[i][0][3] = DHparams[i][3] * math.cos(math.radians(JangleIn[i] + DHparams[i][0]))
            matrices[i][1][0] = math.sin(math.radians(JangleIn[i] + DHparams[i][0]))
            matrices[i][1][1] = math.cos(math.radians(JangleIn[i] + DHparams[i][0])) * math.cos(math.radians(DHparams[i][1]))
            matrices[i][1][2] = -math.cos(math.radians(JangleIn[i] + DHparams[i][0])) * math.sin(math.radians(DHparams[i][1]))
            matrices[i][1][3] = DHparams[i][3] * math.sin(math.radians(JangleIn[i] + DHparams[i][0]))
            matrices[i][2][0] = 0
            matrices[i][2][1] = math.sin(math.radians(DHparams[i][1]))
            matrices[i][2][2] = math.cos(math.radians(DHparams[i][1]))
            matrices[i][2][3] = DHparams[i][2]
            matrices[i][3][0] = 0
            matrices[i][3][1] = 0
            matrices[i][3][2] = 0
            matrices[i][3][3] = 1

        # print("=======DH_MATRICES===========")
        # for matrix in matrices:
        #     print(matrix)
        #     print("=========================")
        # print("=========================")

        return matrices
    
    
    def calc_fwd_matrices(self):
        self.R0Tmatrix = self.J1matrix@self.J2matrix@self.J3matrix@self.J4matrix@self.J5matrix@self.J6matrix@self.toolFrame
        return self.R0Tmatrix
    
    
    def SolveFowardKinematic(self, JangleIn):
        self.JangleIn = JangleIn
        
        self.calc_dh_matrices(self.JangleIn)
        self.R0Tmatrix = self.calc_fwd_matrices()
        
        
        self.xyzuvw_Out[0] = self.R0Tmatrix[0][3]
        self.xyzuvw_Out[1] = self.R0Tmatrix[1][3]
        self.xyzuvw_Out[2] = self.R0Tmatrix[2][3]
        self.xyzuvw_Out[4] = math.atan2(-self.R0Tmatrix[2][0], math.pow((math.pow(self.R0Tmatrix[2][1], 2) + math.pow(self.R0Tmatrix[2][2], 2)), 0.5))
        self.xyzuvw_Out[3] = math.degrees(math.atan2(self.R0Tmatrix[1][0] / math.cos(self.xyzuvw_Out[4]), self.R0Tmatrix[0][0] / math.cos(self.xyzuvw_Out[4])))
        self.xyzuvw_Out[5] = math.degrees(math.atan2(self.R0Tmatrix[2][1] / math.cos(self.xyzuvw_Out[4]), self.R0Tmatrix[2][2] / math.cos(self.xyzuvw_Out[4])))
        self.xyzuvw_Out[4] = math.degrees(self.xyzuvw_Out[4])
        
        return self.xyzuvw_Out
    
    
    
    
        
    def SolveInverseKinematic(self, xyzuvw_In):
        self.xyzuvw_In = xyzuvw_In
        XatJ1zero = 0.0
        YatJ1zero = 0.0
        Length_1 = 0.0
        Length_2 = 0.0
        Length_3 = 0.0
        Length_4 = 0.0
        Theta_A = 0.0
        Theta_B = 0.0
        Theta_C = 0.0
        Theta_D = 0.0
        Theta_E = 0.0

        self.R0T_rev_matrix = np.array([
            [math.cos(math.radians(self.xyzuvw_In[3])) * math.cos(math.radians(self.xyzuvw_In[4])), math.cos(math.radians(self.xyzuvw_In[3])) * math.sin(math.radians(self.xyzuvw_In[4])) * math.sin(math.radians(self.xyzuvw_In[5])) - math.sin(math.radians(self.xyzuvw_In[3])) * math.cos(math.radians(self.xyzuvw_In[5])), math.cos(math.radians(self.xyzuvw_In[3])) * math.sin(math.radians(self.xyzuvw_In[4])) * math.cos(math.radians(self.xyzuvw_In[5])) + math.sin(math.radians(self.xyzuvw_In[3])) * math.sin(math.radians(self.xyzuvw_In[5])), self.xyzuvw_In[0]],
            [math.sin(math.radians(self.xyzuvw_In[3])) * math.cos(math.radians(self.xyzuvw_In[4])), math.sin(math.radians(self.xyzuvw_In[3])) * math.sin(math.radians(self.xyzuvw_In[4])) * math.sin(math.radians(self.xyzuvw_In[5])) + math.cos(math.radians(self.xyzuvw_In[3])) * math.cos(math.radians(self.xyzuvw_In[5])), math.sin(math.radians(self.xyzuvw_In[3])) * math.sin(math.radians(self.xyzuvw_In[4])) * math.cos(math.radians(self.xyzuvw_In[5])) - math.cos(math.radians(self.xyzuvw_In[3])) * math.sin(math.radians(self.xyzuvw_In[5])), self.xyzuvw_In[1]],
            [-math.sin(math.radians(self.xyzuvw_In[4])), math.cos(math.radians(self.xyzuvw_In[4])) * math.sin(math.radians(self.xyzuvw_In[5])), math.cos(math.radians(self.xyzuvw_In[4])) * math.cos(math.radians(self.xyzuvw_In[5])), self.xyzuvw_In[2]],
            [0.0, 0.0, 0.0, 1.0]
        ])

        self.InvtoolFrame = np.array([
            [self.toolFrameRev[0][0], self.toolFrameRev[1][0], self.toolFrameRev[2][0], np.dot(self.toolFrameRev[0:3, 0:3], self.toolFrameRev[0:3, 3])[0]],
            [self.toolFrameRev[0][1], self.toolFrameRev[1][1], self.toolFrameRev[2][1], np.dot(self.toolFrameRev[0:3, 0:3], self.toolFrameRev[0:3, 3])[1]],
            [self.toolFrameRev[0][2], self.toolFrameRev[1][2], self.toolFrameRev[2][2], np.dot(self.toolFrameRev[0:3, 0:3], self.toolFrameRev[0:3, 3])[2]],
            [0.0, 0.0, 0.0, 1.0]
        ])
        
        # self.R06_rev_matrix = np.zeros((4, 4))
        # self.R05_rev_matrix = np.zeros((4, 4))
        # self.R02matrix_rev = np.zeros((4, 4))
        # self.R03matrix_rev = np.zeros((4, 4))
        # self.R03_6matrix = np.zeros((4, 4))


        self.R06_rev_matrix = self.R0T_rev_matrix@self.InvtoolFrame
        self.R05_rev_matrix = self.R06_rev_matrix@self.R06_neg_matrix

        # calc J1 angle
        if self.R05_rev_matrix[0][3] >= 0 and self.R05_rev_matrix[1][3] > 0:
            self.JangleOut[0] = math.degrees(math.atan(self.R05_rev_matrix[1][3] / self.R05_rev_matrix[0][3]))
        elif self.R05_rev_matrix[0][3] >= 0 and self.R05_rev_matrix[1][3] < 0:
            self.JangleOut[0] = math.degrees(math.atan(self.R05_rev_matrix[1][3] / self.R05_rev_matrix[0][3]))
        elif self.R05_rev_matrix[0][3] < 0 and self.R05_rev_matrix[1][3] <= 0:
            self.JangleOut[0] = -180 + math.degrees(math.atan(self.R05_rev_matrix[1][3] / self.R05_rev_matrix[0][3]))
        elif self.R05_rev_matrix[0][3] <= 0 and self.R05_rev_matrix[1][3] > 0:
            self.JangleOut[0] = 180 + math.degrees(math.atan(self.R05_rev_matrix[1][3] / self.R05_rev_matrix[0][3]))

        # calculate J2 & J3 geometry
        XatJ1zero = (self.R05_rev_matrix[0][3] * math.cos(math.radians(-self.JangleOut[0]))) - (self.R05_rev_matrix[1][3] * math.sin(math.radians(-self.JangleOut[0])))
        YatJ1zero = 0.0

        Length_1 = abs(XatJ1zero - self.DHparams[0][3])
        Length_2 = math.sqrt(pow((XatJ1zero - self.DHparams[0][3]), 2) + pow((YatJ1zero - YatJ1zero), 2) + pow((self.R05_rev_matrix[2][3] - self.DHparams[0][2]), 2))
        Length_3 = math.sqrt(pow(self.DHparams[3][2], 2) + pow(self.DHparams[2][3], 2))
        Length_4 = self.R05_rev_matrix[2][3] - self.DHparams[0][2]
        Theta_B = math.degrees(math.atan(Length_1 / Length_4))
        Theta_C = math.degrees(math.acos((pow(self.DHparams[1][3], 2) + pow(Length_2, 2) - pow(Length_3, 2)) / (2 * self.DHparams[1][3] * Length_2)))
        Theta_D = math.degrees(math.acos((pow(Length_3, 2) + pow(self.DHparams[1][3], 2) - pow(Length_2, 2)) / (2 * Length_3 * self.DHparams[1][3])))
        Theta_E = math.degrees(math.atan(self.DHparams[2][3] / self.DHparams[3][2]))

        # calc J2 angle
        if XatJ1zero > self.DHparams[0][3]:
            if Length_4 > 0:
                self.JangleOut[1] = Theta_B - Theta_C
            else:
                self.JangleOut[1] = Theta_B - Theta_C + 180
        else:
            self.JangleOut[1] = -(Theta_B + Theta_C)

        # calc J3 angle
        self.JangleOut[2] = -(Theta_D + Theta_E) + 90
        
        J1matrix_rev = np.zeros((4, 4))
        J2matrix_rev = np.zeros((4, 4))
        J3matrix_rev = np.zeros((4, 4))
        InvR03matrix_rev = np.zeros((4, 4))

        J1matrix_rev [0][0] = math.cos(math.radians(self.JangleOut[0] + self.DHparams[0][0]))
        J1matrix_rev [0][1] = -math.sin(math.radians(self.JangleOut[0] + self.DHparams[0][0])) * math.cos(math.radians(self.DHparams[0][1]))
        J1matrix_rev [0][2] = math.sin(math.radians(self.JangleOut[0] + self.DHparams[0][0])) * math.sin(math.radians(self.DHparams[0][1]))
        J1matrix_rev [0][3] = self.DHparams[0][3] * math.cos(math.radians(self.JangleOut[0] + self.DHparams[0][0]))
        J1matrix_rev [1][0] = math.sin(math.radians(self.JangleOut[0] + self.DHparams[0][0]))
        J1matrix_rev [1][1] = math.cos(math.radians(self.JangleOut[0] + self.DHparams[0][0])) * math.cos(math.radians(self.DHparams[0][1]))
        J1matrix_rev [1][2] = -math.cos(math.radians(self.JangleOut[0] + self.DHparams[0][0])) * math.sin(math.radians(self.DHparams[0][1]))
        J1matrix_rev [1][3] = self.DHparams[0][3] * math.sin(math.radians(self.JangleOut[0] + self.DHparams[0][0]))
        J1matrix_rev [2][0] = 0
        J1matrix_rev [2][1] = math.sin(math.radians(self.DHparams[0][1]))
        J1matrix_rev [2][2] = math.cos(math.radians(self.DHparams[0][1]))
        J1matrix_rev [2][3] = self.DHparams[0][2]
        J1matrix_rev [3][0] = 0
        J1matrix_rev [3][1] = 0
        J1matrix_rev [3][2] = 0
        J1matrix_rev [3][3] = 1

        J2matrix_rev [0][0] = math.cos(math.radians(self.JangleOut[1] + self.DHparams[1][0]))
        J2matrix_rev [0][1] = -math.sin(math.radians(self.JangleOut[1] + self.DHparams[1][0])) * math.cos(math.radians(self.DHparams[1][1]))
        J2matrix_rev [0][2] = math.sin(math.radians(self.JangleOut[1] + self.DHparams[1][0])) * math.sin(math.radians(self.DHparams[1][1]))
        J2matrix_rev [0][3] = self.DHparams[1][3] * math.cos(math.radians(self.JangleOut[1] + self.DHparams[1][0]))
        J2matrix_rev [1][0] = math.sin(math.radians(self.JangleOut[1] + self.DHparams[1][0]))
        J2matrix_rev [1][1] = math.cos(math.radians(self.JangleOut[1] + self.DHparams[1][0])) * math.cos(math.radians(self.DHparams[1][1]))
        J2matrix_rev [1][2] = -math.cos(math.radians(self.JangleOut[1] + self.DHparams[1][0])) * math.sin(math.radians(self.DHparams[1][1]))
        J2matrix_rev [1][3] = self.DHparams[1][3] * math.sin(math.radians(self.JangleOut[1] + self.DHparams[1][0]))
        J2matrix_rev [2][0] = 0
        J2matrix_rev [2][1] = math.sin(math.radians(self.DHparams[1][1]))
        J2matrix_rev [2][2] = math.cos(math.radians(self.DHparams[1][1]))
        J2matrix_rev [2][3] = self.DHparams[1][2]
        J2matrix_rev [3][0] = 0
        J2matrix_rev [3][1] = 0
        J2matrix_rev [3][2] = 0
        J2matrix_rev [3][3] = 1

        J3matrix_rev [0][0] = math.cos(math.radians(self.JangleOut[2] + self.DHparams[2][0]))
        J3matrix_rev [0][1] = -math.sin(math.radians(self.JangleOut[2] + self.DHparams[2][0])) * math.cos(math.radians(self.DHparams[2][1]))
        J3matrix_rev [0][2] = math.sin(math.radians(self.JangleOut[2] + self.DHparams[2][0])) * math.sin(math.radians(self.DHparams[2][1]))
        J3matrix_rev [0][3] = self.DHparams[2][3] * math.cos(math.radians(self.JangleOut[2] + self.DHparams[2][0]))
        J3matrix_rev [1][0] = math.sin(math.radians(self.JangleOut[2] + self.DHparams[2][0]))
        J3matrix_rev [1][1] = math.cos(math.radians(self.JangleOut[2] + self.DHparams[2][0])) * math.cos(math.radians(self.DHparams[2][1]))
        J3matrix_rev [1][2] = -math.cos(math.radians(self.JangleOut[2] + self.DHparams[2][0])) * math.sin(math.radians(self.DHparams[2][1]))
        J3matrix_rev [1][3] = self.DHparams[2][3] * math.sin(math.radians(self.JangleOut[2] + self.DHparams[2][0]))
        J3matrix_rev [2][0] = 0
        J3matrix_rev [2][1] = math.sin(math.radians(self.DHparams[2][1]))
        J3matrix_rev [2][2] = math.cos(math.radians(self.DHparams[2][1]))
        J3matrix_rev [2][3] = self.DHparams[2][2]
        J3matrix_rev [3][0] = 0
        J3matrix_rev [3][1] = 0
        J3matrix_rev [3][2] = 0
        J3matrix_rev [3][3] = 1



        self.R02matrix_rev = J1matrix_rev@J2matrix_rev
        self.R03matrix_rev = self.R02matrix_rev@ J3matrix_rev


        InvR03matrix_rev = self.R03matrix_rev.T
        self.R03_6matrix = InvR03matrix_rev@ self.R05_rev_matrix

        # calculate J5 angle
        if self.WristConstraint == "F":
            self.JangleOut[4] = math.degrees(math.atan2(math.sqrt(1 - pow(self.R03_6matrix[2][2], 2)), self.R03_6matrix[2][2]))
        else:
            self.JangleOut[4] = math.degrees(math.atan2(-math.sqrt(1 - pow(self.R03_6matrix[2][2], 2)), self.R03_6matrix[2][2]))

        # calculate J4 angle
        if self.JangleOut[4] < 0:
            self.JangleOut[3] = -math.degrees(math.atan2(self.R03_6matrix[1][2], -self.R03_6matrix[0][2]))
        else:
            self.JangleOut[3] = -math.degrees(math.atan2(-self.R03_6matrix[1][2], self.R03_6matrix[0][2]))

        # calculate J6 angle
        if self.JangleOut[4] < 0:
            self.JangleOut[5] = math.degrees(math.atan2(-self.R03_6matrix[2][1], self.R03_6matrix[2][0]))
        else:
            self.JangleOut[5] = math.degrees(math.atan2(self.R03_6matrix[2][1], -self.R03_6matrix[2][0]))
        
        return self.JangleOut



if __name__ == "__main__":
    kinematics = Kinematics()
    
    # joint = [10, 20, -10, -30, 45, 29]
    # joint = [0, 0, 0.018, 0, 59.1, 0.045]
    # joint = [2.048, -0.936, 4.346, -1.638, 45.379, 2.835]
    joint = [0, 0, 0, 0, 0, 0]
    kinematics.Set_Tool_Frame(0, 0, 0, 0, 0, 0)
    
    print("og joint", joint)    
    xyzuvw = kinematics.SolveFowardKinematic(joint)
    print("xyzuvw", [int(j) for j in xyzuvw])
    
    
    
    xyzuvw = [323, 0, 474, 0, 90, 0]
    joints = kinematics.SolveInverseKinematic(xyzuvw)
    print("inverse", [int(j) for j in joints])
    
