import RBF as rbf
import copy
import numpy as np
import random
import re


class PSO:
    J = 12
    xDim = 3
    pLength = 1 + J + J * xDim + J
    F = 0.0
    bestF = 0.0
    bias_min = 0
    bias_max = 1
    w_min = 0
    w_max = 1
    m_min = 0
    m_max = 30
    sigma_min = 1e-6
    sigma_max = 10

    def __init__(self):
        self.myrbf = rbf.RBF(self.J, self.xDim)
        self.X = np.zeros((self.pLength), np.float64)
        self.V = np.zeros((self.pLength), np.float64)
        self.bestX = np.zeros((self.pLength), np.float64)

    def generate(self):
        for i in range(0, 1):
            self.X[i] = random.uniform(self.bias_min, self.bias_max)
            self.bestX[i] = random.uniform(self.bias_min, self.bias_max)
            self.V[i] = 0

        for i in range(1, 1 + self.J):
            self.X[i] = random.uniform(self.w_min, self.w_max)
            self.bestX[i] = random.uniform(self.w_min, self.w_max)
            self.V[i] = 0

        for i in range(1 + self.J, 1 + self.J + self.J * self.xDim):
            self.X[i] = random.uniform(self.m_min, self.m_max)
            self.bestX[i] = random.uniform(self.m_min, self.m_max)
            self.V[i] = 0

        for i in range(
            1 + self.J + self.J * self.xDim, 1 + self.J + self.J * self.xDim + self.J
        ):
            self.X[i] = random.uniform(self.sigma_min, self.sigma_max)
            self.bestX[i] = random.uniform(self.sigma_min, self.sigma_max)
            self.V[i] = 0

    # input x[N][xDim], output[N] 計算適應函數直
    def calculateFitness(self, input, output):
        self.setrbf()
        F = 0.0
        for i in range(0, len(output)):
            F += (output[i] - self.myrbf.calculateOutput(input[i])) ** 2
        F /= 2
        self.F = F
        if self.F < self.bestF:
            self.bestX = copy.deepcopy(self.X)
            self.bestF = self.F

    def setrbf(self):
        for i in range(0, 1):
            self.myrbf.bias = min(max(self.X[i], self.bias_min), self.bias_max)
            self.X[i] = self.myrbf.bias

        j = 0
        for i in range(1, 1 + self.J):
            self.myrbf.w[j] = min(max(self.X[i], self.w_min), self.w_max)
            self.X[i] = self.myrbf.w[j]
            j += 1

        j = 0
        for i in range(1 + self.J, 1 + self.J + self.J * self.xDim):
            self.myrbf.m[int(j / self.xDim)][j % self.xDim] = min(
                max(self.X[i], self.m_min), self.m_max
            )
            self.X[i] = self.myrbf.m[int(j / self.xDim)][j % self.xDim]
            j += 1

        j = 0
        for i in range(
            1 + self.J + self.J * self.xDim, 1 + self.J + self.J * self.xDim + self.J
        ):
            self.myrbf.sigma[j] = min(max(self.X[i], self.sigma_min), self.sigma_max)
            self.X[i] = self.myrbf.sigma[j]
            j += 1

    def setPSO(self, psoStr):
        PSOList = re.findall(r"[\w.+-]+", psoStr)
        for i in range(len(PSOList)):
            self.X[i] = PSOList[i]
        self.setrbf()

    def getPSOList(self):
        PSOList = []
        for i in range(0, self.pLength):
            PSOList.append(self.X[i])
        return PSOList

    def getTheta(self, input):
        ret = self.myrbf.calculateOutput(input)
        ret *= 80
        ret -= 40
        return ret
