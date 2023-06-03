import random
import pso as pso
from operator import attrgetter
import copy


class PSOParameter:
    def __init__(self, poolSize=128, maxIteration=30, phi1=0.4, phi2=0.6):
        self.poolSize = poolSize
        self.maxIteration = maxIteration
        self.phi1 = phi1
        self.phi2 = phi2
        self.psoList = []
        self.bestPSO = pso.PSO()
        self.times = 1
        self.Vmax = 30
        self.Vmin = -30

        for i in range(0, poolSize):
            particle = pso.PSO()
            particle.generate()
            self.psoList.append(particle)

    def psoIteration(self, input, output):
        for i in range(0, self.poolSize):
            self.psoList[i].calculateFitness(input, output)
        self.psoList.sort(key=attrgetter("F"))
        self.bestPSO = copy.deepcopy(self.psoList[0])
        for i in range(0, self.maxIteration):
            self.performPSO(input, output)
        return self.bestPSO

    def performPSO(self, input, output):
        self.psoList.sort(key=attrgetter("F"))
        if self.times >= self.maxIteration:
            return self.bestPSO

        for i in range(0, self.poolSize):
            for j in range(0, self.bestPSO.pLength):
                self.psoList[i].V[j] = (
                    self.psoList[i].V[j]
                    + self.phi1 * (self.psoList[i].bestX[j] - self.psoList[i].X[j])
                    + self.phi2 * (self.bestPSO.X[j] - self.psoList[i].X[j])
                )

                if self.psoList[i].V[j] > self.Vmax:
                    self.psoList[i].V[j] = self.Vmax
                elif self.psoList[i].V[j] < self.Vmin:
                    self.psoList[i].V[j] = self.Vmin

                self.psoList[i].X[j] = self.psoList[i].X[j] + self.psoList[i].V[j]
            self.psoList[i].calculateFitness(input, output)
            if self.psoList[i].F < self.bestPSO.F:
                self.bestPSO = copy.deepcopy(self.psoList[i])

        self.times += 1
        return 0
