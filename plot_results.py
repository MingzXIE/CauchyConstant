#!/usr/bin/env python
import numpy
import matplotlib.pyplot as plt

gaussianSolution = numpy.genfromtxt(fname='gaussianSolution.txt',skip_header=1)
cauchySolution = numpy.genfromtxt(fname='cauchySolution.txt',skip_header=1)

plt.figure(1)
gaussianPlot, = plt.plot(gaussianSolution, label='Gaussian')
cauchyPlot, = plt.plot(cauchySolution, label='Cauchy')
plt.legend()
plt.show()
