#!/usr/bin/env python

import numpy
import matplotlib.pyplot as plt

dataset = numpy.genfromtxt(fname='samples.txt',skip_header=1)

plt.figure(1)
plt.plot(dataset)
plt.show()
