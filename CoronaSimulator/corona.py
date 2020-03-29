# -*- coding: utf-8 -*-
"""
Created on Fri Mar 27 17:57:58 2020

@author: Espen
"""
import matplotlib.pyplot as plt
from Simulate import Simulate


plt.style.use("ggplot")
numParts = 250
dim = 50
initialInfected = 2
steps = 5

sim = Simulate(numParts, 
			   dim, 
			   initialInfected, 
			   radius = 1, 
			   probability = 0.50)
sim.performSteps()
#sim.saveGIF()