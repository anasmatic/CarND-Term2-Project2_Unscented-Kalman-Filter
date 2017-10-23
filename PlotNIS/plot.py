# -*- coding: utf-8 -*-
"""
Created on Sun Oct 22 22:11:45 2017

@author: Omnia
"""

import matplotlib.pyplot as plt 

plt.plotfile('NIS_laser.csv', delimiter='\n')
plt.axhline(y=.103,xmin=0,xmax=3,c="green",linewidth=0.5,zorder=0)
plt.axhline(y=5.9,xmin=0,xmax=3,c="red",linewidth=0.5,zorder=0)
plt.show()

plt.plotfile('NIS_rader.csv', delimiter='\n')
plt.axhline(y=.352,xmin=0,xmax=3,c="green",linewidth=0.5,zorder=0)
plt.axhline(y=7.8,xmin=0,xmax=3,c="red",linewidth=0.5,zorder=0)
plt.show()

