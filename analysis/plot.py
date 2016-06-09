import matplotlib.pyplot as plt
import numpy as np

debug_file = open("../debug.csv","r")
headers = debug_file.readline()
labels = headers.split()
print labels

plt.plotfile(debug_file, (0, 1, 2),subplots=False,newfig=False)

plt.show()
