import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


####################################################
# Reading the file and setup variables

df = pd.read_csv("results/rrt.csv")
#dfdep = df.loc[:, df.columns != 'extend']

####################################################

corr = df.corr()
print(f"Correlation matrix is:\n{corr}")