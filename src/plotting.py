import pandas as pd
import matplotlib.pyplot as plt
import os
import argparse
import sys

################################################################################################
## Functions

def multiPlot(data, nRows, nColumns, x, title=""):
    fig, axs = plt.subplots(nRows, nColumns)
    for i in range(0, data.shape[1]):
        axs[int(i/nColumns)][i%nColumns].plot(x, data.iloc[:, i])
        axs[int(i/nColumns)][i%nColumns].set_title(f"Joint {i+1}")
    
    fig.suptitle(title)
    return fig, axs
        

#################################################################################################
## Argparser

parser = argparse.ArgumentParser(description='Plot in sepparate images the different paths.')
parser.add_argument('pathFile', metavar='p', type=str,
                    help='Path to the pathFile')
parser.add_argument('-o','--outputName', type=str,
                    help="Name of the output image")
parser.add_argument('-t','--title', type=str,
                    help='Title of the image')

args = parser.parse_args()

############################################################
## Read the data
data = pd.read_csv(args.pathFile)

# The image itself
t = data.iloc[:, data.shape[1]-1]
fig, axs = multiPlot(data.iloc[:,:-1], 2, 3, t, title="Path to points")

plt.show()