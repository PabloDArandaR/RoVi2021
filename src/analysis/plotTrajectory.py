import pandas as pd
import matplotlib.pyplot as plt
import os
import argparse
import sys

################################################################################################
## Functions

def multiPlot(figOriginal, axOriginal, data, nRows, nColumns, x, title=""):
    for i in range(0, data.shape[1]):
        axs[int(i/nColumns)][i%nColumns].plot(x, data[:, i])
        axs[int(i/nColumns)][i%nColumns].set_title(f"Joint {i+1}")
    
    fig.suptitle(title)
    return fig, axs
        
def addPoints(figOriginal, axOriginal, nRow, nCol, points, t):
    print(f"The shape of the input axis is: {axOriginal.shape}")
    for i in range(0, points.shape[1]):
        axOriginal[int(i/nCol)][i%nCol].scatter(t, points[:,i], s=10)
        axOriginal[int(i/nCol)][i%nCol].set_title(f"Joint {i+1}")

    return figOriginal, axOriginal

#################################################################################################
## Argparser

parser = argparse.ArgumentParser(description='Plot in sepparate images the different paths.')
parser.add_argument('pathFile', metavar='p', type=str,
                    help='Path to the pathFile')
parser.add_argument('pointFile', metavar='i', type=str,
                    help='Path to pointFile')
parser.add_argument('-o','--outputName', type=str,
                    help="Name of the output image")
parser.add_argument('-t','--title', type=str,
                    help='Title of the image')

args = parser.parse_args()

############################################################
## Read the data
data = pd.read_csv(args.pathFile)
points = pd.read_csv(args.pointFile)

print(f"The points dataset is:\n {points}")

# The image itself
tPath = data.iloc[:, data.shape[1]-1]
pathPoints = data.iloc[:,:-1].to_numpy()
pointsTime = points.iloc[:,:-1].to_numpy()
tPoints = points.iloc[:, points.shape[1]-1]

plt.rcParams['figure.figsize'] = [13, 9]
fig, axs = plt.subplots(2, 3)
fig, axs = multiPlot(fig, axs, pathPoints, 2, 3, tPath, title="Path to points")

fig, axs = addPoints(fig, axs, 2, 3, pointsTime, tPoints)

plt.savefig("newfig.png", dpi=300)

plt.show()
