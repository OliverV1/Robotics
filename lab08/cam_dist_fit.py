#!/usr/bin/python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt;
import numpy as np;
import scipy.optimize as opt;

# This is the cost function which parameters we are looking for
def blobSizeToDistance(blob_size, a, b):
    ##########################
    dist = a * 1/blob_size + b # <<<-- Define your function here
    ##########################
    return dist

# Define the experimentally measured data
x_data = [34,37,42,50,60,75,80,110]     # Size [px]
y_data = [1800,1600,1400,1200,1000,800,600,400]    # Distance [mm]

# Lower and upper boundaries for each fitted parameter
bnd_lo = [-np.inf, -np.inf]
bnd_up = [np.inf, np.inf]

# Plot the measured data
plt.plot(x_data, y_data, ".", label="measured data");

# The curve fitting happens here
optimized_parameters, pcov = opt.curve_fit(blobSizeToDistance, x_data, y_data, bounds=(bnd_lo, bnd_up));

a = optimized_parameters[0]
b = optimized_parameters[1]

print("Optimized parameters:")
print("  a = " + str(a)) 
print("  b = " + str(b)) 

# Calculate points with the optimized parameters
x_data_fit = np.linspace(min(x_data), max(x_data), 100)
y_data_fit = blobSizeToDistance(x_data_fit, *optimized_parameters)

# Plot the fit
plt.plot(x_data_fit, y_data_fit, label="fited data");

# Show the graph
plt.legend();
plt.xlabel("Blob size (px)")
plt.ylabel("Distance (mm)")
plt.show();
