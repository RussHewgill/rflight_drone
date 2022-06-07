#!/bin/python3

import matplotlib.pyplot as plt
import numpy as np
import csv
import math

def main():

    xs = np.empty([0])
    ys = np.empty([0])
    zs = np.empty([0])

    # xs = []
    # ys = []
    # zs = []

    names = ["X","Y","Z"]

    with open('gyro.log') as csv_file:
        csv_reader = csv.DictReader(csv_file, fieldnames=names)
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print(f'Column names are {", ".join(row)}')
            # elif line_count >= 100:
            #     break
            else:
                xs = np.append(xs, float(row["X"]))
                ys = np.append(ys, float(row["Y"]))
                zs = np.append(zs, float(row["Z"]))
                # xs.append(row["X"])
                # ys.append(row["Y"])
                # zs.append(row["Z"])
            line_count += 1

    # xs = np.array(xs)
    # ys = np.array(ys)
    # zs = np.array(zs)

    plt.subplot(211)
    ax = plt.gca()
    ax.axes.xaxis.set_visible(False)
    ax.axes.yaxis.set_visible(False)

    plt.title('Spectrogram of a wav file with piano music')
    plt.plot(xs)
    plt.xlabel('Sample')
    plt.ylabel('Amplitude')

    samplingFrequency = 400

    plt.subplot(212)
    # ax = plt.gca()
    # ax.axes.xaxis.set_visible(False)
    # ax.axes.yaxis.set_visible(False)

    powerSpectrum, freqenciesFound, time, imageAxis = plt.specgram(xs, Fs=samplingFrequency)
    plt.xlabel('Time')
    plt.ylabel('Frequency')

    plt.show()


main()

