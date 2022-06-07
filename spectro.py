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

    # path = "gyro.log"
    # path = "gyro_iir.log"
    # path = "gyro_no_iir.log"
    # path = "gyro_builtin_filter_on.log"
    # path = "gyro_builtin_filter_off.log"
    # path = "gyro_iir_0.5.log"
    # path = "gyro_iir_1.0.log"
    # path = "gyro_iir_1.5.log"
    # path = "gyro_iir_2.0.log"

    # path = "gyro_iir_st.log"
    path = "gyro_iir_biquad.log"

    with open("".join(["logs/", path])) as csv_file:
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

    # fig, axs = plt.subplots(4, 2)
    # axs[0, 1].axes.xaxis.set_visible(False)
    # axs[0, 3].axes.xaxis.set_visible(False)
    # axs[1, 1].axes.xaxis.set_visible(False)
    # axs[1, 3].axes.xaxis.set_visible(False)

    plt.subplot(211)
    ax = plt.gca()
    ax.axes.xaxis.set_visible(False)
    ax.axes.yaxis.set_visible(False)

    plt.title(f"Spectrogram of gyro X-axis, {path}")
    plt.plot(xs)
    plt.xlabel('Sample')
    plt.ylabel('Amplitude')

    samplingFrequency = 3330

    # nfft = 128
    nfft = 256
    # nfft = 512

    plt.subplot(212)
    powerSpectrum, freqenciesFound, time, imageAxis = plt.specgram(xs, Fs=samplingFrequency, NFFT=nfft)
    plt.xlabel('Time')
    plt.ylabel('Frequency')

    plt.show()


main()

