#!/bin/python3

import matplotlib.pyplot as plt
import numpy as np
import csv
from scipy import linalg, optimize
import math
from scipy.io import wavfile

def read_csv(path):
    xs = np.empty([0])
    ys = np.empty([0])
    zs = np.empty([0])

    names = ["X", "Y", "Z"]

    with open("".join(["../rflight_drone/logs/", path])) as csv_file:
        csv_reader = csv.DictReader(csv_file, fieldnames=names)
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                # print(f'Column names are {", ".join(row)}')
                pass
            # elif line_count >= 100:
            #     break
            else:
                try:
                    x = float(row["X"])
                    y = float(row["Y"])
                    z = float(row["Z"])
                    xs = np.append(xs, float(row["X"]))
                    ys = np.append(ys, y)
                    zs = np.append(zs, z)
                except:
                    pass

                # xs.append(row["X"])
                # ys.append(row["Y"])
                # zs.append(row["Z"])
            line_count += 1

    return (xs, ys, zs)

def plot_spectrogram(path, ax0, ax1):
    (xs, ys, zs) = read_csv(path)

    # plt.subplot(211)
    # plt.subplot(p0)
    # ax = plt.gca()
    ax0.axes.xaxis.set_visible(False)
    ax0.axes.yaxis.set_visible(False)

    ax0.set_title(f"Spectrogram of gyro X-axis, {path}")
    ax0.plot(xs)
    ax0.set_xlabel('Sample')
    ax0.set_ylabel('Amplitude')

    samplingFrequency = 3330

    # nfft = 128
    nfft = 256
    # nfft = 512

    # plt.subplot(212)
    # plt.subplot(p1)

    # # ax1.set_yscale('log')

    spec = xs

    powerSpectrum, freqenciesFound, time, imageAxis = ax1.specgram(
        spec, Fs=samplingFrequency, NFFT=nfft
    )
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Frequency')
    ax1.set_ylim([0, 400])

    # ax1.hist(xs, bins='auto')
    # ax1.set_ylabel('Frequency')

    # plt.show()

def plot_wav(path):
    samplingFrequency, signalData = wavfile.read(path)

    plt.subplot(211)

    plt.title('Spectrogram of a wav file with piano music')

    plt.plot(signalData)
    plt.xlabel('Sample')
    plt.ylabel('Amplitude')

    plt.subplot(212)
    plt.specgram(signalData,Fs=samplingFrequency)
    plt.xlabel('Time')
    plt.ylabel('Frequency')

    ax = plt.gca()
    ax.set_ylim([0, 2000])

    plt.show()

# plot_wav("test01.wav")
# plot_wav("test02.wav")
# plot_wav("test03.wav")

def plot_allen(path):
    (gx, gy, gz) = read_csv(path)

    sample_rate = 3330 # Hz
    ts = 1 / sample_rate

    thetax = np.cumsum(gx) * ts
    thetay = np.cumsum(gy) * ts
    thetaz = np.cumsum(gz) * ts

    (taux, adx) = AllanDeviation(thetax, sample_rate, maxNumM=200)
    (tauy, ady) = AllanDeviation(thetay, sample_rate, maxNumM=200)
    (tauz, adz) = AllanDeviation(thetaz, sample_rate, maxNumM=200)

    plt.figure()
    plt.title('Gyro Allan Deviations')
    plt.plot(taux, adx, label='gx')
    plt.plot(tauy, ady, label='gy')
    plt.plot(tauz, adz, label='gz')
    plt.xlabel(r'$\tau$ [sec]')
    plt.ylabel('Deviation [deg/sec]')
    plt.grid(True, which="both", ls="-", color='0.65')
    plt.legend()
    plt.xscale('log')
    plt.yscale('log')
    plt.show()

def AllanDeviation(dataArr: np.ndarray, fs: float, maxNumM: int=100):
    """Compute the Allan deviation (sigma) of time-series data.

    Algorithm obtained from Mathworks:
    https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html

    Args
    ----
        dataArr: 1D data array
        fs: Data sample frequency in Hz
        maxNumM: Number of output points

    Returns
    -------
        (taus, allanDev): Tuple of results
        taus (numpy.ndarray): Array of tau values
        allanDev (numpy.ndarray): Array of computed Allan deviations
    """
    ts = 1.0 / fs
    N = len(dataArr)
    Mmax = 2**np.floor(np.log2(N / 2))
    M = np.logspace(np.log10(1), np.log10(Mmax), num=maxNumM)
    M = np.ceil(M)  # Round up to integer
    M = np.unique(M)  # Remove duplicates
    taus = M * ts  # Compute 'cluster durations' tau

    # Compute Allan variance
    allanVar = np.zeros(len(M))
    for i, mi in enumerate(M):
        twoMi = int(2 * mi)
        mi = int(mi)
        allanVar[i] = np.sum(
            (dataArr[twoMi:N] - (2.0 * dataArr[mi:N-mi]) + dataArr[0:N-twoMi])**2
        )

    allanVar /= (2.0 * taus**2) * (N - (2.0 * M))
    return (taus, np.sqrt(allanVar))  # Return deviation (dev = sqrt(var))

def main2():

    # path = "gyro.log"
    path = "gyro_post.log"
    # path = "gyro_iir.log"
    # path = "gyro_no_iir.log"
    # path = "gyro_builtin_filter_on.log"
    # path = "gyro_builtin_filter_off.log"
    # path = "gyro_iir_0.5.log"
    # path = "gyro_iir_1.0.log"
    # path = "gyro_iir_1.5.log"
    # path = "gyro_iir_2.0.log"

    # path = "gyro_iir_st.log"
    # path = "gyro_iir_biquad.log"
    # path = "gyro_iir_biquad_notched.log"

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


def main():

    # path = "gyro_post.log"
    # path = "gyro09_nofilters.log"
    # path = "gyro10_lp_3notch.log"

    path = "throttle_0.125.log"
    # path = "throttle_0.10.log"
    # path = "throttle_0.15.log"
    # path = "throttle_0.20.log"
    # path = "throttle_0.25.log"
    # path = "throttle_0.3.log"

    fig, axs = plt.subplots(2)
    plot_spectrogram(path, axs[0], axs[1])

    # fig, axs = plt.subplots(2, 2)
    # plot_spectrogram("gyro.log", axs[0, 0], axs[1, 0])
    # plot_spectrogram("gyro_post.log", axs[0, 1], axs[1, 1])

    plt.show()

    # path = "gyro04_grounded.log"
    # path = "gyro05_off.log"
    # plot_allen(path)

main()

