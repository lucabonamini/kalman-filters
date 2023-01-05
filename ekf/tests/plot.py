#!/usr/python3

import matplotlib.pyplot as plt
import csv

def plot_results():
    with open('./data.csv','r') as f_input:
        csv_input = csv.reader(f_input, delimiter=',', skipinitialspace=True)
        x_truth= []
        y_truth = []
        x_ekf = []
        y_ekf = []
        for cols in csv_input:
            x_truth.append(float(cols[0]))
            y_truth.append(float(cols[1]))
            x_ekf.append(float(cols[3]))
            y_ekf.append(float(cols[4]))
    plt.plot(x_truth, y_truth, 'r')
    plt.plot(x_ekf, y_ekf, 'b')
    plt.legend(['Truth','Ekf'])
    plt.show()

if __name__ == "__main__":
    plot_results()