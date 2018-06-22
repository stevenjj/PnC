import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 1

# number of state
num_actuated = 1

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    file_path = os.getcwd() + "/../../ExperimentDataCheck/"

    ## read files
    data_time = np.genfromtxt(file_path+'Time.txt', delimiter='\n', dtype=(float))
    data_jeff_des= \
    np.genfromtxt(file_path+'JEffDes.txt', delimiter=None, dtype=(float))
    data_jeff_act= \
    np.genfromtxt(file_path+'JEffAct.txt', delimiter=None, dtype=(float))

    axes = plt.gca()

    ## plot Jeff
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('JTrq')
    for i in range(1, num_actuated+1, 1):
        ax1 = plt.subplot(num_actuated, 1, i)
        if num_actuated == 1:
            plt.plot(data_time, data_jeff_des, "r-")
            plt.plot(data_time, data_jeff_act, "b-")
        else:
            plt.plot(data_time, data_jeff_des[:,i-1], "r-")
            plt.plot(data_time, data_jeff_act[:,i-1], "b-")
        plt.grid(True)
    plt.xlabel('time (sec)')

if __name__ == "__main__":
    create_figures()
    plt.show()
