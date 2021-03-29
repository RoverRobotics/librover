import pandas as pd
import argparse
import matplotlib.pyplot as plt


def plot_pid_data(data, name, motion_data):
    fig, axs = plt.subplots(6, sharex=True)
    # target value and measured value
    axs[0].plot(data["time"], data["col0"],
                color='b', label="target")

    axs[0].plot(data["time"], data["col1"],
                color='r', label="measured")
    axs[0].legend(loc="upper right")

    # errors
    axs[1].plot(data["time"], data["col3"],
                color='orange', label="error")
    axs[1].plot(data["time"], data["col4"],
                color='gray', label="integral_error")
    axs[1].plot(data["time"], data["col5"],
                color='tan', label="delta_error")
    axs[1].legend(loc="upper right")

    # output
    axs[2].plot(data["time"], data["col2"],
                color='green', label="pid_output")
    axs[2].legend(loc="upper right")

    fig.suptitle(name + " kp=%f ki=%f kd=%f" %
                 (data["col6"].iloc[0], data["col7"].iloc[0], data["col8"].iloc[0]))

    # motion (motor rpms)
    axs[3].plot(motion_data["time"], motion_data["col4"],
                color='lime', label="front left rpm")
    axs[3].plot(motion_data["time"], motion_data["col6"],
                color='darkgreen', label="rear left rpm")
    axs[3].legend(loc="upper right")


    axs[4].plot(motion_data["time"], motion_data["col5"],
                color='royalblue', label="front right rpm")
    axs[4].plot(motion_data["time"], motion_data["col7"],
                color='darkviolet', label="rear right rpm")
    axs[4].legend(loc="upper right")

    axs[5].plot(motion_data["time"], motion_data["col2"],
                color='sandybrown', label="est linear vel")
    axs[5].plot(motion_data["time"], motion_data["col3"],
                color='darkviolet', label="est angular vel")
    axs[5].legend(loc="upper right")
    

    plt.savefig(name + '.png', dpi=1000)
    # plt.show()
    plt.clf()


parser = argparse.ArgumentParser()
parser.add_argument('--filename', action='store', type=str, required=True)
args = parser.parse_args()
print("reading file: ", args.filename)

data = pd.read_csv(args.filename)

# left pid
left_pid_data = data[data["name"] == "pid_left"]

# right pid
right_pid_data = data[data["name"] == "pid_right"]

# motion data
motion_data = data[data["type"] == "motion"]

plot_pid_data(
    left_pid_data, 
    left_pid_data["name"].iloc[0] + args.filename.split("/")[-1],
    motion_data
)

plot_pid_data(
    right_pid_data,
    right_pid_data["name"].iloc[0] + args.filename.split("/")[-1],
    motion_data
)
