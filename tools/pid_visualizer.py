import pandas as pd
import argparse
import matplotlib.pyplot as plt


def plot_pid_data(data, name):
    fig, axs = plt.subplots(3, sharex=True)
    axs[0].plot(data["data.time"], data[" data.target_value"],
                color='b', label="target")
    axs[0].plot(data["data.time"], data[" data.measured_value"],
                color='r', label="measured")
    axs[0].legend(loc="upper right")
    axs[1].plot(data["data.time"], data["data.error"],
                color='orange', label="error")
    axs[1].plot(data["data.time"], data["data.integral_error"],
                color='gray', label="integral_error")
    axs[1].legend(loc="upper right")
    axs[2].plot(data["data.time"], data["data.pid_output"],
                color='green', label="pid_output")
    axs[2].legend(loc="upper right")
    fig.suptitle(name + " kd=%f ki=%f kd=%f" % 
                 (data["data.kp"].iloc[0], data["data.ki"].iloc[0], data["data.kd"].iloc[0]))
    plt.savefig(name + '.png')
    #plt.show()
    plt.clf()


parser = argparse.ArgumentParser()
parser.add_argument('--filename', action='store', type=str, required=True)
args = parser.parse_args()
print("reading file: ", args.filename)

data = pd.read_csv(args.filename)

# left pid
left_pid_data = data[data["data.name"] == "pid_left"]

# right pid
right_pid_data = data[data["data.name"] == "pid_right"]

plot_pid_data(left_pid_data, left_pid_data["data.name"].iloc[0] + args.filename )
plot_pid_data(right_pid_data, right_pid_data["data.name"].iloc[0] + args.filename )
