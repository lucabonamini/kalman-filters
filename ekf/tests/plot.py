import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

matplotlib.use('TkAgg')

# Load data from CSV
data = pd.read_csv('/workspaces/kalman-filters/ekf/tests/data.csv')

# Create the figure and labels
fig, ax = plt.subplots()

ax.scatter([], [], label='True Position', color='green')
ax.scatter([], [], label='EKF Estimate', color='red')
ax.legend()
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_title('True Position vs EKF Estimate')

# Function to update data for each frame
def update(frame):
    ax.scatter(data['x_truth_x'][:frame], data['x_truth_y'][:frame], label='True Position', color='green')
    ax.scatter(data['x_ekf_x'][:frame], data['x_ekf_y'][:frame], label='EKF Estimate', color='red')

# Create the animation
anim = animation.FuncAnimation(fig, update, frames=len(data), interval=100)

# Save the animation
anim.save('ekf.gif', writer='imagemagick', fps=15)

plt.show()
