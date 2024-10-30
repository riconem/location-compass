from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import seaborn as sns

def plot(compass_data, limit=1600):
    # Set the width and height of the figure
    plt.figure(figsize=(16,6))

    # Line chart showing how x,y,z evolved over time 
    sns.lineplot(data=compass_data)


    # 2d
    center = (0,0)
    radius = 0.9 * limit
    lim = -limit, limit

    circle = Circle(center, radius, fill = False, alpha=0.5, color="black")
    fig, axs = plt.subplots(2, 2, figsize=(10, 10))
    axs[0,0].add_patch(circle)
    axs[0,0].set_aspect(1)
    axs[0,1].set_aspect(1)
    axs[1,1].set_aspect(1)
    axs[1,0].set_aspect(1)
    axs[0,0].set_xlim(lim)
    axs[0,0].set_ylim(lim)
    axs[0,1].set_xlim(lim)
    axs[0,1].set_ylim(lim)
    axs[1,1].set_xlim(lim)
    axs[1,1].set_ylim(lim)
    axs[1,0].set_xlim(lim)
    axs[1,0].set_ylim(lim)

    sns.scatterplot(x=compass_data['x'], y=compass_data['y'], alpha=0.5, ax=axs[0,0], color="blue")
    sns.scatterplot(x=compass_data['x'], y=compass_data['z'], alpha=0.5, ax=axs[0,0], color="orange")
    sns.scatterplot(x=compass_data['y'], y=compass_data['z'], alpha=0.5, ax=axs[0,0], color="green")

    sns.scatterplot(x=compass_data['x'], y=compass_data['y'], alpha=0.5, ax=axs[0,1], color="blue")
    sns.scatterplot(x=compass_data['x'], y=compass_data['z'], alpha=0.5, ax=axs[1,0], color="orange")
    sns.scatterplot(x=compass_data['y'], y=compass_data['z'], alpha=0.5, ax=axs[1,1], color="green")

    fig.legend(labels=['Circle', 'xy', 'xz', 'yz'])

    # 3d
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(compass_data['x'], compass_data['y'], compass_data['z'])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')