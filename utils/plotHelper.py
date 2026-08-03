def plotRobotAndFrame(robot,q,frame,plt):
    # Plot the robot using the teach command
    robot.plot(q, block=False, backend="pyplot3")

    # Get the current axis of the plot
    ax = plt.gca()

    # Extract the rotation matrix and translation vector
    R = frame.R
    t = frame.t

    # Define the frame size for visualization
    frame_size = 0.1

    # Plot the x-axis of the frame
    ax.quiver(t[0], t[1], t[2], frame_size * R[0, 0], frame_size * R[1, 0], frame_size * R[2, 0], color='r')
    # Plot the y-axis of the frame
    ax.quiver(t[0], t[1], t[2], frame_size * R[0, 1], frame_size * R[1, 1], frame_size * R[2, 1], color='g')
    # Plot the z-axis of the frame
    ax.quiver(t[0], t[1], t[2], frame_size * R[0, 2], frame_size * R[1, 2], frame_size * R[2, 2], color='b')

    # Customize plot limits if necessary
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_title("Robot and target")

    # Show the updated plot with the arbitrary frame
    plt.show()
    plt.pause(600)
    pass


def conditional_teach(robot,q,enabled):
    if (enabled):
        robot.teach(q, backend="pyplot3")