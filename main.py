"""
The first attempt for understanding the frequency of computational updates given particular scenarios
Alastair Shipman
"""
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

mpl.use('TkAgg')  # or can use 'TkAgg', whatever you have/prefer

DECELERATION_RATE = 4.9  # ms^-2
FRAME_NUMBER_BEFORE_ACTION = 5
DISTANCE_SAFETY_FACTOR = 0.9
SPEED_SAFETY_FACTOR = 0.9
MPH_TO_MS = 0.44704

V_ego_0 = [5*i for i in range(1,12)]  # starting speeds of the ego vehicle in mph
V_actor_n = [5*i for i in range(1,12)]  # ending speeds of the actor vehicle in mph

S_n = [30, 100]  # distance between ego at start and actor at end in metres

FRAME_TIMES = {}


def plot_frame_times(frame_time_dict, title):
    array_to_plot = []
    for key, value in frame_time_dict.items():
        value = [np.nan if v == 0 else v for v in value]
        array_to_plot.append(value)
    a = np.array(array_to_plot)
    a = a.T
    fig, ax = plt.subplots()
    ax.matshow(a)
    ax.set_xticks(range(a.shape[1]))
    ax.set_xticklabels(V_actor_n)
    ax.set_yticks(range(a.shape[0]))
    ax.set_yticklabels(V_ego_0)
    ax.set_xlabel("Actor End Speed")
    ax.set_ylabel("Ego Start Speed")
    ax.set_title(f"{title}")
    # now plot the actual numbers
    for i, y_val in enumerate(range(a.shape[0])):
        for j, x_val in enumerate(range(a.shape[1])):
            spf = a[i, j]
            if np.isnan(spf) or spf == 0:
                continue

            plt.text(x_val, y_val, f"{1 / spf:.0f}", ha='center', va='center')

    plt.show()

    return


def main():
    # start by cycling through the allowable distance
    for dist in S_n:
        # cycle through each actor and ego speed
        safe_distance = DISTANCE_SAFETY_FACTOR * dist
        print(safe_distance)
        for actor_speed in V_actor_n:
            actor_speed_specific_frame_times = []
            for ego_speed in V_ego_0:
                # say that the maximum frame rate is 100Hz?
                seconds_per_frame = 1

                initial_ego_speed = ego_speed * MPH_TO_MS
                end_actor_speed = actor_speed * MPH_TO_MS
                end_ego_speed = SPEED_SAFETY_FACTOR * end_actor_speed  # ms^-1

                while True:

                    reaction_time = FRAME_NUMBER_BEFORE_ACTION * seconds_per_frame  # seconds
                    braking_time = (initial_ego_speed - end_ego_speed) / DECELERATION_RATE  # seconds

                    reaction_distance = reaction_time * initial_ego_speed
                    braking_distance = braking_time * 0.5 * (initial_ego_speed + end_ego_speed)
                    total_distance = reaction_distance + braking_distance


                    if (ego_speed < SPEED_SAFETY_FACTOR * actor_speed) and (total_distance < safe_distance):
                        seconds_per_frame = 1
                        break

                    if total_distance > safe_distance:
                        # congrats, you have crashed. take a faster frame time, and try again
                        seconds_per_frame -= 0.5  # go up in 5 hundredths of a second
                        if seconds_per_frame < 0 :
                            # congrats, you guaranteed to crash
                            seconds_per_frame=0
                            break
                    else:

                        break

                actor_speed_specific_frame_times.append(seconds_per_frame)
            # now store the frame time to avoid a collision for each actor end speed
            FRAME_TIMES[actor_speed] = actor_speed_specific_frame_times
        plot_frame_times(FRAME_TIMES, str(dist))


if __name__ == "__main__":
    main()
