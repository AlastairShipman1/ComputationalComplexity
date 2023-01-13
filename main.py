"""
The first attempt for understanding the frequency of computational updates given particular scenarios
Alastair Shipman
"""
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

mpl.use('TkAgg')  # or can use 'TkAgg', whatever you have/prefer

MPH_TO_MS = 0.44704



def plot_frame_times(frame_time_dict, xlabels, ylabels, title):
    array_to_plot = []
    for key, value in frame_time_dict.items():
        value = [np.nan if v == 0 else v for v in value]
        array_to_plot.append(value)
    a = np.array(array_to_plot)
    a = a.T
    fig, ax = plt.subplots()
    ax.matshow(a)
    ax.set_xticks(range(a.shape[1]))
    ax.set_xticklabels(xlabels)
    ax.set_yticks(range(a.shape[0]))
    ax.set_yticklabels(ylabels)
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



def simple_test_case():
    # start by cycling through the allowable distance

    deceleration_rate = 4.9  # ms^-2
    frame_number_before_action = 5
    distance_safety_factor = 0.9
    speed_safety_factor = 0.9
    mph_to_ms = 0.44704
    maximum_frame_time = 1  # in seconds

    frame_times = {}

    v_ego_0 = [5 * i for i in range(1, 10)]  # starting speeds of the ego vehicle in mph
    v_actor_n = [5 * i for i in range(1, 10)]  # ending speeds of the actor vehicle in mph

    s_n = [30, 100]  # distance between ego at start and actor at end in metres

    for dist in s_n:
        # cycle through each actor and ego speed
        safe_distance = distance_safety_factor * dist

        for actor_speed in v_actor_n:
            actor_speed_specific_frame_times = []
            for ego_speed in v_ego_0:
                seconds_per_frame = maximum_frame_time

                initial_ego_speed = ego_speed * mph_to_ms
                end_actor_speed = actor_speed * mph_to_ms
                end_ego_speed = speed_safety_factor * end_actor_speed  # ms^-1

                while True:

                    reaction_time = frame_number_before_action * seconds_per_frame  # seconds
                    braking_time = (initial_ego_speed - end_ego_speed) / deceleration_rate  # seconds

                    reaction_distance = reaction_time * initial_ego_speed
                    braking_distance = braking_time * 0.5 * (initial_ego_speed + end_ego_speed)
                    total_distance = reaction_distance + braking_distance


                    if (ego_speed < speed_safety_factor * actor_speed) and (total_distance < safe_distance):
                        seconds_per_frame = 1
                        break

                    if total_distance > safe_distance:
                        # congrats, you have crashed. take a faster frame time, and try again
                        seconds_per_frame -= 0.01  # go up in 5 hundredths of a second
                        if seconds_per_frame < 0 :
                            # congrats, you guaranteed to crash
                            seconds_per_frame=0
                            break
                    else:

                        break

                actor_speed_specific_frame_times.append(seconds_per_frame)
            # now store the frame time to avoid a collision for each actor end speed
            frame_times[actor_speed] = actor_speed_specific_frame_times
        plot_frame_times(frame_times, v_actor_n, v_ego_0 , dist)


def main():
    simple_test_case()

if __name__ == "__main__":
    main()
