
'''

V= sp. sin(theta), sp.cos(theta)
    = sV\hat

ttca=rel_pos.rel_vel/abs(rel_speed)**2

Ct= Cobjectavoidance + Cmovement

Cmovement = 1-0.5*(Calpha+Cs)
Calpha = e**(-0.5*(alpha_g/sigma_alpha_g)**2)
Cs=e**(-0.5*((s_alpha-s_comf)/sigma_s)**2)


Cobjectavoidance
'''

agents=[]
for agent in agents:
    agent.get_state()
    agent.set_camera()
    all_pixels=agent.render_environment()
    for pixel in all_pixels:
        if pixel.is_visible():
            o=pixel.get_obstacle()
            p,v=o.get_motion()
            p_r, v_r= agent.get_relative_motion(p,v)
            ttca, dca= agent.get_closest_approach_metrics(p_r, v_r)
            dc_ds, dc_dt= agent.gradient_pixel_cost(ttca, dca)
            all_pixels.set_space(dc_ds, dc_dt)

    Delta_C0=agent.gradient_obstacle_cost(all_pixels.set_space)
    Delta_CM=agent.gradient_movement_cost(agent.delta_s, agent.alpha_g)
    Delta_C= agent.grad_cost(Delta_C0, Delta_CM)
for agent in agents:
    agent.velocity= agent.calculate_velocity(agent.grad_cost)
