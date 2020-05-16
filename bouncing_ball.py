import numpy as np
#Sources used to make this: 
#https://yyknosekai.wordpress.com/2015/10/21/losing-height-a-bouncing-ball/
#https://www.youtube.com/watch?v=Mp8bz5P1m4I


class BouncingBall():
    def __init__(self, mass=0, y_threshold=2.65):
        self.mass = mass
        self.y_threshold = y_threshold
        self.rest = .82
        pass
    def trajectory(self, x_wall, y_wall, z_wall, v_init):
        #Perform trajectory calculation until we reach x_rob and z_rob
        #Perform trajectory calculation until
        #So, trajectory generation takes literally forever:
        #How can we come up with a faster way to predict trajectory?
        #Also, how do we figure out the motions?
        y_end = y_wall
        x_end = x_wall
        z_end = z_wall
        y_prev = 0
        x_prev = 0
        v_x_init = 0
        v_y_init = 0
        v_z_init = 0
        cnt = 0

        #if(np.round(v_init[2], 2) == 0):
        time_bounce = (2.65-y_wall)/v_init[1]
        time_bounce_near = (2.85-y_wall)/v_init[1]
        time_bounce_far = (2.45-y_wall)/v_init[1]
        x_end += v_init[0] * time_bounce
        x_world = -1 * x_end + 0.025
        if (np.abs(x_world) < 0.20):
            x_end += v_init[0] * (time_bounce_near - time_bounce)
            return [x_end, 2.85, z_wall]
        elif (np.abs(x_world) > 0.75):
            x_end += v_init[0] * (time_bounce_far - time_bounce)
            return [x_end, 2.45, z_wall-0.1]
        return [x_end, 2.65, z_wall]
        
        # while(y_end < self.y_threshold):
        #     if cnt==0:
        #         v_y_init = v_init[1]
        #         v_x_init = v_init[0]
        #         v_z_init = v_init[2]
        #         p_init = z_wall
        #         y_end += v_y_init * ((v_z_init + np.sqrt(v_z_init**2 + 2 * p_init * (9.81))))/(9.81)
        #         x_end += v_x_init * ((v_z_init + np.sqrt(v_z_init**2 + 2 * p_init * (9.81))))/(9.81)
        #         # print(y_end)
        #         # print(x_end)
        #         cnt += 1
        #     else:
        #         v_y_init = v_init[1] * (self.rest)**cnt
        #         v_x_init = v_init[0] * (self.rest)**cnt
        #         v_z_init = v_init[2] * (self.rest)**cnt
        #         y_end += v_y_init * (2 * v_z_init/(9.81))
        #         x_end += v_x_init * (2 * v_z_init/(9.81))
        #         # print(y_end)
        #         # print(x_end)
        #         cnt += 1
        
        # if cnt == 1:
        #     y_prev = y_wall
        #     x_prev = x_wall
        # else:
        #     y_prev = y_end - v_y_init * ((2 * v_z_init)/(9.81))
        #     x_prev = y_end - v_x_init * ((2 * v_z_init)/(9.81))
        
        # cnt -= 1
        
        # delta_y = self.y_threshold - y_end
        # target_time = 0
        # if v_y_init == 0:
        #     target_time = delta_y
        # else:
        #     target_time = delta_y/v_y_init

        # delta_z = (v_z_init * target_time) - (1/2) * (9.81) * (target_time**2)
        # z_end = delta_z

        # y_pos = y_prev + v_y_init * (target_time)
        # x_pos = x_prev + v_x_init * (target_time)

        # return [x_pos, y_pos, delta_z]
    
    def one_bounce(self, pos_init, v_init):
        pass
        #we look at the amount traveled after one bounce. if x y velocity almost zero, we just do linear from that





        
        #so we have crossed the threshold: Now we need to see what the height is at that moment

