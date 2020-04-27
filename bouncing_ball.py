import numpy as np
#Sources used to make this: 
#https://yyknosekai.wordpress.com/2015/10/21/losing-height-a-bouncing-ball/
#https://www.youtube.com/watch?v=Mp8bz5P1m4I


class BouncingBall():
    def __init__(self, mass, y_threshold):
        self.mass = mass
        self.y_threshold = y_threshold
        self.rest = .82
        pass
    def trajectory(self, x_wall, y_wall, z_wall, v_init, x_rob, z_rob):
        #Perform trajectory calculation until we reach x_rob and z_rob
        #Perform trajectory calculation until
        y_end = y_wall
        x_end = x_wall
        z_end = z_wall
        v_x_init = 0
        v_y_init = 0
        v_z_init = 0
        cnt = 0
        max_h = 0

        while(y_end < self.y_threshold):
            if cnt==0:
                v_y_init = v_init[1]
                v_x_init = v_init[0]
                v_z_init = v_init[2]
                p_init = z_wall
                y_end += v_y_init * ((v_z_init + np.sqrt(v_z_init**2 - 2 * p_init * (9.81))))/(9.81)
                x_end += v_x_init * ((v_z_init + np.sqrt(v_z_init**2 - 2 * p_init * (9.81))))/(9.81)
                cnt += 1
            else:
                v_y_init = v_init[1] * (self.rest)**cnt
                v_x_init = v_init[0] * (self.rest)**cnt
                v_z_init = v_init[2] * (self.rest)**cnt
                y_end += v_y_init * (2 * v_z_init/(9.81))
                x_end += v_x_init * (2 * v_z_init/(9.81))
                cnt += 1
        
        if cnt == 1:
            y_end = y_wall
            x_end = x_wall
        else:
            y_end -= v_y_init * ((2 * v_z_init)/(9.81))
            x_end -= v_x_init * ((2 * v_z_init)/(9.81))
        
        cnt -= 1
        
        delta_y = self.y_threshold - y_end
        target_time = delta_y/v_y_init

        delta_z = v_z_init * target_time - (1/2) * (9.81) * (target_time**2)
        z_end += delta_z






        
        #so we have crossed the threshold: Now we need to see what the height is at that moment

