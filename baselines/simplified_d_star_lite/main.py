from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM 

OBSTACLE = 255
UNOCCUPIED = 0

if __name__ == '__main__':

    """
    set initial values for the map occupancy grid
    |----------> y, column
    |           (x=0,y=2)
    |
    V (x=2, y=0)
    x, row
    """
    x_dim = 100
    y_dim = 80
    start = (10, 10)
    goal = (39, 70)
    view_range = 5

    map  =  OccupancyGridMap(x_dim=x_dim,
                                y_dim=y_dim,
                                exploration_setting='4N')
    
    print(f'occupancy grid: {map}')

    new_position = start
    last_position = start

    # D* Lite (optimized)
    print(f'new map for d* lite looks like: {map.occupancy_grid_map}')
    dstar = DStarLite(map=map,
                      s_start=start,
                      s_goal=goal)
    
    # SLAM to detect vertices
    slam = SLAM(map=map,
                view_range=view_range)
    

    # move and compute path
    path, g, rhs = dstar.move_and_replan(robot_position=new_position)

    print(f'outputted path: {path}')

    while not new_position == goal: # right now this isnt being updated, but it should be
        # update the map 

        new_observation = {"pos": (2,2), "type": 255} # 255 for obstacle, 0 for not
       
       
        # example: update position of obstacles 
        dstar.sensed_map.set_obstacle(pos=new_observation["pos"])

        # example: remove obstacle from a certain spot 
        dstar.sensed_map.remove_obstacle(pos=new_observation["pos"])


        if new_position != last_position:
            last_position = new_position
            
            # slam
            new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map

            # d star
            path, g, rhs = dstar.move_and_replan(robot_position=new_position)
        
