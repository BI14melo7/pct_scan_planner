class ConfigPlanner():
    use_quintic = True
    max_heading_rate = 10

    # Keep the hard traversability gate separate from the soft clearance
    # preference.  The A* cost weight is applied to the inflated tomogram cost,
    # so a route uses the middle of free space when one is available instead
    # of following the first admissible cell beside an obstacle.
    a_star_cost_threshold = 20.0
    a_star_cost_weight = 0.2

    # GPMP smooths the grid path and can otherwise cut back across the soft
    # inflation band.  Lower values start penalising the smoothed path farther
    # away from obstacles (tomogram obstacle cost is 50 at the hard boundary).
    trajectory_safe_cost = 15.0


class ConfigWrapper():
    tomo_dir = '/rsc/tomogram/'


class Config():
    planner = ConfigPlanner()
    wrapper = ConfigWrapper()
