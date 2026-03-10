try:
    import opt_mintime_traj.src
    import opt_mintime_traj.powertrain_src
except ImportError:
    pass  # casadi not installed; mintime optimization unavailable
