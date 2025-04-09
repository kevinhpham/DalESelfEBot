#!/usr/bin/env python3
"""
Test script for path planner module.
Author: Amrith David
"""

import os
import sys
import matplotlib.pyplot as plt

# Add the source directory to the path
sys.path.append('src')

# Import the path planner
from tool_path_planning.path_planner import PathPlanner

def main():
    print("Testing Path Planner")
    print("===================")
    
    # create a path planner instance
    planner = PathPlanner()
    
    # Run the demo
    waypoints = planner.run_demo()
    
    print(f"\nTest complete! Generated {len(waypoints)} waypoints.")
    print("Check the 'output' directory for results.")
    
    #Show visualisation if running interactively
    plt.show()

if __name__ == "__main__":
    main()
