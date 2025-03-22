#!/usr/bin/env python3
"""
Tool Path Planning Module for the Selfie-Drawing Robot
Author: Amrith David
"""

import numpy as np
import matplotlib.pyplot as plt
import cv2
import os

class PathPlanner:
    """tool path planning class with minimal functionality"""
    
    def __init__(self):
        """Initialise the path planner"""
        self.paths = []
        
    def create_sample_image(self):
        """Create a face for demonstration"""
        img = np.zeros((500, 500), dtype=np.uint8)
        # Draw a simple face
        cv2.circle(img, (250, 250), 150, 255, 1)  # Face outline
        cv2.circle(img, (200, 200), 30, 255, 1)   # Left eye
        cv2.circle(img, (300, 200), 30, 255, 1)   # Right eye
        cv2.ellipse(img, (250, 280), (80, 70), 0, 0, 180, 255, 1)  # Smile
        return img
        
    def load_sample_image(self, filename=None):
        """Load a sample image or create one if none provided"""
        if filename and os.path.exists(filename):
            img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
            return img
        else:
            return self.create_sample_image()
        
    def extract_paths_from_image(self, image):
        """Extract basic paths from an image"""
        # Find contours in the image
        contours, _ = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        # Convert contours to simple paths
        self.paths = []
        for contour in contours:
            if len(contour) > 10:  # Skip very small contours
                points = [(point[0][0], point[0][1]) for point in contour]
                self.paths.append(points)
                
        return self.paths
        
    def optimize_paths(self):
        """Placeholder for path optimisation"""

        return self.paths
        
    def generate_waypoints(self):
        """Generate simple waypoints for robot control"""
        if not self.paths:
            return []
            
        waypoints = []
        
        #For each path, create waypoints
        for path in self.paths:
            # Start with pen up
            waypoints.append({
                'x': path[0][0],
                'y': path[0][1],
                'z': 10,  # 10mm above paper
                'pen_down': False
            })
            
            # put pen down and draw the path
            for x, y in path:
                waypoints.append({
                    'x': x,
                    'y': y,
                    'z': 0,  # On the paper
                    'pen_down': True
                })
                
            # Lift pen at the end
            waypoints.append({
                'x': path[-1][0],
                'y': path[-1][1],
                'z': 10,  # 10mm above paper
                'pen_down': False
            })
            
        return waypoints
        
    def visualize_paths(self, show_optimization=False):
        """Create a simple visualisation of the paths"""
        if not self.paths:
            return None
            
        fig, ax = plt.subplots(figsize=(8, 8))
        
        for path in self.paths:
            # Extract x and y coordinates
            x_values = [p[0] for p in path]
            y_values = [p[1] for p in path]
            
            # Plot the path
            ax.plot(x_values, y_values, 'b-', linewidth=1)
            
        ax.set_aspect('equal')
        ax.set_title('Extracted Drawing Paths')
        
        return fig
        
    def run_demo(self, save_dir='./output'):
        """Run a simple demonstration"""
        # Create output directory
        os.makedirs(save_dir, exist_ok=True)
        
        # Create sample image
        print("Creating sample image...")
        image = self.create_sample_image()
        cv2.imwrite(os.path.join(save_dir, "sample_image.png"), image)
        
        #extract paths
        print("Extracting paths...")
        self.extract_paths_from_image(image)
        
        # Visualise paths
        print("Visualising paths...")
        fig = self.visualize_paths()
        if fig:
            fig.savefig(os.path.join(save_dir, "paths.png"))
            plt.close(fig)
            
        #Generate waypoints
        print("Generating waypoints...")
        waypoints = self.generate_waypoints()
        
        # save waypoints to file
        with open(os.path.join(save_dir, "waypoints.txt"), 'w') as f:
            f.write("Robot Drawing Waypoints:\n")
            f.write("----------------------\n\n")
            for i, wp in enumerate(waypoints):
                f.write(f"Point {i}: X={wp['x']}, Y={wp['y']}, Z={wp['z']}, " +
                       f"Pen={'Down' if wp['pen_down'] else 'Up'}\n")
                
        print(f"Demo completed! Output saved to {save_dir}/")
        print(f"Generated {len(waypoints)} waypoints")
        
        return waypoints


if __name__ == "__main__":
    # create planner and run demo
    planner = PathPlanner()
    waypoints = planner.run_demo()
    
    #Show visualisation
    plt.show()
