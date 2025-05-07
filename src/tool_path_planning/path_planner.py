#!/usr/bin/env python3
"""
Path Planner module for the Selfie-Drawing Robot
Author: Amrith David
"""
import cv2 # for image processing
import numpy as np # for numerical operations

class PathPlanner: # handles path planning functionality
    """Tool path planning class"""
    
    def __init__(self): # method initialises class with empty paths list
        """Initialize the path planner"""
        self.paths = []
        self.original_image = None
    
    def extract_paths_from_image(self, image): # processes edge images to extract vector paths
        """Extract vector paths from an edge image"""
        # Store the original image for reference
        self.original_image = image.copy()
        
        # Make sure we're working with a proper binary image
        _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
        
        # Debug: Print image properties
        print(f"Image shape: {binary_image.shape}")
        print(f"Image type: {binary_image.dtype}")
        print(f"Image min/max values: {np.min(binary_image)}/{np.max(binary_image)}")
        
        # using opencv's findContours to detect continuous edges in the image
        contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        print(f"Number of raw contours found: {len(contours)}")
        
        # Convert contours to simple paths
        self.paths = [] # resets path list
        
        for contour in contours: # loops through all contours found
            # Skip very small contours (less than 10 points)
            if len(contour) < 10:
                continue
                
            # Simplify the contour using Douglas-Peucker algorithm
            # This reduces the number of points while preserving the shape
            epsilon = 0.002 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Only keep contours with enough points after simplification
            if len(approx) < 3:
                continue
                
            points = []
            for point in approx: # each contour is stored as a 3D array (nested list)
                x = point[0][0] # extracts x coordinate
                y = point[0][1] # extracts y coordinate
                points.append((x,y)) # adding the (x,y) tuple to the points list
            
            self.paths.append(points) # add all the points of the contour that's stored in points list as an element
        
        print(f"Number of filtered paths: {len(self.paths)}")
        
        return self.paths # returns a list of paths, each path is a list of points, each point is a tuple
    
    def transform_coordinates_with_corners(self, canvas_corners):
        """Transform image coordinates to robot workspace coordinates using canvas corners"""
        if not self.paths or not canvas_corners:
            print("No paths or canvas corners to transform")
            return self.paths
            
        print(f"Transform: Canvas corners received with {len(canvas_corners)} points")
        
        # Extract canvas corner positions
        # Corner order: [BL, BR, TR, TL]
        if len(canvas_corners) < 4:
            print("Error: Need 4 canvas corners for transformation")
            return self.paths
            
        bl = canvas_corners[0]
        br = canvas_corners[1]
        tr = canvas_corners[2]
        tl = canvas_corners[3]
        
        # Apply margin adjustments (inward from edges)
        x_margin_m = 29.5 / 1000.0  # Convert mm to m
        y_margin_m = 21.0 / 1000.0  # Convert mm to m
        
        # Calculate canvas vectors (sides of the canvas)
        bottom_vector = [br['x'] - bl['x'], br['y'] - bl['y']]
        left_vector = [tl['x'] - bl['x'], tl['y'] - bl['y']]
        
        # Calculate unit vectors
        bottom_length = (bottom_vector[0]**2 + bottom_vector[1]**2)**0.5
        left_length = (left_vector[0]**2 + left_vector[1]**2)**0.5
        
        bottom_unit = [bottom_vector[0]/bottom_length, bottom_vector[1]/bottom_length]
        left_unit = [left_vector[0]/left_length, left_vector[1]/left_length]
        
        # Calculate new corner positions with margins
        bl_adjusted = {
            'x': bl['x'] + x_margin_m * bottom_unit[0] + y_margin_m * left_unit[0],
            'y': bl['y'] + x_margin_m * bottom_unit[1] + y_margin_m * left_unit[1],
            'z': bl['z']
        }
        
        br_adjusted = {
            'x': br['x'] - x_margin_m * bottom_unit[0] + y_margin_m * left_unit[0],
            'y': br['y'] - x_margin_m * bottom_unit[1] + y_margin_m * left_unit[1],
            'z': br['z']
        }
        
        tr_adjusted = {
            'x': tr['x'] - x_margin_m * bottom_unit[0] - y_margin_m * left_unit[0],
            'y': tr['y'] - x_margin_m * bottom_unit[1] - y_margin_m * left_unit[1],
            'z': tr['z']
        }
        
        tl_adjusted = {
            'x': tl['x'] + x_margin_m * bottom_unit[0] - y_margin_m * left_unit[0],
            'y': tl['y'] + x_margin_m * bottom_unit[1] - y_margin_m * left_unit[1],
            'z': tl['z']
        }
        
        print(f"Adjusted canvas corners with margins: x={x_margin_m*1000}mm, y={y_margin_m*1000}mm")
        
        # Find bounding box of image paths
        if not self.paths:
            return self.paths
        
        min_x = min(min(point[0] for point in path) for path in self.paths)
        max_x = max(max(point[0] for point in path) for path in self.paths)
        min_y = min(min(point[1] for point in path) for path in self.paths)
        max_y = max(max(point[1] for point in path) for path in self.paths)
        
        img_width = max_x - min_x
        img_height = max_y - min_y
        
        if img_width <= 0 or img_height <= 0:
            print("Error: Invalid image dimensions")
            return self.paths
        
        print(f"Image bounds: X: {min_x} to {max_x}, Y: {min_y} to {max_y}")
        
        # Transform each path
        transformed_paths = []
        for path in self.paths:
            transformed_path = []
            for point in path:
                # Normalise coordinates to [0,1] range
                norm_x = (point[0] - min_x) / img_width
                norm_y = (point[1] - min_y) / img_height
                
                # Invert Y coordinate (image Y is top to bottom, robot Y is bottom to top)
                norm_y = 1.0 - norm_y
                
                # Bilinear interpolation to map to robot coordinates
                robot_x = (1 - norm_x) * (1 - norm_y) * bl_adjusted['x'] + \
                          norm_x * (1 - norm_y) * br_adjusted['x'] + \
                          (1 - norm_x) * norm_y * tl_adjusted['x'] + \
                          norm_x * norm_y * tr_adjusted['x']
                          
                robot_y = (1 - norm_x) * (1 - norm_y) * bl_adjusted['y'] + \
                          norm_x * (1 - norm_y) * br_adjusted['y'] + \
                          (1 - norm_x) * norm_y * tl_adjusted['y'] + \
                          norm_x * norm_y * tr_adjusted['y']
                
                # Use average Z height of corners for Z coordinate
                robot_z = (bl_adjusted['z'] + br_adjusted['z'] + tr_adjusted['z'] + tl_adjusted['z']) / 4.0
                
                transformed_path.append((robot_x, robot_y))
            
            transformed_paths.append(transformed_path)
        
        self.paths = transformed_paths
        
        print(f"Transformation complete: {len(self.paths)} paths transformed")
        return self.paths
    
    def optimise_paths(self):
        """Placeholder for path optimisation"""
        print(f"Optimise: Working with {len(self.paths)} paths")
        return self.paths
    
    def identify_facial_features(self):
        """Placeholder for feature identification"""
        print("Identify: Features identification called")
        return
    
    def prioritise_features(self):
        """Placeholder for feature prioritisation"""
        print("Prioritise: Feature prioritisation called")
        return self.paths
        
    def visualize_paths(self):
        """Create a visualization of the extracted paths"""
        if not self.paths:
            return None
            
        import matplotlib.pyplot as plt
        
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Show original image in the background (if available)
        if self.original_image is not None:
            ax.imshow(self.original_image, cmap='gray', alpha=0.3)
        
        # Show each path in a different color
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        
        for i, path in enumerate(self.paths):
            if not path:
                continue
                
            color = colors[i % len(colors)]
            x_values = [p[0] for p in path]
            y_values = [p[1] for p in path]
            
            ax.plot(x_values, y_values, color=color, linewidth=2)
            
            # Show the starting point of each path
            ax.plot(x_values[0], y_values[0], 'o', color=color)
            
            # Add a label with the path number
            ax.annotate(str(i), (x_values[0], y_values[0]), 
                        fontsize=12, color=color)
        
        ax.set_title(f'Extracted Paths ({len(self.paths)} paths)')
        ax.axis('equal')
        
        return fig