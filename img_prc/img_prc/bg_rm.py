#!/usr/bin/env python3

from rembg import remove
from PIL import Image

# Load the image
input_path = 'test.jpg'
output_path = 'output_image.png'

input_image = Image.open(input_path)

# Remove the background
output_image = remove(input_image)

# Save the output image
output_image.save(output_path)
