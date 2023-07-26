import os
import numpy as np
from PIL import Image

def npy_to_png(input_folder, output_folder):
    # Check if the output folder exists, create it if not
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Get a list of all .npy files in the input folder
    npy_files = [f for f in os.listdir(input_folder) if f.endswith('.npy')]

    # Process each .npy file and save the corresponding .png image
    for npy_file in npy_files:
        npy_path = os.path.join(input_folder, npy_file)
        png_file = os.path.splitext(npy_file)[0] + '.png'
        png_path = os.path.join(output_folder, png_file)

        # Load the .npy file using NumPy
        data = np.load(npy_path)

        # Convert the NumPy array to a PIL Image
        image = Image.fromarray(data)

        # Save the PIL Image as a .png file
        image.save(png_path)

if __name__ == "__main__":
    input_folder = "path/to/your/npy/files"
    output_folder = "png"
    npy_to_png(input_folder, output_folder)
