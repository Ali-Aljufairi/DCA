import numpy as np
from PIL import Image
import os

x = 10
y = 200

if not os.path.exists('images'):
    os.makedirs('images')

with open('data.txt') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if line.strip() == 'Detector #1 data:':
        print(f"frome NO. #{i}")
        data = lines[i+1].split()
        data = [int(d) for d in data]

        # Debugging checks
        print(f"Data size: {len(data)}")
        print(f"Data array: {data}")

        data = np.array(data)

        # Verify if data size matches expected shape
        if data.size != x * y:
            raise ValueError(f"Data size ({data.size}) does not match the expected shape ({x}x{y})")

        data = data.reshape(x, y)

        img = Image.fromarray(data.astype(np.uint8))  # Convert data to uint8 type for image creation
        img.save(f'images/image{i}.png')
