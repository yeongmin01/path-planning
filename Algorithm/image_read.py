from PIL import Image
import matplotlib.pyplot as plt
import numpy as np

img = Image.open('map/map01.png')
imgArray = np.array(img)
print(imgArray[1][15])

plt.imshow(img)
plt.show()