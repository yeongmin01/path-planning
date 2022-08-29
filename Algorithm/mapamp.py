import numpy as np
import cv2
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt

image_size = 50
box_size = 5
no_box = 5
image = Image.new('RGB', (image_size,image_size))
d = ImageDraw.Draw(image)

np.random.seed(777) #6, 7, 9
for i in range(no_box):
    xy = np.random.randint(image_size,size=2)
    rgb = np.random.randint(255, size=3)
    # print(rgb)
    # print(xy)
    d.rectangle([xy[0],xy[1],xy[0]+box_size,xy[1]+box_size], fill=(rgb[0],rgb[1],rgb[2]))

d.rectangle([10,50,20,20], fill=(255,255,255))
d.rectangle([10,10,20,0], fill=(255,255,255))
d.rectangle([30,50,40,40], fill=(255,255,255))
d.rectangle([30,30,40,0], fill=(255,255,255))
# image.show()
imgArray = np.array(image)

map = np.zeros((imgArray.shape[1], imgArray.shape[0]))
for i in range(imgArray.shape[0]):
    for j in range(imgArray.shape[1]):

        map[i][j] = (int(imgArray[i][j][0])+int(imgArray[i][j][1])+int(imgArray[i][j][2]))/(255*3)

map = np.transpose(map)
map = 1- map

plt.imshow(np.transpose(1-map), cmap="jet")
plt.colorbar()
plt.show()
