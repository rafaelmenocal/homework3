import matplotlib.pyplot as plt

import pathlib

import numpy as np

# files = list(pathlib.Path("logs").glob("*"))
# files = [float(f.stem.split("obs")[-1]) for f in files]
# print("MAX", max(files))


# text = pathlib.Path("test.txt").read_text()


# rows = []
# for line in text.splitlines():
#     rows.append([float(val) for val in line.split()])


# data1 = np.asarray(rows)
# data1 = 1 - data1

# plt.matshow(data1, cmap='Greys')
# plt.show()



text = pathlib.Path("test-xy.txt").read_text()


rows = []
for line in text.splitlines():
    rows.append([float(val) for val in line.split()])

for r in rows:
    print(len(r))

import numpy as np

data = np.asarray(rows)

data = 1 - data

plt.matshow(data, cmap='Greys')
plt.show()


from PIL import Image
import cv2
data = None
for f in pathlib.Path("logs").glob("*.txt"):
    print(f)

    text = f.read_text()
    rows = []
    for line in text.splitlines():
        data = [float(val) for val in line.split()]
        if len(data) == 200:
            rows.append(data)

    if rows:
        print(f.name)
        data = np.asarray(rows)
        # if float(f.stem.split("obs")[-1]) == max(files):
        #     print("THISSSS")
        #     plt.matshow(data + data1, cmap='Greys')
        #     plt.show()
        
        plt.imsave(f.with_suffix(".png"), data / np.max(data), cmap='gray')
        print(f"saved to {f.with_suffix('.png')}")




text = pathlib.Path("test-new.txt").read_text()


rows = []
for line in text.splitlines():
    rows.append([float(val) for val in line.split()])

for r in rows:
    print(len(r))

import numpy as np

data = np.asarray(rows)

data = 1 - data

plt.matshow(data, cmap='Greys')
plt.show()


text = pathlib.Path("test-xy.txt").read_text()


rows = []
for line in text.splitlines():
    rows.append([float(val) for val in line.split()])

for r in rows:
    print(len(r))

import numpy as np

data = np.asarray(rows)

data = 1 - data

plt.matshow(data, cmap='Greys')
plt.show()
