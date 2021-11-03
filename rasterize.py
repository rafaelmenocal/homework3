import matplotlib.pyplot as plt

import pathlib


text = pathlib.Path("test.txt").read_text()


rows = []
for line in text.splitlines():
    rows.append([float(val) for val in line.split()])


import numpy as np

data = np.array(rows)
print(data.shape)
plt.matshow(data)
plt.show()