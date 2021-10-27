'''import matplotlib.pyplot as plt
import matplotlib.cbook as cbook'''

import numpy as np
import pandas as pd

msft = pd.read_csv('acceleration_logger.csv')
print(msft)
msft.plot()