import numpy as np

a = np.matrix('1 2; 3 4')
b = np.linalg.inv(a)
c = a*b
eye = np.eye(2)
print(a)
print(b)
print(c)
print(eye)

d = np.linalg.solve(a,eye)
print(d)