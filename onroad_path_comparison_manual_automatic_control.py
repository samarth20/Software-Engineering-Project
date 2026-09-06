from matplotlib import pyplot as plt
import numpy as np

optimal = np.load('XY_optimal_route.npz')
X_o = optimal['arr_0']
Y_o = optimal['arr_1']
automatic = np.load('XY_automatic.npz')
X_a = automatic['arr_0']
Y_a = automatic['arr_1']
manual = np.load('XY_manual_steering_wheel.npz')
X_m = manual['arr_0']
Y_m = manual['arr_1']

plt.plot(-1 * X_o[2:], Y_o[2:], 'bo',
         label='Optimal line')
plt.plot(-1 * X_a[2:], Y_a[2:], 'ro',
         label='Autopilot')
plt.plot(-1 * X_m[2:], Y_m[2:], 'co',
         label='Manual')
plt.annotate('Start Point', xy=(-X_o[1], Y_o[1]),
             xytext=(-X_o[1] - 40, Y_o[1]),
             arrowprops=dict(facecolor='black', shrink=0.05),
             )
plt.annotate('End Point', xy=(-X_o[-1], Y_o[-1]),
             xytext=(-X_o[-1] - 40, Y_o[-1]),
             arrowprops=dict(facecolor='black', shrink=0.05),
             )
plt.legend()
plt.show()