a3 = 0.7790287413076263
a2 = -0.003307914541061115
a1 = -3.9204118247392246e-07
a0 = -2.7261725080037385e-10

skin = 5

calibration_force = a3 + a2 * (skin*1000) + a1* (skin*1000)**2 + a0 * (skin*1000)**3

print(calibration_force)
