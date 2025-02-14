import numpy as np
import matplotlib.pyplot

backLegSensorValues = np.load("data/backLegSensorValues.npy")
frontLegSensorValues = np.load("data/frontLegSensorValues.npy")

#matplotlib.pyplot.plot(backLegSensorValues, label = "Back Leg", linewidth = 3)
#matplotlib.pyplot.plot(frontLegSensorValues, label = "Front Leg")

#matplotlib.pyplot.legend()
#matplotlib.pyplot.show()


frontLeg_targetAngles = np.load("data/frontLeg_targetAngles.npy")
backLeg_targetAngles = np.load("data/backLeg_targetAngles.npy")

matplotlib.pyplot.plot(frontLeg_targetAngles, label = "Front Leg Target Angles")
matplotlib.pyplot.plot(backLeg_targetAngles, label = "Back Leg Target Angles")

matplotlib.pyplot.legend()
matplotlib.pyplot.show()

