
import matplotlib.pyplot as plt
import math
import numpy as np


def main():
	x, y = np.loadtxt('obstacle.dat', delimiter=' ', unpack=True)
	plt.plot(x, y, "*")
	plt.plot(88, 145, "y+")
	plt.plot(100, 199, "r.")
	plt.axis('equal')
	plt.grid()
	# manager = plt.get_current_fig_manager()
	# manager.resize(*manager.window.maxsize())
	plt.show()
	# plt.savefig('obstacle.png', dpi=500)

if __name__ == "__main__":
    main()