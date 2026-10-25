import math

def bezier_1D(P, s):
    n = len(P) - 1
    return sum(
        math.comb(n, i) * (1 - s)**(n - i) * s**i * P[i]
        for i in range(n + 1)
    )

def main():
    import numpy as np
    import matplotlib.pyplot as plt

    phi = np.linspace(0,1,200)
    Px = np.array([0,0.05,0.2,0.3])
    Pz = np.array([0,0.05,0.1,0.0])

    Px_array = []
    Pz_array = []
    for i in range(len(phi)):
        px = bezier_1D(Px, phi[i])
        pz = bezier_1D(Pz, phi[i])
        Px_array.append(px)
        Pz_array.append(pz)
    plt.figure()
    plt.plot(Px_array,Pz_array)
    plt.plot(Px, Pz)
    plt.show()

if __name__ == "__main__":
    main()