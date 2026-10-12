
import numpy as np
import matplotlib.pyplot as plt
from scipy.special import erf


def phase_contact_probability(phi, scheduled_contact, var0, var1):
    s = scheduled_contact
    s_bar = 1 - s

    p_stance = (
        erf(phi / (var0 * np.sqrt(2)))
        + erf((1.0 - phi) / (var1 * np.sqrt(2)))
    )

    p_swing = (
        2.0
        + erf(-phi / (var0 * np.sqrt(2)))
        + erf((phi - 1.0) / (var1 * np.sqrt(2)))
    )

    return 0.5 * (s * p_stance + s_bar * p_swing)



# Normalized phase
phi = np.linspace(0.0, 1.0, 1000)

# Variances shown in Fig. 4
variances = [0.1, 0.025, 0.05]

fig, axes = plt.subplots(2, 1, figsize=(7, 7), sharex=True)

for var in variances:
    sigma = np.sqrt(var)

    # Top: stance is scheduled
    p_stance = phase_contact_probability( phi, scheduled_contact=1,
        var0 = var, var1 =var
    )
    axes[0].plot(
        phi, p_stance,
        label=fr"$\sigma^2={var}$"
    )

    # Bottom: swing is scheduled
    p_swing = phase_contact_probability(
        phi, scheduled_contact=0,
        var0 = var, var1 =var
    )
    axes[1].plot(
        phi, p_swing,
        label=fr"$\sigma^2={var}$"
    )

axes[0].set_title("Scheduled stance")
axes[0].set_ylabel("Contact probability")
axes[0].set_ylim(0.4, 1.05)
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].set_title("Scheduled swing")
axes[1].set_xlabel("Normalized subphase progress, $\phi$")
axes[1].set_ylabel("Contact probability")
axes[1].set_ylim(0, 0.5)
axes[1].legend()
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("fig4_phase_contact_probability.png", dpi=300)
plt.show()
