import numpy as np
import matplotlib.pyplot as plt
import itertools

# -----------------------------
# CONFIG
# -----------------------------
DT = 0.01
T_END = 6.0
T = np.arange(0.0, T_END + DT, DT)

BASE_IXX = 0.044
L_ref = 0.152
S = np.pi * (L_ref / 2.0) ** 2

MAX_DEFLECTION = np.deg2rad(15.0)

RHO = 1.225
A_SOUND = 340.0
BURNOUT_VEL = 0.19 # Mach

C_DAMP_MULTIPLIER = 0.5
CONST_FIN_ERROR = 0.5

# Roll-angle guidance target
PHI_REF = np.deg2rad(20.0)

# -----------------------------
# CONTROLLER (WORKING VALUES)
# -----------------------------
K_p_base = 0.2440
K_i_base = 0.4299
K_d_base = 0.06955

FILTER_ALPHA = 0.6350
V_REF = 120.0

INT_LIM = np.deg2rad(23.50)

# -----------------------------
# INITIAL CONDITIONS
# -----------------------------
phi0 = 0.0
p0 = np.deg2rad(500.0)

IXX_MULTS = [0.5, 1.0, 1.5]
AERO_MULTS = [1.0, 0.5, 0.1]


# -----------------------------
# IMPERFECTION ERROR
# -----------------------------
def constant_imperfection_error(fin_error_deg=0.35):
    delta = np.deg2rad(fin_error_deg)

    d = L_ref
    r = d / 2.0
    S_ref = S

    c_root_f = 0.235
    c_tip_f = 0.11
    b_f = 0.135

    S_fin = 0.5 * (c_root_f + c_tip_f) * b_f

    y_mac_rel = (b_f / 3.0) * ((c_root_f + 2.0 * c_tip_f) / (c_root_f + c_tip_f))
    Y_mac_f = r + y_mac_rel

    AR_f = 2.0 * (b_f ** 2) / S_fin
    CN_alpha_f = (2.0 * np.pi * AR_f) / (2.0 + np.sqrt(4.0 + AR_f ** 2))

    C_l_delta = (4.0 * CN_alpha_f * S_fin * Y_mac_f) / (S_ref * d)

    C_roll_error = C_l_delta * delta
    return C_roll_error


CONST_FIN_ERROR = constant_imperfection_error(CONST_FIN_ERROR)


# -----------------------------
# AERO MODEL (Code 1 upgraded into Code 2)
# -----------------------------
def get_datcom_aero(delta, aero_mult):

    d = L_ref
    r = d / 2.0
    S_ref = S

    EFFICIENCY_FACTOR = aero_mult

    # -------------------------
    # CANARD
    # -------------------------
    c_root_c = 0.06
    b_c = 0.07
    S_can = 0.5 * c_root_c * b_c

    Y_mac_c = r + (b_c / 3.0)

    AR_c = 2.0 * (b_c ** 2) / (S_can + 1e-9)
    CN_alpha_c = (2 * np.pi * AR_c) / (2 + np.sqrt(4 + AR_c ** 2))

    # roll effectiveness
    C_l_delta = (4 * CN_alpha_c * S_can * Y_mac_c) / (S_ref * d + 1e-9)

    C_roll_canard = C_l_delta * delta * EFFICIENCY_FACTOR

    # -------------------------
    # FIN
    # -------------------------
    c_root_f = 0.235
    c_tip_f = 0.135
    b_f = 0.13

    S_fin = 0.5 * (c_root_f + c_tip_f) * b_f

    y_mac_rel = (b_f / 3.0) * ((c_root_f + 2 * c_tip_f) / (c_root_f + c_tip_f))
    Y_mac_f = r + y_mac_rel

    AR_f = 2.0 * (b_f ** 2) / (S_fin + 1e-9)
    CN_alpha_f = (2 * np.pi * AR_f) / (2 + np.sqrt(4 + AR_f ** 2))

    # -------------------------
    # DAMPING (physical structure from Code 1)
    # -------------------------
    C_damp_canards = (8 * CN_alpha_c * S_can * Y_mac_c ** 2) / (S_ref * d ** 2 + 1e-9)
    C_damp_fins = (8 * CN_alpha_f * S_fin * Y_mac_f ** 2) / (S_ref * d ** 2 + 1e-9)

    C_damp_total = (C_damp_canards + C_damp_fins) * C_DAMP_MULTIPLIER

    # -------------------------
    # TOTAL
    # -------------------------
    C_roll_total = C_roll_canard + CONST_FIN_ERROR

    return C_roll_total, C_damp_total


# -----------------------------
# MACH PROFILE
# -----------------------------
def mach_profile(t):
    x = np.clip(t / T_END, 0.0, 1.0)
    return BURNOUT_VEL * (1.0 - x) ** 2


# -----------------------------
# GAIN SCHEDULE
# -----------------------------
def scheduled_gains(V):

    q = 0.5 * RHO * V ** 2
    q_ref = 0.5 * RHO * V_REF ** 2

    scale = np.sqrt(q_ref / (q + 1e-6))
    scale = np.clip(scale, 0.5, 2.0)

    Kp = K_p_base * scale
    Kd = K_d_base * scale

    return Kp, Kd


# -----------------------------
# SIMULATION
# -----------------------------
def run_sim(ixx_mult, aero_mult):

    phi = np.zeros_like(T)
    p = np.zeros_like(T)
    delta_hist = np.zeros_like(T)

    phi[0] = phi0
    p[0] = p0

    current_ixx = BASE_IXX * ixx_mult

    # No servo slew model now: delta follows the controller target directly
    current_delta = 0.0
    filtered_target = 0.0

    integral_error = 0.0

    for i in range(len(T) - 1):

        mach = mach_profile(T[i])
        V = mach * A_SOUND
        q = 0.5 * RHO * V ** 2

        # current measurement, no delay
        phi_measured = phi[i]
        p_measured = p[i]

        # gains
        Kp, Kd = scheduled_gains(V)

        # angle guidance error
        error = PHI_REF - phi_measured

        # anti-windup + small I-term
        if abs(current_delta) < 0.70 * MAX_DEFLECTION and abs(error) < np.deg2rad(60.0):
            integral_error += error * DT

        integral_error = np.clip(integral_error, -INT_LIM, INT_LIM)

        # PID terms on angle; roll rate is used as damping
        p_term = Kp * error
        i_term = K_i_base * integral_error
        d_term = -Kd * p_measured

        raw_target = p_term + i_term + d_term

        # command smoothing only; no actuator slew model
        filtered_target = (
            FILTER_ALPHA * raw_target
            + (1.0 - FILTER_ALPHA) * filtered_target
        )

        target_delta = np.clip(filtered_target, -MAX_DEFLECTION, MAX_DEFLECTION)

        # direct actuator response
        current_delta = target_delta
        delta_hist[i] = current_delta

        # moments
        C_roll, C_damp = get_datcom_aero(current_delta, aero_mult)

        M_ctrl = q * S * L_ref * C_roll

        M_damp = -q * S * L_ref * C_damp * (
            p[i] * L_ref / (2.0 * V + 1e-6)
        )

        M_total = M_ctrl + M_damp

        # dynamics
        p_dot = M_total / current_ixx

        p[i + 1] = p[i] + p_dot * DT
        phi[i + 1] = phi[i] + p[i] * DT

    return phi, p, delta_hist


# -----------------------------
# RUN TESTS
# -----------------------------
results = {}

for im, am in itertools.product(IXX_MULTS, AERO_MULTS):
    phi_sim, p_sim, d_sim = run_sim(im, am)

    results[(im, am)] = {
        "phi_sim": phi_sim,
        "p_sim": p_sim,
        "d_sim": d_sim,
    }


# -----------------------------
# PLOTS
# -----------------------------
angles_deg = np.linspace(-15, 15, 100)
angles_rad = np.deg2rad(angles_deg)
phi_ref_hist = np.full_like(T, PHI_REF)

fig, axs = plt.subplots(2, 2, figsize=(16, 12))
colors = plt.cm.jet(np.linspace(0, 1, len(results)))

# Roll angle tracking
for idx, ((im, am), data) in enumerate(results.items()):
    axs[0, 0].plot(
        T,
        np.rad2deg(data["phi_sim"]),
        color=colors[idx],
        linewidth=1.5,
        label=f"IXX {im}x Aero {am}x",
    )

axs[0, 0].plot(
    T,
    np.rad2deg(phi_ref_hist),
    linestyle="--",
    linewidth=2.0,
    color="black",
    label="Target",
)

axs[0, 0].set_title("Roll Angle Tracking")
axs[0, 0].set_xlabel("Time [s]")
axs[0, 0].set_ylabel("Roll Angle [deg]")
axs[0, 0].grid(True)
axs[0, 0].legend(fontsize="small")

# Roll rate
for idx, ((im, am), data) in enumerate(results.items()):
    axs[0, 1].plot(
        T,
        np.rad2deg(data["p_sim"]),
        color=colors[idx],
        linewidth=1.5,
    )

axs[0, 1].set_title("Roll Rate")
axs[0, 1].set_xlabel("Time [s]")
axs[0, 1].set_ylabel("Roll Rate [deg/s]")
axs[0, 1].grid(True)

# Fin deflection
for idx, ((im, am), data) in enumerate(results.items()):
    axs[1, 0].plot(
        T,
        np.rad2deg(data["d_sim"]),
        color=colors[idx],
        linewidth=1.5,
    )

axs[1, 0].set_title("Controller Deflection")
axs[1, 0].set_xlabel("Time [s]")
axs[1, 0].set_ylabel("Canard Angle [deg]")
axs[1, 0].grid(True)

# Speed / Mach profile
mach_hist = np.array([mach_profile(t) for t in T])
speed_hist = mach_hist * A_SOUND

ax = axs[1, 1]
ax.plot(T, mach_hist, linewidth=2, color="limegreen", label="Mach")
ax.set_title("Mach / Speed Profile")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Mach [-]")
ax.grid(True)

ax2 = ax.twinx()
ax2.plot(T, speed_hist, linewidth=0.1, linestyle="--", color="limegreen", label="Speed")
ax2.set_ylabel("Speed [m/s]")

plt.tight_layout()
plt.show()