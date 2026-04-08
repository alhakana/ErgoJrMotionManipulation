#!/usr/bin/env python3
"""
Poppy Ergo Jr - Circle using IK, matching CoppeliaSim simulation

Replicates the simulation's IK approach:
  - ALL 6 motors active (like simulation: all joints in IK group)
  - m4 has full range (traces the circle), other motors have tight bounds
    around their reference values (small adjustments only, like simulation)
  - Sequential solving from current config (like simulation: handleIk each step)
  - 7 circle points, random CW/CCW each step (like simulation: sysCall_actuation)

In the simulation, damped least squares + pose constraint naturally makes m4
do the heavy lifting while other motors adjust slightly. We achieve the same
by giving m4 full range and constraining others to +/-15 deg of their reference.
The IK genuinely discovers the motor angles — it only knows target 3D positions.

Requirements: pip install ikpy numpy
"""

import os
import sys
import time
import random
import math
import tempfile
import numpy as np

# ===================================================================
# CONFIGURATION
# ===================================================================

NUM_POINTS = 5
HOLD_TIME = 3.0         # same as simulation
MOVE_TIME = 1.5         # seconds between points
IK_ITERATIONS = 200

# How much freedom non-primary motors get (degrees around reference value).
# Simulation's damped LS naturally limits these; we enforce via bounds.
SECONDARY_RANGE = 15

# ===================================================================
# MOTOR PROPERTIES (from poppy_ergo_jr.json)
# ===================================================================

MOTORS = ["m1", "m2", "m3", "m4", "m5", "m6"]
DIRECT = {"m1": True, "m2": False, "m3": False,
           "m4": True, "m5": False, "m6": False}
LIMITS = {"m1": (-150, 150), "m2": (-90, 125), "m3": (-90, 90),
           "m4": (-150, 150), "m5": (-90, 90),  "m6": (-110, 90)}

# ===================================================================
# EMBEDDED URDF (official poppy_ergo_jr.urdf)
# ===================================================================

EMBEDDED_URDF = """\
<?xml version="1.0" ?>
<robot name="PoppyErgoJr">
  <link name="base_link">
    <inertial>
      <origin xyz="-0.00273845833301537 0.00821639745723966 -0.0338977122062889" rpy="0 0 0"/>
      <mass value="0.0802410895423366"/>
      <inertia ixx="0.00011430194806305" ixy="-1.74777682977282E-06" ixz="7.8063859003738E-06"
               iyy="0.000177560979735167" iyz="6.80763560469771E-06" izz="7.51325469990478E-05"/>
    </inertial>
  </link>
  <link name="long_U">
    <inertial>
      <origin xyz="-0.00243058056226709 1.14505937431452E-07 0.0129312523306501" rpy="0 0 0"/>
      <mass value="0.0042118228619085"/>
      <inertia ixx="5.78707668456099E-07" ixy="9.44283869977724E-12" ixz="-1.13587476220175E-07"
               iyy="1.25420346852677E-06" iyz="4.64830147251523E-12" izz="9.27174545760678E-07"/>
    </inertial>
  </link>
  <joint name="m1" type="continuous">
    <origin xyz="0 0 0.0327993216120967" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="long_U"/>
    <axis xyz="0 0 -1"/>
  </joint>
  <link name="section_1">
    <inertial>
      <origin xyz="-0.000168296047337745 0.013441283142678 -3.36410230095056E-09" rpy="0 0 0"/>
      <mass value="0.0201321844066136"/>
      <inertia ixx="6.51960684491032E-06" ixy="-5.35361387413197E-07" ixz="-5.67167110473808E-13"
               iyy="2.37206686086462E-06" iyz="-1.07202400036876E-12" izz="7.46616268702629E-06"/>
    </inertial>
  </link>
  <joint name="m2" type="continuous">
    <origin xyz="0 0 0.0240006783879033" rpy="1.5707963267949 0 0"/>
    <parent link="long_U"/>
    <child link="section_1"/>
    <axis xyz="-1 0 0"/>
  </joint>
  <link name="section_2">
    <inertial>
      <origin xyz="0.000407772974186014 0.00840914919415664 -1.07672250595912E-09" rpy="0 0 0"/>
      <mass value="0.017821725151696"/>
      <inertia ixx="2.52304488643742E-06" ixy="-6.63873566658207E-08" ixz="-2.02107710451767E-13"
               iyy="1.64177128414312E-06" iyz="3.46919494425175E-13" izz="2.86561030339994E-06"/>
    </inertial>
  </link>
  <joint name="m3" type="continuous">
    <origin xyz="0 0.054 0" rpy="0 0 0"/>
    <parent link="section_1"/>
    <child link="section_2"/>
    <axis xyz="-1 0 0"/>
  </joint>
  <link name="section_3">
    <inertial>
      <origin xyz="-0.000529269977744151 -0.0148172531686623 -0.0123545994437081" rpy="0 0 0"/>
      <mass value="0.0201321844066136"/>
      <inertia ixx="5.58716321477767E-06" ixy="3.84615911021611E-09" ixz="3.80051502924447E-07"
               iyy="6.00831891375732E-06" iyz="9.50610838817802E-08" izz="2.36437426368354E-06"/>
    </inertial>
  </link>
  <joint name="m4" type="continuous">
    <origin xyz="0 0.0298217741221248 0" rpy="3.14159265358979 0 0"/>
    <parent link="section_2"/>
    <child link="section_3"/>
    <axis xyz="0 1 0"/>
  </joint>
  <link name="section_4">
    <inertial>
      <origin xyz="-0.000168296047337745 0.013441283142678 -3.36410227452788E-09" rpy="0 0 0"/>
      <mass value="0.0201321844066137"/>
      <inertia ixx="6.51960684491033E-06" ixy="-5.3536138741319E-07" ixz="-5.67167109549002E-13"
               iyy="2.37206686086462E-06" iyz="-1.07202399874137E-12" izz="7.46616268702631E-06"/>
    </inertial>
  </link>
  <joint name="m5" type="continuous">
    <origin xyz="0 -0.0151782258778753 -0.048" rpy="-1.5707963267949 0 0"/>
    <parent link="section_3"/>
    <child link="section_4"/>
    <axis xyz="-1 0 0"/>
  </joint>
  <link name="tip">
    <inertial>
      <origin xyz="1.30842922607677E-09 0.000440812249500631 -0.00687616557228032" rpy="0 0 0"/>
      <mass value="0.0164859649700598"/>
      <inertia ixx="2.21073012081829E-06" ixy="2.00704445336612E-13" ixz="2.61313974888777E-13"
               iyy="1.95245657563094E-06" iyz="5.52468772381494E-08" izz="1.46932588043522E-06"/>
    </inertial>
  </link>
  <joint name="m6" type="continuous">
    <origin xyz="0 0.054 0" rpy="1.5707963267949 1.5707963267949 0"/>
    <parent link="section_4"/>
    <child link="tip"/>
    <axis xyz="0 -1 0"/>
  </joint>
</robot>
"""

# ===================================================================
# HELPERS
# ===================================================================


def get_urdf_path():
    local = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "poppy_ergo_jr.urdf")
    if os.path.exists(local):
        return local
    try:
        import poppy_ergo_jr
        pkg = os.path.join(os.path.dirname(poppy_ergo_jr.__file__),
                           "poppy_ergo_jr.urdf")
        if os.path.exists(pkg):
            return pkg
    except ImportError:
        pass
    fd, path = tempfile.mkstemp(suffix=".urdf", prefix="poppy_")
    with os.fdopen(fd, "w") as f:
        f.write(EMBEDDED_URDF)
    return path


def build_chain(urdf_path):
    """Build chain with ALL 6 joints active (like simulation).

    m4 gets full range (-150 to +150) to trace the circle.
    Other motors get tight bounds around their reference values:
      m1: 0 +/- SECONDARY_RANGE      (base yaw — small adjustments)
      m2: 0 +/- SECONDARY_RANGE      (shoulder — small adjustments)
      m3: -90 +/- SECONDARY_RANGE    (elbow — stays bent)
      m5: -90 +/- SECONDARY_RANGE    (wrist — stays bent)
      m6: 0 +/- SECONDARY_RANGE      (tip roll — small adjustments)

    This mirrors the simulation where damped-LS + pose constraint naturally
    keeps non-primary motors near their reference while m4 rotates freely.
    """
    try:
        from ikpy.chain import Chain
    except ImportError:
        print("ERROR: pip install ikpy")
        sys.exit(1)

    chain = Chain.from_urdf_file(urdf_path, last_link_vector=[0, 0, -0.07])

    S = SECONDARY_RANGE  # shorthand

    # Reference Poppy angles and allowed range for each motor
    # Format: (poppy_center, poppy_half_range)
    motor_config = {
        "m1": (0,   S),    # small adjustments around 0
        "m2": (0,   S),    # small adjustments around 0
        "m3": (-90, S),    # stays near -90
        "m4": (0,   150),  # FULL range for circle
        "m5": (-90, S),    # stays near -90
        "m6": (0,   S),    # small adjustments around 0
    }

    for lnk in chain.links:
        if lnk.name in motor_config:
            center_poppy, half_range = motor_config[lnk.name]
            lo_poppy = center_poppy - half_range
            hi_poppy = center_poppy + half_range

            # Clamp to physical motor limits
            phys_lo, phys_hi = LIMITS[lnk.name]
            lo_poppy = max(lo_poppy, phys_lo)
            hi_poppy = min(hi_poppy, phys_hi)

            if DIRECT[lnk.name]:
                lnk.bounds = (np.deg2rad(lo_poppy), np.deg2rad(hi_poppy))
            else:
                # Indirect: IK angle = -Poppy angle, so negate and flip
                lnk.bounds = (np.deg2rad(-hi_poppy), np.deg2rad(-lo_poppy))

    return chain


def poppy2ik(angles):
    vec = [0.0]
    for m in MOTORS:
        a = angles.get(m, 0.0)
        vec.append(np.deg2rad(-a if not DIRECT[m] else a))
    vec.append(0.0)
    return vec


def ik2poppy(ik_vec):
    out = {}
    for i, m in enumerate(MOTORS):
        a = np.rad2deg(ik_vec[i + 1])
        out[m] = round(-a if not DIRECT[m] else a, 1)
    return out


def clamp(angles):
    return {m: max(LIMITS[m][0], min(LIMITS[m][1], angles[m])) for m in MOTORS}


def fk_pos(chain, poppy_angles):
    return chain.forward_kinematics(poppy2ik(poppy_angles))[:3, 3]


def solve_ik(chain, target_pos, seed):
    """Position-only IK. Joint bounds enforce the simulation-like behavior."""
    kw = {
        "target_position": target_pos,
        "orientation_mode": None,
        "initial_position": seed,
    }
    try:
        kw["max_iter"] = IK_ITERATIONS
        return chain.inverse_kinematics(**kw)
    except TypeError:
        del kw["max_iter"]
        return chain.inverse_kinematics(**kw)


# ===================================================================
# MAIN
# ===================================================================


def main():
    print("=" * 65)
    print("  POPPY ERGO JR  -  Circle IK (matching simulation)")
    print("  All 6 motors active, m4 full range, others +/-{} deg".format(SECONDARY_RANGE))
    print("=" * 65)

    # ---- 1. Build chain (all joints active) ----
    print("\n[1] Building chain (all 6 joints active)...")
    chain = build_chain(get_urdf_path())

    # ---- 2. Reference pose ----
    ref_poppy = {"m1": 0, "m2": 0, "m3": -90, "m4": 0, "m5": -90, "m6": 0}
    ref_ik = poppy2ik(ref_poppy)
    ref_pos = chain.forward_kinematics(ref_ik)[:3, 3]

    print("    Reference pose: m3=-90, m5=-90, m4=0")
    print("    End-effector: [{:.3f}, {:.3f}, {:.3f}] m".format(*ref_pos))
    print("    m4: full range (-150 to +150)")
    print("    m1,m2,m3,m5,m6: +/-{} deg around reference".format(SECONDARY_RANGE))

    # ---- 3. Compute 7 FK target positions ----
    print("\n[2] Computing 7 circle targets via FK...")
    print("-" * 65)

    # 5 points: original 7-point circle with points 2 and 7 removed
    # (those two were too close to point 1 on the robot)
    m4_from_7 = []
    for i in range(7):
        theta = 360.0 * i / 7
        m4 = theta if theta <= 180 else theta - 360
        m4_from_7.append(round(max(-150, min(150, m4)), 1))
    # Keep indices 0, 2, 3, 4, 5 (remove 1 and 6 = old points 2 and 7)
    m4_values = [m4_from_7[k] for k in [0, 2, 3, 4, 5]]

    targets = []
    for i, m4 in enumerate(m4_values):
        pos = fk_pos(chain, {"m1": 0, "m2": 0, "m3": -90,
                              "m4": m4, "m5": -90, "m6": 0})
        targets.append(pos)
        print("  Pt {}: m4={:+7.1f}  ->  [{:+.4f}, {:+.4f}, {:+.4f}]".format(
            i + 1, m4, *pos))
    print("-" * 65)

    # ---- 4. Solve IK (all 6 motors, bounds enforce simulation behavior) ----
    #
    # Two sweeps to handle the m4=+150/-150 discontinuity:
    #   Forward:  0 -> 1 -> 2 -> 3  (m4: 0 -> +150)
    #   Backward: 0 -> 6 -> 5 -> 4  (m4: 0 -> -150)
    # Each step ~51 deg apart, easy for IK to track.
    # Sequential seeding = same as simulation (IK starts from current config).
    #
    print("\n[3] Solving IK (all 6 motors, position constraint)...")
    print("-" * 65)

    ik_vecs = [None] * NUM_POINTS

    # Forward sweep: 0, 1, 2 (m4: 0 -> +102.9 -> +150)
    seed = ref_ik
    for i in [0, 1, 2]:
        ik_v = solve_ik(chain, targets[i], seed)
        ik_vecs[i] = ik_v
        seed = ik_v

    # Backward sweep: 4, 3 (m4: 0 -> -102.9 -> -150)
    seed = ref_ik
    for i in [4, 3]:
        ik_v = solve_ik(chain, targets[i], seed)
        ik_vecs[i] = ik_v
        seed = ik_v

    # Print all solutions
    solutions = []
    for i in range(NUM_POINTS):
        ang = clamp(ik2poppy(ik_vecs[i]))
        solutions.append(ang)

        achieved = chain.forward_kinematics(ik_vecs[i])[:3, 3]
        err = np.linalg.norm(targets[i] - achieved) * 1000

        flag = "OK" if err < 5 else "WARN" if err < 15 else "FAIL"
        print("  Pt {}: {} err={:5.1f}mm  "
              "m1={:+6.1f} m2={:+6.1f} m3={:+6.1f} m4={:+7.1f} m5={:+6.1f} m6={:+6.1f}".format(
                  i + 1, flag, err,
                  ang["m1"], ang["m2"], ang["m3"],
                  ang["m4"], ang["m5"], ang["m6"]))
    print("-" * 65)

    # ---- 5. Connect to robot ----
    print("\n[4] Connecting to robot...")
    try:
        from poppy_ergo_jr import PoppyErgoJr
    except ImportError:
        from pypot.creatures import PoppyErgoJr

    poppy = PoppyErgoJr(camera="dummy")

    try:
        for m in poppy.motors:
            m.compliant = False
        time.sleep(0.5)
        print("    Motors stiff.")

        # ---- 6. Move to circle-ready pose ----
        print("\n[5] Moving to starting pose...")
        for m in MOTORS:
            getattr(poppy, m).goto_position(ref_poppy[m], 2.0, wait=False)
        time.sleep(3.0)
        print("    Ready.")

        # ---- 7. Random start ----
        idx = random.randint(0, NUM_POINTS - 1)
        print("\n[6] Circle: start at point {}".format(idx + 1))

        # Move to start
        for m in MOTORS:
            getattr(poppy, m).goto_position(solutions[idx][m], MOVE_TIME, wait=False)
        time.sleep(MOVE_TIME + HOLD_TIME)

        # ---- 8. Random walk (exactly like simulation) ----
        print("-" * 65)
        for step in range(NUM_POINTS * 3):
            # Random CW or CCW (same as simulation's sysCall_actuation)
            if random.randint(0, 1) == 1:
                idx = (idx + 1) % NUM_POINTS
            else:
                idx = (idx - 1) % NUM_POINTS

            ang = solutions[idx]
            print("  [{}] Pt {}  m1={:+5.1f} m2={:+5.1f} m3={:+6.1f} "
                  "m4={:+7.1f} m5={:+6.1f} m6={:+5.1f}".format(
                      step + 1, idx + 1,
                      ang["m1"], ang["m2"], ang["m3"],
                      ang["m4"], ang["m5"], ang["m6"]))

            for m in MOTORS:
                getattr(poppy, m).goto_position(ang[m], MOVE_TIME, wait=False)
            time.sleep(MOVE_TIME + HOLD_TIME)

        print("-" * 65)
        print("  Circle complete!")

        # ---- 9. Return home ----
        print("\n[7] Returning to home...")
        for m in poppy.motors:
            m.goto_position(0, 2.5, wait=False)
        time.sleep(3.0)

    finally:
        print("    Releasing motors...")
        for m in poppy.motors:
            m.compliant = True
        time.sleep(0.5)
        print("    Motors released.\n")


if __name__ == "__main__":
    main()
