import matplotlib.pyplot as plt
import numpy as np
import os

path = '/home/xma42/reproduce_results/ls60mm_1_35_tx-3mm_0.2_35_tz-3mm_0.2_03'

demo_traj = np.loadtxt(os.path.join(path, 'demo_traj.txt'))
reproduce_traj = np.loadtxt(os.path.join(path, 'reproduce_traj.txt'))

demo_filtered = demo_traj.shape[1] == 10
reproduce_filtered = reproduce_traj.shape[1] == 10
print(demo_traj.shape)
print(reproduce_traj.shape)
print(demo_filtered)
print(reproduce_filtered)

demo_traj[:, 9 if demo_filtered else 6] = demo_traj[:, 9 if demo_filtered else 6] - demo_traj[0, 9 if demo_filtered else 6]
reproduce_traj[:, 9 if demo_filtered else 6] = reproduce_traj[:, 9 if demo_filtered else 6] - reproduce_traj[0, 9 if demo_filtered else 6]

# Joint states
plt.figure(1)
plt.subplot(211)
plt.plot(demo_traj[:, 1], demo_traj[:, 0], '-*')
plt.plot(reproduce_traj[:, 1], reproduce_traj[:, 0], '-*')
plt.title("Joint states: Template x v.s. Linear stage")
plt.xlabel("Linear Stage, mm")
plt.ylabel("Template X, mm")

plt.subplot(212)
plt.plot(demo_traj[:, 1], demo_traj[:, 2], '-*')
plt.plot(reproduce_traj[:, 1], reproduce_traj[:, 2], '-*')
plt.title("Joint states: Template z v.s. Linear stage")
plt.xlabel("Linear Stage, mm")
plt.ylabel("Template Z, mm")


# Tip positions
plt.figure(2)
plt.subplot(211)
plt.plot(demo_traj[:, 4], demo_traj[:, 6 if demo_filtered else 3], '-*')
plt.plot(reproduce_traj[:, 4], reproduce_traj[:, 3], '-*')
plt.plot(reproduce_traj[:, 1], reproduce_traj[:, 0], 'k')
plt.title("Tip Pisition x v.s. Tip Pisition y")
plt.xlabel("Tip Pisition y, mm")
plt.ylabel("Tip Pisition x, mm")


plt.subplot(212)
plt.plot(demo_traj[:, 4], demo_traj[:, 8 if demo_filtered else 5], '-*')
plt.plot(reproduce_traj[:, 4], reproduce_traj[:, 5], '-*')
plt.plot(reproduce_traj[:, 1], reproduce_traj[:, 2], 'k')

plt.title("Tip Pisition z v.s. Tip Pisition y")
plt.xlabel("Tip Pisition y, mm")
plt.ylabel("Tip Pisition z, mm")

# Tip positions
# plt.figure(3)
# plt.subplot(211)
# plt.plot(demo_traj[:, 4], demo_traj[:, 6 if demo_filtered else 3], '-*')
# plt.plot(reproduce_traj[:, 4], reproduce_traj[:, 6], '-*')
# plt.title("Tip Pisition x v.s. Tip Pisition y")
# plt.xlabel("Tip Pisition y, mm")
# plt.ylabel("Tip Pisition x, mm")
# 
# plt.subplot(212)
# plt.plot(demo_traj[:, 4], demo_traj[:, 8 if demo_filtered else 5], '-*')
# plt.plot(reproduce_traj[:, 4], reproduce_traj[:, 8], '-*')
# plt.title("Tip Pisition z v.s. Tip Pisition y")
# plt.xlabel("Tip Pisition y, mm")
# plt.ylabel("Tip Pisition z, mm")
# 
# plt.figure(4)
# plt.subplot(211)
# plt.plot(demo_traj[:, 9 if demo_filtered else 6], demo_traj[:, 6 if demo_filtered else 3], '-*')
# plt.plot(reproduce_traj[:, 9 if reproduce_filtered else 7], reproduce_traj[:, 3], '-*')
# 
# plt.subplot(212)
# plt.plot(demo_traj[:, 9 if demo_filtered else 6], demo_traj[:, 8 if demo_filtered else 5], '-*')
# plt.plot(reproduce_traj[:, 9 if reproduce_filtered else 7], reproduce_traj[:, 5], '-*')

plt.show()