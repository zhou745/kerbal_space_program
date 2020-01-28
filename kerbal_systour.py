import krpc
import time

conn = krpc.connect(name='tour Mun and Minis')
vessel = conn.space_center.active_vessel
center = conn.space_center
print(vessel.name," has been connected.")

#define streams



#first two stage solid fuel
vessel.auto_pilot.target_pitch_and_heading(90, 90)
vessel.auto_pilot.engage()
vessel.control.throttle = 1
time.sleep(1)

print('Launch!')
vessel.control.activate_next_stage()

