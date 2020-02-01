import krpc
import time
import numpy as np
import tools
epsilon=1e-8

conn = krpc.connect(name='tour Mun and Minis')
vessel = conn.space_center.active_vessel
center = conn.space_center
print(vessel.name," has been connected.")
#define streams
ut = conn.add_stream(getattr, conn.space_center, 'ut')

altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
periapsis = conn.add_stream(getattr,vessel.orbit,'periapsis_altitude')
time_to_ap = conn.add_stream(getattr,vessel.orbit,'time_to_apoapsis')

position =conn.add_stream(vessel.position,vessel.orbit.body.orbital_reference_frame)

direct_orb = conn.add_stream(getattr,vessel.flight(reference_frame=vessel.orbit.body.orbital_reference_frame),'direction')
velocity_orb = conn.add_stream(getattr,vessel.flight(reference_frame=vessel.orbit.body.orbital_reference_frame),'velocity')
direct_sur = conn.add_stream(getattr,vessel.flight(reference_frame=vessel.orbit.body.reference_frame),'direction')
velocity_sur = conn.add_stream(getattr,vessel.flight(reference_frame=vessel.orbit.body.reference_frame),'velocity')

stage_9_resources = vessel.resources_in_decouple_stage(stage=9, cumulative=False)
stage_9_fuel = conn.add_stream(stage_9_resources.amount, 'SolidFuel')
stage_7_resources = vessel.resources_in_decouple_stage(stage=7, cumulative=False)
stage_7_fuel = conn.add_stream(stage_7_resources.amount, 'SolidFuel')
stage_5_resources = vessel.resources_in_decouple_stage(stage=5, cumulative=False)
stage_5_fuel = conn.add_stream(stage_5_resources.amount, 'LiquidFuel')
stage_3_resources = vessel.resources_in_decouple_stage(stage=3, cumulative=False)
stage_3_fuel = conn.add_stream(stage_3_resources.amount, 'LiquidFuel')
stage_1_resources = vessel.resources_in_decouple_stage(stage=1, cumulative=False)
stage_1_fuel = conn.add_stream(stage_1_resources.amount, 'LiquidFuel')

fuel_list = [stage_5_fuel,stage_3_fuel,stage_1_fuel]
#first two stage solid fuel
vessel.control.sas = True
vessel.control.throttle = 1
time.sleep(1)

#print('Launch!')
print("fuel check stage 9 ",stage_9_fuel())
print("fuel check stage 7 ",stage_7_fuel())
print("fuel check stage 5 ",stage_5_fuel())
print("fuel check stage 3 ",stage_3_fuel())
print("fuel check stage 1 ",stage_1_fuel())

vessel.control.activate_next_stage()
while stage_9_fuel()>0.1:
    time.sleep(0.1)

vessel.control.activate_next_stage()
time.sleep(1.)
vessel.control.activate_next_stage()

while stage_7_fuel()>0.1:
    time.sleep(0.1)

vessel.control.activate_next_stage()
time.sleep(1.)
vessel.control.activate_next_stage()

vessel.auto_pilot.sas_mode = conn.space_center.SASMode.prograde

#stop at efficient ap_altitude
while apoapsis()<3.6e7:
    time.sleep(0.1)
    if fuel_list[0]()<0.1:
        print(fuel_list[0]())
        fuel_list.pop(0)
        vessel.control.activate_next_stage()
        vessel.control.activate_next_stage()

vessel.control.throttle = 0

dt = time_to_ap()-40
conn.space_center.warp_to(ut()+dt)

#compute direction of heading
vessel.auto_pilot.engage()
vessel.auto_pilot.reference_frame=vessel.orbit.body.orbital_reference_frame

dv_dir,loss = tools.dir_by_ap_pe(3.6e7,3.1e7,apoapsis,periapsis,position,velocity_orb,vessel,conn)

vessel.auto_pilot.target_direction=tuple(dv_dir)
vessel.auto_pilot.wait()
#high stable orbit
vessel.control.throttle = 1

while loss>0.05:
    time.sleep(0.05)
    if fuel_list[0]()<0.1:
        print(fuel_list[0]())
        fuel_list.pop(0)
        vessel.control.activate_next_stage()
        vessel.control.activate_next_stage()
    
    dv_dir,loss = tools.dir_by_ap_pe(3.6e7,3.1e7,apoapsis,periapsis,position,velocity_orb,vessel,conn)
    vessel.auto_pilot.target_direction=tuple(dv_dir)
    vessel.control.throttle = np.min([loss,1.])
    print("current loss is ",loss)
    vessel.auto_pilot.wait()

vessel.control.throttle = 0