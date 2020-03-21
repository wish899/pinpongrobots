-- This is a threaded script, and is just an example!


--simRemoteApi.start(19999)

function radians(degree_angle)
    return degree_angle * (math.pi/180)
end

function sysCall_threadmain()
   jointHandles={-1,-1,-1,-1,-1,-1}
  for i=1,6,1 do
       jointHandles[i]=sim.getObjectHandle('UR3_joint'..i)
   end

   -- Set-up some of the RML vectors:
   vel=180
   accel=40
   jerk=80
   currentVel={0,0,0,0,0,0,0}
   currentAccel={0,0,0,0,0,0,0}
maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
   maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
   maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
   targetVel={0,0,0,0,0,0}
   
 targetPos0 = {radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)}
   sim.rmlMoveToJointPositions(jointHandles, -1, currentVel, currentAccel, maxVel, maxAccel, maxJerk, targetPos0, targetVel)
   
   targetPos1 = {radians(90),radians(90
   ),radians(-92),radians(233),radians(33),radians(24)}
   sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos1,targetVel)

   targetPos2 = {radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)}
   sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos2,targetVel)

   targetPos3={0,0,0,0,0,0}
   sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos3,targetVel)
end

-- function sysCall_init() 
--     -- Prepare a floating view with the camera views:
--     cam=sim.getObjectAssociatedWithScript(sim.handle_self)
--     view=sim.floatingViewAdd(0.9,0.3,0.2,0.2,0)
--     sim.adjustView(view,cam,64)
-- end

--  function sysCall_actuation()
    
--     cam_handle = sim.getObjectHandle('Vision_sensor')
--     sim.handleVisionSensor(cam_handle)
--     result, t1, t2, t3 = sim.readVisionSensor(cam_handle)
--     print(result)
 
--  end

-- function sysCall_cleanup() 
--    if sim.isHandleValid(cam)==1 then
--         sim.floatingViewRemove(view)
--     end
-- end 