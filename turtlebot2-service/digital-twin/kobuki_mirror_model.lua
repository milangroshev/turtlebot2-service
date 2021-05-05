-- #########################################################################
--                      Turtlebot2 Main Control Script
-- #########################################################################
-- -------------------------------------------------------------------------
-- Ivan Fernandez-Vega
-- B.Sc. Degree in Computer Engineering
-- University of Malaga
-- June, 2016
-- -------------------------------------------------------------------------
--   This program is free software: you can redistribute it and/or modify
--   it under the terms of the GNU General Public License as published by
--   the Free Software Foundation, either version 3 of the License, or
--   (at your option) any later version.
--
--   This program is distributed in the hope that it will be useful,
--   but WITHOUT ANY WARRANTY; without even the implied warranty of
--   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--   GNU General Public License for more details.
--
--   You should have received a copy of the GNU General Public License
--   along with this program.  If not, see <http://www.gnu.org/licenses/>.
-- -------------------------------------------------------------------------

if (sim_call_type == sim.syscb_init) then 

    -- CONSTANTS -- 
        -- Simulation
        remoteAPI = sim.getScriptSimulationParameter(sim.handle_self,'remoteAPIEnabled')
 
        -- Topics emulation
        local controller_type = 0
        local controller_p    = 100.000007629
        local controller_i    = 0.10000000149
        local controller_d    = 2.0
        local hardware_ver    = {1, 0, 4}
        local firmware_ver    = {1, 2, 0}
        local software_ver    = {0, 6, 0}
        local udid            = {98434864, 959859523, 1126263088}
        local features        = 3

        -- LED color constants
        blackLed  = {0.4, 0.4, 0.4}
        greenLed  = {0, 1, 0}
        orangeLed = {1, 0.5, 0}
        redLed    = {1, 0, 0}
        lastCommand = -1

    -- VARIABLES --
        lastTimeActuation   = 0
        previousRWPosition  = 0
        previousLWPosition  = 0
        totalRWPosition     = 0
        totalLWPosition     = 0
        lastSpeedCommand    = 0
        lastSpeedCommandCtr = 0    
        led_1_old_state     = 0
        led_2_old_state     = 0
        button_0            = 0
        button_1            = 0    
        button_2            = 0
    
    -- Model's associated parameters
    turtleWidowX  = sim.getObjectAssociatedWithScript(sim.handle_self)
    modelBase     = sim.getObjectHandle('Turtlebot2')
    modelBaseName = sim.getObjectName(modelBase)

    -- HANDLES -- 
        -- Kobuki Wheels
            t_rightWheel  = sim.getObjectHandle('wheel_right_joint')
            t_leftWheel   = sim.getObjectHandle('wheel_left_joint')
        -- Kobuki LEDs
            t_statusLed   = sim.getObjectHandle('status_led')
            t_led_1       = sim.getObjectHandle('led_1')
            t_led_2       = sim.getObjectHandle('led_2')
        -- Kobuki Bumpers
            t_frontBumper = sim.getObjectHandle('bumper_front_joint')
            t_rightBumper = sim.getObjectHandle('bumper_right_joint')
            t_leftBumper  = sim.getObjectHandle('bumper_left_joint')
        -- Kobuki Cliff Sensors
            t_frontCliff  = sim.getObjectHandle('cliff_front_sensor')
            t_rightCliff  = sim.getObjectHandle('cliff_right_sensor')
            t_leftCliff   = sim.getObjectHandle('cliff_left_sensor')
        -- Kobuki Wheel drop sensors
            t_rightWheelDrop = sim.getObjectHandle('wheel_right_drop_sensor')
            t_leftWheelDrop  = sim.getObjectHandle('wheel_left_drop_sensor')
        -- Kobuki Gyroscope
            ref                     = sim.getObjectHandle('gyro_link_visual')
            oldTransformationMatrix = sim.getObjectMatrix(ref, -1)
            lastTime                = sim.getSimulationTime()
        -- Kinect
            if(sim.getBoolParameter(sim.boolparam_vision_sensor_handling_enabled) == true) then
                depthCam  = sim.getObjectHandle('kinect_depth')
                depthView = sim.floatingViewAdd(0.9, 0.9, 0.2, 0.2, 0)
                sim.adjustView(depthView, depthCam, 64)
                colorCam  = sim.getObjectHandle('kinect_rgb')
                colorView = sim.floatingViewAdd(0.69, 0.9, 0.2, 0.2, 0)
                sim.adjustView(colorView, colorCam, 64)
           end

    -- SIGNALS --
        -- Sensing
            -- All sensors
                sim.setStringSignal(modelBaseName..'_kobuki_sensor_state', 'empty')
            -- Pose and Quaternion
                sim.setStringSignal(modelBaseName..'_kobuki_quaternion_vel_accel', 'empty')
            -- Joint states
                sim.setStringSignal(modelBaseName..'_joint_states', 'empty')
            -- Simulation Time
                sim.setFloatSignal(modelBaseName..'_simulation_time', 0)
            -- Odometry
                sim.setIntegerSignal(modelBaseName..'_kobuki_odometry_reset', 0)
        -- Actuation
            -- Kobuki Wheels
                sim.setIntegerSignal(modelBaseName..'_kobuki_motors_enabled', 1)
                wheelSpeedData = sim.packFloatTable({0, 0, 0}, 0, 3)
                sim.setStringSignal(modelBaseName..'_kobuki_wheels_speed', wheelSpeedData) 
            -- Kobuki LEDs
                sim.setIntegerSignal(modelBaseName..'_kobuki_led_1', 0)
                sim.setIntegerSignal(modelBaseName..'_kobuki_led_2', 0)
            -- Other
                local controllerInfoData = sim.packFloatTable({controller_type, controller_p, controller_i, controller_d}, 0, 4)
                sim.setStringSignal(modelBaseName..'_kobuki_controller_info', controllerInfoData)
                local versionInfoData = sim.packFloatTable({hardware_ver[1], hardware_ver[2], hardware_ver[3], 
                                                        firmware_ver[1], firmware_ver[2], firmware_ver[3], 
                                                        software_ver[1], software_ver[2], software_ver[3], 
                                                        udid[1], udid[2], udid[3], features}, 0, 13);
                sim.setStringSignal(modelBaseName..'_kobuki_version_info', controllerInfoData)

    -- USER INTERFACES --
        -- Buttons
            buttons_ui = simGetUIHandle('Kobuki_Buttons_UI')       
        -- Bumpers
            bumpers_ui = simGetUIHandle('Kobuki_Bumper_sensor_UI')
        -- Cliff Sensors 
            cliff_ui = simGetUIHandle('Kobuki_Cliff_sensor_UI')
        -- Wheel drop
            wheeldrop_ui = simGetUIHandle('Kobuki_Wheel_drop_UI')
        -- Odometry
            odometry_ui = simGetUIHandle('Kobuki_Odometry_UI')
        -- Gyroscope
            gyro_ui  = simGetUIHandle('Kobuki_Gyro_sensor_UI')

    -- INITIALIZE LEDs --
            sim.setShapeColor(t_statusLed, nil, 0, greenLed)
            sim.setShapeColor(t_led_1,     nil, 0, blackLed)
            sim.setShapeColor(t_led_2,     nil, 0, blackLed)        
    
    -- REMOTE API --
            if(remoteAPI) then
                simRemoteApi.start(19999, 1300, false, false)
            end

    -- SAVING INITIAL POSITION AND ORIENTATION --
        originPosition    = sim.getObjectPosition(ref, -1)
        originOrientation = oldTransformationMatrix    

    -- INITIAL SIMULATED POSITION AND ORIENTATION --
        pose_x       = 0
        pose_y       = 0
        pose_tita    = 0
        wheel_bias   = 0.23
        wheel_radius = 0.035
        lastScriptCallTime = sim.getSimulationTime()
end    

if (sim_call_type == sim.syscb_cleanup) then 

    -- REMOTE API --
        if(remoteAPI) then
            simRemoteApi.stop(19999) 
        end
      
    -- Shutdown LEDs --
       sim.setShapeColor(t_statusLed, nil, 0, blackLed)
       sim.setShapeColor(t_led_1,     nil, 0, blackLed)
       sim.setShapeColor(t_led_2,     nil, 0, blackLed)
end 

if (sim_call_type == sim.syscb_sensing) then 
    
    -- KOBUKI --
        -- Gyroscope
            local transformationMatrix = sim.getObjectMatrix(ref, -1)
            local oldInverse           = simGetInvertedMatrix(oldTransformationMatrix)
            local m                    = sim.multiplyMatrices(oldInverse, transformationMatrix)
            local euler                = sim.getEulerAnglesFromMatrix(m)
            local currentTime          = sim.getSimulationTime()

            if(currentTime - lastTime > 0.25) then
                local gyroData = {0,0,0}
                local dt       = currentTime - lastTime
                if (dt ~= 0) then
                    gyroData[1] = euler[1] / dt
                    gyroData[2] = euler[2] / dt
                    gyroData[3] = euler[3] / dt
                end

                simSetUIButtonLabel(gyro_ui, 3, string.format("X-Gyro: %.2f dps", ((gyroData[1] * 180) / math.pi)))
                simSetUIButtonLabel(gyro_ui, 4, string.format("Y-Gyro: %.2f dps", ((gyroData[2] * 180) / math.pi)))
                simSetUIButtonLabel(gyro_ui, 5, string.format("Z-Gyro: %.2f dps", ((gyroData[3] * 180) / math.pi)))

                oldTransformationMatrix = sim.copyMatrix(transformationMatrix)
                lastTime = currentTime
            end
        
        -- Orientation
            oldOrientationInverse     = simGetInvertedMatrix(originOrientation)
            matrix_quat               = sim.multiplyMatrices(oldOrientationInverse, transformationMatrix)
            quaternion                = sim.getQuaternionFromMatrix(matrix_quat)    

        -- Speed
            linearSpeed, angularSpeed = sim.getObjectVelocity(ref)
        
        -- Acceleration
            linearAcceleration = {0, 0, 0}    -- (No accelerometer implemented)
   
        -- Wheel Odometry
            -- Right
            dr = sim.getJointPosition(t_rightWheel) - previousRWPosition
            if (dr >= 0) then
                dr_aux = math.mod(dr + math.pi, 2 * math.pi) - math.pi
            else
                dr_aux = math.mod(dr - math.pi,2 * math.pi) + math.pi
            end
            totalRWPosition    = totalRWPosition + dr_aux
            previousRWPosition = sim.getJointPosition(t_rightWheel)
            RWdegrees = (180 * totalRWPosition) / math.pi
        
            RWTicks = RWdegrees * (2578.333869740464 / 360)
            RWTicks = math.mod(RWTicks, 65535)
            if(RWTicks < 0) then
                RWTicks = RWTicks + 65535
            end

            simSetUIButtonLabel(odometry_ui, 6, string.format("Right: %d", RWTicks))

            -- Left
            dl = sim.getJointPosition(t_leftWheel) - previousLWPosition
            if (dl >= 0) then
                dl_aux = math.mod(dl + math.pi, 2 * math.pi) - math.pi
            else
                dl_aux = math.mod(dl - math.pi, 2 * math.pi) + math.pi
            end
            totalLWPosition    = totalLWPosition + dl_aux


            previousLWPosition = sim.getJointPosition(t_leftWheel)
            LWdegrees          = (180 * totalLWPosition) / math.pi
            LWTicks = LWdegrees * (2578.333869740464 / 360)
            LWTicks = math.mod(LWTicks, 65535)
            if(LWTicks < 0) then
                LWTicks = LWTicks + 65535
            end

            simSetUIButtonLabel(odometry_ui, 7, string.format("Left: %d", LWTicks))


        -- Simulated position and orientation
            currentTime = sim.getSimulationTime()
            if(currentTime > 0.5) then
                local t_inc = (currentTime - lastScriptCallTime) 
                lastScriptCallTime = currentTime
                local rightWheel_vel = (dr_aux / t_inc) * wheel_radius
                local leftWheel_vel  = (dl_aux / t_inc) * wheel_radius

                pose_x    = pose_x    + (t_inc * ((rightWheel_vel + leftWheel_vel) / 2) * math.cos(pose_tita))
                pose_y    = pose_y    + (t_inc * ((rightWheel_vel + leftWheel_vel) / 2) * math.sin(pose_tita))
                euler_orientation = sim.getEulerAnglesFromMatrix(matrix_quat)
                pose_tita = euler_orientation[3]    -- From gyroscope
            end
        
        -- Front Bumper
            front_bumper_pos = sim.getJointPosition(t_frontBumper)
            if(front_bumper_pos < -0.001) then
                simSetUIButtonLabel(bumpers_ui, 6, string.format("F. COLLISION!"))
                bumperCenterState = 1
            else
                simSetUIButtonLabel(bumpers_ui, 6, string.format("F. No Collision"))
                bumperCenterState = 0
            end
    
        -- Right Bumper
            right_bumper_pos = sim.getJointPosition(t_rightBumper)
            if(right_bumper_pos < -0.001) then
                simSetUIButtonLabel(bumpers_ui, 7, string.format("R. COLLISION!"))
                bumperRightState = 1
            else
                simSetUIButtonLabel(bumpers_ui, 7, string.format("R. No Collision"))
                bumperRightState = 0
            end

        -- Left Bumper
            left_bumper_pos = sim.getJointPosition(t_leftBumper)
            if(left_bumper_pos < -0.001) then
                simSetUIButtonLabel(bumpers_ui, 8, string.format("L. COLLISION!"))
                bumperLeftState = 1
            else
                simSetUIButtonLabel(bumpers_ui, 8, string.format("L. No Collision"))
                bumperLeftState = 0
            end
    
        -- Front Cliff
            resultFC, distanceFC = sim.readProximitySensor(t_frontCliff)
            if(resultFC == 1) then
                simSetUIButtonLabel(cliff_ui, 6, string.format("Front: %.4f", distanceFC))
                frontCliffState = 0
            else
                simSetUIButtonLabel(cliff_ui, 6, string.format("Front: CLIFF!"))
                frontCliffState = 1
                distanceFC = 4096
            end

        -- Right Cliff
            resultRC, distanceRC = sim.readProximitySensor(t_rightCliff)
            if(resultRC == 1) then
                simSetUIButtonLabel(cliff_ui, 7, string.format("Right: %.4f", distanceRC))
                rightCliffState = 0
            else
                simSetUIButtonLabel(cliff_ui, 7, string.format("Right: CLIFF!"))
                rightCliffState = 1
                distanceRC = 4096
            end

        -- Left Cliff
            resultLC, distanceLC = sim.readProximitySensor(t_leftCliff)
            if(resultLC == 1) then
                simSetUIButtonLabel(cliff_ui, 8, string.format("Left: %.4f", distanceLC))
                leftCliffState = 0
            else
                simSetUIButtonLabel(cliff_ui, 8, string.format("Left: CLIFF!"))
                leftCliffState = 1
                distanceLC = 4096
            end

        -- Wheel Drop
            -- Right
            wheel_drop_right_pos = sim.getJointPosition(t_rightWheelDrop)
            if(wheel_drop_right_pos < -0.001) then
                simSetUIButtonLabel(wheeldrop_ui, 6, string.format("Right: DROP!"))
                wheel_drop_right_state = 1
            else
                simSetUIButtonLabel(wheeldrop_ui, 6, string.format("Right: OK"))
                wheel_drop_right_state = 0
            end
            
            -- Left
            wheel_drop_left_pos = sim.getJointPosition(t_leftWheelDrop)
            if(wheel_drop_left_pos < -0.001) then
                simSetUIButtonLabel(wheeldrop_ui, 7, string.format("Left: DROP!"))
                wheel_drop_left_state = 1
            else
                simSetUIButtonLabel(wheeldrop_ui, 7, string.format("Left: OK"))
                wheel_drop_left_state = 0
            end

        -- Buttons
            button = simGetUIEventButton(buttons_ui)
            if (button == 3) then
                if(button_0 == 0) then
                    button_0 = 1
                else
                    button_0 = 0
                end
            elseif (button == 4) then
                if(button_1 == 0) then
                    button_1 = 1
                else
                    button_1 = 0
                end
            elseif (button == 5) then
                if(button_2 == 0) then
                    button_2 = 1
                else
                    button_2 = 0
                end
            end


        -- SIMULATION TIME --
        simTime = sim.getSimulationTime()

        -- JOINT STATES --
        wlj_pos = sim.getJointPosition(t_leftWheel);  _, wlj_vel = sim.getObjectFloatParameter(t_leftWheel, sim.jointfloatparam_velocity);  wlj_eff = -1;
        wrj_pos = sim.getJointPosition(t_rightWheel); _, wrj_vel = sim.getObjectFloatParameter(t_rightWheel, sim.jointfloatparam_velocity); wrj_eff = -1;
            


        -- PACK AND UPDATE SIGNALS --
        local sensorsData        = sim.packFloatTable({simTime, bumperRightState, bumperCenterState, 
                                            bumperLeftState, wheel_drop_right_state, 
                                            wheel_drop_left_state, rightCliffState, 
                                            frontCliffState, leftCliffState, distanceRC, 
                                            distanceFC, distanceLC, RWTicks, 
                                            LWTicks, button_0, button_1, button_2}, 0, 17);

        local poseQuaternionData = sim.packFloatTable({pose_x, pose_y, 0,
                                            quaternion[1],  quaternion[2], quaternion[3], 
                                            quaternion[4],  linearSpeed[1], 
                                            linearSpeed[2], linearSpeed[3], 
                                            angularSpeed[1], angularSpeed[2], 
                                            angularSpeed[3], linearAcceleration[1], 
                                            linearAcceleration[2], linearAcceleration[3]},
                                            0, 16);    
        local jointStatesData    = sim.packFloatTable({wlj_pos, wlj_vel, wlj_eff, wrj_pos, wrj_vel, wrj_eff}, 0,  6);

        sim.setStringSignal(modelBaseName..'_kobuki_sensor_state',sensorsData)
        sim.setStringSignal(modelBaseName..'_kobuki_quaternion_vel_accel', poseQuaternionData)
        sim.setStringSignal(modelBaseName..'_joint_states', jointStatesData)
        sim.setFloatSignal (modelBaseName..'_simulation_time', simTime)      
      
end
 

if (sim_call_type == sim.syscb_actuation) then 
    
    -- KOBUKI WHEELS --
    if(sim.getIntegerSignal(modelBaseName..'_kobuki_motors_enabled') == 1) then
        wheelSpeedData = sim.getStringSignal(modelBaseName..'_kobuki_wheels_speed')
        wheelSpeed     = sim.unpackFloatTable(wheelSpeedData, 0, 3, 0)
        sim.setJointTargetVelocity(t_rightWheel, wheelSpeed[1])
        sim.setJointTargetVelocity(t_leftWheel,  wheelSpeed[2])
        print(wheelSpeed[3])
        if(wheelSpeed[3] ~= lastSpeedCommandCtr) then
            lastSpeedCommand = sim.getSimulationTime()
            lastSpeedCommandCtr = wheelSpeed[3]
        end


        if(wheelSpeed[1] ~= 0 or wheelSpeed[2] ~= 0) then
               if(sim.getSimulationTime() - lastSpeedCommand > 0.6) then
                    sim.setJointTargetVelocity(t_rightWheel, 0)
                    sim.setJointTargetVelocity(t_leftWheel,  0)
               end
        end
    else
        sim.setJointTargetVelocity(t_rightWheel, 0)
        sim.setJointTargetVelocity(t_leftWheel,  0)
    end
    
    -- KOBUKI ODOMETRY RESET --
    if(sim.getIntegerSignal(modelBaseName..'_kobuki_odometry_reset') == 1) then
        originPosition      = sim.getObjectPosition(ref, -1)
        originOrientation   = sim.getObjectMatrix(ref, -1)
        previousRWPosition  = sim.getJointPosition(t_rightWheel)
        previousLWPosition  = sim.getJointPosition(t_leftWheel)
        totalRWPosition     = 0
        totalLWPosition     = 0
        pose_x              = 0
        pose_y              = 0
        pose_tita           = 0
        sim.setIntegerSignal(modelBaseName..'_kobuki_odometry_reset', 0)
    end

    -- KOBUKI LEDs --
    local led_1_new_state = sim.getIntegerSignal(modelBaseName..'_kobuki_led_1')
    if(led_1_new_state ~= led_1_old_state) then    
        if     (led_1_new_state == 0) then sim.setShapeColor(t_led_1, nil, 0, blackLed)
        elseif (led_1_new_state == 1) then sim.setShapeColor(t_led_1, nil, 0, greenLed)
        elseif (led_1_new_state == 2) then sim.setShapeColor(t_led_1, nil, 0, orangeLed)
        elseif (led_1_new_state == 3) then sim.setShapeColor(t_led_1, nil, 0, redLed)
        end
        led_1_old_state = led_1_new_state
    end

    local led_2_new_state = sim.getIntegerSignal(modelBaseName..'_kobuki_led_2')
    if(led_2_new_state ~=led_2_old_state) then   
        if     (led_2_new_state == 0) then sim.setShapeColor(t_led_2, nil, 0, blackLed)
        elseif (led_2_new_state == 1) then sim.setShapeColor(t_led_2, nil, 0, greenLed)
        elseif (led_2_new_state == 2) then sim.setShapeColor(t_led_2, nil, 0, orangeLed)
        elseif (led_2_new_state == 3) then sim.setShapeColor(t_led_2, nil, 0, redLed)
        end
        led_2_old_state = led_2_new_state
    end
end

if (sim_call_type==sim_childscriptcall_actuation) then
    
    if simROS then
     print("<font color='#0F0'>ROS interface was found.</font>@html")
    else
    print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    print("Is the ros master ready and reachable?")
    end
    
    -- Now move!
    local wheelbase  = 0.23
    local R = 0.035
    local linearX = -0.2
    local angularZ = 0.1
    local test = 4
    

    if(lastCommand==-1) then
    lastCommand = 1
    else
    lastCommand = lastCommand +1%10241024
    print(lastCommand)
    end
    
    
    leftMotorSpeed  = (linearX - (wheelbase / 2) * angularZ) / R
    rightMotorSpeed = (linearX + (wheelbase / 2) * angularZ) / R 
    wheelSpeedData = sim.packFloatTable({leftMotorSpeed,rightMotorSpeed,lastCommand},0,3)
    sim.setStringSignal(modelBaseName..'_kobuki_wheels_speed', wheelSpeedData)
end
