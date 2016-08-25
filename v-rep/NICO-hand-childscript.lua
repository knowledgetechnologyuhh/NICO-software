-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

if (sim_call_type==sim_childscriptcall_initialization) then
    -- Right finger
    r_indexfingers_handle = simGetObjectHandle('r_indexfingers_x')
    r_finger1_handle_1 = simGetObjectHandle('r_indexfinger_1st_x')
    r_finger1_handle_2 = simGetObjectHandle('r_indexfinger_2nd_x')
    r_finger2_handle_0 = simGetObjectHandle('r_ringfingers_x')
    r_finger2_handle_1 = simGetObjectHandle('r_ringfinger_1st_x')
    r_finger2_handle_2 = simGetObjectHandle('r_ringfinger_2nd_x')

    -- Right thumb
    r_thumb_handle = simGetObjectHandle('r_thumb_x')
    r_thumb1_handle = simGetObjectHandle('r_thumb_1st_x')
    r_thumb2_handle = simGetObjectHandle('r_thumb_2nd_x')

    -- Left finger
    l_indexfingers_handle = simGetObjectHandle('l_indexfingers_x')
    l_finger1_handle_1 = simGetObjectHandle('l_indexfinger_1st_x')
    l_finger1_handle_2 = simGetObjectHandle('l_indexfinger_2nd_x')
    l_finger2_handle_0 = simGetObjectHandle('l_ringfingers_x')
    l_finger2_handle_1 = simGetObjectHandle('l_ringfinger_1st_x')
    l_finger2_handle_2 = simGetObjectHandle('l_ringfinger_2nd_x')

    -- left thumb
    l_thumb_handle = simGetObjectHandle('l_thumb_x')
    l_thumb1_handle = simGetObjectHandle('l_thumb_1st_x')
    l_thumb2_handle = simGetObjectHandle('l_thumb_2nd_x')

end


if (sim_call_type==sim_childscriptcall_actuation) then
    -- Right finger
    local r_finger_pos = simGetJointTargetPosition(r_indexfingers_handle)
    simSetJointTargetPosition(r_finger1_handle_1, r_finger_pos)
    simSetJointTargetPosition(r_finger1_handle_2, r_finger_pos)
    simSetJointTargetPosition(r_finger2_handle_0, r_finger_pos)
    simSetJointTargetPosition(r_finger2_handle_1, r_finger_pos)
    simSetJointTargetPosition(r_finger2_handle_2, r_finger_pos)

    -- Right thumb
    local r_thumb_pos = simGetJointTargetPosition(r_thumb_handle)
    simSetJointTargetPosition(r_thumb1_handle, r_thumb_pos)
    simSetJointTargetPosition(r_thumb2_handle, r_thumb_pos)

    -- Left finger
    local l_finger_pos = simGetJointTargetPosition(l_indexfingers_handle)
    simSetJointTargetPosition(l_finger1_handle_1, l_finger_pos)
    simSetJointTargetPosition(l_finger1_handle_2, l_finger_pos)
    simSetJointTargetPosition(l_finger2_handle_0, l_finger_pos)
    simSetJointTargetPosition(l_finger2_handle_1, l_finger_pos)
    simSetJointTargetPosition(l_finger2_handle_2, l_finger_pos)

    -- left thumb
    local r_thumb_pos = simGetJointTargetPosition(l_thumb_handle)
    simSetJointTargetPosition(l_thumb1_handle, r_thumb_pos)
    simSetJointTargetPosition(l_thumb2_handle, r_thumb_pos)

end


if (sim_call_type==sim_childscriptcall_sensing) then
end


if (sim_call_type==sim_childscriptcall_cleanup) then
end