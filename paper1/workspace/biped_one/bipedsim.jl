using RigidBodyDynamics
# 1. load模型
function loadModel(path)
    urdf_path = "C:\\Users\\thinkpad\\OneDrive\\02-code\\JuliaRobotics\\RigidBodyDynamics.jl-master\\test\\urdf\\biped_No1.urdf"
    mechanism = parse_urdf(urdf_path)
    # 模型添加碰撞点

end
# 2. PID控制器

# 3. 机器人的力控制

# 机器人初始状态设置

# 4. 控制器
function biped_run_controller!(torques::AbstractVector, t, state::MechanismState)
    for i in 1:length(torques)
        torques[i] = 0
    end
    # 获取接触信息
    append!(contact_out, state.contact_states[BodyID(2)])
    # 获取当前状态

    # 根据当前状态调整规划数据（动态调整）

    # 对底层输出控制扭矩（optimize）
end
function biped_walk_controller!(torques::AbstractVector, t, state::MechanismState)
    for i in 1:length(torques)
        torques[i] = 0
    end
    append!(sin_out, sin(t))   # 测试能否输出，虽然比较蠢
    # 获取接触信息
    append!(contact_out, state.contact_states[BodyID(2)])
end
# 5. 轨迹规划
# ***************************************************************
# 说明：
#    a. 处于空中&没有规划==>触发规划
#    b. 获取轨迹随时间的曲线
#    c. 输出

# 6. 优化
