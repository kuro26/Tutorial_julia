using RigidBodyDynamics

# 路径
# 1. pwd(),一般不指向文件所在路径
# 2. 
urdf = "C:\\Users\\steve\\OneDrive\\02-code\\JuliaRobotics\\RigidBodyDynamics.jl-master\\test\\urdf\\biped_No1.urdf"
mechanism = parse_urdf(urdf)

# 获取关节，并设计控制器
left_j1, right_j1, left_j2, right_j2, left_j3, right_j3 = joints(mechanism)
function no_control!(torques::AbstractVector, t, state::MechanismState)
    torques[velocity_range(state, left_j1)] .= 0
    torques[velocity_range(state, left_j2)] .= 0
    torques[velocity_range(state, left_j3)] .= 0
    torques[velocity_range(state, right_j1)] .= 0
    torques[velocity_range(state, right_j2)] .= 0
    torques[velocity_range(state, right_j3)] .= 0
end

# 设置初始状态
state = MechanismState(mechanism)
zero_velocity!(state)
set_configuration!(state, left_j1, 0.1)
set_configuration!(state, left_j2, 0.2)
set_configuration!(state, left_j3, 0.1)
set_configuration!(state, right_j1, 0.1)
set_configuration!(state, right_j2, 0.2)
set_configuration!(state, right_j3, 0.1)


# 开始仿真
final_time = 10.
ts, qs, vs = simulate(state, final_time, no_control!; Δt = 1e-3);


# 绘图处理
using MeshCatMechanisms

mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf));
open(mvis)

MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.);





