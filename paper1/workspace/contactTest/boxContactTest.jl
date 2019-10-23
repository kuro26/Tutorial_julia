# -----------------------------------------------------------------------
# version 1
# 说明：本例子中会出现NaN的情况，作者回复可能是因为地面刚度参数比较大，导致计算不稳定
#       或者可以考虑用DifferentialEquations.jl库中的Tsit5变步长积分来替代作者的定步长

using RigidBodyDynamics
using Rotations
# 定义半空间
# frame = CartesianFrame3D()
# point = Point3D(frame, 1.0, 1.0, 1.0)
# normal = FreeVector3D(frame, 0., 0., 1.)
# halfspace = RigidBodyDynamics.Contact.HalfSpace3D(point, normal)

# # 定义接触环境
# con_env = RigidBodyDynamics.Contact.ContactEnvironment{Float64}()
# push!(con_env, halfspace)

#urdf = "C:\\Users\\thinkpad\\OneDrive\\02-code\\JuliaRobotics\\RigidBodyDynamics.jl-master\\test\\urdf\\singleBox.urdf"
urdf = "C:\\Users\\thinkpad\\OneDrive\\02-code\\JuliaRobotics\\RigidBodyDynamics.jl-master\\test\\urdf\\singleBox.urdf"
mechanism = parse_urdf(urdf, floating=true)      # 解析为浮动

# 向mechanism结构体中添加接触点
box = bodies(mechanism)[2]        # bodies中第一个是world
Nmdl = RigidBodyDynamics.Contact.hunt_crossley_hertz()          # 法向碰撞模型
Fmdl = RigidBodyDynamics.Contact.ViscoelasticCoulombModel(0.3, 30e3,0.3)     # 摩擦模型，库伦
scm = RigidBodyDynamics.Contact.SoftContactModel(Nmdl, Fmdl)    # 创建接触模型
for i in [-0.1, 0.1]
    for j in [-0.1, 0.1]
        for k in [-0.1, 0.1]
            point = Point3D(default_frame(box), i, j, k)
            # 在刚体上添加接触点
            cp = RigidBodyDynamics.Contact.ContactPoint(point, scm)
            # 修改mechanism的数据结构，添加接触点
            add_contact_point!(bodies(mechanism)[2], cp)
        end
    end
end

# 添加半空间地形
# 这里就不能自己新建frame
frame = default_frame(bodies(mechanism)[1])
point = Point3D(frame, 0.0, 0.0, 0.0)
normal = FreeVector3D(frame, 0., 0., 1.)
halfspace = RigidBodyDynamics.Contact.HalfSpace3D(point, normal)

push!(mechanism.environment.halfspaces, halfspace)

# 控制函数，嵌入到仿真的过程
# sin_out = []
# time_out = []
# contact_out = []
function no_control!(torques::AbstractVector, t, state::MechanismState)
    for i in 1:length(torques)
        torques[i] = 0
    end
    # append!(sin_out, sin(t))   # 测试能否输出，虽然比较蠢
    # 获取接触信息
    # append!(contact_out, state.contact_states[BodyID(2)])
end

# box设置初始条件
floating_joint = joints(mechanism)[1]
state = MechanismState(mechanism)
zero_velocity!(state)
# 这个函数在文件mechanism_state中，因为目标是修改状态
rot = RotXYZ{Float64}(0.2, 0.4, 0.6)
quat = convert(Quat{Float64}, rot)
set_configuration!(state, floating_joint, [quat.w, quat.x, quat.y, quat.z, 1.0, 1.0, 2])
# set_configuration!(state, floating_joint, )
# 这个from和to设计在translation中我觉得很失败
# from = default_frame(bodies(mechanism)[1])
# to = default_frame(bodies(mechanism)[2])
# trans = Transform3D(from, to, )
# set_translation!()

final_time = 5
ts, qs, vs = simulate(state, final_time, no_control!; Δt = 1e-5);


# using Plots
# plot(sin_out)

using MeshCatMechanisms
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf));
open(mvis)
MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.);