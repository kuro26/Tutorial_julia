# Goal：generate SLIP motion trajectory
# 1. seperate the control law
# 2. 
using DifferentialEquations, ParameterizedFunctions
using Plots

function calc_foot_end(u, p)
    """ calculate the foot position in air """
    x, y, z = u[1:3]
    l0, α, β = p[2:4]
    x_pf = x + l0*cos(β)sin(α)
    y_pf = y + l0*sin(β)
    z_pf = z - l0*cos(β)cos(α)
    return (x_pf, y_pf, z_pf)
end

# --------------air------------------------
# p = [g, l0, α, β]
function diff_fun_fly!(du, u, p, t)
    """ Differential Equations in air"""
    x, y, z, vx, vy, vz = u
    g = p[1]
    du[1], du[2], du[3] = dx, dy, dz = vx, vy, vz
    du[4], du[5], du[6] = 0.0, 0.0, -g 
end

cond_contact(u, t, integrator) = calc_foot_end(u, integrator.p)[3]  # positive-->negative
affect_contact!(integrator) = terminate!(integrator)
cond_peak(u, t, integrator) = u[6]

# -------------ground control------------------------
function is_compressing(u, p)
    """ decide if on compressing """
    x, y, z, vx, vy, vz = u
    x0, y0, z0 = p[6:8]
    vec1 = transpose([x-x0, y-y0, z])  # ground-->mass point
    vec2 = [vx, vy, vz]                # velocity vector
    return vec1*vec2<=0 ? true : false
end
function k1k2Force(u, p)
    """ Control law1: using fixed k1, k2 model """
    k1, k2, x0, y0, z0 = p[4:8]
    l0 = p[3]
    x, y, z= u[1:3]
    l = sqrt((x-x0)^2 + (y-y0)^2 + (z-0)^2)
    k = is_compressing(u, p) ? k1 : k2

    fx = - k * (x - (x0 - l0*(x0-x)/l))
    fy = - k * (y - (y0 - l0*(y0-y)/l))
    fz = - k * (z - ( 0 - l0*(z0-z)/l))
    return (fx, fy, fz)
end
# -------------ground simulation------------------------
# p2 = [m, g, l0, k1, k2, x0, y0, z0]
function diff_fun_support!(du, u, p, t)
    """ Differential Equations on ground """
    m, g = p[1:2]
    x, y, z, vx, vy, vz = u
    
    fx, fy, fz = k1k2Force(u, p)          # force specification
    ax, ay, az = fx/m, fy/m, (fz-m*g)/m
    du[1], du[2], du[3] = vx, vy, vz
    du[4], du[5], du[6] = ax, ay, az
end

function cond_compressed(u, t, integrator)
    x, y, z, vx, vy, vz = u
    x0, y0, z0 = integrator.p[6:8]
    vec1 = transpose([x-x0, y-y0, z])  # ground-->mass point
    vec2 = [vx, vy, vz]                # velocity vector
    return vec1*vec2                   # negative-->positive
end

function cond_leave(u, t, integrator)
    """ Condition: Leave ground """
    x, y, z = u[1:3]
    x0, y0, z0 = integrator.p[6:8]
    l = sqrt((x-x0)^2 + (y-y0)^2 + (z-z0)^2)
    return (l - integrator.p[3])
end


# -------------Period------------------------
# simulation of a period, usually 3 segments;
# s0, [z0, vx0, vy0], initial condition
# ctl, [α, β, k1, k2]
function cycleCalc(m, l0, s0, ctrl)
    α, β, k1, k2 = ctrl
    z0, vx0, vy0 = s0
    u0 = [0.0, 0.0, z0, vx0, vy0, 0.0]
    tspan = (0.0, 4.0)
    p1 = (9.8, l0, α, β, k1, k2)
    # >>segment 1, air-->contact
    prob = ODEProblem(diff_fun_fly!, u0, tspan, p1)
    cb = ContinuousCallback(cond_contact, affect_contact!)
    sol1 = solve(prob, Tsit5(), callback=cb)
    # >>segment 2
    u0 = sol1.u[end]    
    x0, y0, z0 = calc_foot_end(u0, p1)
    p2 = [m, 9.8, l0, k1, k2, x0, y0, z0]
    prob = ODEProblem(diff_fun_support!, u0, tspan, p2)
    cb = ContinuousCallback(cond_compressed, terminate!)
    sol2 = solve(prob, Tsit5(), callback=cb)

    # >>segment 3
    u0 = sol2.u[end]
    p3 = p2
    prob = ODEProblem(diff_fun_support!, u0, tspan, p3)
    cb = ContinuousCallback(cond_leave, terminate!, abstol=1e-9)
    sol3 = solve(prob, Tsit5(), callback=cb)
    # >>segment 4
    u0 = sol3.u[end]
    p4 = p1
    prob = ODEProblem(diff_fun_fly!, u0, tspan, p4)
    cb = ContinuousCallback(cond_peak, terminate!)
    sol4 = solve(prob, Tsit5(), callback=cb)
    # 综合
    return sol1, sol2, sol3, sol4
end

function test1()
    m = 20; l0 = 0.8;
    s0 = [1.0, 1.0, 0.2]
    ctrl = [pi/8, pi/9, 1e3, 1e3]
    sol1, sol2, sol3, sol4 = cycleCalc(m, l0, s0, ctrl)
    x1, y1, z1 = [u[1] for u in sol1.u], [u[2] for u in sol1.u], [u[3] for u in sol1.u]
    x2, y2, z2 = [u[1] for u in sol2.u], [u[2] for u in sol2.u], [u[3] for u in sol2.u]
    x3, y3, z3 = [u[1] for u in sol3.u], [u[2] for u in sol3.u], [u[3] for u in sol3.u]
    x4, y4, z4 = [u[1] for u in sol4.u], [u[2] for u in sol4.u], [u[3] for u in sol4.u]
    plot3d(x1, y1, z1, color=[:red,])
    plot3d!(x2, y2, z2, color=[:green,])
    plot3d!(x3, y3, z3, color=[:blue,])
    plot3d!(x4, y4, z4, color=[:yellow,])
end




















# 目标2：计算一组稳定的轨迹，并将计算数据存储到文档





# 目标3：根据稳定轨迹组，给出一个收敛算法，用于抗扰动





# ----------------------abandon code-------------------
# # basic status of slipmodel
# # actually, data will not store here while simulation
# mutable struct slipModel{T}
#     x::T; y::T; z::T     # mass center position
#     α::T;β::T            # leg status
#     iscontact::Bool      # contact status
# end
# -------------------------
# struct stateControl{T}
#     Vx::T;
#     Vy::T
#     h::T
#     α::T    # 与x轴的夹角
#     β::T
#     k1::T   # 压缩阶段k
#     k2::T
#     l0::T   # 腿原长
# end
# struct force{T}
#     x::T
#     y::T
#     z::T
# end

# # -------------Period using self made integration------------------------
# mutable struct SLIP_model
#     x; y; z; dx; dy; dz;
# end
# function step!(smdl::SLIP_model)
    
# end