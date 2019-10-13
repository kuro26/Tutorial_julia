using DifferentialEquations, ParameterizedFunctions
using Plots

# air situation
function calc_foot_end(u, p)
    """ calculate the foot position in air """
    x, y, z = u[1:3]
    l0, α, β = p[2:4]
    x_pf = x + l0*cos(β)sin(α)
    y_pf = y + l0*sin(β)
    z_pf = z - l0*cos(β)cos(α)
    return (x_pf, y_pf, z_pf)
end

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

# ground situation
function compressingCalc(u, p)
    """ decide if on compressing """
    x, y, z, vx, vy, vz = u
    x0, y0, z0 = p[6:8]
    vec1 = transpose([x-x0, y-y0, z])  # ground-->mass point
    vec2 = [vx, vy, vz]                # velocity vector
    return vec1*vec2
end
function k1k2Force(u, p)
    """ Control law1: using fixed k1, k2 model """
    k1, k2, x0, y0, z0 = p[4:8]
    l0 = p[3]
    x, y, z= u[1:3]
    l = sqrt((x-x0)^2 + (y-y0)^2 + (z-0)^2)
    k = compressingCalc(u, p)<=0 ? k1 : k2

    fx = - k * (x - (x0 - l0*(x0-x)/l))
    fy = - k * (y - (y0 - l0*(y0-y)/l))
    fz = - k * (z - ( 0 - l0*(z0-z)/l))
    return (fx, fy, fz)
end
function jointForce(u, p)
    """Calculation: k force --> joint force"""
    k1, k2, x0, y0, z0 = p[4:8]
    l0 = p[3]
    x, y, z= u[1:3]
    l = sqrt((x-x0)^2 + (y-y0)^2 + (z-0)^2)
    k = compressingCalc(u, p)<=0 ? k1 : k2
    kforce = k*abs(l0-l)
    tmp = l0^2 - l^2
    if tmp < 0
        if tmp > -0.001
            tmp = 0.0
        else
            println("error in mass point position")
        end
    end
    torque = kforce * sqrt(tmp)/2
    return torque
end

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

function cycleCalc(m, l0, s0, ctrl)
    α, β, k1, k2 = ctrl
    z0, vx0, vy0 = s0
    u0 = [0.0, 0.0, z0, vx0, vy0, 0.0]
    tspan = (0.0, 4.0)
    p1 = (9.8, l0, α, β, k1, k2)
    ```------------------------------------------------------
    segment1, air-->contact
    the initial foot positon is not supposed to be underground!
    ---------------------------------------------------------```
    if(calc_foot_end(u0, p1)[3] <= 0)           # check
        return ([], [], [p1,])
    end
    
    prob = ODEProblem(diff_fun_fly!, u0, tspan, p1)
    cb = ContinuousCallback(cond_contact, affect_contact!)
    sol1 = solve(prob, Tsit5(), callback=cb, saveat = 0.01)
    
    ```------------------------------------------------------
    segment2, compressing
    if segment1 end with opposite velocity, simulation may get wired!
    --------------------------------------------------------```
    u0 = sol1.u[end]    
    foot_p = calc_foot_end(u0, p1)
    p2 = [m, 9.8, l0, k1, k2, foot_p...]
    if(compressingCalc(u0, p2) >= 0)
        return ([sol1,], foot_p, [p1, p2])               # check
    end
    
    prob = ODEProblem(diff_fun_support!, u0, tspan, p2)
    cb = ContinuousCallback(cond_compressed, terminate!)
    sol2 = solve(prob, Tsit5(), callback=cb, saveat = 0.01)

    ```-----------------------------------------------------
    segment3, thrusting
    compressing should not be too much! small k
    ------------------------------------------------------```
    u0, p3 = sol2.u[end], p2
    if(u0[3] <= 0)
        return ([sol1, sol2,], foot_p, [p1, p2, p3,])
    end
    prob = ODEProblem(diff_fun_support!, u0, tspan, p3)
    cb = ContinuousCallback(cond_leave, terminate!, abstol=1e-9)
    sol3 = solve(prob, Tsit5(), callback=cb, saveat = 0.01)
    
    ```-----------------------------------------------------
    segment4, air up
    thrusting initial z velocity shou be positive
    ------------------------------------------------------```
    u0, p4 = sol3.u[end], p1
    if(u0[6] <= 0)
        return ([sol1, sol2, sol3,], foot_p, [p1, p2, p3,p4])
    end
    prob = ODEProblem(diff_fun_fly!, u0, tspan, p4)
    cb = ContinuousCallback(cond_peak, terminate!)
    sol4 = solve(prob, Tsit5(), callback=cb, saveat = 0.01)
    
    return ([sol1, sol2, sol3, sol4], foot_p, [p1, p2, p3, p4])
end

# utilities
function maxJointTorque(sol, p)
    if(size(sol)[1] >= 2)
        u = sol[2].u[end]
        p2 = p[2]
        return jointForce(u, p2)
    else
        println("Error: no maxJointTorque!")
        return 0
    end
end

function jointTorqueAcc_Raw(sol, p)
    if(size(sol)[1] < 4)
        println("failed in testing")
        return 0
    end
    
    Tacc = 0
    sol1, sol2, sol3, sol4 = sol
    p1, p2, p3, p4 = p
    torque_1 = jointForce(sol2.u[1], p2)
    for i in 1:length(sol2.t)-1
        torque = jointForce(sol2.u[i+1], p2)
        Tacc += (torque_1 + torque)*(sol2.t[i+1] - sol2.t[i])/2
        torque_1 = torque
    end
    torque_1 = jointForce(sol3.u[1], p3)
    for i in 1:length(sol3.t)-1
        torque = jointForce(sol3.u[i+1], p3)
        Tacc += (torque_1 + torque)*(sol3.t[i+1] - sol3.t[i])/2
        torque_1 = torque
    end
    return Tacc
end



