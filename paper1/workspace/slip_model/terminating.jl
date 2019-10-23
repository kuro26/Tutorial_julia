# em：在事件处结束积分过程
using DifferentialEquations

u0= [1., 0.]
function fun2(du,u,p,t)
    du[2] = -u[1]
    du[1] = u[2]
 end
tspan = (0.0, 10.0)
prob = ODEProblem(fun2,u0,tspan)

# DiscreteCallback会在第一次检测满足条件的时候停止
condition(u,t,integrator) = u[2]>0
affect!(integrator) = terminate!(integrator)
cb = DiscreteCallback(condition,affect!)
sol = solve(prob,Tsit5(),callback=cb)
sol.t[end] - pi


# ContinuousCallback则会刚好停在对应的点
condition(u,t,integrator) = u[2]
affect!(integrator) = terminate!(integrator)
cb = ContinuousCallback(condition,affect!)
sol = solve(prob,Tsit5(),callback=cb)
sol.t[end] - pi

# 更精确停留在对应的点（计算的事件高了很多）
sol = solve(prob,Vern8(),callback=cb,reltol=1e-12,abstol=1e-12)
sol.t[end] - pi

