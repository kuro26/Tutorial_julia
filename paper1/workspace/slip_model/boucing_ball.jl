# from：https://github.com/JuliaDiffEq/DiffEqTutorials.jl/blob/master/tutorials/introduction/04-callbacks_and_events.jmd
# 你会发现这里存在一些小问题，在接近地面并且低速的时候，可能产生接近地面的方向误判
# 小球碰撞
using DifferentialEquations, ParameterizedFunctions

# 动力学方程，这个语法还不太熟
# 参数说明：
# @ode_def,宏标识
# BallBounce,宏的参数1，即name
# begin... end，宏参数2，即ex，是一个表达式
#    表达式是遵循一定的规则的，如dv会被识别为导数，v则被识别为对应的变量
# g，对应宏参数中的params
ball! = @ode_def BallBounce begin
  dy =  v 
  dv = -g
end g
# 或者另一种定义方式？
function ball2!(du, u, p, t)
  g = p[1]
  y, v = u
  du[1] = dy = v
  du[2] = dv = -g
end

# 2. 构造碰撞过程
# 触发条件，位置过零，u[1]==0
function condition(u,t,integrator)
    u[1]
end
# 触发条件后对原系统的影响，速度u[2]反向
# 反向系数来自p[2]，也就是参数2
function affect!(integrator)
    integrator.u[2] = -integrator.p[2] * integrator.u[2]
end
# 创建回调对象
bounce_cb = ContinuousCallback(condition,affect!)


# 3. 创建ode问题
u0 = [50.0,0.0]
tspan = (0.0,15.0)
p = (9.8,0.5)
prob = ODEProblem(ball2!,u0,tspan,p,callback=bounce_cb)
# prob = ODEProblem(ball!,u0,tspan,p,callback=bounce_cb)
# 4. 求解问题
sol = solve(prob,Tsit5())
using Plots; gr()
plot(sol)
