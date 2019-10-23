# 测试选择积分器算法
using DifferentialEquations, ParameterizedFunctions
# 定义微分函数
van! = @ode_def VanDerPol begin
  dy = μ*((1-x^2)*y - x)
  dx = 1*y
end μ
# 构造问题
prob = ODEProblem(van!,[0.0,2.0],(0.0,6.3),1e6)
# 选择使用
sol = solve(prob,Tsit5())

sol = solve(prob,alg_hints = [:stiff])

