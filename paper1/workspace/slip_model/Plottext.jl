using Plots

mutable struct Lorenz
    dt; σ;  ρ; β; x; y; z
end

function step!(l::Lorenz)
    #  diff                        simple integration
    dx = l.σ*(l.y - l.x);          l.x += l.dt *dx;
    dy = l.x*(l.ρ - l.z) - l.y ;   l.y += l.dt * dy
    dz = l.x*l.y - l.β*l.z     ;   l.z += l.dt * dz
end

# construct a Lorenz attractor
attractor = Lorenz((dt = 0.02, σ = 10., ρ = 28., β = 8//3, x = 1., y = 1., z = 1.)...)

# initialize a canvas using plot3d
plt = plot3d(1, xlim=(-25,25), ylim=(-25,25), zlim=(0,50),
                title = "Lorenz Attractor", marker = 2)

@gif for i=1:1500
    step!(attractor)    # integrate oringinal place
    push!(plt, attractor.x, attractor.y, attractor.z)
end every 10


