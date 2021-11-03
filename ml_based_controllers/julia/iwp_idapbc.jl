#!/usr/bin/env julia

using MLBasedESC
using LinearAlgebra
using RobotOS
@rosimport sensor_msgs.msg: JointState
@rosimport std_msgs.msg: Float64
rostypegen()
using .sensor_msgs.msg, .std_msgs.msg

const USE_J2 = false
# const PARAMS = Float32[0, -6.3120513, 2.4831183f-5, -0.00027178923, -4.817491f-7, 0.0051580914, -0.0009012511, -0.0027414176, -5.4961457, 3.1027546, 4.6f-43, 0, 0]
const PARAMS = Float32[0.0, 6.389714, 0.0, 0.2223909, 0.0, 0.006149394, 0.0049287872, 0.0, 0.0, 0.0, -0.30627432, 0.0, 0.0]

function create_true_hamiltonian()
    I1 = 0.0455f0
    I2 = 0.00425f0
    m3 = 0.183f0*9.81f0
    mass_inv = inv(diagm(vcat(I1, I2)))
    # pe(q) = m3*(cos(q[1])-one(eltype(q)))
    pe(q) = begin
        # qbar = input_mapping(q)
        # return -m3*qbar[1]
        return -m3*q[1]
    end
    Hamiltonian(mass_inv, pe, input_jacobian)
end

input_mapping(x) = [
    one(eltype(x))-cos(x[1]); 
    x[1]
    x[2]
    sin(x[1])
]

function input_jacobian(x)
    """
    Input mapping f(x) = [1-cos(x1), sin(x1), 1-cos(x2), sin(x2)]
    This is Jₓf
    """
    T = eltype(x)
    [
        x[4] zero(T); 
        one(T) zero(T); 
        zero(T) one(T); 
        one(T)-x[1] zero(T)
    ]
end

function create_learning_hamiltonian()
    massd_inv = PSDNeuralNetwork(Float32, 2, nin=2)
    # vd = NeuralNetwork(Float32, [4,128,128,1])
    vd = SOSPoly(2, 1:3)
    Hamiltonian(massd_inv, vd)
end

function create_partial_learning_hamiltonian()
    a1,a2,a3 = (0.001f0, -0.002f0, 0.005f0)
    massd = [a1 a2; a2 a3]
    massd_inv = inv(massd)
    # vd = SOSPoly(4, 1:1) # Float32[-6.3120513, 2.4831183f-5, -0.00027178923, -4.817491f-7, 0.0051580914, -0.0009012511, -0.0027414176, -5.4961457, 3.1027546, 4.6f-43]
    vd = IWPSOSPoly() # Float32[6.389714, 0.0, 0.2223909, 0.0, 0.006149394, 0.0049287872, 0.0, 0.0, 0.0, -0.30627432]
    Hamiltonian(massd_inv, vd, input_jacobian)
end

function create_ida_pbc_problem()
    input = vcat(-1.0f0,1.0f0)
    input_annihilator = hcat(1.0f0,1.0f0)
    ham = create_true_hamiltonian()
    # hamd = create_learning_hamiltonian()
    hamd = create_partial_learning_hamiltonian()
    if USE_J2
        J2 = InterconnectionMatrix(
            SkewSymNeuralNetwork(Float32, 2, nin=2),
            SkewSymNeuralNetwork(Float32, 2, nin=2)
        )
        return IDAPBCProblem(ham, hamd, input, input_annihilator, J2)
    else
        return IDAPBCProblem(ham, hamd, input, input_annihilator)
    end
end

function create_known_ida_pbc()
    I1 = 0.0455f0
    I2 = 0.00425f0
    m3 = 0.183f0*9.81f0
    a1 = 1.0f0
    a2 = -1.1f0
    a3 = 2.0f0
    k1 = 0.0001f0
    γ2 = -I1*(a2+a3)/(I2*(a1+a2))
    input = vcat(-1.0f0,1.0f0)
    input_annihilator = hcat(1.0f0,1.0f0)
    
    mass_inv = inv(diagm(vcat(I1, I2)))
    pe(q) = m3*(cos(q[1]) - one(q[1]))
    ham = Hamiltonian(mass_inv, pe)

    massd = [a1 a2; a2 a3]
    massd_inv = inv(massd)
    ϕ(z) = 0.5f0*k1*z^2
    z(q) = q[2] + γ2*q[1]
    ped(q) = I1*m3/(a1+a2)*cos(q[1]) + ϕ(z(q))

    hamd = Hamiltonian(massd_inv, ped)
    return IDAPBCProblem(ham, hamd, input, input_annihilator)
end


function update_state(msg::JointState, state::Vector)
    state[1] = msg.position[1]
    state[2] = msg.position[2]
    state[3] = msg.velocity[1]
    state[4] = msg.velocity[2]
end

function compute_control(x::Vector, swingup_controller::Function)
    q1, q2, q1dot, q2dot = x
    xbar = [sin(q1-pi), sin(q2), q1dot, q2dot]
    if (1+cos(q1)) < (1+cos(pi-pi/10)) && abs(q1dot) < 5.0
        K = [-7.409595362575457, -0.05000000000000429, -1.1791663255097424, -0.03665716263249201]
        effort = -dot(K,xbar)
        return clamp(effort, -1.0f0, 1.0f0)
    else
        I1 = 0.0455f0
        I2 = 0.00425f0
        M = diagm(vcat(I1, I2))
        qbar = input_mapping([q1-Float32(pi),q2])
        # effort = swingup_controller(xbar[1:2], M*xbar[3:4])
        effort = swingup_controller(qbar, M*xbar[3:4])
        # return effort
        return clamp(effort, -2f0, 2f0)
    end
end

function main()
    init_node("ida_pbc_controller")
    state = zeros(Float64,4)
    pub = Publisher{Float64Msg}("theta2_controller/command", queue_size=1)
    sub = Subscriber{JointState}("/joint_states", update_state, (state,), queue_size=1)
    prob = create_ida_pbc_problem()
    # prob = create_known_ida_pbc()
    θ = PARAMS
    u = controller(prob, θ, damping_gain=0.015f0)
    loop_rate = Rate(500.0)
    while !is_shutdown()
        header = std_msgs.msg.Header()
        header.stamp = RobotOS.now()
        # effort = u(state[1:2], M*state[3:4])
        # effort = clamp(effort, -1f0, 1f0)
        effort = compute_control(state, u)
        # effort = clamp(effort, -2f0, 2f0)
        gear_ratio = 1.0
        eta = 0.95 # 0.88
        k_tau = 0.230    # N-m/a
        current = effort / gear_ratio / k_tau / eta
        cmd = Float64Msg(current)
        publish(pub, cmd)
        rossleep(loop_rate)
    end
    safe_shutdown_hack()
end

function safe_shutdown_hack()
    run(`rostopic pub -1 /theta2_controller/command std_msgs/Float64 "data: 0. "`, wait=false);
end
Base.atexit(safe_shutdown_hack)

if !isinteractive()
    main()
end
