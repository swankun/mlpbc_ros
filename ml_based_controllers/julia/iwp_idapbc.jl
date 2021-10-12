#!/usr/bin/env julia

using MLBasedESC
using LinearAlgebra
using RobotOS
@rosimport sensor_msgs.msg: JointState
@rosimport maxon_epos_msgs.msg: MotorState
rostypegen()
using .sensor_msgs.msg, .maxon_epos_msgs.msg

const USE_J2 = false
const MOTOR_NAME = "epos2"

function create_true_hamiltonian()
    I1 = 0.1f0
    I2 = 0.2f0
    m3 = 10.0f0
    mass_inv = inv(diagm(vcat(I1, I2)))
    pe(q) = m3*(q[1] - one(q[1]))
    Hamiltonian(mass_inv, pe, input_jacobian)
end

function input_jacobian(x)
    T = eltype(x)
    [-x[2] zero(T); x[1] zero(T); zero(T) -x[4]; zero(T) x[3]]
end

function create_learning_hamiltonian()
    massd_inv = PSDNeuralNetwork(Float32, 2, nin=4)
    # vd = NeuralNetwork(Float32, [4,128,128,1])
    vd = SOSPoly(4, 2)
    Hamiltonian(massd_inv, vd, input_jacobian)
end

function create_ida_pbc_problem()
    input = vcat(-1.0f0,1.0f0)
    input_annihilator = hcat(1.0f0,1.0f0)
    ham = create_true_hamiltonian()
    hamd = create_learning_hamiltonian()
    if USE_J2
        J2 = InterconnectionMatrix(
            SkewSymNeuralNetwork(Float32, 2, nin=4),
            SkewSymNeuralNetwork(Float32, 2, nin=4)
        )
        return IDAPBCProblem(ham, hamd, input, input_annihilator, J2)
    else
        return IDAPBCProblem(ham, hamd, input, input_annihilator)
    end
end

function create_known_ida_pbc()
    I1 = 0.1f0
    I2 = 0.2f0
    m3 = 10.0f0
    a1 = 1.0f0
    a2 = 0.5f0
    a3 = 2.0f0
    k1 = 1.0f0
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
    prob = IDAPBCProblem(ham, hamd, input, input_annihilator)
end


function update_state(msg::JointState, state::Vector)
    state[1] = msg.position[1]
    state[2] = msg.position[2]
    state[3] = msg.velocity[1]
    state[4] = msg.velocity[2]
end

function main()
    init_node("ida_pbc_controller")
    state = zeros(Float32,4)
    pub = Publisher{MotorState}("epos2_commands", queue_size=1)
    sub = Subscriber{JointState}("states", update_state, (state,), queue_size=1)
    # prob = create_ida_pbc_problem()
    prob = create_known_ida_pbc()
    u = controller(prob)
    loop_rate = Rate(500.0)
    while !is_shutdown()
        header = std_msgs.msg.Header()
        header.stamp = RobotOS.now()
        cmd = MotorState(header, MOTOR_NAME, 0.0, 0.0, u(state[1:2], state[3:4]))
        publish(pub, cmd)
        rossleep(loop_rate)
    end
end

if !isinteractive()
    main()
end
