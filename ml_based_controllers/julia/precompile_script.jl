using MLBasedESC
using LinearAlgebra

const PRECISION = Float64
const I1 = PRECISION(0.0455)
const I2 = PRECISION(0.00425)
const m3 = PRECISION(0.183*9.81)
const b1 = PRECISION(0.001)
const b2 = PRECISION(0.002)

function create_true_hamiltonian()
    mass_inv = inv(diagm(vcat(I1, I2)))
    pe(q) = m3*(cos(q[1])-one(eltype(q)))
    Hamiltonian(mass_inv, pe)
end

function create_learning_hamiltonian()
    massd_inv = PSDMatrix(PRECISION,2,2)
    vd = NeuralNetwork(PRECISION, [2,16,48,1], symmetric=true, fout=x->x.^2, dfout=x->2x)
    Hamiltonian(massd_inv, vd)
end

function create_ida_pbc_problem()
    input = PRECISION[-1.0,1.0]
    input_annihilator = PRECISION[1.0 1.0]
    ham = create_true_hamiltonian()
    hamd = create_learning_hamiltonian()
    IDAPBCProblem(ham, hamd, input, input_annihilator)
end

#t = @elapsed begin
    prob = create_ida_pbc_problem()
    θ = MLBasedESC.params(prob)
    u = controller(prob, θ, damping_gain=1e-3)
#end

#@info "Finished loading script in $t seconds."

