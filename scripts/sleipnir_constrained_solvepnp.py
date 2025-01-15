from jormungandr.autodiff import Gradient, VariableMatrix, Variable, sin, cos
from jormungandr.optimization import OptimizationProblem
import numpy as np

problem = OptimizationProblem()

# robot pose
x = problem.decision_variable()
y = problem.decision_variable()
z = problem.decision_variable()
θ = problem.decision_variable()

sinθ = sin(θ)
cosθ = cos(θ)


field2robot = VariableMatrix(
    [
        [cosθ, -sinθ, 0, x],
        [sinθ, cosθ, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1],
    ]
)

robot2camera = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])

field2camera = field2robot @ robot2camera
