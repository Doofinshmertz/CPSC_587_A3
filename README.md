
# Implementation Methods
CPSC 587 Assignment 3,
Holden Holzer, holden.holzer@ucalgary.ca
## AI Statement
No AI was used. 
## Local To Global Transform Matrix and Forward Kinematics
Forward kinematics are implemented using a local to global coordinate system for each bone.
Each bone has an associated transformation matrix that defines the position and rotation of the current bone relative to the parent bone. The local to global coordinate matrix is then calculated by chaining these transformation matrices from the current bone to the base bone.\
\
Forward kinematics is achieved by updating the matrices with the new bone length and angle values specified by the user.
### The tranformation matrix of a specific bone
The local to global matrix of a bone is defined by a rotation then a translation:
$$ M_i = M_{i-1} \cdot T_i(d_{i-1},0,0) \cdot R_z(\theta_i)$$
Where $M_i$ is the transformation matrix for bone i, which is composed of a rotation about $z$ by the angle of bone i and then a translation by the length of the previous bone $i-1$ in the x direction and finaly the transform of the parent bone $M_{i-1}$.
## Inverse Kinematics with Jacobian Transpose
Inverse kinematics was done using the algorithm outlined in the Assignment 3 Technical Document. A summary of the algorithm is given bellow. 
1. While the error is greater than the allowed error do the following:
2. calculate the position $e = f(\Theta)$
3. calculate the difference between the current position and the target $\Delta{e} = e_{target} - e$
4. calculate the Jacobian $j = \nabla{f(\Theta)}$
5. Solve for the change in angles that moves the end effector towards the target $ \Delta{\Theta} = SolveDeltaAngles(j, \Delta{e})$
6. Update the angles $\Theta \leftarrow \Theta + \Delta{\Theta}$

### Jacobian approximation
Assuming the position of the end effector is given by the function $\vec{e} = f(\Theta)$ where $\Theta = 
\{\theta{_0}, \theta{_1}, ... , \theta{_n} \} $ The Jacobian is calculated using the following method:
$$ j = \left[ \frac{\partial{\vec{f}}}{\partial{\theta{_1}}}, \frac{\partial{\vec{f}}}{\partial{\theta{_2}}}, ... , \frac{\partial{\vec{f}}}{\partial{\theta{_n}}} \right] $$
Where: 
$$\frac{\partial{\vec{f}}}{\partial{\theta{_i}}} \approx \frac{\vec{f(..., \theta{_i} + \epsilon, ...)} - \vec{f(..., \theta{_i} - \epsilon, ...)}}{2 \cdot \epsilon}$$

### Solving for the angle update
The changes in the angles are calculated using the following equation (given in the Assignment 3 Technical Document:
$$\Delta{\Theta} = \alpha J_\theta^T \Delta{e}$$
where:
$$ \alpha = \frac{\Delta{e} \cdot J_\theta J_\theta^T \Delta{e} }{J_\theta J_\theta^T \Delta{e} \cdot J_\theta J_\theta^T \Delta{e}}$$

Here $\alpha$ is calculated as a normalized projection of $\Delta{e}$ on the movement vector $J_\theta^T \Delta{e}$. This ensures that the new angles do not result in the end effector moving by more than the length of $\Delta{e}$.
## Linear Blend Skinning
In linear blend skinning, the position of a particular vertex is computed by first converting its rest position from global coordinates to the local coordinates of the bone(s) to which it is attached in their rest position. Then the point transformed by the local to global transform of the bones in their posed position. The formula for a single bone is given bellow:

$$ p' = M_i' M_i^{-1} p $$

Where $p$ is the position of the point in its rest position, $M_i$ is the local to global matrix of bone $i$ in its rest position, $M_i'$ is the local to global matrix of the bone in its current pose position.\
For linear blend skinning, a points position may be weighted between several bones, each with different associated weights. The equation for transforming a vertex using multiple bones is base on the equation given in the Assignment 3 Technical Document and is shown as follows:
$$ p' = \sum_{i}w_{ij} \cdot M_i' M_i^{-1} p_j $$
Where $w_{ij} is the weight of effect of bone $i$ on vertex $j$. Note that the sum of these weights for a particular vertex must be 1 for the operation to remain affine.
$$\sum_{i} w_{ij} = 1 $$

## Bonus 1: Damped Least Squares IK
For Bonus 1, the damped least squares solver was implemented along with the jacobian transpose for inverse kinematics. The algorithm for inverse kinematics is the same as the one used for the jacobian transpose, but the methode for solving for the angle updates is different and shown bellow:
$$\Delta{\Theta} = (J_{\theta}^T J_{\Theta} + \lambda^2 I^{n \times n})^{-1} \Delta{e}$$
Where $\lambda$ is the 'damping' factor. By adding a sufficiently large $\lambda^2$ to the matrix $ J_{\theta}^T J_{\Theta} $ we can control the speed at which the iterative solver converges, higher damping factors generally result in the solver taking more iterations to converge. Lower damping factors result in the taking less iterations, but being more prone to diverge. The damping factor serves the same end purpose as the $\alpha$ value in the Jacobian Transpose method, with the value of $\alpha$ serving to bound the distance the arm moves in each iteration, meaning small values of $\alpha$ force the solution to converge more slowly and more stably.   

## Bonus 2: Custom blending weights using Gaussian function
Along with the provided blending weights, custom bending weights were also calculated for use in the linear blend skinning. The method used finds the distance from each vertex to each bone, then uses the Gaussian function to compute a weight for each bone. The algorithm used for calculating the distance to each bone is based on the one given in the Assignment 3 Technical Document and is summarized bellow:
1. For each vertex $v_i$ do:
2. For each bone $j$ do:
3. Get the displacement vector from the start of bone $j$ to the start of bone $j+1$: $d = start_{j+1} - start_j$
4. Get the displacement vector from the vertex to the start of bone $j$: $u = v_i - start_j$
5. Project the vertex displacement along the bone and normalize by the bone length: $t = \frac{u \cdot d}{d \cdot d}$
6. If the vertex is behind the bone ($t < 0$) then the distance is the length of $u$
7. If the vertex is past the end of the bone ($t > 0$) then the distance is the length of $v_i - start_{j+1}$
8. If the vertex is beside the bone, then the distance is the normal distance from the bone to the vertex: $||u - t*d||$

Using this distance the Gaussian can be applied to get the relative weight of this bone on the vertex:
$$ Gaussian(d_{ij}) = e^{-d_{ij}^2}$$
Finally these weights must be normalized to get ensure the vertex remains affine after transformation:
$$w_{ij} = \frac{e^{-d_{ij}^2}}{\sum_{j} e^{-d_{ij}^2}}$$
The results Gaussian function in this blending scheme result in the vertices being most effected by the bones they are closest to. However, this method does produce some undesired behavior. In the monkey model, the ears and chin protrude out from the head and are effected more by bones that the rest of the head are not near. This results in the chin and ears moving and bending disproportionally relative to the rest of the head. Ideally we would want the chin and ears to move with the parts of the head to which they are attached. The problem with the current weight calculations is that it is based on straight line distance from vertex to bone, even if the vertex is part of geometry that does not contain the bone (e.g. the head mainly contains the last bone, but the chins geometry puts it near the second bone) this could be solved by manually assigning the weights (using the weight painting tool in Blender), or using a line trace to check if the straight line from the bone to the vertex has to pass through other geometry and modify the weights accordingly. 
# Usage
## Building and Running
To quickly build and run the program it is advised to run the provided shell scripts using the command:
"./QuickBuild.sh" or "./CleanBuild.sh" these will build and run the program in a single command. \
**Building**: To build the program navigate to the directory containing "src", "models", "libs", "CMakeLists.txt". Run the command "cmake -B build", then run the command "cmake --build build". The executable will be named "cpsc587_a3_hh" \
**Running**: Run the command "./build/cpsc587_a3_hh" 
## Controls
* **Use Inverse Kinematics:** This input remains unchanged from the example code. Allows user to activate inverse kinematics.
* **Max Iterations:** Set the maximum number of solver iterations. Higher numbers result in better accuracy, but lower solver speed.
* **Tolerance:** Distance from the target position that is considered solved and the solver can stop iterating. 
* **Finite step (rad):** The distance used for taking the numerical derivatives when calculating the jacobian.
* **Solver Type:** What type of solver to use. Select from "Jacobian Transpose" and "Damped Least Squares".
* **Alpha max:** The maximum distance that the Jacobian Transpose method is allowed to move the end effector by in a single step.
* **lambda:** The damping factor used in the Damped Least Squares solver (The square of this value is added to the diagonals of the matrix to ensure it is diagonally dominant and therefore solvable).
* **Use Skinning Model:** This input remains unchanged from the example code. Allows user to activate the skinning model instead of the bone model.
* **Use Custom Calculated Weights:** Switch the weights from the provided pre-calculated weights to custom skinning weights calculated using the Gaussian weight function.
* **Target Type:** This input remains unchanged from the example code. Allows the user to select a inverse kinematics target from: "Specific", "Animated", "Cursor".
* **Specific Target Position:** Allows user to set the target position for the Specific target.
* **Bone Length and angle controls:** This input remains unchanged from the example code. Allows user to set the lengths and angles of the bones for forward kinematics.
