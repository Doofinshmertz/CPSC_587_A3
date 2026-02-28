
# Implementation Methods
CPSC 587 Assignment 1,
Holden Holzer, holden.holzer@ucalgary.ca
## AI Statement
No AI was used. 
## Hermite Curve
For this assignment, the curve used to interpolate the control points was the Cubic Hermite spline. The definition of which was obtained from the Assignment 1 Technical Specification: $$ C(u) = \vec{p}_i(2u^3 - 3u^2 + 1) \\ + \vec{t}_i(u^3 - 2u^2 + u) 
\\ + \vec{p}_{i+1}(-2u^3 + 3u^2) \\ + \vec{t}_{i+1}(u^3 - u^2)$$
Where u is the **local** u value for the curve, $\vec{p}_i$ is the starting point, $\vec{t}_i$ is the starting tangent, $\vec{p}_{i+1}$ is the end point, and $\vec{t}_{i+1}$ is the end tangent. The curve guarantees a $C^1$ continuity and passes through the control points. The tangents are found by numerical differentiation via the provided function: $$\vec{t}_i = \frac{\vec{p}_{i+1} - \vec{p}_{i-1}}{2}$$
When evaluating the curve, the *u* value must be the interpolation factor between the two consecutive points, this is different from the global *U* value for the entire curve. The local *u* value is found by the following method:
1. find the indices of the control points using $i = floor ((n-1)\cdot U)$ where n is the number of control points
2. find the small u value using $ u = (n-1)\cdot U - floor ((n-1)\cdot U)$
## Arc Length Parameterization Table
The Arc Length Parameterization Table was created using the algorithm outlined in the Assignment 1 Technical Document. A summary as follows: 
1. Move along the curve using a small $\Delta{u}$ value, keeping track of the current curve length that has been traversed as $s$.
2. If the traversed length $s$ exceeds some $\Delta{s} * index$ value, record the current u value in the table and increment the index. This s value roughly maps to this u value.

The curve length increment at each $\Delta(u)$ step is approximated using: $|| C(u + \Delta{u}) - C(u)||$. When looking up values, find the index in the table using $index = floor(\frac{s}{\Delta{s}})$ then use the remainder to interpolate between the u values at $u_1 = table(index)$ and $u_2 = table(index + 1)$. Special case: if *index* is at the end of the table use $u_2 = 1$. 
\
For this implementation the choice of $\Delta{s}$ and $\Delta{u}$ had to ensure that a step of $\Delta{u}$ corresponded to a shorter distance than $\Delta{s}$. This was done by observing that the maximum change in s with respect to u is: $max(dS/du) = numPoints \cdot maxSeparation $. Where numPoints is the number of control points, and maxSeparation is the largest distance between two consecutive control points (Estimated using $max(|| \vec{p}_{i+1} - \vec{p}_i||)$. Then we require that $\Delta{u} < du/dS \cdot \Delta{s} = \Delta{s} / (numPoints \cdot maxSeparation)$
## Velocity profile
The movement of the cart along the track is simulated by recording the carts current position as $s$, then updating the position using the formula: $s \leftarrow s + speed(s) \cdot \Delta{t}$. Where the $\Delta{t}$ is the time step size (usually the time between frames) and $speed(s)$ is the speed at the current position $s$. This algorithm is accurate assuming: The frame rate does not change much, and the speed does not change significantly from position $s$ to position $s + speed(s) \cdot \Delta{t}$.
### Lifting Phase
For this phase the roller coaster cart should move at a steady rate of v_min (minimum velocity). This phase should begin at the start of the curve (U = 0) and end at the highest point on the curve. During the creation of the arc length table, the y value at each position was measured along the curve. The maximum hight *H* along with the s value corresponding to *H* was recorded as s_freefall. If the u value is between 0 and s_freefall then the speed of the cart is set to v_min.
### Gravity driven phase
For this phase the roller coasters speed is determined by the conservation of kinetic and potential energy. mass is not changing so we will use they intensive version of the equation (Fun fact: this is a special case of the Bernoulli equation): $$ \frac{v_1^2}{2} + gh_1 = \frac{v_2^2}{2} + gh_2$$
Setting $h_1$ to be the current height, $h_2$ to be the maximum height *H*, $v_2$ to be the minimum velocity, we get the following: $$ v_1 = \sqrt{2g(H-h) + v_2^2}$$. Thus the velocity of the cart can be calculated using only the hight at its current position.
### Deceleration
The deceleration phase is calculated using the a fraction *decel_frac* (usually set to 0.9) it is the u value at which to start decelerating the cart. This fraction is used to calculate the *s* value at which to start decelerating $u_{dec} = decel_{frac} \cdot arcLength$. The deceleration must start at the speed determined by the conservation of energy at position $u_{dec}$, $v_{dec}$ and decelerate to v_min by the end of the curve. This is obtained by the interpolation: $$ speed(s) = v_{dec} + \frac{s - s_{dec}}{arcLength - s_{dec}} \cdot (v_{min} - v_{dec})$$
This equation is used if the s position is greater than $s_{dec}$
## Cart and Track Rotation
The rotation of the cart and track are calculated by finding total acceleration vector and taking its component that is perpendicular to the track. First the curvature and normal of the curve is calculated using the method provided in the Assignment 1 Technical Specifications. Using the centrifugal acceleration formula: $a = \frac{v^2}{r}$, the speed at the current position $v = speed(s)$, and curvature at the position the acceleration vector from curvature is: $\vec{a}_{curve} = k \cdot n \cdot v^2$. Then acceleration due to gravity is added to get the total acceleration: $\vec{a} = \vec{a}_{curve} - \vec{g}$. Then we get the component of this acceleration that is perpendicular to the curve tangent $\vec{a}_{perp} = \vec{a} - (\vec{a} \cdot \vec{T})\vec{T}$. This vector is normalized to get the normal for the rotation matrix $N = \vec{a}_{perp} / ||\vec{a}_{perp}||$. This can then be used to get the Binormal $B = N \times T$. These vectors then form the rotation matrix used to rotate both the cart and the track pieces.
## Other Stuff
### Track Supports
The track supports where placed using a similar method as the track pieces. The only difference being that the normal was fixed to point in the y (up) direction instead of being based on acceleration. Also the supports were scaled in the y-axis based on the heigh at the current position to ensure they were long enough. 
### Trees
They are just placed using random positions.
# Usage
## Building and Running
To quickly build and run the program it is advised to run the provided shell scripts using the command:
"./QuickBuild.sh" or "./CleanBuild.sh" these will build and run the program in a single command. \
**Building**: To build the program navigate to the directory containing "src", "models", "libs", "CMakeLists.txt". Run the command "cmake -B build", then run the command "cmake --build build". The executable will be named "cpsc587_a1_hh" \
**Running**: Run the command "./build/cpsc587_a1_hh" 
## Controls
* **Loading Control points**: This function remains unchanged from the provided Boilerplate. It can still be used to load new roller coaster curve geometries.
* **Play/Pause**: This function remains unchanged from the provided Boilerplate. Used to start/stop the roller coaster simulation. 
* **Show Curve/Hide Curve**: This is used to show or hide the control point curve (the debug curve that came with the Boilerplate code). 
* **Reset View**: This function remains unchanged from the provided Boilerplate. Resets the camera view.
* **Use Moving Camera/Use Stationary Camera**: pressing "Use Moving Camera" re-centers the camera turntable around the roller coaster cart. pressing "Use Stationary Camera" uses the stationary origin for the turntable center.
* **Reset Simulation**: Resets the position of the cart to the start of the track.
* **Number of Carts**: controls the number of carts in the cart train.
* **Playback Speed**: controls the simulation speed.
* **Look Ahead**: controls the look ahead distance for calculating the curvature.

