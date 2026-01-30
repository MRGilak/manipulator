# manipulator
This is a repo for the object-oriented MATLAB codes for simulaing any manipuator, including all the kinematics, dynamics and control parts. I've tried to include as much information as possible in the README, but more extensive explanations can be found in the accompanying pdf file and in the provided references. 

A 6 DoF manipulator has been chosen to demonstrate the techniques and ideas used in the codebase. The robot's datasheet brochure can be found here.

### TODO
Several control techniques will later be added to control the robot, including Proportional Derivative (PD), Slotine controllers and adaptive and robust controllers.

## Kinematics
The kinematics are solved using the Denavit-Hartenberg technique, hereby after referred to as DH.

A homogenous transformation between the two frames 0 and 1 is defined as

$$
    \begin{align}
        H_1^0 = \begin{bmatrix}
            R_1^0 & d_1^0 \\
            0_{3 \times 1} & 1
        \end{bmatrix}
    \end{align},
$$

where $R_1^0$ denotes the rotation matrix from frame 0 to frame 1 and $d_1^0$ denotes the linear displacement between the two frames. Denavit-Hartenberg method provides a systematic way to define the transformation between two frames with 4 parameters (more information in here).

The DH frames are shown in the image below.

<img width="500" alt="cart-with-spring" src="https://github.com/MRGilak/manipulator/blob/main/images/forward_kinematics.jpg" />

The DH parameters are listed in the following table.

<img width="500" alt="cart-with-spring" src="https://github.com/MRGilak/manipulator/blob/main/images/DH_parameters.jpg" />

With these, the forward kinematics of the robot can be solved. You can use the `fk` method of the `Manipulator` class to find the homogenous transformation to any of the frames (from the base).

## Jacobian matrix
The Jacobian matrix is a fundamental part of any robot. It primarily defines the relation between joint and end-effector velocities, but it has other uses as well. Also, the Jacobian can be defined for any of the $n$ frames, not only the end-effector.

The Jacobian matrix is a $6 \times n$ matrix, where $n$ is the number of joints of the robot. The matrix is composed of two parts. The top half is $J_\omega$ which relates the angular velocity of the frame to the angular/linear velocities of the joints. The bottom half is $J_v$ which relates the linear velocity of the frame to the angular/lienar velocities of the joints. In the `Manipulator` class, methods `jacobianOmega` and `jacobianLinear` provide $J_\omega$ and $J_v$ for any frame, respectively. `jacobian` method provides the full $6 \times n$ Jacobian matrix of the robot. The following formulas are used to calculate the Jacobian matrix, in which $k = [0, 0, 1]^T$.

$$
    J_\omega^i = \begin{cases}
        R_{i-1}^0 k & \text{if joint $i$ is revolute} \\
        0 & \text{if joint $i$ is prismatic}
    \end{cases}
$$

$$
    J_v^i = \begin{cases}
        R_{i-1}^0 k \times (O_n^0 - O_{i-1}^0) & \text{if joint $i$ is revolute} \\
        R_{i-1}^0 k & \text{if joint $i$ is prismatic}
    \end{cases}
$$

The Jacobian matrix derived above is called the _geometric_ Jacobian matrix. It relates the velocity of the selected frames to the velocities of the joints. For deriving the dynamics of the robot, another type of Jacobian matrix is needed which relates the velocity of the Center of Mass (COM) of the links to the velocities of the joints. This matrix, to which we refer as COM Jacobian, can be written as

$$
    \begin{align}
        J_C = \begin{bmatrix}
            I & S(r_{CD}) \\
            0_{3 \times 3} & I
        \end{bmatrix} J_D,
    \end{align}
$$
where $J_D$ is the geometrix Jacobian of the robot and $r_{CD}$ is the vector connecting the center of mass of link $i$ to the selected frame on link $i$, expressed in the frame attached to the center of mass (all the axis of this frame are exactly parallel to the axis of the seected frame on this link). $S(r_{CD})$ is the skew-symmetric matrix form of the vector $r_{CD}$. 

## Dynamics
The dynamics of the robot are in the form

$$
    M \dot{z} + S z + g = F,
$$

in which the following matrices and vectors are used

$$
    \begin{align}
        M_{6n \times 6n} = \begin{bmatrix}
            diag([m_1 I, m_2 I, ..., m_n I]) & 0 \\
            0 & diag([I_1, I_2, ..., I_n])
        \end{bmatrix}
    \end{align},
$$

$$
    \begin{align}
        S_{6n \times 6n} = \begin{bmatrix}
            0 & 0 \\
            0 & diag([-S(I_1 \omega_1), -S(I_2 \omega_2), ..., -S(I_n \omega_n)])
        \end{bmatrix}
    \end{align},
$$

$$
    \begin{align}
        g_{6n \times 1} = \begin{bmatrix}
            \begin{matrix}
                g_1 \\ g_2 \\ ... \\ g_n
            \end{matrix}
            \\
            0
        \end{bmatrix}
    \end{align}, \quad 
    g_i = \begin{bmatrix}
        0 \\ 0 \\ g
    \end{bmatrix} m_i,
$$

$$
    \begin{align}
        z_{6n \times 1} = \begin{bmatrix}
            \begin{matrix}
                v_1 \\ v_2 \\ ... \\ v_n
            \end{matrix}
            \\
            \begin{matrix}
                \omega_1 \\ omega_2 \\ ... \\ omega_n
            \end{matrix}
        \end{bmatrix}
    \end{align},
$$

$$
    \begin{align}
        F_{6n \times 1} = \begin{bmatrix}
            \begin{matrix}
                f_1 \\ f_2 \\ ... \\ f_n
            \end{matrix}
            \\
            \begin{matrix}
                n_1 \\ n_2 \\ ... \\ n_n
            \end{matrix}
        \end{bmatrix}
    \end{align},
$$

in which $v_i$, $\omega_i$, $f_i$ and $n_i$ are the linear velocity of the COM of link $i$, the angular velocity of the COM of link $i$, the net force on link $i$ (expressed in $O_0$) and the torque on link $i$ with respect to its COM and expressed in $O_i$, respectively. The inertia matrices are with respect to the COM of links and expressed in $O_i$, so they are constant.

These equations are the relations governing the movement of the $n$ links, but the connections of these $n$ links together is not yet in the equations. When two joints are connected together, they can move in one direction with respect to each other, and in the five other directions, they are constrained. This is why from the $6n$ equations above, only $n$ are independent.

Considering that $z = J(q) \dot{q}$ and multiplying the set of equations above by $J^T$ ($J$ being the generalized Jacobian matrix, composed of the Jacobians of all the links stacked below each other, defined in `J_all` method in the `Manipulator` class) from left, we get

$$
    D(q) \ddot{q} + C(q, \dot{q}) \dot{q} + G(q) = \tau,
$$
where $D(q)$ is the mass and inertia matrix, $C(q, \dot{q})$ is the gyroscopic matrix, $G(q)$ is the gravity vector and $\tau$ is the vector of the torques in the joints. These matrices are derived as

$$
    D(q) = J^T (q) M J(q),
$$

$$
    C(q, \dot{q}) = J^T (q) M \dot{J} (q, \dot{q}) + J^T (q) S(q, \dot{q}) J(q),
$$

$$
    G(q) = J^T (q) g.
$$

All the matrices mentioned above are defined in the `Manipulator` class.
- `M`: massMatrix
- `S`: coriolisMatrix
- `g`: gravityVector
- `D`: inertiaMatrix
- `C`: coriolisCentrifugalMatrix
- `G`: gravityTorque


