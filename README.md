# `ffea`
`ffea` (_Framework for Finite Element Anaysis_) is a framework for the solution of Partial Differential Equations (PDEs) with the Finite Element Method (FEM) in multiple dimensions.
`ffea` can read and use [Gmsh](https://gmsh.info/) structured and unstructured meshes.

# Capabilities
## Elasticity equations
`ffea` can solve the elasticity equations<sup>1</sup>
$$\frac{\partial{\sigma_x}}{\partial{x}} + \frac{\partial{\tau_{yx}}}{\partial{y}} + \frac{\partial{\tau_{zx}}}{\partial{z}} + b_x = \rho \frac{\partial^2{u}}{\partial{t}^2}$$
$$\frac{\partial{\tau_{xy}}}{\partial{x}} + \frac{\partial{\sigma_{y}}}{\partial{y}} + \frac{\partial{\tau_{zy}}}{\partial{z}} + b_y = \rho \frac{\partial^2{v}}{\partial{t}^2}$$
$$\frac{\partial{\tau_{xz}}}{\partial{x}} + \frac{\partial{\tau_{yz}}}{\partial{y}} + \frac{\partial{\sigma_{z}}}{\partial{z}} + b_z = \rho \frac{\partial^2{w}}{\partial{t}^2}$$

subject to
- Dirichlet boundary conditions: $\textbf{u} = \bar{\textbf{u}}(\textbf{x}, t)$
- Neumann boundary conditions: $\textbf{t} = \bar{\textbf{t}}(\textbf{x}, t)$

![cantilever_beam_mesh](https://github.com/fernandezerbes/ffea/blob/main/resources/ipb200_mesh.png)

![cantilever_beam](https://github.com/fernandezerbes/ffea/blob/main/resources/ipb200_displacements.gif)
_Quasi-static structural problem modeled with `ffea`_.

## Quasi-harmonic equations
`ffea` can solve the quasi-harmonic equations<sup>1</sup>

$$-\left(\frac{\partial{q}}{\partial{x}} + \frac{\partial{q}}{\partial{x}} + \frac{\partial{q}}{\partial{x}}\right) + Q = c \frac{\partial{\phi}}{\partial{t}}$$

subject to
- Dirichlet boundary conditions: $\phi = \bar{\phi}(\textbf{x}, t)$
- Robin boundary conditions: $q_n = \bar{q}(\textbf{x}, t) + H(\textbf{x}, t) [\phi(\textbf{x}, t) - \phi_0(\textbf{x}, t)]$

This equation can be used to model heat transfer, electrostatics, and magnetostatics problems, among others.

![heat_sink_mesh](https://github.com/fernandezerbes/ffea/blob/main/resources/heat_sink_mesh.png)
![heat_sink](https://github.com/fernandezerbes/ffea/blob/main/resources/heat_sink.png)

_Steady state heat transfer problem modeled with `ffea`_.

## Other equations
Thanks to his modular, extensible design, `ffea` can solve any equation that has a similar structure to the previously mentioned.
This can be done by defining [differential operators](https://github.com/fernandezerbes/ffea/blob/main/framework/inc/model/operator.h)
and [physical regions](https://github.com/fernandezerbes/ffea/blob/main/framework/inc/model/physical_region.h) in a similar fashion as
in the [elasticity](https://github.com/fernandezerbes/ffea/tree/main/applications/elasticity)
and [quasi-harmonic](https://github.com/fernandezerbes/ffea/tree/main/applications/quasi_harmonic) applications.

## Notes
<sup>1</sup>_The transient terms are not supported yet, but the user can easily define them with the interfaces provided by the architecture. See more [here](https://github.com/fernandezerbes/ffea/blob/main/framework/inc/model/equation.h)._

# How to use
Prerequisite: `ffea` uses the [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) library for linear algebra. Follow the instructions for installation [here](https://eigen.tuxfamily.org/dox/GettingStarted.html).
1. Clone the repository and submodules with `git clone --recurse-submodules https://github.com/fernandezerbes/ffea`. Note that this will download [`vtu11`](https://github.com/phmkopp/vtu11) and [`googletest`](https://github.com/google/googletest).
1. Use CMake to build the project.
2. Create your driver by following the existing [examples](https://github.com/fernandezerbes/ffea/tree/main/examples).

# Known issues
Refer to [issues](https://github.com/fernandezerbes/ffea/issues).

# Disclaimer
`ffea` was developed exclusively for academic purposes and results are not guaranteed to be correct.
