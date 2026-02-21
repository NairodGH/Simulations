# <p align="center">💻 Simulations 🧊</p>

This project started off a weekend technical test I was asked by a company as part of their recruiting process (removed any elements I thought could lead people to cheat using it).\
Since I used raylib and eigen for visualization, which are solid C++ choices for other projects I had in mind, I decided to continue from their existing setup:
- download [CMake 4.0+](https://cmake.org/download/)
- then either
    - run those commands in a terminal,
    ```
    mkdir build && cd build
    cmake ..
    cmake --build .
    ```
    - run the executables located at the indicated build paths
- or, if you're on [VSCode](https://code.visualstudio.com/) like me, for a more UI approach
    - install the [Cmake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools) extension
    - Search "CMake: Set Launch/Debug Target" in the command palette and select the target you want
    - Click on the bottom left ▶️ button

Available targets:

# <p align="center">📏 Toolpath 🌀</p>

<p align="center">
    <a href="https://en.wiktionary.org/wiki/toolpath">
        <img src="assets/toolpath.gif">
    </a>
</p>

3D printer ramp generation.

Click on the "Linear" toggle to switch back and forth between it and the "Spiral" kind.\
Drag the sliders to change their respective values.\
Drag on the scene to pan around X.

# <p align="center">🦾 Kinematic 🏁</p>

<p align="center">
    <a href="https://en.wikipedia.org/wiki/Kinematics">
        <img src="assets/kinematic.gif">
    </a>
</p>

[Kinematic chain](https://en.wikipedia.org/wiki/Kinematic_chain) with [serial manipulator](https://en.wikipedia.org/wiki/Serial_manipulator).

Click on the "Axis 0 linear" toggle to switch back and forth between it and the "rotary" kind (not possible on the other axes since it'd break the arm).\
Drag the sliders to change their respective values.\
Drag on the scene to pan around and scroll the mouse wheel to zoom/unzoom.