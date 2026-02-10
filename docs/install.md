# Getting Started

Sugarcoat is designed for **ROS 2 Humble** and newer. Choose the installation method that best suits your needs.

:::{admonition} Prerequisites
:class: note
Ensure you have a working [ROS 2 environment](https://docs.ros.org/en/humble/Installation.html) before proceeding.
:::

::::{tab-set}

:::{tab-item} 📦 Binary (Recommended)
:sync: binary

**Best for users who want to get started quickly.**

1. **Install via APT (Ubuntu)**

The easiest way to install Sugarcoat is through our PPA or official release channels.

```bash
sudo apt install ros-$ROS_DISTRO-automatika-ros-sugar

```

2. **Or Install via `.deb`**

Download the specific version from GitHub [Releases](https://github.com/automatika-robotics/sugarcoat/releases).


```bash
sudo dpkg -i ros-$ROS_DISTRO-automatica-ros-sugar_$version$DISTRO_$ARCHITECTURE.deb

```

:::

:::{tab-item} 🛠️ Source (Advanced)
:sync: source

**Best for contributors or users needing the absolute latest features.**


```shell
# 1. Create workspace
mkdir -p ros-sugar-ws/src
cd ros-sugar-ws/src

# 2. Clone repository
git clone https://github.com/automatika-robotics/sugarcoat
cd ..

# 3. Install dependencies
pip install numpy opencv-python-headless 'attrs>=23.2.0' jinja2 msgpack msgpack-numpy setproctitle pyyaml toml

# 4. Build and source
colcon build
source install/setup.bash
```

:::

::::

## Next Steps

Now that you have Sugarcoat installed, let's build something.

:::{card} {material-regular}`rocket_launch;1.5em;sd-text-primary` Create your first Package
:link: advanced/use
:link-type: doc
:class-card: sd-text-center sd-p-4 sd-shadow-md

Learn how to create a robust, event-driven ROS2 package using Sugarcoat.
:::
