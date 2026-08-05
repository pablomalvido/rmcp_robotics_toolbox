# Robotics toolbox - TAU

Repository for the Robotics toolbox module of the Tampere University courses AUT.841 and ROBO.300. This repository uses a modified version of the official robotics toolbox for Python repository. For more information and documentation, check the official repository: https://github.com/petercorke/robotics-toolbox-python.

## Configure Docker in Windows
1. Download and install Docker Desktop: https://docs.docker.com/desktop/setup/install/windows-install/. You may need to restart your computer to finish the installation.

2. After installation, it may ask you to install or update wsl. In that case, write this command in the terminal: 

If not installed:
```
wsl --install
```

If requires update:
```
wsl --update
```
3. Download xlaunch for Windows: https://sourceforge.net/projects/vcxsrv/

4. Set up xlaunch: https://www.youtube.com/watch?v=qWuudNxFGOQ

5. Download Visual Studio Code: https://code.visualstudio.com/download

6. Download 'Docker' and 'Dev Containers' extensions in Visual Studio Code

## Configure Docker in Linux
**Install Docker:**

1. Update packages
```
sudo apt update
sudo apt upgrade -y
```

2. Install prerequisites
```
sudo apt install -y apt-transport-https ca-certificates curl software-properties-common gnupg lsb-release
```

3. Add Docker’s official GPG key
```
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
```

4. Add the Docker repository
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

5. Install Docker Engine
```
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

6. Enable and start Docker
```
sudo systemctl enable docker
sudo systemctl start docker
```

7. Run Docker without sudo
```
sudo usermod -aG docker $USER
newgrp docker
```

**Install Visual Studio Code:**
```
sudo apt update
sudo apt install -y wget gpg apt-transport-https software-properties-common
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update
sudo apt install -y code
```
Then, download 'Docker' and 'Dev Containers' extensions in Visual Studio Code.

## Setup in Windows
**1. Clone the Repository**

Clone the repository in a folder:
```
git clone https://github.com/pablomalvido/rmcp_robotics_toolbox.git
```

**2. Open Visual Studio Code ...**

Then, open the folder ```rmcp_robotics_toolbox``` in Visual Studio Code. Before executing the next step, Docker Desktop must be running.

**3. Choose Reopen in container when prompted**

The container will be built automatically, as required.

**4. Enable the use of your display:**

Follow these steps (check video from 17:48 to 18:40): https://youtu.be/qWuudNxFGOQ?si=XCha-tFMAlJTWY0P&t=1069

## Setup in Linux
**1. Clone the Repository (Linux branch)**

Clone the repository in a folder:
```
git clone https://github.com/pablomalvido/rmcp_robotics_toolbox.git
```

**2. Open Visual Studio Code ...**

Then, open the folder ```rmcp_robotics_toolbox``` in Visual Studio Code

**3. Modify devcontainer**

Open .devcontainer/devcontainer.json and make the following modifications:

Substitute these lines of code:

```
"containerEnv": {
        "DISPLAY": "host.docker.internal:0.0", //"${localEnv:DISPLAY}"
        "LIBGL_ALWAYS_INDIRECT": "1"
    },
```

by these:

```
"containerEnv": {
        "DISPLAY": "${localEnv:DISPLAY}"
    },
```

Uncomment these lines:

```
"mounts": [
    "source=/tmp/.X11-unix,target=/tmp/.X1-unix,type=bind"
],
```

**4. Choose Reopen in container when prompted**

The container will be built automatically, as required.

**5. Enable the use of your display:**

Run this command on your host:

```
sudo xhost +local:docker
```

## Test (Windows and Ubuntu)

After installation and setup, you can test if it works. The terminal must be open in the Docker container, meaning inside the Visual Studio window. Then, you can execute the following line:

```
python3 exercises/lecture1_intro/scara_robot.py
```

If everything works fine, this should open a GUI with your robot representation and print a table in the terminal.