# libfranka-docker

Docker Images for portable [libfranka](https://frankaemika.github.io/docs/libfranka.html) execution for both [franka_ros](https://frankaemika.github.io/docs/franka_ros.html) and [franka_ros2](https://frankaemika.github.io/docs/franka_ros2.html).

<p align="center">
  <img src="https://github.com/user-attachments/assets/6473fe97-a34c-4f46-87c3-77ae1cc42b18" width="700" alt="Screenshot 2024-08-07 222542">
</p>

## Requirements

The use of this repo requires [Docker](https://docs.docker.com/engine/install/) and the [Docker Compose Plugin](https://docs.docker.com/compose/install/linux/) of minimum version 2. It is recommended to use Ubuntu as underlying operating system or at least a suitable WSL environment.

## Installation

Get the repository with 

	git clone --recursive https://github.com/Fjakob/libfranka-docker.git

The libraries `libfranka`, `franka_ros` and `franka_ros2` will be pulled as submodules upon cloning.

You might want to make sure to have the latest commit of all submodules by running

	git submodule update --init --recursive

 

## Usage

In the terminal, `cd` to the `docker_launch_files` directory. Dependent on whether you want to use franka_ros or franka_ros2, run either of the following

	docker compose run franka_ros
	docker compose run franka_ros2

or run

	docker compose up

to start both containers.

To enable display mounting (e.g. for RVIZ or Gazebo), run

	xhost +local:docker
	
on the host machine.

## Remarks

* The workspace in the container mirrors the [docker_volume](/docker_volume/) directory in a respective docker volume, such that changes made in the container are also visible on your host machine
* Using the VS Code [Docker](https://code.visualstudio.com/docs/containers/overview) and [Remote Development](https://code.visualstudio.com/docs/remote/remote-overview) extension is recommended, to attach shells or VS code itself to the container.
* Note that [franka_ros2](https://frankaemika.github.io/docs/franka_ros2.html#) is currently (08/2024) beta software and yet encounter issues.
