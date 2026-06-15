GPU_ID := 0
ROS_DOMAIN_ID := 999

BASE_TAG := base
COURSE_TAG := racing
IMAGE_NAME := curso_uah_irobocity
CONTAINER_NAME := $(IMAGE_NAME)_$(COURSE_TAG)_container

UID := $(shell id -u)
GID := $(shell id -g)
USER_NAME := $(shell whoami)


define run_docker
	@docker run -it --rm \
		--net host \
		--ipc host \
		--gpus device=$(GPU_ID) \
		--ulimit memlock=-1 \
		--ulimit stack=67108864 \
		--name=$(CONTAINER_NAME) \
		--privileged \
		-u $(USER_NAME) \
		-v ./:/ros2_ws \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /var/run/dbus:/var/run/dbus \
		-v /dev:/dev \
  		-v /run/udev:/run/udev \
		-v /home/$(USER_NAME)/librealsense/install:/opt/librealsense \
  		-e CMAKE_PREFIX_PATH=/opt/librealsense \
		-e TERM=xterm-256color \
		-e DISPLAY=$(DISPLAY) \
		-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
		$(IMAGE_NAME):$(COURSE_TAG) \
		/bin/bash -c $(1)
endef

.PHONY: build run attach clear build_base

build_base:
	docker build ./deploy -f deploy/Dockerfile.base \
		-t $(IMAGE_NAME):$(BASE_TAG) \
		--build-arg USER=$(USER_NAME) \
		--build-arg UID=$(UID) \
		--build-arg GID=$(GID)

build: build_base
	docker build ./deploy -f deploy/Dockerfile.racing -t $(IMAGE_NAME):$(COURSE_TAG) --build-arg USER=$(USER_NAME) --build-arg UID=$(UID) --build-arg GID=$(GID)
	@echo "\nBuild complete!"
	@echo "Run 'make run' to start the container."

run:
	@[ -n "$(DISPLAY)" ] && xhost +local:docker || true
	$(call run_docker, "source /ros2_ws/entrypoint.sh && bash")

attach:
	docker exec -it -e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) $(CONTAINER_NAME) /bin/bash -c "source /ros2_ws/entrypoint.sh && bash"

clear:
	@sudo rm -rf .venv/ rbsf_racing_irobocity.egg-info/
	@if [ -d "build" ]; then sudo rm -rf build/; fi
	@if [ -d "install" ]; then sudo rm -rf install/; fi
	@if [ -d "log" ]; then sudo rm -rf log/; fi
	@if [ -f "uv.lock" ]; then sudo rm uv.lock; fi
	@find . -type d -name "__pycache__" -exec sudo rm -rf {} +
	@echo "Cleaned up the project directory."
