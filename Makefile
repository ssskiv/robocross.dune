IMAGE_NAME=robocross
CONTAINER_NAME=robocross
UID:=${shell id -u}

.PHONY: all 
#auto build run start stop clean check-docker install-docker docker-nonroot

all: auto

auto: check-docker clean build run 
	@echo "\nDone. Now you can connect to container!"

build:
# docker build -t $(IMAGE_NAME) --build-arg UID=${UID} --progress=plain --no-cache .
	@echo "Building image $(IMAGE_NAME)..."
	@docker build -t $(IMAGE_NAME) -f Dockerfile.humble --build-arg UID=${UID} .

run:
	@echo "\nCreating container $(CONTAINER_NAME) with image $(IMAGE_NAME)... \n"
	@xhost +local:bmstu
	@docker run -d \
		--name $(CONTAINER_NAME) \
		--net=host \
		-u 1000 \
		--privileged \
		--env DISPLAY=${DISPLAY} \
		--env QT_X11_NO_MITSHM=1 \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="$(HOME)/.Xauthority:/bmstu/.Xauthority:rw" \
		--env XAUTHORITY=/bmstu/.Xauthority \
		--volume="$(PWD)/ros2_ws:/bmstu/ros2_ws" \
		--gpus=all \
		-e NVIDIA_DRIVER_CAPABILITIES=all \
		$(IMAGE_NAME) tail -f /dev/null
	
start:
	docker start $(CONTAINER_NAME)
	 
stop:
	docker stop $(CONTAINER_NAME)

clean:
# docker stop $(CONTAINER_NAME)	
# docker rm $(CONTAINER_NAME)
	@echo "Stopping and removing $(CONTAINER_NAME) and image $(IMAGE_NAME)..."
	@docker stop $(CONTAINER_NAME) || true && docker rm $(CONTAINER_NAME) || true
	@docker rmi $(CONTAINER_NAME)

check-docker:
	@bash -c '\
	if command -v docker >/dev/null 2>&1; then \
		if docker info >/dev/null 2>&1; then \
			echo "Docker found!"; \
		else \
			echo "Docker found, but current user cannot run it. Trying to configure."; \
			$(MAKE) docker-nonroot; \
		fi \
	else \
		$(MAKE) install-docker; \
	fi'

install-docker:
	@echo "Docker not found. Installing Docker..."
	@sudo apt-get update
	@sudo apt-get install -y \
	    ca-certificates \
	    curl \
	    gnupg \
	    lsb-release
	@sudo install -m 0755 -d /etc/apt/keyrings
	@curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
	@sudo chmod a+r /etc/apt/keyrings/docker.asc
	@echo \
	  "deb [arch=$$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
	  $$(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
	@sudo apt-get update
	@sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
	@echo "Docker installation complete! Configuring for non-root."
	@$(MAKE) docker-nonroot

docker-nonroot:
	@echo "Configuring Docker for non-root access."
	@sudo systemctl disable --now docker.service docker.socket
	@sudo rm /var/run/docker.sock
	@sudo apt-get install -y docker-ce-rootless-extras
	@dockerd-rootless-setuptool.sh install
	@echo "Docker configured! Continuing to container configuration."