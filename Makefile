IMAGE_NAME=robocross
CONTAINER_NAME=robocross
MAPPROXY_NAME=mapproxy
UID:=${shell id -u}
NVIDIA_DRIVER:=570
NVIDIA_GPU:=1
gpu=--gpus=all\

.PHONY: all 
#auto build run start stop clean check-docker install-docker docker-nonroot

all: auto

auto: check-docker check-mapproxy clean build run 
	@echo -e "\n\033[0;32m\033[1m=====Done. Now you can connect to container!====="
	
start:
	docker start $(CONTAINER_NAME) || true
	docker start $(MAPPROXY_NAME) || true
	 
# stop:
# 	@bash -c '\
# 		if docker ps --format "{{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
# 			docker stop $(CONTAINER_NAME); \
# 		fi'

# remove:
# 	@bash -c '\
# 		if docker image ps --format "{{.Names}}" | grep -q "^$(CONTAINER_NAME)$$"; then \
# 			docker rmi $(CONTAINER_NAME); \
# 		fi'

clean:
# docker stop $(CONTAINER_NAME)	
# docker rm $(CONTAINER_NAME)
	@echo "Stopping and removing $(CONTAINER_NAME) container and image $(IMAGE_NAME)..."
	@docker stop $(CONTAINER_NAME) || true && docker rm $(CONTAINER_NAME) || true
	@docker rmi $(CONTAINER_NAME) || true

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
	@curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.asc
	@sudo chmod a+r /etc/apt/keyrings/docker.gpg
	@echo \
	  "deb [arch=$$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
	  $$(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
	@sudo apt-get update
	@sudo apt-get install -y uidmap docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
	@echo "Docker installation complete! Configuring for non-root."
	@$(MAKE) docker-nonroot

docker-nonroot:
	@echo "Configuring Docker for non-root access."
	@sudo systemctl disable --now docker.service docker.socket
	@sudo rm -f /var/run/docker.sock
	@sudo apt-get install -y docker-ce-rootless-extras
	@dockerd-rootless-setuptool.sh install
	@echo "Docker configured! Continuing to container configuration."

create-mapproxy:
	@echo "Creating and starting mapproxy"
	@mkdir -p /home/developer/mapproxy
	@docker run --name ${MAPPROXY_NAME} -p 8080:8080 -d -t -v /home/developer/mapproxy://mapproxy danielsnider/mapproxy
	@echo "mapproxy started"

check-mapproxy:
	@bash -c '\
		if docker ps --format "{{.Names}}" | grep -q "^$(MAPPROXY_NAME)$$"; then \
			echo "mapproxy is running"; \
		elif docker ps -a --format "{{.Names}}" | grep -q "^$(MAPPROXY_NAME)$$"; then \
			echo "starting mapproxy"; \
			docker start ${MAPPROXY_NAME}; \
		else \
			echo "mapproxy does not exist."; \
			$(MAKE) create-mapproxy; \
		fi'

build:
# docker build -t $(IMAGE_NAME) --build-arg UID=${UID} --progress=plain --no-cache .
	@echo "Building image $(IMAGE_NAME)..."
	@docker build -t $(IMAGE_NAME) -f Dockerfile.humble --build-arg UID=${UID} --build-arg NVIDIA_DRIVER=$(NVIDIA_DRIVER) --build-arg NVIDIA_GPU=$(NVIDIA_GPU) .

run:
	@echo "\nCreating container $(CONTAINER_NAME) with image $(IMAGE_NAME)... \n"
	@xhost +
	docker run -d \
		--name $(CONTAINER_NAME) \
		--net=host \
		-u 1000 \
		--privileged \
		--env "DISPLAY" \
		--env QT_X11_NO_MITSHM=1 \
		--volume="/tmp/.X11-unix://tmp/.X11-unix:rw" \
		--volume="$(PWD)/ros2_ws://home/developer/robocross.dune/ros2_ws" \
		$(IMAGE_NAME) tail -f /dev/null

#  --volume="$(HOME)/.Xauthority://home/developer/.Xauthority:rw" \
