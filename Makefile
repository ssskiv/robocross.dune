IMAGE_NAME=robocross
CONTAINER_NAME=robocross
UID:=${shell id -u}

build:
	# docker build -t $(IMAGE_NAME) --build-arg UID=${UID} --progress=plain --no-cache .
	docker build -t $(IMAGE_NAME) -f Dockerfile --build-arg UID=${UID} .

run:
	xhost +local:bmstu
	docker run -d \
		--name $(CONTAINER_NAME) \
		--net=host \
		-u 1000 \
		--ipc=host \
		--pid=host \
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
	-docker stop $(CONTAINER_NAME)	
	-docker rm $(CONTAINER_NAME)

