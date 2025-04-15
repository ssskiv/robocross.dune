IMAGE_NAME=robocross
CONTAINER_NAME=robocross

build:
	docker build -t $(IMAGE_NAME) .

run:
	xhost +local:root
	docker run -it --rm \
		--name $(CONTAINER_NAME) \
		--net=host \
		--privileged \
		--env DISPLAY=${DISPLAY} \
		--env QT_X11_NO_MITSHM=1 \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="$(HOME)/.Xauthority:/root/.Xauthority:rw" \
		--env XAUTHORITY=/root/.Xauthority \
		--volume="$(PWD)/ros2_ws:/root/ros2_ws" \
		$(IMAGE_NAME) bash
start:
	docker start $(CONTAINER_NAME) 
stop:
	-docker stop $(CONTAINER_NAME)

clean:
	-docker rm $(CONTAINER_NAME)
	-docker rmi $(IMAGE_NAME)

