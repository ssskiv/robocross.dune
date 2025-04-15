IMAGE_NAME=robocross
CONTAINER_NAME=robocross

build:
	docker build -t $(IMAGE_NAME) .

run:
	xhost +local:root
	docker run -d \
		--name $(CONTAINER_NAME) \
		--net=host \
		--privileged \
		--env DISPLAY=${DISPLAY} \
		--env QT_X11_NO_MITSHM=1 \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="$(HOME)/.Xauthority:/root/.Xauthority:rw" \
		--env XAUTHORITY=/root/.Xauthority \
		--volume="$(PWD)/ros2_ws:/root/ros2_ws" \
		$(IMAGE_NAME) tail -f /dev/null
	
start:
	docker start $(CONTAINER_NAME)
	 
stop:
	docker stop $(CONTAINER_NAME)

clean:
	-docker stop $(CONTAINER_NAME)	
	-docker rm $(CONTAINER_NAME)

