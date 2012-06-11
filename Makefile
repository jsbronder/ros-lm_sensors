all: lm_sensors

build:
	mkdir build

lm_sensors: build
	mkdir build/lm_sensors
	cd build/lm_sensors && cmake ../../lm_sensors && make

clean:
	rm -rf build


