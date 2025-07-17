all: build install

build:
	@echo "Building the package..."
	colcon build

install: build
	@echo "Installing the package..."
	cp neolux.minicon_a.service /etc/systemd/system/
	cp neolux.minicon_a.sh /usr/local/bin/
	chmod +x /usr/local/bin/neolux.minicon_a.sh
	systemctl daemon-reload

clean:
	rm -rf build/ install/ log/
