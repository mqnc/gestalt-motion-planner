
set -e

for VERSION in 20 22 24; do

    docker build --target cpp_export --build-arg UBUNTU_VERSION=${VERSION}.04 -o ./build/cpp_ubuntu${VERSION} .
    docker build --target cpp_test --build-arg UBUNTU_VERSION=${VERSION}.04 -t gestalt_motion_planner:cpptest-ubuntu${VERSION} .
    docker run --rm gestalt_motion_planner:cpptest-ubuntu${VERSION}

    docker build --target python_export --build-arg UBUNTU_VERSION=${VERSION}.04 -o ./build/python .
    docker build --target python_test --build-arg UBUNTU_VERSION=${VERSION}.04 -t gestalt_motion_planner:pytest-ubuntu${VERSION} .

    docker run --name temp gestalt_motion_planner:pytest-ubuntu${VERSION}
    docker cp temp:/app/demo.log.html ./build/demo.ubuntu${VERSION}.html
    docker rm temp

done
