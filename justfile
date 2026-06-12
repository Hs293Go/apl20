set shell := ["bash", "-c"]

build *extra_cmake_args:
  cd ../../ && colcon build --symlink-install --event-handler=console_direct+ --packages-up-to apl20_ros --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=ON \
  -DCMAKE_POLICY_DEFAULT_CMP0144=NEW \
  -DCMAKE_POLICY_DEFAULT_CMP0148=OLD \
  -DCMAKE_POLICY_DEFAULT_CMP0167=NEW \
  -DCMAKE_INSTALL_MESSAGE=LAZY \
  {{extra_cmake_args}}

test: build
  rm -rf ../../build/apl20/test_results ../../build/apl20_ros/test_results
  cd ../../ && colcon test --packages-select apl20 apl20_ros --event-handler=console_direct+ --return-code-on-test-failure && colcon test-result --verbose --test-result-base build/apl20

