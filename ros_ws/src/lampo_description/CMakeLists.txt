cmake_minimum_required(VERSION 3.8)
project(lampo_description)


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch meshes urdf rviz map worlds config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  lampo_description/twist_repub.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
