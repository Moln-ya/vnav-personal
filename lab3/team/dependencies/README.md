# Runtime dependencies

Copied from the working local Lab4 dependency snapshots; these are the course
TESSE ROS bridge/interface, catkin_simple, and ETH mav_msgs. Original copyright
headers and available licenses are retained. The source snapshots are included
so this lab builds without the Lab4 directory or a changing upstream branch.

The bridge has the Python 3 compatibility fixes used by Lab4: executable Python
3 entrypoints and `import cv2` before `cv_bridge` (required on this ARM64 system).
The interface is installed with `pip --no-deps`; Python 3 provides `enum`, while
the Docker image supplies NumPy, Pillow and defusedxml. Do not install enum34.

Upstream projects: https://github.com/MIT-TESSE/tesse-ros-bridge,
https://github.com/MIT-TESSE/tesse-interface,
https://github.com/catkin/catkin_simple,
https://github.com/ethz-asl/mav_comm.
