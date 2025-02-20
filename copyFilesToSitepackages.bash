#!/bin/bash

declare -A FILES_TO_COPY
INSTALLPARENT="."
PYTHONVER="python3.10"

FILES_TO_COPY=(
    ["./src/cone_detection_pkg/cone_detection_pkg/cone_detection_node.py"]="$INSTALLPARENT/install/cone_detection_pkg/lib/$PYTHONVER/site-packages/cone_detection_pkg/cone_detection_node.py"
    ["./src/camera_pkg/camera_pkg/camera_node.py"]="$INSTALLPARENT/install/camera_pkg/lib/$PYTHONVER/site-packages/camera_pkg/camera_node.py"
    ["./src/gui_pkg/gui_pkg/gui_node.py"]="$INSTALLPARENT/install/gui_pkg/lib/$PYTHONVER/site-packages/gui_pkg/gui_node.py"
    ["./src/lidar_pkg/lidar_pkg/lidar_node.py"]="$INSTALLPARENT/install/lidar_pkg/lib/$PYTHONVER/site-packages/lidar_pkg/lidar.py"
    ["./src/occupancy_map_pkg/occupancy_map_pkg/occupancy_node.py"]="$INSTALLPARENT/install/occupancy_map_pkg/lib/$PYTHONVER/site-packages/occupancy_map_pkg/occupancy_node.py"
    ["./src/controller/controller/controller.py"]="$INSTALLPARENT/install/controller/lib/$PYTHONVER/site-packages/controller/controller.py"
)

for SOURCE in "${!FILES_TO_COPY[@]}"; do
    DESTINATION="${FILES_TO_COPY[$SOURCE]}"

    if [ -f "$SOURCE" ]; then
        cp "$SOURCE" "$DESTINATION"
        echo "Copy: $SOURCE -> $DESTINATION"
    else
        echo "Error: Couldn't find file: $SOURCE"
    fi
done

echo "Done."