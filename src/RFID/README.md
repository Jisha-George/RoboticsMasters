# RFID repository
Codes for interacting an RFID reader with ROS.

## reader

Library for interacting with the Mercury API RFID Library from Python.

### Installation
The code is built from source from the install shell script. Alternatively the makefile can be used, but these will not be installed, instead placed in the lib folder.

The shared object Mercury Library file can be compiled by
downloading the source from the ThingMagic website, and compiling with
these commands. A compiled version is already provided in this directory,
in accordance with the ThingMagic License.

Run `install.sh`  to install

Run `uninstall.sh`  to install

### Test the reader

Run the python code `rfid_test.py` to check that everything is working


## RFID ROS packages

### rfid_node
ROS interface for RFID reader and tag operations:
Handles messages for tags' data
Handles reader parameters (power,frequency band...)

### rfid_grid_map
Generates a probabilistic location costmap for each tags.

### rol_server
Creates a service to query rfid_grid_map and obtain location probabilities of tagged objects

### deprecated  
Packages no longer used
- `define_areas`
Creates rectangular regions for `rfid_grid_map`. Superseed by   `define_polygon_areas`

- `define_polygon_areas`
Creates arbitrary shape regions for `rfid_grid_map`. Tiago region definition should provide this info now.
