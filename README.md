# STRAP

This is the repository for the paper *Scenario-based Test Reduction and Prioritization for
Mutli-Module Autonomous Driving Systems*.

## Prerequisites
+ SVL Simulator
+ Apollo 5.0
+ GPU: NVIDIA GTX 1080/1080Ti (Apollo 5.0 doesn't support RTX series)  

## Usage
+ `add_mutants.py` and `recording_collect.py` are used for collect driving recordings
+ `segment_split.py` implements the recording segmentation and reduction
+ `prioritization.py` implements prioritization algorithms.
+ `evaluate.py` is used to run experiments 

### Driving recording collection
1. Install Apollo 5.0 by following the [instruction](https://www.svlsimulator.com/docs/system-under-test/apollo5-0-instructions/).
   Clone the repository to the root path of Apollo 5.0.
   
2. If you install Apollo 5.0 in docker, 
   1. launch the Apollo docker container: `./docker/script/dev_start.sh`
   2. Enter the container: `./docker/script/dev_into.sh`
   3. Compile Apollo: `./apollo.sh build_opt_gpu`
   
3. Install python3-pip in docker container `sudo apt-get update & sudo apt-get -y install python3-pip`

4. Install websocket-client `sudo pip3 install websocket-client`

5. run `python3 recording_collect.py <system_to_test> <number_of_mutants>`.
   `system_to_test` can be `signal, obstacle, planning, prediction`.

### Experiment reproduction
1. Install Apollo 5.0 by following the [instruction](https://www.svlsimulator.com/docs/system-under-test/apollo5-0-instructions/).
   Clone the repository to the root path of Apollo 5.0.

2. Download original driving recordings collected on three maps via the [link](https://drive.google.com/file/d/17qecR2c6Qzl2AMQPd8B5ucHBQyqlWKaP/view?usp=sharing) 
   and unzip record files into the folder `data/records`.
   
3. If you install Apollo 5.0 in docker, 
   1. launch the Apollo docker container: `./docker/script/dev_start.sh`
   2. Enter the container: `./docker/script/dev_into.sh`
   3. Compile Apollo: `./apollo.sh build_opt_gpu`
   
4. run `python recording_collect.py` to get regression recordings.
    
5. run test reduction algorithm to generate segments `python run segment_split.py`

6. run `python evaluate.py` to evaluate the test reduction and prioritization algorithms.
   
    