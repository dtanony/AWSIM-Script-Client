## AWSIM-Script-Client

AWSIM-Script is a scenario description language for [Autoware autonomous driving system](https://github.com/dtanony/autoware0412) 
and [AWSIM-Labs simulator](https://github.com/dtanony/AWSIM-Labs).
Scenarios can be specified in script files
([example here](https://github.com/duongtd23/AWSIM-Labs/tree/v1.3?tab=readme-ov-file#awsim-script))
and fed into AWSIM-Labs 
to simulate the behavior of traffic participants and 
into Autoware to trigger the autonomous driving task.
The grammar of AWSIM-Script is available at https://github.com/dtanony/AWSIM-Labs/blob/main/Assets/AWSIM/Scripts/AWSIM-Script/Generated-code/AWSIMScriptGrammar.g4.

This repository provides a client library for sending such scenario scripts to Autoware and AWSIM-Labs.

### Usage
1. Install [AWSIM-Labs](https://github.com/dtanony/AWSIM-Labs) and [Autoware](https://github.com/dtanony/autoware0412) 
by following their installation instructions.

2. Clone this repo
```bash
git clone https://github.com/dtanony/AWSIM-Script-Client.git
cd AWSIM-Script-Client
source ~/autoware/install/setup.bash
```
Note that you need to source Autoware's setup file before launching the monitor.
In the commands above, Autoware is assumed to be installed in the home directory (`~`). 
If it is installed elsewhere, update the path accordingly.

3. Launch Autoware and AWSIM-Labs, making sure that they are connected.
You might want to launch [AW-Runtime-Monitor](https://github.com/dtanony/AW-Runtime-Monitor)
as well.

4. Launch scenario
```bash
python client.py <path-to-script-file>
```

You can also execute a sequence of scenarios by providing the path to 
a folder containing multiple `.script` files. 
Each scenario will terminate once the ego vehicle reaches its goal.

### Using with AW-Runtime-Monitor
To use the client with AW-Runtime-Monitor, 
simply launch the monitor after Autoware and AWSIM-Labs are connected, 
and before sending the input script(s).