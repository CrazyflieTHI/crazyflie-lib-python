# cflib with Crazyflie CRTP sim driver

This fork of the Crazyflie Python library includes the Crazyflie CRTP link driver (sim driver) for the ROS2 Gazebo Crazyflie Flight Simulator *sim_cf2* https://github.com/CrazyflieTHI/sim_cf2

## Dependencies

Download posix_ipc module for python

```sh
pip install posix-ipc
```

## Loading the Driver

The sim driver is always initialized when calling

```python
cflib.crtp.init_drivers()
```

If no radio dongle is attached to the computer, the sim driver will be loaded and an connection to a simulated Crazyflie can be established.

## Forcing the Driver

When initializing the CRTP drivers, set `enable_sim_driver` to `true` to exclusively load the sim driver.

```python
cflib.crtp.init_drivers(enable_sim_driver=True)
```

## Scanning for Crazyflies

Addresses and their corresponding `link_uri` are hard coded in the sim driver. Scanning for address *0xE7E7E7E701* will return the `link_uri` *'radio://0/80/2M/E7E7E7E701'*

```python
linkUris = {
    0xE7E7E7E701:'radio://0/80/2M/E7E7E7E701',
    0xE7E7E7E702:'radio://0/80/2M/E7E7E7E702',
    0xE7E7E7E703:'radio://0/80/2M/E7E7E7E703',
    0xE7E7E7E704:'radio://0/80/2M/E7E7E7E704',
    0xE7E7E7E705:'radio://0/80/2M/E7E7E7E705',
    0xE7E7E7E706:'radio://0/80/2M/E7E7E7E706',
    0xE7E7E7E707:'radio://0/80/2M/E7E7E7E707',
    0xE7E7E7E708:'radio://0/80/2M/E7E7E7E708',
    0xE7E7E7E709:'radio://0/80/2M/E7E7E7E709',
    0xE7E7E7E70A:'radio://0/80/2M/E7E7E7E70A'
}
```


# cflib: Crazyflie python library [![CI](https://github.com/bitcraze/crazyflie-lib-python/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-lib-python/actions)

cflib is an API written in Python that is used to communicate with the Crazyflie
and Crazyflie 2.0 quadcopters. It is intended to be used by client software to
communicate with and control a Crazyflie quadcopter. For instance the [Crazyflie PC client](https://www.github.com/bitcraze/crazyflie-clients-python)  uses the cflib.

See [below](#platform-notes) for platform specific instruction.
For more info see our [documentation](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/).

## Installation
See the [installation instructions](docs/installation/install.md) in the github docs folder.

## Official Documentation

Check out the [Bitcraze crazyflie-lib-python documentation](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/) on our website.

## Contribute
Go to the [contribute page](https://www.bitcraze.io/contribute/) on our website to learn more.

### Test code for contribution
Run the automated build locally to test your code

	python3 tools/build/build
