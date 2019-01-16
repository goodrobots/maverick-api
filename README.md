# maverick-api

API for Maverick Web Interface - Highly Experimental - DO NOT USE FOR REAL WORLD APPLICATIONS YET!

[![Build Status](http://newdev.maverick.one/jenkins/buildStatus/icon?job=maverick-api/master)](http://newdev.maverick.one/jenkins/blue/organizations/jenkins/maverick-api/activity)
[![Updates](https://pyup.io/repos/github/goodrobots/maverick-api/shield.svg)](https://pyup.io/repos/github/goodrobots/maverick-api/)
[![Python 3](https://pyup.io/repos/github/goodrobots/maverick-api/python-3-shield.svg)](https://pyup.io/repos/github/goodrobots/maverick-api/)
[![Code Style](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/ambv/black)

## About
Maverick-api is the backend code which is used to drive maverick-web. The two projects are designed work together to provide a next generation web based configuration tool and ground station application for your Ardupilot and PX4 based autonomous vehicles.

The server code is primarily written in Python (3.6+ supported) and is built upon and asynchronous web framework called tornado.
For vehicle communication and control maverick-api utlises python bindings for Robot Operating System (ROS) making it agnostic when interfacing with ArduPilot and PX4 based autopilots.
To communicate with maverick-web and the browser, maverick-api exposes a graphql runtime layer using graphql-core-next to serve up an API which is accessed by maverick-web. The use of graphql over a more traditional REST API is a key component which will be discussed at a future date in the documentation. For now you will have to trust me ;)

## Documentation

As we are still in the early stages of this project, providing detailed documentation on how the project is layed out, its feature set and how can be extended is a low priority. 
I am attempting to write clear, self documenting code so if you really want to get stuck in now I would direct you towards the codebase itself.

## Installation
Currently the simplest way to install the dependencies, setup your environment and start using it is by using maverick. If you haven't looked at maverick yet, check it out for a very nice way of installing and configuring a wide range of development tools and software packages which are commonly used in the open source autonomous vehicle community.

Once the software has matured slightly, pip installable packages will be made available to simplify the installation procedure, but for now you will need to git clone the source.


## Usage

Maverick-api has been built to be extended without needing to modify or really understand the underlying server code. Simply put, you don't need to be experience in the tornado web framework to extend the API. Simply drop your custom modules into the supplied folder structure, update the JSON configuration file and you are away!
Maverick-api and maverick-web are also permissively licensed, so you can use them in your product and not have to worry about publishing your secret sauce code changes back to the community.


## Contributing 
Maverick, maveirck-api and maverick-web are architected and built in the spare time of a handful of contributors. Issues, feature requests and pull requests are highly welcomed. If you have an idea that you would like supported or just want to say ‘Hi’, feel free to drop into our gitter channel where we actively discuss development. 
