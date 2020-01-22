# Maverick-api
API for Maverick Web Interface - Highly Experimental - DO NOT USE FOR REAL WORLD APPLICATIONS YET!

[![Code Style](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/ambv/black)

## About
Maverick-api is the backend code which is used to drive [maverick-web](https://github.com/goodrobots/maverick-web). The two projects are designed work together to provide a next generation web based configuration tool and ground station application for your [ArduPilot](http://ardupilot.org/) and [PX4](https://px4.io/) based autonomous vehicles.

The server code is primarily written in Python and is built upon an asynchronous web framework called [tornado](https://www.tornadoweb.org/en/stable/).
For vehicle communication and control maverick-api utlises [mavros](http://wiki.ros.org/mavros) making it agnostic when interfacing with [ArduPilot](http://ardupilot.org/) and [PX4](https://px4.io/) based autopilots.
To communicate with maverick-web and the browser, maverick-api exposes a [GraphQL](https://graphql.org/) runtime layer using [graphql-core-next](https://github.com/graphql-python/graphql-core-next) to serve up an API which is accessed by [maverick-web](https://github.com/goodrobots/maverick-web). The use of [GraphQL](https://graphql.org/) over a more traditional REST API is a key component which will be discussed at a future date in the documentation. For now you will have to trust me ;)

## Documentation
As we are still in the early stages of this project, providing detailed documentation on how the project is layed out, its feature set and how can be extended is a lower priority. We are rapidly adding features and modifying the codebase, so expect breaking changes in the immediate future.
Currently the best souce of documentation is the code itself. As we develop maverick-api we are striving to write clear, self documenting code, so if you would like to get stuck in now and try it out, looking over the codebase is a good place to start. If you do get stuck or have a question, feel free to head over to our [development gitter channel](https://gitter.im/goodrobots/dev).

## Installation
Currently the simplest way to install the dependencies, setup your environment and start using the project is by using [maverick](https://github.com/goodrobots/maverick). If you have come across this project and haven't looked at [maverick](https://github.com/goodrobots/maverick) yet, check it out for a very nice way of installing and configuring a wide range of development tools and software packages which are commonly used in the open source autonomous vehicle community.

Once the software has matured slightly, pip installable packages will be made available to simplify the installation procedure. However, for now you will need to git clone the source.


## Usage
Maverick-api has been built to be extended without needing to modify or understand the underlying server code. Dont worry if you have never used [tornado](https://www.tornadoweb.org/en/stable/) before, simply drop your custom modules into the supplied folder structure, update the JSON configuration file and you are away!
[Maverick](https://github.com/goodrobots/maverick), maverick-api and [maverick-web](https://github.com/goodrobots/maverick-web) are all permissively licensed, so you can use them in your product and not have to worry about legally publishing your secret sauce back to the community.


## Contributing 
[Maverick](https://github.com/goodrobots/maverick), maverick-api and [maverick-web](https://github.com/goodrobots/maverick-web) are architected and built in the spare time of a handful of contributors. Issues, feature requests and pull requests are highly welcomed. If you have an idea that you would like supported or just want to say ‘Hi’, feel free to drop into our [gitter channel](https://gitter.im/goodrobots/dev) where we actively discuss development. 
