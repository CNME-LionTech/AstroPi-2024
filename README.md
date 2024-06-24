# AstroPi Project 2024 Mission

This project is designed to execute both primary and secondary missions using the AstroPi on the International Space Station (ISS).

## Primary Mission

The primary mission of this project is to determine and calculate the ISS speed in orbit. The data collected from the AstroPi's sensors will be used to perform this calculation.

#### Algorithm Description

To achieve this mission, we have developed a data acquisition algorithm that collects data from the AstroPi's sensors and saves it to a log file. The algorithm also captures images of the Earth from the AstroPi's camera and stores them in a separate directory.

- The AstroPi device will record environmental data and save it in a log file for further analysis.
- Each line in the log file will contain a collection of data points.
- Data will be collected every second, recording temperature, humidity, and pressure values along with the time of data collection.

## Secondary Mission

The secondary mission of this project is to perform image processing on the images collected during the primary mission. The goal is to identify patterns and anomalies in the images and study the effects of the space environment on Earth and the stars.

#### Data Acquisition Code Description

To achieve this mission, we have developed separate data acquisition code that captures images from the AstroPi's camera and saves them to a directory. The code also captures sensor data and stores it in a log file for later processing.

## Overview

The AstroPi Project is divided into two main parts: data acquisition and image processing.

The data acquisition part of the project collects data from the AstroPi's sensors and saves it to a file. The main entry point for this part of the project is `code/data_acquisition/main.py`, which initializes the sensors, starts data collection, and saves the data to a file.

The image processing part of the project processes the images collected by the AstroPi. The main entry point for this part of the project is `code/image_processing/process_image.py`, which contains the main logic for processing images collected by the AstroPi.

The project also includes documentation, hardware schematics, and 3D models of the AstroPi case and mounting hardware.

## Folder Structure

- `code/`: Contains the source code for the project, including the data acquisition and image processing algorithms.
- `data/`: Contains the data collected by the AstroPi during the primary mission, including the raw images and sensor data logs.
- `docs/`: Contains the project documentation, including the design documents, user manuals, and API references.
- `hardware/`: Contains the hardware schematics, 3D models, and parts list for the AstroPi.
- `reports/`: Contains the project reports, including the final report and any intermediate reports created during the project.

Please see the individual README files in each folder for more information about their contents and usage.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
See the [LICENSE](LICENSE.txt) file for license rights and limitations (MIT).
