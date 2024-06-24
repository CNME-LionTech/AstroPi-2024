if __name__ == "__main__":
    # Description:  Program written by Team LionTech from
    #               Colegiul National "Mihai Eminescu" Oradea, Romania for
    #               Phase 2 of AstroPi Mission Space Lab competition 2023/2024
    # Team Members: Maia Budur, Rianna Ciobanu, Cristina Andor,
    #               Andrei Tigan, Serban Husaru, David Chereches
    # Teacher:      Daniel Erzse, Amelia Stoian
    
    
    # Libraries needed for experiment
    from sense_hat import SenseHat
    import datetime as dt
    from time import sleep
    from picamera import PiCamera
    import os, csv, sys
    from skyfield.api import load, load_file, Loader, wgs84, EarthSatellite
    from skyfield.api import load, Topos
    from skyfield import almanac
    from math import degrees, radians, hypot
    from orbit import ISS, ephemeris
    import dateutil.parser
    from exif import Image
    import cv2
    from collections import deque
    
    # SenseHat Initialisation
    sense = SenseHat()
    sense.clear()
    
    # Starting the camera
    cam = PiCamera()
    cam.resolution = (4056, 3040)
    
    # Mission parameters
    missiontime = 8       # Running time in minutes
    team = 'LionTech'       # AstroPi team name
    photo_counter = 1
    photo_delay = 4
    GDS = 12648
    speeds = []

    images_deque = deque()
    
    # Working path definition
    dir_path = os.path.dirname(os.path.realpath(__file__))
    
    # Datetime variables to store the start time and the current time
    start_time = dt.datetime.now()
    now_time = dt.datetime.now()
    last_photo_time = dt.datetime.now()
    file_time_stamp = start_time.strftime('%Y%b%d_%Hh%Mm%Ss')
    
    # Create folders for images and logs
    ltdata = dir_path + '/LTdata'
    if (not os.path.exists(ltdata)):
        os.mkdir(ltdata)
    if (not os.path.exists(ltdata + "/logs/")):
        os.mkdir(ltdata + "/logs/")
    if (not os.path.exists(ltdata + "/images/")):
        os.mkdir(ltdata + "/images/")
    if (not os.path.exists(ltdata + '/images/{}'.format(file_time_stamp))):
        os.mkdir(ltdata + '/images/{}'.format(file_time_stamp))
    logfilename = ltdata + '/logs/LionTech_Log_{}.csv'.format(file_time_stamp)
    errorfilename = ltdata + '/logs/Errors{}.txt'.format(file_time_stamp)
    
    ## Download ephemeris
    #try: 
    #    ephem_dir = dir_path + "/Ephem"
    #    if not os.path.exists(ephem_dir):
    #        os.mkdir(ephem_dir)
    #    ephem_path = 'de421.bsp'
    #    if os.path.exists(ephem_path):
    #        ephemeris= load_file(ephem_path)  # ephemeris DE421
    #    else:
    #        load = Loader(ephem_dir)
    #        ephemeris = load('de421.bsp')

    #except Exception as e:
    #    with open(errorfile, 'w') as f:
    #        f.write('Mission error (load ephemeris): ' + str(e))


    def convert(angle):
        """ Convert an ephem angle (degrees:minutes:seconds) to an
        EXIF-appropriate representation (rationals). Return a tuple
        containing a boolean and the converted angle, with the
        boolean indicating if the angle is negative.
        """

        sign, degrees, minutes, seconds = angle.signed_dms()
        exif_angle = f'{degrees:.0f}/1,{minutes:.0f}/1,{seconds*10:.0f}/10'
        return sign < 0, exif_angle


    def ISS_positions(): 
        """
        The function computes the current position of the International Space Station (ISS) and creates 
        an observer at that position to determine whether it is day or night at that location. 
        It does this by checking the altitude of the Sun relative to the observer's position. If the 
        altitude is above the horizon, then it is day, and if it is below the horizon, then it is night.
        """
        try:
            # Obtain the current time `t`
            t = load.timescale().now()

            # TLE data for ISS          
            line1 = '1 25544U 98067A   24049.71026637  .00021015  00000+0  37375-3 0  9997'
            line2 = '2 25544  51.6407 187.4593 0001742 276.4949 214.7207 15.50111483440029'


            # Get the position of the ISS at the current time
            iss = EarthSatellite(line1, line2)
            iss_position = iss.at(t)

            # Compute the coordinates of the Earth location directly beneath the ISS
            location = iss_position.subpoint()

            lon = round(location.longitude.degrees, 6)
            lat = round(location.latitude.degrees, 6)
            height = round(location.elevation.m, 3)

            # Convert the latitude and longitude to EXIF-appropriate representations
            south, exif_latitude = convert(location.latitude)
            west, exif_longitude = convert(location.longitude)

            # Set the EXIF tags specifying the current location
            cam.exif_tags['GPS.GPSLatitude'] = exif_latitude
            cam.exif_tags['GPS.GPSLatitudeRef'] = "S" if south else "N"
            cam.exif_tags['GPS.GPSLongitude'] = exif_longitude
            cam.exif_tags['GPS.GPSLongitudeRef'] = "W" if west else "E"

            return lat, lon, height
            
        except AttributeError:
            print(AttributeError)
    

    def custom_capture(iss, camera, image):
        # Use `camera` to capture an `image` file with lat/long Exif data
        point = iss.coordinates()

        # Convert the latitude and longitude to Exif-appropriate
        # representations
        south, exif_latitude = convert(point.latitude)
        west, exif_longitude = convert(point.longitude)

        # Set the Exif tags specifying the current location
        camera.exif_tags['GPS.GPSLatitude'] = exif_latitude
        camera.exif_tags['GPS.GPSLatitudeRef'] = "S" if south else "N"
        camera.exif_tags['GPS.GPSLongitude'] = exif_longitude
        camera.exif_tags['GPS.GPSLongitudeRef'] = "W" if west else "E"

        # Capture the image
        camera.capture(image)

    
    def readAccelerations():
        """ Function for reading the accelerations on the three axes.
        The reading contains the acceleration on x, y and z-axis, in
        variables rawAccelX, rawAccelY and rawAccelZ.
        """
    
        sense.set_imu_config(False, False, True)
        rawAccel = sense.get_accelerometer_raw()
        rawAccelX = round(rawAccel['x'],3)
        rawAccelY = round(rawAccel['y'],3)
        rawAccelZ = round(rawAccel['z'],3)
        return rawAccelX, rawAccelY, rawAccelZ
    
    
    def readCompass():
        """ Function for reading the data from the compass. The readings
        contains both the direction of the North, in variable comp, and
        the magnetic intensity on x, y and z-axis, in variables rawCompX,
        rawCompY and rawCompZ.
        """
    
        sense.set_imu_config(True, False, False)
        comp = round(sense.get_compass(),3)
        rawComp = sense.get_compass_raw()
        rawCompX = round(rawComp['x'],3)
        rawCompY = round(rawComp['y'],3)
        rawCompZ = round(rawComp['z'],3)
    
        return comp, rawCompX, rawCompY, rawCompZ
    
    
    def readOrientation():
        """ Function for reading the SenseHat orientation data. The readings 
        contain the pitch, roll and yaw as separate values.
        """
    
        sense.set_imu_config(False, True, True)
        rawAccel = sense.get_orientation()
        pitch = round(rawAccel['pitch'],2)
        roll = round(rawAccel['roll'],2)
        yaw = round(rawAccel['yaw'],2)
        return pitch, roll, yaw
    
    
    def create_csv(data_file):
        """ Creates the CSV file and the header of the log file.
        """
        
        with open(data_file, 'w') as f:
            writer = csv.writer(f)
            header = ("Team","Timestamp","Longitude", "Latitude","Height","Temperature","Temp_from_pressure","Humidity","Pressure","AccelX","AccelY","AccelZ","CompassMag","CompassX","CompassY","CompassZ","Pitch","Roll","Yaw")
            writer.writerow(header)
    
    
    def add_csv_data(data_file, data):
        """ Function to add data to the CSV log file.
        """
        
        with open(data_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(data)
    
    
    def get_time(image):
        with open(image, 'rb') as image_file:
            img = Image(image_file)
            time_str = img.get("datetime_original")
            time = dt.datetime.strptime(time_str, '%Y:%m:%d %H:%M:%S')
        return time
    
    
    def get_time_difference(image_1, image_2):
        time_1 = get_time(image_1)
        time_2 = get_time(image_2)
        time_difference = time_2 - time_1
        return time_difference.seconds
        
    
    def convert_to_cv(image_1, image_2):
        image_1_cv = cv2.imread(image_1, 0)
        image_2_cv = cv2.imread(image_2, 0)
        return image_1_cv, image_2_cv
    

    
    def calculate_features(image_1, image_2, feature_number):
        orb = cv2.ORB_create(nfeatures = feature_number)
        keypoints_1, descriptors_1 = orb.detectAndCompute(image_1, None)
        keypoints_2, descriptors_2 = orb.detectAndCompute(image_2, None)
        return keypoints_1, keypoints_2, descriptors_1, descriptors_2
    
    
    def calculate_matches(descriptors_1, descriptors_2):
        brute_force = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = brute_force.match(descriptors_1, descriptors_2)
        matches = sorted(matches, key=lambda x: x.distance)
        return matches
    
    
    def display_matches(image_1_cv, keypoints_1, image_2_cv, keypoints_2, matches):
        match_img = cv2.drawMatches(image_1_cv, keypoints_1, image_2_cv, keypoints_2, matches[:100], None)
        resize = cv2.resize(match_img, (1600,600), interpolation = cv2.INTER_AREA)
        cv2.imshow('matches', resize)
        cv2.waitKey(0)
        cv2.destroyWindow('matches')
     
    
    def find_matching_coordinates(keypoints_1, keypoints_2, matches):
        coordinates_1 = []
        coordinates_2 = []
        for match in matches:
            image_1_idx = match.queryIdx
            image_2_idx = match.trainIdx
            (x1,y1) = keypoints_1[image_1_idx].pt
            (x2,y2) = keypoints_2[image_2_idx].pt
            coordinates_1.append((x1,y1))
            coordinates_2.append((x2,y2))
        return coordinates_1, coordinates_2
    
    
    def calculate_mean_distance(coordinates_1, coordinates_2):
        all_distances = 0
        merged_coordinates = list(zip(coordinates_1, coordinates_2))
        for coordinate in merged_coordinates:
            x_difference = coordinate[0][0] - coordinate[1][0]
            y_difference = coordinate[0][1] - coordinate[1][1]
            # print(x_difference, y_difference)
            distance = hypot(x_difference, y_difference)
            # print('Distance: {}'.format(distance))
            all_distances = all_distances + distance
        return all_distances / len(merged_coordinates)
    
    
    def calculate_speed_in_kmps(feature_distance, GSD, time_difference):
        distance = feature_distance * GSD / 100000
        speed = distance / time_difference
        return speed
    
    
    def process_images(img_1, img_2, time_diff):
        # time_diff = get_time_difference(img_1, img_2)
        img_1_cv, img_2_cv = convert_to_cv(img_1, img_2)
        
        # Get keypoints and descriptors
        kp_1, kp_2, descriptors_1, descriptors_2 = calculate_features(img_1_cv, img_2_cv, 1000)

    
        # Match descriptors 
        matches = calculate_matches(descriptors_1, descriptors_2) 
        
        coordinates_1, coordinates_2 = find_matching_coordinates(kp_1, kp_2, matches)
        average_feature_distance = calculate_mean_distance(coordinates_1, coordinates_2)
        speed = calculate_speed_in_kmps(average_feature_distance, GDS, time_diff)
        return speed
    
    def write_result(speed):
        # Format the estimate_kmps to have a precision
        # of 5 significant figures
        estimate_kmps_formatted = "{:.4f}".format(speed)

        # Create a string to write to the file
        output_string = estimate_kmps_formatted

        # Write to the file
        file_path = "result.txt"  # Replace with your desired file path
        with open(file_path, 'w') as file:
            file.write(output_string)

        print("\n  Data written to", file_path)
    
    # main programm
    print('LionTech Mission Space Lab')
    print('- Systems checked at:         {}'.format(now_time.strftime('%Y-%b-%d %Hh%Mm%Ss')))
    print('- Data collection started at: {}'.format(now_time.strftime('%Y-%b-%d %Hh%Mm%Ss')))
    print('- Collecting data .................................')
    
    create_csv(logfilename)
    #print("Longitude, Latitude, Height,Temperature,temp_p,humidity,pressure,accel, compass, orient")
    
    while (now_time < start_time + dt.timedelta(minutes=missiontime)):
    
        try:
            # Update the current time
            # sleep(0.25)
            now_time = dt.datetime.now()

            # Compute ISS location
            lat, lon, height = ISS_positions()
            
            # Read the temperature, humidity, pressure and other data from the SenseHat
            temperature = round(sense.get_temperature(), 3)
            temp_p = round(sense.get_temperature_from_pressure(), 3)
            humidity = round(sense.get_humidity(), 3)
            pressure = round(sense.get_pressure(), 3)
            accelX, accelY, accelZ = readAccelerations()
            compassMag, compassX, compassY, compassZ = readCompass()
            pitch, roll, yaw = readOrientation()
            
            # Zip readings in a single variable and write them to the csv
            rowdata = (team, now_time, lon, lat, height, temperature,temp_p,humidity,pressure,accelX,accelY, accelZ, compassMag, compassX, compassY, compassZ, pitch, roll, yaw)
            add_csv_data(logfilename, rowdata)
    	    # print(team,now_time,lon,lat,temperature,temp_p,humidity,pressure,accelX,accelY, accelZ, compassMag, compassX, compassY, compassZ, pitch, roll, yaw)
    
            # Image capture
            if now_time > last_photo_time + dt.timedelta(seconds = photo_delay):
                photo_name = ltdata + '/images/{}/LTech_{:04d}.jpg'.format(file_time_stamp, photo_counter) 
                if photo_counter >1:
                    last_photo_name = photo_name
                if len(images_deque) >= 42:
                    os.unlink(images_deque.popleft())
                custom_capture(ISS(), cam, photo_name)
                images_deque.append(photo_name)

                if photo_counter >1:
                    timediff = (now_time -last_photo_time).seconds
                    speed = process_images(last_photo_name, photo_name, timediff)
                    speeds.append(speed)
                
                last_photo_time = now_time
                print('-  Processing image:           {}'.format(photo_counter))
                photo_counter +=1
    
        except Exception as e:
            with open(errorfilename, 'w') as f:
                f.write('Mission error: ' + str(e))
    
    estimated_speed = round(sum(speeds)/len(speeds),4)
    write_result(estimated_speed)
    print('- Mission ended at:           {}'.format(now_time.strftime('%Y-%b-%d %Hh%Mm%Ss')))
    print('- Total runtime:              {}'.format((now_time - start_time)))
    print('- Average speed for ISS:     {}'.format(estimated_speed))
    print('Mission accomplished - Team LionTech')
    sense.clear()
