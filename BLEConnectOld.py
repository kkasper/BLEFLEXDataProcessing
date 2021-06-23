import ctypes

from bleak import BleakClient, discover
from bleak.exc import BleakError
import asyncio
import pandas as pd
import os
import sys
from os import path
import time
import warnings
import RPi.GPIO as GPIO
import struct
from struct import unpack
warnings.simplefilter("ignore", UserWarning)
sys.coinit_flags = 2
adc_data = {}
accel_data = {}
gyro_data = {}
lines = []
connected_devices = 0
LED_PIN = 27
GRAVITY_EARTH = 9.80665
BMI2_GYR_RANGE_2000 = 0
NUMBER_OF_READINGS = 20

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
pin_flash_cycle_duration = 0
DATA_FILE_PATH = os.path.join(os.path.dirname(__file__), "data/")
DATA_FOLDER_PATH = os.path.join(os.path.dirname(__file__), "data")
characteristic_names = {
     39: 'Temperature:',
     43: 'Strain:',
     54: 'Gyro_X:',
     58: 'Gyro_Y:',
     62: 'Gyro_Z:',
     66: 'Accel_X:',
     70: 'Accel_Y:',
     74: 'Accel_Z:',
     86: 'Battery:'
}

global output_file_name
global pin_flash_cycle_number


if os.name == 'nt':
    # addresses = ["80:EA:CA:70:00:07","80:EA:CA:70:00:06", "80:EA:CA:70:00:04"]
    addresses = ["80:EA:CA:70:00:11"]
    # addresses = ["80:EA:CA:70:00:07"]
else:
    addresses = ["80:EA:CA:70:00:11"]

# addresses = ["80:EA:CA:70:00:05"]
address_hashes = {}
address_filePaths = {}


def hash_addresses():
    global addresses
    for device_address in addresses:
        address_byte_array = bytearray.fromhex(device_address.replace(":", ""))
        address_byte_array.reverse()
        # print(address_byte_array)
        # Initialize with some random large-ish prime
        hashed_address = 5381
        # print("")
        for b in address_byte_array:
            hashed_address = ((hashed_address << 5) + hashed_address) + b
            hashed_address &= 0xFFFF

            # print(hex(hashed_address))
            # print(" ::", hex(b))
            # print(hashed_address)
        # print(hex(hashed_address))

        address_hashes[device_address] = hashed_address


# Key data storage
def store_data_as_csv():
    global adc_data
    global accel_data
    global gyro_data
    store_data = False
    # Create new dict to populate and convert to data frame
    packaged_data = {"Time:": [time.time()]}

    # Check if the adc data is ready, if so prepend it. Otherwise use empty strings
    if len(adc_data.keys()) > 2:
        packaged_data['Temperature:'] = adc_data.pop('Temperature:')
        packaged_data['Strain:'] = adc_data.pop('Strain:')
        packaged_data['Battery:'] = adc_data.pop('Battery:')
        store_data = True
    else:
        packaged_data['Temperature:'] = ""
        packaged_data['Strain:'] = ""
        packaged_data['Battery:'] = ""

    # Make sure both accelerometer and gyro data dicts are populated
    if (len(accel_data.keys()) > 2) and (len(gyro_data.keys()) > 2):
        store_data = True
        # Insert XYZ accelerometer and gyroscope
        for key in sorted(accel_data.keys()):
            packaged_data[key] = accel_data.pop(key)

        for key in sorted(gyro_data.keys()):
            packaged_data[key] = gyro_data.pop(key)

    else:
        packaged_data['Gyro_X:'] = ""
        packaged_data['Gyro_Y:'] = ""
        packaged_data['Gyro_Z:'] = ""
        packaged_data['Accel_X:'] = ""
        packaged_data['Accel_Y:'] = ""
        packaged_data['Accel_Z:'] = ""

    global output_file_name
    global pin_flash_cycle_number

    if store_data:
        pin_flash_cycle_number += 1
        if pin_flash_cycle_number > 200:
            GPIO.output(LED_PIN, 1)
            if pin_flash_cycle_number > 210:
                pin_flash_cycle_number = 0
                GPIO.output(LED_PIN, 0)
        print(packaged_data)
        # Create dataframe from packaged data disc and write to CSV file
        # output_file_name = DATA_FILE_PATH + address.replace(":", "_") + ".csv"
        new_df = pd.DataFrame(packaged_data)
        new_df.to_csv(output_file_name,
                      index=False,
                      header=False,
                      mode='a'  # append data to csv file
                      )


def adc_notification_handler(sender, data):
    global adc_data
    global pin_flash_cycle_duration
    char_name = characteristic_names[sender]
    # if char_name == 'Temperature:':
    #     GPIO.output(LED_PIN, 1)
    #     pin_flash_cycle_duration += 1
    # print(sender, int.from_bytes(data, byteorder='little'))
    adc_data[char_name] = [int.from_bytes(data, byteorder='little')]

    # if char_name == 'Temperature:' and pin_flash_cycle_duration >= 5:
    #     GPIO.output(LED_PIN, 0)
    #     pin_flash_cycle_duration = 0

    if len(adc_data.keys()) > 2:
        # print(adc_data)
        store_data_as_csv()


def gyro_notification_handler(sender, data):
    global gyro_data
    # Convert characteristic id number to corresponding characteristic name
    char_name = characteristic_names[sender]
    gyro_data[char_name] = [struct.unpack('f', data)]
    store_data_as_csv()


def accel_notification_handler(sender, data):
    global accel_data
    char_name = characteristic_names[sender]
    accel_data[char_name] = [struct.unpack('f', data)]
    store_data_as_csv()


def gait_notification_handler(sender, data):
    global connected_devices
    if connected_devices == len(address_hashes):
        print("IMU: [", sender, "]:", data)
        list_of_shorts = list(unpack('h' * (len(data) // 2), data))
        print(list_of_shorts)
        list_of_shorts[NUMBER_OF_READINGS*4] = list_of_shorts[NUMBER_OF_READINGS*4] + 2 ** 16

        for i in range(0, NUMBER_OF_READINGS):
            # Convert raw bytearray into list of processed shorts and then package it for storage
            # bytearray structure is [Accel Z, Gyro Z, Address Hash, Timestamp]

            list_of_shorts[0 + i*4] = (9.80665 * list_of_shorts[0 + i*4] * 2) / (float((1 << 16) / 2.0))
            list_of_shorts[1 + i*4] = (2000 / ((float((1 << 16) / 2.0)) + 0)) * list_of_shorts[1 + i*4]

            packaged_data = {"Time:": [time.time()],
                             "Temperature:": '',
                             "Strain:": '',
                             "Battery:": '',
                             'Accel_X:': '',
                             'Accel_Y:': list_of_shorts[0 + i*4],
                             'Accel_Z:': '',
                             'Gyro_X:': '',
                             'Gyro_Y:': list_of_shorts[1 + i*4],
                             'Gyro_Z:': '',
                             'Device Timestamp:': ''}

            # Convert int16_t to uint16_t
            print(list_of_shorts)

            device_address = next((dev for dev in address_hashes if address_hashes[dev] == list_of_shorts[NUMBER_OF_READINGS*4]), None)

            # list_of_shorts[2 + i*4] = int.from_bytes((data[6 + i*4:8 + i*4:] + data[4 + i*4:6 + i*4:]), "little")
            list_of_shorts[2 + i*4] = int.from_bytes((data[6 + i * 8:8 + i * 8:] + data[4 + i * 8:6 + i * 8:]), "little")
            # print(list_of_shorts)
            packaged_data["Device Timestamp:"] = list_of_shorts[2 + i*4]
            # print(packaged_data)

            output_file_name = address_filePaths[device_address]
            new_df = pd.DataFrame(packaged_data)
            new_df.to_csv(output_file_name, index=False, header=False, mode='a')
        # print(list_of_shorts)
    else:
        pass


def battery_notification_handler(sender, data):
    global adc_data
    print(sender, int.from_bytes(data, byteorder='little'))
    adc_data['Battery:'] = int.from_bytes(data, byteorder='little')
    if len(adc_data.keys()) > 2:
        store_data_as_csv()


async def connect_to_device(event_loop, address):
    global connected_devices
    while True:
        try:
            print("Attempting connection to " + address + "...")

            devices = await discover(timeout=2)
            for d in devices:

                if d.name not in ["Unknown", "Microsoft", "Apple, Inc.", "", "LE_WH-1000XM4"]:
                    print(d)

            async with BleakClient(address, loop=event_loop) as client:
                GPIO.output(LED_PIN, 0)
                x = await client.is_connected()
                connected_devices += 1
                print("Connected to " + str(connected_devices) + " devices out of " + str(len(address_hashes)) + ".")
                name = await client.read_gatt_char("00002a00-0000-1000-8000-00805f9b34fb")
                print('\nConnected to device {} ({})'.format(address, name.decode(encoding="utf-8")))
                disconnected_event = asyncio.Event()

                def disconnect_callback(client):
                    global connected_devices
                    GPIO.output(LED_PIN, 1)
                    print("Disconnected callback called!")
                    connected_devices -= 1
                    loop.call_soon_threadsafe(disconnected_event.set)
                    print("Connection lost. Retrying...")

                client.set_disconnected_callback(disconnect_callback)

                # services = await client.get_services()
                # for s in services:
                #     for char in s.characteristics:
                        # print('Characteristic: {0}'.format(await client.get_all_for_characteristic(char)))
                        # print(f'[{char.uuid}] {char.description}:, {char.handle}, {char.properties}')
                        # characteristic_names[char.handle] = (char.description + ':')
                # Temp Read

                await client.start_notify('15005991-b131-3396-014c-664c9867b917', adc_notification_handler)
                # Strain Read
                await client.start_notify('6eb675ab-8bd1-1b9a-7444-621e52ec6823', adc_notification_handler)
                # Battery Monitoring
                await client.start_notify('1587686a-53dc-25b3-0c4a-f0e10c8dee20', adc_notification_handler)

                # Raw IMU Data
                # await client.start_notify('2c86686a-53dc-25b3-0c4a-f0e10c8d9e26', gait_notification_handler)

                await disconnected_event.wait()
                await client.disconnect()

                print("Connected: {0}".format(await client.is_connected()))
        except Exception:
            print("Didn't connect to " + address + " in time.")

        except BleakError as e:
            print(e)
            print('----')


def create_csv_if_not_exist(filename_address):
    global output_file_name
    output_file_name = DATA_FILE_PATH + filename_address.replace(":", "_") + ".csv"
    if not path.exists(output_file_name):
        os.makedirs(DATA_FOLDER_PATH, exist_ok=True)
    else:
        num = 1
        while path.exists(output_file_name):
            output_file_name = DATA_FILE_PATH + filename_address.replace(":", "_") + "(" + str(num) + ")" ".csv"
            num += 1

    address_filePaths[filename_address] = output_file_name
    # print(output_file_name)
    new_file_headers = pd.DataFrame(columns=['Time:', 'Temperature:', 'Strain:', 'Battery:',
                                             "Accel_X:", "Accel_Y:", "Accel_Z:", "Gyro_X:",
                                             "Gyro_Y:", "Gyro_Z:", "Device Timestamp:"])
    new_file_headers.to_csv(output_file_name, encoding='utf-8', index=False)


if __name__ == "__main__":
    global handle_desc_pairs
    connected_devices = 0
    hash_addresses()

    # GPIO.output(LED_PIN, 0)
    for address in addresses:
        create_csv_if_not_exist(address)
    global pin_flash_cycle_number
    pin_flash_cycle_number = 0
    print(address_hashes)
    # print(address_filePaths)
    # error catch`
    GPIO.output(LED_PIN, 1)

    try:
        loop = asyncio.get_event_loop()
        tasks = asyncio.gather(*(connect_to_device(loop, address) for address in addresses))
        loop.run_until_complete(tasks)
    except TimeoutError as e:
        print(e)
