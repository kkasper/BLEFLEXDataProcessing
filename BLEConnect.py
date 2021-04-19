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
# import RPi.GPIO as GPIO
import struct
from struct import unpack
warnings.simplefilter("ignore", UserWarning)
sys.coinit_flags = 2

adc_data = {}
accel_data = {}
gyro_data = {}
lines = []
connected_devices = 0
DEVICE_NAME = 'GUTRUF LAB - Right Arm'
LED_PIN = 27
GRAVITY_EARTH = 9.80665
BMI2_GYR_RANGE_2000 = 0

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(LED_PIN, GPIO.OUT)
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

if os.name == 'nt':
    # addresses = ["80:EA:CA:70:00:05", "80:EA:CA:70:00:04"]
    # addresses = ["80:EA:CA:70:00:04"]
    addresses = ["80:EA:CA:70:00:05"]
else:
    addresses = []

# addresses = ["80:EA:CA:70:00:05"]
address_hashes = {}


def hash_addresses():
    global addresses
    for device_address in addresses:
        address_byte_array = bytearray.fromhex(device_address.replace(":", ""))
        address_byte_array.reverse()
        # Initialize with some random large-ish prime
        hashed_address = 5381
        # print(hex(hashed_address))
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

    if store_data:
        print(packaged_data)
        # Create dataframe from packaged data disc and write to CSV file
        output_file_name = DATA_FILE_PATH + address.replace(":", "_") + ".csv"
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


def raw_imu_notification_handler(sender, data):
    global connected_devices
    if connected_devices == len(address_hashes):
        print("IMU: [", sender, "]:", data)
        new_df = pd.DataFrame
        # Convert raw bytearray into list of processed shorts and then package it for storage
        # bytearray structure is [Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z, Address Hash, Timestamp]
        for sample_num in range(0, 2):
            start = 0 + sample_num*18
            end = 17 + sample_num*18
            sliced_data = data[start:end]
            list_of_shorts = list(unpack('h' * (int(len(sliced_data)/2) // 2), sliced_data))
            for i in range(0, 3):
                list_of_shorts[i] = (9.80665 * list_of_shorts[i] * 2) / (float((1 << 16) / 2.0))
            for i in range(3, 6):
                list_of_shorts[i] = (2000 / ((float((1 << 16) / 2.0)) + 0)) * list_of_shorts[i]

            packaged_data = {"Time:": [time.time()],
                             "Temperature:": '',
                             "Strain:": '',
                             "Battery:": '',
                             'Accel_X:': list_of_shorts[0],
                             'Accel_Y:': list_of_shorts[1],
                             'Accel_Z:': list_of_shorts[2],
                             'Gyro_X:': list_of_shorts[3],
                             'Gyro_Y:': list_of_shorts[4],
                             'Gyro_Z:': list_of_shorts[5]}

            # Convert int16_t to uint16_t
            list_of_shorts[6] = list_of_shorts[6] + 2**16
            # Find next device to have this address hash and return that address
            # print(address_hashes)
            # print(list_of_shorts[6])
            list_of_shorts[7] = int.from_bytes((sliced_data[-2::] + sliced_data[-4:-2:]), "little")
            print(list_of_shorts[7])
            # print(list_of_shorts)

            packaged_data["Device Timestamp:"] = list_of_shorts[7]
            new_df.append(packaged_data)

        device_address = next((dev for dev in address_hashes if address_hashes[dev] == list_of_shorts[6]), None)
        output_file_name = DATA_FILE_PATH + device_address.replace(":", "_") + ".csv"
        new_df.to_csv(output_file_name, index=False, header=False, mode='a')
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
            devices = await discover(timeout=2)
            for d in devices:
                # print(d)
                if d.address == address:
                    print('****')
                    print('Device found.')
                    print("Attempting connection to " + address + "...")
                    print('****')
            # print('----')

            async with BleakClient(address, loop=event_loop) as client:
                x = await client.is_connected()
                connected_devices += 1

                name = await client.read_gatt_char("00002a00-0000-1000-8000-00805f9b34fb")
                print('\nConnected to device {} ({})'.format(address, name.decode(encoding="utf-8")))
                disconnected_event = asyncio.Event()

                def disconnect_callback(client):
                    global connected_devices
                    print("Disconnected callback called!")
                    connected_devices -= 1
                    loop.call_soon_threadsafe(disconnected_event.set)
                    print("Connection lost. Retrying...")

                client.set_disconnected_callback(disconnect_callback)

                services = await client.get_services()
                for s in services:
                    for char in s.characteristics:
                        # print('Characteristic: {0}'.format(await client.get_all_for_characteristic(char)))
                        # print(f'[{char.uuid}] {char.description}:, {char.handle}, {char.properties}')
                        characteristic_names[char.handle] = (char.description + ':')
                # Temp Read

                # await client.start_notify('15005991-b131-3396-014c-664c9867b917', adc_notification_handler)
                # Strain Read
                # await client.start_notify('6eb675ab-8bd1-1b9a-7444-621e52ec6823', adc_notification_handler)
                # Battery Monitoring
                # await client.start_notify('1587686a-53dc-25b3-0c4a-f0e10c8dee20', adc_notification_handler)

                # Raw IMU Data
                await client.start_notify('2c86686a-53dc-25b3-0c4a-f0e10c8d9e26', raw_imu_notification_handler)

                await disconnected_event.wait()
                await client.disconnect()

                print("Connected: {0}".format(await client.is_connected()))
        except asyncio.exceptions.TimeoutError as e:
            print(e)
            print("----")
        except BleakError as e:
            print(e)
            print('----')


def create_csv_if_not_exist(filename_address):
    output_file_name = DATA_FILE_PATH + filename_address.replace(":", "_") + ".csv"
    if not path.exists(output_file_name):
        os.makedirs(DATA_FOLDER_PATH, exist_ok=True)
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

    print(address_hashes)
    # error catch`

    try:
        loop = asyncio.get_event_loop()
        tasks = asyncio.gather(*(connect_to_device(loop, address) for address in addresses))
        loop.run_until_complete(tasks)
    except TimeoutError as e:
        print(e)
