import sys
import time
from rplidar import RPLidar

class LidarReader:
    def __init__(self, port='COM7'):
        self.lidar = RPLidar(port)

    def get_scan(self):
        try:
            for scan in self.lidar.iter_scans():
                return scan
        except Exception as e:
            print(f'Hata: {e}')
            return None

    def stop(self):
        self.lidar.stop()
        self.lidar.disconnect()

if __name__ == '__main__':
    lidar_reader = LidarReader()
    try:
        while True:
            scan = lidar_reader.get_scan()
            if scan:
                print(scan)
    except KeyboardInterrupt:
        print('Durduruluyor...')
    finally:
        lidar_reader.stop() 