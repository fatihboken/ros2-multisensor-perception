import os
import json

class MapTracker:
    def __init__(self, save_dir='maps'):
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)
        self.step = 0

    def save_map(self, points):
        filename = os.path.join(self.save_dir, f'map_step_{self.step}.json')
        with open(filename, 'w') as f:
            json.dump(points, f)
        self.step += 1
        print(f'Harita {filename} olarak kaydedildi.')

