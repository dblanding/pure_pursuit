"""Map utilities - SPOT for map metadata"""
import json

_metadata_cache = None

def load_map_metadata(filepath='map_metadata.json'):
    """Load map metadata (cached)"""
    global _metadata_cache
    if _metadata_cache is None:
        with open(filepath, 'r') as f:
            _metadata_cache = json.load(f)
    return _metadata_cache

def get_home_position():
    """Get home position from map metadata"""
    metadata = load_map_metadata()
    home = metadata['home_position']
    return (home['x'], home['y'], home['theta'])

def get_map_origin():
    """Get map origin"""
    metadata = load_map_metadata()
    return tuple(metadata['origin'])

def get_map_resolution():
    """Get map resolution"""
    metadata = load_map_metadata()
    return metadata['resolution']
