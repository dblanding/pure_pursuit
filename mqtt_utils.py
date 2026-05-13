"""MQTT utilities - connection helper"""
import paho.mqtt.client as mqtt
import socket

def get_broker_address():
    """
    Get MQTT broker address based on where we're running
    
    Returns 'localhost' if on raspibot, 'raspibot.local' otherwise
    """
    hostname = socket.gethostname()
    if hostname == 'raspibot':
        return 'localhost'
    else:
        return 'raspibot.local'

def create_mqtt_client(client_id=None):
    """
    Create and return configured MQTT client
    
    Args:
        client_id: Optional client ID (default: auto-generated)
    
    Returns:
        Configured paho.mqtt.Client
    """
    if client_id:
        client = mqtt.Client(client_id)
    else:
        client = mqtt.Client()
    
    return client

def connect_mqtt_client(client, on_connect=None, on_message=None):
    """
    Connect MQTT client to broker
    
    Args:
        client: paho.mqtt.Client instance
        on_connect: Callback for connection event
        on_message: Callback for message event
    
    Returns:
        broker_address used
    """
    if on_connect:
        client.on_connect = on_connect
    if on_message:
        client.on_message = on_message
    
    broker = get_broker_address()
    
    try:
        client.connect(broker, 1883, 60)
        return broker
    except Exception as e:
        print(f"❌ Failed to connect to MQTT broker at {broker}: {e}")
        raise
