import numpy as np
import time
import Adafruit_ADS1x15
import subprocess
import pyrebase
import torch
import torch.nn as nn
import torch_geometric
from torch_geometric.data import Data
from torch_geometric.nn import GCNConv

# Create Firebase configuration
config = {
    "apiKey": "AIzaSyBqrs1Ns-8-C3l1s7xWnzuG3HAL5lH-yOw",
    "authDomain": "smartmonitoring-1e0d2.firebaseapp.com",
    "databaseURL": "https://smartmonitoring-1e0d2-default-rtdb.asia-southeast1.firebasedatabase.app",
    "projectId": "smartmonitoring-1ed02",
    "storageBucket": "smartmonitoring-1ed02.appspot.com",
    "messagingSenderId": "792382808613",
    "appId": "1:792382808613:web:45c8cf2cc24419de7f15cc"
}

# Initialize Firebase
firebase = pyrebase.initialize_app(config)

# Create an ADC object
adc = Adafruit_ADS1x15.ADS1015()

# Set the gain
GAIN = 1

# Load the trained model
input_dim = 1  # Assuming a single float as input
hidden_dim = 64
output_dim = 1  # Assuming a single float as output

model = GCNConv(input_dim, hidden_dim, output_dim)
state_dict = torch.load('/home/aceng/Desktop/GCNmodel.pt')
model.load_state_dict(state_dict, strict=False)
model.eval()

# Define the edge_index tensor
edge_index = torch.tensor([
    [0, 2], [0, 3], [0, 4], [0, 5],
    [1, 2], [1, 3], [1, 4], [1, 5],
    [2, 6], [3, 6], [4, 6], [5, 6],
    [2, 7], [3, 7], [4, 7], [5, 7]
], dtype=torch.long).t().contiguous()

# Function to check WiFi connection status
def is_wifi_connected():
    try:
        output = subprocess.check_output(["iwgetid", "-r"]).decode("utf-8")
        if len(output.strip()) > 0:
            return True
    except subprocess.CalledProcessError:
        pass
    return False

# Function to reconnect to WiFi
def reconnect_wifi():
    ssid = "Cocomelon"
    password = "azizi123"
    cmd = 'wpa_passphrase "{}" "{}" | sudo tee -a /etc/wpa_supplicant/wpa_supplicant.conf > /dev/null'.format(ssid, password)
    subprocess.call(cmd, shell=True)
    subprocess.call("sudo wpa_cli -i wlan0 reconfigure", shell=True)

# Main Loop
while True:
    if not is_wifi_connected():
        print("WiFi connection lost. Reconnecting...")
        reconnect_wifi()
        time.sleep(10)

    try:
        storage = firebase.storage()
        database = firebase.database()

        reading_cm = database.child("Sensor_Fusion").child("System1(cm)").get()
        reading_g = database.child("Sensor_Fusion").child("System2(g)").get()
        
        fsr_max = 0
        load_max = 2000 #gram
        #y = (x - x1) * (0 - 10) / (x1 - 1936) + 10 where x is fsr reading value, x1 is fsr_max
        
        CH0 = adc.read_adc(0, gain=GAIN)
        CH0_New = abs((((-CH0+fsr_max)*load_max)/(fsr_max-1936)))
        Push_CH0 = database.child("System_2").child("FSR").child("FSR_1(g)").set(CH0_New)
        print("Fsr1: " + str(CH0_New))

        CH1 = adc.read_adc(1, gain=GAIN)
        CH1_New = abs(((-CH1+fsr_max)*load_max)/(fsr_max-1936))
        Push_CH1 = database.child("System_2").child("FSR").child("FSR_2(g)").set(CH1_New)
        print("Fsr2:" + str(CH1_New))

        CH2 = adc.read_adc(2, gain=GAIN)
        CH2_New = abs(((-CH2+fsr_max)*load_max)/(fsr_max-1936))
        Push_CH2 = database.child("System_2").child("FSR").child("FSR_3(g)").set(CH2_New)
        print("Fsr3:" + str(CH2_New))
        #Print FSR val
        #Max bit = 1936 (5V)
        #10000g = 1936
        #x g = 1

        # Read sensor fusion data (replace with your own logic)
        sensor_fusion = reading_cm.val()
        sensor_fusion2 = reading_g.val()

        # Preprocess the sensor fusion data and convert it to a PyTorch tensor
        preprocessed_data = [sensor_fusion] * 8  # Assuming 8 nodes in the graph
        input_data = torch.tensor(preprocessed_data, dtype=torch.float).unsqueeze(-1)  # Add dimension for input_dim

        # Create the Data object
        data = Data(x=input_data, edge_index=edge_index)

        # Perform inference using the model
        with torch.no_grad():
            output = model(data.x, data.edge_index)
            sfusion1 = output[0,0]
            sfusion2 = output[0,8]
            sf1 = sfusion1.item()
            sf2 = sfusion2.item()
            
        Predict_1 = sensor_fusion + (sf1*0.01)
        Predict_2 = sensor_fusion2 + (sf2*0.001)
        Push_sf1 = database.child("Prediction").child("Output").child("Distance (cm)").set(Predict_1)
        Push_sf2 = database.child("Prediction").child("Output").child("Mass (g)").set(Predict_2)
            #prediction = output[0].item()  # Extract the first element and call item()
            #print(prediction)
        
        #database.child("Prediction").child("value").set(prediction)

        time.sleep(1) #30minutes

    except Exception as e:
        print("An error occurred:", str(e))
        time.sleep(5)
