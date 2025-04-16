import asyncio
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg 
from bleak import BleakClient, BleakError

ADDRESS = "EC:E3:34:7B:77:16"
SERVICE_UUID = "00000180D-0000-1000-8000-00805F9B34FB"
HRcp_CHARACTERISTIC_UUID = "000002A39-0000-1000-8000-00805F9B34FB"
HRmax_CHARACTERISTIC_UUID = "000002A37-0000-1000-8000-00805F9B34FB"
HRmesura_CHARACTERISTIC_UUID = "000002A8D-0000-1000-8000-00805F9B34FB"
RESP_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# Emmagatzemar les dades
ecg_data = []
resp_data = []
time_data = []
sns_val = 0.0 
pns_val = 0.0 
stress_val = 0.0
BUFFER_SIZE = 50
async def ble(address):
    try:
        async with BleakClient(address) as client:
            print("Connectat")
            value = await client.read_gatt_char(RESP_CHARACTERISTIC_UUID) #INCORRECTE Mirar què hem d'enviar
            print(f"Informació rebuda: {value.decode()}")
    except BleakError as e:
        print(f"Error rebuda: {e}")
if __name__ == "__main__":
    asyncio.run(ble())

def GenerarGrafic():
    #Inicialitzar gràfica
    plt.ion()
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(20, 16), gridspec_kw={'height_ratios': [2, 2, 1]})
    line_ecg, = ax1.plot([], [], label="ECG") 
    line_resp, = ax2.plot([], [], label="Respiració")
    bar_sns = ax3.bar(["SNS", "PNS"], [0, 0], color=["red", "blue"])
    stress_text = ax3.text(1.5, 0.5, "Estrès: 0.0", fontsize=12)
    ax1.set_title("ECG (mV)") 
    ax2.set_title("Respiració (mV)") 
    ax3.set_title("Activació SNS / PNS i estrès")
    ax1.set_ylim(-1.5, 1.5)
    ax2.set_ylim(2.0, 4.0)
    ax3.set_ylim(0, 10)

def update_plot():
    # Actualitzar línies d'ECG i respiració 
    line_ecg.set_data(time_data[-BUFFER_SIZE:], ecg_data[-BUFFER_SIZE:]) 
    line_resp.set_data(time_data[-BUFFER_SIZE:], resp_data[-BUFFER_SIZE:]) 
    ax1.set_xlim(max(0, time_data[-1] - 10), time_data[-1])
    ax2.set_xlim(max(0, time_data[-1] - 10), time_data[-1])
    # Actualitzar barres SNS i PNS
    bar_sns[0].set_height(sns_val) 
    bar_sns[1].set_height(pns_val) 
    # Actualitzar estrès
    stress_text.set_text(f"Estrès:{stress_val:.1f}")
    plt.pause(0.1)
