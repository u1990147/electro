import asyncio
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg 
from bleak import BleakClient, BleakError

ADDRESS = "EC:E3:34:7B:77:16"
SERVICE_UUID = "0000180D-0000-1000-8000-00805F9B34FB"
HRmesura_CHARACTERISTIC_UUID = "000002A8D-0000-1000-8000-00805F9B34FB"
RESP_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#Variables gràfic
fig = ax1 = ax2 = ax3 = None
line_ecg = line_resp = None
bar_sns = stress_text = None

# Emmagatzemar les dades
ecg_data = []
resp_data = []
time_data = []
sns_val = 0.0 
pns_val = 0.0 
stress_val = 0.0
BUFFER_SIZE = 1000 # 50 mostres per paquet , 20 paquets
INTERVAL = 0.2 #250SPS 50 mostres=0,2s

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

def parse_data(data_str):#Separem per comes les dades
    global sns_val, pns_val, stress_val
    try:
        parts= list(map(float,data_str.strip().split(",")))
        if len(parts) >= 103:
            resp_vals = parts[:50]
            ecg_vals  = parts[50:100]
            sns_val   = parts[100]
            pns_val   = parts[101]
            stress_val= parts[102]
            return ecg_vals, resp_vals
    except Exception as e:
        print("Error al parsejar:", e)
    return [], []

def notification_handler(sender,data):
    global ecg_data, resp_data, time_data
    try:
        #Decodifiquem i extraiem valors
        decoded = data.decode("utf-8")
        ecg_vals, resp_vals = parse_data(decoded)
         #Si tenim dades, les afegim a les llistes globals i actualitzem el temps
        if ecg_vals and resp_vals:
            #l'últim temps o 0 si és la primera vegada
            now = time_data[-1] + INTERVAL if time_data else 0.0

            for i in range(len(ecg_vals)):
                time_data.append(now + i * (INTERVAL / BUFFER_SIZE)) #Reconstruim eix del temps
                ecg_data.append(ecg_vals[i]) #guarda les 50 noves dades enviades
                resp_data.append(resp_vals[i])

            #Refresquem la gràfica
            update_plot()

    except Exception as e:
        print("Error en handler:", e)


async def main():
    try:
        async with BleakClient(ADDRESS) as client:
            print("Connectat")
            GenerarGrafic()
            #Subscribim a les notificacions
            await client.start_notify(RESP_CHARACTERISTIC_UUID, notification_handler)
            while True:
                if client.is_connected():
                    await asyncio.sleep(0.1);
                    print("Informació rebuda")
                else:
                    print("Desconnectat")
                    break
    except BleakError as e:
        print(f"Error rebuda: {e}")
if __name__ == "__main__":
    asyncio.run(main())


