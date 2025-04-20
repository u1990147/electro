import asyncio
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg 
from bleak import BleakClient, BleakError

ADDRESS = "EC:E3:34:7B:77:16"
SERVICE_UUID  = "0000180D-0000-1000-8000-00805F9B34FB"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#Variables gràfic
fig = ax1 = ax2 = ax3 = None
line_ecg = line_resp = None
bar_sns = stress_text = None

# Emmagatzemar les dades
ecg_data = []
resp_data = []
time_data = [] #Eix temporal
tempsPics = []
rrIntervals = [] #Interval RR
sns_val = 0.0 
pns_val = 0.0 
stress_val = 0.0

BUFFER_SIZE = 50 
MOSTRES_GRAFIC = 1000 #Mostres màximes a mostrar al gràfic
INTERVAL = 0.2 #250SPS 50 mostres=0,2s

#Iterporlació i FFT
FS_INTERP=4
DT = 1.0 /FS_INTERP
LF_BAND = (0.04, 0.15)
HF_BAND = (0.15, 0.40)
THRESHOLD = 0.5 #mV, llindar detecció de pics


def generarGrafic():
    global fig, ax1, ax2, ax3, line_ecg, line_resp, bar_sns, stress_text
    #Inicialitzar gràfica
    plt.ion()
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(20, 16), gridspec_kw={'height_ratios': [2, 2, 1]})
    line_ecg, = ax1.plot([], [], label="ECG") 
    line_resp, = ax2.plot([], [], label="Respiració")
    bar_sns = ax3.bar(["SNS", "PNS"], [0, 0], color=["red", "blue"]) #Llista de barres
    stress_text = ax3.text(1.5, 0.5, "Estrès: 0.0", fontsize=12)
    ax1.set_title("ECG (mV)") 
    ax2.set_title("Respiració (mV)") 
    ax3.set_title("Activació SNS / PNS i estrès")
    ax1.set_ylim(-1.5, 1.5)
    ax2.set_ylim(2.0, 4.0)
    ax3.set_ylim(0, 10)

def update_plot():
    global line_ecg, line_resp, bar_sns, stress_text
    # Actualitzar línies d'ECG i respiració 
    line_ecg.set_data(time_data[-MOSTRES_GRAFIC:], ecg_data[-MOSTRES_GRAFIC:]) 
    line_resp.set_data(time_data[-MOSTRES_GRAFIC:], resp_data[-MOSTRES_GRAFIC:]) 
    ax1.set_xlim(max(0, time_data[-1] - 10), time_data[-1])
    ax2.set_xlim(max(0, time_data[-1] - 10), time_data[-1])
    plt.pause(0.1)

def parse_data(data_str):
    try:
        parts= list(map(float,data_str.strip().split(",")))
        if len(parts) >= 100:
            ecg_vals = parts[:50]
            resp_vals  = parts[50:100]
            return ecg_vals, resp_vals
    except Exception as e:
        print("Error al parsejar:", e)
    return [], []

def detectar_pics():
    if len(ecg_data) < BUFFER_SIZE:
        return
    fragment_ecg   = ecg_data[-BUFFER_SIZE:]
    fragment_temps = time_data[-BUFFER_SIZE:]
    for i in range(1, BUFFER_SIZE - 1):
        ant  = fragment_ecg[i-1]
        act  = fragment_ecg[i]
        seg  = fragment_ecg[i+1]
        if act > THRESHOLD and act > ant and act > seg:
            t = fragment_temps[i] #Només guardem el temps si la mostra és més gran que l'anterior i la seguent
            tempsPics.append(t)
            if len(tempsPics) >= 2:
                rrIntervals.append(tempsPics[-1] - tempsPics[-2])

def interpolar_rr():
    global tempsPics, rrIntervals
    N = int((tempsPics[-1] - tempsPics[0]) * FS_INTERP)
    t_interp = np.arange(N) *DT + tempsPics[0]
    rr_interp = np.zeros(N)
    j = 0
    M = len(tempsPics) #Numero de pics
    for i, t in enumerate(t_interp):
        # avanço l'índex j fins que t cau entre temps_pics[j] i temps_pics[j+1]
        while j < M-2 and tempsPics[j+1] < t:
            j += 1
        # si ja no hi ha més parelles (j+1 >= M), trenquem
        if j >= M-1:
            break
        x0, x1 = tempsPics[j],   tempsPics[j+1]
        y0, y1 = rrIntervals[j],  rrIntervals[j+1]
        # interpolació lineal (sempre x1 != x0 perquè temps_pics és estríctament creixent)
        rr_interp[i] = y0 + (y1 - y0) * (t - x0) / (x1 - x0)
    return t_interp, rr_interp


def calcular_fft(rr_interp, DT):
    N = len(rr_interp)
    # FFT directa de senyal real
    Y = np.fft.rfft(rr_interp) #rfft: real (FFT) et retorna només els N/2+1 bins corresponents a les freqüències positives
    # Potència espectral = quadrat del mòdul
    potencies = np.abs(Y)**2
    # Eix de freqüències positives
    freqs = np.fft.rfftfreq(N, DT)
    return potencies, freqs

def calcular_valors(potencies, freqs):
    LF = potencies[(freqs>=LF_BAND[0]) & (freqs<=LF_BAND[1])].sum()
    HF = potencies[(freqs> HF_BAND[0]) & (freqs<=HF_BAND[1])].sum()
    return (LF, HF, LF/HF if HF>0 else np.nan)

def notification_handler(sender,data):
    global ecg_data, resp_data, time_data
    global tempsPics, rrIntervals
    
    try:
        #Decodifiquem i extraiem valors
        decoded = data.decode("utf-8").strip()
        #Si és la comanda de càlcul, fem interpolació→FFT→LF/HF/Estrès
        if decoded == "Calcula":
         if len(tempsPics) >= 2:
            # 1. interpolar la sèrie RR
            t_rr, rr_interp = interpolar_rr()
            # 2. FFT i potències
            potencies, freqs = calcular_fft(rr_interp, DT)
            LF, HF, estres = calcular_valors(potencies, freqs)
            # 3. actualitzar gràfica
            bar_sns[0].set_height(LF)
            bar_sns[1].set_height(HF)
            stress_text.set_text(f"Estrès: {estres:.2f}")
            update_plot()
            # 4. netegem per al següent període de 2 min
            tempsPics.clear()
            rrIntervals.clear()
         return
        #En cas contrari és el paquet de dades
        ecg_vals, resp_vals = parse_data(decoded)
        #Si tenim dades, les afegim a les llistes globals i actualitzem el temps
        if ecg_vals and resp_vals:
            #l'últim temps o 0 si és la primera vegada
            now = time_data[-1] + INTERVAL if time_data else 0.0

            for i in range(len(ecg_vals)):
                time_data.append(now + i * (INTERVAL / BUFFER_SIZE)) #Reconstruim eix del temps
                ecg_data.append(ecg_vals[i]) #guarda les 50 noves dades enviades
                resp_data.append(resp_vals[i])
            # Retallem les dades antigues si cal
            if len(time_data) > MOSTRES_GRAFIC:
                time_data = time_data[-MOSTRES_GRAFIC:]
                ecg_data = ecg_data[-MOSTRES_GRAFIC:]
                resp_data = resp_data[-MOSTRES_GRAFIC:]
            # detectem pics i intervals RR
            detectar_pics()
            # interpolació i FFT
            t_rr, rr_interp = interpolar_rr()
            pot, freq = calcular_fft(rr_interp)
            LF, HF, estres = calcular_valors(pot, freq)
            # actualitzem barres i text
            bar_sns[0].set_height(LF)
            bar_sns[1].set_height(HF)
            stress_text.set_text(f"Estrès: {estres:.2f}" if not np.isnan(estres) else "Estrès: --")
            #Refresquem la gràfica
            update_plot()

    except Exception as e:
        print("Error en handler:", e)


async def main():
    try:
        async with BleakClient(ADDRESS) as client:
            print("Connectat")
            generarGrafic()
            #Subscribim a les notificacions
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
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

