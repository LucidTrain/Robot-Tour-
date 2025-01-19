
import network
from microdot import Microdot, Response
import ujson
from execute import boot2
import machine
import neopixel
import time
def read_webpage():
    with open('index.html', 'r') as file:
        return file.read()
def read_commands():
    with open('commands.json', 'r') as file2:
        data = ujson.load(file2)
        return data
def write_commands(data):
    with open ('commands.json', 'w') as file3:
        ujson.dump(data, file3)
        return True
try:
    runcheck = read_commands()
    if runcheck['launch'] == 'True':
        try:
            boot2()
        except Exception:
            np = neopixel.NeoPixel(machine.Pin(48), 1)
            while True:
                np[0] = (128,0,128)
                np.write()
                time.sleep(0.25)
                np[0] = (0,0,0)
                np.write()
                np[0] = (128,0,128)
                np.write()
                time.sleep(0.25)
                np[0] = (0,0,0)
                np.write()
                np[0] = (128,0,128)
                np.write()
                time.sleep(0.25)
                np[0] = (0,0,0)
                np.write()
        machine.reset()
except Exception:
    pass
ap = network.WLAN(network.AP_IF)
ap.config(essid='CheapChip', password='djjdm!334')
ap.config(authmode=network.AUTH_WPA2_PSK)
ap.active(True)
debug = True # debugging mode disables the webserver and will only process built in commands Only use for testing motor control module
#debug mode should always be false during competion
if debug == True:
    #system test sequence
    for i in range(10):
        #boot 2 crash led test
        np = neopixel.NeoPixel(machine.Pin(48), 1)
        np[0] = (128,0,128)
        np.write()
        time.sleep(0.25)
        np[0] = (0,0,0)
        np.write()
        np[0] = (128,0,128)
        np.write()
        time.sleep(0.25)
        np[0] = (0,0,0)
        np.write()
        np[0] = (128,0,128)
        np.write()
        time.sleep(0.25)
        np[0] = (0,0,0)
        np.write()
    try:
        result = boot2()
    except Exception:
        print("Self Test Failed!!!!!!")

app = Microdot()


@app.route('/')
def index(request):
    with open('index.html', 'r') as file4:
        data = file4.read()
   
    return Response(body=data, status_code=200, headers={'Content-Type': 'text/html'})
@app.route('/submit', methods=['POST'])
def handle_post(request):
    data = request.form.get('data', 'No data received')
    print(f"Received POST data: {data}")  # Log the data
    return {'status': 'success', 'data': data}
app.run(host="0.0.0.0", port=80)


