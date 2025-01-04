
import network
from microdot import Microdot, Response
import ujson
from execute import boot2
import machine

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
runcheck = read_commands()
if runcheck['launch'] == 'True':
    boot2()
    machine.reset()
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid='SETUP-D8J83', password='54142424145415241201414114141')
app = Microdot()


@app.route('/')
def index(request):
    data = "html read from file"
    return Response(body=data, contentmanager='text/html')
@app.route('/submit', methods=['POST'])
def handle_post(request):
    data = request.form.get('data', 'No data received')
    print(f"Received POST data: {data}")  # Log the data
    return {'status': 'success', 'data': data}
app.run(host="0.0.0.0", port=80)

