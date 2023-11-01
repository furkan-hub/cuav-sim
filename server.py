import flask
import requests
import threading
from json import *

telem_1 = {

        "lon": 0,
        "lat": 0,
        "alt": 0,
        "pitch": 0,
        "yaw": 0,
        "roll": 0
    }

telem_2 = {

        "lon": 0,
        "lat": 0,
        "alt": 0,
        "pitch": 0,
        "yaw": 0,
        "roll": 0
    }

datas = [telem_1,telem_2]

api_ip = "127.0.0.1"
api_port = "5000"


# flask api için obje
app = flask.Flask(__name__)
app.config["DEBUG"] = False



@app.route('/test', methods=['GET'])
def test():
    print("testtt")
    return "server is working", 200
   
@app.route('/telem_post_1', methods=['POST'])
def telem_post_1():
    global telem_1

    telem_1 = flask.request.json

    print(telem_1)
    
    return ("ok",200)  # HTTP 200 OK yanıtı

@app.route('/telem_get_1', methods=['GET'])
def telem_get_1():
    global telem_2

    return telem_2, 200

@app.route('/telem_post_2', methods=['POST'])
def telem_post_2():
    global telem_2

    telem_2 = flask.request.json

    print(telem_2)
    
    return ("ok",200)  # HTTP 200 OK yanıtı

@app.route('/telem_get_2', methods=['GET'])
def telem_get_2():
    global telem_1

    return telem_1, 200

th1 = threading.Thread(target=app.run, args=[api_ip, api_port])
th1.start()