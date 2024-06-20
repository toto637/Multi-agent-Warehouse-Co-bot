import requests  # type: ignore
from flask import Flask, jsonify, request # type: ignore
from db import insert_request, get_request, update_dest, clear_request
import json

velocity = 20 #####################################

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret'

locations = {'locations': [
        "location1", "location2", "location3", "location4", "location5"
    ]}


@app.route('/get-all-locations', methods=['GET'])
def get_all_locations():
    return jsonify(locations)

######################################################
@app.route('/submit-request', methods=['POST'])
def submit_request():
    
    data = request.get_json()
    rass_id = send_data_to_ros({'location': data['location'], 'func':'submit_request'}) #### rass ID #################

    print(rass_id.text)
    
    json_obj = json.loads(rass_id.text)
    print(json_obj)
   
    insert_request(data['id'], data['location']) ## worker id , location chosed

    return jsonify({
        'id': json_obj['rid'], 
        #'id': -1,
    })

@app.route('/get-time-of-arrival/<int:id>', methods=['GET'])
def get_time_of_arrival(id: int):

    distance = send_data_to_ros({'func':'path_request', 'rid': id}) 

    print(distance.text)

    json_obj = json.loads(distance.text)
    print(json_obj)
    
    
    return jsonify({
        'time': json_obj['path']/ velocity
    })

@app.route('/retrieve-valid-destinations', methods=['POST'])
def get_destination():


    data = get_request(request.get_json()['uid'])

    return jsonify({'destinations': [_ for _ in locations['locations'] if _ not in data[1]]})

@app.route('/update-destination', methods=['POST'])
def update_destination():
    uid = request.get_json()['uid']

    update_dest(request.get_json()['uid'], request.get_json()['destination'])
    return jsonify({'message': 'Destination updated'})


# get bby3rd
# post ba5d mn user

@app.route('/time-to-destination/<int:id>', methods=['GET'])
def get_time_to_destination(id: int):

    path = send_data_to_ros(id) #### rass ID #################

    path_distance = 0 #path_distance = path
    
    return jsonify({
        'time': path_distance / velocity

    
    })

@app.route('/clear-request/<int:id>', methods=['GET'])
def clear_req(id: int):
    clear_request(id)
    return jsonify({'message': 'Request cleared'})


# ros send to me
@app.route('/ros_data/<data>')
def ros_data(data):
    # Process ROS data here
    print(data)
    return data

#######faksana (bb3t byanat kteeer)
@app.route('/send_data', methods=['POST'])
def send_data():
    data = 'tom'
    send_data_to_ros(data)
    return 'Data sent to ROS 2'


def send_data_to_ros(data):
    url = 'http://localhost:8080/topic'
    return requests.post(url, data=data)


if __name__=="__main__":
    app.run(host="0.0.0.0",port=8000, debug=True)