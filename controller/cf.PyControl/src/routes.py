from flask import Blueprint, jsonify, request, current_app
from statemachine import exceptions
from werkzeug.routing import BaseConverter, ValidationError

from cf_positioning import Point3D

# Define a blueprint
drone_blueprint = Blueprint('drone', __name__)
DEBUG = False

@drone_blueprint.errorhandler(exceptions.TransitionNotAllowed)
def handle_transition_not_allowed_error(error):
    return jsonify({"error": str(error)}), 400

# Print available URLs when the Flask server starts
@drone_blueprint.route('/routes', methods=['GET'])
def routes():
    urls = []
    for rule in current_app.url_map.iter_rules():
        if DEBUG:
            print(f"http://{request.host}{rule}")
        urls.append(f"http://{request.host}{rule}")
    return jsonify({"routes": urls})

# Define REST endpoints for each transition
@drone_blueprint.route('/install', methods=['POST'])
def install():
    drone = current_app.config['DRONE']
    if drone.can_install():
        drone.install()
        return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})
    return jsonify({"message": "Cannot transition to", "state": drone.get_current_state()}), 400

@drone_blueprint.route('/start', methods=['POST'])
def start():
    drone = current_app.config['DRONE']
    drone.start()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/initialize', methods=['POST'])
def initialize():
    drone = current_app.config['DRONE']
    drone.initialize()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/stop', methods=['POST'])
def stop():
    drone = current_app.config['DRONE']
    drone.stop()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/uninstall', methods=['POST'])
def uninstall():
    drone = current_app.config['DRONE']
    drone.uninstall()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/activate_idle', methods=['POST'])
def activate_idle():
    drone = current_app.config['DRONE']
    drone.activate_idle()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/begin_takeoff', methods=['POST'])
def begin_takeoff():
    drone = current_app.config['DRONE']
    drone.begin_takeoff()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/begin_landing', methods=['POST'])
def begin_landing():
    drone = current_app.config['DRONE']
    drone.begin_landing()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/navigate/<string:x>/<string:y>/<string:z>', methods=['POST'])
def navigate_to(x, y, z):
    x, y, z = float(x), float(y), float(z)
    drone = current_app.config['DRONE']
    print(f"REST API: Point3D(x,y,z): Point3D({x},{y},{z})")
    drone.targetPointsQueue.append(Point3D(x,y,z))
    drone.begin_nav_goal_sequence()
    return jsonify({"message": f'Navigating to ({x}, {y}, {z}) completed', "state": drone.get_current_state()})

@drone_blueprint.route('/navigate/append/<string:x>/<string:y>/<string:z>', methods=['POST'])
def append_navigation_goal(x, y, z):
    x, y, z = float(x), float(y), float(z)
    drone = current_app.config['DRONE']
    print(f"REST API: Point3D(x,y,z): Point3D({x},{y},{z})")
    drone.targetPointsQueue.append(Point3D(x,y,z))
    return jsonify({"message": f'Appended Target for Navigation to ({x}, {y}, {z})', "state": drone.get_current_state()})

@drone_blueprint.route('/shutdown_command', methods=['POST'])
def shutdown_command():
    drone = current_app.config['DRONE']
    drone.shutdown_command()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/begin_stopping', methods=['POST'])
def begin_stopping():
    drone = current_app.config['DRONE']
    drone.begin_stopping()
    return jsonify({"message": "Transitioned to", "state": drone.get_current_state()})

@drone_blueprint.route('/state', methods=['GET'])
def state():
    drone = current_app.config['DRONE']
    return jsonify({"state": drone.get_current_state()})

@drone_blueprint.route('/transition', methods=['GET'])
def transition():
    drone = current_app.config['DRONE']
    return jsonify({"transition": drone.get_current_transition()})

@drone_blueprint.route('/status', methods=['GET'])
def status():
    drone = current_app.config['DRONE']
    return jsonify({"state": drone.get_current_state(), "transition": drone.get_current_transition()})