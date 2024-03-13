from dynamic_graph_manager.vicon_sdk import ViconClientEntity

client_name = "vicon_client"
vicon_ip = "10.32.3.16:801"
robot_vicon_name = "solo12"

vicon_client = ViconClientEntity(client_name)
vicon_client.connect_to_vicon(vicon_ip)

vicon_client.add_object_to_track(
    "{}/{}".format(robot_vicon_name, robot_vicon_name)
)

vicon_client.displaySignals()
