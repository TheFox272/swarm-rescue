import numpy as np

from swarm_rescue.solutions.assets.communication.comm_declarations import MsgType
from swarm_rescue.solutions.assets.communication.share import intersect_waypoints, intersect_bases, intersect_occupancy, intersect_entity, intersect_victims
from swarm_rescue.solutions.assets.behavior.think import MAX_BASES_SIZE
from swarm_rescue.solutions.assets.mapping.mapping_constants import TILE_SIZE

# region local constants
MAX_MESSAGE_MEMORY = 20
DONE_DRONE_ID = -1
# endregion


def create_msg(msg_type: MsgType, destination, msg_data, drone_id: int, msg_id: np.ndarray, transmit=False):
    """
    Creates a message

    The structure of a message is :
    sender_id : the drone id of the sender
    tick : the tick when the message was sent (not transmitted)
    msg_id : a count of how many messages was sent by the sender used as msg id
    (!! to differentiate all messages we need to compare sender_id and msg_id)
    dest : the intended receiver drone id (the addressee) OR 'all' if the message should be sent to every drone
    type : the message type used to know how to process the message data
    data : the message data relative to the message type
    transmit : a boolean indicating if the message should be transmitted between drone
    (which means a drone which receive a message that was not (or not only) for him will send the message on its own)
    """
    msg_id[0] += 1
    return {'sender': drone_id, 'msg_id': msg_id[0]-1, 'dest': destination, 'type': msg_type, 'data': msg_data, 'transmit': transmit}


def compute_received_com(received_messages, to_send, alive_received, processed_msg, abandon_victim, id, waypoints, bases, occupancy_map, entity_map, tile_map_size, victims,
                         in_noGPSzone: bool, dead_drones):
    """
    Compute the messages to send and the messages to proceed
    Args:
        received_messages: self.communicator.received_messages
        already_procced_msgs: a list indexed by sender_id containing the msg_id of the messages already procced
        id: the id of the drone receiving the messages

    Returns:

    """

    for msg in received_messages:
        msg = msg[1]
        for msg_content in msg:

            sender_id = msg_content['sender']
            msg_type = msg_content['type']
            dest_id = msg_content['dest']

            # this should never happen, note that a drone cannot receive a message from his own
            if sender_id == id:
                continue
            # if the message was already processed, we ignore it
            elif msg_content['msg_id'] in processed_msg[sender_id]:
                continue
            # if the message is not for all and not for this drone, then ignore it
            elif not msg_content['transmit'] and dest_id != id and dest_id != 'all':
                continue

            # if the message as transmit to true and is not only for us, we transmit it
            elif msg_content['transmit'] and dest_id != id:
                if 'transmit_by' not in msg_content:
                    msg_content['transmit_by'] = []
                if id not in msg_content['transmit_by']:
                    msg_content['transmit_by'].append(id)
                    to_send.append(msg_content)

            if dest_id == 'all' or dest_id == id:
                process_message(msg_type, msg_content['data'], alive_received, abandon_victim, id, waypoints, bases, occupancy_map, entity_map, tile_map_size, victims,
                                in_noGPSzone, dead_drones)
                processed_msg[sender_id] = processed_msg[sender_id][-MAX_MESSAGE_MEMORY:] + [msg_content['msg_id']]

    return to_send


def process_message(msg_type, data, alive_received, abandon_victim, id, waypoints, bases, occupancy_map, entity_map, tile_map_size, victims, in_noGPSzone, dead_drones):
    """
    Process the message
    Args:
        msg_content: the message content
        sender_id: the sender id
        msg_type: the message type

    Returns:

    """
    match msg_type:
        case MsgType.ALIVE.value:
            alive_received.append(data)
            if data[0] == DONE_DRONE_ID:
                x = data[1] * TILE_SIZE
                y = data[2] * TILE_SIZE
                dead_drones.append((x, y))

        case MsgType.SHARE_WAYPOINTS.value:
            intersect_waypoints(waypoints, data)

        case MsgType.SHARE_BASES.value:
            if len(bases) < MAX_BASES_SIZE:
                intersect_bases(bases, data)

        case MsgType.SHARE_OCCUPANCY_MAP.value:
            if not in_noGPSzone:
                intersect_occupancy(occupancy_map, data)

        case MsgType.SHARE_ENTITY_MAP.value:
            # if not in_noGPSzone:
            intersect_entity(entity_map, data, tile_map_size)

        case MsgType.SHARE_VICTIMS.value:
            if not victims:
                victims.extend(data)
            else:
                abandon_victim[0] = intersect_victims(victims, data, id) or abandon_victim[0]

        case _:  # for testing
            print(f"Issue: a message of unknown type {msg_type} was received")


