import numpy as np

from solutions.assets.communication.comm_declarations import MsgType
from solutions.assets.communication.share import intersect_waypoints
from typing import List, Tuple


def create_msg(msg_type: MsgType, dest, msg_data, drone_id: int, msg_id: np.uint, transmit=False):
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
    return {'sender': drone_id, 'msg_id': msg_id, 'dest': dest, 'type': msg_type, 'data': msg_data, 'transmit': transmit}


def compute_received_com(received_messages, to_send, alive_received, processed_msg, id, waypoints, bases):
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
                process_message(msg_type, msg_content['data'], to_send, alive_received, processed_msg, id, waypoints, bases)
                processed_msg[sender_id].append(msg_content['msg_id'])

    return to_send



def process_message(msg_type, data, to_send, alive_received, processed_msg, id, waypoints, bases):
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

        case MsgType.SHARE_WAYPOINTS.value:
            intersect_waypoints(waypoints, data)
















