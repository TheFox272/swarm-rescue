from comm_declarations import MsgType


def create_msg(msg_type: MsgType, msg_data, drone_id: int, tick: int, msg_id):
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
    return {'sender_id': drone_id, 'tick': tick, 'msg_id': msg_id, 'dest': 'all', 'type': msg_type, 'data': msg_data, 'transmit': False}


def compute_received_com(received_messages, already_procced_msgs, id):
    to_send = []
    for msg in received_messages:
        msg = msg[1]
        for msg_content in msg:
            sender_id = msg_content['sender']
            msg_type = msg_content['type']

            # this should never happend, note that a drone cannot receive a message from his own
            if sender_id == id:
                continue

            # if the message as transmit to true and is not only for us, we transmit it
            if 'transmit' in msg_content and msg_content['transmit'] and msg_content['dest'] != self.id and :
                if 'transmit_by' not in msg_content:
                    msg_content['transmit_by'] = []
                if id not in msg_content['transmit_by']:
                    msg_content['transmit_by'].append(id)
                    to_send.append(msg_content)

            # if the message is not for all and not for this drone, then ignore it
            if msg_content['dest'] != id and msg_content['dest'] != 'all':
                continue

    return to_send


