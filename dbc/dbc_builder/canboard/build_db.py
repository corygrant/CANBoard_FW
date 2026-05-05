
import cantools
import canboard

def build_db(base_id):
    db = cantools.database.Database()
    db.name = "CANBoard"
    db.version = "3.1.0" #FW version
    
    db.messages.append(canboard.build_msg_0(base_id + 2))
    db.messages.append(canboard.build_msg_1(base_id + 2))
    db.messages.append(canboard.build_msg_2(base_id + 2))
    db.messages.append(canboard.build_msg_3(base_id + 2))

    return db