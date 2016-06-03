
import socket
import pypot.robot
from pypot.robot import from_json

nicu = from_json('nicu_humanoid_only_upper.json')

def move_joint(joint, speed, position):
    print 'Initial position: '+ str(eval('nicu.'+joint+'.present_position'))
    setattr(eval('nicu.'+joint), 'compliant', False)
    setattr(eval('nicu.'+joint), 'goal_speed', speed)
    setattr(eval('nicu.'+joint), 'goal_position', position)

def go_home():
    move_joint('r_shoulder_y', 8, -90)
    move_joint('r_arm_x', 8, 90)
    move_joint('r_elbow_y', 8, 90)
    move_joint('r_shoulder_z', 8, 0)
    #raw_input("Press enter to continue...")

def go_left():
    move_joint('r_shoulder_y', 8, 0)
    move_joint('r_arm_x', 8, 90)
    move_joint('r_elbow_y', 8, 90)
    move_joint('r_shoulder_z', 8, 60)
    #raw_input("Press enter to continue...")

def go_right():
    move_joint('r_shoulder_y', 8, 0)
    move_joint('r_arm_x', 8, 90)
    move_joint('r_elbow_y', 8, 90)
    move_joint('r_shoulder_z', 8, 120)
    #raw_input("Press enter to continue...")

def get():
    move_joint('r_gripper_x', 128, 30)

def drop():
    move_joint('r_gripper_x', 128, -30)

def main():
    print 'Interactive RL DEMO for cleaning a table is now running... '
    #go_home()
    #go_left()
    #go_right()
    #go_home()

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the port
    server_address = ('134.100.10.165', 10000)
    print 'starting up on %s port %s' % server_address
    sock.bind(server_address)

    # Listen for incoming connections
    sock.listen(1)

    while True:
        # Wait for a connection
        print 'waiting for a connection'
        connection, client_address = sock.accept()

        try:
            print 'connection from', client_address

            # Receive the data in small chunks and retransmit it
            #while True:

            action = connection.recv(1024)
            print 'Received action: "%s"' % action
            if action == '0' or action == '1':
                get()
            if action == '2' or action == '3':
                drop()
            if action == '4':
                go_home()
            if action == '5':
                go_left()
            if action == '6':
                go_right()
            if action == '7':
                break

            #break

            #if data:
            #    print 'sending data back to the client'
            #    connection.sendall(data)
            #else:
            #    print 'no more data from', client_address
            #    break
            
        finally:
            # Clean up the connection
            connection.close()

if __name__ == "__main__":
    main()


