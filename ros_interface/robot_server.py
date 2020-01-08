import socket

class RobotServer():
    def __init__(self, port=10003):
        self.port = port
        self.create_server()


    def create_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('localhost', self.port))
        self.server_socket.listen(1)
        self.connected = False
        self.listen()

    def listen(self):
        while not self.connected:
            connection, client_address = self.server_socket.accept()
            print('connected to', client_address)
            self.connected = True
            while self.connected:
                rx_data = connection.recv(1024)
                if rx_data:
                    print('sending back', rx_data)
                    connection.sendall('ack'+rx_data)
                    if rx_data == 'close':
                        print('received close')
                        #self.server_socket.close()
                        self.connected = False
                        #connection.close()
rs = RobotServer()

